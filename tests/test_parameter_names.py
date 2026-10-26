from pathlib import Path

import pytest

from rosio import extract, join, model, schema
from rosio.collect import cpp_code as cpp
from rosio.config import Config, Entrypoint, ProfileSpec, RootSpec
from rosio.workspace import Workspace

pytestmark = pytest.mark.cpp

SOURCE = """
class Host : public rclcpp::Node {
public:
  Host() : Node("host") {}
  void configure() {
    map_name_ = std::string("/map");
    declare_parameter("map_name", map_name_);
    map_name_ = this->get_parameter("map_name").as_string();
    map_frame_ = std::string("map");
    declare_parameter("map_frame", map_frame_);
    map_frame_ = this->get_parameter("map_frame").as_string();
    odom_frame_ = std::string("odom");
    declare_parameter("odom_frame", odom_frame_);
    odom_frame_ = this->get_parameter("odom_frame").as_string();
  }
  void connect() {
    auto map = create_publisher<nav_msgs::msg::OccupancyGrid>(map_name_, 10);
    auto metadata = create_publisher<nav_msgs::msg::MapMetaData>(map_name_ + "_metadata", 10);
    buffer_->lookupTransform(map_frame_, odom_frame_, time);
  }
};
int main() { auto host = std::make_shared<Host>(); }
"""


def test_parameter_accessors_preserve_name_dependencies_and_suffixes():
    unit = cpp.parse_text(SOURCE, path="host.cpp", pkg="p", relpath="host.cpp")
    cpp.analyse([unit])
    report = cpp.file_report(unit)
    publishers = [row for row in report["endpoints"] if row["kind"] == "publisher"]
    assert len(publishers) == 2
    assert all(row.get("from_parameter") == "map_name" for row in publishers)
    assert {row.get("parameter_suffix", "") for row in publishers} == {"", "_metadata"}
    assert all(row["name"] is None for row in publishers)
    lookup = report["tf"]["lookups"][0]
    assert lookup["target_from_parameter"] == "map_frame"
    assert lookup["source_from_parameter"] == "odom_frame"


def _workspace(tmp_path: Path, unknown: bool, mixed: bool = False):
    package = tmp_path / "src" / "p"
    package.mkdir(parents=True)
    (package / "package.xml").write_text(
        "<package><name>p</name><version>0.0.0</version></package>"
    )
    (package / "CMakeLists.txt").write_text("project(p)\nadd_executable(host host.cpp)\n")
    (package / "host.cpp").write_text(SOURCE)
    launch = """
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(package="p", executable="host", name="host", parameters=[OVERRIDES])
    ])
"""
    values = (
        '{"map_name": Command(["ignored"]), "map_frame": Command(["ignored"]), '
        '"odom_frame": Command(["ignored"])}'
        if unknown
        else '{"map_name": "/chosen", "map_frame": "global", "odom_frame": "wheel"}'
    )
    if unknown is None:
        values = "{}"
    (package / "known.launch.py").write_text(launch.replace("OVERRIDES", values))
    profiles = [ProfileSpec("known", entrypoints=(Entrypoint("launch", "p", "known.launch.py"),))]
    if mixed:
        values = (
            '{"map_name": Command(["ignored"]), "map_frame": Command(["ignored"]), '
            '"odom_frame": Command(["ignored"])}'
        )
        (package / "unknown.launch.py").write_text(launch.replace("OVERRIDES", values))
        profiles.append(
            ProfileSpec("unknown", entrypoints=(Entrypoint("launch", "p", "unknown.launch.py"),))
        )
    roots = (RootSpec("src", str(tmp_path / "src")),)
    return Workspace.discover(roots), Config(
        roots=roots, profiles=tuple(profiles), out_dir=str(tmp_path / "out"), deterministic=True
    )


@pytest.mark.ros
@pytest.mark.parametrize("unknown", [False, True, None])
def test_join_uses_profile_parameter_values_without_guessing(tmp_path: Path, unknown, monkeypatch):
    original = join.join

    def twice(builder):
        result = original(builder)
        before = builder.atlas.canonical_bytes()
        original(builder)
        assert builder.atlas.canonical_bytes() == before
        return result

    monkeypatch.setattr(join, "join", twice)
    workspace, config = _workspace(tmp_path, unknown)
    atlas = extract.run(config, workspace)
    publishers = [edge.dst for edge in atlas.out_edges("node:/host") if edge.kind == "publishes"]
    if unknown:
        assert publishers == []
        assert "frame:map" not in atlas.entities
        assert any(entry.want == "topic_name" for entry in atlas.unresolved)
    else:
        prefix = "/map" if unknown is None else "/chosen"
        assert sorted(publishers) == [f"topic:{prefix}", f"topic:{prefix}_metadata"]
        assert ("frame:map" if unknown is None else "frame:global") in atlas.entities
        assert ("frame:odom" if unknown is None else "frame:wheel") in atlas.entities
        assert not any(entry.want in ("topic_name", "frame_id") for entry in atlas.unresolved)
    assert schema.validate(atlas.to_json()) == []


@pytest.mark.ros
def test_unknown_profile_keeps_shared_source_blind_spots(tmp_path: Path):
    workspace, config = _workspace(tmp_path, False, mixed=True)
    atlas = extract.run(config, workspace)
    assert "topic:/chosen" in atlas.entities
    assert "topic:/map" not in atlas.entities
    assert any(
        entry.want == "topic_name" and entry.owner.startswith("sym:") for entry in atlas.unresolved
    )
    assert any(
        entry.want == "frame_id" and entry.owner.startswith("sym:") for entry in atlas.unresolved
    )


def test_invalid_parameter_dependency_stays_unknown():
    item = join.Instance(
        node_id="node:/host",
        profile="profile:p",
        pkg="p",
        executable="host",
        namespace="/",
        name="host",
        name_source="literal",
        known_name="",
        remaps=(),
    )
    assert join._parameter_name(model.AtlasBuilder().atlas, item, "bad..name", "", "/default") == (
        "",
        "",
    )


def test_later_parameter_member_keeps_its_literal_suffix():
    unit = cpp.parse_text(
        """
class Host : public rclcpp::Node {
public:
  Host() : Node("host") {}
  void connect() { create_publisher<Message>(topic_ + "_metadata", 10); }
  void configure() { topic_ = declare_parameter("topic", std::string("out")); }
};
""",
        path="host.cpp",
        pkg="p",
        relpath="host.cpp",
    )
    cpp.analyse([unit])
    row = cpp.file_report(unit)["endpoints"][0]
    assert row["name"] == "out_metadata"
    assert row["from_parameter"] == "topic"
    assert row["parameter_suffix"] == "_metadata"
