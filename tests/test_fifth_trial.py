"""Defects the fifth first run trial found, each with the source pattern that showed it."""

from __future__ import annotations

from pathlib import Path

from rosio import extract, schema
from rosio.config import Config, Entrypoint, ProfileSpec, RootSpec
from rosio.workspace import Workspace

TWO_SITES = """#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
class Monitor : public rclcpp::Node {
public:
  Monitor() : Node("monitor") {
    socket_ = declare_parameter<std::string>("socket_path", "/tmp/a");
  }
  explicit Monitor(int) : Node("monitor") {
    socket_ = declare_parameter<std::string>("socket_path", "/tmp/a");
  }
  void tick() {
    if (fast_) {
      buffer_->canTransform("map", "base_link", tf2::TimePointZero);
    } else if (slow_) {
      buffer_->canTransform("map", "base_link", tf2::TimePointZero);
    }
  }
  std::string socket_;
  bool fast_{false};
  bool slow_{false};
  std::shared_ptr<tf2_ros::Buffer> buffer_;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Monitor>());
}
"""


def cpp_workspace(tmp_path: Path, source: str, *, name: str = "monitor") -> Config:
    package = tmp_path / "src" / "p"
    package.mkdir(parents=True)
    (package / "package.xml").write_text(
        "<package><name>p</name><version>0.0.0</version></package>"
    )
    (package / "CMakeLists.txt").write_text(f"project(p)\nadd_executable({name} {name}.cpp)\n")
    (package / f"{name}.cpp").write_text(source)
    (package / "run.launch.py").write_text(
        "from launch import LaunchDescription\nfrom launch_ros.actions import Node\n"
        "def generate_launch_description():\n"
        f'    return LaunchDescription([Node(package="p", executable="{name}")])\n'
    )
    roots = (RootSpec("src", str(tmp_path / "src")),)
    return Config(
        roots=roots,
        profiles=(ProfileSpec("p", entrypoints=(Entrypoint("launch", "p", "run.launch.py"),)),),
        out_dir=str(tmp_path / "out"),
        deterministic=True,
    )


def test_two_sites_of_one_symbol_fold_into_one_edge_and_the_extract_finishes(tmp_path: Path):
    config = cpp_workspace(tmp_path, TWO_SITES)
    atlas = extract.run(config, Workspace.discover(config.roots))
    declares = [edge for edge in atlas.edges.values() if edge.kind == "declares_param"]
    assert len(declares) == 1
    assert len(declares[0].facts[0].evidence) == 2
    listens = [edge for edge in atlas.edges.values() if edge.kind == "listens_tf"]
    assert {edge.dst for edge in listens} == {"frame:map", "frame:base_link"}
    assert all(len(edge.facts[0].evidence) == 2 for edge in listens)
    assert extract.model.completeness(atlas)["complete"]
    assert schema.validate(atlas.to_json()) == []
    again = extract.run(config, Workspace.discover(config.roots))
    assert again.canonical_bytes() == atlas.canonical_bytes()


def test_a_join_that_cannot_place_a_site_is_a_refused_pass_and_not_an_empty_run(
    tmp_path: Path, monkeypatch
):
    from rosio import join
    from rosio.errors import ContractError

    def refuse(builder):
        raise ContractError("a site the join cannot place")

    monkeypatch.setattr(join, "join", refuse)
    config = cpp_workspace(tmp_path, TWO_SITES)
    atlas = extract.run(config, Workspace.discover(config.roots))
    assert extract.model.completeness(atlas)["refused_collectors"] == ["join"]
    assert "extract.pass_incomplete" in {finding.rule for finding in atlas.findings}
    assert (tmp_path / "out" / "atlas.json").exists()


FILTER = """#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
class Slam : public rclcpp::Node {
public:
  Slam() : Node("slam") {
    scan_topic_ = std::string("/scan");
    this->declare_parameter("scan_topic", scan_topic_);
    scan_topic_ = this->get_parameter("scan_topic").as_string();
  }
  void setup() {
    scan_filter_sub_ =
      std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
      rclcpp::Node>>(shared_from_this().get(), scan_topic_, rmw_qos_profile_sensor_data);
    info_a_.subscribe(this, "camera/info", rclcpp::QoS(10));
    hidden_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
      this, topic_for(3), rmw_qos_profile_sensor_data);
  }
  std::string topic_for(int);
  std::string scan_topic_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
    rclcpp::Node>> scan_filter_sub_, hidden_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_b_, info_a_;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Slam>());
}
"""


def test_a_message_filters_constructor_subscribes_to_its_parameter_backed_topic(tmp_path: Path):
    config = cpp_workspace(tmp_path, FILTER, name="slam")
    atlas = extract.run(config, Workspace.discover(config.roots))
    subscribed = {
        edge.dst: edge for edge in atlas.out_edges("node:/slam") if edge.kind == "subscribes"
    }
    assert {"topic:/scan", "topic:/camera/info"} <= set(subscribed)
    assert subscribed["topic:/scan"].facts[0].values["type"] == "sensor_msgs/msg/LaserScan"
    assert any(
        entry.want == "topic_name" and "topic_for" in entry.expr for entry in atlas.unresolved
    )
    assert schema.validate(atlas.to_json()) == []


def test_a_component_executable_finds_its_class_when_the_target_sources_are_a_glob(
    tmp_path: Path,
):
    """Autoware builds every component with `ament_auto_add_library(... DIRECTORY src)`, and
    each generated executable was a blind spot saying no source defines its entry point."""
    package = tmp_path / "src" / "p"
    (package / "src").mkdir(parents=True)
    (package / "package.xml").write_text(
        "<package><name>p</name><version>0.0.0</version></package>"
    )
    (package / "CMakeLists.txt").write_text(
        "project(p)\nament_auto_add_library(comp SHARED DIRECTORY src)\n"
        'rclcpp_components_register_node(comp PLUGIN "demo::Comp" EXECUTABLE comp_node)\n'
    )
    (package / "src" / "comp.cpp").write_text(
        "#include <rclcpp/rclcpp.hpp>\nnamespace demo {\n"
        "class Comp : public rclcpp::Node {\npublic:\n"
        '  explicit Comp(const rclcpp::NodeOptions & o) : Node("comp", o) {\n'
        '    pub_ = create_publisher<std_msgs::msg::String>("out", 10);\n  }\n'
        "  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;\n};\n}\n"
        "#include <rclcpp_components/register_node_macro.hpp>\n"
        "RCLCPP_COMPONENTS_REGISTER_NODE(demo::Comp)\n"
    )
    roots = (RootSpec("src", str(tmp_path / "src")),)
    config = Config(
        roots=roots,
        profiles=(ProfileSpec("p", entrypoints=(Entrypoint("run", "p", "comp_node"),)),),
        out_dir=str(tmp_path / "out"),
        deterministic=True,
    )
    atlas = extract.run(config, Workspace.discover(roots))
    assert not [row for row in atlas.unresolved if row.reason == "executable_entry_unresolved"]
    assert schema.validate(atlas.to_json()) == []


def test_a_name_the_source_builds_at_run_time_is_one_blind_spot_and_not_two(tmp_path: Path):
    """merge_maps_kinematic.cpp:104 was counted under the symbol and again under the node."""
    source = """#include <rclcpp/rclcpp.hpp>
class Merge : public rclcpp::Node {
public:
  Merge() : Node("merge") {
    pubs_.push_back(create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map_" + std::to_string(count_), rclcpp::QoS(1)));
  }
  int count_{0};
  std::vector<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr> pubs_;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Merge>());
}
"""
    config = cpp_workspace(tmp_path, source, name="merge")
    atlas = extract.run(config, Workspace.discover(config.roots))
    blind = [row for row in atlas.unresolved if row.want == "topic_name"]
    assert len(blind) == 1
    assert blind[0].owner.startswith("sym:")
