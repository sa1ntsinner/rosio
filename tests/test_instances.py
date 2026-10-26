from pathlib import Path

import pytest

from rosio import extract, ids, join, model, schema
from rosio.collect import cpp_code as cpp
from rosio.config import Config, Entrypoint, ProfileSpec, RootSpec
from rosio.workspace import Workspace

pytestmark = pytest.mark.cpp

PIPE = """
class Pipe : public rclcpp::Node {
public:
  Pipe(const std::string & name, const std::string & input, const std::string & output)
  : Node(name) {
    create_subscription<Message>(input, 10, callback);
    create_publisher<Message>(output, 10);
  }
};
"""


def test_literal_constructions_keep_separate_nodes_endpoints_and_real_symbols():
    unit = cpp.parse_text(
        PIPE
        + """
int main() {
  auto first = std::make_shared<Pipe>("first", "one", "two");
  auto second = std::make_shared<Pipe>("second", "two", "one");
  auto unknown = std::make_shared<Pipe>(runtime_name(), "three", runtime_topic());
}
""",
        path="main.cpp",
        pkg="p",
        relpath="main.cpp",
    )
    cpp.analyse([unit])
    report = cpp.file_report(unit)
    nodes = {row["name"]: row for row in report["nodes"]}
    assert set(nodes) == {"first", "second", None}
    assert len({row["construction"] for row in nodes.values()}) == 3
    endpoints = {
        name: {
            (row["kind"], row["name"])
            for row in report["endpoints"]
            if row["construction"] == node["construction"]
        }
        for name, node in nodes.items()
    }
    assert endpoints["first"] == {("subscriber", "one"), ("publisher", "two")}
    assert endpoints["second"] == {("subscriber", "two"), ("publisher", "one")}
    assert endpoints[None] == {("subscriber", "three"), ("publisher", None)}
    assert all(row["owner_symbol"] == "Pipe::Pipe" for row in report["endpoints"])
    assert not any(row["reason"] == "ambiguous_construction" for row in report["unresolved"])
    assert sum(row["want"] == "topic_name" for row in report["unresolved"]) == 1
    assert sum(row["want"] == "node_name" for row in report["unresolved"]) == 1
    cpp.analyse([unit])
    assert cpp.file_report(unit) == report


def test_included_header_constructions_keep_each_callers_context():
    header = cpp.parse_text(PIPE, path="pipe.hpp", pkg="p", relpath="pipe.hpp")
    first = cpp.parse_text(
        '#include "pipe.hpp"\nint main() {\n'
        'auto a = std::make_shared<Pipe>("first", "one", "two");\n'
        'auto b = std::make_shared<Pipe>("second", "two", "one");\n}\n',
        path="main.cpp",
        pkg="p",
        relpath="main.cpp",
    )
    other = cpp.parse_text(
        '#include "pipe.hpp"\nint main() {\n'
        'auto c = std::make_shared<Pipe>("other", "three", "four");\n}\n',
        path="other.cpp",
        pkg="p",
        relpath="other.cpp",
    )
    cpp.analyse([header, first, other])
    first_rows = cpp.file_report(first)["nodes"]
    other_rows = cpp.file_report(other)["nodes"]
    assert {row["name"] for row in first_rows} == {"first", "second"}
    assert {row["name"] for row in other_rows} == {"other"}
    scopes = {row["construction"] for row in first_rows}
    other_scope = other_rows[0]["construction"]
    report = cpp.file_report(header)
    assert not report["unresolved"]
    assert {
        (row["kind"], row["name"])
        for row in report["endpoints"]
        if row["construction"] == other_scope
    } == {("subscriber", "three"), ("publisher", "four")}
    assert other_scope not in scopes


def test_adding_a_sibling_preserves_construction_discriminators():
    original = (
        PIPE
        + """
int main() {
  auto a = std::make_shared<Pipe>("first", "one", "two");
  auto b = std::make_shared<Pipe>("second", "two", "one");
}
"""
    )
    added = original.replace(
        "int main() {",
        """int main() {
  auto c = std::make_shared<Pipe>("other", "three", "four");""",
    )
    scopes = []
    for source in (original, added):
        unit = cpp.parse_text(source, path="main.cpp", pkg="p", relpath="main.cpp")
        cpp.analyse([unit])
        scopes.append({row["name"]: row["construction"] for row in cpp.file_report(unit)["nodes"]})
    assert scopes[0]["first"] == scopes[1]["first"]
    assert scopes[0]["second"] == scopes[1]["second"]


def _builder_and_carriers(contexts):
    builder = model.AtlasBuilder()
    main_file = ids.build_file("p", "main.cpp")
    header_file = ids.build_file("p", "pipe.hpp")
    node = ids.build_node("/first")
    builder.node(main_file)
    builder.node(header_file)
    builder.node("profile:p")
    builder.node(node, name_source="literal")
    builder.fact(
        node,
        "declared",
        profile="profile:p",
        values={"pkg": "p", "executable": "pipe", "name": "first", "namespace": "/"},
    )
    main = join.Carrier(
        owner=main_file,
        file=main_file,
        symbol="",
        language="cpp",
        nodes=tuple(
            {"name": name, "construction": scope, "name_source": "literal"}
            for name, scope in contexts
        ),
    )
    header = join.Carrier(
        owner=header_file,
        file=header_file,
        symbol="",
        language="cpp",
        endpoints=(
            {"kind": "publisher", "name": "one", "construction": "a", "owner_symbol": "Pipe::Pipe"},
            {"kind": "publisher", "name": "two", "construction": "b", "owner_symbol": "Pipe::Pipe"},
        ),
    )
    return builder, node, (main, header)


def test_join_keeps_only_the_selected_executables_construction_context():
    builder, node, carriers = _builder_and_carriers([("first", "a")])
    item = join.instances(builder.atlas)[0]
    join._bind_instance(builder, item, carriers, "executable", join.JoinReport(), count=True)
    assert {edge.dst for edge in builder.atlas.out_edges(node) if edge.kind == "publishes"} == {
        ids.build_topic("/one")
    }
    assert schema.validate(builder.build().to_json()) == []
    before = builder.atlas.canonical_bytes()
    join._bind_instance(builder, item, carriers, "executable", join.JoinReport(), count=False)
    assert builder.build().canonical_bytes() == before


def test_join_does_not_smear_two_constructions_onto_one_declared_node():
    builder, node, carriers = _builder_and_carriers([("first", "a"), ("second", "b")])
    item = join.instances(builder.atlas)[0]
    join._bind_instance(builder, item, carriers, "executable", join.JoinReport(), count=True)
    assert not [edge for edge in builder.atlas.out_edges(node) if edge.kind == "publishes"]
    assert any(
        entry.owner == node and entry.reason == "multiple_constructed_nodes"
        for entry in builder.atlas.unresolved
    )


def test_join_states_when_no_constructor_context_selected_the_scoped_endpoints():
    builder, node, carriers = _builder_and_carriers([])
    item = join.instances(builder.atlas)[0]
    join._bind_instance(builder, item, carriers, "executable", join.JoinReport(), count=True)
    assert not [edge for edge in builder.atlas.out_edges(node) if edge.kind == "publishes"]
    assert any(
        entry.owner == node and entry.reason == "construction_context_unavailable"
        for entry in builder.atlas.unresolved
    )


@pytest.mark.ros
def test_shared_header_contexts_reach_only_the_constructing_executable(tmp_path: Path):
    package = tmp_path / "src" / "p"
    package.mkdir(parents=True)
    (package / "package.xml").write_text(
        "<package><name>p</name><version>0.0.0</version></package>"
    )
    (package / "CMakeLists.txt").write_text(
        "project(p)\nadd_executable(first first.cpp)\nadd_executable(second second.cpp)\n"
    )
    (package / "pipe.hpp").write_text(PIPE)
    for name, input_name, output in [("first", "one", "two"), ("second", "three", "four")]:
        (package / f"{name}.cpp").write_text(
            '#include "pipe.hpp"\nint main() { auto node = std::make_shared<Pipe>('
            f'"{name}", "{input_name}", "{output}"); }}\n'
        )
    (package / "both.launch.py").write_text(
        "from launch import LaunchDescription\nfrom launch_ros.actions import Node\n"
        "def generate_launch_description():\n"
        '    return LaunchDescription([Node(package="p", executable="first"), '
        'Node(package="p", executable="second")])\n'
    )
    roots = (RootSpec("src", str(tmp_path / "src")),)
    config = Config(
        roots=roots,
        profiles=(ProfileSpec("p", entrypoints=(Entrypoint("launch", "p", "both.launch.py"),)),),
        out_dir=str(tmp_path / "out"),
        deterministic=True,
    )
    atlas = extract.run(config, Workspace.discover(roots))
    for name, output in [("first", "two"), ("second", "four")]:
        assert {
            edge.dst for edge in atlas.out_edges(f"node:/{name}") if edge.kind == "publishes"
        } == {f"topic:/{output}"}
    assert schema.validate(atlas.to_json()) == []


def test_constructor_parameters_and_frames_keep_the_endpoint_context():
    unit = cpp.parse_text(
        """
class Host : public rclcpp::Node {
public:
  Host(const std::string & name, const std::string & parameter, const std::string & frame)
  : Node(name) {
    declare_parameter(parameter, true);
    create_publisher<Message>(frame, 10);
    buffer_->lookupTransform(frame, "base", time);
  }
};
int main() {
  auto first = std::make_shared<Host>("first", "enabled", "map");
  auto second = std::make_shared<Host>("second", "active", "odom");
}
""",
        path="host.cpp",
        pkg="p",
        relpath="host.cpp",
    )
    cpp.analyse([unit])
    report = cpp.file_report(unit)
    contexts = {row["name"]: row["construction"] for row in report["nodes"]}
    assert {(row["name"], row.get("construction")) for row in report["parameters"]} == {
        ("enabled", contexts["first"]),
        ("active", contexts["second"]),
    }
    assert {(row["target"], row.get("construction")) for row in report["tf"]["lookups"]} == {
        ("map", contexts["first"]),
        ("odom", contexts["second"]),
    }
    assert not report["unresolved"]
    cpp.analyse([unit])
    assert cpp.file_report(unit) == report


def test_context_cloning_preserves_wrappers_lifecycle_and_read_ownership():
    unit = cpp.parse_text(
        """
void declareOrGetParameter(const std::string name, bool value) {
  declare_parameter(name, value);
}
class Host : public rclcpp_lifecycle::LifecycleNode {
public:
  Host(const std::string & name, const std::string & parameter, const std::string & read)
  : LifecycleNode(name) {
    declareOrGetParameter(parameter, true);
    get_parameter(read, enabled_);
    create_publisher<Message>("out", 10);
  }
};
int main() {
  auto first = std::make_shared<Host>("first", "enabled", "active");
  auto second = std::make_shared<Host>("second", "active", "enabled");
}
""",
        path="host.cpp",
        pkg="p",
        relpath="host.cpp",
    )
    cpp.analyse([unit])
    report = cpp.file_report(unit)
    assert {row["name"] for row in report["parameters"]} == {"enabled", "active"}
    assert all(not row["reads"] for row in report["parameters"])
    assert all(row.get("publisher_kind") == "lifecycle" for row in report["endpoints"])
    cpp.analyse([unit])
    assert cpp.file_report(unit) == report


def test_constructor_members_reach_out_of_line_methods_per_context():
    header = cpp.parse_text(
        """
class Host : public rclcpp::Node {
public:
  Host(const std::string & name, const std::string & topic) : Node(name), topic_(topic) {}
  void connect();
  std::string topic_;
};
""",
        path="host.hpp",
        pkg="p",
        relpath="host.hpp",
    )
    source = cpp.parse_text(
        "void Host::connect() { create_publisher<Message>(topic_, 10); }",
        path="host.cpp",
        pkg="p",
        relpath="host.cpp",
    )
    main = cpp.parse_text(
        '#include "host.hpp"\nint main() { auto a = std::make_shared<Host>("a", "one"); '
        'auto b = std::make_shared<Host>("b", "two"); }',
        path="main.cpp",
        pkg="p",
        relpath="main.cpp",
    )
    cpp.analyse([header, source, main])
    contexts = {row["name"]: row["construction"] for row in cpp.file_report(main)["nodes"]}
    report = cpp.file_report(source)
    assert {(row["name"], row.get("construction")) for row in report["endpoints"]} == {
        ("one", contexts["a"]),
        ("two", contexts["b"]),
    }
    assert not report["unresolved"]
    cpp.analyse([header, source, main])
    assert cpp.file_report(source) == report


def test_identical_constructor_content_stays_ambiguous_without_an_ordinal():
    unit = cpp.parse_text(
        PIPE + 'int main() { auto a = std::make_shared<Pipe>("same", "in", "out"); '
        'auto b = std::make_shared<Pipe>("same", "in", "out"); }',
        path="main.cpp",
        pkg="p",
        relpath="main.cpp",
    )
    cpp.analyse([unit])
    rows = cpp.file_report(unit)["nodes"]
    assert rows and all(row.get("construction_ambiguous") for row in rows)
    builder, node, carriers = _builder_and_carriers([("first", "a")])
    carriers[0].nodes[0]["construction_ambiguous"] = True
    join._bind_instance(
        builder,
        join.instances(builder.atlas)[0],
        carriers,
        "executable",
        join.JoinReport(),
        count=True,
    )
    assert not [edge for edge in builder.atlas.out_edges(node) if edge.kind == "publishes"]
    assert any(row.reason == "multiple_constructed_nodes" for row in builder.atlas.unresolved)


@pytest.mark.parametrize("reverse", [False, True])
def test_out_of_line_context_members_are_independent_of_unit_order(reverse):
    header = cpp.parse_text(
        "class Host : public rclcpp::Node { public: "
        "Host(const std::string & name) : Node(name) {} };",
        path="host.hpp",
        pkg="p",
        relpath="host.hpp",
    )
    connect = cpp.parse_text(
        "void Host::connect() { create_publisher<Message>(topic_, 10); }",
        path="connect.cpp",
        pkg="p",
        relpath="connect.cpp",
    )
    configure = cpp.parse_text(
        'void Host::configure() { topic_ = std::string("out"); }',
        path="configure.cpp",
        pkg="p",
        relpath="configure.cpp",
    )
    main = cpp.parse_text(
        '#include "host.hpp"\nint main() { auto node = std::make_shared<Host>("host"); }',
        path="main.cpp",
        pkg="p",
        relpath="main.cpp",
    )
    units = [header, *([configure, connect] if reverse else [connect, configure]), main]
    cpp.analyse(units)
    report = cpp.file_report(connect)
    assert report["endpoints"][0]["name"] == "out"
    assert not report["unresolved"]
    cpp.analyse(units)
    assert cpp.file_report(connect) == report


TALKER = """
#include <rclcpp/rclcpp.hpp>
namespace demo {
class Talker : public rclcpp::Node {
public:
  explicit Talker(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("talker", options) {
    pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
  }
};
}
"""
TALKER_MAIN = (
    '#include "demo/talker.hpp"\nint main() { rclcpp::spin(std::make_shared<demo::Talker>()); }\n'
)
TALKER_COMPONENT = '#include "demo/talker.hpp"\nRCLCPP_COMPONENTS_REGISTER_NODE(demo::Talker)\n'


def test_a_header_component_constructed_in_main_keeps_its_unscoped_records():
    units = [
        cpp.parse_text(text, path=relpath, pkg="demo", relpath=relpath)
        for relpath, text in (
            ("include/demo/talker.hpp", TALKER),
            ("src/main.cpp", TALKER_MAIN),
            ("src/component.cpp", TALKER_COMPONENT),
        )
    ]
    cpp.analyse(units, include_dirs={"demo": ["include"]})
    report = cpp.file_report(units[0])
    context = cpp.file_report(units[1])["nodes"][0]["construction"]
    assert {row.get("construction") for row in report["endpoints"]} == {None, context}
    assert [row.get("construction") for row in report["nodes"]] == [None]


def test_a_local_component_constructed_beside_another_class_keeps_its_unscoped_records():
    unit = cpp.parse_text(
        PIPE
        + """
class Relay : public rclcpp::Node {
public:
  Relay() : Node("relay") { create_publisher<Message>("relayed", 10); }
};
int main() {
  auto a = std::make_shared<Pipe>("first", "one", "two");
  auto b = std::make_shared<Relay>();
}
RCLCPP_COMPONENTS_REGISTER_NODE(Relay)
""",
        path="main.cpp",
        pkg="p",
        relpath="main.cpp",
    )
    cpp.analyse([unit])
    report = cpp.file_report(unit)
    relayed = [row for row in report["endpoints"] if row["name"] == "relayed"]
    assert None in {row.get("construction") for row in relayed}
    assert all(row.get("construction") for row in report["endpoints"] if row["name"] != "relayed")


@pytest.mark.ros
def test_a_component_also_constructed_in_main_binds_in_both_loads(tmp_path: Path):
    package = tmp_path / "src" / "demo"
    (package / "include" / "demo").mkdir(parents=True)
    (package / "src").mkdir()
    (package / "launch").mkdir()
    (package / "package.xml").write_text(
        "<package><name>demo</name><version>0.0.0</version></package>"
    )
    (package / "CMakeLists.txt").write_text(
        "project(demo)\n"
        "add_library(talker_component SHARED src/component.cpp)\n"
        "target_include_directories(talker_component PUBLIC include)\n"
        'rclcpp_components_register_nodes(talker_component "demo::Talker")\n'
        "add_executable(talker src/main.cpp)\n"
        "target_include_directories(talker PUBLIC include)\n"
    )
    (package / "include" / "demo" / "talker.hpp").write_text(TALKER)
    (package / "src" / "main.cpp").write_text(TALKER_MAIN)
    (package / "src" / "component.cpp").write_text(TALKER_COMPONENT)
    (package / "launch" / "comp.launch.py").write_text(
        "from launch import LaunchDescription\n"
        "from launch_ros.actions import ComposableNodeContainer\n"
        "from launch_ros.descriptions import ComposableNode\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription([ComposableNodeContainer(\n"
        '        package="rclcpp_components", executable="component_container",\n'
        '        name="container", namespace="",\n'
        "        composable_node_descriptions=[\n"
        '            ComposableNode(package="demo", plugin="demo::Talker", name="talker")])])\n'
    )
    roots = (RootSpec("src", str(tmp_path / "src")),)
    config = Config(
        roots=roots,
        profiles=(
            ProfileSpec("comp", entrypoints=(Entrypoint("launch", "demo", "comp.launch.py"),)),
            ProfileSpec("exe", entrypoints=(Entrypoint("run", "demo", "talker"),)),
        ),
        out_dir=str(tmp_path / "out"),
        deterministic=True,
    )
    atlas = extract.run(config, Workspace.discover(roots))
    published = [edge for edge in atlas.out_edges("node:/talker") if edge.kind == "publishes"]
    assert {edge.dst for edge in published} == {"topic:/chatter"}
    assert {fact.profile for edge in published for fact in edge.facts} >= {
        "profile:comp",
        "profile:exe",
    }
    assert not [row for row in atlas.unresolved if row.reason == "construction_context_unavailable"]
    assert schema.validate(atlas.to_json()) == []
