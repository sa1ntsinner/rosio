"""Defects the sixth first run trial found, each with the source pattern that showed it."""

from __future__ import annotations

from pathlib import Path

import pytest

from rosio import extract, join, model, query, schema
from rosio.collect import cpp_code as cpp
from rosio.collect import py_code
from rosio.config import Config, Entrypoint, ProfileSpec, RootSpec
from rosio.resolve import ros_args
from rosio.workspace import Workspace

PACKAGE_XML = "<package><name>{0}</name><version>0.0.0</version></package>"


def package(tmp_path: Path, name: str, files: dict[str, str]) -> Path:
    root = tmp_path / "src" / name
    root.mkdir(parents=True)
    (root / "package.xml").write_text(PACKAGE_XML.format(name))
    for relpath, text in files.items():
        path = root / relpath
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
    return root


def config(tmp_path: Path, *entrypoints: Entrypoint, name: str = "p") -> Config:
    return Config(
        roots=(RootSpec("src", str(tmp_path / "src")),),
        profiles=(ProfileSpec(name, entrypoints=entrypoints),),
        out_dir=str(tmp_path / "out"),
        deterministic=True,
    )


def run_twice(cfg: Config) -> model.Atlas:
    atlas = extract.run(cfg, Workspace.discover(cfg.roots))
    assert schema.validate(atlas.to_json()) == []
    again = extract.run(cfg, Workspace.discover(cfg.roots))
    assert again.canonical_bytes() == atlas.canonical_bytes()
    return atlas


def launch(*nodes: str) -> str:
    return (
        "from launch import LaunchDescription\n"
        "from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes\n"
        "from launch_ros.descriptions import ComposableNode\n"
        "from launch.conditions import IfCondition\n"
        "def generate_launch_description():\n"
        f"    return LaunchDescription([{', '.join(nodes)}])\n"
    )


def with_yaml(executable: str) -> str:
    """A node of package `pk` handed the `b.yaml` its share directory holds."""
    return launch(
        f'Node(package="pk", executable="{executable}", name="{executable}", '
        'parameters=[__import__("os").path.join(__import__("ament_index_python.packages")'
        '.packages.get_package_share_directory("pk"), "b.yaml")])'
    )


COMPONENT = """#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "nav2_util/twist_publisher.hpp"
namespace demo {
class Helper {
public:
  explicit Helper(rclcpp::Node * node) {
    pub_ = node->create_publisher<std_msgs::msg::String>("status", 10);
  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};
class Server : public rclcpp::Node {
public:
  explicit Server(const rclcpp::NodeOptions & o) : Node("server", o), helper_(this) {
    action_ = rclcpp_action::create_server<Go>(this, "go", nullptr, nullptr, nullptr);
    client_ = rclcpp_action::create_client<Go>(this, "elsewhere");
    twist_ = std::make_unique<nav2_util::TwistPublisher>(this, "cmd_vel", 1);
  }
  Helper helper_;
  rclcpp_action::Server<Go>::SharedPtr action_;
  rclcpp_action::Client<Go>::SharedPtr client_;
  std::unique_ptr<nav2_util::TwistPublisher> twist_;
};
}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(demo::Server)
"""


@pytest.mark.cpp
def test_a_component_reaches_its_helpers_actions_and_twist_publisher(tmp_path: Path):
    """H2, H6 and M5: a composed nav2 server lost every endpoint a helper class makes, every
    action, and the `cmd_vel` its TwistPublisher makes, so no node published `cmd_vel`."""
    package(
        tmp_path,
        "p",
        {
            "CMakeLists.txt": "project(p)\nadd_library(server SHARED src/server.cpp)\n"
            'rclcpp_components_register_nodes(server "demo::Server")\n',
            "src/server.cpp": COMPONENT,
            "run.launch.py": launch(
                'ComposableNodeContainer(name="box", namespace="", package="rclcpp_components",'
                ' executable="component_container", composable_node_descriptions=['
                'ComposableNode(package="p", plugin="demo::Server", name="server")])'
            ),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    edges = {(edge.kind, edge.dst) for edge in atlas.out_edges("node:/server")}
    assert ("publishes", "topic:/status") in edges
    assert ("publishes", "topic:/cmd_vel") in edges
    assert ("serves", "action:/go") in edges
    assert ("calls", "action:/elsewhere") in edges


def test_query_names_the_blind_spots_when_no_endpoint_edge_answers(tmp_path: Path):
    package(
        tmp_path,
        "p",
        {
            "p/__init__.py": "",
            "p/talk.py": "from rclpy.node import Node\n"
            "class Talk(Node):\n"
            "    def __init__(self):\n"
            "        super().__init__('talk')\n"
            "        self.create_publisher(String, self.topic(), 10)\n"
            "        self.create_subscription(String, 'cmd_vel_in', None, 10)\n",
            "setup.py": "from setuptools import setup\nsetup(name='p', entry_points="
            "{'console_scripts': ['talk = p.talk:main']})\n",
            "run.launch.py": launch('Node(package="p", executable="talk", name="talk")'),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    assert any(row.want == "topic_name" for row in atlas.unresolved)
    answer = query.query(atlas, "which nodes publish cmd_vel")
    assert answer.data.get("blind_spots")
    assert "rosio unresolved --want topic_name" in answer.text()


EMPTY_FRAME = """#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
class Pose : public rclcpp::Node {
public:
  Pose() : Node("pose") {}
  void send() {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "";
    t.child_frame_id = "child";
    broadcaster_->sendTransform(t);
  }
  void other() {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "world";
    t.child_frame_id = "child";
    t.transform.translation.x = 1.0;
    broadcaster_->sendTransform(t);
  }
  void third() {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "world";
    t.child_frame_id = "child";
    t.transform.translation.x = 2.0;
    broadcaster_->sendTransform(t);
  }
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pose>());
}
"""


@pytest.mark.cpp
def test_an_empty_frame_is_a_blind_spot_and_two_origins_are_two_edges(tmp_path: Path):
    """H4: `frame_id = ""` minted `frame:` and two literal origins for one pair raised."""
    package(
        tmp_path,
        "p",
        {
            "CMakeLists.txt": "project(p)\nadd_executable(pose pose.cpp)\n",
            "pose.cpp": EMPTY_FRAME,
            "run.launch.py": launch('Node(package="p", executable="pose")'),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    assert "frame:" not in atlas.entities
    assert any(row.reason == cpp.EMPTY_FRAME for row in atlas.unresolved)
    assert extract.model.completeness(atlas)["complete"]


@pytest.mark.cpp
def test_a_file_the_atlas_refuses_is_a_refused_entity(tmp_path: Path, monkeypatch):
    package(
        tmp_path,
        "p",
        {
            "CMakeLists.txt": "project(p)\nadd_executable(pose pose.cpp)\n",
            "pose.cpp": EMPTY_FRAME,
            "run.launch.py": launch('Node(package="p", executable="pose")'),
        },
    )

    def refuse(self, *args, **kwargs):
        raise model.ContractError("folded two sites")

    monkeypatch.setattr(cpp.CppCodeCollector, "_emit", refuse)
    cfg = config(tmp_path, Entrypoint("launch", "p", "run.launch.py"))
    atlas = extract.run(cfg, Workspace.discover(cfg.roots))
    refused = atlas.entities["file:p|pose.cpp"]
    assert refused.attrs["refused_because"] == "cpp_refused_by_atlas"
    assert not refused.facts
    assert schema.validate(atlas.to_json()) == []


@pytest.mark.cpp
def test_a_file_refused_halfway_keeps_nothing_it_wrote_and_the_pass_goes_on(
    tmp_path: Path, monkeypatch
):
    """Review 1: a refusal after the file fact was written collided with `loadable` true,
    escaped, and refused the whole C++ pass with the partial records left on the file."""
    package(
        tmp_path,
        "p",
        {
            "CMakeLists.txt": "project(p)\nadd_executable(pose pose.cpp)\n"
            "add_executable(talk talk.cpp)\n",
            "pose.cpp": EMPTY_FRAME,
            "talk.cpp": "#include <rclcpp/rclcpp.hpp>\nclass Talk : public rclcpp::Node {\n"
            'public:\n  Talk() : Node("talk") {\n'
            '    pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);\n  }\n'
            "  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;\n};\n"
            "int main(int c, char ** v) { rclcpp::init(c, v); "
            "rclcpp::spin(std::make_shared<Talk>()); }\n",
            "run.launch.py": launch('Node(package="p", executable="pose")'),
        },
    )

    def refuse(self, *args, **kwargs):
        raise model.ContractError("folded two sites")

    monkeypatch.setattr(cpp.CppCodeCollector, "_broadcast", refuse)
    cfg = config(tmp_path, Entrypoint("launch", "p", "run.launch.py"))
    atlas = extract.run(cfg, Workspace.discover(cfg.roots))
    assert schema.validate(atlas.to_json()) == []
    assert extract.model.completeness(atlas)["complete"]
    refused = atlas.entities["file:p|pose.cpp"]
    assert refused.attrs["loadable"] is False
    assert refused.attrs["refused_because"] == "cpp_refused_by_atlas"
    assert not refused.facts
    assert "sym:p|pose.cpp|Pose::send" not in atlas.entities
    assert not [
        key
        for key, entity in atlas.entities.items()
        if key.startswith("sym:p|pose.cpp|") and "coded" in entity.layers
    ]
    assert "coded" in atlas.entities["file:p|talk.cpp"].layers
    assert any(row.rule == "extract.input_refused" for row in atlas.findings)


@pytest.mark.parametrize(
    "arguments",
    [
        ["--x", "1", "--frame-id", "base_link", "--child-frame-id", "laser"],
        ["1", "0", "0", "0", "0", "0", "base_link", "laser"],
    ],
)
def test_static_transform_publisher_arguments_are_its_parameters(arguments):
    """H5: the frame pair a static publisher is handed is its `frame_id` and
    `child_frame_id`, never the `frame` and `child` defaults its source declares."""
    found = ros_args.program_assignments("tf2_ros", "static_transform_publisher", arguments)
    assert found["frame_id"][0] == "base_link"
    assert found["child_frame_id"][0] == "laser"
    assert found["translation.x"][0] == "1.0"
    assert ros_args.program_assignments("p", "other", arguments) == {}


def test_roll_pitch_yaw_become_a_quaternion():
    found = ros_args.program_assignments(
        "tf2_ros", "static_transform_publisher", ["--yaw", "3.141592653589793"]
    )
    assert abs(float(found["rotation.z"][0]) - 1.0) < 1e-9
    assert abs(float(found["rotation.w"][0])) < 1e-9


def test_static_transform_publisher_in_a_launch_file_sets_its_frame_parameters(tmp_path: Path):
    package(
        tmp_path,
        "p",
        {
            "run.launch.py": launch(
                'Node(package="tf2_ros", executable="static_transform_publisher", '
                'name="laser_tf", arguments=["--frame-id", "base_link", '
                '"--child-frame-id", "laser"])'
            ),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    values = {
        ids_name: entity.effective["profile:p"]["value"].value
        for ids_name, entity in atlas.entities.items()
        if ids_name.startswith("param:/laser_tf|") and "profile:p" in entity.effective
    }
    assert values.get("param:/laser_tf|frame_id") == "base_link"
    assert values.get("param:/laser_tf|child_frame_id") == "laser"
    assert "frame:child" not in atlas.entities


STEREO = (
    'ComposableNodeContainer(name="box", namespace="", package="rclcpp_components", '
    'executable="component_container", composable_node_descriptions=[]), '
    'LoadComposableNodes(target_container="box", condition=IfCondition("false"), '
    'composable_node_descriptions=[ComposableNode(package="p", plugin="demo::Disparity")])'
)


def test_a_node_only_a_gated_off_site_loads_is_not_an_entity(tmp_path: Path):
    """Trial five item 12: an unnamed component behind a false condition became a synthetic
    node with a name blind spot, although no profile starts it."""
    package(tmp_path, "p", {"run.launch.py": launch(STEREO)})
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    assert not [key for key in atlas.entities if key.startswith("node:~p")]
    assert not [row for row in atlas.unresolved if row.want == "node_name"]
    assert any(
        entity.kind == "launchent" and "Disparity" in key for key, entity in atlas.entities.items()
    )


def test_a_type_through_a_dotted_module_import_names_its_package():
    """M8: `import sensor_msgs.msg` then `sensor_msgs.msg.Image` was `msg/msg/Image`."""
    report = py_code.analyse_source(
        "import sensor_msgs.msg\nimport std_msgs.msg as sm\nfrom rclpy.node import Node\n"
        "class Flip(Node):\n"
        "    def __init__(self):\n"
        "        super().__init__('flip')\n"
        "        self.create_publisher(sensor_msgs.msg.Image, 'image', 10)\n"
        "        self.create_publisher(sm.String, 'text', 10)\n",
        path="flip.py",
        package="p",
        package_relative_path="flip.py",
    )
    types = {row["name"]: row["type"] for row in report["endpoints"]}
    assert types["image"] == "sensor_msgs/msg/Image"
    assert types["text"] == "std_msgs/msg/String"


def test_a_python_broadcaster_reads_every_transform_it_sends():
    """M9: a list of transforms, and a method returning one, named no frame."""
    report = py_code.analyse_source(
        "from rclpy.node import Node\nfrom tf2_ros import StaticTransformBroadcaster\n"
        "from geometry_msgs.msg import TransformStamped\n"
        "class Tf(Node):\n"
        "    def __init__(self):\n"
        "        super().__init__('tf')\n"
        "        self.b = StaticTransformBroadcaster(self)\n"
        "        self.b.sendTransform([self.make('base_link', 'laser'), self.camera()])\n"
        "    def camera(self):\n"
        "        t = TransformStamped()\n"
        "        t.header.frame_id = 'base_link'\n"
        "        t.child_frame_id = 'camera'\n"
        "        return t\n"
        "    def make(self, parent, child):\n"
        "        return parent\n",
        path="tf.py",
        package="p",
        package_relative_path="tf.py",
    )
    pairs = {(row["parent"], row["child"]) for row in report["tf"]["broadcasts"]}
    assert ("base_link", "camera") in pairs
    assert any(row["want"] == "frame_id" for row in report["unresolved"])


PLUGIN = """#include <pluginlib/class_list_macros.hpp>
namespace demo {
class Critic : public Base {
public:
  void initialize(const std::string & name) {
    node_->declare_parameter(name + ".scale", 1.0);
    node_->declare_parameter(getName() + ".weight", 2.0);
    plugin_name_ = name;
    held_ = std::make_unique<Held>(node_, plugin_name_);
    handed_ = std::make_unique<Handed>(node_, name);
  }
  std::string plugin_name_;
};
}
PLUGINLIB_EXPORT_CLASS(demo::Critic, demo::Base)
"""

PLUGIN_HELPERS = """namespace demo {
class Held {
public:
  Held(rclcpp::Node::SharedPtr node, const std::string & prefix) {
    node->declare_parameter(prefix + ".gain", 1.0);
  }
};
class Handed {
public:
  Handed(rclcpp::Node::SharedPtr node, const std::string & prefix) {
    node->declare_parameter(prefix + ".other", 1.0);
  }
};
}
"""


@pytest.mark.cpp
def test_a_plugin_prefixes_its_parameters_with_the_instance_name():
    """H3: `name + ".scale"` in a plugin's setup method is a suffix of its instance and not
    a node parameter whose name is built at run time, and so is the name of a helper the
    plugin hands its instance name, through a member or as the argument it was given."""
    unit = cpp.parse_text(PLUGIN, path="critic.cpp")
    helpers = cpp.parse_text(PLUGIN_HELPERS, path="helpers.cpp")
    cpp.analyse([unit, helpers])
    report = cpp.file_report(unit)
    suffixes = {row["suffix"] for row in report["plugin_parameters"]}
    assert {"scale", "weight"} <= suffixes
    assert not [row for row in report["unresolved"] if row["want"] == "param_name"]
    handed = cpp.file_report(helpers)
    assert {(row["suffix"], row["lookup_name"]) for row in handed["plugin_parameters"]} == {
        ("gain", "demo::Critic"),
        ("other", "demo::Critic"),
    }
    assert not handed["parameters"]
    assert not [row for row in handed["unresolved"] if row["want"] == "param_name"]


NOT_A_PLUGIN = """#include <rclcpp/rclcpp.hpp>
class Bridge : public rclcpp::Node {
public:
  Bridge() : Node("bridge") {
    configure("lidar");
    declare_parameter(getName() + ".groot", false);
  }
  void configure(const std::string & ns) {
    declare_parameter(ns + ".rate", 1.0);
  }
  std::string getName() { return std::string("navigate_to_pose"); }
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Bridge>());
}
"""


@pytest.mark.cpp
def test_configure_and_get_name_of_a_class_no_plugin_registers_name_nothing(tmp_path: Path):
    """Review 2: a node's own `configure(std::string)` and `getName()` made `ns + ".rate"` a
    node parameter `rate`, so `lidar.rate` in the yaml was called undeclared."""
    package(
        tmp_path,
        "pk",
        {
            "CMakeLists.txt": "project(pk)\nadd_executable(bridge bridge.cpp)\n",
            "bridge.cpp": NOT_A_PLUGIN,
            "b.yaml": "bridge:\n  ros__parameters:\n    lidar:\n      rate: 5.0\n",
            "run.launch.py": with_yaml("bridge"),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "pk", "run.launch.py")))
    assert "param:/bridge|rate" not in atlas.entities
    assert "param:/bridge|groot" not in atlas.entities
    assert atlas.entities["param:/bridge|lidar.rate"].state != "undeclared"
    assert not [row for row in atlas.findings if row.rule == "param.set_undeclared"]
    blind = {row.expr for row in atlas.unresolved if row.want == "param_name"}
    assert {'ns + ".rate"', 'getName() + ".groot"'} <= blind


@pytest.mark.cpp
def test_a_plugin_another_file_registers_owns_the_suffixes_its_methods_declare():
    """The registration is often in a file of its own, so the package decides, not the file."""
    critic = cpp.parse_text(
        "namespace demo {\nvoid Critic::initialize(const std::string & name) {\n"
        '  node_->declare_parameter(name + ".scale", 1.0);\n}\n}\n',
        path="critic.cpp",
    )
    plugins = cpp.parse_text(
        "#include <pluginlib/class_list_macros.hpp>\n"
        "PLUGINLIB_EXPORT_CLASS(demo::Critic, demo::Base)\n",
        path="plugins.cpp",
    )
    cpp.analyse([critic, plugins])
    report = cpp.file_report(critic)
    assert [(row["suffix"], row["lookup_name"]) for row in report["plugin_parameters"]] == [
        ("scale", "demo::Critic")
    ]
    assert "plugins" not in report


BRANCHED = """#include <rclcpp/rclcpp.hpp>
class Imu : public rclcpp::Node {
public:
  Imu() : Node("imu") { setup("a"); }
  void setup(const std::string & id) {
    std::string key;
    if (front_) key = "front." + id; else key = "rear." + id;
    declare_parameter(key, 0.0);
  }
  bool front_ = true;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Imu>());
}
"""


@pytest.mark.cpp
def test_a_name_two_branches_build_hides_the_keys_of_both(tmp_path: Path):
    """Review 3: the blind spot kept the last assignment, so `front.a` was undeclared."""
    package(
        tmp_path,
        "pk",
        {
            "CMakeLists.txt": "project(pk)\nadd_executable(imu imu.cpp)\n",
            "imu.cpp": BRANCHED,
            "b.yaml": "imu:\n  ros__parameters:\n    front:\n      a: 5.0\n"
            "    rear:\n      a: 1.0\n",
            "run.launch.py": with_yaml("imu"),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "pk", "run.launch.py")))
    assert atlas.entities["param:/imu|front.a"].state == "other"
    assert atlas.entities["param:/imu|rear.a"].state == "other"
    assert not [row for row in atlas.findings if row.rule == "param.set_undeclared"]


def test_a_returned_transform_is_read_in_its_own_method_and_class():
    """Review 4: the method's `child` was read from the caller, and a method of another
    class with the same name answered for `self`."""
    report = py_code.analyse_source(
        "from rclpy.node import Node\nfrom tf2_ros import StaticTransformBroadcaster\n"
        "from geometry_msgs.msg import TransformStamped\n"
        "class Tf(Node):\n"
        "    def __init__(self):\n"
        "        super().__init__('tf')\n"
        "        self.b = StaticTransformBroadcaster(self)\n"
        "        child = 'imu_link'\n"
        "        self.b.sendTransform(self.laser_tf())\n"
        "        self.b.sendTransform(self.camera_tf())\n"
        "    def laser_tf(self):\n"
        "        child = 'laser'\n"
        "        t = TransformStamped()\n"
        "        t.header.frame_id = 'base_link'\n"
        "        t.child_frame_id = child\n"
        "        return t\n"
        "class Other(Node):\n"
        "    def camera_tf(self):\n"
        "        t = TransformStamped()\n"
        "        t.header.frame_id = 'base_link'\n"
        "        t.child_frame_id = 'camera'\n"
        "        return t\n",
        path="tf.py",
        package="p",
        package_relative_path="tf.py",
    )
    pairs = {(row["parent"], row["child"]) for row in report["tf"]["broadcasts"]}
    assert pairs == {("base_link", "laser")}
    assert [row["expr"] for row in report["unresolved"] if row["want"] == "frame_id"] == [
        "self.camera_tf()"
    ]


def _loc(blocks: list[str]) -> str:
    return (
        "#include <rclcpp/rclcpp.hpp>\n#include <tf2_ros/transform_broadcaster.h>\n"
        'class Loc : public rclcpp::Node {\npublic:\n  Loc() : Node("loc") {}\n'
        "  void tick() {\n" + "".join(blocks) + "  }\n  bool docked_, moving_;\n"
        "  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;\n"
        "  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr a_, b_;\n};\n"
        "int main(int argc, char ** argv) {\n  rclcpp::init(argc, argv);\n"
        "  rclcpp::spin(std::make_shared<Loc>());\n}\n"
    )


DOCKED = """    if (docked_) {
      geometry_msgs::msg::TransformStamped d;
      d.header.frame_id = "map";
      d.child_frame_id = "base_link";
      br_->sendTransform(d);
    }
    b_ = create_publisher<std_msgs::msg::String>("out", rclcpp::QoS(1).transient_local());
    d_ = create_subscription<std_msgs::msg::String>("in", rclcpp::QoS(1).transient_local(), cb);
"""
MOVING = """    if (moving_) {
      geometry_msgs::msg::TransformStamped t;
      t.header.frame_id = "map";
      t.child_frame_id = "base_link";
      br_->sendTransform(t);
    }
    a_ = create_publisher<std_msgs::msg::String>("out", 10);
    c_ = create_subscription<std_msgs::msg::String>("in", 10, cb);
"""


@pytest.mark.cpp
@pytest.mark.parametrize("kind", ["tf_parent", "publishes", "subscribes"])
def test_a_site_inserted_before_another_moves_no_edge_id(tmp_path: Path, kind: str):
    """Review 5: the first site of one symbol kept the plain key, so a sibling written above
    it took that key and moved the edge id of the one already there."""

    def handles(name: str, blocks: list[str]) -> dict[str, str]:
        root = tmp_path / name
        package(
            root,
            "p",
            {
                "CMakeLists.txt": "project(p)\nadd_executable(loc loc.cpp)\n",
                "loc.cpp": _loc(blocks),
                "run.launch.py": launch('Node(package="p", executable="loc", name="loc")'),
            },
        )
        atlas = run_twice(config(root, Entrypoint("launch", "p", "run.launch.py")))
        return {
            key: model.dumps([fact.values for fact in edge.facts])
            for key, edge in atlas.edges.items()
            if edge.kind == kind
        }

    alone = handles("alone", [MOVING])
    both = handles("both", [DOCKED, MOVING])
    assert alone
    assert alone.items() < both.items()


ACTIONS = """#include <rclcpp/rclcpp.hpp>
class Srv : public nav2::LifecycleNode {
public:
  Srv() : nav2::LifecycleNode("srv") {}
  void on_configure() {
    a_ = create_action_server<Go>("member_server", cb, nullptr);
    b_ = nav2::interfaces::create_action_server<Go>(shared_from_this(), "free_server", cb);
    c_ = nav2::interfaces::create_action_client<Go>(node_ptr_, "free_client");
    d_ = this->create_action_client<Go>("member_client");
    e_ = nav2::interfaces::create_action_client<Go>(get_node_base_interface(), "iface");
  }
};
"""


@pytest.mark.cpp
def test_a_nav2_action_factory_reads_the_name_by_the_shape_of_the_call():
    """Review 6: the free function took the name from its first argument whenever the node
    handle was not one of a few spellings."""
    unit = cpp.parse_text(ACTIONS, path="srv.cpp")
    cpp.analyse([unit])
    report = cpp.file_report(unit)
    assert {(row["kind"], row["name"]) for row in report["endpoints"]} == {
        ("action_server", "member_server"),
        ("action_server", "free_server"),
        ("action_client", "free_client"),
        ("action_client", "member_client"),
        ("action_client", "iface"),
    }
    assert not [row for row in report["unresolved"] if row["want"] == "action_name"]


BASE_NODE = """#pragma once
#include <rclcpp/rclcpp.hpp>
namespace demo {
class BaseNode : public rclcpp::Node {
public:
  explicit BaseNode(const std::string & n, const rclcpp::NodeOptions & o) : Node(n, o) {
    hb_ = create_publisher<std_msgs::msg::String>("heartbeat", 10);
  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hb_;
};
}
"""
DERIVED = """#include "p/base_node.hpp"
namespace demo {
class Server : public BaseNode {
public:
  explicit Server(const rclcpp::NodeOptions & o) : BaseNode("server", o) {
    pub_ = create_publisher<std_msgs::msg::String>("out", 10);
  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};
}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(demo::Server)
"""
UNRELATED = """#include "p/base_node.hpp"
namespace demo {
class Unrelated : public rclcpp::Node {
public:
  Unrelated() : Node("unrelated") { p_ = create_publisher<std_msgs::msg::String>("leak", 10); }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr p_;
};
}
int main(int c, char ** v) {
  rclcpp::init(c, v);
  rclcpp::spin(std::make_shared<demo::Unrelated>());
}
"""


@pytest.mark.cpp
def test_a_component_reaches_the_base_class_its_header_defines(tmp_path: Path):
    """Review 7: the component library binds the bases of the class it registers, and only
    those, so another node of the package lends it nothing."""
    package(
        tmp_path,
        "p",
        {
            "CMakeLists.txt": "project(p)\nadd_library(server SHARED src/server.cpp)\n"
            "add_executable(other src/other.cpp)\n"
            'rclcpp_components_register_nodes(server "demo::Server")\n',
            "include/p/base_node.hpp": BASE_NODE,
            "src/server.cpp": DERIVED,
            "src/other.cpp": UNRELATED,
            "run.launch.py": launch(
                'ComposableNodeContainer(name="box", namespace="", package="rclcpp_components",'
                ' executable="component_container", composable_node_descriptions=['
                'ComposableNode(package="p", plugin="demo::Server", name="server")])'
            ),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    edges = {(edge.kind, edge.dst) for edge in atlas.out_edges("node:/server")}
    assert ("publishes", "topic:/heartbeat") in edges
    assert ("publishes", "topic:/out") in edges
    assert ("publishes", "topic:/leak") not in edges


def test_an_angle_that_is_not_a_number_leaves_the_rotation_a_blind_spot(tmp_path: Path):
    """Review 8: `--yaw` that does not read dropped the rotation silently."""
    package(
        tmp_path,
        "p",
        {
            "run.launch.py": launch(
                'Node(package="tf2_ros", executable="static_transform_publisher", '
                'name="laser_tf", arguments=["--yaw", "half_turn", "--frame-id", "base_link", '
                '"--child-frame-id", "laser"])'
            ),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    rotation = atlas.entities["param:/laser_tf|rotation.w"]
    assert not rotation.effective
    blind = [row for row in atlas.unresolved if row.reason == ros_args.ANGLE_UNREADABLE]
    assert blind and all("--yaw half_turn" in row.expr for row in blind)
    assert "profile:p" in atlas.entities["param:/laser_tf|frame_id"].effective


def test_a_positional_rotation_is_cited_as_positional():
    found = ros_args.program_assignments(
        "tf2_ros", "static_transform_publisher", ["0", "0", "0", "1.0", "0", "0", "a", "b"]
    )
    assert found["rotation.z"][1] == "positional"
    flagged = ros_args.program_assignments("tf2_ros", "static_transform_publisher", ["--yaw", "1"])
    assert flagged["rotation.z"][1] == "--yaw"


EMPTY_PY_FRAME = """import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
class Tf(Node):
    def __init__(self):
        super().__init__('tf')
        self.pub = self.create_publisher(TransformStamped, 'pose_out', 10)
        self.b = TransformBroadcaster(self)
        t = TransformStamped()
        t.header.frame_id = ''
        t.child_frame_id = 'laser'
        self.b.sendTransform(t)
def main():
    rclpy.init(); rclpy.spin(Tf())
"""


def test_an_empty_python_frame_is_a_blind_spot_and_the_file_is_read(tmp_path: Path):
    """Review 9: `frame_id = ''` in python refused the whole file as a parse error."""
    package(
        tmp_path,
        "p",
        {
            "p/__init__.py": "",
            "p/tf.py": EMPTY_PY_FRAME,
            "setup.py": "from setuptools import setup\nsetup(name='p', entry_points="
            "{'console_scripts': ['tf = p.tf:main']})\n",
            "run.launch.py": launch('Node(package="p", executable="tf", name="tf")'),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    assert atlas.entities["file:p|p/tf.py"].attrs.get("loadable") is not False
    assert "frame:" not in atlas.entities
    edges = {(edge.kind, edge.dst) for edge in atlas.out_edges("node:/tf")}
    assert ("publishes", "topic:/pose_out") in edges
    assert ("broadcasts_tf", "frame:laser") in edges
    assert any(row.reason == "frame_empty" for row in atlas.unresolved)


def test_a_subscript_is_an_unknown_piece_and_a_literal_inside_it_hides_every_key():
    assert not join.could_name('ids_[i] + ".plugin"', "FollowPath.scale")
    assert join.could_name('ids_[i] + ".plugin"', "FollowPath.plugin")
    assert join.could_name('names_["gain_key"]', "anything")
