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
    Helper helper(node_, name);
  }
};
}
PLUGINLIB_EXPORT_CLASS(demo::Critic, demo::Base)
"""


@pytest.mark.cpp
def test_a_plugin_prefixes_its_parameters_with_the_instance_name():
    """H3: `name + ".scale"` in a plugin's setup method is a suffix of its instance and not
    a node parameter whose name is built at run time."""
    unit = cpp.parse_text(PLUGIN, path="critic.cpp")
    cpp.analyse([unit])
    report = cpp.file_report(unit)
    suffixes = {row["suffix"] for row in report["plugin_parameters"]}
    assert {"scale", "weight"} <= suffixes
    assert not [row for row in report["unresolved"] if row["want"] == "param_name"]


def test_a_subscript_is_an_unknown_piece_and_a_literal_inside_it_hides_every_key():
    assert not join.could_name('ids_[i] + ".plugin"', "FollowPath.scale")
    assert join.could_name('ids_[i] + ".plugin"', "FollowPath.plugin")
    assert join.could_name('names_["gain_key"]', "anything")
