"""Defects the eighth first run trial found, each with the source pattern that showed it."""

from __future__ import annotations

from pathlib import Path

import pytest

from rosio import ids, model
from rosio.config import Entrypoint
from tests.test_seventh_trial import SETUP, config, findings, launch, package, run_twice

HEAD = (
    "from rclpy.node import Node\n"
    "from rclpy.action import ActionClient, ActionServer\n"
    "from sensor_msgs.msg import BatteryState as SensorBattery\n"
    "from fleet_msgs.msg import BatteryState\n"
    "from fleet_msgs.srv import SetMode\n"
    "from fleet_msgs.action import Dock, Undock\n"
    "from tf2_ros import Buffer, TransformListener\n"
    "from geometry_msgs.msg import Twist, TwistStamped\n"
)


def node(name: str, body: str) -> str:
    return (
        f"{HEAD}"
        "class N(Node):\n"
        "    def __init__(self):\n"
        f"        super().__init__('{name}')\n"
        f"{body}"
        "def main():\n"
        "    pass\n"
    )


def python_workspace(tmp_path: Path, nodes: dict[str, str], *extra: str) -> model.Atlas:
    files = {"p/__init__.py": "", "run.launch.py": ""}
    files.update({f"p/{name}.py": text for name, text in nodes.items()})
    files["setup.py"] = SETUP.format(", ".join(f"'{name} = p.{name}:main'" for name in nodes))
    files["run.launch.py"] = launch(
        *(f'Node(package="p", executable="{name}", name="{name}")' for name in nodes), *extra
    )
    package(tmp_path, "p", files)
    return run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))


# ---------------------------------------------------------------------------
# F1, two message types on one topic


def test_a_subscriber_of_another_message_type_is_an_error(tmp_path: Path):
    atlas = python_workspace(
        tmp_path,
        {
            "monitor": node(
                "monitor", "        self.create_publisher(BatteryState, 'battery', 10)\n"
            ),
            "planner": node(
                "planner",
                "        self.create_subscription(SensorBattery, 'battery', self.cb, 10)\n",
            ),
        },
    )
    (found,) = findings(atlas, "type.incompatible")
    assert found.severity == "error"
    assert found.subject == ids.build_topic("/battery")
    assert "fleet_msgs/msg/BatteryState" in found.say
    assert "planner.py:12" in found.say


def test_one_message_type_on_both_ends_is_not_a_type_mismatch(tmp_path: Path):
    atlas = python_workspace(
        tmp_path,
        {
            "monitor": node(
                "monitor", "        self.create_publisher(BatteryState, 'battery', 10)\n"
            ),
            "planner": node(
                "planner",
                "        self.create_subscription(BatteryState, 'battery', self.cb, 10)\n",
            ),
        },
    )
    assert findings(atlas, "type.incompatible") == []


def test_a_node_taking_either_of_two_types_by_a_parameter_is_not_a_mismatch(tmp_path: Path):
    body = (
        "        if self.declare_parameter('stamped', False).value:\n"
        "            self.create_subscription(TwistStamped, 'cmd_vel', self.cb, 10)\n"
        "        else:\n"
        "            self.create_subscription(Twist, 'cmd_vel', self.cb, 10)\n"
    )
    atlas = python_workspace(
        tmp_path,
        {
            "teleop": node("teleop", "        self.create_publisher(Twist, 'cmd_vel', 10)\n"),
            "base": node("base", body),
        },
    )
    assert findings(atlas, "type.incompatible") == []


def test_the_graph_names_both_types_of_a_split_channel(tmp_path: Path):
    from rosio.serve import tools

    atlas = python_workspace(
        tmp_path,
        {
            "monitor": node(
                "monitor", "        self.create_publisher(BatteryState, 'battery', 10)\n"
            ),
            "planner": node(
                "planner",
                "        self.create_subscription(SensorBattery, 'battery', self.cb, 10)\n",
            ),
        },
    )
    text = str(tools.graph(atlas))
    assert "types differ" in text
    assert "sensor_msgs/msg/BatteryState" in text


# ---------------------------------------------------------------------------
# F2, a composable node naming a class no library registers

DOCK_SERVER = """#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "fleet_msgs/action/dock.hpp"
namespace fleet_nodes {
class DockServer : public rclcpp::Node {
public:
  using Dock = fleet_msgs::action::Dock;
  explicit DockServer(const rclcpp::NodeOptions & o) : Node("dock_server", o) {
    server_ = rclcpp_action::create_server<Dock>(this, "dock", nullptr, nullptr, nullptr);
  }
  rclcpp_action::Server<Dock>::SharedPtr server_;
};
}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(fleet_nodes::DockServer)
"""


def _components(tmp_path: Path, plugin: str, client: str = "") -> model.Atlas:
    from tests.test_sixth_trial import launch as component_launch

    files = {
        "CMakeLists.txt": "project(p)\nadd_library(dock SHARED src/dock_server.cpp)\n"
        'rclcpp_components_register_nodes(dock "fleet_nodes::DockServer")\n',
        "src/dock_server.cpp": DOCK_SERVER,
        "run.launch.py": component_launch(
            'ComposableNodeContainer(name="box", namespace="", package="rclcpp_components",'
            ' executable="component_container", composable_node_descriptions=['
            f'ComposableNode(package="p", plugin="{plugin}", name="dock_server")])',
            *(['Node(package="p", executable="client", name="client")'] if client else []),
        ),
    }
    if client:
        files.update(
            {
                "p/__init__.py": "",
                "p/client.py": node("client", client),
                "setup.py": SETUP.format("'client = p.client:main'"),
            }
        )
    package(tmp_path, "p", files)
    return run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))


@pytest.mark.cpp
def test_a_component_class_no_library_registers_names_the_closest_one(tmp_path: Path):
    atlas = _components(tmp_path, "fleet_nodes::DockSever")
    (found,) = findings(atlas, "plugin.unresolved")
    assert "fleet_nodes::DockSever" in found.say
    assert "fleet_nodes::DockServer" in found.fix
    assert any(entry.reason == "component_class_unregistered" for entry in atlas.unresolved)


@pytest.mark.cpp
def test_a_registered_component_class_is_not_unresolved(tmp_path: Path):
    atlas = _components(tmp_path, "fleet_nodes::DockServer")
    assert findings(atlas, "plugin.unresolved") == []
    assert not any(entry.reason == "component_class_unregistered" for entry in atlas.unresolved)


# ---------------------------------------------------------------------------
# F3, an rclpy action client is one action endpoint with its type


def test_an_rclpy_action_client_is_one_action_endpoint(tmp_path: Path):
    atlas = python_workspace(
        tmp_path,
        {"client": node("client", "        self.c = ActionClient(self, Undock, 'dock')\n")},
    )
    names = {edge.dst for edge in atlas.edges.values() if edge.src == ids.build_node("/client")}
    assert ids.build_action("/dock") in names
    assert not any("_action" in name for name in names)


@pytest.mark.cpp
def test_an_action_client_of_another_type_than_its_server_is_an_error(tmp_path: Path):
    atlas = _components(
        tmp_path, "fleet_nodes::DockServer", "        self.c = ActionClient(self, Undock, 'dock')\n"
    )
    (found,) = findings(atlas, "type.incompatible")
    assert found.subject == ids.build_action("/dock")
    assert "fleet_msgs/action/Dock" in found.say


def test_the_services_a_library_makes_are_not_observed_only():
    from rosio.drift import topics
    from tests.test_drift_graph import Bench

    bench = Bench()
    mapper = ids.build_node("/mapper")
    bench.observed(mapper, "mapper", lifecycle_state="unconfigured")
    bench.builder.recompute()
    owner = bench.atlas.entities[mapper]
    assert topics.library_channel("/mapper/get_state", owner)
    assert topics.library_channel("/dock/_action/send_goal", None)
    assert not topics.library_channel("/mapper/get_state", None)
    assert not topics.library_channel("/mapper/save_map", owner)


# ---------------------------------------------------------------------------
# F4, a frame the code looks up by name where only static publishers broadcast

STATIC = (
    'Node(package="tf2_ros", executable="static_transform_publisher", name="{0}", '
    'arguments=["--frame-id", "{1}", "--child-frame-id", "{2}"])'
)
LOOKUP = (
    "        self.buffer = Buffer()\n"
    "        self.listener = TransformListener(self.buffer, self)\n"
    "        self.create_timer(1.0, self.tick)\n"
    "    def tick(self):\n"
    "        self.buffer.lookup_transform('map', '{0}', None)\n"
)


def test_a_frame_looked_up_by_name_that_no_static_publisher_names_has_no_publisher(
    tmp_path: Path,
):
    atlas = python_workspace(
        tmp_path,
        {"planner": node("planner", LOOKUP.format("dock_frame"))},
        STATIC.format("map_to_odom", "map", "odom"),
    )
    found = findings(atlas, "frame.no_publisher")
    assert [item.subject for item in found] == [ids.build_frame("dock_frame")]


def test_a_frame_a_static_publisher_names_is_published(tmp_path: Path):
    atlas = python_workspace(
        tmp_path,
        {"planner": node("planner", LOOKUP.format("odom"))},
        STATIC.format("map_to_odom", "map", "odom"),
    )
    assert findings(atlas, "frame.no_publisher") == []


# ---------------------------------------------------------------------------
# F5, the launch process calling change_state does not manage a node


def _lifecycle(state: str | None, caller_internal: bool) -> model.Atlas:
    from tests.test_drift_graph import OBSERVED, Bench

    bench = Bench()
    mapper = ids.build_node("/mapper")
    bench.declared(mapper, "mapper_node", form="lifecycle_node")
    bench.observed(mapper, "mapper", **({"lifecycle_state": state} if state else {}))
    caller = ids.build_node("/launch_ros_77")
    bench.observed(caller, "launch_ros_77", **({"internal": True} if caller_internal else {}))
    service = ids.build_service("/mapper/change_state")
    bench.builder.node(service)
    bench.builder.edge(
        model.EdgeKind.CALLS.value,
        caller,
        service,
        key=ids.edge_key(profile=bench.profile, raw_name="change_state"),
        layer=OBSERVED,
        profile=bench.profile,
        values={},
    )
    bench.builder.recompute()
    return bench.atlas


def test_a_node_the_probe_reads_unconfigured_is_unmanaged():
    assert len(findings(_lifecycle("unconfigured", False), "lifecycle.unmanaged")) == 1


def test_the_launch_process_calling_change_state_is_no_manager():
    assert len(findings(_lifecycle(None, True), "lifecycle.unmanaged")) == 1


def test_an_active_node_and_a_peer_caller_are_managed():
    assert findings(_lifecycle("active", False), "lifecycle.unmanaged") == []
    assert findings(_lifecycle(None, False), "lifecycle.unmanaged") == []


def test_the_unmanaged_title_names_the_profile():
    (found,) = findings(_lifecycle("unconfigured", False), "lifecycle.unmanaged")
    assert "nx_navigation" in found.title


# ---------------------------------------------------------------------------
# F6, a client of a name nothing serves


def _service(tmp_path: Path, called: str) -> model.Atlas:
    return python_workspace(
        tmp_path,
        {
            "dock": node("dock", "        self.create_service(SetMode, 'set_mode', self.cb)\n"),
            "planner": node("planner", f"        self.create_client(SetMode, '{called}')\n"),
        },
    )


def test_a_client_of_a_name_nothing_serves_names_the_served_one(tmp_path: Path):
    (found,) = findings(_service(tmp_path, "set_drive_mode"), "service.no_server")
    assert found.severity == "warn"
    assert found.subject == ids.build_service("/set_drive_mode")
    assert "/set_mode" in found.fix
    assert "planner.py:12" in found.say


def test_a_client_of_a_served_name_is_not_reported(tmp_path: Path):
    assert findings(_service(tmp_path, "set_mode"), "service.no_server") == []


def test_a_client_is_not_reported_when_a_server_name_is_blind(tmp_path: Path):
    atlas = python_workspace(
        tmp_path,
        {
            "dock": node(
                "dock",
                "        self.create_service(SetMode, self.get_name() + '/mode', self.cb)\n"
                "        self.create_service(SetMode, 'set_mode', self.cb)\n",
            ),
            "planner": node("planner", "        self.create_client(SetMode, 'set_drive_mode')\n"),
        },
    )
    assert findings(atlas, "service.no_server") == []


# ---------------------------------------------------------------------------
# F7, an include argument nothing declares, and one frame broadcast twice

INCLUDE = (
    "IncludeLaunchDescription(PythonLaunchDescriptionSource("
    "os.path.join(os.path.dirname(__file__), 'sensors.launch.py')), "
    "launch_arguments={{'{0}': 'robot1'}}.items())"
)
SENSORS = launch(
    'DeclareLaunchArgument("frame_prefix", default_value="")',
    'Node(package="tf2_ros", executable="static_transform_publisher", name="laser_tf", '
    'arguments=["--frame-id", "base_link", "--child-frame-id", '
    '[LaunchConfiguration("frame_prefix"), "laser"]])',
)
INCLUDE_IMPORTS = (
    "import os\n"
    "from launch.actions import IncludeLaunchDescription\n"
    "from launch.launch_description_sources import PythonLaunchDescriptionSource\n"
)


def _include(tmp_path: Path, argument: str) -> model.Atlas:
    package(
        tmp_path,
        "p",
        {
            "sensors.launch.py": SENSORS,
            "run.launch.py": launch(INCLUDE.format(argument), imports=INCLUDE_IMPORTS),
        },
    )
    return run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))


def test_an_include_argument_the_included_file_never_declares_names_the_close_one(
    tmp_path: Path,
):
    (found,) = findings(_include(tmp_path, "frame_prefx"), "launch.argument_undeclared")
    assert found.severity == "warn"
    assert "frame_prefx" in found.say
    assert "frame_prefix" in found.fix
    assert "run.launch.py" in found.say


def test_an_argument_the_include_already_passes_is_not_offered_as_the_meant_one(
    tmp_path: Path,
):
    package(
        tmp_path,
        "p",
        {
            "sensors.launch.py": SENSORS,
            "run.launch.py": launch(
                INCLUDE.replace(
                    "{{'{0}': 'robot1'}}", "{'frame_prefix': 'a', 'use_frame_prefix': 'b'}"
                ),
                imports=INCLUDE_IMPORTS,
            ),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    (found,) = findings(atlas, "launch.argument_undeclared")
    assert "use_frame_prefix" in found.say
    assert "drop use_frame_prefix" in found.fix


def test_an_include_argument_the_included_file_declares_is_not_reported(tmp_path: Path):
    assert findings(_include(tmp_path, "frame_prefix"), "launch.argument_undeclared") == []


def test_include_arguments_read_by_parameter_substitutions_or_later_actions(tmp_path: Path):
    from rosio.collect.launch_declared import _unread_arguments
    from rosio.resolve import records

    child = tmp_path / "child.launch.xml"
    child.write_text(
        '<launch><node pkg="p" exec="n"><param from="$(var config_file)" '
        'allow_substs="true"/></node></launch>'
    )
    parent = tmp_path / "run.launch.xml"
    parent.write_text(
        '<launch><include file="child.launch.xml"><arg name="later" '
        'value="robot1"/></include><let name="name" value="$(var later)"/></launch>'
    )
    params = tmp_path / "params.yaml"
    params.write_text("enable: $(var initial_pose/enable)\npose: $(arg initial_pose/pose)\n")
    include = records.LaunchInclude(
        str(child),
        args=(("initial_pose/enable", "true"), ("initial_pose/pose", "[]"), ("later", "robot1")),
        site=records.Site(str(parent)),
        entity_path=("Include(child.launch.xml)",),
    )
    walk = records.LaunchWalk(
        str(parent),
        includes=[include],
        nodes=[
            records.ResolvedNode(
                "p",
                "n",
                "/",
                entity_path=(*include.entity_path, "Node(n)"),
                file_reads=(records.FileRead(str(params), "parameters_file"),),
            )
        ],
    )
    assert _unread_arguments(walk, include) == []
    params.unlink()
    assert _unread_arguments(walk, include) == []


def test_include_arguments_read_by_python_helpers_are_not_reported(tmp_path: Path):
    from rosio.collect.launch_declared import _unread_arguments
    from rosio.resolve import records

    child = tmp_path / "child.launch.py"
    child.write_text("from helper import nodes\n")
    helper = tmp_path / "helper.py"
    helper.write_text(
        "from launch.substitutions import LaunchConfiguration\n"
        "nodes = LaunchConfiguration('robot_name')\n"
    )
    include = records.LaunchInclude(
        str(child),
        args=(("robot_name", "robot1"),),
        entity_path=("Include(child)",),
    )
    walk = records.LaunchWalk(str(child), includes=[include])
    assert _unread_arguments(walk, include) == []
    helper.write_text("nodes = []\n")
    assert _unread_arguments(walk, include) == [["robot_name", ""]]
    helper.unlink()
    assert _unread_arguments(walk, include) == []


def test_an_xml_include_argument_finding_is_about_the_launch_file():
    builder = model.AtlasBuilder()
    builder.node("launchfile:p|run.launch.xml")
    builder.fact(
        "launchent:p|entry.launch.xml|Include(sensors)",
        "declared",
        values={
            "included_file": "launchfile:p|sensors.launch.xml",
            "args_unread": [{"name": "frame_prefx", "closest": "frame_prefix"}],
        },
        evidence=[model.Source(file="file:p|run.launch.xml")],
    )
    (found,) = findings(builder.atlas, "launch.argument_undeclared")
    assert found.subject == "launchfile:p|run.launch.xml"


def test_two_nodes_broadcasting_one_child_frame_is_a_finding(tmp_path: Path):
    package(
        tmp_path,
        "p",
        {
            "run.launch.py": launch(
                STATIC.format("a", "odom", "base_link"), STATIC.format("b", "map", "base_link")
            )
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    (found,) = findings(atlas, "frame.broadcast_twice")
    assert found.subject == ids.build_frame("base_link")
    assert "two parents" in found.detail


def test_two_nodes_broadcasting_two_child_frames_is_not_a_finding(tmp_path: Path):
    package(
        tmp_path,
        "p",
        {
            "run.launch.py": launch(
                STATIC.format("a", "odom", "base_link"), STATIC.format("b", "base_link", "laser")
            )
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    assert findings(atlas, "frame.broadcast_twice") == []


# ---------------------------------------------------------------------------
# F8, F9, F10, parameter blocks and where a value comes from


def _params(tmp_path: Path, yaml_text: str, node_line: str) -> model.Atlas:
    package(
        tmp_path,
        "p",
        {
            "p/__init__.py": "",
            "p/planner.py": node(
                "planner",
                "        self.declare_parameter('timeout', 10.0)\n"
                "        self.declare_parameter('use_sim_time', False)\n",
            ),
            "setup.py": SETUP.format("'planner = p.planner:main'"),
            "params.yaml": yaml_text,
            "run.launch.py": launch(
                node_line,
                imports="import os\nHERE = os.path.dirname(__file__)\n",
            ),
        },
    )
    return run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))


PLANNER = (
    'Node(package="p", executable="planner", name="planner", namespace="{0}", '
    'parameters=[os.path.join(HERE, "params.yaml")])'
)


def test_a_bare_key_naming_a_namespaced_node_is_an_owner_name_mismatch(tmp_path: Path):
    yaml_text = "planner:\n  ros__parameters:\n    timeout: 5.0\n"
    atlas = _params(tmp_path, yaml_text, PLANNER.format("robot1"))
    (found,) = findings(atlas, "param.owner_name_mismatch")
    assert "/**/planner" in found.fix or "/robot1/planner" in found.fix
    assert findings(atlas, "param.orphan_block") == []


def test_a_bare_key_naming_a_root_node_is_no_mismatch(tmp_path: Path):
    yaml_text = "planner:\n  ros__parameters:\n    timeout: 5.0\n"
    atlas = _params(tmp_path, yaml_text, PLANNER.format(""))
    assert findings(atlas, "param.owner_name_mismatch") == []


def test_a_later_wildcard_block_shadowing_a_node_key_is_moved_not_deleted(tmp_path: Path):
    yaml_text = (
        "/robot1/planner:\n  ros__parameters:\n    timeout: 30.0\n"
        "/**:\n  ros__parameters:\n    timeout: 10.0\n"
    )
    atlas = _params(tmp_path, yaml_text, PLANNER.format("robot1"))
    (found,) = findings(atlas, "param.shadowed")
    assert "/**" in found.fix
    assert "params.yaml:6" in found.fix or "params.yaml:6" in found.detail


SIM_TIME = """import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
ROBOTS = {
    'robot1': {'use_sim_time': 'true'},
    'robot2': {'use_sim_time': 'false'},
}
def make(context):
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(os.path.dirname(__file__), 'robot.launch.py')),
            launch_arguments={'namespace': name, 'use_sim_time': cfg['use_sim_time']}.items())
        for name, cfg in ROBOTS.items()
    ]
def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=make)])
"""

ROBOT = launch(
    'DeclareLaunchArgument("namespace")',
    'DeclareLaunchArgument("use_sim_time")',
    'Node(package="p", executable="planner", name="planner", '
    'namespace=LaunchConfiguration("namespace"), '
    'parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}])',
)


def test_sim_time_mixed_cites_the_line_that_holds_the_value(tmp_path: Path):
    package(
        tmp_path,
        "p",
        {
            "p/__init__.py": "",
            "p/planner.py": node(
                "planner", "        self.declare_parameter('use_sim_time', False)\n"
            ),
            "setup.py": SETUP.format("'planner = p.planner:main'"),
            "run.launch.py": SIM_TIME,
            "robot.launch.py": ROBOT,
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    (found,) = findings(atlas, "param.sim_time_mixed")
    assert "run.launch.py:6" in found.say or "run.launch.py:7" in found.say, found.say


# ---------------------------------------------------------------------------
# F11, a launch static publisher publishes /tf_static


def test_a_launch_static_publisher_publishes_tf_static(tmp_path: Path):
    package(tmp_path, "p", {"run.launch.py": launch(STATIC.format("a", "odom", "base_link"))})
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    tf_static = ids.build_topic("/tf_static")
    assert any(
        edge.kind == model.EdgeKind.PUBLISHES.value and edge.dst == tf_static
        for edge in atlas.edges.values()
    )
    assert findings(atlas, "topic.no_publisher") == []


# ---------------------------------------------------------------------------
# F12, remaps a member initializer or a built name could take

EVALUATOR = """#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <nav_msgs/msg/odometry.hpp>
namespace demo {
class Evaluator : public rclcpp::Node {
public:
  explicit Evaluator(const rclcpp::NodeOptions & o)
  : Node("evaluator", o), odom_sub_(this, "~/input/localization", rmw_qos_profile_default) {}
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
};
}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(demo::Evaluator)
"""


@pytest.mark.cpp
def test_a_message_filters_member_initializer_takes_its_remap(tmp_path: Path):
    from tests.test_sixth_trial import launch as component_launch

    package(
        tmp_path,
        "p",
        {
            "CMakeLists.txt": "project(p)\nadd_library(ev SHARED src/ev.cpp)\n"
            'rclcpp_components_register_nodes(ev "demo::Evaluator")\n',
            "src/ev.cpp": EVALUATOR,
            "run.launch.py": component_launch(
                'ComposableNodeContainer(name="box", namespace="", package="rclcpp_components",'
                ' executable="component_container", composable_node_descriptions=['
                'ComposableNode(package="p", plugin="demo::Evaluator", name="evaluator", '
                'remappings=[("~/input/localization", "/odom")])])'
            ),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    assert findings(atlas, "remap.matches_nothing") == []
    assert any(
        edge.kind == model.EdgeKind.SUBSCRIBES.value and edge.dst == ids.build_topic("/odom")
        for edge in atlas.edges.values()
    )


INPUT_HPP = """#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
namespace demo {
class Input {
public:
  Input(const std::string & name, rclcpp::Node & node);
private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};
}
"""
INPUT_CPP = """#include "input.hpp"
namespace demo {
Input::Input(const std::string & name, rclcpp::Node & node) {
  sub_ = node.create_subscription<std_msgs::msg::String>(
    "~/inputs/" + name + "/trajectory", 1, [](std_msgs::msg::String::SharedPtr) {});
}
}
"""
GATE = """#include "input.hpp"
namespace demo {
class Gate : public rclcpp::Node {
public:
  explicit Gate(const rclcpp::NodeOptions & o) : Node("gate", o) {
    pub_ = create_publisher<std_msgs::msg::String>("~/output", 1);
    for (const auto & name : declare_parameter<std::vector<std::string>>("inputs")) {
      inputs_.push_back(std::make_unique<Input>(name, *this));
    }
  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  std::vector<std::unique_ptr<Input>> inputs_;
};
}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(demo::Gate)
"""


@pytest.mark.cpp
def test_a_remap_a_helper_with_a_built_name_could_take_is_not_dead(tmp_path: Path):
    from tests.test_sixth_trial import launch as component_launch

    package(
        tmp_path,
        "p",
        {
            "CMakeLists.txt": "project(p)\nadd_library(gate SHARED src/gate.cpp src/input.cpp)\n"
            "target_include_directories(gate PRIVATE src)\n"
            'rclcpp_components_register_nodes(gate "demo::Gate")\n',
            "src/gate.cpp": GATE,
            "src/input.hpp": INPUT_HPP,
            "src/input.cpp": INPUT_CPP,
            "run.launch.py": component_launch(
                'ComposableNodeContainer(name="box", namespace="", package="rclcpp_components",'
                ' executable="component_container", composable_node_descriptions=['
                'ComposableNode(package="p", plugin="demo::Gate", name="gate", '
                'remappings=[("~/inputs/planner/trajectory", "/traj")])])'
            ),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    assert findings(atlas, "remap.matches_nothing") == []


# ---------------------------------------------------------------------------
# F13, one qos mismatch the source and the probe both state is one finding


def test_a_qos_mismatch_two_layers_state_is_one_finding_from_the_source():
    from rosio.drift import topics
    from tests.test_drift_graph import CODED, OBSERVED, Bench, qos

    bench = Bench()
    sub = ids.build_node("/sub")
    bench.observed(sub, "sub")
    for layer in (CODED, OBSERVED):
        bench.endpoint(
            model.EdgeKind.PUBLISHES.value,
            ids.build_node("/io_bridge"),
            "/scan",
            layer,
            key="scan_out",
            values={"qos": qos(reliability="best_effort", depth=5)},
        )
        bench.endpoint(
            model.EdgeKind.SUBSCRIBES.value,
            sub,
            "/scan",
            layer,
            key="scan_in",
            values={"qos": qos(reliability="reliable", depth=5)},
        )
    bench.builder.recompute()
    found = [
        item
        for item in topics.qos_incompatible(bench.atlas)
        if item.subject == ids.build_topic("/scan")
    ]
    assert len(found) == 1
    assert found[0].layers == [CODED, OBSERVED]


# ---------------------------------------------------------------------------
# Part 2, wording


def test_the_extract_summary_counts_profiles_past_a_handful(capsys: pytest.CaptureFixture[str]):
    from rosio import cli

    report = {
        "exceptions": [],
        "profiles": [f"p{index}" for index in range(30)],
        "layers": [],
        "entities": 0,
        "entities_by_kind": {},
        "edges": 0,
        "edges_by_kind": {},
        "unresolved": 0,
        "unresolved_by_want": {},
        "resolution_rate": 1.0,
        "resolved": 0,
        "findings": {},
        "findings_by_rule": {},
        "rules_failed": [],
    }
    cli._print_summary(report, 0.0)
    line = next(item for item in capsys.readouterr().out.splitlines() if "profiles 30" in item)
    assert "p29" not in line
    assert "22 more" in line


def test_drift_md_names_both_severities_of_a_rule_with_a_variant():
    from rosio.render import drift_md

    assert drift_md._severity(model.Atlas(), "frame.not_in_urdf") == "warn or info"
    assert drift_md._severity(model.Atlas(), "plugin.unused") == "info"


def test_hook_install_says_to_commit_the_attribute_line(tmp_path: Path):
    import subprocess

    from rosio import history

    root = tmp_path / "repo"
    subprocess.run(["git", "init", "-q", str(root)], check=True)
    report = history.install(root, measure=False)
    assert any("commit .gitattributes" in line for line in report.lines)


MULTI_LINE = """from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(
            package="p",
            executable="ctrl",
            name="ctrl",
            remappings=[("cmd_vel_out", "cmd_vel")],
        ),
    ])
"""


def test_a_dead_remap_cites_the_line_that_writes_it(tmp_path: Path):
    package(
        tmp_path,
        "p",
        {
            "p/__init__.py": "",
            "p/ctrl.py": node("ctrl", "        self.create_publisher(Twist, 'cmd_vel_raw', 10)\n"),
            "setup.py": SETUP.format("'ctrl = p.ctrl:main'"),
            "run.launch.py": MULTI_LINE,
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    (found,) = findings(atlas, "remap.matches_nothing")
    assert "run.launch.py:9" in found.say


def test_rosio_param_shows_the_value_the_running_system_holds(tmp_path: Path):
    from rosio import query

    yaml_text = "planner:\n  ros__parameters:\n    timeout: 5.0\n"
    atlas = _params(tmp_path, yaml_text, PLANNER.format(""))
    param_id = ids.build_param(ids.build_node("/planner"), "timeout")
    profile = ids.build_profile("p")
    atlas.entities[param_id].facts.append(
        model.LayerFact(layer="observed", profile=profile, values={"value": 7.5})
    )
    answer = query.param(atlas, "/planner", "timeout")
    assert any("observed 7.5" in line for line in answer.lines)
    assert answer.data["profiles"][profile]["observed"] == 7.5


def test_a_launch_configuration_nothing_declares_is_named_in_the_fix(tmp_path: Path):
    package(
        tmp_path,
        "p",
        {
            "run.launch.py": launch(
                'Node(package="ext", executable="aeb", name="aeb", '
                'remappings=[("in", LaunchConfiguration("input_odometry"))])'
            )
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    (found,) = findings(atlas, "launch.raised")
    assert "declare input_odometry with a default" in found.fix
    assert "input_odometry" in found.say


FLEET = """import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
def robots(context):
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(os.path.dirname(__file__), 'robot.launch.py')))]
def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=robots)])
"""


def test_init_comments_out_a_launch_file_another_one_starts(tmp_path: Path):
    import yaml

    from rosio import scaffold
    from rosio.config import default_config

    package(
        tmp_path,
        "fleet",
        {
            "launch/fleet.launch.py": FLEET,
            "launch/robot.launch.py": launch('Node(package="p", executable="x", name="x")'),
        },
    )
    text = scaffold.render(default_config(str(tmp_path)), tmp_path)
    slugs = [item["slug"] for item in yaml.safe_load(text)["profiles"]]
    assert len(slugs) == 1 and "robot" not in slugs[0]
    assert "# fleet/fleet.launch.py includes and starts this one" in text
