"""Defects the seventh first run trial found, each with the source pattern that showed it."""

from __future__ import annotations

import dataclasses
import subprocess
from pathlib import Path

import pytest

from rosio import extract, history, ids, model, query, schema
from rosio.collect import params as params_collector
from rosio.collect import probe, proc
from rosio.config import Config, Entrypoint, ProfileSpec, RootSpec
from rosio.drift import engine
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


def launch(*actions: str, imports: str = "") -> str:
    return (
        "from launch import LaunchDescription\n"
        "from launch.actions import DeclareLaunchArgument, GroupAction\n"
        "from launch.substitutions import LaunchConfiguration\n"
        "from launch_ros.actions import Node, LifecycleNode, SetParameter\n"
        f"{imports}"
        "def generate_launch_description():\n"
        f"    return LaunchDescription([{', '.join(actions)}])\n"
    )


def findings(atlas: model.Atlas, rule: str) -> list[model.Finding]:
    return [item for item in engine.run(atlas, only=[rule]).findings if item.rule == rule]


SETUP = "from setuptools import setup\nsetup(name='p', entry_points={{'console_scripts': [{0}]}})\n"


def python_node(name: str, body: str) -> str:
    return (
        "from rclpy.node import Node\n"
        "from rclpy.qos import QoSProfile, qos_profile_sensor_data\n"
        f"class N(Node):\n"
        "    def __init__(self):\n"
        f"        super().__init__('{name}')\n"
        f"{body}"
        "def main():\n"
        "    pass\n"
    )


# ---------------------------------------------------------------------------
# N1, a remap onto a name the node never creates


def _dead_remap(tmp_path: Path) -> model.Atlas:
    package(
        tmp_path,
        "p",
        {
            "p/__init__.py": "",
            "p/ctrl.py": python_node(
                "ctrl", "        self.create_publisher(Twist, 'cmd_vel_raw', 10)\n"
            ),
            "setup.py": SETUP.format("'ctrl = p.ctrl:main'"),
            "run.launch.py": launch(
                'Node(package="p", executable="ctrl", name="ctrl", '
                'remappings=[("cmd_vel_out", "cmd_vel")])'
            ),
        },
    )
    return run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))


def test_a_remap_whose_source_the_node_never_names_is_a_dead_remap(tmp_path: Path):
    atlas = _dead_remap(tmp_path)
    dead = findings(atlas, "remap.matches_nothing")
    assert len(dead) == 1
    assert dead[0].severity == "warn"
    assert "cmd_vel_raw" in dead[0].detail
    assert "run.launch.py" in dead[0].say
    assert findings(atlas, "remap.unresolved") == []


def test_a_remap_the_node_matches_is_not_dead(tmp_path: Path):
    package(
        tmp_path,
        "p",
        {
            "p/__init__.py": "",
            "p/ctrl.py": python_node(
                "ctrl", "        self.create_publisher(Twist, 'cmd_vel_out', 10)\n"
            ),
            "setup.py": SETUP.format("'ctrl = p.ctrl:main'"),
            "run.launch.py": launch(
                'Node(package="p", executable="ctrl", name="ctrl", '
                'remappings=[("cmd_vel_out", "cmd_vel")])'
            ),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    assert findings(atlas, "remap.matches_nothing") == []


def test_an_observed_topic_with_no_publisher_names_the_remap_that_moved_it():
    from rosio.drift import topics
    from tests.test_drift_graph import OBSERVED, Bench

    bench = Bench()
    node = ids.build_node("/ctrl")
    bench.observed(node, "ctrl", remaps=[["cmd_vel_out", "/cmd_vel_in"]])
    bench.endpoint(model.EdgeKind.PUBLISHES.value, node, "/cmd_vel_raw", OBSERVED, key="raw")
    bench.endpoint(model.EdgeKind.SUBSCRIBES.value, node, "/cmd_vel_raw", OBSERVED, key="rawin")
    bench.endpoint(
        model.EdgeKind.SUBSCRIBES.value,
        ids.build_node("/left_over"),
        "/cmd_vel_in",
        OBSERVED,
        key="in",
    )
    bench.builder.recompute()
    found = [
        item
        for item in topics.topic_no_publisher(bench.atlas)
        if item.subject == ids.build_topic("/cmd_vel_in")
    ]
    assert len(found) == 1
    assert "cmd_vel_out" in found[0].detail


# ---------------------------------------------------------------------------
# N2, QoSProfile(depth=10) is reliable and volatile


def test_a_depth_only_qos_profile_requests_reliable_and_meets_a_sensor_data_publisher(
    tmp_path: Path,
):
    package(
        tmp_path,
        "p",
        {
            "p/__init__.py": "",
            "p/pub.py": python_node(
                "pub",
                "        self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)\n",
            ),
            "p/sub.py": python_node(
                "sub",
                "        self.create_subscription(LaserScan, '/scan', None, "
                "QoSProfile(depth=10))\n",
            ),
            "setup.py": SETUP.format("'pub = p.pub:main', 'sub = p.sub:main'"),
            "run.launch.py": launch(
                'Node(package="p", executable="pub", name="pub")',
                'Node(package="p", executable="sub", name="sub")',
            ),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    found = findings(atlas, "qos.incompatible")
    assert [item.subject for item in found] == [ids.build_topic("/scan")]


# ---------------------------------------------------------------------------
# N3, one node stating one endpoint from two sites


def _one_endpoint(coded_sites: tuple[str, ...], coded_depth: int, observed_depth: int):
    from tests.test_drift_graph import CODED, OBSERVED, Bench, qos

    bench = Bench()
    node = ids.build_node("/io_bridge")
    publishes = model.EdgeKind.PUBLISHES.value
    for key in coded_sites:
        bench.endpoint(
            publishes,
            node,
            "/twice",
            CODED,
            key=key,
            values={"qos": qos(history="keep_last", depth=coded_depth)},
        )
    bench.endpoint(
        publishes,
        node,
        "/twice",
        OBSERVED,
        key="seen",
        values={"qos": qos(history="keep_last", depth=observed_depth)},
    )
    bench.endpoint(
        model.EdgeKind.SUBSCRIBES.value,
        ids.build_node("/left_over"),
        "/twice",
        OBSERVED,
        key="twice_in",
    )
    bench.builder.recompute()
    return bench.atlas


def test_a_source_site_and_the_observed_edge_of_one_endpoint_are_compared_once():
    from rosio.drift import topics

    twice = ids.build_topic("/twice")
    found = [
        item
        for item in topics.qos_differ(_one_endpoint(("site_a",), 10, 5))
        if twice in item.subject_key
    ]
    assert len(found) == 1
    assert "depth" in found[0].detail


def test_two_sites_of_one_endpoint_leave_the_observed_one_unreported():
    from rosio.drift import topics

    twice = ids.build_topic("/twice")
    atlas = _one_endpoint(("site_a", "site_b"), 10, 10)
    assert [item for item in topics.qos_differ(atlas) if twice in item.subject_key] == []
    assert [
        item for item in topics.endpoint_observed_only(atlas) if twice in item.subject_key
    ] == []


# ---------------------------------------------------------------------------
# N5, selector lists and blocks named by a string


def test_filters_a_motion_model_and_a_nested_plugin_are_selected():
    winners = {
        "global_costmap.filters": ["keepout_filter"],
        "global_costmap.keepout_filter.plugin": "nav2_costmap_2d::KeepoutFilter",
        "FollowPath.plugin": "nav2_mppi_controller::MPPIController",
        "FollowPath.motion_model": "diff_drive",
        "FollowPath.diff_drive.plugin": "x::DiffDrive",
        "FollowPath.Validator.plugin": "x::Validator",
        "controller_plugins": ["FollowPath"],
    }
    selected = params_collector._selected_instances(winners)
    assert "global_costmap.keepout_filter" in selected
    assert "FollowPath.diff_drive" in selected
    assert "FollowPath.Validator" in selected


# ---------------------------------------------------------------------------
# M2, a lifecycle node that starts itself


def test_an_autostarted_lifecycle_node_is_managed(tmp_path: Path):
    pytest.importorskip("launch_ros")
    from launch_ros.actions import LifecycleNode

    if "autostart" not in LifecycleNode.__init__.__code__.co_varnames:
        pytest.skip("this launch_ros has no autostart")
    package(
        tmp_path,
        "p",
        {
            "run.launch.py": launch(
                'LifecycleNode(package="ext", executable="sim", name="sim", namespace="", '
                "autostart=True)"
            ),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    managed = [
        edge for edge in atlas.edges.values() if edge.kind == model.EdgeKind.MANAGES_LIFECYCLE.value
    ]
    assert [edge.facts[0].values["mechanism"] for edge in managed] == ["autostart"]
    assert findings(atlas, "lifecycle.unmanaged") == []


# ---------------------------------------------------------------------------
# M3, one SetParameter line handed to a group


def test_a_set_parameter_line_is_one_finding_that_never_asks_to_edit_a_dictionary(
    tmp_path: Path,
):
    package(
        tmp_path,
        "p",
        {
            "p/__init__.py": "",
            "p/a.py": python_node("a", "        self.declare_parameter('rate', 1.0)\n"),
            "p/b.py": python_node("b", "        self.declare_parameter('rate', 1.0)\n"),
            "setup.py": SETUP.format("'a = p.a:main', 'b = p.b:main'"),
            "run.launch.py": launch(
                "GroupAction([*[SetParameter(name=k, value=v) for k, v in "
                "{'wheel_width': 0.5, 'wheel_base': 1.0}.items()], "
                'Node(package="p", executable="a", name="a"), '
                'Node(package="p", executable="b", name="b")])'
            ),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    found = findings(atlas, "param.set_undeclared")
    assert len(found) == 1
    assert "dictionary" not in found[0].fix
    assert "SetParameter" in found[0].fix
    assert "/a" in found[0].detail and "/b" in found[0].detail
    mechanisms = {
        row.mechanism
        for entity in atlas.by_kind("param")
        for row in entity.candidates
        if entity.label.endswith("wheel_width")
    }
    assert mechanisms == {model.Mechanism.LAUNCH_SET_PARAMETER.value}


# ---------------------------------------------------------------------------
# M4 and M6, endpoints behind nav2 and Autoware wrappers


WRAPPED = """#include <rclcpp/rclcpp.hpp>
#include "nav2_util/twist_subscriber.hpp"
#include "autoware_utils/ros/polling_subscriber.hpp"
class Smoother : public rclcpp::Node {
public:
  Smoother() : Node("smoother") {
    twist_ = std::make_unique<nav2_util::TwistSubscriber>(this, "cmd_vel", 1, nullptr, nullptr);
  }
  std::unique_ptr<nav2_util::TwistSubscriber> twist_;
  autoware_utils::InterProcessPollingSubscriber<Odometry> odom_sub_{this, "~/input/odometry"};
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Smoother>());
}
"""


@pytest.mark.cpp
def test_a_twist_subscriber_and_a_polling_subscriber_subscribe(tmp_path: Path):
    package(
        tmp_path,
        "p",
        {
            "CMakeLists.txt": "project(p)\nadd_executable(smoother src/smoother.cpp)\n",
            "src/smoother.cpp": WRAPPED,
            "run.launch.py": launch('Node(package="p", executable="smoother", name="smoother")'),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    edges = {(edge.kind, edge.dst) for edge in atlas.out_edges("node:/smoother")}
    assert ("subscribes", "topic:/cmd_vel") in edges
    assert ("subscribes", "topic:/smoother/input/odometry") in edges


CALIB_HPP = """#include <rclcpp/rclcpp.hpp>
#include "autoware_utils/ros/polling_subscriber.hpp"
namespace calib {
class Calib : public rclcpp::Node {
public:
  explicit Calib(const rclcpp::NodeOptions & o);
private:
  autoware_utils::InterProcessPollingSubscriber<Odometry> odom_sub_{this, "~/input/odometry"};
};
}
"""

CALIB_CPP = """#include "p/calib.hpp"
namespace calib {
Calib::Calib(const rclcpp::NodeOptions & o) : Node("calib", o) {}
}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(calib::Calib)
"""


@pytest.mark.cpp
def test_a_polling_subscriber_a_header_builds_belongs_to_the_constructor(tmp_path: Path):
    package(
        tmp_path,
        "p",
        {
            "CMakeLists.txt": "project(p)\ninclude_directories(include)\n"
            "add_library(calib SHARED src/calib.cpp)\n"
            'rclcpp_components_register_node(calib PLUGIN "calib::Calib" EXECUTABLE calib_node)\n',
            "include/p/calib.hpp": CALIB_HPP,
            "src/calib.cpp": CALIB_CPP,
            "run.launch.py": launch('Node(package="p", executable="calib_node", name="calib")'),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    edges = {(edge.kind, edge.dst) for edge in atlas.out_edges("node:/calib")}
    assert ("subscribes", "topic:/calib/input/odometry") in edges


def test_the_remaps_one_file_writes_on_one_node_are_one_finding(tmp_path: Path):
    package(
        tmp_path,
        "p",
        {
            "run.launch.py": launch(
                'Node(package="ext", executable="calib", name="calib", '
                'remappings=[("~/input/a", "/a"), ("~/input/b", "/b")])'
            ),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    found = findings(atlas, "remap.unresolved")
    assert len(found) == 1
    assert "2 remaps" in found[0].title


# ---------------------------------------------------------------------------
# M5, a missing parameters file a launch argument names


def test_a_missing_default_cites_the_argument_and_names_the_file_one_slip_away(tmp_path: Path):
    package(
        tmp_path,
        "p",
        {
            "config/lidar.yaml": "lidar:\n  ros__parameters:\n    rate: 10\n",
            "run.launch.py": launch(
                "DeclareLaunchArgument('params', default_value="
                "os.path.join(os.path.dirname(__file__), 'config', 'lidar_params.yaml'))",
                'Node(package="ext", executable="lidar", name="lidar", '
                "parameters=[LaunchConfiguration('params')])",
                imports="import os\n",
            ),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    found = findings(atlas, "extract.input_refused")
    assert len(found) == 1
    assert "DeclareLaunchArgument" in " ".join(row.locator for row in found[0].where)
    assert "lidar.yaml" in found[0].fix


# ---------------------------------------------------------------------------
# M7, what the probe states about processes


def test_a_python_node_is_named_by_its_script():
    row = proc.parse(
        [
            "/usr/bin/python3",
            "/ws/install/p/lib/p/obstacle_controller",
            "--ros-args",
            "-r",
            "__node:=obstacle_controller",
        ]
    )
    assert row is not None
    assert row.executable == "obstacle_controller"


def test_a_zenoh_router_selects_no_middleware_pass():
    router = proc.parse(
        ["/usr/bin/python3", "/opt/ros/jazzy/bin/ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"]
    )
    assert router is not None and router.is_router
    router = dataclasses.replace(router, rmw="rmw_zenoh_cpp")
    assert probe.plan_passes([router]) == ["default"]


def test_the_probe_summary_counts_the_frames_a_transform_names():
    from rosio.cli import _observed_counts

    builder = model.AtlasBuilder()
    builder.declare_layer("observed", present=True)
    profile = builder.add_profile("p")
    for frame in ("odom", "base_link"):
        builder.node(ids.build_frame(frame))
    builder.edge(
        model.EdgeKind.TF_PARENT.value,
        ids.build_frame("base_link"),
        ids.build_frame("odom"),
        key=ids.edge_key(profile=profile, pair="odom>base_link"),
        layer="observed",
        profile=profile,
    )
    assert _observed_counts(builder.atlas)["frames"] == 2


# ---------------------------------------------------------------------------
# H5, a static transform publisher's arguments are a broadcast


def test_a_static_transform_publisher_broadcasts_the_frames_its_arguments_name(tmp_path: Path):
    package(
        tmp_path,
        "p",
        {
            "run.launch.py": launch(
                'Node(package="tf2_ros", executable="static_transform_publisher", '
                'arguments=["--frame-id", "base_link", "--child-frame-id", "laser"])'
            ),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    assert "frame:laser" in atlas.entities
    broadcasts = [
        edge
        for edge in atlas.edges.values()
        if edge.kind == model.EdgeKind.BROADCASTS_TF.value and edge.dst == "frame:laser"
    ]
    assert len(broadcasts) == 1
    assert broadcasts[0].facts[0].values["parent"] == "base_link"


# ---------------------------------------------------------------------------
# Part 2 wording


def test_an_explain_miss_names_a_handful_of_profiles_and_the_count():
    builder = model.AtlasBuilder()
    for index in range(9):
        builder.add_profile(f"p{index}")
    atlas = builder.atlas
    from unittest import mock

    with mock.patch.object(
        query, "raised_profiles", return_value={f"profile:p{i}" for i in range(9)}
    ):
        text = " ".join(query.not_found_lines(atlas, "/nothing"))
    assert "9 profiles" in text and "4 more" in text


def test_init_names_ros2_run_lines_when_the_workspace_has_no_launch_file(tmp_path: Path, capsys):
    from rosio.cli import main

    package(
        tmp_path,
        "vision",
        {
            "CMakeLists.txt": "project(vision)\nadd_executable(cam src/cam.cpp)\n"
            "install(TARGETS cam DESTINATION lib/${PROJECT_NAME})\n",
            "src/cam.cpp": "int main() {}\n",
        },
    )
    main(["init", str(tmp_path), "--config", str(tmp_path / "rosio.yaml")])
    out = capsys.readouterr().out
    assert "ros2 run vision cam" in out


def test_the_report_says_why_a_section_has_no_table():
    from rosio.render import report

    builder = model.AtlasBuilder()
    builder.declare_layer("declared", present=True)
    text = report.render_report(builder.atlas)
    assert "This atlas has no profile" in text
    assert "No pluginlib manifest exports a plugin" in text


def test_drift_md_does_not_say_static_rules_need_a_probe():
    from rosio.render import drift_md

    assert "lifecycle.unmanaged" not in drift_md.NEEDS_LAYER
    assert "node.ambiguous_identity" not in drift_md.NEEDS_LAYER


# ---------------------------------------------------------------------------
# M1, the hooks and a tracked atlas


def git(root: Path, *args: str) -> str:
    done = subprocess.run(
        ["git", "-C", str(root), *args], capture_output=True, text=True, check=True
    )
    return done.stdout.strip()


def test_a_hook_never_rewrites_a_tracked_atlas(tmp_path: Path):
    root = tmp_path / "repo"
    package(root, "p", {"config/x.yaml": "n:\n  ros__parameters:\n    a: 1\n"})
    git(tmp_path, "init", "-q", str(root))
    git(root, "config", "user.email", "dev@example.com")
    git(root, "config", "user.name", "developer")
    history.install(root, measure=False)
    history.run_rebuild(root, event="manual")
    atlas = root / "rosio-out" / "atlas.json"
    git(root, "add", "-A")
    git(root, "commit", "-qm", "with atlas")
    (root / "src" / "p" / "config" / "x.yaml").write_text("n:\n  ros__parameters:\n    a: 2\n")
    git(root, "commit", "-qam", "change")
    before = atlas.read_bytes()
    record = history.run_rebuild(root, event="post-commit")
    assert record["left_tracked"] is True
    assert atlas.read_bytes() == before
    assert git(root, "status", "--porcelain") == ""


def test_a_rebuild_says_it_dropped_the_observed_layer(tmp_path: Path):
    root = tmp_path / "repo"
    package(root, "p", {"config/x.yaml": "n:\n  ros__parameters:\n    a: 1\n"})
    git(tmp_path, "init", "-q", str(root))
    history.install(root, measure=False)
    history.run_rebuild(root, event="manual")
    atlas = root / "rosio-out" / "atlas.json"
    loaded = model.Atlas.loads(atlas.read_bytes())
    loaded.layers["observed"] = model.LayerStatus(present=True)
    atlas.write_bytes(loaded.to_bytes())
    record = history.run_rebuild(root, event="post-checkout", reuse=False)
    assert record["dropped_layers"] == ["observed"]


# ---------------------------------------------------------------------------
# H3 and H6, saying why rather than stating an absence


def test_a_parameter_a_built_name_could_declare_says_which_expression(tmp_path: Path):
    package(
        tmp_path,
        "p",
        {
            "p/__init__.py": "",
            "p/mon.py": python_node(
                "mon",
                "        self.declare_parameter(self.polygon + '.action_type', 'stop')\n",
            ),
            "setup.py": SETUP.format("'mon = p.mon:main'"),
            "config/mon.yaml": "mon:\n  ros__parameters:\n    Footprint:\n"
            "      action_type: approach\n",
            "run.launch.py": launch(
                'Node(package="p", executable="mon", name="mon", '
                "parameters=[os.path.join(os.path.dirname(__file__), 'config', 'mon.yaml')])",
                imports="import os\n",
            ),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    text = query.param(atlas, "/mon", "Footprint.action_type").text()
    assert "self.polygon" in text


def test_the_graph_says_a_missing_end_was_not_found_when_names_are_blind(tmp_path: Path):
    from rosio.serve import tools

    package(
        tmp_path,
        "p",
        {
            "p/__init__.py": "",
            "p/talk.py": python_node(
                "talk",
                "        self.create_publisher(String, self.topic(), 10)\n"
                "        self.create_subscription(String, 'cmd_vel_in', None, 10)\n",
            ),
            "setup.py": SETUP.format("'talk = p.talk:main'"),
            "run.launch.py": launch('Node(package="p", executable="talk", name="talk")'),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    text = tools.graph(atlas)
    assert "no publisher found, 1 node has unresolved topic names" in text


def test_a_remap_a_helper_somewhere_could_match_is_not_dead(tmp_path: Path):
    """An Autoware controller node builds its longitudinal controller from another package,
    so a name any source spells is not proven absent from the node."""
    package(
        tmp_path,
        "p",
        {
            "p/__init__.py": "",
            "p/ctrl.py": python_node(
                "ctrl", "        self.create_publisher(Twist, 'cmd_vel_raw', 10)\n"
            ),
            "p/helper.py": python_node(
                "helper", "        self.create_publisher(Twist, 'cmd_vel_out', 10)\n"
            ),
            "setup.py": SETUP.format("'ctrl = p.ctrl:main'"),
            "run.launch.py": launch(
                'Node(package="p", executable="ctrl", name="ctrl", '
                'remappings=[("cmd_vel_out", "cmd_vel")])'
            ),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    assert findings(atlas, "remap.matches_nothing") == []


# ---------------------------------------------------------------------------
# review 13: a dead remap is said only of a node whose endpoint set is known


def _node_source(name: str, body: str, imports: str = "", base: str = "Node") -> str:
    return (
        "from rclpy.node import Node\n"
        "from rclpy.lifecycle import LifecycleNode\n"
        "from std_msgs.msg import String\n"
        f"{imports}"
        f"class N({base}):\n"
        "    def __init__(self):\n"
        f"        super().__init__('{name}')\n"
        "        self.create_publisher(String, 'status', 10)\n"
        f"{body}"
        "def main():\n"
        "    pass\n"
    )


def _python_dead(tmp_path: Path, body: str, remap: str, imports="", base="Node", extra=None):
    files = {
        "p/__init__.py": "",
        "p/ctrl.py": _node_source("ctrl", body, imports, base),
        "setup.py": SETUP.format("'ctrl = p.ctrl:main'"),
        "run.launch.py": launch(
            f'Node(package="p", executable="ctrl", name="ctrl", remappings=[{remap}])'
        ),
        **(extra or {}),
    }
    package(tmp_path, "p", files)
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    return atlas, findings(atlas, "remap.matches_nothing")


HELPER_DYNAMIC = (
    "def make_subs(n):\n    n.create_subscription(String, n.prefix + 'scan', None, 10)\n"
)


@pytest.mark.parametrize(
    ("body", "remap", "imports", "base", "extra"),
    [
        ("", '("ctrl:status", "/x")', "", "Node", None),
        (
            "        self.create_lifecycle_publisher(String, 'out', 10)\n",
            '("out", "/x")',
            "",
            "LifecycleNode",
            None,
        ),
        (
            "        self.srv = InteractiveMarkerServer(self, 'markers')\n",
            '("markers/feedback", "/x")',
            "from interactive_markers import InteractiveMarkerServer\n",
            "Node",
            None,
        ),
        (
            "        self.h = SomeLib(self)\n",
            '("odom", "/robot/odom")',
            "from some_ros_lib import SomeLib\n",
            "Node",
            None,
        ),
        (
            "        Helper(self)\n",
            '("scan", "/lidar")',
            "from p.helper import Helper\n",
            "Node",
            {
                "p/helper.py": "class Helper:\n    def __init__(self, n):\n        print 'x'\n"
                "        n.create_subscription(String, 'scan', None, 10)\n"
            },
        ),
        (
            "        make_subs(self)\n",
            '("scan", "/lidar")',
            "from p.util import make_subs\n",
            "Node",
            {"p/util.py": HELPER_DYNAMIC},
        ),
        ("", '("~/change_state", "/x")', "", "LifecycleNode", None),
        ("", '("~/get_type_description", "/x")', "", "Node", None),
        (
            "        getattr(self, 'create_subscription')(String, 'scan', None, 10)\n",
            '("scan", "/lidar")',
            "",
            "Node",
            None,
        ),
        (
            "        extra.setup(self.handle)\n",
            '("scan", "/lidar")',
            "from p import extra\n",
            "Node",
            {"p/extra.py": "def setup(n):\n    print 'refused'\n"},
        ),
        (
            "",
            '("scan", "/lidar")',
            "from p.base import Base\n",
            "Base",
            {
                "p/base.py": "from rclpy.node import Node\nclass Base(Node):\n"
                "    def __init__(self, n):\n        super().__init__(n)\n"
                "        self.create_subscription(String, self.prefix + 'scan', None, 10)\n"
            },
        ),
    ],
    ids=[
        "node_scoped",
        "lifecycle_publisher",
        "interactive_markers",
        "unread_library",
        "refused_helper",
        "helper_builds_name",
        "lifecycle_service",
        "type_description",
        "literal_backstop",
        "refused_module",
        "base_builds_name",
    ],
)
def test_a_remap_a_node_could_take_through_unread_code_is_not_dead(
    tmp_path: Path, body, remap, imports, base, extra
):
    _atlas, found = _python_dead(tmp_path, body, remap, imports, base, extra)
    assert found == []


def test_a_remap_scoped_to_the_node_it_is_written_on_is_still_dead(tmp_path: Path):
    _atlas, found = _python_dead(tmp_path, "", '("ctrl:statos", "/x")')
    assert len(found) == 1
    assert "status" in found[0].say


def test_a_python_handoff_and_its_literals_are_stated_on_the_file(tmp_path: Path):
    atlas, _found = _python_dead(
        tmp_path,
        "        self.h = SomeLib(self)\n",
        '("odom", "/x")',
        "from some_ros_lib import SomeLib\n",
    )
    values = atlas.entities["file:p|p/ctrl.py"].fact("coded").values
    assert values["node_handoffs"] == ["some_ros_lib.SomeLib"]
    assert "status" in values["name_literals"]


CPP_SERVER = """#include <rclcpp/rclcpp.hpp>
namespace demo {
class Server : public rclcpp::Node {
public:
  explicit Server(const rclcpp::NodeOptions & o) : Node("server", o) {
    pub_ = create_publisher<std_msgs::msg::String>("status", 10);
    BODY
  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};
}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(demo::Server)
"""


def _cpp_dead(tmp_path: Path, body: str, remap: str) -> list[model.Finding]:
    from tests.test_sixth_trial import launch as component_launch

    package(
        tmp_path,
        "p",
        {
            "CMakeLists.txt": "project(p)\nadd_library(server SHARED src/server.cpp)\n"
            'rclcpp_components_register_nodes(server "demo::Server")\n',
            "src/server.cpp": CPP_SERVER.replace("BODY", body),
            "run.launch.py": component_launch(
                'ComposableNodeContainer(name="box", namespace="", package="rclcpp_components",'
                ' executable="component_container", composable_node_descriptions=['
                'ComposableNode(package="p", plugin="demo::Server", name="server", '
                f"remappings=[{remap}])])"
            ),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    return findings(atlas, "remap.matches_nothing")


@pytest.mark.cpp
@pytest.mark.parametrize(
    ("body", "remap"),
    [
        (
            'gen_ = create_generic_subscription("in", "std_msgs/msg/String", rclcpp::QoS(10), '
            "[](std::shared_ptr<rclcpp::SerializedMessage>) {});",
            '("in", "/x")',
        ),
        ('other_lib::attach_diagnostics(this, "diag_in");', '("diag_in", "/x")'),
        ("", '("server:status", "/x")'),
        (
            "helper_ = std::make_shared<other_lib::Helper>(get_node_topics_interface());",
            '("odom", "/x")',
        ),
        ('filter_.attach(this, "scan_in");', '("scan_in", "/x")'),
    ],
    ids=["generic", "free_function", "node_scoped", "node_interface", "literal_backstop"],
)
def test_a_cpp_remap_a_node_could_take_through_unread_code_is_not_dead(
    tmp_path: Path, body: str, remap: str
):
    assert _cpp_dead(tmp_path, body, remap) == []


@pytest.mark.cpp
def test_a_cpp_remap_no_source_could_take_is_still_dead(tmp_path: Path):
    body = "rclcpp::spin_some(this->get_node_base_interface());"
    assert len(_cpp_dead(tmp_path, body, '("stat", "/x")')) == 1


# ---------------------------------------------------------------------------
# review 13: the ids of grouped findings


def _undeclared(tmp_path: Path, profiles: list[tuple[str, str]]) -> list[model.Finding]:
    cfg = Config(
        roots=(RootSpec("src", str(tmp_path / "src")),),
        profiles=tuple(
            ProfileSpec(name, entrypoints=(Entrypoint("launch", "p", entry),))
            for name, entry in profiles
        ),
        out_dir=str(tmp_path / "out"),
        deterministic=True,
    )
    atlas = extract.run(cfg, Workspace.discover(cfg.roots))
    return findings(atlas, "param.set_undeclared")


def test_a_profile_that_adds_a_node_moves_no_set_undeclared_id(tmp_path: Path):
    yaml_path = '__import__("os").path.join(__import__("os").path.dirname(__file__), "p.yaml")'
    package(
        tmp_path,
        "p",
        {
            "p/__init__.py": "",
            "p/a.py": python_node("a", "        self.declare_parameter('rate', 1.0)\n"),
            "p/b.py": python_node("b", "        self.declare_parameter('rate', 1.0)\n"),
            "setup.py": SETUP.format("'a = p.a:main', 'b = p.b:main'"),
            "p.yaml": "/**:\n  ros__parameters:\n    bogus: 1\n",
            "one.launch.py": launch(
                f'Node(package="p", executable="a", name="a", parameters=[{yaml_path}])'
            ),
            "two.launch.py": launch(
                f'Node(package="p", executable="b", name="b", parameters=[{yaml_path}])'
            ),
        },
    )
    (one,) = _undeclared(tmp_path, [("p1", "one.launch.py")])
    (both,) = _undeclared(tmp_path, [("p1", "one.launch.py"), ("p2", "two.launch.py")])
    assert one.id == both.id
    assert one.subject == "file:p|p.yaml"
    assert one.fingerprint != both.fingerprint


GROUPS = """from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, SetParameter
def generate_launch_description():
    return LaunchDescription([
        GroupAction([
            SetParameter(name='{first}', value=1.0),
            Node(package='p', executable='a', name='a'),
        ]),
        GroupAction([
            SetParameter(name='{second}', value=1.0),
            Node(package='p', executable='b', name='b'),
        ]),
    ])
"""


def _groups(tmp_path: Path, first: str, second: str) -> list[model.Finding]:
    package(
        tmp_path,
        "p",
        {
            "p/__init__.py": "",
            "p/a.py": python_node("a", "        self.declare_parameter('rate', 1.0)\n"),
            "p/b.py": python_node("b", "        self.declare_parameter('gain', 1.0)\n"),
            "setup.py": SETUP.format("'a = p.a:main', 'b = p.b:main'"),
            "run.launch.py": GROUPS.format(first=first, second=second),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    return findings(atlas, "param.set_undeclared")


def test_a_set_parameter_one_group_reads_is_still_reported_in_another(tmp_path: Path):
    (found,) = _groups(tmp_path, "rate", "rate")
    assert "/b" in found.detail and "/a " not in found.detail
    assert [row.line_hint for row in found.where] == [11]


def test_two_set_parameter_lines_of_one_name_are_two_findings(tmp_path: Path):
    found = _groups(tmp_path, "speed", "speed")
    assert sorted([row.line_hint for row in item.where] for item in found) == [[7], [11]]


def _one_node(root: Path, node: str) -> model.Atlas:
    package(
        root,
        "p",
        {
            "p/__init__.py": "",
            "p/a.py": python_node("a", "        self.declare_parameter('rate', 1.0)\n"),
            "setup.py": SETUP.format("'a = p.a:main'"),
            "run.launch.py": launch(node),
        },
    )
    return run_twice(config(root, Entrypoint("launch", "p", "run.launch.py")))


def test_a_key_added_to_a_launch_dict_moves_no_set_undeclared_id(tmp_path: Path):
    def ids_of(name: str, keys: str) -> list[str]:
        node = f'Node(package="p", executable="a", name="a", parameters=[{{{keys}}}])'
        atlas = _one_node(tmp_path / name, node)
        return [item.id for item in findings(atlas, "param.set_undeclared")]

    one = ids_of("one", "'bogus': 1")
    assert one and one == ids_of("two", "'bogus': 1, 'other': 2")


def test_a_remap_added_beside_another_moves_no_remap_unresolved_id(tmp_path: Path):
    def ids_of(name: str, remaps: str) -> list[str]:
        node = f'Node(package="ext", executable="calib", name="calib", remappings=[{remaps}])'
        root = tmp_path / name
        package(root, "p", {"run.launch.py": launch(node)})
        atlas = run_twice(config(root, Entrypoint("launch", "p", "run.launch.py")))
        return [item.id for item in findings(atlas, "remap.unresolved")]

    one = ids_of("one", '("~/input/a", "/a")')
    assert one and one == ids_of("two", '("~/input/a", "/a"), ("~/input/b", "/b")')


# ---------------------------------------------------------------------------
# review 13: what a site edge key digests, and where a moved endpoint points


def test_a_site_edge_key_does_not_depend_on_the_order_a_qos_block_was_built_in():
    from rosio import join

    built = {"history": "keep_last", "depth": 10}
    again = {"depth": 10, "history": "keep_last"}
    assert join._tf_key("p", "s", {"qos": built}) == join._tf_key("p", "s", {"qos": again})
    assert join._stated(built) == join._stated(again)
    assert join._stated("x") == "x"


@pytest.mark.cpp
def test_a_header_endpoint_moved_to_its_constructor_cites_no_header_line(tmp_path: Path):
    package(
        tmp_path,
        "p",
        {
            "CMakeLists.txt": "project(p)\ninclude_directories(include)\n"
            "add_library(calib SHARED src/calib.cpp)\n"
            'rclcpp_components_register_node(calib PLUGIN "calib::Calib" EXECUTABLE calib_node)\n',
            "include/p/calib.hpp": CALIB_HPP,
            "src/calib.cpp": CALIB_CPP,
            "run.launch.py": launch('Node(package="p", executable="calib_node", name="calib")'),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    rows = [
        (row.file, row.line_hint)
        for edge in atlas.out_edges("node:/calib")
        if edge.kind == "subscribes"
        for fact in edge.facts
        for row in fact.evidence
        if row.file == "file:p|src/calib.cpp"
    ]
    assert rows and all(line is None for _file, line in rows)


# ---------------------------------------------------------------------------
# N4, one direction for tf_parent, and a frame the code and the model name two ways

MODEL = """<?xml version="1.0"?>
<robot name="bot">
  <link name="base_link"/>
  <link name="CHILD"/>
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="CHILD"/>
    <origin xyz="0.12 0 0.18" rpy="0 0 0"/>
  </joint>
</robot>
"""

LIDAR = """#include <rclcpp/rclcpp.hpp>
#include "tf2_ros/static_transform_broadcaster.h"
class Lidar : public rclcpp::Node {
public:
  Lidar() : Node("lidar") {
    br_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "base_link";
    t.child_frame_id = "CHILD";
    br_->sendTransform(t);
  }
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br_;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Lidar>());
}
"""

BOT_LAUNCH = launch(
    'Node(package="robot_state_publisher", executable="robot_state_publisher", '
    "parameters=[{'robot_description': open(os.path.join(os.path.dirname(__file__), "
    "'bot.urdf')).read()}])",
    'Node(package="p", executable="lidar", name="lidar")',
    imports="import os\n",
)


def _bot(tmp_path: Path, model_child: str, code_child: str) -> model.Atlas:
    package(
        tmp_path,
        "p",
        {
            "CMakeLists.txt": "project(p)\nadd_executable(lidar src/lidar.cpp)\n",
            "src/lidar.cpp": LIDAR.replace("CHILD", code_child),
            "bot.urdf": MODEL.replace("CHILD", model_child),
            "run.launch.py": BOT_LAUNCH,
        },
    )
    return run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))


@pytest.mark.cpp
def test_one_pair_two_layers_state_runs_from_the_child_to_the_parent(tmp_path: Path):
    atlas = _bot(tmp_path, "laser_link", "laser_link")
    pairs = {
        (edge.src, edge.dst, fact.layer)
        for edge in atlas.edges.values()
        if edge.kind == model.EdgeKind.TF_PARENT.value
        for fact in edge.facts
    }
    assert pairs == {
        ("frame:laser_link", "frame:base_link", "declared"),
        ("frame:laser_link", "frame:base_link", "coded"),
    }


def _renamed(atlas: model.Atlas) -> list[model.Finding]:
    return [
        item for item in findings(atlas, "frame.not_in_urdf") if item.subject == "frame:laser_frame"
    ]


@pytest.mark.cpp
def test_a_frame_the_code_broadcasts_under_a_close_model_name_is_reported(tmp_path: Path):
    """Trial 7 D4: the lidar broadcasts base_link to laser_frame and the model calls the same
    part laser_link, which no rule said because a broadcast frame counts as tied."""
    found = _renamed(_bot(tmp_path, "laser_link", "laser_frame"))
    assert len(found) == 1
    item = found[0]
    assert item.severity == "warn"
    assert item.profiles == ["profile:p"]
    assert "laser_link" in item.say and "p/src/lidar.cpp:" in item.say
    assert "p/bot.urdf:" in item.say and "profile p" in item.say
    assert sorted(row.file for row in item.where) == ["file:p|bot.urdf", "file:p|src/lidar.cpp"]


@pytest.mark.cpp
def test_a_frame_the_code_broadcasts_beside_an_unrelated_link_is_not(tmp_path: Path):
    assert _renamed(_bot(tmp_path, "camera_mount", "laser_frame")) == []


def test_a_path_between_two_frames_walks_the_tree_the_transform_tool_walks():
    from rosio.serve import tools

    builder = model.AtlasBuilder()
    builder.declare_layer("declared", present=True)
    builder.declare_layer("coded", present=True)
    urdf = ids.build_file("p", "bot.urdf")
    builder.node(urdf)
    node = ids.build_node("/odometry")
    builder.node(node, name_source="literal")
    for frame in ("laser", "base_link", "odom"):
        builder.node(ids.build_frame(frame))
    builder.edge(
        "tf_parent",
        ids.build_frame("laser"),
        ids.build_frame("base_link"),
        key=ids.edge_key(model=urdf, joint="laser_joint"),
        layer="declared",
        values={
            "model": urdf,
            "joint": "laser_joint",
            "joint_type": "fixed",
            "strategy": "urdf",
            "xyz": [0.1, 0.0, 0.2],
            "rpy": [0.0, 0.0, 0.0],
        },
        confidence="certain",
    )
    # one node broadcasting both frames is a two hop data route between them
    for frame, parent in (("base_link", "odom"), ("laser", "base_link")):
        builder.edge(
            "broadcasts_tf",
            node,
            ids.build_frame(frame),
            key=ids.edge_key(symbol="odometry", child=frame),
            layer="coded",
            values={"parent": parent, "child": frame, "static": False},
        )
    builder.edge(
        "tf_parent",
        ids.build_frame("base_link"),
        ids.build_frame("odom"),
        key=ids.edge_key(symbol="odometry"),
        layer="coded",
        values={"source": "cpp_broadcast", "node": node},
    )
    atlas = builder.build()
    answer = query.path(atlas, "frame:laser", "frame:odom")
    assert answer.data["tree_route"] is True
    assert [(hop["kind"], hop["to"], hop["forward"]) for hop in answer.data["hops"]] == [
        ("tf_parent", "frame:base_link", True),
        ("tf_parent", "frame:odom", True),
    ]
    text = tools.transform(atlas, "laser", "odom")
    assert "joint laser_joint" in text and "base_link -> laser" in text


NAV2_BROADCASTER = """#include <rclcpp/rclcpp.hpp>
#include "nav2_ros_common/tf2_factories.hpp"
class Sim : public rclcpp::Node {
public:
  Sim() : Node("sim") {
    tf_broadcaster_ = nav2::create_transform_broadcaster(this);
    tf_listener_ = nav2::create_transform_listener(*buffer_, this);
  }
  void tick() {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_footprint";
    tf_broadcaster_->sendTransform(t);
  }
  nav2::TransformBroadcaster::SharedPtr tf_broadcaster_;
  nav2::TransformListener::SharedPtr tf_listener_;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Sim>());
}
"""


@pytest.mark.cpp
def test_a_nav2_transform_broadcaster_broadcasts_and_publishes_tf(tmp_path: Path):
    """The nav2 loopback simulator broadcasts through nav2::TransformBroadcaster, which the
    reader did not know, so its transforms carried no broadcaster and no /tf publisher."""
    package(
        tmp_path,
        "p",
        {
            "CMakeLists.txt": "project(p)\nadd_executable(sim sim.cpp)\n",
            "sim.cpp": NAV2_BROADCASTER,
            "run.launch.py": launch('Node(package="p", executable="sim", name="sim")'),
        },
    )
    atlas = run_twice(config(tmp_path, Entrypoint("launch", "p", "run.launch.py")))
    edges = {(edge.kind, edge.dst): edge for edge in atlas.out_edges("node:/sim")}
    assert ("publishes", "topic:/tf") in edges
    assert ("subscribes", "topic:/tf_static") in edges
    broadcast = edges[("broadcasts_tf", "frame:base_footprint")]
    assert broadcast.facts[0].values["broadcaster"] == "nav2::TransformBroadcaster"


# ---------------------------------------------------------------------------
# r12fix leftovers, the names a string local builds

APPENDED = """class N : public rclcpp::Node {
  N() : Node("n") {
    std::string key = "sensor.";
    key.append(id_);
    declare_parameter(key, 1.0);
    std::string known = "lidar.";
    known.append("rate");
    declare_parameter(known, 2.0);
  }
};
"""

BRANCHED_LITERALS = """class N : public rclcpp::Node {
  N() : Node("n") {
    std::string name;
    if (front_) { name = "alpha"; } else { name = "beta"; }
    declare_parameter(name, 1.0);
    std::string same = "gamma";
    declare_parameter(same, 2.0);
    same = "delta";
    declare_parameter(same, 3.0);
  }
};
"""


def _names(text: str) -> tuple[list[str], list[str]]:
    from rosio.collect import cpp_code as cpp

    unit = cpp.parse_text(text, path="n.cpp")
    cpp.analyse([unit])
    report = cpp.file_report(unit)
    declared = sorted(row["name"] for row in report.get("parameters", []))
    blind = [row["expr"] for row in report.get("unresolved", []) if row["want"] == "param_name"]
    return declared, blind


@pytest.mark.cpp
def test_an_appended_name_is_the_built_name_or_a_blind_spot():
    assert _names(APPENDED) == (["lidar.rate"], ['"sensor." + id_'])


@pytest.mark.cpp
def test_a_name_two_branches_assign_literals_is_neither_literal():
    """Section 7: a condition choosing between names could build any of them, so the
    declaration is a blind spot and declares neither. A straight reassignment is read in order."""
    assert _names(BRANCHED_LITERALS) == (["delta", "gamma"], ["name"])


# ---------------------------------------------------------------------------
# review 14


@pytest.mark.cpp
@pytest.mark.parametrize(
    ("model_child", "code_child"),
    [
        ("imu_link", "gps_link"),
        ("lidar_link", "imu_link"),
        ("camera_link", "camera_link_optical"),
        ("left_wheel", "right_wheel"),
        ("base_footprint", "base_link_inertia"),
    ],
)
def test_a_broadcast_beside_a_link_of_close_spelling_and_another_stem_is_not_renamed(
    tmp_path: Path, model_child: str, code_child: str
):
    """Two frames spelled alike are two parts unless only their frame suffix differs."""
    atlas = _bot(tmp_path, model_child, code_child)
    assert [
        item
        for item in findings(atlas, "frame.not_in_urdf")
        if item.subject == f"frame:{code_child}"
    ] == []


HANDED = """from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('with_b', default_value='false'),
        SetParameter(name='bogus', value=1.0),
        Node(package='p', executable='a', name='a'),
        Node(package='p', executable='b', name='b',
             condition=IfCondition(LaunchConfiguration('with_b'))),
    ])
"""


def test_a_profile_that_starts_one_more_node_of_a_set_parameter_moves_no_id(tmp_path: Path):
    package(
        tmp_path,
        "p",
        {
            "p/__init__.py": "",
            "p/a.py": python_node("a", "        self.declare_parameter('rate', 1.0)\n"),
            "p/b.py": python_node("b", "        self.declare_parameter('rate', 1.0)\n"),
            "setup.py": SETUP.format("'a = p.a:main', 'b = p.b:main'"),
            "run.launch.py": HANDED,
        },
    )
    entry = Entrypoint("launch", "p", "run.launch.py")
    with_b = Entrypoint("launch", "p", "run.launch.py", args=(("with_b", "true"),))

    def found(profiles: tuple[ProfileSpec, ...]) -> list[model.Finding]:
        cfg = Config(
            roots=(RootSpec("src", str(tmp_path / "src")),),
            profiles=profiles,
            out_dir=str(tmp_path / "out"),
            deterministic=True,
        )
        return findings(extract.run(cfg, Workspace.discover(cfg.roots)), "param.set_undeclared")

    (one,) = found((ProfileSpec("p1", entrypoints=(entry,)),))
    (both,) = found(
        (ProfileSpec("p1", entrypoints=(entry,)), ProfileSpec("p2", entrypoints=(with_b,)))
    )
    assert one.id == both.id
    assert "/b" in both.detail and "/b" not in one.detail


def test_a_set_parameter_is_located_by_its_group_and_never_by_a_line(tmp_path: Path):
    found = _groups(tmp_path, "speed", "speed")
    locators = sorted(row.locator for item in found for row in item.where)
    assert len(set(locators)) == 2
    assert all(item.startswith("SetParameter(") for item in locators)


SWITCHED = """class N : public rclcpp::Node {
  N() : Node("n") {
    std::string n = "front";
    switch (mode_) {
      case 0: n = "left"; break;
      case 1: n = "right"; break;
    }
    declare_parameter(n, 1.0);
    std::string m = "base";
    for (auto & s : items_) { m = "item"; }
    declare_parameter(m, 1.0);
    std::string w = "idle";
    while (busy_) { w = "busy"; }
    declare_parameter(w, 1.0);
    std::string t = "one";
    try { t = "two"; } catch (...) {}
    declare_parameter(t, 1.0);
  }
};
"""


@pytest.mark.cpp
def test_a_name_a_switch_arm_or_a_loop_body_assigns_is_neither_literal():
    assert _names(SWITCHED) == (["two"], ["m", "n", "w"])


def test_a_name_literal_is_no_file_path_and_no_comment():
    from rosio.collect import cpp_code, py_code

    source = (
        b'auto a = "/scan"; auto b = "/tmp/amcl_saved_pose"; auto c = "/dev/ttyUSB0";\n'
        b'// see "/var/lib/map"\n/* "commented" */ auto d = \'"\'; auto e = "odom";\n'
    )
    assert cpp_code.name_literals(source) == ["/scan", "odom"]
    assert [
        item
        for item in ("/scan", "/tmp/pose", "/media/u/map", "/dev/ttyUSB0", "/tmp")
        if py_code.is_name_literal(item)
    ] == ["/scan", "/tmp"]


def test_an_action_client_built_by_keywords_is_an_endpoint_and_no_handoff(tmp_path: Path):
    atlas, found = _python_dead(
        tmp_path,
        "        self.c = ActionClient(node=self, action_type=Go, action_name='go_to')\n",
        '("go_too", "/x")',
        "from rclpy.action.client import ActionClient\nfrom p.msg import Go\n",
    )
    assert "node_handoffs" not in atlas.entities["file:p|p/ctrl.py"].fact("coded").values
    assert len(found) == 1
    assert "go_to" in found[0].say
