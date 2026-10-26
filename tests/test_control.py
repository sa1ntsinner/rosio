from pathlib import Path

import pytest

from rosio.resolve import launch_exec


@pytest.mark.ros
@pytest.mark.parametrize(
    "path",
    [
        "PathJoinSubstitution([FindPackageShare('missing_package'), 'view.rviz'])",
        "PathSubstitution(FindPackageShare('missing_package')) / 'view.rviz'",
    ],
)
def test_missing_argument_package_keeps_launch_loadable_and_later_nodes(tmp_path: Path, path: str):
    entry = tmp_path / "partial.launch.py"
    entry.write_text(
        "from launch import LaunchDescription\n"
        "from launch.substitutions import Command, PathJoinSubstitution, PathSubstitution\n"
        "from launch_ros.actions import Node\n"
        "from launch_ros.substitutions import FindPackageShare\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription([\n"
        "        Node(package='robot_state_publisher', executable='robot_state_publisher',\n"
        "             parameters=[{'robot_description': Command(['xacro missing.xacro'])}]),\n"
        "        Node(package='rviz2', executable='rviz2', name='rviz2',\n"
        f"             arguments=['-d', {path}]),\n"
        "        Node(package='demo', executable='later', name='later')])\n"
    )
    walk = launch_exec.walk(str(entry), launch_sandbox="off")
    assert walk.loadable
    assert any(node.name == "later" for node in walk.nodes)
    assert any(error.exc_type == "PackageNotFoundError" for error in walk.exceptions)
    assert {error.kept for error in walk.exceptions if "missing_package" in error.message} == {
        "Node(rviz2)"
    }


@pytest.mark.ros
def test_one_missing_package_in_a_parameters_item_is_one_failure(tmp_path: Path):
    """ros2_control_demos example 3: launch_ros fails the same parameters item inside
    `_perform_substitutions` and again when the walk evaluates the parameters, one cause."""
    entry = tmp_path / "params.launch.py"
    entry.write_text(
        "from launch import LaunchDescription\n"
        "from launch.substitutions import PathJoinSubstitution\n"
        "from launch_ros.actions import Node\n"
        "from launch_ros.substitutions import FindPackageShare\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription([\n"
        "        Node(package='controller_manager', executable='ros2_control_node',\n"
        "             parameters=[PathJoinSubstitution(\n"
        "                 [FindPackageShare('missing_package'), 'config', 'c.yaml'])]),\n"
        "        Node(package='demo', executable='later', name='later',\n"
        "             arguments=['-d', PathJoinSubstitution(\n"
        "                 [FindPackageShare('missing_package'), 'view.rviz'])])])\n"
    )
    walk = launch_exec.walk(str(entry), launch_sandbox="off")
    assert sorted(error.kept for error in walk.exceptions) == [
        "Node(later)",
        "Node(ros2_control_node)",
    ]


@pytest.mark.ros
def test_a_raise_that_drops_its_action_keeps_nothing(tmp_path: Path):
    entry = tmp_path / "dropped.launch.py"
    entry.write_text(
        "from launch import LaunchDescription\n"
        "from launch.actions import OpaqueFunction\n"
        "from launch_ros.actions import Node\n"
        "def boom(context):\n"
        "    raise RuntimeError('boom')\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription([OpaqueFunction(function=boom),\n"
        "        Node(package='demo', executable='later', name='later')])\n"
    )
    walk = launch_exec.walk(str(entry), launch_sandbox="off")
    assert any(node.name == "later" for node in walk.nodes)
    assert [error.kept for error in walk.exceptions if "boom" in error.message] == [""]


@pytest.mark.ros
def test_a_failing_parameters_item_keeps_the_node_remaps(tmp_path: Path):
    """launch_ros expands the remappings after the parameters, so a raise there leaves them
    unexpanded and the walk reads them from the action itself."""
    entry = tmp_path / "remaps.launch.py"
    entry.write_text(
        "from launch import LaunchDescription\n"
        "from launch.substitutions import PathJoinSubstitution\n"
        "from launch_ros.actions import Node, SetRemap\n"
        "from launch_ros.substitutions import FindPackageShare\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription([SetRemap(src='scan', dst='/robot/scan'),\n"
        "        Node(package='demo_nodes_cpp', executable='talker', name='talker',\n"
        "             parameters=[PathJoinSubstitution(\n"
        "                 [FindPackageShare('missing_package'), 'c.yaml'])],\n"
        "             remappings=[('chatter', '/robot/chatter')])])\n"
    )
    walk = launch_exec.walk(str(entry), launch_sandbox="off")
    (node,) = [node for node in walk.nodes if node.name == "talker"]
    assert [(remap.frm, remap.to) for remap in node.remaps] == [
        ("scan", "/robot/scan"),
        ("chatter", "/robot/chatter"),
    ]
