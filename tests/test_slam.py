from pathlib import Path

import pytest

from rosio.resolve import launch_exec


@pytest.mark.ros
@pytest.mark.parametrize(
    ("arguments", "managed"),
    [
        ((), True),
        ((("autostart", "false"),), False),
        ((("use_lifecycle_manager", "true"),), False),
    ],
)
@pytest.mark.parametrize("events_first", [False, True])
def test_launch_lifecycle_events_reach_the_target(
    tmp_path: Path, arguments, managed, events_first
) -> None:
    entry = tmp_path / "lifecycle.launch.py"
    entry.write_text(
        "from launch import LaunchDescription\n"
        "from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler\n"
        "from launch.conditions import IfCondition\n"
        "from launch.events import matches_action\n"
        "from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution\n"
        "from launch_ros.actions import LifecycleNode\n"
        "from launch_ros.event_handlers import OnStateTransition\n"
        "from launch_ros.events.lifecycle import ChangeState\n"
        "def generate_launch_description():\n"
        '    node = LifecycleNode(package="slam_toolbox",\n'
        '        executable="async_slam_toolbox_node", name="slam_toolbox", namespace="")\n'
        '    condition = IfCondition(AndSubstitution(LaunchConfiguration("autostart"),\n'
        '        NotSubstitution(LaunchConfiguration("use_lifecycle_manager"))))\n'
        "    configure = EmitEvent(event=ChangeState(\n"
        "        lifecycle_node_matcher=matches_action(node), transition_id=1),\n"
        "        condition=condition)\n"
        "    activate = RegisterEventHandler(OnStateTransition(target_lifecycle_node=node,\n"
        '        start_state="configuring", goal_state="inactive", entities=[\n'
        "            EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(node),\n"
        "                transition_id=3))]), condition=condition)\n"
        "    actions = [configure, activate, node] if "
        + repr(events_first)
        + " else [node, configure, activate]\n"
        "    return LaunchDescription([\n"
        '        DeclareLaunchArgument("autostart", default_value="true"),\n'
        '        DeclareLaunchArgument("use_lifecycle_manager", default_value="false"),\n'
        "        *actions])\n"
    )
    walk = launch_exec.walk(str(entry), args=arguments)
    assert not walk.exceptions
    assert len(walk.nodes) == 1
    transitions = walk.nodes[0].attrs.get("lifecycle_transitions", [])
    assert bool(transitions) is managed
    if managed:
        assert set(transitions) == {"configure", "activate"}


@pytest.mark.ros
def test_an_unknown_lifecycle_event_condition_does_not_prove_management(tmp_path: Path) -> None:
    entry = tmp_path / "unknown.launch.py"
    entry.write_text(
        "from launch import LaunchDescription\n"
        "from launch.actions import EmitEvent\n"
        "from launch.conditions import IfCondition\n"
        "from launch.events import matches_action\n"
        "from launch.substitutions import Command\n"
        "from launch_ros.actions import LifecycleNode\n"
        "from launch_ros.events.lifecycle import ChangeState\n"
        "def generate_launch_description():\n"
        "    node = LifecycleNode(package='demo', executable='node', name='node', namespace='')\n"
        "    event = EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(node),\n"
        "        transition_id=1), condition=IfCondition(Command(['echo true'])))\n"
        "    return LaunchDescription([node, event])\n"
    )
    walk = launch_exec.walk(str(entry))
    assert len(walk.nodes) == 1
    assert not walk.nodes[0].attrs.get("lifecycle_transitions")


@pytest.mark.ros
def test_a_lifecycle_handler_without_transition_events_is_not_management(tmp_path: Path) -> None:
    entry = tmp_path / "observer.launch.py"
    entry.write_text(
        "from launch import LaunchDescription\n"
        "from launch.actions import LogInfo, RegisterEventHandler\n"
        "from launch_ros.actions import LifecycleNode\n"
        "from launch_ros.event_handlers import OnStateTransition\n"
        "def generate_launch_description():\n"
        "    node = LifecycleNode(package='demo', executable='node', name='node', namespace='')\n"
        "    observer = RegisterEventHandler(OnStateTransition(target_lifecycle_node=node,\n"
        "        goal_state='inactive', entities=[LogInfo(msg='ready')]))\n"
        "    return LaunchDescription([node, observer])\n"
    )
    walk = launch_exec.walk(str(entry))
    assert len(walk.nodes) == 1
    assert not walk.nodes[0].attrs.get("lifecycle_transitions")
