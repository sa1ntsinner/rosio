"""Case 5: a TimerAction delaying a node, plus a gate whose on_exit holds another gate.

deep_node is two OnProcessExit hops from the description root, so its gate trail is
ordered outer first. late_node exists only after a five second timer.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    stage_one = ExecuteProcess(
        name="stage_one",
        cmd=["python3", "-c", "import sys; sys.exit(0)"],
    )
    stage_two = ExecuteProcess(
        name="stage_two",
        cmd=["python3", "-c", "import sys; sys.exit(0)"],
    )
    late_node = Node(
        package="lx_sensors",
        executable="scan_relay",
        name="late_node",
    )
    deep_node = Node(
        package="lx_sensors",
        executable="lidar_driver",
        name="deep_node",
        namespace="deep",
    )
    inner_gate = RegisterEventHandler(OnProcessExit(target_action=stage_two, on_exit=[deep_node]))
    outer_gate = RegisterEventHandler(
        OnProcessExit(target_action=stage_one, on_exit=[stage_two, inner_gate])
    )
    return LaunchDescription(
        [
            TimerAction(period=5.0, actions=[late_node]),
            stage_one,
            outer_gate,
        ]
    )
