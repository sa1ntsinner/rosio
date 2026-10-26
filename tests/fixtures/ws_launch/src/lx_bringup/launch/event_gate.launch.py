"""Case 4: nodes reachable only as the on_exit payload of an OnProcessExit handler.

The gate process is not a ROS node: it carries no --ros-args tail, so it must not become
a node entity. The two nodes behind it both carry the same gate trail.
"""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

_CONFIG = os.path.normpath(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir, "config")
)


def generate_launch_description():
    gate = ExecuteProcess(
        name="ready_gate",
        cmd=["python3", "-c", "import sys; sys.exit(0)"],
    )
    driver = Node(
        package="lx_sensors",
        executable="lidar_driver",
        name="os_driver",
        namespace="ouster",
        parameters=[os.path.join(_CONFIG, "zoo_node_wildcard.yaml")],
    )
    manager = Node(
        package="lx_sensors",
        executable="health_monitor",
        name="lifecycle_manager_sensors",
        parameters=[{"autostart": True, "node_names": ["ouster/os_driver"]}],
    )
    return LaunchDescription(
        [
            gate,
            RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=[driver, manager])),
        ]
    )
