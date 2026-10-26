"""Case 6: IfCondition and UnlessCondition on two nodes, both branches present.

With the default argument only unless_node is created, so a walk that reports both nodes
without a gate trail claims a pair that never coexists.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    extra = LaunchConfiguration("enable_extra")
    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_extra", default_value="false"),
            Node(
                package="lx_sensors",
                executable="scan_relay",
                name="if_node",
                condition=IfCondition(extra),
            ),
            Node(
                package="lx_sensors",
                executable="imu_driver",
                name="unless_node",
                condition=UnlessCondition(extra),
            ),
        ]
    )
