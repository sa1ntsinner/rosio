"""Case 10, child half. Also usable as an entry point on its own, with its own defaults.

The node name and one inline parameter value are substitutions, so the same file resolves
to two different node names depending on who included it. The rate value is a string in the
launch argument and arrives at the node as a float, because launch_ros infers the type.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("child_name", default_value="relay_default"),
            DeclareLaunchArgument("child_rate", default_value="10.0"),
            Node(
                package="lx_sensors",
                executable="scan_relay",
                name=LaunchConfiguration("child_name"),
                parameters=[{"rate_hz": LaunchConfiguration("child_rate")}],
            ),
        ]
    )
