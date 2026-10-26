"""Case 14: a launch argument with no default.

A walk with no arguments cannot resolve params_file and the description fails while the
arguments are being visited. The entry point must be reported as unresolvable without
arguments, not silently skipped.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                description="Absolute path to the merged parameters file.",
            ),
            Node(
                package="lx_bringup",
                executable="lx_controller",
                name="controller_server",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
