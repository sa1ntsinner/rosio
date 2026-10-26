"""A launch file living in the second package, so discovery cannot assume one launch dir.

It resolves its own config out of the source tree next to the file rather than out of the
install share, which is why the same package can hand two different roots to one node. The
namespace argument exists because the launcher passes it, and an argument a launch file does
not declare makes an include raise.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_CONFIG = os.path.normpath(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir, "config")
)


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            Node(
                package="lx_sensors",
                executable="lidar_driver",
                name="lidar_driver",
                namespace=LaunchConfiguration("namespace"),
                parameters=[os.path.join(_CONFIG, "lidar_driver.yaml")],
                remappings=[("points", "/points")],
            ),
        ]
    )
