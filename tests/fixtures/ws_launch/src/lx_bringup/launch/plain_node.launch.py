"""Case 1: a plain Node with a literal name, two remaps and one parameters file."""

import os

from launch import LaunchDescription
from launch_ros.actions import Node

_CONFIG = os.path.normpath(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir, "config")
)


def generate_launch_description():
    params = os.path.join(_CONFIG, "lidar_driver.yaml")
    return LaunchDescription(
        [
            Node(
                package="lx_sensors",
                executable="lidar_driver",
                name="lidar_driver",
                namespace="sensors",
                parameters=[params],
                remappings=[
                    ("points", "/sensors/points_raw"),
                    ("/tf", "tf"),
                ],
            ),
        ]
    )
