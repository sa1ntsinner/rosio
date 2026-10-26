"""Case 2: a Node with no name= and no namespace=.

The runtime name comes from the executable itself, so the declared layer must say
name_source executable_default and must not invent a name. The namespace is likewise
not stated by launch, and inventing the root namespace here fabricates drift.
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node

_CONFIG = os.path.normpath(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir, "config")
)


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="lx_sensors",
                executable="imu_driver",
                parameters=[os.path.join(_CONFIG, "imu_driver.yaml")],
            ),
        ]
    )
