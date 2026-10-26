"""Case 11: raises at description build time unless an environment variable is set.

Importing the module is safe. Calling generate_launch_description without LX_ROOT raises,
which must be recorded per file and carried on rather than emptying the layer.
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    if not os.environ.get("LX_ROOT"):
        raise RuntimeError("LX_ROOT must point at the fixture workspace root")
    return LaunchDescription(
        [
            Node(
                package="lx_sensors",
                executable="health_monitor",
                name="health_monitor",
                parameters=[{"root_configured": True}],
            ),
        ]
    )
