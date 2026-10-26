"""Case 13: the parameters list order decides the winner.

inline_wins gets [file, dict] so the dict wins for width. file_wins gets [dict, file] so
the file wins for the same key. Both nodes read the same file, so a resolver that merges
without keeping the list order inverts one of the two deliberately.
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node

_CONFIG = os.path.normpath(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir, "config")
)


def generate_launch_description():
    order_file = os.path.join(_CONFIG, "order_base.yaml")
    return LaunchDescription(
        [
            Node(
                package="lx_bringup",
                executable="lx_controller",
                name="inline_wins",
                parameters=[order_file, {"width": 40}],
            ),
            Node(
                package="lx_bringup",
                executable="lx_controller",
                name="file_wins",
                parameters=[{"width": 40}, order_file],
            ),
        ]
    )
