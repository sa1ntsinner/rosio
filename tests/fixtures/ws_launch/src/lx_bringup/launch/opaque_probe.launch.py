"""Case 3: nodes reachable only through an OpaqueFunction that probes the filesystem.

The function performs a LaunchConfiguration, asks os.path.exists about a serialised map
and picks a different executable per outcome, so the node identity depends on a file on
disk and not on the launch arguments alone. An AST walker sees zero nodes here.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_CONFIG = os.path.normpath(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir, "config")
)


def _setup(context, *_args, **_kwargs):
    map_base = LaunchConfiguration("map_base").perform(context)
    has_map = bool(map_base) and os.path.exists(map_base + ".posegraph")
    executable = "localization_slam" if has_map else "async_slam"
    overrides = {"mode": "localization" if has_map else "mapping"}
    return [
        Node(
            package="lx_sensors",
            executable=executable,
            name="slam_toolbox",
            parameters=[os.path.join(_CONFIG, "slam.yaml"), overrides],
            remappings=[("/map", "/slam_map")],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_base", default_value=""),
            OpaqueFunction(function=_setup),
        ]
    )
