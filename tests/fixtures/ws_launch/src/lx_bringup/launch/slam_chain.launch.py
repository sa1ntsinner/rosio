"""Case 5: both halves of the hard chain in one file.

An OpaqueFunction performs a LaunchConfiguration, asks the filesystem about a serialised
map, picks the executable from the outcome, and hands the nodes to the on_exit list of a
gate process. Neither half on its own reaches them: a walker that cannot enter the function
sees nothing, and one that stops at the event handler sees the gate process only.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_CONFIG = os.path.normpath(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir, "config")
)


def _setup(context, *_args, **_kwargs):
    map_base = LaunchConfiguration("map_base").perform(context)
    has_map = bool(map_base) and os.path.exists(map_base + ".posegraph")
    gate = ExecuteProcess(
        name="map_gate",
        cmd=["python3", "-c", "import sys; sys.exit(0)"],
    )
    slam = Node(
        package="lx_sensors",
        executable="localization_slam" if has_map else "async_slam",
        name="slam_toolbox",
        parameters=[
            os.path.join(_CONFIG, "slam.yaml"),
            {"mode": "localization" if has_map else "mapping"},
        ],
        remappings=[("/map", "/slam_map")],
    )
    sanitizer = Node(
        package="lx_sensors",
        executable="scan_relay",
        name="grid_trimmer",
        remappings=[("/scan", "/scan_filtered")],
    )
    return [
        gate,
        RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=[slam, sanitizer])),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_base", default_value=""),
            OpaqueFunction(function=_setup),
        ]
    )
