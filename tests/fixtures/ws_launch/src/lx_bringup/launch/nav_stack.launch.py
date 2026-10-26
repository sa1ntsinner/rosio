"""The parameter zoo entry point. Every expected candidate chain is anchored here.

The node set is spelled out with literal names and explicit namespaces on purpose, so the
owner of every parameter is unambiguous and a chain can be checked without depending on a
name resolution table. The base and the overlay are handed to one
node as two ordered list entries, which is the shape the launcher produces, and params_file
is an argument so the launcher can displace the base with its own merged temp file.

The use_sim_time entry is a launch argument passed straight into an inline dict. launch_ros
infers the type of such a value from its text, so the string 'false' arrives at the node as
the boolean False. A resolver that keeps the raw text instead reports a type change.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_CONFIG = os.path.normpath(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir, "config")
)


def _cfg(name):
    return os.path.join(_CONFIG, name)


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    overlay = _cfg("zoo_overlay.yaml")
    wildcard = _cfg("zoo_wildcard.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=_cfg("zoo_base.yaml")),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("profile", default_value="navigation_zoo"),
            Node(
                package="lx_bringup",
                executable="lx_controller",
                name="controller_server",
                namespace="",
                parameters=[params_file, overlay, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="lx_bringup",
                executable="lx_controller",
                name="local_costmap",
                namespace="local_costmap",
                parameters=[params_file, overlay],
            ),
            Node(
                package="lx_sensors",
                executable="health_monitor",
                name="map_server",
                namespace="",
                parameters=[params_file],
            ),
            Node(
                package="lx_sensors",
                executable="health_monitor",
                name="lifecycle_manager_navigation",
                namespace="",
                parameters=[{"autostart": True, "node_names": ["controller_server"]}],
            ),
            Node(
                package="lx_sensors",
                executable="lidar_driver",
                name="os_driver",
                namespace="ouster",
                parameters=[_cfg("zoo_node_wildcard.yaml"), wildcard],
            ),
            Node(
                package="lx_sensors",
                executable="scan_relay",
                name="scan_relay",
                namespace="",
                parameters=[_cfg("zoo_dotted.yaml"), wildcard],
            ),
        ]
    )
