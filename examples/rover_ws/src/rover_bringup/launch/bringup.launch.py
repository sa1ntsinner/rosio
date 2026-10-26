"""The example rover: description, lidar filter and safety stop."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory("rover_bringup")
    description = get_package_share_directory("rover_description")
    params = os.path.join(bringup, "config", "rover.yaml")
    with open(os.path.join(description, "urdf", "rover.urdf")) as urdf:
        robot_description = urdf.read()

    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
            ),
            Node(
                package="rover_perception",
                executable="scan_filter",
                name="scan_filter",
                parameters=[params, {"max_range": 10.0}],
                remappings=[("scan", "/lidar/scan")],
            ),
            Node(
                package="rover_control",
                executable="safety_stop",
                name="safety_stop",
                parameters=[params],
            ),
        ]
    )
