from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = str(Path(get_package_share_directory("two_nodes")) / "config" / "pair.yaml")
    return LaunchDescription(
        [
            Node(
                package="two_nodes",
                executable="pair",
                name="pair",
                parameters=[params],
            ),
        ]
    )
