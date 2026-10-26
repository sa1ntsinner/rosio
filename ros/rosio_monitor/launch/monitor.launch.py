from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rosio_monitor",
                executable="monitor",
                name="rosio_monitor",
                output="screen",
            ),
        ]
    )
