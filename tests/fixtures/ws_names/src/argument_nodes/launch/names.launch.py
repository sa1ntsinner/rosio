from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="argument_nodes", executable=executable)
            for executable in (
                "caller",
                "py_talker",
                "component_talker",
                "ambiguous",
                "header_default",
                "py_default",
            )
        ]
    )
