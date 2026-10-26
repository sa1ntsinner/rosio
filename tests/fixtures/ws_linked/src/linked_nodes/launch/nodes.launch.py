from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="linked_nodes",
                executable="filter_node",
                name="selected_filter",
                remappings=[("filtered", "filtered_local")],
            ),
            Node(
                package="linked_nodes",
                executable="navigation_node",
                name="selected_navigation",
                remappings=[("position", "position_local")],
            ),
        ]
    )
