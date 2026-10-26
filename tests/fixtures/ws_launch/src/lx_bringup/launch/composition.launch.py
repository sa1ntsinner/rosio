"""Case 8: a ComposableNodeContainer with two ComposableNode descriptions.

A component has no executable, and its namespace comes from launch_ros rather than from
the container, so the namespaced component is a sibling of the container and not a child.
"""

import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

_CONFIG = os.path.normpath(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir, "config")
)


def generate_launch_description():
    return LaunchDescription(
        [
            ComposableNodeContainer(
                package="rclcpp_components",
                executable="component_container",
                name="sensor_container",
                namespace="",
                composable_node_descriptions=[
                    ComposableNode(
                        package="lx_sensors",
                        plugin="lx_sensors::CloudFilter",
                        name="cloud_filter",
                        parameters=[os.path.join(_CONFIG, "filter.yaml")],
                        remappings=[("input", "/points")],
                    ),
                    ComposableNode(
                        package="lx_sensors",
                        plugin="lx_sensors::ScanMaker",
                        name="scan_maker",
                        namespace="scan",
                    ),
                ],
            ),
        ]
    )
