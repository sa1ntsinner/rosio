"""Case 7: GroupAction with PushRosNamespace, SetRemap and SetParameter.

The group scoped remap and the group scoped parameter apply to every launch_ros Node in
the group, and to nothing outside it. Group values are applied before the node's own, so
the order of the remap list and of the parameters list is the precedence rule.
"""

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace, SetParameter, SetRemap


def generate_launch_description():
    return LaunchDescription(
        [
            GroupAction(
                [
                    PushRosNamespace("yard"),
                    SetRemap("scan", "/yard/scan_filtered"),
                    SetParameter(name="use_sim_time", value=True),
                    Node(
                        package="lx_sensors",
                        executable="scan_relay",
                        name="scan_relay",
                        remappings=[("input", "/yard/points")],
                    ),
                    Node(
                        package="lx_bringup",
                        executable="lx_controller",
                        name="controller",
                        parameters=[{"controller_frequency": 20.0}],
                    ),
                ]
            ),
            Node(
                package="lx_sensors",
                executable="imu_driver",
                name="outside_node",
            ),
        ]
    )
