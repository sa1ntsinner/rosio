from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = Path(get_package_share_directory("uninstalled_config"))
    params = str(share / "config" / "params.yaml")
    xacro = PathJoinSubstitution(
        [FindPackageShare("uninstalled_config"), "urdf", "robot.urdf.xacro"]
    )
    description = ParameterValue(Command(["xacro ", xacro]), value_type=str)
    return LaunchDescription(
        [
            Node(
                package="uninstalled_config",
                executable="talker",
                name="talker",
                parameters=[params],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": description, "use_sim_time": False}],
                remappings=[("joint_states", "joints")],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(share / "rviz" / "view.rviz")],
            ),
        ]
    )
