from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav = Path(get_package_share_directory("demo_nav"))
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("demo_nav"), "launch", "nav.launch.py"])
                )
            ),
            Node(
                package="demo_nav",
                executable="controller",
                name="controller",
                parameters=[str(nav / "params" / "nav.yaml")],
            ),
            Node(
                package="demo_nav",
                executable="recovery",
                name="recovery",
                parameters=[str(nav / "params" / "recovery.yaml")],
            ),
        ]
    )
