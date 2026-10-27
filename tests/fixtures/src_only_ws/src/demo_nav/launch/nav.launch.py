from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare("demo_nav"), "params", "nav.yaml"])
    return LaunchDescription(
        [Node(package="demo_nav", executable="planner", name="planner", parameters=[params])]
    )
