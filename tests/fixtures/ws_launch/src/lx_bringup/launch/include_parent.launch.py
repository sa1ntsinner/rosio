"""Case 10: IncludeLaunchDescription passing launch_arguments into a child launch file.

The child path is a substitution, not a resolved string, so include resolution has to run
the substitution engine.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, ThisLaunchFileDir


def generate_launch_description():
    child = PythonLaunchDescriptionSource(
        PathJoinSubstitution([ThisLaunchFileDir(), "include_child.launch.py"])
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                child,
                launch_arguments={
                    "child_name": "relay_child",
                    "child_rate": "30.0",
                }.items(),
            ),
        ]
    )
