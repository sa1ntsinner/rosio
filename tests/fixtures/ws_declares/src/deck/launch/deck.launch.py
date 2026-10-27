from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node

_CONFIG = Path(__file__).resolve().parent.parent / "config"


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="deck",
                executable="deck",
                name="deck",
                parameters=[str(_CONFIG / "deck.yaml")],
            )
        ]
    )
