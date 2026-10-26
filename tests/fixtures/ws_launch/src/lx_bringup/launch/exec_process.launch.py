"""Case 9: an ExecuteProcess that is really a ROS node.

Its whole identity is in the argv: name, namespace, one remap, one parameter assignment
and one parameters file. Both the -r form and the bare __node form of a remap rule appear,
because both are accepted at runtime. The file is listed after the -p assignment, so for
the one key they share the file is the later input.
"""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess

_SHARE = os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir))


def generate_launch_description():
    return LaunchDescription(
        [
            ExecuteProcess(
                name="cropper_proc",
                cmd=[
                    "python3",
                    os.path.join(_SHARE, "scripts", "crop_cloud.py"),
                    "--ros-args",
                    "-p",
                    "rate_hz:=20.0",
                    "-p",
                    "decimate_rings:=2",
                    "-r",
                    "input:=/points_raw",
                    "__node:=cropper",
                    "__ns:=/prep",
                    "--params-file",
                    os.path.join(_SHARE, "config", "crop.yaml"),
                ],
            ),
        ]
    )
