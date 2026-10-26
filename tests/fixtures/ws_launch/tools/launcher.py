"""Case 16: a launcher outside any package, the way the real profiles are composed.

No launch machinery here at all. Every command is a string built by f string interpolation,
so the only way to recover the profile is to build the string and parse the ros2 launch or
ros2 run argv out of it. One command names a launch file that does not exist, one runs a
node with a --params-file and a -p override, and one hands a launch file a parameters file
that this module merged itself and wrote to a temp path.
"""

from __future__ import annotations

import os
import tempfile

import yaml

REPO = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
CONFIG = os.path.join(REPO, "src", "lx_bringup", "config")
ZOO_BASE = os.path.join(CONFIG, "zoo_base.yaml")
ZOO_OVERLAY = os.path.join(CONFIG, "zoo_overlay.yaml")
CROP_PARAMS = os.path.join(CONFIG, "crop.yaml")

PROFILE = "navigation_zoo"
SPEED_LIMIT = 1.0


def _deep_merge(base: dict, overlay: dict) -> dict:
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def merged_params() -> str:
    """Merge the pair the way the runtime does and hand back a temp path.

    The path is worthless as provenance: it exists in no repository. What matters is the
    two source files, the merge order and this function as the merge site.
    """
    with open(ZOO_BASE) as handle:
        params = yaml.safe_load(handle)
    with open(ZOO_OVERLAY) as handle:
        overlay = yaml.safe_load(handle)
    _deep_merge(params, overlay)
    # A value that comes from python and from no file at all.
    params["controller_server"]["ros__parameters"]["FollowPath"]["desired_linear_vel"] = SPEED_LIMIT
    target = tempfile.NamedTemporaryFile(
        mode="w", suffix=f"_{PROFILE}.yaml", dir="/tmp", delete=False
    )
    yaml.safe_dump(params, target)
    target.close()
    return target.name


def navigation_command(params_file: str, use_sim_time: bool = False) -> str:
    return (
        f"ros2 launch lx_bringup nav_stack.launch.py"
        f" params_file:={params_file}"
        f" use_sim_time:={'true' if use_sim_time else 'false'}"
        f" profile:={PROFILE}"
    )


def sensors_command(namespace: str = "sensors") -> str:
    return f"ros2 launch lx_sensors sensors_only.launch.py namespace:={namespace}"


def mapping_command(map_base: str) -> str:
    return f"ros2 launch lx_bringup opaque_probe.launch.py map_base:={map_base}"


def missing_command(mode: str = "grid") -> str:
    """Names a launch file that is not in the workspace. A walker must tolerate that."""
    return f"ros2 launch lx_bringup absent_profile.launch.py mode:={mode}"


def health_command(window_sec: float = 2.0) -> str:
    return (
        f"ros2 run lx_sensors health_monitor --ros-args"
        f" --params-file {CROP_PARAMS}"
        f" -p window_sec:={window_sec}"
        f" -p profile:={PROFILE}"
    )


def relay_command(input_topic: str = "/points") -> str:
    return f"ros2 run lx_sensors scan_relay --ros-args -r input:={input_topic} -r __ns:=/prep"


def commands() -> list[str]:
    return [
        navigation_command(merged_params()),
        sensors_command(),
        mapping_command(os.path.join(REPO, "maps", "yard")),
        missing_command(),
        health_command(),
        relay_command(),
    ]
