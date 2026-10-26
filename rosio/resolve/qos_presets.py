"""What a QoS written as a preset or as a bare depth means, from the ROS 2 client libraries.

A source that says `rclcpp::SensorDataQoS()` or `create_publisher(Twist, "cmd_vel", 10)` states a
QoS as surely as one that spells every policy: the client library defines what those forms are.
Collectors record the form as written; this table is how a rule reads the policies the form
stands for. Only the two policies that decide whether a publisher and a subscriber connect are
filled in, reliability and durability, and `system_default` is left unknown on purpose, because it
means whatever the middleware chooses.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

PRESETS: dict[str, dict[str, str]] = {
    "sensor_data": {"reliability": "best_effort", "durability": "volatile"},
    "parameters": {"reliability": "reliable", "durability": "volatile"},
    "services": {"reliability": "reliable", "durability": "volatile"},
    "services_default": {"reliability": "reliable", "durability": "volatile"},
    "parameter_events": {"reliability": "reliable", "durability": "volatile"},
    "clock": {"reliability": "best_effort", "durability": "volatile"},
    "rosout": {"reliability": "reliable", "durability": "transient_local"},
    "action_status_default": {"reliability": "reliable", "durability": "transient_local"},
}
"""`rmw/qos_profiles.h` and the rclcpp and rclpy presets built on it."""

DEPTH_DEFAULT = {"reliability": "reliable", "durability": "volatile"}
"""A bare history depth, `QoS(10)` or `10`, is the default profile at that depth."""

CONNECTING_POLICIES = ("reliability", "durability")


def policies(qos: Mapping[str, Any] | None) -> dict[str, str]:
    """The reliability and durability a recorded QoS states or its form implies.

    A policy the record states wins over what its form implies, so `SensorDataQoS().reliable()`
    reads as reliable. A form this table does not know states nothing beyond what it spells.
    """
    if not isinstance(qos, Mapping):
        return {}
    implied: dict[str, str] = {}
    preset = qos.get("preset")
    if isinstance(preset, str) and preset in PRESETS:
        implied = dict(PRESETS[preset])
    elif qos.get("form") == "depth":
        implied = dict(DEPTH_DEFAULT)
    for key in CONNECTING_POLICIES:
        value = qos.get(key)
        if isinstance(value, str) and value:
            implied[key] = value
    return {key: implied[key] for key in CONNECTING_POLICIES if key in implied}


def incompatible(offered: Mapping[str, str], requested: Mapping[str, str]) -> list[str]:
    """Why a publisher offering one QoS and a subscriber requesting another never connect.

    The two request versus offer rules of DDS and Zenoh alike: a reliable subscription needs a
    reliable publisher, and a transient local subscription needs a transient local publisher.
    A policy either side leaves unknown decides nothing.
    """
    reasons = []
    if requested.get("reliability") == "reliable" and offered.get("reliability") == "best_effort":
        reasons.append("reliability")
    if requested.get("durability") == "transient_local" and offered.get("durability") == "volatile":
        reasons.append("durability")
    return reasons
