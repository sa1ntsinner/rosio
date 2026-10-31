"""Cases from the ninth first run trial and the review that followed the eighth."""

from __future__ import annotations

from pathlib import Path

from rosio import ids
from tests.test_eighth_trial import LOOKUP, STATIC, findings, node, python_workspace

UNREAD = 'Node(package="robot_localization", executable="ekf_node", name="ekf")'


# ---------------------------------------------------------------------------
# a node rosio could not read may broadcast the frame a lookup wants


def test_a_frame_an_unread_node_could_broadcast_has_no_publisher_claim(tmp_path: Path):
    atlas = python_workspace(
        tmp_path,
        {"planner": node("planner", LOOKUP.format("base_footprint"))},
        STATIC.format("base_to_laser", "base_footprint", "laser"),
        UNREAD,
    )
    assert findings(atlas, "frame.no_publisher") == []


def test_a_frame_nothing_could_broadcast_still_has_no_publisher(tmp_path: Path):
    atlas = python_workspace(
        tmp_path,
        {"planner": node("planner", LOOKUP.format("base_footprint"))},
        STATIC.format("base_to_laser", "base_footprint", "laser"),
    )
    found = findings(atlas, "frame.no_publisher")
    assert [item.subject for item in found] == [ids.build_frame("base_footprint")]
