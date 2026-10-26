"""The one deep merge rule and its trace."""

from __future__ import annotations

import yaml

from rosio.resolve import merge

BASE = {
    "controller_server": {
        "ros__parameters": {
            "controller_frequency": 20.0,
            "controller_plugins": ["FollowPath"],
            "FollowPath": {
                "plugin": "lx_bringup/PathFollower",
                "desired_linear_vel": 1.2,
                "lookahead_dist": 2.4,
            },
        }
    },
    "local_costmap": {"local_costmap": {"ros__parameters": {"width": 20, "height": 20}}},
}

OVERLAY = {
    "controller_server": {
        "ros__parameters": {
            "controller_frequency": 20.0,
            "FollowPath": {
                "plugin": "lx_bringup/OrbitFollower",
                "lookahead_dist": 2.4,
                "state_timeout": 0.4,
            },
        }
    },
    "local_costmap": {"local_costmap": {"ros__parameters": {"width": 40}}},
}


def test_overlay_wins_per_leaf():
    merged = merge.deep_merge(BASE, OVERLAY)
    params = merged["controller_server"]["ros__parameters"]
    assert params["FollowPath"]["plugin"] == "lx_bringup/OrbitFollower"
    assert params["FollowPath"]["desired_linear_vel"] == 1.2
    assert merged["local_costmap"]["local_costmap"]["ros__parameters"]["width"] == 40
    assert merged["local_costmap"]["local_costmap"]["ros__parameters"]["height"] == 20


def test_inputs_are_not_mutated():
    before = yaml.safe_dump(BASE, sort_keys=True)
    merge.deep_merge(BASE, OVERLAY)
    assert yaml.safe_dump(BASE, sort_keys=True) == before


def test_a_list_replaces_wholesale():
    merged = merge.deep_merge({"plugins": ["a", "b"]}, {"plugins": ["c"]})
    assert merged["plugins"] == ["c"]


def test_a_scalar_replaces_a_mapping_and_the_trace_says_so():
    events = merge.events_by_name(merge.merge_trace({"a": {"b": 1}}, {"a": 2}))
    assert events["a"].kind == merge.TYPE_CHANGED
    assert merge.deep_merge({"a": {"b": 1}}, {"a": 2}) == {"a": 2}


def test_a_mapping_replacing_a_scalar_is_one_event():
    events = merge.merge_trace({"a": 1}, {"a": {"b": 2}})
    assert [event.name for event in events] == ["a"]
    assert events[0].kind == merge.TYPE_CHANGED


def test_trace_classifies_every_leaf():
    events = merge.events_by_name(merge.merge_trace(BASE, OVERLAY))
    prefix = "controller_server.ros__parameters"
    assert events[f"{prefix}.controller_frequency"].kind == merge.RESTATED
    assert events[f"{prefix}.FollowPath.plugin"].kind == merge.CHANGED
    assert events[f"{prefix}.FollowPath.lookahead_dist"].kind == merge.RESTATED
    assert events[f"{prefix}.FollowPath.state_timeout"].kind == merge.INTRODUCED
    assert events["local_costmap.local_costmap.ros__parameters.width"].kind == merge.CHANGED
    assert f"{prefix}.FollowPath.desired_linear_vel" not in events


def test_trace_keeps_both_values_and_the_base_presence():
    event = merge.events_by_name(merge.merge_trace(BASE, OVERLAY))[
        "controller_server.ros__parameters.FollowPath.plugin"
    ]
    assert event.base_value == "lx_bringup/PathFollower"
    assert event.overlay_value == "lx_bringup/OrbitFollower"
    assert event.base_present is True
    assert (
        merge.events_by_name(merge.merge_trace(BASE, OVERLAY))[
            "controller_server.ros__parameters.FollowPath.state_timeout"
        ].base_present
        is False
    )


def test_an_int_becoming_a_float_is_a_type_change_not_a_value_change():
    events = merge.events_by_name(merge.merge_trace({"width": 20}, {"width": 20.0}))
    assert events["width"].kind == merge.TYPE_CHANGED


def test_a_bool_is_not_an_int():
    events = merge.events_by_name(merge.merge_trace({"flag": 1}, {"flag": True}))
    assert events["flag"].kind == merge.TYPE_CHANGED


def test_an_introduced_subtree_yields_one_event_per_leaf():
    events = merge.merge_trace({}, {"group": {"a": 1, "b": {"c": 2}}})
    assert [(event.name, event.kind) for event in events] == [
        ("group.a", merge.INTRODUCED),
        ("group.b.c", merge.INTRODUCED),
    ]


def test_trace_is_sorted_and_stable():
    left = merge.merge_trace(BASE, OVERLAY)
    right = merge.merge_trace(BASE, OVERLAY)
    assert [event.sort_key() for event in left] == sorted(event.sort_key() for event in left)
    assert left == right


def test_an_empty_overlay_mapping_is_no_event_because_it_is_no_change():
    base = {"a": {"b": 1}}
    assert merge.deep_merge(base, {"a": {}}) == base
    assert merge.merge_trace(base, {"a": {}}) == []
    replaced = merge.merge_trace({"a": 1}, {"a": {}})
    assert [(event.name, event.kind) for event in replaced] == [("a", merge.TYPE_CHANGED)]
    assert merge.deep_merge({"a": 1}, {"a": {}}) == {"a": {}}


def test_empty_inputs_are_not_an_error():
    assert merge.deep_merge(None, None) == {}
    assert merge.merge_trace(None, {"a": 1})[0].kind == merge.INTRODUCED
    assert merge.merge_trace({"a": 1}, None) == []


def test_the_reference_merge_helpers_agree_with_this_one():
    """The reference repo holds four copies of this merge. Reproduce one and compare."""

    def reference(base: dict, overlay: dict) -> dict:
        for key, value in overlay.items():
            if isinstance(value, dict) and isinstance(base.get(key), dict):
                reference(base[key], value)
            else:
                base[key] = value
        return base

    expected = reference(yaml.safe_load(yaml.safe_dump(BASE)), OVERLAY)
    assert merge.deep_merge(BASE, OVERLAY) == expected
