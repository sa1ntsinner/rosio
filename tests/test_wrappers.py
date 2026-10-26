"""The known wrapper table: shape, citations and the two name rules."""

from __future__ import annotations

import pytest

from rosio.errors import ContractError
from rosio.resolve import wrappers


def test_every_row_has_a_citation_and_a_known_language():
    for row in wrappers.rows():
        assert row.evidence, f"{row.symbol} has no citation"
        assert row.language in wrappers.LANGUAGES
        assert row.endpoints


def test_rows_are_sorted_and_unique():
    names = [row.symbol for row in wrappers.rows()]
    assert names == sorted(names)
    assert len(names) == len(set(names))


def test_the_blind_spots_the_reference_stack_has_are_all_covered():
    for symbol in (
        "diagnostic_updater.Updater",
        "diagnostic_updater::Updater",
        "tf2_ros.TransformListener",
        "tf2_ros::TransformListener",
        "tf2_ros.TransformBroadcaster",
        "message_filters.Subscriber",
        "message_filters::Subscriber",
        "rclpy.action.ActionClient",
    ):
        assert wrappers.is_known(symbol), symbol


def test_diagnostics_is_a_fixed_publisher():
    assert wrappers.endpoints_for("diagnostic_updater.Updater") == (
        ("publisher", "/diagnostics", "diagnostic_msgs/msg/DiagnosticArray"),
    )


def test_the_transform_listener_hides_two_subscriptions():
    found = wrappers.endpoints_for("tf2_ros.TransformListener")
    assert [item[1] for item in found] == ["/tf", "/tf_static"]
    assert {item[0] for item in found} == {"subscriber"}


def test_a_wrapper_that_takes_its_name_from_the_call_states_none():
    row = wrappers.lookup("message_filters.Subscriber")
    assert row.takes_name
    assert wrappers.endpoints_for("message_filters.Subscriber") == ()
    assert wrappers.endpoints_for("message_filters.Subscriber", "/lidar_left/points") == (
        ("subscriber", "/lidar_left/points", ""),
    )


def test_an_action_name_expands_into_three_services_and_two_topics():
    found = wrappers.endpoints_for("rclpy.action.ActionClient", "/navigate_to_pose")
    names = sorted(item[1] for item in found)
    assert names == [
        "/navigate_to_pose/_action/cancel_goal",
        "/navigate_to_pose/_action/feedback",
        "/navigate_to_pose/_action/get_result",
        "/navigate_to_pose/_action/send_goal",
        "/navigate_to_pose/_action/status",
    ]
    kinds = {name: kind for kind, name, _type in found}
    assert kinds["/navigate_to_pose/_action/send_goal"] == "service_client"
    assert kinds["/navigate_to_pose/_action/status"] == "subscriber"


def test_the_action_server_serves_what_the_client_calls():
    client = wrappers.endpoints_for("rclpy.action.ActionClient", "/park")
    server = wrappers.endpoints_for("rclpy.action.ActionServer", "/park")
    assert sorted(item[1] for item in client) == sorted(item[1] for item in server)
    assert {item[0] for item in server} == {"service_server", "publisher"}


def test_both_spellings_of_one_wrapper_agree():
    for bare in ("TransformListener", "Updater", "Subscriber"):
        assert wrappers.lookup(bare) is not None


def test_a_bare_name_two_different_wrappers_share_is_refused():
    assert wrappers.lookup("nonexistent.Thing") is None
    assert wrappers.endpoints_for("nonexistent.Thing", "/x") == ()


def test_implicit_rows_are_never_materialised():
    row = wrappers.lookup("rclpy.node.Node")
    assert row.implicit
    assert wrappers.endpoints_for("rclpy.node.Node") == ()
    implicit = wrappers.implicit_endpoints("python")
    assert ("publisher", "/rosout", "rcl_interfaces/msg/Log") in implicit
    assert ("service_server", "~/set_parameters", "") in implicit


def test_the_base_class_publishes_parameter_events_and_does_not_subscribe_it():
    for language in wrappers.LANGUAGES:
        implicit = wrappers.implicit_endpoints(language)
        kinds = {kind for kind, name, _type in implicit if name == "/parameter_events"}
        assert kinds == {"publisher"}, language


def test_a_row_that_states_no_endpoint_is_refused():
    with pytest.raises(ContractError):
        wrappers.Wrapper(symbol="x.Y", library="x", language="python", endpoints=(), evidence="e")


def test_a_fixed_endpoint_needs_its_name():
    with pytest.raises(ContractError):
        wrappers.WrapperEndpoint("publisher")
    with pytest.raises(ContractError):
        wrappers.WrapperEndpoint("publisher", "/tf", name_source="argument")


def test_an_unknown_endpoint_kind_is_refused():
    with pytest.raises(ContractError):
        wrappers.WrapperEndpoint("broadcaster", "/tf")


# ---------------------------------------------------------------------------
# first party wrappers


def _candidate(symbol, constructs=(), takes_node=True, **kw):
    return wrappers.Candidate(
        symbol=symbol, constructs=tuple(constructs), takes_node=takes_node, **kw
    )


def test_a_class_that_wraps_an_updater_and_takes_a_node_is_a_wrapper():
    """The reference stack reaches twelve /diagnostics publishers through exactly this."""
    rows = wrappers.discover([_candidate("p.diagnostics.Reporter", ["diagnostic_updater.Updater"])])
    assert [row.symbol for row in rows] == ["p.diagnostics.Reporter"]
    row = rows[0]
    assert row.discovered
    assert row.via == ("diagnostic_updater.Updater",)
    assert row.library == "diagnostic_updater"
    assert row.resolved() == (("publisher", "/diagnostics", "diagnostic_msgs/msg/DiagnosticArray"),)


def test_a_class_that_takes_no_node_is_not_a_wrapper():
    assert (
        wrappers.discover(
            [_candidate("p.d.Reporter", ["diagnostic_updater.Updater"], takes_node=False)]
        )
        == ()
    )


def test_a_class_that_constructs_nothing_known_is_not_a_wrapper():
    unrelated = _candidate("p.d.Holder", ["p.other.Thing", "collections.Counter"])
    assert wrappers.discover([unrelated]) == ()


def test_a_wrapper_whose_library_takes_the_name_from_the_call_is_not_inherited():
    """message_filters names nothing of its own, so the wrapper would have to invent one."""
    assert wrappers.discover([_candidate("p.d.Sync", ["message_filters.Subscriber"])]) == ()


def test_the_base_class_alone_promotes_nothing():
    assert wrappers.discover([_candidate("p.d.Holder", ["rclpy.node.Node"])]) == ()


def test_a_wrapper_over_a_wrapper_resolves_whatever_order_it_is_offered_in():
    outer = _candidate("p.a.Outer", ["p.b.Inner"])
    inner = _candidate("p.b.Inner", ["tf2_ros.TransformBroadcaster"])
    forward = wrappers.discover([inner, outer])
    backward = wrappers.discover([outer, inner])
    assert [row.symbol for row in forward] == ["p.a.Outer", "p.b.Inner"]
    assert [row.symbol for row in backward] == [row.symbol for row in forward]
    assert forward[0].resolved() == (("publisher", "/tf", "tf2_msgs/msg/TFMessage"),)


def test_discovery_is_a_pure_function_of_its_input():
    rows = [
        _candidate("p.b.Inner", ["tf2_ros.TransformBroadcaster"]),
        _candidate("p.a.Outer", ["p.b.Inner", "diagnostic_updater.Updater"]),
    ]
    first = wrappers.discover(rows)
    second = wrappers.discover(list(reversed(rows)))
    assert [(row.symbol, row.resolved()) for row in first] == [
        (row.symbol, row.resolved()) for row in second
    ]
    assert first[0].library == "first_party"


def test_two_classes_that_construct_each_other_terminate():
    """The fixpoint has to stop on a cycle, and give the same answer either way round."""
    rows = [
        _candidate("p.a.A", ["p.b.B", "diagnostic_updater.Updater"]),
        _candidate("p.b.B", ["p.a.A"]),
    ]
    forward = wrappers.discover(rows)
    backward = wrappers.discover(list(reversed(rows)))
    assert [(row.symbol, row.resolved()) for row in forward] == [
        (row.symbol, row.resolved()) for row in backward
    ]
    assert [row.symbol for row in forward] == ["p.a.A", "p.b.B"]


def test_a_discovered_row_is_never_written_into_the_committed_table():
    before = wrappers.symbols()
    wrappers.discover([_candidate("p.d.Reporter", ["diagnostic_updater.Updater"])])
    assert wrappers.symbols() == before
    assert wrappers.lookup("p.d.Reporter") is None


def test_every_discovered_row_carries_a_citation():
    rows = wrappers.discover(
        [_candidate("p.d.Reporter", ["diagnostic_updater.Updater"], file="p/d.py")]
    )
    assert "p/d.py" in rows[0].evidence
    assert "diagnostic_updater.Updater" in rows[0].evidence


def test_the_index_is_keyed_by_symbol():
    rows = wrappers.discover([_candidate("p.d.Reporter", ["diagnostic_updater.Updater"])])
    assert wrappers.index(rows) == {"p.d.Reporter": rows[0]}


def test_a_candidate_needs_a_symbol_and_a_known_language():
    with pytest.raises(ContractError):
        wrappers.Candidate(symbol="")
    with pytest.raises(ContractError):
        wrappers.Candidate(symbol="p.X", language="rust")
