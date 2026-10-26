"""The atlas model: contract enforcement, joins, determinism, schema validation."""

import ast
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from rosio import ids, model, schema
from rosio.errors import ContractError
from rosio.model import (
    ANY_PROFILE,
    Agreement,
    Atlas,
    AtlasBuilder,
    Candidate,
    Confidence,
    EdgeRef,
    Finding,
    Layer,
    Mechanism,
    NameSource,
    ParamState,
    Severity,
    ShadowKind,
    Source,
    Unresolved,
    Want,
)

REPO_ROOT = Path(__file__).resolve().parent.parent

PKG_OSF = ids.build_pkg("self_filter")
PKG_BRG = ids.build_pkg("nx_bringup")
FILE_OSF = ids.build_file("self_filter", "src/self_filter_node.cpp")
FILE_LAUNCH = ids.build_file("nx_bringup", "launch/sensors.launch.py")
FILE_BASE = ids.build_file("nx_bringup", "config/nav2_params.yaml")
FILE_OVERLAY = ids.build_file("nx_bringup", "config/nav2_overrides.yaml")
FILE_LAUNCHER = ids.build_file(ids.REPO_PKG, "dx_app/dx_launcher/profiles.py")
FILE_BT = ids.build_file("nx_bringup", "src/motion_gate_bt_node.cpp")
SYM_CTOR = ids.build_sym(
    "self_filter",
    "src/self_filter_node.cpp",
    "self_filter::SelfFilterNode::SelfFilterNode",
)
SYM_MERGE = ids.build_sym(ids.REPO_PKG, "dx_app/dx_launcher/profiles.py", "_merge_overlay")
SYM_BT = ids.build_sym(
    "nx_bringup",
    "src/motion_gate_bt_node.cpp",
    "nx_bringup::MotionGate::MotionGate",
)
LAUNCHFILE = ids.build_launchfile("nx_bringup", "launch/sensors.launch.py")
LAUNCHENT = ids.build_launchent(
    "nx_bringup",
    "launch/sensors.launch.py",
    (ids.LaunchStep("OnProcessExit", "ready_gate"), ids.LaunchStep("Node", "self_filter")),
)
NODE_OSF = ids.build_node("/self_filter")
TOPIC = ids.build_topic("/ouster/points")
TYPE_PC2 = ids.build_type("sensor_msgs", "msg", "PointCloud2")
ROLE = ids.role_digest(package="nav2_controller", executable="controller_server", remaps=[])
NODE_CTL = ids.build_node_synthetic("nav2_controller", "controller_server", "/", ROLE)
PARAM = ids.build_param(NODE_CTL, "FollowPath.plugin")
PROFILE = ids.build_profile("nx_navigation")


def tiny_atlas() -> Atlas:
    """A small but real slice of the reference stack.

    It carries the two cases the whole design exists for: a relative C++ subscription
    that only a launch remap can resolve, and a parameter whose winning value comes
    from an overlay merged outside the launch system.
    """
    builder = AtlasBuilder(
        tool_version="0.1.0",
        capabilities=["layer.coded", "layer.declared", "layer.observed", "param.candidates"],
        generated={
            "workspace": "/srv/nx_ws",
            "wall_clock_utc": "2026-08-14T09:12:03Z",
        },
    )
    for pkg in (PKG_OSF, PKG_BRG):
        builder.node(pkg)
    for file_id in (FILE_OSF, FILE_LAUNCH, FILE_BASE, FILE_OVERLAY, FILE_LAUNCHER, FILE_BT):
        builder.node(file_id)
    for sym in (SYM_CTOR, SYM_MERGE, SYM_BT):
        builder.node(sym)
    builder.node(LAUNCHFILE)
    builder.node(LAUNCHENT)
    builder.node(TYPE_PC2)

    builder.add_profile(
        "nx_navigation",
        label="Navigation (real robot)",
        entry=[SYM_MERGE],
        entry_kind="launcher_command",
        inputs={
            "launch_args": {"autostart": "true", "namespace": ""},
            "path_probes": {"maps/site_map_test.posegraph": False},
            "param_merge_sites": [SYM_MERGE],
        },
    )
    run_declared = builder.add_run(
        "declared",
        profile=PROFILE,
        input_digest="9f2c1a4b",
        roots={"src": "sha256:9f2c11ab4d70"},
        env={"NX_WS_ROOT": "<repo>"},
        predicates={"maps/site_map_test.posegraph": False},
        volatile={"duration_s": 4.1},
    )
    run_coded = builder.add_run("coded", profile=PROFILE, input_digest="0e77d310")
    run_observed = builder.add_run("observed", profile=PROFILE, input_digest="c41b9a02")
    builder.declare_layer("declared", present=True, run=run_declared)
    builder.declare_layer("coded", present=True, run=run_coded)
    builder.declare_layer("observed", present=True, run=run_observed, root="runtime")
    builder.declare_layer("recorded", present=False, reason="no bag given")

    builder.edge("contains", PKG_OSF, FILE_OSF)
    builder.edge("contains", PKG_BRG, FILE_LAUNCH)
    builder.edge("contains", PKG_BRG, LAUNCHFILE)
    builder.edge("contains", LAUNCHFILE, LAUNCHENT)
    builder.edge("defines", FILE_OSF, SYM_CTOR)

    builder.node(NODE_OSF, name_source=NameSource.LITERAL.value)
    builder.fact(
        NODE_OSF,
        Layer.CODED.value,
        profile=PROFILE,
        run=run_coded,
        values={"namespace": "/", "name": "self_filter"},
        evidence=[Source(file=FILE_OSF, locator="SelfFilterNode", line_hint=23)],
    )
    builder.edge("defines", SYM_CTOR, NODE_OSF)
    builder.edge(
        "launches",
        LAUNCHENT,
        NODE_OSF,
        key=ids.edge_key(site=LAUNCHENT, profile=PROFILE),
        layer=Layer.DECLARED.value,
        profile=PROFILE,
        run=run_declared,
        values={"respawn": False, "condition": None},
        evidence=[Source(file=FILE_LAUNCH, locator="Node(self_filter)", line_hint=250)],
    )
    builder.fact(
        TOPIC,
        Layer.DECLARED.value,
        profile=PROFILE,
        run=run_declared,
        values={"via": ["remap"]},
        evidence=[Source(file=FILE_LAUNCH, locator="remappings[0]", line_hint=256)],
    )
    builder.fact(
        TOPIC,
        Layer.OBSERVED.value,
        profile=PROFILE,
        run=run_observed,
        values={"pub_count": 1, "sub_count": 2, "volatile": {"rate_hz": 9.7}},
        confidence=Confidence.CERTAIN.value,
    )
    builder.edge(
        "remaps",
        LAUNCHENT,
        TOPIC,
        key=ids.edge_key(site=LAUNCHENT, from_name="input"),
        layer=Layer.DECLARED.value,
        profile=PROFILE,
        run=run_declared,
        values={"from_name": "input", "source": "launch"},
        evidence=[Source(file=FILE_LAUNCH, locator="remappings[0]", line_hint=256)],
    )
    builder.edge("uses_type", TOPIC, TYPE_PC2)

    endpoint_key = ids.edge_key(site=LAUNCHENT, profile=PROFILE, role="subscribes")
    builder.edge(
        "subscribes",
        NODE_OSF,
        TOPIC,
        key=endpoint_key,
        layer=Layer.CODED.value,
        profile=PROFILE,
        run=run_coded,
        values={
            "raw_name": "input",
            "relative": True,
            "resolved_via": "remaps",
            "type": TYPE_PC2,
            "qos": {"reliability": "reliable", "durability": "volatile", "depth": 5},
        },
        evidence=[Source(file=FILE_OSF, locator="SelfFilterNode", line_hint=122)],
    )
    builder.edge(
        "subscribes",
        NODE_OSF,
        TOPIC,
        key=endpoint_key,
        layer=Layer.OBSERVED.value,
        profile=PROFILE,
        run=run_observed,
        values={
            "type": TYPE_PC2,
            "qos": {"reliability": "best_effort", "durability": "volatile", "depth": 5},
            "volatile": {"rate_hz": 9.7, "samples": 97},
        },
        confidence=Confidence.CERTAIN.value,
    )

    builder.node(NODE_CTL, name_source=NameSource.UNKNOWN.value)
    builder.set_param(
        PARAM,
        state=ParamState.EFFECTIVE.value,
        candidates=[
            Candidate(
                value="nx_controller/DelayAwareController",
                value_type="string",
                mechanism=Mechanism.YAML_OVERLAY_MERGE.value,
                source=Source(
                    file=FILE_OVERLAY,
                    locator="controller_server.ros__parameters.FollowPath.plugin",
                    line_hint=12,
                ),
                rank=0,
                wins=True,
                profile=PROFILE,
                applied_by=SYM_MERGE,
            ),
            Candidate(
                value="nx_planner/DockingController",
                value_type="string",
                mechanism=Mechanism.YAML_BASE.value,
                source=Source(
                    file=FILE_BASE,
                    locator="controller_server.ros__parameters.FollowPath.plugin",
                    line_hint=149,
                ),
                rank=1,
                profile=PROFILE,
                shadow_kind=ShadowKind.VALUE_CHANGED.value,
                shadowed_by=0,
            ),
        ],
        note="the plugin swap leaves the base FollowPath keys addressed to a controller "
        "that does not declare them",
    )
    builder.edge("has_param", NODE_CTL, PARAM)
    builder.edge(
        "sets_param",
        FILE_OVERLAY,
        PARAM,
        key=ids.edge_key(mechanism="yaml_overlay_merge", profile=PROFILE),
        layer=Layer.DECLARED.value,
        profile=PROFILE,
        run=run_declared,
        values={"rank": 0, "wins": True},
    )
    builder.edge(
        "sets_param",
        FILE_BASE,
        PARAM,
        key=ids.edge_key(mechanism="yaml_base", profile=PROFILE),
        layer=Layer.DECLARED.value,
        profile=PROFILE,
        run=run_declared,
        values={"rank": 1, "wins": False},
    )

    builder.count_resolved(Want.TOPIC_NAME.value, 44)
    builder.unresolved(
        Unresolved(
            want=Want.TOPIC_NAME.value,
            owner=SYM_BT,
            expr='getInput("confirmation_topic")',
            reason="bt_blackboard_port",
            role="subscribes",
            resolvable_by=["behavior_trees/*.xml"],
            evidence=[Source(file=FILE_BT, locator="MotionGate::MotionGate", line_hint=31)],
        )
    )
    builder.finding(
        Finding(
            rule="param.shadowed",
            severity=Severity.INFO.value,
            subject=PARAM,
            title="the launcher overlay wins over the base parameter file",
            detail="nav2_overrides.yaml is merged over nav2_params.yaml outside the "
            "launch system, by the desktop launcher.",
            fix="edit nav2_overrides.yaml, editing nav2_params.yaml has no effect here",
            say=(
                "FollowPath.plugin on controller_server is "
                "nx_controller/DelayAwareController from "
                "nav2_overrides.yaml; nav2_params.yaml sets "
                "nx_planner/DockingController and loses to the launcher merge."
            ),
            discriminator="nav2_params",
            profiles=[PROFILE],
            layers=[Layer.DECLARED.value],
            where=[
                Source(file=FILE_OVERLAY, locator="FollowPath.plugin", line_hint=12),
                Source(file=FILE_BASE, locator="FollowPath.plugin", line_hint=149),
            ],
        )
    )
    builder.finding(
        Finding(
            rule="qos.differ",
            severity=Severity.WARN.value,
            subject=EdgeRef("subscribes", NODE_OSF, TOPIC, endpoint_key),
            title="the running subscription is best_effort, the source asks for reliable",
            detail="self_filter_node.cpp builds the profile in a local variable.",
            fix="state the reliability at the call site or accept best_effort",
            say=(
                "/self_filter subscribes /ouster/points as best_effort while "
                "self_filter_node.cpp asks for reliable."
            ),
            profiles=[PROFILE],
            layers=[Layer.CODED.value, Layer.OBSERVED.value],
            where=[Source(file=FILE_OSF, locator="SelfFilterNode", line_hint=120)],
        )
    )
    return builder.build()


def minimal_builder() -> AtlasBuilder:
    builder = AtlasBuilder()
    builder.node(PKG_OSF)
    builder.node(FILE_OSF)
    builder.node(NODE_OSF, name_source=NameSource.LITERAL.value)
    builder.node(TOPIC)
    return builder


# ---------------------------------------------------------------------------
# contract


def test_a_fact_creates_the_entity_with_the_kind_from_its_id():
    builder = AtlasBuilder()
    builder.fact(TOPIC, Layer.CODED.value, values={"type": TYPE_PC2})
    node = builder.atlas.entities[TOPIC]
    assert node.kind == "topic"
    assert node.label == "/ouster/points"
    assert node.layers == ["coded"]


def test_asking_for_a_conflicting_kind_raises():
    builder = AtlasBuilder()
    builder.node(TOPIC)
    with pytest.raises(ContractError):
        builder.node(TOPIC, kind="param")


def test_only_entity_kinds_can_be_entities():
    builder = AtlasBuilder()
    with pytest.raises(ContractError):
        builder.node("find:param.shadowed|param:/x|y|-")


def test_edges_must_reference_existing_entities():
    builder = minimal_builder()
    with pytest.raises(ContractError) as exc:
        builder.edge("publishes", NODE_OSF, ids.build_topic("/nowhere"), key="k")
    assert "not in the atlas" in str(exc.value)


def test_edge_domains_are_closed():
    builder = minimal_builder()
    with pytest.raises(ContractError) as exc:
        builder.edge("publishes", TOPIC, NODE_OSF, key="k")
    assert "not allowed" in str(exc.value)


def test_repeatable_edge_kinds_need_a_key():
    builder = minimal_builder()
    with pytest.raises(ContractError) as exc:
        builder.edge("subscribes", NODE_OSF, TOPIC)
    assert "needs a key" in str(exc.value)


def test_two_sites_stay_two_edges_and_one_site_stays_one():
    builder = minimal_builder()
    first = builder.edge("subscribes", NODE_OSF, TOPIC, key="site_a", layer="coded")
    second = builder.edge("subscribes", NODE_OSF, TOPIC, key="site_b", layer="coded")
    again = builder.edge("subscribes", NODE_OSF, TOPIC, key="site_a", layer="observed")
    assert first is not second
    assert again is first
    assert len(builder.atlas.edges) == 2
    assert first.layers == ["coded", "observed"]


def test_a_second_fragment_under_one_key_is_two_sites_and_is_refused():
    """The reason a key exists. Two publication sites under one key merged silently, so one
    edge carried half of each and the atlas reported one site where the workspace has two."""
    builder = minimal_builder()
    builder.edge(
        "subscribes", NODE_OSF, TOPIC, key="k", layer="coded", values={"raw_name": "input"}
    )
    with pytest.raises(ContractError) as exc:
        builder.edge(
            "subscribes", NODE_OSF, TOPIC, key="k", layer="coded", values={"relative": True}
        )
    assert "two producing sites share one key" in str(exc.value)
    assert "values.relative" in str(exc.value)
    edge = builder.atlas.edges[ids.build_edge("subscribes", NODE_OSF, TOPIC, "k")]
    assert edge.fact("coded").values == {"raw_name": "input"}


def test_a_changed_value_under_one_key_is_refused_too():
    builder = minimal_builder()
    builder.edge(
        "subscribes", NODE_OSF, TOPIC, key="k", layer="coded", values={"raw_name": "input"}
    )
    with pytest.raises(ContractError) as exc:
        builder.edge(
            "subscribes", NODE_OSF, TOPIC, key="k", layer="coded", values={"raw_name": "output"}
        )
    assert "two producing sites share one key" in str(exc.value)


def test_restating_one_keyed_site_is_allowed_so_a_rerun_is_a_fixpoint():
    builder = minimal_builder()
    site = Source(file=FILE_OSF, locator="self_filter.py", line_hint=40)
    for _ in range(2):
        builder.edge(
            "subscribes",
            NODE_OSF,
            TOPIC,
            key="k",
            layer="coded",
            values={"raw_name": "input"},
            evidence=[site],
        )
    edge = builder.atlas.edges[ids.build_edge("subscribes", NODE_OSF, TOPIC, "k")]
    assert edge.fact("coded").values == {"raw_name": "input"}
    assert edge.fact("coded").evidence == [site]


def test_a_second_evidence_row_under_one_key_is_refused():
    builder = minimal_builder()
    builder.edge(
        "subscribes",
        NODE_OSF,
        TOPIC,
        key="k",
        layer="coded",
        evidence=[Source(file=FILE_OSF, locator="self_filter.py", line_hint=40)],
    )
    with pytest.raises(ContractError) as exc:
        builder.edge(
            "subscribes",
            NODE_OSF,
            TOPIC,
            key="k",
            layer="coded",
            evidence=[Source(file=FILE_OSF, locator="self_filter.py", line_hint=91)],
        )
    assert "two producing sites share one key" in str(exc.value)


def test_an_unkeyed_edge_still_takes_a_second_fragment_and_never_an_overwrite():
    """Only a keyed kind names a site. has_param repeats legitimately, so it still merges,
    and a changed value is still refused there."""
    builder = minimal_builder()
    builder.node(NODE_CTL, name_source=NameSource.UNKNOWN.value)
    builder.set_param(PARAM, state="undeclared")
    builder.edge("has_param", NODE_CTL, PARAM, layer="coded", values={"declared": True})
    builder.edge("has_param", NODE_CTL, PARAM, layer="coded", values={"read": True})
    edge = builder.atlas.edges[ids.build_edge("has_param", NODE_CTL, PARAM, "")]
    assert edge.fact("coded").values == {"declared": True, "read": True}
    with pytest.raises(ContractError) as exc:
        builder.edge("has_param", NODE_CTL, PARAM, layer="coded", values={"read": False})
    assert "refusing to overwrite" in str(exc.value)


def test_edge_values_without_a_layer_are_refused():
    builder = minimal_builder()
    with pytest.raises(ContractError):
        builder.edge("subscribes", NODE_OSF, TOPIC, key="k", values={"raw_name": "input"})


def test_a_parameter_always_carries_a_state():
    builder = minimal_builder()
    builder.node(NODE_CTL, name_source=NameSource.UNKNOWN.value)
    builder.fact(PARAM, Layer.DECLARED.value, values={"seen": True})
    with pytest.raises(ContractError) as exc:
        builder.build()
    assert "has no state" in str(exc.value)


def test_a_non_effective_parameter_has_no_effective_value():
    builder = minimal_builder()
    builder.node(NODE_CTL, name_source=NameSource.UNKNOWN.value)
    builder.set_param(
        PARAM,
        state=ParamState.OWNER_NAME_MISMATCH.value,
        candidates=[
            Candidate(
                value="x",
                value_type="string",
                mechanism=Mechanism.YAML_BASE.value,
                source=Source(file=FILE_BASE),
                rank=0,
                wins=True,
            )
        ],
    )
    atlas = builder.build()
    assert atlas.entities[PARAM].effective == {}


def test_two_winning_candidates_in_one_profile_are_refused():
    builder = minimal_builder()
    builder.node(NODE_CTL, name_source=NameSource.UNKNOWN.value)
    rows = [
        Candidate(
            value="a",
            value_type="string",
            mechanism=Mechanism.YAML_BASE.value,
            source=Source(file=FILE_BASE),
            rank=0,
            wins=True,
            profile=PROFILE,
        ),
        Candidate(
            value="b",
            value_type="string",
            mechanism=Mechanism.YAML_OVERLAY_MERGE.value,
            source=Source(file=FILE_OVERLAY),
            rank=1,
            wins=True,
            profile=PROFILE,
        ),
    ]
    with pytest.raises(ContractError):
        builder.set_param(PARAM, state=ParamState.EFFECTIVE.value, candidates=rows)


def test_two_candidates_at_one_rank_are_refused():
    builder = minimal_builder()
    builder.node(NODE_CTL, name_source=NameSource.UNKNOWN.value)
    rows = [
        Candidate(
            value="a",
            value_type="string",
            mechanism=Mechanism.YAML_BASE.value,
            source=Source(file=FILE_BASE),
            rank=0,
            profile=PROFILE,
        ),
        Candidate(
            value="b",
            value_type="string",
            mechanism=Mechanism.YAML_BASE.value,
            source=Source(file=FILE_OVERLAY),
            rank=0,
            profile=PROFILE,
        ),
    ]
    with pytest.raises(ContractError):
        builder.set_param(PARAM, state=ParamState.EFFECTIVE.value, candidates=rows)


def test_unknown_enum_members_are_refused_at_build_time():
    builder = minimal_builder()
    with pytest.raises(ContractError):
        builder.fact(TOPIC, "guessed")
    with pytest.raises(ContractError):
        builder.edge("teleports", NODE_OSF, TOPIC, key="k")


def test_non_finite_numbers_are_refused():
    builder = minimal_builder()
    with pytest.raises(ContractError):
        builder.fact(TOPIC, Layer.OBSERVED.value, values={"rate_hz": float("nan")})


def test_a_finding_may_not_quote_a_synthetic_id():
    builder = minimal_builder()
    builder.node(NODE_CTL, name_source=NameSource.UNKNOWN.value)
    with pytest.raises(ContractError):
        builder.finding(
            Finding(
                rule="node.unnamed",
                severity=Severity.INFO.value,
                subject=NODE_CTL,
                title="unnamed node",
                say=f"the node {NODE_CTL} has no name",
            )
        )


def test_a_finding_must_be_about_a_known_entity():
    builder = minimal_builder()
    with pytest.raises(ContractError):
        builder.finding(
            Finding(
                rule="topic.no_subscriber",
                severity=Severity.INFO.value,
                subject=ids.build_topic("/nowhere"),
                title="no subscriber",
                say="nothing subscribes /nowhere.",
            )
        )


# ---------------------------------------------------------------------------
# joins and derived answers


def test_layers_land_on_one_edge_and_the_conflict_is_named():
    atlas = tiny_atlas()
    handle = ids.build_edge(
        "subscribes",
        NODE_OSF,
        TOPIC,
        ids.edge_key(site=LAUNCHENT, profile=PROFILE, role="subscribes"),
    )
    edge = atlas.edges[handle]
    assert edge.layers == ["coded", "observed"]
    assert edge.agreement == Agreement.DIFFER.value
    assert edge.conflicts == ["qos.reliability"]
    assert handle in atlas.indexes["differ"]


def test_the_running_system_wins_the_effective_endpoint_field():
    atlas = tiny_atlas()
    handle = ids.build_edge(
        "subscribes",
        NODE_OSF,
        TOPIC,
        ids.edge_key(site=LAUNCHENT, profile=PROFILE, role="subscribes"),
    )
    effective = atlas.edges[handle].effective[PROFILE]
    assert effective["qos.reliability"].value == "best_effort"
    assert effective["qos.reliability"].layer == "observed"
    assert effective["raw_name"].layer == "coded"


def test_a_profile_own_fact_wins_over_a_shared_one_in_one_layer():
    """`*` is a fallback: it fills what the profile leaves open and never outbids it."""
    builder = minimal_builder()
    profile = builder.add_profile("demo")
    builder.fact(NODE_OSF, Layer.CODED.value, values={"rate_hz": 10, "frame": "base"})
    builder.fact(NODE_OSF, Layer.CODED.value, profile=profile, values={"rate_hz": 20})
    builder.fact(NODE_OSF, Layer.OBSERVED.value, profile=profile, values={"rate_hz": 20})
    node = builder.build().entities[NODE_OSF]
    assert node.conflicts == []
    coded_only = minimal_builder()
    profile = coded_only.add_profile("demo")
    coded_only.fact(NODE_OSF, Layer.CODED.value, values={"rate_hz": 10, "frame": "base"})
    coded_only.fact(NODE_OSF, Layer.CODED.value, profile=profile, values={"rate_hz": 20})
    node = coded_only.build().entities[NODE_OSF]
    assert node.effective[profile]["rate_hz"].value == 20
    assert node.effective[profile]["frame"].value == "base"


def test_volatile_values_never_reach_conflicts_or_effective():
    atlas = tiny_atlas()
    handle = ids.build_edge(
        "subscribes",
        NODE_OSF,
        TOPIC,
        ids.edge_key(site=LAUNCHENT, profile=PROFILE, role="subscribes"),
    )
    effective = atlas.edges[handle].effective[PROFILE]
    assert not [key for key in effective if key.startswith("volatile")]


def test_two_profiles_may_disagree_without_it_being_drift():
    builder = minimal_builder()
    other = builder.add_profile("nx_sim.alt")
    builder.add_profile("nx_navigation")
    builder.edge(
        "subscribes",
        NODE_OSF,
        TOPIC,
        key="k",
        layer="declared",
        profile=PROFILE,
        values={"qos": {"depth": 5}},
    )
    builder.edge(
        "subscribes",
        NODE_OSF,
        TOPIC,
        key="k",
        layer="declared",
        profile=other,
        values={"qos": {"depth": 1}},
    )
    atlas = builder.build()
    edge = atlas.edges[ids.build_edge("subscribes", NODE_OSF, TOPIC, "k")]
    assert edge.conflicts == []
    assert edge.agreement == Agreement.SINGLE_LAYER.value
    assert edge.effective[PROFILE]["qos.depth"].value == 5
    assert edge.effective[other]["qos.depth"].value == 1


def test_an_unknown_confidence_never_manufactures_drift():
    builder = minimal_builder()
    builder.edge(
        "subscribes",
        NODE_OSF,
        TOPIC,
        key="k",
        layer="coded",
        values={"qos": {"depth": 10}},
        confidence=Confidence.UNKNOWN.value,
    )
    builder.edge(
        "subscribes",
        NODE_OSF,
        TOPIC,
        key="k",
        layer="observed",
        values={"qos": {"depth": 5}},
        confidence=Confidence.CERTAIN.value,
    )
    atlas = builder.build()
    edge = atlas.edges[ids.build_edge("subscribes", NODE_OSF, TOPIC, "k")]
    assert edge.conflicts == []


def test_a_parameter_answers_from_its_candidate_chain():
    atlas = tiny_atlas()
    param = atlas.entities[PARAM]
    assert param.state == ParamState.EFFECTIVE.value
    effective = param.effective[PROFILE]["value"]
    assert effective.value == "nx_controller/DelayAwareController"
    assert effective.from_candidate == 0
    assert effective.source.file == FILE_OVERLAY
    assert [candidate.rank for candidate in param.candidates] == [0, 1]


def test_unresolved_entries_are_mirrored_on_their_owner():
    atlas = tiny_atlas()
    entry = atlas.unresolved[0]
    assert entry.want == Want.TOPIC_NAME.value
    assert atlas.entities[SYM_BT].unresolved == [entry.id]
    assert atlas.counters["unresolved"] == {"topic_name": 1}
    assert atlas.counters["resolution_rate"]["topic_name"] == round(44 / 45, 6)


def test_findings_read_top_to_bottom_as_a_failure_list():
    atlas = tiny_atlas()
    assert [finding.severity for finding in atlas.findings] == ["warn", "info"]
    assert atlas.counters["findings"] == {"error": 0, "warn": 1, "info": 1}
    for finding in atlas.findings:
        assert finding.say and finding.say.endswith(".")
        assert finding.fingerprint.startswith("sha256:")


def test_indexes_answer_fan_in_and_fan_out_without_a_scan():
    atlas = tiny_atlas()
    out = atlas.indexes["out"][NODE_OSF]
    assert len(out) == 1
    assert atlas.edges[out[0]].dst == TOPIC
    assert LAUNCHENT in atlas.indexes["in"] or LAUNCHENT in atlas.indexes["out"]
    assert atlas.indexes["by_kind"]["topic"] == [TOPIC]
    assert atlas.indexes["findings_by_rule"]["qos.differ"]


def test_recompute_is_a_fixpoint():
    builder = AtlasBuilder()
    builder.fact(TOPIC, Layer.CODED.value, values={"type": TYPE_PC2})
    builder.recompute()
    once = builder.atlas.to_bytes()
    builder.recompute()
    builder.recompute()
    assert builder.atlas.to_bytes() == once


# ---------------------------------------------------------------------------
# identity


def test_supersede_rewrites_edges_and_cascades_to_derived_ids():
    builder = minimal_builder()
    builder.node(NODE_CTL, name_source=NameSource.UNKNOWN.value)
    builder.node(FILE_BASE)
    builder.set_param(
        PARAM,
        state=ParamState.EFFECTIVE.value,
        candidates=[
            Candidate(
                value="x",
                value_type="string",
                mechanism=Mechanism.YAML_BASE.value,
                source=Source(file=FILE_BASE),
                rank=0,
                wins=True,
                profile=PROFILE,
            )
        ],
    )
    builder.add_profile("nx_navigation")
    builder.edge("has_param", NODE_CTL, PARAM)
    builder.edge("sets_param", FILE_BASE, PARAM, key="k", layer="declared", values={"rank": 0})
    real = ids.build_node("/controller_server")
    builder.supersede(NODE_CTL, real)
    atlas = builder.build()

    real_param = ids.build_param(real, "FollowPath.plugin")
    assert NODE_CTL not in atlas.entities
    assert PARAM not in atlas.entities
    assert atlas.entities[real].aka == [NODE_CTL]
    assert atlas.entities[real_param].aka == [PARAM]
    assert atlas.entities[real_param].effective[PROFILE]["value"].value == "x"
    assert ids.build_edge("has_param", real, real_param) in atlas.edges
    assert ids.build_edge("sets_param", FILE_BASE, real_param, "k") in atlas.edges
    for edge in atlas.edges.values():
        assert NODE_CTL not in (edge.src, edge.dst)
        assert PARAM not in (edge.src, edge.dst)


def test_supersede_refuses_a_kind_change_and_an_unknown_source():
    builder = minimal_builder()
    with pytest.raises(ContractError):
        builder.supersede(NODE_OSF, TOPIC)
    with pytest.raises(ContractError):
        builder.supersede(ids.build_node("/gone"), NODE_OSF)


def test_supersede_moves_findings_and_unresolved_owners():
    builder = minimal_builder()
    builder.node(NODE_CTL, name_source=NameSource.UNKNOWN.value)
    builder.finding(
        Finding(
            rule="node.unnamed",
            severity=Severity.INFO.value,
            subject=NODE_CTL,
            title="the launch file gives this node no name",
            say="nav2_controller/controller_server is launched with no name.",
        )
    )
    builder.unresolved(
        Unresolved(
            want=Want.NODE_NAME.value,
            owner=NODE_CTL,
            expr="Node(package='nav2_controller', executable='controller_server')",
            reason="no_name_kwarg",
        )
    )
    real = ids.build_node("/controller_server")
    builder.supersede(NODE_CTL, real)
    atlas = builder.build()
    assert atlas.findings[0].subject == real
    assert atlas.unresolved[0].owner == real
    assert atlas.entities[real].unresolved == [atlas.unresolved[0].id]


def _blind_owners() -> list[str]:
    """Three synthetic identities of one process, differing only in role content."""
    return [
        ids.build_node_synthetic(
            "nav2_controller",
            "controller_server",
            "/",
            ids.role_digest(
                package="nav2_controller",
                executable="controller_server",
                remaps=[],
                content=[f"argv:--params-file={name}"],
            ),
        )
        for name in ("a.yaml", "b.yaml", "c.yaml")
    ]


def _blind_builder(owners: list[str]) -> AtlasBuilder:
    builder = minimal_builder()
    builder.node(FILE_LAUNCH)
    for index, owner in enumerate(owners):
        builder.node(owner, name_source=NameSource.UNKNOWN.value)
        builder.unresolved(
            Unresolved(
                want=Want.TOPIC_NAME.value,
                owner=owner,
                expr="self.declare_parameter('cmd_vel_topic').value",
                reason="runtime_attribute_default",
                resolvable_by=[f"probe.{index}"],
                evidence=[Source(file=FILE_LAUNCH, locator=f"Node[{index}]", line_hint=10 + index)],
            )
        )
    return builder


def test_superseding_an_owner_merges_the_blind_spots_that_become_one():
    """Three synthetic identities resolving to one name leave one blind spot, not three
    rows sharing one id."""
    owners = _blind_owners()
    builder = _blind_builder(owners)
    real = ids.build_node("/controller_server")
    for owner in owners:
        builder.supersede(owner, real)
    atlas = builder.build()

    assert len(atlas.unresolved) == 1
    entry = atlas.unresolved[0]
    assert entry.owner == real
    assert [source.locator for source in entry.evidence] == ["Node[0]", "Node[1]", "Node[2]"]
    assert entry.resolvable_by == ["probe.0", "probe.1", "probe.2"]
    assert atlas.entities[real].unresolved == [entry.id]
    assert schema.validate(atlas.to_json()) == []


def test_the_merged_blind_spot_does_not_depend_on_the_merge_order():
    owners = _blind_owners()
    real = ids.build_node("/controller_server")
    bytes_seen = set()
    for order in ((0, 1, 2), (2, 0, 1), (1, 2, 0)):
        builder = _blind_builder(owners)
        for index in order:
            builder.supersede(owners[index], real)
        bytes_seen.add(builder.build().canonical_bytes())
    assert len(bytes_seen) == 1


# ---------------------------------------------------------------------------
# determinism and loading


def test_two_runs_over_the_same_input_are_byte_identical():
    first = tiny_atlas()
    second = tiny_atlas()
    assert first.canonical_bytes() == second.canonical_bytes()
    assert first.content_hash() == second.content_hash()


def test_two_processes_with_different_hash_seeds_agree_on_the_bytes():
    script = (
        f"import sys; sys.path.insert(0, {str(REPO_ROOT)!r})\n"
        "from tests.test_model import tiny_atlas\n"
        "print(tiny_atlas().content_hash())\n"
    )
    digests = set()
    for seed in ("0", "1", "12345"):
        env = dict(os.environ, PYTHONHASHSEED=seed)
        result = subprocess.run(
            [sys.executable, "-c", script],
            capture_output=True,
            text=True,
            check=True,
            env=env,
            cwd=str(REPO_ROOT),
        )
        digests.add(result.stdout.strip())
    assert len(digests) == 1
    assert digests == {tiny_atlas().content_hash()}


def test_canonical_bytes_exclude_generation_metadata_and_volatile_data():
    atlas = tiny_atlas()
    text = atlas.canonical_bytes().decode("utf-8")
    assert "wall_clock_utc" not in text
    assert "duration_s" not in text
    assert "rate_hz" not in text
    assert "line_hint" not in text
    full = atlas.to_bytes().decode("utf-8")
    assert "wall_clock_utc" in full
    assert "line_hint" in full


def test_a_different_clock_does_not_change_the_compared_bytes():
    first = tiny_atlas()
    second = tiny_atlas()
    second.generated["wall_clock_utc"] = "2027-01-01T00:00:00Z"
    assert first.canonical_bytes() == second.canonical_bytes()
    assert first.to_bytes() != second.to_bytes()


def test_serialisation_is_pinned():
    text = tiny_atlas().to_bytes().decode("utf-8")
    assert text.endswith("\n")
    payload = json.loads(text)
    assert list(payload) == sorted(payload)
    assert text == model.dumps(payload)


def test_round_trip_through_json_keeps_the_compared_bytes():
    atlas = tiny_atlas()
    reloaded = Atlas.loads(atlas.to_bytes())
    assert reloaded.canonical_bytes() == atlas.canonical_bytes()
    assert reloaded.entities[PARAM].candidates[0].wins is True
    assert reloaded.entities[PARAM].effective[PROFILE]["value"].value == (
        "nx_controller/DelayAwareController"
    )
    assert reloaded.findings[0].rule == "qos.differ"


def test_loading_tolerates_a_newer_minor_and_refuses_a_newer_major():
    payload = tiny_atlas().to_json()
    payload["schema_version"] = "1.7"
    payload["x"] = {"future": True}
    payload["something_new"] = [1, 2]
    payload["entities"][TOPIC]["future_key"] = "kept"
    payload["entities"][TOPIC]["facts"][0]["confidence"] = "guessed"
    loaded = Atlas.from_json(payload)
    assert loaded.schema_version == "1.7"
    assert loaded.extra["something_new"] == [1, 2]
    assert loaded.entities[TOPIC].extra["future_key"] == "kept"
    assert "future_key" in loaded.to_json()["entities"][TOPIC]
    payload["schema_version"] = "2.0"
    with pytest.raises(ContractError):
        Atlas.from_json(payload)


# ---------------------------------------------------------------------------
# schema


def test_a_hand_written_atlas_validates():
    payload = tiny_atlas().to_json()
    assert schema.validate(payload) == []
    schema.validate_or_raise(payload)


def test_schema_rejects_a_document_that_is_not_an_atlas():
    assert schema.validate({}) != []
    assert schema.validate([]) == ["atlas: document is not an object"]  # type: ignore[arg-type]


def test_schema_catches_a_hand_edited_counter():
    payload = tiny_atlas().to_json()
    payload["counters"]["entities"] = 3
    assert any("counters.entities" in problem for problem in schema.validate(payload))


def test_schema_catches_a_dangling_edge_endpoint():
    payload = tiny_atlas().to_json()
    del payload["entities"][TOPIC]
    problems = schema.validate(payload)
    assert any("is not an entity" in problem for problem in problems)


def test_schema_catches_an_id_that_does_not_match_its_kind():
    payload = tiny_atlas().to_json()
    payload["entities"][TOPIC]["kind"] = "param"
    assert any("does not match the id prefix" in problem for problem in schema.validate(payload))


def test_schema_catches_an_effective_value_on_an_inert_parameter():
    payload = tiny_atlas().to_json()
    payload["entities"][PARAM]["state"] = "inert_by_design"
    assert any("effective must be empty" in problem for problem in schema.validate(payload))


def test_schema_catches_unsorted_findings_and_a_missing_sentence():
    payload = tiny_atlas().to_json()
    payload["findings"].reverse()
    payload["findings"][0]["say"] = ""
    problems = schema.validate(payload)
    assert any("must be sorted" in problem for problem in problems)
    assert any("quotable sentence" in problem for problem in problems)


def test_schema_catches_a_keyed_edge_with_no_key():
    builder = minimal_builder()
    builder.edge("subscribes", NODE_OSF, TOPIC, key="k", layer="coded")
    payload = builder.build().to_json()
    handle = ids.build_edge("subscribes", NODE_OSF, TOPIC, "k")
    edge = payload["edges"].pop(handle)
    edge["key"] = ""
    edge["id"] = ids.build_edge("subscribes", NODE_OSF, TOPIC, "")
    payload["edges"][edge["id"]] = edge
    payload["indexes"]["out"][NODE_OSF] = [edge["id"]]
    payload["indexes"]["in"][TOPIC] = [edge["id"]]
    assert any("needs a key" in problem for problem in schema.validate(payload))


def test_schema_catches_a_node_with_no_name_source():
    payload = tiny_atlas().to_json()
    del payload["entities"][NODE_OSF]["name_source"]
    assert any("name_source" in problem for problem in schema.validate(payload))


def test_schema_catches_an_unmirrored_blind_spot():
    payload = tiny_atlas().to_json()
    payload["entities"][SYM_BT]["unresolved"] = []
    assert any("not mirrored" in problem for problem in schema.validate(payload))


def test_schema_document_is_serialisable_and_names_the_closed_vocabularies():
    text = schema.schema_json()
    document = json.loads(text)
    assert document["$id"].endswith("atlas-1.json")
    entity = document["properties"]["entities"]["additionalProperties"]
    assert "param" in entity["properties"]["kind"]["enum"]
    assert set(document["properties"]["layers"]["additionalProperties"]["required"]) == {
        "present",
        "runs",
        "root",
    }


def test_a_parameter_reports_the_profiles_of_its_candidates():
    atlas = tiny_atlas()
    param = atlas.entities[PARAM]
    assert param.profiles == [PROFILE]
    assert param.agreement == Agreement.SINGLE_LAYER.value
    assert PARAM in atlas.indexes["by_profile"][PROFILE]


def test_lifecycle_may_be_driven_by_a_launch_entity_with_no_manager_node():
    # io_bridge_cpp.launch.py drives the transitions with EmitEvent plus
    # OnStateTransition, so the source of the relation is not a node.
    builder = minimal_builder()
    builder.node(LAUNCHFILE)
    builder.node(LAUNCHENT)
    edge = builder.edge(
        "manages_lifecycle",
        LAUNCHENT,
        NODE_OSF,
        key=ids.edge_key(mechanism="launch_emit_event"),
        layer="declared",
        values={"mechanism": "launch_emit_event", "goal_state": "active"},
    )
    assert edge.src == LAUNCHENT
    assert builder.build().counters["edges_by_kind"]["manages_lifecycle"] == 1


def test_lookup_helpers_answer_without_a_scan_by_the_caller():
    atlas = tiny_atlas()
    assert atlas.get(TOPIC).kind == "topic"
    assert atlas.get("topic:/absent") is None
    assert [edge.kind for edge in atlas.out_edges(NODE_OSF)] == ["subscribes"]
    assert {edge.kind for edge in atlas.in_edges(TOPIC)} == {"remaps", "subscribes"}
    assert [node.id for node in atlas.by_kind("param")] == [PARAM]


def test_the_vocabularies_stay_closed_and_documented():
    assert model.LAYER_ORDER == ("declared", "coded", "observed", "recorded")
    assert model.EFFECTIVE_PRECEDENCE == ("observed", "recorded", "declared", "coded")
    assert set(model.EDGE_DOMAINS) == {item.value for item in model.EdgeKind}
    assert set(model.EDGE_DOMAINS) >= model.KEYED_EDGE_KINDS
    assert set(ids.SEGMENTS) >= {item.value for item in model.NodeKind}
    assert ANY_PROFILE == "*"


def test_agreement_is_per_profile_and_never_unions_the_layers_seen():
    # One declared fact in one profile and one coded fact in another profile means no two
    # layers were ever compared, so the entity is single_layer and not agree.
    builder = minimal_builder()
    first = builder.add_profile("nx_navigation")
    second = builder.add_profile("nx_sim.alt")
    builder.fact(TOPIC, Layer.DECLARED.value, profile=first, values={"type": TYPE_PC2})
    builder.fact(TOPIC, Layer.CODED.value, profile=second, values={"type": TYPE_PC2})
    atlas = builder.build()
    node = atlas.entities[TOPIC]
    assert node.conflicts == []
    assert node.agreement == Agreement.SINGLE_LAYER.value


def test_agreement_is_agree_when_one_profile_saw_two_layers():
    builder = minimal_builder()
    first = builder.add_profile("nx_navigation")
    second = builder.add_profile("nx_sim.alt")
    builder.fact(TOPIC, Layer.DECLARED.value, profile=first, values={"type": TYPE_PC2})
    builder.fact(TOPIC, Layer.CODED.value, profile=first, values={"type": TYPE_PC2})
    builder.fact(TOPIC, Layer.CODED.value, profile=second, values={"type": TYPE_PC2})
    node = builder.build().entities[TOPIC]
    assert node.agreement == Agreement.AGREE.value


def test_a_profile_independent_fact_is_compared_inside_every_profile():
    builder = minimal_builder()
    first = builder.add_profile("nx_navigation")
    builder.fact(TOPIC, Layer.CODED.value, values={"type": TYPE_PC2})
    builder.fact(TOPIC, Layer.DECLARED.value, profile=first, values={"type": TYPE_PC2})
    node = builder.build().entities[TOPIC]
    assert node.agreement == Agreement.AGREE.value


def test_agreement_differs_when_any_one_profile_differs():
    builder = minimal_builder()
    first = builder.add_profile("nx_navigation")
    second = builder.add_profile("nx_sim.alt")
    builder.fact(TOPIC, Layer.DECLARED.value, profile=first, values={"type": TYPE_PC2})
    builder.fact(TOPIC, Layer.CODED.value, profile=first, values={"type": TYPE_PC2})
    builder.fact(TOPIC, Layer.DECLARED.value, profile=second, values={"type": TYPE_PC2})
    builder.fact(
        TOPIC,
        Layer.CODED.value,
        profile=second,
        values={"type": ids.build_type("sensor_msgs", "msg", "LaserScan")},
    )
    node = builder.build().entities[TOPIC]
    assert node.agreement == Agreement.DIFFER.value
    assert node.conflicts == ["type"]


def _row(value: str, profile: str, *, rank: int = 0, wins: bool = True) -> Candidate:
    return Candidate(
        value=value,
        value_type="string",
        mechanism=Mechanism.YAML_BASE.value,
        source=Source(file=FILE_BASE, locator=f"{profile}.{value}"),
        rank=rank,
        wins=wins,
        profile=profile,
    )


def test_a_second_profile_keeps_the_chain_of_the_first():
    """One run covers one profile while the parameter entity is shared by all of them, so
    a second call must replace the profiles it names and keep the ones it does not."""
    other = ids.build_profile("nx_mapping")
    builder = minimal_builder()
    builder.node(NODE_CTL, name_source=NameSource.UNKNOWN.value)
    builder.set_param(PARAM, state=ParamState.EFFECTIVE.value, candidates=[_row("a", PROFILE)])
    builder.set_param(PARAM, state=ParamState.EFFECTIVE.value, candidates=[_row("b", other)])
    atlas = builder.build()
    node = atlas.entities[PARAM]
    assert {(row.profile, row.value) for row in node.candidates} == {(PROFILE, "a"), (other, "b")}
    assert node.effective[PROFILE]["value"].value == "a"
    assert node.effective[other]["value"].value == "b"


def test_writing_one_profile_again_replaces_only_that_profile():
    other = ids.build_profile("nx_mapping")
    builder = minimal_builder()
    builder.node(NODE_CTL, name_source=NameSource.UNKNOWN.value)
    builder.set_param(PARAM, state=ParamState.EFFECTIVE.value, candidates=[_row("a", PROFILE)])
    builder.set_param(PARAM, state=ParamState.EFFECTIVE.value, candidates=[_row("b", other)])
    builder.set_param(PARAM, state=ParamState.EFFECTIVE.value, candidates=[_row("c", PROFILE)])
    node = builder.build().entities[PARAM]
    assert {(row.profile, row.value) for row in node.candidates} == {(PROFILE, "c"), (other, "b")}


def test_the_merged_chain_still_refuses_a_repeated_rank():
    builder = minimal_builder()
    builder.node(NODE_CTL, name_source=NameSource.UNKNOWN.value)
    builder.set_param(PARAM, state=ParamState.EFFECTIVE.value, candidates=[_row("a", PROFILE)])
    with pytest.raises(ContractError):
        builder.set_param(
            PARAM,
            state=ParamState.EFFECTIVE.value,
            candidates=[
                _row("b", ids.build_profile("nx_mapping")),
                _row("c", PROFILE),
                _row("d", PROFILE, wins=False),
            ],
        )
    assert [row.value for row in builder.atlas.entities[PARAM].candidates] == ["a"]


def test_a_profile_that_resolved_a_value_keeps_it_when_another_is_blind():
    """A blind profile never wins a candidate, so the value it could not follow stays
    absent while the profile that did follow it keeps its effective entry."""
    other = ids.build_profile("nx_mapping")
    builder = minimal_builder()
    builder.node(NODE_CTL, name_source=NameSource.UNKNOWN.value)
    builder.set_param(PARAM, state=ParamState.EFFECTIVE.value, candidates=[_row("a", PROFILE)])
    builder.set_param(
        PARAM,
        state=ParamState.OTHER.value,
        candidates=[_row("b", other, wins=False)],
    )
    node = builder.build().entities[PARAM]
    assert node.state == ParamState.EFFECTIVE.value
    assert node.effective[PROFILE]["value"].value == "a"
    assert other not in node.effective


def test_a_call_with_no_candidates_keeps_the_chain():
    builder = minimal_builder()
    builder.node(NODE_CTL, name_source=NameSource.UNKNOWN.value)
    builder.set_param(PARAM, state=ParamState.EFFECTIVE.value, candidates=[_row("a", PROFILE)])
    builder.set_param(PARAM, state=ParamState.EFFECTIVE.value)
    assert [row.value for row in builder.atlas.entities[PARAM].candidates] == ["a"]


def test_supersede_unions_the_candidate_chains_by_profile():
    """Candidates are keyed by profile and each profile is a truth of its own, so a
    synthetic identity resolving to a real one must not delete the profiles the real one
    never carried. Keeping only one side left the parameter with one profile out of two,
    in the chain and in the effective block both."""
    other = ids.build_profile("nx_mapping")
    real_node = ids.build_node("/controller_server")
    real_param = ids.build_param(real_node, "FollowPath.plugin")
    builder = minimal_builder()
    builder.node(NODE_CTL, name_source=NameSource.UNKNOWN.value)
    builder.node(real_node, name_source=NameSource.LITERAL.value)
    builder.set_param(PARAM, state=ParamState.EFFECTIVE.value, candidates=[_row("a", PROFILE)])
    builder.set_param(real_param, state=ParamState.EFFECTIVE.value, candidates=[_row("b", other)])
    builder.supersede(NODE_CTL, real_node)
    node = builder.build().entities[real_param]
    assert [(row.profile, row.value) for row in node.candidates] == [
        (other, "b"),
        (PROFILE, "a"),
    ]
    assert node.effective[PROFILE]["value"].value == "a"
    assert node.effective[other]["value"].value == "b"


def test_supersede_keeps_one_chain_when_both_sides_state_the_same_profile():
    """One profile stated identically on both sides is one truth, not a collision."""
    real_node = ids.build_node("/controller_server")
    real_param = ids.build_param(real_node, "FollowPath.plugin")
    builder = minimal_builder()
    builder.node(NODE_CTL, name_source=NameSource.UNKNOWN.value)
    builder.node(real_node, name_source=NameSource.LITERAL.value)
    builder.set_param(PARAM, state=ParamState.EFFECTIVE.value, candidates=[_row("a", PROFILE)])
    builder.set_param(real_param, state=ParamState.EFFECTIVE.value, candidates=[_row("a", PROFILE)])
    builder.supersede(NODE_CTL, real_node)
    node = builder.build().entities[real_param]
    assert [(row.profile, row.value) for row in node.candidates] == [(PROFILE, "a")]


def test_supersede_refuses_two_different_chains_for_one_profile():
    """Two chains for one profile is a genuine collision: picking one names a value the
    other profile row contradicts."""
    real_node = ids.build_node("/controller_server")
    real_param = ids.build_param(real_node, "FollowPath.plugin")
    builder = minimal_builder()
    builder.node(NODE_CTL, name_source=NameSource.UNKNOWN.value)
    builder.node(real_node, name_source=NameSource.LITERAL.value)
    builder.set_param(PARAM, state=ParamState.EFFECTIVE.value, candidates=[_row("a", PROFILE)])
    builder.set_param(real_param, state=ParamState.EFFECTIVE.value, candidates=[_row("b", PROFILE)])
    with pytest.raises(ContractError):
        builder.supersede(NODE_CTL, real_node)


def test_every_parameter_state_is_ranked():
    """A closed vocabulary needs a total order. Two unranked members compared equal, so the
    state a parameter ended up with was decided by the order the profiles were written."""
    assert sorted(model.PARAM_STATE_PRIORITY) == sorted(state.value for state in ParamState)
    assert len(set(model.PARAM_STATE_PRIORITY)) == len(model.PARAM_STATE_PRIORITY)


@pytest.mark.parametrize("left", [state.value for state in ParamState])
@pytest.mark.parametrize("right", [state.value for state in ParamState])
def test_the_stronger_state_does_not_depend_on_the_order_of_the_pair(left, right):
    assert model.stronger_param_state(left, right) == model.stronger_param_state(right, left)


def test_two_profiles_holding_undeclared_and_declared_unread_agree_either_way():
    """The pair the reviewer reproduced: undeclared one way round, declared_unread the
    other, from the same two facts."""
    assert (
        model.stronger_param_state(ParamState.UNDECLARED.value, ParamState.DECLARED_UNREAD.value)
        == ParamState.UNDECLARED.value
    )
    assert (
        model.stronger_param_state(ParamState.DECLARED_UNREAD.value, ParamState.UNDECLARED.value)
        == ParamState.UNDECLARED.value
    )


def test_a_non_finite_number_has_no_json_spelling_and_gets_none():
    """The representation decision: json has no infinity and no not a number, so the
    encoder refuses one rather than inventing a spelling for it."""
    for value in (float("inf"), float("-inf"), float("nan")):
        with pytest.raises(ContractError):
            model.canonical_value(value)
        assert model.NON_FINITE in (model.unrepresentable(value) or "")
    assert model.unrepresentable([1.0, float("inf")]) is not None
    assert model.unrepresentable({"a": float("nan")}) is not None


def test_unrepresentable_accepts_everything_the_encoder_writes():
    for value in (0, 1, -(2**63), 20.0, 1e308, True, False, None, "", "text", [1, "a"], {"k": 1}):
        assert model.unrepresentable(value) is None
    assert model.unrepresentable(object()) is not None


def test_the_dump_of_a_refused_value_never_happens_silently():
    with pytest.raises(ContractError):
        model.dumps({"gain": float("inf")})


# ---------------------------------------------------------------------------
# pass completeness, which the resolution rate cannot express


def _run_with_passes(*rows: tuple[str, str, str]) -> model.Atlas:
    builder = minimal_builder()
    run = builder.add_run("coded", profile=PROFILE, input_digest="c0ffee01")
    for collector, state, reason in rows:
        builder.record_pass(run, collector, state=state, reason=reason, missing="what it reads")
    return builder.build()


def test_a_run_records_whether_each_collector_finished():
    atlas = _run_with_passes(
        ("cpp_code", "refused", "CollectorError: tree-sitter is missing"),
        ("py_code", "completed", ""),
        ("urdf", "not_attempted", "no urdf model is under any root"),
    )
    rows = model.passes_of(atlas)
    assert [row["collector"] for _, row in rows] == ["cpp_code", "py_code", "urdf"]
    assert [row["state"] for _, row in rows] == ["refused", "completed", "not_attempted"]
    assert [row["collector"] for _, row in model.incomplete_passes(atlas)] == ["cpp_code"]


def test_the_counter_answers_whether_the_atlas_is_whole_without_walking_the_runs():
    """The resolution rate cannot: a pass that died contributes neither a resolved nor an
    unresolved site, so its want keeps a perfect rate or leaves the counter entirely."""
    atlas = _run_with_passes(
        ("cpp_code", "refused", "CollectorError: tree-sitter is missing"),
        ("py_code", "completed", ""),
        ("urdf", "not_attempted", "no urdf model is under any root"),
    )
    assert atlas.counters["passes"] == {
        "attempted": 2,
        "completed": 1,
        "refused": 1,
        "not_attempted": 1,
        "refused_collectors": ["cpp_code"],
        "complete": False,
    }
    assert atlas.counters["resolution_rate"] == {}, "the rate says nothing about the loss"


def test_a_collector_with_nothing_to_read_leaves_the_atlas_whole():
    atlas = _run_with_passes(("urdf", "not_attempted", "no urdf model is under any root"))
    assert atlas.counters["passes"]["complete"] is True
    assert atlas.counters["passes"]["attempted"] == 0
    assert model.incomplete_passes(atlas) == []


def test_an_atlas_that_records_no_pass_claims_no_loss():
    atlas = minimal_builder().build()
    assert atlas.counters["passes"]["complete"] is True
    assert atlas.counters["passes"]["attempted"] == 0


def test_a_pass_needs_a_run_a_collector_and_a_reason_when_it_did_not_complete():
    builder = minimal_builder()
    run = builder.add_run("coded", profile=PROFILE, input_digest="c0ffee01")
    with pytest.raises(ContractError):
        builder.record_pass(NODE_OSF, "cpp_code", state="completed")
    with pytest.raises(ContractError):
        builder.record_pass(run, "", state="completed")
    with pytest.raises(ContractError):
        builder.record_pass(run, "cpp_code", state="refused")
    with pytest.raises(ContractError):
        builder.record_pass(run, "cpp_code", state="finished_ish")
    builder.record_pass(run, "cpp_code", state="completed")
    builder.record_pass(run, "cpp_code", state="completed")  # one outcome twice is one row
    with pytest.raises(ContractError):
        builder.record_pass(run, "cpp_code", state="refused", reason="and now it stopped")
    assert len(model.passes_of(builder.build())) == 1


def test_the_pass_rows_are_sorted_and_the_atlas_stays_byte_identical():
    rows = (("urdf", "completed", ""), ("cpp_code", "refused", "ValueError: inf in an origin"))
    first = _run_with_passes(*rows)
    second = _run_with_passes(*reversed(rows))
    assert first.canonical_bytes() == second.canonical_bytes()
    assert schema.validate(first.to_json()) == []


# ---------------------------------------------------------------------------
# the schema minor


def _minor_note() -> str:
    """The prose that has to say what the minor added, read from the module itself."""
    source = Path(model.__file__).read_text(encoding="utf-8")
    return source.split('SCHEMA_MINOR = "', 1)[1].split("SCHEMA_VERSION", 1)[0]


def test_the_minor_is_bumped_and_says_what_each_addition_was():
    """Three additions stood under an unchanged 1.0: a rule id, the pass record, and the
    rename of a `refused_because` value. All three are additive, so the minor moves."""
    assert model.SCHEMA_VERSION == "1.15"
    assert model.SCHEMA_VERSION.endswith("." + model.SCHEMA_MINOR)
    note = _minor_note()
    for addition in (
        "passes",
        "extract.input_refused",
        "plugin_manifest_unreadable",
        "stamp",
        "time.*",
        "rmw.mixed",
        "zenoh.router_unreachable",
        "qos.incompatible",
        "param.sim_time_mixed",
        "launch.file_not_installed",
        "file_not_installed",
        "volatile.stamps",
        "volatile.stamp_verdict",
        "file_reads",
        "definition_file",
        "class_templates",
        "unresolved entry `construction`",
        "launch exception row `kept`",
        "record `guard_parameters`",
        "frame `urdf_geometry`",
        "model.mesh_missing",
        "meshes_not_installed",
        "meshes_absent",
    ):
        assert addition in note


def test_a_reader_pinned_to_the_old_minor_still_loads_a_file_written_at_the_new_one(
    monkeypatch,
):
    """The check the bump is worth. The loader compares the major alone, and the keys and
    the enum members that build never heard of are kept and re emitted rather than dropped."""
    payload = tiny_atlas().to_json()
    assert payload["schema_version"] == "1.15"
    run_id = ids.build_run("declared", "nx_navigation", "9f2c1a4b")
    payload["entities"][run_id] = {
        "id": run_id,
        "kind": "run",
        "label": "declared",
        "attrs": {
            "passes": [{"collector": "cpp_code", "state": "not_attempted", "reason": "no C++ here"}]
        },
    }
    payload["entities"][TOPIC]["attrs"]["refused_because"] = "plugin_manifest_unreadable"
    frame = ids.build_frame("base_link")
    shapes = {"visuals": [{"geometry": {"type": "sphere", "radius": 0.05}}], "collisions": []}
    payload["entities"][frame] = {
        "id": frame,
        "kind": "frame",
        "label": "base_link",
        "attrs": {"urdf_geometry": {"file:example|robot.urdf": shapes}},
    }
    payload["entities"]["file:example|robot.urdf"] = {
        "id": "file:example|robot.urdf",
        "kind": "file",
        "label": "robot.urdf",
        "attrs": {"meshes_absent": ["package://example/meshes/gone.stl"]},
    }
    stamps = AtlasBuilder()
    recording = ids.build_bag("sample")
    topic = ids.build_bagtopic("sample", "/scan")
    stamps.fact(recording, "recorded", values={"stamp_verdict": {"level": "ok", "hypotheses": []}})
    stamps.fact(topic, "recorded", values={"stamps": {"messages": 4, "checks": []}})
    launchent = ids.build_launchent("example", "launch/demo.py", [ids.LaunchStep("Node", "robot")])
    stamps.fact(
        launchent,
        "declared",
        values={"file_reads": [{"file": "file:example|robot.xacro", "role": "robot_model"}]},
    )
    source = "file:example|main.cpp"
    stamps.fact(
        source,
        "coded",
        values={
            "constructions": [
                {"class": "Listener", "definition_file": "file:example|listener.hpp"}
            ],
            "class_templates": [{"name": "Filter", "definitions": 1, "specialized": False}],
        },
    )
    payload["entities"].update(stamps.build().to_json()["entities"])
    monkeypatch.setattr(model, "SCHEMA_VERSION", "1.0")
    old = Atlas.from_json(payload)
    assert old.schema_version == "1.15"
    assert old.entities[source].fact("coded").values["class_templates"] == [
        {"name": "Filter", "definitions": 1, "specialized": False}
    ]
    assert (
        old.entities[source].fact("coded").values["constructions"][0]["definition_file"]
        == "file:example|listener.hpp"
    )
    assert old.entities[topic].fact("recorded").values["stamps"]["messages"] == 4
    assert old.entities[recording].fact("recorded").values["stamp_verdict"]["level"] == "ok"
    assert old.entities[run_id].attrs["passes"][0]["state"] == "not_attempted"
    assert old.entities[launchent].fact("declared").values["file_reads"][0]["role"] == "robot_model"
    assert old.entities[TOPIC].attrs["refused_because"] == "plugin_manifest_unreadable"
    assert old.entities[frame].attrs["urdf_geometry"]["file:example|robot.urdf"] == shapes
    assert "not_attempted" in model.dumps(old.to_json())
    payload["schema_version"] = "2.0"
    with pytest.raises(ContractError):
        Atlas.from_json(payload)


# ---------------------------------------------------------------------------
# one encoder, and the walk that keeps it one


ENCODER_OWNER = "rosio/model.py"
"""The one module allowed to reach the json encoder. AGENTS section 9 point 1."""


def encoder_calls(root: Path) -> list[str]:
    """Every place under `rosio/` that reaches the json encoder itself.

    The walk parses rather than greps, so a line of python held in a string, which is a
    program run by another interpreter and not a call this package makes, is not a hit and
    a call spelled over two lines is.
    """
    found: list[str] = []
    for path in sorted(root.glob("**/*.py")):
        rel = path.relative_to(root.parent).as_posix()
        if rel == ENCODER_OWNER:
            continue
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Attribute) and node.attr in ("dumps", "dump"):
                if isinstance(node.value, ast.Name) and node.value.id == "json":
                    found.append(f"{rel}:{node.lineno} json.{node.attr}")
            elif isinstance(node, ast.ImportFrom) and node.module == "json":
                for alias in node.names:
                    if alias.name in ("dumps", "dump"):
                        found.append(f"{rel}:{node.lineno} from json import {alias.name}")
    return found


def test_no_module_but_the_model_reaches_the_json_encoder():
    """Section 9 point 1 was an unenforced sentence, and three writers had drifted out from
    under it: the hook state file, the hook log and the cache metadata, all in history.py.
    The rule governs what this package encodes, so `model.dumps` and `model.dumps_line` are
    the two doors, and a module that wants a third asks for it here first."""
    assert encoder_calls(REPO_ROOT / "rosio") == []


def test_the_walk_sees_a_direct_call_that_is_put_back(tmp_path: Path):
    package = tmp_path / "rosio"
    package.mkdir()
    (package / "model.py").write_text("import json\njson.dumps({})\n", encoding="utf-8")
    (package / "history.py").write_text(
        "import json\n"
        "PROBE = 'print(json.dumps(row))'\n"
        "def write(row):\n"
        "    return json.dumps(row, sort_keys=True)\n",
        encoding="utf-8",
    )
    assert encoder_calls(package) == ["rosio/history.py:4 json.dumps"]


def test_the_line_encoder_is_the_same_encoder():
    row = {"b": 2, "a": {"d": True, "c": "x"}}
    assert model.dumps_line(row) == '{"a": {"c": "x", "d": true}, "b": 2}'
    assert json.loads(model.dumps_line(row)) == json.loads(model.dumps(row))
    assert "\n" not in model.dumps_line(row)
    with pytest.raises(ContractError):
        model.dumps_line({"a": float("nan")})
