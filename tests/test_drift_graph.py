"""The node and topic rule families: what fires, what stays quiet, and what is stable.

Every rule gets three tests, per AGENTS section 14: it fires on a crafted atlas, it stays
silent on the near miss, and its finding id and fingerprint are the same over two builds. On
top of that, the rules that need a running system are checked against an atlas with the
observed layer removed, because a rule that fires on an uncollected layer is worse than a
rule that never fires at all.

The crafted atlas carries observed facts the M5 probe has not been written yet to produce.
That is deliberate: the shape it writes is the contract stated in the two module docstrings,
and these tests are what holds the rules to it.
"""

from __future__ import annotations

from typing import Any

import pytest

from rosio import ids, model
from rosio.drift import nodes as node_rules
from rosio.drift import topics as topic_rules

REAL = "profile:nx_navigation"
PKG = "nx_bridge"

OBSERVED = model.Layer.OBSERVED.value
DECLARED = model.Layer.DECLARED.value
CODED = model.Layer.CODED.value

BRIDGE = ids.build_node("/io_bridge")
SUPERVISOR = ids.build_node("/motion_supervisor")
GATED = ids.build_node("/map_saver")
LEFTOVER = ids.build_node("/left_over")
TOOLING = ids.build_node("/_ros2cli_daemon_1")
RUNNING_TF = ids.build_node("/static_transform_publisher_9911")
STATIC_A = ids.build_node_synthetic("tf2_ros", "static_transform_publisher", "/", "aa11bb22")
STATIC_B = ids.build_node_synthetic("tf2_ros", "static_transform_publisher", "/", "cc33dd44")
UNNAMED = ids.build_node_synthetic(PKG, "speed_limiter", "/", "ee55ff66")
UNREAD = ids.build_node_synthetic(PKG, "obstacle_governor", "/", "77aa22bb")
REGRESSED = ids.build_node_synthetic(PKG, "pose_health", "/", "1177aa33")

SOURCE = model.Source(file=ids.build_file(PKG, f"{PKG}/io_bridge.py"), locator="main", line_hint=7)


def qos(**parts: Any) -> dict[str, Any]:
    """A QoS the way both layers spell it: every property present, null when unstated."""
    out: dict[str, Any] = {
        "raw": "",
        "form": "depth",
        "history": None,
        "depth": None,
        "reliability": None,
        "durability": None,
        "preset": None,
    }
    out.update(parts)
    return out


class Bench:
    """A small joined atlas, built by hand, with an observed layer."""

    def __init__(self) -> None:
        self.builder = model.AtlasBuilder()
        for layer in (DECLARED, CODED, OBSERVED):
            self.builder.declare_layer(layer, present=True)
        self.profile = self.builder.add_profile("nx_navigation")
        self._nodes()
        self._graph()
        self.builder.recompute()

    # -- entities ---------------------------------------------------------
    def declared(self, node_id: str, executable: str, **values: Any) -> None:
        payload = {"pkg": PKG, "executable": executable, "form": "node"}
        payload.update(values)
        self.builder.node(node_id, name_source=values.pop("name_source", "literal"))
        self.builder.fact(
            node_id, DECLARED, profile=self.profile, values=payload, evidence=[SOURCE]
        )

    def observed(self, node_id: str, name: str, **values: Any) -> None:
        payload: dict[str, Any] = {"name": name, "namespace": "/"}
        payload.update(values)
        self.builder.node(node_id)
        self.builder.fact(node_id, OBSERVED, profile=self.profile, values=payload)

    def _nodes(self) -> None:
        self.declared(BRIDGE, "io_bridge")
        self.observed(BRIDGE, "io_bridge")
        self.builder.fact(
            BRIDGE, CODED, profile=self.profile, values={"name": "io_bridge"}, evidence=[SOURCE]
        )
        self.declared(SUPERVISOR, "motion_supervisor")
        self.declared(GATED, "map_saver")
        site = ids.build_launchent(PKG, "launch/maps.launch.py", (("Node", "map_saver"),))
        self.builder.node(site)
        gate = {"mechanism": "OnProcessExit", "expr": "map_server"}
        self.builder.fact(site, DECLARED, profile=self.profile, values={"gates": [gate]})
        self.builder.edge(
            model.EdgeKind.LAUNCHES.value,
            site,
            GATED,
            key=ids.edge_key(launchent=site, profile=self.profile),
            layer=DECLARED,
            profile=self.profile,
        )
        self.observed(LEFTOVER, "left_over")
        self.observed(TOOLING, "_ros2cli_daemon_1")
        for node_id, role in ((STATIC_A, "aa11bb22"), (STATIC_B, "cc33dd44")):
            self.declared(
                node_id,
                "static_transform_publisher",
                name_source=model.NameSource.RUNTIME_RANDOMISED.value,
                role=role,
            )
        self.observed(RUNNING_TF, "static_transform_publisher_9911")
        # read, and the source passes no name to the constructor
        self.declared(UNNAMED, "speed_limiter", name_source=model.NameSource.UNKNOWN.value)
        self.builder.fact(UNNAMED, CODED, profile=self.profile, values={}, evidence=[SOURCE])
        # launched the same way, source never read, which is the reference stack case
        self.declared(UNREAD, "obstacle_governor", name_source=model.NameSource.UNKNOWN.value)
        self.declared(REGRESSED, "pose_health")
        self.builder.atlas.entities[REGRESSED].aka = [ids.build_node("/pose_health")]

    # -- graph ------------------------------------------------------------
    def endpoint(
        self,
        kind: str,
        node_id: str,
        topic: str,
        layer: str,
        *,
        key: str = "",
        values: dict[str, Any] | None = None,
    ) -> model.Edge:
        topic_id = ids.build_topic(topic)
        self.builder.node(topic_id)
        return self.builder.edge(
            kind,
            node_id,
            topic_id,
            key=key or ids.edge_key(profile=self.profile, raw_name=topic, kind=kind),
            layer=layer,
            profile=self.profile,
            values=values or {},
            evidence=[SOURCE],
        )

    def _graph(self) -> None:
        publishes = model.EdgeKind.PUBLISHES.value
        subscribes = model.EdgeKind.SUBSCRIBES.value
        # published and nobody listens, seen at runtime and in the source
        for layer in (CODED, OBSERVED):
            self.endpoint(publishes, BRIDGE, "/vehicle/command", layer)
            self.endpoint(subscribes, BRIDGE, "/vehicle/state", layer)
            self.endpoint(publishes, BRIDGE, "/cmd_vel", layer)
        # both ends present, one of them known only from the source
        self.endpoint(subscribes, SUPERVISOR, "/cmd_vel", CODED)
        # a library made this one, and its name is in the wrapper table
        self.endpoint(publishes, BRIDGE, "/tf", OBSERVED, values={"via_known_wrapper": True})
        # two layers state the same endpoint, with a different qos and a different type
        self.qos_edge = self.endpoint(
            publishes,
            BRIDGE,
            "/vehicle/telemetry",
            CODED,
            key="telemetry",
            values={"qos": qos(history="keep_last", depth=10), "type": "nav_msgs::msg::Odometry"},
        )
        self.endpoint(
            publishes,
            BRIDGE,
            "/vehicle/telemetry",
            OBSERVED,
            key="telemetry",
            values={
                "qos": qos(history="keep_last", depth=1),
                "type": "nav_msgs/msg/Odometry",
                "volatile": {"rate_hz": 4.0},
                "rate_hz": None,
            },
        )
        # a rate the source states and the running system does not hold
        self.rate_edge = self.endpoint(
            publishes, BRIDGE, "/vehicle/heartbeat", CODED, key="beat", values={"rate_hz": 20.0}
        )
        self.endpoint(
            publishes,
            BRIDGE,
            "/vehicle/heartbeat",
            OBSERVED,
            key="beat",
            values={"volatile": {"rate_hz": 5.0}},
        )
        # something consumes both of those, so only the intended rule has anything to say
        self.endpoint(subscribes, SUPERVISOR, "/vehicle/telemetry", OBSERVED, key="telemetry_in")
        self.endpoint(subscribes, SUPERVISOR, "/vehicle/heartbeat", OBSERVED, key="beat_in")
        # only the probe knows about this one, on a node whose source was read
        self.only_edge = self.endpoint(publishes, BRIDGE, "/vehicle/extra", OBSERVED, key="extra")
        self.endpoint(subscribes, LEFTOVER, "/vehicle/extra", OBSERVED, key="extra_in")
        # the same shape on a node whose source was never read
        self.endpoint(publishes, SUPERVISOR, "/vehicle/spare", OBSERVED, key="spare")
        self.endpoint(subscribes, LEFTOVER, "/vehicle/spare", OBSERVED, key="spare_in")
        # a complete probe states the endpoints of every node it reports, this one included
        self.endpoint(
            publishes,
            RUNNING_TF,
            "/tf_static",
            OBSERVED,
            key="tf_static",
            values={"via_known_wrapper": True},
        )

    # -- variants ---------------------------------------------------------
    @property
    def atlas(self) -> model.Atlas:
        return self.builder.atlas

    def without_observed(self) -> model.Atlas:
        atlas = model.Atlas.loads(self.atlas.to_bytes())
        for entity in atlas.entities.values():
            entity.facts = [fact for fact in entity.facts if fact.layer != OBSERVED]
        for edge in atlas.edges.values():
            edge.facts = [fact for fact in edge.facts if fact.layer != OBSERVED]
        atlas.layers[OBSERVED] = model.LayerStatus(present=False, reason="not collected")
        return atlas


@pytest.fixture()
def bench() -> Bench:
    return Bench()


def rule(name: str):
    table = {**dict(node_rules.RULES), **dict(topic_rules.RULES)}
    return table[name]


def fired(atlas: model.Atlas, name: str) -> list[model.Finding]:
    return rule(name)(atlas)


def subjects(findings: list[model.Finding]) -> list[str]:
    return sorted(finding.subject_key for finding in findings)


# ---------------------------------------------------------------------------
# the family is silent without the layer it needs


NEEDS_OBSERVED = (
    "node.declared_not_observed",
    "node.observed_only",
    "node.ambiguous_identity",
    "topic.no_subscriber",
    "topic.no_publisher",
    "qos.differ",
    "type.differ",
    "rate.below_declared",
    "endpoint.observed_only",
)

READS_THE_REPOSITORY = ("node.identity_regressed", "node.unnamed")


@pytest.mark.parametrize("name", NEEDS_OBSERVED)
def test_a_rule_that_needs_a_running_system_is_silent_without_one(bench: Bench, name: str) -> None:
    assert fired(bench.without_observed(), name) == []


@pytest.mark.parametrize("name", READS_THE_REPOSITORY)
def test_a_repository_only_rule_still_speaks_without_the_observed_layer(
    bench: Bench, name: str
) -> None:
    assert fired(bench.without_observed(), name)


def test_no_rule_reads_a_layer_the_atlas_only_claims_to_have(bench: Bench) -> None:
    """A layer declared present and never collected must count as absent."""
    atlas = bench.without_observed()
    atlas.layers[OBSERVED] = model.LayerStatus(present=True)
    for name in NEEDS_OBSERVED:
        assert fired(atlas, name) == [], name


def test_the_two_modules_cover_the_committed_rule_ids() -> None:
    assert [item[0] for item in node_rules.RULES] + [item[0] for item in topic_rules.RULES] == [
        "node.ambiguous_identity",
        "node.declared_not_observed",
        "node.identity_regressed",
        "node.observed_only",
        "node.unnamed",
        "endpoint.observed_only",
        "qos.differ",
        "qos.incompatible",
        "rate.below_declared",
        "topic.no_publisher",
        "topic.no_subscriber",
        "type.differ",
    ]


# ---------------------------------------------------------------------------
# node.declared_not_observed


def test_declared_not_observed_fires_on_a_node_that_did_not_start(bench: Bench) -> None:
    found = fired(bench.atlas, "node.declared_not_observed")
    assert subjects(found) == [SUPERVISOR]
    assert found[0].severity == model.Severity.WARN.value
    assert "motion_supervisor" in found[0].say
    assert found[0].where == [SOURCE]


def test_a_finding_never_cites_a_file_of_rosio_itself() -> None:
    """The committed name table is evidence for a fact, and a finding that cited it sent a
    reader to `rosio/resolve/known_names.py`, which is in no tree of theirs."""
    from rosio.drift import rules

    bench = Bench()
    table = model.Source(
        file=ids.build_file(ids.REPO_PKG, "rosio/resolve/known_names.py"),
        locator=f"{PKG}/motion_supervisor",
    )
    bench.atlas.entities[SUPERVISOR].fact(DECLARED, bench.profile).evidence.append(table)
    assert table in fired(bench.atlas, "node.declared_not_observed")[0].where
    (found,) = rules.findings_for(bench.atlas, ["node.declared_not_observed"])
    assert found.where == [SOURCE]


def test_declared_not_observed_is_silent_on_a_gated_node(bench: Bench) -> None:
    assert GATED not in subjects(fired(bench.atlas, "node.declared_not_observed"))


def test_declared_not_observed_is_silent_on_a_node_that_did_start(bench: Bench) -> None:
    assert BRIDGE not in subjects(fired(bench.atlas, "node.declared_not_observed"))


def test_declared_not_observed_is_silent_on_a_name_that_is_not_identity(bench: Bench) -> None:
    """A randomised or unnamed node cannot be looked for in a probe by its name."""
    found = subjects(fired(bench.atlas, "node.declared_not_observed"))
    assert STATIC_A not in found and STATIC_B not in found and UNNAMED not in found


def test_declared_not_observed_has_a_stable_id() -> None:
    left = fired(Bench().atlas, "node.declared_not_observed")[0]
    right = fired(Bench().atlas, "node.declared_not_observed")[0]
    assert left.id == right.id
    assert left.fingerprint == right.fingerprint


# ---------------------------------------------------------------------------
# node.observed_only


def test_observed_only_fires_on_a_node_nothing_launches(bench: Bench) -> None:
    found = fired(bench.atlas, "node.observed_only")
    assert subjects(found) == [LEFTOVER]
    assert "left_over" in found[0].say


def test_observed_only_is_silent_on_command_line_tooling(bench: Bench) -> None:
    assert TOOLING not in subjects(fired(bench.atlas, "node.observed_only"))


def test_observed_only_is_silent_on_a_node_a_launch_file_states(bench: Bench) -> None:
    assert BRIDGE not in subjects(fired(bench.atlas, "node.observed_only"))


def test_observed_only_is_silent_on_a_wrapper_node(bench: Bench) -> None:
    bench.observed(ids.build_node("/inner_listener"), "inner_listener", via_known_wrapper=True)
    assert ids.build_node("/inner_listener") not in subjects(
        fired(bench.atlas, "node.observed_only")
    )


def test_observed_only_is_silent_on_a_node_a_launched_parent_creates(bench: Bench) -> None:
    parent = ids.build_node("/controller_server")
    child = ids.build_node("/local_costmap/local_costmap")
    bench.declared(parent, "controller_server")
    bench.observed(child, "local_costmap", namespace="/local_costmap")
    bench.builder.node(child, attrs={"child_of": {parent: "nav2_controller/controller_server"}})
    assert child not in subjects(fired(bench.atlas, "node.observed_only"))


def test_observed_only_still_fires_when_the_parent_is_not_launched(bench: Bench) -> None:
    parent = ids.build_node("/controller_server")
    child = ids.build_node("/local_costmap/local_costmap")
    bench.observed(child, "local_costmap", namespace="/local_costmap")
    bench.builder.node(child, attrs={"child_of": {parent: "nav2_controller/controller_server"}})
    assert child in subjects(fired(bench.atlas, "node.observed_only"))


def test_observed_only_has_a_stable_id() -> None:
    left = fired(Bench().atlas, "node.observed_only")[0]
    right = fired(Bench().atlas, "node.observed_only")[0]
    assert left.id == right.id
    assert left.fingerprint == right.fingerprint


# ---------------------------------------------------------------------------
# node.ambiguous_identity


def test_ambiguous_identity_fires_when_two_instances_could_be_one_running_node(
    bench: Bench,
) -> None:
    found = fired(bench.atlas, "node.ambiguous_identity")
    assert subjects(found) == [RUNNING_TF]
    assert ":~" not in found[0].say and ":~" not in found[0].title
    assert "static_transform_publisher" in found[0].say


def test_ambiguous_identity_is_silent_when_only_one_instance_matches(bench: Bench) -> None:
    del bench.atlas.entities[STATIC_B]
    assert fired(bench.atlas, "node.ambiguous_identity") == []


def test_ambiguous_identity_is_silent_when_the_join_already_said_it(bench: Bench) -> None:
    bench.builder.finding(
        model.Finding(
            rule="node.ambiguous_identity",
            severity=model.Severity.WARN.value,
            subject=RUNNING_TF,
            title="already reported by the join",
            detail="two sources behind one executable state two names",
        )
    )
    assert fired(bench.atlas, "node.ambiguous_identity") == []


def test_ambiguous_identity_has_a_stable_id() -> None:
    left = fired(Bench().atlas, "node.ambiguous_identity")[0]
    right = fired(Bench().atlas, "node.ambiguous_identity")[0]
    assert left.id == right.id
    assert left.fingerprint == right.fingerprint


def test_ambiguous_nodes_are_not_reported_as_missing_or_as_extra(bench: Bench) -> None:
    """An identity nobody can decide must produce one finding, not three."""
    assert RUNNING_TF not in subjects(fired(bench.atlas, "node.observed_only"))
    missing = subjects(fired(bench.atlas, "node.declared_not_observed"))
    assert STATIC_A not in missing and STATIC_B not in missing


# ---------------------------------------------------------------------------
# node.identity_regressed


def test_identity_regressed_fires_when_a_named_node_became_synthetic(bench: Bench) -> None:
    found = fired(bench.atlas, "node.identity_regressed")
    assert subjects(found) == [REGRESSED]
    assert found[0].severity == model.Severity.ERROR.value
    assert "pose_health" in found[0].say
    assert ":~" not in found[0].say


def test_identity_regressed_is_silent_on_a_normal_supersession(bench: Bench) -> None:
    real = ids.build_node("/command_gate")
    bench.declared(real, "command_gate")
    bench.atlas.entities[real].aka = [
        ids.build_node_synthetic(PKG, "command_gate", "/", "5e2b5a36")
    ]
    assert subjects(fired(bench.atlas, "node.identity_regressed")) == [REGRESSED]


def test_identity_regressed_has_a_stable_id() -> None:
    left = fired(Bench().atlas, "node.identity_regressed")[0]
    right = fired(Bench().atlas, "node.identity_regressed")[0]
    assert left.id == right.id
    assert left.fingerprint == right.fingerprint


# ---------------------------------------------------------------------------
# node.unnamed


def test_unnamed_fires_on_a_node_no_layer_names(bench: Bench) -> None:
    found = fired(bench.atlas, "node.unnamed")
    assert subjects(found) == [UNNAMED]
    assert found[0].severity == model.Severity.INFO.value
    assert "speed_limiter" in found[0].say
    assert ":~" not in found[0].say and ":~" not in found[0].title


def test_unnamed_is_silent_on_a_randomised_or_table_resolved_name(bench: Bench) -> None:
    found = subjects(fired(bench.atlas, "node.unnamed"))
    assert STATIC_A not in found and BRIDGE not in found


def test_unnamed_is_silent_when_the_source_was_never_read(bench: Bench) -> None:
    """An unknown name source is a gap in the reader until the source has been read.

    Every ANT bridge executable passes a literal to `super().__init__`, and the python half of
    the coded layer reaches no atlas yet, so all six carry name_source unknown. Reporting them
    would be reporting the reader.
    """
    assert UNREAD not in subjects(fired(bench.atlas, "node.unnamed"))


def test_unnamed_is_silent_when_the_source_states_the_name(bench: Bench) -> None:
    bench.builder.fact(UNREAD, CODED, profile=bench.profile, values={"name": "obstacle_governor"})
    assert UNREAD not in subjects(fired(bench.atlas, "node.unnamed"))


def test_unnamed_names_the_command_when_the_node_is_a_bare_process(bench: Bench) -> None:
    node_id = ids.build_node_synthetic(PKG, "io_bridge", "/", "32e7a3dd")
    bench.declared(node_id, "io_bridge", form="process", name_source=model.NameSource.UNKNOWN.value)
    bench.builder.fact(node_id, CODED, profile=bench.profile, values={}, evidence=[SOURCE])
    found = [item for item in fired(bench.atlas, "node.unnamed") if item.subject_key == node_id]
    assert "__node" in found[0].fix


def test_unnamed_has_a_stable_id() -> None:
    left = fired(Bench().atlas, "node.unnamed")[0]
    right = fired(Bench().atlas, "node.unnamed")[0]
    assert left.id == right.id
    assert left.fingerprint == right.fingerprint


# ---------------------------------------------------------------------------
# topic.no_subscriber and topic.no_publisher


def test_no_subscriber_fires_on_a_topic_the_probe_saw_with_one_end(bench: Bench) -> None:
    found = fired(bench.atlas, "topic.no_subscriber")
    assert subjects(found) == [ids.build_topic("/vehicle/command")]
    assert "/vehicle/command" in found[0].say


def test_no_subscriber_is_silent_when_the_other_end_is_in_any_layer(bench: Bench) -> None:
    """The subscriber of /cmd_vel is only in the coded layer, and that is still an end."""
    assert ids.build_topic("/cmd_vel") not in subjects(fired(bench.atlas, "topic.no_subscriber"))


def test_no_subscriber_is_silent_on_a_topic_a_library_creates(bench: Bench) -> None:
    assert ids.build_topic("/tf") not in subjects(fired(bench.atlas, "topic.no_subscriber"))


def test_no_subscriber_is_silent_when_the_only_subscriber_is_behind_a_wrapper(
    bench: Bench,
) -> None:
    """A tf2_ros listener and a message_filters subscription are real consumers.

    They are never a subject, and they are still an end, so the publisher they consume from is
    not a dead topic. Sixteen of the first and two of the second exist on the reference stack.
    """
    bench.endpoint(model.EdgeKind.PUBLISHES.value, BRIDGE, "/points_filtered", OBSERVED, key="pts")
    bench.endpoint(
        model.EdgeKind.SUBSCRIBES.value,
        SUPERVISOR,
        "/points_filtered",
        OBSERVED,
        key="pts_in",
        values={"via_known_wrapper": True},
    )
    bench.observed(SUPERVISOR, "motion_supervisor")
    assert ids.build_topic("/points_filtered") not in subjects(
        fired(bench.atlas, "topic.no_subscriber")
    )


def test_no_subscriber_is_silent_on_a_topic_the_probe_never_saw(bench: Bench) -> None:
    bench.endpoint(model.EdgeKind.PUBLISHES.value, BRIDGE, "/paper_only", CODED)
    assert ids.build_topic("/paper_only") not in subjects(fired(bench.atlas, "topic.no_subscriber"))


def test_no_publisher_fires_on_an_input_that_never_arrives(bench: Bench) -> None:
    found = fired(bench.atlas, "topic.no_publisher")
    assert subjects(found) == [ids.build_topic("/vehicle/state")]
    assert "/vehicle/state" in found[0].say


def test_no_publisher_is_silent_on_a_topic_with_a_publisher(bench: Bench) -> None:
    assert ids.build_topic("/cmd_vel") not in subjects(fired(bench.atlas, "topic.no_publisher"))


def test_a_topic_with_no_endpoint_at_all_is_never_a_finding(bench: Bench) -> None:
    """30 of 52 topic entities on the reference stack are in exactly this state."""
    bench.builder.node(ids.build_topic("/minted_from_a_remap"))
    for name in ("topic.no_publisher", "topic.no_subscriber"):
        assert ids.build_topic("/minted_from_a_remap") not in subjects(fired(bench.atlas, name))


@pytest.mark.parametrize("name", ("topic.no_subscriber", "topic.no_publisher"))
def test_topic_connectivity_needs_a_probe_that_listed_every_endpoint(
    bench: Bench, name: str
) -> None:
    """One node reported with no endpoints means the missing end could be on that node."""
    bench.observed(ids.build_node("/half_read"), "half_read")
    assert topic_rules.probe_enumerated_endpoints(bench.atlas, bench.profile) is False
    assert fired(bench.atlas, name) == []


@pytest.mark.parametrize("name", ("topic.no_subscriber", "topic.no_publisher"))
def test_topic_connectivity_has_a_stable_id(name: str) -> None:
    left = fired(Bench().atlas, name)[0]
    right = fired(Bench().atlas, name)[0]
    assert left.id == right.id
    assert left.fingerprint == right.fingerprint


def test_coverage_reports_what_the_atlas_could_see(bench: Bench) -> None:
    seen = topic_rules.coverage(bench.atlas, bench.profile)
    assert seen.observed is True
    assert seen.nodes_with_code == 2
    assert 0.0 < seen.coded_fraction < 1.0


# ---------------------------------------------------------------------------
# qos.differ and type.differ


def test_qos_differ_fires_on_two_stated_depths(bench: Bench) -> None:
    found = fired(bench.atlas, "qos.differ")
    assert len(found) == 1
    assert found[0].subject_key == bench.qos_edge.id
    assert "depth" in found[0].detail and "10" in found[0].say


def test_qos_differ_is_silent_when_one_side_states_nothing(bench: Bench) -> None:
    edge = bench.endpoint(
        model.EdgeKind.PUBLISHES.value,
        BRIDGE,
        "/vehicle/quiet",
        CODED,
        key="quiet",
        values={"qos": qos(depth=10)},
    )
    bench.endpoint(
        model.EdgeKind.PUBLISHES.value,
        BRIDGE,
        "/vehicle/quiet",
        OBSERVED,
        key="quiet",
        values={"qos": qos()},
    )
    assert edge.id not in subjects(fired(bench.atlas, "qos.differ"))


def test_qos_differ_never_expands_a_preset_into_numbers(bench: Bench) -> None:
    edge = bench.endpoint(
        model.EdgeKind.PUBLISHES.value,
        BRIDGE,
        "/vehicle/sensor",
        CODED,
        key="sensor",
        values={"qos": qos(form="preset", preset="sensor_data")},
    )
    bench.endpoint(
        model.EdgeKind.PUBLISHES.value,
        BRIDGE,
        "/vehicle/sensor",
        OBSERVED,
        key="sensor",
        values={"qos": qos(depth=5, reliability="best_effort")},
    )
    assert edge.id not in subjects(fired(bench.atlas, "qos.differ"))


def test_qos_differ_is_silent_on_a_wrapper_endpoint(bench: Bench) -> None:
    edge = bench.endpoint(
        model.EdgeKind.PUBLISHES.value,
        BRIDGE,
        "/tf_static",
        CODED,
        key="wrapped",
        values={"qos": qos(depth=1), "via_known_wrapper": True},
    )
    bench.endpoint(
        model.EdgeKind.PUBLISHES.value,
        BRIDGE,
        "/tf_static",
        OBSERVED,
        key="wrapped",
        values={"qos": qos(depth=10)},
    )
    assert edge.id not in subjects(fired(bench.atlas, "qos.differ"))


def test_qos_differ_has_a_stable_id() -> None:
    left = fired(Bench().atlas, "qos.differ")[0]
    right = fired(Bench().atlas, "qos.differ")[0]
    assert left.id == right.id
    assert left.fingerprint == right.fingerprint


def test_type_differ_reads_two_spellings_of_one_type_as_one(bench: Bench) -> None:
    assert fired(bench.atlas, "type.differ") == []
    assert topic_rules.type_key("nav_msgs::msg::Odometry") == topic_rules.type_key(
        "nav_msgs/msg/Odometry"
    )


def test_type_differ_fires_on_two_types(bench: Bench) -> None:
    edge = bench.endpoint(
        model.EdgeKind.SUBSCRIBES.value,
        BRIDGE,
        "/joy",
        CODED,
        key="joy",
        values={"type": "geometry_msgs/msg/Twist"},
    )
    bench.endpoint(
        model.EdgeKind.SUBSCRIBES.value,
        BRIDGE,
        "/joy",
        OBSERVED,
        key="joy",
        values={"type": "geometry_msgs/msg/TwistStamped"},
    )
    found = fired(bench.atlas, "type.differ")
    assert subjects(found) == [edge.id]
    assert found[0].severity == model.Severity.ERROR.value


def test_type_differ_is_silent_on_a_spelling_it_cannot_read(bench: Bench) -> None:
    edge = bench.endpoint(
        model.EdgeKind.SUBSCRIBES.value,
        BRIDGE,
        "/mystery",
        CODED,
        key="mystery",
        values={"type": "MessageT"},
    )
    bench.endpoint(
        model.EdgeKind.SUBSCRIBES.value,
        BRIDGE,
        "/mystery",
        OBSERVED,
        key="mystery",
        values={"type": "std_msgs/msg/Empty"},
    )
    assert edge.id not in subjects(fired(bench.atlas, "type.differ"))


def test_type_differ_has_a_stable_id() -> None:
    def build() -> model.Finding:
        bench = Bench()
        bench.endpoint(
            model.EdgeKind.SUBSCRIBES.value,
            BRIDGE,
            "/joy",
            CODED,
            key="joy",
            values={"type": "geometry_msgs/msg/Twist"},
        )
        bench.endpoint(
            model.EdgeKind.SUBSCRIBES.value,
            BRIDGE,
            "/joy",
            OBSERVED,
            key="joy",
            values={"type": "geometry_msgs/msg/TwistStamped"},
        )
        return fired(bench.atlas, "type.differ")[0]

    assert build().id == build().id
    assert build().fingerprint == build().fingerprint


# ---------------------------------------------------------------------------
# rate.below_declared


def test_rate_below_declared_fires_well_under_the_stated_rate(bench: Bench) -> None:
    found = fired(bench.atlas, "rate.below_declared")
    assert subjects(found) == [bench.rate_edge.id]
    assert "20 Hz" in found[0].say
    assert found[0].measured == "5 Hz"
    assert "Measured: 5 Hz." in found[0].spoken


def test_rate_below_declared_keeps_the_measurement_out_of_compared_bytes() -> None:
    """Two probes of one slow system must not produce two fingerprints or two atlases."""
    first = fired(Bench().atlas, "rate.below_declared")[0]
    bench = Bench()
    for edge in bench.atlas.edges.values():
        for fact in edge.facts:
            if fact.layer == OBSERVED and isinstance(fact.values.get("volatile"), dict):
                fact.values["volatile"]["rate_hz"] = 4.5
    second = fired(bench.atlas, "rate.below_declared")[0]
    assert first.fingerprint == second.fingerprint
    assert first.measured != second.measured
    stripped = [model.strip_volatile(item.to_json()) for item in (first, second)]
    assert stripped[0] == stripped[1]


def test_rate_below_declared_is_silent_on_a_near_miss(bench: Bench) -> None:
    for edge in bench.atlas.edges.values():
        for fact in edge.facts:
            if fact.layer == OBSERVED and isinstance(fact.values.get("volatile"), dict):
                fact.values["volatile"]["rate_hz"] = 18.0
    assert fired(bench.atlas, "rate.below_declared") == []


def test_rate_below_declared_holds_the_measurement_against_the_stronger_layer(
    bench: Bench,
) -> None:
    """A launch parameter that lowers a timer is the rate the system is meant to hold."""
    bench.endpoint(
        model.EdgeKind.PUBLISHES.value,
        BRIDGE,
        "/vehicle/loop",
        CODED,
        key="loop",
        values={"rate_hz": 20.0},
    )
    bench.endpoint(
        model.EdgeKind.PUBLISHES.value,
        BRIDGE,
        "/vehicle/loop",
        DECLARED,
        key="loop",
        values={"rate_hz": 5.0},
    )
    bench.endpoint(
        model.EdgeKind.PUBLISHES.value,
        BRIDGE,
        "/vehicle/loop",
        OBSERVED,
        key="loop",
        values={"volatile": {"rate_hz": 5.5}},
    )
    assert "/vehicle/loop" not in [
        item.subject.dst for item in fired(bench.atlas, "rate.below_declared")
    ]


def test_rate_below_declared_is_silent_when_nothing_states_a_rate(bench: Bench) -> None:
    edge = bench.endpoint(
        model.EdgeKind.PUBLISHES.value,
        BRIDGE,
        "/vehicle/free",
        OBSERVED,
        key="free",
        values={"volatile": {"rate_hz": 0.5}},
    )
    assert edge.id not in subjects(fired(bench.atlas, "rate.below_declared"))


def test_rate_below_declared_has_a_stable_id() -> None:
    left = fired(Bench().atlas, "rate.below_declared")[0]
    right = fired(Bench().atlas, "rate.below_declared")[0]
    assert left.id == right.id
    assert left.fingerprint == right.fingerprint


# ---------------------------------------------------------------------------
# endpoint.observed_only


def test_endpoint_observed_only_fires_when_the_source_was_read(bench: Bench) -> None:
    found = fired(bench.atlas, "endpoint.observed_only")
    assert subjects(found) == [bench.only_edge.id]
    assert found[0].severity == model.Severity.INFO.value


def test_endpoint_observed_only_is_silent_on_the_parameter_services(bench: Bench) -> None:
    """rclcpp and rclpy create six of these on every node, under the node's own namespace.

    The wrapper table spells them `~/get_parameters`, so an absolute name set never matches
    them and each one would be a finding on every node whose source was read.
    """
    served = ids.build_service("/io_bridge/get_parameters")
    bench.builder.node(served)
    edge = bench.builder.edge(
        model.EdgeKind.SERVES.value,
        BRIDGE,
        served,
        key="params",
        layer=OBSERVED,
        profile=bench.profile,
        values={},
    )
    assert topic_rules.is_quiet("/io_bridge/get_parameters") is True
    assert edge.id not in subjects(fired(bench.atlas, "endpoint.observed_only"))


def test_endpoint_observed_only_is_silent_when_the_source_was_never_read(bench: Bench) -> None:
    """/vehicle/spare hangs off a node with no coded fact, which is coverage and not drift."""
    assert ids.build_topic("/vehicle/spare") not in [
        item.subject.dst for item in fired(bench.atlas, "endpoint.observed_only")
    ]


def test_endpoint_observed_only_is_silent_on_a_known_wrapper(bench: Bench) -> None:
    assert ids.build_topic("/tf") not in [
        item.subject.dst for item in fired(bench.atlas, "endpoint.observed_only")
    ]


def test_endpoint_observed_only_has_a_stable_id() -> None:
    left = fired(Bench().atlas, "endpoint.observed_only")[0]
    right = fired(Bench().atlas, "endpoint.observed_only")[0]
    assert left.id == right.id
    assert left.fingerprint == right.fingerprint


# ---------------------------------------------------------------------------
# the whole family


def test_every_finding_is_addressed_to_something_the_atlas_holds(bench: Bench) -> None:
    for finding in node_rules.check(bench.atlas) + topic_rules.check(bench.atlas):
        if isinstance(finding.subject, model.EdgeRef):
            assert finding.subject.id in bench.atlas.edges
        else:
            assert finding.subject in bench.atlas.entities
        assert finding.title and finding.detail and finding.fix and finding.say
        assert ":~" not in finding.say and ":~" not in finding.title


def test_the_builder_accepts_every_finding_this_family_produces(bench: Bench) -> None:
    """The builder refuses a synthetic id in prose and a duplicate finding id."""
    for finding in node_rules.check(bench.atlas) + topic_rules.check(bench.atlas):
        bench.builder.finding(finding)


def test_two_builds_of_the_family_produce_the_same_report() -> None:
    def report() -> list[str]:
        atlas = Bench().atlas
        return [item.id for item in node_rules.check(atlas) + topic_rules.check(atlas)]

    assert report() == report()


# ---------------------------------------------------------------------------
# qos.incompatible


def _pair(bench: Bench, topic: str, offered: dict, requested: dict) -> None:
    bench.endpoint(
        model.EdgeKind.PUBLISHES.value, BRIDGE, topic, CODED, key="pub", values={"qos": offered}
    )
    bench.endpoint(
        model.EdgeKind.SUBSCRIBES.value,
        SUPERVISOR,
        topic,
        CODED,
        key="sub",
        values={"qos": requested},
    )


def test_qos_incompatible_fires_on_a_reliable_subscription_to_sensor_data(bench: Bench) -> None:
    _pair(
        bench,
        "/lidar/filtered",
        qos(form="preset", preset="sensor_data"),
        qos(depth=10, reliability="reliable"),
    )
    found = [f for f in fired(bench.atlas, "qos.incompatible") if "filtered" in f.subject_key]
    assert len(found) == 1
    assert found[0].severity == "error"
    assert "requesting reliable" in found[0].say and "best effort" in found[0].say


def test_qos_incompatible_fires_on_transient_local_from_volatile(bench: Bench) -> None:
    _pair(
        bench,
        "/map_latched",
        qos(depth=1, reliability="reliable", durability="volatile"),
        qos(depth=1, reliability="reliable", durability="transient_local"),
    )
    found = [f for f in fired(bench.atlas, "qos.incompatible") if "latched" in f.subject_key]
    assert len(found) == 1 and "transient local" in found[0].say


@pytest.mark.parametrize(
    ("offered", "requested"),
    [
        (qos(form="depth", depth=10), qos(depth=10, reliability="reliable")),
        (qos(form="preset", preset="sensor_data"), qos(form="preset", preset="sensor_data")),
        (qos(form="preset", preset="system_default"), qos(depth=10, reliability="reliable")),
    ],
)
def test_qos_incompatible_is_silent_on_a_matching_or_unknown_pair(
    bench: Bench, offered: dict, requested: dict
) -> None:
    _pair(bench, "/lidar/quiet", offered, requested)
    assert not [f for f in fired(bench.atlas, "qos.incompatible") if "quiet" in f.subject_key]


def test_qos_incompatible_meets_a_publisher_stated_for_every_profile(bench: Bench) -> None:
    """A `*` fact holds in every profile, so it meets a subscription stated for one."""
    topic = ids.build_topic("/lidar/shared")
    bench.builder.node(topic)
    bench.builder.edge(
        model.EdgeKind.PUBLISHES.value,
        BRIDGE,
        topic,
        key="pub",
        layer=CODED,
        profile=model.ANY_PROFILE,
        values={"qos": qos(form="preset", preset="sensor_data")},
        evidence=[SOURCE],
    )
    bench.endpoint(
        model.EdgeKind.SUBSCRIBES.value,
        SUPERVISOR,
        "/lidar/shared",
        CODED,
        key="sub",
        values={"qos": qos(depth=10, reliability="reliable")},
    )
    found = [f for f in fired(bench.atlas, "qos.incompatible") if "shared" in f.subject_key]
    assert len(found) == 1
    assert found[0].profiles == [bench.profile]


def test_qos_incompatible_does_not_claim_silence_when_another_publisher_matches(
    bench: Bench,
) -> None:
    _pair(
        bench,
        "/lidar/mixed",
        qos(form="preset", preset="sensor_data"),
        qos(depth=10, reliability="reliable"),
    )
    other = ids.build_node("/second_bridge")
    bench.builder.node(other)
    bench.endpoint(
        model.EdgeKind.PUBLISHES.value,
        other,
        "/lidar/mixed",
        CODED,
        key="pub2",
        values={"qos": qos(depth=10, reliability="reliable")},
    )
    found = [f for f in fired(bench.atlas, "qos.incompatible") if "mixed" in f.subject_key]
    assert len(found) == 1
    assert "never receives" not in found[0].title
    assert "no message arrives" not in found[0].say
    assert "/second_bridge" in found[0].say


def test_qos_incompatible_keeps_its_id_across_two_runs() -> None:
    def run() -> list[str]:
        bench = Bench()
        _pair(
            bench,
            "/lidar/filtered",
            qos(form="preset", preset="sensor_data"),
            qos(depth=10, reliability="reliable"),
        )
        return [f.id for f in fired(bench.atlas, "qos.incompatible")]

    assert run() == run()
