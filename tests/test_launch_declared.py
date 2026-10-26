"""The declared layer, launch side.

The walks come from two places on purpose: `tests/fixtures/expected/launch/*.json`, which is
the committed answer for every hard case, and hand built records for the shapes no fixture
file carries (an include graph, a randomised name family, an in launch lifecycle transition).
Neither path needs the launch resolver, so this suite does not wait on it.
"""

from __future__ import annotations

import json
import tempfile
import threading
import time
from pathlib import Path

import pytest

from rosio import ids, model, schema
from rosio.collect import CollectContext, get, launch_declared
from rosio.config import Config, Entrypoint, ProfileSpec, RootSpec
from rosio.drift import build as drift_build
from rosio.resolve import _launch_child as launch_child
from rosio.resolve import identity, records
from rosio.workspace import Workspace

WS = Path(__file__).parent / "fixtures" / "ws_launch"
EXPECTED = Path(__file__).parent / "fixtures" / "expected" / "launch"

LAYER = model.Layer.DECLARED.value

# Every expected walk that resolves or records something, with the package that owns it.
CASES = {
    "composition": "lx_bringup",
    "conditions.default": "lx_bringup",
    "conditions.enabled": "lx_bringup",
    "default_name": "lx_bringup",
    "env_required.missing": "lx_bringup",
    "env_required.set": "lx_bringup",
    "event_gate": "lx_bringup",
    "exec_process": "lx_bringup",
    "group_scope": "lx_bringup",
    "include_child": "lx_bringup",
    "include_parent": "lx_bringup",
    "legacy_ros1": "lx_bringup",
    "nav_stack": "lx_bringup",
    "no_default_arg.missing": "lx_bringup",
    "no_default_arg.supplied": "lx_bringup",
    "opaque_probe.no_map": "lx_bringup",
    "opaque_probe.with_map": "lx_bringup",
    "param_order": "lx_bringup",
    "plain_node": "lx_bringup",
    "rewritten": "lx_bringup",
    "sensors_only": "lx_sensors",
    "timer_chain": "lx_bringup",
}


# ---------------------------------------------------------------------------
# loading the committed answers


def _abs(relative: str) -> str:
    return str(WS / relative)


def _remap(payload: dict) -> records.Remap:
    return records.Remap(
        frm=payload["from"],
        to=payload["to"],
        scope=payload.get("scope", "node"),
    )


def _param(payload: dict) -> records.ParamSource:
    return records.ParamSource(
        order=payload["order"],
        kind=payload["kind"],
        path=_abs(payload["path"]) if payload.get("path") else None,
        source_path=_abs(payload["source_path"]) if payload.get("source_path") else None,
        root_key=payload.get("root_key"),
        rewrites=tuple((key, value) for key, value in payload.get("rewrites") or []),
        values=tuple((key, value) for key, value in payload.get("values") or []),
        note=payload.get("note", ""),
    )


def _gate(payload: dict) -> records.Gate:
    return records.Gate(
        mechanism=payload["mechanism"],
        expression=payload.get("expression", ""),
        period=payload.get("period"),
        target=payload.get("target", ""),
        outcome=payload.get("outcome", ""),
    )


def _node(payload: dict) -> records.ResolvedNode:
    return records.ResolvedNode(
        pkg=payload["pkg"],
        executable=payload["executable"],
        namespace=payload["namespace"],
        form=payload["form"],
        name=payload.get("name"),
        name_source=payload.get("name_source", "unknown"),
        container=payload.get("container"),
        plugin_class=payload.get("plugin_class"),
        remaps=tuple(_remap(item) for item in payload.get("remaps") or []),
        params=tuple(_param(item) for item in payload.get("params") or []),
        argv=tuple(payload.get("argv") or []),
        gates=tuple(_gate(item) for item in payload.get("gates") or []),
        conditions=tuple(payload.get("conditions") or []),
    )


def walk_for(case: str, profile: str) -> records.LaunchWalk:
    """One committed walk, with every path made absolute in the fixture workspace."""
    payload = json.loads((EXPECTED / f"{case}.json").read_text(encoding="utf-8"))
    return records.LaunchWalk(
        entry_path=_abs(payload["entry"]),
        profile=profile,
        args=tuple((key, value) for key, value in payload.get("args") or []),
        env=tuple((key, value) for key, value in payload.get("env") or []),
        nodes=[_node(item) for item in payload.get("nodes") or []],
        arg_declarations=[
            records.ArgDeclaration(
                name=item["name"],
                default=item.get("default"),
                description=item.get("description", ""),
            )
            for item in payload.get("arg_declarations") or []
        ],
        exceptions=[
            records.WalkException(
                path=_abs(item["path"]),
                phase=item["phase"],
                exc_type=item["exc_type"],
                message=item["message"],
            )
            for item in payload.get("exceptions") or []
        ],
        predicates=dict(payload.get("predicates") or {}),
        loadable=payload.get("loadable", True),
        strategy=payload.get("strategy", "executed"),
    )


def entry_for(case: str) -> Entrypoint:
    payload = json.loads((EXPECTED / f"{case}.json").read_text(encoding="utf-8"))
    target = Path(payload["entry"]).name
    return Entrypoint(
        verb="launch",
        pkg=CASES[case],
        target=target,
        args=tuple((key, value) for key, value in payload.get("args") or []),
    )


# ---------------------------------------------------------------------------
# harness


@pytest.fixture(scope="module")
def ws() -> Workspace:
    roots = (
        RootSpec(kind="src", path=str(WS / "src")),
        RootSpec(kind="install", path=str(WS / "install")),
    )
    return Workspace.discover(roots)


def build(ws: Workspace, cases: dict[str, list[str]]) -> model.Atlas:
    """Run the collector over one profile per entry in `cases`, with committed walks."""
    profiles = tuple(
        ProfileSpec(
            slug=slug,
            label=slug,
            entrypoints=tuple(entry_for(case) for case in names),
        )
        for slug, names in cases.items()
    )
    cfg = Config(
        roots=tuple(RootSpec(kind=spec.kind, path=spec.path) for spec in ws.roots),
        profiles=profiles,
    )
    builder = model.AtlasBuilder()
    ctx = CollectContext(ws=ws, cfg=cfg, builder=builder)
    for slug, names in cases.items():
        for case in names:
            ctx.declared.walks.append(walk_for(case, slug))
    get(launch_declared.NAME).collect(ctx)
    return builder.build()


# One case per launch file: a profile launches a file once, with one argument set.
ONE_PER_FILE = sorted(
    set(CASES)
    - {"conditions.default", "opaque_probe.no_map", "env_required.set", "no_default_arg.missing"}
)

ALL_CASES = {"everything": ONE_PER_FILE}


@pytest.fixture(scope="module")
def full(ws: Workspace) -> model.Atlas:
    return build(ws, ALL_CASES)


def node_ids(atlas: model.Atlas) -> list[str]:
    return [entity.id for entity in atlas.by_kind("node")]


def edges_of(atlas: model.Atlas, kind: str) -> list[model.Edge]:
    return [edge for edge in atlas.edges.values() if edge.kind == kind]


# ---------------------------------------------------------------------------
# the committed answers load


def test_every_expected_walk_loads() -> None:
    for case in CASES:
        walk = walk_for(case, "everything")
        assert walk.entry_path.startswith(str(WS))
        assert Path(walk.entry_path).is_file()


def test_registered() -> None:
    collector = get(launch_declared.NAME)
    assert collector.layer == LAYER
    assert collector.name == launch_declared.NAME


# ---------------------------------------------------------------------------
# contract level


def test_atlas_validates(full: model.Atlas) -> None:
    assert schema.validate(full.to_json()) == []


def test_two_runs_are_byte_identical(ws: Workspace) -> None:
    first = build(ws, ALL_CASES)
    second = build(ws, ALL_CASES)
    assert first.canonical_bytes() == second.canonical_bytes()


def test_recompute_is_a_fixpoint(ws: Workspace) -> None:
    profiles = (ProfileSpec(slug="p", entrypoints=(entry_for("nav_stack"),)),)
    cfg = Config(
        roots=tuple(RootSpec(kind=spec.kind, path=spec.path) for spec in ws.roots),
        profiles=profiles,
    )
    builder = model.AtlasBuilder()
    ctx = CollectContext(ws=ws, cfg=cfg, builder=builder)
    ctx.declared.walks.append(walk_for("nav_stack", "p"))
    get(launch_declared.NAME).collect(ctx)
    once = builder.build().canonical_bytes()
    twice = builder.recompute().canonical_bytes()
    assert once == twice


def test_counts_are_stable(full: model.Atlas) -> None:
    counters = full.counters
    assert counters["entities_by_kind"] == {
        "file": 18,
        "launchent": 29,
        "launchfile": 18,
        "node": 27,
        "pkg": 2,
        "profile": 1,
        "run": 1,
        "topic": 1,
    }
    assert counters["edges_by_kind"] == {
        "contains": 65,
        "gates": 6,
        "launches": 56,
        "manages_lifecycle": 2,
        "remaps": 1,
    }
    assert counters["resolution_rate"] == {
        "node_name": 0.964286,
        "other": 0.615385,
        "topic_name": 1.0,
    }
    # Two of the eighteen entry points could not produce a node set at all, the one node left
    # without a name is the executable that is in no name table, and the eight remaining
    # blind spots are remap rules whose endpoint kind nothing in this layer states.
    assert counters["unresolved"] == {"node_name": 1, "other": 10}
    # The raise is a run row here, and the `launch.raised` drift rule turns it into a finding.
    assert counters["findings"] == {"error": 0, "info": 0, "warn": 1}


# ---------------------------------------------------------------------------
# profiles name their entry points


def test_profile_names_its_entry_points(ws: Workspace) -> None:
    atlas = build(ws, {"sensing": ["plain_node", "event_gate"]})
    profile = atlas.get(ids.build_profile("sensing"))
    assert profile is not None
    assert profile.attrs["entry"] == [
        ids.build_launchfile("lx_bringup", "launch/event_gate.launch.py"),
        ids.build_launchfile("lx_bringup", "launch/plain_node.launch.py"),
    ]
    assert profile.attrs["entry_kind"] == "launch"
    fact = profile.fact(LAYER, ids.build_profile("sensing"))
    assert fact is not None
    assert fact.values["entrypoints"] == [
        "ros2 launch lx_bringup plain_node.launch.py",
        "ros2 launch lx_bringup event_gate.launch.py",
    ]
    assert fact.values["declared_nodes"] == 3


def test_run_record_carries_the_inputs(ws: Workspace) -> None:
    atlas = build(ws, {"probe": ["opaque_probe.with_map"]})
    runs = atlas.by_kind("run")
    assert len(runs) == 1
    attrs = runs[0].attrs
    assert attrs["profile"] == ids.build_profile("probe")
    assert set(attrs["roots"]) == {"src", "install"}
    assert isinstance(attrs["predicates"].pop("launch_sandbox:bwrap"), bool)
    assert attrs["predicates"] == {
        "lx_bringup/launch/opaque_probe.launch.py:map_base.posegraph": True
    }
    assert ids.parse_run(runs[0].id).input_digest == attrs["input_digest"]
    assert atlas.layers[LAYER].present is True
    assert atlas.layers[LAYER].runs == [runs[0].id]


def test_the_input_digest_moves_with_the_predicate(ws: Workspace) -> None:
    with_map = build(ws, {"probe": ["opaque_probe.with_map"]}).by_kind("run")[0]
    no_map = build(ws, {"probe": ["opaque_probe.no_map"]}).by_kind("run")[0]
    assert with_map.attrs["input_digest"] != no_map.attrs["input_digest"]


# ---------------------------------------------------------------------------
# launch files


def test_unloadable_file_is_refused_and_recorded(ws: Workspace) -> None:
    atlas = build(ws, {"legacy": ["legacy_ros1"]})
    launchfile = atlas.get(ids.build_launchfile("lx_bringup", "launch/legacy_ros1.launch"))
    assert launchfile is not None
    assert launchfile.attrs["loadable"] is False
    exceptions = atlas.by_kind("run")[0].attrs["exceptions"]
    assert [row["note"] for row in exceptions] == ["launch.unloadable"]
    assert not atlas.by_kind("node")


def test_a_raised_description_is_recorded_and_the_file_stays_loadable(ws: Workspace) -> None:
    atlas = build(ws, {"env": ["env_required.missing", "plain_node"]})
    launchfile = atlas.get(ids.build_launchfile("lx_bringup", "launch/env_required.launch.py"))
    assert launchfile is not None
    assert launchfile.attrs["loadable"] is True
    exceptions = atlas.by_kind("run")[0].attrs["exceptions"]
    assert [(row["exc_type"], row["note"]) for row in exceptions] == [
        ("RuntimeError", "launch.raised")
    ]
    # The other entry point still contributed its node set.
    assert ids.build_node("/sensors/lidar_driver") in node_ids(atlas)


def test_a_required_argument_is_reported_not_skipped(ws: Workspace) -> None:
    """The walk stopped at a declaration with no default, and says so.

    It writes no argument list and no node count: both are lower bounds after an early stop,
    and the fact slot is written once, so a partial list would be a wrong answer rather than
    a short one.
    """
    atlas = build(ws, {"needs_args": ["no_default_arg.missing"]})
    launchfile = atlas.get(ids.build_launchfile("lx_bringup", "launch/no_default_arg.launch.py"))
    fact = launchfile.fact(LAYER, ids.build_profile("needs_args"))
    assert fact.values["partial"] is True
    assert fact.values["stopped_at"] == ["argument"]
    assert "arg_declarations" not in fact.values
    assert "required_args" not in fact.values
    assert "node_count" not in fact.values
    exceptions = atlas.by_kind("run")[0].attrs["exceptions"]
    assert exceptions[0]["phase"] == "argument"
    assert "params_file" in exceptions[0]["message"]


def test_a_partial_walk_does_not_poison_the_slot_a_complete_one_fills(ws: Workspace) -> None:
    """One profile launching the same file twice, once without the argument it needs.

    The complete walk owns the argument list. The partial one adds its own keys and refuses
    to restate a list it could not finish, so the second write adds keys instead of raising.
    """
    atlas = build(ws, {"twice": ["no_default_arg.missing", "no_default_arg.supplied"]})
    launchfile = atlas.get(ids.build_launchfile("lx_bringup", "launch/no_default_arg.launch.py"))
    values = launchfile.fact(LAYER, ids.build_profile("twice")).values
    assert values["arg_declarations"] == ["params_file"]
    assert values["partial"] is True
    rows = atlas.by_kind("run")[0].attrs["exceptions"]
    assert not [row for row in rows if row["phase"] == "visit"]


def test_a_missing_launch_file_is_reported(ws: Workspace) -> None:
    cfg = Config(
        roots=tuple(RootSpec(kind=spec.kind, path=spec.path) for spec in ws.roots),
        profiles=(
            ProfileSpec(
                slug="ghost",
                entrypoints=(Entrypoint(verb="launch", pkg="lx_bringup", target="rtabmap.py"),),
            ),
        ),
    )
    builder = model.AtlasBuilder()
    get(launch_declared.NAME).collect(CollectContext(ws=ws, cfg=cfg, builder=builder))
    atlas = builder.build()
    profile = atlas.get(ids.build_profile("ghost"))
    fact = profile.fact(LAYER, ids.build_profile("ghost"))
    assert fact.values["missing_targets"] == ["ros2 launch lx_bringup rtabmap.py"]
    assert atlas.by_kind("run")[0].attrs["exceptions"][0]["exc_type"] == "MissingLaunchFile"


# ---------------------------------------------------------------------------
# node identity


def test_a_literal_name_and_namespace_give_a_real_id(full: model.Atlas) -> None:
    entity = full.get(ids.build_node("/sensors/lidar_driver"))
    assert entity is not None
    assert entity.name_source == "literal"
    fact = entity.fact(LAYER, ids.build_profile("everything"))
    assert fact.values["pkg"] == "lx_sensors"
    assert fact.values["namespace"] == "/sensors"
    assert fact.values["name"] == "lidar_driver"


def test_an_unnamed_node_is_synthetic_and_says_why(full: model.Atlas) -> None:
    synthetic = [
        entity for entity in full.by_kind("node") if entity.name_source == "executable_default"
    ]
    assert [entity.id for entity in synthetic] == [
        ids.build_node_synthetic(
            "lx_sensors",
            "imu_driver",
            "/",
            launch_declared._role(
                records.ResolvedNode(
                    pkg="lx_sensors",
                    executable="imu_driver",
                    namespace="/",
                    name=None,
                    name_source="executable_default",
                    params=(
                        records.ParamSource(
                            order=0,
                            kind="repo_file",
                            path=str(WS / "src/lx_bringup/config/imu_driver.yaml"),
                        ),
                    ),
                )
            ),
        )
    ]
    entry = [item for item in full.unresolved if item.owner == synthetic[0].id]
    assert [item.want for item in entry] == ["node_name"]
    assert entry[0].id in synthetic[0].unresolved


def test_an_unstated_namespace_is_the_root_on_an_executed_walk(full: model.Atlas) -> None:
    """launch_ros writes its namespace sentinel into the expanded field when the file states
    nothing, which is a statement that the node runs at the root, not a blind spot."""
    named = [
        entity
        for entity in full.by_kind("node")
        if entity.name_source == "literal" and not ids.is_synthetic(entity.id)
    ]
    assert [entity.id for entity in named if entity.id == ids.build_node("/late_node")]
    entity = full.get(ids.build_node("/late_node"))
    fact = entity.fact(LAYER, ids.build_profile("everything"))
    assert fact.values["namespace"] == "/"
    launchent = next(edge.src for edge in full.in_edges(entity.id) if edge.kind == "launches")
    assert full.get(launchent).fact(LAYER, ids.build_profile("everything")) is not None


def test_the_namespace_sentinel_is_the_root_and_not_a_stated_namespace(ws: Workspace) -> None:
    """A record that still carries the sentinel resolves to the root and says launch stated
    nothing, which is what keeps a sentinel out of every id."""
    walk = records.LaunchWalk(
        entry_path=str(WS / "src/lx_bringup/launch/plain_node.launch.py"),
        profile="sentinel",
        nodes=[
            records.ResolvedNode(
                pkg="lx_sensors",
                executable="scan_relay",
                namespace="/",
                name="relay",
                name_source="literal",
                attrs={"namespace_sentinel": launch_declared.UNSPECIFIED_NAMESPACE},
            )
        ],
    )
    atlas = _run_walk(ws, walk, "sentinel", "plain_node.launch.py")
    entity = atlas.get(ids.build_node("/relay"))
    assert entity is not None
    launchent = next(edge.src for edge in atlas.in_edges(entity.id) if edge.kind == "launches")
    site = atlas.get(launchent).fact(LAYER, ids.build_profile("sentinel"))
    assert site.values["namespace"] == "/"
    assert site.values["namespace_stated"] is False


def test_a_walk_that_was_not_executed_keeps_the_namespace_unknown(ws: Workspace) -> None:
    """An ast read saw no sentinel and no namespace, so it knows nothing about one."""
    walk = records.LaunchWalk(
        entry_path=str(WS / "src/lx_bringup/launch/plain_node.launch.py"),
        profile="read",
        strategy="ast",
        nodes=[
            records.ResolvedNode(
                pkg="lx_sensors",
                executable="scan_relay",
                namespace="",
                name="relay",
                name_source="literal",
            )
        ],
    )
    atlas = _run_walk(ws, walk, "read", "plain_node.launch.py")
    entity = atlas.by_kind("node")[0]
    assert ids.is_synthetic(entity.id)
    assert "namespace" not in entity.fact(LAYER, ids.build_profile("read")).values


def test_two_unnamed_nodes_in_one_namespace_stay_two_nodes(ws: Workspace) -> None:
    def unnamed(executable: str, topic: str) -> records.ResolvedNode:
        return records.ResolvedNode(
            pkg="tf2_ros",
            executable="static_transform_publisher",
            namespace="/",
            argv=("--frame-id", "base_link", "--child-frame-id", topic),
        )

    walk = records.LaunchWalk(
        entry_path=str(WS / "src/lx_bringup/launch/plain_node.launch.py"),
        profile="tf",
        nodes=[
            unnamed("static_transform_publisher", "os_sensor"),
            unnamed("static_transform_publisher", "gps"),
        ],
    )
    atlas = _run_walk(ws, walk, "tf", "plain_node.launch.py")
    nodes = atlas.by_kind("node")
    assert len(nodes) == 2
    assert {entity.name_source for entity in nodes} == {"runtime_randomised"}
    assert len({entity.id for entity in nodes}) == 2


def test_the_name_table_is_used_and_cited(ws: Workspace) -> None:
    """A third party executable with no name= joins on the committed table row."""
    walk = records.LaunchWalk(
        entry_path=str(WS / "src/lx_bringup/launch/nav_stack.launch.py"),
        profile="third_party",
        nodes=[
            records.ResolvedNode(
                pkg="nav2_controller",
                executable="controller_server",
                namespace="/",
                name=None,
                name_source="executable_default",
            )
        ],
    )
    atlas = _run_walk(ws, walk, "third_party", "nav_stack.launch.py")
    entity = atlas.get(ids.build_node("/controller_server"))
    assert entity is not None
    assert entity.name_source == "executable_default"
    fact = entity.fact(LAYER, ids.build_profile("third_party"))
    table = ids.build_file(ids.REPO_PKG, "rosio/resolve/known_names.py")
    assert table in [source.file for source in fact.evidence]
    assert table in atlas.entities
    assert [source.locator for source in fact.evidence if source.file == table] == [
        "nav2_controller/controller_server"
    ]


def test_a_randomised_table_row_never_joins_on_its_name(ws: Workspace) -> None:
    """static_transform_publisher is in the table only to say that its name is not identity."""
    walk = records.LaunchWalk(
        entry_path=str(WS / "src/lx_bringup/launch/nav_stack.launch.py"),
        profile="frames",
        nodes=[
            records.ResolvedNode(
                pkg="tf2_ros",
                executable="static_transform_publisher",
                namespace="/",
                argv=("--frame-id", "base_link", "--child-frame-id", "os_sensor"),
            )
        ],
    )
    atlas = _run_walk(ws, walk, "frames", "nav_stack.launch.py")
    entity = atlas.by_kind("node")[0]
    assert ids.is_synthetic(entity.id)
    assert entity.name_source == "runtime_randomised"
    assert [item.reason for item in atlas.unresolved] == [
        "the runtime appends or randomises this executable's node name"
    ]


def test_a_named_static_publisher_still_joins_on_its_name(ws: Workspace) -> None:
    """A stated name is the runtime name even in a randomised family."""
    walk = records.LaunchWalk(
        entry_path=str(WS / "src/lx_bringup/launch/nav_stack.launch.py"),
        profile="tie",
        nodes=[
            records.ResolvedNode(
                pkg="tf2_ros",
                executable="static_transform_publisher",
                namespace="/",
                name="lio_body_frame_tie",
                name_source="literal",
                argv=("--frame-id", "os_sensor", "--child-frame-id", "body"),
            )
        ],
    )
    atlas = _run_walk(ws, walk, "tie", "nav_stack.launch.py")
    entity = atlas.get(ids.build_node("/lio_body_frame_tie"))
    assert entity is not None
    assert entity.name_source == "literal"


def test_the_sentinels_never_reach_an_id(ws: Workspace) -> None:
    """`node_name` on a launch_ros Node returns the namespace sentinel joined to the name.
    The tail is the name, the head means the root, and neither string may become an id."""
    walk = records.LaunchWalk(
        entry_path=str(WS / "src/lx_bringup/launch/plain_node.launch.py"),
        profile="sentinel",
        nodes=[
            records.ResolvedNode(
                pkg="slam_toolbox",
                executable="async_slam_toolbox_node",
                namespace=launch_declared.UNSPECIFIED_NAMESPACE,
                name=f"{launch_declared.UNSPECIFIED_NAMESPACE}/slam_toolbox",
                name_source="literal",
            ),
            records.ResolvedNode(
                pkg="lx_sensors",
                executable="scan_relay",
                namespace=launch_declared.UNSPECIFIED_NAMESPACE,
                name=launch_declared.UNSPECIFIED_NAME,
                name_source="literal",
            ),
        ],
    )
    atlas = _run_walk(ws, walk, "sentinel", "plain_node.launch.py")
    assert ids.build_node("/slam_toolbox") in atlas.entities
    assert not [entity.id for entity in atlas.by_kind("node") if "unspecified" in entity.id]
    assert not [entity.id for entity in atlas.by_kind("launchent") if "unspecified" in entity.id]
    unnamed = next(entity for entity in atlas.by_kind("node") if ids.is_synthetic(entity.id))
    assert unnamed.name_source == "unknown"


# ---------------------------------------------------------------------------
# launch entities


def test_one_name_under_two_namespaces_gives_two_launch_entities(full: model.Atlas) -> None:
    entities = [
        entity
        for entity in full.by_kind("launchent")
        if ids.parse_launchent(entity.id).relpath == "launch/rewritten.launch.py"
    ]
    assert [ids.parse_launchent(entity.id).steps for entity in entities] == [
        (ids.LaunchStep(action="Node", disc="rewritten_node"),),
        (ids.LaunchStep(action="Node", disc="robot1/rewritten_node"),),
    ]


def test_the_gate_trail_is_in_the_launch_entity_id(full: model.Atlas) -> None:
    deep = [
        entity
        for entity in full.by_kind("launchent")
        if ids.parse_launchent(entity.id).steps[-1].disc == "deep/deep_node"
    ]
    assert len(deep) == 1
    assert [step.action for step in ids.parse_launchent(deep[0].id).steps] == [
        "OnProcessExit",
        "OnProcessExit",
        "Node",
    ]


def test_launch_entities_are_contained_and_launch_their_node(full: model.Atlas) -> None:
    launchfile = ids.build_launchfile("lx_bringup", "launch/plain_node.launch.py")
    contained = [edge.dst for edge in full.out_edges(launchfile) if edge.kind == "contains"]
    assert len(contained) == 1
    launchent = contained[0]
    launches = [edge for edge in full.out_edges(launchent) if edge.kind == "launches"]
    assert [edge.dst for edge in launches] == [ids.build_node("/sensors/lidar_driver")]
    assert all(edge.key for edge in launches)
    from_profile = [
        edge
        for edge in full.in_edges(launchent)
        if edge.kind == "launches" and edge.src == ids.build_profile("everything")
    ]
    assert len(from_profile) == 1
    assert from_profile[0].key


# ---------------------------------------------------------------------------
# gates


def test_every_gate_becomes_one_edge_with_its_mechanism(full: model.Atlas) -> None:
    gates = edges_of(full, "gates")
    seen = sorted(
        (
            edge.fact(LAYER, ids.build_profile("everything")).values["mechanism"],
            edge.fact(LAYER, ids.build_profile("everything")).values["target"],
            edge.fact(LAYER, ids.build_profile("everything")).values["period"],
        )
        for edge in gates
    )
    assert seen == [
        ("IfCondition", "", None),
        ("OnProcessExit", "ready_gate", None),
        ("OnProcessExit", "ready_gate", None),
        ("OnProcessExit", "stage_one", None),
        ("OnProcessExit", "stage_two", None),
        ("TimerAction", "", 5.0),
    ]
    assert len({edge.key for edge in gates}) == len(gates)


def test_a_timer_gate_keeps_its_period(full: model.Atlas) -> None:
    late = [
        edge
        for edge in full.edges.values()
        if edge.kind == "gates" and edge.dst == ids.build_node("/late_node")
    ]
    assert len(late) == 1
    timers = [
        edge
        for edge in edges_of(full, "gates")
        if edge.fact(LAYER, ids.build_profile("everything")).values["mechanism"] == "TimerAction"
    ]
    assert [
        edge.fact(LAYER, ids.build_profile("everything")).values["period"] for edge in timers
    ] == [5.0]


# ---------------------------------------------------------------------------
# remaps


def test_a_remap_of_a_known_interface_becomes_that_kind(full: model.Atlas) -> None:
    """`/tf` is a built in topic, so the rule that renames it names a topic.

    `points` is not: a ROS 2 remap rule renames a topic, a service and an action at once, so
    a rule over a name nothing here knows states no kind and mints no entity.
    """
    launchent = ids.build_launchent(
        "lx_bringup",
        "launch/plain_node.launch.py",
        (ids.LaunchStep(action="Node", disc="sensors/lidar_driver"),),
    )
    remaps = [edge for edge in full.out_edges(launchent) if edge.kind == "remaps"]
    values = sorted(
        (
            edge.dst,
            edge.fact(LAYER, ids.build_profile("everything")).values["raw_name"],
        )
        for edge in remaps
    )
    assert values == [(ids.build_topic("/sensors/tf"), "/tf")]
    assert ids.build_topic("/sensors/points_raw") not in full.entities
    blind = [item for item in full.unresolved if item.expr == "points:=/sensors/points_raw"]
    assert [item.role for item in blind] == ["remap"]
    assert "/sensors/points_raw" in blind[0].reason
    # The rule itself is still readable on the launch entity that wrote it.
    site = full.get(launchent).fact(LAYER, ids.build_profile("everything"))
    assert ["points", "/sensors/points_raw", "node"] in site.values["remaps"]


def test_an_action_sub_interface_is_not_called_a_topic(ws: Workspace) -> None:
    walk = records.LaunchWalk(
        entry_path=str(WS / "src/lx_bringup/launch/plain_node.launch.py"),
        profile="acts",
        nodes=[
            records.ResolvedNode(
                pkg="lx_sensors",
                executable="scan_relay",
                namespace="/",
                name="relay",
                name_source="literal",
                remaps=(
                    records.Remap(frm="dock/_action/send_goal", to="berth/_action/send_goal"),
                    records.Remap(frm="dock/_action/feedback", to="berth/_action/feedback"),
                ),
            )
        ],
    )
    atlas = _run_walk(ws, walk, "acts", "plain_node.launch.py")
    assert [entity.id for entity in atlas.by_kind("service")] == [
        ids.build_service("/berth/_action/send_goal")
    ]
    assert [entity.id for entity in atlas.by_kind("topic")] == [
        ids.build_topic("/berth/_action/feedback")
    ]


def test_a_group_scoped_remap_cites_the_file_that_pushed_it(ws: Workspace) -> None:
    pushed = records.Site(str(WS / "src/lx_bringup/launch/include_parent.launch.py"), "SetRemap", 7)
    walk = records.LaunchWalk(
        entry_path=str(WS / "src/lx_bringup/launch/plain_node.launch.py"),
        profile="scoped",
        nodes=[
            records.ResolvedNode(
                pkg="lx_sensors",
                executable="scan_relay",
                namespace="/",
                name="relay",
                name_source="literal",
                remaps=(
                    records.Remap(frm="/map", to="/grid", scope="global", site=pushed),
                    records.Remap(frm="/scan", to="/laser"),
                ),
                site=records.Site(str(WS / "src/lx_bringup/launch/plain_node.launch.py")),
            )
        ],
    )
    atlas = _run_walk(ws, walk, "scoped", "plain_node.launch.py")
    rows = {entry.expr: entry.evidence[0] for entry in atlas.unresolved}
    assert rows["/map:=/grid"].file == ws.file_id(pushed.path)
    assert (rows["/map:=/grid"].locator, rows["/map:=/grid"].line_hint) == ("SetRemap", 7)
    assert rows["/scan:=/laser"].file == ws.file_id(walk.entry_path)
    again = _run_walk(ws, walk, "scoped", "plain_node.launch.py")
    assert again.canonical_bytes() == atlas.canonical_bytes()


def test_a_relative_target_resolves_at_the_root_the_walk_stated(ws: Workspace) -> None:
    """An executed walk states the root namespace through the sentinel, so a relative remap
    target resolves there like any other."""
    walk = records.LaunchWalk(
        entry_path=str(WS / "src/lx_bringup/launch/plain_node.launch.py"),
        profile="loose",
        nodes=[
            records.ResolvedNode(
                pkg="lx_sensors",
                executable="scan_relay",
                namespace="",
                name="relay",
                name_source="literal",
                remaps=(records.Remap(frm="/tf", to="tf_filtered"),),
            )
        ],
    )
    atlas = _run_walk(ws, walk, "loose", "plain_node.launch.py")
    assert [entity.id for entity in atlas.by_kind("topic")] == [ids.build_topic("/tf_filtered")]
    assert not atlas.unresolved
    assert atlas.counters["resolution_rate"]["topic_name"] == 1.0


def test_a_relative_target_on_an_unexecuted_walk_resolves_to_nothing(ws: Workspace) -> None:
    walk = records.LaunchWalk(
        entry_path=str(WS / "src/lx_bringup/launch/plain_node.launch.py"),
        profile="loose",
        strategy="ast",
        nodes=[
            records.ResolvedNode(
                pkg="lx_sensors",
                executable="scan_relay",
                namespace="",
                name="relay",
                name_source="literal",
                remaps=(records.Remap(frm="input", to="points_filtered"),),
            )
        ],
    )
    atlas = _run_walk(ws, walk, "loose", "plain_node.launch.py")
    assert not atlas.by_kind("topic")
    assert sorted(entry.want for entry in atlas.unresolved) == ["node_name", "topic_name"]
    remap = next(entry for entry in atlas.unresolved if entry.want == "topic_name")
    assert remap.expr == "input:=points_filtered"
    assert atlas.counters["unresolved"]["topic_name"] == 1
    assert atlas.counters["resolution_rate"]["topic_name"] == 0.0


def test_resolved_endpoints_have_a_denominator(full: model.Atlas) -> None:
    """Only the endpoints whose kind something states are counted, and they all resolved.

    The rules whose kind nothing states are `other`, which is why that rate is below one.
    """
    assert full.counters["resolved"]["topic_name"] == 1
    assert full.counters["resolution_rate"]["topic_name"] == 1.0
    assert 0.0 < full.counters["resolution_rate"]["other"] < 1.0
    assert 0.0 < full.counters["resolution_rate"]["node_name"] < 1.0


# ---------------------------------------------------------------------------
# lifecycle


def test_node_names_becomes_manages_lifecycle(full: model.Atlas) -> None:
    edges = edges_of(full, "manages_lifecycle")
    rows = sorted(
        (
            edge.src,
            edge.dst,
            edge.fact(LAYER, ids.build_profile("everything")).values["raw_name"],
        )
        for edge in edges
    )
    assert rows == [
        (
            ids.build_node("/lifecycle_manager_navigation"),
            ids.build_node("/controller_server"),
            "controller_server",
        ),
        (
            ids.build_node("/lifecycle_manager_sensors"),
            ids.build_node("/ouster/os_driver"),
            "ouster/os_driver",
        ),
    ]


def test_the_manager_target_joins_the_node_the_walk_launched(ws: Workspace) -> None:
    atlas = build(ws, {"gate": ["event_gate"]})
    driver = ids.build_node("/ouster/os_driver")
    assert driver in atlas.entities
    managed = [edge for edge in atlas.in_edges(driver) if edge.kind == "manages_lifecycle"]
    assert len(managed) == 1
    # The manager states no namespace, which the executed walk read as the root, so the
    # relative name in its node_names list resolved there.
    assert managed[0].src == ids.build_node("/lifecycle_manager_sensors")


def test_an_in_launch_transition_manages_without_a_manager_node(ws: Workspace) -> None:
    walk = records.LaunchWalk(
        entry_path=str(WS / "src/lx_bringup/launch/nav_stack.launch.py"),
        profile="bridge",
        nodes=[
            records.ResolvedNode(
                pkg="lx_bringup",
                executable="lx_controller",
                namespace="/",
                form="lifecycle_node",
                name="io_bridge",
                name_source="literal",
                gates=(
                    records.Gate(
                        mechanism="OnStateTransition",
                        target="io_bridge",
                        outcome="inactive",
                    ),
                ),
                attrs={"lifecycle_transitions": ["configure", "activate"]},
            )
        ],
    )
    atlas = _run_walk(ws, walk, "bridge", "nav_stack.launch.py")
    edges = edges_of(atlas, "manages_lifecycle")
    assert len(edges) == 1
    assert ids.kind_of(edges[0].src) == "launchent"
    values = edges[0].fact(LAYER, ids.build_profile("bridge")).values
    assert values["mechanism"] == "OnStateTransition"
    assert values["in_launch"] is True
    assert values["transitions"] == ["activate", "configure", "inactive"]
    assert schema.validate(atlas.to_json()) == []


# ---------------------------------------------------------------------------
# includes


def test_includes_become_edges_and_unresolved_entries(ws: Workspace) -> None:
    parent = WS / "src/lx_bringup/launch/include_parent.launch.py"
    child = WS / "src/lx_bringup/launch/include_child.launch.py"
    walk = records.LaunchWalk(
        entry_path=str(parent),
        profile="inc",
        includes=[
            records.LaunchInclude(
                path=str(child),
                args=(("child_name", "relay_child"),),
                site=records.Site(path=str(parent), locator="IncludeLaunchDescription"),
            ),
            records.LaunchInclude(
                path="",
                resolved=False,
                site=records.Site(path=str(parent), locator="IncludeLaunchDescription"),
            ),
        ],
    )
    atlas = _run_walk(ws, walk, "inc", "include_parent.launch.py")
    parent_id = ids.build_launchfile("lx_bringup", "launch/include_parent.launch.py")
    child_id = ids.build_launchfile("lx_bringup", "launch/include_child.launch.py")
    includes = [edge for edge in atlas.out_edges(parent_id) if edge.kind == "includes"]
    assert [edge.dst for edge in includes] == [child_id]
    assert includes[0].fact(LAYER, ids.build_profile("inc")).values["args"] == [
        ["child_name", "relay_child"]
    ]
    assert [entry.role for entry in atlas.unresolved] == ["include"]
    assert atlas.get(parent_id).unresolved == [atlas.unresolved[0].id]


def test_one_file_included_twice_with_two_argument_sets_keeps_both(ws: Workspace) -> None:
    """nav2's multi robot sim includes one bringup per robot on one action trail. Both
    includes became one launch entity whose second argument set refused to overwrite the
    first, and the ContractError dropped every node of the profile."""
    parent = WS / "src/lx_bringup/launch/include_parent.launch.py"
    child = WS / "src/lx_bringup/launch/include_child.launch.py"

    def include(robot: str) -> records.LaunchInclude:
        return records.LaunchInclude(
            path=str(child),
            args=(("namespace", robot),),
            entity_path=("Include(include_child.launch.py)",),
            site=records.Site(path=str(parent), locator="IncludeLaunchDescription"),
        )

    def relay(robot: str) -> records.ResolvedNode:
        return records.ResolvedNode(
            pkg="lx_bringup",
            executable="lx_relay",
            namespace=f"/{robot}",
            name="relay",
            name_source="literal",
            entity_path=("Include(include_child.launch.py)", "Node(relay)"),
        )

    walk = records.LaunchWalk(
        entry_path=str(parent),
        profile="twice",
        includes=[include("robot1"), include("robot2")],
        nodes=[relay("robot1"), relay("robot2")],
    )
    atlas = _run_walk(ws, walk, "twice", "include_parent.launch.py")
    assert not atlas.by_kind("run")[0].attrs["exceptions"]
    assert {ids.build_node("/robot1/relay"), ids.build_node("/robot2/relay")} <= set(atlas.entities)
    stated = sorted(
        fact.values["args"]
        for entity in atlas.by_kind("launchent")
        for fact in entity.facts
        if "args" in fact.values
    )
    assert stated == [[["namespace", "robot1"]], [["namespace", "robot2"]]]


def test_a_file_an_unrun_process_writes_is_a_blind_spot_and_not_a_raise(ws: Workspace) -> None:
    """tb3_simulation_launch.py removes, on shutdown, the world file `xacro -o` writes. The
    walk starts no process, so the file never exists: that is a blind spot of the walk, not
    a launch file that raised, and nothing about it names the sandbox."""
    from rosio.resolve import launch_exec

    entry = WS / "src/lx_bringup/launch/plain_node.launch.py"
    message = (
        "OpaqueFunction(fn0a1b2c3d): FileNotFoundError on a file the process "
        "`xacro -o @output world.sdf.xacro` writes when it runs, and the walk starts no process"
    )
    walk = records.LaunchWalk(
        entry_path=str(entry),
        profile="unrun",
        exceptions=[
            records.WalkException(
                path=str(entry), phase="visit", exc_type=launch_exec.UNRUN_OUTPUT, message=message
            )
        ],
    )
    atlas = _run_walk(ws, walk, "unrun", "plain_node.launch.py")
    assert not [f for f in atlas.findings if f.rule.startswith("launch.")]
    rows = atlas.by_kind("run")[0].attrs["exceptions"]
    assert [row["exc_type"] for row in rows] == [launch_exec.UNRUN_OUTPUT]
    assert "note" not in rows[0]
    reasons = [entry.reason for entry in atlas.unresolved]
    assert len(reasons) == 1 and "xacro -o @output" in reasons[0]
    assert "sandbox refused" not in model.dumps(atlas.to_json())


def _expansion_walk(kept: str) -> records.LaunchWalk:
    entry = WS / "src/lx_bringup/launch/plain_node.launch.py"
    failure = records.WalkException(
        path=str(entry),
        phase="visit",
        exc_type="PackageNotFoundError",
        message="Node: \"package 'absent_description' not found\"",
        kept=kept,
    )
    return records.LaunchWalk(entry_path=str(entry), profile="kept", exceptions=[failure])


def test_a_node_kept_after_its_expansion_raised_is_named_and_not_called_missing(
    ws: Workspace,
) -> None:
    """ros2_control_demos example 1: rviz2's `-d` argument names an unbuilt package. The walk
    records rviz2 without that argument and carries on, so no node is missing."""
    atlas = _run_walk(ws, _expansion_walk("Node(rviz2)"), "kept", "plain_node.launch.py")
    assert not [f for f in atlas.findings if f.rule == "launch.raised"]
    (finding,) = drift_build.launch_raised(atlas)
    assert "Node(rviz2)" in finding.say
    assert "missing from this atlas" not in finding.say
    assert "stopped" not in finding.detail
    (row,) = atlas.by_kind("run")[0].attrs["exceptions"]
    assert row["kept"] == "Node(rviz2)"
    (blind,) = [entry for entry in atlas.unresolved if entry.owner.startswith("launchfile:")]
    assert "node set is incomplete" not in blind.reason
    again = _run_walk(ws, _expansion_walk("Node(rviz2)"), "kept", "plain_node.launch.py")
    assert [f.id for f in drift_build.launch_raised(again)] == [finding.id]


def test_a_raise_that_kept_no_action_still_reports_the_missing_nodes(ws: Workspace) -> None:
    atlas = _run_walk(ws, _expansion_walk(""), "kept", "plain_node.launch.py")
    (finding,) = drift_build.launch_raised(atlas)
    assert "missing from this atlas" in finding.say
    assert "kept" not in atlas.by_kind("run")[0].attrs["exceptions"][0]
    (blind,) = [entry for entry in atlas.unresolved if entry.owner.startswith("launchfile:")]
    assert "node set is incomplete" in blind.reason


# ---------------------------------------------------------------------------
# the walks are handed on


def test_produced_walks_are_left_on_the_bundle(ws: Workspace) -> None:
    cfg = Config(
        roots=tuple(RootSpec(kind=spec.kind, path=spec.path) for spec in ws.roots),
        profiles=(ProfileSpec(slug="legacy", entrypoints=(entry_for("legacy_ros1"),)),),
    )
    builder = model.AtlasBuilder()
    ctx = CollectContext(ws=ws, cfg=cfg, builder=builder)
    get(launch_declared.NAME).collect(ctx)
    assert [walk.strategy for walk in ctx.declared.walks] == ["refused"]
    assert ctx.declared.walks[0].profile == "legacy"


def test_preloaded_walks_are_reused(ws: Workspace) -> None:
    atlas = build(ws, {"reuse": ["plain_node"]})
    assert ids.build_node("/sensors/lidar_driver") in atlas.entities


def test_the_same_file_in_two_profiles_keeps_two_slots(ws: Workspace) -> None:
    atlas = build(ws, {"one": ["plain_node"], "two": ["plain_node"]})
    entity = atlas.get(ids.build_node("/sensors/lidar_driver"))
    assert [fact.profile for fact in entity.facts] == [
        ids.build_profile("one"),
        ids.build_profile("two"),
    ]
    assert entity.profiles == [ids.build_profile("one"), ids.build_profile("two")]
    assert schema.validate(atlas.to_json()) == []


# ---------------------------------------------------------------------------


def _run_walk(ws: Workspace, walk: records.LaunchWalk, slug: str, target: str) -> model.Atlas:
    package = "lx_sensors" if "sensors_only" in target else "lx_bringup"
    cfg = Config(
        roots=tuple(RootSpec(kind=spec.kind, path=spec.path) for spec in ws.roots),
        profiles=(
            ProfileSpec(
                slug=slug,
                entrypoints=(Entrypoint(verb="launch", pkg=package, target=target),),
            ),
        ),
    )
    builder = model.AtlasBuilder()
    ctx = CollectContext(ws=ws, cfg=cfg, builder=builder)
    ctx.declared.walks.append(walk)
    get(launch_declared.NAME).collect(ctx)
    return builder.build()


# ---------------------------------------------------------------------------
# a resolver supplied action trail


def test_siblings_on_a_supplied_trail_stay_separate(ws: Workspace) -> None:
    """The real resolver names the trail after the action, so the leaf discriminator has to
    come from the node's own content or four unnamed siblings become one launch entity."""

    def publisher(child: str) -> records.ResolvedNode:
        return records.ResolvedNode(
            pkg="tf2_ros",
            executable="static_transform_publisher",
            namespace="",
            argv=("--frame-id", "base_link", "--child-frame-id", child),
            entity_path=("Node(static_transform_publisher)",),
        )

    walk = records.LaunchWalk(
        entry_path=str(WS / "src/lx_bringup/launch/plain_node.launch.py"),
        profile="trail",
        nodes=[publisher("os_sensor"), publisher("gps"), publisher("imu")],
    )
    atlas = _run_walk(ws, walk, "trail", "plain_node.launch.py")
    assert len(atlas.by_kind("launchent")) == 3
    assert len(atlas.by_kind("node")) == 3
    assert not atlas.by_kind("run")[0].attrs["exceptions"]


def test_one_name_under_two_namespaces_on_a_supplied_trail(ws: Workspace) -> None:
    """The resolver spells both nodes `Node(rewritten_node)`. Keeping that as the id would
    merge them, drop the second node and lose one parameter chain."""

    def node(namespace: str) -> records.ResolvedNode:
        return records.ResolvedNode(
            pkg="lx_bringup",
            executable="lx_controller",
            namespace=namespace,
            name="rewritten_node",
            name_source="literal",
            entity_path=("Node(rewritten_node)",),
        )

    walk = records.LaunchWalk(
        entry_path=str(WS / "src/lx_bringup/launch/rewritten.launch.py"),
        profile="two_ns",
        nodes=[node(""), node("/robot1")],
    )
    atlas = _run_walk(ws, walk, "two_ns", "rewritten.launch.py")
    assert len(atlas.by_kind("launchent")) == 2
    assert ids.build_node("/robot1/rewritten_node") in atlas.entities
    assert not atlas.by_kind("run")[0].attrs["exceptions"]


# ---------------------------------------------------------------------------
# a walk belongs to its argument set


def test_a_walk_with_other_arguments_is_not_reused(ws: Workspace) -> None:
    """`opaque_probe` resolves a different executable per argument set, so handing the
    argument free walk to the entry point that supplies one answers the wrong question."""
    cfg = Config(
        roots=tuple(RootSpec(kind=spec.kind, path=spec.path) for spec in ws.roots),
        profiles=(ProfileSpec(slug="probe", entrypoints=(entry_for("opaque_probe.with_map"),)),),
    )
    builder = model.AtlasBuilder()
    ctx = CollectContext(ws=ws, cfg=cfg, builder=builder)
    ctx.declared.walks.append(walk_for("opaque_probe.no_map", "probe"))
    get(launch_declared.NAME).collect(ctx)
    assert len(ctx.declared.walks) == 2
    assert ctx.declared.walks[0].args == ()
    # The second walk was resolved for the entry point, so it carries its arguments.
    assert dict(ctx.declared.walks[1].args).get("map_base")


# ---------------------------------------------------------------------------
# managed names are never minted from a namespace nobody stated


def _manager(names: list[str], namespace: str = "") -> records.ResolvedNode:
    return records.ResolvedNode(
        pkg="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        namespace=namespace,
        name="lifecycle_manager_navigation",
        name_source="literal",
        params=(records.ParamSource(order=0, kind="inline_dict", values=(("node_names", names),)),),
    )


def test_a_relative_managed_name_joins_the_node_the_walk_launched(ws: Workspace) -> None:
    """Neither node states a namespace, and on an executed walk that means the root, so both
    the manager and the node it names land on real ids and one edge joins them."""
    walk = records.LaunchWalk(
        entry_path=str(WS / "src/lx_bringup/launch/nav_stack.launch.py"),
        profile="managed",
        nodes=[
            _manager(["controller_server"]),
            records.ResolvedNode(
                pkg="nav2_controller",
                executable="controller_server",
                namespace="",
                name=None,
                name_source="executable_default",
            ),
        ],
    )
    atlas = _run_walk(ws, walk, "managed", "nav_stack.launch.py")
    assert len(atlas.by_kind("node")) == 2
    edges = edges_of(atlas, "manages_lifecycle")
    assert [edge.dst for edge in edges] == [ids.build_node("/controller_server")]


def test_a_managed_name_on_an_unexecuted_walk_mints_nothing(ws: Workspace) -> None:
    """Nothing stated a namespace and nothing was executed, so the namespace the name
    resolves in is unknown and no id is minted for it."""
    walk = records.LaunchWalk(
        entry_path=str(WS / "src/lx_bringup/launch/nav_stack.launch.py"),
        profile="orphan",
        strategy="ast",
        nodes=[_manager(["map_saver"])],
    )
    atlas = _run_walk(ws, walk, "orphan", "nav_stack.launch.py")
    assert ids.build_node("/map_saver") not in atlas.entities
    assert not edges_of(atlas, "manages_lifecycle")
    reasons = [entry.reason for entry in atlas.unresolved if entry.role == "lifecycle"]
    assert reasons == [
        "the manager states no namespace, so the namespace this managed name resolves in "
        "is unknown and no launched node carries it"
    ]


def test_a_managed_name_under_a_stated_root_is_minted(ws: Workspace) -> None:
    """The executed walk stated the root, so the managed name is a fully qualified name and
    the manager owns a node the profile did not launch itself."""
    walk = records.LaunchWalk(
        entry_path=str(WS / "src/lx_bringup/launch/nav_stack.launch.py"),
        profile="orphan",
        nodes=[_manager(["map_saver"])],
    )
    atlas = _run_walk(ws, walk, "orphan", "nav_stack.launch.py")
    assert ids.build_node("/map_saver") in atlas.entities
    assert len(edges_of(atlas, "manages_lifecycle")) == 1


def test_an_absolute_managed_name_is_still_minted(ws: Workspace) -> None:
    walk = records.LaunchWalk(
        entry_path=str(WS / "src/lx_bringup/launch/nav_stack.launch.py"),
        profile="absolute",
        nodes=[_manager(["/map_saver"])],
    )
    atlas = _run_walk(ws, walk, "absolute", "nav_stack.launch.py")
    assert ids.build_node("/map_saver") in atlas.entities
    assert len(edges_of(atlas, "manages_lifecycle")) == 1


def test_two_spellings_of_one_managed_name_are_two_sites(ws: Workspace) -> None:
    """One list, two entries, one node. Two sites must not collapse onto one key, and the
    refusal to overwrite must not empty the layer."""
    walk = records.LaunchWalk(
        entry_path=str(WS / "src/lx_bringup/launch/nav_stack.launch.py"),
        profile="spelling",
        nodes=[_manager(["/map_saver", "map_saver"], namespace="/")],
    )
    atlas = _run_walk(ws, walk, "spelling", "nav_stack.launch.py")
    edges = edges_of(atlas, "manages_lifecycle")
    assert len(edges) == 2
    assert len({edge.key for edge in edges}) == 2
    assert not atlas.by_kind("run")[0].attrs["exceptions"]
    assert schema.validate(atlas.to_json()) == []


# ---------------------------------------------------------------------------
# entry points with the verb run


def _run_profile(
    ws: Workspace, slug: str, *entries: Entrypoint
) -> tuple[model.Atlas, CollectContext]:
    cfg = Config(
        roots=tuple(RootSpec(kind=spec.kind, path=spec.path) for spec in ws.roots),
        profiles=(ProfileSpec(slug=slug, entrypoints=entries),),
    )
    builder = model.AtlasBuilder()
    ctx = CollectContext(ws=ws, cfg=cfg, builder=builder)
    get(launch_declared.NAME).collect(ctx)
    return builder.build(), ctx


def _health_monitor(*argv: str) -> Entrypoint:
    return Entrypoint(verb="run", pkg="lx_sensors", target="health_monitor", argv=tuple(argv))


def test_a_run_entry_point_becomes_a_node_of_the_profile(ws: Workspace) -> None:
    atlas, _ctx = _run_profile(
        ws,
        "runs",
        _health_monitor("--ros-args", "-r", "__node:=watchdog", "-r", "__ns:=/sensors"),
    )
    entity = atlas.get(ids.build_node("/sensors/watchdog"))
    assert entity is not None
    assert entity.name_source == "argv"
    fact = entity.fact(LAYER, ids.build_profile("runs"))
    assert fact.values["form"] == "process"
    assert fact.values["executable"] == "health_monitor"
    assert fact.values["entrypoint"] == "ros2 run lx_sensors health_monitor"
    assert fact.values["namespace"] == "/sensors"
    launched = [edge for edge in atlas.in_edges(entity.id) if edge.kind == "launches"]
    assert [edge.src for edge in launched] == [ids.build_profile("runs")]
    assert ids.build_exe("lx_sensors", "health_monitor") in atlas.entities
    assert schema.validate(atlas.to_json()) == []


def test_a_run_entry_point_keeps_its_parameter_inputs_in_argv_order(ws: Workspace) -> None:
    """A `--params-file` written after a `-p` wins the keys they share, so the order is the
    precedence rule and it has to survive into the record."""
    params = str(WS / "src/lx_bringup/config/imu_driver.yaml")
    atlas, ctx = _run_profile(
        ws,
        "ordered",
        _health_monitor(
            "--ros-args",
            "-p",
            "rate:=20",
            "--params-file",
            params,
            "-p",
            "enabled:=true",
        ),
    )
    node = atlas.by_kind("node")[0]
    inputs = node.fact(LAYER, ids.build_profile("ordered")).values["parameter_inputs"]
    assert [(item["order"], item["kind"]) for item in inputs] == [
        (0, "ros_args_assign"),
        (1, "ros_args_file"),
        (2, "ros_args_assign"),
    ]
    assert inputs[1]["path"] == "lx_bringup/config/imu_driver.yaml"
    # The same record is handed to the parameter chain, with a site on every input.
    walk = next(item for item in ctx.declared.walks if item.profile == "ordered")
    assert [source.kind for source in walk.nodes[0].params] == [
        "ros_args_assign",
        "ros_args_file",
        "ros_args_assign",
    ]
    assert all(source.site for source in walk.nodes[0].params)


def test_a_run_entry_point_with_no_name_in_argv_says_so(ws: Workspace) -> None:
    atlas, _ctx = _run_profile(ws, "unnamed", _health_monitor())
    entity = atlas.by_kind("node")[0]
    assert ids.is_synthetic(entity.id)
    assert entity.name_source == "unknown"
    assert [item.reason for item in atlas.unresolved] == [
        "the argv states no name and the executable default is not in the name table"
    ]


def test_a_run_entry_point_outside_the_workspace_still_becomes_a_node(ws: Workspace) -> None:
    """rviz2 is not in this workspace, so there is no file to cite and no executable entity,
    and the node is still declared because the profile really starts it."""
    atlas, _ctx = _run_profile(
        ws,
        "third_party",
        Entrypoint(verb="run", pkg="rviz2", target="rviz2", argv=("-d", "x.rviz")),
    )
    entity = atlas.by_kind("node")[0]
    assert ids.is_synthetic(entity.id)
    assert entity.name_source == "runtime_randomised"
    fact = entity.fact(LAYER, ids.build_profile("third_party"))
    assert fact.values["argv"] == ["-d", "x.rviz"]
    assert not fact.evidence
    assert not atlas.by_kind("exe")
    assert schema.validate(atlas.to_json()) == []


def test_two_runs_of_one_executable_keep_both_parameter_sets(ws: Workspace) -> None:
    """Two invocations of one executable are two nodes, so both records have to reach the
    parameter chain. Keying the record on the entry point alone drops the second node's
    parameters while its node entity stays in the atlas."""
    atlas, ctx = _run_profile(
        ws,
        "twice",
        _health_monitor("--ros-args", "-p", "rate:=20"),
        _health_monitor("--ros-args", "-p", "rate:=40"),
    )
    assert len(atlas.by_kind("node")) == 2
    handed = [walk for walk in ctx.declared.walks if walk.profile == "twice"]
    assert [walk.nodes[0].params[0].values for walk in handed] == [
        (("rate", "20"),),
        (("rate", "40"),),
    ]


def test_one_run_entry_point_is_handed_on_once(ws: Workspace) -> None:
    atlas, ctx = _run_profile(
        ws,
        "same",
        _health_monitor("--ros-args", "-p", "rate:=20"),
        _health_monitor("--ros-args", "-p", "rate:=20"),
    )
    assert len(atlas.by_kind("node")) == 1
    assert len([walk for walk in ctx.declared.walks if walk.profile == "same"]) == 1


def test_the_profile_counts_its_run_entry_points(ws: Workspace) -> None:
    atlas, _ctx = _run_profile(
        ws,
        "mixed",
        Entrypoint(verb="launch", pkg="lx_bringup", target="plain_node.launch.py"),
        _health_monitor("--ros-args", "-r", "__node:=watchdog"),
    )
    profile = atlas.get(ids.build_profile("mixed"))
    assert profile.attrs["entry_kind"] == "mixed"
    assert profile.attrs["inputs"]["run_entrypoints"] == ["ros2 run lx_sensors health_monitor"]
    fact = profile.fact(LAYER, ids.build_profile("mixed"))
    assert fact.values["run_nodes"] == 1
    assert fact.values["declared_nodes"] == 2


# ---------------------------------------------------------------------------
# no temp path inside a role digest


def test_a_temp_parameters_file_never_moves_the_synthetic_id(ws: Workspace) -> None:
    """The merged parameters file a launcher writes is a random name under the temp
    directory. Hashing it makes one node two nodes on the second extract."""

    def walk(temp_name: str) -> records.LaunchWalk:
        return records.LaunchWalk(
            entry_path=str(WS / "src/lx_bringup/launch/plain_node.launch.py"),
            profile="temp",
            nodes=[
                records.ResolvedNode(
                    pkg="lx_sensors",
                    executable="imu_driver",
                    namespace="/",
                    params=(
                        records.ParamSource(
                            order=0,
                            kind="external_file",
                            path=f"{tempfile.gettempdir()}/{temp_name}_ant.yaml",
                        ),
                    ),
                )
            ],
        )

    first = _run_walk(ws, walk("tmp8f31a"), "temp", "plain_node.launch.py")
    second = _run_walk(ws, walk("tmpb92cc"), "temp", "plain_node.launch.py")
    assert node_ids(first) == node_ids(second)
    assert ids.is_synthetic(node_ids(first)[0])
    assert first.canonical_bytes() == second.canonical_bytes()


def test_a_temp_argument_never_reaches_an_include_fact(ws: Workspace) -> None:
    """The launcher hands the merged parameters file to an include as an argument. Its name
    is random per run, so leaving it in a compared value makes two identical extracts
    differ."""
    parent = WS / "src/lx_bringup/launch/include_parent.launch.py"
    child = WS / "src/lx_bringup/launch/include_child.launch.py"

    def walk(temp_name: str) -> records.LaunchWalk:
        return records.LaunchWalk(
            entry_path=str(parent),
            profile="inc",
            includes=[
                records.LaunchInclude(
                    path=str(child),
                    args=(
                        ("params_file", f"{tempfile.gettempdir()}/{temp_name}_nav.yaml"),
                        ("use_sim_time", "false"),
                    ),
                    site=records.Site(path=str(parent), locator="IncludeLaunchDescription"),
                )
            ],
        )

    first = _run_walk(ws, walk("tmp8f31a"), "inc", "include_parent.launch.py")
    second = _run_walk(ws, walk("tmpb92cc"), "inc", "include_parent.launch.py")
    edge = next(edge for edge in first.edges.values() if edge.kind == "includes")
    assert edge.fact(LAYER, ids.build_profile("inc")).values["args"] == [
        ["params_file", "external_file"],
        ["use_sim_time", "false"],
    ]
    assert first.canonical_bytes() == second.canonical_bytes()


def test_a_tracked_path_contributes_its_basename_and_a_ros_name_is_untouched() -> None:
    assert identity.role_token(f"{tempfile.gettempdir()}/x_merged.yaml") == "external_file"
    assert identity.role_token("/opt/ros/humble/share/nav2/params.yaml") == "params.yaml"
    assert identity.role_token("/ouster/points") == "/ouster/points"
    assert identity.role_token("--frame-id") == "--frame-id"


# ---------------------------------------------------------------------------
# the two collector owned findings


def test_an_unloadable_file_becomes_one_finding(ws: Workspace) -> None:
    atlas = build(ws, {"legacy": ["legacy_ros1"]})
    assert [finding.rule for finding in atlas.findings] == ["launch.unloadable"]
    finding = atlas.findings[0]
    assert finding.severity == "warn"
    assert finding.subject == ids.build_launchfile("lx_bringup", "launch/legacy_ros1.launch")
    assert "launch/legacy_ros1.launch" in finding.say
    assert [source.file for source in finding.where] == [
        ids.build_file("lx_bringup", "launch/legacy_ros1.launch")
    ]
    assert [source.locator for source in finding.where] == ["import"]
    assert finding.profiles == [ids.build_profile("legacy")]


def test_a_raised_description_becomes_one_finding_with_its_phase(ws: Workspace) -> None:
    atlas = build(ws, {"env": ["env_required.missing"]})
    finding = next(item for item in drift_build.launch_raised(atlas))
    assert finding.severity == "error"
    assert finding.subject == ids.build_launchfile("lx_bringup", "launch/env_required.launch.py")
    assert [source.locator for source in finding.where] == ["describe"]
    assert "RuntimeError" in finding.say
    assert "describe" in finding.say
    assert "launch/env_required.launch.py" in finding.say


def test_one_file_failing_in_two_profiles_is_one_finding(ws: Workspace) -> None:
    atlas = build(ws, {"one": ["legacy_ros1"], "two": ["legacy_ros1"]})
    assert len(atlas.findings) == 1
    assert atlas.findings[0].profiles == [
        ids.build_profile("one"),
        ids.build_profile("two"),
    ]


# ---------------------------------------------------------------------------
# a package directory is only an alias for the package.xml name


def test_a_directory_alias_finds_the_launch_file() -> None:
    """The LIO_PKG directory holds the package lio_pkg. An entry point spelled either way
    names the same file, and reporting the launch file as missing is false."""
    index = {("lio_pkg", "launch/mapping.launch.py"): "/ws/src/LIO_PKG/launch/mapping.launch.py"}
    aliases = {"LIO_PKG": "lio_pkg"}
    assert launch_declared._locate(index, "lio_pkg", "mapping.launch.py", aliases)
    assert launch_declared._locate(index, "LIO_PKG", "mapping.launch.py", aliases)
    assert launch_declared._locate(index, "LIO_PKG", "mapping.launch.py") is None


def test_aliases_come_from_the_manifest_name(ws: Workspace) -> None:
    from rosio.workspace import Package

    packages = dict(ws.packages)
    packages["lio_pkg"] = Package(
        name="lio_pkg",
        src_path=str(WS / "src/LIO_PKG"),
        share_path=None,
        build_type="ament_cmake",
        executables=(),
        plugin_manifests=(),
    )
    reference = Workspace(ws.roots, packages, ws.files(), ws.repo_root)
    assert launch_declared._aliases(reference) == {"LIO_PKG": "lio_pkg"}


# ---------------------------------------------------------------------------
# the committed child node table


def test_the_child_node_table_answers_the_two_costmaps() -> None:
    from rosio.resolve import known_names

    assert known_names.child_nodes("nav2_controller", "controller_server") == ("local_costmap",)
    assert known_names.child_nodes("nav2_planner", "planner_server") == ("global_costmap",)
    assert known_names.child_nodes("nav2_smoother", "smoother_server") == ()


def test_a_child_node_is_named_under_its_own_name() -> None:
    from rosio.resolve import known_names

    row = known_names.child_rows("nav2_controller", "controller_server")[0]
    assert row.fqn("/") == "/local_costmap/local_costmap"
    assert row.fqn("/robot1") == "/robot1/local_costmap/local_costmap"


def test_an_unverified_child_row_is_never_returned_as_a_name() -> None:
    from rosio.resolve import known_names

    assert known_names.child_nodes("nav2_costmap_2d", "nav2_costmap_2d") == ()
    assert [row.child for row in known_names.child_rows("nav2_costmap_2d", "nav2_costmap_2d")] == [
        "costmap"
    ]


def test_every_child_row_carries_its_evidence() -> None:
    from rosio.resolve import known_names

    assert known_names.CHILD_TABLE
    for row in known_names.CHILD_TABLE:
        assert row.evidence
        assert row.rule == known_names.CHILD_NAMESPACE_RULE


# ---------------------------------------------------------------------------
# the launch machinery itself


def _down(monkeypatch, module: str = "lark") -> None:
    """Pin the answer about the launch machinery, so the case is the same with ROS on the
    path and without it."""
    from rosio.resolve import launch_exec

    report = launch_exec.LaunchImport(
        ok=False,
        module=module,
        exc_type="ModuleNotFoundError",
        message=f"No module named '{module}'",
        fix=launch_exec.VENV_FIX,
    )
    monkeypatch.setattr(launch_exec, "probe", lambda *_setup: report)


def test_a_launch_layer_that_did_not_happen_is_reported_once_at_the_run(
    ws: Workspace, monkeypatch
) -> None:
    _down(monkeypatch)
    atlas, _ctx = _run_profile(ws, "down", entry_for("plain_node"), entry_for("nav_stack"))
    rows = [
        row
        for entity in atlas.by_kind("run")
        for row in entity.attrs["exceptions"]
        if row["exc_type"] == launch_declared.LAUNCH_UNAVAILABLE
    ]
    assert len(rows) == 1, "one row for the layer, not one per file"
    assert "No module named 'lark'" in rows[0]["message"]
    assert "--system-site-packages" in rows[0]["message"]
    assert schema.validate(atlas.to_json()) == []


def test_every_entry_point_is_unloadable_when_nothing_can_load_one(
    ws: Workspace, monkeypatch
) -> None:
    _down(monkeypatch)
    atlas, _ctx = _run_profile(ws, "down", entry_for("plain_node"), entry_for("nav_stack"))
    files = atlas.by_kind("launchfile")
    assert files, "the entry points are still recorded"
    assert [node.attrs["loadable"] for node in files] == [False] * len(files)


def test_the_unloadable_finding_names_the_import_and_not_the_file_format(
    ws: Workspace, monkeypatch
) -> None:
    _down(monkeypatch)
    atlas, _ctx = _run_profile(ws, "down", entry_for("plain_node"))
    found = [item for item in atlas.findings if item.rule == launch_declared.RULE_UNLOADABLE]
    assert len(found) == 1
    assert "cannot import the launch package" in found[0].say
    assert "ROS 1" not in found[0].say
    assert "launch and launch_ros" in found[0].fix
    rows = [
        row
        for entity in atlas.by_kind("run")
        for row in entity.attrs["exceptions"]
        if row.get("note") == launch_declared.RULE_UNLOADABLE
    ]
    assert [row["refused_because"] for row in rows] == [launch_declared.REFUSED_UNAVAILABLE]


def test_a_ros2_xml_file_with_a_launch_suffix_is_not_called_ros1(tmp_path: Path) -> None:
    """ros_gz_sim installs ROS 2 XML as `gz_server.launch`. The suffix said ROS 1 and the
    finding told a reader to port a file that already runs under ROS 2."""
    package = tmp_path / "src" / "gz_demo"
    (package / "launch").mkdir(parents=True)
    (package / "package.xml").write_text(
        '<?xml version="1.0"?>\n<package format="3"><name>gz_demo</name><version>0.0.0'
        "</version><description>d</description><maintainer email='a@b.c'>a</maintainer>"
        "<license>Apache-2.0</license></package>\n",
        encoding="utf-8",
    )
    (package / "launch" / "server.launch").write_text(
        '<launch>\n  <arg name="world" default="empty.sdf"/>\n'
        '  <gz_server world_sdf_file="$(var world)"/>\n</launch>\n',
        encoding="utf-8",
    )
    workspace = Workspace.discover((RootSpec(kind="src", path=str(tmp_path / "src")),))
    entry = Entrypoint(verb="launch", pkg="gz_demo", target="server.launch")
    atlas, _ctx = _run_profile(workspace, "gz", entry)
    rows = [
        row
        for entity in atlas.by_kind("run")
        for row in entity.attrs["exceptions"]
        if row.get("note") == launch_declared.RULE_UNLOADABLE
    ]
    assert len(rows) == 1
    assert rows[0]["refused_because"] != launch_declared.REFUSED_ROS1
    found = [item for item in atlas.findings if item.rule == launch_declared.RULE_UNLOADABLE]
    assert "ROS 1" not in found[0].say


def test_a_ros1_file_still_reads_as_a_ros1_file(ws: Workspace) -> None:
    atlas, _ctx = _run_profile(ws, "legacy", entry_for("legacy_ros1"))
    rows = [
        row
        for entity in atlas.by_kind("run")
        for row in entity.attrs["exceptions"]
        if row.get("note") == launch_declared.RULE_UNLOADABLE
    ]
    assert [row["refused_because"] for row in rows] == [launch_declared.REFUSED_ROS1]
    found = [item for item in atlas.findings if item.rule == launch_declared.RULE_UNLOADABLE]
    assert "ROS 1 style launch file" in found[0].say


def test_a_ros2_markup_entry_point_is_not_called_a_ros1_file() -> None:
    """XML is a supported ROS 2 launch format, so the sentence for it says what is true."""
    say = launch_declared.unloadable_say("lx/x.launch.xml", launch_declared.REFUSED_MARKUP)
    assert "XML or YAML ROS 2 launch file" in say
    assert "ROS 1" not in say
    assert "python launch description" in launch_declared.unloadable_fix(
        launch_declared.REFUSED_MARKUP
    )


def test_every_include_that_collapsed_is_one_row_on_the_run(ws: Workspace) -> None:
    """Three includes that did not resolve took three files with them, so the run counts
    three and not one."""
    walk = records.LaunchWalk(
        entry_path=_abs("src/lx_bringup/launch/plain_node.launch.py"),
        profile="partial",
        entry_pkg="lx_bringup",
        includes=[
            records.LaunchInclude(path="source(one_dir, 'slam.launch.py')", resolved=False),
            records.LaunchInclude(path="source(two_dir, 'navigation.launch.py')", resolved=False),
        ],
    )
    atlas = _run_walk(ws, walk, "partial", "plain_node.launch.py")
    rows = [
        row
        for entity in atlas.by_kind("run")
        for row in entity.attrs["exceptions"]
        if row["exc_type"] == launch_declared.INCLUDE_UNRESOLVED
    ]
    assert len(rows) == 2
    assert sorted({row["path"] for row in rows}) == ["lx_bringup/launch/plain_node.launch.py"]
    assert len({entry.id for entry in atlas.unresolved if entry.role == "include"}) == 2


def test_an_include_expression_never_carries_a_memory_address(ws: Workspace) -> None:
    walk = records.LaunchWalk(
        entry_path=_abs("src/lx_bringup/launch/plain_node.launch.py"),
        profile="repr",
        entry_pkg="lx_bringup",
        includes=[
            records.LaunchInclude(
                path="<launch.substitutions.text_substitution.TextSubstitution object at 0xff01>",
                resolved=False,
            )
        ],
    )
    atlas = _run_walk(ws, walk, "repr", "plain_node.launch.py")
    entry = next(item for item in atlas.unresolved if item.role == "include")
    assert " object at 0x" not in entry.expr
    assert entry.expr == "<launch.substitutions.text_substitution.TextSubstitution>"


def test_an_argv_under_a_temp_root_is_folded_out_of_the_fact(ws: Workspace) -> None:
    """A launch time rewrite writes a file whose name is random per run, and an argv that
    keeps it makes two extracts of one unchanged workspace differ."""
    temp = str(Path(tempfile.gettempdir()) / "tmp9f2c1a4b")
    node = records.ResolvedNode(
        pkg="lx_bringup",
        executable="rviz2",
        namespace="/",
        name="rviz2",
        name_source="literal",
        argv=("-d", temp),
        entity_path=("Node(rviz2)",),
    )
    walk = records.LaunchWalk(
        entry_path=_abs("src/lx_bringup/launch/plain_node.launch.py"),
        profile="rewritten",
        entry_pkg="lx_bringup",
        nodes=[node],
    )
    atlas = _run_walk(ws, walk, "rewritten", "plain_node.launch.py")
    fact = atlas.get(ids.build_node("/rviz2")).fact(LAYER, ids.build_profile("rewritten"))
    assert fact.values["argv"] == ["-d", launch_declared.EXTERNAL_TOKEN]
    assert temp not in atlas.to_bytes().decode("utf-8")


def test_the_collector_and_the_drift_rule_word_one_refusal_the_same_way(ws: Workspace) -> None:
    """Two wordings for one refusal gave one file two fingerprints and two waivers."""
    from rosio.drift import build as build_rules

    atlas, _ctx = _run_profile(ws, "legacy", entry_for("legacy_ros1"))
    mine = [item for item in atlas.findings if item.rule == launch_declared.RULE_UNLOADABLE]
    theirs = build_rules.launch_unloadable(atlas)
    assert len(mine) == len(theirs) == 1
    assert mine[0].fingerprint == theirs[0].fingerprint
    assert mine[0].id == theirs[0].id


# ---------------------------------------------------------------------------
# the sandbox boundary


def test_a_value_the_boundary_cannot_carry_is_marked_and_the_rest_crosses() -> None:
    """One inline `1e400` used to kill the walk of a launch file real ROS 2 loads, and the
    parent then reported the file as not loadable, which was never true."""
    carried = launch_child._plain({"good_a": 1, "gain": 1e400, "good_b": "two"})
    assert carried["good_a"] == 1
    assert carried["good_b"] == "two"
    mark = launch_child.uncarried_reason(carried["gain"])
    assert mark is not None
    expr, reason = mark
    assert expr == repr(1e400)
    assert model.NON_FINITE in reason
    assert model.dumps(carried)


def test_a_finite_value_carries_its_own_type() -> None:
    carried = launch_child._plain({"i": 20, "f": 20.5, "b": True, "n": None, "l": [1, 2.5]})
    assert carried == {"i": 20, "f": 20.5, "b": True, "n": None, "l": [1, 2.5]}
    assert launch_child.uncarried_reason(carried) is None
    assert launch_child.uncarried_reason({"~uncarried": "text"}) is None


def test_the_last_guard_marks_a_refused_value_anywhere_in_the_payload() -> None:
    """A timer period and an argument default cross the same boundary as a parameter, and
    one of them ending the walk would cost the whole file for one number."""
    payload = {"nodes": [{"gates": [{"period_s": float("inf")}]}], "notes": []}
    scrubbed, refused = launch_child._scrub(payload)
    assert len(refused) == 1
    assert model.NON_FINITE in refused[0]
    assert launch_child.uncarried_reason(scrubbed["nodes"][0]["gates"][0]["period_s"]) is not None
    assert model.dumps(scrubbed)
    again, none = launch_child._scrub({"nodes": [{"gates": [{"period_s": 2.0}]}]})
    assert none == []
    assert again["nodes"][0]["gates"][0]["period_s"] == 2.0


def test_a_marked_value_is_counted_so_the_file_can_state_it() -> None:
    """The note lands on the launch file, which stays loadable and keeps its nodes."""
    payload = {"nodes": [{"params": launch_child._plain({"gain": 1e400})}]}
    scrubbed, refused = launch_child._scrub(payload)
    assert [model.NON_FINITE in reason for reason in refused] == [True]
    assert scrubbed == payload
    assert model.dumps(scrubbed)


@pytest.mark.parametrize("outcomes", [("false", "true"), ("unknown", "unknown")])
def test_two_conditional_loads_of_one_name_are_two_launch_entities(ws: Workspace, outcomes):
    """jazzy's `localization_launch.py` loads `map_server` twice, gated on whether a map was
    given, and one id for both refused the second write and dropped its parameters."""
    trail = ("Include(localization_launch.py)", "ComposableNode(map_server)")
    file_only = records.ParamSource(order=0, kind="repo_file", path="/ws/params.yaml")
    inline = records.ParamSource(order=1, kind="inline_dict", values=(("yaml_filename", "m"),))
    twins = [
        records.ResolvedNode(
            pkg="nav2_map_server",
            executable="",
            namespace="",
            form="composable_node",
            name="map_server",
            name_source="literal",
            plugin_class="nav2_map_server::MapServer",
            params=params,
            entity_path=trail,
            conditions=(condition,),
            gates=(records.Gate(mechanism="IfCondition", expression=condition, outcome=outcome),),
            attrs={"unreachable": True} if outcome == "false" else {},
        )
        for params, condition, outcome in (
            ((file_only,), "map == ''", outcomes[0]),
            ((file_only, inline), "map != ''", outcomes[1]),
        )
    ]
    alone = records.ResolvedNode(
        pkg="nav2_amcl",
        executable="",
        namespace="",
        form="composable_node",
        name="amcl",
        name_source="literal",
        plugin_class="nav2_amcl::AmclNode",
        entity_path=("Include(localization_launch.py)", "ComposableNode(amcl)"),
    )
    trails = launch_declared._distinct_steps([*twins, alone])
    assert trails[0] != trails[1]
    assert trails[0][:-1] == trails[1][:-1]
    assert all(step.disc.startswith("map_server@") for step in (trails[0][-1], trails[1][-1]))
    assert trails[2][-1].disc == "amcl"
    assert launch_declared._distinct_steps([*twins, alone]) == trails
    walk = records.LaunchWalk(
        entry_path=str(WS / "src/lx_bringup/launch/plain_node.launch.py"),
        profile="conditional",
        nodes=twins,
    )
    atlas = _run_walk(ws, walk, "conditional", "plain_node.launch.py")
    assert all(not run.attrs.get("exceptions") for run in atlas.by_kind("run"))
    assert len(edges_of(atlas, "gates")) == 2
    assert sorted(row.facts[0].values["conditions"] for row in atlas.by_kind("launchent")) == [
        ["map != ''"],
        ["map == ''"],
    ]
    assert schema.validate(atlas.to_json()) == []
    assert (
        atlas.canonical_bytes()
        == _run_walk(ws, walk, "conditional", "plain_node.launch.py").canonical_bytes()
    )


def test_launch_entity_evidence_keeps_advisory_lines(ws: Workspace) -> None:
    from dataclasses import replace

    entry = str(WS / "src/lx_bringup/launch/plain_node.launch.py")
    child = str(WS / "src/lx_bringup/launch/include_child.launch.py")
    nodes = [
        records.ResolvedNode(
            pkg="lx_bringup",
            executable="lx_controller",
            namespace="/",
            name=name,
            name_source="literal",
            entity_path=(f"Node({name})",),
            site=records.Site(entry, f"Node({name})", line),
        )
        for name, line in (("first", 7), ("second", 8))
    ]
    include = records.LaunchInclude(
        path=child,
        site=records.Site(entry, "IncludeLaunchDescription", 9),
        entity_path=("Include(include_child.launch.py)",),
    )
    walk = records.LaunchWalk(entry_path=entry, profile="lines", nodes=nodes, includes=[include])
    atlas = _run_walk(ws, walk, "lines", "plain_node.launch.py")
    rows = atlas.by_kind("launchent")
    assert len(rows) == 3
    assert sorted(row.facts[0].evidence[0].line_hint for row in rows) == [7, 8, 9]
    assert schema.validate(atlas.to_json()) == []
    moved = replace(
        walk,
        nodes=[replace(n, site=replace(n.site, line_hint=n.site.line_hint + 20)) for n in nodes],
        includes=[replace(include, site=replace(include.site, line_hint=29))],
    )
    after = _run_walk(ws, moved, "lines", "plain_node.launch.py")
    assert atlas.canonical_bytes() == after.canonical_bytes()
    assert atlas.to_bytes() != after.to_bytes()


def test_includes_with_the_same_basename_keep_existing_node_ids(ws: Workspace):
    from dataclasses import replace

    entry = str(WS / "src/lx_bringup/launch/plain_node.launch.py")
    child = str(WS / "src/lx_bringup/launch/include_child.launch.py")
    other = str(WS / "src/lx_sensors/launch/include_child.launch.py")
    path = ("Include(include_child.launch.py)",)
    nodes = [
        records.ResolvedNode(
            pkg="lx_bringup",
            executable="lx_controller",
            namespace="/",
            name=name,
            name_source="literal",
            entity_path=(*path, f"Node({name})"),
            site=records.Site(source, f"Node({name})", 4),
        )
        for name, source in (("first", child), ("second", other))
    ]
    walk = records.LaunchWalk(
        entry_path=entry,
        profile="twins",
        nodes=nodes,
        includes=[
            records.LaunchInclude(
                path=source,
                entity_path=path,
                site=records.Site(entry, "IncludeLaunchDescription", line),
            )
            for source, line in ((child, 7), (other, 8))
        ],
    )
    baseline = _run_walk(
        ws,
        replace(walk, includes=[replace(i, entity_path=()) for i in walk.includes]),
        "twins",
        "plain_node.launch.py",
    )
    atlas = _run_walk(ws, walk, "twins", "plain_node.launch.py")
    assert node_ids(atlas) == node_ids(baseline) == ["node:/first", "node:/second"]
    assert not next(iter(atlas.by_kind("run"))).attrs["exceptions"]
    assert set(baseline.entities) <= set(atlas.entities)
    assert len(atlas.by_kind("launchent")) == 4
    assert schema.validate(atlas.to_json()) == []


# ---------------------------------------------------------------------------
# walks started ahead of their passes


def _ready(monkeypatch: pytest.MonkeyPatch) -> None:
    from rosio.resolve import launch_exec

    monkeypatch.setattr(launch_exec, "probe", lambda _setup=(): launch_exec.LaunchImport(ok=True))


def _context(ws: Workspace, cases: dict[str, list[str]]) -> CollectContext:
    profiles = tuple(
        ProfileSpec(slug=slug, label=slug, entrypoints=tuple(entry_for(case) for case in names))
        for slug, names in cases.items()
    )
    cfg = Config(
        roots=tuple(RootSpec(kind=spec.kind, path=spec.path) for spec in ws.roots),
        profiles=profiles,
        launch_sandbox="off",
    )
    return CollectContext(ws=ws, cfg=cfg, builder=model.AtlasBuilder())


def test_walks_queued_for_later_passes_stop_with_the_run(
    ws: Workspace, monkeypatch: pytest.MonkeyPatch
) -> None:
    from rosio.resolve import launch_exec

    started: list[str] = []
    release = threading.Event()

    def walk(path: str, **_options: object) -> records.LaunchWalk:
        started.append(path)
        release.wait(5)
        return records.LaunchWalk(entry_path=path)

    _ready(monkeypatch)
    monkeypatch.setattr(launch_exec, "walk", walk)
    monkeypatch.setattr(launch_declared, "WALKERS", 1)
    ctx = _context(ws, {f"p{index}": [case] for index, case in enumerate(ONE_PER_FILE[:4])})
    launch_declared._start_walks(ctx, launch_declared._LaunchLayer(ros_setup=()))
    queued = [future for key, future in ctx.launch_walks.items() if key != "started"]
    assert len(queued) == 4
    ctx.close()
    release.set()
    # A future cancelled before it ran never notifies a waiter, so `futures.wait` would hang.
    deadline = time.monotonic() + 10
    while not all(future.done() for future in queued) and time.monotonic() < deadline:
        time.sleep(0.01)
    assert len(started) <= 1
    assert sum(future.cancelled() for future in queued) >= 3


def test_a_walk_that_timed_out_beside_others_is_walked_again_alone(
    ws: Workspace, monkeypatch: pytest.MonkeyPatch
) -> None:
    from rosio.resolve import launch_exec

    cases = {"first": ["plain_node"], "second": ["param_order"]}
    by_path = {
        _abs(json.loads((EXPECTED / f"{case}.json").read_text())["entry"]): (slug, case)
        for slug, names in cases.items()
        for case in names
    }

    def walk(path: str, **_options: object) -> records.LaunchWalk:
        slug, case = by_path[path]
        if threading.current_thread() is not threading.main_thread():
            timed_out = records.LaunchWalk(entry_path=path, strategy="refused", loadable=False)
            timed_out.exceptions.append(
                records.WalkException(
                    path=path, phase="visit", exc_type="TimeoutExpired", message="slow"
                )
            )
            return timed_out
        return walk_for(case, slug)

    _ready(monkeypatch)
    monkeypatch.setattr(launch_exec, "walk", walk)
    ctx = _context(ws, cases)
    try:
        get(launch_declared.NAME).collect(ctx)
    finally:
        ctx.close()
    assert ctx.builder.build().canonical_bytes() == build(ws, cases).canonical_bytes()
