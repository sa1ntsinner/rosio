"""The observed layer: the probe codec, the collector, and the process table reader."""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from rosio import ids, model, schema
from rosio.collect import CollectContext
from rosio.collect import probe as probe_collect
from rosio.collect import proc as proc_collect
from rosio.config import Config, ProfileSpec, RootSpec
from rosio.errors import CollectorError
from rosio.resolve import identity
from rosio.workspace import Workspace

PROFILE = "live"
PROFILE_ID = ids.build_profile(PROFILE)


# ---------------------------------------------------------------------------
# fixtures


@pytest.fixture
def ws(workspace: Path) -> Workspace:
    (workspace / "src" / "fx_pkg").mkdir(parents=True)
    (workspace / "src" / "fx_pkg" / "package.xml").write_text(
        "<package><name>fx_pkg</name></package>", encoding="utf-8"
    )
    return Workspace.discover([RootSpec("src", str(workspace / "src"))])


@pytest.fixture
def cfg(ws: Workspace) -> Config:
    return Config(
        roots=tuple(RootSpec(kind=spec.kind, path=spec.path) for spec in ws.roots),
        profiles=(ProfileSpec(slug=PROFILE),),
    )


@pytest.fixture
def ctx(ws: Workspace, cfg: Config) -> CollectContext:
    builder = model.AtlasBuilder()
    builder.add_profile(PROFILE)
    return CollectContext(ws=ws, cfg=cfg, builder=builder, profile=PROFILE)


def _system() -> probe_collect.ObservedSystem:
    """One small running system: a talker, a listener with a remap, and a tf broadcaster."""
    qos = probe_collect.Qos(
        history="keep_last", depth=10, reliability="reliable", durability="volatile"
    )
    return probe_collect.ObservedSystem(
        nodes=[
            probe_collect.ObservedNode(
                name="talker",
                namespace="/",
                fqn="/talker",
                parameters=(
                    probe_collect.Parameter(name="use_sim_time", value=False, type="bool"),
                    probe_collect.Parameter(name="loop_period", value=20, type="integer"),
                ),
            ),
            probe_collect.ObservedNode(
                name="listener",
                namespace="/demo",
                fqn="/demo/listener",
                lifecycle_state="active",
            ),
            probe_collect.ObservedNode(
                name="robot_state_publisher",
                namespace="/",
                fqn="/robot_state_publisher",
                parameters_read=False,
                parameters_reason="the node did not answer list_parameters within the timeout",
            ),
        ],
        endpoints=[
            probe_collect.Endpoint(
                kind="publisher",
                node="/talker",
                name="/chatter",
                type="std_msgs/msg/String",
                qos=qos,
            ),
            probe_collect.Endpoint(
                kind="subscriber",
                node="/demo/listener",
                name="/chatter",
                type="std_msgs/msg/String",
                qos=None,
                qos_reason="the middleware reported no endpoint information for this end",
            ),
            probe_collect.Endpoint(
                kind="service_server",
                node="/talker",
                name="/talker/describe_parameters",
                type="rcl_interfaces/srv/DescribeParameters",
            ),
            probe_collect.Endpoint(
                kind="publisher",
                node="/robot_state_publisher",
                name="/tf_static",
                type="tf2_msgs/msg/TFMessage",
                qos=qos,
            ),
        ],
        tf=[
            probe_collect.TfLink(
                parent="base_link",
                child="os_sensor",
                static=True,
                broadcaster="/robot_state_publisher",
            ),
            probe_collect.TfLink(
                parent="odom",
                child="base_footprint",
                static=False,
                broadcaster=None,
                broadcaster_reason="2 nodes publish /tf",
            ),
        ],
        rates=[
            probe_collect.TopicRate(topic="/chatter", count=42, window_s=5.0, hz=8.4),
            probe_collect.TopicRate(topic="/tf_static", count=1, window_s=5.0, hz=0.2),
        ],
        domain_id="77",
        localhost_only="1",
    ).sort()


def _collect(ctx: CollectContext, system: probe_collect.ObservedSystem) -> model.Atlas:
    probe_collect.ProbeCollector(system=system).collect(ctx)
    return ctx.builder.build()


# ---------------------------------------------------------------------------
# codec


def test_the_codec_round_trips_every_field() -> None:
    system = _system()
    again = probe_collect.decode_system(probe_collect.encode_system(system))
    assert probe_collect.encode_system(again) == probe_collect.encode_system(system)
    talker = next(item for item in again.nodes if item.fqn == "/talker")
    assert talker.parameters[0].value is False
    published = next(
        item for item in again.endpoints if item.name == "/chatter" and item.kind == "publisher"
    )
    assert published.qos is not None
    assert published.qos.depth == 10


def test_a_qos_nobody_stated_stays_none() -> None:
    payload = probe_collect.encode_system(_system())
    subscriber = next(item for item in payload["endpoints"] if item["kind"] == "subscriber")
    assert subscriber["qos"] is None


def test_the_payload_is_json_and_sorted() -> None:
    first = model.dumps(probe_collect.encode_system(_system()))
    second = model.dumps(probe_collect.encode_system(_system()))
    assert first == second


# ---------------------------------------------------------------------------
# the collector


def test_the_collector_writes_nodes_endpoints_and_frames(ctx: CollectContext) -> None:
    atlas = _collect(ctx, _system())
    assert schema.validate(atlas.to_json()) == []
    talker = atlas.get(ids.build_node("/talker"))
    assert talker is not None
    assert talker.name_source == model.NameSource.OBSERVED.value
    fact = talker.fact(model.Layer.OBSERVED.value, PROFILE_ID)
    assert fact is not None
    assert fact.values["namespace"] == "/"
    listener = atlas.get(ids.build_node("/demo/listener"))
    assert listener is not None
    assert listener.fact(model.Layer.OBSERVED.value, PROFILE_ID).values["lifecycle_state"] == (
        "active"
    )
    assert atlas.get(ids.build_topic("/chatter")) is not None
    assert atlas.get(ids.build_service("/talker/describe_parameters")) is not None
    assert atlas.get(ids.build_frame("os_sensor")) is not None


def test_two_runs_of_one_system_are_byte_identical(ctx: CollectContext, ws, cfg) -> None:
    first = _collect(ctx, _system())
    second_ctx = CollectContext(ws=ws, cfg=cfg, builder=model.AtlasBuilder(), profile=PROFILE)
    second_ctx.builder.add_profile(PROFILE)
    second = _collect(second_ctx, _system())
    assert first.canonical_bytes() == second.canonical_bytes()


def test_the_measured_rate_is_volatile_and_stays_out_of_compared_data(
    ctx: CollectContext,
) -> None:
    atlas = _collect(ctx, _system())
    handle = next(
        key
        for key, edge in atlas.edges.items()
        if edge.kind == model.EdgeKind.PUBLISHES.value and edge.dst == ids.build_topic("/chatter")
    )
    edge = atlas.edges[handle]
    fact = edge.fact(model.Layer.OBSERVED.value, PROFILE_ID)
    assert fact.values["volatile"]["rate_hz"] == 8.4
    assert b"rate_hz" not in atlas.canonical_bytes()


def test_a_publisher_carries_the_qos_the_middleware_stated(ctx: CollectContext) -> None:
    atlas = _collect(ctx, _system())
    edge = next(
        item
        for item in atlas.edges.values()
        if item.kind == model.EdgeKind.PUBLISHES.value and item.dst == ids.build_topic("/chatter")
    )
    qos = edge.fact(model.Layer.OBSERVED.value, PROFILE_ID).values["qos"]
    assert qos == {
        "history": "keep_last",
        "depth": 10,
        "reliability": "reliable",
        "durability": "volatile",
    }


def test_an_unstated_qos_becomes_a_blind_spot_and_never_a_default(ctx: CollectContext) -> None:
    atlas = _collect(ctx, _system())
    edge = next(
        item for item in atlas.edges.values() if item.kind == model.EdgeKind.SUBSCRIBES.value
    )
    assert "qos" not in edge.fact(model.Layer.OBSERVED.value, PROFILE_ID).values
    wants = {entry.want for entry in atlas.unresolved}
    assert model.Want.QOS.value in wants


def test_a_qos_stating_only_liveliness_is_still_a_blind_spot(ctx: CollectContext) -> None:
    """Liveliness is not one of the four keys the comparison reads, so an endpoint that
    states nothing else knows nothing the comparison can use."""
    system = _system()
    system.endpoints.append(
        probe_collect.Endpoint(
            kind="publisher",
            node="/talker",
            name="/thin",
            type="std_msgs/msg/String",
            qos=probe_collect.Qos(liveliness="automatic"),
        )
    )
    atlas = _collect(ctx, system.sort())
    rows = [
        entry
        for entry in atlas.unresolved
        if entry.want == model.Want.QOS.value and "/thin" in entry.expr
    ]
    assert len(rows) == 1
    edge = next(
        item
        for item in atlas.edges.values()
        if item.kind == model.EdgeKind.PUBLISHES.value and item.dst == ids.build_topic("/thin")
    )
    assert edge.fact(model.Layer.OBSERVED.value, PROFILE_ID).values["qos"]["reliability"] is None


def test_observed_parameters_are_facts_with_a_state(ctx: CollectContext) -> None:
    atlas = _collect(ctx, _system())
    param = atlas.get(ids.build_param(ids.build_node("/talker"), "loop_period"))
    assert param is not None
    assert param.state == model.ParamState.EFFECTIVE.value
    fact = param.fact(model.Layer.OBSERVED.value, PROFILE_ID)
    assert fact.values["value"] == 20
    assert isinstance(fact.values["value"], int)
    assert any(
        edge.kind == model.EdgeKind.HAS_PARAM.value and edge.dst == param.id
        for edge in atlas.edges.values()
    )


def test_a_declared_chain_keeps_its_state_when_the_probe_speaks(ctx: CollectContext) -> None:
    node_id = ids.build_node("/talker")
    param_id = ids.build_param(node_id, "loop_period")
    ctx.builder.set_param(
        param_id,
        state=model.ParamState.OWNER_NAME_MISMATCH.value,
        candidates=[
            model.Candidate(
                value=5,
                value_type="int",
                mechanism=model.Mechanism.YAML_BASE.value,
                source=model.Source(file=ids.build_file("fx_pkg", "config/fx.yaml")),
                rank=0,
                wins=False,
                profile=PROFILE_ID,
            )
        ],
    )
    atlas = _collect(ctx, _system())
    param = atlas.get(param_id)
    assert param.state == model.ParamState.OWNER_NAME_MISMATCH.value
    assert param.fact(model.Layer.OBSERVED.value, PROFILE_ID).values["value"] == 20


def test_a_path_value_reads_the_same_in_the_declared_and_the_observed_layer(
    ctx: CollectContext, ws: Workspace
) -> None:
    """The declared chain names a root by its label, so the probe has to as well, or every
    path valued parameter disagrees with itself and no absolute path reaches the slot."""
    path = f"{ws.roots[0].path}/fx_pkg/maps/map.yaml"
    node_id = ids.build_node("/map_server")
    param_id = ids.build_param(node_id, "yaml_filename")
    ctx.builder.fact(
        param_id,
        model.Layer.DECLARED.value,
        profile=PROFILE_ID,
        values={"value": ws.stated_value(path)},
        confidence=model.Confidence.CERTAIN.value,
    )
    system = probe_collect.ObservedSystem(
        nodes=[
            probe_collect.ObservedNode(
                name="map_server",
                namespace="/",
                fqn="/map_server",
                parameters=(
                    probe_collect.Parameter(name="yaml_filename", value=path, type="string"),
                ),
            )
        ],
    ).sort()
    param = _collect(ctx, system).get(param_id)
    assert param.fact(model.Layer.OBSERVED.value, PROFILE_ID).values["value"] == (
        "@src/fx_pkg/maps/map.yaml"
    )
    assert param.conflicts == []
    assert param.agreement == "agree"


def test_a_node_that_did_not_answer_is_unread_and_never_empty(ctx: CollectContext) -> None:
    atlas = _collect(ctx, _system())
    owner = ids.build_node("/robot_state_publisher")
    rows = [entry for entry in atlas.unresolved if entry.owner == owner]
    assert [entry.want for entry in rows] == [model.Want.PARAM_VALUE.value]
    assert not [
        edge
        for edge in atlas.edges.values()
        if edge.kind == model.EdgeKind.HAS_PARAM.value and edge.src == owner
    ]


def test_one_publisher_on_tf_static_attributes_the_frame(ctx: CollectContext) -> None:
    atlas = _collect(ctx, _system())
    edges = [
        edge for edge in atlas.edges.values() if edge.kind == model.EdgeKind.BROADCASTS_TF.value
    ]
    assert [edge.dst for edge in edges] == [ids.build_frame("os_sensor")]
    parent = next(
        edge
        for edge in atlas.edges.values()
        if edge.kind == model.EdgeKind.TF_PARENT.value and edge.src == ids.build_frame("os_sensor")
    )
    assert parent.dst == ids.build_frame("base_link")
    assert parent.fact(model.Layer.OBSERVED.value, PROFILE_ID).values["static"] is True


def test_two_publishers_on_tf_leave_the_broadcaster_unknown(ctx: CollectContext) -> None:
    atlas = _collect(ctx, _system())
    owner = ids.build_frame("base_footprint")
    rows = [entry for entry in atlas.unresolved if entry.owner == owner]
    assert [entry.want for entry in rows] == [model.Want.NODE_NAME.value]
    assert not [
        edge
        for edge in atlas.edges.values()
        if edge.kind == model.EdgeKind.BROADCASTS_TF.value and edge.dst == owner
    ]


def test_a_system_with_no_node_declares_the_layer_absent(ctx: CollectContext) -> None:
    empty = probe_collect.ObservedSystem(domain_id="77")
    probe_collect.ProbeCollector(system=empty).collect(ctx)
    atlas = ctx.builder.build()
    assert atlas.layers[model.Layer.OBSERVED.value].present is False
    assert not [node for node in atlas.entities.values() if node.kind == "node"]


def test_a_probe_that_never_attached_does_not_claim_it_reached_the_middleware(
    ctx: CollectContext,
) -> None:
    system = probe_collect.ObservedSystem(
        reached=False,
        exceptions=[
            probe_collect.ProbeException(
                phase="attach",
                exc_type="ModuleNotFoundError",
                message="No module named 'rclpy'",
            )
        ],
    )
    probe_collect.ProbeCollector(system=system).collect(ctx)
    atlas = ctx.builder.build()
    layer = atlas.layers[model.Layer.OBSERVED.value]
    assert layer.present is False
    assert "never attached" in layer.reason
    assert "rclpy" in layer.reason


def test_the_run_records_the_domain_and_keeps_the_window_volatile(ctx: CollectContext) -> None:
    atlas = _collect(ctx, _system())
    run = next(node for node in atlas.entities.values() if node.kind == "run")
    assert run.attrs["env"]["ROS_DOMAIN_ID"] == "77"
    assert run.attrs["volatile"]["window_s"] == probe_collect.DEFAULT_WINDOW
    assert b"window_s" not in atlas.canonical_bytes()


def test_what_the_probe_knows_it_did_not_read_reaches_the_atlas(ctx: CollectContext) -> None:
    """A note names a blind spot: two nodes answering to one name, endpoints read only in
    part, a node that answered after the graph was read. A note that stays in the record is
    a blind spot no reader can find."""
    system = _system()
    system.notes.append("2 running nodes answer to /talker, so the endpoints of one were read")
    atlas = _collect(ctx, system.sort())
    run = next(node for node in atlas.entities.values() if node.kind == "run")
    assert run.attrs["notes"] == [
        "2 running nodes answer to /talker, so the endpoints of one were read"
    ]
    assert b"2 running nodes answer" in atlas.canonical_bytes()


def test_summary_counts_what_was_seen() -> None:
    counts = probe_collect.summary(_system())
    assert counts["nodes"] == 3
    assert counts["endpoints"]["publisher"] == 2
    assert counts["frames"] == 2


def test_internal_names_are_tooling() -> None:
    assert probe_collect.internal_name("_ros2cli_daemon_0")
    assert probe_collect.internal_name("rosio_probe")
    assert not probe_collect.internal_name("controller_server")


def test_an_endpoint_kind_the_table_does_not_know_is_a_blind_spot(ctx: CollectContext) -> None:
    system = _system()
    system.endpoints.append(
        probe_collect.Endpoint(kind="intra_process", node="/talker", name="/inner", type="t")
    )
    atlas = _collect(ctx, system.sort())
    owner = ids.build_node("/talker")
    rows = [
        entry
        for entry in atlas.unresolved
        if entry.owner == owner and entry.want == model.Want.OTHER.value
    ]
    assert len(rows) == 1
    assert atlas.get(ids.build_topic("/inner")) is None
    assert atlas.get(ids.build_topic("/chatter")) is not None


def test_the_asked_for_domain_is_exported_after_the_setup_files() -> None:
    """A login profile and a workspace setup file both export ROS_DOMAIN_ID, and a probe
    that attaches to a domain the caller did not ask for is the worst failure here."""
    line = probe_collect._sourced(
        ["/opt/ros/humble/setup.bash"],
        "exec python3 child.py",
        {"ROS_DOMAIN_ID": "77", "ROS_LOCALHOST_ONLY": "1"},
    )
    assert line.index("setup.bash") < line.index("export ROS_DOMAIN_ID=77")
    assert line.index("export ROS_LOCALHOST_ONLY=1") < line.index("exec python3")


def test_the_collector_pins_the_domain_the_caller_is_on(
    ctx: CollectContext, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A setup file that exports ROS_DOMAIN_ID is sourced by the child, so the domain the
    caller is on has to be asked for or the probe attaches to another system."""
    monkeypatch.setenv("ROS_DOMAIN_ID", "77")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    seen: dict[str, object] = {}

    def _fake(**kwargs: object) -> probe_collect.ObservedSystem:
        seen.update(kwargs)
        return _system()

    monkeypatch.setattr(probe_collect, "probe", _fake)
    probe_collect.ProbeCollector().collect(ctx)
    assert seen["env"] == {"ROS_DOMAIN_ID": "77", "ROS_LOCALHOST_ONLY": "1"}


def test_the_two_halves_of_the_layer_read_in_either_order(ws, cfg) -> None:
    """The probe and the process table describe one running node. One slot per layer and
    profile is written by one run, so the second half adds keys to the slot the first half
    opened, whichever half ran first."""
    rows = [proc_collect.parse(TALKER_ARGV, pid=4242)]

    def build(probe_first: bool) -> model.Atlas:
        builder = model.AtlasBuilder()
        builder.add_profile(PROFILE)
        ctx = CollectContext(ws=ws, cfg=cfg, builder=builder, profile=PROFILE)
        order = ["probe", "proc"] if probe_first else ["proc", "probe"]
        for name in order:
            if name == "probe":
                probe_collect.ProbeCollector(system=_talking_system()).collect(ctx)
            else:
                proc_collect.ProcCollector(processes=rows).collect(ctx)
        return builder.build()

    first, second = build(True), build(False)
    for atlas in (first, second):
        node = atlas.get(ids.build_node("/demo/talker"))
        values = node.fact(model.Layer.OBSERVED.value, PROFILE_ID).values
        assert values["executable"] == "talker"
        assert values["namespace"] == "/demo"
        assert node.name_source == model.NameSource.ARGV.value
    assert first.entities.keys() == second.entities.keys()


def _talking_system() -> probe_collect.ObservedSystem:
    """The same node the process table row names, as the middleware answers for it."""
    return probe_collect.ObservedSystem(
        nodes=[
            probe_collect.ObservedNode(name="talker", namespace="/demo", fqn="/demo/talker"),
        ],
        domain_id="77",
        localhost_only="1",
    ).sort()


def test_the_probe_never_replaces_a_name_source_another_layer_stated(ctx: CollectContext) -> None:
    node_id = ids.build_node("/talker")
    ctx.builder.node(node_id, name_source=model.NameSource.LITERAL.value)
    atlas = _collect(ctx, _system())
    assert atlas.get(node_id).name_source == model.NameSource.LITERAL.value
    assert atlas.get(ids.build_node("/demo/listener")).name_source == (
        model.NameSource.OBSERVED.value
    )


def test_a_missing_child_is_a_collector_error(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(probe_collect, "CHILD", "/nonexistent/_probe_child.py")
    with pytest.raises(CollectorError):
        probe_collect.probe(window=0.0, settle=0.0, timeout=30.0)


# ---------------------------------------------------------------------------
# the process table


TALKER_ARGV = (
    "/opt/ros/humble/lib/demo_nodes_cpp/talker",
    "--ros-args",
    "-r",
    "__node:=talker",
    "-r",
    "__ns:=/demo",
    "-r",
    "chatter:=/demo/chatter",
    "--params-file",
    "/tmp/launch_params_9f2c",
    "-p",
    "use_sim_time:=true",
)


def test_an_argv_with_ros_args_recovers_the_whole_identity() -> None:
    found = proc_collect.parse(TALKER_ARGV, pid=4242)
    assert found is not None
    assert found.fqn == "/demo/talker"
    assert found.executable == "talker"
    assert found.params_files == ("/tmp/launch_params_9f2c",)
    assert ("chatter", "/demo/chatter") in found.remaps
    assert found.assignments == (("use_sim_time", "true"),)
    assert found.pid == 4242
    assert found.is_node


def test_a_process_without_ros_args_is_not_claimed() -> None:
    assert proc_collect.parse(("/usr/bin/vim", "notes.md")) is None
    assert proc_collect.parse(()) is None


def test_a_ros2_command_line_is_a_launcher_and_not_a_node() -> None:
    found = proc_collect.parse(("/usr/bin/ros2", "launch", "fx_pkg", "nav.launch.py"))
    assert found is not None
    assert found.is_launcher
    assert found.verb == "launch"
    assert found.fqn is None
    python = proc_collect.parse(("/usr/bin/python3", "/opt/ros/humble/bin/ros2", "run", "a", "b"))
    assert python is not None
    assert python.verb == "run"
    assert python.target == "a b"


def test_the_ros2_shebang_does_not_eat_the_thing_the_command_starts() -> None:
    """The entry point carries a shebang, so a real process table shows the interpreter
    first and the launch file is the token that would fall off the end."""
    found = proc_collect.parse(
        (
            "/usr/bin/python3",
            "/opt/ros/humble/bin/ros2",
            "launch",
            "fx_pkg",
            "nav.launch.py",
            "use_sim:=false",
        )
    )
    assert found is not None
    assert found.target == "fx_pkg nav.launch.py"


def test_an_argv_with_no_node_assignment_names_no_node() -> None:
    found = proc_collect.parse(("/opt/ros/humble/lib/rviz2/rviz2", "--ros-args"))
    assert found is not None
    assert found.fqn is None
    assert found.is_node


def test_read_table_reads_command_lines(tmp_path: Path) -> None:
    (tmp_path / "17").mkdir()
    (tmp_path / "17" / "cmdline").write_bytes(b"/bin/talker\0--ros-args\0-r\0__node:=talker\0")
    (tmp_path / "self").mkdir()
    rows = proc_collect.read_table(str(tmp_path))
    assert rows == [(17, ("/bin/talker", "--ros-args", "-r", "__node:=talker"))]


def test_read_table_refuses_a_machine_with_no_process_table(tmp_path: Path) -> None:
    with pytest.raises(CollectorError):
        proc_collect.read_table(str(tmp_path / "missing"))


def test_scan_sorts_and_never_raises_on_a_vanished_process(tmp_path: Path) -> None:
    for pid, line in ((3, b"/bin/b\0--ros-args\0-r\0__node:=b\0"), (2, b"/bin/a\0")):
        (tmp_path / str(pid)).mkdir()
        (tmp_path / str(pid) / "cmdline").write_bytes(line)
    (tmp_path / "9").mkdir()
    rows = proc_collect.scan(str(tmp_path))
    assert [item.fqn for item in rows] == ["/b"]


def test_match_binds_by_name_and_by_executable() -> None:
    rows = [proc_collect.parse(TALKER_ARGV)]
    assert proc_collect.match(rows, fqn="/demo/talker") == rows
    assert proc_collect.match(rows, fqn="/talker") == []
    assert proc_collect.match(rows, executable="talker") == rows


def test_the_proc_collector_writes_argv_with_no_machine_path(ctx: CollectContext) -> None:
    rows = [proc_collect.parse(TALKER_ARGV, pid=4242)]
    proc_collect.ProcCollector(processes=rows).collect(ctx)
    atlas = ctx.builder.build()
    assert schema.validate(atlas.to_json()) == []
    node = atlas.get(ids.build_node("/demo/talker"))
    assert node is not None
    assert node.name_source == model.NameSource.ARGV.value
    values = node.fact(model.Layer.OBSERVED.value, PROFILE_ID).values
    assert values["executable"] == "talker"
    assert values["params_files"] == [
        identity.relative_path("/tmp/launch_params_9f2c", [spec.path for spec in ctx.ws.roots])
    ]
    assert not values["params_files"][0].startswith("/")
    assert not [item for item in values["argv"] if item.startswith("/opt/")]
    assert not [item for item in values["argv"] if item.startswith("/tmp/")]


def test_a_parameter_assignment_never_carries_the_workspace_path(
    ctx: CollectContext, ws: Workspace
) -> None:
    path = f"{ws.roots[0].path}/fx_pkg/maps/map.yaml"
    argv = (*TALKER_ARGV, "-p", f"yaml_filename:={path}")
    proc_collect.ProcCollector(processes=[proc_collect.parse(argv)]).collect(ctx)
    atlas = ctx.builder.build()
    values = atlas.get(ids.build_node("/demo/talker")).fact(model.Layer.OBSERVED.value, PROFILE_ID)
    assert ["yaml_filename", "@src/fx_pkg/maps/map.yaml"] in values.values["assignments"]
    assert "yaml_filename:=@src/fx_pkg/maps/map.yaml" in values.values["argv"]
    assert ws.roots[0].path.encode() not in atlas.canonical_bytes()


def test_the_proc_collector_records_an_unnamed_process_as_a_blind_spot(
    ctx: CollectContext,
) -> None:
    rows = [proc_collect.parse(("/opt/ros/humble/lib/rviz2/rviz2", "--ros-args"))]
    proc_collect.ProcCollector(processes=rows).collect(ctx)
    atlas = ctx.builder.build()
    assert [entry.want for entry in atlas.unresolved] == [model.Want.NODE_NAME.value]
    assert not [node for node in atlas.entities.values() if node.kind == "node"]


def test_the_proc_collector_keeps_the_pid_out_of_compared_data(ctx: CollectContext) -> None:
    rows = [proc_collect.parse(TALKER_ARGV, pid=4242)]
    proc_collect.ProcCollector(processes=rows).collect(ctx)
    atlas = ctx.builder.build()
    assert b"4242" not in atlas.canonical_bytes()
    run = next(node for node in atlas.entities.values() if node.kind == "run")
    assert run.attrs["volatile"]["pids"] == [4242]


def test_the_proc_collector_records_the_launcher_command(ctx: CollectContext) -> None:
    rows = [proc_collect.parse(("/usr/bin/ros2", "launch", "fx_pkg", "nav.launch.py"))]
    proc_collect.ProcCollector(processes=rows).collect(ctx)
    atlas = ctx.builder.build()
    run = next(node for node in atlas.entities.values() if node.kind == "run")
    assert run.attrs["commands"] == ["launch fx_pkg nav.launch.py"]


def test_a_launch_file_named_by_path_never_reaches_the_command_raw(ctx: CollectContext) -> None:
    rows = [proc_collect.parse(("/usr/bin/ros2", "launch", "/home/operator/nav.launch.py"))]
    proc_collect.ProcCollector(processes=rows).collect(ctx)
    atlas = ctx.builder.build()
    run = next(node for node in atlas.entities.values() if node.kind == "run")
    assert run.attrs["commands"] == ["launch @outside/nav.launch.py"]
    assert b"/home/operator" not in atlas.canonical_bytes()


def test_summarise_counts_a_scan() -> None:
    rows = [
        proc_collect.parse(TALKER_ARGV),
        proc_collect.parse(("/usr/bin/ros2", "launch", "fx_pkg", "nav.launch.py")),
    ]
    counts = proc_collect.summarise(rows)
    assert counts == {"processes": 2, "named": 1, "launchers": 1, "params_files": 1}


def test_the_real_process_table_is_readable() -> None:
    if not proc_collect.available():
        pytest.skip("no /proc on this machine")
    rows = proc_collect.read_table()
    assert rows
    assert all(isinstance(pid, int) and argv for pid, argv in rows)


# ---------------------------------------------------------------------------
# live, and only ever on the isolated domain


@pytest.mark.ros
@pytest.mark.slow
def test_the_probe_attaches_to_an_isolated_domain(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ROS_DOMAIN_ID", "77")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    setup = [
        path
        for path in (f"/opt/ros/{os.environ.get('ROS_DISTRO', 'humble')}/setup.bash",)
        if Path(path).is_file()
    ]
    system = probe_collect.probe(window=1.0, settle=1.0, ros_setup=setup)
    assert system.reached
    assert system.domain_id == "77"
    names = [item.name for item in system.nodes]
    assert "rosio_probe" in names
    assert all(item.internal for item in system.nodes if item.name == "rosio_probe")


@pytest.mark.parametrize("order, marker", [("little", b"\x00\x01"), ("big", b"\x00\x00")])
@pytest.mark.parametrize("sec", [123, -2])
def test_header_stamp_cdr(order, marker, sec):
    from rosio.collect._probe_child import header_stamp

    raw = marker + b"\x00\x00" + sec.to_bytes(4, order, signed=True)
    raw += (456789).to_bytes(4, order)
    assert header_stamp(raw) == sec * 1_000_000_000 + 456789
    assert header_stamp(raw[:11]) is None
    assert header_stamp(b"\x00\x03" + raw[2:]) is None


@pytest.mark.ros
def test_header_stamp_serialized_laserscan():
    serialization = pytest.importorskip("rclpy.serialization")
    sensor = pytest.importorskip("sensor_msgs.msg")
    from rosio.collect._probe_child import header_stamp

    message = sensor.LaserScan()
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = 987654321
    assert header_stamp(serialization.serialize_message(message)) == 123987654321


def test_stamp_callback_keeps_counting_after_sample_limit(monkeypatch):
    from rosio.collect import _probe_child as child

    counts = {}
    samples = probe_collect.TopicStamps("/scan")
    monkeypatch.setattr(child.time, "time_ns", lambda: 123)
    callback = child._counter(counts, "/scan", samples)
    raw = b"\x00\x01\x00\x00" + (4).to_bytes(4, "little") + bytes(4)
    for _ in range(10003):
        callback(raw)
    assert counts["/scan"] == 10003
    assert len(samples.stamp_ns) == len(samples.receive_ns) == 10000
    assert samples.stamp_ns[0] == 4_000_000_000
    assert samples.receive_ns[0] == 123


def _stamp_system(jump=500_000_000):
    receive = [1_800_000_000_000_000_000 + i * 50_000_000 for i in range(80)]
    return probe_collect.ObservedSystem(
        nodes=[probe_collect.ObservedNode("sensor", "/", "/sensor")],
        endpoints=[
            probe_collect.Endpoint("publisher", "/sensor", topic, "sensor_msgs/msg/LaserScan")
            for topic in ("/scan", "/clean")
        ],
        stamps=[
            probe_collect.TopicStamps(
                "/scan",
                receive,
                [stamp - (jump if i >= 40 else 0) for i, stamp in enumerate(receive)],
            ),
            probe_collect.TopicStamps("/clean", receive, receive.copy()),
        ],
    )


def test_observed_stamp_checks_are_volatile_and_findings_stable(ctx, monkeypatch):
    from rosio.drift import rules

    system = _stamp_system().sort()
    payload = probe_collect.encode_system(system)
    assert probe_collect.encode_system(probe_collect.decode_system(payload)) == payload
    other = probe_collect.encode_system(_stamp_system(700_000_000))
    merged = probe_collect.merge_observations({"a": payload, "b": other})
    assert merged.stamps == system.sort().stamps
    source = model.Source(ids.build_file("fx_pkg", "sensor.py"), "publish", 12)
    ctx.builder.node(source.file)
    ctx.builder.node(ids.build_node("/sensor"), name_source="literal")
    ctx.builder.node(ids.build_topic("/scan"))
    ctx.builder.edge(
        "publishes",
        ids.build_node("/sensor"),
        ids.build_topic("/scan"),
        key=ids.edge_key(site="sensor"),
        layer="coded",
        evidence=[source],
    )
    monkeypatch.setattr(probe_collect, "probe", lambda **kwargs: system)
    probe_collect.ProbeCollector(stamps=True, processes=[], rmws=["default"]).collect(ctx)
    atlas = ctx.builder.build()
    assert schema.validate(atlas.to_json()) == []
    found = rules.findings_for(atlas, only=["time.backward"])
    assert len(found) == 1
    assert found[0].subject == "topic:/scan"
    checks = (
        atlas.entities["topic:/scan"]
        .fact("observed", PROFILE_ID)
        .values["volatile"]["stamps"]["checks"]
    )
    backward = next(row for row in checks if row["check"] == "monotonic")
    measured = backward["evidence"]["largest_backward_jump"]
    assert measured == pytest.approx(0.45, abs=1e-6)
    assert found[0].measured == f"{measured} s"
    assert "in the running system" in found[0].say
    assert str(measured) not in found[0].say + found[0].detail + found[0].title
    assert found[0].where == [source]
    assert "source clock stepped backwards" in found[0].fix
    run = atlas.by_kind("run")[0]
    assert "stamps" not in run.attrs["observations"]["default"]
    assert "receive_ns" not in atlas.to_bytes().decode()
    repeat = CollectContext(ws=ctx.ws, cfg=ctx.cfg, builder=model.AtlasBuilder(), profile=PROFILE)
    repeat.builder.add_profile(PROFILE)
    repeat.builder.node(source.file)
    repeat.builder.node(ids.build_node("/sensor"), name_source="literal")
    repeat.builder.node(ids.build_topic("/scan"))
    repeat.builder.edge(
        "publishes",
        ids.build_node("/sensor"),
        ids.build_topic("/scan"),
        key=ids.edge_key(site="sensor"),
        layer="coded",
        evidence=[source],
    )
    probe_collect.ProbeCollector(stamps=True, processes=[], rmws=["default"]).collect(repeat)
    repeated = repeat.builder.build()
    assert atlas.canonical_bytes() == repeated.canonical_bytes()
    before = atlas.canonical_bytes()
    topic = atlas.entities["topic:/scan"].fact("observed", PROFILE_ID)
    next(
        row for row in topic.values["volatile"]["stamps"]["checks"] if row["check"] == "monotonic"
    )["evidence"]["largest_backward_jump"] = 0.65
    assert atlas.canonical_bytes() == before
    again = rules.findings_for(atlas, only=["time.backward"])
    assert again[0].fingerprint == found[0].fingerprint
    assert again[0].detail == found[0].detail
    assert again[0].say == found[0].say
    assert again[0].measured != found[0].measured


@pytest.mark.ros
@pytest.mark.slow
def test_live_header_stamp_backward(ctx, tmp_path, monkeypatch):
    import subprocess
    import sys
    import uuid

    pytest.importorskip("rclpy")
    pytest.importorskip("sensor_msgs.msg")
    from rosio.drift import rules

    domain = str(1 + int(uuid.uuid4().hex[:8], 16) % 220)
    monkeypatch.setenv("ROS_DOMAIN_ID", domain)
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.delenv("RMW_IMPLEMENTATION", raising=False)
    script = tmp_path / "publisher.py"
    script.write_text(
        """import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
rclpy.init()
node = Node("stamp_sensor")
publisher = node.create_publisher(LaserScan, "/stamp_scan", 10)
def publish():
    message = LaserScan()
    stamp = time.time_ns()
    if int(time.monotonic() * 2) % 2:
        stamp -= 500_000_000
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    publisher.publish(message)
node.create_timer(0.05, publish)
print("ready", flush=True)
rclpy.spin(node)
""",
        encoding="utf-8",
    )
    child = subprocess.Popen(
        [sys.executable, str(script)],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    try:
        # Initialisation precedes DDS discovery, so allow both before taking the graph.
        assert child.stdout.readline().strip() == "ready"
        probe_collect.ProbeCollector(
            window=4, settle=8, stamps=True, processes=[], rmws=["default"]
        ).collect(ctx)
        atlas = ctx.builder.build()
        found = rules.findings_for(atlas, only=["time.backward"])
        seen = atlas.entities.get("topic:/stamp_scan")
        assert [row.subject for row in found] == ["topic:/stamp_scan"], (
            [fact.values for fact in seen.facts] if seen else sorted(atlas.entities)
        )
        assert schema.validate(atlas.to_json()) == []
    finally:
        child.terminate()
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait(timeout=5)


def test_two_endpoints_of_one_node_on_one_topic_keep_no_order_dependent_qos() -> None:
    """The graph names an endpoint by node and topic only, so two publishers of one node with
    two policies cannot be told apart, and whichever arrived last must not win."""
    from types import SimpleNamespace

    from rosio.collect import _probe_child

    def end(reliability: int) -> SimpleNamespace:
        qos = SimpleNamespace(
            history=1, depth=10, reliability=reliability, durability=2, liveliness=1
        )
        return SimpleNamespace(
            node_namespace="/",
            node_name="worker",
            qos_profile=qos,
            topic_type="std_msgs/msg/String",
        )

    def observed(order: list[int]) -> list:
        child = _probe_child.Probe({})
        child.node = SimpleNamespace(
            get_publishers_info_by_topic=lambda _topic: [end(item) for item in order],
            get_subscriptions_info_by_topic=lambda _topic: [],
        )
        child._endpoints({"/worker": {"rows": [("publisher", "/out", "std_msgs/msg/String")]}})
        return child.system.endpoints

    first, second = observed([1, 2]), observed([2, 1])
    assert first == second
    assert first[0].qos is None
    assert "2 publishers on this topic with different qos" in first[0].qos_reason
    assert observed([1, 1])[0].qos.reliability == "reliable"
