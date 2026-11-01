"""The join phase: binding, remaps, supersession, parameters and idempotence."""

from __future__ import annotations

from typing import Any

import pytest

from rosio import collect, ids, join, model
from rosio.errors import ContractError
from rosio.resolve import records

PKG = "nx_bridge"
EXE = "io_bridge"
RELPATH = "nx_bridge/io_bridge.py"
SYM = ids.build_sym(PKG, RELPATH, "nx_bridge.io_bridge.IoBridge.__init__")
OWNER = ids.build_sym(PKG, RELPATH, "IoBridge.__init__")
"""The sym id the join forms from the `owner_symbol` a record row states. Every edge value
and every edge key spells a site this way, whatever the record spelled it."""
FILE = ids.build_file(PKG, RELPATH)
EXE_ID = ids.build_exe(PKG, EXE)
REAL = "profile:nx_navigation"
SIM = "profile:nx_sim"


def endpoint(kind: str, name: str | None, **extra: Any) -> dict[str, Any]:
    row: dict[str, Any] = {"kind": kind, "owner_symbol": "IoBridge.__init__", "line_hint": 12}
    if name is not None:
        row["name"] = name
    row.update(extra)
    return row


CODED_VALUES: dict[str, Any] = {
    "nodes": [{"name": EXE, "name_source": "literal", "form": "node", "symbol": SYM}],
    "endpoints": [
        endpoint("publisher", "cmd_vel", type="geometry_msgs/msg/Twist", qos=10),
        endpoint("subscriber", "/vehicle/state", type="nx_msgs/msg/RobotStatus"),
        endpoint("subscriber", None, raw="speed_limit_topic", reason="parameter with no default"),
        endpoint("subscriber", "", wrapper="tf2_ros.TransformListener"),
        endpoint("service_client", "/navigate_to_pose/_action/cancel_goal"),
        endpoint("timer", None, period_s=0.1),
    ],
    "parameters": [
        {"name": "publish_tf", "type": "bool", "default": True, "owner_symbol": "IoBridge"},
        {"name": "cmd_vel_topic", "type": "str", "default": "cmd_vel", "owner_symbol": "IoBridge"},
        {"name": "gain", "type": "double", "default_state": "runtime_attribute"},
    ],
}


def _profile(builder: model.AtlasBuilder, slug: str) -> str:
    return builder.add_profile(slug)


def _declared_node(
    builder: model.AtlasBuilder,
    node_id: str,
    profile: str,
    *,
    remaps: list[list[str]],
    namespace: str = "/",
    name: str = "",
    name_source: str = model.NameSource.UNKNOWN.value,
    known_name: str = "",
    disc: str = "",
    pkg: str = PKG,
    executable: str = EXE,
) -> None:
    slug = ids.parse_profile(profile)
    launchfile = ids.build_launchfile(PKG, "launch/bridge.launch.py")
    launchent = ids.build_launchent(
        PKG, "launch/bridge.launch.py", ((("Node"), f"{EXE}_{disc or slug}"),)
    )
    builder.node(launchfile)
    builder.node(launchent)
    builder.node(node_id, name_source=name_source)
    values: dict[str, Any] = {
        "pkg": pkg,
        "executable": executable,
        "namespace": namespace,
        "remaps": remaps,
    }
    if name:
        values["name"] = name
    if known_name:
        values["known_name"] = known_name
    builder.fact(node_id, join.DECLARED, profile=profile, values=values)
    builder.fact(
        launchent,
        join.DECLARED,
        profile=profile,
        values={"pkg": pkg, "executable": executable, "namespace": namespace, "remaps": remaps},
    )
    builder.edge(
        model.EdgeKind.LAUNCHES.value,
        launchent,
        node_id,
        key=ids.edge_key(launchent=launchent, profile=slug),
        layer=join.DECLARED,
        profile=profile,
    )


def _coded(builder: model.AtlasBuilder, values: dict[str, Any] | None = None) -> None:
    builder.node(ids.build_pkg(PKG))
    builder.node(FILE)
    builder.node(SYM)
    builder.node(EXE_ID)
    builder.edge(model.EdgeKind.DEFINES.value, FILE, SYM, layer=join.CODED)
    builder.edge(model.EdgeKind.RUNS.value, EXE_ID, SYM, layer=join.DECLARED)
    builder.fact(SYM, join.CODED, values=values if values is not None else CODED_VALUES)


def synthetic_id(role: str = "5f3a1c22") -> str:
    return ids.build_node_synthetic(PKG, EXE, "/", role)


@pytest.fixture
def builder() -> model.AtlasBuilder:
    out = model.AtlasBuilder()
    _profile(out, "nx_navigation")
    _profile(out, "nx_sim")
    _coded(out)
    _declared_node(
        out,
        synthetic_id(),
        REAL,
        remaps=[["cmd_vel", "cmd_vel_raw", "node"]],
    )
    return out


# ---------------------------------------------------------------------------
# reading what the collectors left


@pytest.mark.parametrize(
    ("pkg", "executable", "expected"),
    [
        (
            "robot_state_publisher",
            "robot_state_publisher",
            {
                ("publishes", "topic:/robot/description"),
                ("publishes", "topic:/robot/tf"),
                ("publishes", "topic:/tf_static"),
                ("subscribes", "topic:/robot/joint_states"),
            },
        ),
        (
            "joint_state_publisher",
            "joint_state_publisher",
            {
                ("publishes", "topic:/robot/joint_states"),
                ("subscribes", "topic:/robot/description"),
            },
        ),
    ],
)
def test_known_executable_endpoints_bind_without_source(pkg, executable, expected):
    from rosio import schema
    from rosio.drift import frames, topics

    builder = model.AtlasBuilder()
    _profile(builder, "nx_navigation")
    node = "node:/robot/driver"
    _declared_node(
        builder,
        node,
        REAL,
        pkg=pkg,
        executable=executable,
        namespace="/robot",
        name="driver",
        name_source="literal",
        remaps=[
            ["robot_description", "description"],
            ["robot_description", "wrong"],
            ["driver:/tf", "tf"],
        ],
    )
    site = next(edge.src for edge in builder.atlas.in_edges(node) if edge.kind == "launches")
    builder.unresolved(
        model.Unresolved(
            want="other",
            role="remap",
            owner=site,
            expr="robot_description:=description",
            reason="endpoint kind unknown",
        )
    )
    join.join(builder)
    atlas = builder.atlas
    ends = [edge for edge in atlas.out_edges(node) if edge.kind in ("publishes", "subscribes")]
    assert {(edge.kind, edge.dst) for edge in ends} == expected
    for edge in ends:
        fact = edge.fact(join.DECLARED, REAL)
        assert fact.values["via_known_wrapper"] is True
        assert fact.values["type"] in (
            "std_msgs/msg/String",
            "sensor_msgs/msg/JointState",
            "tf2_msgs/msg/TFMessage",
        )
        assert fact.values["qos"]
        assert any(
            source.file == "file:@repo|rosio/resolve/known_names.py" for source in fact.evidence
        )
    assert not frames.remap_unresolved(atlas)
    assert not frames.remap_matches_nothing(atlas)
    assert not topics.topic_no_publisher(atlas)
    assert not topics.topic_no_subscriber(atlas)
    assert schema.validate(atlas.to_json()) == []
    once = atlas.canonical_bytes()
    join.join(builder)
    assert builder.atlas.canonical_bytes() == once


def test_known_endpoints_do_not_supplement_read_source():
    builder = model.AtlasBuilder()
    _profile(builder, "nx_navigation")
    node = "node:/driver"
    _declared_node(
        builder,
        node,
        REAL,
        remaps=[],
        name="driver",
        name_source="literal",
        pkg="robot_state_publisher",
        executable="robot_state_publisher",
    )
    exe = "exe:robot_state_publisher|robot_state_publisher"
    source = "file:robot_state_publisher|src/main.cpp"
    builder.node(exe)
    builder.fact(exe, join.DECLARED, values={"sources": [source]})
    builder.fact(source, join.CODED, values={"language": "cpp"})
    join.join(builder)
    assert not [
        edge for edge in builder.atlas.out_edges(node) if edge.kind in ("publishes", "subscribes")
    ]


def test_a_coded_carrier_is_bound_to_its_executable(builder):
    atlas = builder.atlas
    found = join.carriers(atlas)
    assert [item.owner for item in found] == [SYM]
    assert found[0].file == FILE
    assert join.executables_of(atlas, SYM) == (EXE_ID,)
    assert join.executables_of(atlas, FILE) == (EXE_ID,)


def test_records_in_attrs_are_read_the_same_way():
    other = model.AtlasBuilder()
    other.node(FILE, attrs=CODED_VALUES)
    found = join.carriers(other.atlas)
    assert len(found) == 1
    assert len(found[0].endpoints) == 6


def test_one_executable_binds_to_every_instance_it_launched(builder):
    _declared_node(
        builder,
        ids.build_node("/sim/io_bridge"),
        SIM,
        namespace="/sim",
        name=EXE,
        name_source=model.NameSource.LITERAL.value,
        remaps=[["cmd_vel", "cmd_vel_sim", "node"]],
    )
    report = join.join(builder)
    assert report.instances == 2
    assert report.executables_bound == 1


# ---------------------------------------------------------------------------
# the remap engine


def test_a_relative_literal_plus_a_launch_remap_is_the_real_topic():
    bound = join.apply_remaps("cmd_vel", "/", "io_bridge", (("cmd_vel", "cmd_vel_raw"),))
    assert bound.name == "/cmd_vel_raw"
    assert bound.rule == ("cmd_vel", "cmd_vel_raw")


def test_a_relative_name_with_no_rule_takes_the_namespace():
    bound = join.apply_remaps("cmd_vel", "/robot1", "io_bridge", ())
    assert bound.name == "/robot1/cmd_vel"
    assert bound.rule is None


def test_the_first_matching_rule_wins():
    rules = (("cmd_vel", "one"), ("cmd_vel", "two"))
    assert join.apply_remaps("cmd_vel", "/", "n", rules).name == "/one"


def test_a_rule_matches_the_expanded_name_as_well():
    rules = (("/robot1/cmd_vel", "/safe"),)
    assert join.apply_remaps("cmd_vel", "/robot1", "n", rules).name == "/safe"


def test_a_private_name_resolves_under_the_node():
    assert join.apply_remaps("~/state", "/robot1", "bridge").name == "/robot1/bridge/state"


def test_a_node_prefixed_rule_only_applies_to_that_node():
    rules = (("bridge:cmd_vel", "/only_bridge"),)
    assert join.apply_remaps("cmd_vel", "/", "bridge", rules).name == "/only_bridge"
    assert join.apply_remaps("cmd_vel", "/", "other", rules).name == "/cmd_vel"


def test_argv_rules_are_not_name_remaps():
    rules = (("__node", "renamed"),)
    assert join.apply_remaps("cmd_vel", "/", "n", rules).name == "/cmd_vel"


def test_a_name_that_cannot_resolve_stays_unresolved():
    assert join.apply_remaps("", "/", "n") is None
    assert join.apply_remaps("~/state", "/", "") is None
    assert join.apply_remaps("cmd_vel", "/", "n", (("cmd_vel", "$(var out)"),)) is None


# ---------------------------------------------------------------------------
# binding


def test_the_join_produces_the_topic_neither_layer_states(builder):
    report = join.join(builder)
    atlas = builder.atlas
    assert "topic:/cmd_vel_raw" in atlas.entities
    assert report.endpoints_remapped == 1
    edges = [
        edge
        for edge in atlas.out_edges("node:/io_bridge")
        if edge.kind == model.EdgeKind.PUBLISHES.value
    ]
    assert [edge.dst for edge in edges] == ["topic:/cmd_vel_raw"]
    values = edges[0].fact(join.CODED, REAL).values
    assert values["raw_name"] == "cmd_vel"
    assert values["remapped"] is True
    assert values["remap"] == ["cmd_vel", "cmd_vel_raw"]
    assert values["type"] == "geometry_msgs/msg/Twist"


def test_an_absolute_literal_binds_without_a_remap(builder):
    join.join(builder)
    [edge] = [
        edge
        for edge in builder.atlas.out_edges("node:/io_bridge")
        if edge.kind == model.EdgeKind.SUBSCRIBES.value and edge.dst == "topic:/vehicle/state"
    ]
    assert edge.fact(join.CODED, REAL).values["remapped"] is False


def test_a_service_client_binds_to_a_service_and_not_to_a_topic(builder):
    join.join(builder)
    assert "service:/navigate_to_pose/_action/cancel_goal" in builder.atlas.entities
    assert "topic:/navigate_to_pose/_action/cancel_goal" not in builder.atlas.entities


def test_a_wrapper_endpoint_is_marked_and_never_reported_as_observed_only(builder):
    report = join.join(builder)
    assert report.wrapper_endpoints == 2
    for name in ("topic:/tf", "topic:/tf_static"):
        edge = next(
            edge
            for edge in builder.atlas.in_edges(name)
            if edge.kind == model.EdgeKind.SUBSCRIBES.value
        )
        assert edge.fact(join.CODED, REAL).values["via_known_wrapper"] is True


def test_an_endpoint_the_source_does_not_name_stays_unresolved(builder):
    report = join.join(builder)
    assert report.endpoints_unresolved == 1
    entries = [item for item in builder.atlas.unresolved if item.owner == "node:/io_bridge"]
    assert [item.expr for item in entries] == ["speed_limit_topic"]
    assert entries[0].want == model.Want.TOPIC_NAME.value
    assert "topic:speed_limit_topic" not in builder.atlas.entities


def test_a_timer_creates_no_endpoint(builder):
    join.join(builder)
    kinds = {ids.kind_of(key) for key in builder.atlas.entities}
    assert "topic" in kinds
    assert not [key for key in builder.atlas.entities if key.endswith("0.1")]


def test_the_same_literal_binds_differently_per_profile(builder):
    _declared_node(
        builder,
        ids.build_node("/sim/io_bridge"),
        SIM,
        namespace="/sim",
        name=EXE,
        name_source=model.NameSource.LITERAL.value,
        remaps=[["cmd_vel", "cmd_vel_sim", "node"]],
    )
    join.join(builder)
    assert "topic:/cmd_vel_raw" in builder.atlas.entities
    assert "topic:/sim/cmd_vel_sim" in builder.atlas.entities
    edge = next(
        edge
        for edge in builder.atlas.out_edges("node:/sim/io_bridge")
        if edge.kind == model.EdgeKind.PUBLISHES.value
    )
    assert edge.profiles == [SIM]


def test_the_message_type_is_attached_to_the_endpoint(builder):
    join.join(builder)
    edges = [
        edge
        for edge in builder.atlas.out_edges("topic:/cmd_vel_raw")
        if edge.kind == model.EdgeKind.USES_TYPE.value
    ]
    assert [edge.dst for edge in edges] == ["type:geometry_msgs|msg|Twist"]


# ---------------------------------------------------------------------------
# identity


def test_a_synthetic_identity_is_superseded_by_the_name_the_code_states(builder):
    report = join.join(builder)
    atlas = builder.atlas
    assert report.superseded == 1
    assert synthetic_id() not in atlas.entities
    node = atlas.entities["node:/io_bridge"]
    assert node.name_source == model.NameSource.LITERAL.value
    assert synthetic_id() in node.aka
    assert node.fact(join.DECLARED, REAL) is not None


def test_supersession_cascades_to_the_parameters_of_that_node(builder):
    old = ids.build_param(synthetic_id(), "publish_tf")
    builder.set_param(
        old,
        state=model.ParamState.EFFECTIVE.value,
        candidates=[
            model.Candidate(
                value=False,
                value_type="bool",
                mechanism=model.Mechanism.YAML_BASE.value,
                source=model.Source(file=FILE, locator="publish_tf"),
                rank=0,
                wins=True,
                profile=REAL,
            )
        ],
    )
    join.join(builder)
    new = ids.build_param("node:/io_bridge", "publish_tf")
    assert old not in builder.atlas.entities
    assert builder.atlas.entities[new].effective[REAL]["value"].value is False


def test_a_name_launch_stated_is_never_replaced_by_the_code(builder):
    other = model.AtlasBuilder()
    _profile(other, "nx_navigation")
    _coded(other)
    _declared_node(
        other,
        ids.build_node("/renamed_bridge"),
        REAL,
        name="renamed_bridge",
        name_source=model.NameSource.LITERAL.value,
        remaps=[],
    )
    report = join.join(other)
    assert report.superseded == 0
    assert "node:/renamed_bridge" in other.atlas.entities
    assert "node:/io_bridge" not in other.atlas.entities


def test_two_code_names_for_one_executable_never_merge(builder):
    values = dict(CODED_VALUES)
    values["nodes"] = [
        {"name": EXE, "name_source": "literal"},
        {"name": "bridge_twin", "name_source": "literal"},
    ]
    other = model.AtlasBuilder()
    _profile(other, "nx_navigation")
    _coded(other, values)
    _declared_node(other, synthetic_id(), REAL, remaps=[])
    report = join.join(other)
    assert report.superseded == 0
    assert report.ambiguous == 1
    findings = [item for item in other.atlas.findings if item.rule == "node.ambiguous_identity"]
    assert len(findings) == 1
    assert ":~" not in findings[0].say
    assert synthetic_id() in other.atlas.entities
    # turtlebot3_ros runs two nodes in one process, and a launch name renames both.
    assert "name the node in the launch file" not in findings[0].fix
    assert "would rename every node in the process" in findings[0].fix
    assert "in one process" in findings[0].say


def test_a_name_another_instance_owns_still_asks_for_a_launch_name():
    other = model.AtlasBuilder()
    _profile(other, "nx_navigation")
    _coded(other)
    _declared_node(
        other,
        ids.build_node(f"/{EXE}"),
        REAL,
        remaps=[],
        name=EXE,
        name_source=model.NameSource.LITERAL.value,
        disc="named",
    )
    _declared_node(other, synthetic_id(), REAL, remaps=[], disc="unnamed")
    join.join(other)
    (found,) = [item for item in other.atlas.findings if item.rule == "node.ambiguous_identity"]
    assert found.fix.startswith("name the node in the launch file")


def test_one_executable_launched_twice_in_one_profile_keeps_both_nodes():
    other = model.AtlasBuilder()
    _profile(other, "nx_navigation")
    _coded(other)
    left = ids.build_node_synthetic(PKG, EXE, "/", "aaaaaaaa")
    right = ids.build_node_synthetic(PKG, EXE, "/", "bbbbbbbb")
    _declared_node(other, left, REAL, remaps=[["cmd_vel", "cmd_vel_left"]], disc="left")
    _declared_node(other, right, REAL, remaps=[["cmd_vel", "cmd_vel_right"]], disc="right")
    report = join.join(other)
    assert report.superseded == 0
    assert report.ambiguous == 1
    assert left in other.atlas.entities
    assert right in other.atlas.entities
    assert "topic:/cmd_vel_left" in other.atlas.entities
    assert "topic:/cmd_vel_right" in other.atlas.entities
    findings = [item for item in other.atlas.findings if item.rule == "node.ambiguous_identity"]
    assert len(findings) == 1


def test_a_launch_site_off_in_the_profile_is_not_a_second_instance():
    """image_pipeline loads one list into a new container or into an existing one, and the
    condition picks exactly one, so the site that never runs states no second node."""
    other = model.AtlasBuilder()
    _profile(other, "nx_navigation")
    _coded(other)
    live = ids.build_node_synthetic(PKG, EXE, "/", "aaaaaaaa")
    dead = ids.build_node_synthetic(PKG, EXE, "/", "bbbbbbbb")
    _declared_node(other, live, REAL, remaps=[], disc="new_container")
    _declared_node(other, dead, REAL, remaps=[], disc="load_into")
    site = ids.build_launchent(PKG, "launch/bridge.launch.py", (("Node", f"{EXE}_load_into"),))
    off = {"mechanism": "IfCondition", "expr": "container != ''", "outcome": "false"}
    other.fact(site, join.DECLARED, profile=REAL, values={"gates": [off]})
    report = join.join(other)
    assert report.ambiguous == 0
    assert report.superseded == 1
    assert ids.build_node(f"/{EXE}") in other.atlas.entities
    assert dead in other.atlas.entities


def test_two_conditional_endpoints_on_one_name_are_two_sites(builder):
    values = dict(CODED_VALUES)
    values["endpoints"] = [
        endpoint("subscriber", "orient", type="nav_msgs/msg/Odometry", condition="odometry"),
        endpoint("subscriber", "orient", type="sensor_msgs/msg/Imu", condition="imu"),
    ]
    other = model.AtlasBuilder()
    _profile(other, "nx_navigation")
    _coded(other, values)
    _declared_node(
        other,
        ids.build_node("/io_bridge"),
        REAL,
        remaps=[],
        name=EXE,
        name_source=model.NameSource.LITERAL.value,
    )
    report = join.join(other)
    edges = [
        edge
        for edge in other.atlas.out_edges("node:/io_bridge")
        if edge.kind == model.EdgeKind.SUBSCRIBES.value
    ]
    assert report.endpoints_bound == 2
    assert len(edges) == 2
    assert sorted(edge.fact(join.CODED, REAL).values["type"] for edge in edges) == [
        "nav_msgs/msg/Odometry",
        "sensor_msgs/msg/Imu",
    ]


def test_the_wrapper_key_the_collectors_emit_is_the_one_that_is_read(builder):
    values = dict(CODED_VALUES)
    values["endpoints"] = [
        endpoint(
            "subscriber",
            "/tf",
            via_known_wrapper="tf2_ros.TransformListener",
            type="tf2_msgs/msg/TFMessage",
        )
    ]
    other = model.AtlasBuilder()
    _profile(other, "nx_navigation")
    _coded(other, values)
    _declared_node(
        other,
        ids.build_node("/io_bridge"),
        REAL,
        remaps=[],
        name=EXE,
        name_source=model.NameSource.LITERAL.value,
    )
    report = join.join(other)
    edge = next(
        edge
        for edge in other.atlas.in_edges("topic:/tf")
        if edge.kind == model.EdgeKind.SUBSCRIBES.value
    )
    assert edge.fact(join.CODED, REAL).values["via_known_wrapper"] is True
    assert report.wrapper_endpoints == 1
    assert report.endpoints_bound == 1


def test_the_record_spellings_cpp_code_uses_are_read(builder):
    other = model.AtlasBuilder()
    _profile(other, "nx_navigation")
    _coded(
        other,
        {
            "nodes": [{"name": EXE, "name_source": "literal"}],
            "endpoints": [endpoint("publisher", "cmd_vel", type="geometry_msgs/msg/Twist")],
            "parameters": [{"name": "gain", "type": "double", "default": 1.0, "reads": True}],
        },
    )
    _declared_node(other, synthetic_id(), REAL, remaps=[])
    report = join.join(other)
    assert report.superseded == 1
    param = other.atlas.entities[ids.build_param("node:/io_bridge", "gain")]
    assert param.fact(join.CODED, REAL).values["read"] is True


def test_a_name_table_row_the_code_contradicts_is_reported(builder):
    other = model.AtlasBuilder()
    _profile(other, "nx_navigation")
    _coded(other)
    _declared_node(
        other,
        ids.build_node("/io_bridge_table"),
        REAL,
        name="io_bridge_table",
        name_source=model.NameSource.EXECUTABLE_DEFAULT.value,
        known_name="io_bridge_table",
        remaps=[],
    )
    report = join.join(other)
    assert report.contradictions == [(PKG, EXE, "io_bridge_table", EXE)]
    assert report.superseded == 1
    assert "node:/io_bridge" in other.atlas.entities


# ---------------------------------------------------------------------------
# parameters


def test_a_parameter_no_file_sets_is_visible_with_its_code_default(builder):
    join.join(builder)
    param = builder.atlas.entities[ids.build_param("node:/io_bridge", "publish_tf")]
    assert param.state == model.ParamState.EFFECTIVE.value
    assert param.effective[REAL]["value"].value is True
    assert param.candidates[0].mechanism == model.Mechanism.CODE_DEFAULT.value
    assert param.layers == [join.CODED]


def test_a_parameter_yaml_sets_keeps_its_chain_and_gains_the_coded_fact(builder):
    param_id = ids.build_param(synthetic_id(), "cmd_vel_topic")
    builder.set_param(
        param_id,
        state=model.ParamState.EFFECTIVE.value,
        candidates=[
            model.Candidate(
                value="cmd_vel_raw",
                value_type="str",
                mechanism=model.Mechanism.YAML_BASE.value,
                source=model.Source(file=FILE, locator="cmd_vel_topic"),
                rank=0,
                wins=True,
                profile=REAL,
            )
        ],
    )
    join.join(builder)
    param = builder.atlas.entities[ids.build_param("node:/io_bridge", "cmd_vel_topic")]
    assert len(param.candidates) == 1
    assert param.effective[REAL]["value"].value == "cmd_vel_raw"
    assert param.fact(join.CODED, REAL).values["declared_in_code"] is True
    assert param.layers == [join.CODED]


def test_a_default_the_code_cannot_state_is_not_invented(builder):
    join.join(builder)
    param = builder.atlas.entities[ids.build_param("node:/io_bridge", "gain")]
    assert param.state == model.ParamState.OTHER.value
    assert param.effective == {}
    assert [item.wins for item in param.candidates] == [False]
    assert param.candidates[0].value is None
    assert param.candidates[0].value_type == "unknown"


def test_one_node_in_two_profiles_gets_the_code_default_in_both(builder):
    node_id = ids.build_node("/io_bridge")
    builder.node(node_id, name_source=model.NameSource.LITERAL.value)
    _declared_node(builder, node_id, SIM, remaps=[], name=EXE, disc="sim")
    join.join(builder)
    param = builder.atlas.entities[ids.build_param(node_id, "publish_tf")]
    assert sorted(item.profile for item in param.candidates) == sorted([SIM, REAL])
    assert sorted(param.effective) == sorted([SIM, REAL])
    assert param.effective[SIM]["value"].value is True
    assert param.effective[REAL]["value"].value is True


def test_a_chain_the_yaml_wrote_for_one_profile_is_not_overwritten(builder):
    node_id = ids.build_node("/io_bridge")
    builder.node(node_id, name_source=model.NameSource.LITERAL.value)
    _declared_node(builder, node_id, SIM, remaps=[], name=EXE, disc="sim")
    param_id = ids.build_param(node_id, "publish_tf")
    builder.set_param(
        param_id,
        state=model.ParamState.EFFECTIVE.value,
        candidates=[
            model.Candidate(
                value=False,
                value_type="bool",
                mechanism=model.Mechanism.YAML_BASE.value,
                source=model.Source(file=FILE, locator="publish_tf"),
                rank=0,
                wins=True,
                profile=REAL,
            )
        ],
    )
    join.join(builder)
    param = builder.atlas.entities[param_id]
    assert param.effective[REAL]["value"].value is False
    assert param.effective[SIM]["value"].value is True
    assert [item.mechanism for item in param.candidates if item.profile == REAL] == [
        model.Mechanism.YAML_BASE.value
    ]


def test_the_owner_node_says_its_code_was_read(builder):
    join.join(builder)
    node = builder.atlas.entities["node:/io_bridge"]
    assert node.attrs["coded_params_known"] is True
    assert node.fact(join.CODED, REAL).values["exe"] == EXE_ID
    assert node.fact(join.CODED, REAL).values["code_name"] == EXE


def test_the_atlas_stays_valid_after_the_join(builder):
    join.join(builder)
    builder.build()


# ---------------------------------------------------------------------------
# idempotence, the property the whole phase depends on


def test_running_the_join_twice_changes_nothing(builder):
    join.join(builder)
    first = builder.atlas.canonical_bytes()
    join.join(builder)
    assert builder.atlas.canonical_bytes() == first


def test_running_the_join_twice_changes_nothing_with_two_profiles(builder):
    _declared_node(
        builder,
        ids.build_node("/sim/io_bridge"),
        SIM,
        namespace="/sim",
        name=EXE,
        name_source=model.NameSource.LITERAL.value,
        remaps=[["cmd_vel", "cmd_vel_sim", "node"]],
    )
    report = join.join(builder)
    first = builder.atlas.canonical_bytes()
    again = join.join(builder)
    assert builder.atlas.canonical_bytes() == first
    assert again.endpoints_bound == report.endpoints_bound


def test_the_second_pass_does_not_double_count_the_resolution_rate(builder):
    join.join(builder)
    rates = dict(builder.atlas.counters["resolved"])
    join.join(builder)
    assert builder.atlas.counters["resolved"] == rates


def test_an_ambiguous_finding_is_emitted_once(builder):
    values = dict(CODED_VALUES)
    values["nodes"] = [
        {"name": EXE, "name_source": "literal"},
        {"name": "bridge_twin", "name_source": "literal"},
    ]
    other = model.AtlasBuilder()
    _profile(other, "nx_navigation")
    _coded(other, values)
    _declared_node(other, synthetic_id(), REAL, remaps=[])
    join.join(other)
    join.join(other)
    assert len(other.atlas.findings) == 1


def test_a_carrier_no_executable_claims_is_left_alone():
    other = model.AtlasBuilder()
    other.node(SYM)
    other.fact(SYM, join.CODED, values=CODED_VALUES)
    report = join.join(other)
    assert report.unbound_carriers == 1
    assert report.endpoints_bound == 0


def test_an_empty_atlas_joins_to_nothing():
    other = model.AtlasBuilder()
    report = join.join(other)
    assert report.endpoints_bound == 0
    assert other.atlas.entities == {}


def test_a_bad_parameter_name_is_refused_and_not_guessed(builder):
    with pytest.raises(ContractError):
        ids.build_param("node:/io_bridge", "")


# ---------------------------------------------------------------------------
# frames


BROADCASTS = [
    {
        "kind": "dynamic",
        "parent": "odom",
        "child": "base_footprint",
        "broadcaster": "tf2_ros::TransformBroadcaster",
        "owner_symbol": "IoBridge.__init__",
        "line_hint": 40,
    },
    {
        "kind": "static",
        "parent": "base_link",
        "child": None,
        "child_state": "unresolved",
        "owner_symbol": "IoBridge.__init__",
    },
    {
        "kind": "dynamic",
        "parent": "map",
        "child": "odom",
        "child_from_parameter": "odom_frame",
        "owner_symbol": "IoBridge.__init__",
    },
]

LOOKUPS = [
    {
        "api": "lookup_transform",
        "target": "base_link",
        "source": "map",
        "owner_symbol": "IoBridge.__init__",
        "line_hint": 61,
    },
    {
        "api": "can_transform",
        "target": None,
        "source": "map",
        "target_state": "unresolved",
        "owner_symbol": "IoBridge.__init__",
    },
]


def _tf_values(**extra: Any) -> dict[str, Any]:
    values = dict(CODED_VALUES)
    values["tf_broadcasts"] = BROADCASTS
    values["tf_lookups"] = LOOKUPS
    values.update(extra)
    return values


def _tf_builder(namespace: str = "/", node_id: str = "node:/io_bridge") -> model.AtlasBuilder:
    other = model.AtlasBuilder()
    _profile(other, "nx_navigation")
    _coded(other, _tf_values())
    _declared_node(
        other,
        node_id,
        REAL,
        remaps=[],
        namespace=namespace,
        name=EXE,
        name_source=model.NameSource.LITERAL.value,
    )
    return other


def _tf_edges(atlas: model.Atlas, frame: str) -> list[model.Edge]:
    """The broadcasts into a frame and the tf_parent edges out of it, which is where a child
    states its parent."""
    return atlas.in_edges(frame) + atlas.out_edges(frame)


def _tf_edge(atlas: model.Atlas, kind: str, frame: str):
    return next(edge for edge in _tf_edges(atlas, frame) if edge.kind == kind)


def test_a_broadcast_becomes_a_node_to_frame_edge():
    other = _tf_builder()
    report = join.join(other)
    edge = _tf_edge(other.atlas, model.EdgeKind.BROADCASTS_TF.value, "frame:base_footprint")
    values = edge.fact(join.CODED, REAL).values
    assert edge.src == "node:/io_bridge"
    assert values["parent"] == "odom"
    assert values["static"] is False
    assert values["broadcaster"] == "tf2_ros::TransformBroadcaster"
    assert report.tf_edges > 0


def test_a_broadcast_also_states_the_parent_of_the_child():
    other = _tf_builder()
    join.join(other)
    edge = _tf_edge(other.atlas, model.EdgeKind.TF_PARENT.value, "frame:base_footprint")
    assert (edge.src, edge.dst) == ("frame:base_footprint", "frame:odom")
    assert edge.fact(join.CODED, REAL).values["source"] == "coded_broadcast"
    assert edge.fact(join.CODED, REAL).values["node"] == "node:/io_bridge"
    assert edge.profiles == [REAL]


def test_a_child_the_source_never_states_creates_no_frame():
    other = _tf_builder()
    report = join.join(other)
    assert "frame:base_link" in other.atlas.entities
    assert not [
        edge
        for edge in other.atlas.out_edges("node:/io_bridge")
        if edge.kind == model.EdgeKind.BROADCASTS_TF.value
        and edge.fact(join.CODED, REAL).values["parent"] == "base_link"
    ]
    assert report.tf_unresolved == 2


def test_a_lookup_becomes_a_listens_edge_on_both_ends():
    other = _tf_builder()
    join.join(other)
    edges = [
        edge
        for edge in other.atlas.out_edges("node:/io_bridge")
        if edge.kind == model.EdgeKind.LISTENS_TF.value
    ]
    # Two lookups naming the same frame are two sites, so the key keeps them apart.
    assert sorted(edge.dst for edge in edges) == ["frame:base_link", "frame:map", "frame:map"]
    values = (
        _tf_edge(other.atlas, model.EdgeKind.LISTENS_TF.value, "frame:base_link")
        .fact(join.CODED, REAL)
        .values
    )
    assert values["api"] == "lookup_transform"
    assert values["role"] == "target"


def test_two_tf_sites_that_differ_only_in_their_content_are_two_edges():
    other = model.AtlasBuilder()
    _profile(other, "nx_navigation")
    _coded(
        other,
        _tf_values(
            tf_broadcasts=[
                {
                    "kind": "dynamic",
                    "parent": "odom",
                    "child": "base_footprint",
                    "condition": "publish_odom_tf",
                    "owner_symbol": "IoBridge.__init__",
                },
                {
                    "kind": "dynamic",
                    "parent": "odom",
                    "child": "base_footprint",
                    "condition": "publish_wheel_tf",
                    "owner_symbol": "IoBridge.__init__",
                },
            ],
            tf_lookups=[
                {"api": "lookup_transform", "target": "map", "source": "odom"},
                {"api": "lookup_transform", "target": "map", "source": "base_link"},
            ],
        ),
    )
    _declared_node(
        other,
        ids.build_node("/io_bridge"),
        REAL,
        remaps=[],
        name=EXE,
        name_source=model.NameSource.LITERAL.value,
    )
    join.join(other)
    broadcasts = [
        edge
        for edge in other.atlas.out_edges("node:/io_bridge")
        if edge.kind == model.EdgeKind.BROADCASTS_TF.value
    ]
    listens = [
        edge
        for edge in other.atlas.out_edges("node:/io_bridge")
        if edge.kind == model.EdgeKind.LISTENS_TF.value and edge.dst == "frame:map"
    ]
    assert len(broadcasts) == 2
    assert len(listens) == 2


def test_a_frame_that_comes_from_a_parameter_takes_the_effective_value():
    other = _tf_builder()
    param_id = ids.build_param("node:/io_bridge", "odom_frame")
    other.set_param(
        param_id,
        state=model.ParamState.EFFECTIVE.value,
        candidates=[
            model.Candidate(
                value="odom_filtered",
                value_type="str",
                mechanism=model.Mechanism.YAML_BASE.value,
                source=model.Source(file=FILE, locator="odom_frame"),
                rank=0,
                wins=True,
                profile=REAL,
            )
        ],
    )
    report = join.join(other)
    edge = _tf_edge(other.atlas, model.EdgeKind.BROADCASTS_TF.value, "frame:odom_filtered")
    values = edge.fact(join.CODED, REAL).values
    assert values["child"] == "odom_filtered"
    assert values["child_from_parameter"] == "odom_frame"
    assert report.tf_from_parameter == 1


def test_a_parameter_with_no_winner_leaves_the_literal_the_source_states():
    other = _tf_builder()
    join.join(other)
    assert "frame:odom" in other.atlas.entities
    edge = _tf_edge(other.atlas, model.EdgeKind.BROADCASTS_TF.value, "frame:odom")
    assert edge.fact(join.CODED, REAL).values["child"] == "odom"


def test_a_frame_is_global_and_never_prefixed_by_the_launch_namespace():
    other = _tf_builder(namespace="/robot1", node_id=ids.build_node("/robot1/io_bridge"))
    join.join(other)
    assert "frame:base_footprint" in other.atlas.entities
    assert "frame:robot1/base_footprint" not in other.atlas.entities
    edge = _tf_edge(other.atlas, model.EdgeKind.BROADCASTS_TF.value, "frame:base_footprint")
    assert edge.fact(join.CODED, REAL).values["namespace"] == "/robot1"


def test_a_node_relative_frame_name_takes_the_namespace():
    assert join.frame_id("~/imu", "/robot1") == "frame:robot1/imu"
    assert join.frame_id("/odom", "/robot1") == "frame:odom"
    assert join.frame_id("", "/robot1") == ""


# ---------------------------------------------------------------------------
# the record block, which is the contract the coded collectors write against


def test_the_record_block_has_one_spelling_per_record_kind():
    """AGENTS section 2.2. One key per record kind, and a key outside the block is not a
    record. `nodes`, `broadcasts` and `lookups` used to be spelled two ways each and an alias
    table absorbed the difference, which is how a load bearing contract stops being one."""
    assert join.RECORD_KEYS == (
        "classes",
        "class_templates",
        "constructions",
        "type_aliases",
        "endpoints",
        "nodes",
        "parameters",
        "parameter_reads",
        "parameter_writes",
        "plugin_parameters",
        "tf_broadcasts",
        "tf_lookups",
        "plugin_loaders",
    )
    assert len(set(join.RECORD_KEYS)) == len(join.RECORD_KEYS)
    assert not hasattr(join, "LEGACY_NODE_KEY")
    assert not hasattr(join, "RECORD_ALIASES")
    assert not hasattr(join, "TF_BLOCK")


def test_a_key_the_block_does_not_name_is_not_a_record():
    """A nested `tf` block was read as the two tf lists, and no collector has ever written
    one into a fact. A shape nothing writes is a shape nothing keeps true."""
    other = model.AtlasBuilder()
    other.node(FILE, attrs={"tf": {"broadcasts": BROADCASTS, "lookups": LOOKUPS}})

    assert join.carriers(other.atlas) == []


def test_the_tf_rows_are_read_under_the_names_the_block_states():
    other = model.AtlasBuilder()
    other.node(FILE, attrs={"tf_broadcasts": BROADCASTS, "tf_lookups": LOOKUPS})

    found = join.carriers(other.atlas)

    assert len(found[0].tf_broadcasts) == 3
    assert len(found[0].tf_lookups) == 2


def test_a_plugin_parameter_is_its_own_record_kind_and_not_a_parameter():
    """The two lists were merged into one bucket, which said a suffix only declaration and a
    node parameter are the same record. They are read at one call site, because the row
    decides which it is, and they arrive as two kinds."""
    other = model.AtlasBuilder()
    other.node(
        FILE,
        attrs={
            "parameters": [{"name": "rate", "owner_symbol": "IoBridge.__init__"}],
            "plugin_parameters": [
                {
                    "base_class": "nav2_core::Controller",
                    "suffix": "max_speed",
                    "owner_symbol": "IoBridge.__init__",
                }
            ],
        },
    )

    carrier = join.carriers(other.atlas)[0]

    assert [row["name"] for row in carrier.parameters] == ["rate"]
    assert [row["suffix"] for row in carrier.plugin_parameters] == ["max_speed"]
    assert len(carrier.declarations()) == 2


def test_a_plugin_parameter_reaches_the_plugin_it_declares(builder):
    """End to end for the key that moved: the row arrives under `plugin_parameters`, the
    bound instance walks it with the node parameters, and the suffix only declaration lands
    on the plugin as a `pparam` with a `declares_param` edge. The alias put it in the same
    list as a node parameter, and the row is what tells them apart, so the merge could only
    ever be undone by reading the row again."""
    base = "nav2_core::Controller"
    lookup = "dwb_core::DWBLocalPlanner"
    builder.node(ids.build_plugin(base, lookup))
    builder.fact(
        SYM,
        join.CODED,
        values={
            "plugin_parameters": [
                {
                    "base_class": base,
                    "lookup_name": lookup,
                    "suffix": "BaseObstacle.scale",
                    "default": 0.02,
                    "has_default": True,
                    "owner_symbol": "IoBridge.__init__",
                }
            ]
        },
    )

    join.join(builder)

    pparam = ids.build_pparam(base, lookup, "BaseObstacle.scale")
    assert pparam in builder.atlas.entities
    assert builder.atlas.entities[pparam].fact(join.CODED, REAL).values["default"] == 0.02
    [edge] = [
        edge
        for edge in builder.atlas.in_edges(pparam)
        if edge.kind == model.EdgeKind.DECLARES_PARAM.value
        and edge.src == ids.build_plugin(base, lookup)
    ]
    assert edge.fact(join.CODED, REAL).values["suffix"] == "BaseObstacle.scale"


def test_a_symbols_node_rows_reach_the_join_under_the_one_spelling():
    """A composable node reaches its code through its symbol and not through a file, so the
    rows on the symbol are the only statement of its name. `cpp_code` and `py_code` spelled
    them `node` there and `nodes` on the file fact of the same pass, and the join carried a
    fallback for the difference. Both write `nodes` now and the fallback is gone."""
    rows = [{"name": "io_bridge", "name_source": "literal", "symbol": SYM}]
    other = model.AtlasBuilder()
    other.node(FILE)
    other.node(SYM)
    other.fact(SYM, join.CODED, values={"nodes": rows})

    carrier = next(item for item in join.carriers(other.atlas) if item.owner == SYM)

    assert [row["name"] for row in carrier.nodes] == ["io_bridge"]


def test_the_retired_node_spelling_reaches_nothing():
    """The fallback is gone, so a fact that still spells it `node` carries no record at all,
    which is what a key outside the block means."""
    rows = [{"name": "io_bridge", "name_source": "literal", "symbol": SYM}]
    other = model.AtlasBuilder()
    other.node(FILE)
    other.node(SYM)
    other.fact(SYM, join.CODED, values={"node": rows})

    assert join.carriers(other.atlas) == []


# ---------------------------------------------------------------------------
# composable nodes, which have no executable at all


COMPONENT_CLASS = "nx_bridge::BridgeComponent"
COMPONENT_PKG = "nx_bridge_cpp"
COMPONENT_RELPATH = "src/bridge_component.cpp"
COMPONENT_FILE = ids.build_file(COMPONENT_PKG, COMPONENT_RELPATH)
COMPONENT_SYM = ids.build_sym(COMPONENT_PKG, COMPONENT_RELPATH, f"{COMPONENT_CLASS}::configure")


def _component_builder(*, registration: bool = True, plugin: bool = False) -> model.AtlasBuilder:
    other = model.AtlasBuilder()
    _profile(other, "nx_navigation")
    other.node(COMPONENT_FILE)
    other.node(COMPONENT_SYM)
    other.edge(model.EdgeKind.DEFINES.value, COMPONENT_FILE, COMPONENT_SYM, layer=join.CODED)
    values = {
        "nodes": [{"name": "bridge_component", "name_source": "literal"}],
        "endpoints": [
            {
                "kind": "publisher",
                "name": "state",
                "type": "nx_msgs/msg/RobotStatus",
                "owner_symbol": f"{COMPONENT_CLASS}::configure",
            }
        ],
    }
    other.fact(COMPONENT_SYM, join.CODED, values=values)
    if registration:
        other.fact(
            COMPONENT_FILE,
            join.CODED,
            values={
                "registrations": [
                    {
                        "macro": "RCLCPP_COMPONENTS_REGISTER_NODE",
                        "type": COMPONENT_CLASS,
                        "base_class": "",
                    }
                ]
            },
        )
    if plugin:
        plugin_id = ids.build_plugin("rclcpp_components::NodeFactory", COMPONENT_CLASS)
        other.node(plugin_id)
        other.edge(model.EdgeKind.DEFINES.value, COMPONENT_FILE, plugin_id, layer=join.DECLARED)
    node_id = ids.build_node("/bridge_component")
    other.node(node_id, name_source=model.NameSource.LITERAL.value)
    other.fact(
        node_id,
        join.DECLARED,
        profile=REAL,
        values={
            "pkg": COMPONENT_PKG,
            "form": "composable_node",
            "plugin_class": COMPONENT_CLASS,
            "namespace": "/",
            "name": "bridge_component",
            "container": "/bridge_container",
            "remaps": [["state", "vehicle/state", "node"]],
        },
    )
    return other


def test_a_composable_node_reaches_its_code_through_the_registered_class():
    other = _component_builder()
    report = join.join(other)
    assert report.components_joined == 1
    assert report.executables_bound == 0
    edge = next(
        edge
        for edge in other.atlas.out_edges("node:/bridge_component")
        if edge.kind == model.EdgeKind.PUBLISHES.value
    )
    assert edge.dst == "topic:/vehicle/state"
    node = other.atlas.entities["node:/bridge_component"]
    assert node.fact(join.CODED, REAL).values["bound_through"] == "registered_class"
    assert node.fact(join.CODED, REAL).values["plugin_class"] == COMPONENT_CLASS


def test_the_plugin_lookup_name_is_followed_before_the_registration():
    other = _component_builder(plugin=True)
    join.join(other)
    node = other.atlas.entities["node:/bridge_component"]
    assert node.fact(join.CODED, REAL).values["bound_through"] == "plugin_lookup"


def test_a_component_with_no_registration_still_reaches_its_symbol():
    other = _component_builder(registration=False)
    report = join.join(other)
    assert report.components_joined == 1
    node = other.atlas.entities["node:/bridge_component"]
    assert node.fact(join.CODED, REAL).values["bound_through"] == "symbol"


def test_a_composable_node_no_source_states_binds_to_nothing():
    other = _component_builder(registration=False)
    other.atlas.entities["node:/bridge_component"].facts[0].values["plugin_class"] = "other::Class"
    report = join.join(other)
    assert report.components_joined == 0
    assert report.endpoints_bound == 0


def test_the_component_join_is_idempotent():
    other = _component_builder()
    join.join(other)
    first = other.atlas.canonical_bytes()
    join.join(other)
    assert other.atlas.canonical_bytes() == first


# ---------------------------------------------------------------------------
# one source, two executables


PERCEPTION = "nx_perception"
COMPONENT_EXE = ids.build_exe(PERCEPTION, "nx_detector_node_component")
PLAIN_EXE = ids.build_exe(PERCEPTION, "nx_detector_node")
NODE_SOURCE = ids.build_file(PERCEPTION, "src/detector_node.cpp")
MAIN_SOURCE = ids.build_file(PERCEPTION, "src/detector_main.cpp")
NODE_SYM = ids.build_sym(PERCEPTION, "src/detector_node.cpp", "nx_perception::DetectorNode")


def _two_executables() -> model.AtlasBuilder:
    other = model.AtlasBuilder()
    _profile(other, "nx_navigation")
    for entity_id in (NODE_SOURCE, MAIN_SOURCE, NODE_SYM, COMPONENT_EXE, PLAIN_EXE):
        other.node(entity_id)
    other.edge(model.EdgeKind.DEFINES.value, NODE_SOURCE, NODE_SYM, layer=join.CODED)
    other.edge(model.EdgeKind.RUNS.value, COMPONENT_EXE, NODE_SYM, layer=join.DECLARED)
    other.fact(
        COMPONENT_EXE,
        join.DECLARED,
        profile=REAL,
        values={
            "mechanism": "rclcpp_components_register_node",
            "sources": [NODE_SOURCE],
            "target": "detector_component",
        },
    )
    other.fact(
        PLAIN_EXE,
        join.DECLARED,
        profile=REAL,
        values={
            "mechanism": "add_executable",
            "sources": [MAIN_SOURCE],
            "target": "nx_detector_node",
            "linked_targets": ["detector_component"],
        },
    )
    other.fact(
        NODE_SYM,
        join.CODED,
        values={
            "nodes": [{"name": "detector_node", "name_source": "literal"}],
            "endpoints": [
                {
                    "kind": "subscriber",
                    "name": "points",
                    "type": "sensor_msgs/msg/PointCloud2",
                    "owner_symbol": "nx_perception::DetectorNode",
                }
            ],
        },
    )
    return other


def test_a_source_two_executables_are_built_from_reaches_both():
    other = _two_executables()
    assert join.executables_of(other.atlas, NODE_SOURCE) == (PLAIN_EXE, COMPONENT_EXE)


def test_the_launched_executable_is_the_one_that_gets_the_endpoints():
    other = _two_executables()
    node_id = ids.build_node("/detector_node")
    other.node(node_id, name_source=model.NameSource.LITERAL.value)
    other.fact(
        node_id,
        join.DECLARED,
        profile=REAL,
        values={
            "pkg": PERCEPTION,
            "executable": "nx_detector_node",
            "namespace": "/",
            "name": "detector_node",
            "remaps": [["points", "/ouster/points", "node"]],
        },
    )
    report = join.join(other)
    assert report.endpoints_bound == 1
    edge = next(
        edge
        for edge in other.atlas.out_edges(node_id)
        if edge.kind == model.EdgeKind.SUBSCRIBES.value
    )
    assert edge.dst == "topic:/ouster/points"
    assert report.shared_sources == [(NODE_SOURCE, (PLAIN_EXE, COMPONENT_EXE))]
    assert other.atlas.entities[NODE_SOURCE].attrs["executables"] == [PLAIN_EXE, COMPONENT_EXE]


def test_one_executable_of_the_two_does_not_take_the_other_ones_instance():
    other = _two_executables()
    node_id = ids.build_node("/detector_node")
    other.node(node_id, name_source=model.NameSource.LITERAL.value)
    other.fact(
        node_id,
        join.DECLARED,
        profile=REAL,
        values={
            "pkg": PERCEPTION,
            "executable": "nx_detector_node",
            "namespace": "/",
            "name": "detector_node",
            "remaps": [],
        },
    )
    join.join(other)
    assert not [
        edge
        for edge in other.atlas.out_edges(node_id)
        if edge.kind == model.EdgeKind.SUBSCRIBES.value and edge.dst != "topic:/points"
    ]


def test_a_source_one_executable_declares_records_nothing_shared():
    other = _two_executables()
    other.atlas.entities[PLAIN_EXE].facts[0].values["linked_targets"] = []
    report = join.join(other)
    assert report.shared_sources == []
    assert join.executables_of(other.atlas, NODE_SOURCE) == (COMPONENT_EXE,)


# ---------------------------------------------------------------------------
# the whole pass, twice


def test_the_full_join_is_idempotent_over_endpoints_frames_and_components():
    other = _tf_builder()
    join.join(other)
    first = other.atlas.canonical_bytes()
    again = join.join(other)
    assert other.atlas.canonical_bytes() == first
    assert again.tf_edges > 0
    other.build()


def test_the_second_pass_creates_no_second_frame_edge():
    other = _tf_builder()
    join.join(other)
    before = len(other.atlas.edges)
    join.join(other)
    assert len(other.atlas.edges) == before


# ---------------------------------------------------------------------------
# one fact, one producer, one shape


def _python_file(builder: model.AtlasBuilder) -> None:
    builder.node(FILE, attrs={"language": "python"})


def test_the_tf_parent_edge_names_the_language_of_the_broadcast():
    """It names the statement the geometry came from and not the pass that wrote the edge,
    and the edge runs from the child to the parent like a urdf joint."""
    other = _tf_builder()
    _python_file(other)
    join.join(other)
    edge = _tf_edge(other.atlas, model.EdgeKind.TF_PARENT.value, "frame:base_footprint")
    values = edge.fact(join.CODED, REAL).values
    assert (edge.src, edge.dst) == ("frame:base_footprint", "frame:odom")
    assert values["source"] == "py_broadcast"
    assert values["symbol"] == OWNER
    assert edge.profiles == [REAL]


def test_one_frame_pair_carries_one_coded_edge_per_profile():
    other = _tf_builder()
    _python_file(other)
    _declared_node(
        other,
        ids.build_node("/sim/io_bridge"),
        SIM,
        remaps=[],
        namespace="/sim",
        name=EXE,
        name_source=model.NameSource.LITERAL.value,
    )
    join.join(other)
    edges = [
        edge
        for edge in other.atlas.out_edges("frame:base_footprint")
        if edge.kind == model.EdgeKind.TF_PARENT.value
    ]
    assert len(edges) == 2
    assert sorted(edge.profiles[0] for edge in edges) == sorted([SIM, REAL])


def test_every_tf_edge_spells_its_site_as_a_symbol_id():
    other = _tf_builder()
    join.join(other)
    for edge in other.atlas.edges.values():
        for fact in edge.facts:
            symbol = fact.values.get("symbol")
            if symbol:
                assert ids.kind_of(str(symbol)) == "sym"


# ---------------------------------------------------------------------------
# parameter reads


READS = [
    {
        "name": "publish_rate_hz",
        "api": "get_parameter",
        "owner_symbol": "IoBridge.__init__",
        "line_hint": 100,
    },
    {
        "name": "publish_tf",
        "api": "get_parameter",
        "owner_symbol": "IoBridge.__init__",
        "line_hint": 101,
    },
]


def _with_reads() -> model.AtlasBuilder:
    other = model.AtlasBuilder()
    _profile(other, "nx_navigation")
    _coded(other, {**CODED_VALUES, "parameter_reads": READS})
    # The collector emits one sym entity per owner_symbol a record names.
    other.node(OWNER)
    other.edge(model.EdgeKind.DEFINES.value, FILE, OWNER, layer=join.CODED)
    _declared_node(
        other,
        ids.build_node("/io_bridge"),
        REAL,
        remaps=[],
        name=EXE,
        name_source=model.NameSource.LITERAL.value,
    )
    return other


def test_a_parameter_read_the_symbol_did_not_declare_is_an_edge_on_the_parameter():
    other = _with_reads()
    report = join.join(other)
    param_id = ids.build_param("node:/io_bridge", "publish_rate_hz")
    edge = next(
        edge
        for edge in other.atlas.in_edges(param_id)
        if edge.kind == model.EdgeKind.READS_PARAM.value
    )
    assert edge.src == OWNER
    assert edge.fact(join.CODED, REAL).values["api"] == "get_parameter"
    assert other.atlas.entities[param_id].state == model.ParamState.OTHER.value
    assert report.parameter_reads_bound == 2


def test_a_read_states_no_value_and_no_effective_block():
    other = _with_reads()
    join.join(other)
    node = other.atlas.entities[ids.build_param("node:/io_bridge", "publish_rate_hz")]
    assert node.effective == {}
    assert node.candidates == []


def test_a_declaration_the_code_reads_back_is_marked_read():
    other = _with_reads()
    join.join(other)
    node = other.atlas.entities[ids.build_param("node:/io_bridge", "publish_tf")]
    assert node.fact(join.CODED, REAL).values["read"] is True
    assert node.state == model.ParamState.EFFECTIVE.value


def test_a_symbol_fact_spelling_of_the_reads_list_reaches_the_join():
    other = model.AtlasBuilder()
    _coded(other, {**CODED_VALUES, "parameter_reads": READS})
    found = join.carriers(other.atlas)
    assert [row["name"] for row in found[0].parameter_reads] == ["publish_rate_hz", "publish_tf"]


def test_reads_are_read_from_the_record_the_collector_writes():
    other = model.AtlasBuilder()
    other.node(FILE, attrs={"parameter_reads": READS})
    assert len(join.carriers(other.atlas)[0].parameter_reads) == 2


READER_RELPATH = "nx_bridge/aaa_reader.py"
"""Sorts before the file that declares, which is the order that used to decide the answer."""


def _split_read_and_declaration() -> model.AtlasBuilder:
    """One instance whose reading file and whose declaring file are two different files."""
    other = model.AtlasBuilder()
    profile = _profile(other, "nx_navigation")
    for relpath, values in (
        (
            READER_RELPATH,
            {
                "parameter_reads": [
                    {"name": "alpha", "api": "get_parameter", "owner_symbol": "Reader.tick"},
                    {"name": "beta", "api": "get_parameter", "owner_symbol": "Reader.tick"},
                ]
            },
        ),
        (
            RELPATH,
            {
                "nodes": [{"name": EXE, "name_source": "literal", "form": "node", "symbol": SYM}],
                "parameters": [
                    {"name": "alpha", "type": "double", "default": 2.0, "owner_symbol": "IoBridge"}
                ],
            },
        ),
    ):
        file_id = ids.build_file(PKG, relpath)
        sym_id = ids.build_sym(PKG, relpath, "reader" if relpath == READER_RELPATH else "declarer")
        other.node(file_id, attrs={"language": "python"})
        other.node(sym_id)
        other.node(EXE_ID)
        other.edge(model.EdgeKind.DEFINES.value, file_id, sym_id, layer=join.CODED)
        other.edge(model.EdgeKind.RUNS.value, EXE_ID, sym_id, layer=join.DECLARED)
        other.fact(sym_id, join.CODED, values=values)
    _declared_node(
        other,
        ids.build_node("/io_bridge"),
        profile,
        remaps=[],
        name=EXE,
        name_source=model.NameSource.LITERAL.value,
    )
    return other


def test_a_read_in_one_file_never_denies_a_declaration_in_another():
    other = _split_read_and_declaration()
    join.join(other)
    node = other.atlas.entities[ids.build_param("node:/io_bridge", "alpha")]
    assert node.state == model.ParamState.EFFECTIVE.value
    assert node.attrs.get("notes") is None
    assert node.fact(join.CODED, REAL).values["read"] is True


def test_a_read_of_a_name_no_file_declares_still_states_the_blind_spot():
    other = _split_read_and_declaration()
    join.join(other)
    node = other.atlas.entities[ids.build_param("node:/io_bridge", "beta")]
    assert node.state == model.ParamState.OTHER.value
    assert node.attrs["notes"] == ["read in code, no declaration this reader could follow"]
    assert node.effective == {}


# ---------------------------------------------------------------------------
# the namespace the code states


NAMESPACED = {
    **CODED_VALUES,
    "nodes": [
        {
            "name": EXE,
            "name_source": "literal",
            "namespace": "rig",
            "namespace_source": "literal",
            "form": "node",
            "symbol": SYM,
        }
    ],
}


def _namespaced(namespace: str = "") -> model.AtlasBuilder:
    other = model.AtlasBuilder()
    _profile(other, "nx_navigation")
    _coded(other, NAMESPACED)
    _declared_node(other, synthetic_id(), REAL, remaps=[], namespace=namespace)
    return other


def test_a_namespace_stated_in_code_places_the_node_the_launch_file_left_at_the_root():
    other = _namespaced()
    report = join.join(other)
    node = other.atlas.entities[ids.build_node("/rig/io_bridge")]
    assert node.fact(join.CODED, REAL).values["code_namespace"] == "/rig"
    assert report.superseded == 1
    assert [edge.dst for edge in other.atlas.out_edges(node.id) if edge.kind == "publishes"] == [
        "topic:/rig/cmd_vel"
    ]


def test_a_launch_namespace_still_overrides_the_one_the_code_states():
    other = _namespaced("/robot1")
    join.join(other)
    assert ids.build_node("/robot1/io_bridge") in other.atlas.entities
    assert ids.build_node("/rig/io_bridge") not in other.atlas.entities


# ---------------------------------------------------------------------------
# a process a launch file starts by path

SCRIPT_PKG = "nx_bringup"
SCRIPT_RELPATH = "scripts/initial_nudge.py"
SCRIPT_FILE = ids.build_file(SCRIPT_PKG, SCRIPT_RELPATH)
SCRIPT_SYM = ids.build_sym(SCRIPT_PKG, SCRIPT_RELPATH, "scripts.initial_nudge.InitialNudge")
SCRIPT_VALUES: dict[str, Any] = {
    "nodes": [
        {
            "name": "initial_nudge",
            "name_source": "literal",
            "form": "node",
            "symbol": SCRIPT_SYM,
        }
    ],
    "endpoints": [
        {
            "kind": "publisher",
            "name": "/cmd_vel_raw",
            "type": "geometry_msgs/msg/Twist",
            "owner_symbol": "scripts.initial_nudge.InitialNudge",
            "line_hint": 63,
        }
    ],
    "parameters": [
        {
            "name": "chain",
            "type": "str",
            "default": "rpp",
            "owner_symbol": "scripts.initial_nudge.InitialNudge",
        }
    ],
}


def _script_process(argv: list[str]) -> model.AtlasBuilder:
    """One ExecuteProcess started by path, with the script the workspace holds behind it."""
    other = model.AtlasBuilder()
    _profile(other, "nx_sim")
    other.node(SCRIPT_FILE, attrs={"language": "python"})
    other.node(SCRIPT_SYM)
    other.edge(model.EdgeKind.DEFINES.value, SCRIPT_FILE, SCRIPT_SYM, layer=join.CODED)
    other.fact(SCRIPT_SYM, join.CODED, values=SCRIPT_VALUES)
    node_id = ids.build_node_synthetic("@unknown", "initial_nudge.py", "/", "59ce0bc2")
    other.node(node_id, name_source=model.NameSource.EXECUTABLE_DEFAULT.value)
    other.fact(
        node_id,
        join.DECLARED,
        profile=SIM,
        values={
            "pkg": "",
            "executable": "initial_nudge.py",
            "namespace": "/",
            "form": "process",
            "argv": argv,
        },
    )
    return other


INSTALLED = "/ws/install/nx_bringup/share/nx_bringup/scripts/initial_nudge.py"


def test_a_process_started_by_path_binds_to_the_script_its_argv_names():
    other = _script_process(["python3", INSTALLED, "--ros-args", "-p", "chain:=overlay"])
    report = join.join(other)
    assert report.scripts_bound == 1
    assert report.components_joined == 0
    node = other.atlas.entities[ids.build_node("/initial_nudge")]
    assert node.fact(join.CODED, SIM).values["bound_through"] == "script_path"
    assert [edge.dst for edge in other.atlas.out_edges(node.id) if edge.kind == "publishes"] == [
        "topic:/cmd_vel_raw"
    ]


def test_a_bare_file_name_in_an_argv_binds_to_no_script():
    other = _script_process(["python3", "/somewhere/else/initial_nudge.py"])
    report = join.join(other)
    assert report.scripts_bound == 0
    assert ids.build_node("/initial_nudge") not in other.atlas.entities


def test_a_default_the_command_line_of_the_process_overrides_never_wins():
    other = _script_process(["python3", INSTALLED, "--ros-args", "-p", "chain:=overlay"])
    join.join(other)
    param = other.atlas.entities[ids.build_param(ids.build_node("/initial_nudge"), "chain")]
    assert param.state == model.ParamState.OTHER.value
    assert param.effective == {}
    assert [candidate.wins for candidate in param.candidates] == [False]
    assert param.candidates[0].value == "rpp"


def test_a_default_no_command_line_touches_still_states_the_value():
    other = _script_process(["python3", INSTALLED])
    join.join(other)
    param = other.atlas.entities[ids.build_param(ids.build_node("/initial_nudge"), "chain")]
    assert param.state == model.ParamState.EFFECTIVE.value
    assert param.effective[SIM]["value"].value == "rpp"


def test_the_script_join_is_idempotent():
    other = _script_process(["python3", INSTALLED, "--ros-args", "-p", "chain:=overlay"])
    join.join(other)
    first = other.atlas.to_bytes()
    join.join(other)
    assert other.atlas.to_bytes() == first


# ---------------------------------------------------------------------------
# the other inter module contract: the declared bundle handoff, AGENTS section 2.1
#
# The join is one of the two contracts between collectors and the bundle is the other, and
# the registry that enforces it has no test file of its own. Both live here so a reader
# meets them together.


class _Fake:
    """A collector that only records whether it reached the bundle."""

    layer = "declared"

    def __init__(self, name: str, absent: str = "") -> None:
        self.name = name
        self.absent = absent
        self.walks: list[Any] | None = None

    def inputs_present(self, ctx: Any) -> Any:
        return self.absent or True

    def collect(self, ctx: Any) -> None:
        self.walks = list(ctx.declared.walks)


def _ctx(**extra: Any) -> Any:
    return collect.CollectContext(ws=None, cfg=None, builder=model.AtlasBuilder(), **extra)


def _registered(monkeypatch: Any, name: str, collector: Any) -> Any:
    monkeypatch.setitem(collect.REGISTRY, name, lambda: collector)
    return collect.get(name)


def test_the_bundle_names_both_halves_of_the_obligation():
    assert collect.BUNDLE_WRITERS == ("entrypoints", "launch_declared")
    assert collect.BUNDLE_READERS == ("params",)


def test_a_consumer_that_runs_before_its_producer_fails_loudly(monkeypatch: Any):
    """It answered with an empty bundle, which reads exactly like a workspace that launches
    nothing, so a parameter pass that had read nothing looked like one that read everything."""
    reader = _registered(monkeypatch, "params", _Fake("params"))
    ctx = _ctx()

    with pytest.raises(ContractError) as caught:
        reader.collect(ctx)

    assert "launch_declared" in str(caught.value)
    assert reader.walks is None


def test_the_producer_opens_the_bundle_and_the_consumer_then_reads_it(monkeypatch: Any):
    writer = _registered(monkeypatch, "launch_declared", _Fake("launch_declared"))
    reader = _registered(monkeypatch, "params", _Fake("params"))
    ctx = _ctx()

    writer.collect(ctx)
    ctx.declared.walks.append("one walk")
    reader.collect(ctx)

    assert ctx.produced == ["launch_declared"]
    assert reader.walks == ["one walk"]


def test_a_producer_with_no_input_still_answers_and_the_consumer_reads_an_empty_bundle(
    monkeypatch: Any,
):
    """A workspace that declares no profile is not a workspace nobody read. The pass was
    reached, the bundle is what it left behind, and an empty read after it is honest."""
    writer = _registered(monkeypatch, "launch_declared", _Fake("launch_declared", "no profile"))
    reader = _registered(monkeypatch, "params", _Fake("params"))
    ctx = _ctx()

    assert writer.inputs_present(ctx) == "no profile"
    reader.collect(ctx)

    assert reader.walks == []


def test_a_caller_that_hands_the_records_in_owns_them(monkeypatch: Any):
    """One collector under test against records a test wrote is not an ordering question."""
    reader = _registered(monkeypatch, "params", _Fake("params"))
    ctx = _ctx(declared=records.DeclaredBundle(walks=["one walk"]))

    reader.collect(ctx)

    assert reader.walks == ["one walk"]


def test_a_collector_built_directly_is_not_bound_by_the_registry():
    plain = _Fake("params")
    plain.collect(_ctx())
    assert plain.walks == []


def test_the_profile_scope_carries_the_same_bundle_and_the_same_state(monkeypatch: Any):
    writer = _registered(monkeypatch, "launch_declared", _Fake("launch_declared"))
    ctx = _ctx()
    writer.collect(ctx)

    scoped = ctx.for_profile("nx_navigation")

    assert scoped.declared is ctx.declared
    assert scoped.produced == ctx.produced
    assert scoped.profile == "nx_navigation"


def test_the_registry_hands_out_the_collector_it_was_given(monkeypatch: Any):
    """The binding is around the pass and never in place of the collector: every attribute a
    caller reads is the collector's own."""
    inner = _Fake("params")
    bound = _registered(monkeypatch, "params", inner)

    assert bound.name == "params"
    assert bound.layer == "declared"
    assert isinstance(bound, collect.Collector)


# ---------------------------------------------------------------------------
# parameter guards


ACCEL = endpoint(
    "publisher",
    "accel/filtered",
    type="geometry_msgs/msg/AccelWithCovarianceStamped",
    condition="publish_acceleration",
    guard_parameters=[{"name": "publish_acceleration", "value": True}],
)


def _guarded(
    rows: list[dict[str, Any]], values: dict[str, Any], **extra: Any
) -> model.AtlasBuilder:
    """/io_bridge in both profiles, with publish_acceleration set per profile as given."""
    other = model.AtlasBuilder()
    for profile in (REAL, SIM):
        _profile(other, ids.parse_profile(profile))
    _coded(other, {"nodes": CODED_VALUES["nodes"], "endpoints": rows, **extra})
    node_id = ids.build_node("/io_bridge")
    for profile in (REAL, SIM):
        _declared_node(
            other,
            node_id,
            profile,
            remaps=[],
            name=EXE,
            name_source=model.NameSource.LITERAL.value,
        )
    param_id = ids.build_param(node_id, "publish_acceleration")
    for profile, value in sorted(values.items()):
        other.set_param(
            param_id,
            state=model.ParamState.EFFECTIVE.value,
            candidates=[
                model.Candidate(
                    value=value,
                    value_type=type(value).__name__,
                    mechanism=model.Mechanism.YAML_BASE.value,
                    source=model.Source(file=FILE, locator="publish_acceleration"),
                    rank=0,
                    wins=True,
                    profile=profile,
                )
            ],
        )
    return other


def _published(atlas: model.Atlas, topic: str) -> list[str]:
    return sorted(
        profile
        for edge in atlas.in_edges(topic)
        if edge.kind == model.EdgeKind.PUBLISHES.value
        for profile in edge.profiles
    )


def test_an_endpoint_whose_parameter_guard_holds_is_bound():
    other = _guarded([ACCEL], {REAL: True, SIM: True})
    join.join(other)
    assert _published(other.atlas, "topic:/accel/filtered") == [REAL, SIM]


def test_an_endpoint_whose_parameter_guard_is_false_is_not_bound_in_that_profile():
    other = _guarded([ACCEL], {REAL: True, SIM: False})
    report = join.join(other)
    assert _published(other.atlas, "topic:/accel/filtered") == [REAL]
    assert SIM not in other.atlas.entities["topic:/accel/filtered"].profiles
    assert report.guarded_off == 1


def test_an_unknown_parameter_value_leaves_the_endpoint_bound():
    other = _guarded([ACCEL], {REAL: False})
    join.join(other)
    assert _published(other.atlas, "topic:/accel/filtered") == [SIM]


def test_a_value_that_is_not_a_bool_is_not_evaluated():
    other = _guarded([ACCEL], {REAL: 0, SIM: "false"})
    join.join(other)
    assert _published(other.atlas, "topic:/accel/filtered") == [REAL, SIM]


def test_the_else_arm_binds_where_the_parameter_is_false():
    other_arm = endpoint(
        "publisher",
        "accel/raw",
        condition="!publish_acceleration",
        guard_parameters=[{"name": "publish_acceleration", "value": False}],
    )
    other = _guarded([ACCEL, other_arm], {REAL: True, SIM: False})
    join.join(other)
    assert _published(other.atlas, "topic:/accel/filtered") == [REAL]
    assert _published(other.atlas, "topic:/accel/raw") == [SIM]


def test_a_conjunction_with_a_raw_term_is_not_evaluated():
    """The collector states guard_parameters only when every term is a parameter, so a
    condition text alone is never read, whatever it looks like."""
    raw = endpoint("publisher", "accel/filtered", condition="publish_acceleration && ready")
    other = _guarded([raw], {REAL: False, SIM: False})
    join.join(other)
    assert _published(other.atlas, "topic:/accel/filtered") == [REAL, SIM]


def test_one_false_term_of_a_parameter_conjunction_is_enough():
    both = endpoint(
        "publisher",
        "accel/filtered",
        condition="publish_acceleration && verbose",
        guard_parameters=[
            {"name": "publish_acceleration", "value": True},
            {"name": "verbose", "value": True},
        ],
    )
    other = _guarded([both], {REAL: True, SIM: False})
    join.join(other)
    assert _published(other.atlas, "topic:/accel/filtered") == [REAL]


def test_a_broadcast_whose_parameter_guard_is_false_is_not_bound():
    broadcast = {
        "kind": "dynamic",
        "parent": "odom",
        "child": "base_footprint",
        "condition": "publish_acceleration",
        "guard_parameters": [{"name": "publish_acceleration", "value": True}],
        "owner_symbol": "IoBridge.__init__",
    }
    other = _guarded([], {REAL: True, SIM: False}, tf_broadcasts=[broadcast])
    join.join(other)
    edges = [
        edge
        for edge in _tf_edges(other.atlas, "frame:base_footprint")
        if edge.kind in (model.EdgeKind.BROADCASTS_TF.value, model.EdgeKind.TF_PARENT.value)
    ]
    assert len(edges) == 2
    assert all(edge.profiles == [REAL] for edge in edges)


def test_guard_evaluation_reads_the_same_twice():
    first = _guarded([ACCEL], {REAL: True, SIM: False})
    second = _guarded([ACCEL], {REAL: True, SIM: False})
    join.join(first)
    join.join(second)
    assert first.atlas.canonical_bytes() == second.atlas.canonical_bytes()
    join.join(first)
    assert first.atlas.canonical_bytes() == second.atlas.canonical_bytes()


def test_a_class_a_source_brings_in_with_using_finds_its_methods():
    """turtlebot3.cpp says `using robotis::turtlebot3::TurtleBot3;` and defines
    `TurtleBot3::TurtleBot3`, so the construction in main never reached its node name."""
    table = {"TurtleBot3": ["ctor"], "other::Twin": ["a"], "Twin": ["b"]}
    classes = {"robotis::turtlebot3::TurtleBot3", "other::Twin", "more::Twin"}
    assert join._methods_of(table, "robotis::turtlebot3::TurtleBot3", classes) == ["ctor"]
    assert join._methods_of(table, "other::Twin", classes) == ["a"]
    assert join._methods_of(table, "more::Twin", classes) == []


def test_a_helper_construction_speaks_only_for_the_node_it_builds():
    """`main` building a parameter client to take any parameter says nothing of the node the
    launch file starts from the same executable."""
    found = (
        join.Carrier(
            owner=ids.build_file("demo", "src/main.cpp"),
            file=ids.build_file("demo", "src/main.cpp"),
            symbol="main",
            nodes=(
                {"name": "scan_filter", "class": "ScanFilter"},
                {"name": "param_client", "class": "rclcpp::Node"},
            ),
            constructions=(
                {"class": "rclcpp::Node", "auto_declare": True},
                {"class": "ScanFilter"},
            ),
        ),
    )
    assert join.auto_declare_of("scan_filter", found) is None
    assert join.auto_declare_of("param_client", found) is True
    assert join.auto_declare_of("", found) is True
