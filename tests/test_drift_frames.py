"""Tests for the frame, remap, lifecycle, build and launch rule families.

Per rule: it fires on a crafted atlas, it stays silent on the near miss, and the finding id
is the same across two runs. The near miss is the case that would make the rule noise on a
real stack, not a trivially empty atlas.
"""

from __future__ import annotations

from dataclasses import replace

import pytest

from rosio import ids, model
from rosio.drift import build as build_rules
from rosio.drift import frames as frame_rules
from rosio.drift import rules
from rosio.errors import ContractError
from rosio.model import AtlasBuilder, Candidate, Source, Unresolved

DECLARED = model.Layer.DECLARED.value
CODED = model.Layer.CODED.value
CERTAIN = model.Confidence.CERTAIN.value
NAV = "profile:nx_navigation"
SIM = "profile:nx_sim"
MAP = "profile:nx_mapping"


# ---------------------------------------------------------------------------
# fixture helpers


def _builder() -> AtlasBuilder:
    builder = AtlasBuilder()
    builder.add_profile("nx_navigation")
    builder.add_profile("nx_sim")
    builder.add_profile("nx_mapping")
    return builder


def _model_file(builder: AtlasBuilder, name: str) -> str:
    file_id = ids.build_file("nx_bringup", f"urdf/{name}")
    builder.node(file_id)
    return file_id


def _link(builder: AtlasBuilder, frame: str, models: list[str]) -> str:
    frame_id = ids.build_frame(frame)
    builder.node(frame_id, attrs={"urdf_models": sorted(models)})
    builder.fact(
        frame_id,
        DECLARED,
        values={"declared_in_urdf": True},
        confidence=CERTAIN,
        evidence=[Source(file=models[0], locator=f"link[{frame}]")],
    )
    return frame_id


def _joint(
    builder: AtlasBuilder,
    file_id: str,
    joint: str,
    parent: str,
    child: str,
    xyz: list[float],
    profile: str = model.ANY_PROFILE,
) -> None:
    builder.edge(
        model.EdgeKind.TF_PARENT.value,
        ids.build_frame(child),
        ids.build_frame(parent),
        key=ids.edge_key(model=file_id, joint=joint, profile=profile),
        layer=DECLARED,
        profile=profile,
        values={
            "strategy": "urdf",
            "model": file_id,
            "joint": joint,
            "xyz": list(xyz),
            "rpy": [0.0, 0.0, 0.0],
        },
        confidence=CERTAIN,
        evidence=[Source(file=file_id, locator=f"joint[{joint}]", line_hint=12)],
    )


def _describe(builder: AtlasBuilder, profile: str, frames: list[str]) -> None:
    """The robot_description one profile hands its state publisher, which is what says
    which model that profile loads."""
    text = "".join(f'<link name="{frame}"/>' for frame in sorted(frames))
    builder.set_param(
        ids.build_param(ids.build_node("/robot_state_publisher"), "robot_description"),
        state=model.ParamState.EFFECTIVE.value,
        candidates=[
            Candidate(
                value=f'<robot name="fx_robot">{text}</robot>',
                value_type="str",
                mechanism=model.Mechanism.LAUNCH_INLINE_DICT.value,
                source=Source(
                    file=ids.build_file("nx_bringup", "launch/rsp.launch.py"),
                    locator="parameters",
                ),
                rank=0,
                wins=True,
                profile=profile,
            )
        ],
    )


def _fleet(
    builder: AtlasBuilder, fleet: dict[str, list[str]], loaded: dict[str, str]
) -> dict[str, str]:
    """Every model with the links it declares, and the profile that loads each one."""
    files = {name: _model_file(builder, name) for name in sorted(fleet)}
    owners: dict[str, list[str]] = {}
    for name in sorted(fleet):
        for frame in fleet[name]:
            owners.setdefault(frame, []).append(files[name])
    for frame in sorted(owners):
        _link(builder, frame, sorted(owners[frame]))
    for name in sorted(loaded):
        _describe(builder, loaded[name], fleet[name])
    return files


def _two_models(same_origin: bool, loaded: dict[str, str] | None = None) -> AtlasBuilder:
    builder = _builder()
    files = _fleet(
        builder,
        {
            "fx_robot.urdf": ["base_link", "os_sensor", "hitch_link"],
            "fx_robot_sim.urdf": ["base_link", "os_sensor", "imu_link"],
        },
        {"fx_robot.urdf": NAV, "fx_robot_sim.urdf": SIM} if loaded is None else loaded,
    )
    real, sim = files["fx_robot.urdf"], files["fx_robot_sim.urdf"]
    _joint(builder, real, "sensor_joint", "base_link", "os_sensor", [0.4, 0.0, 0.25])
    _joint(
        builder,
        sim,
        "sensor_joint",
        "base_link",
        "os_sensor",
        [0.4, 0.0, 0.25] if same_origin else [0.55, 0.0, 0.31],
    )
    return builder


def _node(builder: AtlasBuilder, fqn: str, profile: str, values: dict) -> str:
    node_id = ids.build_node(fqn)
    builder.node(node_id, name_source=model.NameSource.LITERAL.value)
    builder.fact(
        node_id,
        DECLARED,
        profile=profile,
        values=values,
        confidence=model.Confidence.RESOLVED.value,
        evidence=[Source(file=ids.build_file("nx_bringup", "launch/x.launch.py"))],
    )
    return node_id


def _param(
    builder: AtlasBuilder,
    owner: str,
    name: str,
    value: object,
    profile: str,
    value_type: str = "str",
) -> str:
    param_id = ids.build_param(owner, name)
    builder.set_param(
        param_id,
        state=model.ParamState.EFFECTIVE.value,
        candidates=[
            Candidate(
                value=value,
                value_type=value_type,
                mechanism=model.Mechanism.YAML_BASE.value,
                source=Source(
                    file=ids.build_file("nx_bringup", "config/nav2_params.yaml"),
                    locator=name,
                    line_hint=40,
                ),
                rank=0,
                wins=True,
                profile=profile,
            )
        ],
    )
    builder.edge(model.EdgeKind.HAS_PARAM.value, owner, param_id, layer=DECLARED, profile=profile)
    return param_id


def _config_frame_atlas(frame: str) -> AtlasBuilder:
    builder = _builder()
    real = _model_file(builder, "fx_robot.urdf")
    _link(builder, "base_link", [real])
    _link(builder, "os_sensor", [real])
    _joint(builder, real, "sensor_joint", "base_link", "os_sensor", [0.4, 0.0, 0.25])
    owner = _node(builder, "/ouster/os_driver", NAV, {"executable": "os_driver"})
    _param(builder, owner, "imu_frame", frame, NAV)
    return builder


def _ids(findings) -> list[str]:
    return sorted(finding.id for finding in findings)


# ---------------------------------------------------------------------------
# frame.parent_differs


def test_parent_differs_fires_when_two_models_place_one_frame_apart():
    atlas = _two_models(same_origin=False).build()
    found = frame_rules.frame_parent_differs(atlas)
    assert [item.subject for item in found] == ["frame:os_sensor"]
    assert "0.4" in found[0].say and "0.55" in found[0].say
    assert "fx_robot_sim.urdf" in found[0].say


def test_parent_differs_names_the_profile_that_loads_each_model():
    found = frame_rules.frame_parent_differs(_two_models(same_origin=False).build())
    assert "loaded by profile nx_navigation" in found[0].say
    assert "loaded by profile nx_sim" in found[0].say
    assert found[0].profiles == sorted([SIM, NAV])
    assert "loaded by profile nx_navigation" in found[0].detail


def test_parent_differs_is_silent_when_the_models_agree():
    atlas = _two_models(same_origin=True).build()
    assert frame_rules.frame_parent_differs(atlas) == []


def test_parent_differs_is_silent_when_only_one_of_the_two_models_is_loaded():
    atlas = _two_models(same_origin=False, loaded={"fx_robot.urdf": NAV}).build()
    assert frame_rules.frame_parent_differs(atlas) == []


def test_parent_differs_is_silent_when_no_profile_states_a_model():
    atlas = _two_models(same_origin=False, loaded={}).build()
    assert frame_rules.frame_parent_differs(atlas) == []


def test_parent_differs_is_silent_on_one_model_and_on_unknown_confidence():
    builder = _builder()
    files = _fleet(
        builder,
        {"fx_robot.urdf": ["base_link", "os_sensor"]},
        {"fx_robot.urdf": NAV},
    )
    _joint(builder, files["fx_robot.urdf"], "sensor_joint", "base_link", "os_sensor", [0.4, 0, 0])
    assert frame_rules.frame_parent_differs(builder.build()) == []

    builder = _two_models(same_origin=False)
    for edge in builder.atlas.edges.values():
        for fact in edge.facts:
            fact.confidence = model.Confidence.UNKNOWN.value
    assert frame_rules.frame_parent_differs(builder.build()) == []


def _three_models(third_agrees_with_first: bool, trailer_loaded: bool = True) -> AtlasBuilder:
    builder = _builder()
    loaded = {"fx_robot.urdf": NAV, "fx_robot_sim.urdf": SIM}
    if trailer_loaded:
        loaded["fx_robot_trailer.urdf"] = MAP
    files = _fleet(
        builder,
        {
            "fx_robot.urdf": ["base_link", "os_sensor", "hitch_link"],
            "fx_robot_sim.urdf": ["base_link", "os_sensor", "imu_link"],
            "fx_robot_trailer.urdf": ["base_link", "os_sensor", "trailer_link"],
        },
        loaded,
    )
    real, sim = files["fx_robot.urdf"], files["fx_robot_sim.urdf"]
    _joint(builder, real, "sensor_joint", "base_link", "os_sensor", [0.4, 0.0, 0.25])
    _joint(builder, sim, "sensor_joint", "base_link", "os_sensor", [0.55, 0.0, 0.31])
    tail = [0.4, 0.0, 0.25] if third_agrees_with_first else [0.6, 0.0, 0.35]
    _joint(builder, files["fx_robot_trailer.urdf"], "sensor_joint", "base_link", "os_sensor", tail)
    return builder


def test_parent_differs_quotes_two_models_that_really_hold_two_values():
    found = frame_rules.frame_parent_differs(_three_models(third_agrees_with_first=True).build())
    assert len(found) == 1
    assert "0.4 0 0.25" in found[0].say and "0.55 0 0.31" in found[0].say
    assert found[0].say.count("0.4 0 0.25") == 1


def test_parent_differs_drops_the_model_no_profile_loads():
    """The trailer robot is the only model that differs, and nothing builds it."""
    builder = _three_models(third_agrees_with_first=False, trailer_loaded=False)
    for edge in builder.atlas.edges.values():
        for fact in edge.facts:
            if str(fact.values.get("model", "")).endswith("fx_robot_sim.urdf"):
                fact.values["xyz"] = [0.4, 0.0, 0.25]
    assert frame_rules.frame_parent_differs(builder.build()) == []


def _parent_atlas(trailer_loaded: bool) -> AtlasBuilder:
    builder = _builder()
    loaded = {"fx_robot.urdf": NAV, "fx_robot_sim.urdf": SIM}
    if trailer_loaded:
        loaded["fx_robot_trailer.urdf"] = MAP
    files = _fleet(
        builder,
        {
            "fx_robot.urdf": ["base_link", "os_sensor", "imu_link", "hitch_link"],
            "fx_robot_sim.urdf": ["base_link", "os_sensor", "imu_link", "lidar_left"],
            "fx_robot_trailer.urdf": ["base_link", "os_sensor", "imu_link", "trailer_link"],
        },
        loaded,
    )
    _joint(builder, files["fx_robot.urdf"], "imu_joint", "base_link", "imu_link", [0.0, 0.0, 0.0])
    _joint(
        builder,
        files["fx_robot_trailer.urdf"],
        "imu_joint",
        "os_sensor",
        "imu_link",
        [0.0, 0.0, 0.0],
    )
    return builder


def test_parent_differs_quotes_two_parents_that_really_differ():
    found = frame_rules.frame_parent_differs(_parent_atlas(trailer_loaded=True).build())
    assert len(found) == 1
    assert "under base_link" in found[0].say and "under os_sensor" in found[0].say


def test_parent_differs_is_silent_when_the_second_parent_is_in_a_model_nothing_loads():
    assert frame_rules.frame_parent_differs(_parent_atlas(trailer_loaded=False).build()) == []


def test_parent_differs_keeps_its_id_across_two_runs_and_over_a_value_edit():
    first = _ids(frame_rules.frame_parent_differs(_two_models(same_origin=False).build()))
    second = _ids(frame_rules.frame_parent_differs(_two_models(same_origin=False).build()))
    assert first == second

    builder = _two_models(same_origin=False)
    for edge in builder.atlas.edges.values():
        for fact in edge.facts:
            if fact.values.get("model", "").endswith("fx_robot_sim.urdf"):
                fact.values["xyz"] = [9.0, 9.0, 9.0]
    assert _ids(frame_rules.frame_parent_differs(builder.build())) == first


# ---------------------------------------------------------------------------
# frame.orphan_root


LIO_SYMBOL = "sym:lio_pkg|src/laserMapping.cpp|publish_odometry"
LIO_GUARD = "scan_timestamp.seconds() > 0.0"


def _broadcast_tie(builder: AtlasBuilder, parent: str, child: str, guard: str = "") -> None:
    """A code broadcast of parent to child, written in publish_odometry and cited from main."""
    source_file = ids.build_file("lio_pkg", "src/laserMapping.cpp")
    builder.node(ids.build_frame(parent))
    builder.node(ids.build_frame(child))
    values = {"source": "cpp_broadcast", "symbol": LIO_SYMBOL}
    if guard:
        values["condition"] = guard
    builder.edge(
        model.EdgeKind.TF_PARENT.value,
        ids.build_frame(parent),
        ids.build_frame(child),
        key=ids.edge_key(sym=LIO_SYMBOL, guard=guard),
        layer=CODED,
        values=values,
        confidence=CERTAIN,
        evidence=[
            Source(file=source_file, locator="main", line_hint=9),
            Source(file=source_file, locator="publish_odometry", line_hint=684),
        ],
    )


def _orphan_atlas(tied: bool, guard: str = "") -> AtlasBuilder:
    builder = _builder()
    real = _model_file(builder, "fx_robot.urdf")
    _link(builder, "base_link", [real])
    _link(builder, "os_sensor", [real])
    _joint(builder, real, "sensor_joint", "base_link", "os_sensor", [0.4, 0.0, 0.25])
    _broadcast_tie(builder, "camera_init", "body", guard)
    if tied:
        _node(
            builder,
            "/lio_body_frame_tie",
            NAV,
            {
                "executable": "static_transform_publisher",
                "argv": ["--frame-id", "os_sensor", "--child-frame-id", "body"],
            },
        )
    return builder


def test_orphan_root_fires_when_a_second_tree_is_tied_to_nothing():
    found = frame_rules.frame_orphan_root(_orphan_atlas(tied=False).build())
    assert [item.subject for item in found] == ["frame:camera_init"]
    assert "camera_init" in found[0].say and "base_link" in found[0].say


def test_orphan_root_ignores_an_installed_model_no_profile_loads():
    """robot_localization on /opt/ros/jazzy loads no robot model, and the install tree ships
    a simulation turtlebot beside it. That model is not a second tree of anything."""
    builder = _orphan_atlas(tied=False)
    for edge in builder.atlas.edges.values():
        for fact in edge.facts:
            fact.evidence = [replace(row, root="install") for row in fact.evidence]
    found = frame_rules.frame_orphan_root(builder.build())
    assert found == []


def test_orphan_root_is_silent_when_a_static_publisher_ties_the_two_trees():
    assert frame_rules.frame_orphan_root(_orphan_atlas(tied=True).build()) == []


def _two_robots_atlas(*, broken_joint: bool) -> AtlasBuilder:
    """Two robot models in one tree of files, the way an install tree ships them."""
    builder = _builder()
    robot = _model_file(builder, "fx_robot.urdf")
    arm = _model_file(builder, "fx_test_arm.urdf")
    for frame in ("base_link", "os_sensor", "laser"):
        _link(builder, frame, [robot])
    _joint(builder, robot, "sensor_joint", "base_link", "os_sensor", [0.4, 0.0, 0.25])
    if broken_joint:
        # a second root inside the same model: the laser joint names a parent nobody has
        _link(builder, "laser_mount", [robot])
        _joint(builder, robot, "laser_joint", "laser_mount", "laser", [0.1, 0.0, 0.2])
    for frame in ("world", "arm_link"):
        _link(builder, frame, [arm])
    _joint(builder, arm, "arm_joint", "world", "arm_link", [0.0, 0.0, 1.0])
    return builder


def test_orphan_root_is_silent_on_a_second_robot_model():
    """jazzy installs a turtlebot and a test arm side by side; two robots are not one robot
    with a broken tree."""
    assert frame_rules.frame_orphan_root(_two_robots_atlas(broken_joint=False).build()) == []


def test_orphan_root_still_fires_inside_one_model_beside_another_robot():
    found = frame_rules.frame_orphan_root(_two_robots_atlas(broken_joint=True).build())
    assert [item.subject for item in found] == ["frame:laser_mount"]


def _prefixed_atlas(*, broken_joint: bool) -> AtlasBuilder:
    """One model expanded with its defaults for every profile, and with a namespace argument
    for one profile, the way turtlebot3 passes `namespace:=tb3/` to xacro."""
    builder = _builder()
    robot = _model_file(builder, "fx_robot.urdf")
    for frame in ("base_link", "os_sensor"):
        _link(builder, frame, [robot])
    _joint(builder, robot, "sensor_joint", "base_link", "os_sensor", [0.4, 0.0, 0.25])
    for frame in ("tb3/base_link", "tb3/os_sensor", "tb3/laser"):
        _link(builder, frame, [robot])
    joint = ("tb3/sensor_joint", "tb3/base_link", "tb3/os_sensor", [0.4, 0.0, 0.25])
    _joint(builder, robot, *joint, profile=NAV)
    if broken_joint:
        _link(builder, "tb3/laser_mount", [robot])
        _joint(builder, robot, "tb3/laser_joint", "tb3/laser_mount", "tb3/laser", [0.1, 0, 0], NAV)
    return builder


def test_orphan_root_is_silent_on_a_model_one_profile_expands_with_its_own_arguments():
    assert frame_rules.frame_orphan_root(_prefixed_atlas(broken_joint=False).build()) == []


def test_orphan_root_still_fires_inside_the_expansion_one_profile_loads():
    found = frame_rules.frame_orphan_root(_prefixed_atlas(broken_joint=True).build())
    assert [item.subject for item in found] == ["frame:tb3/laser_mount"]


def test_orphan_root_keeps_its_id_across_two_runs():
    first = _ids(frame_rules.frame_orphan_root(_orphan_atlas(tied=False).build()))
    second = _ids(frame_rules.frame_orphan_root(_orphan_atlas(tied=False).build()))
    assert first == second and len(first) == 1


def test_orphan_root_says_which_tie_of_the_second_tree_holds_only_under_a_guard():
    found = frame_rules.frame_orphan_root(_orphan_atlas(tied=False, guard=LIO_GUARD).build())
    assert [item.subject for item in found] == ["frame:camera_init"]
    clause = f"camera_init to body is broadcast only when {LIO_GUARD}, in "
    assert found[0].say.endswith(f"fails, and {clause}lio_pkg/src/laserMapping.cpp:684.")
    assert found[0].detail.endswith(f"Of those ties, {clause}lio_pkg/src/laserMapping.cpp.")


def test_orphan_root_states_no_guard_when_an_unguarded_site_ties_the_same_pair():
    builder = _orphan_atlas(tied=False, guard=LIO_GUARD)
    _broadcast_tie(builder, "camera_init", "body")
    found = frame_rules.frame_orphan_root(builder.build())
    assert len(found) == 1 and "only when" not in found[0].say + found[0].detail


def test_orphan_root_is_silent_when_a_guarded_broadcast_ties_the_two_trees():
    """The guard is never evaluated, so a guarded tie still joins the trees."""
    builder = _orphan_atlas(tied=False)
    _broadcast_tie(builder, "os_sensor", "body", LIO_GUARD)
    assert frame_rules.frame_orphan_root(builder.build()) == []


def test_orphan_root_keeps_its_id_when_the_tie_gains_a_guard():
    plain = _ids(frame_rules.frame_orphan_root(_orphan_atlas(tied=False).build()))
    guarded = _ids(frame_rules.frame_orphan_root(_orphan_atlas(False, LIO_GUARD).build()))
    again = _ids(frame_rules.frame_orphan_root(_orphan_atlas(False, LIO_GUARD).build()))
    assert plain == guarded == again and len(plain) == 1


# ---------------------------------------------------------------------------
# frame.not_in_urdf


def test_not_in_urdf_fires_on_a_frame_only_a_yaml_line_states():
    found = frame_rules.frame_not_in_urdf(_config_frame_atlas("os_imu").build())
    assert [item.subject for item in found] == ["param:/ouster/os_driver|imu_frame"]
    assert "os_imu" in found[0].say and "nav2_params.yaml:40" in found[0].say
    assert found[0].severity == rules.severity_of("frame.not_in_urdf")


@pytest.mark.parametrize("frame", ["os_sensor", "map", "odom", "earth"])
def test_not_in_urdf_is_silent_on_a_declared_link_and_on_a_world_frame(frame):
    assert frame_rules.frame_not_in_urdf(_config_frame_atlas(frame).build()) == []


def test_not_in_urdf_is_silent_when_a_transform_names_the_frame():
    builder = _config_frame_atlas("os_imu")
    _node(
        builder,
        "/ouster_imu_tie",
        NAV,
        {
            "executable": "static_transform_publisher",
            "argv": ["--frame-id", "os_sensor", "--child-frame-id", "os_imu"],
        },
    )
    assert frame_rules.frame_not_in_urdf(builder.build()) == []


@pytest.mark.parametrize(
    "values",
    [
        {"argv": ["0", "0", "0", "0", "0", "0", "os_sensor", "os_imu"]},
        {
            "command_line": [
                "@outside/static_transform_publisher",
                *["0", "0", "0", "0", "0", "0", "1"],
                *["os_sensor", "os_imu"],
            ]
        },
        {
            "command_line": [
                "@outside/static_transform_publisher",
                *["0", "0", "0", "0", "0", "0"],
                *["os_sensor", "os_imu", "--ros-args", "-r", "__node:=st"],
            ]
        },
    ],
)
def test_a_static_publisher_names_its_positional_pair_on_a_whole_command_line(values):
    """The process table states the whole command line, the program first and the rules
    launch appends after `--ros-args` last, so the positional pair is read between the two."""
    builder = _config_frame_atlas("os_imu")
    _node(builder, "/ouster_imu_tie", NAV, {"executable": "static_transform_publisher", **values})
    assert frame_rules.frame_not_in_urdf(builder.build()) == []


def test_not_in_urdf_is_silent_when_a_broadcast_edge_names_the_frame():
    builder = _config_frame_atlas("os_imu")
    source = _node(builder, "/fastlio_mapping", NAV, {"executable": "fastlio_mapping"})
    builder.node(ids.build_frame("os_imu"))
    builder.edge(
        model.EdgeKind.BROADCASTS_TF.value,
        source,
        ids.build_frame("os_imu"),
        key=ids.edge_key(sym="sym:lio_pkg|src/laserMapping.cpp|publish_odometry"),
        layer=CODED,
        profile=NAV,
        values={"frame": "os_imu"},
        confidence=CERTAIN,
        evidence=[Source(file=ids.build_file("lio_pkg", "src/laserMapping.cpp"))],
    )
    assert frame_rules.frame_not_in_urdf(builder.build()) == []


def test_not_in_urdf_is_silent_when_a_guarded_broadcast_names_the_frame():
    builder = _config_frame_atlas("os_imu")
    _broadcast_tie(builder, "os_sensor", "os_imu", LIO_GUARD)
    assert frame_rules.frame_not_in_urdf(builder.build()) == []


def test_the_transform_publish_flags_are_the_documented_set():
    """A member is a behaviour: removing one makes `frame.not_in_urdf` fire on a driver that
    states it. `tf_send_en` is Point-LIO's `publish.tf_send_en`."""
    assert sorted(frame_rules.TF_PUBLISH_FLAGS) == [
        "broadcast_tf",
        "pub_static_tf",
        "publish_tf",
        "publish_transform",
        "tf",
        "tf_broadcast",
        "tf_send_en",
    ]


@pytest.mark.parametrize(
    "flag", ["pub_static_tf", "publish_tf", "publish.tf", "publish.tf_send_en"]
)
def test_not_in_urdf_is_silent_when_the_driver_publishes_its_own_transforms(flag):
    builder = _config_frame_atlas("os_imu")
    _param(builder, ids.build_node("/ouster/os_driver"), flag, True, NAV, value_type="bool")
    assert frame_rules.frame_not_in_urdf(builder.build()) == []


def test_not_in_urdf_still_fires_when_the_same_flag_is_off():
    builder = _config_frame_atlas("os_imu")
    _param(
        builder, ids.build_node("/ouster/os_driver"), "pub_static_tf", False, NAV, value_type="bool"
    )
    found = frame_rules.frame_not_in_urdf(builder.build())
    assert [item.subject for item in found] == ["param:/ouster/os_driver|imu_frame"]


def test_not_in_urdf_is_silent_when_no_line_states_the_value():
    """A vendor defaults document a launch file loads out of the ros share tree arrives as
    an inline dict with no line, so the atlas cannot say where the frame id is written."""
    builder = _builder()
    real = _model_file(builder, "fx_robot.urdf")
    _link(builder, "base_link", [real])
    owner = _node(builder, "/microstrain/microstrain_inertial_driver", NAV, {"executable": "imu"})
    param_id = ids.build_param(owner, "gnss1_frame_id")
    builder.set_param(
        param_id,
        state=model.ParamState.EFFECTIVE.value,
        candidates=[
            Candidate(
                value="gnss_1_antenna_link",
                value_type="str",
                mechanism=model.Mechanism.LAUNCH_INLINE_DICT.value,
                source=Source(
                    file=ids.build_file("nx_bringup", "launch/microstrain.launch.py"),
                    locator="parameters",
                ),
                rank=0,
                wins=True,
                profile=NAV,
            )
        ],
    )
    builder.edge(model.EdgeKind.HAS_PARAM.value, owner, param_id, layer=DECLARED, profile=NAV)
    assert frame_rules.frame_not_in_urdf(builder.build()) == []


def test_not_in_urdf_says_where_the_frame_id_came_from():
    found = frame_rules.frame_not_in_urdf(_config_frame_atlas("os_imu").build())
    assert "nav2_params.yaml:40" in found[0].say
    assert "nothing in this atlas publishes it" in found[0].say
    assert "written in nx_bringup/config/nav2_params.yaml." in found[0].detail


def test_not_in_urdf_keeps_the_line_out_of_the_fingerprint():
    """The fingerprint is a waiver key, so a line added above the value must not move it."""
    builder = _config_frame_atlas("os_imu")
    node = builder.atlas.entities[ids.build_param(ids.build_node("/ouster/os_driver"), "imu_frame")]
    first = frame_rules.frame_not_in_urdf(builder.build())[0].fingerprint
    for candidate in node.candidates:
        candidate.source = Source(
            file=candidate.source.file,
            locator=candidate.source.locator,
            line_hint=candidate.source.line_hint + 6,
        )
    builder.recompute()
    assert frame_rules.frame_not_in_urdf(builder.build())[0].fingerprint == first


def test_not_in_urdf_skips_a_sub_key_of_a_plugin_that_is_not_selected():
    builder = _config_frame_atlas("os_sensor")
    owner = ids.build_node("/controller_server")
    builder.node(owner, name_source=model.NameSource.LITERAL.value)
    builder.node(ids.build_pinst(owner, "FollowPath"))
    _param(builder, owner, "Absent.state_frame", "ghost_frame", NAV)
    assert frame_rules.frame_not_in_urdf(builder.build()) == []


def test_not_in_urdf_names_the_line_that_sets_the_key_it_quotes():
    builder = _config_frame_atlas("os_sensor")
    owner = _node(builder, "/gnss/gnss_driver", NAV, {"executable": "gnss"})
    for name, line in (("frame_id", 12), ("aux1_frame_id", 18)):
        param_id = ids.build_param(owner, name)
        builder.set_param(
            param_id,
            state=model.ParamState.EFFECTIVE.value,
            candidates=[
                Candidate(
                    value="gnss_aux",
                    value_type="str",
                    mechanism=model.Mechanism.YAML_BASE.value,
                    source=Source(
                        file=ids.build_file("nx_bringup", "config/gnss.yaml"),
                        locator=name,
                        line_hint=line,
                    ),
                    rank=0,
                    wins=True,
                    profile=NAV,
                )
            ],
        )
        builder.edge(model.EdgeKind.HAS_PARAM.value, owner, param_id, layer=DECLARED, profile=NAV)
    found = frame_rules.frame_not_in_urdf(builder.build())
    assert len(found) == 1
    assert "aux1_frame_id" in found[0].say and "gnss.yaml:18" in found[0].say
    assert "gnss.yaml:12" not in found[0].say


def test_not_in_urdf_keeps_its_id_across_two_runs():
    first = _ids(frame_rules.frame_not_in_urdf(_config_frame_atlas("os_imu").build()))
    second = _ids(frame_rules.frame_not_in_urdf(_config_frame_atlas("os_imu").build()))
    assert first == second and len(first) == 1


# ---------------------------------------------------------------------------
# frame.no_publisher


def _listener_atlas(broadcast: bool, rsp: bool) -> AtlasBuilder:
    builder = _builder()
    real = _model_file(builder, "fx_robot.urdf")
    _link(builder, "base_link", [real])
    _link(builder, "os_sensor", [real])
    _joint(builder, real, "sensor_joint", "base_link", "os_sensor", [0.4, 0.0, 0.25])
    builder.node(ids.build_frame("radar_link"))
    listener = _node(builder, "/nx_perception", NAV, {"executable": "detector_node"})
    for frame in ("os_sensor", "radar_link"):
        builder.edge(
            model.EdgeKind.LISTENS_TF.value,
            listener,
            ids.build_frame(frame),
            key=ids.edge_key(sym="sym:nx_perception|src/detector_node.cpp|onInit", frame=frame),
            layer=CODED,
            profile=NAV,
            values={"frame": frame},
            confidence=CERTAIN,
            evidence=[Source(file=ids.build_file("nx_perception", "src/detector_node.cpp"))],
        )
    if rsp:
        _node(builder, "/robot_state_publisher", NAV, {"executable": "robot_state_publisher"})
    if broadcast:
        source = _node(builder, "/fastlio_mapping", NAV, {"executable": "fastlio_mapping"})
        builder.node(ids.build_frame("body"))
        builder.edge(
            model.EdgeKind.BROADCASTS_TF.value,
            source,
            ids.build_frame("body"),
            key=ids.edge_key(sym="sym:lio_pkg|src/laserMapping.cpp|publish_odometry"),
            layer=CODED,
            profile=NAV,
            values={"frame": "body"},
            confidence=CERTAIN,
            evidence=[Source(file=ids.build_file("lio_pkg", "src/laserMapping.cpp"))],
        )
    return builder


def test_no_publisher_fires_on_a_listened_frame_nothing_publishes():
    found = frame_rules.frame_no_publisher(_listener_atlas(broadcast=True, rsp=True).build())
    assert [item.subject for item in found] == ["frame:radar_link"]
    assert found[0].profiles == [NAV]


def test_no_publisher_is_silent_without_broadcast_coverage():
    assert frame_rules.frame_no_publisher(_listener_atlas(broadcast=False, rsp=True).build()) == []


def test_no_publisher_is_silent_on_a_urdf_link_a_state_publisher_carries():
    found = frame_rules.frame_no_publisher(_listener_atlas(broadcast=True, rsp=True).build())
    assert "frame:os_sensor" not in [item.subject for item in found]


def _unused_model_with_radar(identified: bool) -> AtlasBuilder:
    builder = _listener_atlas(broadcast=True, rsp=True)
    spare = _model_file(builder, "fx_spare.urdf")
    builder.atlas.entities.pop(ids.build_frame("radar_link"))
    _link(builder, "radar_link", [spare])
    _joint(builder, spare, "radar_joint", "base_link", "radar_link", [1.0, 0.0, 0.5])
    if identified:
        _describe(builder, NAV, ["base_link", "os_sensor"])
    return builder


def test_no_publisher_ignores_a_model_the_profile_does_not_load():
    """The spare model places radar_link, but the profile loads the other model."""
    found = frame_rules.frame_no_publisher(_unused_model_with_radar(identified=True).build())
    assert [item.subject for item in found] == ["frame:radar_link"]


def test_no_publisher_counts_every_model_when_the_loaded_one_is_unknown():
    found = frame_rules.frame_no_publisher(_unused_model_with_radar(identified=False).build())
    assert found == []


def test_no_publisher_is_silent_when_a_broadcaster_builds_a_frame_name_at_run_time():
    """A broadcaster whose child frame comes from a parameter may be the publisher."""
    builder = _listener_atlas(broadcast=True, rsp=True)
    source_file = ids.build_file("lio_pkg", "src/laserMapping.cpp")
    builder.node(source_file)
    builder.fact(
        ids.build_node("/fastlio_mapping"),
        CODED,
        profile=NAV,
        values={"coded_sources": [source_file]},
    )
    owner = ids.build_sym("lio_pkg", "src/laserMapping.cpp", "publish_odometry")
    builder.node(owner)
    builder.unresolved(
        Unresolved(
            want=model.Want.FRAME_ID.value,
            owner=owner,
            expr="odom.child_frame_id",
            reason="not_stated_in_source",
        )
    )
    assert frame_rules.frame_no_publisher(builder.build()) == []


def test_no_publisher_counts_a_broadcaster_under_a_guard():
    """A guard rosio never evaluates cannot ground a claim that nothing publishes the frame."""
    builder = _listener_atlas(broadcast=True, rsp=True)
    builder.edge(
        model.EdgeKind.BROADCASTS_TF.value,
        ids.build_node("/fastlio_mapping"),
        ids.build_frame("radar_link"),
        key=ids.edge_key(sym=LIO_SYMBOL, guard=LIO_GUARD),
        layer=CODED,
        profile=NAV,
        values={"frame": "radar_link", "condition": LIO_GUARD},
        confidence=CERTAIN,
        evidence=[Source(file=ids.build_file("lio_pkg", "src/laserMapping.cpp"))],
    )
    assert frame_rules.frame_no_publisher(builder.build()) == []


def test_no_publisher_keeps_its_id_across_two_runs():
    first = _ids(frame_rules.frame_no_publisher(_listener_atlas(broadcast=True, rsp=True).build()))
    second = _ids(frame_rules.frame_no_publisher(_listener_atlas(broadcast=True, rsp=True).build()))
    assert first == second and len(first) == 1


# ---------------------------------------------------------------------------
# remap.unresolved


def _remap_atlas(bound: bool) -> AtlasBuilder:
    builder = _builder()
    launchent = ids.build_launchent(
        "nx_bringup", "launch/sensors.launch.py", ((("Node"), "scan_from_cloud"),)
    )
    builder.node(launchent)
    builder.fact(launchent, DECLARED, profile=NAV, values={"kind": "Node"}, confidence=CERTAIN)
    builder.unresolved(
        Unresolved(
            want=model.Want.OTHER.value,
            owner=launchent,
            expr="cloud_in:=/perception/obstacle_cloud",
            reason="the remap resolves to /perception/obstacle_cloud, and nothing states its kind",
            role="remap",
            evidence=[
                Source(
                    file=ids.build_file("nx_bringup", "launch/sensors.launch.py"),
                    locator="Node(scan_from_cloud)",
                )
            ],
        )
    )
    if bound:
        topic = ids.build_topic("/perception/obstacle_cloud")
        builder.node(topic)
        publisher = _node(builder, "/nx_perception", NAV, {"executable": "detector_node"})
        builder.edge(
            model.EdgeKind.PUBLISHES.value,
            publisher,
            topic,
            key=ids.edge_key(sym="sym:nx_perception|src/detector_node.cpp|onInit"),
            layer=CODED,
            profile=NAV,
            values={"qos_depth": 10},
            confidence=CERTAIN,
            evidence=[Source(file=ids.build_file("nx_perception", "src/detector_node.cpp"))],
        )
    return builder


def test_remap_unresolved_fires_once_per_site_with_the_raw_name():
    found = frame_rules.remap_unresolved(_remap_atlas(bound=False).build())
    assert len(found) == 1
    assert "cloud_in" in found[0].say and "/perception/obstacle_cloud" in found[0].say
    assert "sensors.launch.py" in found[0].say
    # Two remaps onto one destination connect two nodes, so "binds to nothing" was false.
    assert "binds to nothing" not in found[0].title
    assert found[0].title.startswith("nothing states what kind of endpoint the remap")


def test_remap_unresolved_never_asks_for_a_package_the_workspace_holds():
    """image_pipeline: the fix told the user to add depth_image_proc, which was in src."""
    builder = _remap_atlas(bound=False)
    launchent = next(item for item in builder.atlas.entities if item.startswith("launchent:"))
    builder.fact(launchent, DECLARED, profile=NAV, values={"pkg": "nx_perception"})
    assert "add the package" in frame_rules.remap_unresolved(builder.build())[0].fix
    builder.node(ids.build_pkg("nx_perception"))
    (found,) = frame_rules.remap_unresolved(builder.build())
    assert "add the package" not in found.fix
    assert found.fix.startswith("the source of nx_perception is in the workspace")


def test_remap_unresolved_is_silent_once_the_join_binds_the_destination():
    assert frame_rules.remap_unresolved(_remap_atlas(bound=True).build()) == []


def test_remap_unresolved_is_silent_when_a_wrapper_endpoint_binds_the_destination():
    builder = _remap_atlas(bound=False)
    topic = ids.build_topic("/perception/obstacle_cloud")
    builder.node(topic)
    publisher = _node(builder, "/nx_perception", NAV, {"executable": "detector_node"})
    builder.edge(
        model.EdgeKind.PUBLISHES.value,
        publisher,
        topic,
        key=ids.edge_key(sym="sym:nx_perception|src/detector_node.cpp|onInit"),
        layer=CODED,
        profile=NAV,
        values={"via_known_wrapper": "message_filters.Subscriber"},
        confidence=CERTAIN,
        evidence=[Source(file=ids.build_file("nx_perception", "src/detector_node.cpp"))],
    )
    assert frame_rules.remap_unresolved(builder.build()) == []


def _relative_atlas(namespace: str = "/") -> AtlasBuilder:
    """A remap whose destination is written the way the launch file writes it, relative."""
    builder = _builder()
    launchent = ids.build_launchent(
        "nx_bringup", "launch/nx_sim_launch.py", ((("Node"), "cloud_to_scan"),)
    )
    builder.node(launchent)
    builder.fact(launchent, DECLARED, profile=NAV, values={"kind": "Node"}, confidence=CERTAIN)
    node = _node(builder, f"{namespace.rstrip('/')}/cloud_to_scan", NAV, {"executable": "pc2scan"})
    builder.edge(
        model.EdgeKind.LAUNCHES.value,
        launchent,
        node,
        key=ids.edge_key(profile=NAV, launchent=launchent),
        layer=DECLARED,
        profile=NAV,
        confidence=CERTAIN,
    )
    builder.unresolved(
        Unresolved(
            want=model.Want.OTHER.value,
            owner=launchent,
            expr="scan:=scan",
            reason="nothing states the kind of the destination",
            role="remap",
            evidence=[
                Source(
                    file=ids.build_file("nx_bringup", "launch/nx_sim_launch.py"),
                    locator="Node(cloud_to_scan)",
                )
            ],
        )
    )
    return builder


def _sensor_topic(builder: AtlasBuilder, file_id: str, topic: str) -> None:
    """A gazebo sensor block of a robot model, sending its output onto a topic."""
    topic_id = ids.build_topic(topic)
    builder.node(topic_id)
    builder.edge(
        model.EdgeKind.REMAPS.value,
        file_id,
        topic_id,
        key=ids.edge_key(model=file_id, frm="~/out", to=topic),
        layer=DECLARED,
        values={"raw_name": "~/out"},
        confidence=CERTAIN,
        evidence=[Source(file=file_id, locator="gazebo/sensor[lidar]")],
    )


def test_remap_unresolved_fires_on_a_relative_destination_nothing_states():
    found = frame_rules.remap_unresolved(_relative_atlas().build())
    assert len(found) == 1
    assert "remaps scan to scan" in found[0].say


def test_remap_unresolved_resolves_a_relative_destination_before_it_compares():
    builder = _relative_atlas()
    topic = ids.build_topic("/scan")
    builder.node(topic)
    publisher = _node(builder, "/initial_nudge", NAV, {"executable": "initial_nudge.py"})
    builder.edge(
        model.EdgeKind.PUBLISHES.value,
        publisher,
        topic,
        key=ids.edge_key(sym="sym:nx_bringup|scripts/initial_nudge.py|main"),
        layer=CODED,
        profile=NAV,
        values={"raw_name": "/scan"},
        confidence=CERTAIN,
        evidence=[Source(file=ids.build_file("nx_bringup", "scripts/initial_nudge.py"))],
    )
    assert frame_rules.remap_unresolved(builder.build()) == []


def test_remap_unresolved_is_silent_when_the_model_the_profile_loads_states_the_topic():
    builder = _relative_atlas()
    files = _fleet(
        builder,
        {"fx_robot_sim.urdf": ["base_link", "os_sensor", "imu_link"]},
        {"fx_robot_sim.urdf": NAV},
    )
    _sensor_topic(builder, files["fx_robot_sim.urdf"], "/scan")
    assert frame_rules.remap_unresolved(builder.build()) == []


def test_remap_unresolved_still_fires_where_no_profile_loads_the_model_that_states_it():
    builder = _relative_atlas()
    files = _fleet(
        builder,
        {"fx_robot_sim.urdf": ["base_link", "os_sensor", "imu_link"]},
        {"fx_robot_sim.urdf": SIM},
    )
    _sensor_topic(builder, files["fx_robot_sim.urdf"], "/scan")
    found = frame_rules.remap_unresolved(builder.build())
    assert len(found) == 1
    assert found[0].profiles == [NAV]


def test_remap_unresolved_names_the_file_that_holds_the_remap():
    builder = _relative_atlas()
    builder.unresolved(
        Unresolved(
            want=model.Want.OTHER.value,
            owner=ids.build_launchent(
                "nx_bringup",
                "launch/nx_sim_launch.py",
                ((("Include"), "nx_navigation.launch.py"), (("Node"), "behavior_server")),
            ),
            expr="cmd_vel:=cmd_vel_raw",
            reason="nothing states the kind of the destination",
            role="remap",
            evidence=[
                Source(
                    file=ids.build_file("nx_bringup", "launch/nx_navigation.launch.py"),
                    locator="Include(nx_navigation.launch.py)/Node(behavior_server)",
                )
            ],
        )
    )
    builder.node(
        ids.build_launchent(
            "nx_bringup",
            "launch/nx_sim_launch.py",
            ((("Include"), "nx_navigation.launch.py"), (("Node"), "behavior_server")),
        )
    )
    found = [
        item for item in frame_rules.remap_unresolved(builder.build()) if "cmd_vel_raw" in item.say
    ]
    assert len(found) == 1
    assert "nx_navigation.launch.py" in found[0].say
    assert "nx_sim_launch.py" not in found[0].say


def test_remap_unresolved_mints_no_topic_and_keeps_its_id():
    atlas = _remap_atlas(bound=False).build()
    first = _ids(frame_rules.remap_unresolved(atlas))
    second = _ids(frame_rules.remap_unresolved(_remap_atlas(bound=False).build()))
    assert first == second
    assert ids.build_topic("/perception/obstacle_cloud") not in atlas.entities


def _outside_remap_atlas(written_in: str) -> AtlasBuilder:
    """turtlebot3 starts nav2 through nav2_bringup, so the remap sits in a file no root holds
    and the blind spot cites the include line of the entry point."""
    builder = _builder()
    entry = ids.build_file("nx_bringup", "launch/navigation2.launch.py")
    builder.node(entry)
    owner = ids.build_launchent(
        "nx_bringup",
        "launch/navigation2.launch.py",
        (("Include", "navigation_launch.py"), ("ComposableNode", "controller_server")),
    )
    builder.node(owner)
    builder.unresolved(
        Unresolved(
            want=model.Want.OTHER.value,
            owner=owner,
            expr="cmd_vel:=cmd_vel_nav",
            reason="nothing states the kind of the destination",
            role="remap",
            evidence=[
                Source(
                    file=entry,
                    locator="Include(navigation_launch.py)/ComposableNode(controller_server)",
                )
            ],
            written_in=written_in,
        )
    )
    return builder


def test_remap_unresolved_names_the_outside_file_that_writes_the_remap_and_the_include():
    stated = "@outside/share/nav2_bringup/launch/navigation_launch.py"
    found = frame_rules.remap_unresolved(_outside_remap_atlas(stated).build())
    assert len(found) == 1
    assert found[0].say == (
        "nav2_bringup/launch/navigation_launch.py remaps cmd_vel to cmd_vel_nav, included from "
        "navigation2.launch.py, and nothing in the workspace states what kind of endpoint "
        "cmd_vel_nav is, so the connection is unverified."
    )
    assert "navigation2.launch.py includes it" in found[0].detail
    assert found[0].fix.endswith("so change it at the include in navigation2.launch.py")
    assert found[0].subject == ids.build_file("nx_bringup", "launch/navigation2.launch.py")
    away = frame_rules.remap_unresolved(_outside_remap_atlas("@outside/robot_nav.py").build())
    assert away[0].say.startswith("@outside/robot_nav.py remaps cmd_vel to cmd_vel_nav")


def test_remap_unresolved_names_the_entry_file_when_a_root_holds_the_remap():
    found = frame_rules.remap_unresolved(_outside_remap_atlas("").build())
    assert found[0].say.startswith("navigation2.launch.py remaps cmd_vel to cmd_vel_nav and")
    assert "included from" not in found[0].say and "outside" not in found[0].fix


def _inline_remap(builder: AtlasBuilder) -> AtlasBuilder:
    """The entry point itself starts a node with the same remap, on a line it holds."""
    entry = ids.build_file("nx_bringup", "launch/navigation2.launch.py")
    owner = ids.build_launchent(
        "nx_bringup", "launch/navigation2.launch.py", (("Node", "cmd_relay"),)
    )
    builder.node(entry)
    builder.node(owner)
    builder.unresolved(
        Unresolved(
            want=model.Want.OTHER.value,
            owner=owner,
            expr="cmd_vel:=cmd_vel_nav",
            reason="nothing states the kind of the destination",
            role="remap",
            evidence=[Source(file=entry, locator="Node(cmd_relay)", line_hint=11)],
        )
    )
    return builder


def test_remap_unresolved_names_each_writer_of_one_remap_for_its_own_nodes():
    stated = "@outside/share/nav2_bringup/launch/navigation_launch.py"
    found = frame_rules.remap_unresolved(_inline_remap(_outside_remap_atlas(stated)).build())
    assert len(found) == 1
    finding = found[0]
    assert finding.say == (
        "navigation2.launch.py remaps cmd_vel to cmd_vel_nav, and so does "
        "nav2_bringup/launch/navigation_launch.py, which it includes, and nothing in the "
        "workspace states what kind of endpoint cmd_vel_nav is, so the connection is "
        "unverified."
    )
    assert finding.detail.startswith(
        "navigation2.launch.py remaps cmd_vel to cmd_vel_nav on Node(cmd_relay). "
        "nav2_bringup/launch/navigation_launch.py remaps cmd_vel to cmd_vel_nav on "
        "ComposableNode(controller_server). navigation2.launch.py includes it,"
    )
    assert finding.detail.endswith(
        "so nothing states whether cmd_vel_nav is a topic, a "
        "service or an action, and the endpoint stays unbound."
    )
    assert finding.fix.endswith(
        "; navigation2.launch.py writes the rule for Node(cmd_relay) itself, and "
        "nav2_bringup/launch/navigation_launch.py writes it for ComposableNode(controller_server) "
        "outside every root, so change that one at the include in navigation2.launch.py"
    )
    alone = frame_rules.remap_unresolved(_inline_remap(_builder()).build())
    assert _ids(found) == _ids(alone)
    assert alone[0].say.startswith("navigation2.launch.py remaps cmd_vel to cmd_vel_nav and")


def test_remap_unresolved_keeps_its_id_when_the_outside_file_is_named():
    stated = "@outside/share/nav2_bringup/launch/navigation_launch.py"
    named = _ids(frame_rules.remap_unresolved(_outside_remap_atlas(stated).build()))
    again = _ids(frame_rules.remap_unresolved(_outside_remap_atlas(stated).build()))
    unnamed = _ids(frame_rules.remap_unresolved(_outside_remap_atlas("").build()))
    assert named == again == unnamed and len(named) == 1


def _shared_remap_atlas(sim_gate: str = "true") -> AtlasBuilder:
    """One remap line in an included file, reached from two entry points in two profiles."""
    builder = _builder()
    held = ids.build_file("nx_bringup", "launch/nx_navigation.launch.py")
    builder.node(held)
    for profile, entry, gate in (
        (NAV, "launch/nx_nav_launch.py", "true"),
        (SIM, "launch/nx_sim_launch.py", sim_gate),
    ):
        owner = ids.build_launchent(
            "nx_bringup",
            entry,
            (("Include", "nx_navigation.launch.py"), ("Node", "controller_server")),
        )
        builder.node(owner)
        builder.fact(
            owner,
            DECLARED,
            profile=profile,
            values={
                "kind": "Node",
                "gates": [{"mechanism": "IfCondition", "expression": "x", "outcome": gate}],
            },
            confidence=CERTAIN,
        )
        builder.unresolved(
            Unresolved(
                want=model.Want.OTHER.value,
                owner=owner,
                expr="cmd_vel:=cmd_vel_nav",
                reason="nothing states the kind of the destination",
                role="remap",
                evidence=[
                    Source(
                        file=held,
                        locator=f"Include({entry.rsplit('/', 1)[-1]})/Node(controller_server)",
                    )
                ],
            )
        )
    return builder


def test_remap_unresolved_fires_once_for_one_line_read_through_two_entry_points():
    found = frame_rules.remap_unresolved(_shared_remap_atlas().build())
    assert len(found) == 1
    assert found[0].subject == ids.build_file("nx_bringup", "launch/nx_navigation.launch.py")
    assert found[0].profiles == sorted([NAV, SIM])
    assert len(found[0].where) == 2
    assert "nx_navigation.launch.py remaps cmd_vel to cmd_vel_nav" in found[0].say


def test_remap_unresolved_is_silent_in_a_profile_whose_condition_is_false():
    found = frame_rules.remap_unresolved(_shared_remap_atlas(sim_gate="false").build())
    assert len(found) == 1 and found[0].profiles == [NAV]
    builder = _relative_atlas()
    owner = ids.build_launchent(
        "nx_bringup", "launch/nx_sim_launch.py", (("Node", "cloud_to_scan"),)
    )
    builder.fact(
        owner,
        DECLARED,
        profile=NAV,
        values={"gates": [{"mechanism": "IfCondition", "expression": "x", "outcome": "false"}]},
        confidence=CERTAIN,
    )
    assert frame_rules.remap_unresolved(builder.build()) == []


def test_remap_unresolved_keys_one_line_on_its_file_across_two_runs():
    first = _ids(frame_rules.remap_unresolved(_shared_remap_atlas().build()))
    second = _ids(frame_rules.remap_unresolved(_shared_remap_atlas().build()))
    assert first == second and len(first) == 1
    assert "launchent" not in first[0]


# ---------------------------------------------------------------------------
# lifecycle.unmanaged


def _lifecycle_atlas(managed: bool, listed: str = "os_driver") -> AtlasBuilder:
    builder = _builder()
    manager = _node(builder, "/lifecycle_manager_sensors", NAV, {"executable": "lifecycle_manager"})
    driver = _node(
        builder, "/ouster/os_driver", NAV, {"executable": "os_driver", "form": "lifecycle_node"}
    )
    _param(builder, manager, "node_names", [listed], NAV, value_type="list")
    if managed:
        builder.edge(
            model.EdgeKind.MANAGES_LIFECYCLE.value,
            manager,
            driver,
            key=ids.edge_key(profile=NAV, raw="ouster/os_driver"),
            layer=DECLARED,
            profile=NAV,
            values={"mechanism": "node_names", "raw_name": "ouster/os_driver"},
            confidence=CERTAIN,
            evidence=[Source(file=ids.build_file("nx_bringup", "launch/sensors.launch.py"))],
        )
    return builder


def test_lifecycle_unmanaged_fires_on_a_lifecycle_node_with_no_manager():
    found = frame_rules.lifecycle_unmanaged(_lifecycle_atlas(managed=False).build())
    subjects = [item.subject for item in found]
    assert "node:/ouster/os_driver" in subjects
    assert all(item.severity == rules.severity_of("lifecycle.unmanaged") for item in found)


def test_lifecycle_unmanaged_fires_when_node_names_lists_a_node_the_profile_never_starts():
    found = frame_rules.lifecycle_unmanaged(_lifecycle_atlas(managed=True, listed="ghost").build())
    assert [item.subject for item in found] == ["param:/lifecycle_manager_sensors|node_names"]
    assert "ghost" in found[0].say


def _stopped_walk(builder: AtlasBuilder) -> None:
    launchfile = ids.build_launchfile("nx_bringup", "launch/sensors.launch.py")
    builder.fact(launchfile, DECLARED, profile=NAV, values={"raised": True}, confidence=CERTAIN)


def test_lifecycle_unmanaged_is_silent_in_a_profile_whose_walk_stopped():
    """nav2's simple commander examples stop on a simulator world file after the lifecycle
    managers were read and before any server was, which read as 22 managers waiting forever."""
    builder = _lifecycle_atlas(managed=False, listed="ghost")
    _stopped_walk(builder)
    atlas = builder.build()
    assert frame_rules.lifecycle_unmanaged(atlas) == []
    (row,) = frame_rules.silences(atlas)
    assert row["rule"] == "lifecycle.unmanaged" and row["profile"] == NAV
    assert "nx_bringup/launch/sensors.launch.py" in row["reason"]


def test_lifecycle_unmanaged_names_the_profile_and_counts_in_plural():
    builder = _manager_atlas(ids.build_node("/lifecycle_manager_navigation"), ["a", "b"])
    (found,) = frame_rules.lifecycle_unmanaged(builder.build())
    assert found.title == "/lifecycle_manager_navigation waits for 2 nodes it never gets"
    assert f"and profile {ids.parse_profile(NAV)} starts no such node" in found.say
    assert frame_rules.silences(builder.build()) == []


def test_lifecycle_unmanaged_is_silent_when_the_wiring_closes():
    atlas = _lifecycle_atlas(managed=True, listed="ouster/os_driver").build()
    assert frame_rules.lifecycle_unmanaged(atlas) == []


def test_lifecycle_unmanaged_is_a_note_when_unread_code_may_drive_the_node():
    """The lifecycle demo starts a lifecycle talker beside a small client that drives it
    through its services. With no manager in the profile and the client's code unread, the
    node is not provably left alone, so the rule says so at info and names the client."""
    builder = _builder()
    _node(builder, "/lc_talker", NAV, {"executable": "lifecycle_talker", "form": "lifecycle_node"})
    _node(builder, "/lifecycle_service_client", NAV, {"executable": "lifecycle_service_client"})
    found = frame_rules.lifecycle_unmanaged(builder.build())
    assert [item.subject for item in found] == ["node:/lc_talker"]
    assert found[0].severity == model.Severity.INFO.value
    assert "/lifecycle_service_client" in found[0].say


def _manager_atlas(owner: str, listed: list[str]) -> AtlasBuilder:
    builder = _builder()
    builder.node(owner, name_source=model.NameSource.RUNTIME_RANDOMISED.value)
    builder.fact(
        owner,
        DECLARED,
        profile=NAV,
        values={"executable": "lifecycle_manager"},
        confidence=CERTAIN,
        evidence=[Source(file=ids.build_file("nx_bringup", "launch/x.launch.py"))],
    )
    _param(builder, owner, "node_names", listed, NAV, value_type="list")
    return builder


def test_lifecycle_unmanaged_reads_a_synthetic_manager_identity_through_the_id_grammar():
    owner = ids.build_node_synthetic("nav2_lifecycle_manager", "lifecycle_manager", "/", "abcd1234")
    found = frame_rules.lifecycle_unmanaged(_manager_atlas(owner, ["controller_server"]).build())
    assert [item.subject for item in found] == [ids.build_param(owner, "node_names")]
    assert "controller_server" in found[0].say


def test_lifecycle_unmanaged_is_silent_on_a_listed_name_the_grammar_cannot_resolve():
    owner = ids.build_node("/lifecycle_manager_navigation")
    atlas = _manager_atlas(owner, ["$(var node)", "", "a node~name"]).build()
    assert frame_rules.lifecycle_unmanaged(atlas) == []


def test_lifecycle_unmanaged_keeps_its_id_across_two_runs():
    first = _ids(frame_rules.lifecycle_unmanaged(_lifecycle_atlas(managed=False).build()))
    second = _ids(frame_rules.lifecycle_unmanaged(_lifecycle_atlas(managed=False).build()))
    assert first == second and first


# ---------------------------------------------------------------------------
# build.stale


def _stale_atlas(differs: bool, shared: bool = True) -> AtlasBuilder:
    builder = _builder()
    file_id = ids.build_file("nx_bringup", "config/nav2_params.yaml")
    attrs = {"via_package_share": True} if shared else {}
    if differs:
        attrs["install_copy_differs"] = True
    builder.node(file_id, attrs=attrs)
    builder.fact(
        file_id,
        DECLARED,
        profile=NAV,
        values={"read": True},
        confidence=CERTAIN,
        evidence=[Source(file=file_id, root=model.Root.INSTALL.value)],
    )
    return builder


def test_build_stale_fires_once_per_file_whose_two_copies_differ():
    found = build_rules.build_stale(_stale_atlas(differs=True).build())
    assert [item.subject for item in found] == ["file:nx_bringup|config/nav2_params.yaml"]
    assert "shows the installed copy, which is what runs" in found[0].say
    assert found[0].where[0].root == model.Root.INSTALL.value


def test_build_stale_says_a_python_module_was_read_from_src_and_does_not_run_yet():
    """awk_ws: the default in safety_stop.py was edited in src and only the yaml was reported."""
    (found,) = build_rules.build_stale(_stale_atlas(differs=True, shared=False).build())
    assert "shows the src copy, and the installed one is what runs" in found.say
    assert found.where[0].root == model.Root.SRC.value


def test_build_stale_is_silent_when_the_install_copy_still_matches():
    assert build_rules.build_stale(_stale_atlas(differs=False).build()) == []


def test_build_stale_keeps_its_id_across_two_runs():
    first = _ids(build_rules.build_stale(_stale_atlas(differs=True).build()))
    second = _ids(build_rules.build_stale(_stale_atlas(differs=True).build()))
    assert first == second and len(first) == 1


# ---------------------------------------------------------------------------
# launch.unloadable and launch.raised


def _refusal_atlas(rows: list[dict]) -> AtlasBuilder:
    builder = _builder()
    for relpath in ("launch/rover.launch", "launch/nx_navigation.launch.py"):
        package = "gnss_driver" if relpath.endswith(".launch") else "nx_bringup"
        builder.node(ids.build_launchfile(package, relpath), attrs={"loadable": False})
    builder.add_run(
        DECLARED,
        profile=NAV,
        input_digest="deadbeef",
        exceptions=rows,
    )
    return builder


_UNLOADABLE_ROW = {
    "path": "gnss_driver/launch/rover.launch",
    "phase": "import",
    "exc_type": "Unloadable",
    "message": "the file is not a ROS 2 loadable launch file",
    "note": "launch.unloadable",
}
_RAISED_ROW = {
    "path": "nx_bringup/launch/nx_navigation.launch.py",
    "phase": "argument",
    "exc_type": "RuntimeError",
    "message": "required launch argument 'params_file' was not provided",
    "note": "launch.raised",
}


def test_launch_unloadable_fires_on_a_recorded_refusal():
    found = build_rules.launch_unloadable(_refusal_atlas([_UNLOADABLE_ROW]).build())
    assert [item.subject for item in found] == ["launchfile:gnss_driver|launch/rover.launch"]
    assert found[0].discriminator == ""
    assert found[0].profiles == [NAV]


def test_launch_unloadable_is_silent_on_a_run_that_recorded_no_refusal():
    assert build_rules.launch_unloadable(_refusal_atlas([]).build()) == []
    assert build_rules.launch_unloadable(_refusal_atlas([_RAISED_ROW]).build()) == []


def test_launch_raised_fires_and_carries_the_phase_in_its_discriminator():
    found = build_rules.launch_raised(_refusal_atlas([_RAISED_ROW]).build())
    assert len(found) == 1
    assert found[0].severity == rules.severity_of("launch.raised")
    assert found[0].discriminator == "h" + ids.digest(
        "argument", "RuntimeError", "required launch argument 'params_file' was not provided"
    )


def test_launch_raised_points_at_the_sandbox_when_the_sandbox_refused_the_access():
    from rosio.resolve import launch_exec

    message = f"OpaqueFunction(fn1): No such file or directory. {launch_exec.SANDBOX_REFUSED}"
    row = dict(_RAISED_ROW, phase="visit", exc_type="FileNotFoundError", message=message)
    (found,) = build_rules.launch_raised(_refusal_atlas([row]).build())
    assert "--launch-sandbox off" in found.fix
    assert "inside the launch sandbox" in found.say
    assert ".." not in found.detail


def test_launch_raised_keeps_the_argument_fix_when_no_sandbox_was_involved():
    (found,) = build_rules.launch_raised(_refusal_atlas([_RAISED_ROW]).build())
    assert "state params_file under args of the entry point of profile nx_navigation" in found.fix
    assert "sandbox" not in found.say


def test_launch_raised_names_the_node_the_walk_kept_and_does_not_call_it_missing():
    row = dict(
        _RAISED_ROW,
        phase="visit",
        exc_type="PackageNotFoundError",
        message="Node: \"package 'absent_description' not found\"",
        kept="Node(rviz2)",
    )
    (found,) = build_rules.launch_raised(_refusal_atlas([row]).build())
    assert "Node(rviz2)" in found.say
    assert "missing from this atlas" not in found.say
    assert "stopped" not in found.detail
    assert found.discriminator == "h" + ids.digest(
        "visit", "PackageNotFoundError", row["message"], "Node(rviz2)"
    )
    (again,) = build_rules.launch_raised(_refusal_atlas([row]).build())
    assert again.id == found.id


def test_launch_raised_without_a_kept_action_keeps_its_wording_and_id():
    row = dict(_RAISED_ROW, phase="visit", message="IncludeLaunchDescription: boom")
    (found,) = build_rules.launch_raised(_refusal_atlas([row]).build())
    assert "missing from this atlas" in found.say
    assert found.discriminator == "h" + ids.digest(
        "visit", "RuntimeError", "IncludeLaunchDescription: boom"
    )


def test_launch_raised_names_a_missing_module_and_its_apt_package():
    """The row held `No module named 'controller_manager'` and the fix said to supply an
    argument."""
    row = dict(
        _RAISED_ROW,
        phase="import",
        exc_type="ModuleNotFoundError",
        message="No module named 'controller_manager', searched: ROS jazzy",
    )
    (found,) = build_rules.launch_raised(_refusal_atlas([row]).build())
    assert "sudo apt install ros-jazzy-controller-manager" in found.fix
    assert "python module controller_manager" in found.say
    (again,) = build_rules.launch_raised(_refusal_atlas([row]).build())
    assert again.id == found.id


def test_launch_raised_names_a_module_no_ros_package_ships_without_apt():
    row = dict(
        _RAISED_ROW,
        phase="import",
        exc_type="ModuleNotFoundError",
        message="No module named 'acme_private_helpers', searched: ROS jazzy",
    )
    (found,) = build_rules.launch_raised(_refusal_atlas([row]).build())
    assert "apt" not in found.fix
    assert "python module acme_private_helpers" in found.fix


def test_a_missing_module_names_the_release_that_was_searched():
    from rosio.collect import launch_declared

    message = "No module named 'controller_manager'"
    assert (
        launch_declared.relabel_module(message, ["/opt/ros/jazzy/setup.bash"])
        == f"{message}, searched: ROS jazzy"
    )
    assert launch_declared.relabel_module(message, ["/home/me/ws/install/setup.bash"]) == message
    assert launch_declared.missing_module(f"{message}, searched: ROS jazzy") == (
        "controller_manager",
        "jazzy",
    )


def test_launch_raised_is_silent_when_the_file_is_not_in_the_atlas():
    builder = _builder()
    builder.add_run(DECLARED, profile=NAV, input_digest="deadbeef", exceptions=[_RAISED_ROW])
    assert build_rules.launch_raised(builder.build()) == []


def test_launch_rules_keep_their_ids_across_two_runs():
    rows = [_UNLOADABLE_ROW, _RAISED_ROW]
    first = _ids(build_rules.collect(_refusal_atlas(rows).build()))
    second = _ids(build_rules.collect(_refusal_atlas(rows).build()))
    assert first == second and len(first) == 2


def test_two_profiles_hitting_one_refusal_make_one_finding():
    builder = _refusal_atlas([_UNLOADABLE_ROW])
    builder.add_run(DECLARED, profile=SIM, input_digest="feedface", exceptions=[_UNLOADABLE_ROW])
    found = build_rules.launch_unloadable(builder.build())
    assert len(found) == 1
    assert found[0].profiles == sorted([SIM, NAV])


# ---------------------------------------------------------------------------
# resolution.below_budget


def _budget_atlas(unresolved: int) -> AtlasBuilder:
    builder = _builder()
    owner = _node(builder, "/self_filter", NAV, {"executable": "self_filter"})
    builder.count_resolved(model.Want.TOPIC_NAME.value, 9)
    for index in range(unresolved):
        builder.unresolved(
            Unresolved(
                want=model.Want.TOPIC_NAME.value,
                owner=owner,
                expr=f"topic_from_variable_{index}",
                reason="not_stated_in_source",
                evidence=[Source(file=ids.build_file("self_filter", "src/node.cpp"))],
            )
        )
    return builder


def test_below_budget_fires_and_names_the_rate_the_budget_and_the_profile():
    atlas = _budget_atlas(unresolved=4).build()
    found = build_rules.resolution_below_budget(atlas)
    assert [item.subject for item in found] == ["profile:nx_mapping"], "the first profile"
    assert found[0].profiles == [NAV]
    assert "0.692" in found[0].say and "0.950" in found[0].say
    # The default budget only informs: a first run is not red for blind spots no line of
    # the user's code can fix. A budget the configuration states makes it a warning.
    assert found[0].severity == "info"
    (stated,) = build_rules.resolution_below_budget(atlas, budget=0.95)
    assert stated.severity == rules.severity_of("resolution.below_budget") == "warn"
    assert stated.id == found[0].id


def test_below_budget_counts_one_unresolved_site_in_the_singular():
    (found,) = build_rules.resolution_below_budget(_budget_atlas(unresolved=1).build(), 0.95)
    assert "with 1 unresolved site," in found.detail
    (found,) = build_rules.resolution_below_budget(_budget_atlas(unresolved=4).build())
    assert "with 4 unresolved sites," in found.detail


def test_below_budget_is_silent_when_the_rate_clears_the_budget():
    assert build_rules.resolution_below_budget(_budget_atlas(unresolved=0).build()) == []


def test_below_budget_reads_the_budget_from_the_atlas_and_from_the_argument():
    atlas = _budget_atlas(unresolved=1).build()
    assert build_rules.resolution_below_budget(atlas)
    assert build_rules.resolution_below_budget(atlas, budget=0.85) == []
    atlas.x["drift"] = {build_rules.BUDGET_KEY: 0.85}
    assert build_rules.resolution_below_budget(atlas) == []
    assert build_rules.resolution_below_budget(atlas, budget=0.95)
    with pytest.raises(ContractError):
        build_rules.resolution_below_budget(atlas, budget=7.0)


def test_below_budget_fires_when_no_profile_owns_the_blind_spots():
    """A workspace without profiles still has a gate, so a failed budget is never silent."""
    builder = AtlasBuilder()
    owner = ids.build_file("acme", "acme/worker.py")
    builder.node(owner)
    builder.unresolved(
        Unresolved(
            want=model.Want.TOPIC_NAME.value,
            owner=owner,
            expr="name_from_variable",
            reason="not_stated_in_source",
            evidence=[Source(file=owner)],
        )
    )
    atlas = builder.build()
    found = build_rules.resolution_below_budget(atlas)
    assert len(found) == 1
    assert found[0].subject == owner
    assert found[0].profiles == []
    assert "0.000" in found[0].say


def test_below_budget_keeps_its_id_when_a_profile_is_added():
    """The subject was the first profile, so a profile whose slug sorts first moved the id
    and un waived the finding. The workspace entity extraction always writes does not move."""
    builder = _budget_atlas(unresolved=4)
    builder.node(ids.build_pkg(ids.REPO_PKG))
    before = _ids(build_rules.resolution_below_budget(builder.build()))
    builder.node(ids.build_profile("aaa_first"))
    after = _ids(build_rules.resolution_below_budget(builder.build()))
    assert before == after and len(before) == 1
    assert before[0].startswith("find:resolution.below_budget|pkg:@repo|")


def test_below_budget_keeps_its_id_across_two_runs():
    first = _ids(build_rules.resolution_below_budget(_budget_atlas(unresolved=4).build()))
    second = _ids(build_rules.resolution_below_budget(_budget_atlas(unresolved=4).build()))
    assert first == second and len(first) == 1


# ---------------------------------------------------------------------------
# extract.pass_incomplete


CPP_REASON = "CollectorError: tree-sitter is missing"
CPP_MISSING = "the endpoints, parameter defaults, frames and rates the C++ sources state"


def _pass_atlas(*rows: tuple[str, str, str]) -> AtlasBuilder:
    builder = _builder()
    run = builder.add_run(CODED, profile=NAV, input_digest="c0ffee01")
    for collector, state, reason in rows:
        builder.record_pass(
            run,
            collector,
            state=state,
            reason=reason,
            missing=CPP_MISSING if reason else "",
        )
    return builder


def test_pass_incomplete_fires_and_names_the_collector_the_reason_and_what_is_missing():
    atlas = _pass_atlas(("cpp_code", "refused", CPP_REASON)).build()
    found = build_rules.extract_pass_incomplete(atlas)
    assert len(found) == 1
    assert found[0].severity == "error"
    assert found[0].subject.startswith("run:")
    assert "cpp_code" in found[0].say
    assert "tree-sitter is missing" in found[0].say
    assert CPP_MISSING in found[0].say
    assert found[0].profiles == [NAV] and found[0].layers == [CODED]
    assert atlas.counters["passes"] == {
        "attempted": 1,
        "completed": 0,
        "refused": 1,
        "not_attempted": 0,
        "refused_collectors": ["cpp_code"],
        "complete": False,
    }


def test_pass_incomplete_is_silent_when_a_collector_had_nothing_to_read():
    """The near miss that would make this rule noise on every small repository: a workspace
    with no C++ at all. A collector that read an empty tree is complete, and one that was
    never attempted says so without claiming a loss."""
    atlas = _pass_atlas(
        ("cpp_code", "not_attempted", "no C++ source is under any root"),
        ("py_code", "completed", ""),
    ).build()
    assert build_rules.extract_pass_incomplete(atlas) == []
    assert atlas.counters["passes"]["complete"] is True
    assert atlas.counters["passes"]["not_attempted"] == 1


def test_pass_incomplete_keeps_its_id_across_two_runs():
    rows = (("cpp_code", "refused", CPP_REASON), ("urdf", "refused", "ValueError: inf"))
    first = _ids(build_rules.extract_pass_incomplete(_pass_atlas(*rows).build()))
    second = _ids(build_rules.extract_pass_incomplete(_pass_atlas(*rows).build()))
    assert first == second and len(first) == 2


def test_pass_incomplete_says_nothing_about_an_atlas_that_records_no_pass():
    """Every atlas written before passes were recorded, which must not read as a loss."""
    atlas = _builder().build()
    assert build_rules.extract_pass_incomplete(atlas) == []
    assert atlas.counters["passes"] == {
        "attempted": 0,
        "completed": 0,
        "refused": 0,
        "not_attempted": 0,
        "refused_collectors": [],
        "complete": True,
    }


# ---------------------------------------------------------------------------
# contract


def test_every_rule_id_is_committed_and_implemented_once():
    committed = {
        "frame.no_publisher",
        "frame.orphan_root",
        "frame.not_in_urdf",
        "frame.parent_differs",
        "remap.unresolved",
        "lifecycle.unmanaged",
        "build.stale",
        "launch.unloadable",
        "launch.raised",
        "resolution.below_budget",
        "extract.pass_incomplete",
        "extract.input_refused",
        "launch.file_not_installed",
        "model.mesh_missing",
        "build.unbuilt",
        "build.underlay",
    }
    assert set(frame_rules.RULES) | set(build_rules.RULES) == committed
    assert not set(frame_rules.RULES) & set(build_rules.RULES)
    assert committed <= set(rules.COMMITTED)
    for rule in committed:
        assert rules.canonical(rule) == rule
        assert rules.severity_of(rule) in {"error", "warn", "info"}


def test_a_rule_never_mutates_the_atlas_and_repeats_itself():
    builder = _two_models(same_origin=False)
    _node(
        builder,
        "/ouster/os_driver",
        NAV,
        {"executable": "os_driver", "form": "lifecycle_node"},
    )
    atlas = builder.build()
    before = atlas.canonical_bytes()
    first = frame_rules.collect(atlas) + build_rules.collect(atlas)
    second = frame_rules.collect(atlas) + build_rules.collect(atlas)
    assert atlas.canonical_bytes() == before
    assert [item.id for item in first] == [item.id for item in second]
    assert [item.fingerprint for item in first] == [item.fingerprint for item in second]


def test_no_finding_carries_a_synthetic_id_in_its_prose():
    builder = _two_models(same_origin=False)
    node_id = ids.build_node_synthetic("nx_bridge", "io_bridge", "/", "abcd1234")
    builder.node(node_id, name_source=model.NameSource.UNKNOWN.value)
    builder.fact(node_id, DECLARED, profile=NAV, values={"form": "lifecycle_node"})
    atlas = builder.build()
    sigil = ids.KIND_SEP + ids.SYNTHETIC
    for finding in frame_rules.collect(atlas) + build_rules.collect(atlas):
        assert sigil not in finding.say
        assert sigil not in finding.title


# ---------------------------------------------------------------------------
# what the unloadable sentence says, and what a rule may conclude without the launch tree


def test_the_unloadable_sentence_follows_the_recorded_reason():
    from rosio.collect import launch_declared

    markup = dict(_UNLOADABLE_ROW)
    markup["refused_because"] = launch_declared.REFUSED_MARKUP
    found = build_rules.launch_unloadable(_refusal_atlas([markup]).build())
    assert "ROS 1" not in found[0].say
    assert "XML or YAML ROS 2 launch file" in found[0].say

    ros1 = dict(_UNLOADABLE_ROW)
    ros1["refused_because"] = launch_declared.REFUSED_ROS1
    found = build_rules.launch_unloadable(_refusal_atlas([ros1]).build())
    assert "ROS 1 style launch file" in found[0].say


def test_the_unloadable_detail_is_unchanged_by_the_reason():
    """The fingerprint is over the detail, so a stored waiver survives the new wording."""
    with_reason = dict(_UNLOADABLE_ROW, refused_because="ros1_xml")
    first = build_rules.launch_unloadable(_refusal_atlas([_UNLOADABLE_ROW]).build())
    second = build_rules.launch_unloadable(_refusal_atlas([with_reason]).build())
    assert [item.fingerprint for item in first] == [item.fingerprint for item in second]


def _plugin_builder(*, loadable: bool = True, include_blind: bool = False) -> AtlasBuilder:
    builder = _builder()
    manifest = ids.build_file("fx_plugins", "plugins.xml")
    builder.node(manifest)
    package = ids.build_pkg("fx_plugins")
    builder.node(package)
    launchfile = ids.build_launchfile("fx_bringup", "launch/bringup.launch.py")
    builder.node(launchfile, attrs={"loadable": loadable})
    plugin_id = ids.build_plugin("nav2_core::Controller", "fx_plugins/Spare")
    builder.fact(
        plugin_id,
        DECLARED,
        values={"base_class": "nav2_core::Controller", "lookup_name": "fx_plugins/Spare"},
        confidence=CERTAIN,
        evidence=[Source(file=manifest, locator="class[fx_plugins/Spare]")],
    )
    builder.edge(model.EdgeKind.PROVIDES_PLUGIN.value, package, plugin_id)
    selector = ids.build_param(ids.build_node("/controller_server"), "controller_plugins")
    profiles = [ids.build_profile(slug) for slug in ("nx_navigation", "nx_sim", "nx_mapping")]
    for profile in profiles:
        builder.fact(selector, DECLARED, profile=profile, values={"value": ["FollowPath"]})
    builder.set_param(
        selector,
        state=model.ParamState.EFFECTIVE.value,
        candidates=[
            model.Candidate(
                value=["FollowPath"],
                value_type="list",
                mechanism="yaml_base",
                source=Source(
                    file=ids.build_file("fx_bringup", "config/params.yaml"),
                    locator="controller_server.ros__parameters.controller_plugins",
                ),
                rank=0,
                wins=True,
                profile=profile,
            )
            for profile in profiles
        ],
    )
    if include_blind:
        builder.unresolved(
            Unresolved(
                want=model.Want.OTHER.value,
                owner=launchfile,
                expr="source(launch_dir, 'navigation.launch.py')",
                reason="the included launch file path did not resolve",
                role="include",
            )
        )
    return builder


def test_plugin_unused_fires_when_every_launch_file_was_read():
    from rosio.drift import plugins as plugin_rules

    atlas = _plugin_builder().build()
    assert plugin_rules.launch_tree_read(atlas) is True
    (found,) = plugin_rules.unused(atlas)
    # The key is named bare: "a operations list" and "a the selector list" read badly.
    assert found.say.endswith("Add it to controller_plugins or stop exporting it.")


def test_plugin_unused_is_silent_when_an_entry_point_never_loaded():
    """The selector list that would name the user arrives through the launch walk, so a
    launch tree that was not read cannot support the claim that nobody selects a class."""
    from rosio.drift import plugins as plugin_rules

    atlas = _plugin_builder(loadable=False).build()
    assert plugin_rules.launch_tree_read(atlas) is False
    assert plugin_rules.unused(atlas) == []


def test_plugin_unused_is_silent_when_an_include_collapsed():
    from rosio.drift import plugins as plugin_rules

    atlas = _plugin_builder(include_blind=True).build()
    assert plugin_rules.launch_tree_read(atlas) is False
    assert plugin_rules.unused(atlas) == []


def test_an_atlas_with_no_launch_file_at_all_is_not_treated_as_a_collapse():
    """Nothing was refused, so nothing is missing. A workspace whose profiles are composed
    without launch files keeps the rule."""
    from rosio.drift import plugins as plugin_rules

    builder = _plugin_builder()
    builder.atlas.entities.pop(ids.build_launchfile("fx_bringup", "launch/bringup.launch.py"))
    atlas = builder.build()
    assert plugin_rules.launch_tree_read(atlas) is True
    assert len(plugin_rules.unused(atlas)) == 1


def test_the_detail_is_the_message_the_run_recorded():
    from rosio.collect import launch_declared

    row = dict(
        _UNLOADABLE_ROW,
        message="ROS 1 XML launch file, refused by shape",
        refused_because=launch_declared.REFUSED_ROS1,
    )
    found = build_rules.launch_unloadable(_refusal_atlas([row]).build())
    assert found[0].detail.startswith("ROS 1 XML launch file, refused by shape.")


# ---------------------------------------------------------------------------
# extract.input_refused


def _refused_file_atlas(*, refused: bool) -> AtlasBuilder:
    builder = _builder()
    read = ids.build_file("nx_bringup", "urdf/fx_robot.urdf")
    builder.node(read, attrs={"loadable": True, "robot_name": "fx"})
    builder.fact(read, DECLARED, profile=NAV, values={"frames": 4}, confidence=CERTAIN)
    torn = ids.build_file("nx_bringup", "urdf/trailer.urdf")
    attrs = {"loadable": True}
    if refused:
        attrs = {
            "loadable": False,
            "refused_because": "urdf_unreadable",
            "refused_detail": "not well-formed (invalid token): line 3, column 8",
        }
    builder.node(torn, attrs=attrs)
    if refused:
        builder.unresolved(
            Unresolved(
                want=model.Want.FRAME_ID.value,
                owner=torn,
                expr="not well-formed (invalid token)",
                reason="urdf_unreadable",
                role="urdf",
            )
        )
    return builder


def test_extract_input_refused_names_the_file_the_reason_and_what_is_missing():
    found = build_rules.extract_input_refused(_refused_file_atlas(refused=True).build())
    assert [item.subject for item in found] == ["file:nx_bringup|urdf/trailer.urdf"]
    assert found[0].severity == "warn"
    assert "urdf_unreadable" in found[0].say
    assert "nx_bringup/urdf/trailer.urdf" in found[0].say
    assert build_rules.REFUSAL_COSTS["urdf_unreadable"] in found[0].say
    assert "not well-formed" in found[0].detail
    assert [row.file for row in found[0].where] == ["file:nx_bringup|urdf/trailer.urdf"]


def test_extract_input_refused_names_the_yaml_line_and_the_profile_that_loads_it():
    """awk_ws: teleop.yaml line 4 was over-indented, and the finding named no line, read like
    a permissions problem, and claimed every profile."""
    builder = _builder()
    teleop = ids.build_file("nx_bringup", "config/teleop.yaml")
    builder.node(
        teleop,
        attrs={
            "loadable": False,
            "refused_because": "params_file_unreadable",
            "refused_detail": "teleop.yaml is not readable as yaml: line 4, column 15: "
            "mapping values are not allowed here",
        },
    )
    builder.unresolved(
        Unresolved(
            want=model.Want.OTHER.value,
            owner=teleop,
            expr="teleop.yaml",
            reason="params_file_unreadable",
            role="parameters_file",
            evidence=[Source(file=teleop, line_hint=4)],
        )
    )
    site = ids.build_launchent("nx_bringup", "launch/teleop.launch.py", (("Node", "teleop"),))
    builder.node(site)
    builder.fact(
        site, DECLARED, profile=NAV, values={"file_reads": [{"file": teleop, "role": "params"}]}
    )
    (found,) = build_rules.extract_input_refused(builder.build())
    assert found.title == "nx_bringup/config/teleop.yaml:4 is not valid yaml"
    assert "line 4, column 15: mapping values are not allowed here" in found.say
    assert (
        found.fix == "correct the yaml at line 4, column 15 of nx_bringup/config/teleop.yaml, "
        "then extract again"
    )
    assert found.where[0].line_hint == 4
    assert found.profiles == [NAV]
    assert "Until this rule existed" not in found.detail


def test_extract_input_refused_is_silent_on_a_file_that_was_read():
    """The near miss that matters: every other file of the same package was read, and the
    completeness rule is silent here because the pass did finish."""
    atlas = _refused_file_atlas(refused=False).build()
    assert build_rules.extract_input_refused(atlas) == []
    assert build_rules.extract_pass_incomplete(atlas) == []


def test_extract_input_refused_keeps_its_id_across_two_runs():
    first = _ids(build_rules.extract_input_refused(_refused_file_atlas(refused=True).build()))
    second = _ids(build_rules.extract_input_refused(_refused_file_atlas(refused=True).build()))
    assert first == second and len(first) == 1


def test_extract_input_refused_says_an_exported_manifest_is_not_installed():
    """theta star exports a manifest it does not install, and the fix said to make the file
    readable."""
    builder = _builder()
    manifest = ids.build_file("nav2_theta_star_planner", "global_planner_plugin.xml")
    builder.node(
        manifest,
        attrs={"loadable": False, "refused_because": "plugin_manifest_missing", "exists": False},
    )
    (found,) = build_rules.extract_input_refused(builder.build())
    assert found.fix.startswith(
        "nav2_theta_star_planner/package.xml exports "
        "nav2_theta_star_planner/global_planner_plugin.xml, which the package does not install"
    )
    assert "readable" not in found.fix


def _unbuilt_atlas(because: str) -> AtlasBuilder:
    builder = _builder()
    package = ids.build_pkg("rcd_demo")
    builder.node(package)
    reader = ids.build_file("rcd_demo", "bringup/launch/demo.launch.py")
    builder.node(reader)
    for relpath in ("config/demo_controllers.yaml", "urdf/demo.urdf.xacro"):
        file_id = ids.build_file("rcd_demo", relpath)
        attrs = {"loadable": False, "refused_because": because, "refused_detail": because}
        if because == "file_after_build":
            attrs["found_elsewhere"] = [f"bringup/{relpath}"]
        builder.node(file_id, attrs=attrs)
        builder.edge(model.EdgeKind.CONTAINS.value, package, file_id)
        builder.unresolved(
            Unresolved(
                want=model.Want.OTHER.value,
                owner=file_id,
                expr=relpath.rsplit("/", 1)[1],
                reason=because,
                evidence=[Source(file=reader, locator="parameters", line_hint=30)],
            )
        )
    return builder


def test_build_unbuilt_reports_one_finding_with_where_the_source_holds_each_file():
    """Two dozen refusals for one missing `colcon build` buried the one action that repairs
    them all."""
    atlas = _unbuilt_atlas("file_after_build").build()
    (found,) = build_rules.build_unbuilt(atlas)
    assert found.subject == "pkg:rcd_demo"
    assert found.severity == "warn"
    assert "colcon build --packages-up-to rcd_demo" in found.fix
    assert (
        "rcd_demo: config/demo_controllers.yaml is in the source at "
        "bringup/config/demo_controllers.yaml" in found.detail
    )
    assert found.title.startswith("rcd_demo is not built, and 2 files read from a package share")
    assert [row.file for row in found.where] == ["file:rcd_demo|bringup/launch/demo.launch.py"]
    assert build_rules.extract_input_refused(atlas) == []
    again = build_rules.build_unbuilt(_unbuilt_atlas("file_after_build").build())
    assert [item.id for item in again] == [found.id]


def test_build_unbuilt_is_silent_on_a_file_no_tree_holds():
    atlas = _unbuilt_atlas("file_missing").build()
    assert build_rules.build_unbuilt(atlas) == []
    refused = build_rules.extract_input_refused(atlas)
    assert len(refused) == 2
    assert all(item.fix.startswith("correct the path that names") for item in refused)


def test_extract_input_refused_reads_a_recording_and_an_unknown_reason():
    builder = _builder()
    bag_id = ids.build_bag("2026_08_12-14_03_05_site")
    builder.node(
        bag_id,
        attrs={"loadable": False, "refused_because": "bag_unreadable", "refused_detail": "no db3"},
    )
    later = ids.build_file("nx_perception", "config/later.yaml")
    builder.node(later, attrs={"loadable": False, "refused_because": "a_reason_from_later"})
    found = build_rules.extract_input_refused(builder.build())
    assert [item.subject for item in found] == [bag_id, later]
    assert "2026_08_12-14_03_05_site was refused" in found[0].say
    assert found[0].where == []
    assert build_rules.UNKNOWN_COST in found[1].say


def test_extract_input_refused_cites_the_tree_the_file_was_read_from():
    """A refused model in an install tree cited as src sent an editor to a checkout that does
    not hold it. The blind spot the refusal left states the tree."""
    builder = _builder()
    torn = ids.build_file("nx_description", "urdf/create3.urdf.xacro")
    builder.node(torn, attrs={"loadable": False, "refused_because": "xacro_argument_missing"})
    builder.unresolved(
        Unresolved(
            want=model.Want.FRAME_ID.value,
            owner=torn,
            expr="namespace",
            reason="xacro_argument_missing",
            role="urdf",
            evidence=[Source(file=torn, root=model.Root.INSTALL.value)],
        )
    )
    found = build_rules.extract_input_refused(builder.build())
    assert [(row.file, row.root) for row in found[0].where] == [(torn, "install")]


def _lookup_atlas(frame: str, profile: str = model.ANY_PROFILE) -> AtlasBuilder:
    builder = _builder()
    robot = _model_file(builder, "fx_robot.urdf")
    for link in ("base_link", "laser_link"):
        _link(builder, link, [robot])
    _joint(builder, robot, "laser_joint", "base_link", "laser_link", [0.2, 0.0, 0.2])
    node = ids.build_node("/scan_filter")
    builder.node(node)
    source = Source(
        file=ids.build_file("fx_perception", "src/scan_filter.cpp"), locator="on_scan", line_hint=34
    )
    builder.node(source.file)
    builder.node(ids.build_frame(frame))
    builder.edge(
        model.EdgeKind.LISTENS_TF.value,
        node,
        ids.build_frame(frame),
        key=ids.edge_key(sym="sym:fx_perception|src/scan_filter.cpp|on_scan", frame=frame),
        layer=CODED,
        profile=profile,
        values={"api": "canTransform", "role": "source", "source": frame, "target": "base_link"},
        confidence=CERTAIN,
        evidence=[source],
    )
    return builder


def test_a_frame_the_code_looks_up_and_no_model_has_is_a_warning_with_the_near_link():
    found = frame_rules.frame_not_in_urdf(_lookup_atlas("laser_frame").build())
    assert [item.subject for item in found] == ["frame:laser_frame"]
    assert found[0].severity == "warn"
    assert "the model has laser_link" in found[0].say
    assert "scan_filter.cpp:34" in found[0].say


def test_a_frame_the_code_looks_up_names_the_profiles_it_was_looked_up_in():
    """`frame.not_in_urdf` carried `"profiles": []` unlike every other finding of rover."""
    (found,) = frame_rules.frame_not_in_urdf(_lookup_atlas("laser_frame", NAV).build())
    assert found.profiles == [NAV]
    (again,) = frame_rules.frame_not_in_urdf(_lookup_atlas("laser_frame").build())
    assert again.id == found.id


def test_a_frame_the_code_looks_up_that_the_model_has_is_silent():
    assert frame_rules.frame_not_in_urdf(_lookup_atlas("laser_link").build()) == []


def test_lifecycle_unmanaged_is_silent_when_a_launched_peer_calls_change_state():
    builder = _builder()
    _node(builder, "/lc_talker", NAV, {"executable": "lifecycle_talker", "form": "lifecycle_node"})
    client = _node(builder, "/lc_client", NAV, {"executable": "lifecycle_service_client"})
    service = ids.build_service("/lc_talker/change_state")
    builder.node(service)
    builder.edge(
        model.EdgeKind.CALLS.value,
        client,
        service,
        key=ids.edge_key(symbol="LifecycleServiceClient::init"),
        layer=model.Layer.CODED.value,
        values={"raw_name": "lc_talker/change_state"},
        confidence=CERTAIN,
        evidence=[Source(file=ids.build_file("lifecycle", "src/lifecycle_service_client.cpp"))],
    )
    assert frame_rules.lifecycle_unmanaged(builder.build()) == []


def test_orphan_root_ignores_a_transform_only_a_test_broadcasts():
    builder = _orphan_atlas(tied=False)
    for edge in builder.atlas.edges.values():
        for fact in edge.facts:
            fact.evidence = [
                replace(row, file=ids.build_file("lio_pkg", "test/test_mapping.cpp"))
                if row.file.endswith("laserMapping.cpp")
                else row
                for row in fact.evidence
            ]
    assert frame_rules.frame_orphan_root(builder.build()) == []


# ---------------------------------------------------------------------------
# launch.file_not_installed


def _not_installed_atlas(*, because: str = "file_not_installed", build_type: str = "ament_cmake"):
    builder = _builder()
    builder.node(ids.build_pkg("nx_bringup"), attrs={"build_type": build_type})
    launch = ids.build_file("nx_bringup", "launch/bringup.launch.py")
    builder.node(launch)
    params = ids.build_file("nx_bringup", "config/params.yaml")
    builder.node(
        params,
        attrs={"loadable": False, "refused_because": because, "refused_detail": "no such file"},
    )
    builder.unresolved(
        Unresolved(
            want=model.Want.OTHER.value,
            owner=params,
            expr="params.yaml",
            reason="no such file",
            role="parameters_file",
            evidence=[Source(file=launch, locator="Node(talker)", line_hint=12, root="src")],
        )
    )
    return builder.build()


def test_launch_file_not_installed_names_the_install_line_and_the_reader():
    found = build_rules.launch_file_not_installed(_not_installed_atlas())
    assert [item.subject for item in found] == ["file:nx_bringup|config/params.yaml"]
    assert found[0].severity == "error"
    assert "install(DIRECTORY config DESTINATION share/${PROJECT_NAME})" in found[0].say
    assert "nx_bringup/CMakeLists.txt" in found[0].say
    assert "nx_bringup/launch/bringup.launch.py" in found[0].say
    assert [(row.file, row.root) for row in found[0].where] == [
        ("file:nx_bringup|config/params.yaml", "src"),
        ("file:nx_bringup|launch/bringup.launch.py", "src"),
    ]
    assert build_rules.extract_input_refused(_not_installed_atlas()) == []


def test_launch_file_not_installed_tells_a_python_package_to_edit_setup_py():
    found = build_rules.launch_file_not_installed(_not_installed_atlas(build_type="ament_python"))
    assert "data_files in nx_bringup/setup.py" in found[0].fix
    assert "glob('config/*')" in found[0].fix


def test_launch_file_not_installed_is_silent_on_a_file_that_is_only_unreadable():
    atlas = _not_installed_atlas(because="params_file_unreadable")
    assert build_rules.launch_file_not_installed(atlas) == []
    assert len(build_rules.extract_input_refused(atlas)) == 1


def test_launch_file_not_installed_keeps_its_id_across_two_runs():
    first = _ids(build_rules.launch_file_not_installed(_not_installed_atlas()))
    second = _ids(build_rules.launch_file_not_installed(_not_installed_atlas()))
    assert first == second and len(first) == 1


# ---------------------------------------------------------------------------
# model.mesh_missing

BODY = "package://nx_description/meshes/body.stl"
WHEEL = "package://nx_description/meshes/wheel.stl"


MOVED_WHEEL = "package://nx_description/models/nx/meshes/wheel.stl"


def _mesh_atlas(
    *,
    loaded: bool = True,
    installed: bool = False,
    build_type: str = "ament_cmake",
    moved: bool = False,
):
    builder = _builder()
    builder.node(ids.build_pkg("nx_description"), attrs={"build_type": build_type})
    robot = ids.build_file("nx_description", "urdf/robot.urdf")
    attrs = {"missing_meshes": [WHEEL] if installed else sorted([BODY, WHEEL])}
    attrs["meshes_absent"] = [WHEEL]
    if moved:
        attrs["meshes_elsewhere"] = {WHEEL: [MOVED_WHEEL]}
    if not installed:
        attrs["meshes_not_installed"] = [BODY]
    builder.node(robot, attrs=attrs)
    for frame, name in (("base_link", BODY), ("left_wheel", WHEEL), ("right_wheel", WHEEL)):
        rows = {"visuals": [{"geometry": {"type": "mesh", "filename": name}}], "collisions": []}
        builder.node(ids.build_frame(frame), attrs={"urdf_geometry": {robot: rows}})
    launch = ids.build_file("nx_bringup", "launch/robot.launch.py")
    builder.node(launch)
    site = ids.build_launchent(
        "nx_bringup", "launch/robot.launch.py", [ids.LaunchStep("Node", "robot_state_publisher")]
    )
    reads = [{"file": robot if loaded else launch, "role": "robot_model"}]
    for profile in (NAV, SIM):
        builder.fact(
            site,
            DECLARED,
            profile=profile,
            values={"file_reads": reads, "gates": []},
            confidence=CERTAIN,
            evidence=[Source(file=launch, locator="Node(robot_state_publisher)", line_hint=9)],
        )
    return builder.build()


def test_model_mesh_missing_reports_an_uninstalled_mesh_as_an_error_and_a_wrong_path_as_a_warning():
    found = build_rules.model_mesh_missing(_mesh_atlas())
    by_mesh = {next(m for m in (BODY, WHEEL) if m in item.say): item for item in found}
    assert len(found) == 2 and set(by_mesh) == {BODY, WHEEL}
    body, wheel = by_mesh[BODY], by_mesh[WHEEL]
    assert {item.subject for item in found} == {"file:nx_description|urdf/robot.urdf"}
    assert body.severity == "error"
    assert "install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})" in body.fix
    assert "nx_description/CMakeLists.txt" in body.say
    assert "nx_description/urdf/robot.urdf" in body.say and "link base_link" in body.say
    assert wheel.severity == "warn"
    assert "links left_wheel, right_wheel" in wheel.say
    assert "correct the mesh filename" in wheel.fix
    assert body.profiles == [NAV, SIM] and "profiles nx_navigation, nx_sim" in body.say
    assert [row.file for row in body.where] == [
        "file:nx_bringup|launch/robot.launch.py",
        "file:nx_description|urdf/robot.urdf",
    ]
    assert all(":~" not in item.say for item in found)


def test_model_mesh_missing_tells_a_python_package_to_edit_setup_py():
    found = build_rules.model_mesh_missing(_mesh_atlas(build_type="ament_python"))
    error = next(item for item in found if item.severity == "error")
    assert "data_files in nx_description/setup.py" in error.fix


def test_model_mesh_missing_is_silent_on_an_installed_mesh_and_on_a_model_nobody_loads():
    found = build_rules.model_mesh_missing(_mesh_atlas(installed=True))
    assert [item.severity for item in found] == ["warn"]
    assert build_rules.model_mesh_missing(_mesh_atlas(loaded=False)) == []


def test_model_mesh_missing_names_the_place_the_package_holds_the_mesh():
    """nav2's tb3 model names models/tire.dae and the package holds it under
    models/turtlebot3_model/meshes, so committing the mesh was the wrong advice."""
    found = build_rules.model_mesh_missing(_mesh_atlas(moved=True))
    wheel = next(item for item in found if WHEEL in item.say)
    assert wheel.fix == (
        f"change the mesh filename {WHEEL} in nx_description/urdf/robot.urdf to {MOVED_WHEEL}"
    )
    assert MOVED_WHEEL in wheel.say and "commit" not in wheel.say
    plain = next(
        item for item in build_rules.model_mesh_missing(_mesh_atlas()) if WHEEL in item.say
    )
    assert "commit the mesh" in plain.fix
    assert wheel.id == plain.id


def test_model_mesh_missing_keeps_its_ids_across_two_runs():
    first = _ids(build_rules.model_mesh_missing(_mesh_atlas()))
    second = _ids(build_rules.model_mesh_missing(_mesh_atlas()))
    assert first == second and len(first) == 2


def test_launch_raised_on_a_missing_package_names_it_and_the_apt_name():
    """TurtleBot3 robot.launch.py needs hls_lfcd_lds_driver, and the finding told the user to
    supply an argument or an environment variable."""
    message = "\"package 'hls_lfcd_lds_driver' not found, searched: ROS jazzy, the src root\""
    row = dict(_RAISED_ROW, phase="describe", exc_type="PackageNotFoundError", message=message)
    (found,) = build_rules.launch_raised(_refusal_atlas([row]).build())
    assert "needs the package hls_lfcd_lds_driver" in found.say
    assert "sudo apt install ros-jazzy-hls-lfcd-lds-driver" in found.fix
    assert "add the tree that holds it as a root in rosio.yaml" in found.fix
    assert "supply the argument" not in found.fix
    assert "@outside" not in found.say + found.detail
    unknown = dict(row, message="\"package 'absent_driver' not found\"")
    (near,) = build_rules.launch_raised(_refusal_atlas([unknown]).build())
    assert "ros-$ROS_DISTRO-absent-driver" in near.fix
    (again,) = build_rules.launch_raised(_refusal_atlas([row]).build())
    assert again.id == found.id


def test_launch_raised_names_every_unset_variable_the_walk_saw():
    row = dict(
        _RAISED_ROW,
        phase="describe",
        exc_type="KeyError",
        message="'TURTLEBOT3_MODEL'",
        env="TURTLEBOT3_MODEL LDS_MODEL",
    )
    (found,) = build_rules.launch_raised(_refusal_atlas([row]).build())
    assert "environment variables TURTLEBOT3_MODEL and LDS_MODEL" in found.say
    assert found.fix.startswith("set TURTLEBOT3_MODEL and LDS_MODEL under env of profile")


def _two_profile_budget(nav: int, sim: int) -> model.Atlas:
    builder = _builder()
    for profile, executable, count in ((NAV, "planner", nav), (SIM, "mapper", sim)):
        owner = _node(builder, f"/{executable}", profile, {"executable": executable})
        for index in range(count):
            builder.unresolved(
                Unresolved(
                    want=model.Want.TOPIC_NAME.value,
                    owner=owner,
                    expr=f"topic_from_variable_{index}",
                    reason="not_stated_in_source",
                    evidence=[Source(file=ids.build_file(executable, "src/node.cpp"))],
                )
            )
    builder.count_resolved(model.Want.TOPIC_NAME.value, 9)
    return builder.build()


def test_below_budget_keeps_its_id_when_another_profile_becomes_the_worst():
    """A waiver keyed on the id stopped matching on a run that repaired blind spots."""
    (before,) = build_rules.resolution_below_budget(_two_profile_budget(4, 1))
    (after,) = build_rules.resolution_below_budget(_two_profile_budget(1, 4))
    assert before.id == after.id
    assert before.title.endswith(
        "most blind spots in nx_navigation (4), " + ids.parse_profile(SIM) + " (1)"
    )
    assert after.title.endswith(
        "most blind spots in " + ids.parse_profile(SIM) + " (4), nx_navigation (1)"
    )


def test_below_budget_is_one_finding_that_names_the_worst_profiles_first():
    """One finding per profile repeated the run's rate once per profile and blamed a profile
    holding one blind spot as loudly as the one holding four."""
    builder = _budget_atlas(unresolved=4)
    other = _node(builder, "/mapper", SIM, {"executable": "mapper"})
    builder.unresolved(
        Unresolved(
            want=model.Want.TOPIC_NAME.value,
            owner=other,
            expr="topic_from_mapper",
            reason="not_stated_in_source",
            evidence=[Source(file=ids.build_file("mapper", "src/node.cpp"))],
        )
    )
    (found,) = build_rules.resolution_below_budget(builder.build())
    assert found.subject == "profile:nx_mapping", "one subject whichever profile is worst"
    assert found.title.endswith(
        f"most blind spots in nx_navigation (4), {ids.parse_profile(SIM)} (1)"
    )
    assert set(found.profiles) == {NAV, SIM}
    assert "rosio unresolved" in found.say
    assert "resolution_budget in rosio.yaml" in found.fix
