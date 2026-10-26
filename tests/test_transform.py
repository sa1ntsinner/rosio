"""The transform tool: one frame relative to another, composed from the joints a robot model
states, and an honest stop wherever the atlas holds no static number."""

from __future__ import annotations

import math

import pytest

from rosio import ids, model
from rosio.model import ANY_PROFILE as ANY
from rosio.model import Source
from rosio.serve import tools

REAL = "profile:real"
SIM = "profile:sim"
MODEL_A = ids.build_file("bot", "urdf/a.urdf")
MODEL_B = ids.build_file("bot", "urdf/b.urdf")
ODOM_SRC = ids.build_file("bot_control", "src/odometry.cpp")
ODOMETRY = ids.build_node("/odometry")

JOINTS = (
    ("base_joint", "fixed", "base_footprint", "base_link", [0.0, 0.0, 0.1], [0.0, 0.0, 0.0], 10),
    ("mast_joint", "fixed", "base_link", "mast", [0.1, 0.0, 0.5], [0.0, 0.0, math.pi / 2], 20),
    ("pan_joint", "revolute", "mast", "pan", [0.0, 0.0, 0.1], [0.0, 0.0, 0.0], 30),
    ("laser_joint", "fixed", "pan", "laser_link", [0.2, 0.0, 0.0], [0.0, 0.0, 0.0], 40),
)


def build(second_model: bool = True, static: list[str] | None = None) -> model.Atlas:
    builder = model.AtlasBuilder()
    builder.declare_layer("declared", present=True)
    builder.declare_layer("coded", present=True)
    builder.add_profile("real")
    builder.add_profile("sim")
    for file_id in (MODEL_A, MODEL_B, ODOM_SRC):
        builder.node(file_id)
    models = [(MODEL_A, REAL, 0.5)] + ([(MODEL_B, SIM, 0.4)] if second_model else [])
    for file_id, profile, mast_height in models:
        for joint, kind, parent, child, xyz, rpy, line in JOINTS:
            if joint == "mast_joint":
                xyz = [0.1, 0.0, mast_height]
            for frame in (parent, child):
                builder.node(ids.build_frame(frame))
            builder.edge(
                "tf_parent",
                ids.build_frame(child),
                ids.build_frame(parent),
                key=ids.edge_key(model=file_id, joint=joint, profile=profile),
                layer="declared",
                profile=profile,
                values={
                    "model": file_id,
                    "joint": joint,
                    "joint_type": kind,
                    "strategy": "urdf",
                    "xyz": xyz,
                    "rpy": rpy,
                },
                confidence="certain",
                evidence=[Source(file=file_id, locator=f"joint[{joint}]", line_hint=line)],
            )
    builder.node(ids.build_frame("odom"))
    builder.node(ODOMETRY, name_source="literal")
    builder.edge(
        "broadcasts_tf",
        ODOMETRY,
        ids.build_frame("base_footprint"),
        key=ids.edge_key(symbol="odometry"),
        layer="coded",
        profile=REAL,
        values={"parent": "odom", "child": "base_footprint", "static": False},
        evidence=[Source(file=ODOM_SRC, locator="sendTransform", line_hint=88)],
    )
    builder.node(ids.build_frame("camera_optical"))
    if static is not None:
        _static_tie(builder, static)
    return builder.build()


@pytest.fixture
def atlas() -> model.Atlas:
    return build()


def test_a_three_joint_chain_with_a_rotated_joint_composes_to_the_right_numbers(atlas):
    answer = tools.transform(atlas, "laser_link", "frame:base_link", model="urdf/a.urdf")

    assert 'lookupTransform("base_link", "laser_link")' in answer
    assert "translation xyz [0.1 0.2 0.6] m" in answer
    assert "rpy [0 0 1.570796] rad, quaternion xyzw [0 0 0.707107 0.707107]" in answer
    assert "joint mast_joint (fixed) base_link -> mast: xyz [0.1 0 0.5] rpy" in answer
    assert "bot/urdf/a.urdf:20" in answer
    assert "chain from base_link down to laser_link, 3 links" in answer
    assert len(answer.encode()) <= tools.MAX_BYTES


def test_the_inverse_question_inverts_the_pose(atlas):
    answer = tools.transform(atlas, "base_link", "laser_link", model="urdf/a.urdf")

    assert "translation xyz [-0.2 0.1 -0.6] m" in answer
    assert "rpy [0 0 -1.570796]" in answer
    assert "chain from laser_link up to base_link, 3 links" in answer


def test_a_revolute_joint_is_taken_at_zero_and_says_so(atlas):
    answer = tools.transform(atlas, "laser_link", "base_link", model="urdf/a.urdf")

    assert "joint pan_joint (revolute) mast -> pan" in answer
    assert "pan, moves, taken at position zero" not in answer
    assert "moves, taken at position zero" in answer
    assert "1 movable joint on the chain, taken at position zero" in answer


def test_a_broadcast_on_the_chain_stops_and_names_the_node(atlas):
    answer = tools.transform(atlas, "laser_link", "odom", profile="real")

    assert "not static: odom -> base_footprint is broadcast by /odometry" in answer
    assert "bot_control/src/odometry.cpp:88" in answer
    assert "translation" not in answer


def test_two_models_that_disagree_answer_one_each_with_their_profiles(atlas):
    answer = tools.transform(atlas, "laser_link", "base_link")

    assert "2 robot models state this chain differently" in answer
    assert "model bot/urdf/a.urdf, loaded by profile real" in answer
    assert "model bot/urdf/b.urdf, loaded by profile sim" in answer
    assert "translation xyz [0.1 0.2 0.6] m" in answer
    assert "translation xyz [0.1 0.2 0.5] m" in answer


def test_one_model_gives_one_answer():
    answer = tools.transform(build(second_model=False), "laser_link", "base_link")

    assert "differently" not in answer
    assert answer.count("translation xyz") == 1


def test_an_unknown_frame_is_named_and_never_a_zero(atlas):
    answer = tools.transform(atlas, "laser_frame", "base_link")

    assert "no frame named 'laser_frame'" in answer
    assert "frames: base_footprint, base_link, camera_optical" in answer
    assert "translation" not in answer


def test_a_frame_no_joint_reaches_shares_no_ancestor(atlas):
    answer = tools.transform(atlas, "camera_optical", "base_link", model="urdf/a.urdf")

    assert "share no ancestor" in answer
    assert "translation" not in answer


def test_an_unreadable_origin_is_said_and_not_composed():
    atlas = build(second_model=False)
    edge = next(
        item
        for item in atlas.edges.values()
        if item.kind == "tf_parent" and item.facts[0].values["joint"] == "pan_joint"
    )
    del edge.facts[0].values["xyz"]
    edge.facts[0].values["unreadable"] = ["origin.xyz"]

    answer = tools.transform(atlas, "laser_link", "base_link")

    assert "joint pan_joint states an origin that is not numbers (origin.xyz)" in answer
    assert "xyz [unknown]" in answer
    assert "translation" not in answer


def test_the_answer_is_the_same_twice(atlas):
    assert tools.transform(atlas, "laser_link", "base_link") == tools.transform(
        atlas, "laser_link", "base_link"
    )


def _joint(atlas: model.Atlas, joint: str, file_id: str = MODEL_A) -> dict:
    return next(
        item.facts[0].values
        for item in atlas.edges.values()
        if item.kind == "tf_parent"
        and item.facts[0].values["joint"] == joint
        and item.facts[0].values["model"] == file_id
    )


def test_an_unreadable_limit_does_not_stop_a_readable_origin():
    atlas = build(second_model=False)
    _joint(atlas, "pan_joint")["unreadable"] = ["limit.lower", "mimic.multiplier"]

    answer = tools.transform(atlas, "laser_link", "base_link")

    assert "not numbers" not in answer
    assert "translation xyz [0.1 0.2 0.6] m" in answer


def test_two_models_with_one_pose_through_different_joints_are_two_answers(atlas):
    _joint(atlas, "mast_joint", MODEL_B)["xyz"] = [0.1, 0.0, 0.5]
    pan = _joint(atlas, "pan_joint", MODEL_B)
    pan["joint"], pan["joint_type"] = "pan_mount", "fixed"

    answer = tools.transform(atlas, "laser_link", "base_link")

    assert "2 robot models state this chain differently" in answer
    assert "joint pan_mount (fixed) mast -> pan" in answer
    assert "bot/urdf/b.urdf:30" in answer


def test_a_half_turn_prints_one_quaternion_whichever_way_it_was_written():
    answers = []
    for yaw in (math.pi, -math.pi):
        atlas = build(second_model=False)
        _joint(atlas, "mast_joint")["rpy"] = [0.0, 0.0, yaw]
        answers.append(tools.transform(atlas, "mast", "base_link"))

    assert all("quaternion xyzw [0 0 1 0]" in answer for answer in answers)


LAUNCH = ids.build_file("bot_bringup", "launch/tie.launch.py")
TIE = ids.build_node("/laser_tie")


def with_static(argv: list[str]) -> model.Atlas:
    """Model a, plus a static_transform_publisher a launch file starts on laser_link."""
    return build(second_model=False, static=argv)


def _static_tie(builder: model.AtlasBuilder, argv: list[str]) -> None:
    # no collector mints the child frame a static publisher names, so `scan` has no entity
    builder.node(LAUNCH)
    builder.node(TIE, name_source="literal")
    builder.fact(
        TIE,
        "declared",
        profile=REAL,
        values={"executable": "static_transform_publisher", "pkg": "tf2_ros", "argv": argv},
        evidence=[
            Source(file="file:@repo|rosio/resolve/known_names.py", locator="tf2_ros"),
            Source(file=LAUNCH, locator="Node(static_transform_publisher)", line_hint=30),
        ],
    )


FLAGS = ["--x", "0.1", "--yaw", "1.5707963267948966", "--frame-id", "laser_link"]


def test_a_static_publisher_on_the_chain_composes_and_names_its_launch_line():
    atlas = with_static([*FLAGS, "--child-frame-id", "scan", "--ros-args", "-r", "__node:=tie"])
    answer = tools.transform(atlas, "scan", "base_link", profile="real")

    assert "translation xyz [0.1 0.3 0.6] m" in answer
    assert "static laser_link -> scan by /laser_tie (declared): xyz [0.1 0 0]" in answer
    assert "bot_bringup/launch/tie.launch.py:30" in answer
    assert "known_names" not in answer
    assert "not static" not in answer


def test_the_positional_quaternion_form_composes_the_same_pose():
    half = "0.7071068"
    atlas = with_static(["0.1", "0", "0", "0", "0", half, half, "laser_link", "scan"])
    answer = tools.transform(atlas, "scan", "base_link", profile="real")

    assert "translation xyz [0.1 0.3 0.6] m" in answer
    assert "quaternion xyzw [0 0 0.7071068 0.7071068] at" in answer


def test_a_static_argument_that_is_not_a_number_is_named_and_not_composed():
    atlas = with_static(
        ["--x", "$(var offset)", "--frame-id", "laser_link", "--child-frame-id", "scan"]
    )
    answer = tools.transform(atlas, "scan", "base_link", profile="real")

    assert "/laser_tie starts with arguments that are not numbers (xyz)" in answer
    assert "translation" not in answer


def _expansion(builder: model.AtlasBuilder, profile: str, prefix: str, mast: float) -> None:
    for joint, kind, parent, child, xyz, rpy, _line in JOINTS:
        if joint == "mast_joint":
            xyz = [0.1, 0.0, mast]
        for frame in (parent, child):
            builder.node(ids.build_frame(prefix + frame))
        builder.edge(
            "tf_parent",
            ids.build_frame(prefix + child),
            ids.build_frame(prefix + parent),
            key=ids.edge_key(model=MODEL_A, joint=prefix + joint, profile=profile),
            layer="declared",
            profile=profile,
            values={
                "model": MODEL_A,
                "joint": prefix + joint,
                "joint_type": kind,
                "strategy": "xacro",
                "xyz": xyz,
                "rpy": rpy,
            },
            confidence="certain",
            evidence=[Source(file=MODEL_A, locator=f"joint[{prefix}{joint}]")],
        )


def test_a_profile_expansion_of_a_model_replaces_the_defaults_for_that_profile():
    """A profile that hands xacro its own arguments runs that expansion. Asked for that
    profile, the expansion answers. Asked for no profile, the defaults answer."""
    builder = model.AtlasBuilder()
    builder.declare_layer("declared", present=True)
    builder.add_profile("sim")
    builder.node(MODEL_A)
    _expansion(builder, ANY, "", 0.5)
    _expansion(builder, SIM, "", 0.7)
    _expansion(builder, SIM, "tb3/", 0.9)
    atlas = builder.build()

    assert "translation xyz [0.1 0.2 0.8] m" in tools.transform(
        atlas, "laser_link", "base_link", profile="sim"
    )
    assert "translation xyz [0.1 0.2 1] m" in tools.transform(
        atlas, "tb3/laser_link", "tb3/base_link", profile="sim"
    )
    assert "translation xyz [0.1 0.2 0.6] m" in tools.transform(atlas, "laser_link", "base_link")
    assert "translation xyz [0.1 0.2 1] m" in tools.transform(
        atlas, "tb3/laser_link", "tb3/base_link"
    )
