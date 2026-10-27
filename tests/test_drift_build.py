"""The build family over a tree that is not what runs, and the prose every finding shares.

Per rule: it fires on a crafted atlas, it stays silent on the near miss, and the finding id is
the same across two runs.
"""

from __future__ import annotations

from rosio import ids, model
from rosio.collect import cpp_code
from rosio.collect import launch_declared as launch
from rosio.drift import build as build_rules
from rosio.drift import engine, rules
from rosio.model import AtlasBuilder

DECLARED = model.Layer.DECLARED.value
PROFILE = "profile:online_async"


def _underlay_atlas(**attrs: object) -> model.Atlas:
    builder = AtlasBuilder()
    builder.add_profile("online_async")
    builder.node(ids.build_pkg("slam_toolbox"), attrs=attrs)
    builder.node(ids.build_pkg("rover_bringup"))
    builder.add_run(
        DECLARED,
        profile=PROFILE,
        input_digest="deadbeef",
        predicates={
            **({f"{launch.UNDERLAY_PREDICATE}slam_toolbox": True} if attrs else {}),
            **({launch.INSTALL_UNNAMED: True} if attrs.get("share_build") else {}),
        },
    )
    return builder.build()


def test_build_underlay_names_the_package_the_tree_and_the_build_that_makes_src_count():
    """slam_toolbox in src was read from /opt/ros, and the answer hid it: an edit under src
    changed nothing and nothing said why."""
    atlas = _underlay_atlas(share_read_from="ROS jazzy")
    (found,) = build_rules.build_underlay(atlas)
    assert found.subject == "pkg:slam_toolbox"
    assert found.severity == "warn"
    assert found.profiles == [PROFILE]
    assert found.title == "slam_toolbox is read from ROS jazzy, not from the workspace"
    assert "colcon build --packages-up-to slam_toolbox" in found.fix
    assert "edits in src take effect only after colcon build" in found.say
    (again,) = build_rules.build_underlay(_underlay_atlas(share_read_from="ROS jazzy"))
    assert again.id == found.id


def test_build_underlay_is_silent_on_a_package_read_from_the_workspace():
    assert build_rules.build_underlay(_underlay_atlas()) == []


def test_build_underlay_names_the_setup_file_when_the_install_tree_is_not_sourced():
    atlas = _underlay_atlas(share_read_from="ROS jazzy", install_not_sourced=True)
    (found,) = build_rules.build_underlay(atlas)
    assert "ros_setup" in found.fix
    assert "colcon build" not in found.fix
    assert "is built but its install tree is not sourced" in found.say


def test_build_underlay_says_the_build_beside_src_was_read_and_not_that_nothing_is_built():
    """roots [src] and install/setup.bash sourced after colcon build: the walk read install,
    and the finding said the package in src was not built."""
    tree = "@checkout/install/slam_toolbox"
    (found,) = build_rules.build_underlay(
        _underlay_atlas(share_read_from=tree, share_build="copied")
    )
    found = rules.plain_finding(found)
    assert "not built" not in found.say + found.detail
    assert found.say.startswith(
        "slam_toolbox is read from install/slam_toolbox, the build of its src package"
    )
    assert "edits in src take effect only after colcon build" in found.say
    assert "kind: install" in found.fix
    (linked,) = build_rules.build_underlay(
        _underlay_atlas(share_read_from=tree, share_build="symlinked")
    )
    assert "a file the build links back to src takes effect as soon as it is saved" in linked.say
    assert "after colcon build" in linked.say


def test_a_build_fix_names_the_install_tree_beside_src_that_no_root_names():
    builder = AtlasBuilder()
    builder.add_profile("online_async")
    builder.node(ids.build_pkg("slam_toolbox"), attrs={"share_read_from": "ROS jazzy"})
    builder.add_run(
        DECLARED,
        profile=PROFILE,
        input_digest="deadbeef",
        predicates={launch.INSTALL_UNNAMED: True},
    )
    (found,) = build_rules.build_underlay(builder.build())
    assert "an install tree already sits beside src" in found.fix


# ---------------------------------------------------------------------------
# launch.raised


def _raised_atlas(row: dict, *, refused: str = "") -> model.Atlas:
    builder = AtlasBuilder()
    builder.add_profile("rrbot")
    builder.node(ids.build_launchfile("demo_1", "bringup/launch/rrbot.launch.py"))
    missing = ids.build_file("demo_1", "config/rrbot_controllers.yaml")
    if refused:
        builder.node(missing, attrs={"loadable": False, "refused_because": refused})
    builder.add_run(DECLARED, profile="profile:rrbot", input_digest="deadbeef", exceptions=[row])
    return builder.build()


_MISSING_ROW = {
    "path": "demo_1/bringup/launch/rrbot.launch.py",
    "phase": "visit",
    "exc_type": "FileNotFoundError",
    "message": "parameters: [Errno 2] No such file or directory: "
    "'demo_1/config/rrbot_controllers.yaml'",
    "kept": "Node(spawner)",
    "missing": "file:demo_1|config/rrbot_controllers.yaml",
    "note": "launch.raised",
}


def test_launch_raised_leaves_a_share_file_only_a_build_installs_to_build_unbuilt():
    """One spawner failing on a share file an install rule puts in place gave two error
    findings in an unbuilt workspace that build.unbuilt already covered."""
    assert build_rules.launch_raised(_raised_atlas(_MISSING_ROW, refused="file_after_build")) == []


def test_launch_raised_still_reports_a_missing_file_no_build_puts_in_place():
    (found,) = build_rules.launch_raised(_raised_atlas(_MISSING_ROW, refused="file_missing"))
    assert "demo_1/config/rrbot_controllers.yaml" in found.say
    assert "Node(spawner)" in found.say
    (again,) = build_rules.launch_raised(_raised_atlas(_MISSING_ROW, refused="file_missing"))
    assert again.id == found.id


def test_launch_raised_on_a_required_argument_names_it_and_the_yaml_to_write():
    row = {
        "path": "demo_1/bringup/launch/rrbot.launch.py",
        "phase": "argument",
        "exc_type": "RuntimeError",
        "message": "required launch argument 'description_file' was not provided",
        "note": "launch.raised",
    }
    (found,) = build_rules.launch_raised(_raised_atlas(row))
    assert "args: {description_file: <value>}" in found.fix
    assert "profile rrbot" in found.fix
    assert "needs the launch argument description_file" in found.say


# ---------------------------------------------------------------------------
# extract.pass_incomplete


def test_pass_incomplete_on_a_missing_parser_names_the_install_line():
    builder = AtlasBuilder()
    run = builder.add_run(model.Layer.CODED.value, input_digest="deadbeef")
    reason = (
        "CollectorError: coded.cpp_code: the C++ collector needs tree-sitter and "
        f"tree-sitter-cpp (tree_sitter is missing). Install the extra: {cpp_code.INSTALL_HINT}"
    )
    builder.record_pass(run, "cpp_code", state="refused", reason=reason, missing="C++ sources")
    (found,) = build_rules.extract_pass_incomplete(builder.build())
    assert cpp_code.INSTALL_HINT in found.fix
    assert "repair the input" not in found.fix


# ---------------------------------------------------------------------------
# prose


def test_finding_prose_states_file_ids_and_root_labels_as_paths():
    text = (
        "No such file or directory: @src/rcd/example_10/ros2_control/rrbot.xacro when "
        "processing file: file:demo_10|description/urdf/rrbot.urdf.xacro, traceback in "
        "@outside/lib/python3.12/site-packages/xacro/__init__.py"
    )
    assert rules.plain(text) == (
        "No such file or directory: src/rcd/example_10/ros2_control/rrbot.xacro when "
        "processing file: demo_10/description/urdf/rrbot.urdf.xacro, traceback in "
        "lib/python3.12/site-packages/xacro/__init__.py"
    )


def test_the_engine_hands_out_findings_in_plain_prose():
    atlas = _raised_atlas(
        dict(
            _MISSING_ROW,
            message="Node: No such file or directory: @src/ws/demo_1/config/x.yaml",
            missing="",
        )
    )
    result = engine.run(atlas, only=["launch.raised"])
    (found,) = result.findings
    for text in (found.title, found.detail, found.fix, found.say):
        assert "@src" not in text and "file:" not in text
