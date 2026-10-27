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


def test_build_underlay_names_the_package_the_tree_and_the_build_that_makes_it_run():
    """slam_toolbox in src runs from /opt/ros until a build, and the atlas reads src."""
    atlas = _underlay_atlas(launch_runs_from="ROS jazzy")
    (found,) = build_rules.build_underlay(atlas)
    assert found.subject == PROFILE
    assert found.severity == "warn"
    assert found.profiles == [PROFILE]
    assert found.title == (
        "slam_toolbox runs from ROS jazzy, and the atlas reads it from the workspace"
    )
    assert "colcon build --packages-up-to slam_toolbox" in found.fix
    assert "an edit takes effect only once the workspace is built and sourced" in found.say
    (again,) = build_rules.build_underlay(_underlay_atlas(launch_runs_from="ROS jazzy"))
    assert again.id == found.id


def test_build_underlay_is_one_finding_for_the_workspace_naming_every_package():
    """nav2 main on jazzy: one finding per package, 42 of them, for one missing build."""
    builder = AtlasBuilder()
    builder.add_profile("online_async")
    builder.node(ids.build_pkg(ids.REPO_PKG))
    for name in ("costmap_queue", "nav2_bringup", "nav2_common"):
        builder.node(ids.build_pkg(name), attrs={"launch_runs_from": "ROS jazzy"})
    builder.add_run(DECLARED, profile=PROFILE, input_digest="deadbeef")
    (found,) = build_rules.build_underlay(builder.build())
    assert found.subject == ids.build_pkg(ids.REPO_PKG)
    assert "costmap_queue, nav2_bringup and nav2_common" in found.say
    assert found.title.startswith("3 packages run from another tree")


def test_build_underlay_is_silent_on_a_package_read_from_the_workspace():
    assert build_rules.build_underlay(_underlay_atlas()) == []


def test_build_underlay_names_the_setup_file_when_the_install_tree_is_not_sourced():
    atlas = _underlay_atlas(launch_runs_from="ROS jazzy", install_not_sourced=True)
    (found,) = build_rules.build_underlay(atlas)
    assert "ros_setup" in found.fix
    assert "colcon build" not in found.fix
    assert "the setup file of the install tree that holds it is not sourced" in found.detail


def test_build_underlay_says_the_build_beside_src_runs_and_not_that_nothing_is_built():
    """roots [src] and install/setup.bash sourced after colcon build: ros2 launch runs the
    build, and the finding said the package in src was not built."""
    tree = "@checkout/install/slam_toolbox"
    (found,) = build_rules.build_underlay(
        _underlay_atlas(launch_runs_from=tree, share_build="copied")
    )
    found = rules.plain_finding(found)
    assert "not built" not in found.say + found.detail
    assert "the build of its src package" in found.detail
    assert "an edit takes effect only once the workspace is built and sourced" in found.say
    assert "kind: install" in found.fix
    (linked,) = build_rules.build_underlay(
        _underlay_atlas(launch_runs_from=tree, share_build="symlinked")
    )
    assert "a file the build links back to src takes effect as soon as it is saved" in linked.say
    assert "after colcon build" in linked.say


def test_a_build_fix_names_the_install_tree_beside_src_that_no_root_names():
    builder = AtlasBuilder()
    builder.add_profile("online_async")
    builder.node(ids.build_pkg("slam_toolbox"), attrs={"launch_runs_from": "ROS jazzy"})
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


def _import_row(exc_type: str, message: str) -> dict:
    return {
        "path": "demo_1/bringup/launch/rrbot.launch.py",
        "phase": "import",
        "exc_type": exc_type,
        "message": message,
        "note": "launch.raised",
    }


def test_launch_raised_on_a_launch_name_the_release_lacks_asks_for_a_newer_release():
    """cloned_multi_tb3_simulation_launch.py imports ForEach, which jazzy's launch lacks, and
    the fix said to supply an argument or the environment."""
    row = _import_row(
        "ImportError", "cannot import name 'ForEach' from 'launch.actions' (read from ROS jazzy)"
    )
    (found,) = build_rules.launch_raised(_raised_atlas(row))
    assert "needs a newer ROS release than ROS jazzy" in found.fix
    assert "supply the argument" not in found.fix
    assert "ForEach" in found.say


def test_launch_raised_on_a_module_the_workspace_holds_names_both_copies():
    row = _import_row(
        "ImportError",
        "cannot import name 'LaunchConfigAsBool' from 'nav2_common.launch' (read from ROS "
        f"jazzy{launch.WORKSPACE_COPY}@src/nav2/nav2_common/nav2_common)",
    )
    (found,) = build_rules.launch_raised(_raised_atlas(row))
    assert "src/nav2/nav2_common/nav2_common" in found.fix
    assert "ROS jazzy" in found.fix


def test_launch_raised_never_asks_to_install_a_package_that_is_installed():
    """bringup_launch.py imports nav2_bringup, the installed ros-jazzy-nav2-bringup ships no
    module of that name, and the fix was sudo apt install ros-jazzy-nav2-bringup."""
    message = (
        "No module named 'nav2_bringup', searched: ROS jazzy, "
        + launch.INSTALLED_WITHOUT_MODULE.format("nav2_bringup")
    )
    (found,) = build_rules.launch_raised(_raised_atlas(_import_row("ModuleNotFoundError", message)))
    assert "apt install" not in found.fix + found.say
    assert "ships no python module nav2_bringup" in found.fix
    plain = _import_row(
        "ModuleNotFoundError", "No module named 'nav2_bringup', searched: ROS jazzy"
    )
    (missing,) = build_rules.launch_raised(_raised_atlas(plain))
    assert "apt install ros-jazzy-nav2-bringup" in missing.fix


def test_relabel_import_names_the_release_read_and_the_copy_the_workspace_holds(tmp_path):
    from rosio.config import RootSpec
    from rosio.workspace import Workspace

    package = tmp_path / "src" / "nav2_common"
    (package / "nav2_common").mkdir(parents=True)
    (package / "package.xml").write_text(
        "<package format='3'><name>nav2_common</name><version>1</version></package>"
    )
    (package / "CMakeLists.txt").write_text("ament_python_install_package(nav2_common)\n")
    (package / "nav2_common" / "__init__.py").write_text("")
    ws = Workspace.discover([RootSpec("src", str(tmp_path / "src"))])
    message = launch.relabel_import(
        ws,
        "cannot import name 'LaunchConfigAsBool' from 'nav2_common.launch' "
        "(@outside/lib/python3.12/site-packages/nav2_common/launch/__init__.py)",
        ("/opt/ros/jazzy/setup.bash",),
    )
    name, module, tree, held = launch.cannot_import(message)
    assert (name, module, tree) == ("LaunchConfigAsBool", "nav2_common.launch", "ROS jazzy")
    assert held.endswith("nav2_common/nav2_common")
    plain = launch.relabel_import(
        ws, "cannot import name 'ForEach' from 'launch.actions' (@outside/x.py)", ()
    )
    assert launch.cannot_import(plain)[2:] == ("@outside/x.py", "")


def test_relabel_module_says_when_the_release_installs_the_package_without_the_module(tmp_path):
    prefix = tmp_path / "opt" / "ros" / "jazzy"
    (prefix / "share" / "nav2_bringup").mkdir(parents=True)
    (prefix / "share" / "nav2_bringup" / "package.xml").write_text("<package/>")
    setup = str(prefix / "setup.bash")
    message = launch.relabel_module("No module named 'nav2_bringup'", (setup,))
    assert launch.INSTALLED_WITHOUT_MODULE.format("nav2_bringup") in message
    assert launch.INSTALLED_WITHOUT_MODULE.format("other") not in launch.relabel_module(
        "No module named 'other'", (setup,)
    )


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


def test_launch_raised_on_an_unknown_tag_names_it_and_the_missing_extension():
    """`<autoware_node>` needs a launch frontend extension, and the fix said to supply an
    argument or the environment."""
    row = {
        "path": "demo_1/bringup/launch/rrbot.launch.py",
        "phase": "describe",
        "exc_type": "RuntimeError",
        "message": "Unrecognized entity of the type: autoware_node",
        "note": "launch.raised",
    }
    (found,) = build_rules.launch_raised(_raised_atlas(row))
    assert "<autoware_node>" in found.fix
    assert "frontend extension" in found.say
    assert "supply the argument" not in found.fix
    assert [row.locator for row in found.where] == [""]
