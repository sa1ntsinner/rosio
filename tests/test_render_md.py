"""The two markdown deliverables: what they must state, and that they state it twice the same.

The fixture is small on purpose and carries one instance of every shape the renderers have
to survive: a node whose source was read and one whose source was not, a synthetic identity,
a candidate chain a python write wins, an exported plugin nothing selects, a two model frame
and a blind spot.
"""

from __future__ import annotations

import sys
import types
from pathlib import Path

import pytest

from rosio import ids, model, render
from rosio.drift import rules
from rosio.model import Candidate, Finding, Source, Unresolved
from rosio.render import DRIFT_NAME, REPORT_NAME, table, write_all
from rosio.render.drift_md import render_drift
from rosio.render.report import render_report

REAL = "profile:nx_navigation"
SIM = "profile:nx_sim"

PARAMS_FILE = ids.build_file("nx_bringup", "config/nav2_params.yaml")
LAUNCHER_FILE = ids.build_file(ids.REPO_PKG, "dx_app/dx_launcher/profiles.py")
URDF = ids.build_file("nx_bringup", "urdf/fx_robot.urdf")
PLUGIN_XML = ids.build_file("nx_perception", "obstacle_layer_plugins.xml")

BT = ids.build_node("/bt_navigator")
MONITOR = ids.build_node("/collision_monitor")
RVIZ = ids.build_node_synthetic("rviz2", "rviz2", "/", "6962a40e")
CMD_VEL = ids.build_topic("/cmd_vel_safe")
BT_XML = ids.build_param(BT, "default_nav_to_pose_bt_xml")
PERCEPTION_PKG = ids.build_pkg("nx_perception")
DYNAMIC_LAYER = ids.build_plugin("nav2_costmap_2d::Layer", "nx_perception::MovingObstacleLayer")
INFLATION = ids.build_plugin("nav2_costmap_2d::Layer", "nav2_costmap_2d::InflationLayer")
INFLATION_INSTANCE = ids.build_pinst(MONITOR, "inflation_layer")
BASE_LINK = ids.build_frame("base_link")
CAMERA = ids.build_frame("camera_link")
ODOM = ids.build_frame("odom")
SYM = ids.build_sym(ids.REPO_PKG, "dx_app/dx_launcher/profiles.py", "nav2_command")


def build() -> model.Atlas:
    builder = model.AtlasBuilder(capabilities=["drift.findings", "layer.coded", "layer.declared"])
    builder.declare_layer("declared", present=True)
    builder.declare_layer("coded", present=True)
    builder.add_profile(
        "nx_navigation",
        label="Navigation, the real robot stack",
        entry=[ids.build_launchfile("nx_bringup", "launch/nx_navigation.launch.py")],
    )
    builder.add_profile("nx_sim", label="Simulation")
    for file_id in (PARAMS_FILE, LAUNCHER_FILE, URDF, PLUGIN_XML):
        builder.node(file_id)
    builder.node(SYM)
    builder.node(PERCEPTION_PKG)

    builder.fact(
        REAL,
        "declared",
        profile=REAL,
        values={
            "entrypoints": ["ros2 launch nx_bringup nx_navigation.launch.py"],
            "launch_files": [ids.build_launchfile("nx_bringup", "launch/nav.launch.py")],
            "declared_nodes": 2,
        },
    )
    builder.fact(SIM, "declared", profile=SIM, values={"entrypoints": ["ros2 launch a b.py"]})

    builder.fact(BT, "declared", profile=REAL, values={"pkg": "nav2_bt_navigator"})
    builder.fact(BT, "coded", profile=REAL, values={"pkg": "nav2_bt_navigator"})
    builder.node(BT, name_source="literal")
    builder.fact(MONITOR, "declared", profile=REAL, values={"pkg": "nav2_collision_monitor"})
    builder.node(MONITOR, name_source="executable_default")
    builder.fact(RVIZ, "declared", profile=REAL, values={"pkg": "rviz2"})
    builder.node(RVIZ, name_source="runtime_randomised")

    builder.node(CMD_VEL)
    builder.edge(
        "publishes",
        MONITOR,
        CMD_VEL,
        key=ids.edge_key(symbol=SYM),
        layer="coded",
        profile=REAL,
        values={"raw_name": "cmd_vel", "remapped": True},
    )
    builder.edge(
        "publishes",
        MONITOR,
        CMD_VEL,
        key=ids.edge_key(symbol=SYM, site="watchdog"),
        layer="coded",
        profile=REAL,
    )
    builder.edge(
        "subscribes", BT, CMD_VEL, key=ids.edge_key(symbol=SYM), layer="coded", profile=REAL
    )

    builder.set_param(
        BT_XML,
        state="effective",
        candidates=[
            Candidate(
                value="nx_ws/src/nx_bringup/behavior_trees/nx_nav_to_pose.xml",
                value_type="str",
                mechanism="python_override",
                source=Source(file=LAUNCHER_FILE, locator="_nav2_overrides_command", line_hint=430),
                rank=0,
                wins=True,
                profile=REAL,
                applied_by=SYM,
            ),
            Candidate(
                value="nx_nav_to_pose.xml",
                value_type="str",
                mechanism="yaml_base",
                source=Source(file=PARAMS_FILE, locator="bt_navigator", line_hint=60),
                rank=1,
                wins=False,
                profile=REAL,
                shadow_kind="value_changed",
                shadowed_by=0,
            ),
        ],
    )
    builder.edge("has_param", BT, BT_XML)

    builder.fact(DYNAMIC_LAYER, "declared", values={"lookup": "nx_perception::MovingObstacleLayer"})
    builder.edge("provides_plugin", PERCEPTION_PKG, DYNAMIC_LAYER, layer="declared")
    builder.fact(
        INFLATION, "declared", profile=REAL, values={"lookup": "nav2_costmap_2d::InflationLayer"}
    )
    builder.fact(INFLATION_INSTANCE, "declared", profile=REAL, values={"selector": "plugins"})
    builder.edge("instantiates", INFLATION_INSTANCE, INFLATION, layer="declared", profile=REAL)

    builder.node(BASE_LINK)
    builder.node(CAMERA)
    builder.node(ODOM)
    builder.edge(
        "tf_parent",
        CAMERA,
        BASE_LINK,
        key=ids.edge_key(model=URDF, joint="camera_joint"),
        layer="declared",
        values={"strategy": "urdf", "joint": "camera_joint", "xyz": [3.61, 0.0, 0.36]},
        confidence="certain",
    )
    # The code side states the pair the other way round, parent first.
    builder.edge(
        "tf_parent",
        ODOM,
        BASE_LINK,
        key=ids.edge_key(symbol=SYM, frames="odom>base_link"),
        layer="coded",
        values={"source": "py_broadcast", "symbol": SYM, "static": False},
        confidence="certain",
    )

    builder.unresolved(
        Unresolved(
            want="param_value",
            owner=SYM,
            expr="os.environ.get('NX_MOTION_EXTENSION', 'auto')",
            reason="environment_variable",
            evidence=[Source(file=LAUNCHER_FILE, line_hint=91)],
        )
    )
    builder.unresolved(
        Unresolved(
            want="other",
            owner=SYM,
            expr="/map:=/slam_map",
            reason="the remap resolves to /slam_map, and nothing states the endpoint kind",
            evidence=[Source(file=LAUNCHER_FILE, line_hint=12)],
        )
    )

    builder.finding(
        Finding(
            rule="param.shadowed",
            severity="warn",
            subject=BT_XML,
            title="default_nav_to_pose_bt_xml at nav2_params.yaml:60 is overridden",
            detail="the launcher assigns the key after the merge, so the write wins",
            fix="edit profiles.py:430",
            say="In profile nx_navigation, /bt_navigator default_nav_to_pose_bt_xml is set "
            "at dx_app/dx_launcher/profiles.py:430.",
            profiles=[REAL],
            where=[Source(file=PARAMS_FILE, locator="bt_navigator", line_hint=60)],
        )
    )
    builder.finding(
        Finding(
            rule="plugin.unused",
            severity="info",
            subject=DYNAMIC_LAYER,
            title="nx_perception::MovingObstacleLayer is built and nothing loads it",
            detail="no plugins list in any profile names it",
            fix="add it to a plugins list or stop exporting it",
            say="nx_perception exports nx_perception::MovingObstacleLayer and no profile "
            "selects it.",
            where=[Source(file=PLUGIN_XML, locator="class")],
        )
    )
    builder.finding(
        Finding(
            rule="frame.parent_differs",
            severity="error",
            subject=CAMERA,
            title="camera_link sits at a different origin in 2 robot models",
            detail="the two models state two origins for one joint",
            fix="make the models agree, or record which model each profile loads",
            say="camera_joint puts camera_link at xyz [3.61 0 0.36] in fx_robot.urdf.",
            where=[Source(file=URDF, locator="joint[camera_joint]", line_hint=346)],
        )
    )
    atlas = builder.build()
    atlas.counters["drift"] = {
        "rules_run": 31,
        "rules_failed": 0,
        "waived": 0,
        "failed": [],
        "by_rule": {"param.shadowed": 1, "plugin.unused": 1, "frame.parent_differs": 1},
    }
    atlas.x["join"] = {"endpoints_bound": 2, "endpoints_remapped": 1, "endpoints_unresolved": 0}
    return atlas


@pytest.fixture
def atlas() -> model.Atlas:
    return build()


# ---------------------------------------------------------------------------
# shared shaping


def test_a_table_escapes_a_separator_and_refuses_an_empty_body():
    assert table(["a"], []) == []
    rows = table(["name"], [["left|right"]])
    assert rows[-1] == "| left\\|right |"


# ---------------------------------------------------------------------------
# the atlas report


def test_the_report_states_coverage_before_it_states_anything_else(atlas):
    text = render_report(atlas)

    coverage = text.index("## Coverage")
    assert coverage < text.index("## Node inventory")
    assert coverage < text.index("## Parameter surface")
    assert "1 of 3 (33 percent)" in text


def test_the_report_names_the_profiles_and_the_commands_that_build_them(atlas):
    text = render_report(atlas)

    assert "nx_navigation" in text
    assert "ros2 launch nx_bringup nx_navigation.launch.py" in text
    assert "Navigation, the real robot stack" in text


def test_the_report_says_which_layers_are_missing_and_what_each_present_one_gave(atlas):
    text = render_report(atlas)

    assert "observed and recorded layers are not in this atlas" in text
    assert "C++ and python sources" in text


def test_the_report_counts_the_dead_parameters_next_to_the_effective_ones(atlas):
    text = render_report(atlas)

    assert "## Parameter surface" in text
    assert "python_override" in text
    assert "value_changed 1" in text


def test_the_report_groups_blind_spots_by_cause_and_never_invents_one(atlas):
    text = render_report(atlas)

    # A code is shown as its sentence, the code itself is for --json.
    assert "an environment variable at run time" in text
    assert "environment_variable" not in text
    assert "the remap resolves to ..." in text
    assert "2 values could not be resolved" in text


def test_the_report_states_the_facts_that_need_two_sources_at_once(atlas):
    text = render_report(atlas)

    section = text[text.index("## What no single layer states") :]
    assert "profiles.py:430" in section
    assert "nx_perception::MovingObstacleLayer" in section
    assert "'cmd_vel' in its source" in section


def test_one_of_a_thing_is_written_as_one_and_not_as_many(atlas):
    section = render_report(atlas)
    section = section[section.index("## What no single layer states") :]

    assert "1 endpoint carries one name in the code and another at run time." in section
    assert "1 plugin is built and exported by a package in this workspace" in section
    assert "no profile selects it." in section
    assert "1 plugins" not in section
    assert "1 endpoints" not in section


def test_a_topic_counts_the_nodes_on_it_and_a_node_counts_its_own_call_sites(atlas):
    """One node publishing one topic from two call sites is one publisher of that topic and
    two endpoints of that node. Counting edges under both headings says two publishers."""
    text = render_report(atlas)

    section = text[text.index("## Busiest nodes") : text.index("## Parameter surface")]
    assert "| /collision_monitor | 2 | 0 | 0 |" in section
    assert "| /cmd_vel_safe | 1 | 1 |" in section


def test_the_frame_tree_reads_the_direction_each_transform_states(atlas):
    """A urdf joint states the child first and a code broadcast states the parent first.
    Assuming the edge direction hangs odom under base_link, which is the tree upside down."""
    text = render_report(atlas)

    tree = text[text.index("## Frames") : text.index("## Plugins")]
    lines = [line for line in tree.splitlines() if line.startswith("    ")]

    assert lines[0] == "    odom"
    assert lines[1] == "      base_link"
    assert lines[2] == "        camera_link"


def test_no_synthetic_id_and_no_dash_punctuation_reaches_either_page(atlas):
    for text in (render_report(atlas), render_drift(atlas)):
        assert ":~" not in text
        assert "\u2014" not in text
    assert "rviz2/rviz2 (unnamed by launch)" in render_report(atlas)


def test_a_default_the_collector_could_not_read_is_never_printed_as_a_value():
    """`value_type` empty is the collector saying the default is not a literal. Printing the
    None it stored would state a value the source never spells."""
    builder = model.AtlasBuilder()
    builder.add_profile("nx_navigation")
    param = ids.build_pparam("nav2_core::Controller", "nx_planner/Docking", "dock_gain")
    builder.set_param(
        param,
        state="other",
        candidates=[
            Candidate(
                value=None,
                value_type="",
                mechanism="code_default",
                source=Source(
                    file=ids.build_file("nx_planner", "src/docking_controller.cpp"),
                    locator="configure:declare_parameter",
                    line_hint=52,
                ),
                rank=0,
                wins=False,
                note="the default is not a literal",
            )
        ],
    )
    text = render_report(builder.build())

    assert "defaults to None" not in text
    assert "run on the default written in their own source" not in text


def test_a_probe_atlas_is_not_told_it_describes_no_running_system():
    builder = model.AtlasBuilder()
    builder.add_profile("nx_navigation")
    builder.declare_layer("declared", present=True)
    builder.declare_layer("observed", present=True)
    text = render_report(builder.build())

    assert "coded and recorded layers are not in this atlas" in text
    assert "nothing below is a statement about a running system" not in text


def test_two_renders_of_one_atlas_are_byte_identical(atlas):
    assert render_report(atlas) == render_report(build())
    assert render_drift(atlas) == render_drift(build())


# ---------------------------------------------------------------------------
# the drift report


def test_drift_leads_with_the_counts_and_the_rule_table(atlas):
    text = render_drift(atlas)

    head = text[: text.index("##")]
    assert "3 findings: 1 error, 1 warn, 1 info." in head
    assert "31 rules ran" in head
    assert "| rule | findings | error | warn | info | profiles |" in head


def test_drift_orders_families_by_their_worst_severity(atlas):
    text = render_drift(atlas)

    assert text.index("## Frames and transforms") < text.index("## Parameters")
    assert text.index("## Parameters") < text.index("## Plugins")


def test_every_finding_carries_the_say_the_site_the_reason_and_the_fix(atlas):
    text = render_drift(atlas)

    block = text[text.index("### default_nav_to_pose_bt_xml") : text.index("## Plugins")]
    assert "> In profile nx_navigation" in block
    assert "nx_bringup/config/nav2_params.yaml:60 bt_navigator" in block
    assert "Why: the launcher assigns the key after the merge" in block
    assert "Fix: edit profiles.py:430" in block
    assert "Waiver key `find:param.shadowed" in block


def test_a_finding_with_no_profile_says_so_rather_than_claiming_all_of_them(atlas):
    text = render_drift(atlas)

    assert "not scoped to a profile" in text
    assert "every profile" not in text


def test_the_silent_rules_are_listed_with_the_reason_they_are_silent(atlas):
    text = render_drift(atlas)

    tail = text[text.index("## Rules that said nothing") :]
    assert "topic.no_publisher | warn | needs the observed layer" in tail
    assert "param.type_changed | error | ran over this atlas and matched nothing" in tail
    assert "param.shadowed" not in tail


def test_a_rule_that_failed_is_not_reported_as_a_clean_result(atlas):
    atlas.counters["drift"]["failed"] = ["build.stale"]

    text = render_drift(atlas)
    tail = text[text.index("## Rules that said nothing") :]
    assert "build.stale | warn | the rule failed this run" in tail


def test_an_atlas_no_rule_ever_ran_over_is_not_reported_as_a_clean_stack(atlas):
    """A rule that never loaded reads exactly like a rule that matched nothing, which is why
    the count is stated and why no row may claim a run that did not happen."""
    del atlas.counters["drift"]
    atlas.findings = []

    tail = render_drift(atlas)[render_drift(atlas).index("## Rules that said nothing") :]

    assert "ran over this atlas and matched nothing" not in tail
    assert "no drift run is recorded in this atlas" in tail
    assert f"0 of the {len(rules.COMMITTED)} committed rules ran" in tail


def test_findings_without_a_recorded_run_are_not_read_as_a_full_pass(atlas):
    """An atlas that carries findings and no drift counters states neither how many rules ran
    nor that the rest matched nothing, because neither is known."""
    del atlas.counters["drift"]

    text = render_drift(atlas)

    assert "No drift run is recorded in this atlas and 3 rules produced something." in text
    assert "ran over this atlas and matched nothing" not in text
    assert f"0 of the {len(rules.COMMITTED)} committed rules ran" in text


def test_an_atlas_with_no_findings_still_says_which_rules_were_silent():
    builder = model.AtlasBuilder()
    builder.add_profile("nx_navigation")
    text = render_drift(builder.build())

    assert "0 findings: none." in text
    assert f"## Rules that said nothing, {len(rules.COMMITTED)}" in text


# ---------------------------------------------------------------------------
# writing


def test_write_all_writes_both_files_and_returns_them(atlas, tmp_path):
    written = write_all(atlas, out_dir=tmp_path, formats=["md"])

    assert [item.rsplit("/", 1)[-1] for item in written] == [REPORT_NAME, DRIFT_NAME]
    assert (tmp_path / REPORT_NAME).read_text().startswith("# Atlas report")
    assert (tmp_path / DRIFT_NAME).read_text().startswith("# Drift")


def test_a_rendered_file_names_its_atlas_so_a_stale_one_is_found(atlas, tmp_path):
    """A report left from an earlier extract described 172 entities beside an atlas of 689."""
    from rosio.render import stale

    write_all(atlas, out_dir=tmp_path, formats=["md", "html"])
    for name in (REPORT_NAME, DRIFT_NAME, "atlas.html"):
        assert f"Rendered from atlas {atlas.content_hash()}" in (tmp_path / name).read_text()
    assert stale(tmp_path, atlas) == []
    atlas.findings = []
    assert stale(tmp_path, atlas) == [REPORT_NAME, DRIFT_NAME, "atlas.html"]


def test_an_absent_html_renderer_is_left_out_rather_than_claimed(atlas, tmp_path, monkeypatch):
    """The html page is a separate module. Missing it must not fail the markdown."""
    monkeypatch.delattr(render, "html", raising=False)
    monkeypatch.setitem(sys.modules, "rosio.render.html", None)

    written = write_all(atlas, out_dir=tmp_path, formats=["md", "html"])

    assert all(item.endswith(".md") for item in written)


def test_the_html_renderer_is_used_when_it_is_there(atlas, tmp_path, monkeypatch):
    page = types.ModuleType("rosio.render.html")
    page.write = lambda _atlas, out: Path(out) / "atlas.html"
    monkeypatch.setattr(render, "html", page, raising=False)
    monkeypatch.setitem(sys.modules, "rosio.render.html", page)

    written = write_all(atlas, out_dir=tmp_path, formats=["md", "html"])

    assert written[-1].endswith("atlas.html")


def test_every_file_ends_with_one_newline(atlas):
    for text in (render_report(atlas), render_drift(atlas)):
        assert text.endswith("\n")
        assert not text.endswith("\n\n")


# ---------------------------------------------------------------------------
# the rules a gate held back


GATED_PROFILE = "profile:demo"


def gated() -> model.Atlas:
    """An atlas whose only launch file was refused, which is the shape that gates
    `plugin.unused`: nothing states which classes a profile selects."""
    builder = model.AtlasBuilder()
    builder.add_profile("demo")
    builder.node(
        ids.build_launchfile("demo_bringup", "launch/demo.launch.py"),
        attrs={"loadable": False},
    )
    atlas = builder.build()
    atlas.counters["drift"] = {
        "rules_run": 31,
        "rules_failed": 0,
        "waived": 0,
        "failed": [],
        "by_rule": {},
    }
    return atlas


def silence_rows(atlas: model.Atlas) -> list[dict]:
    from rosio.drift import params as params_rules
    from rosio.drift import plugins as plugin_rules

    return params_rules.silences(atlas) + plugin_rules.silences(atlas)


def quiet_row(text: str, rule: str) -> str:
    tail = text[text.index("## Rules that said nothing") :]
    return next(line for line in tail.splitlines() if line.startswith(f"| {rule} |"))


def test_a_rule_a_gate_held_back_is_never_rendered_as_a_rule_that_matched_nothing():
    """The two silences are opposite claims. A reader who takes one for the other reads a
    question nobody answered as a clean result, which is the defect this product reports."""
    atlas = gated()
    held = {row["rule"] for row in silence_rows(atlas)}
    assert held

    text = render_drift(atlas)

    for rule in sorted(held):
        assert "ran over this atlas and matched nothing" not in quiet_row(text, rule)
        assert "a gate held it back" in quiet_row(text, rule)


def test_the_gate_table_names_the_rule_the_profile_and_the_reason():
    text = render_drift(gated())

    section = text[text.index("## Rules a gate held back") : text.index("## Rules that said")]
    assert "| rule | profile | what the gate held back |" in section
    assert "| plugin.unused | demo |" in section
    assert "launch/demo.launch.py" in section
    assert "Nothing here was disabled" not in text


def test_the_header_and_the_heading_count_the_same_unit(atlas, monkeypatch):
    """One family emits a row per gated block per profile, so the heading counted 4 while
    the header counted 2 in one file. Both were true and they read as a contradiction."""
    from rosio.drift import params as params_rules

    rows = [
        {"rule": rule, "profile": profile, "reason": f"{block} was not read"}
        for rule in ("param.orphan_block", "param.owner_name_mismatch")
        for profile, block in ((REAL, "map_saver"), (SIM, "robot_state_publisher"))
    ]
    monkeypatch.setattr(params_rules, "silences", lambda _atlas: rows)

    text = render_drift(atlas)
    head = text[: text.index("## ")]

    assert "A gate held 2 rules back over part of this atlas" in head
    assert "## Rules a gate held back, 2" in text
    assert "2 rules over 4 rows" in text
    section = text[text.index("## Rules a gate held back") : text.index("## Rules that said")]
    assert section.count("| param.orphan_block | ") == 2


def test_a_report_over_an_atlas_no_gate_touched_says_that_much(atlas):
    text = render_drift(atlas)

    assert not silence_rows(atlas)
    assert "## Rules a gate held back, 0" in text
    assert "No gate held any rule back on this atlas" in text
    assert "param.type_changed | error | ran over this atlas and matched nothing" in text


def test_a_rule_that_fired_and_was_also_gated_is_named_at_the_top(atlas, monkeypatch):
    """A gate on one profile and findings on another is a lower bound, and a reader who
    stops at the rule table above sees a count that is not the whole story."""
    from rosio.drift import params as params_rules

    monkeypatch.setattr(
        params_rules,
        "silences",
        lambda _atlas: [
            {"rule": "param.shadowed", "profile": SIM, "reason": "the sim tree was not read"}
        ],
    )

    text = render_drift(atlas)
    head = text[: text.index("## ")]

    assert "A gate held 1 rule back over part of this atlas: param.shadowed." in head
    assert "| param.shadowed | nx_sim | the sim tree was not read |" in text
    assert "param.shadowed" not in text[text.index("## Rules that said nothing") :]


def test_a_gate_that_holds_for_the_whole_atlas_is_not_printed_as_a_profile(atlas, monkeypatch):
    from rosio.drift import plugins as plugin_rules

    monkeypatch.setattr(
        plugin_rules,
        "silences",
        lambda _atlas: [
            {"rule": "plugin.unresolved", "profile": "", "reason": "no manifest was read"}
        ],
    )

    text = render_drift(atlas)

    assert "| plugin.unresolved | the whole atlas | no manifest was read |" in text
    assert "a gate held it back over the whole atlas" in quiet_row(text, "plugin.unresolved")


def test_the_gate_rows_are_read_from_the_families_and_not_restated_here(atlas, monkeypatch):
    """One reason, written by the family that owns the rule. A renderer that rewords it
    becomes a second place where a gate is described."""
    from rosio.drift import params as params_rules

    reason = "the launch tree of this profile was not read to the end (demo_bringup/a.launch.py)"
    monkeypatch.setattr(
        params_rules,
        "silences",
        lambda _atlas: [{"rule": "param.orphan_block", "profile": REAL, "reason": reason}],
    )

    assert reason in render_drift(atlas)


def test_two_renders_of_a_gated_atlas_are_byte_identical():
    assert render_drift(gated()) == render_drift(gated())
