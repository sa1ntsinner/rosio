"""Entry point recovery from the things that are not launch files."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest

from rosio import ids, model, schema
from rosio.collect import CollectContext, entrypoints
from rosio.config import Config, RootSpec
from rosio.errors import UsageError
from rosio.resolve import py_pipeline, records
from rosio.workspace import Workspace

LAUNCHER = "tools/launcher.py"


@pytest.fixture
def launch_ws(fixtures_dir: Path) -> Path:
    root = fixtures_dir / "ws_launch"
    if not (root / LAUNCHER).is_file():
        pytest.skip("the launch fixture workspace is not present")
    return root


@pytest.fixture
def ws(launch_ws: Path) -> Workspace:
    return Workspace.discover(
        [
            RootSpec(kind="src", path=str(launch_ws)),
            RootSpec(kind="install", path=str(launch_ws / "install")),
        ]
    )


def _scrape(tmp_path: Path, name: str, text: str):
    target = tmp_path / name
    target.write_text(text, encoding="utf-8")
    return entrypoints.scrape_file(str(target))


# ---------------------------------------------------------------------------
# the python launcher fixture


def test_launcher_fixture_recovers_every_invocation(launch_ws: Path):
    result = entrypoints.scrape_file(str(launch_ws / LAUNCHER))
    assert result.parse_error == ""
    rows = [(entry.verb, entry.pkg, entry.target) for entry in result.entrypoints]
    assert rows == [
        ("launch", "lx_bringup", "nav_stack.launch.py"),
        ("launch", "lx_sensors", "sensors_only.launch.py"),
        ("launch", "lx_bringup", "opaque_probe.launch.py"),
        ("launch", "lx_bringup", "absent_profile.launch.py"),
        ("run", "lx_sensors", "health_monitor"),
        ("run", "lx_sensors", "scan_relay"),
    ]


def test_launcher_fixture_argument_recovery(launch_ws: Path):
    result = entrypoints.scrape_file(str(launch_ws / LAUNCHER))
    by_target = {entry.target: entry for entry in result.entrypoints}
    navigation = by_target["nav_stack.launch.py"]
    assert dict(navigation.args)["profile"] == "navigation_zoo", "a module level constant"
    assert dict(navigation.args)["params_file"] == entrypoints.UNRESOLVED
    assert navigation.confidence == "unknown"
    sensors = by_target["sensors_only.launch.py"]
    assert sensors.args == (("namespace", "sensors"),), "a parameter default"
    assert sensors.confidence == "inferred"
    relay = by_target["scan_relay"]
    assert relay.argv == ("--ros-args", "-r", "input:=/points", "-r", "__ns:=/prep")
    health = by_target["health_monitor"]
    assert "--params-file" in health.argv
    assert "profile:=navigation_zoo" in health.argv


def test_launcher_fixture_sites_are_stable(launch_ws: Path):
    result = entrypoints.scrape_file(str(launch_ws / LAUNCHER))
    entry = next(item for item in result.entrypoints if item.target == "absent_profile.launch.py")
    assert entry.site is not None
    assert entry.site.locator == "missing_command:ros2 launch lx_bringup absent_profile.launch.py"
    assert entry.site.line_hint is not None


def test_module_docstring_prose_is_not_a_command(launch_ws: Path):
    """The fixture docstring talks about ros2 launch, which is not an invocation."""
    result = entrypoints.scrape_file(str(launch_ws / LAUNCHER))
    assert all(entry.pkg != "or" for entry in result.entrypoints)


# ---------------------------------------------------------------------------
# shells, units and makefiles


def test_shell_script_with_continuations_and_operators(tmp_path: Path):
    result = _scrape(
        tmp_path,
        "bringup.sh",
        "#!/bin/bash\n"
        "# ros2 launch commented_out never.launch.py\n"
        "ros2 launch lx_bringup nav_stack.launch.py \\\n"
        "    params_file:=/etc/params.yaml use_sim_time:=false &\n"
        "sleep 2 && ros2 run lx_sensors health_monitor --ros-args -p window_sec:=2.0\n",
    )
    rows = [(entry.verb, entry.pkg, entry.target) for entry in result.entrypoints]
    assert rows == [
        ("launch", "lx_bringup", "nav_stack.launch.py"),
        ("run", "lx_sensors", "health_monitor"),
    ]
    first, second = result.entrypoints
    assert first.args == (("params_file", "/etc/params.yaml"), ("use_sim_time", "false"))
    assert first.argv == ()
    assert second.argv == ("--ros-args", "-p", "window_sec:=2.0")
    assert first.confidence == "resolved"


def test_systemd_unit(tmp_path: Path):
    result = _scrape(
        tmp_path,
        "ant-nav.service",
        "[Unit]\nDescription=nav\n\n[Service]\n"
        "ExecStart=/opt/ros/humble/bin/ros2 launch lx_bringup nav_stack.launch.py profile:=yard\n"
        "ExecStopPost=/bin/true\n",
    )
    assert [entry.target for entry in result.entrypoints] == ["nav_stack.launch.py"]
    assert result.entrypoints[0].args == (("profile", "yard"),)


def test_makefile_recipe(tmp_path: Path):
    result = _scrape(
        tmp_path,
        "Makefile",
        "SHELL := /bin/bash\n\nnav:\n\tros2 launch lx_bringup nav_stack.launch.py profile:=yard\n"
        "\nsensors:\n\tros2 run lx_sensors scan_relay --ros-args -r input:=/points\n",
    )
    rows = [(entry.verb, entry.target) for entry in result.entrypoints]
    assert rows == [("launch", "nav_stack.launch.py"), ("run", "scan_relay")]


def test_two_invocations_in_one_line(tmp_path: Path):
    result = _scrape(
        tmp_path,
        "chain.sh",
        "ros2 launch a one.launch.py x:=1 && ros2 launch b two.launch.py y:=2\n",
    )
    assert [(entry.pkg, entry.target, entry.args) for entry in result.entrypoints] == [
        ("a", "one.launch.py", (("x", "1"),)),
        ("b", "two.launch.py", (("y", "2"),)),
    ]


def test_launch_files_are_left_to_the_launch_collector(launch_ws: Path):
    launch_file = launch_ws / "src/lx_bringup/launch/nav_stack.launch.py"
    assert not entrypoints.is_scrapeable(str(launch_file))
    assert not entrypoints.is_scrapeable("legacy_ros1.launch")
    assert entrypoints.is_scrapeable(str(launch_ws / LAUNCHER))
    assert entrypoints.is_scrapeable("deploy/robot/start_stack.sh")
    assert entrypoints.is_scrapeable("Makefile")
    assert not entrypoints.is_scrapeable("config/params.yaml")


# ---------------------------------------------------------------------------
# interpolation


def test_glued_interpolation_keeps_the_target_checkable(tmp_path: Path):
    result = _scrape(
        tmp_path,
        "glue.py",
        "def cmd(map_arg, rviz_arg):\n"
        '    return f"ros2 launch lx_bringup nav_stack.launch.py{map_arg}{rviz_arg}"\n',
    )
    entry = result.entrypoints[0]
    assert entry.target == "nav_stack.launch.py"
    assert entry.argv and entrypoints.UNRESOLVED in entry.argv[0]
    assert entry.confidence == "unknown"


def test_shell_expansion_is_not_reported_as_resolved(tmp_path: Path):
    result = _scrape(
        tmp_path,
        "expand.sh",
        "ros2 launch lx_bringup nav_stack.launch.py params_file:=$REPO/params.yaml\n",
    )
    assert result.entrypoints[0].confidence == "inferred"


def test_unrecovered_prefix_glued_onto_the_command(tmp_path: Path):
    """The three Gazebo profiles are built as f"{gl_prefix}ros2 launch ...", so the command
    word arrives glued to whatever the shell prefix holds."""
    result = _scrape(
        tmp_path,
        "prefix.py",
        "def cmd(opts, use_rviz):\n"
        "    gl_prefix = _probe(opts)\n"
        '    return (f"{gl_prefix}"\n'
        '            f"ros2 launch lx_bringup nav_stack.launch.py "\n'
        '            f"use_rviz:={use_rviz}")\n',
    )
    assert [(entry.verb, entry.pkg, entry.target) for entry in result.entrypoints] == [
        ("launch", "lx_bringup", "nav_stack.launch.py")
    ]
    assert result.entrypoints[0].confidence == "unknown"


def test_command_wrapped_in_bash_c(tmp_path: Path):
    """A `bash -c '...'` payload is one shlex token holding a whole command line."""
    result = _scrape(
        tmp_path,
        "wrapped.py",
        'CMD = ("bash -c \'"\n'
        '       "python3 gate.py 120 && "\n'
        '       "ros2 run lx_sensors scan_relay -d /etc/view.rviz"\n'
        '       "\'")\n',
    )
    assert [(entry.verb, entry.pkg, entry.target) for entry in result.entrypoints] == [
        ("run", "lx_sensors", "scan_relay")
    ]
    assert result.entrypoints[0].argv == ("-d", "/etc/view.rviz")


def test_plain_string_command_in_python(tmp_path: Path):
    result = _scrape(
        tmp_path,
        "plain.py",
        'CMD = "ros2 run lx_sensors scan_relay --ros-args -r input:=/points"\n',
    )
    assert [(entry.verb, entry.target) for entry in result.entrypoints] == [("run", "scan_relay")]


def test_broken_python_is_reported_not_raised(tmp_path: Path):
    result = _scrape(tmp_path, "broken.py", "def f(:\n    pass\n")
    assert result.entrypoints == ()
    assert "SyntaxError" in result.parse_error


def test_a_file_too_large_to_read_is_reported(tmp_path: Path, monkeypatch):
    monkeypatch.setattr(entrypoints, "MAX_BYTES", 8)
    result = _scrape(tmp_path, "big.sh", "ros2 launch lx_bringup nav_stack.launch.py\n")
    assert result.entrypoints == ()
    assert "larger than" in result.parse_error


def test_as_command_round_trips(launch_ws: Path):
    result = entrypoints.scrape_file(str(launch_ws / LAUNCHER))
    entry = next(item for item in result.entrypoints if item.target == "sensors_only.launch.py")
    assert (
        entrypoints.as_command(entry)
        == "ros2 launch lx_sensors sensors_only.launch.py namespace:=sensors"
    )


# ---------------------------------------------------------------------------
# target resolution


def test_target_file_finds_a_launch_file_and_an_executable(ws: Workspace, launch_ws: Path):
    result = entrypoints.scrape_file(str(launch_ws / LAUNCHER))
    by_target = {entry.target: entry for entry in result.entrypoints}
    found = entrypoints.target_file(ws, by_target["nav_stack.launch.py"])
    assert found is not None and found.endswith("launch/nav_stack.launch.py")
    assert entrypoints.target_file(ws, by_target["health_monitor"]) == "health_monitor"
    assert entrypoints.target_file(ws, by_target["absent_profile.launch.py"]) is None


# ---------------------------------------------------------------------------
# the collector


def _context(ws: Workspace) -> CollectContext:
    return CollectContext(
        ws=ws,
        cfg=Config(roots=tuple(RootSpec(kind=r.kind, path=r.path) for r in ws.roots), profiles=()),
        builder=model.AtlasBuilder(),
    )


def test_collector_records_the_launcher(ws: Workspace):
    ctx = _context(ws)
    entrypoints.EntrypointsCollector().collect(ctx)
    targets = {entry.target for entry in ctx.declared.scraped}
    assert "nav_stack.launch.py" in targets
    assert "absent_profile.launch.py" in targets
    atlas = ctx.builder.atlas
    launcher = [
        node
        for node in atlas.entities.values()
        if node.kind == "file" and node.id.endswith(LAUNCHER)
    ]
    assert len(launcher) == 1
    fact = launcher[0].facts[0]
    assert fact.layer == "declared"
    commands = [row["command"] for row in fact.values["entrypoints"]]
    assert "ros2 launch lx_bringup absent_profile.launch.py mode:=grid" in commands


def test_collector_notes_a_launch_file_that_does_not_exist(ws: Workspace):
    ctx = _context(ws)
    entrypoints.EntrypointsCollector().collect(ctx)
    notes = [
        note for node in ctx.builder.atlas.entities.values() for note in node.attrs.get("notes", [])
    ]
    assert any("absent_profile.launch.py" in note for note in notes)
    rows = [
        row
        for node in ctx.builder.atlas.entities.values()
        for fact in node.facts
        for row in fact.values.get("entrypoints", [])
    ]
    missing = [row for row in rows if row.get("target_missing")]
    assert [row["target"] for row in missing] == ["absent_profile.launch.py"]
    resolved = [row for row in rows if row.get("target_file")]
    assert any(row["target"] == "nav_stack.launch.py" for row in resolved)


def test_collector_binds_the_run_and_the_layer(ws: Workspace):
    ctx = _context(ws)
    entrypoints.EntrypointsCollector().collect(ctx)
    atlas = ctx.builder.atlas
    runs = [node for node in atlas.entities.values() if node.kind == "run"]
    assert len(runs) == 1
    assert atlas.layers["declared"].present
    assert atlas.layers["declared"].runs == [runs[0].id]
    assert ids.parse_run(runs[0].id).layer == "declared"


def test_collector_output_validates(ws: Workspace):
    ctx = _context(ws)
    entrypoints.EntrypointsCollector().collect(ctx)
    assert schema.validate(ctx.builder.recompute().to_json()) == []


def test_collector_carries_a_profile_as_an_id(ws: Workspace):
    """A run and a fact slot take a profile id, not the bare slug the context holds."""
    ctx = _context(ws)
    ctx.profile = "navigation_zoo"
    entrypoints.EntrypointsCollector().collect(ctx)
    atlas = ctx.builder.atlas
    run = next(node for node in atlas.entities.values() if node.kind == "run")
    assert ids.parse_run(run.id).profile == "navigation_zoo"
    slots = {fact.profile for node in atlas.entities.values() for fact in node.facts}
    assert slots == {ids.build_profile("navigation_zoo")}


def test_collector_is_deterministic(ws: Workspace):
    first = _context(ws)
    entrypoints.EntrypointsCollector().collect(first)
    second = _context(ws)
    entrypoints.EntrypointsCollector().collect(second)
    first.builder.recompute()
    second.builder.recompute()
    assert first.builder.atlas.canonical_bytes() == second.builder.atlas.canonical_bytes()


def test_collector_is_registered():
    from rosio import collect

    collector = collect.get(entrypoints.NAME)
    assert collector.layer == "declared"


def test_a_root_with_no_file_is_absent_inputs_and_never_a_refusal(tmp_path: Path):
    """A configured root holding no file is a workspace where nothing is wrong.

    Raising here recorded the pass as refused, which fires `extract.pass_incomplete` at
    error and turns a green gate red on a repository that lost nothing. The answer is the
    absent inputs state, stated before the pass is attempted.
    """
    (tmp_path / "src").mkdir()
    ws = Workspace.discover([RootSpec(kind="src", path=str(tmp_path / "src"))])
    assert not ws.files()
    ctx = _context(ws)
    collector = entrypoints.EntrypointsCollector()

    assert collector.inputs_present(ctx) == entrypoints.NO_FILE
    collector.collect(ctx)
    assert ctx.builder.atlas.entities == {}, "a collector with nothing to read states nothing"


def test_a_root_with_one_file_has_inputs(ws: Workspace):
    assert entrypoints.EntrypointsCollector().inputs_present(_context(ws)) is True


# ---------------------------------------------------------------------------
# parameter pipelines

PATHS_MODULE = """
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
BASE = REPO / "config/nav2_params.yaml"
OVERLAY = REPO / "config/overrides.yaml"
BT_MAIN = REPO / "behavior_trees/nav_to_pose.xml"
"""

LAUNCHER_MODULE = """
import tempfile

import yaml

from .paths import BASE, BT_MAIN, OVERLAY


def _blend(base, overlay):
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _blend(base[key], value)
        else:
            base[key] = value
    return base


def nav_command():
    with open(BASE) as handle:
        params = yaml.safe_load(handle) or {}
    with open(OVERLAY) as extra:
        overlay = yaml.safe_load(extra) or {}
    _blend(params, overlay)
    bt = params.setdefault("bt_navigator", {}).setdefault("ros__parameters", {})
    bt["default_nav_to_pose_bt_xml"] = str(BT_MAIN)
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix="_nav.yaml", delete=False, dir="/tmp")
    yaml.safe_dump(params, tmp)
    params_file = tmp.name
    return f"ros2 launch lx_bringup nav_stack.launch.py params_file:={params_file}"


def plain_command():
    return "ros2 launch lx_bringup nav_stack.launch.py params_file:=/etc/params.yaml"
"""

PACKAGE_XML = (
    '<?xml version="1.0"?>\n<package format="3">\n  <name>lx_bringup</name>\n'
    "  <version>0.0.1</version>\n  <description>bench</description>\n"
    '  <maintainer email="a@b.c">bench</maintainer>\n  <license>Apache-2.0</license>\n'
    "</package>\n"
)


@pytest.fixture
def pipeline_ws(tmp_path: Path) -> Path:
    (tmp_path / ".git").mkdir()
    package = tmp_path / "src" / "lx_bringup"
    (package / "launch").mkdir(parents=True)
    (package / "package.xml").write_text(PACKAGE_XML, encoding="utf-8")
    (package / "launch" / "nav_stack.launch.py").write_text("# launch\n", encoding="utf-8")
    app = tmp_path / "app"
    app.mkdir()
    (app / "__init__.py").write_text("", encoding="utf-8")
    (app / "paths.py").write_text(PATHS_MODULE, encoding="utf-8")
    (app / "launcher.py").write_text(LAUNCHER_MODULE, encoding="utf-8")
    return tmp_path


def _launcher_pipelines(root: Path):
    return entrypoints.pipelines_in(str(root / "app" / "launcher.py"), "app/launcher.py")


def test_pipelines_in_reads_the_launcher_module(pipeline_ws: Path):
    pipelines = _launcher_pipelines(pipeline_ws)
    assert [pipeline.applied_by for pipeline in pipelines] == ["launcher.nav_command"]
    pipeline = pipelines[0]
    assert pipeline.base == "config/nav2_params.yaml"
    assert pipeline.overlays == ("config/overrides.yaml",)
    assert pipeline.overrides[0].value == "behavior_trees/nav_to_pose.xml"


def test_pipelines_in_ignores_anything_that_is_not_python(tmp_path: Path):
    target = tmp_path / "run.sh"
    target.write_text("ros2 launch lx_bringup nav_stack.launch.py\n", encoding="utf-8")
    assert entrypoints.pipelines_in(str(target)) == ()


def test_a_pipeline_binds_to_the_invocation_it_feeds(pipeline_ws: Path):
    result = entrypoints.scrape_file(str(pipeline_ws / "app" / "launcher.py"))
    bound = entrypoints.attach(result.entrypoints, _launcher_pipelines(pipeline_ws))
    assert len(bound) == 1
    assert entrypoints.entrypoint_of(bound[0]).startswith("ros2 launch lx_bringup")
    fed = {
        entry.site.locator.split(":")[0]: entrypoints.pipelines_for(entry, bound)
        for entry in result.entrypoints
    }
    assert fed["nav_command"] == bound
    assert fed["plain_command"] == (), "a literal argument is not fed by a pipeline"


TWO_PIPELINES = """
import tempfile

import yaml


def both():
    with open("config/one.yaml") as first_handle:
        one = yaml.safe_load(first_handle)
    one["n"]["p"] = 1
    first_tmp = tempfile.NamedTemporaryFile(mode="w", suffix="_one.yaml", delete=False)
    yaml.safe_dump(one, first_tmp)
    first = first_tmp.name
    with open("config/two.yaml") as second_handle:
        two = yaml.safe_load(second_handle)
    two["n"]["p"] = 2
    second_tmp = tempfile.NamedTemporaryFile(mode="w", suffix="_two.yaml", delete=False)
    yaml.safe_dump(two, second_tmp)
    second = second_tmp.name
    head = f"ros2 launch lx_bringup nav_stack.launch.py params_file:={first}"
    tail = f"ros2 launch lx_bringup other.launch.py params_file:={second}"
    return head + " && " + tail
"""


def test_two_pipelines_in_one_function_do_not_feed_each_other(tmp_path: Path):
    target = tmp_path / "two.py"
    target.write_text(TWO_PIPELINES, encoding="utf-8")
    result = entrypoints.scrape_file(str(target))
    bound = entrypoints.attach(result.entrypoints, entrypoints.pipelines_in(str(target), "two.py"))
    fed = {
        entry.target: [pipeline.base for pipeline in entrypoints.pipelines_for(entry, bound)]
        for entry in result.entrypoints
    }
    assert fed == {
        "nav_stack.launch.py": ["config/one.yaml"],
        "other.launch.py": ["config/two.yaml"],
    }


def test_pipeline_row_never_states_a_temp_path(pipeline_ws: Path):
    row = entrypoints.pipeline_row(_launcher_pipelines(pipeline_ws)[0])
    assert row["argument"] == "params_file"
    assert row["base"] == "config/nav2_params.yaml"
    assert row["applied_by"] == "launcher.nav_command"
    assert row["overrides"][0]["key"] == "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml"
    assert not any(str(value).startswith("/tmp/") for value in row.values())


def test_pipeline_row_carries_the_sink_binding(pipeline_ws: Path):
    row = entrypoints.pipeline_row(_launcher_pipelines(pipeline_ws)[0])
    assert row["sink"] == {
        "argument": "params_file",
        "target": "lx_bringup nav_stack.launch.py",
    }
    assert row["function"] == "nav_command"


def test_the_absolute_form_sits_beside_the_relative_one(pipeline_ws: Path):
    row = entrypoints.pipeline_row(_launcher_pipelines(pipeline_ws)[0], str(pipeline_ws))
    assert row["base"] == "config/nav2_params.yaml"
    volatile = row["volatile"]
    assert volatile["base_absolute"] == f"{pipeline_ws}/config/nav2_params.yaml"
    assert volatile["overlays_absolute"] == [f"{pipeline_ws}/config/overrides.yaml"]
    written = volatile["override_values_absolute"]
    key = "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml"
    assert written[key] == f"{pipeline_ws}/behavior_trees/nav_to_pose.xml"


def test_the_absolute_form_never_enters_the_compared_bytes(pipeline_ws: Path):
    row = entrypoints.pipeline_row(_launcher_pipelines(pipeline_ws)[0], str(pipeline_ws))
    assert "volatile" not in model.strip_volatile(row)


GUARDED = """
import tempfile

import yaml


def build(opts):
    with open("config/base.yaml") as handle:
        params = yaml.safe_load(handle)
    params["n"]["ros__parameters"]["always"] = 1
    if opts.get("commissioned"):
        params["n"]["ros__parameters"]["sometimes"] = 2
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix="_n.yaml", delete=False)
    yaml.safe_dump(params, tmp)
    out = tmp.name
    return f"ros2 launch lx_bringup nav_stack.launch.py params_file:={out}"
"""


def test_a_guarded_write_cannot_be_read_as_live_configuration(tmp_path: Path):
    target = tmp_path / "guarded.py"
    target.write_text(GUARDED, encoding="utf-8")
    row = entrypoints.pipeline_row(entrypoints.pipelines_in(str(target), "guarded.py")[0])
    by_key = {item["key"]: item for item in row["overrides"]}
    guarded = by_key["n.ros__parameters.sometimes"]
    assert guarded["conditional"] is True
    assert guarded["resolved"] is False
    assert guarded["value"] is None
    assert guarded["value_when_guard_holds"] == 2
    assert guarded["guard"] == "opts.get('commissioned')"
    plain = by_key["n.ros__parameters.always"]
    assert plain["conditional"] is False
    assert plain["resolved"] is True
    assert plain["value"] == 1


SAME_ARGUMENT = """
import tempfile

import yaml


def stock():
    with open("config/one.yaml") as handle:
        doc = yaml.safe_load(handle)
    doc["a"] = 1
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix="_one.yaml", delete=False)
    yaml.safe_dump(doc, tmp)
    out = tmp.name
    return f"ros2 launch lx_bringup nav_stack.launch.py params_file:={out}"


def house():
    with open("config/two.yaml") as handle:
        doc = yaml.safe_load(handle)
    doc["a"] = 2
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix="_two.yaml", delete=False)
    yaml.safe_dump(doc, tmp)
    out = tmp.name
    return f"ros2 launch lx_bringup other.launch.py params_file:={out}"
"""


def test_two_pipelines_in_one_module_bind_to_one_invocation_each(tmp_path: Path):
    target = tmp_path / "both.py"
    target.write_text(SAME_ARGUMENT, encoding="utf-8")
    result = entrypoints.scrape_file(str(target))
    built = entrypoints.pipelines_in(str(target), "both.py")
    fed = {
        entry.target: [pipeline.base for pipeline in entrypoints.pipelines_for(entry, built)]
        for entry in result.entrypoints
    }
    assert fed == {
        "nav_stack.launch.py": ["config/one.yaml"],
        "other.launch.py": ["config/two.yaml"],
    }


def test_a_pipeline_refuses_an_invocation_its_command_does_not_name(tmp_path: Path):
    target = tmp_path / "both.py"
    target.write_text(SAME_ARGUMENT, encoding="utf-8")
    built = entrypoints.pipelines_in(str(target), "both.py")
    stock = next(item for item in built if item.base == "config/one.yaml")
    stranger = records.ScrapedEntrypoint(
        verb="launch",
        pkg="lx_bringup",
        target="other.launch.py",
        args=(("params_file", entrypoints.UNRESOLVED),),
        site=records.Site(path=str(target), locator="stock:ros2 launch lx_bringup other.launch.py"),
    )
    assert entrypoints.pipelines_for(stranger, [stock]) == ()


INTERPOLATED_TARGET = """
import tempfile

import yaml


def stock(pkg):
    with open("config/one.yaml") as handle:
        doc = yaml.safe_load(handle)
    doc["a"] = 1
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix="_one.yaml", delete=False)
    yaml.safe_dump(doc, tmp)
    out = tmp.name
    return f"ros2 launch {pkg} nav_stack.launch.py params_file:={out}"
"""


def test_a_pipeline_that_could_not_read_its_command_still_binds(tmp_path: Path):
    target = tmp_path / "interpolated.py"
    target.write_text(INTERPOLATED_TARGET, encoding="utf-8")
    built = entrypoints.pipelines_in(str(target), "interpolated.py")
    assert py_pipeline.sink_target(built[0]) == ""
    assert not built[0].complete, "the command it feeds is a blind spot, not a guess"
    entry = records.ScrapedEntrypoint(
        verb="launch",
        pkg="lx_bringup",
        target="nav_stack.launch.py",
        args=(("params_file", entrypoints.UNRESOLVED),),
        site=records.Site(path=str(target), locator="stock:ros2 launch lx_bringup nav_stack.py"),
    )
    assert entrypoints.pipelines_for(entry, built) == tuple(built)


# ---------------------------------------------------------------------------
# the config escape hatch


DECLARED = {
    "applied_by": "launcher.nav_command",
    "argument": "params_file",
    "source": "app/launcher.py",
    "base": "config/nav2_params.yaml",
    "overlays": ["config/overrides.yaml"],
    "overrides": [
        {"key": "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml", "value": "trees/a.xml"},
        {"key": "bt_navigator.ros__parameters.deadline", "expr": "opts['deadline']"},
    ],
}


def test_a_declared_pipeline_takes_the_analysed_shape():
    pipeline = entrypoints.pipeline_from_mapping(DECLARED)
    assert pipeline.output_token == "params_file"
    assert pipeline.base == "config/nav2_params.yaml"
    assert pipeline.overlays == ("config/overrides.yaml",)
    first, second = pipeline.overrides
    assert first.key_path == ("bt_navigator", "ros__parameters", "default_nav_to_pose_bt_xml")
    assert first.value == "trees/a.xml" and first.resolved
    assert second.value is None and not second.resolved
    assert second.expr == "opts['deadline']"
    assert not pipeline.complete, "an unresolved write is not a complete pipeline"


def test_a_declared_pipeline_resolves_its_paths_against_the_workspace(tmp_path: Path):
    pipeline = entrypoints.pipeline_from_mapping(DECLARED, str(tmp_path))
    assert pipeline.base == str(tmp_path / "config/nav2_params.yaml")
    assert pipeline.site is not None
    assert pipeline.site.path == str(tmp_path / "app/launcher.py")


def test_a_profile_carries_its_declared_pipelines():
    profile = SimpleNamespace(slug="navigation", pipelines=[DECLARED])
    declared = entrypoints.declared_pipelines(profile)
    assert len(declared) == 1
    assert declared[0].applied_by == "launcher.nav_command"
    assert entrypoints.declared_pipelines(SimpleNamespace(slug="none")) == ()


@pytest.mark.parametrize(
    "payload",
    [
        "not a mapping",
        {"argument": "params_file", "unknown": 1},
        {"base": "config/a.yaml"},
        {"argument": "params_file", "overrides": [{"value": 1}]},
    ],
)
def test_a_broken_declaration_is_a_usage_error(payload):
    with pytest.raises(UsageError):
        entrypoints.pipeline_from_mapping(payload)


# ---------------------------------------------------------------------------
# the collector side


@pytest.fixture
def pipeline_context(pipeline_ws: Path) -> CollectContext:
    workspace = Workspace.discover([RootSpec(kind="src", path=str(pipeline_ws))])
    return CollectContext(
        ws=workspace,
        cfg=Config(roots=(RootSpec(kind="src", path=str(pipeline_ws)),), profiles=()),
        builder=model.AtlasBuilder(),
    )


def test_the_collector_stores_pipelines_on_the_bundle(pipeline_context: CollectContext):
    entrypoints.EntrypointsCollector().collect(pipeline_context)
    pipelines = pipeline_context.declared.pipelines
    assert [pipeline.applied_by for pipeline in pipelines] == ["launcher.nav_command"]
    assert pipelines[0].base == "config/nav2_params.yaml"
    assert entrypoints.entrypoint_of(pipelines[0])


def test_the_collector_puts_the_pipeline_on_the_entry_point_row(pipeline_context: CollectContext):
    entrypoints.EntrypointsCollector().collect(pipeline_context)
    rows = [
        row
        for node in pipeline_context.builder.atlas.entities.values()
        for fact in node.facts
        for row in fact.values.get("entrypoints", [])
    ]
    fed = [row for row in rows if row.get("pipelines")]
    assert len(fed) == 1
    pipeline = fed[0]["pipelines"][0]
    assert pipeline["argument"] == "params_file"
    assert pipeline["overlays"] == ["config/overrides.yaml"]


def test_the_collector_output_still_validates_with_pipelines(pipeline_context: CollectContext):
    entrypoints.EntrypointsCollector().collect(pipeline_context)
    assert schema.validate(pipeline_context.builder.recompute().to_json()) == []


def test_the_collector_is_deterministic_with_pipelines(pipeline_ws: Path):
    def run() -> bytes:
        workspace = Workspace.discover([RootSpec(kind="src", path=str(pipeline_ws))])
        ctx = CollectContext(
            ws=workspace,
            cfg=Config(roots=(RootSpec(kind="src", path=str(pipeline_ws)),), profiles=()),
            builder=model.AtlasBuilder(),
        )
        entrypoints.EntrypointsCollector().collect(ctx)
        return ctx.builder.recompute().canonical_bytes()

    assert run() == run()


SPLIT_DOCUMENT = """
import tempfile

import yaml


def merged():
    with open("config/split.yaml") as handle:
        params = yaml.safe_load(handle)
    params["n"]["ros__parameters"]["depth"] = 7
    with open("run/merged.yaml", "w") as out:
        yaml.safe_dump(params, out)
"""


def test_a_module_with_no_command_string_still_contributes(pipeline_ws: Path):
    (pipeline_ws / "app" / "documents.py").write_text(SPLIT_DOCUMENT, encoding="utf-8")
    workspace = Workspace.discover([RootSpec(kind="src", path=str(pipeline_ws))])
    ctx = CollectContext(
        ws=workspace,
        cfg=Config(roots=(RootSpec(kind="src", path=str(pipeline_ws)),), profiles=()),
        builder=model.AtlasBuilder(),
    )
    entrypoints.EntrypointsCollector().collect(ctx)
    built = [item for item in ctx.declared.pipelines if item.applied_by == "documents.merged"]
    assert len(built) == 1, "a launcher split over two modules must not lose the document"
    assert built[0].base == "config/split.yaml"
    assert built[0].output_token == "run/merged.yaml"


UNFOLDABLE = """
import tempfile

import yaml


def build(choice):
    with open(choice.path) as handle:
        params = yaml.safe_load(handle)
    params["n"]["ros__parameters"]["depth"] = choice.depth
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix="_n.yaml", delete=False)
    yaml.safe_dump(params, tmp)
    out = tmp.name
    return f"ros2 launch lx_bringup nav_stack.launch.py params_file:={out}"
"""


def test_an_incomplete_pipeline_reaches_the_unresolved_table(pipeline_ws: Path):
    (pipeline_ws / "app" / "opaque.py").write_text(UNFOLDABLE, encoding="utf-8")
    workspace = Workspace.discover([RootSpec(kind="src", path=str(pipeline_ws))])
    ctx = CollectContext(
        ws=workspace,
        cfg=Config(roots=(RootSpec(kind="src", path=str(pipeline_ws)),), profiles=()),
        builder=model.AtlasBuilder(),
    )
    entrypoints.EntrypointsCollector().collect(ctx)
    atlas = ctx.builder.recompute()
    expressions = [entry.expr for entry in atlas.unresolved]
    assert any("choice.depth" in item for item in expressions)
    assert any("choice.path" in item for item in expressions)
    assert atlas.counters.get("resolved", {}).get("param_value", 0) >= 1
    assert schema.validate(atlas.to_json()) == []


def test_a_declared_profile_pipeline_reaches_the_bundle(pipeline_ws: Path):
    workspace = Workspace.discover([RootSpec(kind="src", path=str(pipeline_ws))])
    profile = SimpleNamespace(slug="navigation", pipelines=[DECLARED], entrypoints=())
    cfg = Config(roots=(RootSpec(kind="src", path=str(pipeline_ws)),), profiles=())
    object.__setattr__(cfg, "profiles", (profile,))
    ctx = CollectContext(ws=workspace, cfg=cfg, builder=model.AtlasBuilder(), profile="navigation")
    entrypoints.EntrypointsCollector().collect(ctx)
    applied = [pipeline.applied_by for pipeline in ctx.declared.pipelines]
    assert applied.count("launcher.nav_command") == 2, "the declared one and the analysed one"
