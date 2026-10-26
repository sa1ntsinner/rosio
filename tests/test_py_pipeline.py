"""Parameter documents a launcher module builds before launch ever sees one."""

from __future__ import annotations

from pathlib import Path

import pytest

from rosio.errors import CollectorError
from rosio.resolve import py_pipeline

PATHS = '''
"""Where the launcher keeps its paths."""
import os
from pathlib import Path

HOME = Path.home()
_env = os.environ.get("LX_REPO")
REPO = Path(_env).expanduser().resolve() if _env else Path(__file__).resolve().parents[1]
CONFIG = REPO / "config"
BASE = CONFIG / "nav2_params.yaml"
OVERLAY = CONFIG / "overrides.yaml"
TREES = REPO / "trees"
BT_MAIN = TREES / "nav_to_pose.xml"
ELSEWHERE = HOME / "private.yaml"
'''

LAUNCHER = '''
import tempfile

import yaml

from .paths import BASE, BT_MAIN, OVERLAY


def _fold(base, overlay):
    """A recursive two dict walk under a name nothing can match on."""
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _fold(base[key], value)
        else:
            base[key] = value
    return base


def nav_command(opts):
    with open(BASE) as handle:
        params = yaml.safe_load(handle) or {}
    with open(OVERLAY) as overlay_handle:
        overlay = yaml.safe_load(overlay_handle) or {}
    _fold(params, overlay)
    bt = params.setdefault("bt_navigator", {}).setdefault("ros__parameters", {})
    bt["default_nav_to_pose_bt_xml"] = str(BT_MAIN)
    bt["number_of_retries"] = 3
    bt["deadline"] = opts["deadline"]
    if opts.get("commissioning"):
        _slow(params)
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix="_nav.yaml", delete=False, dir="/tmp")
    yaml.safe_dump(params, tmp)
    tmp.flush()
    params_file = tmp.name
    return f"ros2 launch lx_bringup nav_stack.launch.py params_file:={params_file}"


def _slow(params):
    smoother = params["velocity_smoother"]["ros__parameters"]
    smoother.update({"max_velocity": [1.0, 0.0, 0.65], "max_accel": [0.3, 0.0, 0.3]})
'''


@pytest.fixture
def launcher(tmp_path: Path) -> Path:
    package = tmp_path / "app"
    package.mkdir()
    (package / "__init__.py").write_text("", encoding="utf-8")
    (package / "paths.py").write_text(PATHS, encoding="utf-8")
    target = package / "launcher.py"
    target.write_text(LAUNCHER, encoding="utf-8")
    return target


def _analyse(target: Path) -> py_pipeline.ModuleResult:
    return py_pipeline.analyse_file(str(target), "app/launcher.py")


def _by_key(pipeline, key: str):
    return next(item for item in pipeline.overrides if py_pipeline.dotted(item.key_path) == key)


# ---------------------------------------------------------------------------
# the whole shape


def test_the_pipeline_is_recovered(launcher: Path):
    result = _analyse(launcher)
    assert result.parse_error == ""
    assert len(result.pipelines) == 1
    pipeline = result.pipelines[0]
    assert pipeline.applied_by == "launcher.nav_command"
    assert pipeline.base == "config/nav2_params.yaml"
    assert pipeline.overlays == ("config/overrides.yaml",)
    assert pipeline.output_token == "params_file"
    assert py_pipeline.sink_argument(pipeline) == "params_file"
    assert py_pipeline.temp_suffix(pipeline) == "_nav.yaml"
    assert py_pipeline.sink_target(pipeline) == "lx_bringup nav_stack.launch.py"
    assert py_pipeline.sink_function(pipeline) == "nav_command"


TWO_PIPELINES = """
import tempfile

import yaml

BASE = "config/base.yaml"


def _write(doc, suffix):
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=suffix, delete=False)
    yaml.safe_dump(doc, tmp)
    return tmp.name


def build():
    plain = yaml.safe_load(open(BASE))
    plain["a"] = 1
    plain_tmp = tempfile.NamedTemporaryFile(mode="w", suffix="_plain.yaml", delete=False)
    yaml.safe_dump(plain, plain_tmp)
    plain_file = plain_tmp.name
    revised = yaml.safe_load(open(BASE))
    revised["a"] = 2
    revised_tmp = tempfile.NamedTemporaryFile(mode="w", suffix="_revised.yaml", delete=False)
    yaml.safe_dump(revised, revised_tmp)
    revised_file = revised_tmp.name
    first = f"ros2 launch stock nav.launch.py params_file:={plain_file}"
    second = f"ros2 launch house nav_revised.launch.py params_file:={revised_file}"
    return first, second
"""


def test_two_pipelines_in_one_module_are_told_apart_by_the_command(tmp_path: Path):
    target = tmp_path / "two.py"
    target.write_text(TWO_PIPELINES, encoding="utf-8")
    pipelines = py_pipeline.analyse_file(str(target), "two.py").pipelines
    assert len(pipelines) == 2
    arguments = {py_pipeline.sink_argument(item) for item in pipelines}
    assert arguments == {"params_file"}, "the argument name alone binds nothing"
    targets = sorted(py_pipeline.sink_target(item) for item in pipelines)
    assert targets == ["house nav_revised.launch.py", "stock nav.launch.py"]


def _one_command(tmp_path: Path, name: str, command: str, body: str = "    d['k'] = 1\n"):
    target = tmp_path / name
    target.write_text(
        "import tempfile\n"
        "import yaml\n"
        "BASE = 'config/base.yaml'\n"
        "def build(pkg, extra):\n"
        "    d = yaml.safe_load(open(BASE))\n" + body + "    tmp = tempfile.NamedTemporaryFile("
        "suffix='.yaml', delete=False)\n"
        "    yaml.safe_dump(d, tmp)\n"
        "    out = tmp.name\n"
        f"    return {command}\n",
        encoding="utf-8",
    )
    return py_pipeline.analyse_file(str(target), name).pipelines[0]


def test_the_command_that_takes_the_file_is_the_one_before_it(tmp_path: Path):
    second = _one_command(
        tmp_path,
        "second.py",
        "f'ros2 launch a first.launch.py && ros2 run b exe --params-file {out}'",
    )
    assert py_pipeline.sink_target(second) == "b exe"
    first = _one_command(
        tmp_path,
        "first.py",
        "f'ros2 launch a first.launch.py params_file:={out} && ros2 run b exe'",
    )
    assert py_pipeline.sink_target(first) == "a first.launch.py"


def test_an_interpolated_package_is_reported_not_invented(tmp_path: Path):
    pipeline = _one_command(
        tmp_path, "pkg.py", "f'ros2 launch {pkg} x.launch.py params_file:={out}'"
    )
    assert py_pipeline.sink_target(pipeline) == "", "half a name binds the wrong invocation"
    assert not pipeline.complete
    assert py_pipeline.sink_argument(pipeline) == "params_file"


def test_an_interpolated_launch_file_is_reported_not_invented(tmp_path: Path):
    pipeline = _one_command(tmp_path, "file.py", "f'ros2 launch pkg_a {extra} params_file:={out}'")
    assert py_pipeline.sink_target(pipeline) == ""
    assert not pipeline.complete


def test_a_shell_prefix_in_front_of_the_command_still_binds(tmp_path: Path):
    pipeline = _one_command(
        tmp_path, "prefix.py", "f'{extra}ros2 launch pkg_a x.launch.py params_file:={out}'"
    )
    assert py_pipeline.sink_target(pipeline) == "pkg_a x.launch.py"


def test_a_key_holding_the_guard_word_is_not_a_guarded_write(tmp_path: Path):
    pipeline = _one_command(
        tmp_path,
        "guardword.py",
        "f'ros2 launch pkg_a x.launch.py params_file:={out}'",
        body="    d['node.ros__parameters.thing if enabled'] = 20\n",
    )
    written = _by_key(pipeline, "node.ros__parameters.thing if enabled")
    assert py_pipeline.override_guard(written) == ""
    assert written.resolved and written.value == 20


def test_a_guard_holding_a_conditional_expression_survives_whole(tmp_path: Path):
    pipeline = _one_command(
        tmp_path,
        "ternary.py",
        "f'ros2 launch pkg_a x.launch.py params_file:={out}'",
        body="    if (1 if pkg else 2):\n        d['h'] = 6\n",
    )
    written = _by_key(pipeline, "h")
    assert py_pipeline.override_guard(written) == "1 if pkg else 2"
    assert not written.resolved


def test_text_glued_onto_a_path_still_folds(tmp_path: Path):
    target = tmp_path / "glue.py"
    target.write_text(
        "import tempfile\n"
        "import yaml\n"
        "from pathlib import Path\n"
        "BASE = 'config/base.yaml'\n"
        "TREE = Path('trees') / 'main.xml'\n"
        "def build():\n"
        "    params = yaml.safe_load(open(BASE))\n"
        "    params['backup'] = str(TREE) + '.bak'\n"
        "    params['tree'] = str(TREE)\n"
        "    tmp = tempfile.NamedTemporaryFile(suffix='.yaml', delete=False)\n"
        "    yaml.safe_dump(params, tmp)\n"
        "    out = tmp.name\n"
        "    return f'ros2 launch p f.launch.py params_file:={out}'\n",
        encoding="utf-8",
    )
    pipeline = py_pipeline.analyse_file(str(target), "glue.py").pipelines[0]
    assert _by_key(pipeline, "backup").value == "trees/main.xml.bak"
    assert py_pipeline.path_valued(pipeline) == ("tree",), "glued text is no longer a path"
    assert pipeline.complete


def test_a_path_valued_write_is_marked_as_a_path(tmp_path: Path):
    target = tmp_path / "bt.py"
    target.write_text(
        "import tempfile\n"
        "import yaml\n"
        "from pathlib import Path\n"
        "BASE = 'config/base.yaml'\n"
        "TREE = Path('trees') / 'main.xml'\n"
        "def build():\n"
        "    params = yaml.safe_load(open(BASE))\n"
        "    params['bt'] = str(TREE)\n"
        "    params['depth'] = 3\n"
        "    tmp = tempfile.NamedTemporaryFile(suffix='.yaml', delete=False)\n"
        "    yaml.safe_dump(params, tmp)\n"
        "    out = tmp.name\n"
        "    return f'ros2 launch p f.launch.py params_file:={out}'\n",
        encoding="utf-8",
    )
    pipeline = py_pipeline.analyse_file(str(target), "bt.py").pipelines[0]
    assert py_pipeline.path_valued(pipeline) == ("bt",)
    assert _by_key(pipeline, "bt").value == "trees/main.xml"
    assert _by_key(pipeline, "depth").value == 3


def test_constants_fold_through_the_sibling_module(launcher: Path):
    pipeline = _analyse(launcher).pipelines[0]
    written = _by_key(pipeline, "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml")
    assert written.value == "trees/nav_to_pose.xml"
    assert written.resolved
    assert written.expr == "str(BT_MAIN)", "the raw expression is kept whatever folded"


def test_every_write_is_recorded_with_its_key_path(launcher: Path):
    pipeline = _analyse(launcher).pipelines[0]
    keys = [py_pipeline.dotted(item.key_path) for item in pipeline.overrides]
    assert keys == [
        "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml",
        "bt_navigator.ros__parameters.number_of_retries",
        "bt_navigator.ros__parameters.deadline",
        "velocity_smoother.ros__parameters.max_velocity",
        "velocity_smoother.ros__parameters.max_accel",
    ]
    assert _by_key(pipeline, "bt_navigator.ros__parameters.number_of_retries").value == 3


def test_a_write_that_does_not_fold_is_unresolved_and_never_guessed(launcher: Path):
    pipeline = _analyse(launcher).pipelines[0]
    written = _by_key(pipeline, "bt_navigator.ros__parameters.deadline")
    assert written.value is None
    assert not written.resolved
    assert written.expr == "opts['deadline']"
    assert not py_pipeline.override_folded(written)
    assert pipeline.complete, "one unknown value is a blind spot on one key, not on the document"
    assert not any(note.startswith(py_pipeline.NOTE_UNRESOLVED) for note in pipeline.notes)


def test_a_write_whose_key_is_unknown_makes_the_document_incomplete():
    source = """
import tempfile
import yaml

def _fold(base, overlay):
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _fold(base[key], value)
        else:
            base[key] = value
    return base

def nav_command(opts):
    with open("config/nav2_params.yaml") as handle:
        params = yaml.safe_load(handle)
    params[opts["block"]]["ros__parameters"]["width"] = 40
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix="_nav.yaml", delete=False, dir="/tmp")
    yaml.safe_dump(params, tmp)
    return f"ros2 launch lx_bringup nav_stack.launch.py params_file:={tmp.name}"
"""
    pipeline = py_pipeline.analyse_source("app/launcher.py", source, "app/launcher.py").pipelines[0]
    assert not pipeline.complete, "the write could have landed anywhere in the document"
    assert any(note.startswith(py_pipeline.NOTE_UNRESOLVED) for note in pipeline.notes)


def test_a_conditional_write_carries_its_guard(launcher: Path):
    pipeline = _analyse(launcher).pipelines[0]
    written = _by_key(pipeline, "velocity_smoother.ros__parameters.max_velocity")
    assert py_pipeline.override_guard(written) == "opts.get('commissioning')"
    assert written.value == [1.0, 0.0, 0.65], "the value the write would put there"
    unconditional = _by_key(pipeline, "bt_navigator.ros__parameters.number_of_retries")
    assert py_pipeline.override_guard(unconditional) == ""


def test_a_guarded_write_is_never_live_configuration(launcher: Path):
    pipeline = _analyse(launcher).pipelines[0]
    written = _by_key(pipeline, "velocity_smoother.ros__parameters.max_velocity")
    assert not written.resolved, "reading value and resolved must not yield a live value"
    assert written.expr.endswith(" if opts.get('commissioning')")
    unconditional = _by_key(pipeline, "bt_navigator.ros__parameters.number_of_retries")
    assert unconditional.resolved


def test_a_guard_does_not_make_the_pipeline_incomplete(launcher: Path):
    pipeline = _analyse(launcher).pipelines[0]
    assert any(note.startswith(py_pipeline.NOTE_CONDITIONAL) for note in pipeline.notes)
    assert not any(
        "max_velocity" in note
        for note in pipeline.notes
        if note.startswith(py_pipeline.NOTE_UNRESOLVED)
    ), "the analyser saw the write, it is the condition that is open"


def test_a_write_helper_is_followed_and_keeps_its_own_site(launcher: Path):
    pipeline = _analyse(launcher).pipelines[0]
    written = _by_key(pipeline, "velocity_smoother.ros__parameters.max_accel")
    assert written.site is not None
    assert written.site.locator.startswith("_slow:velocity_smoother.ros__parameters.max_accel")


def test_the_environment_read_is_reported_not_invented(launcher: Path):
    result = _analyse(launcher)
    assert f"{py_pipeline.NOTE_ENV_UNSET}LX_REPO" in result.notes


def test_two_runs_agree(launcher: Path):
    first = _analyse(launcher)
    second = _analyse(launcher)
    assert first == second


# ---------------------------------------------------------------------------
# folding


def test_repo_paths_fold_relative_to_the_repository_root(launcher: Path):
    folded = py_pipeline.fold_constants(str(launcher.parent / "paths.py"), "app/paths.py")
    assert folded["BASE"] == "config/nav2_params.yaml"
    assert folded["REPO"] == ""
    assert "HOME" not in folded, "the home directory is a machine fact"
    assert "ELSEWHERE" not in folded


def test_an_absolute_literal_stays_absolute(tmp_path: Path):
    target = tmp_path / "paths.py"
    target.write_text(
        "from pathlib import Path\nROOT = Path('/etc/lx')\nFILE = ROOT / 'params.yaml'\n",
        encoding="utf-8",
    )
    folded = py_pipeline.fold_constants(str(target), "paths.py")
    assert folded["FILE"] == "/etc/lx/params.yaml"


def test_a_module_without_a_relative_path_cannot_anchor(launcher: Path):
    folded = py_pipeline.fold_constants(str(launcher.parent / "paths.py"))
    assert "BASE" not in folded, "parents[n] has nothing to count from"


def test_folding_a_missing_file_is_a_collector_error(tmp_path: Path):
    with pytest.raises(CollectorError):
        py_pipeline.fold_constants(str(tmp_path / "absent.py"), "absent.py")


def test_repo_path_never_climbs_above_the_repository_root():
    path = py_pipeline.RepoPath("app/paths.py")
    assert path.up(2) == py_pipeline.RepoPath("")
    assert path.up(3) is py_pipeline.UNKNOWN


# ---------------------------------------------------------------------------
# the merge helper, by shape


MERGE_SHAPES = [
    (
        "def merge_a(dst, src):\n"
        "    for key, value in src.items():\n"
        "        if isinstance(value, dict) and isinstance(dst.get(key), dict):\n"
        "            merge_a(dst[key], value)\n"
        "        else:\n"
        "            dst[key] = value\n"
        "    return dst\n"
    ),
    (
        "def _apply(base, layer):\n"
        "    for key in layer.keys():\n"
        "        if isinstance(layer[key], (dict,)) and key in base:\n"
        "            _apply(base[key], layer[key])\n"
        "            continue\n"
        "        base[key] = layer[key]\n"
    ),
]

NOT_MERGE = [
    "def plain(base, overlay):\n    base.update(overlay)\n    return base\n",
    "def walk(node):\n    for item in node.items():\n        walk(item)\n",
]


@pytest.mark.parametrize("text", MERGE_SHAPES)
def test_a_recursive_two_dict_walk_is_a_merge_helper(text: str):
    import ast

    assert py_pipeline.is_merge_helper(ast.parse(text).body[0])


@pytest.mark.parametrize("text", NOT_MERGE)
def test_a_shallow_update_is_not_a_merge_helper(text: str):
    import ast

    assert not py_pipeline.is_merge_helper(ast.parse(text).body[0])


def test_the_merge_helper_is_matched_under_any_name(tmp_path: Path):
    target = tmp_path / "other.py"
    target.write_text(
        "import tempfile\n"
        "import yaml\n"
        "BASE = 'config/base.yaml'\n"
        "EXTRA = 'config/extra.yaml'\n"
        "def stir(dst, src):\n"
        "    for key, value in src.items():\n"
        "        if isinstance(value, dict) and isinstance(dst.get(key), dict):\n"
        "            stir(dst[key], value)\n"
        "        else:\n"
        "            dst[key] = value\n"
        "    return dst\n"
        "def build():\n"
        "    params = yaml.safe_load(open(BASE))\n"
        "    extra = yaml.safe_load(open(EXTRA))\n"
        "    stir(params, extra)\n"
        "    tmp = tempfile.NamedTemporaryFile(suffix='.yaml', delete=False)\n"
        "    yaml.safe_dump(params, tmp)\n"
        "    out = tmp.name\n"
        "    return f'ros2 launch p f.launch.py params_file:={out}'\n",
        encoding="utf-8",
    )
    pipeline = py_pipeline.analyse_file(str(target), "other.py").pipelines[0]
    assert pipeline.base == "config/base.yaml"
    assert pipeline.overlays == ("config/extra.yaml",)
    assert pipeline.complete


# ---------------------------------------------------------------------------
# refusals


def test_an_unresolvable_base_is_reported_not_guessed(tmp_path: Path):
    target = tmp_path / "opaque.py"
    target.write_text(
        "import tempfile\n"
        "import yaml\n"
        "def build(choice):\n"
        "    params = yaml.safe_load(open(choice.path))\n"
        "    params['a'] = 1\n"
        "    tmp = tempfile.NamedTemporaryFile(suffix='.yaml', delete=False)\n"
        "    yaml.safe_dump(params, tmp)\n"
        "    out = tmp.name\n"
        "    return f'ros2 launch p f.launch.py params_file:={out}'\n",
        encoding="utf-8",
    )
    pipeline = py_pipeline.analyse_file(str(target), "opaque.py").pipelines[0]
    assert pipeline.base is None
    assert not pipeline.complete
    assert any("choice.path" in note for note in pipeline.notes)


def test_a_document_that_reaches_no_command_is_reported_not_dropped(tmp_path: Path):
    target = tmp_path / "settings.py"
    target.write_text(
        "import yaml\n"
        "SETTINGS = 'data/settings.yaml'\n"
        "def load():\n"
        "    data = yaml.safe_load(open(SETTINGS))\n"
        "    data['seen'] = True\n"
        "    return data\n",
        encoding="utf-8",
    )
    pipeline = py_pipeline.analyse_file(str(target), "settings.py").pipelines[0]
    assert not pipeline.complete
    assert pipeline.output_token == "load:data"
    assert f"{py_pipeline.NOTE_UNRESOLVED}the document is never dumped" in pipeline.notes


def test_a_plain_read_is_not_a_pipeline(tmp_path: Path):
    target = tmp_path / "registry.py"
    target.write_text(
        "import yaml\n"
        "REGISTRY = 'data/registry.yaml'\n"
        "def read():\n"
        "    return yaml.safe_load(open(REGISTRY)) or {}\n",
        encoding="utf-8",
    )
    assert py_pipeline.analyse_file(str(target), "registry.py").pipelines == ()


def test_a_dump_to_a_named_path_is_a_pipeline(tmp_path: Path):
    target = tmp_path / "named.py"
    target.write_text(
        "import yaml\n"
        "BASE = 'config/base.yaml'\n"
        "OUT = 'run/merged.yaml'\n"
        "def build():\n"
        "    params = yaml.safe_load(open(BASE))\n"
        "    params['use_sim_time'] = False\n"
        "    with open(OUT, 'w') as handle:\n"
        "        yaml.safe_dump(params, handle)\n",
        encoding="utf-8",
    )
    pipeline = py_pipeline.analyse_file(str(target), "named.py").pipelines[0]
    assert pipeline.complete, "a fixed output path is an answer, not a blind spot"
    assert pipeline.output_token == "run/merged.yaml"
    assert py_pipeline.output_path(pipeline) == "run/merged.yaml"
    assert py_pipeline.temp_suffix(pipeline) == ""


def test_a_returned_path_is_a_sink_too(tmp_path: Path):
    target = tmp_path / "returned.py"
    target.write_text(
        "import tempfile\n"
        "import yaml\n"
        "BASE = 'config/base.yaml'\n"
        "def build():\n"
        "    params = yaml.safe_load(open(BASE))\n"
        "    params['use_sim_time'] = False\n"
        "    tmp = tempfile.NamedTemporaryFile(suffix='.yaml', delete=False)\n"
        "    yaml.safe_dump(params, tmp)\n"
        "    written = tmp.name\n"
        "    return written\n",
        encoding="utf-8",
    )
    pipeline = py_pipeline.analyse_file(str(target), "returned.py").pipelines[0]
    assert pipeline.output_token == "written"
    assert py_pipeline.sink_argument(pipeline) == ""
    assert _by_key(pipeline, "use_sim_time").value is False


def test_broken_python_is_reported_not_raised(tmp_path: Path):
    target = tmp_path / "broken.py"
    target.write_text("def f(:\n    pass\n", encoding="utf-8")
    result = py_pipeline.analyse_file(str(target), "broken.py")
    assert result.pipelines == ()
    assert "SyntaxError" in result.parse_error


def test_a_missing_file_is_reported_not_raised(tmp_path: Path):
    result = py_pipeline.analyse_file(str(tmp_path / "absent.py"), "absent.py")
    assert result.parse_error


def test_a_recursive_write_helper_terminates(tmp_path: Path):
    target = tmp_path / "deep.py"
    target.write_text(
        "import tempfile\n"
        "import yaml\n"
        "BASE = 'config/base.yaml'\n"
        "def _apply(doc):\n"
        "    doc['a']['b'] = 1\n"
        "    _apply(doc['a'])\n"
        "def build():\n"
        "    params = yaml.safe_load(open(BASE))\n"
        "    _apply(params)\n"
        "    tmp = tempfile.NamedTemporaryFile(suffix='.yaml', delete=False)\n"
        "    yaml.safe_dump(params, tmp)\n"
        "    out = tmp.name\n"
        "    return f'ros2 launch p f.launch.py params_file:={out}'\n",
        encoding="utf-8",
    )
    pipeline = py_pipeline.analyse_file(str(target), "deep.py").pipelines[0]
    keys = [py_pipeline.dotted(item.key_path) for item in pipeline.overrides]
    assert keys == ["a.b", "a.a.b"], "the second level is under the key it was handed"
    assert not pipeline.complete
    assert any("below depth" in note for note in pipeline.notes)


def test_a_helper_handed_a_sub_dict_keeps_the_full_key_path(tmp_path: Path):
    target = tmp_path / "block.py"
    target.write_text(
        "import tempfile\n"
        "import yaml\n"
        "BASE = 'config/base.yaml'\n"
        "def _tune(block):\n"
        "    block['desired_linear_vel'] = 1.0\n"
        "def build():\n"
        "    params = yaml.safe_load(open(BASE))\n"
        "    _tune(params['controller_server']['ros__parameters'])\n"
        "    tmp = tempfile.NamedTemporaryFile(suffix='.yaml', delete=False)\n"
        "    yaml.safe_dump(params, tmp)\n"
        "    out = tmp.name\n"
        "    return f'ros2 launch p f.launch.py params_file:={out}'\n",
        encoding="utf-8",
    )
    pipeline = py_pipeline.analyse_file(str(target), "block.py").pipelines[0]
    written = _by_key(pipeline, "controller_server.ros__parameters.desired_linear_vel")
    assert written.value == 1.0


def test_a_write_after_the_dump_is_not_in_the_file(tmp_path: Path):
    target = tmp_path / "late.py"
    target.write_text(
        "import tempfile\n"
        "import yaml\n"
        "BASE = 'config/base.yaml'\n"
        "def build():\n"
        "    params = yaml.safe_load(open(BASE))\n"
        "    params['early'] = 1\n"
        "    tmp = tempfile.NamedTemporaryFile(suffix='.yaml', delete=False)\n"
        "    yaml.safe_dump(params, tmp)\n"
        "    params['late'] = 2\n"
        "    out = tmp.name\n"
        "    return f'ros2 launch p f.launch.py params_file:={out}'\n",
        encoding="utf-8",
    )
    pipeline = py_pipeline.analyse_file(str(target), "late.py").pipelines[0]
    keys = [py_pipeline.dotted(item.key_path) for item in pipeline.overrides]
    assert keys == ["early"]
    assert f"{py_pipeline.NOTE_AFTER_DUMP}late = 2" in pipeline.notes


def test_a_log_line_does_not_take_the_sink_argument(tmp_path: Path):
    target = tmp_path / "logged.py"
    target.write_text(
        "import tempfile\n"
        "import yaml\n"
        "BASE = 'config/base.yaml'\n"
        "def build():\n"
        "    params = yaml.safe_load(open(BASE))\n"
        "    params['a'] = 1\n"
        "    tmp = tempfile.NamedTemporaryFile(suffix='.yaml', delete=False)\n"
        "    yaml.safe_dump(params, tmp)\n"
        "    out = tmp.name\n"
        "    message = f'merged parameters written to {out}'\n"
        "    command = f'ros2 launch p f.launch.py params_file:={out}'\n"
        "    return message and command\n",
        encoding="utf-8",
    )
    pipeline = py_pipeline.analyse_file(str(target), "logged.py").pipelines[0]
    assert py_pipeline.sink_argument(pipeline) == "params_file"


def test_provenance_terminates_at_the_symbol_not_at_the_temp_path(launcher: Path):
    pipeline = _analyse(launcher).pipelines[0]
    assert pipeline.applied_by == "launcher.nav_command"
    assert pipeline.site is not None and pipeline.site.path.endswith("app/launcher.py")
    assert all(item.site.path.endswith(".py") for item in pipeline.overrides if item.site)
    assert not any(note.startswith("/") for note in pipeline.notes)


# ---------------------------------------------------------------------------
# a launch file that assembles its own document


LAUNCH_FILE = """
import os
import tempfile

import yaml as _yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("lx_bringup")
    base = os.path.join(share, "config", "nav2_params.yaml")
    overlay = os.path.join(share, "config", "overrides.yaml")

    def _merge(left, right):
        for key, value in right.items():
            if isinstance(value, dict) and isinstance(left.get(key), dict):
                _merge(left[key], value)
            else:
                left[key] = value
        return left

    def _nav_setup(context, *args, **kwargs):
        with open(base, "r") as handle:
            params = _yaml.safe_load(handle)
        with open(overlay, "r") as handle:
            _merge(params, _yaml.safe_load(handle) or {})
        params.setdefault("bt_navigator", {}).setdefault("ros__parameters", {})["retries"] = 4
        tmp = tempfile.NamedTemporaryFile(
            mode="w", suffix="_nav_lx.yaml", delete=False, dir="/tmp")
        _yaml.dump(params, tmp)
        tmp.flush()
        return [IncludeLaunchDescription(
            None, launch_arguments={"params_file": tmp.name}.items())]

    return LaunchDescription([OpaqueFunction(function=_nav_setup)])
"""


@pytest.fixture
def launch_file(tmp_path: Path) -> Path:
    target = tmp_path / "nav_stack.launch.py"
    target.write_text(LAUNCH_FILE, encoding="utf-8")
    return target


def _analyse_launch(target: Path) -> py_pipeline.ModuleResult:
    return py_pipeline.analyse_file(
        str(target),
        "src/lx_bringup/launch/nav_stack.launch.py",
        packages={"lx_bringup": "install/lx_bringup/share/lx_bringup"},
        launch_file=True,
    )


def test_a_merge_inside_a_launch_file_is_the_same_pipeline(launch_file: Path):
    """The constants are locals of the enclosing function, the merge helper is nested, and
    the document reaches a launch argument instead of a command string."""
    pipelines = _analyse_launch(launch_file).pipelines
    assert len(pipelines) == 1
    pipeline = pipelines[0]
    assert pipeline.base == "install/lx_bringup/share/lx_bringup/config/nav2_params.yaml"
    assert pipeline.overlays == ("install/lx_bringup/share/lx_bringup/config/overrides.yaml",)
    assert pipeline.applied_by == "nav_stack.launch._nav_setup"
    assert pipeline.complete


def test_the_launch_file_itself_is_the_sink(launch_file: Path):
    pipeline = _analyse_launch(launch_file).pipelines[0]
    assert py_pipeline.launch_argument(pipeline) == "params_file"
    assert py_pipeline.sink_target(pipeline) == "nav_stack.launch.py"
    assert py_pipeline.temp_suffix(pipeline) == "_nav_lx.yaml"
    assert py_pipeline.sink_argument(pipeline) == "", "there is no command string to bind to"


def test_the_write_inside_the_launch_file_is_recovered(launch_file: Path):
    pipeline = _analyse_launch(launch_file).pipelines[0]
    written = _by_key(pipeline, "bt_navigator.ros__parameters.retries")
    assert written.value == 4
    assert written.resolved


def test_a_package_share_path_that_cannot_fold_is_never_guessed(launch_file: Path):
    pipeline = py_pipeline.analyse_file(
        str(launch_file),
        "src/lx_bringup/launch/nav_stack.launch.py",
        launch_file=True,
    ).pipelines[0]
    assert pipeline.base is None
    assert not pipeline.complete
    assert any(note.startswith(py_pipeline.NOTE_UNRESOLVED) for note in pipeline.notes)


def test_the_launch_pipeline_is_stable_across_two_reads(launch_file: Path):
    left = _analyse_launch(launch_file).pipelines
    right = _analyse_launch(launch_file).pipelines
    assert left == right
