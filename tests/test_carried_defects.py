"""The four defects carried into M6, plus the one the installer attack surfaced.

Each test states the wrong answer the code gave before, because the point of every one of
them is that the tool stayed silent rather than said something false.
"""

from __future__ import annotations

import stat
from pathlib import Path
from typing import Any

import pytest

from rosio import extract, ids, join, model
from rosio.agent import install as agent_install
from rosio.drift import engine, rules
from rosio.errors import UsageError
from rosio.resolve import _launch_child, identity

PKG = "nx_bringup"
RELPATH = "scripts/sector_crop.py"
SYM = ids.build_sym(PKG, RELPATH, "scripts.sector_crop.SectorCrop.__init__")
FILE = ids.build_file(PKG, RELPATH)
EXE_ID = ids.build_exe(PKG, "sector_crop.py")
LAUNCHFILE = ids.build_launchfile(PKG, "launch/nx_sim_launch.py")
LAUNCH_SRC = ids.build_file(PKG, "launch/nx_sim_launch.py")
LAUNCHENT = ids.build_launchent(
    PKG, "launch/nx_sim_launch.py", (("ExecuteProcess", "sector_crop.py@fdb48af6"),)
)
NODE = ids.build_node("/sector_crop")
SIM = "profile:nx_sim"

ARGV = (
    "python3",
    "/install/share/nx_bringup/scripts/sector_crop.py",
    "--ros-args",
    "-p",
    "output_topic:=/ouster/points_cropped",
    "-p",
    "decimate_rings:=2",
    "-p",
    "input_topic:=/ouster/points_raw",
)

CODED_VALUES: dict[str, Any] = {
    "nodes": [
        {
            "name": "sector_crop",
            "name_source": "literal",
            "form": "node",
            "symbol": SYM,
        }
    ],
    "parameters": [
        {
            "name": "output_topic",
            "type": "str",
            "default": "/ouster/points",
            "read": True,
            "owner_symbol": "SectorCrop.__init__",
            "line_hint": 27,
        },
        {
            "name": "decimate_rings",
            "type": "int",
            "default": 1,
            "read": True,
            "owner_symbol": "SectorCrop.__init__",
            "line_hint": 39,
        },
        {
            "name": "input_topic",
            "type": "str",
            "default": "/ouster/points_raw",
            "read": True,
            "owner_symbol": "SectorCrop.__init__",
            "line_hint": 26,
        },
        {
            "name": "azimuth_max_deg",
            "type": "double",
            "default": 90.0,
            "read": True,
            "owner_symbol": "SectorCrop.__init__",
            "line_hint": 31,
        },
    ],
}


def _built(*, evidence: bool = True) -> model.AtlasBuilder:
    """One ExecuteProcess whose argv states three parameters, and the script that declares
    them. The launch file names no node, which is why the declared layer built no chain."""
    builder = model.AtlasBuilder()
    builder.add_profile("nx_sim")
    builder.node(ids.build_pkg(PKG))
    builder.node(FILE)
    builder.node(SYM)
    builder.node(EXE_ID)
    builder.node(LAUNCHFILE)
    builder.node(LAUNCH_SRC)
    builder.node(LAUNCHENT)
    builder.node(NODE, name_source=model.NameSource.EXECUTABLE_DEFAULT.value)
    builder.edge(model.EdgeKind.DEFINES.value, FILE, SYM, layer=join.CODED)
    builder.edge(model.EdgeKind.RUNS.value, EXE_ID, SYM, layer=join.DECLARED)
    builder.fact(SYM, join.CODED, values=CODED_VALUES)
    where = [model.Source(file=LAUNCH_SRC, locator="ExecuteProcess(sector_crop.py@fdb48af6)")]
    builder.fact(
        NODE,
        join.DECLARED,
        profile=SIM,
        values={
            "pkg": PKG,
            "executable": "sector_crop.py",
            "namespace": "/",
            "form": "process",
            "argv": list(ARGV),
        },
        evidence=where if evidence else [],
    )
    builder.edge(
        model.EdgeKind.LAUNCHES.value,
        LAUNCHENT,
        NODE,
        key=ids.edge_key(launchent=LAUNCHENT, profile="nx_sim"),
        layer=join.DECLARED,
        profile=SIM,
    )
    return builder


def _chain(atlas: model.Atlas, name: str) -> list[model.Candidate]:
    entity = atlas.entities[ids.build_param(NODE, name)]
    return sorted(entity.candidates, key=model.Candidate.sort_key)


# ---------------------------------------------------------------------------
# a. the value argv states reaches the chain, and silence is never the answer


def test_a_parameter_argv_states_wins_over_the_code_default():
    builder = _built()
    join.join(builder)
    atlas = builder.atlas
    entity = atlas.entities[ids.build_param(NODE, "output_topic")]

    assert entity.state == model.ParamState.EFFECTIVE.value
    winner, loser = _chain(atlas, "output_topic")
    assert (winner.rank, winner.wins, winner.value) == (0, True, "/ouster/points_cropped")
    assert winner.mechanism == model.Mechanism.ROS_ARGS_P.value
    assert winner.source.file == LAUNCH_SRC
    assert winner.source.locator.endswith("-p output_topic")
    assert (loser.rank, loser.wins, loser.value) == (1, False, "/ouster/points")
    assert loser.mechanism == model.Mechanism.CODE_DEFAULT.value
    assert loser.shadow_kind == model.ShadowKind.VALUE_CHANGED.value
    assert loser.shadowed_by == 0
    assert entity.effective[SIM]["value"].value == "/ouster/points_cropped"
    assert entity.effective[SIM]["value"].layer == join.DECLARED


def test_the_argv_value_keeps_the_type_rcl_would_give_it():
    builder = _built()
    join.join(builder)
    winner = _chain(builder.atlas, "decimate_rings")[0]
    assert winner.value == 2
    assert winner.value_type == "int"


def test_an_argv_value_equal_to_the_default_is_restated_and_not_changed():
    builder = _built()
    join.join(builder)
    loser = _chain(builder.atlas, "input_topic")[1]
    assert loser.shadow_kind == model.ShadowKind.RESTATED_IDENTICAL.value


def test_a_parameter_argv_never_names_still_takes_its_code_default():
    builder = _built()
    join.join(builder)
    entity = builder.atlas.entities[ids.build_param(NODE, "azimuth_max_deg")]
    assert entity.state == model.ParamState.EFFECTIVE.value
    only = _chain(builder.atlas, "azimuth_max_deg")
    assert len(only) == 1
    assert (only[0].mechanism, only[0].wins) == (model.Mechanism.CODE_DEFAULT.value, True)


def test_an_argv_row_with_no_file_to_cite_becomes_a_blind_spot():
    builder = _built(evidence=False)
    join.join(builder)
    atlas = builder.atlas
    param_id = ids.build_param(NODE, "output_topic")
    assert atlas.entities[param_id].state == model.ParamState.OTHER.value
    blind = [row for row in atlas.unresolved if row.owner == param_id]
    assert len(blind) == 1
    assert blind[0].want == model.Want.PARAM_VALUE.value
    assert blind[0].expr == "-p output_topic:=/ouster/points_cropped"
    assert atlas.entities[param_id].unresolved == [blind[0].id]


def test_no_parameter_of_the_process_is_left_without_a_value_or_a_blind_spot():
    for evidence in (True, False):
        builder = _built(evidence=evidence)
        join.join(builder)
        atlas = builder.atlas
        silent = [
            entity.id
            for entity in atlas.entities.values()
            if entity.kind == "param"
            and not entity.effective
            and not entity.unresolved
            and entity.candidates
            and not any(row.wins for row in entity.candidates)
        ]
        assert silent == []


def test_the_join_stays_idempotent_with_the_argv_row_in():
    builder = _built()
    join.join(builder)
    once = builder.atlas.canonical_bytes()
    join.join(builder)
    assert builder.atlas.canonical_bytes() == once


# ---------------------------------------------------------------------------
# b. the rule vocabulary loads every family, not two of six


def test_every_committed_rule_is_registered_in_a_fresh_process():
    registered = rules.registered()
    assert len(registered) == len(rules.COMMITTED)
    assert [name for name in rules.COMMITTED if name not in registered] == []


def test_load_rules_with_no_argument_loads_them_all():
    loaded, failures = engine.load_rules()
    assert failures == []
    assert len(loaded) == len(rules.COMMITTED)


def test_every_family_module_is_named_in_the_family_list():
    here = Path(rules.__file__).parent
    families = {
        f"rosio.drift.{path.stem}"
        for path in here.glob("*.py")
        if path.stem not in ("__init__", "rules", "engine")
    }
    assert families <= set(rules.FAMILIES)


# ---------------------------------------------------------------------------
# c. the opt in gate covers every collector that can declare the observed layer


def test_the_opt_in_gate_covers_every_observed_collector():
    assert set(extract.collectors_for(model.Layer.OBSERVED.value)) <= extract.OPT_IN


def test_extract_refuses_an_observed_collector_by_name():
    with pytest.raises(UsageError) as caught:
        extract._select(["proc"])
    assert "proc" in str(caught.value)
    assert extract.PROBE_VERB in str(caught.value)


def test_the_probe_verb_still_reaches_both_observed_collectors():
    chosen = extract._select(["proc", "probe"], opt_in=sorted(extract.OPT_IN))
    assert chosen == ("proc", "probe")


def test_a_default_run_reaches_no_observed_collector():
    assert set(extract.default_collectors()).isdisjoint(extract.OPT_IN)


# ---------------------------------------------------------------------------
# d. a temp file another writer made is external, whatever this process's TMPDIR is


def _walker(roots: tuple[str, ...] = ()) -> Any:
    return _launch_child.Walker({"entry_path": "x.launch.py", "roots": list(roots)})


def test_a_temp_file_outside_this_process_temp_root_is_not_a_repository_file(monkeypatch):
    monkeypatch.setenv("TMPDIR", "/ssd/rosio-tmp")
    walker = _walker()
    for root in identity.TEMP_ROOTS:
        assert walker._file_kind(f"{root}/tmp9f2c_merged.yaml") == "external_file"


def test_the_process_own_temp_root_is_external_too(tmp_path, monkeypatch):
    monkeypatch.setenv("TMPDIR", str(tmp_path))
    walker = _walker()
    assert walker._file_kind(str(tmp_path / "merged.yaml")) == "external_file"


def test_a_file_under_a_tracked_root_stays_a_repository_file():
    walker = _walker(("/ws/src",))
    assert walker._file_kind("/ws/src/pkg/config/base.yaml") == "repo_file"
    assert walker._file_kind("/elsewhere/base.yaml") == "external_file"


def test_with_no_roots_a_plain_path_is_still_a_repository_file():
    assert _walker()._file_kind("/ws/src/pkg/config/base.yaml") == "repo_file"


# ---------------------------------------------------------------------------
# the installer refusal the attack surfaced: a filesystem error is a rosio error


@pytest.mark.unreadable
def test_a_read_only_project_root_is_a_rosio_error_and_not_an_oserror(tmp_path):
    root = tmp_path / "project"
    root.mkdir()
    root.chmod(stat.S_IRUSR | stat.S_IXUSR)
    try:
        with pytest.raises(UsageError) as caught:
            agent_install.install("claude-code", root=root, home=tmp_path / "home")
    finally:
        root.chmod(stat.S_IRWXU)
    assert "could not be written" in str(caught.value)
    assert "--dry-run" in str(caught.value)


@pytest.mark.unreadable
def test_a_read_only_file_is_a_rosio_error_and_the_exit_code_is_usage(tmp_path, capsys):
    root = tmp_path / "project"
    home = tmp_path / "home"
    agent_install.install("claude-code", root=root, home=home)
    managed = root / "CLAUDE.md"
    managed.write_text(managed.read_text().replace("## rosio:", "## stale:"), encoding="utf-8")
    managed.chmod(stat.S_IRUSR)
    try:
        code = agent_install.main(["claude-code", "--root", str(root), "--home", str(home)])
    finally:
        managed.chmod(stat.S_IRUSR | stat.S_IWUSR)
    assert code == 2
    assert "could not be written" in capsys.readouterr().err


def test_a_file_the_sandbox_did_not_find_outside_tmp_is_missing_on_this_machine():
    """A spawner's share file that only a build installs read as a sandbox refusal, and the
    message dropped the file it named."""
    walker = _walker(("/tmp/ws/src",))
    assert walker._really_missing("/ws/src/demo/config/rrbot_controllers.yaml")
    assert walker._really_missing("/tmp/ws/src/demo/config/x.yaml")
    assert not walker._really_missing("/tmp/merged_params.yaml")
    assert not walker._really_missing(None)
