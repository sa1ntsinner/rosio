"""The example workspaces, built and extracted end to end, against the faults they plant.

`examples/rover_ws/README.md` lists what is wrong in the rover on purpose. This test builds it
with colcon, extracts it with its own `rosio.yaml` and holds the findings to exactly that list:
a planted fault rosio stops seeing is a regression, and a finding nobody planted is a false one.
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

from rosio import cli, model

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"
FIXTURES = Path(__file__).resolve().parent / "fixtures"

PLANTED = {
    ("qos.incompatible", "topic:/scan_filtered"),
    ("frame.not_in_urdf", "frame:laser_frame"),
    ("param.set_undeclared", "file:rover_bringup|config/rover.yaml"),
    ("param.shadowed", "param:/scan_filter|max_range"),
    ("param.orphan_block", "file:rover_bringup|config/rover.yaml"),
}


@pytest.mark.ros
@pytest.mark.slow
def test_the_rover_reports_exactly_the_faults_it_plants(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    if shutil.which("colcon") is None:
        pytest.skip("colcon is not installed")
    workspace = tmp_path / "rover_ws"
    shutil.copytree(EXAMPLES / "rover_ws", workspace)
    built = subprocess.run(
        ["colcon", "build"], cwd=workspace, capture_output=True, text=True, timeout=600
    )
    assert built.returncode == 0, built.stdout[-2000:] + built.stderr[-2000:]

    monkeypatch.delenv("PYTHONDONTWRITEBYTECODE", raising=False)
    out = tmp_path / "out"
    code = cli.main(
        [
            "extract",
            str(workspace),
            "--config",
            str(workspace / "rosio.yaml"),
            "--out",
            str(out),
            "--quiet",
        ]
    )
    assert code in (0, 1)
    atlas = model.Atlas.loads((out / "atlas.json").read_text(encoding="utf-8"))
    found = {(finding.rule, finding.subject_key) for finding in atlas.findings}
    assert found == PLANTED, sorted(found ^ PLANTED)
    reads = {
        (fact.profile, read["file"], read["role"])
        for entity in atlas.by_kind("launchent")
        for fact in entity.facts
        for read in fact.values.get("file_reads", ())
        if read["role"] == "robot_model"
    }
    assert reads == {("profile:rover", "file:rover_description|urdf/rover.urdf", "robot_model")}
    assert not list((workspace / "src").rglob("__pycache__")), "extract wrote into the sources"


@pytest.mark.ros
@pytest.mark.slow
def test_the_rover_extracts_as_shipped_without_a_build(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The example once declared an install root, so the README sequence stopped at
    `install root does not exist` until somebody ran colcon build."""
    workspace = tmp_path / "rover_ws"
    shutil.copytree(EXAMPLES / "rover_ws", workspace)
    monkeypatch.chdir(workspace)
    monkeypatch.delenv("PYTHONDONTWRITEBYTECODE", raising=False)
    assert cli.main(["extract", "--quiet"]) in (0, 1)
    atlas = model.Atlas.loads((workspace / "rosio-out" / "atlas.json").read_text(encoding="utf-8"))
    found = {(finding.rule, finding.subject_key) for finding in atlas.findings}
    assert found == PLANTED, sorted(found ^ PLANTED)
    assert [node.id for node in atlas.by_kind("profile")] == ["profile:rover"]


@pytest.mark.ros
@pytest.mark.slow
def test_a_line_says_whether_it_wins_and_a_fix_shows_as_one_finding_gone(tmp_path: Path) -> None:
    from rosio import query

    workspace = tmp_path / "rover_ws"
    shutil.copytree(EXAMPLES / "rover_ws", workspace)
    before = _extract(workspace, tmp_path / "out")
    shadowed = query.explain(before, "config/rover.yaml:4").text()
    assert "loses to 10.0 from rover_bringup/launch/bringup.launch.py:32" in shadowed
    typo = query.explain(before, "src/rover_bringup/config/rover.yaml:9").text()
    assert "replace 'stop_distnce' with 'stop_distance'" in typo

    config = workspace / "src/rover_bringup/config/rover.yaml"
    config.write_text(config.read_text().replace("stop_distnce", "stop_distance"))
    delta = query.finding_delta(before, _extract(workspace, tmp_path / "out")).data
    assert [row["rule"] for row in delta["gone"]] == ["param.set_undeclared"]
    assert delta["new"] == []


@pytest.mark.ros
@pytest.mark.slow
def test_findings_a_broken_extraction_lost_are_not_reported_as_fixed(tmp_path: Path) -> None:
    """A coding agent's sandbox held ~/.ros read only, the launch walk raised, and extract
    said six findings went."""
    from rosio import query

    workspace = tmp_path / "rover_ws"
    shutil.copytree(EXAMPLES / "rover_ws", workspace)
    before = _extract(workspace, tmp_path / "out")
    launch = workspace / "src/rover_bringup/launch/bringup.launch.py"
    launch.write_text(launch.read_text().replace("    return", "    raise OSError(30)\n    return"))
    delta = query.finding_delta(before, _extract(workspace, tmp_path / "out"))
    assert delta.data["read_less"] == ["launch.raised"]
    assert "unread rather than fixed" in delta.text()


def _extract(workspace: Path, out: Path) -> model.Atlas:
    code = cli.main(
        [
            "extract",
            str(workspace),
            "--config",
            str(workspace / "rosio.yaml"),
            "--out",
            str(out),
            "--quiet",
        ]
    )
    assert code in (0, 1)
    return model.Atlas.loads((out / "atlas.json").read_text(encoding="utf-8"))


@pytest.mark.ros
@pytest.mark.slow
def test_a_parameters_file_the_build_does_not_install_is_reported_at_the_launch_line(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    if shutil.which("colcon") is None:
        pytest.skip("colcon is not installed")
    workspace = tmp_path / "not_installed_ws"
    shutil.copytree(FIXTURES / "not_installed_ws", workspace)
    built = subprocess.run(
        ["colcon", "build"], cwd=workspace, capture_output=True, text=True, timeout=600
    )
    assert built.returncode == 0, built.stdout[-2000:] + built.stderr[-2000:]
    monkeypatch.delenv("PYTHONDONTWRITEBYTECODE", raising=False)

    atlas = _extract(workspace, tmp_path / "out")
    found = [f for f in atlas.findings if f.rule == "launch.file_not_installed"]
    readers = {
        f.subject_key: next(row for row in f.where if row.file != f.subject_key) for f in found
    }
    assert {key: row.line_hint for key, row in readers.items()} == {
        "file:uninstalled_config|config/params.yaml": 20,
        "file:uninstalled_config|urdf/robot.urdf.xacro": 26,
        "file:uninstalled_config|rviz/view.rviz": 32,
    }
    assert all(
        row.file == "file:uninstalled_config|launch/demo.launch.py" for row in readers.values()
    )
    assert "install(DIRECTORY config" in found[0].say
    assert not [f for f in atlas.findings if f.rule == "launch.raised"]
    parameter = atlas.entities["param:/robot_state_publisher|robot_description"]
    assert parameter.state == "other"
    assert not parameter.effective
    assert len(parameter.candidates) == 1
    assert not parameter.candidates[0].wins
    blind = [u for u in atlas.unresolved if u.owner == parameter.id]
    assert len(blind) == 1
    assert blind[0].reason == "command_substitution"
    assert blind[0].want == model.Want.PARAM_VALUE.value
    assert blind[0].expr == "xacro file:uninstalled_config|urdf/robot.urdf.xacro"
    assert all(not atlas.entities[f.subject_key].facts for f in found)
    assert str(workspace) not in atlas.canonical_bytes().decode("utf-8")

    (workspace / "src/uninstalled_config/config/params.yaml").unlink()
    atlas = _extract(workspace, tmp_path / "out_wrong_path")
    assert len([f for f in atlas.findings if f.rule == "launch.file_not_installed"]) == 2
    assert [f.rule for f in atlas.findings if f.rule == "extract.input_refused"] == [
        "extract.input_refused"
    ]


@pytest.mark.ros
@pytest.mark.slow
def test_an_unbuilt_workspace_reads_its_own_packages_from_source(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    workspace = tmp_path / "src_only_ws"
    shutil.copytree(FIXTURES / "src_only_ws", workspace)
    monkeypatch.delenv("PYTHONDONTWRITEBYTECODE", raising=False)

    atlas = _extract(workspace, tmp_path / "out")
    nodes = {
        node.id
        for node in atlas.by_kind("node")
        if any(fact.profile == "profile:demo" for fact in node.facts)
    }
    assert nodes == {"node:/planner", "node:/controller", "node:/recovery"}
    assert not [f for f in atlas.findings if f.rule in ("launch.raised", "launch.unloadable")]
    for name in ("planner|tolerance", "controller|frequency"):
        parameter = atlas.entities[f"param:/{name}"]
        assert parameter.state == "effective"
        assert [(c.source.file, c.source.root) for c in parameter.candidates] == [
            ("file:demo_nav|params/nav.yaml", "src")
        ]
    notes = {
        note
        for entity in atlas.by_kind("launchfile")
        for fact in entity.facts
        for note in fact.values.get("notes", ())
    }
    assert "the share directory of demo_nav was read from its source tree" in notes

    # A path the source does not hold is the refusal it always was, never a guess.
    refused = [f for f in atlas.findings if f.rule == "extract.input_refused"]
    assert [f.subject_key for f in refused] == ["file:demo_nav|params/recovery.yaml"]
    assert not [f for f in atlas.findings if f.rule == "launch.file_not_installed"]
    assert not list((workspace / "src").rglob("__pycache__")), "extract wrote into the sources"
    assert str(workspace) not in atlas.canonical_bytes().decode("utf-8")


@pytest.mark.ros
@pytest.mark.slow
def test_a_node_that_declares_from_overrides_keeps_every_value_a_file_sets(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Neither node of the rover declares the key the file sets once it takes its parameters
    from overrides, and each still runs with the value."""
    workspace = tmp_path / "rover_ws"
    shutil.copytree(EXAMPLES / "rover_ws", workspace)
    stop = workspace / "src/rover_control/rover_control/safety_stop.py"
    stop.write_text(
        stop.read_text(encoding="utf-8").replace(
            'super().__init__("safety_stop")',
            'super().__init__("safety_stop", automatically_declare_parameters_from_overrides=True)',
        ),
        encoding="utf-8",
    )
    scan = workspace / "src/rover_perception/src/scan_filter.cpp"
    scan.write_text(
        scan.read_text(encoding="utf-8").replace(
            ': Node("scan_filter"),',
            ': Node("scan_filter", rclcpp::NodeOptions()'
            ".automatically_declare_parameters_from_overrides(true)),",
        ),
        encoding="utf-8",
    )
    params = workspace / "src/rover_bringup/config/rover.yaml"
    params.write_text(
        params.read_text(encoding="utf-8").replace(
            "    frame_id: laser_link\n", "    frame_id: laser_link\n    extra_gain: 2.0\n"
        ),
        encoding="utf-8",
    )
    monkeypatch.delenv("PYTHONDONTWRITEBYTECODE", raising=False)
    atlas = _extract(workspace, tmp_path / "out")
    for name in ("safety_stop|stop_distnce", "scan_filter|extra_gain"):
        parameter = atlas.entities[f"param:/{name}"]
        assert parameter.state == "effective", name
        assert parameter.effective, name
    assert not [f for f in atlas.findings if f.rule == "param.set_undeclared"]
