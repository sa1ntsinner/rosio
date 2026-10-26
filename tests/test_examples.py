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
    ("param.set_undeclared", "param:/safety_stop|stop_distnce"),
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
