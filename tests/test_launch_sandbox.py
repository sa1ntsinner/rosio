from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from rosio import cli, extract, ids, model
from rosio.config import Config, Entrypoint, ProfileSpec, RootSpec, load_config
from rosio.errors import UsageError
from rosio.resolve import launch_exec
from rosio.workspace import Workspace


def configuration(tmp_path: Path, mode: str = "auto") -> Config:
    package = tmp_path / "src" / "sandbox_demo"
    package.mkdir(parents=True, exist_ok=True)
    (package / "package.xml").write_text(
        '<package format="3"><name>sandbox_demo</name><version>0.0.0</version></package>'
    )
    (package / "demo.launch.py").write_text("def generate_launch_description(): return None\n")
    return Config(
        roots=(RootSpec("src", str(tmp_path / "src")),),
        profiles=(
            ProfileSpec(
                "demo", entrypoints=(Entrypoint("launch", "sandbox_demo", "demo.launch.py"),)
            ),
        ),
        out_dir=str(tmp_path / "out"),
        launch_sandbox=mode,
    )


@pytest.mark.parametrize("mode", ["auto", "required", "off"])
def test_configuration_and_cli_sandbox(tmp_path: Path, mode: str) -> None:
    configuration(tmp_path)
    config = tmp_path / "rosio.yaml"
    config.write_text(f"launch_sandbox: {mode}\n")
    cfg = load_config(str(config), workspace=tmp_path)
    assert cfg.launch_sandbox == mode
    args = cli.build_parser().parse_args(["extract", "--launch-sandbox", mode])
    assert (
        cli._with_overrides(replace(cfg, launch_sandbox="off"), args, "out").launch_sandbox == mode
    )
    args = cli.build_parser().parse_args(["extract"])
    assert cli._with_overrides(cfg, args, "out").launch_sandbox == mode


def test_sandbox_default_and_validation(tmp_path: Path) -> None:
    cfg = configuration(tmp_path)
    assert cfg.launch_sandbox == "auto"
    with pytest.raises(UsageError, match="launch_sandbox"):
        replace(cfg, launch_sandbox="invalid")
    with pytest.raises(SystemExit):
        cli.build_parser().parse_args(["extract", "--launch-sandbox", "invalid"])


def test_log_environment_probes_stay_unchanged_without_the_sandbox() -> None:
    from rosio.resolve._launch_child import _Probes

    plain = _Probes()
    sandboxed = _Probes(sandbox=True)
    for probes in (plain, sandboxed):
        assert probes._wrap_env(lambda *_: "logs")("ROS_HOME") == "logs"
    assert plain.seen == {"env:ROS_HOME=logs": True}
    assert sandboxed.seen == {}


def test_required_without_bwrap_refuses_the_pass(tmp_path: Path, monkeypatch) -> None:
    cfg = configuration(tmp_path, "required")
    monkeypatch.setattr(launch_exec, "probe_sandbox", lambda *_: "")
    atlas = extract.run(cfg, Workspace.discover(cfg.roots), collectors=["launch_declared"])
    rows = [row for run in atlas.by_kind("run") for row in run.attrs.get("passes", [])]
    assert [row["state"] for row in rows] == ["refused"]
    assert "bwrap" in rows[0]["reason"] and "apt install bubblewrap" in rows[0]["reason"]
    assert any(f.rule == "extract.pass_incomplete" for f in atlas.findings)


def test_auto_without_bwrap_uses_the_old_walk(tmp_path: Path, monkeypatch) -> None:
    entry = tmp_path / "read.launch.py"
    entry.write_text(
        "from launch import LaunchDescription\n"
        "def generate_launch_description(): return LaunchDescription([])\n"
    )
    monkeypatch.setattr(launch_exec, "probe_sandbox", lambda *_: "")
    monkeypatch.setattr(launch_exec, "_python", lambda *_: (["python3", "child"], None))

    def run(_self, command, *_args):
        assert command == ["python3", "child"]
        return SimpleNamespace(stdout=launch_exec.MARKER + model.dumps({}), returncode=0)

    monkeypatch.setattr(launch_exec.Children, "run", run)
    auto = launch_exec.walk(str(entry))
    off = launch_exec.walk(str(entry), launch_sandbox="off")
    assert not auto.exceptions
    assert launch_exec.encode_walk(auto) == launch_exec.encode_walk(off)


def test_sandbox_is_probed_once_per_extraction(tmp_path: Path, monkeypatch) -> None:
    cfg = configuration(tmp_path)
    cfg = replace(cfg, profiles=(*cfg.profiles, replace(cfg.profiles[0], slug="second")))
    probes = []

    def probe(*_):
        probes.append(True)
        return ""

    monkeypatch.setattr(launch_exec, "probe_sandbox", probe)
    monkeypatch.setattr(launch_exec, "probe", lambda *_: launch_exec.LaunchImport(ok=False))
    atlas = extract.run(cfg, Workspace.discover(cfg.roots), collectors=["launch_declared"])
    assert len(probes) == 1
    assert all(
        run.attrs["predicates"]["launch_sandbox:bwrap"] is False for run in atlas.by_kind("run")
    )


@pytest.mark.parametrize("outcome", [None, 1, 0])
def test_sandbox_probe_requires_a_working_tool(monkeypatch, outcome) -> None:
    monkeypatch.setattr(launch_exec, "sourced_environment", lambda *_: {"PATH": "/bin"})
    monkeypatch.setattr(
        launch_exec.shutil, "which", lambda *_, **__: None if outcome is None else "/bin/bwrap"
    )

    def run(command, **kwargs):
        assert command[-2:] == ["--", "/bin/true"]
        assert "--unshare-net" in command and "--unshare-pid" in command
        return SimpleNamespace(returncode=outcome)

    monkeypatch.setattr(launch_exec.subprocess, "run", run)
    assert launch_exec.probe_sandbox() == ("/bin/bwrap" if outcome == 0 else "")


def test_shape_refusal_does_not_claim_sandbox_execution(tmp_path: Path, monkeypatch) -> None:
    cfg = configuration(tmp_path)
    entry = tmp_path / "src/sandbox_demo/demo.launch.py"
    entry.write_text("<launch></launch>")
    monkeypatch.setattr(launch_exec, "probe_sandbox", lambda *_: "/bin/bwrap")
    monkeypatch.setattr(launch_exec, "probe", lambda *_: launch_exec.LaunchImport(ok=True))
    atlas = extract.run(cfg, Workspace.discover(cfg.roots), collectors=["launch_declared"])
    assert all(
        run.attrs["predicates"]["launch_sandbox:bwrap"] is False for run in atlas.by_kind("run")
    )


@pytest.fixture
def bwrap() -> str:
    command = launch_exec.probe_sandbox()
    if not command:
        pytest.skip("bwrap is missing or cannot create the required namespaces")
    return command


@pytest.mark.ros
@pytest.mark.parametrize(
    "operation",
    [
        "Path(__file__).with_name('written').write_text('forbidden')",
        "socket.socket(socket.AF_INET, socket.SOCK_STREAM)",
        "hidden_file",
    ],
)
def test_sandbox_refusals_reach_launch_raised(tmp_path: Path, bwrap: str, operation: str) -> None:
    cfg = configuration(tmp_path, "required")
    if operation == "hidden_file":
        hidden = tmp_path / "hidden"
        hidden.write_text("host content")
        operation = f"Path({str(hidden)!r}).read_text()"
    entry = tmp_path / "src/sandbox_demo/demo.launch.py"
    entry.write_text(
        "from pathlib import Path\nimport socket\n"
        "from launch import LaunchDescription\n"
        f"def generate_launch_description():\n    {operation}\n    return LaunchDescription([])\n"
    )
    atlas = extract.run(cfg, Workspace.discover(cfg.roots), collectors=["launch_declared"])
    findings = [f for f in atlas.findings if f.rule == "launch.raised"]
    assert len(findings) == 1
    assert "sandbox refused" in findings[0].detail
    assert "--launch-sandbox off" in findings[0].detail
    assert not entry.with_name("written").exists()
    assert str(tmp_path) not in model.dumps(atlas.to_json())


@pytest.mark.ros
def test_sandbox_reads_preserve_the_description(tmp_path: Path, bwrap: str) -> None:
    entry = tmp_path / "read.launch.py"
    (tmp_path / "name").write_text("reader")
    entry.write_text(
        "from pathlib import Path\nfrom launch import LaunchDescription\n"
        "from launch_ros.actions import Node\n"
        "def generate_launch_description():\n"
        "    name = Path(__file__).with_name('name').read_text()\n"
        "    return LaunchDescription([Node(package='demo_nodes_cpp', "
        "executable='talker', name=name)])\n"
    )
    sandboxed = launch_exec.walk(str(entry), launch_sandbox="required")
    plain = launch_exec.walk(str(entry), launch_sandbox="off")
    assert not sandboxed.exceptions
    assert sandboxed.nodes[0].name == "reader"
    assert launch_exec.encode_walk(sandboxed) == launch_exec.encode_walk(plain)
    assert sandboxed.sandboxed and not plain.sandboxed


@pytest.mark.ros
def test_ros_and_temporary_writes_stay_inside_the_sandbox(tmp_path: Path, bwrap: str) -> None:
    entry = tmp_path / "logs.launch.py"
    entry.write_text(
        "import os, tempfile\nfrom pathlib import Path\nfrom launch import LaunchDescription\n"
        "def generate_launch_description():\n"
        "    for key in ('ROS_HOME', 'ROS_LOG_DIR'):\n"
        "        directory = Path(os.environ[key])\n"
        "        directory.mkdir(parents=True, exist_ok=True)\n"
        "        (directory / 'written').write_text('private')\n"
        "    with tempfile.TemporaryFile() as output: output.write(b'private')\n"
        "    return LaunchDescription([])\n"
    )
    host = tmp_path / "host-logs"
    result = launch_exec.walk(
        str(entry), launch_sandbox="required", env=(("ROS_LOG_DIR", str(host)),)
    )
    assert not result.exceptions
    assert result.sandboxed
    assert not host.exists()


@pytest.mark.ros
def test_a_symlink_install_under_tmp_reaches_its_build_tree(tmp_path: Path, bwrap: str) -> None:
    built = tmp_path / "ws/build/demo/name"
    built.parent.mkdir(parents=True)
    built.write_text("reader")
    share = tmp_path / "ws/install/demo/share/demo"
    share.mkdir(parents=True)
    (share / "name").symlink_to(built)
    (tmp_path / "ws/src").mkdir()
    (tmp_path / "outside").write_text("host content")
    entry = tmp_path / "ws/src/read.launch.py"
    entry.write_text(
        "from pathlib import Path\nfrom launch import LaunchDescription\n"
        "from launch_ros.actions import Node\n"
        "def generate_launch_description():\n"
        f"    name = Path({str(share / 'name')!r}).read_text()\n"
        f"    assert not Path({str(tmp_path / 'outside')!r}).exists()\n"
        "    return LaunchDescription([Node(package='demo_nodes_cpp', "
        "executable='talker', name=name)])\n"
    )
    roots = (
        RootSpec("src", str(tmp_path / "ws/src")),
        RootSpec("install", str(tmp_path / "ws/install")),
    )
    result = launch_exec.walk(
        str(entry),
        launch_sandbox="required",
        workspace=SimpleNamespace(roots=roots, package_of=lambda path: ""),
    )
    assert not result.exceptions
    assert result.nodes[0].name == "reader"


UNRUN_OUTPUT_LAUNCH = """\
import os, tempfile
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    world = tempfile.mktemp(prefix='nav2_', suffix='.sdf')
    child = str(Path(__file__).with_name('child.launch.py'))
    return LaunchDescription([
        ExecuteProcess(cmd=['xacro', '-o', world, 'headless:=True', 'world.sdf.xacro']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(child),
            launch_arguments={'gz_args': '-r -s ' + world}.items(),
        ),
        Node(package='demo_nodes_cpp', executable='talker', name='talker'),
        RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=lambda _: os.remove(world)),
        ])),
    ])
"""


@pytest.mark.ros
@pytest.mark.parametrize("mode", ["off", "required"])
def test_a_file_an_unrun_process_writes_is_named_by_its_command(
    tmp_path: Path, request, mode: str
) -> None:
    """nav2's simulation launch files name a world file `xacro -o` writes under /tmp and
    remove it on shutdown. The walk runs no process, so the removal raised: the sandbox was
    blamed, and without the sandbox the random name reached the finding id."""
    if mode == "required":
        request.getfixturevalue("bwrap")
    cfg = configuration(tmp_path, mode)
    launch = tmp_path / "src/sandbox_demo"
    (launch / "demo.launch.py").write_text(UNRUN_OUTPUT_LAUNCH)
    (launch / "child.launch.py").write_text(
        "from launch import LaunchDescription\n"
        "from launch.actions import DeclareLaunchArgument\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription([DeclareLaunchArgument('gz_args', default_value='')])\n"
    )
    first, second = (
        extract.run(cfg, Workspace.discover(cfg.roots), collectors=["launch_declared"])
        for _ in range(2)
    )
    assert first.canonical_bytes() == second.canonical_bytes()
    text = model.dumps(first.to_json())
    assert "nav2_" not in text and str(tmp_path) not in text
    assert "sandbox refused" not in text
    assert not [f for f in first.findings if f.rule == "launch.raised"]
    assert "@output(xacro -o @output headless:=True world.sdf.xacro)" in text
    assert any("xacro -o @output" in entry.reason for entry in first.unresolved)
    assert ids.build_node("/talker") in first.entities
