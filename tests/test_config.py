"""Run configuration: roots, profiles, entry points and the default fallback."""

from __future__ import annotations

from pathlib import Path

import pytest

from rosio.config import (
    CONFIG_ENV,
    CONFIG_NAME,
    Config,
    Entrypoint,
    ProfileSpec,
    RootSpec,
    default_config,
    load_config,
)
from rosio.errors import UsageError
from rosio.workspace import Workspace

MANIFEST = "<package format='3'><name>{name}</name><export/></package>"


def _write(path: Path, text: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return path


@pytest.fixture
def bench(workspace: Path) -> Path:
    """Two packages, three launch files, one of them a name that collides."""
    for name in ("alpha_pkg", "beta_pkg"):
        _write(workspace / "src" / name / "package.xml", MANIFEST.format(name=name))
        _write(
            workspace / "src" / name / "launch" / "mapping.launch.py",
            "def generate_launch_description():\n    pass\n",
        )
    _write(
        workspace / "src" / "alpha_pkg" / "launch" / "nx_navigation.launch.py",
        "def generate_launch_description():\n    pass\n",
    )
    return workspace


def test_default_config_has_one_profile_per_launch_file(bench: Path) -> None:
    cfg = default_config(str(bench))
    slugs = [profile.slug for profile in cfg.profiles]
    assert slugs == ["alpha_pkg.mapping", "beta_pkg.mapping", "nx_navigation"]
    assert cfg.out_dir == "rosio-out"
    assert cfg.deterministic is False
    assert [spec.kind for spec in cfg.roots] == ["src", "install"]


def test_default_config_entrypoint_names_its_origin(bench: Path) -> None:
    cfg = default_config(str(bench))
    profile = cfg.profile("nx_navigation")
    entry = profile.entrypoints[0]
    assert entry.verb == "launch"
    assert entry.pkg == "alpha_pkg"
    assert entry.target == "nx_navigation.launch.py"
    assert entry.origin is not None
    assert entry.origin.path.endswith("launch/nx_navigation.launch.py")


def test_default_config_accepts_a_discovered_workspace(bench: Path) -> None:
    ws = Workspace.discover([RootSpec("src", str(bench / "src"))])
    cfg = default_config(ws)
    assert [spec.path for spec in cfg.roots] == [str(bench / "src")]
    assert cfg.profiles


def test_default_config_refuses_a_missing_directory(tmp_path: Path) -> None:
    with pytest.raises(UsageError):
        default_config(str(tmp_path / "nope"))


def test_an_absent_config_file_is_not_an_error(bench: Path) -> None:
    assert load_config(None, workspace=str(bench)).profiles


def test_an_explicit_missing_config_file_is_an_error(bench: Path) -> None:
    with pytest.raises(UsageError):
        load_config(str(bench / "nope.yaml"), workspace=str(bench))


def test_load_config_reads_roots_profiles_and_settings(bench: Path) -> None:
    _write(
        bench / CONFIG_NAME,
        """
roots:
  - kind: src
    path: src
  - kind: install
    path: install
out: atlas-out
deterministic: true
launch_timeout: 12.5
ros_setup: /opt/ros/humble/setup.bash
ignore:
  - "*.bag"
profiles:
  - slug: nx_navigation
    label: Navigation
    env:
      NX_WS_ROOT: /srv/nx_ws
    entrypoints:
      - launch: alpha_pkg nx_navigation.launch.py
        args:
          params_file: /tmp/merged.yaml
          use_sim_time: false
      - run: beta_pkg motion_supervisor
        argv: ["--ros-args", "-p", "orientation.required:=true"]
""",
    )
    cfg = load_config(None, workspace=str(bench))
    assert cfg.out_dir == "atlas-out"
    assert cfg.deterministic is True
    assert cfg.launch_timeout == 12.5
    assert cfg.ros_setup == ("/opt/ros/humble/setup.bash",)
    assert cfg.ignore == ("*.bag",)
    assert [spec.path for spec in cfg.roots] == [str(bench / "src"), str(bench / "install")]
    profile = cfg.profile("nx_navigation")
    assert profile.label == "Navigation"
    assert profile.env == (("NX_WS_ROOT", "/srv/nx_ws"),)
    first, second = profile.entrypoints
    assert first.verb == "launch"
    assert first.args == (("params_file", "/tmp/merged.yaml"), ("use_sim_time", "false"))
    assert second.verb == "run"
    assert second.pkg == "beta_pkg"
    assert second.target == "motion_supervisor"
    assert second.argv == ("--ros-args", "-p", "orientation.required:=true")


def test_config_env_variable_is_honoured(bench: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    elsewhere = _write(
        bench / "other.yaml",
        "profiles:\n"
        "  - slug: only\n"
        "    entrypoints:\n"
        "      - launch: alpha_pkg mapping.launch.py\n",
    )
    monkeypatch.setenv(CONFIG_ENV, str(elsewhere))
    cfg = load_config(None, workspace=str(bench))
    assert [profile.slug for profile in cfg.profiles] == ["only"]


def test_config_without_profiles_falls_back_to_launch_files(bench: Path) -> None:
    _write(bench / CONFIG_NAME, "roots:\n  - kind: src\n    path: src\n")
    cfg = load_config(None, workspace=str(bench))
    assert [profile.slug for profile in cfg.profiles] == [
        "alpha_pkg.mapping",
        "beta_pkg.mapping",
        "nx_navigation",
    ]


def test_a_bare_string_root_is_a_source_root(bench: Path) -> None:
    _write(bench / CONFIG_NAME, "roots:\n  - src\n")
    cfg = load_config(None, workspace=str(bench))
    assert cfg.roots == (RootSpec("src", str(bench / "src")),)


def test_a_command_string_is_an_entrypoint(bench: Path) -> None:
    _write(
        bench / CONFIG_NAME,
        "profiles:\n"
        "  - slug: slam\n"
        "    entrypoints:\n"
        "      - ros2 launch alpha_pkg mapping.launch.py localization:=true map:=/tmp/yard\n",
    )
    entry = load_config(None, workspace=str(bench)).profile("slam").entrypoints[0]
    assert entry.pkg == "alpha_pkg"
    assert entry.target == "mapping.launch.py"
    assert entry.args == (("localization", "true"), ("map", "/tmp/yard"))


def test_args_as_a_list_of_assignments(bench: Path) -> None:
    _write(
        bench / CONFIG_NAME,
        "profiles:\n"
        "  - slug: slam\n"
        "    entrypoints:\n"
        "      - verb: launch\n"
        "        pkg: alpha_pkg\n"
        "        target: mapping.launch.py\n"
        "        args: [localization:=true, map=/tmp/yard]\n",
    )
    entry = load_config(None, workspace=str(bench)).profile("slam").entrypoints[0]
    assert entry.args == (("localization", "true"), ("map", "/tmp/yard"))


@pytest.mark.parametrize(
    "body",
    [
        "profils: []\n",
        "roots: {}\n",
        "roots:\n  - kind: nowhere\n    path: src\n",
        "profiles:\n  - slug: a\n    unknown: 1\n",
        "profiles:\n  - slug: a\n    entrypoints:\n      - verb: sing\n        pkg: p\n"
        "        target: t\n",
        "profiles:\n  - slug: a\n    entrypoints:\n      - launch: only_one_word\n",
        "profiles:\n  - slug: a\n  - slug: a\n",
        "profiles:\n  - label: no slug\n",
        "- not a mapping\n",
    ],
)
def test_bad_config_is_a_usage_error(bench: Path, body: str) -> None:
    _write(bench / CONFIG_NAME, body)
    with pytest.raises(UsageError):
        load_config(None, workspace=str(bench))


def test_config_lookups(bench: Path) -> None:
    cfg = default_config(str(bench))
    assert cfg.root("src") == str(bench / "src")
    assert cfg.root("bag") is None
    with pytest.raises(UsageError):
        cfg.profile("nope")


def test_a_config_needs_a_root() -> None:
    with pytest.raises(UsageError):
        Config(roots=(), profiles=())


def test_entrypoint_and_profile_validation() -> None:
    with pytest.raises(UsageError):
        Entrypoint(verb="sing", pkg="p", target="t")
    with pytest.raises(UsageError):
        Entrypoint(verb="launch", pkg="", target="t")
    with pytest.raises(UsageError):
        ProfileSpec(slug="")
    with pytest.raises(UsageError):
        RootSpec("elsewhere", "/tmp")
