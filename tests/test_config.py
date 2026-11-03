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
    locate_config,
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


def test_a_repository_that_is_one_package_is_its_own_src_root(workspace: Path) -> None:
    """A cloned single package repository holds `src/` for its C++ sources, not packages."""
    _write(workspace / "package.xml", MANIFEST.format(name="solo"))
    _write(workspace / "src" / "node.cpp", "int main() {}\n")
    cfg = default_config(str(workspace))
    assert [(spec.kind, Path(spec.path)) for spec in cfg.roots] == [("src", workspace.absolute())]


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


def test_an_empty_profile_list_is_taken_as_written(bench: Path) -> None:
    """perception_pcl: init wrote `profiles: []`, extract read it as absent and walked one
    profile per launch file, ROS 1 files among them."""
    _write(bench / CONFIG_NAME, "roots:\n  - kind: src\n    path: src\nprofiles: []\n")
    assert load_config(None, workspace=str(bench)).profiles == ()


def test_the_fallback_leaves_out_ros1_and_test_launch_files(bench: Path) -> None:
    _write(
        bench / "src" / "beta_pkg" / "launch" / "legacy.launch",
        '<launch>\n  <node pkg="beta_pkg" type="talker" name="talker"/>\n</launch>\n',
    )
    _write(
        bench / "src" / "beta_pkg" / "test" / "smoke.launch.py",
        "def generate_launch_description():\n    pass\n",
    )
    _write(bench / CONFIG_NAME, "roots:\n  - kind: src\n    path: src\n")
    cfg = load_config(None, workspace=str(bench))
    assert cfg.ignore == ("test", "tests")
    assert [profile.slug for profile in cfg.profiles] == [
        "alpha_pkg.mapping",
        "beta_pkg.mapping",
        "nx_navigation",
    ]
    stated = _write(bench / "other.yaml", "roots:\n  - kind: src\n    path: src\nignore: []\n")
    assert load_config(str(stated), workspace=str(bench)).ignore == ()


def test_the_default_ignore_keeps_a_test_launch_file_a_profile_names(tmp_path: Path) -> None:
    """`ros2 launch bot test/launch/hil.launch.py` was skipped by the default ignore and
    reported as launch.unloadable with a launch_file_missing blind spot."""
    from rosio.collect.launch_declared import _launch_index
    from rosio.config import _as_workspace

    _write(tmp_path / "src" / "bot" / "package.xml", MANIFEST.format(name="bot"))
    _write(
        tmp_path / "src" / "bot" / "test" / "launch" / "hil.launch.py",
        "def generate_launch_description():\n    pass\n",
    )
    _write(
        tmp_path / CONFIG_NAME,
        "roots:\n  - kind: src\n    path: src\nprofiles:\n  - slug: hil\n"
        "    entrypoints: ['ros2 launch bot test/launch/hil.launch.py']\n",
    )
    cfg = load_config(None, workspace=str(tmp_path))
    assert cfg.ignore == ("tests",)
    ws = _as_workspace(str(tmp_path), roots=cfg.roots, ignore=cfg.ignore)
    assert ("bot", "test/launch/hil.launch.py") in _launch_index(ws)


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


def test_a_top_level_env_says_it_belongs_under_a_profile(bench: Path) -> None:
    _write(bench / CONFIG_NAME, "env:\n  TURTLEBOT3_MODEL: burger\n")
    with pytest.raises(UsageError, match="env goes under a profile"):
        load_config(None, workspace=str(bench))


def test_a_config_in_the_working_directory_is_read_after_the_workspace_one(
    bench: Path, tmp_path_factory: pytest.TempPathFactory, monkeypatch: pytest.MonkeyPatch
) -> None:
    """`rosio init /opt/ros/jazzy --config ./rosio.yaml` then `rosio extract /opt/ros/jazzy`
    from the same directory walked every launch file of the distribution."""
    here = tmp_path_factory.mktemp("here")
    monkeypatch.chdir(here)
    monkeypatch.delenv(CONFIG_ENV, raising=False)
    assert locate_config(None, workspace=str(bench)) is None
    _write(here / CONFIG_NAME, f"roots:\n  - {bench}\nprofiles:\n  - slug: from_here\n")
    assert locate_config(None, workspace=str(bench)) == here / CONFIG_NAME
    assert [p.slug for p in load_config(None, workspace=str(bench)).profiles] == ["from_here"]
    _write(bench / CONFIG_NAME, "profiles:\n  - slug: from_workspace\n")
    assert locate_config(None, workspace=str(bench)) == bench / CONFIG_NAME
    explicit = _write(here / "other.yaml", "profiles:\n  - slug: named\n")
    assert locate_config(str(explicit), workspace=str(bench)) == explicit


def test_a_config_in_the_working_directory_written_for_another_tree_is_ignored(
    bench: Path, tmp_path_factory: pytest.TempPathFactory, monkeypatch: pytest.MonkeyPatch
) -> None:
    """One workspace gave two atlases, by the directory the shell stood in."""
    here = tmp_path_factory.mktemp("elsewhere")
    monkeypatch.chdir(here)
    monkeypatch.delenv(CONFIG_ENV, raising=False)
    notes: list[str] = []
    _write(here / CONFIG_NAME, "profiles:\n  - slug: elsewhere\n")
    assert locate_config(None, workspace=str(bench), note=notes.append) is None
    assert [p.slug for p in load_config(None, workspace=str(bench)).profiles] != ["elsewhere"]
    (said,) = notes
    assert str(here / CONFIG_NAME) in said and "--config" in said
    _write(here / CONFIG_NAME, f"roots:\n  - {here}\nprofiles:\n  - slug: elsewhere\n")
    assert locate_config(None, workspace=str(bench)) is None
    _write(here / CONFIG_NAME, f"roots:\n  - {bench / 'src'}\nprofiles:\n  - slug: inside\n")
    assert locate_config(None, workspace=str(bench)) == here / CONFIG_NAME


def test_a_resolution_budget_key_that_names_no_rate_is_refused(bench: Path) -> None:
    """`frame_ids` loaded, entered the fingerprint and never gated anything."""
    _write(bench / CONFIG_NAME, "resolution_budget: {frame_ids: 0.99}\n")
    with pytest.raises(UsageError, match="no rate called frame_ids") as refused:
        load_config(None, workspace=str(bench))
    assert "frame_id" in str(refused.value) and "overall" in str(refused.value)


def test_the_resolution_budget_is_a_config_key_that_decides_the_atlas(bench: Path) -> None:
    _write(bench / CONFIG_NAME, "resolution_budget: 0.8\n")
    stated = load_config(None, workspace=str(bench))
    assert stated.resolution_budget == 0.8
    _write(bench / CONFIG_NAME, "resolution_budget: {overall: 0.8, topic_name: 0.5}\n")
    per_want = load_config(None, workspace=str(bench))
    assert per_want.resolution_budget == {"overall": 0.8, "topic_name": 0.5}
    assert default_config(str(bench)).resolution_budget is None
    assert (
        len(
            {stated.fingerprint(), per_want.fingerprint(), default_config(str(bench)).fingerprint()}
        )
        == 3
    )
    _write(bench / CONFIG_NAME, "resolution_budget: high\n")
    with pytest.raises(UsageError, match="resolution_budget"):
        load_config(None, workspace=str(bench))
