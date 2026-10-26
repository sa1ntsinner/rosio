"""`rosio init`: the fallback configuration, written out and read back."""

from __future__ import annotations

import shutil
from pathlib import Path

import pytest
import yaml

from rosio import cli, scaffold
from rosio.config import CONFIG_NAME, default_config, load_config
from rosio.errors import UsageError

WS = Path(__file__).parent / "fixtures" / "ws_launch"


@pytest.fixture
def workspace(tmp_path: Path) -> Path:
    shutil.copytree(WS / "src", tmp_path / "src")
    return tmp_path


def test_the_written_file_reads_back_as_the_same_configuration(workspace: Path) -> None:
    target, written = scaffold.write(workspace)
    assert target == workspace / CONFIG_NAME
    read = load_config(str(target), workspace=str(workspace))
    assert [profile.slug for profile in read.profiles] == [p.slug for p in written.profiles]
    assert [profile.slug for profile in read.profiles] == [
        p.slug for p in default_config(str(workspace)).profiles
    ]
    assert {Path(spec.path).resolve() for spec in read.roots} == {
        Path(spec.path).resolve() for spec in written.roots
    }


def test_paths_inside_the_workspace_are_written_relative(workspace: Path) -> None:
    text = scaffold.render(default_config(str(workspace)), workspace)
    payload = yaml.safe_load(text)
    assert all(not Path(root["path"]).is_absolute() for root in payload["roots"])
    assert str(workspace) not in text
    assert text.startswith("# rosio configuration, written by `rosio init`.")


def test_an_existing_file_is_kept_unless_asked(workspace: Path) -> None:
    (workspace / CONFIG_NAME).write_text("roots: []\n", encoding="utf-8")
    with pytest.raises(UsageError, match="already exists"):
        scaffold.write(workspace)
    scaffold.write(workspace, force=True)
    assert "profiles:" in (workspace / CONFIG_NAME).read_text(encoding="utf-8")


def test_print_writes_nothing(workspace: Path, capsys: pytest.CaptureFixture[str]) -> None:
    assert cli.main(["init", str(workspace), "--print"]) == 0
    assert "profiles:" in capsys.readouterr().out
    assert not (workspace / CONFIG_NAME).exists()


def _installed(prefix: Path, package: str, launch: str = "bringup.launch.py") -> None:
    """One package as colcon and the distro packages install it: index marker and share."""
    index = prefix / "share" / "ament_index" / "resource_index" / "packages"
    index.mkdir(parents=True, exist_ok=True)
    (index / package).write_text("", encoding="utf-8")
    share = prefix / "share" / package
    (share / "launch").mkdir(parents=True)
    (share / "package.xml").write_text(
        f'<package format="3"><name>{package}</name></package>', encoding="utf-8"
    )
    (share / "launch" / launch).write_text("", encoding="utf-8")


def _profiles(cfg) -> dict[str, str]:
    return {
        profile.slug: f"{entry.pkg} {entry.target}"
        for profile in cfg.profiles
        for entry in profile.entrypoints
    }


@pytest.fixture
def merged(tmp_path: Path) -> Path:
    prefix = tmp_path / "jazzy"
    _installed(prefix, "alpha")
    _installed(prefix, "beta", "beta_launch.py")
    # /opt/ros/<distro> holds a src and launch files outside every package share.
    (prefix / "src" / "gtest").mkdir(parents=True)
    (prefix / "share" / "doc").mkdir()
    (prefix / "share" / "doc" / "example.launch.py").write_text("", encoding="utf-8")
    return prefix


def test_a_merged_install_tree_is_one_install_root(merged: Path) -> None:
    cfg = default_config(str(merged))
    assert [(spec.kind, spec.path) for spec in cfg.roots] == [("install", str(merged))]
    assert _profiles(cfg) == {"bringup": "alpha bringup.launch.py", "beta": "beta beta_launch.py"}
    payload = yaml.safe_load(scaffold.render(cfg, merged))
    assert payload["roots"] == [{"kind": "install", "path": "."}]
    assert [profile["slug"] for profile in payload["profiles"]] == ["beta", "bringup"]
    assert payload["profiles"][1]["entrypoints"] == [{"launch": "alpha bringup.launch.py"}]


def test_an_isolated_install_tree_is_one_install_root(tmp_path: Path) -> None:
    install = tmp_path / "ws" / "install"
    _installed(install / "alpha", "alpha")
    _installed(install / "beta", "beta")
    for path in (install, install.parent):
        cfg = default_config(str(path))
        assert [(spec.kind, spec.path) for spec in cfg.roots] == [("install", str(install))]
        assert _profiles(cfg) == {
            "alpha.bringup": "alpha bringup.launch.py",
            "beta.bringup": "beta bringup.launch.py",
        }


def test_a_workspace_with_only_install_reads_install(tmp_path: Path) -> None:
    _installed(tmp_path / "install", "alpha")
    cfg = default_config(str(tmp_path))
    assert [spec.kind for spec in cfg.roots] == ["install"]
    assert _profiles(cfg) == {"bringup": "alpha bringup.launch.py"}


def test_src_and_install_side_by_side_read_both(workspace: Path) -> None:
    from_src = _profiles(default_config(str(workspace)))
    _installed(workspace / "install", "binary_only")
    cfg = default_config(str(workspace))
    assert [spec.kind for spec in cfg.roots] == ["src", "install"]
    assert set(_profiles(cfg).values()) == {*from_src.values(), "binary_only bringup.launch.py"}


def test_a_package_filter_keeps_the_slugs_the_whole_tree_gives(merged: Path) -> None:
    _installed(merged, "gamma")
    whole = _profiles(default_config(str(merged)))
    only = _profiles(default_config(str(merged), ["alpha", "beta"]))
    assert only == {slug: label for slug, label in whole.items() if not label.startswith("gamma")}
    assert "alpha.bringup" in only
    with pytest.raises(UsageError, match="no launch file found in package: nowhere"):
        default_config(str(merged), ["alpha", "nowhere"])


def test_init_writes_beside_a_tree_it_may_not_write_into(merged: Path, tmp_path: Path) -> None:
    target = tmp_path / "elsewhere" / CONFIG_NAME
    target.parent.mkdir()
    argv = ["init", str(merged), "--package", "beta", "--config", str(target)]
    assert cli.main(argv) == 0
    assert not (merged / CONFIG_NAME).exists()
    payload = yaml.safe_load(target.read_text(encoding="utf-8"))
    assert payload["roots"] == [{"kind": "install", "path": str(merged)}]
    assert [profile["slug"] for profile in payload["profiles"]] == ["beta"]
    read = load_config(str(target), workspace=str(merged))
    assert [(spec.kind, spec.path) for spec in read.roots] == [("install", str(merged))]


def test_the_written_yaml_is_the_same_bytes_wherever_the_tree_sits(tmp_path: Path) -> None:
    texts = []
    for name in ("one", "two"):
        _installed(tmp_path / name / "install" / "beta", "beta")
        _installed(tmp_path / name / "install" / "alpha", "alpha", "alpha_launch.py")
        target, _cfg = scaffold.write(tmp_path / name)
        texts.append(target.read_bytes())
        assert scaffold.write(tmp_path / name, force=True)[0].read_bytes() == texts[-1]
    assert texts[0] == texts[1]
