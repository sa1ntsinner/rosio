"""What a release carries: the sdist and the wheel held to AGENTS section 1 rule 9, one version
across every shipped part, and a vsix holding what the extension loads and nothing else."""

from __future__ import annotations

import fnmatch
import importlib.util
import json
import re
import shutil
import subprocess
import sys
import tarfile
import zipfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
# hatchling always copies .gitignore into the sdist, so a rebuild excludes what this tree does
ALLOWED = {"rosio", "LICENSE", "NOTICE", "README.md", "pyproject.toml", "PKG-INFO", ".gitignore"}
FORBIDDEN_DIRS = {"bench", "corpus", "examples", "tests", "editors", "crates", "target"}
MACHINE_PATHS = (b"D:\\", b"C:\\Users", b"/home/", b"/mnt/")


@pytest.fixture(scope="module")
def artefacts(tmp_path_factory: pytest.TempPathFactory) -> dict[str, dict[str, bytes]]:
    if importlib.util.find_spec("build") is None:
        pytest.skip("the build package is not installed, pip install build")
    out = tmp_path_factory.mktemp("dist")
    command = [sys.executable, "-m", "build", "--sdist", "--wheel", "--outdir", str(out)]
    # a python without ensurepip cannot make the isolated environment, uv can
    if shutil.which("uv"):
        command.append("--installer=uv")
    done = subprocess.run([*command, str(ROOT)], capture_output=True, text=True, check=False)
    assert done.returncode == 0, done.stdout + done.stderr
    (sdist,) = out.glob("*.tar.gz")
    (wheel,) = out.glob("*.whl")
    with tarfile.open(sdist) as archive:
        members = {
            m.name.split("/", 1)[1]: archive.extractfile(m).read() for m in archive if m.isfile()
        }
    with zipfile.ZipFile(wheel) as archive:
        wheel_members = {name: archive.read(name) for name in archive.namelist()}
    return {"sdist": members, "wheel": wheel_members}


def top(name: str) -> str:
    first = name.split("/", 1)[0]
    return "metadata" if first.endswith(".dist-info") else first


@pytest.mark.parametrize("kind", ["sdist", "wheel"])
def test_every_member_is_under_an_allowed_top_level(artefacts, kind):
    stray = sorted(name for name in artefacts[kind] if top(name) not in ALLOWED | {"metadata"})
    assert stray == []


def test_the_wheel_holds_no_repository_directory(artefacts):
    parts = {part for name in artefacts["wheel"] for part in name.split("/")[:-1]}
    assert parts.isdisjoint(FORBIDDEN_DIRS), sorted(parts & FORBIDDEN_DIRS)


@pytest.mark.parametrize("kind", ["sdist", "wheel"])
def test_no_member_names_a_machine_path(artefacts, kind):
    found = sorted(
        f"{name}: {marker.decode()}"
        for name, data in artefacts[kind].items()
        for marker in MACHINE_PATHS
        if marker in data
    )
    assert found == []


# ---------------------------------------------------------------------------
# what a tag releases: one version everywhere, and a vsix that holds what the extension loads


def _version(path: str, pattern: str) -> str:
    return re.search(pattern, (ROOT / path).read_text(encoding="utf-8"), re.MULTILINE).group(1)


def test_every_shipped_part_states_one_version():
    from rosio import __version__

    toml = r'^version = "([^"]+)"'
    stated = {
        "rosio/__init__.py": __version__,
        "pyproject.toml": _version("pyproject.toml", toml),
        "editors/vscode/package.json": json.loads(
            (ROOT / "editors/vscode/package.json").read_text(encoding="utf-8")
        )["version"],
        "editors/vscode/package-lock.json": json.loads(
            (ROOT / "editors/vscode/package-lock.json").read_text(encoding="utf-8")
        )["version"],
        "crates/rosio-lsp/Cargo.toml": _version("crates/rosio-lsp/Cargo.toml", toml),
        "Cargo.lock": _version("Cargo.lock", r'^name = "rosio-lsp"\nversion = "([^"]+)"'),
        "editors/zed/extension.toml": _version("editors/zed/extension.toml", toml),
        "editors/zed/Cargo.toml": _version("editors/zed/Cargo.toml", toml),
    }
    assert len(set(stated.values())) == 1, stated


VSCODE = ROOT / "editors" / "vscode"


def _vscode_ignored(path: str) -> bool:
    patterns = (VSCODE / ".vscodeignore").read_text(encoding="utf-8").split()
    return any(fnmatch.fnmatch(path, pattern) for pattern in patterns)


def test_the_vsix_carries_every_bundle_and_stylesheet_the_extension_loads():
    build = (VSCODE / "scripts" / "build.mjs").read_text(encoding="utf-8")
    bundles = set(re.findall(r'outfile: "(out/[\w.]+)"', build))
    source = "".join(p.read_text(encoding="utf-8") for p in (VSCODE / "src").glob("*.ts"))
    loaded = {f"out/{name}" for name in re.findall(r'joinPath\(out, "([\w.]+)"\)', source)}
    styles = {f"media/{name}" for name in re.findall(r'joinPath\(media, "([\w.]+)"\)', source)}
    assert {"out/robot.js", "media/robot.css"} <= loaded | styles
    assert loaded <= bundles, sorted(loaded - bundles)
    assert all((VSCODE / name).is_file() for name in styles), sorted(styles)
    shipped = bundles | styles | {"media/icon.png", "server/rosio-lsp", "server/rosio-lsp.exe"}
    assert [name for name in sorted(shipped) if _vscode_ignored(name)] == []


def test_the_vsix_leaves_out_sources_maps_tests_and_the_other_targets():
    workflow = (ROOT / ".github" / "workflows" / "release.yml").read_text(encoding="utf-8")
    # the release job downloads every target's server here before packaging one per target
    assert "path: editors/vscode/servers" in workflow
    for name in (
        "servers/rosio-lsp-linux-x64",
        "out/robot.js.map",
        "src/robot.ts",
        "webview/robotModel.ts",
        "test-e2e/run.mjs",
        "test-unit/robot.test.cjs",
        "node_modules/x/index.js",
    ):
        assert _vscode_ignored(name), name
