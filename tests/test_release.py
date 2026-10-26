"""What the sdist and the wheel carry, held to AGENTS section 1 rule 9."""

from __future__ import annotations

import importlib.util
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
