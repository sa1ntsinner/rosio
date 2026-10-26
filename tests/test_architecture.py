"""The module map, held to the tree it describes.

ARCHITECTURE.md listed three drift modules and one agent module that do not exist, and left
out the drift engine, the drift build family, the history module, the merge driver, the
serve package, the agent package, the identity resolver, the python pipeline resolver and
the process collector. rosio reports for a living that a written description and a real
system have drifted apart, so this file is the same check turned on the project itself, in
the shape `tests/test_cli_docs.py` already uses for the published numbers.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DOCUMENT = "ARCHITECTURE.md"

MODULE = re.compile(r"`(rosio/[A-Za-z0-9_/]+\.py)`")


def doc(name: str) -> str:
    path = ROOT / name
    if not path.is_file():
        pytest.skip(f"{name} is not in this checkout")
    return path.read_text(encoding="utf-8")


def modules_on_disk(package: Path) -> set[str]:
    parent = package.parent
    return {path.relative_to(parent).as_posix() for path in package.glob("**/*.py")}


def modules_listed(text: str) -> set[str]:
    """The first cell of every table row, which is where a module is listed."""
    found: set[str] = set()
    for line in text.splitlines():
        if not line.startswith("|"):
            continue
        first = line.strip().strip("|").split("|")[0].strip()
        found.update(MODULE.findall(first))
    return found


def modules_mentioned(text: str) -> set[str]:
    """Every module path the document names anywhere, table or prose."""
    return set(MODULE.findall(text))


def disagreement(package: Path, text: str) -> tuple[list[str], list[str]]:
    """What the document names that is not there, and what is there and unnamed."""
    real = modules_on_disk(package)
    absent = sorted(name for name in modules_mentioned(text) if name not in real)
    unlisted = sorted(real - modules_listed(text))
    return absent, unlisted


def test_the_map_names_no_module_that_does_not_exist():
    absent, _ = disagreement(ROOT / "rosio", doc(DOCUMENT))
    assert absent == []


def test_the_map_lists_every_module_that_exists():
    _, unlisted = disagreement(ROOT / "rosio", doc(DOCUMENT))
    assert unlisted == []


def test_every_module_is_listed_once():
    text = doc(DOCUMENT)
    rows = [
        line.strip().strip("|").split("|")[0].strip()
        for line in text.splitlines()
        if line.startswith("|")
    ]
    named = [name for cell in rows for name in MODULE.findall(cell)]
    assert sorted(named) == sorted(set(named))


def test_every_listed_module_carries_a_description():
    text = doc(DOCUMENT)
    thin = []
    for line in text.splitlines():
        if not line.startswith("|"):
            continue
        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        if len(cells) < 2 or not MODULE.findall(cells[0]):
            continue
        if len(cells[1].split()) < 4:
            thin.append(cells[0])
    assert thin == []


# ---------------------------------------------------------------------------
# the check, checked


def fixture_tree(tmp_path: Path, names: list[str]) -> Path:
    package = tmp_path / "rosio"
    for name in names:
        target = package / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text("", encoding="utf-8")
    return package


def test_a_renamed_module_is_reported_from_both_sides(tmp_path: Path):
    package = fixture_tree(tmp_path, ["model.py", "drift/engine.py"])
    text = "| `rosio/model.py` | the model |\n| `rosio/drift/rules.py` | the vocabulary |\n"
    assert disagreement(package, text) == (["rosio/drift/rules.py"], ["rosio/drift/engine.py"])


def test_a_module_named_only_in_prose_is_still_unlisted(tmp_path: Path):
    package = fixture_tree(tmp_path, ["model.py", "history.py"])
    text = "| `rosio/model.py` | the model |\n\n`rosio/history.py` runs on a git hook.\n"
    assert disagreement(package, text) == ([], ["rosio/history.py"])


def test_a_map_that_matches_its_tree_reports_nothing(tmp_path: Path):
    package = fixture_tree(tmp_path, ["model.py", "serve/tools.py"])
    text = "| `rosio/model.py` | the model |\n| `rosio/serve/tools.py` | the tools |\n"
    assert disagreement(package, text) == ([], [])
