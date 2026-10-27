"""Node classes declared in a header, defined in a sibling source and constructed in main.

The fixture is `tests/fixtures/ws_split`: one class whose topic is a constructor argument,
five mains that pass it in five ways, and one class whose parameter defaults are members the
constructor or the class body initialises.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from rosio import extract, join, model, schema
from rosio.collect import cpp_code as cpp
from rosio.config import Config, RootSpec
from rosio.workspace import Workspace

pytestmark = pytest.mark.cpp

PACKAGE = Path(__file__).parent / "fixtures" / "ws_split" / "src" / "split_nodes"


def _units(drop: tuple[str, ...] = ()) -> dict[str, cpp.Unit]:
    units = {
        path.relative_to(PACKAGE).as_posix(): cpp.parse_file(
            str(path), pkg="split_nodes", relpath=path.relative_to(PACKAGE).as_posix()
        )
        for path in sorted(PACKAGE.rglob("*.?pp"))
        if path.name not in drop
    }
    cpp.analyse(list(units.values()), include_dirs={"split_nodes": ["include"]})
    return units


def _topics(units: dict[str, cpp.Unit]) -> dict[str, list[str | None]]:
    by_main: dict[str, list[str | None]] = {}
    report = cpp.file_report(units["src/nodes.cpp"])
    contexts = {
        row["construction"]: relpath
        for relpath, unit in units.items()
        for row in cpp.file_report(unit)["nodes"]
        if row.get("construction")
    }
    for row in report["endpoints"]:
        if row["kind"] == "publisher":
            by_main.setdefault(contexts.get(row.get("construction", ""), ""), []).append(
                row["name"]
            )
    return by_main


def test_each_main_states_the_topic_its_construction_passes():
    assert _topics(_units()) == {
        "src/stated_main.cpp": ["stated_chatter"],
        "src/default_main.cpp": ["default_chatter"],
        "src/constant_main.cpp": ["other_chatter"],
        "src/argv_main.cpp": [None],
    }


def test_a_branch_written_argument_stays_a_blind_spot():
    report = cpp.file_report(_units()["src/nodes.cpp"])
    blind = [row for row in report["unresolved"] if row["want"] == "topic_name"]
    assert [(row["expr"], row["owner_symbol"]) for row in blind] == [
        ("topic_", "Talker::initialize")
    ]


def test_without_the_header_nothing_is_scoped_or_guessed():
    units = _units(drop=("nodes.hpp",))
    report = cpp.file_report(units["src/nodes.cpp"])
    names = [row["name"] for row in report["endpoints"] if row["kind"] == "publisher"]
    assert names == [None]
    assert not any(row.get("construction") for row in report["endpoints"])


def test_a_registered_component_keeps_its_unscoped_records():
    units = {}
    for path in sorted(PACKAGE.rglob("*.?pp")):
        relpath = path.relative_to(PACKAGE).as_posix()
        text = path.read_text()
        if relpath == "src/nodes.cpp":
            text += "\nRCLCPP_COMPONENTS_REGISTER_NODE(Talker)\n"
        units[relpath] = cpp.parse_text(text, path=relpath, pkg="split_nodes", relpath=relpath)
    cpp.analyse(list(units.values()), include_dirs={"split_nodes": ["include"]})
    report = cpp.file_report(units["src/nodes.cpp"])
    publishers = [row for row in report["endpoints"] if row["kind"] == "publisher"]
    assert [row.get("construction") for row in publishers] == [None]


def test_a_member_default_is_the_literal_only_when_nothing_else_writes_it():
    report = cpp.file_report(_units()["src/nodes.cpp"])
    defaults = {
        row["name"]: (row["default"], row.get("default_state", "known"))
        for row in report["parameters"]
    }
    assert defaults == {
        "gain": (0.5, "known"),
        "offset": (None, "unresolved"),
        "scale": (2.0, "known"),
        "window": (4, "known"),
    }
    assert isinstance(defaults["window"][0], int) and not isinstance(defaults["window"][0], bool)
    blind = [row["expr"] for row in report["unresolved"] if row["want"] == "param_value"]
    assert blind == ["offset_"]


def test_a_second_constructor_leaves_the_member_default_unknown():
    header = (PACKAGE / "include" / "split_nodes" / "nodes.hpp").read_text()
    source = (PACKAGE / "src" / "nodes.cpp").read_text()
    units = [
        cpp.parse_text(
            header.replace("  Filter();", "  Filter();\n  explicit Filter(double gain);"),
            path="nodes.hpp",
            pkg="split_nodes",
            relpath="include/split_nodes/nodes.hpp",
        ),
        cpp.parse_text(
            source + '\nFilter::Filter(double gain)\n: Node("filter"),\n  gain_(gain) {}\n',
            path="nodes.cpp",
            pkg="split_nodes",
            relpath="src/nodes.cpp",
        ),
    ]
    cpp.analyse(units)
    defaults = {
        row["name"]: row.get("default_state", "known")
        for row in cpp.file_report(units[1])["parameters"]
    }
    assert defaults["gain"] == "unresolved"
    assert defaults["scale"] == "unresolved"


@pytest.mark.parametrize(
    "write",
    [
        "auto self = this; self->gain_ = 3.0;",
        "double & alias = gain_; alias = 3.0;",
        "std::swap(gain_, offset_);",
        'this->get_parameter("gain", gain_);',
    ],
)
def test_a_write_anywhere_else_leaves_the_member_default_unknown(write):
    header = (PACKAGE / "include" / "split_nodes" / "nodes.hpp").read_text()
    source = (PACKAGE / "src" / "nodes.cpp").read_text().replace("offset_ = 3.0;", write)
    units = [
        cpp.parse_text(header, path="nodes.hpp", pkg="split_nodes", relpath="include/n.hpp"),
        cpp.parse_text(source, path="nodes.cpp", pkg="split_nodes", relpath="src/nodes.cpp"),
    ]
    cpp.analyse(units)
    defaults = {
        row["name"]: row.get("default_state", "known")
        for row in cpp.file_report(units[1])["parameters"]
    }
    assert defaults["gain"] == "unresolved"
    assert defaults["window"] == "known"


@pytest.mark.ros
def test_the_fixture_extracts_twice_into_the_same_valid_atlas(tmp_path):
    roots = (RootSpec(kind="src", path=str(PACKAGE.parent)),)
    cfg = Config(roots=roots, profiles=(), out_dir=str(tmp_path / "out"), deterministic=True)
    first = extract.run(cfg, Workspace.discover(roots))
    second = extract.run(cfg, Workspace.discover(roots))
    assert schema.validate(first.to_json()) == []
    assert first.canonical_bytes() == second.canonical_bytes()
    topics = sorted(row.expr for row in first.unresolved if row.want == model.Want.TOPIC_NAME.value)
    assert topics == ["topic_"]


BENCH_SOURCES_ENV = "ROSIO_BENCH_SOURCES"

# Measured on the public source checkouts this change was written against: ros2/demos and
# robot_localization over a jazzy install, each with its own rosio.yaml beside `src`.
PUBLIC_SOURCES = {
    "demos_src": {
        "unresolved": 13,
        "findings": {"extract.input_refused": 1},
        # One of them is pkg:@repo, the workspace every extraction writes.
        "entities": 1883,
        "edges": 2404,
    },
    "rl_src": {
        "unresolved": 58,
        "findings": {
            "extract.input_refused": 1,
            "param.set_undeclared": 1,
            "resolution.below_budget": 1,
        },
        # The yaml or the code default reads publish_acceleration and publish_filtered_gps
        # false in four node profiles, so four publishes edges, the uses_type edge and the
        # /accel/filtered topic are not bound.
        "entities": 1684,
        "edges": 2328,
    },
}


@pytest.mark.ros
@pytest.mark.slow
@pytest.mark.parametrize("stack", sorted(PUBLIC_SOURCES))
def test_public_source_stacks_lose_no_output(stack, tmp_path):
    """No more blind spots, the same findings and the same entity and edge counts.

    Skips unless `ROSIO_BENCH_SOURCES` names a directory holding the two workspaces.
    """
    base = os.environ.get(BENCH_SOURCES_ENV)
    if not base:
        pytest.skip(f"{BENCH_SOURCES_ENV} is not set")
    workspace = Path(base) / stack
    if not (workspace / "rosio.yaml").is_file():
        pytest.skip(f"{workspace} holds no rosio.yaml")
    out = tmp_path / stack
    done = subprocess.run(
        [
            sys.executable,
            *("-m", "rosio", "extract", "."),
            *("--config", "rosio.yaml", "--out", str(out), "--quiet"),
        ],
        capture_output=True,
        text=True,
        cwd=workspace,
    )
    assert done.returncode == 0, done.stderr[-800:]
    atlas = json.loads((out / "atlas.json").read_text(encoding="utf-8"))
    expected = PUBLIC_SOURCES[stack]
    assert len(atlas["unresolved"]) <= expected["unresolved"]
    rules: dict[str, int] = {}
    for row in atlas["findings"]:
        rules[row["rule"]] = rules.get(row["rule"], 0) + 1
    assert rules == expected["findings"]
    assert len(atlas["entities"]) == expected["entities"]
    assert len(atlas["edges"]) == expected["edges"]


@pytest.mark.ros
def test_two_blind_contexts_of_one_source_stay_two_blind_spots(tmp_path):
    package = tmp_path / "src" / "split_nodes"
    shutil.copytree(PACKAGE, package)
    argv = (package / "src" / "argv_main.cpp").read_text()
    (package / "src" / "argv2_main.cpp").write_text(argv.replace("argv_chatter", "argv2_chatter"))
    cmake = package / "CMakeLists.txt"
    cmake.write_text(cmake.read_text().replace(" argv ", " argv argv2 "))
    roots = (RootSpec(kind="src", path=str(tmp_path / "src")),)
    cfg = Config(roots=roots, profiles=(), out_dir=str(tmp_path / "out"), deterministic=True)
    atlas = extract.run(cfg, Workspace.discover(roots))
    assert schema.validate(atlas.to_json()) == []
    topics = [row for row in atlas.unresolved if row.want == model.Want.TOPIC_NAME.value]
    assert [row.expr for row in topics] == ["topic_", "topic_"]
    assert len({row.construction for row in topics}) == 2


def test_a_source_blind_spot_retires_per_construction():
    builder = model.AtlasBuilder()
    owner = "sym:p|src/nodes.cpp|Talker::initialize"
    builder.node(owner)
    for context in ("a", "b"):
        builder.unresolved(
            model.Unresolved(
                want=model.Want.TOPIC_NAME.value,
                owner=owner,
                expr="topic_",
                reason="from_parameter",
                role="cpp_source",
                construction=context,
            )
        )
    report = join.JoinReport()
    report.resolved_sites.add((model.Want.TOPIC_NAME.value, owner, "topic_", "a"))
    report.blocked_sites.add((model.Want.TOPIC_NAME.value, owner, "topic_", "b"))
    join._retire_resolved_sites(builder, report)
    assert [row.construction for row in builder.atlas.unresolved] == ["b"]
