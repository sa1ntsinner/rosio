"""Collect changed nav2 node defaults for `rosio migrate`.

Run `python corpus/defaults/collect.py --cache /path/outside/repository` from the
repository root with the cpp extra installed. The four shallow branch clones are
reused on later runs. Remove a cached clone to refresh that branch. Output is sorted
and byte stable for identical sources, with branch commits recorded for reproduction.

Plugin parameters, including dotted instance names, are excluded. A node name must
come from coded node rows or a component executable bound by the known name table.
Unknown defaults stay unknown.
An ambiguous declaration is excluded across releases. A declaration moved to an
unbound class in the same package is not evidence of removal from a node.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from itertools import pairwise
from pathlib import Path

from rosio.collect import cpp_code as cpp
from rosio.collect import plugins
from rosio.config import RootSpec
from rosio.errors import CollectorError, UsageError
from rosio.model import dumps
from rosio.resolve import known_names
from rosio.workspace import Workspace

BRANCHES = ("humble", "jazzy", "kilted", "lyrical")
REPOSITORY = "https://github.com/ros-navigation/navigation2"
ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "rosio/migrate/data/defaults.json"


def source_file(path: Path) -> bool:
    return path.suffix in cpp.SUFFIXES and not any(
        "test" in part.lower() or "benchmark" in part.lower() for part in path.parts
    )


def test_sources(base: Path) -> set[Path]:
    excluded, production = set(), set()
    for path in sorted(base.rglob("CMakeLists.txt")):
        text = path.read_text(encoding="utf-8")
        stack = []
        lines = set()
        for command in plugins.commands(text):
            if command.name == "if":
                stack.append("BUILD_TESTING" in command.args and "NOT" not in command.args)
            elif command.name in ("else", "elseif") and stack:
                stack[-1] = not stack[-1]
            elif command.name == "endif" and stack:
                stack.pop()
            if any(stack):
                lines.add(command.line_hint)
                if command.name == "add_subdirectory" and command.args:
                    directory = path.parent / command.args[0]
                    excluded.update(p.resolve() for p in directory.rglob("*") if p.is_file())
        for target in plugins.scan_cmake(text).targets.values():
            destinations = excluded if target.line_hint in lines else production
            destinations.update((path.parent / source).resolve() for source in target.sources)
    return excluded - production


def collect_release(root: Path, branch: str) -> dict:
    parser = cpp.load_parser()
    units = []
    incomplete = set()
    for manifest in sorted(root.rglob("package.xml")):
        try:
            package = ET.fromstring(manifest.read_text()).findtext("name")
        except (OSError, ET.ParseError) as exc:
            raise CollectorError(f"cannot read {manifest.relative_to(root)}") from exc
        excluded = test_sources(manifest.parent)
        for path in sorted(manifest.parent.rglob("*")):
            if (
                path.is_file()
                and source_file(path.relative_to(root))
                and path.resolve() not in excluded
            ):
                unit = cpp.parse_file(
                    str(path),
                    pkg=package,
                    relpath=path.relative_to(manifest.parent).as_posix(),
                    parser=parser,
                )
                if unit.hidden_regions:
                    incomplete.add(package)
                    print(
                        f"{branch}: unread ROS region in {path.relative_to(root)}", file=sys.stderr
                    )
                units.append(unit)
    cpp.analyse(units)
    workspace = Workspace.discover([RootSpec("src", str(root))])
    _, components = cpp.package_tables(workspace)
    names = defaultdict(set)
    sites = {}
    rows = defaultdict(list)
    for unit in units:
        report = cpp.file_report(unit)
        path = Path(unit.path).relative_to(root).as_posix()
        for node in report["nodes"]:
            key = (unit.pkg, node["class"])
            sites[key] = {"branch": branch, "path": path, "line": node["line_hint"]}
            if node["name"]:
                names[key].add(node["name"])
            else:
                names[key]
        for item in unit.params:
            if item.scope == "node" and item.name.resolved and "." not in item.name.value:
                rows[(unit.pkg, item.owner_class, item.name.value)].append(
                    {
                        "declared": True,
                        "default_state": item.default_state,
                        "default": item.default if item.default_state == "known" else None,
                        "source": {"branch": branch, "path": path, "line": item.line},
                    }
                )
    for (package, klass), candidates in names.items():
        executable = components.get(klass)
        known = known_names.name_for(package, executable) if executable else None
        if not candidates and known:
            candidates.add(known)
    result = {}
    for (package, klass), candidates in sorted(names.items()):
        if len(candidates) != 1 or package in incomplete:
            continue
        node = next(iter(candidates))
        ambiguous = []
        for key, values in sorted(rows.items()):
            if key[:2] != (package, klass):
                continue
            defaults = {(v["default_state"], dumps(v["default"])) for v in values}
            if len(defaults) != 1:
                ambiguous.append(key[2])
                continue
            result[key] = dict(values[0], node=node)
        result[(package, klass, "")] = {
            "node": node,
            "source": sites[(package, klass)],
            "ambiguous": ambiguous,
            "elsewhere": sorted({key[2] for key in rows if key[0] == package and key[1] != klass}),
        }
    return result


def differences(releases: dict) -> list[dict]:
    branches = [branch for branch in BRANCHES if branch in releases]
    keys = sorted({key for rows in releases.values() for key in rows if key[2]})
    result = []
    for package, klass, name in keys:
        values = {}
        nodes = set()
        ambiguous = False
        for branch, rows in releases.items():
            owner = rows.get((package, klass, ""))
            if owner is None:
                continue
            if name in owner["ambiguous"]:
                ambiguous = True
            row = rows.get((package, klass, name))
            if row is None and name in owner.get("elsewhere", ()):
                continue
            nodes.add(owner["node"])
            values[branch] = (
                {k: v for k, v in row.items() if k != "node"}
                if row
                else {"declared": False, "source": owner["source"]}
            )
        changed = False
        for old, new in pairwise(branches):
            a, b = values.get(old), values.get(new)
            if not a or not b or not a["declared"]:
                continue
            changed |= not b["declared"] or (
                a.get("default_state") == b.get("default_state") == "known"
                and dumps(a["default"]) != dumps(b["default"])
            )
        if changed and not ambiguous and len(nodes) == 1:
            result.append(
                {
                    "package": package,
                    "class": klass,
                    "node": next(iter(nodes)),
                    "name": name,
                    "releases": values,
                }
            )
    ambiguous_nodes = set()
    for branch in branches:
        by_name = defaultdict(set)
        for (_package, _klass, name), row in releases[branch].items():
            if name:
                by_name[(row["node"], name)].add((row["default_state"], dumps(row["default"])))
        ambiguous_nodes.update(key for key, defaults in by_name.items() if len(defaults) > 1)
    return [row for row in result if (row["node"], row["name"]) not in ambiguous_nodes]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--cache", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=OUT)
    args = parser.parse_args()
    cache = args.cache.expanduser().resolve()
    if cache == ROOT or ROOT in cache.parents:
        raise UsageError("the clone cache must be outside this repository")
    cache.mkdir(parents=True, exist_ok=True)
    releases, commits = {}, {}
    for branch in BRANCHES:
        checkout = cache / branch
        if not checkout.exists():
            subprocess.run(
                [
                    "git",
                    "clone",
                    "--depth",
                    "1",
                    "--branch",
                    branch,
                    REPOSITORY,
                    str(checkout),
                ],
                check=True,
            )
        commits[branch] = subprocess.check_output(
            ["git", "-C", str(checkout), "rev-parse", "HEAD"],
            text=True,
        ).strip()
        releases[branch] = collect_release(checkout, branch)
        print(f"{branch}: {len(releases[branch])} records", file=sys.stderr)
    data = {
        "repository": REPOSITORY,
        "commits": commits,
        "checked": list(BRANCHES),
        "helpers": sorted(cpp.DECLARE_CALLS),
        "parameters": differences(releases),
    }
    args.out.write_text(dumps(data), encoding="utf-8", newline="\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
