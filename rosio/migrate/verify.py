"""Build copied sources in the target release and match diagnostics to the plan."""

from __future__ import annotations

import posixpath
import re
import subprocess
import tarfile
import tempfile
from collections.abc import Mapping, Sequence
from copy import deepcopy
from pathlib import Path

from rosio.errors import UsageError
from rosio.migrate import plan, report, source_workspace
from rosio.migrate.scan import Change, SourcePackage
from rosio.workspace import SRC_SKIP_DIRS

SOURCE = "/rosio_ws/src/"
_ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
_EVENT = re.compile(r"^(Finished|Failed|Aborted)\s+<<<\s+(\S+)")
_COMPILER = re.compile(r"^(.+?):(\d+)(?::\d+)?:\s*(?:fatal )?error:\s*(.*)")
_CMAKE = re.compile(r"^CMake Error at (.+?):(\d+)(?:\s+\([^)]*\))?:\s*(.*)")
_ERROR = re.compile(
    r"error|failed|fatal|refused|cannot|could not|undefined reference|exception|traceback|"
    r"not found|no such file",
    re.I,
)
_CONTEXT = re.compile(
    r"^(?:-- |(?:g?make|ninja).*\b(?:Error|stopped)|\s*(?:\d*\s*\||In file included from|from\s))"
)

BUILD = r"""
set -e
mkdir -p /rosio_ws
cd /rosio_ws
tar -xf -
. /opt/ros/$ROS_DISTRO/setup.sh
setup() {
  apt-get update -qq &&
  (command -v colcon && command -v rosdep ||
    apt-get install -y -qq python3-colcon-common-extensions python3-rosdep) &&
  (test -f /etc/ros/rosdep/sources.list.d/20-default.list || rosdep init) &&
  rosdep update -q &&
  rosdep install --from-paths src -iry
}
if ! setup > /rosio_setup.log 2>&1; then
  echo '--- stderr: @setup'
  cat /rosio_setup.log
  echo '---'
  echo 'Failed <<< @setup'
fi
colcon build --continue-on-error --event-handlers console_cohesion+ \
  --cmake-args -DBUILD_TESTING=OFF
"""


def _clean(text: str) -> str:
    text = _ANSI.sub("", text).replace(SOURCE, "").replace("/rosio_ws/", "")
    text = re.sub(r"(?<![\w./\\])(?:[A-Za-z]:[\\/]|/)[^\s\"'<>():,;]+", "<external>", text)
    text = re.sub(
        r"\b\d{4}-\d{2}-\d{2}[T ]\d{2}:\d{2}:\d{2}(?:\.\d+)?(?:Z|[+-]\d{2}:?\d{2})?",
        "<time>",
        text,
    )
    text = re.sub(r"\b\d{1,2}:\d{2}:\d{2}(?:\.\d+)?\b", "<time>", text)
    text = re.sub(r"(?<=\[)\d+(?:\.\d+)?s\b", "<duration>", text)
    return text.strip()


def _file(raw: str, package: str) -> str | None:
    raw = raw.replace("\\", "/")
    if raw.startswith(SOURCE):
        raw = raw[len(SOURCE) :]
    elif raw.startswith("/") or re.match(r"^[A-Za-z]:/", raw):
        return None
    elif not raw.startswith(package + "/"):
        raw = posixpath.join(package, raw)
    path = posixpath.normpath(raw)
    return None if path == ".." or path.startswith("../") else path


def parse_output(output: str, paths: Mapping[str, str]) -> list[dict]:
    """Read cohesive colcon blocks, retaining failures without recognised diagnostics."""
    packages = {name: {"name": name, "status": "skipped", "errors": []} for name in paths}
    blocks: dict[str, list[str]] = {}
    current = None
    for line in _ANSI.sub("", output).splitlines():
        event = _EVENT.match(line)
        if event:
            state, name = event.groups()
            packages.setdefault(name, {"name": name, "errors": []})["status"] = {
                "Finished": "finished",
                "Failed": "failed",
                "Aborted": "skipped",
            }[state]
            current = None
        elif line.startswith(("--- stderr: ", "--- output: ")):
            current = line.split(": ", 1)[1].strip()
            blocks.setdefault(current, [])
        elif line.strip() == "---":
            current = None
        elif current is not None:
            blocks[current].append(line)
        else:
            summary = re.match(r"\s*\d+ packages? (failed|aborted|skipped):\s*(.*)", line)
            if summary:
                for name in summary[2].split():
                    packages.setdefault(name, {"name": name, "errors": []})["status"] = (
                        "failed" if summary[1] == "failed" else "skipped"
                    )
    for name, package in packages.items():
        if package["status"] != "failed":
            continue
        block = blocks.get(name, [])
        errors = package["errors"]
        i = 0
        while i < len(block):
            line = block[i]
            compiler = _COMPILER.match(line)
            cmake = _CMAKE.match(line)
            match = compiler or cmake
            if match:
                raw, number, message = match.groups()
                if cmake:
                    while i + 1 < len(block) and (
                        not block[i + 1].strip() or block[i + 1].startswith((" ", "\t"))
                    ):
                        i += 1
                        message += "\n" + block[i]
                file = _file(raw, paths.get(name, ""))
                errors.append(
                    {
                        "file": file,
                        "line": int(number) if file else None,
                        "message": _clean(message),
                    }
                )
            elif _ERROR.search(line) and not _CONTEXT.match(line):
                errors.append({"file": None, "line": None, "message": _clean(line)})
            i += 1
        if not errors:
            errors.append(
                {
                    "file": None,
                    "line": None,
                    "message": _clean("\n".join(block)) or "build failed without diagnostics",
                }
            )
        unique = {(e["file"] or "", e["line"] or 0, e["message"]): e for e in errors}
        package["errors"] = [unique[key] for key in sorted(unique)]
    return [packages[name] for name in sorted(packages)]


def match_errors(packages: Sequence[dict], changes: Sequence[Change]) -> list[dict]:
    """Match only the same package and file, over the lines the planned expression spans."""
    packages = deepcopy(list(packages))
    for package in packages:
        for error in package["errors"]:
            matches = [
                {"rule": change.rule, "say": change.say}
                for change in changes
                if change.package == package["name"]
                and change.file == error["file"]
                and error["line"] is not None
                and change.line <= error["line"] <= change.line + change.found.count("\n")
            ]
            error["status"] = "listed" if matches else "not_listed"
            error["matches"] = sorted(matches, key=lambda m: (m["rule"], m["say"]))
        package["all_errors_listed"] = package["status"] == "failed" and all(
            e["status"] == "listed" for e in package["errors"]
        )
    return packages


def build(source: Sequence[SourcePackage], root: Path, target: str, image: str | None = None):
    try:
        ready = subprocess.run(["docker", "info"], capture_output=True, timeout=15)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise UsageError("Docker must be installed and running.") from exc
    if ready.returncode:
        raise UsageError("Docker must be installed and running.")
    if image is None:
        image = f"rosio-facts:{target}"
        present = subprocess.run(["docker", "image", "inspect", image], capture_output=True)
        if present.returncode:
            image = f"ros:{target}-ros-base"
    try:
        with tempfile.TemporaryFile() as archive:
            with tarfile.open(fileobj=archive, mode="w", dereference=True) as tar:
                for package in sorted(source, key=lambda p: p.name):
                    relative = package.path.relative_to(root).as_posix()
                    tar.add(package.path, arcname=f"src/{relative}", filter=_source_member)
            archive.seek(0)
            done = subprocess.run(
                ["docker", "run", "--rm", "-i", "--", image, "bash", "-c", BUILD],
                stdin=archive,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
            )
    except (OSError, tarfile.TarError) as exc:
        raise UsageError("Could not copy sources or run the target container.") from exc
    if done.returncode in (125, 126, 127):
        raise UsageError("Docker could not start the target build container.")
    return done.stdout + "\n" + done.stderr, done.returncode, image


def _source_member(member: tarfile.TarInfo) -> tarfile.TarInfo | None:
    return None if any(part in SRC_SKIP_DIRS for part in Path(member.name).parts) else member


def verify_workspace(
    workspace: str | Path,
    start: str,
    target: str,
    config: str | None = None,
    image: str | None = None,
) -> dict:
    source, root = source_workspace(workspace, config)
    if not source:
        raise UsageError("The workspace has no source packages to verify.")
    changes = plan(source, start, target, root)
    output, code, image = build(source, root, target, image)
    paths = {p.name: p.path.relative_to(root).as_posix() for p in source}
    packages = parse_output(output, paths)
    if code and not any(p["status"] == "failed" for p in packages):
        packages.append(
            {
                "name": "@build",
                "status": "failed",
                "errors": [
                    {
                        "file": None,
                        "line": None,
                        "message": _clean(output) or "build failed without diagnostics",
                    }
                ],
            }
        )
        packages.sort(key=lambda p: p["name"])
    packages = match_errors(packages, changes)
    errors = [e for p in packages for e in p["errors"]]
    built = sum(p["status"] == "finished" for p in packages if p["name"] in paths)
    data = report.data(changes, start, target)
    data.update(
        {
            "image": image,
            "ok": code == 0 and built == len(source) and not errors,
            "packages": packages,
        }
    )
    data["summary"].update(
        {
            "built": built,
            "total": len(source),
            "failed": sum(p["status"] == "failed" for p in packages),
            "skipped": sum(p["status"] == "skipped" for p in packages),
            "listed": sum(e["status"] == "listed" for e in errors),
            "not_listed": sum(e["status"] == "not_listed" for e in errors),
        }
    )
    return data


def lines(data: dict) -> list[str]:
    counts = data["summary"]
    out = [
        f"rosio migrate verify {data['from']} -> {data['to']}: "
        f"{counts['built']} of {counts['total']} packages built, "
        f"{counts['failed']} failed, {counts['skipped']} skipped",
        f"errors: {counts['listed']} listed, {counts['not_listed']} not_listed",
    ]
    for package in data["packages"]:
        if package["status"] == "finished":
            continue
        status = package["status"]
        if package["all_errors_listed"]:
            status += ", all errors listed"
        out.extend(["", f"{package['name']}: {status}"])
        for error in package["errors"]:
            where = f"{error['file']}:{error['line']}" if error["file"] else "unplaced"
            out.append(f"  {where} {error['status']}: {error['message']}")
            out.extend(f"    {m['rule']}: {m['say']}" for m in error["matches"])
    out.extend(["", f"plan changes still open: {counts['changes']}"])
    out.extend(f"  {c['file']}:{c['line']} {c['rule']}: {c['say']}" for c in data["changes"])
    return out
