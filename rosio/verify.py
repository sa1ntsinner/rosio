"""Build and test the packages a change touched, and say in a few lines what failed.

It runs the build tool and reads no atlas, so a session that claims an edit works can be held
to what colcon printed.
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
from collections.abc import Sequence
from pathlib import Path
from xml.etree import ElementTree

from rosio.errors import UsageError

TIMEOUT = 1800
MAX_LINES = 60
SKIP = {"build", "install", "log", ".git"}
_SUMMARY = re.compile(r"Summary: (\d+) tests?, (\d+) errors?, (\d+) failures?, (\d+) skipped")


def workspace_root(start: str | Path = ".") -> Path:
    """The nearest directory holding `src` that is not itself a package, else `start`."""
    start = Path(start).expanduser().resolve()
    for directory in (start, *start.parents):
        if (directory / "src").is_dir() and not (directory / "package.xml").is_file():
            return directory
    return start


def _git(root: Path, *args: str) -> str:
    try:
        done = subprocess.run(
            ["git", "-C", str(root), *args], capture_output=True, text=True, timeout=60
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise UsageError("git could not run here; name the packages with --packages") from exc
    if done.returncode:
        said = (done.stderr or "").strip().splitlines()
        raise UsageError(f"git {args[0]} failed: {said[-1] if said else done.returncode}")
    return done.stdout


def changed_files(root: Path, since: str = "") -> list[Path]:
    """Files git reports changed or untracked, plus those changed since `since`, as absolute
    paths."""
    top = Path(_git(root, "rev-parse", "--show-toplevel").strip())
    names: set[str] = set()
    tokens = _git(root, "status", "--porcelain", "-z", "--untracked-files=all").split("\0")
    i = 0
    while i < len(tokens):
        entry = tokens[i]
        i += 1
        if len(entry) > 3:
            names.add(entry[3:])
            if "R" in entry[:2] or "C" in entry[:2]:
                i += 1  # the source path of a rename or copy follows
    if since:
        names.update(_git(root, "diff", "--name-only", "-z", since).split("\0"))
    return sorted(top / name for name in names if name)


def package_of(path: Path, root: Path) -> tuple[str, str] | None:
    """The (name, root relative directory) of the nearest package holding `path` inside
    `root`."""
    root = root.resolve()
    for directory in Path(path).resolve().parents:
        if directory == root or root not in directory.parents:
            return None
        if SKIP.intersection(directory.relative_to(root).parts):
            return None
        manifest = directory / "package.xml"
        if manifest.is_file():
            try:
                name = (ElementTree.parse(manifest).getroot().findtext("name") or "").strip()
            except (ElementTree.ParseError, OSError):
                return None
            return (name, directory.relative_to(root).as_posix()) if name else None
    return None


def touched_packages(root: Path, files: Sequence[Path]) -> dict[str, str]:
    """Package name to its root relative directory, for every file inside a package."""
    found = (package_of(path, root) for path in files)
    return dict(sorted(item for item in found if item))


def _ready() -> str:
    colcon = shutil.which("colcon")
    if not os.environ.get("AMENT_PREFIX_PATH") and not os.environ.get("ROS_DISTRO"):
        raise UsageError("no ROS environment: source /opt/ros/<distro>/setup.bash first")
    if not colcon:
        raise UsageError(
            "colcon is not on PATH: install python3-colcon-common-extensions and source the "
            "ROS setup file"
        )
    return colcon


def _run(argv: list[str], root: Path, timeout: int) -> tuple[int, str]:
    try:
        done = subprocess.run(
            argv,
            cwd=root,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout,
        )
    except subprocess.TimeoutExpired as exc:
        raise UsageError(
            f"{' '.join(argv[1:3])} did not finish within {timeout} seconds, raise --timeout"
        ) from exc
    output = done.stdout + "\n" + done.stderr
    return done.returncode, output.replace(str(root) + os.sep, "").replace(str(root), ".")


def parse_build(output: str, paths: dict[str, str]) -> dict[str, dict]:
    """Per package `built`, `build failed` or `not built`, with the decisive error lines."""
    from rosio.migrate.verify import parse_output

    states = {"finished": "built", "failed": "build failed", "skipped": "not built"}
    out = {}
    for package in parse_output(output, paths):
        if package["name"] not in paths:
            continue
        errors = [
            f"{e['file']}:{e['line']}: {e['message']}" if e["file"] else e["message"]
            for e in package["errors"]
            if not e["message"].startswith("Traceback (most recent call last)")
        ]
        out[package["name"]] = {"build": states[package["status"]], "errors": errors}
    return out


def parse_tests(output: str) -> dict:
    """The counts and the failed test names, each with its first message line, from
    `colcon test-result --verbose`."""
    failures: list[dict] = []
    for line in output.splitlines():
        if line.startswith("- "):
            failures.append({"name": line[2:].strip(), "line": ""})
        elif failures and line.startswith(" ") and not failures[-1]["line"]:
            text = line.strip()
            if text and not text.startswith(("<<<", ">>>")):
                failures[-1]["line"] = text
    match = _SUMMARY.search(output)
    total, errors, failed, skipped = map(int, match.groups()) if match else (0, 0, 0, 0)
    return {
        "total": total,
        "failed": errors + failed,
        "skipped": skipped,
        "failures": failures,
    }


def verify(
    workspace: str | Path = ".",
    packages: Sequence[str] = (),
    since: str = "",
    test: bool = True,
    timeout: int = TIMEOUT,
) -> dict:
    """Build, and unless `test` is false test, the touched packages of the workspace."""
    root = workspace_root(workspace)
    if packages:
        paths = dict.fromkeys(packages, "")
    else:
        paths = touched_packages(root, changed_files(root, since))
    data: dict = {"ok": True, "packages": []}
    if not paths:
        return data
    colcon = _ready()
    select = ["--packages-select", *paths]
    handlers = ["--event-handlers", "console_cohesion+"]
    code, output = _run([colcon, "build", *select, *handlers], root, timeout)
    results = parse_build(output, paths)
    built = [name for name in paths if results.get(name, {}).get("build") == "built"]
    if code and not any(r["build"] == "build failed" for r in results.values()):
        said = [line for line in output.splitlines() if line.strip()][-3:]
        data["errors"] = said
    if test and built:
        _, ran = _run([colcon, "test", "--packages-select", *built, *handlers], root, timeout)
        steps = parse_build(ran, {name: paths[name] for name in built})
        for name in built:
            base = ["--test-result-base", f"build/{name}", "--verbose"]
            _, said = _run([colcon, "test-result", *base], root, timeout)
            tests = results[name]["tests"] = parse_tests(said)
            # a test step that died before writing results, but not one that found no tests
            died = steps.get(name, {}).get("build") == "build failed"
            if died and not tests["failed"] and "NO TESTS RAN" not in ran:
                tests["errors"] = steps[name]["errors"]
                tests["failed"] = 1
    for name in paths:
        row = {"name": name, "build": "not built", "errors": [], **results.get(name, {})}
        data["packages"].append(row)
        failed = row["build"] != "built" or row.get("tests", {}).get("failed")
        data["ok"] = data["ok"] and not failed
    data["ok"] = data["ok"] and not data.get("errors")
    return data


def lines(data: dict, cap: int = MAX_LINES) -> list[str]:
    if not data["packages"]:
        return ["rosio verify: no package changed, nothing to build"]
    out = []
    for row in data["packages"]:
        state = row["build"]
        tests = row.get("tests")
        if tests is not None:
            if tests.get("errors"):
                state += ", test run failed"
            elif tests["failed"]:
                state += f", {tests['failed']} tests failed"
            else:
                state += ", tests passed" if tests["total"] else ", no tests"
        out.append(f"{row['name']}: {state}")
        out.extend(f"  {error}" for error in row["errors"] + (tests or {}).get("errors", []))
        for failure in (tests or {}).get("failures", []):
            out.append(f"  {failure['name']}" + (f": {failure['line']}" if failure["line"] else ""))
    out.extend(f"  {line}" for line in data.get("errors", []))
    if len(out) > cap:
        cut = len(out) - cap + 1
        out = [*out[: cap - 1], f"... {cut} lines cut, --json has them all"]
    return out


__all__ = [
    "changed_files",
    "lines",
    "package_of",
    "parse_build",
    "parse_tests",
    "touched_packages",
    "verify",
    "workspace_root",
]
