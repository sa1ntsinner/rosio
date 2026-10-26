"""`rosio migrate`: move a workspace from one ROS 2 release to another.

`plan` lists every change the move asks of the workspace's own packages, each with the file
and line, the release where it lands, the source that says so, and either the exact
rewrite or one sentence for the person making it. `apply` performs the rewrites and nothing
else, and writes the diff it applied. `verify` builds copied sources in the target container
and matches the errors to the remaining plan.

What the move breaks is decided by measurements (`distros.json` and `defaults.json`)
and a short table of hand written rules with their sources (`rules.yaml`).
"""

from __future__ import annotations

import difflib
import subprocess
from collections.abc import Sequence
from pathlib import Path

from rosio.errors import UsageError
from rosio.migrate.distros import DISTROS
from rosio.migrate.scan import (
    Change,
    SourcePackage,
    contained_path,
    exports_targets,
    package_depends,
    scan,
)

__all__ = ["DISTROS", "Change", "SourcePackage", "apply", "packages", "plan"]


def packages(ws) -> list[SourcePackage]:
    """The source packages of a discovered workspace. Installed packages are somebody
    else's code, the move edits only what the workspace builds from source."""
    found: list[SourcePackage] = []
    by_package: dict[str, list[Path]] = {}
    for path in ws.files():
        owner = ws.package_of(path)
        if owner is not None:
            by_package.setdefault(owner, []).append(Path(path))
    for name, package in ws.packages.items():
        if not package.src_path:
            continue
        base = Path(package.src_path)
        files = tuple(sorted(p for p in by_package.get(name, []) if base == p or base in p.parents))
        manifest = base / "package.xml"
        cmakelists = base / "CMakeLists.txt"
        found.append(
            SourcePackage(
                name=name,
                path=base,
                files=files,
                depends=package_depends(_read(manifest)),
                exports_targets=exports_targets(_read(cmakelists)),
            )
        )
    return found


def plan(source: Sequence[SourcePackage], start: str, target: str, root: Path) -> list[Change]:
    return scan(source, start, target, root)


def plan_workspace(
    workspace: str | Path, start: str, target: str, config: str | None = None
) -> tuple[list[Change], Path]:
    """Discover the workspace the way `extract` does and plan the move over its sources.
    Returns the changes and the root their paths are relative to."""
    source, root = source_workspace(workspace, config)
    return plan(source, start, target, root), root


def source_workspace(
    workspace: str | Path, config: str | None = None
) -> tuple[list[SourcePackage], Path]:
    from rosio.config import load_config
    from rosio.model import Root
    from rosio.workspace import Workspace

    directory = Path(workspace).expanduser()
    if not directory.is_dir():
        raise UsageError(f"workspace directory does not exist: {directory}")
    cfg = load_config(config, workspace=str(directory))
    roots = tuple(spec for spec in cfg.roots if spec.kind != Root.BAG.value)
    ws = Workspace.discover(roots, cfg.ignore)
    root = Path(ws.repo_root)
    return packages(ws), root


def apply(changes: Sequence[Change], root: Path, *, force: bool = False) -> str:
    """Write every rewrite in `changes` and return the unified diff of what was written.

    A file with uncommitted edits is refused unless `force`: the diff is the only record of
    the rewrite, and mixing it into somebody's unsaved work makes both hard to review.
    """
    by_file: dict[str, list[Change]] = {}
    for change in changes:
        if change.automatic and change.span is not None:
            by_file.setdefault(change.file, []).append(change)
    for relative in by_file:
        if contained_path(root, root / relative) is None:
            raise UsageError(f"{relative} resolves outside the workspace")
    if not force:
        dirty = _uncommitted(root, sorted(by_file))
        if dirty:
            listed = ", ".join(dirty)
            raise UsageError(f"uncommitted changes in {listed}; commit them or pass --force")
    diff: list[str] = []
    for relative, edits in sorted(by_file.items()):
        path = contained_path(root, root / relative)
        if path is None:
            raise UsageError(f"{relative} resolves outside the workspace")
        with path.open(encoding="utf-8", newline="") as handle:
            before = handle.read()
        after = before
        for change in sorted(edits, key=lambda c: c.span[0], reverse=True):
            start, end = change.span
            if after[start:end] != change.found:
                raise UsageError(f"{relative} changed since it was planned, plan again")
            after = after[:start] + change.replacement + after[end:]
        destination = contained_path(root, root / relative)
        if destination is None or destination != path:
            raise UsageError(f"{relative} resolves outside the workspace or changed, plan again")
        with destination.open("w", encoding="utf-8", newline="") as handle:
            handle.write(after)
        diff.extend(
            difflib.unified_diff(
                before.splitlines(keepends=True),
                after.splitlines(keepends=True),
                fromfile=f"a/{relative}",
                tofile=f"b/{relative}",
            )
        )
    return "".join(diff)


def _uncommitted(root: Path, files: Sequence[str]) -> list[str]:
    if not files:
        return []
    try:
        result = subprocess.run(
            ["git", "-C", str(root), "status", "--porcelain", "--", *files],
            capture_output=True,
            text=True,
            check=False,
        )
    except OSError:
        return []
    if result.returncode != 0:
        return []
    return sorted(line[3:].strip() for line in result.stdout.splitlines() if line.strip())


def _read(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError):
        return ""
