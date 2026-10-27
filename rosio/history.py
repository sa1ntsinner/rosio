"""Keeping the atlas current: the git hooks, and the two revision mode of diff.

An atlas is only useful while it still describes the tree it was built from, and it stops
describing it the moment somebody edits a launch file. A hook rebuilds it after a commit,
after a checkout and after a merge, so the file in the working tree is never older than the
sources beside it.

Three properties decide whether a hook survives contact with a real team:

* It embeds the interpreter it was installed with. A hook started by a graphical client or
  by CI gets neither the shell profile nor the PATH the installer had, so `rosio` is not
  a command there and `python` may be a different one.
* It never blocks. Every failure prints one line and exits zero, because a tool that can
  refuse a commit gets uninstalled the first time it is wrong.
* It states its cost. The wall time of every rebuild is recorded next to the hook, outside
  the atlas and outside the working tree, and `hook status` reports it.

Comparing two revisions builds both of them, which is the same cost twice. A built atlas is
therefore kept under a digest of what it was built from, so the second comparison touching a
revision reads it instead. The rules are in `CACHE_RULES`, every answer names the flag that
prints them, and `--cache-rules` does: there is no incremental rebuild here, and a fast answer
a reader cannot check is worse than a slow one.

Nothing here writes into a repository that was not named, and an existing hook file that
rosio did not write is never replaced without `--force`.

Nothing here imports `rosio.cli`. The rebuild a hook runs is `build_atlas`, an operation
over a repository and a stored command, and `rosio hook run` is one caller of it. It was
the other way around for two milestones: this module imported the entry point, changed the
working directory of the process and called `main` again from inside the function the hook
invokes, so a library operation depended on an argument parser, on exit codes and on where
the process happened to stand. The interpreter the hook embeds is still the one that builds,
which is what `_build` decides.
"""

from __future__ import annotations

import json
import os
import stat
import subprocess
import sys
import tarfile
import tempfile
import time
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from rosio.config import Config
    from rosio.workspace import Workspace

from rosio import __version__, ids
from rosio.errors import UsageError
from rosio.model import dumps, dumps_line
from rosio.render import plural

HOOK_EVENTS = ("post-checkout", "post-commit", "post-merge")
"""What the atlas has to survive: a commit, a branch change, and a merge that brought
somebody else's launch files in."""

MARKER = "managed by rosio hook install"
STATE_NAME = "rosio-hook.json"
LOG_NAME = "rosio-hook.log"
LOG_KEEP = 20
"""Rebuild records kept beside the hook. Enough for a median, small enough to ignore."""

DRIVER_NAME = "rosio-atlas"
ATTRIBUTES = ".gitattributes"
ATTRIBUTE_NOTE = "# rosio: the atlas is merged by union, see rosio hook status"

DEFAULT_OUT = "rosio-out"
ATLAS_NAME = "atlas.json"

CACHE_NAME = "rosio-atlas-cache"
CACHE_KEEP = 12
CACHE_BYTES = 256 * 1024 * 1024
"""Built atlases kept beside the hook, newest first, under both a count and a size. The atlas
of a large stack can be tens of megabytes, so a count alone is not a bound anybody can
predict, and a cache that fills a disk is a defect and not a saving."""


class Report:
    """One reply, in the shape every other verb answers in."""

    def __init__(self, verb: str, ok: bool = True) -> None:
        self.verb = verb
        self.ok = ok
        self.data: dict[str, Any] = {}
        self.lines: list[str] = []

    def text(self) -> str:
        return "\n".join(self.lines)

    def to_json(self) -> dict[str, Any]:
        return {"verb": self.verb, "ok": self.ok, "data": self.data}


# ---------------------------------------------------------------------------
# finding the repository, without needing git on PATH


def find_repo(start: str | Path | None = None) -> Path:
    """The work tree holding `start`. A hook runs with the work tree as its directory."""
    here = Path(start).expanduser().resolve() if start else Path.cwd().resolve()
    if not here.exists():
        raise UsageError(f"no such directory: {here}")
    for candidate in [here, *here.parents]:
        if (candidate / ".git").exists():
            return candidate
    raise UsageError(f"{here} is not inside a git repository, so there is nothing to hook")


def git_dir(repo: Path) -> Path:
    """The git directory of a work tree, following the one line a worktree file holds."""
    marker = repo / ".git"
    if marker.is_dir():
        return marker
    text = marker.read_text(encoding="utf-8").strip()
    if not text.startswith("gitdir:"):
        raise UsageError(f"{marker} is neither a directory nor a gitdir file")
    target = Path(text.split(":", 1)[1].strip()).expanduser()
    return target if target.is_absolute() else (repo / target).resolve()


def hooks_dir(repo: Path) -> Path:
    """Where git looks for hooks, asking git first so core.hooksPath is honoured."""
    found = _git(repo, "rev-parse", "--git-path", "hooks", check=False)
    if found:
        path = Path(found)
        return path if path.is_absolute() else (repo / path)
    return git_dir(repo) / "hooks"


def _git(repo: Path, *args: str, check: bool = True) -> str:
    try:
        done = subprocess.run(
            ["git", "-C", str(repo), *args],
            capture_output=True,
            text=True,
            check=False,
        )
    except FileNotFoundError as exc:
        if check:
            raise UsageError(
                "git is not on PATH, so the merge driver cannot be registered"
            ) from exc
        return ""
    if done.returncode != 0:
        if check:
            raise UsageError(f"git {' '.join(args)} failed: {done.stderr.strip()}")
        return ""
    return done.stdout.strip()


# ---------------------------------------------------------------------------
# what the hook remembers


@dataclass
class HookState:
    """What `hook install` was told, read back by every later run.

    It lives in the git directory and not in the work tree, because it holds an absolute
    interpreter path and a machine local answer, and neither belongs in a commit.
    """

    python: str
    argv: list[str]
    events: list[str] = field(default_factory=list)
    version: str = __version__
    merge_driver: bool = False
    attribute_added: bool = False
    """Whether install wrote the `.gitattributes` line. A repository that carries the line
    in its own history hands it to every clone, and uninstall there must leave it, exactly
    as it leaves a hook file rosio did not write."""

    def to_json(self) -> dict[str, Any]:
        return {
            "python": self.python,
            "argv": list(self.argv),
            "events": sorted(self.events),
            "version": self.version,
            "merge_driver": bool(self.merge_driver),
            "attribute_added": bool(self.attribute_added),
        }

    @classmethod
    def from_json(cls, payload: dict[str, Any]) -> HookState:
        return cls(
            python=str(payload.get("python", "")),
            argv=[str(item) for item in payload.get("argv") or []],
            events=[str(item) for item in payload.get("events") or []],
            version=str(payload.get("version", "")),
            merge_driver=bool(payload.get("merge_driver", False)),
            attribute_added=bool(payload.get("attribute_added", False)),
        )


def state_path(repo: Path) -> Path:
    return git_dir(repo) / STATE_NAME


def log_path(repo: Path) -> Path:
    return git_dir(repo) / LOG_NAME


def load_state(repo: Path) -> HookState:
    target = state_path(repo)
    if not target.is_file():
        raise UsageError(
            f"no rosio hook is installed in {repo}. run rosio hook install there first"
        )
    return HookState.from_json(json.loads(target.read_text(encoding="utf-8")))


# ---------------------------------------------------------------------------
# the hook script


def _quote(text: str) -> str:
    return "'" + text.replace("'", "'\\''") + "'"


def hook_script(event: str, python: str, version: str = __version__) -> str:
    """The script one event runs. POSIX sh, no PATH lookup, and never a non zero exit."""
    if event not in HOOK_EVENTS:
        raise UsageError(f"{event} is not a hook rosio installs: {', '.join(HOOK_EVENTS)}")
    guard = ""
    if event == "post-checkout":
        # $3 is 1 for a branch checkout and 0 for a checkout of single files.
        guard = '[ "$3" = "1" ] || exit 0\n'
    return (
        "#!/bin/sh\n"
        f"# rosio {version} {event}, {MARKER}.\n"
        "# The interpreter is embedded because this runs from a graphical client and from\n"
        "# CI, where the shell profile does not apply and rosio is not on PATH.\n"
        "# A failure here prints and exits zero: the atlas is never worth blocking work for.\n"
        f"ROSIO_PYTHON={_quote(python)}\n"
        f"{guard}"
        'if [ ! -x "$ROSIO_PYTHON" ]; then\n'
        '    echo "rosio: $ROSIO_PYTHON is gone, the atlas was not rebuilt" >&2\n'
        "    exit 0\n"
        "fi\n"
        f'"$ROSIO_PYTHON" -m rosio hook run --event {event} ||\n'
        '    echo "rosio: the atlas rebuild failed, your work is unaffected" >&2\n'
        "exit 0\n"
    )


def _is_ours(path: Path) -> bool:
    if not path.is_file():
        return False
    try:
        return MARKER in path.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        return False


# ---------------------------------------------------------------------------
# install, status, uninstall


def default_workspace(repo: Path) -> str:
    return "src" if (repo / "src").is_dir() else "."


def extract_argv(workspace: str, out: str, config: str | None) -> list[str]:
    argv = ["extract", workspace, "--out", out, "--quiet"]
    if config:
        argv += ["--config", config]
    return argv


def _previously_added(repo: Path) -> bool:
    """Whether an earlier install of this repository wrote the attribute line.

    Install is idempotent, so the second one adds nothing and would otherwise record that
    rosio never owned the line.
    """
    target = state_path(repo)
    if not target.is_file():
        return False
    return HookState.from_json(json.loads(target.read_text(encoding="utf-8"))).attribute_added


def _interpreter(python: str | None) -> str:
    """The interpreter to embed, absolute and with its symlinks left alone.

    A virtual environment is a symlink to the interpreter it was made from, and following
    that symlink hands the hook the base interpreter instead: no rosio, no optional
    extra, and none of the environment the workspace is read in. The path is normalised
    and never resolved.
    """
    if not python:
        return sys.executable
    candidate = Path(python).expanduser()
    if not candidate.is_absolute():
        candidate = Path.cwd() / candidate
    return str(Path(os.path.normpath(candidate)))


def install(
    repo: str | Path | None = None,
    *,
    python: str | None = None,
    workspace: str | None = None,
    out: str | None = None,
    config: str | None = None,
    force: bool = False,
    dry_run: bool = False,
    measure: bool = True,
) -> Report:
    """Write the hooks, register the merge driver, and say what a rebuild costs."""
    root = find_repo(repo)
    interpreter = _interpreter(python)
    hooks = hooks_dir(root)
    argv = extract_argv(workspace or default_workspace(root), out or DEFAULT_OUT, config)

    foreign = [
        event for event in HOOK_EVENTS if (hooks / event).exists() and not _is_ours(hooks / event)
    ]
    if foreign and not force:
        raise UsageError(
            f"{', '.join(foreign)} already exists in {hooks} and rosio did not write it. "
            "move it aside, or pass --force to replace it"
        )

    report = Report("hook install")
    written: list[str] = []
    if not dry_run:
        hooks.mkdir(parents=True, exist_ok=True)
    for event in HOOK_EVENTS:
        target = hooks / event
        script = hook_script(event, interpreter)
        if not dry_run:
            target.write_text(script, encoding="utf-8")
            target.chmod(target.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
        written.append(event)

    state = HookState(python=interpreter, argv=argv, events=written, merge_driver=True)
    driver = merge_command(interpreter)
    attributes = attribute_line(argv_out(argv))
    if not dry_run:
        register_merge_driver(root, interpreter)
        added = write_attribute(root, attributes)
        state.attribute_added = added or _previously_added(root)
        state_path(root).write_text(dumps(state.to_json()), encoding="utf-8")
    else:
        added = attributes not in _attribute_text(root).splitlines()

    report.data = {
        "repo": str(root),
        "hooks_dir": str(hooks),
        "hooks": written,
        "python": interpreter,
        "argv": argv,
        "merge_driver": {"name": DRIVER_NAME, "command": driver, "attribute": attributes},
        "attribute_added": added,
        "replaced_foreign": sorted(foreign),
        "dry_run": bool(dry_run),
    }

    verb = "would install" if dry_run else "installed"
    report.lines = [
        f"{verb} {plural(len(written), 'hook')} in {hooks}",
        f"  events {', '.join(written)}",
        f"  rebuild: {' '.join(argv)}",
        f"  interpreter {interpreter} (embedded, no PATH lookup)",
        f"  merge driver {DRIVER_NAME}: {driver}",
        f"  {ATTRIBUTES}: {attributes}" + ("" if added else " (already there)"),
        "  a failed rebuild prints one line and exits zero, it never blocks a commit",
    ]
    if foreign:
        report.lines.insert(
            1, f"  replaced {', '.join(sorted(foreign))}, which rosio did not write"
        )
    if measure and not dry_run:
        record = run_rebuild(root, event="install", reuse=False)
        again = run_rebuild(root, event="install-again") if record["ok"] else None
        report.data["measured"] = record
        report.data["measured_unchanged"] = again
        report.lines.extend(_cost_lines(record, again))
    elif not dry_run:
        report.lines.append("  not measured, run rosio hook run to time one rebuild")
    return report


SLOW_SECONDS = 5.0
"""Above this a rebuild is worth a sentence rather than a number. A hook nobody wants is
uninstalled quietly, and an atlas that is silently stale afterwards is worse than one that
was never hooked."""


def _cost_lines(record: dict[str, Any], unchanged: dict[str, Any] | None = None) -> list[str]:
    seconds = record["volatile"]["seconds"]
    if not record.get("ok"):
        return [f"  a rebuild takes {seconds}s and failed: {record['error']}"]
    lines = [f"  a rebuild takes {seconds}s here, which is what a commit that moved a root pays"]
    if unchanged and unchanged.get("cached"):
        lines.append(
            f"  a commit that moved nothing the extraction reads pays "
            f"{unchanged['volatile']['seconds']}s, read back from the build cache"
        )
    if seconds > SLOW_SECONDS:
        lines.append(
            "  that is long enough to be worth deciding on: keep it, or run rosio hook "
            "uninstall and rebuild the atlas in CI instead"
        )
    return lines


def status(repo: str | Path | None = None) -> Report:
    """What is installed, what it embeds, and what it costs."""
    root = find_repo(repo)
    hooks = hooks_dir(root)
    target = state_path(root)
    state = load_state(root) if target.is_file() else None

    rows: dict[str, str] = {}
    for event in HOOK_EVENTS:
        path = hooks / event
        if not path.exists():
            rows[event] = "absent"
        elif not _is_ours(path):
            rows[event] = "foreign"
        elif state and f"rosio {state.version} " not in path.read_text(encoding="utf-8"):
            rows[event] = "other version"
        else:
            rows[event] = "installed"

    interpreter = state.python if state else ""
    alive = bool(interpreter) and Path(interpreter).exists()
    driver = _git(root, "config", "--local", "--get", f"merge.{DRIVER_NAME}.driver", check=False)
    attribute = attribute_line(argv_out(state.argv)) if state else ""
    attached = bool(attribute) and attribute in _attribute_text(root)
    rebuilds = read_log(root)

    healthy = all(value == "installed" for value in rows.values()) and alive
    report = Report("hook status", ok=(state is None) or healthy)
    report.data = {
        "repo": str(root),
        "installed": state is not None,
        "hooks_dir": str(hooks),
        "hooks": rows,
        "python": {"path": interpreter, "exists": alive},
        "argv": list(state.argv) if state else [],
        "merge_driver": {"configured": bool(driver), "command": driver, "attribute": attached},
        "rebuilds": _timing(rebuilds),
        "cache": cache_state(root),
        "version": state.version if state else "",
    }
    if state is None:
        report.lines = [
            f"no rosio hook in {root}",
            "  rosio hook install writes the post-commit, post-checkout and post-merge hooks",
        ]
        return report
    report.lines = [
        f"rosio hook {state.version} in {root}",
        "  " + ", ".join(f"{event} {rows[event]}" for event in HOOK_EVENTS),
        f"  interpreter {interpreter}" + ("" if alive else "  MISSING, rebuilds do nothing"),
        f"  rebuild: {' '.join(state.argv)}",
        f"  merge driver {DRIVER_NAME}: "
        + ("configured" if driver else "not configured")
        + (f", {ATTRIBUTES} attached" if attached else f", {ATTRIBUTES} not attached"),
    ]
    report.lines.append("  " + _timing_line(report.data["rebuilds"]))
    cache = report.data["cache"]
    report.lines.append(
        f"  build cache: {cache['entries']} entries, {cache['bytes']} bytes in {cache['dir']}, "
        "read by diff --rev and by a rebuild that finds every root where it left it"
    )
    return report


def uninstall(
    repo: str | Path | None = None, *, force: bool = False, dry_run: bool = False
) -> Report:
    """Remove what install wrote, and nothing else."""
    root = find_repo(repo)
    hooks = hooks_dir(root)
    removed: list[str] = []
    kept: list[str] = []
    for event in HOOK_EVENTS:
        path = hooks / event
        if not path.exists():
            continue
        if not _is_ours(path) and not force:
            kept.append(event)
            continue
        if not dry_run:
            path.unlink()
        removed.append(event)
    # The attribute line goes only when install put it there. A repository that carries it
    # in its own history hands it to every clone, and deleting it on one machine takes the
    # driver away from the whole team through the next commit.
    attribute = ""
    state_file = state_path(root)
    if state_file.is_file():
        state = HookState.from_json(json.loads(state_file.read_text(encoding="utf-8")))
        if state.attribute_added or force:
            attribute = attribute_line(argv_out(state.argv))
        if not dry_run:
            state_file.unlink()
    dropped = False
    if attribute and not dry_run:
        dropped = drop_attribute(root, attribute)
    cleared = 0
    if not dry_run:
        _git(root, "config", "--local", "--remove-section", f"merge.{DRIVER_NAME}", check=False)
        log = log_path(root)
        if log.is_file():
            log.unlink()
        cleared = clear_cache(root)

    report = Report("hook uninstall")
    report.data = {
        "repo": str(root),
        "removed": sorted(removed),
        "kept": sorted(kept),
        "attribute_removed": dropped,
        "cache_cleared": cleared,
        "dry_run": bool(dry_run),
    }
    verb = "would remove" if dry_run else "removed"
    report.lines = [f"{verb} {plural(len(removed), 'hook')} from {hooks}"]
    if removed:
        report.lines.append(f"  {', '.join(sorted(removed))}")
    if kept:
        report.lines.append(
            f"  left {', '.join(sorted(kept))} alone, rosio did not write it, pass --force"
        )
    report.lines.append(f"  merge driver {DRIVER_NAME} unregistered")
    return report


# ---------------------------------------------------------------------------
# the merge driver registration


def merge_command(python: str) -> str:
    """The driver git runs. %O is the base, %A is ours and the result, %B is theirs."""
    return f"{_quote(python)} -m rosio hook merge %O %A %B --path %P"


def argv_out(argv: list[str]) -> str:
    """The output directory the stored rebuild command writes to."""
    for index, word in enumerate(argv):
        if word == "--out" and index + 1 < len(argv):
            return argv[index + 1]
    return DEFAULT_OUT


def attribute_line(out: str) -> str:
    return f"{out.rstrip('/')}/{ATLAS_NAME} merge={DRIVER_NAME}"


def register_merge_driver(repo: Path, python: str) -> None:
    _git(repo, "config", "--local", f"merge.{DRIVER_NAME}.name", "rosio atlas union merge")
    _git(repo, "config", "--local", f"merge.{DRIVER_NAME}.driver", merge_command(python))


def _attribute_text(repo: Path) -> str:
    target = repo / ATTRIBUTES
    return target.read_text(encoding="utf-8") if target.is_file() else ""


def write_attribute(repo: Path, line: str) -> bool:
    """Append the one line that binds the atlas to the driver, once."""
    target = repo / ATTRIBUTES
    text = _attribute_text(repo)
    if line in text.splitlines():
        return False
    prefix = "" if not text or text.endswith("\n") else "\n"
    target.write_text(f"{text}{prefix}{ATTRIBUTE_NOTE}\n{line}\n", encoding="utf-8")
    return True


def drop_attribute(repo: Path, line: str) -> bool:
    target = repo / ATTRIBUTES
    if not target.is_file():
        return False
    kept = [
        row
        for row in target.read_text(encoding="utf-8").splitlines()
        if row not in (line, ATTRIBUTE_NOTE)
    ]
    if not [row for row in kept if row.strip()]:
        target.unlink()
        return True
    target.write_text("\n".join(kept) + "\n", encoding="utf-8")
    return True


# ---------------------------------------------------------------------------
# the rebuild itself, and what it costs


def run_rebuild(
    repo: str | Path | None = None,
    *,
    event: str = "manual",
    reuse: bool = True,
) -> dict[str, Any]:
    """Rebuild the atlas and record what it cost. This never raises: the caller is a hook.

    The seconds are under a `volatile` key, the same one the atlas uses, so nothing here can
    be mistaken for compared data.

    `reuse` reads the built atlas back when nothing the extraction reads has moved. The
    documented workflow pays for two rebuilds per change, one for the source commit and one
    for the commit that carries the atlas rebuilt from it, and the second reads the same
    bytes out of the same files as the first. It is a read rather than an incremental build:
    there is no incremental extraction here, and a partial answer that is fast is worse than
    a whole one that is slow.

    The build runs in the interpreter the hook embeds. It is the one that has rosio, the
    optional extras and the environment the workspace is read in, and it is the same one
    every later rebuild will use, so timing anything else states a cost no commit pays.
    """
    root = find_repo(repo)
    state = load_state(root)
    atlas = root / argv_out(state.argv) / ATLAS_NAME
    started = time.perf_counter()
    key, built_from = worktree_key(root, state.argv, state.python)
    if reuse and key:
        text = read_cache(root, key)
        if text is not None and _place(atlas, text):
            return _rebuild_record(root, event, 0, "", atlas, started, cached=True, key=key)
    code, error = _build(root, state)
    record = _rebuild_record(root, event, code, error, atlas, started, cached=False, key=key)
    if record["ok"] and key and atlas.is_file():
        write_cache(
            root,
            key,
            atlas.read_text(encoding="utf-8"),
            event=event,
            built_from=built_from,
            volatile={"seconds": record["volatile"]["seconds"]},
        )
    return record


def _build(root: Path, state: HookState) -> tuple[int, str]:
    """Run the stored rebuild command, in the embedded interpreter when that is another one."""
    if _is_this_interpreter(state.python):
        return _build_here(root, state.argv)
    done = subprocess.run(
        [state.python, "-m", "rosio", *state.argv],
        cwd=str(root),
        capture_output=True,
        text=True,
        check=False,
    )
    if done.returncode == 0:
        return 0, ""
    tail = (done.stderr or done.stdout or "").strip().splitlines()[-3:]
    return int(done.returncode), " ".join(tail)


def _build_here(root: Path, argv: Sequence[str]) -> tuple[int, str]:
    try:
        build_atlas(root, argv)
    except (Exception, SystemExit) as exc:
        # Anything the extraction raised is a line on stderr, never a failed commit. A
        # keyboard interrupt is not caught: that one is the person asking to stop.
        return 1, f"{type(exc).__name__}: {exc}"
    return 0, ""


def rebuild_inputs(repo: Path, argv: Sequence[str]) -> tuple[Config, Workspace]:
    """The configuration and the indexed workspace the stored rebuild command reads.

    Every path is resolved against the repository, so nothing here depends on the working
    directory of the caller. A hook runs with the work tree as its directory and a person
    calling the same operation from anywhere gets the same atlas out of it.
    """
    import dataclasses

    from rosio.config import load_config, with_install_tree
    from rosio.model import Root
    from rosio.workspace import Workspace as Index

    workspace = repo / argv_workspace(argv)
    if not workspace.is_dir():
        raise UsageError(f"workspace directory does not exist: {argv_workspace(argv)}")
    config = argv_config(argv)
    cfg = load_config(str(repo / config) if config else None, workspace=str(workspace))
    tree = argv_install_tree(argv)
    if tree:
        cfg = with_install_tree(cfg, str(repo / tree))
    cfg = dataclasses.replace(
        cfg,
        out_dir=str(repo / argv_out(argv)),
        deterministic="--no-deterministic" not in argv,
    )
    roots = tuple(spec for spec in cfg.roots if spec.kind != Root.BAG.value)
    return cfg, Index.discover(roots, cfg.ignore)


def build_atlas(repo: Path, argv: Sequence[str]) -> Path:
    """Extract, as a library call. Returns the atlas that was written.

    The stored rebuild command is a command line because that is what the hook script runs
    in the interpreter it embeds, and it is read here rather than parsed and run: a library
    module that calls the command line back would put the entry point under the operation it
    exists to orchestrate, and the argument parser, the exit codes and the working directory
    of a process would decide what a rebuild does.
    """
    from rosio import extract as extract_mod
    from rosio.model import Layer, Root

    cfg, ws = rebuild_inputs(repo, argv)
    collectors = None
    if any(spec.kind == Root.BAG.value for spec in cfg.roots):
        collectors = [
            *extract_mod.default_collectors(),
            *extract_mod.collectors_for(Layer.RECORDED.value),
        ]
    extract_mod.run(cfg, ws, collectors=collectors)
    return extract_mod.out_path(cfg)


def _is_this_interpreter(python: str) -> bool:
    """Whether the embedded interpreter is the one running. Compared by normalised path and
    never resolved: two virtual environments are two symlinks to one base interpreter, and
    they hold two different sets of packages."""
    return os.path.normpath(python) == os.path.normpath(sys.executable)


def _place(target: Path, text: str) -> bool:
    """Write a cached atlas where the rebuild would have written it. False if that failed.

    A file that already holds these bytes is left alone, so a hit does not touch the work
    tree and `git status` stays as clean as it was.
    """
    try:
        if target.is_file() and target.read_text(encoding="utf-8") == text:
            return True
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(text, encoding="utf-8", newline="\n")
    except OSError:
        return False
    return True


def _rebuild_record(
    root: Path,
    event: str,
    code: int,
    error: str,
    atlas: Path,
    started: float,
    *,
    cached: bool,
    key: str,
) -> dict[str, Any]:
    record = {
        "event": event,
        "ok": code == 0 and not error,
        "exit": int(code),
        "error": error,
        "atlas": str(atlas),
        "cached": bool(cached),
        "key": key,
        "volatile": {
            "seconds": round(time.perf_counter() - started, 3),
            "bytes": atlas.stat().st_size if atlas.is_file() else 0,
        },
    }
    append_log(root, record)
    return record


_PROBE = (
    "import json;from rosio import __version__;from rosio.history import capability_set;"
    "print(json.dumps({'rosio': __version__, **capability_set()}))"
)


def interpreter_facts(python: str) -> dict[str, str]:
    """What the interpreter that builds can import, and which rosio it would run.

    Asked of that interpreter and not of this one. They are one process on every hook run,
    because a hook starts the interpreter it embeds, and they differ in exactly one place:
    the timed rebuild `hook install` runs, which is started by whoever typed the command. A
    key built from the installer's answer is missed by every rebuild that follows it, and a
    cost measured in the installer's interpreter is not the cost a commit pays.

    Empty when that interpreter could not answer, which turns the cache off rather than
    keying it on a guess.
    """
    if _is_this_interpreter(python):
        return {"rosio": __version__, **capability_set()}
    try:
        done = subprocess.run(
            [python, "-c", _PROBE], capture_output=True, text=True, check=False, timeout=60
        )
    except (OSError, subprocess.SubprocessError):
        return {}
    if done.returncode != 0:
        return {}
    try:
        row = json.loads(done.stdout)
    except ValueError:
        return {}
    return {str(key): str(value) for key, value in sorted(row.items())}


def worktree_key(repo: Path, argv: Sequence[str], python: str = "") -> tuple[str, dict[str, str]]:
    """The digest of everything the atlas of the working tree is built from.

    A working tree is not a commit, so there is no content address to ask git for. The one
    the extraction already pins its own inputs by is used instead: the hash of every root,
    which covers every name under it and the content of every file worth reading. An
    uncommitted edit inside a root moves a root hash and misses, which is the property this
    has to have, and a commit that only added the rebuilt atlas moves nothing, because the
    output directory is not inside a root.

    Empty, which means no caching at all, when the run would not be reproducible anyway: a
    configuration that asked for a non deterministic atlas writes a clock into the file.

    What this does not cover is what the run records do not cover either: the content of the
    ament prefixes outside the workspace, and a file a launch substitution reads from outside
    every configured root. Reading a stale answer needs one of those to move while every root
    hash stays where it was.

    `python` is the interpreter that will run the build, which is the one the hook embeds.
    """
    if "--no-deterministic" in argv:
        return "", {}
    facts = interpreter_facts(python or sys.executable)
    if not facts:
        return "", {}
    try:
        cfg, ws = rebuild_inputs(repo, argv)
        hashes = ws.root_hashes()
    except (Exception, SystemExit):
        # A configuration this rebuild is about to fail on is not a cache question.
        return "", {}
    built_from = {
        "config": cfg.fingerprint(),
        "python": python or sys.executable,
        "roots": " ".join(f"{name}={value}" for name, value in sorted(hashes.items())),
        "rosio": facts.get("rosio", __version__),
    }
    return (
        ids.digest_fields(
            {
                "ament": os.environ.get("AMENT_PREFIX_PATH", ""),
                "argv": list(argv),
                "capabilities": [f"{name}={value}" for name, value in facts.items()],
                "config": built_from["config"],
                "python": built_from["python"],
                "roots": built_from["roots"],
            },
            length=32,
        ),
        built_from,
    )


def argv_workspace(argv: Sequence[str]) -> str:
    """The workspace the stored rebuild command reads. `extract` takes it as the one
    positional, so the first word that is not a flag and not a flag's value is it."""
    skip = False
    for word in list(argv)[1:]:
        if skip:
            skip = False
            continue
        if word.startswith("--"):
            skip = word in ("--out", "--config", "--install-tree", "--layer", "--collector")
            continue
        return word
    return "."


def argv_config(argv: Sequence[str]) -> str | None:
    return _argv_value(argv, "--config")


def argv_install_tree(argv: Sequence[str]) -> str | None:
    return _argv_value(argv, "--install-tree")


def _argv_value(argv: Sequence[str], flag: str) -> str | None:
    words = list(argv)
    for index, word in enumerate(words):
        if word == flag and index + 1 < len(words):
            return words[index + 1]
    return None


def append_log(repo: Path, record: dict[str, Any]) -> None:
    target = log_path(repo)
    rows = read_log(repo)
    rows.append(record)
    target.write_text(
        "".join(dumps_line(row) + "\n" for row in rows[-LOG_KEEP:]),
        encoding="utf-8",
    )


def read_log(repo: Path) -> list[dict[str, Any]]:
    target = log_path(repo)
    if not target.is_file():
        return []
    rows = []
    for line in target.read_text(encoding="utf-8").splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def _timing(rows: list[dict[str, Any]]) -> dict[str, Any]:
    """The recorded rebuilds. A cache read is counted and kept out of the median: it is not
    what a rebuild costs, and averaging the two states a cost nothing ever paid."""
    built = [row for row in rows if not row.get("cached")]
    times = sorted(float(row.get("volatile", {}).get("seconds", 0.0)) for row in built)
    middle = times[len(times) // 2] if times else 0.0
    return {
        "count": len(rows),
        "built": len(built),
        "cached": len(rows) - len(built),
        "failures": sum(1 for row in rows if not row.get("ok")),
        "volatile": {
            "last_seconds": rows[-1]["volatile"]["seconds"] if rows else 0.0,
            "median_seconds": middle,
            "slowest_seconds": times[-1] if times else 0.0,
        },
    }


def _timing_line(timing: dict[str, Any]) -> str:
    if not timing["count"]:
        return "no rebuild recorded yet"
    volatile = timing["volatile"]
    failures = f", {timing['failures']} failed" if timing["failures"] else ""
    read = f", {timing['cached']} read from the cache" if timing["cached"] else ""
    return (
        f"{timing['count']} rebuilds recorded: last {volatile['last_seconds']}s, "
        f"median of the {timing['built']} that built {volatile['median_seconds']}s, "
        f"slowest {volatile['slowest_seconds']}s{read}{failures}"
    )


# ---------------------------------------------------------------------------
# the build cache

CACHE_RULES = (
    f"cached: one built atlas per revision and one per state of the working tree, keyed by a "
    f"digest over the tree that was read, the workspace inside it, the interpreter, what it "
    f"can import, the ament prefixes and the rosio version. The newest {CACHE_KEEP} "
    f"entries are kept, and at most {CACHE_BYTES // (1024 * 1024)} MiB of them",
    "invalidated by: any of those moving. one edited byte is a different tree id and a "
    "miss, and two revisions holding one tree (a rebase that only reordered commits) are "
    "one entry",
    "still paid in full: the first build of a revision, and the first rebuild after an edit. "
    "a rebuild of the working tree is keyed by the hash of every root instead of by a "
    "commit, so an uncommitted edit inside a root is a miss and the commit that carries the "
    "rebuilt atlas is a hit",
    "never wrong about the tree: an entry is written only after a build that exited zero, an "
    "entry is never rewritten in place, and a read checks the stored key and version before "
    "the atlas is used",
    "not in the key: what the ament prefixes hold, and a file a launch substitution reads "
    "from outside every configured root. what this interpreter can import is in it, by "
    "version and by the file that would be imported, so installing or removing an optional "
    "extra is a miss and never a stale read",
)
"""What the cache holds, what invalidates it, what it does not save, what it is certain
about and what it is not. Printed under a revision comparison asked with `--cache-rules`,
because a reader deciding whether to trust a fast answer needs all five, and named on every
other one, because five paragraphs under every diff were read once and skipped after. The
last one is what is left after the capability set went into the key: two runs in one
repository from one interpreter, with
tree-sitter importable in the first and not in the second, used to produce one key and two
different atlases, and now produce two keys."""


OPTIONAL_MODULES = (
    "launch",
    "launch_ros",
    "rclpy",
    "rosbag2_py",
    "tree_sitter",
    "tree_sitter_cpp",
    "yaml",
)
"""Every package an extraction reaches for and can do without. What the interpreter can
import of these decides the atlas, so it decides a cache key too."""


def capability_set(names: Sequence[str] = OPTIONAL_MODULES) -> dict[str, str]:
    """What this interpreter can import of each of them, and which file it would import.

    An interpreter path is not what it can import. Measured: one repository, one interpreter,
    one tree, tree-sitter importable on one run and shadowed on the next, one key both times,
    and the second run served the first run's atlas. The file is in the answer as well as the
    version, because shadowing an installed package with a broken one at the front of the
    path leaves the version where it was and moves the file.

    Nothing here is imported. `find_spec` reads the finders, so the cost is a stat per name
    and the launch machinery is not started to decide whether it is there.
    """
    import importlib.metadata
    import importlib.util

    out: dict[str, str] = {}
    for name in names:
        try:
            spec = importlib.util.find_spec(name)
        except (ImportError, ValueError, AttributeError):
            spec = None
        if spec is None:
            out[name] = "absent"
            continue
        origin = spec.origin or ""
        if not origin and spec.submodule_search_locations:
            origin = sorted(spec.submodule_search_locations)[0]
        out[name] = f"{_dist_version(name)}@{origin}"
    return dict(sorted(out.items()))


def _dist_version(name: str) -> str:
    """The distribution version of a module, empty when nothing on disk states one.

    A ROS package installed by the distribution carries no python metadata, which is not a
    failure: the origin beside it is what separates two of those.
    """
    import importlib.metadata

    for candidate in (name, name.replace("_", "-")):
        try:
            return importlib.metadata.version(candidate)
        except Exception:
            continue
    return ""


def cache_dir(repo: Path) -> Path:
    """Inside the git directory. A built atlas is machine specific and revision specific,
    so it belongs neither in the work tree nor in a commit."""
    return git_dir(repo) / CACHE_NAME


def revision_key(repo: Path, revision: str, workspace: str) -> str:
    """The digest of everything the atlas of one revision is built from.

    The revision name is not in it and the tree it points at is, so a branch, a tag and a
    sha naming one tree share one entry, and a rebase that reordered two commits hits the
    entry the first order wrote.
    """
    tree = _git(repo, "rev-parse", f"{revision}^{{tree}}", check=False)
    if not tree:
        return ""
    return ids.digest_fields(
        {
            "ament": os.environ.get("AMENT_PREFIX_PATH", ""),
            "capabilities": [f"{name}={value}" for name, value in capability_set().items()],
            "python": sys.executable,
            "rosio": __version__,
            "tree": tree,
            "workspace": workspace,
        },
        length=32,
    )


def read_cache(repo: Path, key: str) -> str | None:
    """The atlas stored under one key, or None. A row that does not state this key and this
    version is not read, so a half written entry and an entry from another release miss."""
    if not key:
        return None
    meta, atlas = _cache_files(repo, key)
    if not (meta.is_file() and atlas.is_file()):
        return None
    try:
        row = json.loads(meta.read_text(encoding="utf-8"))
    except ValueError:
        return None
    if row.get("key") != key or row.get("version") != __version__:
        return None
    return atlas.read_text(encoding="utf-8")


def write_cache(repo: Path, key: str, text: str, **facts: Any) -> None:
    """Store one built atlas. The metadata is written last and is what a read looks for,
    so an interrupted write leaves a miss and never a wrong answer."""
    if not key:
        return
    where = cache_dir(repo)
    where.mkdir(parents=True, exist_ok=True)
    meta, atlas = _cache_files(repo, key)
    part = atlas.with_suffix(".part")
    part.write_text(text, encoding="utf-8")
    part.replace(atlas)
    row = {"key": key, "version": __version__, "bytes": len(text.encode("utf-8")), **facts}
    meta.write_text(dumps(row), encoding="utf-8")
    trim_cache(repo)


def _cache_files(repo: Path, key: str) -> tuple[Path, Path]:
    where = cache_dir(repo)
    return where / f"{key}.json", where / f"{key}.atlas.json"


def trim_cache(repo: Path, keep: int = CACHE_KEEP, budget: int = CACHE_BYTES) -> list[str]:
    """Drop the least recently written entries, by count and by size. Returns the keys."""
    where = cache_dir(repo)
    if not where.is_dir():
        return []
    rows = [item for item in where.glob("*.json") if not item.name.endswith(".atlas.json")]
    # The name breaks a tie, so which entry goes is decided and not left to the filesystem.
    rows.sort(key=lambda item: (item.stat().st_mtime, item.name), reverse=True)
    dropped = []
    held = 0
    for index, meta in enumerate(rows):
        atlas = meta.with_suffix(".atlas.json")
        held += atlas.stat().st_size if atlas.is_file() else 0
        if index < keep and held <= budget:
            continue
        key = meta.name[: -len(".json")]
        for path in _cache_files(repo, key):
            if path.is_file():
                path.unlink()
        dropped.append(key)
    return sorted(dropped)


def cache_state(repo: Path) -> dict[str, Any]:
    where = cache_dir(repo)
    rows = []
    if where.is_dir():
        for meta in sorted(where.glob("*.json")):
            if meta.name.endswith(".atlas.json"):
                continue
            try:
                rows.append(json.loads(meta.read_text(encoding="utf-8")))
            except ValueError:
                continue
    return {
        "dir": str(where),
        "entries": len(rows),
        "bytes": sum(int(row.get("bytes") or 0) for row in rows),
    }


def clear_cache(repo: Path) -> int:
    """Remove every entry. Returns how many were removed."""
    where = cache_dir(repo)
    if not where.is_dir():
        return 0
    gone = len([item for item in where.glob("*.json") if not item.name.endswith(".atlas.json")])
    for item in sorted(where.iterdir()):
        if item.is_file():
            item.unlink()
    where.rmdir()
    return gone


# ---------------------------------------------------------------------------
# two revisions of one workspace


def export_revision(repo: Path, revision: str, dest: Path) -> Path:
    """One revision of the work tree, written to a directory the caller owns.

    `git archive` is used rather than a worktree or a checkout, because reading a revision
    must not touch the repository a person is working in: no index, no reflog, no lock.
    """
    dest.mkdir(parents=True, exist_ok=True)
    proc = subprocess.Popen(
        ["git", "-C", str(repo), "archive", "--format=tar", revision],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if proc.stdout is None:
        raise UsageError("git archive produced no output to read")
    try:
        with tarfile.open(fileobj=proc.stdout, mode="r|") as archive:
            try:
                # The tar filter refuses an absolute path and a name that climbs out. The
                # stricter data filter also refuses a relative symlink, which is what a
                # workspace built with symlinks is made of.
                archive.extractall(dest, filter="tar")
            except TypeError:
                archive.extractall(dest)
    except tarfile.TarError as exc:
        proc.kill()
        raise UsageError(f"could not read revision {revision}: {exc}") from exc
    finally:
        proc.stdout.close()
    if proc.wait() != 0:
        message = proc.stderr.read().decode("utf-8", "replace").strip() if proc.stderr else ""
        raise UsageError(f"git archive {revision} failed: {message or 'unknown revision'}")
    if proc.stderr:
        proc.stderr.close()
    return dest


def revision_env(tree: Path) -> dict[str, str]:
    """The environment one revision is built in.

    A launch file asks the ament index where a package's share directory is, and the index
    of the working tree answers for whatever is checked out now, which is a different
    revision. An export that carries its own install tree answers for itself instead.
    """
    env = dict(os.environ)
    prefixes = [
        str(marker.parent.parent)
        for pattern in ("install/*/share/ament_index", "install/share/ament_index")
        for marker in sorted(tree.glob(pattern))
    ]
    if prefixes:
        held = env.get("AMENT_PREFIX_PATH", "")
        env["AMENT_PREFIX_PATH"] = ":".join([*prefixes, held]) if held else ":".join(prefixes)
    package_root = str(Path(__file__).resolve().parent.parent)
    held = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = f"{package_root}:{held}" if held else package_root
    return env


def build_revision(repo: Path, revision: str, dest: Path, workspace: str) -> Path:
    """Export one revision and extract an atlas from it. Returns the atlas path.

    Each revision is built in its own process. Two launch sandboxes in one interpreter
    would share whatever the first one imported, and a revision has to be read as if
    nothing else had been read before it.
    """
    tree = export_revision(repo, revision, dest / "tree")
    source = tree / workspace
    if not source.is_dir():
        raise UsageError(f"{workspace} is not a directory in revision {revision}")
    out = dest / DEFAULT_OUT
    done = subprocess.run(
        [sys.executable, "-m", "rosio", "extract", str(source), "--out", str(out), "--quiet"],
        env=revision_env(tree),
        capture_output=True,
        text=True,
        check=False,
    )
    atlas = out / ATLAS_NAME
    if done.returncode != 0 or not atlas.is_file():
        tail = (done.stderr or done.stdout or "").strip().splitlines()[-3:]
        raise UsageError(
            f"extraction of revision {revision} did not write an atlas: {' '.join(tail)}"
        )
    return atlas


def compare_revisions(
    before: str,
    after: str,
    *,
    repo: str | Path | None = None,
    workspace: str | None = None,
    limit: int = 12,
    keep: str | Path | None = None,
    cache: bool = True,
    cache_rules: bool = False,
):
    """Build two revisions of one workspace and answer what changed between them.

    This is the question after a rebase, and it is the only place in rosio where a verb
    extracts: the two atlases it compares do not exist yet. A revision that was built once
    before is read rather than built again, and the answer states which side that was.
    """
    from rosio import query
    from rosio.model import Atlas

    root = find_repo(repo)
    where = Path(workspace).name if workspace and Path(workspace).is_absolute() else workspace
    where = where or default_workspace(root)
    holder = Path(keep).expanduser() if keep else None
    if holder:
        holder.mkdir(parents=True, exist_ok=True)
    scratch = tempfile.mkdtemp(prefix="rosio-rev-") if holder is None else str(holder)
    try:
        holders = (Path(scratch) / "before", Path(scratch) / "after")
        left, left_build = revision_atlas(root, before, holders[0], where, cache=cache)
        right, right_build = revision_atlas(root, after, holders[1], where, cache=cache)
        answer = query.diff(Atlas.loads(left), Atlas.loads(right), limit=limit)
    finally:
        if holder is None:
            _remove_tree(Path(scratch))
    scope = f"revision {before} against {after}, workspace {where}"
    answer.data["scope"] = scope
    answer.data["revisions"] = {"before": before, "after": after, "workspace": where}
    answer.data["builds"] = [left_build, right_build]
    if answer.lines:
        answer.lines[0] = f"diff, {scope}"
    answer.lines.extend(_build_cost_lines([left_build, right_build], rules=cache_rules))
    return answer


def revision_atlas(
    repo: Path,
    revision: str,
    dest: Path,
    workspace: str,
    *,
    cache: bool = True,
) -> tuple[str, dict[str, Any]]:
    """The atlas of one revision as text, and what getting it cost.

    The directory the revision was exported into is replaced by the workspace token before
    anything else sees the text, so a cached atlas is not tied to the temporary directory
    the build that wrote it happened to use.
    """
    key = revision_key(repo, revision, workspace) if cache else ""
    started = time.perf_counter()
    text = read_cache(repo, key)
    if text is not None:
        seconds = round(time.perf_counter() - started, 3)
        return text, {
            "revision": revision,
            "cached": True,
            "key": key,
            "volatile": {"seconds": seconds},
        }
    atlas = build_revision(repo, revision, dest, workspace)
    text = atlas.read_text(encoding="utf-8").replace(str(dest / "tree"), WORKSPACE_TOKEN)
    seconds = round(time.perf_counter() - started, 3)
    write_cache(
        repo, key, text, revision=revision, workspace=workspace, volatile={"seconds": seconds}
    )
    return text, {
        "revision": revision,
        "cached": False,
        "key": key,
        "volatile": {"seconds": seconds},
    }


def _build_cost_lines(builds: list[dict[str, Any]], *, rules: bool = False) -> list[str]:
    lines = []
    for row in builds:
        how = "read from the build cache" if row["cached"] else "built"
        lines.append(f"  {row['revision']}: {how} in {row['volatile']['seconds']}s")
    if rules:
        lines.extend(f"  {rule}" for rule in CACHE_RULES)
    else:
        lines.append("  --cache-rules says what the build cache keys on and what it cannot know")
    return lines


WORKSPACE_TOKEN = "${workspace}"
"""What the directory a revision was exported into is called in its atlas.

A value that is an absolute path (a map file a launch file built from the package share)
carries the directory the revision was read in. Two revisions are read in two directories,
so leaving that in reports every such value as changed, when the only thing that moved is
where the tool put the tree. The tail of the path, which is what a person can act on,
compares unchanged."""


def _remove_tree(path: Path) -> None:
    import shutil

    shutil.rmtree(path, ignore_errors=True)


__all__ = [
    "ATTRIBUTES",
    "CACHE_RULES",
    "DRIVER_NAME",
    "HOOK_EVENTS",
    "MARKER",
    "OPTIONAL_MODULES",
    "HookState",
    "Report",
    "argv_config",
    "argv_install_tree",
    "argv_out",
    "argv_workspace",
    "attribute_line",
    "build_atlas",
    "build_revision",
    "cache_dir",
    "cache_state",
    "capability_set",
    "clear_cache",
    "compare_revisions",
    "export_revision",
    "find_repo",
    "hook_script",
    "hooks_dir",
    "install",
    "interpreter_facts",
    "load_state",
    "merge_command",
    "read_cache",
    "read_log",
    "rebuild_inputs",
    "revision_atlas",
    "revision_key",
    "run_rebuild",
    "status",
    "trim_cache",
    "uninstall",
    "worktree_key",
    "write_cache",
]
