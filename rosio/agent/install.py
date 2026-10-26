"""Write and remove the files an assistant reads, so rosio needs one command to reach it.

Three properties the caller can rely on:

* idempotent. Installing twice changes nothing the second time, and every action says
  which of created, updated, unchanged, kept or removed it was.
* no silent overwrite. A file rosio did not write is left alone and reported as kept,
  unless the caller passes force. Files rosio owns end with a managed marker, sections
  inside a shared file sit between two markers, and an MCP entry is one named key.
* undoable by hand. Every action prints its absolute path and the one edit that reverses
  it, so a person who never runs uninstall can still get the tree back.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path

from rosio.agent import platforms as plat
from rosio.errors import RosioError, UsageError, exit_code_for
from rosio.model import dumps

TEMPLATES = Path(__file__).parent / "templates"

MANAGED_MARK = "rosio:managed"
MANAGED_NOTE = "<!-- rosio:managed, written by `rosio install`, delete this file to remove it -->"
MD_BEGIN = "<!-- rosio:begin managed section, `rosio uninstall` removes it -->"
MD_END = "<!-- rosio:end -->"
TOML_BEGIN = "# rosio:begin managed section, `rosio uninstall` removes it"
TOML_END = "# rosio:end"

CREATED = "created"
UPDATED = "updated"
UNCHANGED = "unchanged"
KEPT = "kept"
REMOVED = "removed"
ABSENT = "absent"
PRUNED = "pruned"


@dataclass
class Action:
    """One file, what happened to it, and the edit that reverses it."""

    path: Path
    mechanism: str
    status: str
    detail: str = ""
    undo: str = ""

    def line(self, *, dry_run: bool = False) -> str:
        head = f"would {self.status}" if dry_run and self.status != UNCHANGED else self.status
        detail = f"   {self.detail}" if self.detail else ""
        return f"  {head:<9} {self.path}{detail}"


@dataclass
class Report:
    """Everything one install or uninstall call did."""

    platform: str
    scope: str
    root: Path
    home: Path
    verb: str = "install"
    dry_run: bool = False
    actions: list[Action] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)

    @property
    def kept(self) -> list[Action]:
        return [action for action in self.actions if action.status == KEPT]

    @property
    def changed(self) -> list[Action]:
        return [a for a in self.actions if a.status in (CREATED, UPDATED, REMOVED, PRUNED)]

    def lines(self) -> list[str]:
        out = [f"rosio {self.verb} {self.platform}, scope {self.scope}"]
        if not self.actions:
            out.append("  nothing to do, this platform has no file at this scope")
        out.extend(action.line(dry_run=self.dry_run) for action in self.actions)
        out.extend(f"  undo: {a.undo}" for a in self.actions if a.undo and a.status != UNCHANGED)
        out.extend(f"  note: {note}" for note in self.notes)
        return out

    def text(self) -> str:
        return "\n".join(self.lines())


def render(template: str) -> str:
    """The template with the shared body substituted in."""
    body = (TEMPLATES / "core.md").read_text(encoding="utf-8").strip()
    text = (TEMPLATES / template).read_text(encoding="utf-8")
    return text.replace("{{core}}", body)


def _owned_text(template: str) -> str:
    return f"{render(template).rstrip()}\n\n{MANAGED_NOTE}\n"


def _markers(fmt: str) -> tuple[str, str]:
    return (TOML_BEGIN, TOML_END) if fmt == "toml" else (MD_BEGIN, MD_END)


def _toml_value(value) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, list):
        return "[" + ", ".join(_toml_value(item) for item in value) + "]"
    if isinstance(value, str):
        return '"' + value.replace("\\", "\\\\").replace('"', '\\"') + '"'
    return str(value)


def _section_body(target: plat.Target, command: str) -> str:
    if target.mechanism == "mcp":
        entry = plat.server_entry(target, command)
        lines = [f"[{target.container}.{plat.SKILL_NAME}]"]
        lines += [f"{key} = {_toml_value(entry[key])}" for key in sorted(entry)]
        lines.append("# a key written under this block joins this table, put yours above it")
        return "\n".join(lines)
    return render(target.template).strip()


def _read(path: Path) -> str:
    if not path.is_file():
        return ""
    try:
        return path.read_text(encoding="utf-8")
    except OSError as exc:
        raise UsageError(f"{path} could not be read: {exc.strerror or exc}") from exc


def _write(path: Path, text: str) -> None:
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
    except OSError as exc:
        raise UsageError(
            f"{path} could not be written: {exc.strerror or exc}. "
            "fix the permissions on it, or pass --dry-run to see what install wants to write"
        ) from exc


def _remove(path: Path) -> None:
    try:
        path.unlink()
    except OSError as exc:
        raise UsageError(f"{path} could not be removed: {exc.strerror or exc}") from exc


def _rmdir(path: Path) -> None:
    try:
        path.rmdir()
    except OSError as exc:
        raise UsageError(f"{path} could not be removed: {exc.strerror or exc}") from exc


def _split_section(text: str, fmt: str) -> tuple[str, str, str] | None:
    begin, end = _markers(fmt)
    start = text.find(begin)
    if start < 0:
        return None
    stop = text.find(end, start)
    if stop < 0:
        raise UsageError(f"a rosio section opens but never closes, fix it by hand: {begin}")
    return text[:start], text[start : stop + len(end)], text[stop + len(end) :]


def _container(doc: dict, container: str, *, create: bool) -> dict | None:
    node = doc
    for key in container.split("."):
        child = node.get(key)
        if child is None:
            if not create:
                return None
            child = {}
            node[key] = child
        if not isinstance(child, dict):
            raise UsageError(f"{container} is not an object in this config, fix it by hand")
        node = child
    return node


def _load_json(path: Path) -> dict:
    text = _read(path)
    if not text.strip():
        return {}
    try:
        doc = json.loads(text)
    except ValueError as exc:
        raise UsageError(f"{path} is not valid json, fix it by hand: {exc}") from exc
    if not isinstance(doc, dict):
        raise UsageError(f"{path} is not a json object, fix it by hand")
    return doc


def _prune(path: Path, stop: Path, actions: list[Action], *, dry_run: bool) -> None:
    """Remove the directories the install created, once they hold nothing else."""
    parent = path.parent
    while parent != stop and stop in parent.parents:
        if not parent.is_dir() or any(parent.iterdir()):
            return
        actions.append(Action(parent, "dir", PRUNED, "empty after uninstall"))
        if not dry_run:
            _rmdir(parent)
        parent = parent.parent


def _paths(platform: plat.Platform, scope: str, root: Path, home: Path) -> list[tuple]:
    base = root if scope == "project" else home
    return [(target, base / target.path_for(scope)) for target in platform.targets_for(scope)]


def install(
    platform: str,
    scope: str = "project",
    *,
    root: Path | str | None = None,
    home: Path | str | None = None,
    dry_run: bool = False,
    force: bool = False,
    command: str = plat.SERVE_COMMAND,
) -> Report:
    """Write every file the platform reads at this scope."""
    row = plat.get(platform)
    plat.check_scope(scope)
    root = Path(root or Path.cwd()).resolve()
    home = Path(home or Path.home()).resolve()
    report = Report(row.key, scope, root, home, verb="install", dry_run=dry_run)

    for target, path in _paths(row, scope, root, home):
        if target.writer == "whole_file":
            _install_whole(target, path, report, force=force)
        elif target.writer == "section":
            _install_section(target, path, report, command)
        elif target.writer == "json_merge":
            _install_json(target, path, report, command, force=force)
        else:
            _install_section(target, path, report, command)

    _add_notes(row, scope, report)
    return report


def _install_whole(target: plat.Target, path: Path, report: Report, *, force: bool) -> None:
    wanted = _owned_text(target.template)
    have = _read(path)
    undo = f"delete {path}"
    if have == wanted:
        report.actions.append(Action(path, target.mechanism, UNCHANGED, "same content", undo))
        return
    if have and MANAGED_MARK not in have and not force:
        report.actions.append(
            Action(path, target.mechanism, KEPT, "this file is not rosio's, pass force to replace")
        )
        return
    status = UPDATED if have else CREATED
    detail = "replaces content rosio did not write" if have and MANAGED_MARK not in have else ""
    if not report.dry_run:
        _write(path, wanted)
    report.actions.append(Action(path, target.mechanism, status, detail, undo))


def _install_section(target: plat.Target, path: Path, report: Report, command: str) -> None:
    begin, end = _markers(target.fmt)
    block = f"{begin}\n{_section_body(target, command)}\n{end}\n"
    have = _read(path)
    found = _split_section(have, target.fmt)
    undo = f"delete the lines from {begin} to {end} in {path}"
    if found is None:
        wanted = f"{have.rstrip()}\n\n{block}" if have.strip() else block
        status = UPDATED if have.strip() else CREATED
        detail = "managed section appended, the rest of the file is untouched" if have else ""
    else:
        before, _, after = found
        wanted = before + block.rstrip("\n") + after
        if wanted == have:
            report.actions.append(
                Action(path, target.mechanism, UNCHANGED, "managed section is current", undo)
            )
            return
        status = UPDATED
        detail = "managed section replaced, the rest of the file is untouched"
    if not report.dry_run:
        _write(path, wanted)
    report.actions.append(Action(path, target.mechanism, status, detail, undo))


def _install_json(
    target: plat.Target, path: Path, report: Report, command: str, *, force: bool
) -> None:
    doc = _load_json(path)
    node = _container(doc, target.container, create=True)
    key = f"{target.container}.{plat.SKILL_NAME}"
    undo = f"delete the {key} entry in {path}"
    entry = plat.server_entry(target, command)
    have = node.get(plat.SKILL_NAME)
    if have == entry:
        report.actions.append(Action(path, target.mechanism, UNCHANGED, f"{key} is current", undo))
        return
    if have is not None and not force:
        report.actions.append(
            Action(path, target.mechanism, KEPT, f"{key} already exists, pass force to replace")
        )
        return
    node[plat.SKILL_NAME] = entry
    status = UPDATED if path.is_file() else CREATED
    detail = f"{key} set to {command} {' '.join(plat.SERVE_ARGS)}"
    if not report.dry_run:
        _write(path, dumps(doc))
    report.actions.append(Action(path, target.mechanism, status, detail, undo))


def uninstall(
    platform: str,
    scope: str = "project",
    *,
    root: Path | str | None = None,
    home: Path | str | None = None,
    dry_run: bool = False,
    force: bool = False,
) -> Report:
    """Remove what install wrote, and nothing else."""
    row = plat.get(platform)
    plat.check_scope(scope)
    root = Path(root or Path.cwd()).resolve()
    home = Path(home or Path.home()).resolve()
    stop = root if scope == "project" else home
    report = Report(row.key, scope, root, home, verb="uninstall", dry_run=dry_run)

    for target, path in _paths(row, scope, root, home):
        if target.writer == "whole_file":
            _uninstall_whole(target, path, report, force=force)
        elif target.writer == "json_merge":
            _uninstall_json(target, path, report)
        else:
            _uninstall_section(target, path, report)
        _prune(path, stop, report.actions, dry_run=dry_run)

    return report


def _uninstall_whole(target: plat.Target, path: Path, report: Report, *, force: bool) -> None:
    have = _read(path)
    if not path.is_file():
        report.actions.append(Action(path, target.mechanism, ABSENT))
        return
    if MANAGED_MARK not in have and not force:
        report.actions.append(
            Action(path, target.mechanism, KEPT, "this file is not rosio's, pass force to delete")
        )
        return
    if not report.dry_run:
        _remove(path)
    report.actions.append(Action(path, target.mechanism, REMOVED))


def _uninstall_section(target: plat.Target, path: Path, report: Report) -> None:
    have = _read(path)
    found = _split_section(have, target.fmt)
    if found is None:
        report.actions.append(Action(path, target.mechanism, ABSENT, "no managed section"))
        return
    before, _, after = found
    rest = (before.rstrip() + "\n" + after.lstrip("\n")).strip()
    if rest:
        if not report.dry_run:
            _write(path, rest + "\n")
        report.actions.append(
            Action(path, target.mechanism, REMOVED, "section only, the rest of the file stays")
        )
        return
    if not report.dry_run:
        _remove(path)
    report.actions.append(Action(path, target.mechanism, REMOVED, "file held nothing else"))


def _uninstall_json(target: plat.Target, path: Path, report: Report) -> None:
    if not path.is_file():
        report.actions.append(Action(path, target.mechanism, ABSENT))
        return
    doc = _load_json(path)
    node = _container(doc, target.container, create=False)
    key = f"{target.container}.{plat.SKILL_NAME}"
    if node is None or plat.SKILL_NAME not in node:
        report.actions.append(Action(path, target.mechanism, ABSENT, f"no {key} entry"))
        return
    del node[plat.SKILL_NAME]
    _drop_empty(doc, target.container)
    if doc:
        if not report.dry_run:
            _write(path, dumps(doc))
        report.actions.append(
            Action(path, target.mechanism, REMOVED, f"{key} only, the rest of the file stays")
        )
        return
    if not report.dry_run:
        _remove(path)
    report.actions.append(Action(path, target.mechanism, REMOVED, "file held nothing else"))


def _drop_empty(doc: dict, container: str) -> None:
    keys = container.split(".")
    node = doc
    chain = [(None, doc)]
    for key in keys:
        node = node.get(key)
        if not isinstance(node, dict):
            return
        chain.append((key, node))
    for index in range(len(chain) - 1, 0, -1):
        key, child = chain[index]
        if child:
            return
        del chain[index - 1][1][key]


def _add_notes(platform: plat.Platform, scope: str, report: Report) -> None:
    if not platform.targets_for(scope):
        report.notes.append(
            f"{platform.key} has no file at {scope} scope, try the other scope: "
            f"{', '.join(s for s in plat.SCOPES if platform.targets_for(s))}"
        )
    for target in platform.targets_for(scope):
        if not target.verified:
            report.notes.append(f"{target.mechanism} path is unverified: {target.note}")
    if platform.note:
        report.notes.append(platform.note)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="rosio install",
        description="Register rosio with a coding assistant by writing the files it reads.",
    )
    parser.add_argument("platform", nargs="?", help="assistant to register with")
    parser.add_argument("--scope", default="project", choices=list(plat.SCOPES))
    parser.add_argument("--root", default=None, help="project root, default the working directory")
    parser.add_argument("--home", default=None, help="home directory, for user scope")
    parser.add_argument("--dry-run", action="store_true", help="print what would change")
    parser.add_argument("--force", action="store_true", help="replace content rosio did not write")
    parser.add_argument("--remove", action="store_true", help="uninstall instead of install")
    parser.add_argument("--list", action="store_true", help="print the platform table and exit")
    return parser


def _table_lines() -> list[str]:
    """The platform table, with what was actually demonstrated per row.

    A target carries a project path and a user path, and those are two claims. Printing one
    `unverified` over both puts a row somebody watched a tool read beside a row taken from a
    manual, so the row prints the phrase the table carries instead of a flag.
    """
    lines = ["platform      mechanism     project                          user"]
    for row in plat.table():
        lines.append(
            f"{row['platform']:<13} {row['mechanism']:<13} "
            f"{row['project'] or '-':<32} {row['user'] or '-'}"
        )
        lines.append(f"{'':<13} proof: {row['proof']}")
    lines.append("")
    lines.append(
        "Demonstrated means somebody started that tool and watched it read the file at that "
        "path. A row not demonstrated at every path it offers stays unverified, and it is "
        "installable anyway: the path then comes from that tool's documentation, and install "
        "prints the same note per file."
    )
    return lines


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.list or not args.platform:
        print("\n".join(_table_lines()))
        return 0 if args.list else 2
    try:
        call = uninstall if args.remove else install
        report = call(
            args.platform,
            args.scope,
            root=args.root,
            home=args.home,
            dry_run=args.dry_run,
            force=args.force,
        )
    except RosioError as exc:
        print(f"rosio: {exc}", file=sys.stderr)
        return exit_code_for(exc)
    print(report.text())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
