"""What the Claude Code hooks state: a briefing when a session starts, a slice of the atlas for
the names a prompt spells, and the findings an edit added when a session that edited stops.

Every function here returns text or nothing, and the hooks print it as it is. A hook that
cannot answer, because there is no atlas or rosio broke, says nothing and exits zero.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import subprocess
import sys
import tempfile
import time
from collections.abc import Iterable
from pathlib import Path

from rosio import __version__, ids
from rosio.model import Atlas, Finding, dumps_line

DEFAULT_OUT_DIR = "rosio-out"
"""`config.DEFAULT_OUT_DIR`, spelled here because importing the configuration costs a prompt
hook more than everything else it does."""
ATLAS = f"{DEFAULT_OUT_DIR}/atlas.json"
INDEX_FORMAT = 1
"""Raised whenever what the name index holds changes, so a cached index is built again."""
BRIEF_BYTES = 800
SLICE_BYTES = 1200
SLICES = 3
STOP_SECONDS = 120
STALE_SECONDS = 1.0
SKIP = {".git", DEFAULT_OUT_DIR, "build", "install", "log", "node_modules", "__pycache__"}
SUFFIXES = {
    ".py", ".cpp", ".cc", ".cxx", ".c", ".h", ".hh", ".hpp", ".hxx", ".ipp",
    ".xml", ".yaml", ".yml", ".urdf", ".xacro", ".sdf", ".launch", ".msg", ".srv", ".action",
}  # fmt: skip
NAMES = {"CMakeLists.txt", "package.xml"}
CHANNELS = ("topic", "service", "action")
SIDES = {"publishes": "published by", "subscribes": "subscribed by", "serves": "served by"}
SIDES["calls"] = "called by"


class _Query:
    """`rosio.query`, imported on first use: a prompt the cached index answers never needs it."""

    def __getattr__(self, name: str):
        from rosio import query

        return getattr(query, name)


slices = _Query()


def watched(path: str) -> bool:
    """Whether an edit of this file can move what rosio reads."""
    name = Path(path).name
    return name in NAMES or Path(path).suffix in SUFFIXES


def root_of(data: dict) -> Path:
    return Path(os.environ.get("CLAUDE_PROJECT_DIR") or data.get("cwd") or ".")


def load(path: Path) -> Atlas:
    # the hooks read the file extract wrote, so the schema check of the MCP server is skipped
    return Atlas.from_json(json.loads(path.read_bytes()))


def finding_line(item: Finding) -> str:
    where = slices.site(item.where[0]) if item.where else "-"
    return f"{where}  {item.rule}  {item.title}"


def _fit(lines: list[str], cap: int) -> str:
    out: list[str] = []
    size = 0
    for line in lines:
        size += len(line.encode("utf-8")) + 1
        if size > cap:
            break
        out.append(line)
    return "\n".join(out)


# ---------------------------------------------------------------------------
# SessionStart


def newer_source(root: Path, since: float) -> str:
    """One workspace file changed after `since`, or nothing. Gives up after STALE_SECONDS."""
    start = time.monotonic()
    for top, dirs, files in os.walk(root):
        dirs[:] = [name for name in dirs if name not in SKIP and not name.startswith(".")]
        if time.monotonic() - start > STALE_SECONDS:
            return ""
        for name in files:
            path = Path(top, name)
            try:
                if watched(name) and path.stat().st_mtime > since:
                    return path.relative_to(root).as_posix()
            except OSError:
                continue
    return ""


def session_mark(data: dict) -> Path | None:
    """Where the SessionStart hook notes whether the atlas was older than the sources."""
    session = str(data.get("session_id") or "")
    if not session:
        return None
    key = hashlib.sha256(session.encode("utf-8")).hexdigest()[:16]
    return Path(tempfile.gettempdir()) / "rosio-hooks" / f"session-{key}.json"


def stale_at_start(data: dict) -> str:
    """The source the SessionStart hook found newer than the atlas, or nothing."""
    mark = session_mark(data)
    try:
        return str(json.loads(mark.read_text(encoding="utf-8")).get("stale") or "")
    except (AttributeError, OSError, ValueError):
        return ""


def briefing(root: Path, mark: Path | None = None) -> str:
    target = root / ATLAS
    if not target.is_file():
        return f"rosio: no atlas here. rosio extract . --out {DEFAULT_OUT_DIR} builds one."
    atlas = load(target)
    names(target, atlas)
    profiles = [slices.profile_name(item.id) for item in atlas.by_kind("profile")]
    named = ", ".join(profiles[:6]) + (", ..." if len(profiles) > 6 else "")
    findings = sorted(atlas.findings, key=Finding.sort_key)
    counts: dict[str, int] = {}
    for item in findings:
        counts[item.severity] = counts.get(item.severity, 0) + 1
    shown = ", ".join(
        f"{counts[name]} {name}" for name in ("error", "warn", "info") if name in counts
    )
    lines = [
        f"rosio atlas {ATLAS}: {len(profiles)} profile{'s' if len(profiles) != 1 else ''}"
        + (f": {named}" if named else ""),
        f"findings: {shown or 'none'}",
    ]
    changed = newer_source(root, target.stat().st_mtime)
    # the first start of a session decides, a resume finds the session's own edits
    if mark is not None and not mark.exists():
        mark.parent.mkdir(parents=True, exist_ok=True)
        mark.write_text(dumps_line({"stale": changed}), encoding="utf-8")
    if changed:
        lines.insert(
            1,
            f"{changed} changed after this atlas was extracted; "
            f"rosio extract . --out {DEFAULT_OUT_DIR} rebuilds it.",
        )
    top = [item for item in findings if item.severity in ("error", "warn")][:5]
    lines.extend(f"  {finding_line(item)}" for item in top)
    lines.append("rosio explain <file>:<line> | rosio param <node> <name> | rosio drift")
    tail = lines.pop()
    return _fit(lines, BRIEF_BYTES - len(tail) - 1) + "\n" + tail


# ---------------------------------------------------------------------------
# UserPromptSubmit


def index_path(target: Path) -> Path:
    key = hashlib.sha256(str(target.resolve()).encode("utf-8")).hexdigest()[:16]
    return Path(tempfile.gettempdir()) / "rosio-hooks" / f"{key}.json"


def names(target: Path, atlas: Atlas | None = None) -> dict:
    """The name index of one atlas: name to slice ids, and one short text per slice. Cached
    outside the workspace by the atlas's size and mtime, because a large atlas takes seconds
    to load and a prompt hook runs on every prompt."""
    stat = target.stat()
    stamp = [stat.st_size, stat.st_mtime_ns, __version__, INDEX_FORMAT]
    cache = index_path(target)
    try:
        held = json.loads(cache.read_text(encoding="utf-8"))
        if held.get("stamp") == stamp:
            return held
    except (OSError, ValueError):
        pass
    index = build_index(atlas or load(target))
    index["stamp"] = stamp
    try:
        cache.parent.mkdir(parents=True, exist_ok=True)
        cache.write_text(dumps_line(index), encoding="utf-8")
    except OSError:
        pass
    return index


def _keyed(name: str) -> bool:
    # ponytail: a bare word (map, odom) names too much prose, so it needs a mark to match
    return bool(re.search(r"[/_.\d]", name))


def build_index(atlas: Atlas) -> dict:
    about: dict[str, list[Finding]] = {}
    for item in sorted(atlas.findings, key=Finding.sort_key):
        subjects = [item.subject_key]
        if not isinstance(item.subject, str):
            subjects = [item.subject.src, item.subject.dst]
        subjects += [row.file for row in item.where]
        for subject in dict.fromkeys(subjects):
            about.setdefault(subject, []).append(item)
    keys: dict[str, list[str]] = {}
    params: dict[str, dict[str, str]] = {}
    texts: dict[str, str] = {}

    def add(name: str, slice_id: str) -> None:
        if _keyed(name):
            keys.setdefault(name, [])
            if slice_id not in keys[name]:
                keys[name].append(slice_id)

    for node in atlas.entities.values():
        if ids.parse(node.id).synthetic:
            continue
        if node.kind in ("node", *CHANNELS):
            add(node.label, node.id)
            add(node.label.lstrip("/"), node.id)
            texts[node.id] = _endpoint_text(atlas, node.id, about)
        elif node.kind == "frame":
            add(node.label, node.id)
            texts[node.id] = _frame_text(atlas, node.id, about)
        elif node.kind == "file":
            add(Path(ids.parse_file(node.id)[1]).name, node.id)
            texts[node.id] = _file_text(node.id, about)
        elif node.kind == "param":
            owner, name = ids.parse_param(node.id)
            params.setdefault(owner, {})[name] = node.id
            texts[node.id] = _param_text(atlas, node.id, about)
    rules = {item.rule for item in atlas.findings}
    for rule in sorted(rules):
        hits = [item for item in atlas.findings if item.rule == rule]
        rows = [f"rule {rule}: {len(hits)} finding{'s' if len(hits) != 1 else ''}"]
        rows += [f"  {finding_line(item)}" for item in sorted(hits, key=Finding.sort_key)[:3]]
        texts[f"rule:{rule}"] = "\n".join(rows)
        keys[rule] = [f"rule:{rule}"]
    return {"keys": keys, "params": params, "texts": texts}


def _findings(entity_id: str, about: dict[str, list[Finding]]) -> list[str]:
    return [f"  finding {finding_line(item)}" for item in about.get(entity_id, [])[:2]]


def _evidence(rows: Iterable) -> str:
    for fact in rows:
        if fact.evidence:
            return slices.site(fact.evidence[0])
    return ""


def _endpoint_text(atlas: Atlas, entity_id: str, about: dict) -> str:
    from rosio.serve.tools import _into, _out, _qos

    node = atlas.entities[entity_id]
    lines = [f"{node.kind} {node.label}"]
    if node.kind == "node":
        for layer, word in (("declared", "launched at"), ("coded", "code at")):
            where = _evidence(fact for fact in node.facts if fact.layer == layer)
            if where:
                lines[0] += f"  {word} {where}"
        rows = []
        for edge in _out(atlas, entity_id):
            if edge.kind in SIDES:
                qos = _qos(edge, "")
                rows.append(
                    f"  {edge.kind} {slices.label(edge.dst)}" + (f" [{qos}]" if qos else "")
                )
    else:
        rows = []
        for edge in _into(atlas, entity_id):
            if edge.kind in SIDES:
                qos = _qos(edge, "")
                where = _evidence(edge.facts)
                text = f"  {SIDES[edge.kind]} {slices.label(edge.src)}"
                rows.append(text + (f" [{qos}]" if qos else "") + (f" at {where}" if where else ""))
    # one site launched in several profiles is one edge per launch, and says one thing
    lines += list(dict.fromkeys(rows))[:6]
    return "\n".join(lines + _findings(entity_id, about))


def _frame_text(atlas: Atlas, entity_id: str, about: dict) -> str:
    from rosio.serve.tools import _out

    lines = [f"frame {atlas.entities[entity_id].label}"]
    parents = [edge for edge in _out(atlas, entity_id) if edge.kind == "tf_parent"]
    rows = []
    for edge in parents:
        where = _evidence(edge.facts)
        rows.append(f"  parent {slices.label(edge.dst)}" + (f" at {where}" if where else ""))
    lines += list(dict.fromkeys(rows))[:3]
    return "\n".join(lines + _findings(entity_id, about))


def _file_text(entity_id: str, about: dict) -> str:
    return "\n".join([f"file {slices.file_label(entity_id)}", *_findings(entity_id, about)])


def _param_text(atlas: Atlas, entity_id: str, about: dict) -> str:
    node = atlas.entities[entity_id]
    lines = [f"parameter {slices.label(entity_id)}  state {node.state}"]
    said: dict[str, list[str]] = {}
    for profile in sorted({item.profile for item in node.candidates}):
        chain = [item for item in node.candidates if item.profile == profile]
        shown = next((item for item in chain if item.wins), chain[0])
        text = f"{shown.value!r} at {slices.site(shown.source)}"
        said.setdefault(text if shown.wins else f"{text}, wins nothing", []).append(profile)
    for text, profiles in said.items():
        names = ", ".join(slices.profile_name(item) for item in profiles[:2])
        more = f" and {len(profiles) - 2} more" if len(profiles) > 2 else ""
        lines.append(f"  {names}{more}: {text}")
    return "\n".join(lines[:4] + _findings(entity_id, about))


def prompt_slice(root: Path, prompt: str) -> str:
    target = root / ATLAS
    if not target.is_file() or not prompt.strip():
        return ""
    index = names(target)
    words = [word.rstrip(".,;:!?)'\"") for word in re.findall(r"[\w./~-]+", prompt)]
    hits: list[str] = []
    for word in words:
        for slice_id in index["keys"].get(word, []):
            if slice_id not in hits:
                hits.append(slice_id)
    for owner in [hit for hit in hits if hit.startswith("node:")]:
        for word in words:
            param = index["params"].get(owner, {}).get(word)
            if param and param not in hits:
                hits.insert(hits.index(owner) + 1, param)
    texts = [index["texts"][hit] for hit in hits[:SLICES] if hit in index["texts"]]
    if not texts:
        return ""
    return _fit(["rosio atlas, names in this prompt:", *"\n".join(texts).splitlines()], SLICE_BYTES)


# ---------------------------------------------------------------------------
# Stop


def stop_check(root: Path, changed: list[str], said: str, stale: str = "") -> str:
    """Extract the workspace again into a temporary directory and state the findings the
    session's edits added. The atlas in rosio-out stays as it was. When it was already older
    than `stale` as the session began, earlier edits may have added them, and nothing says
    which."""
    target = root / ATLAS
    edits = [path for path in changed if watched(path)]
    if not edits or not target.is_file():
        return ""
    with tempfile.TemporaryDirectory(prefix="rosio-stop-") as out:
        command = [sys.executable, "-m", "rosio", "extract", str(root), "--out", out, "--quiet"]
        done = subprocess.run(
            command, cwd=root, capture_output=True, text=True, timeout=STOP_SECONDS
        )
        if done.returncode != 0:
            return ""
        before, after = load(target), load(Path(out) / "atlas.json")
    delta = slices.finding_delta(before, after)
    new = {row["id"] for row in delta.data["new"]}
    added = [item for item in sorted(after.findings, key=Finding.sort_key) if item.id in new]
    added = [item for item in added if finding_line(item) not in said]
    if not added:
        return ""
    gone = len(delta.data["gone"])
    count = f"{len(added)} finding{'s' if len(added) != 1 else ''}"
    head = f"rosio extracted this workspace again after the edits to {', '.join(edits[:5])}: "
    if stale:
        head += (
            f"{count} not in {ATLAS}, {gone} gone. That atlas was older than {stale} when this "
            "session began, so earlier edits may have added them."
        )
    else:
        head += f"{count} added, {gone} gone."
    lines = [head]
    for item in added[:8]:
        lines.append(finding_line(item))
        lines.extend(f"  {row}" for row in slices.edit_lines(item))
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# entry points


def _quiet(answer) -> int:
    try:
        data = json.load(sys.stdin)
        text = answer(data)
        if text:
            print(text)
    except Exception as exc:
        print(f"rosio hook skipped: {' '.join(str(exc).splitlines())}", file=sys.stderr)
    return 0


def brief_main() -> int:
    return _quiet(lambda data: briefing(root_of(data), session_mark(data)))


def prompt_main() -> int:
    return _quiet(lambda data: prompt_slice(root_of(data), str(data.get("prompt") or "")))
