"""The query surface: the seven answers that make the atlas usable without reading it.

A full atlas on a real stack is megabytes, so the atlas file is not the agent surface. This
module is. Every verb returns an `Answer` carrying a compact structure and a plain text
rendering, both small enough to paste into a chat, and both carrying the provenance a reader
needs before editing anything.

Two rules hold everywhere here. Nothing in this module reads a file, re extracts or mutates
an entity: it is a pure reader of one loaded atlas. And no synthetic id ever reaches a
reader, because a runtime randomised name is not something a person can act on: the prose
carries the label and the machine structure carries the id.
"""

from __future__ import annotations

import re
import shlex
from collections import deque
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

from rosio import here, ids
from rosio.errors import ContractError, UsageError
from rosio.model import (
    ANY_PROFILE,
    SEVERITY_ORDER,
    Atlas,
    Candidate,
    Edge,
    Finding,
    Node,
    Source,
    strip_volatile,
)
from rosio.render import plural

DEFAULT_LIMIT = 12
"""Rows per section. A reader who needs more asks for one entity at a time."""

MAX_PATH_HOPS = 12


@dataclass
class Answer:
    """One reply: the machine structure and the text rendering of the same content."""

    verb: str
    ok: bool = True
    data: dict[str, Any] = field(default_factory=dict)
    lines: list[str] = field(default_factory=list)

    def text(self) -> str:
        return "\n".join(self.lines)

    def to_json(self) -> dict[str, Any]:
        return {"verb": self.verb, "ok": self.ok, "data": self.data}


# ---------------------------------------------------------------------------
# names, sites and other shared shaping


def label(entity_id: str) -> str:
    """The prose name of an entity. A synthetic id is never returned."""
    parsed = ids.parse(entity_id)
    if parsed.synthetic:
        if parsed.kind == "param":
            owner, name = ids.parse_param(entity_id)
            return f"{label(owner)} {name}"
        if parsed.kind == "pinst":
            owner, instance = ids.parse_pinst(entity_id)
            return f"{label(owner)} {instance}"
        package, executable, namespace, _role = parsed.segments[:4]
        where = "" if namespace == "/" else f" in {namespace}"
        return f"{package}/{executable}{where} (unnamed by launch)"
    if parsed.kind == "param":
        owner, name = ids.parse_param(entity_id)
        return f"{label(owner)} {name}"
    if parsed.kind == "file":
        return file_label(entity_id)
    if parsed.kind == "sym":
        package, relpath, qualname = ids.parse_sym(entity_id)
        return f"{_short_path(package, relpath)} {qualname}"
    if parsed.kind == "profile":
        return parsed.segments[0]
    if parsed.kind == "launchent":
        entity = ids.parse_launchent(entity_id)
        return f"{_short_path(entity.package, entity.relpath)} {parsed.segments[-1]}"
    if parsed.kind == "launchfile":
        return file_label(ids.build_file(*ids.parse_launchfile(entity_id)))
    if parsed.kind == "pparam":
        parts = ids.parse_pparam(entity_id)
        return f"{parts.lookup} {parts.suffix}"
    return parsed.segments[-1]


def file_label(file_id: str) -> str:
    package, relpath = ids.parse_file(file_id)
    return _short_path(package, relpath)


def _short_path(package: str, relpath: str) -> str:
    return relpath if package == ids.REPO_PKG else f"{package}/{relpath}"


NAME_TABLE = ids.build_file(ids.REPO_PKG, "rosio/resolve/known_names.py")


def site(source: Source | Mapping[str, Any] | None, *, locator: bool = False, tag: str = "") -> str:
    """One provenance row as a person reads it: the file, the line, and the key."""
    if source is None:
        return ""
    row = source.to_json() if isinstance(source, Source) else dict(source)
    if row.get("file") == NAME_TABLE:
        # The committed name table is data inside rosio, not a file of the workspace.
        return f"the node name rosio's table states for {row.get('locator') or 'it'}"
    text = file_label(str(row.get("file", "")))
    line = row.get("line_hint")
    if line is not None:
        text = f"{text}:{line}"
    if tag:
        text = f"{text} [{tag}]"
    elif str(row.get("root", "src")) != "src":
        text = f"{text} [{row['root']}]"
    if locator and row.get("locator"):
        text = f"{text} {row['locator']}"
    return text


def tree_tag(atlas: Atlas, source: Source | None) -> str:
    """Which tree a cited value runs from, when the cited file alone would mislead.

    A value read from the install tree while src holds an edit, and a python default read
    from src while the installed copy is what runs, both cite a file a reader can open whose
    line is not what runs now.
    """
    if source is None:
        return ""
    node = atlas.entities.get(source.file)
    root = source.root or "src"
    if node is not None and node.attrs.get("install_copy_differs"):
        return "install, src differs" if root == "install" else "src, the install copy runs"
    return ""


_FILE_TOKEN = re.compile(r"\bfile:[^\s|'\"]+\|[^\s'\"]+")
_ROOT_TOKEN = re.compile(r"@([a-z][a-z0-9_]*)/([^\s'\"]*)")
_TRAILING = ".,;:)]"
OUTSIDE = "outside"
"""The label `Workspace.stated` gives a path under no root."""


def readable(text: str, atlas: Atlas | None = None) -> str:
    """Prose with the tokens the atlas stores paths as turned back into paths a person reads.

    A file id becomes `package/relpath`, and a root token such as `@src/pkg_dir/x.yaml`
    becomes the package relative path when the atlas names the package holding it. A path
    under no root keeps its tail behind a marker, because a reader seeing `@outside/...`
    reads it as a file rosio opened by that name."""
    if "file:" not in text and "@" not in text:
        return text
    generated = (atlas.generated or {}) if atlas is not None else {}
    labels = {str(row.get("label")) for row in generated.get("roots") or ()} or {"src", "install"}
    packages = generated.get("packages") or {}

    def file_token(found: re.Match) -> str:
        token, tail = _split_tail(found.group(0))
        try:
            return file_label(token) + tail
        except ContractError:
            return found.group(0)

    def root_token(found: re.Match) -> str:
        root, (rest, tail) = found.group(1), _split_tail(found.group(2))
        if root == OUTSIDE:
            # after a substitution such as $(find-pkg-share x) the token only joined a name
            glued = found.start() > 0 and found.string[found.start() - 1] == ")"
            return f"{'' if glued else '<outside the workspace>'}/{rest}{tail}"
        if root not in labels:
            return found.group(0)
        best = ""
        for package, dirs in packages.items():
            prefix = str((dirs or {}).get(root) or "")
            if prefix and rest.startswith(prefix + "/") and len(prefix) > len(best):
                best, name = prefix, package
        if best:
            return f"{name}/{rest[len(best) + 1 :]}{tail}"
        return f"{root}/{rest}{tail}"

    return _ROOT_TOKEN.sub(root_token, _FILE_TOKEN.sub(file_token, text))


def _split_tail(token: str) -> tuple[str, str]:
    """A token and the punctuation of the sentence it ends, which is not part of the path."""
    body = token.rstrip(_TRAILING)
    return body, token[len(body) :]


def profile_name(profile_id: str) -> str:
    return profile_id[len("profile:") :] if profile_id.startswith("profile:") else profile_id


def _profiles(items: Iterable[str]) -> str:
    names = [profile_name(item) for item in items]
    return ", ".join(names) if names else "every profile"


def _value(value: Any) -> str:
    if isinstance(value, str):
        return repr(value) if value == "" or value.strip() != value else value
    return repr(value)


def _stated(value: Any) -> str:
    """A fact value as a person reads it: an entity id by its label, a list without python
    quoting. `_value` keeps the quotes a parameter value needs to show an empty string."""
    if isinstance(value, str) and value.partition(":")[0] in ids.ENTITY_KINDS:
        try:
            return label(value)
        except ContractError:
            return value
    if isinstance(value, (list, tuple)):
        return "[" + ", ".join(_stated(item) for item in value) + "]"
    return _value(value)


def _said(value: Any) -> bool:
    """Whether a fact value says anything. An empty string or list is left out of a line."""
    return value not in ("", [], (), {}, None)


def _clip(text: str, width: int = 200) -> str:
    """Text cut to width at a word boundary, so a reason never ends mid word."""
    if len(text) <= width:
        return text
    head = text[: width - 3]
    cut = head.rsplit(" ", 1)[0] if " " in head else head
    return cut.rstrip(" ,;") + "..."


def _cap(rows: Sequence[Any], limit: int) -> tuple[list[Any], int]:
    kept = list(rows[:limit])
    return kept, max(0, len(rows) - len(kept))


def _more(lines: list[str], hidden: int, what: str) -> None:
    if hidden > 0:
        lines.append(f"  and {hidden} more {what}")


# ---------------------------------------------------------------------------
# selectors


ROS_NAME_KINDS = frozenset({"node", "topic", "service", "action", "param", "pinst"})
"""The kinds a leading slash names. A file path also holds `/turtlebot3_node`, and a reader
who typed a ROS name never meant a header."""

MATCH_ORDER = ("label", "suffix", "substring")

_ANCHORS = "/|:. "


@dataclass
class Match:
    """What a selector named, and how: `id`, `label`, `suffix`, `substring`, or empty."""

    ids: list[str]
    how: str = ""


def match(atlas: Atlas, selector: str, *, profile: str = "") -> Match:
    """Entity ids a selector names, from the best rank that names any.

    An exact id wins, then an exact label, then a label or id that ends with the selector at
    a name boundary (`/controller_server` names `/robot1/controller_server` only when nothing
    is called exactly that), then a label holding it anywhere. A `kind:text` selector keeps to
    one kind, and a leading slash keeps to ROS names. A profile is applied before the ranks,
    so the node of that profile answers rather than an ambiguity across profiles.
    """
    text = (selector or "").strip()
    if not text:
        raise UsageError("a selector is needed, for example /planner_server or topic:/scan")
    if text in atlas.entities:
        return Match([text], "id")
    for entity_id in sorted(atlas.entities):
        if text in atlas.entities[entity_id].aka:
            return Match([entity_id], "id")
    kind = ""
    needle = text
    head, sep, tail = text.partition(":")
    if sep and tail and head in ids.ENTITY_KINDS:
        kind, needle = head, tail
    ros_only = not kind and needle.startswith("/")
    lowered = needle.lower()
    ranks: dict[str, list[str]] = {how: [] for how in MATCH_ORDER}
    for entity_id in sorted(atlas.entities):
        node = atlas.entities[entity_id]
        if (kind and node.kind != kind) or (ros_only and node.kind not in ROS_NAME_KINDS):
            continue
        name = label(entity_id).lower()
        if name == lowered:
            ranks["label"].append(entity_id)
        elif _anchored(name, lowered) or _anchored(entity_id.lower(), lowered):
            ranks["suffix"].append(entity_id)
        elif lowered in name:
            ranks["substring"].append(entity_id)
    if profile:
        scoped = {
            how: [item for item in rows if profile in atlas.entities[item].profiles]
            for how, rows in ranks.items()
        }
        if any(scoped.values()):
            ranks = scoped
    for how in MATCH_ORDER:
        if ranks[how]:
            return Match(ranks[how], how)
    return Match([])


def _anchored(name: str, needle: str) -> bool:
    if not name.endswith(needle) or len(name) == len(needle):
        return False
    return needle[0] in _ANCHORS or name[-len(needle) - 1] in _ANCHORS


def resolve(atlas: Atlas, selector: str, *, profile: str = "") -> list[str]:
    """Entity ids a selector names, from the best rank that names any. See `match`."""
    return match(atlas, selector, profile=profile).ids


def _one(atlas: Atlas, selector: str) -> tuple[str | None, list[str], str]:
    found = match(atlas, selector)
    return (found.ids[0] if len(found.ids) == 1 else None), found.ids, found.how


def substring_note(selector: str, found: str) -> str:
    """The line an answer found by substring carries, so a typo is never answered silently."""
    return f"  nothing is named {selector!r}, so this answers for {found}, the one name holding it"


def raised_profiles(atlas: Atlas) -> list[str]:
    """Profiles a launch file raised in, which is why a name can be missing from them."""
    return sorted(
        {
            profile_name(name)
            for item in atlas.findings
            if item.rule == "launch.raised"
            for name in item.profiles
        }
    )


def not_found_lines(atlas: Atlas, selector: str) -> list[str]:
    """The reply when a selector names nothing, shared by the command line and the tools."""
    text = selector.strip()
    slash = text.startswith("/")
    if slash:
        lines = [
            f"nothing in this atlas is named {text!r}, "
            "searched nodes, topics, services, actions and parameters"
        ]
    else:
        lines = [f"nothing in this atlas is named {text!r}"]
    raised = sorted(raised_profiles(atlas))
    if raised:
        shown = _profiles(raised[:5])
        if len(raised) > 5:
            shown = f"{plural(len(raised), 'profile')}, {shown} and {len(raised) - 5} more"
        lines.append(
            f"  a launch file raised in {shown}, so names it starts may be missing, "
            "see the launch.raised findings"
        )
    if slash:
        lines.append("  a leading slash searches ROS names only, drop it to search every kind")
    else:
        lines.append("  try a node name, a topic name, a frame or a package, or a full id")
    return lines


def pasteable(entity_id: str, *, shell: bool = True) -> str:
    """The id a reader pastes back, quoted for a shell. A synthetic id is never offered."""
    if ids.parse(entity_id).synthetic:
        return ""
    return shlex.quote(entity_id) if shell else entity_id


def choice_lines(
    atlas: Atlas, selector: str, matches: Sequence[str], *, shell: bool = True
) -> list[str]:
    """The reply when a selector names several entities: each with the id that names it alone."""
    kept, hidden = _cap(list(matches), DEFAULT_LIMIT)
    lines = [f"{selector!r} names {plural(len(matches), 'entity')}, pick one and pass its id"]
    for entity_id in kept:
        handle = pasteable(entity_id, shell=shell)
        tail = f"   {handle}" if handle else ""
        lines.append(f"  {atlas.entities[entity_id].kind} {label(entity_id)}{tail}")
    _more(lines, hidden, "matches")
    return lines


# ---------------------------------------------------------------------------
# explain


def explain(atlas: Atlas, selector: str, *, limit: int = DEFAULT_LIMIT) -> Answer:
    """One entity in full, or one finding in full when the selector names a finding."""
    text = (selector or "").strip()
    if text.startswith("find:") or text.startswith("sha256:"):
        return _explain_finding(atlas, text)
    if is_rule(text):
        return _explain_rule(atlas, text, limit)
    entity_id, matches, how = _one(atlas, text)
    if entity_id is None:
        return _ambiguous("explain", text, matches, atlas)
    node = atlas.entities[entity_id]
    data: dict[str, Any] = {
        "id": entity_id,
        "kind": node.kind,
        "label": label(entity_id),
        "layers": node.layers,
        "profiles": node.profiles,
        "agreement": node.agreement,
        "conflicts": list(node.conflicts),
        "facts": _fact_rows(node, limit, atlas),
        "effective": _effective_rows(node, limit),
        "edges": _edge_rows(atlas, entity_id, limit),
        "unresolved": _unresolved_rows(atlas, entity_id, limit),
        "findings": [item.id for item in atlas.findings if item.subject_key == entity_id],
    }
    if node.state is not None:
        data["state"] = node.state
    if node.aka:
        data["aka"] = list(node.aka)
    data["matched_by"] = how
    lines = [f"{node.kind} {label(entity_id)}"]
    if how == "substring":
        lines.append(substring_note(text, label(entity_id)))
    lines.append(
        f"  layers {', '.join(node.layers) or 'none'}"
        f"   profiles {_profiles(node.profiles)}"
        f"   agreement {node.agreement}"
    )
    if node.state is not None:
        lines.append(f"  state {node.state}")
    if node.conflicts:
        lines.append(f"  layers disagree about: {', '.join(node.conflicts)}")
    for row in data["facts"]:
        lines.append(
            f"  {row['layer']} in {profile_name(row['profile'])} ({row['confidence']}): "
            f"{row['values']}"
        )
        for where in row["where"]:
            lines.append(f"      {where}")
    for profile, per_key in data["effective"].items():
        pairs = ", ".join(
            f"{key}={_stated(row['value'])}" for key, row in per_key.items() if _said(row["value"])
        )
        # Whole, like the fact lines above it: a cut line had no way to the rest but --json.
        hidden = len(node.effective.get(profile, {})) - len(per_key)
        more = f", and {hidden} more, --limit shows more" if hidden > 0 else ""
        lines.append(f"  effective in {profile_name(profile)}: {readable(pairs, atlas)}{more}")
        for key, row in per_key.items():
            if row["where"]:
                lines.append(f"      {key} from {row['where']}")
    for way, arrow in (("out", "->"), ("in", "<-")):
        counts = data["edges"][f"{way}_by_kind"]
        if not counts:
            continue
        lines.append(
            f"  edges {way}: " + ", ".join(f"{kind} {count}" for kind, count in counts.items())
        )
        for row in data["edges"][way]:
            lines.append(f"    {arrow} {row['kind']} {row['label']}")
    for row in data["unresolved"]:
        lines.append(f"  unresolved {row['want']}: {row['expr']} ({row['reason']}) {row['where']}")
    if data["findings"]:
        lines.append(f"  findings {len(data['findings'])}, see drift")
    if node.kind in ("param", "pparam"):
        lines.append("  run param for the candidate chain")
    return Answer(verb="explain", data=data, lines=lines)


def _explain_finding(atlas: Atlas, handle: str) -> Answer:
    for item in atlas.findings:
        if handle in (item.id, item.fingerprint):
            data = item.to_json()
            lines = [f"{item.severity} {item.rule}: {item.title}"]
            lines.append(f"  about {_subject_label(item)}")
            if item.detail:
                lines.append(f"  {item.detail}")
            if item.fix:
                lines.append(f"  fix: {here(item.fix)}")
            if item.say:
                lines.append(f"  say: {item.spoken}")
            for source in item.where:
                lines.append(f"      {site(source, locator=True)}")
            lines.append(f"  profiles {_profiles(item.profiles)}")
            return Answer(verb="explain", data=data, lines=lines)
    return Answer(
        verb="explain",
        ok=False,
        data={"handle": handle, "findings": len(atlas.findings)},
        lines=[f"no finding with that id or fingerprint, this atlas holds {len(atlas.findings)}"],
    )


def is_rule(text: str) -> bool:
    """Whether a selector is a drift rule id, current or retired."""
    from rosio.drift.rules import ALIASES, COMMITTED

    return text in COMMITTED or text in ALIASES


def _explain_rule(atlas: Atlas, rule: str, limit: int) -> Answer:
    """Every finding of one rule, each with the fingerprint that explains it alone."""
    from rosio.drift.rules import ALIASES

    rule = ALIASES.get(rule, rule)
    rows = [item for item in sorted(atlas.findings, key=Finding.sort_key) if item.rule == rule]
    data = {"rule": rule, "total": len(rows), "findings": [_finding_row(item) for item in rows]}
    if not rows:
        return Answer(
            verb="explain",
            data=data,
            lines=[f"{rule}: no finding of this rule in this atlas"],
        )
    kept, hidden = _cap(rows, limit)
    lines = [f"{rule}: {plural(len(rows), 'finding')}"]
    for item in kept:
        lines.append(f"  {item.severity:<5} {item.title}")
        if item.say:
            lines.append(f"        {item.spoken}")
        for source in item.where[:2]:
            lines.append(f"        {site(source, locator=True)}")
        lines.append(f"        explain {item.fingerprint}")
    _more(lines, hidden, "findings")
    return Answer(verb="explain", data=data, lines=lines)


def _subject_label(item: Finding) -> str:
    if isinstance(item.subject, str):
        return label(item.subject)
    return f"{label(item.subject.src)} {item.subject.kind} {label(item.subject.dst)}"


def _ambiguous(verb: str, selector: str, matches: Sequence[str], atlas: Atlas) -> Answer:
    lines = choice_lines(atlas, selector, matches) if matches else not_found_lines(atlas, selector)
    return Answer(
        verb=verb, ok=False, data={"selector": selector, "matches": list(matches)}, lines=lines
    )


def _fact_rows(node: Node, limit: int, atlas: Atlas | None = None) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for fact in sorted(node.facts, key=lambda item: item.sort_key()):
        keys = sorted(key for key in fact.values if _said(fact.values[key]))
        shown = ", ".join(f"{key}={_stated(fact.values[key])}" for key in keys[:limit])
        if len(keys) > limit:
            shown = f"{shown}, and {len(keys) - limit} more keys"
        text = readable(shown, atlas)
        clipped = _clip(text)
        if clipped != text:
            clipped += " (--json shows every value)"
        rows.append(
            {
                "layer": fact.layer,
                "profile": fact.profile,
                "confidence": fact.confidence,
                "values": clipped or "no values",
                "where": [site(source) for source in fact.evidence[:limit]],
            }
        )
    return rows


def _effective_rows(node: Node, limit: int) -> dict[str, dict[str, Any]]:
    out: dict[str, dict[str, Any]] = {}
    for profile in sorted(node.effective):
        per_key: dict[str, Any] = {}
        for key in sorted(node.effective[profile])[:limit]:
            value = node.effective[profile][key]
            per_key[key] = {
                "value": value.value,
                "layer": value.layer,
                "value_type": value.value_type,
                "where": site(value.source, locator=True),
            }
        out[profile] = per_key
    return out


def _edge_rows(atlas: Atlas, entity_id: str, limit: int) -> dict[str, Any]:
    out_rows = _unique(
        [
            {"kind": edge.kind, "id": edge.dst, "label": label(edge.dst)}
            for edge in _out(atlas, entity_id)
        ]
    )
    in_rows = _unique(
        [
            {"kind": edge.kind, "id": edge.src, "label": label(edge.src)}
            for edge in _into(atlas, entity_id)
        ]
    )
    return {
        "out": _sample(out_rows, limit),
        "in": _sample(in_rows, limit),
        "out_by_kind": _by_kind(out_rows),
        "in_by_kind": _by_kind(in_rows),
    }


def _by_kind(rows: Sequence[dict[str, str]]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for row in rows:
        counts[row["kind"]] = counts.get(row["kind"], 0) + 1
    return dict(sorted(counts.items()))


def _sample(rows: Sequence[dict[str, str]], limit: int, per_kind: int = 3) -> list[dict[str, str]]:
    """A few of each kind rather than the first N of one kind. A node with four hundred
    parameters would otherwise hide the one topic it publishes."""
    counts = _by_kind(rows)
    order = sorted(counts, key=lambda kind: (counts[kind], kind))
    taken: dict[str, int] = {}
    kept: list[dict[str, str]] = []
    for kind in order:
        for row in rows:
            if row["kind"] != kind or taken.get(kind, 0) >= per_kind or len(kept) >= limit:
                continue
            taken[kind] = taken.get(kind, 0) + 1
            kept.append(row)
    return sorted(kept, key=lambda row: (row["kind"], row["label"]))


def _unique(rows: Sequence[dict[str, str]]) -> list[dict[str, str]]:
    seen: set[tuple[str, str]] = set()
    kept: list[dict[str, str]] = []
    for row in rows:
        key = (row["kind"], row["id"])
        if key in seen:
            continue
        seen.add(key)
        kept.append(row)
    return kept


def _unresolved_rows(atlas: Atlas, entity_id: str, limit: int) -> list[dict[str, str]]:
    rows = [
        {
            "want": entry.want,
            "expr": readable(entry.expr, atlas),
            "reason": reason_text(entry.reason, entry.want),
            "where": site(entry.evidence[0]) if entry.evidence else "",
        }
        for entry in sorted(atlas.unresolved, key=lambda item: item.id)
        if entry.owner == entity_id
    ]
    return _cap(rows, limit)[0]


def _out(atlas: Atlas, entity_id: str) -> list[Edge]:
    handles = (atlas.indexes.get("out") or {}).get(entity_id)
    if handles is None:
        return atlas.out_edges(entity_id)
    return [atlas.edges[handle] for handle in sorted(handles) if handle in atlas.edges]


def _into(atlas: Atlas, entity_id: str) -> list[Edge]:
    handles = (atlas.indexes.get("in") or {}).get(entity_id)
    if handles is None:
        return atlas.in_edges(entity_id)
    return [atlas.edges[handle] for handle in sorted(handles) if handle in atlas.edges]


# ---------------------------------------------------------------------------
# param, the highest value question in the product


def param(
    atlas: Atlas,
    node: str,
    name: str | None = None,
    *,
    profile: str | None = None,
) -> Answer:
    """The effective value of one parameter, the winner with its file and line, and the
    full shadowed chain. This is the question an engineer actually asks."""
    wanted = _profile_id(atlas, profile)
    param_id, matches, how = _param_id(atlas, node, name, wanted)
    if param_id is None:
        return _no_single_param(atlas, node, name, matches)
    entity = atlas.entities[param_id]
    owner = ids.parse_param(param_id).owner if entity.kind == "param" else ""
    data: dict[str, Any] = {
        "id": param_id,
        "node": label(owner) if owner else "",
        "name": ids.parse_param(param_id).name if entity.kind == "param" else label(param_id),
        "state": entity.state,
        "matched_by": how,
        "profiles": {},
    }
    state = f"state {entity.state}"
    lost = bool(wanted) and entity.state == "effective" and wanted not in entity.effective
    if lost:
        # The entity keeps the strongest state of all profiles, which is not this one's.
        others = ", ".join(profile_name(item) for item in sorted(entity.effective))
        state = f"state effective in {others}, no value wins in {profile_name(wanted)}"
        data["profile_state"] = "no_winner"
    lines = [f"{data['name']} on {data['node'] or label(param_id)}   {state}"]
    if how == "substring":
        lines.append(substring_note(f"{node} {name or ''}".strip(), label(param_id)))
    if entity.state != "effective":
        lines.append("  no effective value: " + _state_reason(atlas, entity, wanted))
    profiles = sorted({item.profile for item in entity.candidates} | set(entity.effective))
    if wanted:
        profiles = [item for item in profiles if item == wanted]
        if not profiles:
            lines.append(f"  this parameter is not set in {profile_name(wanted)}")
        elif lost:
            notes = sorted(
                {row.note for row in entity.candidates if row.profile == wanted and row.note}
            )
            lines.append(
                f"  no value wins in {profile_name(wanted)}: "
                + ("; ".join(notes) or "no candidate of this profile wins, see the chain")
            )
    for item in profiles:
        chain = sorted(
            [row for row in entity.candidates if row.profile in (item, ANY_PROFILE)],
            key=Candidate.sort_key,
        )
        effective = entity.effective.get(item, {}).get("value")
        rows = [_candidate_row(row, tree_tag(atlas, row.source)) for row in chain]
        tag = tree_tag(atlas, effective.source) if effective else ""
        data["profiles"][item] = {
            "value": effective.value if effective else None,
            "value_type": effective.value_type if effective else None,
            "winner": site(effective.source, locator=True, tag=tag) if effective else "",
            "chain": rows,
        }
        if effective is not None:
            data["profiles"][item]["read_from"] = tag or effective.source.root or "src"
        lines.append(f"  profile {profile_name(item)}")
        if effective is not None:
            lines.append(
                f"    effective {_value(effective.value)} ({effective.value_type})"
                f"   edit {site(effective.source, tag=tag)}"
            )
        for row in rows:
            mark = "wins " if row["wins"] else "     "
            shadow = (
                f"   shadowed by rank {row['shadowed_by']}, {row['shadow_kind']}"
                if not row["wins"] and row["shadow_kind"]
                else ""
            )
            lines.append(
                f"    rank {row['rank']} {mark}{row['mechanism']:<20} "
                f"{_value(row['value'])} ({row['value_type']})"
            )
            lines.append(f"        {row['where']}{shadow}")
            if row["applied_by"]:
                lines.append(f"        applied by {label(row['applied_by'])}")
    for entry in _unresolved_rows(atlas, param_id, DEFAULT_LIMIT):
        lines.append(f"  unresolved {entry['want']}: {entry['expr']} ({entry['reason']})")
    return Answer(verb="param", data=data, lines=lines)


def _no_single_param(atlas: Atlas, node: str, name: str | None, matches: Sequence[str]) -> Answer:
    """The reply when a param query named no single parameter.

    One token is a parameter name with no node, which every node may carry, so the reply
    names the nodes that carry it and the two token form that picks one.
    """
    selector = (node or "").strip() if name is None else f"{node} {name}".strip()
    if not matches and " " in selector:
        return _no_owner_or_name(atlas, selector)
    if name is not None or " " in selector:
        return _ambiguous("param", selector, matches, atlas)
    data = {"selector": selector, "matches": list(matches)}
    if not matches:
        owners = [item for item in resolve(atlas, selector) if atlas.entities[item].kind == "node"]
        if owners:
            return Answer(
                verb="param",
                ok=False,
                data=data,
                lines=[
                    f"{label(owners[0])} is a node, and param answers about one of its parameters",
                    f"  name one: param {label(owners[0])} use_sim_time",
                    "  explain that node for the parameters it carries",
                ],
            )
        return Answer(
            verb="param",
            ok=False,
            data=data,
            lines=[
                f"no parameter named {selector!r} is in this atlas",
                f"  param takes a node and a name: param /controller_server {selector}",
                "  query the name on its own to see what does carry it",
            ],
        )
    kept, hidden = _cap(list(matches), DEFAULT_LIMIT)
    lines = [
        f"{plural(len(matches), 'parameter')} match {selector!r} and a param answer is about one, "
        "so name the node as well"
    ]
    for entity_id in kept:
        if atlas.entities[entity_id].kind == "param":
            owner, param_name = ids.parse_param(entity_id)
            lines.append(f"  param {label(owner)} {param_name}")
        else:
            lines.append(f"  {atlas.entities[entity_id].kind} {label(entity_id)}")
    _more(lines, hidden, "matches")
    return Answer(verb="param", ok=False, data=data, lines=lines)


def _no_owner_or_name(atlas: Atlas, selector: str) -> Answer:
    """Which half of a node and a name found nothing. One quoted string read as one name."""
    node, name = selector.split(None, 1)
    head = node.partition(":")[0]
    owners = match(atlas, node if head in ids.ENTITY_KINDS else f"node:{node}").ids
    owners = [item for item in owners if atlas.entities[item].kind == "node"]
    data = {"selector": selector, "matches": []}
    if not owners:
        lines = not_found_lines(atlas, node)
        lines[0] = f"no node {node} in this atlas, searched nodes"
        return Answer(verb="param", ok=False, data=data, lines=lines)
    lines = [
        f"{label(owners[0])} carries no parameter {name}",
        f"  explain {label(owners[0])} for the parameters it carries",
    ]
    return Answer(verb="param", ok=False, data=data, lines=lines)


def _candidate_row(row: Candidate, tag: str = "") -> dict[str, Any]:
    return {
        "rank": row.rank,
        "wins": row.wins,
        "value": row.value,
        "value_type": row.value_type,
        "mechanism": row.mechanism,
        "where": site(row.source, locator=True, tag=tag),
        "shadow_kind": row.shadow_kind,
        "shadowed_by": row.shadowed_by,
        "applied_by": row.applied_by,
    }


_STATE_NOTES = {
    "inert_by_design": "the block is not read by the node that owns the name",
    "undeclared": "the value is set but the code never declares the parameter",
    "declared_unread": "the code declares it and never reads it",
    "owner_name_mismatch": "the yaml key matches no launched node, so nothing reads this block",
    "unresolved_owner": "the owning node could not be resolved",
    "other": "the chain could not be followed to a winner",
}


def _state_note(state: str) -> str:
    return _STATE_NOTES.get(state, "see the state field")


def _state_reason(atlas: Atlas, entity: Node, wanted: str) -> str:
    """Why no value wins, from the notes the chain carries before the generic sentence, and
    with the declarations whose names are built at run time that could be this one."""
    from rosio import join

    notes = sorted(
        {
            row.note
            for row in entity.candidates
            if row.note and (not wanted or row.profile == wanted)
        }
    )
    if not notes:
        return _state_note(str(entity.state))
    text = "; ".join(notes)
    if join.BLIND_NOTE in notes:
        owner, name = ids.parse_param(entity.id)
        node = atlas.entities.get(owner)
        read = {
            source
            for fact in (node.facts if node is not None else [])
            if fact.layer == "coded"
            for source in fact.values.get("coded_sources") or []
        }
        built = sorted(
            f"{expr} in {ids.parse_file(file).package}/{ids.parse_file(file).relpath}"
            for file, exprs in join.blind_names(atlas, "param_name").items()
            if file in read
            for expr in exprs
            if join.could_name(expr, name)
        )
        if built:
            text += f" ({', '.join(built[:3])}{', and more' if len(built) > 3 else ''})"
    return text


def _param_id(
    atlas: Atlas, node: str, name: str | None, profile: str = ""
) -> tuple[str | None, list[str], str]:
    """The parameter a node and a name select, with how it was found. A name only some
    parameter holds is answered and said so, never handed over as if it were exact."""
    text = (node or "").strip()
    if name is None and " " in text:
        text, name = text.split(None, 1)
    if name is None:
        found = match(atlas, text, profile=profile)
        matches = [item for item in found.ids if atlas.entities[item].kind in ("param", "pparam")]
        return (matches[0] if len(matches) == 1 else None), matches, found.how
    head = text.partition(":")[0]
    owners = match(atlas, text if head in ids.ENTITY_KINDS else f"node:{text}", profile=profile)
    owner_ids = [item for item in owners.ids if atlas.entities[item].kind == "node"]
    wanted = ids.normalise_param_name(name)
    matches = [
        ids.build_param(owner, wanted)
        for owner in owner_ids
        if ids.build_param(owner, wanted) in atlas.entities
    ]
    how = owners.how
    if not matches:
        how = "substring"
        lowered = wanted.lower()
        for entity_id in sorted(atlas.entities):
            if atlas.entities[entity_id].kind != "param":
                continue
            owner, param_name = ids.parse_param(entity_id)
            if owner in owner_ids and lowered in param_name.lower():
                matches.append(entity_id)
    return (matches[0] if len(matches) == 1 else None), matches, how


def _profile_id(atlas: Atlas, profile: str | None) -> str:
    if not profile:
        return ""
    wanted = profile if profile.startswith("profile:") else ids.build_profile(profile)
    if wanted not in atlas.entities:
        raise UsageError(
            f"unknown profile {profile!r}, this atlas holds: "
            + ", ".join(profile_name(item.id) for item in atlas.by_kind("profile"))
        )
    return wanted


# ---------------------------------------------------------------------------
# query, matched by name and kind with no language model


_STOPWORD_TEXT = (
    "a an and are as at by do does for from get give has have how in into is it its list me my "
    "of on or please show that the their them there these this to what when where which who "
    "whom whose why with"
)
_STOPWORDS = frozenset(_STOPWORD_TEXT.split())

_KIND_WORDS = {
    "node": "node",
    "nodes": "node",
    "topic": "topic",
    "topics": "topic",
    "service": "service",
    "services": "service",
    "action": "action",
    "actions": "action",
    "param": "param",
    "params": "param",
    "parameter": "param",
    "parameters": "param",
    "frame": "frame",
    "frames": "frame",
    "plugin": "plugin",
    "plugins": "plugin",
    "package": "pkg",
    "packages": "pkg",
    "file": "file",
    "files": "file",
    "launch": "launchfile",
    "profile": "profile",
    "profiles": "profile",
    "type": "type",
    "types": "type",
    "executable": "exe",
    "symbol": "sym",
}

_RELATION_WORDS = {
    "publish": "publishes",
    "publishes": "publishes",
    "published": "publishes",
    "publisher": "publishes",
    "publishers": "publishes",
    "subscribe": "subscribes",
    "subscribes": "subscribes",
    "subscriber": "subscribes",
    "subscribers": "subscribes",
    "remap": "remaps",
    "remaps": "remaps",
    "remapped": "remaps",
    "declares": "declares_param",
    "reads": "reads_param",
    "sets": "sets_param",
    "broadcast": "broadcasts_tf",
    "broadcasts": "broadcasts_tf",
    "launches": "launches",
    "serves": "serves",
    "calls": "calls",
}

_WORD_RE = re.compile(r"[A-Za-z0-9_/.:~-]+")

_ENDPOINT_WANTS = {
    "publishes": ("topic_name",),
    "subscribes": ("topic_name",),
    "serves": ("action_name", "service_name"),
    "calls": ("action_name", "service_name"),
}
"""The blind spot want that could hide an edge of each endpoint relation."""


def query(atlas: Atlas, question: str, *, limit: int = DEFAULT_LIMIT) -> Answer:
    """A scoped subgraph for a plain language question, matched by name and kind only.

    There is no language model anywhere in this. Words that name a kind or a relation scope
    the search, the rest are matched against entity names. When nothing matches, the answer
    says so and says what it looked for, rather than returning a plausible subgraph.
    """
    words = [word.lower() for word in _WORD_RE.findall(question or "")]
    kinds = {_KIND_WORDS[word] for word in words if word in _KIND_WORDS}
    relations = {_RELATION_WORDS[word] for word in words if word in _RELATION_WORDS}
    needles = [
        word
        for word in words
        if word not in _STOPWORDS
        and word not in _KIND_WORDS
        and word not in _RELATION_WORDS
        and len(word) >= 3
    ]
    data: dict[str, Any] = {
        "question": question,
        "kinds": sorted(kinds),
        "relations": sorted(relations),
        "terms": needles,
        "matched": [],
        "edges": [],
    }
    if not needles and not kinds:
        return Answer(
            verb="query",
            ok=False,
            data=data,
            lines=[
                "this question carries no name and no kind, so it was not understood",
                "  name a node, a topic, a parameter or a frame, for example "
                '"which nodes publish cmd_vel"',
            ],
        )

    stated = [token for token in (question or "").split() if _names_an_entity(token)]
    missing = [token for token in stated if token not in atlas.entities]
    if missing:
        return _no_such_entity(atlas, missing[0], data, limit)
    matched = list(dict.fromkeys(stated)) or _matches(atlas, needles, kinds)
    widened = not matched and bool(kinds) and bool(needles)
    if not matched and kinds:
        # "which nodes publish cmd_vel" names a topic and asks for nodes. Keep the named
        # entity and let the relations carry the answer, rather than reporting nothing.
        matched = _matches(atlas, needles, set())
    if not matched:
        looked = ", ".join(needles + sorted(kinds)) or "nothing"
        inventory = ", ".join(
            f"{len(value)} {key}"
            for key, value in sorted((atlas.indexes.get("by_kind") or {}).items())
        )
        return Answer(
            verb="query",
            ok=False,
            data=data,
            lines=[
                f"nothing matched {looked}, so this question was not answered",
                f"  this atlas holds {inventory or 'no entities'}",
            ],
        )

    # Relations come from every match before any is cut, so a cap never turns into "no
    # such edge", and the matches that carry an asked relation are listed first.
    if relations:
        related = {
            entity_id
            for entity_id in matched
            if any(
                edge.kind in relations for edge in _out(atlas, entity_id) + _into(atlas, entity_id)
            )
        }
        matched = [item for item in matched if item in related] + [
            item for item in matched if item not in related
        ]
    kept, hidden = _cap(matched, limit)
    rows: list[dict[str, str]] = []
    for entity_id in matched if relations else kept:
        for edge in _out(atlas, entity_id) + _into(atlas, entity_id):
            if relations and edge.kind not in relations:
                continue
            rows.append(
                {
                    "kind": edge.kind,
                    "src": edge.src,
                    "dst": edge.dst,
                    "src_label": label(edge.src),
                    "dst_label": label(edge.dst),
                }
            )
    rows = _unique_edges(rows)
    edge_rows, edge_hidden = _cap(rows, limit * 2)
    data["matched"] = [
        {
            "id": entity_id,
            "kind": atlas.entities[entity_id].kind,
            "label": label(entity_id),
            "layers": atlas.entities[entity_id].layers,
            "profiles": atlas.entities[entity_id].profiles,
        }
        for entity_id in kept
    ]
    data["edges"] = edge_rows
    data["widened"] = widened
    verb = "matches" if len(matched) == 1 else "match"
    lines = [
        f"{plural(len(matched), 'entity')} {verb} {', '.join(needles) or ', '.join(sorted(kinds))}"
    ]
    if widened:
        lines.append(
            f"  nothing of kind {', '.join(sorted(kinds))} carries that name, so every kind "
            "was searched and the relations below carry the answer"
        )
    for row in data["matched"]:
        lines.append(
            f"  {row['kind']} {row['label']}   layers {', '.join(row['layers']) or 'none'}"
        )
    _more(lines, hidden, "entities")
    if edge_rows:
        lines.append("  relations")
        for row in edge_rows:
            lines.append(f"    {row['src_label']} {row['kind']} {row['dst_label']}")
        _more(lines, edge_hidden, "relations")
    elif relations:
        wants = sorted({want for kind in relations for want in _ENDPOINT_WANTS.get(kind, ())})
        blind = sum(1 for entry in atlas.unresolved if entry.want in wants)
        data["blind_spots"] = blind
        if blind:
            lines.append(
                f"  no {', '.join(sorted(relations))} edge touches these in this atlas, and "
                f"{plural(blind, 'endpoint name')} the source builds at run time could be one "
                f"of them: rosio unresolved --want {' --want '.join(wants)}"
            )
        else:
            lines.append(
                f"  no {', '.join(sorted(relations))} edge touches these in this atlas, "
                "which is an answer and not a failure"
            )
    lines.append("  explain any of these for the facts and the provenance")
    return Answer(verb="query", data=data, lines=lines)


def _names_an_entity(token: str) -> bool:
    """`topic:/scan` is a selector, read exactly. A word that merely holds a colon is not."""
    kind, colon, rest = token.partition(":")
    return bool(colon and rest) and kind in ids.ENTITY_KINDS


def _no_such_entity(atlas: Atlas, selector: str, data: dict[str, Any], limit: int) -> Answer:
    """A selector names one entity, so a near name is offered and never answered for it."""
    kind, _, rest = selector.partition(":")
    try:
        name = label(selector)
    except ContractError:
        name = rest
    near = sorted(
        label(entity.id)
        for entity in atlas.by_kind(kind)
        if name.lower() in label(entity.id).lower()
    )
    kept, hidden = _cap(near, limit)
    line = f"no {kind} {name} in this atlas"
    if kept:
        line += f", near: {', '.join(kept)}" + (f" and {hidden} more" if hidden else "")
    else:
        line += f", and no {kind} name holds it"
    return Answer(verb="query", ok=False, data=data, lines=[line])


def _matches(atlas: Atlas, needles: Sequence[str], kinds: set[str]) -> list[str]:
    scored: list[tuple[int, str]] = []
    for entity_id in sorted(atlas.entities):
        node = atlas.entities[entity_id]
        if kinds and node.kind not in kinds:
            continue
        name = f"{label(entity_id)} {entity_id}".lower()
        hits = sum(1 for needle in needles if needle in name)
        if needles and not hits:
            continue
        scored.append((-hits, entity_id))
    return [entity_id for _hits, entity_id in sorted(scored)]


def _unique_edges(rows: Sequence[dict[str, str]]) -> list[dict[str, str]]:
    seen: set[tuple[str, str, str]] = set()
    kept: list[dict[str, str]] = []
    for row in sorted(rows, key=lambda item: (item["kind"], item["src"], item["dst"])):
        key = (row["kind"], row["src"], row["dst"])
        if key in seen:
            continue
        seen.add(key)
        kept.append(row)
    return kept


# ---------------------------------------------------------------------------
# path


def path(atlas: Atlas, source: str, target: str, *, max_hops: int = MAX_PATH_HOPS) -> Answer:
    """The shortest path between two entities, with the edge kind of every hop.

    The walk is undirected, because data reaches a subscriber by a publishes edge followed
    by a subscribes edge read backwards, and a reader wants that path. The direction of each
    hop is stated, so nothing here claims a flow that does not exist. The walk keeps to data
    edges first, and a route through lifecycle management, launch or parameters is shown only
    when no data route exists, and says so.
    """
    start, start_matches, start_how = _one(atlas, source)
    if start is None:
        return _ambiguous("path", source, start_matches, atlas)
    end, end_matches, end_how = _one(atlas, target)
    if end is None:
        return _ambiguous("path", target, end_matches, atlas)
    notes = [
        substring_note(asked, label(found))
        for asked, found, how in ((source, start, start_how), (target, end, end_how))
        if how == "substring"
    ]
    if start == end:
        return Answer(
            verb="path",
            data={"from": start, "to": end, "hops": [], "found": True, "data_route": True},
            lines=[f"{label(start)} is the entity asked for on both ends, no hops"],
        )
    hops = _shortest(atlas, start, end, max_hops, DATA_EDGE_KINDS)
    data_route = bool(hops)
    if not hops:
        hops = _shortest(atlas, start, end, max_hops)
    data: dict[str, Any] = {
        "from": start,
        "to": end,
        "hops": hops or [],
        "found": bool(hops),
        "data_route": data_route,
        "data_direction": _data_direction(hops) if data_route else "",
    }
    if not hops:
        return Answer(
            verb="path",
            ok=False,
            data=data,
            lines=[
                f"no path from {label(start)} to {label(end)} within {max_hops} hops",
                "  the two are in different components of this atlas, or the binding is "
                "unresolved, see unresolved",
            ],
        )
    lines = [f"{plural(len(hops), 'hop')} from {label(start)} to {label(end)}", *notes]
    if not data_route:
        other = sorted({hop["kind"] for hop in hops} - DATA_EDGE_KINDS)
        lines.append(
            f"  no data route connects them, this one runs through {', '.join(other)}, "
            "which carries no message between them"
        )
    elif data["data_direction"] == "backward":
        lines.append(f"  data flows the other way, from {label(end)} to {label(start)}")
    elif data["data_direction"] == "mixed":
        lines.append("  no message travels this route end to end, some hops run against the data")
    lines.append(f"  {label(start)}")
    for hop in hops:
        arrow = "->" if hop["forward"] else "<-"
        lines.append(f"  {arrow} {hop['kind']} {label(hop['to'])}")
    return Answer(verb="path", data=data, lines=lines)


DATA_EDGE_KINDS = frozenset(
    {"publishes", "subscribes", "serves", "calls", "remaps", "broadcasts_tf"}
)
"""The edges data travels along. A path is looked for through these first."""

DATA_LEAVES_SOURCE = frozenset({"publishes", "calls", "broadcasts_tf"})
"""Data kinds whose message leaves the edge source. A subscription and a server receive."""


def _data_direction(hops: Sequence[Mapping[str, Any]]) -> str:
    """`forward` when every hop runs with the data, `backward` when every hop runs against
    it, `mixed` otherwise. A remap carries no message and takes no side."""
    sides = {
        (hop["kind"] in DATA_LEAVES_SOURCE) == hop["forward"]
        for hop in hops
        if hop["kind"] != "remaps"
    }
    if sides == {False}:
        return "backward"
    return "mixed" if len(sides) > 1 else "forward"


def _shortest(
    atlas: Atlas,
    start: str,
    end: str,
    max_hops: int,
    kinds: frozenset[str] | None = None,
) -> list[dict[str, Any]]:
    if start == end:
        return []
    seen = {start}
    queue: deque[tuple[str, list[dict[str, Any]]]] = deque([(start, [])])
    while queue:
        current, trail = queue.popleft()
        if len(trail) >= max_hops:
            continue
        for edge, forward in _neighbours(atlas, current):
            if kinds is not None and edge.kind not in kinds:
                continue
            other = edge.dst if forward else edge.src
            if other in seen:
                continue
            step = {"kind": edge.kind, "to": other, "forward": forward, "key": edge.key}
            if other == end:
                return [*trail, step]
            seen.add(other)
            queue.append((other, [*trail, step]))
    return []


def _neighbours(atlas: Atlas, entity_id: str) -> list[tuple[Edge, bool]]:
    rows = [(edge, True) for edge in _out(atlas, entity_id)]
    rows += [(edge, False) for edge in _into(atlas, entity_id)]
    return sorted(rows, key=lambda item: (item[0].id, not item[1]))


# ---------------------------------------------------------------------------
# diff


def diff(
    left: Atlas,
    right: Atlas | None = None,
    *,
    left_profile: str | None = None,
    right_profile: str | None = None,
    limit: int = DEFAULT_LIMIT,
) -> Answer:
    """What changed between two atlases, or between two profiles of one, and whether it matters.

    Volatile keys and line hints are excluded at every depth, so a file gaining a line and a
    rate moving by a hair are not differences. What is left is grouped by what a reader
    actually asks about, in this order: an effective value that moved, an entry point or a
    profile that arrived or went, a node that appeared or vanished, a topic that lost an
    endpoint, a finding that is new or gone, and a parameter whose state changed. The ids
    and the counts come after all of that, because they answer a question nobody asked
    first, and the build identity comes last because it answers a different question.
    """
    other = right if right is not None else left
    if left_profile or right_profile:
        if not (left_profile and right_profile):
            raise UsageError("comparing profiles needs both, left_profile and right_profile")
        left_id = _profile_id(left, left_profile)
        right_id = _profile_id(other, right_profile)
        before = _profile_view(left, left_id)
        after = _profile_view(other, right_id)
        sides = (_side(left, left_id), _side(other, right_id))
        scope = f"profile {profile_name(left_profile)} against {profile_name(right_profile)}"
    else:
        if right is None:
            raise UsageError("diff needs a second atlas, or two profiles of one atlas")
        before = _atlas_view(left)
        after = _atlas_view(other)
        sides = (_side(left), _side(other))
        scope = "atlas against atlas"
    entities = _compare(before["entities"], after["entities"])
    edges = _compare(before["edges"], after["edges"])
    matters = _matters(sides[0], sides[1], limit)
    build = {
        "entities": _counts(entities["build"]),
        "edges": _counts(edges["build"]),
        "note": BUILD_NOTE,
    }
    data: dict[str, Any] = {
        "scope": scope,
        "matters": matters,
        "matters_total": _matters_total(matters),
        "entities": _diff_rows(entities, limit),
        "edges": _diff_rows(edges, limit),
        "counts": {"entities": _counts(entities), "edges": _counts(edges)},
        "build": build,
        "identical": _nothing_moved(entities) and _nothing_moved(edges),
    }
    lines = [f"diff, {scope}"]
    if data["identical"]:
        lines.append("  no semantic difference, line hints and volatile keys excluded")
        return Answer(verb="diff", data=data, lines=lines)
    ids_below = any(sum(data["counts"][what].values()) for what in ("entities", "edges"))
    lines.extend(_matters_lines(matters, limit, ids_below))
    for what in ("entities", "edges"):
        counts = data["counts"][what]
        lines.append(
            f"  {what}: {counts['added']} added, {counts['removed']} removed, "
            f"{counts['changed']} changed"
        )
        for key in ("added", "removed", "changed"):
            kept, _hidden = _cap(data[what][key], min(limit, 5))
            for entry in kept:
                fields = f"   {', '.join(entry['fields'])}" if entry.get("fields") else ""
                lines.append(f"    {key} {_clip(entry['label'], 120)}{fields}")
            _more(lines, counts[key] - len(kept), f"{key} {what}")
    lines.extend(_build_lines(build))
    return Answer(verb="diff", data=data, lines=lines)


def _nothing_moved(compared: Mapping[str, Any]) -> bool:
    return not any(_counts(compared).values()) and not any(_counts(compared["build"]).values())


def _build_lines(build: Mapping[str, Any]) -> list[str]:
    """The build identity, apart, last and in one line, so it answers its own question and
    no other. The counts are in the data."""
    counts = [build[what] for what in ("entities", "edges")]
    if not any(sum(row.values()) for row in counts):
        return ["  build identity: one build wrote both sides, nothing here moved"]
    return [
        "  build identity moved too (run ids, input digests, root hashes), which says "
        "nothing about the system"
    ]


def _side(atlas: Atlas, profile: str | None = None) -> dict[str, Any]:
    """One side of a comparison, in the terms a reader thinks in.

    In profile mode the profile is taken out of every key, because the whole point of
    comparing two profiles is to line up the same fact under two different names.
    """
    wanted = (profile, ANY_PROFILE) if profile else None
    values: dict[tuple, dict[str, Any]] = {}
    candidates: dict[tuple, dict[str, Any]] = {}
    states: dict[str, str] = {}
    nodes: dict[str, str] = {}
    launch: dict[str, str] = {}
    profiles: dict[str, str] = {}
    kinds: dict[str, str] = {}
    for entity_id, node in atlas.entities.items():
        kinds[entity_id] = node.kind
        if node.kind == "node":
            nodes[entity_id] = label(entity_id)
        if node.kind == "launchfile":
            launch[entity_id] = label(entity_id)
        if node.kind == "profile":
            profiles[entity_id] = label(entity_id)
        if node.kind not in ("param", "pparam"):
            continue
        if node.state:
            states[entity_id] = node.state
        for row in node.candidates:
            if wanted and row.profile not in wanted:
                continue
            spot = (row.source.file, row.source.locator) if row.source else ("", "")
            seat = (entity_id, *spot) if wanted else (entity_id, row.profile, *spot)
            candidates[seat] = {
                "value": row.value,
                "profile": row.profile,
                "wins": row.wins,
                "locator": site(row.source, locator=True),
            }
        for slot in sorted(node.effective):
            if wanted and slot not in wanted:
                continue
            for key, value in node.effective[slot].items():
                seat = (entity_id, key) if wanted else (entity_id, slot, key)
                values[seat] = {
                    "value": value.value,
                    "value_type": value.value_type,
                    "profile": slot,
                    "where": site(value.source),
                    "locator": site(value.source, locator=True),
                }
    endpoints: dict[tuple[str, str], set[str]] = {}
    for edge in atlas.edges.values():
        if edge.kind not in ("publishes", "subscribes"):
            continue
        if wanted and not any(fact.profile in wanted for fact in edge.facts):
            continue
        endpoints.setdefault((edge.dst, edge.kind), set()).add(edge.src)
    # Keyed by the finding id and not by the fingerprint. The fingerprint covers the
    # normalised detail, and a rate stated to six places moves whenever anything at all
    # resolves, so keying on it reports one finding twice, once as gone and once as new.
    findings = {}
    prints = {}
    for item in atlas.findings:
        if wanted and item.profiles and not any(name in wanted for name in item.profiles):
            continue
        if profile and item.profiles == [profile]:
            # the discriminator carries the profile, so a finding of this profile alone is
            # lined up by its title
            title = _without_profile(" ".join(item.title.split()), profile)
            key = "|".join((item.rule, item.subject_key, title))
            said = " ".join(f"{item.say} {item.detail}".split())
            prints[key] = _without_profile(said, profile)
        else:
            key = _finding_key(item)
            prints[key] = _finding_print(item)
        findings[key] = item
    return {
        "values": values,
        "candidates": candidates,
        "states": states,
        "nodes": nodes,
        "launch": launch,
        "profiles": profiles,
        "kinds": kinds,
        "endpoints": endpoints,
        "findings": findings,
        "prints": prints,
    }


def _without_profile(text: str, profile: str) -> str:
    name = re.escape(profile_name(profile))
    return re.sub(rf"(?<![\w.]){name}(?![\w]|\.\w)", "<profile>", text)


def _matters(before: dict[str, Any], after: dict[str, Any], limit: int) -> dict[str, Any]:
    """The change a person asked about, grouped, before any id or count is printed."""
    room = max(limit, 1) * 4
    return {
        "values": _value_moves(before, after)[:room],
        "candidates": _candidate_moves(before, after)[:room],
        "launch": _gone_and_new(before["launch"], after["launch"], room),
        "profiles": _gone_and_new(before["profiles"], after["profiles"], room),
        "nodes": {
            "appeared": _named(after["nodes"], set(after["nodes"]) - set(before["nodes"]))[:room],
            "vanished": _named(before["nodes"], set(before["nodes"]) - set(after["nodes"]))[:room],
        },
        "topics": _endpoint_moves(before, after, room),
        "findings": {
            "new": _finding_moves(after, before)[:room],
            "gone": _finding_moves(before, after)[:room],
            "restated": _finding_restated(before, after)[:room],
        },
        "states": _state_moves(before, after)[:room],
    }


def _value_moves(before: dict[str, Any], after: dict[str, Any]) -> list[dict[str, Any]]:
    """Effective values that moved. The line is reported and never compared."""
    rows = []
    for seat in sorted(set(before["values"]) & set(after["values"]), key=str):
        was, now = before["values"][seat], after["values"][seat]
        if was["value"] == now["value"] and type(was["value"]) is type(now["value"]):
            continue
        entity_id = seat[0]
        rows.append(
            {
                "id": entity_id,
                "label": label(entity_id),
                "key": seat[-1],
                "profile": now["profile"],
                "before": was["value"],
                "after": now["value"],
                "value_type": now["value_type"],
                "where": now["where"],
                "locator": now["locator"],
            }
        )
    return rows


def _candidate_moves(before: dict[str, Any], after: dict[str, Any]) -> list[dict[str, Any]]:
    """Lines whose value moved while the effective value stayed, such as an edit to a line
    another one shadows. The edit is real and it changed nothing, and both are said."""
    moved_values = {row["id"] for row in _value_moves(before, after)}
    rows = []
    for seat in sorted(set(before["candidates"]) & set(after["candidates"]), key=str):
        was, now = before["candidates"][seat], after["candidates"][seat]
        if seat[0] in moved_values or (
            was["value"] == now["value"] and type(was["value"]) is type(now["value"])
        ):
            continue
        effective = [
            row["value"]
            for key, row in after["values"].items()
            if key[0] == seat[0] and row["profile"] in (now["profile"], ANY_PROFILE)
        ]
        rows.append(
            {
                "id": seat[0],
                "label": label(seat[0]),
                "profile": now["profile"],
                "before": was["value"],
                "after": now["value"],
                "wins": now["wins"],
                "effective": effective[0] if effective else None,
                "locator": now["locator"],
            }
        )
    return rows


def _named(labels: Mapping[str, str], keys: Iterable[str]) -> list[dict[str, str]]:
    return [{"id": key, "label": labels[key]} for key in sorted(keys)]


def _gone_and_new(
    before: Mapping[str, str], after: Mapping[str, str], room: int
) -> dict[str, list[dict[str, str]]]:
    """An entry point or a profile that arrived or went.

    A launch file that is gone takes a whole system with it, and the nodes it started are
    the consequence and not the change. Named on its own, before any node.
    """
    return {
        "appeared": _named(after, set(after) - set(before))[:room],
        "vanished": _named(before, set(before) - set(after))[:room],
    }


def _endpoint_moves(before: dict[str, Any], after: dict[str, Any], room: int) -> dict[str, Any]:
    """A topic that lost or gained its last endpoint of one direction.

    A topic that went with its endpoints is not reported here: it is one removal, and
    saying it twice reads as two problems.
    """
    out: dict[str, list[dict[str, str]]] = {}
    for kind, what in (("publishes", "publisher"), ("subscribes", "subscriber")):
        lost, gained = [], []
        topics = {
            seat[0] for seat in [*before["endpoints"], *after["endpoints"]] if seat[1] == kind
        }
        for topic in sorted(topics):
            had = before["endpoints"].get((topic, kind)) or set()
            has = after["endpoints"].get((topic, kind)) or set()
            if had and not has and topic in after["kinds"]:
                lost.append({"id": topic, "label": label(topic), "was": len(had)})
            elif has and not had and topic in before["kinds"]:
                gained.append({"id": topic, "label": label(topic), "now": len(has)})
        out[f"lost_{what}"] = lost[:room]
        out[f"gained_{what}"] = gained[:room]
    return out


def _finding_key(item: Finding) -> str:
    """What identifies one finding across two builds.

    The id is it, except for a finding about a run. That subject carries the digest of the
    inputs, so one byte anywhere in the tree renames the finding and the same refusal on
    both sides reads as one gone and one new, in the grouped head a reader trusts first. The
    digest leaves the key, and the layer and the profile of the run stay in it, which is what
    two builds of one tree agree on. The row still reports the real id, because that is what
    a waiver keys on.
    """
    subject = item.subject_key
    if not _names_a_build(subject):
        return item.id
    run = ids.parse_run(subject)
    return "|".join(("find", item.rule, run.layer, run.profile, item.discriminator or "-"))


def _finding_print(item: Finding) -> str:
    """The fingerprint of a finding with the build out of it.

    The fingerprint covers the subject, so a finding about a run carries the input digest
    into it and two builds of one tree never agree on one. Restated means the same finding
    saying it with different numbers, and a renamed run is not different numbers.
    """
    if not _names_a_build(item.subject_key):
        return item.fingerprint
    return "sha256:" + ids.digest(
        item.rule,
        _finding_key(item),
        item.discriminator,
        " ".join(item.detail.split()),
        length=16,
    )


def _finding_moves(side: dict[str, Any], other: dict[str, Any]) -> list[dict[str, Any]]:
    rows = []
    for key in sorted(set(side["findings"]) - set(other["findings"])):
        item = side["findings"][key]
        rows.append(
            {
                "id": item.id,
                "rule": item.rule,
                "severity": item.severity,
                "say": item.say or item.title,
                "fingerprint": item.fingerprint,
            }
        )
    return sorted(rows, key=lambda row: (SEVERITY_ORDER.get(row["severity"], 9), row["rule"]))


def _finding_restated(before: dict[str, Any], after: dict[str, Any]) -> list[dict[str, Any]]:
    """The same finding on both sides, saying it with different numbers.

    It is neither new nor gone, and a waiver on it still matches, so it is reported as
    itself rather than as two events.
    """
    rows = []
    for key in sorted(set(before["findings"]) & set(after["findings"])):
        was, now = before["findings"][key], after["findings"][key]
        if before["prints"][key] == after["prints"][key]:
            continue
        rows.append(
            {
                "id": now.id,
                "rule": now.rule,
                "severity": now.severity,
                "say": now.say or now.title,
                "fingerprint": now.fingerprint,
                "was_fingerprint": was.fingerprint,
            }
        )
    return sorted(rows, key=lambda row: (SEVERITY_ORDER.get(row["severity"], 9), row["rule"]))


def _state_moves(before: dict[str, Any], after: dict[str, Any]) -> list[dict[str, Any]]:
    rows = []
    for entity_id in sorted(set(before["states"]) & set(after["states"])):
        was, now = before["states"][entity_id], after["states"][entity_id]
        if was != now:
            rows.append({"id": entity_id, "label": label(entity_id), "before": was, "after": now})
    return rows


def _matters_total(matters: Mapping[str, Any]) -> int:
    total = len(matters["values"]) + len(matters["candidates"]) + len(matters["states"])
    for group in ("launch", "profiles", "nodes", "findings"):
        total += sum(len(rows) for rows in matters[group].values())
    return total + sum(len(rows) for rows in matters["topics"].values())


_ENDPOINT_PHRASE = {
    "lost_publisher": "has no publisher now",
    "lost_subscriber": "has no subscriber now",
    "gained_publisher": "has a publisher now",
    "gained_subscriber": "has a subscriber now",
}


_FINDING_PHRASE = {
    "new": "finding new",
    "gone": "finding gone",
    "restated": "finding kept with new numbers,",
}


def _matters_lines(matters: Mapping[str, Any], limit: int, ids_below: bool = True) -> list[str]:
    room = min(max(limit, 1), 5)
    lines: list[str] = []
    if not _matters_total(matters):
        tail = "only ids and counts below" if ids_below else "and only the build identity moved"
        lines.append(f"  nothing a reader asks about moved, {tail}")
        return lines
    for row in matters["values"][:room]:
        key = "" if row["key"] == "value" else f" {row['key']}"
        lines.append(
            f"  value {_clip(row['label'], 90)}{key}, profile {profile_name(row['profile'])}"
        )
        lines.append(
            f"    {_value(row['before'])} -> {_value(row['after'])}"
            f"  edit {row['locator'] or row['where']}"
        )
    _more(lines, len(matters["values"]) - min(len(matters["values"]), room), "values moved")
    for row in matters["candidates"][:room]:
        lines.append(f"  line {_clip(row['label'], 90)}, profile {profile_name(row['profile'])}")
        stays = "" if row["effective"] is None else f", effective stays {_value(row['effective'])}"
        role = "the winning line" if row["wins"] else f"a shadowed line{stays}"
        lines.append(
            f"    {_value(row['before'])} -> {_value(row['after'])}  at {row['locator']}, {role}"
        )
    _more(
        lines,
        len(matters["candidates"]) - min(len(matters["candidates"]), room),
        "lines moved",
    )
    for group, noun in (("launch", "launch file"), ("profiles", "profile")):
        for what, rows in sorted(matters[group].items()):
            for row in rows[:room]:
                lines.append(f"  {noun} {what} {_clip(row['label'], 90)}")
            _more(lines, len(rows) - min(len(rows), room), f"{noun}s {what}")
    for what, rows in sorted(matters["nodes"].items()):
        for row in rows[:room]:
            lines.append(f"  node {what} {_clip(row['label'], 90)}")
        _more(lines, len(rows) - min(len(rows), room), f"nodes {what}")
    for what, rows in sorted(matters["topics"].items()):
        for row in rows[:room]:
            lines.append(f"  topic {_clip(row['label'], 90)} {_ENDPOINT_PHRASE[what]}")
        _more(lines, len(rows) - min(len(rows), room), f"topics that {_ENDPOINT_PHRASE[what]}")
    for what, rows in sorted(matters["findings"].items()):
        for row in rows[:room]:
            lines.append(
                f"  {_FINDING_PHRASE[what]} {row['severity']} {row['rule']}: {_clip(row['say'])}"
            )
        _more(lines, len(rows) - min(len(rows), room), f"findings {what}")
    for row in matters["states"][:room]:
        lines.append(
            f"  parameter {_clip(row['label'], 90)} is {row['after']}, it was {row['before']}"
        )
    _more(lines, len(matters["states"]) - min(len(matters["states"]), room), "parameter states")
    return lines


def _atlas_view(atlas: Atlas) -> dict[str, dict[str, Any]]:
    return {
        "entities": {key: strip_volatile(node.to_json()) for key, node in atlas.entities.items()},
        "edges": {key: strip_volatile(edge.to_json()) for key, edge in atlas.edges.items()},
    }


def _profile_view(atlas: Atlas, profile: str) -> dict[str, dict[str, Any]]:
    """One profile's slice of an atlas: the facts, the chains and the effective values that
    profile actually carries. Comparing two of these is comparing two systems."""
    entities: dict[str, Any] = {}
    for key, node in atlas.entities.items():
        facts = [fact.to_json() for fact in node.facts if fact.profile in (profile, ANY_PROFILE)]
        chain = [
            row.to_json()
            for row in sorted(node.candidates, key=Candidate.sort_key)
            if row.profile in (profile, ANY_PROFILE)
        ]
        # Absent and empty are two states. A profile that carries an effective map with
        # nothing in it is blind here, and handing it the workspace wide value shows it a
        # value it does not have.
        if profile in node.effective:
            effective = node.effective[profile]
        else:
            effective = node.effective.get(ANY_PROFILE) or {}
        if not facts and not chain and not effective:
            continue
        entities[key] = strip_volatile(
            {
                "kind": node.kind,
                "facts": facts,
                "candidates": chain,
                "effective": {name: value.to_json() for name, value in effective.items()},
                "state": node.state,
            }
        )
    edges: dict[str, Any] = {}
    for key, edge in atlas.edges.items():
        facts = [fact.to_json() for fact in edge.facts if fact.profile in (profile, ANY_PROFILE)]
        if not facts:
            continue
        edges[key] = strip_volatile({"kind": edge.kind, "facts": facts})
    return {"entities": entities, "edges": edges}


BUILD_KINDS = ("run",)
"""Entity kinds that name an observation and not a thing observed."""

BUILD_NOTE = (
    "a run id, its input digest, its root hashes and the run a fact was written by name the "
    "build. One byte anywhere renames every run, so two builds of one tree move all of them "
    "and nothing about the system"
)


def _is_build_reference(pointer: str) -> bool:
    """Whether one field names the build rather than the system.

    `rosio.mergetool` states the rule for the run a fact was written by, and the class is
    wider than the one field: the identity of a run, the digest of its inputs and the hash
    of a root say which build looked, not what it saw. A run entity is the whole class at
    once and is taken by kind rather than by pointer.
    """
    parts = pointer.split("/")
    return parts[-1] == "run" and "facts" in parts


def _names_a_build(handle: str) -> bool:
    return not handle.startswith("edge:") and ids.kind_of(handle) in BUILD_KINDS


def _build_free(payload: Any, pointer: str = "") -> Any:
    if isinstance(payload, dict):
        return {
            key: _build_free(value, f"{pointer}/{key}")
            for key, value in payload.items()
            if not _is_build_reference(f"{pointer}/{key}")
        }
    if isinstance(payload, list):
        return [_build_free(item, f"{pointer}/*") for item in payload]
    return payload


def _moved_fields(left: Mapping[str, Any], right: Mapping[str, Any]) -> list[str]:
    return sorted(name for name in set(left) | set(right) if left.get(name) != right.get(name))


def _compare(before: Mapping[str, Any], after: Mapping[str, Any]) -> dict[str, Any]:
    """Two sides, split into what moved and what only names the build that looked."""
    rows: dict[str, list[Any]] = {"added": [], "removed": [], "changed": []}
    build: dict[str, list[Any]] = {"added": [], "removed": [], "changed": []}
    for key in sorted(set(after) - set(before)):
        (build if _names_a_build(key) else rows)["added"].append(key)
    for key in sorted(set(before) - set(after)):
        (build if _names_a_build(key) else rows)["removed"].append(key)
    for key in sorted(set(before) & set(after)):
        left, right = before[key], after[key]
        if left == right:
            continue
        if _names_a_build(key):
            build["changed"].append((key, _moved_fields(left, right)))
            continue
        bare_left, bare_right = _build_free(left), _build_free(right)
        if bare_left == bare_right:
            build["changed"].append((key, _moved_fields(left, right)))
            continue
        rows["changed"].append((key, _moved_fields(bare_left, bare_right)))
    return {**rows, "build": build}


def _counts(compared: Mapping[str, Any]) -> dict[str, int]:
    return {key: len(compared[key]) for key in ("added", "removed", "changed")}


def _diff_rows(
    compared: Mapping[str, Sequence[Any]], limit: int
) -> dict[str, list[dict[str, Any]]]:
    out: dict[str, list[dict[str, Any]]] = {}
    for key in ("added", "removed"):
        rows = [{"id": item, "label": _diff_label(item)} for item in compared[key][: limit * 4]]
        out[key] = rows
    out["changed"] = [
        {"id": item, "label": _diff_label(item), "fields": fields}
        for item, fields in compared["changed"][: limit * 4]
    ]
    return out


def _diff_label(handle: str) -> str:
    if handle.startswith("edge:"):
        edge = ids.parse_edge(handle)
        return f"{edge.kind} {_clip(label(edge.src), 48)} -> {_clip(label(edge.dst), 48)}"
    return f"{ids.kind_of(handle)} {label(handle)}"


# ---------------------------------------------------------------------------
# unresolved


def unresolved(
    atlas: Atlas,
    *,
    want: str | None = None,
    limit: int = DEFAULT_LIMIT,
) -> Answer:
    """The blind spots, grouped by what was wanted and by who owns them.

    A reader of a report needs to tell a hollow extraction from a complete one, so this is
    a first class answer and not a footnote.
    """
    entries = [
        entry
        for entry in sorted(atlas.unresolved, key=lambda item: item.id)
        if want is None or entry.want == want
    ]
    by_want: dict[str, dict[str, Any]] = {}
    by_owner: dict[str, int] = {}
    for entry in entries:
        row = by_want.setdefault(
            entry.want, {"count": 0, "reasons": {}, "sentences": {}, "examples": []}
        )
        row["count"] += 1
        row["reasons"][entry.reason] = row["reasons"].get(entry.reason, 0) + 1
        said = reason_text(entry.reason, entry.want)
        row["sentences"][said] = row["sentences"].get(said, 0) + 1
        if len(row["examples"]) < limit:
            row["examples"].append(
                {
                    "owner": label(entry.owner),
                    "expr": _COLLECTOR_PREFIX.sub("", readable(entry.expr, atlas)),
                    "reason": entry.reason,
                    "where": site(entry.evidence[0]) if entry.evidence else "",
                    "resolvable_by": list(entry.resolvable_by),
                }
            )
        by_owner[label(entry.owner)] = by_owner.get(label(entry.owner), 0) + 1
    rates = (atlas.counters.get("resolution_rate") or {}) if atlas.counters else {}
    data: dict[str, Any] = {
        "total": len(entries),
        "by_want": {key: by_want[key] for key in sorted(by_want)},
        "by_owner": dict(
            sorted(by_owner.items(), key=lambda item: (-item[1], item[0]))[: limit * 2]
        ),
        "resolution_rate": dict(sorted(rates.items())),
    }
    overall = overall_rate(atlas)
    if overall is not None:
        data["overall_rate"] = overall
    lines = [f"{len(entries)} unresolved values" + (f", want {want}" if want else "")]
    if overall is not None:
        lines[0] += f", {overall:.3f} of every wanted value resolved"
    for key in sorted(by_want):
        row = by_want[key]
        rate = rates.get(key)
        tail = f", {rate:.3f} of this want resolved" if isinstance(rate, (int, float)) else ""
        lines.append(f"  {key}: {row['count']} unresolved{tail}")
        ranked = sorted(row["sentences"].items(), key=lambda item: (-item[1], item[0]))
        for name, count in ranked[:limit]:
            lines.append(f"    {count:>3}  {name}")
        if len(ranked) > limit:
            lines.append(f"    and {len(ranked) - limit} more reasons, --limit shows more")
        for example in row["examples"]:
            lines.append(f"      {example['expr']}  in {example['owner']}")
            if example["where"]:
                lines.append(f"        {example['where']}")
        if row["count"] > len(row["examples"]):
            hidden = row["count"] - len(row["examples"])
            lines.append(f"      and {hidden} more, --limit shows more")
    if data["by_owner"]:
        lines.append("  owners")
        for owner, count in list(data["by_owner"].items())[:limit]:
            lines.append(f"    {count:>3}  {owner}")
    return Answer(verb="unresolved", data=data, lines=lines)


REASONS = {
    "argv_value": "a constructor argument the caller fills in",
    "branch_dependent": "the branches of an if give it different values",
    "call_result": "the result of a call rosio does not evaluate",
    "command_substitution": "the output of a command the launch file runs",
    "environment_variable": "an environment variable at run time",
    "executable_entry_unresolved": "no source file defines this executable's entry point",
    "external_descriptor_file": "a file the node reads at run time",
    "cpp_refused_by_atlas": "a C++ source stating a value the atlas cannot hold",
    "file_after_build": "a file only colcon build puts in place",
    "file_missing": "the file it names does not exist",
    "frame_empty": "a frame id that is an empty string",
    "file_not_installed": "the install tree lacks a file the source holds",
    "local_variable": "a local variable whose value rosio does not follow",
    "loop_variable": "an element of a loop rosio cannot unroll",
    "message_field": "a field of a message that arrives at run time",
    "not_stated_in_source": "the source does not state this name and no remap resolves it",
    "package_not_found": "a package that is not installed",
    "params_file_unreadable": "a parameters file rosio could not read",
    "plugin_manifest_missing": "a plugin manifest the package exports and does not ship",
    "plugin_manifest_unreadable": "a plugin manifest that is not valid xml",
    "py_parse_error": "a python file that does not parse",
    "runtime_attribute_default": "an attribute of an object built at run time",
    "unbound_name": "a name bound nowhere rosio can see",
    "urdf_unreadable": "a robot model that is not valid xml",
    "xacro_argument_unresolved": "a xacro argument the launch walk could not evaluate",
    "xacro_failed": "xacro refused to expand the model",
}
"""One sentence per recorded reason code. The code stays in --json, where a script keys on
it, and a person reads the sentence."""

_CODE = re.compile(r"[a-z][a-z0-9]*(?:_[a-z0-9]+)+")
_COLLECTOR_PREFIX = re.compile(r"^[a-z]+\.[a-z_]+: ")


REMAPPED_WANTS = frozenset({"topic_name", "service_name", "action_name", "node_name"})
"""The wants a remap can resolve, so the only ones whose sentence may mention one."""


def reason_text(reason: str, want: str = "") -> str:
    """A recorded reason as a sentence, whether it was stored as one or as a code."""
    if not _CODE.fullmatch(reason):
        return reason
    if reason == "not_stated_in_source" and want and want not in REMAPPED_WANTS:
        return "the source does not state this name"
    return REASONS.get(reason, reason.replace("_", " "))


def overall_rate(atlas: Atlas) -> float | None:
    """Resolved over resolved plus unresolved, summed over every want, as extract prints it."""
    counters = atlas.counters or {}
    resolved = sum((counters.get("resolved") or {}).values())
    wanted = resolved + sum((counters.get("unresolved") or {}).values())
    return resolved / wanted if wanted else None


# ---------------------------------------------------------------------------
# drift


def drift(
    atlas: Atlas,
    *,
    min_severity: str | None = None,
    rule: str | None = None,
    limit: int = DEFAULT_LIMIT,
    waived: Iterable[str] = (),
) -> Answer:
    """The findings this atlas carries, filterable by severity and by rule. A finding a
    waiver matched carries the mark on its own line, by its id in `waived`."""
    floor = SEVERITY_ORDER.get(min_severity, len(SEVERITY_ORDER)) if min_severity else None
    rows = [
        item
        for item in sorted(atlas.findings, key=Finding.sort_key)
        if (rule is None or item.rule == rule)
        and (floor is None or SEVERITY_ORDER.get(item.severity, 9) <= floor)
    ]
    counts = dict.fromkeys(sorted(SEVERITY_ORDER), 0)
    by_rule: dict[str, int] = {}
    for item in rows:
        counts[item.severity] = counts.get(item.severity, 0) + 1
        by_rule[item.rule] = by_rule.get(item.rule, 0) + 1
    kept, hidden = _cap(rows, limit)
    accepted = set(waived)
    data: dict[str, Any] = {
        "total": len(rows),
        "by_severity": counts,
        "by_rule": dict(sorted(by_rule.items())),
        "findings": [
            {**_finding_row(item), **({"waived": True} if item.id in accepted else {})}
            for item in kept
        ],
    }
    summary = ", ".join(f"{count} {name}" for name, count in counts.items())
    lines = [f"{plural(len(rows), 'finding')}: {summary}"]
    if not rows:
        lines.append("  no drift reported" + (f" for rule {rule}" if rule else ""))
        return Answer(verb="drift", data=data, lines=lines)
    for row in data["findings"]:
        mark = "   waived" if row.get("waived") else ""
        lines.append(f"  {row['severity']:<5} {row['rule']}  {row['title']}{mark}")
        if row["say"]:
            lines.append(f"        {row['say']}")
        for where in row["where"][:2]:
            lines.append(f"        {where}")
        if row["fix"]:
            lines.append(f"        fix: {here(row['fix'])}")
        lines.append(f"        explain {row['fingerprint']}")
    _more(lines, hidden, "findings, --limit shows more")
    lines.append("  explain takes a fingerprint above, a finding id or a rule id")
    return Answer(verb="drift", data=data, lines=lines)


def _finding_row(item: Finding) -> dict[str, Any]:
    return {
        "id": item.id,
        "rule": item.rule,
        "severity": item.severity,
        "title": item.title,
        "say": item.spoken if item.say else "",
        "fix": item.fix,
        "profiles": [profile_name(name) for name in item.profiles],
        "where": [site(source, locator=True) for source in item.where],
        "fingerprint": item.fingerprint,
    }


__all__ = [
    "DEFAULT_LIMIT",
    "Answer",
    "diff",
    "drift",
    "explain",
    "file_label",
    "label",
    "param",
    "path",
    "profile_name",
    "query",
    "resolve",
    "site",
    "unresolved",
]
