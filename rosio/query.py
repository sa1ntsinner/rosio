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
from collections import deque
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

from rosio import ids
from rosio.errors import UsageError
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


def site(source: Source | Mapping[str, Any] | None, *, locator: bool = False) -> str:
    """One provenance row as a person reads it: the file, the line, and the key."""
    if source is None:
        return ""
    row = source.to_json() if isinstance(source, Source) else dict(source)
    text = file_label(str(row.get("file", "")))
    line = row.get("line_hint")
    if line is not None:
        text = f"{text}:{line}"
    if str(row.get("root", "src")) != "src":
        text = f"{text} [{row['root']}]"
    if locator and row.get("locator"):
        text = f"{text} {row['locator']}"
    return text


def profile_name(profile_id: str) -> str:
    return profile_id[len("profile:") :] if profile_id.startswith("profile:") else profile_id


def _profiles(items: Iterable[str]) -> str:
    names = [profile_name(item) for item in items]
    return ", ".join(names) if names else "every profile"


def _value(value: Any) -> str:
    if isinstance(value, str):
        return repr(value) if value == "" or value.strip() != value else value
    return repr(value)


def _clip(text: str, width: int = 200) -> str:
    return text if len(text) <= width else text[: width - 3] + "..."


def _cap(rows: Sequence[Any], limit: int) -> tuple[list[Any], int]:
    kept = list(rows[:limit])
    return kept, max(0, len(rows) - len(kept))


def _more(lines: list[str], hidden: int, what: str) -> None:
    if hidden:
        lines.append(f"  and {hidden} more {what}")


# ---------------------------------------------------------------------------
# selectors


def resolve(atlas: Atlas, selector: str) -> list[str]:
    """Entity ids a selector names, best match first.

    An exact id wins. Otherwise a `kind:text` selector restricts by kind and a bare word
    matches a label, so a reader can type `/scan` or `topic:/scan` or the full id.
    """
    text = (selector or "").strip()
    if not text:
        raise UsageError("a selector is needed, for example /planner_server or topic:/scan")
    if text in atlas.entities:
        return [text]
    for entity_id in sorted(atlas.entities):
        if text in atlas.entities[entity_id].aka:
            return [entity_id]
    kind = ""
    needle = text
    head, sep, tail = text.partition(":")
    if sep and tail and head in ids.ENTITY_KINDS:
        kind, needle = head, tail
    lowered = needle.lower()
    exact: list[str] = []
    partial: list[str] = []
    for entity_id in sorted(atlas.entities):
        node = atlas.entities[entity_id]
        if kind and node.kind != kind:
            continue
        name = label(entity_id).lower()
        if name == lowered or entity_id.lower().endswith(lowered):
            exact.append(entity_id)
        elif lowered in name:
            partial.append(entity_id)
    return exact or partial


def _one(atlas: Atlas, selector: str) -> tuple[str | None, list[str]]:
    matches = resolve(atlas, selector)
    return (matches[0] if len(matches) == 1 else None), matches


# ---------------------------------------------------------------------------
# explain


def explain(atlas: Atlas, selector: str, *, limit: int = DEFAULT_LIMIT) -> Answer:
    """One entity in full, or one finding in full when the selector names a finding."""
    text = (selector or "").strip()
    if text.startswith("find:") or text.startswith("sha256:"):
        return _explain_finding(atlas, text)
    entity_id, matches = _one(atlas, text)
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
        "facts": _fact_rows(node, limit),
        "effective": _effective_rows(node, limit),
        "edges": _edge_rows(atlas, entity_id, limit),
        "unresolved": _unresolved_rows(atlas, entity_id, limit),
        "findings": [item.id for item in atlas.findings if item.subject_key == entity_id],
    }
    if node.state is not None:
        data["state"] = node.state
    if node.aka:
        data["aka"] = list(node.aka)
    lines = [f"{node.kind} {label(entity_id)}"]
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
        pairs = ", ".join(f"{key}={_value(row['value'])}" for key, row in per_key.items())
        lines.append(f"  effective in {profile_name(profile)}: {_clip(pairs)}")
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
                lines.append(f"  fix: {item.fix}")
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


def _subject_label(item: Finding) -> str:
    if isinstance(item.subject, str):
        return label(item.subject)
    return f"{label(item.subject.src)} {item.subject.kind} {label(item.subject.dst)}"


def _ambiguous(verb: str, selector: str, matches: Sequence[str], atlas: Atlas) -> Answer:
    if not matches:
        return Answer(
            verb=verb,
            ok=False,
            data={"selector": selector, "matches": []},
            lines=[
                f"nothing in this atlas is named {selector!r}",
                "  try a node name, a topic name, a frame or a package, or a full id",
            ],
        )
    kept, hidden = _cap(list(matches), DEFAULT_LIMIT)
    lines = [f"{selector!r} names {len(matches)} entities, pick one"]
    for entity_id in kept:
        lines.append(f"  {atlas.entities[entity_id].kind} {label(entity_id)}")
    _more(lines, hidden, "matches")
    return Answer(
        verb=verb, ok=False, data={"selector": selector, "matches": list(matches)}, lines=lines
    )


def _fact_rows(node: Node, limit: int) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for fact in sorted(node.facts, key=lambda item: item.sort_key()):
        keys = sorted(fact.values)
        shown = ", ".join(f"{key}={_value(fact.values[key])}" for key in keys[:limit])
        if len(keys) > limit:
            shown = f"{shown}, and {len(keys) - limit} more keys"
        rows.append(
            {
                "layer": fact.layer,
                "profile": fact.profile,
                "confidence": fact.confidence,
                "values": _clip(shown) or "no values",
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
            "expr": entry.expr,
            "reason": entry.reason,
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
    param_id, matches = _param_id(atlas, node, name)
    if param_id is None:
        return _no_single_param(atlas, node, name, matches)
    entity = atlas.entities[param_id]
    owner = ids.parse_param(param_id).owner if entity.kind == "param" else ""
    wanted = _profile_id(atlas, profile)
    data: dict[str, Any] = {
        "id": param_id,
        "node": label(owner) if owner else "",
        "name": ids.parse_param(param_id).name if entity.kind == "param" else label(param_id),
        "state": entity.state,
        "profiles": {},
    }
    lines = [f"{data['name']} on {data['node'] or label(param_id)}   state {entity.state}"]
    if entity.state != "effective":
        lines.append("  no effective value: " + _state_note(str(entity.state)))
    profiles = sorted({item.profile for item in entity.candidates} | set(entity.effective))
    if wanted:
        profiles = [item for item in profiles if item == wanted]
        if not profiles:
            lines.append(f"  this parameter is not set in {profile_name(wanted)}")
    for item in profiles:
        chain = sorted(
            [row for row in entity.candidates if row.profile in (item, ANY_PROFILE)],
            key=Candidate.sort_key,
        )
        effective = entity.effective.get(item, {}).get("value")
        rows = [_candidate_row(row) for row in chain]
        data["profiles"][item] = {
            "value": effective.value if effective else None,
            "value_type": effective.value_type if effective else None,
            "winner": site(effective.source, locator=True) if effective else "",
            "chain": rows,
        }
        lines.append(f"  profile {profile_name(item)}")
        if effective is not None:
            lines.append(
                f"    effective {_value(effective.value)} ({effective.value_type})"
                f"   edit {site(effective.source)}"
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
        f"{len(matches)} parameters match {selector!r} and a param answer is about one, "
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


def _candidate_row(row: Candidate) -> dict[str, Any]:
    return {
        "rank": row.rank,
        "wins": row.wins,
        "value": row.value,
        "value_type": row.value_type,
        "mechanism": row.mechanism,
        "where": site(row.source, locator=True),
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


def _param_id(atlas: Atlas, node: str, name: str | None) -> tuple[str | None, list[str]]:
    text = (node or "").strip()
    if name is None and " " in text:
        text, name = text.split(None, 1)
    if name is None:
        matches = [
            item
            for item in resolve(atlas, text)
            if atlas.entities[item].kind in ("param", "pparam")
        ]
        return (matches[0] if len(matches) == 1 else None), matches
    owner_matches = [item for item in resolve(atlas, text) if atlas.entities[item].kind == "node"]
    wanted = ids.normalise_param_name(name)
    matches: list[str] = []
    for owner in owner_matches:
        candidate = ids.build_param(owner, wanted)
        if candidate in atlas.entities:
            matches.append(candidate)
    if not matches:
        lowered = wanted.lower()
        owners = set(owner_matches)
        for entity_id in sorted(atlas.entities):
            if atlas.entities[entity_id].kind != "param":
                continue
            owner, param_name = ids.parse_param(entity_id)
            if owner in owners and lowered in param_name.lower():
                matches.append(entity_id)
    return (matches[0] if len(matches) == 1 else None), matches


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

    matched = _matches(atlas, needles, kinds)
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

    kept, hidden = _cap(matched, limit)
    scope = set(kept)
    rows: list[dict[str, str]] = []
    for entity_id in kept:
        for edge in _out(atlas, entity_id) + _into(atlas, entity_id):
            if relations and edge.kind not in relations:
                continue
            if edge.src not in scope and edge.dst not in scope:
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
    lines = [f"{len(matched)} entities match {', '.join(needles) or ', '.join(sorted(kinds))}"]
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
        lines.append(
            f"  no {', '.join(sorted(relations))} edge touches these in this atlas, "
            "which is an answer and not a failure"
        )
    lines.append("  explain any of these for the facts and the provenance")
    return Answer(verb="query", data=data, lines=lines)


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
    hop is stated, so nothing here claims a flow that does not exist.
    """
    start, start_matches = _one(atlas, source)
    if start is None:
        return _ambiguous("path", source, start_matches, atlas)
    end, end_matches = _one(atlas, target)
    if end is None:
        return _ambiguous("path", target, end_matches, atlas)
    if start == end:
        return Answer(
            verb="path",
            data={"from": start, "to": end, "hops": [], "found": True},
            lines=[f"{label(start)} is the entity asked for on both ends, no hops"],
        )
    hops = _shortest(atlas, start, end, max_hops)
    data: dict[str, Any] = {
        "from": start,
        "to": end,
        "hops": hops or [],
        "found": bool(hops),
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
    lines = [f"{len(hops)} hops from {label(start)} to {label(end)}"]
    lines.append(f"  {label(start)}")
    for hop in hops:
        arrow = "->" if hop["forward"] else "<-"
        lines.append(f"  {arrow} {hop['kind']} {label(hop['to'])}")
    return Answer(verb="path", data=data, lines=lines)


def _shortest(atlas: Atlas, start: str, end: str, max_hops: int) -> list[dict[str, Any]]:
    if start == end:
        return []
    seen = {start}
    queue: deque[tuple[str, list[dict[str, Any]]]] = deque([(start, [])])
    while queue:
        current, trail = queue.popleft()
        if len(trail) >= max_hops:
            continue
        for edge, forward in _neighbours(atlas, current):
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
                lines.append(f"    {key[:3]} {_clip(entry['label'], 120)}{fields}")
            _more(lines, counts[key] - len(kept), f"{key} {what}")
    lines.extend(_build_lines(build))
    return Answer(verb="diff", data=data, lines=lines)


def _nothing_moved(compared: Mapping[str, Any]) -> bool:
    return not any(_counts(compared).values()) and not any(_counts(compared["build"]).values())


def _build_lines(build: Mapping[str, Any]) -> list[str]:
    """The build identity, apart and last, so it answers its own question and no other."""
    counts = [(what, build[what]) for what in ("entities", "edges")]
    if not any(sum(row.values()) for _what, row in counts):
        return ["  build identity: one build wrote both sides, nothing here moved"]
    lines = ["  build identity, which is not a statement about the system"]
    for what, row in counts:
        if sum(row.values()):
            lines.append(
                f"    {what}: {row['added']} added, {row['removed']} removed, "
                f"{row['changed']} changed"
            )
    lines.append(f"    {BUILD_NOTE}")
    return lines


def _side(atlas: Atlas, profile: str | None = None) -> dict[str, Any]:
    """One side of a comparison, in the terms a reader thinks in.

    In profile mode the profile is taken out of every key, because the whole point of
    comparing two profiles is to line up the same fact under two different names.
    """
    wanted = (profile, ANY_PROFILE) if profile else None
    values: dict[tuple, dict[str, Any]] = {}
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
    for item in atlas.findings:
        if wanted and item.profiles and not any(name in wanted for name in item.profiles):
            continue
        findings[_finding_key(item)] = item
    return {
        "values": values,
        "states": states,
        "nodes": nodes,
        "launch": launch,
        "profiles": profiles,
        "kinds": kinds,
        "endpoints": endpoints,
        "findings": findings,
    }


def _matters(before: dict[str, Any], after: dict[str, Any], limit: int) -> dict[str, Any]:
    """The change a person asked about, grouped, before any id or count is printed."""
    room = max(limit, 1) * 4
    return {
        "values": _value_moves(before, after)[:room],
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
        if _finding_print(was) == _finding_print(now):
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
    total = len(matters["values"]) + len(matters["states"])
    for group in ("launch", "profiles", "nodes", "findings"):
        total += sum(len(rows) for rows in matters[group].values())
    return total + sum(len(rows) for rows in matters["topics"].values())


_ENDPOINT_PHRASE = {
    "lost_publisher": "has no publisher now",
    "lost_subscriber": "has no subscriber now",
    "gained_publisher": "has a publisher now",
    "gained_subscriber": "has a subscriber now",
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
            lines.append(f"  finding {what} {row['severity']} {row['rule']}: {_clip(row['say'])}")
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
        row = by_want.setdefault(entry.want, {"count": 0, "reasons": {}, "examples": []})
        row["count"] += 1
        row["reasons"][entry.reason] = row["reasons"].get(entry.reason, 0) + 1
        if len(row["examples"]) < limit:
            row["examples"].append(
                {
                    "owner": label(entry.owner),
                    "expr": entry.expr,
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
    lines = [f"{len(entries)} unresolved values" + (f", want {want}" if want else "")]
    for key in sorted(by_want):
        row = by_want[key]
        ranked = sorted(row["reasons"].items(), key=lambda item: (-item[1], item[0]))
        reasons = ", ".join(f"{_clip(name, 60)} {count}" for name, count in ranked[:3])
        if len(ranked) > 3:
            reasons = f"{reasons}, and {len(ranked) - 3} more reasons"
        rate = rates.get(key)
        tail = f"   resolution rate {rate}" if rate is not None else ""
        lines.append(f"  {key} {row['count']}: {reasons}{tail}")
        for example in row["examples"][:3]:
            lines.append(f"      {example['expr']}  in {example['owner']}")
            if example["where"]:
                lines.append(f"        {example['where']}")
    if data["by_owner"]:
        lines.append("  owners")
        for owner, count in list(data["by_owner"].items())[:limit]:
            lines.append(f"    {count:>3}  {owner}")
    return Answer(verb="unresolved", data=data, lines=lines)


# ---------------------------------------------------------------------------
# drift


def drift(
    atlas: Atlas,
    *,
    min_severity: str | None = None,
    rule: str | None = None,
    limit: int = DEFAULT_LIMIT,
) -> Answer:
    """The findings this atlas carries, filterable by severity and by rule."""
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
    data: dict[str, Any] = {
        "total": len(rows),
        "by_severity": counts,
        "by_rule": dict(sorted(by_rule.items())),
        "findings": [_finding_row(item) for item in kept],
    }
    summary = ", ".join(f"{count} {name}" for name, count in counts.items())
    lines = [f"{len(rows)} findings: {summary}"]
    if not rows:
        lines.append(
            "  no drift reported"
            + (f" for rule {rule}" if rule else "")
            + ". run the drift engine over this atlas if that is unexpected"
        )
        return Answer(verb="drift", data=data, lines=lines)
    for row in data["findings"]:
        lines.append(f"  {row['severity']:<5} {row['rule']}  {row['title']}")
        if row["say"]:
            lines.append(f"        {row['say']}")
        for where in row["where"][:2]:
            lines.append(f"        {where}")
    _more(lines, hidden, "findings")
    lines.append("  explain a finding id for the full evidence")
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
