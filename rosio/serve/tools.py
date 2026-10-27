"""The tool set an assistant calls instead of reading the atlas.

A full atlas on a real stack is megabytes, so no model reads it. Every function here takes
one loaded atlas and returns one plain text answer, capped so it fits a small context
window, carrying the file and line a reader needs before editing anything. There is no
language model in any of it, nothing here reads a file except the atlas itself, and a
synthetic id never reaches the text.

The consumer is assumed to know nothing about this project, so every answer states what it
is, and every answer that hit a cap says how to narrow the next call.
"""

from __future__ import annotations

import json
import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field
from pathlib import Path

from rosio import ids, schema
from rosio import query as slices
from rosio.errors import ContractError, UsageError
from rosio.model import ANY_PROFILE, SCHEMA_MINOR, SEVERITY_ORDER, Atlas, Edge, Finding, Node
from rosio.render import plural

ATLAS_NAME = "atlas.json"
DRIFT_CAPABILITY = "drift.findings"

MAX_BYTES = 4000
"""Ceiling on one answer. A model with a 4k window has to hold the question too."""

DEFAULT_LIMIT = 8
WIDE_LIMIT = 5
"""Rows that carry a whole sentence each, so fewer of them fit one answer."""

MAX_LIMIT = 50


# ---------------------------------------------------------------------------
# the atlas, loaded once


class AtlasSource:
    """One atlas, read from disk on the first question and answered from memory after."""

    def __init__(self, path: str | Path, *, confined: bool = False) -> None:
        self.path = Path(path).expanduser()
        self._atlas: Atlas | None = None
        self.confined = confined
        """Whether a tool that reads the file system keeps to the served workspace, which
        is how the http transport serves a caller who is not the person who started it."""

    def atlas(self) -> Atlas:
        if self._atlas is None:
            self._atlas = load(self.path)
        return self._atlas

    @property
    def loaded(self) -> bool:
        return self._atlas is not None

    def workspace(self) -> Path:
        """The directory above the one that holds the atlas, where extract wrote it."""
        target = self.path / ATLAS_NAME if self.path.is_dir() else self.path
        return target.absolute().parent.parent.resolve()

    @property
    def present(self) -> bool:
        """Whether there is a file to read. A server starts without one, and every tool that
        needs it says how to build it until it is there."""
        target = self.path / ATLAS_NAME if self.path.is_dir() else self.path
        return target.is_file()


def load(path: str | Path) -> Atlas:
    """Read one atlas.json, validated, with a message a caller can act on.

    Every answer this module gives is a statement about a workspace, so the document is
    checked before it is answered from. A hand edited or truncated file otherwise reaches a
    reader as a confident sentence about layers or counters that the file does not support.

    The one document that is loaded without being held to this build's schema is one that
    states a newer minor: its unknown keys and unknown enum members are exactly what a
    newer minor is allowed to add, and section 9a says a reader tolerates them. Wrong types
    on the closed fields are refused by the loader itself, in both cases.
    """
    target = Path(path).expanduser()
    if target.is_dir():
        target = target / ATLAS_NAME
    if not target.is_file():
        raise UsageError(
            f"no atlas at {target.absolute()} yet. build one with: rosio extract <workspace> "
            f"--out {target.parent.absolute()}, then ask again"
        )
    try:
        payload = json.loads(target.read_text(encoding="utf-8"))
    except ValueError as exc:
        raise UsageError(f"{target} does not parse as json: {exc}") from exc
    try:
        atlas = Atlas.from_json(payload)
    except ContractError as exc:
        raise ContractError(
            f"{exc}. extract this workspace again with the installed rosio, "
            "or upgrade rosio to the build that wrote the file"
        ) from exc
    if not _newer_minor(atlas.schema_version):
        problems = schema.validate(payload)
        if problems:
            raise ContractError(
                f"{target.name} does not satisfy the atlas schema, so it is not answered "
                f"from: {_named(problems, top=3, more='violations')}. "
                "extract this workspace again with: rosio extract <workspace> --out <dir>"
            )
    return atlas


def _newer_minor(version: str) -> bool:
    _, _, tail = str(version).partition(".")
    try:
        return int(tail or "0") > int(SCHEMA_MINOR)
    except ValueError:
        return False


# ---------------------------------------------------------------------------
# shaping


def _fit(lines: Sequence[str], hint: str, cap: int = MAX_BYTES) -> str:
    """One answer, capped, saying so and saying how to ask for less."""
    text = "\n".join(lines)
    if len(text.encode("utf-8")) <= cap:
        return text
    kept: list[str] = []
    size = 0
    room = cap - len(hint.encode("utf-8")) - 80
    for line in lines:
        size += len(line.encode("utf-8")) + 1
        if size > room:
            break
        kept.append(line)
    kept.append(f"truncated after {len(kept)} of {len(lines)} lines, {hint}")
    return "\n".join(kept)


def _limit(value: int | None, fallback: int = DEFAULT_LIMIT) -> int:
    if not value:
        return fallback
    return max(1, min(int(value), MAX_LIMIT))


def _text(value: str | None) -> str:
    return (value or "").strip()


def _ref(entity_id: str) -> str:
    """The id a caller may quote back, empty when the identity is synthetic."""
    return "" if ids.parse(entity_id).synthetic else entity_id


def _profile_id(atlas: Atlas, profile: str | None) -> str:
    wanted = _text(profile)
    if not wanted:
        return ""
    full = wanted if wanted.startswith("profile:") else ids.build_profile(wanted)
    if full not in atlas.entities:
        known = _named(
            [slices.profile_name(item.id) for item in atlas.by_kind("profile")],
            more="more profiles",
        )
        raise UsageError(f"no profile named {wanted}, this atlas holds: {known}")
    return full


def _entity(atlas: Atlas, selector: str) -> tuple[str, list[str]]:
    """One entity id and a note when it was found by substring, or an empty id and the lines
    that say why. The command line resolves through the same `match`."""
    found = slices.match(atlas, selector)
    if len(found.ids) == 1:
        note = (
            [slices.substring_note(selector, slices.label(found.ids[0]))]
            if found.how == "substring"
            else []
        )
        return found.ids[0], note
    if not found.ids:
        return "", slices.not_found_lines(atlas, selector)
    return "", slices.choice_lines(atlas, selector, found.ids, shell=False)


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


def _counted(rows: Iterable[str]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for name in rows:
        counts[name] = counts.get(name, 0) + 1
    return counts


def _ranked(counts: dict[str, int], top: int = 8, noun: str = "kinds") -> str:
    order = sorted(counts.items(), key=lambda item: (-item[1], item[0]))
    shown = ", ".join(f"{_clip(name, 90)} {count}" for name, count in order[:top])
    if len(order) > top:
        shown = f"{shown}, and {len(order) - top} more {noun}"
    return shown or "none"


def _named(items: Sequence[str], top: int = DEFAULT_LIMIT, more: str = "more") -> str:
    """A list of names, capped. Every answer is capped, and so is every refusal: an atlas
    with a thousand profiles once put seven kilobytes into a message promising four."""
    shown = ", ".join(_clip(str(item), 90) for item in items[:top])
    if len(items) > top:
        shown = f"{shown}, and {len(items) - top} {more}"
    return shown or "none"


def _clip(text: str, width: int) -> str:
    return slices._clip(text, width)


def _subject(item: Finding) -> str:
    if isinstance(item.subject, str):
        return slices.label(item.subject)
    return f"{slices.label(item.subject.src)} {item.subject.kind} {slices.label(item.subject.dst)}"


# ---------------------------------------------------------------------------
# overview, the first call


def overview(atlas: Atlas) -> str:
    """Profiles, layers, counts, coverage and the worst findings, in one screen."""
    counters = atlas.counters or {}
    kinds = dict(counters.get("entities_by_kind") or {})
    resolved = dict(counters.get("resolved") or {})
    blind = dict(counters.get("unresolved") or {})
    total_resolved = sum(resolved.values())
    total_wanted = total_resolved + sum(blind.values())
    rate = (total_resolved / total_wanted) if total_wanted else 1.0
    present = [name for name, status in sorted(atlas.layers.items()) if status.present]
    profiles = [slices.profile_name(item.id) for item in atlas.by_kind("profile")]
    findings = sorted(atlas.findings, key=Finding.sort_key)
    counts = _counted(item.severity for item in findings)

    lines = [
        f"rosio atlas, schema {atlas.schema_version}, "
        f"{plural(len(atlas.entities), 'entity')} and {plural(len(atlas.edges), 'edge')}",
        f"layers present: {', '.join(present) or 'none'}"
        f"   capabilities: {', '.join(atlas.capabilities) or 'none'}",
        f"profiles ({len(profiles)}): {_named(profiles, top=10, more='more profiles')}",
        f"entities: {_ranked(kinds, top=10)}",
        f"coverage: {sum(blind.values())} values unresolved of {total_wanted} wanted, "
        f"rate {rate:.3f}",
    ]
    weak = sorted((counters.get("resolution_rate") or {}).items(), key=lambda item: item[1])[:3]
    if weak:
        lines.append("  weakest: " + ", ".join(f"{name} {value:.2f}" for name, value in weak))
    if DRIFT_CAPABILITY not in atlas.capabilities:
        lines.append("drift: the rules never ran over this atlas, so it carries no findings")
    else:
        summary = ", ".join(f"{count} {name}" for name, count in sorted(counts.items())) or "none"
        lines.append(f"drift: {plural(len(findings), 'finding')}, {summary}")
        for item in findings[:4]:
            where = slices.site(item.where[0]) if item.where else ""
            lines.append(f"  {item.severity:<5} {item.rule}  {_clip(item.title, 90)}   {where}")
    lines.append(
        "next: param_chain for one parameter value, drift for the findings, "
        "explain for one entity, query for plain words"
    )
    return _fit(lines, "this answer is already the summary")


# ---------------------------------------------------------------------------
# param_chain, the highest value question in the product


def param_chain(
    atlas: Atlas,
    node: str,
    name: str | None = None,
    *,
    profile: str | None = None,
) -> str:
    """The effective value, its type, the line that set it and every line it shadows."""
    owner = _text(node)
    if not owner:
        raise UsageError("param_chain needs a node, for example /controller_server")
    wanted = _profile_id(atlas, profile)
    answer = slices.param(atlas, owner, _text(name) or None, profile=wanted or None)
    if not answer.ok:
        return _fit(
            answer.lines,
            "name one node and one parameter, for example /controller_server FollowPath.plugin",
        )
    hint = "narrow with profile=<name>" if not wanted else "this parameter has a long chain"
    return _fit(answer.lines, hint)


# ---------------------------------------------------------------------------
# drift


def drift(
    atlas: Atlas,
    *,
    min_severity: str | None = None,
    rule: str | None = None,
    profile: str | None = None,
    limit: int | None = None,
) -> str:
    """The findings, each with the sentence to quote and the line to edit."""
    if DRIFT_CAPABILITY not in atlas.capabilities:
        return (
            "this atlas carries no findings because the drift rules never ran over it. "
            f"it declares {', '.join(atlas.capabilities) or 'nothing'}, and "
            f"{DRIFT_CAPABILITY} is what says they ran. extract the workspace again"
        )
    floor = None
    if _text(min_severity):
        floor = SEVERITY_ORDER.get(_text(min_severity))
        if floor is None:
            raise UsageError(
                f"no severity named {min_severity!r}, use one of: {', '.join(SEVERITY_ORDER)}"
            )
    wanted_rule = _text(rule)
    wanted_profile = _profile_id(atlas, profile)
    rows = [
        item
        for item in sorted(atlas.findings, key=Finding.sort_key)
        if (floor is None or SEVERITY_ORDER.get(item.severity, 9) <= floor)
        and (not wanted_rule or item.rule == wanted_rule)
        and (not wanted_profile or not item.profiles or wanted_profile in item.profiles)
    ]
    fired = _counted(item.rule for item in atlas.findings)
    if wanted_rule and wanted_rule not in fired:
        return _fit(
            [
                f"no finding of rule {wanted_rule} in this atlas",
                f"  rules that fired: {_ranked(fired, top=12, noun='rules')}",
            ],
            "call drift with no only for all of them",
        )
    scope = [
        f"severity {min_severity} or worse" if floor is not None else "",
        f"only {wanted_rule}" if wanted_rule else "",
        f"profile {slices.profile_name(wanted_profile)}" if wanted_profile else "",
    ]
    filters = ", ".join(part for part in scope if part)
    counts = _counted(item.severity for item in rows)
    summary = ", ".join(f"{count} {name}" for name, count in sorted(counts.items())) or "none"
    head = f"{plural(len(rows), 'finding')} of {len(atlas.findings)}: {summary}"
    lines = [f"{head}   filters: {filters}" if filters else head]
    kept = rows[: _limit(limit, WIDE_LIMIT)]
    for item in kept:
        lines.append(f"{item.severity:<5} {item.rule}  on {_subject(item)}")
        lines.append(f"      say: {item.spoken}")
        for row in item.where[:2]:
            lines.append(f"      at: {slices.site(row, locator=True)}")
        if item.fix:
            lines.append(f"      fix: {item.fix}")
        lines.append(f"      explain: {item.fingerprint}")
    if len(rows) > len(kept):
        lines.append(f"  and {len(rows) - len(kept)} more, raise limit or filter by only")
    if rows:
        lines.append("call explain with a fingerprint above for the full evidence")
    elif filters:
        lines.append("nothing matches those filters, which is an answer and not a failure")
    return _fit(lines, "narrow with min_severity=error, only=<rule> or a smaller limit")


# ---------------------------------------------------------------------------
# explain


def explain(atlas: Atlas, selector: str, *, limit: int | None = None) -> str:
    """One entity, one finding or the findings of one rule in full: layers, facts, effective
    values and provenance."""
    selector = _text(selector)
    if not selector:
        raise UsageError("explain needs a selector, for example /controller_server or /scan")
    if selector.startswith(("find:", "sha256:")) or slices.is_rule(selector):
        return _fit(
            slices.explain(atlas, selector, limit=_limit(limit)).lines,
            "pass a fingerprint for one finding",
        )
    entity_id, note = _entity(atlas, selector)
    if not entity_id:
        return _fit(note, "call query with the plain words instead")
    rows = _limit(limit)
    data = slices.explain(atlas, entity_id, limit=rows).data
    node = atlas.entities[entity_id]
    lines = [f"{data['kind']} {data['label']}", *note]
    handle = _ref(entity_id)
    if handle:
        lines.append(f"  id {handle}")
    lines.append(
        f"  layers {', '.join(data['layers']) or 'none'}"
        f"   profiles {', '.join(slices.profile_name(item) for item in data['profiles']) or 'any'}"
        f"   agreement {data['agreement']}"
    )
    if node.state is not None:
        lines.append(f"  state {node.state}")
    if data["conflicts"]:
        lines.append(f"  layers disagree about: {', '.join(data['conflicts'])}")
    for row in data["facts"][:rows]:
        lines.append(
            f"  {row['layer']} in {slices.profile_name(row['profile'])} "
            f"({row['confidence']}): {row['values']}"
        )
        for where in row["where"][:2]:
            lines.append(f"      {where}")
    states_more = len(data["layers"]) > 1 or node.kind in ("param", "pparam")
    for profile, per_key in data["effective"].items():
        if not per_key or not states_more:
            continue
        pairs = ", ".join(
            f"{key}={slices._stated(row['value'])}"
            for key, row in list(per_key.items())[:rows]
            if slices._said(row["value"])
        )
        pairs = slices.readable(pairs, atlas)
        lines.append(f"  effective in {slices.profile_name(profile)}: {_clip(pairs, 220)}")
        for key, row in list(per_key.items())[:rows]:
            if row["where"]:
                lines.append(f"      {key} ({row['value_type']}) from {row['where']}")
    for way, arrow in (("out", "->"), ("in", "<-")):
        counts = data["edges"][f"{way}_by_kind"]
        if not counts:
            continue
        lines.append(f"  {way} edges: {_ranked(counts)}")
        for row in data["edges"][way]:
            lines.append(f"    {arrow} {row['kind']} {row['label']}")
    for row in data["unresolved"][:3]:
        lines.append(f"  unresolved {row['want']}: {row['expr']} ({row['reason']}) {row['where']}")
    if data["findings"]:
        lines.append(f"  {plural(len(data['findings']), 'finding')}, call drift for the sentences")
    if node.kind in ("param", "pparam"):
        lines.append("  call param_chain for the candidate chain and the line to edit")
    lines.append("  call neighbours for the full fan in and fan out")
    return _fit(lines, "call neighbours with a kind, or explain a neighbour instead")


# ---------------------------------------------------------------------------
# query


def query(atlas: Atlas, question: str, *, limit: int | None = None) -> str:
    """A scoped subgraph for a question in plain words, matched by name and kind only."""
    if not _text(question):
        raise UsageError('query needs a question, for example "which nodes publish cmd_vel"')
    answer = slices.query(atlas, question, limit=_limit(limit))
    lines = list(answer.lines)
    if answer.ok and not answer.data.get("kinds") and not answer.data.get("relations"):
        lines.append(
            "  these matched the words by name only. name a kind (node, topic, parameter, "
            "frame) or a relation (publishes, subscribes) to mean something narrower"
        )
    return _fit(lines, "ask for one name at a time, or lower limit")


# ---------------------------------------------------------------------------
# path


def path(atlas: Atlas, source: str, target: str) -> str:
    """The shortest path between two entities, with the kind and direction of every hop."""
    if not _text(source) or not _text(target):
        raise UsageError(
            "path needs a source and a target, for example /controller_server and /cmd_vel"
        )
    answer = slices.path(atlas, _text(source), _text(target))
    return _fit(answer.lines, "name both ends exactly, an id is accepted")


# ---------------------------------------------------------------------------
# neighbours


def neighbours(
    atlas: Atlas,
    selector: str,
    *,
    kind: str | None = None,
    direction: str | None = None,
    limit: int | None = None,
) -> str:
    """Fan in and fan out of one entity, counted by edge kind, without scanning."""
    entity_id, complaint = _entity(atlas, _text(selector))
    if not entity_id:
        return _fit(complaint, "call query with the plain words instead")
    way = _text(direction).lower() or "both"
    if way not in ("out", "in", "both"):
        raise UsageError(f"no direction named {direction!r}, use out, in or both")
    wanted = _text(kind)
    rows = _limit(limit)
    out_edges = _out(atlas, entity_id)
    in_edges = _into(atlas, entity_id)
    lines = [
        f"{atlas.entities[entity_id].kind} {slices.label(entity_id)}: "
        f"{plural(len(out_edges), 'edge')} out, {plural(len(in_edges), 'edge')} in"
    ]
    kinds = _counted(edge.kind for edge in out_edges + in_edges)
    if wanted and wanted not in kinds:
        lines.append(f"  no {wanted} edge touches it. kinds here: {_ranked(kinds, top=12)}")
        return _fit(lines, "pick one of the kinds above")
    for name, edges, arrow in (("out", out_edges, "->"), ("in", in_edges, "<-")):
        if way not in (name, "both"):
            continue
        # counted by neighbour and not by edge, because one relation can hold at several
        # launch sites and a reader wants the entity list to match the number above it
        pairs = sorted({(edge.kind, edge.dst if name == "out" else edge.src) for edge in edges})
        if wanted:
            pairs = [pair for pair in pairs if pair[0] == wanted]
        if not pairs:
            lines.append(f"  {name}: none")
            continue
        lines.append(f"  {name}: {_ranked(_counted(kind for kind, _other in pairs), top=12)}")
        for edge_kind, other in pairs[:rows]:
            lines.append(f"    {arrow} {edge_kind} {slices.label(other)}")
        if len(pairs) > rows:
            lines.append(f"    and {len(pairs) - rows} more, raise limit or pass a kind")
    return _fit(lines, "pass kind=<edge kind> and direction=out or in")


# ---------------------------------------------------------------------------
# graph

LINK_KINDS = ("publishes", "subscribes", "serves", "calls")
CHANNEL_KINDS = ("topic", "service", "action")
QOS_KEYS = ("preset", "reliability", "durability", "history", "depth")
_TOPIC_SIDES = (
    ("publishes", "published by", "no publisher"),
    ("subscribes", "subscribed by", "no subscriber"),
)
_SERVER_SIDES = (("serves", "served by", "no server"), ("calls", "called by", "no client"))
_SIDES = {"topic": _TOPIC_SIDES, "service": _SERVER_SIDES, "action": _SERVER_SIDES}

LIFECYCLE_ENDPOINTS = {
    "change_state": "serves",
    "get_state": "serves",
    "get_available_states": "serves",
    "get_available_transitions": "serves",
    "get_transition_graph": "serves",
    "transition_event": "publishes",
}
"""What rcl_lifecycle creates under every lifecycle node, with no call in its source. Checked
in the string table of `/opt/ros/jazzy/lib/librcl_lifecycle.so`, which holds `~/change_state`,
`~/get_state`, `~/get_available_states`, `~/get_available_transitions`,
`~/get_transition_graph` and `~/transition_event`."""


def _in_profile(item: Node | Edge, wanted: str) -> bool:
    if not wanted:
        return True
    return wanted in item.profiles or any(
        fact.profile in (wanted, ANY_PROFILE) for fact in item.facts
    )


def _qos(edge: Edge, wanted: str) -> str:
    """The QoS the atlas states for one endpoint, never a default. Empty when it states none."""
    if wanted:
        stated = edge.effective.get(wanted) or edge.effective.get(ANY_PROFILE) or {}
    else:
        stated = edge.effective[min(edge.effective)] if edge.effective else {}
    parts = []
    for key in QOS_KEYS:
        row = stated.get(f"qos.{key}")
        if row is None or row.value is None:
            continue
        if key == "preset":
            parts.append(f"{row.value} preset")
        elif key == "depth":
            parts.append(f"depth {row.value}")
        else:
            parts.append(str(row.value))
    if edge.conflicts:
        parts.append(f"layers differ on {', '.join(edge.conflicts)}")
    return ", ".join(parts)


def _wrapper(edge: Edge, wanted: str) -> bool:
    return edge.attrs.get("via_known_wrapper") is True or any(
        fact.values.get("via_known_wrapper") is True
        for fact in edge.facts
        if not wanted or fact.profile in (wanted, ANY_PROFILE)
    )


def _channel_type(atlas: Atlas, channel: str, wanted: str) -> str:
    names = {
        "/".join(ids.parse(edge.dst).segments)
        for edge in _out(atlas, channel)
        if edge.kind == "uses_type" and _in_profile(edge, wanted)
    }
    return next(iter(names)) if len(names) == 1 else ""


def _rules(atlas: Atlas, channel: str, wanted: str) -> list[str]:
    """Rule ids of the findings about one channel or about an endpoint edge on it."""
    rules = set()
    for item in atlas.findings:
        if wanted and item.profiles and not {wanted, ANY_PROFILE} & set(item.profiles):
            continue
        subject = item.subject
        if subject == channel or (
            not isinstance(subject, str) and subject.kind in LINK_KINDS and subject.dst == channel
        ):
            rules.add(item.rule)
    return sorted(rules)


@dataclass
class GraphSelection:
    """What graph shows: channels in reading order and the endpoint edges ending on each.

    `complaint` is set instead when the focus named nothing usable, with `hint` beside it.
    """

    wanted: str
    hops: int
    head: str = ""
    focus_id: str = ""
    complaint: list[str] = field(default_factory=list)
    hint: str = ""
    channels: list[str] = field(default_factory=list)
    rules: dict[str, list[str]] = field(default_factory=dict)
    by_channel: dict[str, list[Edge]] = field(default_factory=dict)
    hidden: int = 0
    beyond: int = 0
    nodes: int = 0
    endpoints: int = 0
    lifecycle: dict[str, tuple[str, str]] = field(default_factory=dict)


def select_graph(
    atlas: Atlas,
    *,
    profile: str | None = None,
    focus: str | None = None,
    depth: int | None = None,
    wrappers: bool = False,
) -> GraphSelection:
    """The selection behind `graph` and `diagram`, made once for both.

    The same selection as the editor's system graph: endpoint edges between nodes and
    channels of one profile, cut to the neighbourhood of a focus within depth hops.
    """
    wanted = _profile_id(atlas, profile)
    scope = {
        entity_id
        for entity_id, entity in atlas.entities.items()
        if (entity.kind == "node" or entity.kind in CHANNEL_KINDS) and _in_profile(entity, wanted)
    }
    links = [
        edge
        for edge in atlas.edges.values()
        if edge.kind in LINK_KINDS
        and edge.src in scope
        and edge.dst in scope
        and _in_profile(edge, wanted)
    ]
    where = f"profile {slices.profile_name(wanted)}" if wanted else "every profile"
    hops = 2 if depth is None else max(1, int(depth))
    selector = _text(focus)
    focus_id = ""
    beyond = 0
    if selector:
        focus_id, complaint = _entity(atlas, selector)
        if not focus_id:
            return GraphSelection(
                wanted, hops, complaint=complaint, hint="call query with the plain words instead"
            )
        kind = atlas.entities[focus_id].kind
        if kind != "node" and kind not in CHANNEL_KINDS:
            return GraphSelection(
                wanted,
                hops,
                complaint=[
                    f"{kind} {slices.label(focus_id)} is not a node, topic, service or action",
                    "  call neighbours for what it is connected to",
                ],
                hint="focus on a node or a channel",
            )
        reached = {focus_id} & scope
        frontier = set(reached)
        for _ in range(hops):
            step = set()
            for edge in links:
                for here, there in ((edge.src, edge.dst), (edge.dst, edge.src)):
                    if here in frontier and there not in reached:
                        step.add(there)
            if not step:
                break
            reached |= step
            frontier = step
        beyond = sum(1 for edge in links if (edge.src in reached) != (edge.dst in reached))
        scope = reached
        links = [edge for edge in links if edge.src in scope and edge.dst in scope]

    shown = links if wrappers else [edge for edge in links if not _wrapper(edge, wanted)]
    hidden = len(links) - len(shown)
    by_channel: dict[str, list[Edge]] = {}
    for edge in shown:
        by_channel.setdefault(edge.dst, []).append(edge)
    library_only = {edge.dst for edge in links} - set(by_channel)
    channels = sorted(
        (entity_id for entity_id in scope if atlas.entities[entity_id].kind in CHANNEL_KINDS),
        key=slices.label,
    )
    channels = [entity_id for entity_id in channels if entity_id not in library_only]
    rules = {entity_id: _rules(atlas, entity_id, wanted) for entity_id in channels}
    channels.sort(key=lambda entity_id: not rules[entity_id])

    lifecycle = {
        channel: found
        for channel in channels
        if (found := _lifecycle_side(atlas, channel, by_channel.get(channel, []), wanted))
    }
    nodes = sum(1 for entity_id in scope if atlas.entities[entity_id].kind == "node")
    counts = _counted(atlas.entities[entity_id].kind for entity_id in channels)
    tally = ", ".join(plural(counts.get(kind, 0), kind) for kind in CHANNEL_KINDS)
    head = f"{plural(nodes, 'node')}, {tally}, {plural(len(shown), 'endpoint')} in {where}"
    if focus_id:
        around = f"{atlas.entities[focus_id].kind} {slices.label(focus_id)}"
        head = f"around {around} within {plural(hops, 'hop')}: {head}"
        if focus_id not in scope:
            head = f"{around} has no endpoint in {where}"
    return GraphSelection(
        wanted,
        hops,
        head=head,
        focus_id=focus_id,
        channels=channels,
        rules=rules,
        by_channel=by_channel,
        hidden=hidden,
        beyond=beyond,
        nodes=nodes,
        endpoints=len(shown) + hidden,
        lifecycle=lifecycle,
    )


def _lifecycle_side(
    atlas: Atlas, channel: str, edges: Sequence[Edge], wanted: str
) -> tuple[str, str] | None:
    """The edge kind and the lifecycle node behind an endpoint rcl_lifecycle creates, when
    the atlas states that side of the channel nowhere else."""
    parent, _, leaf = slices.label(channel).rpartition("/")
    kind = LIFECYCLE_ENDPOINTS.get(leaf)
    if not kind or not parent or any(edge.kind == kind for edge in edges):
        return None
    if (kind == "serves") != (atlas.entities[channel].kind == "service"):
        return None
    try:
        owner = ids.build_node(parent)
    except ContractError:
        return None
    node = atlas.entities.get(owner)
    if node is None or not any(
        fact.values.get("form") == "lifecycle_node"
        for fact in node.facts
        if not wanted or fact.profile in (wanted, ANY_PROFILE)
    ):
        return None
    return kind, owner


def _pass(cli: bool, name: str, value: str = "") -> str:
    """One argument as the caller spells it: `--focus NODE` on the command line and
    `focus=<node>` in MCP. Without a value, the bare name in either spelling."""
    if not value:
        return f"--{name}" if cli else name
    if not cli:
        return f"{name}={value}"
    return f"--{name}" if value == "true" else f"--{name} {value.strip('<>').upper()}"


def _why_empty(atlas: Atlas, chosen: GraphSelection, cli: bool) -> str:
    """One line on why a graph holds no endpoint, from what the atlas states about itself."""
    raised = set(slices.raised_profiles(atlas))
    where = slices.profile_name(chosen.wanted) if chosen.wanted else ""
    if where and where in raised and not chosen.nodes:
        how = "rosio drift --only launch.raised" if cli else "drift with only=launch.raised"
        return f"no node in profile {where}: a launch file raised there, {how} names it"
    if not any(edge.kind in LINK_KINDS for edge in atlas.edges.values()):
        if not atlas.by_kind("sym"):
            reason = (
                "endpoints come from the C++ and Python sources and from a probe, "
                "and this atlas read no source"
            )
        else:
            reason = (
                "no endpoint in the sources it read reached a launched node, "
                f"{'rosio unresolved' if cli else 'unresolved'} lists the names it could not follow"
            )
        return f"no node publishes, subscribes, serves or calls anything here: {reason}"
    if where:
        return (
            f"no endpoint in profile {where}, other profiles have some, "
            f"drop {_pass(cli, 'profile')} to see them"
        )
    return "no endpoint between the nodes of this atlas"


def graph(
    atlas: Atlas,
    *,
    profile: str | None = None,
    focus: str | None = None,
    depth: int | None = None,
    wrappers: bool = False,
    limit: int | None = None,
    cli: bool = False,
) -> str:
    """Who talks to what: nodes and the topics, services and actions between them.

    The same selection as the editor's system graph: endpoint edges between nodes and
    channels of one profile, cut to the neighbourhood of a focus within depth hops. QoS is
    the edge's effective block for the profile and is left out where the atlas states none.
    `cli` spells every hint as a flag of `rosio graph` instead of a tool argument.
    """
    chosen = select_graph(atlas, profile=profile, focus=focus, depth=depth, wrappers=wrappers)
    if chosen.complaint:
        return _fit(chosen.complaint, chosen.hint)
    wanted, hops = chosen.wanted, chosen.hops
    lines = [chosen.head]
    if not chosen.endpoints and not chosen.focus_id:
        lines.append(_why_empty(atlas, chosen, cli))
    rows = _limit(limit)
    for entity_id in chosen.channels[:rows]:
        kind = atlas.entities[entity_id].kind
        rules = chosen.rules[entity_id]
        flags = f" [{', '.join(rules)}]" if rules else ""
        sides = []
        for edge_kind, verb, none in _SIDES[kind]:
            # one entry per distinct QoS, so a clip never cuts into a bracket
            ends = {
                (slices.label(edge.src), _qos(edge, wanted))
                for edge in chosen.by_channel.get(entity_id, [])
                if edge.kind == edge_kind
            }
            stated = {name for name, qos in ends if qos}
            named = [
                name + (f" [{qos}]" if qos else "")
                for name, qos in sorted(ends)
                if qos or name not in stated
            ]
            implicit = chosen.lifecycle.get(entity_id)
            if not named and implicit and implicit[0] == edge_kind:
                named = [f"{slices.label(implicit[1])} (lifecycle node)"]
            sides.append(f"{verb} {_named(named, more='more')}" if named else none)
        type_name = _channel_type(atlas, entity_id, wanted)
        typed = f" ({type_name})" if type_name else ""
        lines.append(f"{slices.label(entity_id)}{typed}{flags}: " + "; ".join(sides))
    if len(chosen.channels) > rows:
        lines.append(
            f"and {plural(len(chosen.channels) - rows, 'more channel')}, raise "
            f"{_pass(cli, 'limit')} or pass {_pass(cli, 'focus', '<node>')}"
        )
    if chosen.hidden:
        lines.append(
            f"{plural(chosen.hidden, 'library endpoint')} left out (tf2_ros, diagnostic_updater "
            f"and similar wrappers), pass {_pass(cli, 'wrappers', 'true')} to list them"
        )
    if chosen.focus_id and chosen.beyond:
        lines.append(
            f"{plural(chosen.beyond, 'endpoint')} beyond {plural(hops, 'hop')}, "
            f"raise {_pass(cli, 'depth')} to follow them"
        )
    if not wanted and len(atlas.by_kind("profile")) > 1:
        lines.append(f"these merge every profile, narrow with {_pass(cli, 'profile', '<name>')}")
    return _fit(lines, f"pass {_pass(cli, 'focus', '<node>')} or a smaller {_pass(cli, 'limit')}")


# ---------------------------------------------------------------------------
# diagram, the graph selection drawn

DIAGRAM_FORMATS = ("dot", "mermaid")
_FLOWS_OUT = ("publishes", "calls")
"""Data leaves a publisher or a caller, and reaches a subscriber or a server."""

_DOT_SHAPES = {
    "node": "shape=box",
    "topic": "shape=box, style=rounded",
    "service": "shape=hexagon",
    "action": "shape=octagon",
}
_MERMAID_SHAPES = {
    "node": ('["', '"]'),
    "topic": ('(["', '"])'),
    "service": ('{{"', '"}}'),
    "action": ('[["', '"]]'),
}
_MERMAID_CODES = {
    "#": "#35;",
    '"': "#quot;",
    "<": "#lt;",
    ">": "#gt;",
    "|": "#124;",
    "&": "#amp;",
    "`": "#96;",
    "\r": "",
    "\n": "<br/>",
}
_FINDING_COLOUR = "#d00000"


def _dot_text(text: str) -> str:
    text = text.replace("\\", "\\\\").replace('"', '\\"')
    return text.replace("\r\n", "\n").replace("\r", "\n").replace("\n", "\\n")


def _mermaid_text(text: str) -> str:
    return "".join(_MERMAID_CODES.get(char, char) for char in text)


def _comment(text: str) -> str:
    return " ".join(text.split())


def diagram(
    atlas: Atlas,
    fmt: str,
    *,
    profile: str | None = None,
    focus: str | None = None,
    depth: int | None = None,
    wrappers: bool = False,
    limit: int | None = None,
) -> str:
    """The graph selection as a Graphviz digraph or a Mermaid flowchart, data left to right.

    Nodes are boxes and channels rounded, with the message type on a second line when the
    atlas states one. A channel with findings is drawn red and names their rules. An arrow
    carries QoS only where the atlas states it. `limit` counts channels, as in `graph`, and is
    the only cap: a diagram cut at a byte count does not parse.
    """
    if fmt not in DIAGRAM_FORMATS:
        raise UsageError(f"no diagram format named {fmt!r}, use one of: dot, mermaid")
    chosen = select_graph(atlas, profile=profile, focus=focus, depth=depth, wrappers=wrappers)
    if chosen.complaint:
        raise UsageError("; ".join(line.strip() for line in chosen.complaint))
    rows = max(1, int(limit)) if limit else len(chosen.channels)
    boxes: dict[str, list[str]] = {}
    marked: set[str] = set()
    arrows: dict[tuple[str, str], set[str]] = {}
    for channel in chosen.channels[:rows]:
        boxes[channel] = [slices.label(channel)]
        type_name = _channel_type(atlas, channel, chosen.wanted)
        if type_name:
            boxes[channel].append(type_name)
        if chosen.rules[channel]:
            boxes[channel].append(f"[{', '.join(chosen.rules[channel])}]")
            marked.add(channel)
        for edge in chosen.by_channel.get(channel, []):
            boxes.setdefault(edge.src, [slices.label(edge.src)])
            ends = (edge.src, channel) if edge.kind in _FLOWS_OUT else (channel, edge.src)
            qos = _qos(edge, chosen.wanted)
            arrows.setdefault(ends, set()).update([qos] if qos else [])
        if channel in chosen.lifecycle:
            kind, owner = chosen.lifecycle[channel]
            boxes.setdefault(owner, [slices.label(owner)])
            arrows.setdefault((owner, channel) if kind in _FLOWS_OUT else (channel, owner), set())
    notes = [chosen.head]
    if len(chosen.channels) > rows:
        notes.append(f"{plural(len(chosen.channels) - rows, 'more channel')} not drawn")
    if chosen.hidden:
        notes.append(f"{plural(chosen.hidden, 'library endpoint')} left out")

    def name(entity_id: str) -> str:
        return f"{atlas.entities[entity_id].kind[0]}_{ids.digest(entity_id, length=12)}"

    order = sorted(boxes, key=lambda entity_id: (boxes[entity_id], entity_id))
    links = sorted(
        arrows.items(),
        key=lambda item: (boxes[item[0][0]], boxes[item[0][1]], item[0]),
    )
    if fmt == "dot":
        lines = ["digraph rosio {", *(f"  // {_comment(note)}" for note in notes)]
        lines.append("  rankdir=LR;")
        for entity_id in order:
            label = "\\n".join(_dot_text(line) for line in boxes[entity_id])
            style = _DOT_SHAPES[atlas.entities[entity_id].kind]
            if entity_id in marked:
                style += f', color="{_FINDING_COLOUR}", fontcolor="{_FINDING_COLOUR}", penwidth=2'
            lines.append(f'  {name(entity_id)} [label="{label}", {style}];')
        for (src, dst), qos in links:
            label = f' [label="{_dot_text("; ".join(sorted(qos)))}"]' if qos else ""
            lines.append(f"  {name(src)} -> {name(dst)}{label};")
        lines.append("}")
    else:
        lines = ["flowchart LR", *(f"  %% {_comment(note)}" for note in notes)]
        for entity_id in order:
            label = "<br/>".join(_mermaid_text(line) for line in boxes[entity_id])
            opening, closing = _MERMAID_SHAPES[atlas.entities[entity_id].kind]
            lines.append(f"  {name(entity_id)}{opening}{label}{closing}")
        for (src, dst), qos in links:
            label = f'|"{_mermaid_text("; ".join(sorted(qos)))}"|' if qos else ""
            lines.append(f"  {name(src)} -->{label} {name(dst)}")
        if marked:
            lines.append(
                f"  classDef finding stroke:{_FINDING_COLOUR},stroke-width:2px,"
                f"color:{_FINDING_COLOUR}"
            )
            lines.append(f"  class {','.join(sorted(name(item) for item in marked))} finding")
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------
# unresolved


def unresolved(
    atlas: Atlas,
    *,
    want: str | None = None,
    owner: str | None = None,
    limit: int | None = None,
) -> str:
    """The blind spots: what this atlas could not resolve, and where each one sits."""
    rows = _limit(limit, WIDE_LIMIT)
    entries = sorted(atlas.unresolved, key=lambda item: item.id)
    wanted = _text(want)
    kinds = _counted(entry.want for entry in entries)
    if wanted and wanted not in kinds:
        return _fit(
            [
                f"nothing unresolved of want {wanted}",
                f"  wants in this atlas: {_ranked(kinds, top=12, noun='wants')}",
            ],
            "call unresolved with no want for all of them",
        )
    owner_id = ""
    if _text(owner):
        owner_id, complaint = _entity(atlas, _text(owner))
        if not owner_id:
            return _fit(complaint, "name the owning node, symbol or file")
    kept = [
        entry
        for entry in entries
        if (not wanted or entry.want == wanted) and (not owner_id or entry.owner == owner_id)
    ]
    rates = dict(sorted((atlas.counters.get("resolution_rate") or {}).items()))
    counters = atlas.counters or {}
    total_wanted = sum((counters.get("resolved") or {}).values()) + sum(
        (counters.get("unresolved") or {}).values()
    )
    whole = f"{len(entries)} values unresolved of {total_wanted} wanted"
    lines = [
        f"{len(kept)} values unresolved"
        + (f", want {wanted}" if wanted else "")
        + (f", owner {slices.label(owner_id)}" if owner_id else "")
        + f"; {whole} in this atlas"
        if wanted or owner_id
        else whole
    ]
    lines.append(
        f"  by want: {_ranked(_counted(entry.want for entry in kept), top=10, noun='wants')}"
    )
    if rates:
        lines.append(
            "  resolved per want: "
            + ", ".join(f"{name} {value:.2f}" for name, value in list(rates.items())[:8])
        )
    owners = _counted(slices.label(entry.owner) for entry in kept)
    if not owner_id and owners:
        lines.append(f"  owners: {_ranked(owners, top=3, noun='owners')}")
    for entry in kept[:rows]:
        lines.append(f"  {entry.want}: {slices.readable(entry.expr, atlas)}")
        lines.append(
            f"      in {slices.label(entry.owner)} ({entry.reason})"
            + (f" at {slices.site(entry.evidence[0])}" if entry.evidence else "")
        )
        if entry.resolvable_by:
            lines.append(f"      resolvable by {', '.join(entry.resolvable_by)}")
    if len(kept) > rows:
        lines.append(f"  and {len(kept) - rows} more, narrow with want or owner")
    return _fit(lines, "narrow with want=<want> or owner=<node>")


# ---------------------------------------------------------------------------
# stamps


def stamps(atlas: Atlas, *, bag: str | None = None, limit: int | None = None) -> str:
    """The header stamp verdict of every recording, and the topics whose checks failed."""
    rows = _limit(limit)
    wanted = _text(bag)
    bags = []
    for entity in atlas.by_kind("bag"):
        fact = entity.fact("recorded")
        verdict = (fact.values.get("stamp_verdict") if fact else None) or {}
        if verdict and (not wanted or entity.label == wanted):
            bags.append((entity, verdict))
    if not bags:
        known = sorted(entity.label for entity in atlas.by_kind("bag"))
        if wanted and wanted not in known:
            raise UsageError(
                f"no recording named {wanted} in this atlas, it holds: {_named(known)}. bag "
                "takes the directory name of a recording the atlas holds, and rosio stamps "
                "reads one it does not"
            )
        return _fit(
            ["no recording in this atlas carries header stamp checks"],
            "add a bag root to rosio.yaml and extract, or run rosio stamps on a recording",
        )

    lines = [f"{plural(len(bags), 'recording')} with header stamp checks"]
    for entity, verdict in sorted(bags, key=lambda item: item[0].label):
        lines.append(f"{entity.label}: {verdict.get('level', 'unknown')}")
        lines.extend(f"  {hypothesis}" for hypothesis in verdict.get("hypotheses") or [])
        failed = []
        for edge in _out(atlas, entity.id):
            topic = atlas.entities.get(edge.dst)
            fact = (
                topic.fact("recorded") if topic is not None and topic.kind == "bagtopic" else None
            )
            checks = ((fact.values.get("stamps") or {}).get("checks") or []) if fact else []
            failed.extend((ids.parse_bagtopic(topic.id).topic, row) for row in checks)
        for name, row in sorted(failed, key=lambda item: (item[1]["severity"], item[0]))[:rows]:
            lines.append(
                f"  {row['severity']:<5} {name} {row['check']}: {_clip(row['message'], 110)}"
            )
        if len(failed) > rows:
            lines.append(f"  and {len(failed) - rows} more failed checks, raise limit")
    lines.append("next: drift only=time.backward and the other time.* rules for the line to fix")
    return _fit(lines, "narrow with bag=<recording>")


# ---------------------------------------------------------------------------
# transform


MOVABLE_JOINTS = frozenset({"continuous", "floating", "planar", "prismatic", "revolute"})
"""Joint types whose transform depends on the joint position, composed here at zero."""


@dataclass
class _Hop:
    """One parent to child link: a joint a model states, or a broadcast that ends the walk."""

    parent: str
    child: str
    values: dict
    site: str
    node: str = ""
    layer: str = ""
    static: bool = False
    """A static_transform_publisher a launch file starts: its numbers are in its argv."""


def _frame_ref(name: object) -> str:
    text = str(name or "").strip()
    if text.startswith("frame:"):
        text = text[len("frame:") :]
    try:
        return ids.build_frame(text)
    except ContractError:  # a value that is not a frame id names no frame
        return ""


def _first_site(evidence: Sequence) -> str:
    return slices.site(evidence[0]) if evidence else ""


def _model_joints(atlas: Atlas, wanted: str) -> dict[str, dict[str, _Hop]]:
    """Model file id to child frame to the joint placing it. A urdf edge runs child first.

    A profile that loads a model with its own xacro arguments states that expansion in its
    own slot. Asked for that profile, it replaces the expansion with the defaults. Asked for
    no profile, the defaults answer for every frame they place, and the profile expansions
    for the frames only they name."""
    own: dict[str, dict[str, _Hop]] = {}
    shared: dict[str, dict[str, _Hop]] = {}
    for handle in sorted(atlas.edges):
        edge = atlas.edges[handle]
        if edge.kind != "tf_parent":
            continue
        for fact in edge.facts:
            model_id = str(fact.values.get("model") or "")
            if not model_id or fact.confidence == "unknown":
                continue
            if wanted and fact.profile not in (wanted, ANY_PROFILE):
                continue
            into = shared if fact.profile == ANY_PROFILE else own
            hop = _Hop(edge.dst, edge.src, fact.values, _first_site(fact.evidence))
            into.setdefault(model_id, {}).setdefault(edge.src, hop)
    if wanted:
        return {**shared, **own}
    return {key: {**own.get(key, {}), **shared.get(key, {})} for key in sorted({*own, *shared})}


def _broadcast_hops(atlas: Atlas, wanted: str) -> dict[str, list[_Hop]]:
    """Child frame to the broadcasts that state its parent at run time."""
    out: dict[str, list[_Hop]] = {}
    for handle in sorted(atlas.edges):
        edge = atlas.edges[handle]
        if edge.kind != "broadcasts_tf":
            continue
        for fact in edge.facts:
            if wanted and fact.profile not in (wanted, ANY_PROFILE):
                continue
            parent = _frame_ref(fact.values.get("parent"))
            if parent and not any(
                hop.node == edge.src and hop.parent == parent for hop in out.get(edge.dst, [])
            ):
                out.setdefault(edge.dst, []).append(
                    _Hop(
                        parent,
                        edge.dst,
                        fact.values,
                        _first_site(fact.evidence),
                        edge.src,
                        fact.layer,
                    )
                )
    return out


STATIC_TF = "static_transform_publisher"

_STATIC_FLAGS = {
    "--x": ("xyz", 0),
    "--y": ("xyz", 1),
    "--z": ("xyz", 2),
    "--roll": ("rpy", 0),
    "--pitch": ("rpy", 1),
    "--yaw": ("rpy", 2),
    "--qx": ("quaternion", 0),
    "--qy": ("quaternion", 1),
    "--qz": ("quaternion", 2),
    "--qw": ("quaternion", 3),
}
_STATIC_PARENT = ("--frame-id", "--frame_id")
_STATIC_CHILD = ("--child-frame-id", "--child_frame_id")


def _static_argv(argv: Sequence[object]) -> tuple[str, str, dict] | None:
    """(parent, child, values) from the argv of a static_transform_publisher.

    Both forms tf2_ros reads: the flags, where an omitted number is the zero and the identity
    rotation tf2_ros itself takes, and the positional `x y z yaw pitch roll parent child` or
    `x y z qx qy qz qw parent child`. A word that is not a number is named, never read as 0.
    """
    words = [str(item) for item in argv]
    if "--ros-args" in words:
        words = words[: words.index("--ros-args")]
    stated: dict[str, list[str]] = {
        "xyz": ["0", "0", "0"],
        "rpy": ["0", "0", "0"],
        "quaternion": ["0", "0", "0", "1"],
    }
    given: set[str] = set()
    parent = child = ""
    if any(word.startswith("--") for word in words):
        for index, word in enumerate(words[:-1]):
            if word in _STATIC_PARENT:
                parent = words[index + 1]
            elif word in _STATIC_CHILD:
                child = words[index + 1]
            elif word in _STATIC_FLAGS:
                key, slot = _STATIC_FLAGS[word]
                stated[key][slot] = words[index + 1]
                given.add(key)
    elif len(words) == 8:
        stated["xyz"] = words[:3]
        stated["rpy"] = [words[5], words[4], words[3]]
        parent, child = words[6], words[7]
    elif len(words) == 9:
        stated["xyz"], stated["quaternion"] = words[:3], words[3:7]
        given.add("quaternion")
        parent, child = words[7], words[8]
    parent, child = _frame_ref(parent), _frame_ref(child)
    if not parent or not child:
        return None
    quaternion = "quaternion" in given
    keys = ("xyz", "quaternion") if quaternion else ("xyz", "rpy")
    values: dict = {}
    unreadable = []
    for key in keys:
        try:
            values[key] = [float(word) for word in stated[key]]
        except ValueError:
            unreadable.append(key)
    if unreadable:
        values["unreadable"] = unreadable
    elif quaternion:
        values["rpy"] = _rpy_of(_rotation_q(values["quaternion"]))
    return parent, child, values


def _rotation_q(q: Sequence[float]) -> Matrix:
    norm = math.sqrt(sum(item * item for item in q)) or 1.0
    x, y, z, w = (item / norm for item in q)
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def _static_hops(atlas: Atlas, wanted: str) -> dict[str, list[_Hop]]:
    """Child frame to the static_transform_publisher nodes that place it, read off the argv
    the declared layer recorded, with the launch line that starts each one."""
    out: dict[str, list[_Hop]] = {}
    for node in atlas.by_kind("node"):
        for fact in node.facts:
            if fact.values.get("executable") != STATIC_TF or fact.confidence == "unknown":
                continue
            if wanted and fact.profile not in (wanted, ANY_PROFILE):
                continue
            read = _static_argv(fact.values.get("argv") or [])
            if read is None:
                continue
            parent, child, values = read
            # The launch line, not the name table row that may sit beside it.
            sites = [row for row in fact.evidence if not row.file.startswith("file:@repo|")]
            hop = _Hop(parent, child, values, _first_site(sites), node.id, fact.layer, True)
            if not any(item.node == hop.node for item in out.get(child, [])):
                out.setdefault(child, []).append(hop)
    return out


def _model_profiles(atlas: Atlas) -> dict[str, set[str]]:
    """Model file id to the profiles that load it, read the way the robot view reads it."""
    out: dict[str, set[str]] = {}
    for entity in atlas.by_kind("launchent"):
        for fact in entity.facts:
            for read in fact.values.get("file_reads") or []:
                if isinstance(read, dict) and read.get("role") == "robot_model":
                    out.setdefault(str(read.get("file")), set()).add(fact.profile)
    for edge in atlas.edges.values():
        if edge.kind == "tf_parent":
            for fact in edge.facts:
                if fact.values.get("model"):
                    out.setdefault(str(fact.values["model"]), set()).add(fact.profile)
    return {key: {p for p in value if p.startswith("profile:")} for key, value in out.items()}


def _walk_up(frame: str, joints: dict[str, _Hop], casts: dict[str, list[_Hop]]) -> list[_Hop]:
    hops: list[_Hop] = []
    seen = {frame}
    while True:
        hop = joints.get(frame) or next(iter(casts.get(frame, [])), None)
        if hop is None or hop.parent in seen:
            return hops
        hops.append(hop)
        seen.add(hop.parent)
        frame = hop.parent


Matrix = list[list[float]]
Pose = tuple[Matrix, list[float]]


def _rotation(rpy: Sequence[float]) -> Matrix:
    """URDF rpy, fixed axis: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    sr, cr = math.sin(rpy[0]), math.cos(rpy[0])
    sp, cp = math.sin(rpy[1]), math.cos(rpy[1])
    sy, cy = math.sin(rpy[2]), math.cos(rpy[2])
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _compose(a: Pose, b: Pose) -> Pose:
    (ra, ta), (rb, tb) = a, b
    rot = [[sum(ra[i][k] * rb[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return rot, [sum(ra[i][k] * tb[k] for k in range(3)) + ta[i] for i in range(3)]


def _invert(pose: Pose) -> Pose:
    rot, t = pose
    back = [[rot[j][i] for j in range(3)] for i in range(3)]
    return back, [-sum(back[i][k] * t[k] for k in range(3)) for i in range(3)]


def _rpy_of(r: Matrix) -> list[float]:
    pitch = math.asin(max(-1.0, min(1.0, -r[2][0])))
    if abs(math.cos(pitch)) > 1e-9:
        return [math.atan2(r[2][1], r[2][2]), pitch, math.atan2(r[1][0], r[0][0])]
    return [0.0, pitch, math.atan2(-r[0][1], r[1][1])]


def _quaternion_of(r: Matrix) -> list[float]:
    """x, y, z, w in one form per rotation: w is kept non negative, and when w is zero (a half
    turn) the first non zero of x, y and z is kept positive, since q and -q are one rotation."""
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0:
        s = 2 * math.sqrt(1 + trace)
        q = [(r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s, s / 4]
    elif r[0][0] >= r[1][1] and r[0][0] >= r[2][2]:
        s = 2 * math.sqrt(1 + r[0][0] - r[1][1] - r[2][2])
        q = [s / 4, (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s, (r[2][1] - r[1][2]) / s]
    elif r[1][1] >= r[2][2]:
        s = 2 * math.sqrt(1 + r[1][1] - r[0][0] - r[2][2])
        q = [(r[0][1] + r[1][0]) / s, s / 4, (r[1][2] + r[2][1]) / s, (r[0][2] - r[2][0]) / s]
    else:
        s = 2 * math.sqrt(1 + r[2][2] - r[0][0] - r[1][1])
        q = [(r[0][2] + r[2][0]) / s, (r[1][2] + r[2][1]) / s, s / 4, (r[1][0] - r[0][1]) / s]
    lead = next((item for item in (q[3], *q[:3]) if abs(item) > 1e-9), 0.0)
    return [-item for item in q] if lead < 0 else q


def _computed(values: Sequence[float]) -> str:
    out = []
    for item in values:
        text = f"{item:.6f}".rstrip("0").rstrip(".")
        out.append("0" if text in ("", "-0") else text)
    return " ".join(out)


def _stated(values: object) -> str:
    if not isinstance(values, (list, tuple)):
        return "unknown"
    out = []
    for item in values:
        text = repr(item)
        text = text[:-2] if text.endswith(".0") else text
        out.append("0" if text == "-0" else text)
    return " ".join(out)


ORIGIN_FIELDS = ("origin.xyz", "origin.rpy")


def _unread_origin(hop: _Hop) -> list[str]:
    """What of the origin did not read. A limit, a mimic or an axis does not place a frame."""
    return [item for item in hop.values.get("unreadable") or () if item in ORIGIN_FIELDS]


def _readable(hop: _Hop) -> bool:
    return not _unread_origin(hop) and all(
        isinstance(hop.values.get(key), (list, tuple))
        and len(hop.values[key]) == 3
        and all(isinstance(item, (int, float)) for item in hop.values[key])
        for key in ("xyz", "rpy")
    )


def _hop_line(hop: _Hop) -> str:
    frames = f"{ids.parse_frame(hop.parent)} -> {ids.parse_frame(hop.child)}"
    if hop.static:
        rotation = (
            f"quaternion xyzw [{_stated(hop.values['quaternion'])}]"
            if "quaternion" in hop.values
            else f"rpy [{_stated(hop.values.get('rpy'))}]"
        )
        return (
            f"    static {frames} by {slices.label(hop.node)} ({hop.layer}): "
            f"xyz [{_stated(hop.values.get('xyz'))}] {rotation} at {hop.site}"
        )
    if hop.node:
        return f"    broadcast {frames} by {slices.label(hop.node)} ({hop.layer}) at {hop.site}"
    kind = str(hop.values.get("joint_type") or "unknown type")
    line = (
        f"    joint {hop.values.get('joint', '?')} ({kind}) {frames}: "
        f"xyz [{_stated(hop.values.get('xyz'))}] rpy [{_stated(hop.values.get('rpy'))}]"
        f" at {hop.site}"
    )
    if kind in MOVABLE_JOINTS:
        line += ", moves, taken at position zero"
    return line


def _chain_answer(
    source: str, target: str, joints: dict[str, _Hop], casts: dict[str, list[_Hop]]
) -> tuple[tuple, list[str]]:
    """The answer one model gives, as a comparable key and the lines that state it."""
    name_s, name_t = ids.parse_frame(source), ids.parse_frame(target)
    up_s, up_t = _walk_up(source, joints, casts), _walk_up(target, joints, casts)
    above_s = [source] + [hop.parent for hop in up_s]
    above_t = [target] + [hop.parent for hop in up_t]
    common = next((frame for frame in above_s if frame in above_t), "")
    if not common:
        roots = f"{ids.parse_frame(above_s[-1])} and {ids.parse_frame(above_t[-1])}"
        return ("apart", above_s[-1], above_t[-1]), [
            f"  {name_s} and {name_t} share no ancestor: their chains end at {roots}, "
            "and no joint or broadcast in this atlas links the two"
        ]
    rise = up_t[: above_t.index(common)]
    down = up_s[: above_s.index(common)]
    hops = rise + list(reversed(down))
    key = tuple(
        (
            hop.parent,
            hop.child,
            hop.node,
            str(hop.values.get("joint") or ""),
            str(hop.values.get("joint_type") or ""),
            *(_stated(hop.values.get(k)) for k in ("xyz", "rpy")),
        )
        for hop in hops
    )
    legs = [f"up to {ids.parse_frame(common)}"] if rise else []
    legs += [f"down to {name_s}"] if down else []
    chain = [
        f"  chain from {name_t} {' and '.join(legs) or 'to itself'}, {plural(len(hops), 'link')}:",
        *(_hop_line(hop) for hop in hops),
    ]
    casted = [hop for hop in hops if hop.node and not hop.static]
    if casted:
        first = casted[0]
        return ("stop", key), [
            f"  not static: {ids.parse_frame(first.parent)} -> {ids.parse_frame(first.child)} "
            f"is broadcast by {slices.label(first.node)} at {first.site}, so its value exists "
            "only at run time and nothing is composed across it",
            *chain,
        ]
    blind = [hop for hop in hops if not _readable(hop)]
    if blind and blind[0].static:
        return ("unreadable", key), [
            f"  {slices.label(blind[0].node)} starts with arguments that are not numbers "
            f"({', '.join(blind[0].values.get('unreadable') or ['origin'])}), so nothing is "
            "composed",
            *chain,
        ]
    if blind:
        return ("unreadable", key), [
            f"  joint {blind[0].values.get('joint', '?')} states an origin that is not numbers "
            f"({', '.join(_unread_origin(blind[0]) or ['origin'])}), so nothing is "
            "composed; expand the model with xacro or write the numbers",
            *chain,
        ]

    def placed(path: Sequence[_Hop]) -> Pose:
        pose: Pose = ([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], [0.0, 0.0, 0.0])
        for hop in reversed(path):
            step = (_rotation(hop.values["rpy"]), [float(item) for item in hop.values["xyz"]])
            pose = _compose(pose, step)
        return pose

    rot, xyz = _compose(_invert(placed(rise)), placed(down))
    lines = [
        f"  translation xyz [{_computed(xyz)}] m",
        f"  rotation rpy [{_computed(_rpy_of(rot))}] rad, "
        f"quaternion xyzw [{_computed(_quaternion_of(rot))}]",
    ]
    moving = [hop for hop in hops if hop.values.get("joint_type") in MOVABLE_JOINTS]
    if moving:
        lines.append(
            f"  {plural(len(moving), 'movable joint')} on the chain, taken at position zero, "
            "so the live value differs whenever a joint moves"
        )
    return ("pose", key), lines + chain


def _model_choice(models: Iterable[str], wanted: str) -> list[str]:
    return [
        item
        for item in models
        if wanted in (item, slices.label(item), item.rpartition("|")[2])
        or slices.label(item).endswith("/" + wanted)
    ]


def _loaded_text(profiles: Iterable[str]) -> str:
    names = sorted(slices.profile_name(item) for item in profiles)
    if not names:
        return "loaded by no profile this atlas states"
    return f"loaded by {'profile' if len(names) == 1 else 'profiles'} {', '.join(names)}"


def transform(
    atlas: Atlas,
    source: str,
    target: str,
    *,
    profile: str | None = None,
    model: str | None = None,
) -> str:
    """Where `source` sits relative to `target`, composed from the robot model joints and the
    static_transform_publisher nodes the launch files start.

    The pose of the source frame in the target frame, the transform lookupTransform(target,
    source) returns. A broadcast on the chain stops the composition, movable joints are taken
    at zero, and models that state the chain differently answer one each.
    """
    wanted = _profile_id(atlas, profile)
    casts = _static_hops(atlas, wanted)
    # a static publisher names its child frame in its argv, and no collector mints an entity
    # for it, so the frames it places are frames here too
    placed = {frame for hops in casts.values() for hop in hops for frame in (hop.child, hop.parent)}
    frames = []
    for name in (source, target):
        frame = _frame_ref(name)
        if not frame or (frame not in atlas.entities and frame not in placed):
            every = sorted(item.label for item in atlas.by_kind("frame"))
            near = [item for item in every if _text(name).lower() in item.lower()]
            return _fit(
                [
                    f"no frame named {_text(name)!r} in this atlas",
                    f"  frames with that text: {_named(near)}"
                    if near
                    else f"  frames: {_named(every, more='more frames')}",
                ],
                "name a frame exactly, such as base_link",
            )
        frames.append(frame)
    src, tgt = frames
    joints = _model_joints(atlas, wanted)
    for child, hops in _broadcast_hops(atlas, wanted).items():
        casts.setdefault(child, []).extend(hops)
    loads = _model_profiles(atlas)
    models = sorted(joints)
    if _text(model):
        models = _model_choice(models, _text(model))
        if not models:
            return _fit(
                [
                    f"no robot model named {_text(model)!r} states a joint here",
                    f"  models: {_named([slices.label(item) for item in sorted(joints)])}",
                ],
                "name a model by its file, such as urdf/robot.urdf",
            )
    elif wanted:
        models = [item for item in models if wanted in loads.get(item, set())] or models

    def touches(item: str) -> bool:
        return any(
            frame in (hop.child, hop.parent) for hop in joints[item].values() for frame in frames
        )

    models = [item for item in models if touches(item)]
    name_s, name_t = ids.parse_frame(src), ids.parse_frame(tgt)
    lines = [
        f"{name_s} relative to {name_t}: the pose of {name_s} in the {name_t} frame, as "
        f'lookupTransform("{name_t}", "{name_s}") returns it',
        "  static robot model geometry and static publisher arguments, metres and radians, "
        "rpy fixed axis "
        "R = Rz(yaw) Ry(pitch) Rx(roll), line numbers advisory",
    ]
    if not models:
        key, body = _chain_answer(src, tgt, {}, casts)
        if key[0] == "apart" and src != tgt:
            body = [f"  no robot model in this atlas states a joint on {name_s} or {name_t}", *body]
        return _fit(lines + body, "name the frames of one robot model")
    groups: dict[tuple, tuple[list[str], list[str]]] = {}
    for item in models:
        key, body = _chain_answer(src, tgt, joints[item], casts)
        groups.setdefault(key, ([], body))[0].append(item)
    if len(groups) > 1:
        lines.append(f"{len(groups)} robot models state this chain differently, one answer each")
    for named, body in groups.values():
        profiles = set().union(*(loads.get(item, set()) for item in named))
        lines.append(
            f"{'models' if len(named) > 1 else 'model'} "
            f"{', '.join(slices.label(item) for item in named)}, {_loaded_text(profiles)}"
        )
        lines.extend(body)
    return _fit(lines, "pass model=<file> or profile=<slug> for one answer")


# ---------------------------------------------------------------------------
# migrate


ROS_PREFIX = Path("/opt/ros")


def current_release(start: str | None) -> tuple[str, str]:
    """The release a migration starts from, and a sentence when it was not stated.

    A server an assistant starts gets a minimal environment, so ROS_DISTRO is usually not
    set there even on a machine with ROS sourced in every shell. One release installed under
    /opt/ros is then the answer, and the reply says it was read from there."""
    import os

    if _text(start):
        return _text(start), ""
    if os.environ.get("ROS_DISTRO"):
        return os.environ["ROS_DISTRO"], ""
    try:
        found = sorted(p.name for p in ROS_PREFIX.iterdir() if (p / "setup.bash").is_file())
    except OSError:
        found = []
    if len(found) == 1:
        return found[0], (
            f"from {found[0]}, the one release under {ROS_PREFIX}, because ROS_DISTRO is not "
            "set where this server runs. pass start to name another"
        )
    held = f"holds {', '.join(found)}" if found else "holds no release"
    return "", (
        f"the current release is unknown: ROS_DISTRO is not set where this server runs and "
        f"{ROS_PREFIX} {held}. pass start, for example start=humble"
    )


CONFINED = (
    "over http migrate reads only the workspace the served atlas was written in, so name a "
    "directory inside it or run rosio migrate on that machine"
)


def migrate(
    workspace: str,
    target: str,
    *,
    start: str | None = None,
    package: str | None = None,
    limit: int | None = None,
    within: Path | None = None,
) -> str:
    """What moving the workspace to another release asks, by hand items first.

    The exact rewrites are listed so an agent can say what apply will change, and handed
    to `rosio migrate apply`, because an agent that retypes them can only add mistakes. This
    reads source files, not the atlas, and edits nothing. `within` keeps it to one directory,
    because over http a caller would otherwise read any tree the server user can.
    """
    from rosio import migrate as release

    source, how = current_release(start)
    if not source:
        return _fit([how], "call migrate with start=<release>")
    directory = _text(workspace) or "."
    if within is not None:
        directory = str((within / directory).resolve())
        if not Path(directory).is_relative_to(within):
            raise UsageError(CONFINED)
    try:
        changes, root = release.plan_workspace(directory, source, target)
    except OSError as exc:
        raise UsageError(
            f"migrate could not read {exc.filename or directory}: {exc.strerror or exc}"
        ) from exc
    if within is not None:
        # A link inside the workspace can point anywhere, and its lines must not leave.
        changes = [
            change
            for change in changes
            if (Path(root) / change.file).resolve().is_relative_to(within)
        ]
    wanted = _text(package)
    if wanted:
        changes = [change for change in changes if change.package == wanted]
    automatic = [change for change in changes if change.automatic]
    by_hand = [change for change in changes if not change.automatic]
    rows = _limit(limit, fallback=MAX_LIMIT)
    lines = [
        f"{source} -> {target}: {plural(len(changes), 'change')}, "
        f"{plural(len(automatic), 'exact rewrite')}, {len(by_hand)} by hand"
    ]
    if how:
        lines.append(how)
    if automatic:
        lines.append(
            f"rewrites: run `rosio migrate apply --from {source} --to {target}` once, "
            "do not retype them"
        )
    for change in by_hand[:rows]:
        lines.append(
            f"{change.file}:{change.line} {change.effect} in {change.release}: {change.say}"
        )
    if len(by_hand) > rows:
        lines.append(f"and {len(by_hand) - rows} more by hand, narrow with package")
    room = max(0, rows - len(by_hand))
    for change in automatic[:room]:
        first, *rest = change.replacement.splitlines() or [""]
        more = " ..." if rest else ""
        lines.append(f"{change.file}:{change.line} apply rewrites {change.found} to {first}{more}")
    if len(automatic) > room:
        lines.append(f"and {len(automatic) - room} more rewrites, narrow with package")
    if not changes:
        lines.append("nothing in this workspace changes")
    else:
        lines.append(f"then run `rosio migrate verify --from {source} --to {target}` to confirm")
    return _fit(lines, "narrow with package=<name>")


__all__ = [
    "DEFAULT_LIMIT",
    "MAX_BYTES",
    "AtlasSource",
    "current_release",
    "diagram",
    "drift",
    "explain",
    "graph",
    "load",
    "migrate",
    "neighbours",
    "overview",
    "param_chain",
    "path",
    "query",
    "stamps",
    "transform",
    "unresolved",
]
