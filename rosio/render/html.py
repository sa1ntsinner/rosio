"""atlas.html: the whole atlas as one self contained page.

Nothing is fetched at run time. The css and the js are vendored under `assets/` and
inlined, and the data is embedded as two JSON blocks.

The split is what keeps a real stack usable. The index block holds every entity and
every edge with the small fields only (label, kind, layer mask, profile mask, the
precomputed flags and a laid out coordinate), so the page paints from it alone. The
detail block holds the bulk (facts, effective values, candidate chains, provenance) and
is parsed on the first click that needs it, from the same file.

Determinism: the layout is computed here from sorted input with no clock and no random
source, every collection is sorted, and the only encoder is `model.dumps`. Two runs over
one atlas produce identical bytes.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from rosio import ids, model
from rosio.errors import ContractError
from rosio.model import Atlas, Node
from rosio.render import HTML_NAME, rendered_from

ASSETS = Path(__file__).parent / "assets"

SEVERITY_RANK = {"info": 1, "warn": 2, "error": 3}

HUB_KINDS = ("node", "pkg", "profile", "bag")
"""Kinds that own a cluster. Everything else is drawn inside the cluster of the first
of these it reaches through an ownership edge."""

ANCHOR_EDGES = (
    "has_param",
    "declares_param",
    "reads_param",
    "contains",
    "defines",
    "launches",
    "runs",
    "instantiates",
    "provides_plugin",
    "loads_plugin",
    "publishes",
    "subscribes",
    "serves",
    "calls",
    "broadcasts_tf",
    "listens_tf",
    "tf_parent",
    "uses_type",
    "expands_to",
    "includes",
    "samples",
    "manages_lifecycle",
    "gates",
    "remaps",
)
"""Ownership direction for the layout, most specific first. The destination of the first
matching edge is drawn inside the source's cluster."""

_GOLDEN = 2.399963229728653


# ---------------------------------------------------------------------------
# small helpers


def _file_path(entity_id: str) -> str:
    """`file:pkg|rel/path` reads as `pkg/rel/path`. A repo file drops the marker."""
    try:
        parsed = ids.parse(entity_id)
    except ContractError:
        return entity_id
    if parsed.kind not in ("file", "launchfile") or len(parsed.segments) != 2:
        return entity_id
    package, relpath = parsed.segments
    if package == "@repo":
        return relpath
    return f"{package}/{relpath}"


def _bits(members: list[str], table: dict[str, int]) -> int:
    mask = 0
    for member in members:
        if member in table:
            mask |= 1 << table[member]
    return mask


def _short(value: Any, limit: int = 48) -> str:
    if isinstance(value, bool) or value is None:
        text = {True: "true", False: "false", None: "unknown"}[value]
    elif isinstance(value, (list, dict)):
        text = f"{len(value)} entries"
    else:
        text = str(value)
    text = " ".join(text.split())
    if len(text) > limit:
        return text[: limit - 1] + "…"
    return text


# ---------------------------------------------------------------------------
# layout


def _anchors(atlas: Atlas) -> dict[str, str]:
    """One owner per entity, from the ownership edges, taken in a fixed kind order so
    the answer does not depend on the edge iteration order."""
    rank = {kind: index for index, kind in enumerate(ANCHOR_EDGES)}
    best: dict[str, tuple[int, str]] = {}
    for handle in sorted(atlas.edges):
        edge = atlas.edges[handle]
        order = rank.get(edge.kind)
        if order is None or edge.src == edge.dst:
            continue
        if edge.src not in atlas.entities or edge.dst not in atlas.entities:
            continue
        # a tf_parent edge runs from the child up, and the parent owns the child
        owner, owned = (edge.dst, edge.src) if edge.kind == "tf_parent" else (edge.src, edge.dst)
        current = best.get(owned)
        if current is None or (order, owner) < current:
            best[owned] = (order, owner)
    return {dst: owner for dst, (_order, owner) in best.items()}


def _hub_of(entity_id: str, atlas: Atlas, anchors: dict[str, str]) -> str:
    seen = {entity_id}
    current = entity_id
    for _hop in range(8):
        node = atlas.entities.get(current)
        if node is not None and node.kind in HUB_KINDS:
            return current
        parent = anchors.get(current)
        if parent is None or parent in seen:
            break
        seen.add(parent)
        current = parent
    return current


def _pack(radii: list[tuple[str, float]]) -> dict[str, tuple[float, float]]:
    """Ring packing, largest cluster in the middle. Deterministic, so the picture is stable
    between runs. Rings are spaced by the widest member of the ring below, which leaves a
    few neighbouring clusters touching (2 of 458 on the reference stack) and never stacks
    one on another."""
    placed: dict[str, tuple[float, float]] = {}
    if not radii:
        return placed
    first, first_radius = radii[0]
    placed[first] = (0.0, 0.0)
    ring_radius = first_radius * 1.5 + 30.0
    ring: list[tuple[str, float]] = []
    used = 0.0
    largest = 0.0

    def flush() -> None:
        nonlocal used, ring, largest
        angle = 0.0
        for name, radius in ring:
            step = 2.0 * math.asin(min(0.999, (radius + 14.0) / ring_radius))
            placed[name] = (
                ring_radius * math.cos(angle + step / 2.0),
                ring_radius * math.sin(angle + step / 2.0),
            )
            angle += step
        ring = []
        used = 0.0
        largest = 0.0

    for name, radius in radii[1:]:
        step = 2.0 * math.asin(min(0.999, (radius + 14.0) / ring_radius))
        if used + step > 2.0 * math.pi and ring:
            previous = largest
            flush()
            ring_radius += previous + radius + 60.0
            step = 2.0 * math.asin(min(0.999, (radius + 14.0) / ring_radius))
        ring.append((name, radius))
        used += step
        largest = max(largest, radius)
    flush()
    return placed


def _layout(atlas: Atlas) -> tuple[dict[str, tuple[float, float]], dict[str, float]]:
    """Positions for every entity, plus the cluster radius of every hub. The radius is
    what the page uses to decide which cluster names are legible at a given zoom."""
    anchors = _anchors(atlas)
    members: dict[str, list[str]] = {}
    for entity_id in sorted(atlas.entities):
        members.setdefault(_hub_of(entity_id, atlas, anchors), []).append(entity_id)
    radii = [
        (hub, 14.0 + 4.6 * math.sqrt(len(items)))
        for hub, items in sorted(members.items(), key=lambda row: (-len(row[1]), row[0]))
    ]
    centres = _pack(radii)
    out: dict[str, tuple[float, float]] = {}
    for hub, radius in radii:
        centre_x, centre_y = centres[hub]
        items = members[hub]
        inner = radius - 6.0
        count = len(items)
        for index, entity_id in enumerate(items):
            if entity_id == hub:
                out[entity_id] = (centre_x, centre_y)
                continue
            share = (index + 0.5) / count
            distance = inner * math.sqrt(share)
            angle = index * _GOLDEN
            out[entity_id] = (
                centre_x + distance * math.cos(angle),
                centre_y + distance * math.sin(angle),
            )
    return out, dict(radii)


# ---------------------------------------------------------------------------
# projection


def _entity_flags(node: Node) -> int:
    flags = 0
    if node.conflicts:
        flags |= 1
    if node.unresolved:
        flags |= 2
    if any(candidate.shadow_kind for candidate in node.candidates):
        flags |= 4
    if node.effective:
        flags |= 8
    if node.candidates:
        flags |= 16
    if node.aka:
        flags |= 32
    return flags


def _effective_text(per_key: dict[str, Any]) -> str:
    """The effective value, or nothing.

    A block that names no value has none to show. A `pparam` effective block carries
    `base_class`, `lookup` and `suffix` and no value, 266 of them on the reference stack,
    and any of those three in the value column reads as the parameter's value.
    """
    for key in ("name", "value"):
        if key in per_key:
            return _short(per_key[key].value)
    return ""


def _effective_values(node: Node, profile_index: dict[str, int]) -> tuple[str, list[str] | None]:
    """The short effective value, plus the per profile list when the profiles disagree.

    One value stated without its profile is the wrong number as soon as two profiles
    configure the same parameter differently, which they do 103 times on the reference
    stack, so the divergent case carries every profile and the caller picks.
    """
    texts = {profile: _effective_text(per_key) for profile, per_key in node.effective.items()}
    distinct = {text for text in texts.values() if text}
    if len(distinct) <= 1:
        return (distinct.pop() if distinct else ""), None
    if not profile_index:
        return texts[sorted(texts)[0]], None
    shared = texts.get(model.ANY_PROFILE, "")
    slots = [shared] * len(profile_index)
    for profile, text in sorted(texts.items()):
        slot = profile_index.get(profile)
        if slot is not None:
            slots[slot] = text
    return "", slots


def _shadow_mask(node: Node, profile_index: dict[str, int]) -> int:
    """The profiles in which some candidate lost. A candidate held by every profile sets
    every bit, so the shadowed filter never widens or narrows silently."""
    every = (1 << len(profile_index)) - 1
    mask = 0
    for candidate in node.candidates:
        if not candidate.shadow_kind:
            continue
        slot = profile_index.get(candidate.profile)
        mask |= (1 << slot) if slot is not None else every
    return mask


def _index(
    atlas: Atlas,
    layout: dict[str, tuple[float, float]],
    radii: dict[str, float],
) -> dict[str, Any]:
    entity_ids = sorted(atlas.entities)
    position = {entity_id: index for index, entity_id in enumerate(entity_ids)}

    kinds = sorted({atlas.entities[key].kind for key in entity_ids})
    kind_index = {kind: index for index, kind in enumerate(kinds)}
    layers = sorted({layer for key in entity_ids for layer in atlas.entities[key].layers})
    layers = [name for name in model.LAYER_ORDER if name in layers] + [
        name for name in layers if name not in model.LAYER_ORDER
    ]
    layer_index = {layer: index for index, layer in enumerate(layers)}
    profiles = sorted(entity for entity in entity_ids if entity.startswith("profile:"))
    profile_index = {profile: index for index, profile in enumerate(profiles)}
    states = sorted({atlas.entities[key].state or "" for key in entity_ids} - {""})
    state_index = {state: index for index, state in enumerate(states)}
    agreements = ["agree", "differ", "single_layer", "unknown"]

    severity: dict[str, int] = {}
    for finding in atlas.findings:
        for subject in _finding_subjects(finding):
            rank = SEVERITY_RANK.get(finding.severity, 0)
            severity[subject] = max(severity.get(subject, 0), rank)

    columns: dict[str, list[Any]] = {
        "id": [],
        "label": [],
        "kind": [],
        "layers": [],
        "profiles": [],
        "agreement": [],
        "flags": [],
        "finding": [],
        "state": [],
        "value": [],
        "shadow": [],
        "x": [],
        "y": [],
        "radius": [],
    }
    value_by_profile: dict[str, list[str]] = {}
    for row, entity_id in enumerate(entity_ids):
        node = atlas.entities[entity_id]
        point = layout.get(entity_id, (0.0, 0.0))
        columns["id"].append(entity_id)
        columns["label"].append(node.label)
        columns["kind"].append(kind_index[node.kind])
        columns["layers"].append(_bits(node.layers, layer_index))
        columns["profiles"].append(_bits(node.profiles, profile_index))
        columns["agreement"].append(
            agreements.index(node.agreement) if node.agreement in agreements else 3
        )
        columns["flags"].append(_entity_flags(node))
        columns["finding"].append(severity.get(entity_id, 0))
        columns["state"].append(state_index.get(node.state or "", -1))
        if node.kind in ("param", "pparam"):
            value, per_profile = _effective_values(node, profile_index)
            if per_profile is not None:
                value_by_profile[str(row)] = per_profile
        else:
            value = ""
        columns["value"].append(value)
        columns["shadow"].append(_shadow_mask(node, profile_index))
        columns["x"].append(round(point[0], 1))
        columns["y"].append(round(point[1], 1))
        columns["radius"].append(round(radii.get(entity_id, 0.0), 1))

    edge_kinds = sorted({edge.kind for edge in atlas.edges.values()})
    edge_kind_index = {kind: index for index, kind in enumerate(edge_kinds)}
    flat: list[int] = []
    for handle in sorted(atlas.edges):
        edge = atlas.edges[handle]
        src = position.get(edge.src)
        dst = position.get(edge.dst)
        if src is None or dst is None:
            continue
        flat.extend((src, dst, edge_kind_index[edge.kind], _bits(edge.profiles, profile_index)))

    findings = []
    for finding in sorted(atlas.findings, key=model.Finding.sort_key):
        findings.append(
            {
                "rule": finding.rule,
                "severity": finding.severity,
                "title": finding.title,
                "say": finding.spoken if finding.say else "",
                "detail": finding.detail,
                "fix": finding.fix,
                "profiles": [profile_index[item] for item in finding.profiles if item in profiles],
                "subjects": [
                    position[subject]
                    for subject in _finding_subjects(finding)
                    if subject in position
                ],
                "where": [
                    {
                        "file": _file_path(source.file),
                        "line": source.line_hint,
                        "locator": source.locator,
                    }
                    for source in finding.where
                ],
            }
        )

    paths = {
        entity_id: _file_path(entity_id)
        for entity_id in entity_ids
        if atlas.entities[entity_id].kind in ("file", "launchfile")
    }

    return {
        "schema_version": atlas.schema_version,
        "generator": dict(atlas.generator),
        "capabilities": sorted(atlas.capabilities),
        "counters": atlas.counters,
        "kinds": kinds,
        "layer_names": layers,
        "profiles": [
            {"id": profile, "label": atlas.entities[profile].label} for profile in profiles
        ],
        "states": states,
        "agreements": agreements,
        "edge_kinds": edge_kinds,
        "entities": columns,
        "value_by_profile": value_by_profile,
        "edges": flat,
        "findings": findings,
        "paths": paths,
    }


def _finding_subjects(finding: model.Finding) -> list[str]:
    if isinstance(finding.subject, model.EdgeRef):
        return sorted({finding.subject.src, finding.subject.dst})
    return [finding.subject]


_DETAIL_DROP = ("id", "kind", "label", "layers", "profiles")


def _detail(atlas: Atlas) -> dict[str, Any]:
    entities: dict[str, Any] = {}
    for entity_id in sorted(atlas.entities):
        payload = atlas.entities[entity_id].to_json()
        for key in _DETAIL_DROP:
            payload.pop(key, None)
        entities[entity_id] = payload
    unresolved: dict[str, list[Any]] = {}
    for entry in atlas.unresolved:
        unresolved.setdefault(entry.owner, []).append(entry.to_json())
    return {"entities": entities, "unresolved": unresolved}


# ---------------------------------------------------------------------------
# assembly


def _compact(text: str) -> str:
    """Drop the indentation of `model.dumps` output.

    `model.dumps` writes with `ensure_ascii`, so a raw newline never occurs inside a
    string and the leading spaces of a line are always indentation. Nothing else is
    touched, so the result parses to the same document.
    """
    return "".join(line.lstrip(" ") for line in text.split("\n"))


def _embed(payload: Any) -> str:
    text = _compact(model.dumps(payload))
    return text.replace("<", "\\u003c")


def _asset(name: str) -> str:
    path = ASSETS / name
    try:
        return path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ContractError(f"missing render asset {name}: {exc}") from exc


def render(atlas: Atlas) -> str:
    """The whole page, as text. Same atlas in, same bytes out."""
    layout, radii = _layout(atlas)
    index = _index(atlas, layout, radii)
    detail = _detail(atlas)
    counts = index["counters"].get("findings") or {}
    summary = ", ".join(
        f"{counts.get(name, 0)} {name}" for name in ("error", "warn", "info") if counts.get(name)
    )
    head = (
        f"{len(index['entities']['id'])} entities, "
        f"{len(index['edges']) // 4} edges"
        f"{', ' + summary if summary else ''}"
    )
    return (
        "<!doctype html>\n"
        f"<!-- {rendered_from(atlas)} -->\n"
        '<html lang="en"><head><meta charset="utf-8">\n'
        '<meta name="viewport" content="width=device-width, initial-scale=1">\n'
        "<title>rosio atlas</title>\n"
        f"<style>\n{_asset('atlas.css')}</style>\n"
        "</head><body>\n"
        f"<noscript>This atlas needs javascript. {head}.</noscript>\n"
        f"{_asset('atlas.html')}"
        f'<script type="application/json" id="rosio-index">{_embed(index)}</script>\n'
        f"<script>\n{_asset('atlas.js')}</script>\n"
        f'<script type="application/json" id="rosio-detail">{_embed(detail)}</script>\n'
        "</body></html>\n"
    )


def write(atlas: Atlas, out_dir: str | Path) -> Path:
    """Write atlas.html into `out_dir` and return the path."""
    target = Path(out_dir) / HTML_NAME
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(render(atlas), encoding="utf-8")
    return target


__all__ = ["render", "write"]
