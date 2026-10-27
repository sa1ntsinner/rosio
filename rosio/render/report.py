"""ATLAS_REPORT.md: what this system is, from the atlas alone.

The order is deliberate. Coverage comes before the inventory, because a reader who does not
know that half the launched nodes had their source read will misread every count that
follows. The cross layer section comes last, because it is only meaningful once a reader
knows which layers spoke.

Nothing here reads a file or a clock, every collection is sorted, and no synthetic id
reaches the page: prose carries the label that `rosio.query` builds.
"""

from __future__ import annotations

from collections.abc import Iterable
from typing import Any

from rosio import ids, query
from rosio.model import LAYER_ORDER, Atlas, Candidate, Edge, Node
from rosio.render import clip, heading, joined, plural, rendered_from, share, table

ENDPOINT_KINDS = ("publishes", "subscribes", "serves", "calls")
YAML_MECHANISMS = ("yaml_base", "yaml_overlay_merge", "yaml_wildcard")
PARENT_FIRST = ("cpp_broadcast", "py_broadcast")
"""tf_parent sources whose edge runs from the parent to the child. The collectors do not
share one direction, so it is read from the fact that states the source, the same way
`rosio.drift.frames` reads it, and never assumed from the edge."""
TOP = 10
EXAMPLES = 3

LAYER_SOURCES = {
    "declared": "launch files, yaml profiles, urdf models, pluginlib manifests and the "
    "launcher entry points",
    "coded": "the C++ and python sources of the packages in the workspace",
    "observed": "a live system probed over rclpy",
    "recorded": "rosbag2 metadata and per topic statistics",
}


def render_report(atlas: Atlas) -> str:
    lines: list[str] = []
    lines.extend(_header(atlas))
    lines.extend(_coverage(atlas))
    lines.extend(_profiles(atlas))
    lines.extend(_inventory(atlas))
    lines.extend(_busiest(atlas))
    lines.extend(_parameters(atlas))
    lines.extend(_frames(atlas))
    lines.extend(_plugins(atlas))
    lines.extend(_layers(atlas))
    lines.extend(_blind_spots(atlas))
    lines.extend(_crossings(atlas))
    return "\n".join(lines).rstrip("\n") + "\n"


# ---------------------------------------------------------------------------
# shared shaping


def _profile_ids(atlas: Atlas) -> list[str]:
    return [item.id for item in atlas.by_kind("profile")]


def _slug(profile_id: str) -> str:
    return query.profile_name(profile_id)


def _nodes(atlas: Atlas) -> list[Node]:
    return atlas.by_kind("node")


def _in_profile(entity: Node, profile: str) -> bool:
    return profile in entity.profiles


def _read(entity: Node) -> bool:
    """Whether a layer that reads source code said anything about this entity."""
    return "coded" in entity.layers


def _value(entity: Node, key: str) -> Any:
    """One effective value of an entity, from the first profile that carries it."""
    for profile in sorted(entity.effective):
        row = entity.effective[profile].get(key)
        if row is not None:
            return row.value
    return None


def _fact_value(item: Node | Edge, key: str) -> Any:
    for fact in sorted(item.facts, key=lambda row: row.sort_key()):
        if key in fact.values:
            return fact.values[key]
    return None


def _edges_of_kind(atlas: Atlas, kind: str) -> list[Edge]:
    return [
        atlas.edges[handle] for handle in sorted(atlas.edges) if atlas.edges[handle].kind == kind
    ]


def _section(title: str, *, level: int = 2) -> list[str]:
    return ["", heading(level, title), ""]


def _percent(part: int, whole: int) -> str:
    return "unknown" if whole <= 0 else f"{round(100 * part / whole)} percent"


def _verb(count: int, singular: str, plural_form: str) -> str:
    return singular if count == 1 else plural_form


def _rank(counts: dict[str, int], limit: int = TOP) -> list[tuple[str, int]]:
    return sorted(counts.items(), key=lambda row: (-row[1], row[0]))[:limit]


# ---------------------------------------------------------------------------
# header and coverage


def _header(atlas: Atlas) -> list[str]:
    profiles = _profile_ids(atlas)
    present = [
        name for name in LAYER_ORDER if atlas.layers.get(name, None) and atlas.layers[name].present
    ]
    absent = [name for name in LAYER_ORDER if name not in present]
    counters = atlas.counters or {}
    lines = [heading(1, "Atlas report"), ""]
    lines.append(
        f"One ROS 2 system as {plural(len(present), 'layer')} of evidence describe it, "
        f"in {plural(len(profiles), 'profile')}: {joined(_slug(item) for item in profiles)}."
    )
    lines.append(
        f"{counters.get('entities', 0)} entities, {counters.get('edges', 0)} edges and "
        f"{len(atlas.findings)} findings, generated from atlas.json alone. "
        f"{rendered_from(atlas)}."
    )
    passes = counters.get("passes") or {}
    if int(passes.get("attempted") or 0) and not passes.get("complete", True):
        lines.append(
            f"This atlas is not whole. {plural(int(passes.get('refused') or 0), 'collector pass')} "
            f"did not finish ({joined(passes.get('refused_collectors') or [])}), so everything "
            "below is a lower bound."
        )
    if absent:
        names = absent[0] if len(absent) == 1 else ", ".join(absent[:-1]) + " and " + absent[-1]
        live = "observed" in present or "recorded" in present
        tail = (
            "so a claim below holds for the layers that spoke and for no other."
            if live
            else "so nothing below is a statement about a running system."
        )
        lines.append(
            f"The {names} {'layer is' if len(absent) == 1 else 'layers are'} not in this atlas, "
            f"{tail}"
        )
    return lines


def _completeness(atlas: Atlas) -> list[str]:
    """Whether every collector pass finished, stated before any count is read.

    A reader who believes an incomplete atlas is complete misreads everything below it, and
    the resolution rate cannot carry this: a pass that stopped takes its own denominator
    with it, so the rate reads best when the loss is worst.
    """
    passes = (atlas.counters or {}).get("passes") or {}
    attempted = int(passes.get("attempted") or 0)
    if not attempted:
        return []
    completed = int(passes.get("completed") or 0)
    skipped = int(passes.get("not_attempted") or 0)
    nothing = f", and {plural(skipped, 'pass', 'es')} had nothing to read" if skipped else ""
    if passes.get("complete", True):
        return [
            f"{completed} of {attempted} collector passes finished{nothing}, so nothing "
            "in this atlas was dropped on the way in."
        ]
    return [
        f"Only {completed} of {attempted} collector passes finished{nothing}. "
        f"{joined(passes.get('refused_collectors') or [])} stopped, so every count below is a "
        "lower bound, and the resolution rate below cannot show it: a pass that stopped "
        "contributes neither a resolved nor an unresolved site."
    ]


def _coverage(atlas: Atlas) -> list[str]:
    nodes = _nodes(atlas)
    read = [node for node in nodes if _read(node)]
    lines = _section("Coverage, read this before the counts")
    for paragraph in _completeness(atlas):
        lines.extend([paragraph, ""])
    lines.append(
        f"{share(len(read), len(nodes))} of the nodes this atlas names have their source read. "
        "Every endpoint, parameter default and frame below comes from that share, so an absent "
        "publisher is more often a package outside the workspace than a missing publisher."
    )
    rows = []
    for profile in _profile_ids(atlas):
        here = [node for node in nodes if _in_profile(node, profile)]
        rows.append(
            [
                _slug(profile),
                len(here),
                len([node for node in here if _read(node)]),
                _percent(len([node for node in here if _read(node)]), len(here)),
            ]
        )
    lines.append("")
    lines.extend(table(["profile", "nodes launched", "source read", "share"], rows))
    join = (atlas.x or {}).get("join") or {}
    if join:
        lines.append("")
        lines.append(
            f"The join bound {join.get('endpoints_bound', 0)} code endpoints to a launched node, "
            f"{join.get('endpoints_remapped', 0)} of them through a launch remap, and left "
            f"{join.get('endpoints_unresolved', 0)} unbound. "
            f"{join.get('wrapper_endpoints', 0)} endpoints sit behind a known wrapper and are "
            "recorded rather than reported."
        )
    rates = (atlas.counters or {}).get("resolution_rate") or {}
    if rates:
        lines.append("")
        lines.append(
            f"{plural(len(atlas.unresolved), 'value')} could not be resolved and none of them was "
            "guessed. The rate per want is in the blind spots section."
        )
    return lines


# ---------------------------------------------------------------------------
# profiles and inventory


def _profiles(atlas: Atlas) -> list[str]:
    lines = _section("Profiles and the entry points that build them")
    lines.append(
        "A profile is one running system, named by the ordered commands that start it. A union "
        "of launch files is not a profile, because it describes a system nobody runs."
    )
    lines.append("")
    for profile_id in _profile_ids(atlas):
        entity = atlas.entities[profile_id]
        entries = _value(entity, "entrypoints") or []
        launch_files = _value(entity, "launch_files") or []
        lines.append(heading(3, _slug(profile_id)))
        if entity.label and entity.label != _slug(profile_id):
            lines.append(entity.label)
        lines.append("")
        here = [node for node in _nodes(atlas) if _in_profile(node, profile_id)]
        lines.append(
            f"{plural(len(entries), 'entry point')}, {plural(len(launch_files), 'launch file')}, "
            f"{plural(len(here), 'node')}."
        )
        lines.append("")
        for command in entries:
            lines.append(f"    {command}")
        missing = _value(entity, "missing_targets") or []
        if missing:
            lines.append("")
            lines.append(
                f"Entry points that named a target rosio could not find: {joined(missing)}."
            )
        lines.append("")
    return lines


def _inventory(atlas: Atlas) -> list[str]:
    nodes = _nodes(atlas)
    lines = _section("Node inventory")
    lines.append(
        "One row per node, with the profiles that launch it and whether its source was read. "
        "`name source` says where the runtime name comes from, and a randomised name is a node "
        "that cannot be addressed by name at all."
    )
    rows = []
    for node in nodes:
        pkg = _value(node, "pkg") or ids.parse_node(node.id).package or ""
        executable = _value(node, "executable") or ids.parse_node(node.id).executable or ""
        rows.append(
            [
                query.label(node.id),
                f"{pkg}/{executable}".strip("/"),
                joined((_slug(item) for item in node.profiles), empty="none"),
                "yes" if _read(node) else "no",
                node.name_source or "unknown",
            ]
        )
    lines.append("")
    lines.extend(
        table(["node", "package and executable", "profiles", "source read", "name source"], rows)
    )
    return lines


def _busiest(atlas: Atlas) -> list[str]:
    publishes: dict[str, int] = {}
    subscribes: dict[str, int] = {}
    other: dict[str, int] = {}
    # A node states one topic from several call sites, so a node column counts sites and a
    # topic column counts nodes. Counting sites under a heading that says publishers reports
    # three publishers of /wheel/speed where one node publishes it three times.
    topic_pub: dict[str, set[str]] = {}
    topic_sub: dict[str, set[str]] = {}
    for kind in ENDPOINT_KINDS:
        for edge in _edges_of_kind(atlas, kind):
            if kind == "publishes":
                publishes[edge.src] = publishes.get(edge.src, 0) + 1
                topic_pub.setdefault(edge.dst, set()).add(edge.src)
            elif kind == "subscribes":
                subscribes[edge.src] = subscribes.get(edge.src, 0) + 1
                topic_sub.setdefault(edge.dst, set()).add(edge.src)
            else:
                other[edge.src] = other.get(edge.src, 0) + 1
    totals: dict[str, int] = {}
    for table_of in (publishes, subscribes, other):
        for key, count in table_of.items():
            totals[key] = totals.get(key, 0) + count
    lines = _section("Busiest nodes and topics")
    lines.append(
        "Counted over the endpoints the source layer states, so a node whose package is outside "
        "the workspace sits at zero and is not quiet. A node row counts its own call sites and a "
        "topic row counts the nodes on the other end."
    )
    rows = [
        [
            query.label(node_id),
            publishes.get(node_id, 0),
            subscribes.get(node_id, 0),
            other.get(node_id, 0),
        ]
        for node_id, _count in _rank(totals)
    ]
    lines.append("")
    lines.extend(table(["node", "publishes", "subscribes", "services and actions"], rows))
    topics: dict[str, int] = {}
    for key in set(topic_pub) | set(topic_sub):
        topics[key] = len(topic_pub.get(key, ())) + len(topic_sub.get(key, ()))
    rows = [
        [query.label(topic_id), len(topic_pub.get(topic_id, ())), len(topic_sub.get(topic_id, ()))]
        for topic_id, _count in _rank(topics)
    ]
    lines.append("")
    lines.extend(table(["topic", "publishers", "subscribers"], rows))
    orphans = [
        entity.id
        for entity in atlas.by_kind("topic")
        if entity.id not in topic_pub and entity.id not in topic_sub
    ]
    if orphans:
        lines.append("")
        lines.append(
            f"{plural(len(orphans), 'topic')} carry a name and no endpoint edge at all. They are "
            "named by a remap or a parameter, and the code that would publish or subscribe them "
            "is outside the read set."
        )
    return lines


# ---------------------------------------------------------------------------
# parameters


def _params(atlas: Atlas) -> list[Node]:
    return atlas.by_kind("param") + atlas.by_kind("pparam")


def _parameters(atlas: Atlas) -> list[str]:
    params = _params(atlas)
    by_state: dict[str, int] = {}
    for entity in params:
        by_state[str(entity.state)] = by_state.get(str(entity.state), 0) + 1
    effective = by_state.get("effective", 0)
    lines = _section("Parameter surface")
    lines.append(
        f"{plural(len(params), 'parameter')} {_verb(len(params), 'is', 'are')} configured "
        "somewhere in this workspace. "
        f"{share(effective, len(params))} resolve to a value that a profile actually reads. The "
        f"rest are dead: the block belongs to no launched node, the plugin that owns the key is "
        "not the selected one, or the chain could not be followed to a winner."
    )
    lines.append("")
    lines.extend(
        table(
            ["state", "parameters", "meaning"],
            [[state, by_state[state], _STATE_TEXT.get(state, "")] for state in sorted(by_state)],
        )
    )
    rows = []
    for profile in _profile_ids(atlas):
        here = [entity for entity in params if _in_profile(entity, profile)]
        won = [entity for entity in here if profile in entity.effective]
        rows.append([_slug(profile), len(here), len(won), len(here) - len(won)])
    lines.append("")
    lines.extend(table(["profile", "parameters set", "effective", "dead in this profile"], rows))
    mechanisms: dict[str, list[int]] = {}
    shadows: dict[str, int] = {}
    for entity in params:
        for candidate in entity.candidates:
            row = mechanisms.setdefault(candidate.mechanism, [0, 0])
            row[0] += 1
            row[1] += 1 if candidate.wins else 0
            if candidate.shadow_kind:
                shadows[candidate.shadow_kind] = shadows.get(candidate.shadow_kind, 0) + 1
    lines.append("")
    lines.append(
        "Every value that was written anywhere is a candidate, and one candidate per profile "
        "wins. The mechanisms below are how a value reaches a node, in the order a reader would "
        "not expect: a python launcher and an inline launch dict both outrank the yaml file that "
        "spells the same key."
    )
    lines.append("")
    lines.extend(
        table(
            ["mechanism", "candidates", "wins"],
            [[name, row[0], row[1]] for name, row in sorted(mechanisms.items())],
        )
    )
    if shadows:
        lines.append("")
        lines.append(
            f"{sum(shadows.values())} candidates lose to another: "
            + joined(f"{name} {count}" for name, count in sorted(shadows.items()))
            + "."
        )
    return lines


_STATE_TEXT = {
    "effective": "read by the node that owns the name",
    "inert_by_design": "the block belongs to a plugin the profile does not select",
    "undeclared": "set in configuration and never declared in code",
    "declared_unread": "declared in code and never read",
    "owner_name_mismatch": "the block key matches no launched node",
    "unresolved_owner": "the owning node could not be resolved",
    "other": "the chain could not be followed to a winner",
}


# ---------------------------------------------------------------------------
# frames


def _link(edge: Edge) -> tuple[str, str]:
    """The child and the parent one tf_parent edge states, in that order."""
    for fact in sorted(edge.facts, key=lambda row: row.sort_key()):
        source = str(fact.values.get("strategy") or fact.values.get("source") or "")
        if source in PARENT_FIRST:
            return edge.dst, edge.src
        if source:
            break
    return edge.src, edge.dst


def _frames(atlas: Atlas) -> list[str]:
    edges = _edges_of_kind(atlas, "tf_parent")
    parents: dict[str, set[str]] = {}
    children: dict[str, set[str]] = {}
    for edge in edges:
        child, parent = _link(edge)
        parents.setdefault(child, set()).add(parent)
        children.setdefault(parent, set()).add(child)
    frames = atlas.by_kind("frame")
    connected = set(parents) | set(children)
    roots = sorted(frame for frame in connected if frame not in parents)
    lines = _section("Frames")
    lines.append(
        f"{plural(len(frames), 'frame')} are named in this atlas and "
        f"{plural(len(edges), 'transform')} state a parent. The tree below is every frame some "
        "transform names, from each root down."
    )
    lines.append("")
    for root in roots:
        lines.extend(_frame_tree(root, children, prefix="", seen=()))
    isolated = sorted(frame.id for frame in frames if frame.id not in connected)
    if isolated:
        lines.append("")
        lines.append(
            f"{plural(len(isolated), 'frame')} {_verb(len(isolated), 'is', 'are')} named by a "
            "parameter or a source literal, and no transform reaches "
            f"{_verb(len(isolated), 'it', 'them')}: "
            + joined(clip(query.label(item), 40) for item in isolated)
            + "."
        )
    forked = sorted(frame for frame, above in parents.items() if len(above) > 1)
    if forked:
        lines.append("")
        lines.append(
            f"{plural(len(forked), 'frame')} {_verb(len(forked), 'hangs', 'hang')} off more than "
            "one parent, which is two robot models describing two different vehicles rather than "
            "one tree: " + joined(query.label(item) for item in forked) + "."
        )
    return lines


def _frame_tree(
    frame_id: str,
    children: dict[str, set[str]],
    *,
    prefix: str,
    seen: tuple[str, ...],
) -> list[str]:
    if frame_id in seen:
        return [f"    {prefix}{query.label(frame_id)} (already above, the transforms form a loop)"]
    lines = [f"    {prefix}{query.label(frame_id)}"]
    for child in sorted(children.get(frame_id, ())):
        lines.extend(_frame_tree(child, children, prefix=prefix + "  ", seen=(*seen, frame_id)))
    return lines


# ---------------------------------------------------------------------------
# plugins


def _instance_text(pinst_id: str) -> str:
    owner, instance = ids.parse_pinst(pinst_id)
    return f"{query.label(owner)} {instance}"


def _plugins(atlas: Atlas) -> list[str]:
    plugins = atlas.by_kind("plugin")
    exported: dict[str, list[str]] = {}
    for edge in _edges_of_kind(atlas, "provides_plugin"):
        exported.setdefault(edge.dst, []).append(edge.src)
    selected: dict[str, list[str]] = {}
    for edge in _edges_of_kind(atlas, "instantiates"):
        selected.setdefault(edge.dst, []).append(edge.src)
    lines = _section("Plugins")
    lines.append(
        "A plugin is in play when a profile selects it by lookup name. The instances column "
        "carries the node and the block that selects it, which is what decides whether the "
        "parameters under that block are live."
    )
    rows = []
    for entity in plugins:
        parsed = ids.parse_plugin(entity.id)
        instances = sorted(selected.get(entity.id, []))
        rows.append(
            [
                parsed.lookup,
                parsed.base_class,
                joined(
                    (ids.parse_pkg(item) for item in sorted(exported.get(entity.id, []))),
                    "not in this workspace",
                ),
                joined((_instance_text(item) for item in instances), "nothing selects it"),
                joined((_slug(item) for item in entity.profiles), "none"),
            ]
        )
    lines.append("")
    lines.extend(
        table(["lookup name", "base class", "exported by", "selected by", "profiles"], rows)
    )
    return lines


# ---------------------------------------------------------------------------
# layers and blind spots


def _layers(atlas: Atlas) -> list[str]:
    entities: dict[str, dict[str, int]] = {}
    for entity in atlas.entities.values():
        for layer in entity.layers:
            entities.setdefault(layer, {})[entity.kind] = (
                entities.setdefault(layer, {}).get(entity.kind, 0) + 1
            )
    edges: dict[str, int] = {}
    for edge in atlas.edges.values():
        for layer in edge.layers:
            edges[layer] = edges.get(layer, 0) + 1
    lines = _section("Layers and what each one contributed")
    rows = []
    for name in LAYER_ORDER:
        status = atlas.layers.get(name)
        present = bool(status and status.present)
        kinds = entities.get(name, {})
        rows.append(
            [
                name,
                "yes" if present else "no",
                len(status.runs) if status else 0,
                sum(kinds.values()),
                edges.get(name, 0),
                joined((f"{kind} {count}" for kind, count in _rank(kinds, 5)), "nothing"),
            ]
        )
    lines.append("")
    lines.extend(table(["layer", "present", "runs", "entities", "edges", "largest kinds"], rows))
    lines.append("")
    for name in LAYER_ORDER:
        status = atlas.layers.get(name)
        if status and status.present:
            lines.append(f"* {name}: {LAYER_SOURCES.get(name, 'unstated')}")
    only = (atlas.indexes or {}).get("layer_only") or {}
    if only:
        lines.append("")
        lines.append(
            "Entities only one layer ever mentioned: "
            + joined(f"{name} {len(value)}" for name, value in sorted(only.items()))
            + ". That is coverage and not disagreement, which is why no rule reports it."
        )
    return lines


def _cause(reason: str) -> str:
    """The grouping key for a blind spot.

    A reason is either a short code or a sentence that names the value it failed on. Codes
    group as they are, and a sentence groups by its opening, so thirty remaps that differ
    only in a topic name are one cause and not thirty.
    """
    words = reason.split()
    return reason if len(words) <= 2 else " ".join(words[:4]) + " ..."


def _blind_spots(atlas: Atlas) -> list[str]:
    grouped: dict[str, dict[str, Any]] = {}
    for entry in sorted(atlas.unresolved, key=lambda item: item.id):
        row = grouped.setdefault(
            _cause(entry.reason),
            {"count": 0, "wants": set(), "reason": entry.reason, "example": entry},
        )
        row["count"] += 1
        row["wants"].add(entry.want)
    lines = _section("Blind spots, grouped by cause")
    rates = (atlas.counters or {}).get("resolution_rate") or {}
    lines.append(
        f"{plural(len(atlas.unresolved), 'value')} could not be resolved. None of them was "
        "guessed, and none of them became an entity, so nothing in this report is a placeholder."
    )
    rows = [
        [
            cause,
            row["count"],
            joined(sorted(row["wants"])),
            clip(_example(row["example"], atlas), 80),
        ]
        for cause, row in sorted(grouped.items(), key=lambda item: (-item[1]["count"], item[0]))
    ]
    lines.append("")
    lines.extend(table(["cause", "count", "what was wanted", "one example"], rows))
    if rates:
        lines.append("")
        lines.extend(
            table(
                ["want", "resolution rate"],
                [[name, rates[name]] for name in sorted(rates)],
            )
        )
    return lines


# ---------------------------------------------------------------------------
# the cross layer section, which is what this tool exists for


def _example(entry, atlas: Atlas) -> str:
    return f"{query.readable(entry.expr, atlas)} in {query.label(entry.owner)}"


def _crossings(atlas: Atlas) -> list[str]:
    lines = _section("What no single layer states")
    lines.append(
        "Each statement below needs two sources at once. No launch file, no yaml file and no "
        "source file carries any of them on its own, which is the reason this atlas exists."
    )
    for block in (
        _crossing_overridden(atlas),
        _crossing_remapped(atlas),
        _crossing_code_only(atlas),
        _crossing_dead_config(atlas),
        _crossing_unused_plugins(atlas),
        _crossing_unnamed(atlas),
    ):
        if block:
            lines.append("")
            lines.extend(block)
    return lines


def _claim(text: str, examples: Iterable[str]) -> list[str]:
    lines = [f"* {text}"]
    for item in list(examples)[:EXAMPLES]:
        lines.append(f"    {item}")
    return lines


def _winner(entity: Node, profile: str) -> Candidate | None:
    for candidate in sorted(entity.candidates, key=Candidate.sort_key):
        if candidate.profile == profile and candidate.wins:
            return candidate
    return None


def _crossing_overridden(atlas: Atlas) -> list[str]:
    hits: list[tuple[str, str]] = []
    for entity in _params(atlas):
        for profile in sorted(entity.effective):
            winner = _winner(entity, profile)
            if winner is None or winner.mechanism in YAML_MECHANISMS:
                continue
            losers = [
                candidate
                for candidate in sorted(entity.candidates, key=Candidate.sort_key)
                if candidate.profile == profile
                and not candidate.wins
                and candidate.mechanism in YAML_MECHANISMS
            ]
            if not losers:
                continue
            hits.append(
                (
                    entity.id,
                    f"{query.label(entity.id)} in {_slug(profile)} is "
                    f"{clip(repr(winner.value), 48)} from {query.site(winner.source)} "
                    f"({winner.mechanism}), not the value at {query.site(losers[0].source)}",
                )
            )
    if not hits:
        return []
    return _claim(
        f"{plural(len(hits), 'configured value')} {_verb(len(hits), 'is', 'are')} decided "
        "outside the yaml file that spells them, by a "
        "launch argument, an inline dict or a python launcher. Reading the yaml alone gives the "
        "wrong number, and reading the launcher alone does not show which key it lands on.",
        (text for _key, text in sorted(hits)),
    )


def _crossing_remapped(atlas: Atlas) -> list[str]:
    hits: list[str] = []
    for kind in ("publishes", "subscribes"):
        for edge in _edges_of_kind(atlas, kind):
            raw = _fact_value(edge, "raw_name")
            if not raw or not _fact_value(edge, "remapped"):
                continue
            hits.append(
                f"{query.label(edge.src)} {kind} {raw!r} in its source and launch binds it to "
                f"{query.label(edge.dst)}"
            )
    if not hits:
        return []
    distinct = sorted(dict.fromkeys(hits))
    spread = "" if len(distinct) == len(hits) else f", {len(distinct)} of them distinct"
    return _claim(
        f"{plural(len(hits), 'endpoint')} {_verb(len(hits), 'carries', 'carry')} one name in the "
        f"code and another at run time{spread}. The source "
        "states a relative name and the launch file states the remap, so neither file names the "
        "topic that actually carries the data.",
        distinct,
    )


def _crossing_code_only(atlas: Atlas) -> list[str]:
    hits: list[str] = []
    for entity in _params(atlas):
        chain = sorted(entity.candidates, key=Candidate.sort_key)
        if not chain or any(candidate.mechanism != "code_default" for candidate in chain):
            continue
        # An empty value_type is the collector refusing to state a default, and a loser reaches
        # no node, so neither one names a value that runs.
        stated = [row for row in chain if row.wins and row.value_type]
        if not stated:
            continue
        hits.append(
            f"{query.label(entity.id)} defaults to {clip(repr(stated[0].value), 40)} "
            f"at {query.site(stated[0].source)}"
        )
    if not hits:
        return []
    return _claim(
        f"{plural(len(hits), 'parameter')} {_verb(len(hits), 'runs', 'run')} on the default "
        f"written in {_verb(len(hits), 'its', 'their')} own source and no configuration file "
        f"mentions {_verb(len(hits), 'it', 'them')}. Each one is tunable and undocumented at the "
        "same time.",
        sorted(hits),
    )


def _crossing_dead_config(atlas: Atlas) -> list[str]:
    dead = [
        entity
        for entity in _params(atlas)
        if entity.state in ("owner_name_mismatch", "unresolved_owner", "inert_by_design")
    ]
    if not dead:
        return []
    files: dict[str, int] = {}
    for entity in dead:
        for candidate in entity.candidates:
            where = query.file_label(candidate.source.file)
            files[where] = files.get(where, 0) + 1
    return _claim(
        f"{plural(len(dead), 'configured key')} {_verb(len(dead), 'reaches', 'reach')} no node in "
        "any profile. Deciding that needs the yaml "
        "block on one side and the launched node names on the other, and neither file states the "
        "other.",
        (f"{plural(count, 'key')} in {where}" for where, count in _rank(files, EXAMPLES)),
    )


def _crossing_unused_plugins(atlas: Atlas) -> list[str]:
    selected = {edge.dst for edge in _edges_of_kind(atlas, "instantiates")}
    exported = {edge.dst: edge.src for edge in _edges_of_kind(atlas, "provides_plugin")}
    unused = sorted(item for item in exported if item not in selected)
    if not unused:
        return []
    return _claim(
        f"{plural(len(unused), 'plugin')} {_verb(len(unused), 'is', 'are')} built and exported by "
        f"a package in this workspace and no profile selects {_verb(len(unused), 'it', 'them')}. "
        "The manifest states the export and the yaml states the selection, "
        "and only the two together say the class is never loaded.",
        (
            f"{ids.parse_plugin(item).lookup} exported by {ids.parse_pkg(exported[item])}"
            for item in unused
        ),
    )


def _crossing_unnamed(atlas: Atlas) -> list[str]:
    rows = [
        node
        for node in _nodes(atlas)
        if node.name_source in ("runtime_randomised", "unknown", "executable_default")
    ]
    if not rows:
        return []
    return _claim(
        f"{plural(len(rows), 'node')} {_verb(len(rows), 'carries', 'carry')} a runtime name that "
        "no file in the repository spells. The name comes from an executable default or from the "
        "runtime itself, so a parameter block "
        "keyed on a guessed name silently configures nothing.",
        (f"{query.label(node.id)}, name from {node.name_source}" for node in rows),
    )


__all__ = ["render_report"]
