"""Graph rules over endpoints: the topic, qos, type, rate and endpoint families.

Six rules live here: `topic.no_subscriber`, `topic.no_publisher`, `qos.differ`,
`type.differ`, `rate.below_declared` and `endpoint.observed_only`. Each is a pure function
from the joined atlas to a list of findings, and the shared reading helpers come from
`rosio.drift.nodes` so there is one implementation of profile scoping and fact selection.

When each rule may speak, which is the whole design here
--------------------------------------------------------

A missing endpoint is only a finding when the atlas was in a position to see it. On the
reference stack the coded layer covers 5 of 56 launched node instances, so a topic with a
publisher and no subscriber almost always means the subscriber's source was never read:
`/lidar/points_filtered` is consumed by `pointcloud_to_laserscan` and by the costmap
layers, and `/odometry/lidar` by a `robot_localization` node, none of whose sources are
in the workspace. 29 of the 57 topic and service entities carry no endpoint edge at all,
minted from a remap destination or from an absolute literal in a symbol that binds to no
executable. A rule that counts edges would report all of it.

So the two connectivity rules require the observed layer for the profile they speak about,
because a probe of a running system enumerates every endpoint on every node and is the only
source that can say a topic has no subscriber rather than that rosio did not find one.
They further require that the surviving side was itself observed, and that the probe stated
the endpoints of every node it reported: one node listed without its endpoints could be
carrying the missing subscriber, so it silences both rules for that profile. Until the
observed layer lands both return an empty list, which is the honest answer and not a gap.

`endpoint.observed_only` needs the observed layer by definition, and additionally requires
that the node's source was read in that profile: an endpoint that only the probe knows about
is drift when the source is in the read set and is coverage otherwise.

`qos.differ` and `type.differ` compare only what two layers both stated, for one edge, in
one profile. A QoS nobody stated is unknown and never a default: a null property is skipped,
a preset name is not expanded into numbers, and a type that does not parse into a package
and a message name is not compared. Both rules are written against any pair of layers, so
they do not name the observed layer anywhere, and on an atlas where only one layer states an
endpoint they produce nothing.

Endpoints marked `via_known_wrapper` are never a subject of any rule here, from the table in
`rosio/resolve/wrappers.py`. The names that table states, `/tf`, `/tf_static`,
`/diagnostics`, `/rosout`, `/parameter_events` and the parameter services, are skipped as
subjects as well: they are created by a library on the node's behalf and their two ends are
never both in the source. Not being a subject is not the same as not existing: the two
connectivity rules still count a wrapper endpoint as an end, because a `tf2_ros` listener or
a `message_filters` subscription is a real consumer, and dropping it would turn the publisher
it consumes from into a dead topic.

The observed contract this reads
--------------------------------

One fact per endpoint edge per profile, layer `observed`, values optionally `qos` (the same
shape the coded layer produces: `history`, `depth`, `reliability`, `durability`, with null
for a property nobody stated), `type`, and `volatile.rate_hz` for a publisher. The measured
rate stays out of every `detail`, so a fingerprint does not move between two probes of the
same healthy system.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from rosio import ids, model
from rosio.drift import rules
from rosio.drift.nodes import (
    CODED,
    DECLARED,
    OBSERVED,
    disc_for,
    evidence_of,
    human,
    is_tooling,
    observed_profiles,
    runtime_name,
    says_anything,
    slug,
    stated,
)
from rosio.errors import ContractError
from rosio.resolve import qos_presets, wrappers

TOPIC = model.NodeKind.TOPIC.value
PUBLISHES = model.EdgeKind.PUBLISHES.value
SUBSCRIBES = model.EdgeKind.SUBSCRIBES.value

ENDPOINT_KINDS = (
    PUBLISHES,
    SUBSCRIBES,
    model.EdgeKind.SERVES.value,
    model.EdgeKind.CALLS.value,
)

QOS_KEYS = ("history", "depth", "reliability", "durability")
"""The properties two layers can compare. `raw`, `form`, `variable` and `preset` are how one
side spelled it, not what it is, and expanding a preset into numbers would be inventing the
rmw defaults this product refuses to invent."""

RATE_TOLERANCE = 0.2
"""How far under the declared rate a measurement must fall before it is reported. A timer
that states 20 Hz and measures 19 Hz is a scheduler, not drift."""


# ---------------------------------------------------------------------------
# reading the graph


@dataclass(frozen=True)
class Coverage:
    """What the atlas could see of one profile. The connectivity rules read `observed`, and
    the rest is what a report cites when a rule stays silent."""

    profile: str
    nodes: int
    nodes_with_code: int
    observed: bool

    @property
    def coded_fraction(self) -> float:
        return 0.0 if not self.nodes else self.nodes_with_code / self.nodes


def coverage(atlas: model.Atlas, profile: str) -> Coverage:
    seen = observed_profiles(atlas)
    nodes = 0
    with_code = 0
    for entity in atlas.by_kind(model.NodeKind.NODE.value):
        if not says_anything(entity, DECLARED, profile):
            continue
        nodes += 1
        if says_anything(entity, CODED, profile):
            with_code += 1
    return Coverage(
        profile=profile, nodes=nodes, nodes_with_code=with_code, observed=profile in seen
    )


def _table_names() -> tuple[frozenset[str], frozenset[str]]:
    absolute: set[str] = set()
    private: set[str] = set()
    rows = list(wrappers.implicit_endpoints())
    for symbol in wrappers.symbols():
        rows.extend(wrappers.endpoints_for(symbol))
    for _kind, name, _type in rows:
        if name.startswith("/"):
            absolute.add(name)
        elif name.startswith("~/"):
            private.add(name[1:])
    return frozenset(absolute), frozenset(private)


def quiet_names() -> frozenset[str]:
    """Absolute endpoint names a library creates on a node's behalf, from the wrapper
    table. Both ends of these are inside a library, so neither end is ever in a source."""
    return _table_names()[0]


def quiet_suffixes() -> frozenset[str]:
    """The same table's private names, as the tail they resolve to under any namespace.

    `~/get_parameters` on `/controller_server` runs as `/controller_server/get_parameters`, so
    an absolute name set never matches it, and the six parameter services rclcpp and rclpy
    create on every node would each become a finding as soon as a probe enumerates them.
    """
    return _table_names()[1]


def is_quiet(name: str) -> bool:
    """Whether an endpoint name belongs to a library rather than to this workspace."""
    return name in quiet_names() or any(name.endswith(tail) for tail in quiet_suffixes())


LIFECYCLE_TAILS = frozenset(
    {
        "change_state",
        "get_state",
        "get_available_states",
        "get_available_transitions",
        "get_transition_graph",
        "transition_event",
    }
)
"""What rcl_lifecycle creates under every lifecycle node, with no call in its source."""


def lifecycle_node(entity: model.Node | None) -> bool:
    return entity is not None and any(
        fact.values.get("form") == "lifecycle_node" or fact.values.get("lifecycle_state")
        for fact in entity.facts
    )


def library_channel(name: str, owner: model.Node | None) -> bool:
    """A name the client library makes for a node: the parameter services, the services and
    topics behind an action (`/dock/_action/send_goal`), whose action is the endpoint the
    source states, and the lifecycle services of a lifecycle node."""
    if is_quiet(name) or "/_action/" in name:
        return True
    parent, _, tail = name.rpartition("/")
    if tail not in LIFECYCLE_TAILS or owner is None or ids.is_synthetic(owner.id):
        return False
    return parent == ids.parse_node(owner.id).fqn and lifecycle_node(owner)


def endpoint_edges(atlas: model.Atlas, kinds: tuple[str, ...] = ENDPOINT_KINDS) -> list[model.Edge]:
    return [
        atlas.edges[handle] for handle in sorted(atlas.edges) if atlas.edges[handle].kind in kinds
    ]


def _via_wrapper(edge: model.Edge, profile: str) -> bool:
    for fact in edge.facts:
        if fact.profile not in (profile, model.ANY_PROFILE):
            continue
        if fact.values.get("via_known_wrapper"):
            return True
    return bool(edge.attrs.get("via_known_wrapper"))


def _ref(edge: model.Edge) -> model.EdgeRef:
    return model.EdgeRef(kind=edge.kind, src=edge.src, dst=edge.dst, key=edge.key)


def _layer_facts(edge: model.Edge, profile: str) -> dict[str, model.LayerFact]:
    out: dict[str, model.LayerFact] = {}
    for layer in model.LAYER_ORDER:
        fact = stated(edge, layer, profile)
        if fact is not None:
            out[layer] = fact
    return out


def _sites(atlas: model.Atlas, kinds: tuple[str, ...] = ENDPOINT_KINDS) -> list[list[model.Edge]]:
    """Endpoint edges grouped by node, kind and channel."""
    out: dict[tuple[str, str, str], list[model.Edge]] = {}
    for edge in endpoint_edges(atlas, kinds):
        out.setdefault((edge.kind, edge.src, edge.dst), []).append(edge)
    return [out[key] for key in sorted(out)]


def _views(edges: list[model.Edge]) -> list[tuple[model.Edge, str, dict[str, model.LayerFact]]]:
    """(subject edge, profile, facts per layer) for one endpoint as the layers state it.

    The probe keys an endpoint by its name and the join by the source site, so one endpoint
    two layers both state sits on two edges. Where each layer states it on one edge at most,
    those edges are one endpoint and are compared as one, cited at the edge a source states;
    a layer stating two sites keeps every edge apart, because then the sites differ.
    """
    out = []
    profiles = sorted({p for edge in edges for p in edge.profiles} or {model.ANY_PROFILE})
    for profile in profiles:
        each = [(edge, _layer_facts(edge, profile)) for edge in edges]
        each = [(edge, facts) for edge, facts in each if facts]
        counts: dict[str, int] = {}
        for _edge, facts in each:
            for layer in facts:
                counts[layer] = counts.get(layer, 0) + 1
        if len(each) > 1 and all(count == 1 for count in counts.values()):
            merged: dict[str, model.LayerFact] = {}
            for _edge, facts in each:
                merged.update(facts)
            subject = next((edge for edge, facts in each if set(facts) - {OBSERVED}), each[0][0])
            out.append((subject, profile, merged))
        else:
            out.extend(
                (edge, profile, facts)
                for edge, facts in each
                if profile in (set(edge.profiles) or {model.ANY_PROFILE})
            )
    return out


# ---------------------------------------------------------------------------
# topic connectivity


def probe_enumerated_endpoints(atlas: model.Atlas, profile: str) -> bool:
    """Whether every node the probe reported also has its endpoints in the atlas.

    This is the completeness the connectivity rules stand on. A probe that lists nodes and
    not their endpoints leaves the missing subscriber on any node it did not enumerate, and
    the two rules would report the gap as a dead topic. One node without endpoints silences
    both rules for the profile, which is the difference between a graph and a sample.
    """
    owners = {
        edge.src
        for edge in endpoint_edges(atlas)
        for fact in edge.facts
        if fact.layer == OBSERVED and fact.profile in (profile, model.ANY_PROFILE)
    }
    for entity in atlas.by_kind(model.NodeKind.NODE.value):
        fact = stated(entity, OBSERVED, profile)
        if fact is None or fact.values.get("via_known_wrapper") or fact.values.get("internal"):
            continue
        if is_tooling(*runtime_name(entity, fact)):
            continue
        if entity.id not in owners:
            return False
    return True


Sides = tuple[dict[str, list[model.Edge]], dict[str, list[model.Edge]], dict[str, set[str]]]


def _states_endpoint(edge: model.Edge, profile: str) -> bool:
    """Whether any layer filed a usable fact for this endpoint in this profile.

    Deliberately not `stated`: a wrapper endpoint is not a subject, and it is still an end.
    """
    unknown = model.Confidence.UNKNOWN.value
    return any(
        fact.profile in (profile, model.ANY_PROFILE) and fact.confidence != unknown
        for fact in edge.facts
    )


def _sides(atlas: model.Atlas, profile: str) -> Sides:
    """The endpoints a rule may report, and separately every end that exists.

    A `via_known_wrapper` endpoint is never a subject, per AGENTS section 6, and erasing it
    from the graph as well would turn the sixteen `tf2_ros` listeners and the two
    `message_filters` PointCloud2 subscriptions on this stack into dead publishers. The
    wrapper end proves the other side exists even though nothing may be said about it.
    """
    publishers: dict[str, list[model.Edge]] = {}
    subscribers: dict[str, list[model.Edge]] = {}
    ends: dict[str, set[str]] = {PUBLISHES: set(), SUBSCRIBES: set()}
    for edge in endpoint_edges(atlas, (PUBLISHES, SUBSCRIBES)):
        if ids.kind_of(edge.dst) != TOPIC or not _states_endpoint(edge, profile):
            continue
        ends[edge.kind].add(edge.dst)
        if _via_wrapper(edge, profile):
            continue
        if not any(stated(edge, layer, profile) for layer in model.LAYER_ORDER):
            continue
        target = publishers if edge.kind == PUBLISHES else subscribers
        target.setdefault(edge.dst, []).append(edge)
    return publishers, subscribers, ends


def _lonely(
    atlas: model.Atlas,
    *,
    rule: str,
    present_is_publisher: bool,
) -> list[model.Finding]:
    out: list[model.Finding] = []
    for profile in observed_profiles(atlas):
        if not probe_enumerated_endpoints(atlas, profile):
            continue
        publishers, subscribers, ends = _sides(atlas, profile)
        present, missing_kind = (
            (publishers, SUBSCRIBES) if present_is_publisher else (subscribers, PUBLISHES)
        )
        for topic_id in sorted(present):
            name = ids.parse_topic(topic_id)
            if library_channel(name, None) or topic_id in ends[missing_kind]:
                continue
            live = [edge for edge in present[topic_id] if stated(edge, OBSERVED, profile)]
            if not live:
                continue
            extra_where: list[model.Source] = []
            owners = sorted({human(edge.src) for edge in live})
            who = ", ".join(owners)
            where = slug(profile)
            if present_is_publisher:
                title = f"{name} is published and nothing subscribes it"
                detail = (
                    f"{who} publishes {name} in {where}, and the probe of the running system "
                    "found no subscription to it on any node"
                )
                say = (
                    f"{who} publishes {name} in {where} and no node subscribes it, so the "
                    "messages are produced and dropped"
                )
                fix = (
                    "remap the subscriber onto this name, or stop publishing it if nothing needs it"
                )
            else:
                title = f"{name} is subscribed and nothing publishes it"
                detail = (
                    f"{who} subscribes {name} in {where}, and the probe of the running system "
                    "found no publisher of it on any node"
                )
                say = (
                    f"{who} waits for {name} in {where} and no node publishes it, so that input "
                    "never arrives"
                )
                fix = (
                    "remap the publisher onto this name, or point the subscriber at the name "
                    "that is actually published"
                )
                onto = _remaps_onto(atlas, name, profile)
                if onto:
                    rules_text = "; ".join(
                        f"{node} runs with the remap {frm} to {to}" for node, frm, to, _ in onto
                    )
                    lines = [rules.where(rows[0]) for *_head, rows in onto if rows]
                    detail += (
                        f". {rules_text}, and no endpoint of "
                        f"{'that node' if len(onto) == 1 else 'those nodes'} landed on {name}, "
                        "so the remap matched nothing the node creates"
                    )
                    say += f"; {rules_text}, which matched no endpoint of it"
                    fix = (
                        f"correct the remap {onto[0][1]} to {onto[0][2]}"
                        + (f" at {', '.join(lines)}" if lines else "")
                        + " to the name the node publishes, or "
                        + fix
                    )
                    extra_where = [row for *_head, rows in onto for row in rows]
            out.append(
                model.Finding(
                    rule=rule,
                    severity=rules.severity_of(rule),
                    subject=topic_id,
                    title=title,
                    detail=detail,
                    fix=fix,
                    say=say,
                    discriminator=disc_for(profile),
                    profiles=[profile],
                    layers=[OBSERVED],
                    where=evidence_of(*[stated(edge, OBSERVED, profile) for edge in live])
                    + extra_where,
                )
            )
    return out


def _remaps_onto(
    atlas: model.Atlas, name: str, profile: str
) -> list[tuple[str, str, str, list[model.Source]]]:
    """(node, from, to, remap line) for every running node started with a rule onto `name`
    and holding no endpoint on it, with the launch line that writes the rule when the
    declared layer holds it. Such a rule matched nothing the node creates."""
    out = []
    on_topic = {edge.src for edge in atlas.in_edges(ids.build_topic(name))}
    for node in atlas.by_kind(model.NodeKind.NODE.value):
        fact = stated(node, OBSERVED, profile)
        if fact is None or node.id in on_topic:
            continue
        namespace = str(fact.values.get("namespace") or "/")
        own = str(fact.values.get("name") or "") or None
        for row in fact.values.get("remaps") or []:
            if not isinstance(row, (list, tuple)) or len(row) != 2:
                continue
            frm, to = str(row[0]), str(row[1])
            try:
                if ids.normalise_ros_name(to, namespace, own) != name:
                    continue
            except ContractError:
                continue
            sites = [edge.src for edge in atlas.in_edges(node.id) if edge.kind == "launches"]
            written = [
                source
                for entry in atlas.unresolved
                if entry.role == "remap" and entry.owner in sites and entry.expr == f"{frm}:={to}"
                for source in entry.evidence
            ]
            out.append((human(node.id), frm, to, rules.sources(written)))
    return sorted(out, key=lambda row: (row[0], row[1], row[2]))


@rules.register("topic.no_subscriber")
def topic_no_subscriber(atlas: model.Atlas) -> list[model.Finding]:
    """A topic published in a running profile that nothing subscribes."""
    return _lonely(atlas, rule="topic.no_subscriber", present_is_publisher=True)


@rules.register("topic.no_publisher")
def topic_no_publisher(atlas: model.Atlas) -> list[model.Finding]:
    """A topic subscribed in a running profile that nothing publishes."""
    return _lonely(atlas, rule="topic.no_publisher", present_is_publisher=False)


# ---------------------------------------------------------------------------
# what two layers both stated


def _qos_of(fact: model.LayerFact) -> dict[str, Any]:
    raw = fact.values.get("qos")
    if not isinstance(raw, dict):
        return {}
    return {key: raw[key] for key in QOS_KEYS if raw.get(key) is not None}


def type_key(text: str) -> tuple[str, str] | None:
    """(package, message name) or None when the spelling cannot be read.

    `nav_msgs::msg::Odometry` and `nav_msgs/msg/Odometry` are one type. Anything that does
    not resolve to a package and a name is unknown and never enters a comparison.
    """
    parts = [part for part in text.replace("::", "/").split("/") if part]
    if len(parts) == 3 and parts[1] in ("msg", "srv", "action"):
        return (parts[0], parts[2])
    if len(parts) == 2 and "::" not in text:
        return (parts[0], parts[1])
    return None


@rules.register("qos.differ")
def qos_differ(atlas: model.Atlas) -> list[model.Finding]:
    """Two layers stated a different QoS for one endpoint."""
    out: list[model.Finding] = []
    for edge, profile, facts in (view for group in _sites(atlas) for view in _views(group)):
        if _via_wrapper(edge, profile):
            continue
        stated_by = {layer: _qos_of(fact) for layer, fact in facts.items()}
        stated_by = {layer: values for layer, values in stated_by.items() if values}
        if len(stated_by) < 2:
            continue
        layers = sorted(stated_by)
        differ: dict[str, list[str]] = {}
        for key in QOS_KEYS:
            seen = {layer: stated_by[layer][key] for layer in layers if key in stated_by[layer]}
            if len(seen) > 1 and len(set(map(repr, seen.values()))) > 1:
                differ[key] = [f"{layer} says {seen[layer]}" for layer in sorted(seen)]
        if not differ:
            continue
        keys = sorted(differ)
        spelled = "; ".join(f"{key}: {', '.join(differ[key])}" for key in keys)
        label = f"{human(edge.src)} {edge.kind} {ids.label_for(edge.dst)}"
        out.append(
            model.Finding(
                rule="qos.differ",
                severity=rules.severity_of("qos.differ"),
                subject=_ref(edge),
                title=f"{label} has two different qos settings",
                detail=(
                    f"in {slug(profile)} the layers disagree about {', '.join(keys)} on this "
                    f"endpoint: {spelled}"
                ),
                fix=(
                    "make the qos in the source match the one the running system uses, or "
                    "the connection will not be made at all for an incompatible pair"
                ),
                say=(
                    f"{label} is created with a different qos than the running system "
                    f"reports ({spelled}), which is what stops a publisher and a subscriber "
                    "from connecting"
                ),
                discriminator=disc_for(profile, *keys),
                profiles=[profile] if profile != model.ANY_PROFILE else [],
                layers=layers,
                where=evidence_of(*facts.values()),
            )
        )
    return out


@rules.register("qos.incompatible")
def qos_incompatible(atlas: model.Atlas) -> list[model.Finding]:
    """A subscription asks for a QoS no publisher of its topic offers, so nothing arrives.

    The silent topic: both ends exist, `ros2 topic list` shows the topic, the callback never
    runs. Compared inside one layer and one profile, from what each side states or what the
    preset it names means; an end whose policy is unknown decides nothing.
    """
    out: list[model.Finding] = []
    by_topic: dict[tuple[str, str, str], dict[str, list[Ended]]] = {}
    for edge in endpoint_edges(atlas, (PUBLISHES, SUBSCRIBES)):
        for profile in sorted(set(edge.profiles) or {model.ANY_PROFILE}):
            facts = _layer_facts(edge, profile)
            if _via_wrapper(edge, profile):
                # a wrapper picks its own profile unless the source hands it one
                facts = {
                    layer: fact
                    for layer in model.LAYER_ORDER
                    for fact in [edge.fact(layer, profile) or edge.fact(layer, model.ANY_PROFILE)]
                    if fact is not None
                    and fact.confidence != model.Confidence.UNKNOWN.value
                    and qos_presets.policies(fact.values.get("qos"))
                }
            for layer, fact in facts.items():
                sides = by_topic.setdefault((edge.dst, profile, layer), {})
                sides.setdefault(edge.kind, []).append((edge, fact))
    found: dict[tuple[str, str], dict[str, tuple]] = {}
    for (topic, profile, layer), sides in sorted(by_topic.items()):
        if profile != model.ANY_PROFILE:
            # A `*` fact holds in this profile too; a pair of two `*` facts is reported once,
            # by the `*` group.
            everywhere = by_topic.get((topic, model.ANY_PROFILE, layer), {})
            sides = {
                kind: sides.get(kind, []) + everywhere.get(kind, [])
                for kind in sides.keys() | everywhere.keys()
            }
        pair = _worst_pair(sides, concrete=profile != model.ANY_PROFILE)
        if pair is not None:
            found.setdefault((topic, profile), {})[layer] = pair
    # One topic in one profile is one finding, told from the layer that cites a line: the
    # probe confirming what the source states adds no line to edit.
    for (topic, profile), by_layer in sorted(found.items()):
        layer = next(item for item in TYPE_LAYERS if item in by_layer)
        finding = _incompatible(topic, profile, layer, *by_layer[layer])
        finding.layers = sorted(by_layer)
        out.append(finding)
    return out


Ended = tuple[model.Edge, model.LayerFact]


def _worst_pair(
    sides: dict[str, list[Ended]], *, concrete: bool
) -> tuple[Ended, Ended, list[str], list[Ended], int] | None:
    """The incompatible pair to report, a starved subscription first.

    Returns the publisher, the subscriber, the policies they disagree on, the publishers that
    do match the subscriber and how many incompatible pairs the topic has.
    """
    found = []
    for sub in sides.get(SUBSCRIBES, []):
        requested = qos_presets.policies(sub[1].values.get("qos"))
        refused, matched = [], []
        for pub in sides.get(PUBLISHES, []):
            if concrete and model.ANY_PROFILE == pub[1].profile == sub[1].profile:
                continue
            reasons = qos_presets.incompatible(
                qos_presets.policies(pub[1].values.get("qos")), requested
            )
            (refused if reasons else matched).append((pub, reasons))
        found.extend(
            (bool(matched), pub, sub, reasons, [row[0] for row in matched])
            for pub, reasons in refused
        )
    if not found:
        return None
    _fed, pub, sub, reasons, matched = min(found, key=lambda row: row[0])
    return pub, sub, reasons, matched, len(found)


def _incompatible(
    topic: str,
    profile: str,
    layer: str,
    pub: Ended,
    sub: Ended,
    reasons: list[str],
    matched: list[Ended],
    pairs: int,
) -> model.Finding:
    (pub_edge, pub_fact), (sub_edge, sub_fact) = pub, sub
    offered = qos_presets.policies(pub_fact.values.get("qos"))
    requested = qos_presets.policies(sub_fact.values.get("qos"))
    asked = ", ".join(requested[key].replace("_", " ") for key in reasons)
    given = ", ".join(offered[key].replace("_", " ") for key in reasons)
    name = ids.label_for(topic)
    publisher, subscriber = human(pub_edge.src), human(sub_edge.src)
    more = f" ({pairs - 1} more pairs on this topic)" if pairs > 1 else ""
    if matched:
        feeding = ", ".join(sorted({human(edge.src) for edge, _fact in matched}))
        title = f"{subscriber} gets nothing from {publisher} on {name} in {slug(profile)}"
        outcome = f"so those two never connect and only {feeding} reaches it."
    else:
        title = f"{subscriber} never receives {name} in {slug(profile)}"
        outcome = "so the two never connect and no message arrives."
    return model.Finding(
        rule="qos.incompatible",
        severity=rules.severity_of("qos.incompatible"),
        subject=topic,
        title=title,
        detail=(
            f"in {slug(profile)} the {layer} layer has {publisher} offering {given} on "
            f"{name} and {subscriber} requesting {asked}; a subscription that asks for "
            f"more than a publisher offers is never matched{more}"
        ),
        fix=(
            f"make {subscriber} request {given}, or make {publisher} offer {asked}, "
            "on both ends of the topic"
        ),
        say=(
            f"{subscriber} subscribes to {name} requesting {asked} while {publisher} "
            f"publishes it {given}, {outcome}"
        ),
        discriminator=disc_for(profile),
        profiles=[profile] if profile != model.ANY_PROFILE else [],
        layers=[layer],
        where=evidence_of(sub_fact, pub_fact),
    )


@rules.register("type.differ")
def type_differ(atlas: model.Atlas) -> list[model.Finding]:
    """Two layers stated a different message type for one endpoint."""
    out: list[model.Finding] = []
    for edge, profile, facts in (view for group in _sites(atlas) for view in _views(group)):
        if _via_wrapper(edge, profile):
            continue
        spelled: dict[str, str] = {}
        keys: dict[str, tuple[str, str]] = {}
        for layer, fact in facts.items():
            text = str(fact.values.get("type") or "")
            key = type_key(text) if text else None
            if key is None:
                continue
            spelled[layer] = text
            keys[layer] = key
        if len(keys) < 2 or len(set(keys.values())) < 2:
            continue
        layers = sorted(keys)
        said = ", ".join(f"{layer} says {spelled[layer]}" for layer in layers)
        label = f"{human(edge.src)} {edge.kind} {ids.label_for(edge.dst)}"
        out.append(
            model.Finding(
                rule="type.differ",
                severity=rules.severity_of("type.differ"),
                subject=_ref(edge),
                title=f"{label} has two different message types",
                detail=(
                    f"in {slug(profile)} the layers disagree about the type of this "
                    f"endpoint: {said}"
                ),
                fix="use one type on both ends of this endpoint",
                say=(
                    f"{label} is declared with {said}, and two types on one name never "
                    "connect, so the data does not flow"
                ),
                discriminator=disc_for(profile),
                profiles=[profile] if profile != model.ANY_PROFILE else [],
                layers=layers,
                where=evidence_of(*facts.values()),
            )
        )
    return out


PRODUCERS = frozenset({PUBLISHES, model.EdgeKind.SERVES.value})
TYPE_LAYERS = (CODED, DECLARED, OBSERVED)
"""The layer a type mismatch is reported from, the one that cites a line first."""

Typed = tuple[model.Edge, model.LayerFact, tuple[str, str], str]


@rules.register("type.incompatible")
def type_incompatible(atlas: model.Atlas) -> list[model.Finding]:
    """Two ends of one topic, service or action state two types in one layer and profile.

    The ends never connect: a subscription of one type takes nothing a publisher of another
    type sends, and a client of one service or action type never reaches a server of another.
    Only a type each side states in a form that names a package is compared, so a template
    parameter or an alias nobody resolved decides nothing.
    """
    grouped: dict[tuple[str, str], dict[str, list[Typed]]] = {}
    for edge in endpoint_edges(atlas):
        for profile in sorted(set(edge.profiles) or {model.ANY_PROFILE}):
            if _via_wrapper(edge, profile):
                continue
            for layer, fact in _layer_facts(edge, profile).items():
                text = str(fact.values.get("type") or "")
                key = type_key(text) if text else None
                if key is not None:
                    row = (edge, fact, key, text)
                    grouped.setdefault((edge.dst, profile), {}).setdefault(layer, []).append(row)
    sites: dict[tuple, list[tuple[str, list[str], list[Typed], list[Typed]]]] = {}
    concrete = {channel for channel, profile in grouped if profile != model.ANY_PROFILE}
    for (channel, profile), by_layer in sorted(grouped.items()):
        if profile != model.ANY_PROFILE:
            everywhere = grouped.get((channel, model.ANY_PROFILE), {})
            by_layer = {
                layer: by_layer.get(layer, []) + everywhere.get(layer, [])
                for layer in by_layer.keys() | everywhere.keys()
            }
        elif channel in concrete:
            continue
        by_layer = {layer: _comparable(rows) for layer, rows in by_layer.items()}
        split = [layer for layer in TYPE_LAYERS if _crossed(by_layer.get(layer, []))]
        if not split:
            continue
        right, wrong = _sides_of_type(by_layer[split[0]])
        lines = tuple(sorted({src.sort_key() for row in wrong for src in row[1].evidence}))
        # One line stated under several namespaces is one finding, so one edit is one report.
        key = (profile, split[0], right[0][3], tuple(sorted({row[3] for row in wrong})), lines)
        sites.setdefault(key if lines else (*key, channel), []).append(
            (channel, split, right, wrong)
        )
    return [_type_mismatch(key[0], rows) for key, rows in sorted(sites.items())]


@rules.register("topic.namespace_escape")
def topic_namespace_escape(atlas: model.Atlas) -> list[model.Finding]:
    """One executable started in two namespaces publishes one absolute topic from both.

    The code spells the name with a leading slash, so the namespace each copy runs in does not
    reach it and every robot writes the same topic, which a reader cannot tell apart. Only a
    coded publisher with no remap counts, and names a library shares on purpose (`/tf`,
    `/rosout`, `/diagnostics`) are left out.
    """
    sites: dict[tuple[str, str, tuple[str, str]], list[tuple[str, model.LayerFact]]] = {}
    for edge in endpoint_edges(atlas, ("publishes",)):
        name = ids.parse(edge.dst).segments[0]
        if is_quiet(name) or name == "/clock":
            continue
        node = atlas.entities.get(edge.src)
        for fact in edge.facts:
            raw = str(fact.values.get("raw_name") or "")
            if (
                fact.layer != model.Layer.CODED.value
                or not rules.usable(fact)
                or not raw.startswith("/")
                or fact.values.get("remapped")
                or fact.values.get("condition")
            ):
                continue
            started = _started_as(node, fact.profile)
            if started is not None:
                sites.setdefault((fact.profile, edge.dst, started), []).append((edge.src, fact))
    out = []
    for (profile, topic, (pkg, exe)), rows in sorted(sites.items()):
        nodes = sorted({node for node, _fact in rows})
        spaces = {
            ids.parse_node(node).fqn.rpartition("/")[0]
            for node in nodes
            if not ids.is_synthetic(node)
        }
        if len(spaces) < 2:
            continue
        name = ids.parse(topic).segments[0]
        where = rules.sources([row for _node, fact in rows for row in fact.evidence])
        at = rules.where(where[0]) if where else f"{pkg}/{exe}"
        told = ", ".join(rules.node_text(node) for node in nodes)
        out.append(
            model.Finding(
                rule="topic.namespace_escape",
                severity=rules.severity_of("topic.namespace_escape"),
                subject=topic,
                discriminator=ids.digest(profile, pkg, exe),
                title=f"{told} all publish {name} in {rules.slug_of(profile)}",
                detail=(
                    f"In {rules.slug_of(profile)} {pkg}/{exe} runs as {told}, and {at} publishes "
                    f"the absolute name {name}, which no namespace changes, so every copy writes "
                    "the same topic and a subscriber cannot tell them apart."
                ),
                fix=f"drop the leading slash of {name} at {at}, or remap it per namespace",
                say=(
                    f"{told} all publish {name} because {at} spells it with a leading slash; "
                    f"drop the slash so each namespace gets its own topic."
                ),
                profiles=[profile],
                layers=[model.Layer.CODED.value],
                where=where,
            )
        )
    return out


def _started_as(node: model.Node | None, profile: str) -> tuple[str, str] | None:
    """(package, executable) the profile's launch starts this node from."""
    if node is None:
        return None
    for fact in node.facts:
        if fact.layer == model.Layer.DECLARED.value and fact.profile == profile:
            pkg, exe = fact.values.get("pkg"), fact.values.get("executable")
            if pkg and exe:
                return str(pkg), str(exe)
    return None


def _comparable(rows: list[Typed]) -> list[Typed]:
    """The ends that state one type unconditionally. A node stating two types on one name
    (`TwistStamped` or `Twist` by a parameter) holds alternatives, and either may run."""
    types: dict[str, set[tuple[str, str]]] = {}
    for row in rows:
        types.setdefault(row[0].src, set()).add(row[2])
    return [
        row for row in rows if len(types[row[0].src]) == 1 and not row[1].values.get("condition")
    ]


def _crossed(rows: list[Typed]) -> bool:
    """Whether a producing end and a consuming end of two nodes state two types."""
    made = {(row[0].src, row[2]) for row in rows if row[0].kind in PRODUCERS}
    taken = {(row[0].src, row[2]) for row in rows if row[0].kind not in PRODUCERS}
    return any(a != b and key != other for a, key in made for b, other in taken)


def _sides_of_type(rows: list[Typed]) -> tuple[list[Typed], list[Typed]]:
    """The ends that state the type the producing side states most, and the others."""
    producers = [row for row in rows if row[0].kind in PRODUCERS]
    counts: dict[tuple[str, str], int] = {}
    for row in producers:
        counts[row[2]] = counts.get(row[2], 0) + 1
    wanted = min(counts, key=lambda key: (-counts[key], key))
    right = sorted((row for row in rows if row[2] == wanted), key=lambda row: (row[3], row[0].src))
    wrong = sorted(
        (row for row in rows if row[2] != wanted), key=lambda row: (row[0].src, row[0].kind, row[3])
    )
    return right, wrong


def _type_mismatch(
    profile: str, channels: list[tuple[str, list[str], list[Typed], list[Typed]]]
) -> model.Finding:
    channel, layers, right, wrong = channels[0]
    spelling = right[0][3]
    name = ids.label_for(channel)
    odd = wrong[0]
    who = sorted({human(row[0].src) for row in right})
    others = sorted({f"{human(row[0].src)} {row[0].kind} it as {row[3]}" for row in wrong})
    line = rules.where(odd[1].evidence[0] if odd[1].evidence else None)
    also = sorted(ids.label_for(item[0]) for item in channels[1:])
    more = f", and the same holds on {', '.join(also)}" if also else ""
    rows = [row for item in channels for row in item[3] + item[2]]
    return model.Finding(
        rule="type.incompatible",
        severity=rules.severity_of("type.incompatible"),
        subject=channel,
        title=f"{', '.join([name, *also])} {'have' if also else 'has'} two message types in "
        f"{slug(profile)}",
        detail=(
            f"in {slug(profile)} the {layers[0]} layer has {', '.join(who)} on {name} as "
            f"{spelling}, while {'; '.join(others)}{more}. Two ends of one name that state two "
            "types are never matched, so nothing passes between them"
        ),
        fix=(
            f"use {spelling} for {human(odd[0].src)} on {name} at {line}, or change the other "
            "ends to the type it states"
        ),
        say=(
            f"{human(odd[0].src)} {odd[0].kind} {name} as {odd[3]} at {line}, while "
            f"{', '.join(who)} uses {spelling}, so the two ends never connect{more}; change "
            f"it to {spelling}"
        ),
        discriminator=disc_for(profile),
        profiles=[profile] if profile != model.ANY_PROFILE else [],
        layers=sorted({layer for item in channels for layer in item[1]}),
        where=evidence_of(*(row[1] for row in rows)),
    )


SERVES = model.EdgeKind.SERVES.value
CALLS = model.EdgeKind.CALLS.value


@rules.register("service.no_server")
def service_no_server(atlas: model.Atlas) -> list[model.Finding]:
    """A client calls a service or action name nothing in the profile serves, while a node
    whose whole source was read serves another name of the same type.

    Said only when the client's source and the source of every server of that type were read
    whole, and no node of the profile holds a service or action name the reader could not
    follow, so the server could not hide under a name rosio did not see. The closest served
    name of the type is the likely target, and the finding names it.
    """
    from rosio.drift import frames

    whole = frames.read_whole(atlas)
    clients: dict[tuple[str, str], list[tuple[model.Edge, model.LayerFact]]] = {}
    servers: dict[tuple[str, tuple[str, str]], dict[str, set[str]]] = {}
    served: set[tuple[str, str]] = set()
    for edge in endpoint_edges(atlas, (SERVES, CALLS)):
        for profile in sorted(edge.profiles):
            if profile == model.ANY_PROFILE or not _states_endpoint(edge, profile):
                continue
            if edge.kind == SERVES:
                served.add((edge.dst, profile))
            fact = stated(edge, CODED, profile)
            key = type_key(str(fact.values.get("type") or "")) if fact else None
            if fact is None or key is None or _via_wrapper(edge, profile):
                continue
            if edge.kind == SERVES:
                servers.setdefault((profile, key), {}).setdefault(edge.dst, set()).add(edge.src)
            else:
                clients.setdefault((edge.dst, profile), []).append((edge, fact))
    blind: dict[str, tuple[set[str], tuple[str, ...]]] = {}
    # A remap whose target ends like the name may hand it to a node rosio did not read.
    remapped = {
        name
        for name in (ids.label_for(channel) for channel, _profile in clients)
        for entry in atlas.unresolved
        if entry.role == "remap"
        and entry.expr.partition(":=")[2].rstrip("/").endswith(name.rpartition("/")[2])
    }
    groups: dict[tuple, list[tuple[str, list[tuple[model.Edge, model.LayerFact]], str]]] = {}
    for (channel, profile), calls in sorted(clients.items()):
        name = ids.label_for(channel)
        if (channel, profile) in served or _library_service(name) or name in remapped:
            continue
        if profile not in blind:
            blind[profile] = _blind_owners(atlas, profile)
        owners, prefixes = blind[profile]
        if owners or name.startswith(prefixes):
            continue
        calls = [row for row in calls if whole(row[0].src, profile)]
        if not calls:
            continue
        key = type_key(str(calls[0][1].values.get("type")))
        options = {
            other: owners
            for other, owners in servers.get((profile, key), {}).items()
            if ids.kind_of(other) == ids.kind_of(channel)
            and all(whole(owner, profile) for owner in owners)
        }
        if not options:
            continue
        near = _closest_name(name, sorted(ids.label_for(other) for other in options))
        lines = tuple(sorted({src.sort_key() for _edge, fact in calls for src in fact.evidence}))
        groups.setdefault((profile, lines) if lines else (profile, channel), []).append(
            (channel, calls, near)
        )
    return [_no_server(key[0], rows) for key, rows in sorted(groups.items())]


def _blind_owners(atlas: model.Atlas, profile: str) -> tuple[set[str], tuple[str, ...]]:
    """The nodes of a profile that could serve any name, and the name prefixes under which
    an unread node whose services `known_names.SERVICE_SCOPE` states could serve one."""
    from rosio import join
    from rosio.drift import controllers, frames
    from rosio.resolve import known_names

    wants = (model.Want.SERVICE_NAME.value, model.Want.ACTION_NAME.value)
    whole = frames.read_whole(atlas)
    unread: set[str] = set()
    prefixes: list[str] = []
    for node in atlas.by_kind(model.NodeKind.NODE.value):
        fact = stated(node, DECLARED, profile)
        if fact is None or whole(node.id, profile):
            continue
        scope = known_names.SERVICE_SCOPE.get(controllers.executable(fact))
        if scope is None or (ids.is_synthetic(node.id) and scope != "none"):
            unread.add(node.id)
            continue
        if scope == "none":
            continue
        fqn = ids.parse_node(node.id).fqn
        prefixes.append(fqn + "/")
        if scope == "controllers":
            namespace = fqn.rpartition("/")[0]
            loaded = controllers.defined(atlas, node.id, profile)
            if loaded:
                prefixes.extend(f"{namespace}/{name}/" for name in loaded)
            else:
                # a manager whose configuration was not read could load any controller
                prefixes.append(f"{namespace}/")
    return join.blind_nodes(atlas, wants, profile) | unread, tuple(prefixes)


def _library_service(name: str) -> bool:
    """A service the client library serves for a node, which no source states."""
    return is_quiet(name) or "/_action/" in name or name.rpartition("/")[2] in LIFECYCLE_TAILS


def _closest_name(name: str, options: list[str]) -> str:
    """The served name most like the one called: in the caller's namespace first, then by
    spelling, and the first of the options when none is close."""
    import difflib

    head = name.rpartition("/")[0]
    local = [item for item in options if item.rpartition("/")[0] == head] or options
    near = difflib.get_close_matches(name, local, n=1, cutoff=0.0)
    return near[0] if near else local[0]


def _no_server(
    profile: str, rows: list[tuple[str, list[tuple[model.Edge, model.LayerFact]], str]]
) -> model.Finding:
    channel, calls, near = rows[0]
    name = ids.label_for(channel)
    kind = ids.kind_of(channel)
    who = sorted({human(edge.src) for edge, _fact in calls})
    type_text = str(calls[0][1].values.get("type"))
    line = rules.where(calls[0][1].evidence[0] if calls[0][1].evidence else None)
    also = sorted(ids.label_for(item[0]) for item in rows[1:])
    more = f", and the same holds for {', '.join(also)}" if also else ""
    return model.Finding(
        rule="service.no_server",
        severity=rules.severity_of("service.no_server"),
        subject=channel,
        title=f"{', '.join(who)} calls {name} and nothing serves it in {slug(profile)}",
        detail=(
            f"in {slug(profile)} {', '.join(who)} calls the {kind} {name} ({type_text}) at "
            f"{line}, and no node of the profile serves it, while {near} of the same type is "
            f"served by a node whose source was read{more}. A client of a name nobody serves "
            "waits for ever"
        ),
        fix=f"call {near} at {line}, or start the node that serves {name}",
        say=(
            f"{', '.join(who)} calls {name} at {line}, which nothing in {slug(profile)} serves; "
            f"the served {kind} of type {type_text} closest to it is {near}"
        ),
        discriminator=disc_for(profile),
        profiles=[profile],
        layers=[CODED],
        where=evidence_of(*(fact for item in rows for _edge, fact in item[1])),
    )


def _expected_rate(fact: model.LayerFact) -> float | None:
    rate = fact.values.get("rate_hz")
    if isinstance(rate, (int, float)) and not isinstance(rate, bool) and rate > 0:
        return float(rate)
    period = fact.values.get("period_s")
    if isinstance(period, (int, float)) and not isinstance(period, bool) and period > 0:
        return 1.0 / float(period)
    return None


def _measured_rate(fact: model.LayerFact) -> float | None:
    volatile = fact.values.get("volatile")
    if not isinstance(volatile, dict):
        return None
    rate = volatile.get("rate_hz")
    if isinstance(rate, (int, float)) and not isinstance(rate, bool) and rate >= 0:
        return float(rate)
    return None


def _recorded_topics(atlas: model.Atlas) -> dict[str, list[tuple[model.Node, model.Edge]]]:
    out: dict[str, list[tuple[model.Node, model.Edge]]] = {}
    for edge in sorted(atlas.edges.values(), key=lambda row: row.id):
        if edge.kind != "samples":
            continue
        entity = atlas.entities.get(edge.src)
        if entity is None or entity.kind != "bagtopic":
            continue
        out.setdefault(edge.dst, []).append((entity, edge))
    return out


def _authority(stated_by: dict[str, float | None]) -> str:
    """Which layer's rate the measurement is held against.

    `EFFECTIVE_PRECEDENCE` from AGENTS section 6, not a sorted layer name: a launch parameter
    that lowers a timer below the default in the source is the rate the system is meant to
    hold, and comparing against the source default instead reports a healthy node as slow.
    """
    for layer in model.EFFECTIVE_PRECEDENCE:
        if layer in stated_by:
            return layer
    return sorted(stated_by)[0]


@rules.register("rate.below_declared")
def rate_below_declared(atlas: model.Atlas) -> list[model.Finding]:
    """A publisher measured well under the rate its source or its launch states.

    The measured number is volatile and lives under the finding's own `volatile` key, so two
    probes of one slow system produce the same compared bytes.
    """
    out: list[model.Finding] = []
    recorded = _recorded_topics(atlas)
    for edge, profile, facts in (
        view for group in _sites(atlas, (PUBLISHES,)) for view in _views(group)
    ):
        if _via_wrapper(edge, profile):
            continue
        live = facts.get(OBSERVED)
        measured = _measured_rate(live) if live is not None else None
        measurements = (
            [("", live, measured)]
            if measured is not None
            else [
                (entity.id, fact, _measured_rate(fact))
                for entity, samples in recorded.get(edge.dst, [])
                if stated(samples, "recorded", profile) is not None
                and (fact := stated(entity, "recorded", profile)) is not None
            ]
        )
        stated_by = {
            layer: _expected_rate(fact)
            for layer, fact in facts.items()
            if layer != OBSERVED and _expected_rate(fact) is not None
        }
        if not stated_by:
            continue
        layer = _authority(stated_by)
        expected = stated_by[layer]
        if expected is None:
            continue
        label = f"{human(edge.src)} publishing {ids.label_for(edge.dst)}"
        for identity, measurement, measured in measurements:
            if measured is None or measured >= expected * (1.0 - RATE_TOLERANCE):
                continue
            measured_layer = "recorded" if identity else OBSERVED
            out.append(
                model.Finding(
                    rule="rate.below_declared",
                    severity=rules.severity_of("rate.below_declared"),
                    subject=_ref(edge),
                    title=f"{label} runs slower than the {expected:g} Hz it states",
                    detail=(
                        f"in {slug(profile)} the {layer} layer states {expected:g} Hz for this "
                        f"publisher and the {measured_layer} layer measures more than "
                        f"{int(RATE_TOLERANCE * 100)} percent under it"
                    ),
                    fix=(
                        "find what blocks the publishing loop, or state the rate the node can "
                        "actually hold"
                    ),
                    say=(
                        f"{label} states {expected:g} Hz and the {measured_layer} layer measures "
                        "a rate below it, so anything "
                        "downstream that assumes the stated rate is working on stale data."
                    ),
                    discriminator=disc_for(profile, identity) if identity else disc_for(profile),
                    profiles=[profile] if profile != model.ANY_PROFILE else [],
                    layers=[layer, measured_layer],
                    where=evidence_of(facts.get(layer), measurement),
                    extra={"volatile": {"measured": f"{measured:g} Hz"}},
                )
            )
    return out


@rules.register("endpoint.observed_only")
def endpoint_observed_only(atlas: model.Atlas) -> list[model.Finding]:
    """An endpoint only the running system knows about, on a node whose source was read."""
    out: list[model.Finding] = []
    siblings = {(edge.kind, edge.src, edge.dst): group for group in _sites(atlas) for edge in group}
    for edge in endpoint_edges(atlas):
        owner = atlas.entities.get(edge.src)
        if owner is None:
            continue
        for profile in observed_profiles(atlas):
            live = stated(edge, OBSERVED, profile)
            if live is None or _via_wrapper(edge, profile):
                continue
            # The probe keys an endpoint by name and the join by source site, so the source
            # may state this endpoint on another edge between the same node and channel.
            if any(
                stated(other, CODED, profile) or stated(other, DECLARED, profile)
                for other in siblings[(edge.kind, edge.src, edge.dst)]
            ):
                continue
            if not says_anything(owner, CODED, profile):
                # The source of this node was never read, so the endpoint is coverage.
                continue
            name = ids.label_for(edge.dst)
            if library_channel(name, owner):
                continue
            # The time source of a node run with use_sim_time subscribes /clock itself.
            if edge.kind == SUBSCRIBES and name == "/clock":
                continue
            label = human(edge.src)
            where = slug(profile)
            out.append(
                model.Finding(
                    rule="endpoint.observed_only",
                    severity=rules.severity_of("endpoint.observed_only"),
                    subject=_ref(edge),
                    title=f"{label} {edge.kind} {name} only at runtime",
                    detail=(
                        f"the probe reported this endpoint in {where}, the source of {label} was "
                        "read, and no statement in it or in the launch description creates the "
                        "endpoint, so it comes from a call the reader could not follow"
                    ),
                    fix=(
                        "state the name as a literal or a parameter default in the source, or "
                        "add the library that creates it to the wrapper table"
                    ),
                    say=(
                        f"{label} {edge.kind} {name} while it is running, and nothing in its "
                        "source says so, so a reader of the code cannot find where that "
                        "endpoint comes from"
                    ),
                    discriminator=disc_for(profile),
                    profiles=[profile],
                    layers=[OBSERVED],
                    where=evidence_of(live, *owner.facts),
                )
            )
    return out


RULES: tuple[tuple[str, Callable[[model.Atlas], list[model.Finding]]], ...] = (
    ("endpoint.observed_only", endpoint_observed_only),
    ("qos.differ", qos_differ),
    ("qos.incompatible", qos_incompatible),
    ("rate.below_declared", rate_below_declared),
    ("service.no_server", service_no_server),
    ("topic.no_publisher", topic_no_publisher),
    ("topic.no_subscriber", topic_no_subscriber),
    ("type.differ", type_differ),
    ("type.incompatible", type_incompatible),
)
"""Rule id to implementation, in rule id order, for a loader that reads a module list."""


def check(atlas: model.Atlas) -> list[model.Finding]:
    out: list[model.Finding] = []
    for _rule, function in RULES:
        out.extend(function(atlas))
    return sorted(out, key=model.Finding.sort_key)


__all__ = [
    "QOS_KEYS",
    "RATE_TOLERANCE",
    "RULES",
    "Coverage",
    "check",
    "coverage",
    "endpoint_observed_only",
    "is_quiet",
    "probe_enumerated_endpoints",
    "qos_differ",
    "quiet_names",
    "quiet_suffixes",
    "rate_below_declared",
    "service_no_server",
    "topic_no_publisher",
    "topic_no_subscriber",
    "type_differ",
    "type_incompatible",
    "type_key",
]
