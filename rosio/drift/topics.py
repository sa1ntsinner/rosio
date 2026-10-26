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
            if is_quiet(name) or topic_id in ends[missing_kind]:
                continue
            live = [edge for edge in present[topic_id] if stated(edge, OBSERVED, profile)]
            if not live:
                continue
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
                    where=evidence_of(*[stated(edge, OBSERVED, profile) for edge in live]),
                )
            )
    return out


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
    if len(parts) == 2:
        return (parts[0], parts[1])
    return None


@rules.register("qos.differ")
def qos_differ(atlas: model.Atlas) -> list[model.Finding]:
    """Two layers stated a different QoS for one endpoint."""
    out: list[model.Finding] = []
    for edge in endpoint_edges(atlas):
        for profile in sorted(set(edge.profiles) or {model.ANY_PROFILE}):
            if _via_wrapper(edge, profile):
                continue
            stated_by = {
                layer: _qos_of(fact) for layer, fact in _layer_facts(edge, profile).items()
            }
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
                    where=evidence_of(*_layer_facts(edge, profile).values()),
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
            if _via_wrapper(edge, profile):
                continue
            for layer, fact in _layer_facts(edge, profile).items():
                sides = by_topic.setdefault((edge.dst, profile, layer), {})
                sides.setdefault(edge.kind, []).append((edge, fact))
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
            out.append(_incompatible(topic, profile, layer, *pair))
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
        title = f"{subscriber} gets nothing from {publisher} on {name}"
        outcome = f"so those two never connect and only {feeding} reaches it."
    else:
        title = f"{subscriber} never receives {name}"
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
        discriminator=disc_for(profile, layer),
        profiles=[profile] if profile != model.ANY_PROFILE else [],
        layers=[layer],
        where=evidence_of(sub_fact, pub_fact),
    )


@rules.register("type.differ")
def type_differ(atlas: model.Atlas) -> list[model.Finding]:
    """Two layers stated a different message type for one endpoint."""
    out: list[model.Finding] = []
    for edge in endpoint_edges(atlas):
        for profile in sorted(set(edge.profiles) or {model.ANY_PROFILE}):
            if _via_wrapper(edge, profile):
                continue
            spelled: dict[str, str] = {}
            keys: dict[str, tuple[str, str]] = {}
            for layer, fact in _layer_facts(edge, profile).items():
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
                    where=evidence_of(*_layer_facts(edge, profile).values()),
                )
            )
    return out


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
    for edge in endpoint_edges(atlas, (PUBLISHES,)):
        for profile in sorted(set(edge.profiles) or {model.ANY_PROFILE}):
            if _via_wrapper(edge, profile):
                continue
            facts = _layer_facts(edge, profile)
            live = facts.get(OBSERVED)
            if live is None:
                continue
            measured = _measured_rate(live)
            if measured is None:
                continue
            stated_by = {
                layer: _expected_rate(fact)
                for layer, fact in facts.items()
                if layer != OBSERVED and _expected_rate(fact) is not None
            }
            if not stated_by:
                continue
            layer = _authority(stated_by)
            expected = stated_by[layer]
            if expected is None or measured >= expected * (1.0 - RATE_TOLERANCE):
                continue
            label = f"{human(edge.src)} publishing {ids.label_for(edge.dst)}"
            out.append(
                model.Finding(
                    rule="rate.below_declared",
                    severity=rules.severity_of("rate.below_declared"),
                    subject=_ref(edge),
                    title=f"{label} runs slower than the {expected:g} Hz it states",
                    detail=(
                        f"in {slug(profile)} the {layer} layer states {expected:g} Hz for this "
                        f"publisher and the running system stays more than "
                        f"{int(RATE_TOLERANCE * 100)} percent under it"
                    ),
                    fix=(
                        "find what blocks the publishing loop, or state the rate the node can "
                        "actually hold"
                    ),
                    say=(
                        f"{label} states {expected:g} Hz and runs below it, so anything "
                        "downstream that assumes the stated rate is working on stale data"
                    ),
                    discriminator=disc_for(profile),
                    profiles=[profile] if profile != model.ANY_PROFILE else [],
                    layers=[layer, OBSERVED],
                    where=evidence_of(facts.get(layer), live),
                    extra={"volatile": {"measured": f"{measured:g} Hz"}},
                )
            )
    return out


@rules.register("endpoint.observed_only")
def endpoint_observed_only(atlas: model.Atlas) -> list[model.Finding]:
    """An endpoint only the running system knows about, on a node whose source was read."""
    out: list[model.Finding] = []
    for edge in endpoint_edges(atlas):
        owner = atlas.entities.get(edge.src)
        if owner is None:
            continue
        for profile in observed_profiles(atlas):
            live = stated(edge, OBSERVED, profile)
            if live is None or _via_wrapper(edge, profile):
                continue
            if stated(edge, CODED, profile) or stated(edge, DECLARED, profile):
                continue
            if not says_anything(owner, CODED, profile):
                # The source of this node was never read, so the endpoint is coverage.
                continue
            name = ids.label_for(edge.dst)
            if is_quiet(name):
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
    ("topic.no_publisher", topic_no_publisher),
    ("topic.no_subscriber", topic_no_subscriber),
    ("type.differ", type_differ),
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
    "topic_no_publisher",
    "topic_no_subscriber",
    "type_differ",
    "type_key",
]
