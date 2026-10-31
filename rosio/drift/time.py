"""Recorded and observed header stamp failures and their downstream clock symptoms."""

from __future__ import annotations

from statistics import median
from typing import Any

from rosio import ids, model
from rosio.drift import rules
from rosio.stamps.clock import MIN_TOPICS_FOR_SKEW, SKEW_ERROR_S, SKEW_WARNING_S

RECORDED = model.Layer.RECORDED.value
OBSERVED = model.Layer.OBSERVED.value
SYMPTOM = (
    "A consumer can report a TF error because the timestamp on the message is earlier "
    "than all the data in the transform cache, even when that consumer is not at fault."
)


def _publish_sources(atlas: model.Atlas, topic: str, profile: str) -> list[model.Source]:
    return sorted(
        rules.sources(
            [
                source
                for edge in atlas.edges.values()
                if edge.kind == "publishes" and edge.dst == topic
                for fact in edge.facts
                if fact.layer in ("coded", "declared")
                and fact.profile in (profile, model.ANY_PROFILE)
                and rules.usable(fact)
                for source in fact.evidence
            ]
        ),
        key=model.Source.sort_key,
    )


def _skew_by_node(atlas: model.Atlas) -> dict[str, list[dict[str, Any]]]:
    """Recorded skew results with one vote per publishing node.

    The stamps check takes the median over topics, so a node publishing two late topics
    outvotes a third node and the punctual topic is blamed. Where the atlas names the one
    node publishing a recorded topic, that node's topics share a vote; a topic no node is known
    to publish keeps its own. A recording where no topic has a known publisher keeps the
    stored results.
    """
    owners: dict[str, set[str]] = {}
    for edge in atlas.edges.values():
        if edge.kind == "publishes" and any(
            fact.layer in ("coded", "declared") and rules.usable(fact) for fact in edge.facts
        ):
            owners.setdefault(ids.label_for(edge.dst), set()).add(edge.src)
    bags: dict[str, dict[str, tuple[str, float]]] = {}
    for entity in atlas.by_kind(model.NodeKind.BAGTOPIC.value):
        for fact in entity.facts:
            offset = fact.values.get("stamps", {}).get("offset_s")
            if fact.layer == RECORDED and isinstance(offset, (int, float)):
                identity = ids.parse_bagtopic(entity.id)
                bags.setdefault(identity.bag, {})[entity.id] = (identity.topic, float(offset))
    out: dict[str, list[dict[str, Any]]] = {}
    for topics in bags.values():
        if len(topics) < MIN_TOPICS_FOR_SKEW:
            continue
        if not any(len(owners.get(topic, ())) == 1 for topic, _ in topics.values()):
            continue
        votes: dict[str, list[float]] = {}
        for topic, offset in topics.values():
            found = owners.get(topic, set())
            votes.setdefault(next(iter(found)) if len(found) == 1 else topic, []).append(offset)
        reference = median(median(values) for values in votes.values())
        for entity_id, (topic, offset) in topics.items():
            delta = offset - reference
            if abs(delta) <= SKEW_WARNING_S:
                out[entity_id] = []
                continue
            out[entity_id] = [
                {
                    "check": "inter_topic_skew",
                    "severity": "error" if abs(delta) > SKEW_ERROR_S else "warning",
                    "message": (
                        f"{topic} offset {offset:.3f} s deviates {delta:.3f} s from the bag "
                        f"reference {reference:.3f} s, taken with one vote per publishing node"
                    ),
                    "evidence": {
                        "topic_offset_s": offset,
                        "reference_offset_s": reference,
                        "delta_s": delta,
                    },
                }
            ]
    return out


def _findings(
    atlas: model.Atlas,
    rule: str,
    check: str,
    title: str,
    measurement: str,
    unit: str,
    fallback: str,
    recorded: dict[str, list[dict[str, Any]]] | None = None,
) -> list[model.Finding]:
    out = []
    for entity in sorted(atlas.entities.values(), key=lambda row: row.id):
        live = entity.kind == model.NodeKind.TOPIC.value
        if not live and entity.kind != model.NodeKind.BAGTOPIC.value:
            continue
        layer = OBSERVED if live else RECORDED
        for fact in sorted(entity.facts, key=model.LayerFact.sort_key):
            if fact.layer != layer or not rules.usable(fact):
                continue
            values = fact.values.get("volatile", {}) if live else fact.values
            results = values.get("stamps", {}).get("checks", [])
            if not live and recorded is not None and entity.id in recorded:
                results = recorded[entity.id]
            for result in results:
                if result.get("check") != check:
                    continue
                evidence: dict[str, Any] = result.get("evidence", {})
                if measurement not in evidence:
                    continue
                variant = ""
                if rule == "time.epoch_offset" and result.get("severity") == "warning":
                    variant = "unclassified"
                elif (
                    rule in ("time.topic_skew", "time.drift") and result.get("severity") == "error"
                ):
                    variant = "strong"
                measured = f"{rules.value_text(evidence[measurement])} {unit}"
                kind = title.lower()
                extra: dict[str, Any] = {}
                if live:
                    # A live measurement moves every probe, so it stays out of compared text.
                    topic = ids.label_for(entity.id)
                    heading = f"{topic}: {kind}"
                    say = f"{topic} in the running system has a {kind}."
                    extra = {"volatile": {"measured": measured}}
                    detail = f"Header stamps in the running system fail the {check} check."
                    where = _publish_sources(atlas, entity.id, fact.profile)
                else:
                    identity = ids.parse_bagtopic(entity.id)
                    topic = identity.topic
                    shown = f"{_measured(evidence[measurement], unit)} {unit}"
                    heading = f"{topic}: {kind} of {shown}"
                    say = f"{topic} in recording {identity.bag} has a {kind} of {shown}."
                    detail = f"{result['message']}."
                    where = rules.sources(fact.evidence)
                out.append(
                    model.Finding(
                        rule=rule,
                        severity=(
                            "info"
                            if rule == "time.zero" and evidence.get("all_zero")
                            else rules.severity_of(rule, variant)
                        ),
                        subject=entity.id,
                        title=heading,
                        say=say,
                        detail=f"{detail} {SYMPTOM}",
                        fix=fallback,
                        where=where,
                        discriminator=fact.profile if fact.profile != model.ANY_PROFILE else "",
                        profiles=[] if fact.profile == model.ANY_PROFILE else [fact.profile],
                        layers=[layer],
                        extra=extra,
                    )
                )
    return out


def _measured(value: Any, unit: str) -> str:
    """Seconds to the millisecond, as the stamps command prints them."""
    if unit == "s" and isinstance(value, float):
        return f"{value:.3f}"
    return rules.value_text(value)


@rules.register("time.backward")
def backward(atlas: model.Atlas) -> list[model.Finding]:
    return _findings(
        atlas,
        "time.backward",
        "monotonic",
        "Backward stamp step",
        "largest_backward_jump",
        "s",
        "The source clock stepped backwards or messages were reordered. "
        "Keep the source clock monotonic and publish messages in stamp order.",
    )


@rules.register("time.epoch_offset")
def epoch_offset(atlas: model.Atlas) -> list[model.Finding]:
    return _findings(
        atlas,
        "time.epoch_offset",
        "epoch_offset",
        "Stamp epoch offset",
        "median_offset_s",
        "s",
        "Use the receiver's timescale, check the PTP timescale and synchronize the source clock.",
    )


@rules.register("time.topic_skew")
def topic_skew(atlas: model.Atlas) -> list[model.Finding]:
    return _findings(
        atlas,
        "time.topic_skew",
        "inter_topic_skew",
        "Stamp skew",
        "delta_s",
        "s",
        "Synchronize this source clock with the clocks of the other topics.",
        _skew_by_node(atlas),
    )


@rules.register("time.frozen")
def frozen(atlas: model.Atlas) -> list[model.Finding]:
    return _findings(
        atlas,
        "time.frozen",
        "frozen_stamps",
        "Frozen stamp span",
        "receive_span_s",
        "s",
        "Refresh the header stamp for every newly acquired message.",
    )


@rules.register("time.zero")
def zero(atlas: model.Atlas) -> list[model.Finding]:
    return _findings(
        atlas,
        "time.zero",
        "zero_stamps",
        "Zero stamp count",
        "count",
        "messages",
        "Set the header stamp to the acquisition time before publishing.",
    )


@rules.register("time.gap")
def gap(atlas: model.Atlas) -> list[model.Finding]:
    return _findings(
        atlas,
        "time.gap",
        "stamp_gaps",
        "Stamp gap",
        "max_gap_s",
        "s",
        "Inspect acquisition stalls and dropped messages at the source.",
    )


@rules.register("time.drift")
def drift(atlas: model.Atlas) -> list[model.Finding]:
    return _findings(
        atlas,
        "time.drift",
        "clock_drift",
        "Clock drift",
        "slope_s_per_s",
        "s per s",
        "Discipline the source clock against the receiver clock.",
    )
