"""Recorded and observed header stamp failures and their downstream clock symptoms."""

from __future__ import annotations

import re
from typing import Any

from rosio import ids, model
from rosio.drift import rules

RECORDED = model.Layer.RECORDED.value
OBSERVED = model.Layer.OBSERVED.value
SYMPTOM = (
    "A consumer can report a TF error because the timestamp on the message is earlier "
    "than all the data in the transform cache, even when that consumer is not at fault."
)


def _fix(atlas: model.Atlas, verdict_on: str, topic: str, profile: str, fallback: str) -> str:
    """Quote the verdict's hypotheses for this topic, from a bag fact or a live probe run."""
    entity = atlas.entities.get(verdict_on)
    fact = entity.fact(RECORDED, profile) if entity else None
    if entity and entity.kind == model.NodeKind.RUN.value:
        hypotheses = entity.attrs.get("volatile", {}).get("stamp_verdict", {}).get("hypotheses", [])
    elif rules.usable(fact):
        hypotheses = fact.values.get("stamp_verdict", {}).get("hypotheses", [])
    else:
        return fallback
    pattern = re.compile(r"(?<![\w/])" + re.escape(topic) + r"(?![\w/])")
    matching = [text for text in hypotheses if pattern.search(text) or text.startswith("All ")]
    return " ".join(matching) if matching else fallback


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


def _findings(
    atlas: model.Atlas,
    rule: str,
    check: str,
    title: str,
    measurement: str,
    unit: str,
    fallback: str,
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
            for result in values.get("stamps", {}).get("checks", []):
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
                    verdict_on = fact.run
                    where = _publish_sources(atlas, entity.id, fact.profile)
                else:
                    identity = ids.parse_bagtopic(entity.id)
                    topic = identity.topic
                    heading = f"{topic}: {kind} of {measured}"
                    say = f"{topic} in recording {identity.bag} has a {kind} of {measured}."
                    detail = f"{result['message']}."
                    verdict_on = ids.build_bag(identity.bag)
                    where = rules.sources(fact.evidence)
                out.append(
                    model.Finding(
                        rule=rule,
                        severity=rules.severity_of(rule, variant),
                        subject=entity.id,
                        title=heading,
                        say=say,
                        detail=f"{detail} {SYMPTOM}",
                        fix=_fix(atlas, verdict_on, topic, fact.profile, fallback),
                        where=where,
                        discriminator=fact.profile if fact.profile != model.ANY_PROFILE else "",
                        profiles=[] if fact.profile == model.ANY_PROFILE else [fact.profile],
                        layers=[layer],
                        extra=extra,
                    )
                )
    return out


@rules.register("time.backward")
def backward(atlas: model.Atlas) -> list[model.Finding]:
    return _findings(
        atlas,
        "time.backward",
        "monotonic",
        "Backward stamp step",
        "largest_backward_jump",
        "s",
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
        "Use the receiver's timescale and synchronize the source clock.",
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
