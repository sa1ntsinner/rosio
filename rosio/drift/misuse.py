"""ROS API misuse one source file proves on its own, with no other layer to compare.

The coded readers state the sites as a file value, `blocking_waits`, and only where the
source settles the defect: a wait whose reply the callback's own group holds back. A site the reader could not settle is
not stated, so these rules read what they are handed and decide nothing more.
"""

from __future__ import annotations

from collections import defaultdict
from typing import Any

from rosio import ids, model
from rosio.drift import rules

CODED = model.Layer.CODED.value

_REGISTRATION_WORDS = {
    "create_subscription": "subscription",
    "create_timer": "timer",
    "create_wall_timer": "timer",
    "create_service": "service",
}


def _rows(atlas: model.Atlas, key: str) -> list[tuple[str, model.LayerFact, dict[str, Any]]]:
    out: list[tuple[str, model.LayerFact, dict[str, Any]]] = []
    for entity in atlas.by_kind(model.NodeKind.FILE.value):
        for fact in entity.facts:
            if fact.layer != CODED or not rules.usable(fact):
                continue
            for row in fact.values.get(key) or []:
                if isinstance(row, dict) and row.get("line_hint"):
                    out.append((entity.id, fact, row))
    return out


def _source(file_id: str, fact: model.LayerFact, locator: str, line: Any) -> model.Source:
    return model.Source(
        file=file_id,
        locator=locator,
        line_hint=line,
        root=fact.evidence[0].root if fact.evidence else model.Root.SRC.value,
    )


@rules.register("callback.blocks_executor")
def callback_blocks_executor(atlas: model.Atlas) -> list[model.Finding]:
    """A node callback waits for a reply the executor running it can never deliver.

    The reader states a wait only when both the callback and the client sit in the node's
    default callback group, which is mutually exclusive, so the reply waits for the callback
    to return under any executor, or when the callback hands its own node to a second
    executor, which rclcpp refuses with an exception."""
    grouped: dict[tuple[str, str, str, str], list[tuple[model.LayerFact, dict[str, Any]]]] = (
        defaultdict(list)
    )
    for file_id, fact, row in _rows(atlas, "blocking_waits"):
        key = (
            file_id,
            str(row.get("callback") or ""),
            str(row.get("call") or ""),
            str(row.get("via") or ""),
        )
        grouped[key].append((fact, row))
    out: list[model.Finding] = []
    for (file_id, callback, call, via), rows in sorted(grouped.items()):
        fact, row = rows[0]
        where = rules.sources(
            [
                _source(file_id, item_fact, "blocking_wait", item["line_hint"])
                for item_fact, item in rows
            ]
        )
        site = rules.where(where[0])
        kind = _REGISTRATION_WORDS.get(str(row.get("registration") or ""), "callback")
        name = callback if callback != "lambda" else f"the {kind} lambda"
        reached = f", through {via}," if via else ""
        client = str(row.get("client") or "")
        spins = call.startswith("rclcpp::")
        if spins:
            why = (
                "hands the node it runs on to a second executor, and rclcpp throws because the "
                "node already belongs to the executor running the callback"
            )
            fix = (
                f"drop {call} from {site} and handle the response in a callback passed to "
                "async_send_request, or create the client on a separate callback group and spin "
                "the node with a MultiThreadedExecutor"
            )
        else:
            why = (
                f"waits on {client}, a client in the node's default callback group, the same "
                "mutually exclusive group the callback runs in, so the executor cannot deliver "
                "the reply until the callback returns and the wait never ends"
            )
            fix = (
                f"replace {call} at {site} with the asynchronous call and handle the reply in a "
                "done callback, or put the client in its own callback group and spin the node "
                "with a MultiThreadedExecutor"
            )
        out.append(
            model.Finding(
                rule="callback.blocks_executor",
                severity=rules.severity_of("callback.blocks_executor"),
                subject=file_id,
                discriminator=ids.digest(callback, call, via),
                title=f"{call} inside the {kind} callback {name} blocks its executor",
                detail=(
                    f"{site}{reached} in {name}, the {kind} callback registered at line "
                    f"{row.get('registration_line')}, {why}."
                ),
                fix=fix,
                say=(
                    f"{call} at {site}{reached} runs inside the {kind} callback {name} and "
                    f"{'throws' if spins else 'deadlocks'}: {fix}."
                ),
                layers=[CODED],
                where=where,
            )
        )
    return out
