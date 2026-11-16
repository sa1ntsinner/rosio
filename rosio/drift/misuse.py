"""ROS API misuse one source file proves on its own, with no other layer to compare.

The coded readers state the sites as file values, `blocking_waits` and `clock_mixes`, and
only where the source settles the defect: a wait whose reply the callback's own group holds
back, and two times whose clock types the file names. A site the reader could not settle is
not stated, so these rules read what they are handed and decide nothing more.
"""

from __future__ import annotations

from collections import defaultdict
from typing import Any

from rosio import ids, model
from rosio.drift import rules

CODED = model.Layer.CODED.value

_CLOCK_WORDS = {"ROS_TIME": "ROS time", "SYSTEM_TIME": "system time", "STEADY_TIME": "steady time"}
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
        name = callback if callback != "lambda" else "lambda"
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
                "async_send_request, which the executor already spinning the node delivers"
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


@rules.register("clock.mixed")
def clock_mixed(atlas: model.Atlas) -> list[model.Finding]:
    """Two times of different clock types compared or subtracted, which rclcpp refuses with
    `can't compare times with different time sources` and rclpy with a TypeError, every time
    the line runs. The reader states the site only when the file settles both clock types."""
    grouped: dict[tuple[str, str, str, str], list[tuple[model.LayerFact, dict[str, Any]]]] = (
        defaultdict(list)
    )
    for file_id, fact, row in _rows(atlas, "clock_mixes"):
        key = (
            file_id,
            str(row.get("expr") or ""),
            str(row.get("left_clock") or ""),
            str(row.get("right_clock") or ""),
        )
        grouped[key].append((fact, row))
    out: list[model.Finding] = []
    for (file_id, expr, left_clock, right_clock), rows in sorted(grouped.items()):
        row = rows[0][1]
        where = rules.sources(
            [_source(file_id, fact, "clock_mix", item["line_hint"]) for fact, item in rows]
        )
        site = rules.where(where[0])
        left, right = str(row.get("left") or ""), str(row.get("right") or "")
        first = _CLOCK_WORDS.get(left_clock, left_clock)
        second = _CLOCK_WORDS.get(right_clock, right_clock)
        verb = "subtracts" if row.get("operation") == "subtract" else "compares"
        python = rules.path_of(file_id).endswith(".py")
        raised = (
            "rclpy raises TypeError"
            if python
            else "rclcpp throws can't compare times with different time sources"
        )
        system_side = right if right_clock != "ROS_TIME" else left
        fix = (
            f"build {system_side} at {site} from the node clock (`self.get_clock().now()`, or "
            "`clock_type=self.get_clock().clock_type`)"
            if python
            else f"build {system_side} at {site} with the node clock's type (`this->now()`, or "
            "`rclcpp::Time(..., get_clock()->get_clock_type())`)"
        )
        out.append(
            model.Finding(
                rule="clock.mixed",
                severity=rules.severity_of("clock.mixed"),
                subject=file_id,
                discriminator=ids.digest(expr, left_clock, right_clock),
                title=f"{expr} mixes {first} and {second}",
                detail=(
                    f"{site} {verb} {left}, which is {first}, and {right}, which is {second}. "
                    f"Times of two clock types cannot be compared or subtracted, so {raised} "
                    "each time the line runs."
                ),
                fix=fix,
                say=(f"{expr} at {site} {verb} {first} with {second}, so {raised} there; {fix}."),
                layers=[CODED],
                where=where,
            )
        )
    return out
