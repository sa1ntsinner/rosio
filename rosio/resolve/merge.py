"""The one deep merge rule, with a trace of what the overlay actually did.

Four independent reimplementations of this merge exist in the reference repository (the
desktop launcher, one sim launch file and two CI checkers) with identical semantics:

  * overlay wins per leaf
  * a non mapping overlay value replaces whatever was there, so a list replaces wholesale
    and is never merged element wise
  * an overlay key the base never had is inserted, never rejected

This module is that rule, once. `merge_trace` is what makes a python side override citable:
it says which leaf changed, which leaf was restated with the identical value, which leaf the
base never had, and where each of them sits.
"""

from __future__ import annotations

import copy
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

CHANGED = "changed"
RESTATED = "restated"
INTRODUCED = "introduced"
TYPE_CHANGED = "type_changed"

EVENT_KINDS = (CHANGED, RESTATED, INTRODUCED, TYPE_CHANGED)


@dataclass(frozen=True)
class MergeEvent:
    """One thing the overlay did to the base, at one key path.

    `type_changed` is separate from `changed` on purpose: 20 becoming 20.0 is a phantom
    drift on its own, and a mapping replacing a scalar is a different edit again.
    """

    path: tuple[str, ...]
    kind: str
    base_value: Any = None
    overlay_value: Any = None

    @property
    def name(self) -> str:
        return ".".join(self.path)

    @property
    def base_present(self) -> bool:
        return self.kind != INTRODUCED

    def sort_key(self) -> tuple[tuple[str, ...], str]:
        return (self.path, self.kind)


def deep_merge(base: Mapping[str, Any] | None, overlay: Mapping[str, Any] | None) -> dict:
    """Merged copy of base and overlay. Neither input is mutated."""
    result: dict[Any, Any] = copy.deepcopy(dict(base or {}))
    for key, value in (overlay or {}).items():
        current = result.get(key)
        if isinstance(value, Mapping) and isinstance(current, Mapping):
            result[key] = deep_merge(current, value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def merge_trace(
    base: Mapping[str, Any] | None,
    overlay: Mapping[str, Any] | None,
) -> list[MergeEvent]:
    """What the merge changed, what it introduced, and where. Sorted by key path."""
    events: list[MergeEvent] = []
    _walk(dict(base or {}), dict(overlay or {}), (), events)
    return sorted(events, key=MergeEvent.sort_key)


def merged_with_trace(
    base: Mapping[str, Any] | None,
    overlay: Mapping[str, Any] | None,
) -> tuple[dict, list[MergeEvent]]:
    return deep_merge(base, overlay), merge_trace(base, overlay)


def events_by_name(events: list[MergeEvent]) -> dict[str, MergeEvent]:
    """Trace indexed by dotted key path, for a caller that has a parameter name."""
    return {event.name: event for event in events}


def _walk(
    base: Mapping[str, Any],
    overlay: Mapping[str, Any],
    prefix: tuple[str, ...],
    events: list[MergeEvent],
) -> None:
    for key in sorted(overlay, key=str):
        path = (*prefix, str(key))
        value = overlay[key]
        present = key in base
        current = base.get(key) if present else None
        if isinstance(value, Mapping) and isinstance(current, Mapping):
            # An empty overlay mapping over a mapping changes nothing, so it is no event.
            _walk(current, value, path, events)
            continue
        if isinstance(value, Mapping) and value:
            if present:
                events.append(
                    MergeEvent(
                        path=path,
                        kind=TYPE_CHANGED,
                        base_value=current,
                        overlay_value=value,
                    )
                )
            else:
                _walk({}, value, path, events)
            continue
        events.append(
            MergeEvent(
                path=path,
                kind=_leaf_kind(present, current, value),
                base_value=current,
                overlay_value=value,
            )
        )


def _leaf_kind(present: bool, current: Any, value: Any) -> str:
    if not present:
        return INTRODUCED
    if _kind_of(current) != _kind_of(value):
        return TYPE_CHANGED
    if current == value:
        return RESTATED
    return CHANGED


def _kind_of(value: Any) -> str:
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int):
        return "int"
    if isinstance(value, float):
        return "float"
    if isinstance(value, str):
        return "str"
    if isinstance(value, Mapping):
        return "dict"
    if isinstance(value, (list, tuple)):
        return "list"
    return type(value).__name__


__all__ = [
    "CHANGED",
    "EVENT_KINDS",
    "INTRODUCED",
    "RESTATED",
    "TYPE_CHANGED",
    "MergeEvent",
    "deep_merge",
    "events_by_name",
    "merge_trace",
    "merged_with_trace",
]
