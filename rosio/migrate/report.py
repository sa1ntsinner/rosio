"""The plan as text for a person and as data for an agent or an editor."""

from __future__ import annotations

from collections.abc import Sequence

from rosio.migrate.scan import Change
from rosio.render import plural


def summary(changes: Sequence[Change]) -> dict[str, int]:
    return {
        "changes": len(changes),
        "packages": len({c.package for c in changes}),
        "automatic": sum(1 for c in changes if c.automatic),
        "by_hand": sum(1 for c in changes if not c.automatic),
        "removed": sum(1 for c in changes if c.effect == "removed"),
    }


def data(changes: Sequence[Change], start: str, target: str) -> dict:
    return {
        "from": start,
        "to": target,
        "summary": summary(changes),
        "changes": [change.to_json() for change in changes],
    }


def lines(changes: Sequence[Change], start: str, target: str) -> list[str]:
    counts = summary(changes)
    if not changes:
        return [f"rosio migrate {start} -> {target}: nothing in this workspace changes"]
    out = [
        f"rosio migrate {start} -> {target}: {plural(counts['changes'], 'change')} in "
        f"{plural(counts['packages'], 'package')}, {counts['automatic']} rewritten by apply, "
        f"{counts['by_hand']} by hand",
    ]
    package = None
    for change in changes:
        if change.package != package:
            package = change.package
            out.extend(["", package])
        where = f"{change.file}:{change.line}"
        out.append(f"  {where}  {change.effect} in {change.release}  {change.say}")
        pad = " " * (len(where) + 4)
        if change.automatic:
            first, *rest = change.replacement.splitlines() or [""]
            out.append(f"{pad}apply: {first}" + (" ..." if rest else ""))
        out.append(f"{pad}{change.source}")
    return out
