"""The merge driver for atlas.json.

An atlas is meant to be committed, so two developers who both rebuild it hand git a
conflict in a large generated json file that nobody will ever resolve by hand. This module
resolves it instead, by union, and it states the rule rather than hiding it:

1. A side that did not change a value never overrides the side that did.
2. A key only one side has is kept. A key one side deleted while the other left it alone
   is deleted.
3. Where the two sides genuinely disagree about the same fact, both are kept. The encoding
   that sorts first is the one in place, and the other is recorded under `x.merge`, with
   the base value beside it. Which side a value came from decides nothing, so merging the
   same pair the other way round produces the same bytes.
4. A candidate chain is atomic per profile. Interleaving two chains would invent a
   parameter precedence that neither developer wrote, and ranks that no run produced.
5. Everything derived is recomputed rather than merged: the counters, the indexes, the
   per entity conflicts, the agreement and the effective values all follow from the facts,
   and merging them separately would let them disagree with the facts under them.
6. `counters.resolved` takes the smaller of the two sides for each want, over the union of
   the blind spots, so a merged atlas never reads healthier than either atlas it came from.
7. A counter that follows from nothing in the file survives the merge anyway. The drift
   counter is the one: no finding list says how many rules ran, so recomputing cannot bring
   it back, and an atlas carrying every finding while stating that no drift run is recorded
   is false about itself.

A merge that silently prefers one side is worse than a conflict, because the value that
lost leaves no trace. Nothing here is silent: every genuine disagreement is in the file and
in the report the driver prints.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from rosio import __version__
from rosio.errors import ContractError, UsageError
from rosio.model import Atlas, AtlasBuilder, dumps, strip_volatile

MERGE_KEY = "merge"
"""Where the marks live: `x.merge`, the extension namespace, so no schema key changes."""

RULE = (
    "union: a side that did not change a value never overrides the side that did, and "
    "where both sides changed one value differently both are kept, the encoding that sorts "
    "first in place and the other recorded here"
)

_MISSING = object()

_IDENTITY = {
    "facts": lambda row: f"{row.get('layer', '')}@{row.get('profile', '*')}",
    "findings": lambda row: str(row.get("id", "")),
    "unresolved": lambda row: str(row.get("id", "")),
    "passes": lambda row: str(row.get("collector", "")),
    "roots": lambda row: str(row.get("path", "")),
}
"""How a row in a list is recognised across two rebuilds. A list not named here is merged
as a set of whole rows, which keeps both versions of a row rather than guessing which
fields identify it."""

RUN_KIND = "run"

_BUILD_INPUTS = ("env", "input_digest", "predicates", "roots", "tool_version")
"""What a run entity states about the build itself. Everything else a run carries, the
exceptions and the pass rows, is what the extraction lost and stays a reported difference."""

_DERIVED_TOP = ("counters", "indexes")
_DERIVED_ENTITY = ("layers", "profiles", "agreement", "conflicts", "effective", "unresolved")
_DERIVED_EDGE = ("layers", "profiles", "agreement", "conflicts", "effective")


@dataclass
class MergeResult:
    """The merged document, and everything the merge had to decide."""

    payload: dict[str, Any]
    conflicts: list[dict[str, Any]] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    counts: dict[str, int] = field(default_factory=dict)

    def to_bytes(self) -> bytes:
        return dumps(self.payload).encode("utf-8")

    def lines(self, limit: int = 10) -> list[str]:
        head = (
            f"merged atlas: {self.counts.get('entities', 0)} entities, "
            f"{self.counts.get('edges', 0)} edges, {len(self.conflicts)} kept both"
        )
        out = [head, f"  rule: {RULE}"]
        for row in self.conflicts[:limit]:
            out.append(f"  both changed {row['pointer']}")
            out.append(f"    in place {_gist(row['in_place'])}")
            if row["kind"] == "deleted_on_one_side":
                out.append("    the other side deleted it, so the value that moved is kept")
            else:
                out.append(f"    also stated {_gist(row['other'])}")
        hidden = len(self.conflicts) - min(len(self.conflicts), limit)
        if hidden:
            out.append(f"  and {hidden} more, all of them under x.merge in the file")
        for note in self.notes:
            out.append(f"  {note}")
        return out


def _short(value: Any, width: int = 90) -> str:
    text = dumps(value).strip().replace("\n", " ")
    return text if len(text) <= width else text[: width - 3] + "..."


def _gist(value: Any) -> str:
    """What a person needs to see of a kept value: the winner and the line it is on."""
    if isinstance(value, list):
        winners = [row for row in value if isinstance(row, dict) and row.get("wins")]
        if winners:
            row = winners[0]
            return f"{dumps(row.get('value')).strip()} from {_where(row.get('source') or {})}"
    if isinstance(value, dict) and "values" in value:
        return _short(value["values"])
    return _short(value)


def _where(source: dict[str, Any]) -> str:
    text = str(source.get("file", "")).replace("file:", "", 1).replace("|", "/")
    line = source.get("line_hint")
    return f"{text}:{line}" if line is not None else text


# ---------------------------------------------------------------------------
# the entry points


def merge_files(
    base: str | Path,
    ours: str | Path,
    theirs: str | Path,
    *,
    out: str | Path | None = None,
    path: str = "",
) -> MergeResult:
    """Merge three files on disk and write the result where git expects it.

    git hands the driver the base as %O, our version as %A and their version as %B, and
    reads the result back out of %A.
    """
    result = merge_documents(_read(base, allow_empty=True), _read(ours), _read(theirs), path=path)
    target = Path(out) if out is not None else Path(ours)
    target.write_bytes(result.to_bytes())
    return result


def _read(target: str | Path, *, allow_empty: bool = False) -> dict[str, Any] | None:
    source = Path(target)
    if not source.is_file():
        if allow_empty:
            return None
        raise UsageError(f"no such file: {source}")
    text = source.read_text(encoding="utf-8").strip()
    if not text:
        if allow_empty:
            return None
        raise UsageError(f"{source} is empty, so there is nothing to merge")
    try:
        payload = json.loads(text)
    except json.JSONDecodeError as exc:
        raise UsageError(f"{source} is not json: {exc}") from exc
    if not isinstance(payload, dict):
        raise UsageError(f"{source} is not an atlas document")
    return payload


def merge_documents(
    base: dict[str, Any] | None,
    ours: dict[str, Any],
    theirs: dict[str, Any],
    *,
    path: str = "",
) -> MergeResult:
    """Three way merge of two atlases, with everything derived recomputed at the end."""
    version = _schema_version(ours, theirs)
    resolved = _resolved_counts(ours, theirs)
    conflicts: list[dict[str, Any]] = []
    notes: list[str] = []

    generated = _pick_generated(ours, theirs, notes)
    sides = [_strip(item) for item in (base, ours, theirs)]
    merged = _merge3("", sides[0] if base is not None else _MISSING, sides[1], sides[2], conflicts)
    merged["schema_version"] = version
    if generated is not None:
        merged["generated"] = generated

    atlas = Atlas.from_json(merged)
    builder = AtlasBuilder()
    builder.atlas = atlas
    for want, count in sorted(resolved.items()):
        builder.count_resolved(want, count)
    builder.recompute()
    drift = _drift_counter(ours, theirs, atlas, notes)
    if drift is not None:
        atlas.counters["drift"] = drift

    builds = [row for row in conflicts if row["kind"] == "build_reference"]
    advisory = [row for row in conflicts if row["kind"] == "advisory"]
    conflicts = [row for row in conflicts if row["kind"] not in ("build_reference", "advisory")]
    if builds:
        notes.append(
            f"{len(builds)} values state what a build was rather than what the system does, "
            "and both builds are in the file with their inputs"
        )
    if advisory:
        notes.append(
            f"{len(advisory)} line hints or volatile values differ between the two sides, "
            "which is not a disagreement about the system and is not reported as one"
        )
    conflicts.sort(key=lambda row: (row["pointer"], dumps(row.get("in_place"))))
    known = set(atlas.entities) | set(atlas.edges)
    atlas.x = dict(atlas.x)
    atlas.x[MERGE_KEY] = {
        "tool": f"rosio {__version__}",
        "rule": RULE,
        "path": path,
        "conflicts": conflicts,
        "subjects": sorted({_subject(row["pointer"], known) for row in conflicts} - {""}),
        "notes": list(notes),
    }
    payload = atlas.to_json()
    return MergeResult(
        payload=payload,
        conflicts=conflicts,
        notes=notes,
        counts={
            "entities": len(atlas.entities),
            "edges": len(atlas.edges),
            "findings": len(atlas.findings),
            "conflicts": len(conflicts),
        },
    )


def _subject(pointer: str, known: set[str]) -> str:
    """The entity or edge a pointer sits inside.

    Cutting at the first slash is wrong: every node, topic and parameter id holds one, so
    `entities/param:/controller_server|max_vel_x/candidates` used to answer `param:`. The
    id is the longest prefix of the pointer that the merged atlas actually holds.
    """
    parts = pointer.split("/")
    if len(parts) < 2 or parts[0] not in ("entities", "edges"):
        return parts[0] if parts else ""
    body = ""
    found = ""
    for part in parts[1:]:
        body = f"{body}/{part}" if body else part
        if body in known:
            found = body
    return found or parts[1]


def _schema_version(ours: dict[str, Any], theirs: dict[str, Any]) -> str:
    left = str(ours.get("schema_version", ""))
    right = str(theirs.get("schema_version", ""))
    if left.split(".", 1)[0] != right.split(".", 1)[0]:
        raise ContractError(
            f"schema {left} and schema {right} are different majors, so the two files do not "
            "describe the same shape. rebuild both with one version of rosio"
        )
    return max((left, right), key=lambda text: [int(part) for part in text.split(".")])


def _resolved_counts(ours: dict[str, Any], theirs: dict[str, Any]) -> dict[str, int]:
    """The smaller of the two counts per want, over the sides that state it.

    The blind spots are a union, so pairing them with the larger denominator would let a
    merged atlas read healthier than either atlas it came from.
    """
    left = dict((ours.get("counters") or {}).get("resolved") or {})
    right = dict((theirs.get("counters") or {}).get("resolved") or {})
    out: dict[str, int] = {}
    for want in set(left) | set(right):
        stated = [int(side[want]) for side in (left, right) if want in side]
        out[str(want)] = min(stated)
    return out


_DRIFT_RULE_KEYS = ("rules_run", "rules_failed")
"""What a drift counter states about the rules that ran. The rest of it counts one build's
own finding list, which a merge of two lists is not."""


def _drift_counter(
    ours: dict[str, Any],
    theirs: dict[str, Any],
    atlas: Atlas,
    notes: list[str],
) -> dict[str, Any] | None:
    """The drift counter of the merged atlas.

    `_strip` takes the derived counters out and `recompute` puts back everything that follows
    from the facts, which this one does not: no number of rules can be read off a finding
    list. Without it a merged atlas carrying every finding states that no drift run is
    recorded, which is a false statement about itself. Both sides ran the same rules over
    their own atlas, so what the two agree about the rules is true of the merged file, the
    per rule tally is counted again over the findings that are in it, and a tally of one
    build's own run (what it produced, filtered, waived or gated) is dropped rather than
    restated over a list it never saw.
    """
    sides = [side for side in (_drift_of(ours), _drift_of(theirs)) if side is not None]
    if not sides:
        return None
    merged = sorted(item.id for item in atlas.findings)
    if _eq(sides[0], sides[-1]) and all(_finding_ids(doc) == merged for doc in (ours, theirs)):
        return dict(sides[0])
    out: dict[str, Any] = {}
    for key in _DRIFT_RULE_KEYS:
        stated = [side[key] for side in sides if key in side]
        if stated and all(_eq(stated[0], value) for value in stated):
            out[key] = stated[0]
    by_rule: dict[str, int] = {}
    for item in atlas.findings:
        by_rule[item.rule] = by_rule.get(item.rule, 0) + 1
    out["by_rule"] = dict(sorted(by_rule.items()))
    notes.append(
        "the two sides ran the rules separately, so the drift counter states the rules that "
        "ran and the findings this file holds, and neither build's own tallies"
    )
    return out


def _drift_of(payload: dict[str, Any]) -> dict[str, Any] | None:
    found = (payload.get("counters") or {}).get("drift")
    return dict(found) if isinstance(found, dict) else None


def _finding_ids(payload: dict[str, Any]) -> list[str]:
    rows = payload.get("findings") or []
    return sorted(str(row.get("id", "")) for row in rows if isinstance(row, dict))


def _pick_generated(
    ours: dict[str, Any], theirs: dict[str, Any], notes: list[str]
) -> dict[str, Any] | None:
    """The generation block is not compared data, so it is chosen and not merged."""
    left = ours.get("generated")
    right = theirs.get("generated")
    if left == right:
        return left
    notes.append("the two sides were generated separately, the generation block is one of them")
    return min([left, right], key=dumps)


def _strip(payload: dict[str, Any] | None) -> dict[str, Any]:
    """A copy with everything derived taken out, so the merge only sees stated facts."""
    if payload is None:
        return {}
    out = {key: value for key, value in payload.items() if key not in _DERIVED_TOP}
    out.pop("generated", None)
    out.pop("schema_version", None)
    entities = {}
    for key, row in (out.get("entities") or {}).items():
        entities[key] = {name: value for name, value in row.items() if name not in _DERIVED_ENTITY}
    if "entities" in out:
        out["entities"] = entities
    edges = {}
    for key, row in (out.get("edges") or {}).items():
        edges[key] = {name: value for name, value in row.items() if name not in _DERIVED_EDGE}
    if "edges" in out:
        out["edges"] = edges
    extension = {name: value for name, value in (out.get("x") or {}).items() if name != MERGE_KEY}
    if "x" in out:
        out["x"] = extension
    return out


# ---------------------------------------------------------------------------
# the three way rule


def _settled(base: Any, ours: Any, theirs: Any) -> tuple[bool, Any]:
    """The cases that need no decision: agreement, one side untouched, or no ancestor."""
    if _eq(ours, theirs):
        return True, ours
    if _eq(ours, base):
        return True, theirs
    if _eq(theirs, base):
        return True, ours
    if base is _MISSING:
        # No ancestor here, so a key only one side has was added and not deleted.
        if ours is _MISSING:
            return True, theirs
        if theirs is _MISSING:
            return True, ours
    return False, None


def _merge3(
    pointer: str, base: Any, ours: Any, theirs: Any, conflicts: list[dict[str, Any]]
) -> Any:
    settled, value = _settled(base, ours, theirs)
    if settled:
        return value
    if base is not _MISSING and (ours is _MISSING or theirs is _MISSING):
        kept = theirs if ours is _MISSING else ours
        conflicts.append(_row(pointer, "deleted_on_one_side", kept, _MISSING, base))
        return kept
    if isinstance(ours, dict) and isinstance(theirs, dict):
        return _merge_dict(pointer, base, ours, theirs, conflicts)
    if isinstance(ours, list) and isinstance(theirs, list):
        return _merge_list(pointer, base, ours, theirs, conflicts)
    first, second = sorted([ours, theirs], key=dumps)
    kind = "both_changed"
    if _is_build_reference(pointer):
        kind = "build_reference"
    elif _is_advisory(pointer):
        kind = "advisory"
    conflicts.append(_row(pointer, kind, first, second, base))
    return first


def _is_advisory(pointer: str) -> bool:
    """A line hint is for a person and a volatile value is a measurement, and section 9 of
    the contract keeps both out of every comparison. Two sides stating a different one is
    not a disagreement about the system, so one is taken and nothing is reported."""
    parts = pointer.split("/")
    return parts[-1] == "line_hint" or "volatile" in parts


def _is_build_reference(pointer: str) -> bool:
    """What a build is, rather than what the system it read does.

    Two rebuilds of one workspace write the same slots from two runs, so the run a fact was
    written by reads as a disagreement on every slot. So does the record of what a run took
    in: two clones hash different trees, and a collector whose input digest does not cover
    the roots keeps one run id across both, which puts the two content hashes on one field.
    Nine of the ten rows on the first real merge were that one field. None of it is a
    disagreement about the system, and all of it stays in the file: both run entities are in
    the merged atlas with their inputs, and the fact points at one of them.
    """
    parts = pointer.split("/")
    if parts[-1] == "run" and "/facts/" in pointer:
        return True
    if parts[0] != "entities" or "attrs" not in parts:
        return False
    at = parts.index("attrs")
    subject = "/".join(parts[1:at])
    return (
        at + 1 < len(parts)
        and subject.startswith(f"{RUN_KIND}:")
        and parts[at + 1] in _BUILD_INPUTS
    )


def _merge_dict(
    pointer: str, base: Any, ours: dict, theirs: dict, conflicts: list[dict[str, Any]]
) -> dict:
    ancestor = base if isinstance(base, dict) else None
    out: dict[str, Any] = {}
    for key in sorted(set(ours) | set(theirs) | set(ancestor or {})):
        merged = _merge3(
            f"{pointer}/{key}" if pointer else key,
            ancestor.get(key, _MISSING) if ancestor is not None else _MISSING,
            ours.get(key, _MISSING),
            theirs.get(key, _MISSING),
            conflicts,
        )
        if merged is not _MISSING:
            out[key] = merged
    return out


def _merge_list(
    pointer: str, base: Any, ours: list, theirs: list, conflicts: list[dict[str, Any]]
) -> list:
    name = pointer.rsplit("/", 1)[-1]
    if name == "candidates":
        return _merge_chains(pointer, base, ours, theirs, conflicts)
    if not _rows(ours) and not _rows(theirs):
        # A list of plain values is a set: the union, minus whatever both sides dropped.
        keep = {dumps(item): item for item in [*ours, *theirs]}
        return [keep[key] for key in sorted(keep)]
    identity = _IDENTITY.get(name, _content_key)
    ancestor = base if isinstance(base, list) else []
    keyed = [_by_identity(rows, identity) for rows in (ancestor, ours, theirs)]
    merged = _merge_dict(
        pointer,
        keyed[0] if isinstance(base, list) else _MISSING,
        keyed[1],
        keyed[2],
        conflicts,
    )
    return [merged[key] for key in sorted(merged, key=str)]


def _content_key(row: Any) -> str:
    """A row with no identity of its own is recognised by its content, with the line hint
    out of it, so one provenance row whose line moved does not become two."""
    return dumps(strip_volatile(row))


def _by_identity(rows: list, identity) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for row in rows:
        key = identity(row) if isinstance(row, dict) else dumps(row)
        out[str(key)] = row
    return out


def _rows(items: list) -> bool:
    return any(isinstance(item, dict) for item in items)


def _merge_chains(
    pointer: str, base: Any, ours: list, theirs: list, conflicts: list[dict[str, Any]]
) -> list:
    """A candidate chain is one unit per profile.

    Merging two chains row by row would state a precedence neither developer wrote: rank 0
    from one side beside rank 0 from the other is two winners for one parameter, which the
    model refuses and a reader could not act on.
    """
    ancestor = _by_profile(base) if isinstance(base, list) else None
    mine, yours = _by_profile(ours), _by_profile(theirs)
    out: list[Any] = []
    for profile in sorted(set(mine) | set(yours) | set(ancestor or {})):
        chain = _one_chain(
            f"{pointer}[{profile}]",
            ancestor.get(profile, _MISSING) if ancestor is not None else _MISSING,
            mine.get(profile, _MISSING),
            yours.get(profile, _MISSING),
            conflicts,
        )
        if chain is not _MISSING:
            out.extend(chain)
    return out


def _one_chain(
    pointer: str, base: Any, ours: Any, theirs: Any, conflicts: list[dict[str, Any]]
) -> Any:
    """One profile's chain, whole. Never half of one side and half of the other."""
    settled, value = _settled(base, ours, theirs)
    if settled:
        return value
    if ours is _MISSING or theirs is _MISSING:
        kept = theirs if ours is _MISSING else ours
        conflicts.append(_row(pointer, "deleted_on_one_side", kept, _MISSING, base))
        return kept
    first, second = sorted([ours, theirs], key=dumps)
    if _content_key(first) == _content_key(second):
        conflicts.append(_row(pointer, "advisory", first, second, base))
        return first
    conflicts.append(_row(pointer, "chain_changed_on_both_sides", first, second, base))
    return first


def _by_profile(rows: list) -> dict[str, list]:
    out: dict[str, list] = {}
    for row in rows:
        profile = str(row.get("profile", "*")) if isinstance(row, dict) else "*"
        out.setdefault(profile, []).append(row)
    return out


def _row(pointer: str, kind: str, in_place: Any, other: Any, base: Any) -> dict[str, Any]:
    row: dict[str, Any] = {"pointer": pointer, "kind": kind, "in_place": in_place}
    if other is not _MISSING:
        row["other"] = other
    if base is not _MISSING:
        row["base"] = base
    return row


def _eq(left: Any, right: Any) -> bool:
    """Equality as json means it: a bool is not an int and 20 is not 20.0."""
    if left is _MISSING or right is _MISSING:
        return left is right
    if isinstance(left, bool) != isinstance(right, bool):
        return False
    if isinstance(left, dict) and isinstance(right, dict):
        return set(left) == set(right) and all(_eq(left[key], right[key]) for key in left)
    if isinstance(left, list) and isinstance(right, list):
        return len(left) == len(right) and all(_eq(a, b) for a, b in zip(left, right, strict=True))
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return type(left) is type(right) and left == right
    return left == right


__all__ = [
    "MERGE_KEY",
    "RULE",
    "MergeResult",
    "merge_documents",
    "merge_files",
]
