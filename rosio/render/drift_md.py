"""DRIFT.md: every finding, ordered by severity and grouped by rule family.

Four decisions shape this file. Severity decides the order, because the head of the file
is the failure list. The rule family decides the grouping, because ten findings of one rule
are one repair with ten sites and reading them together is faster than reading them apart.
The rules that produced nothing get a closing section of their own, because a rule that
chose not to speak is information: it says which question this atlas cannot answer yet.
And a rule a gate held back is separated from a rule that ran to the end, because the two
silences are opposite statements and printing them alike is the defect this product exists
to report. The gates are read from the families themselves: a rule module exposes
`silences(atlas)`, and every row it returns is printed with the profile it covers and the
reason. A rule named there is never given the reason that it ran and matched nothing, and a
rule that a gate held back on one profile while it fired on another is still named, because
its findings are a lower bound.

Every evidence row is printed. The `drift` verb caps them at two per finding to stay small,
which means a three model comparison never prints the third model it quotes. A file has no
reason to cap anything.
"""

from __future__ import annotations

from collections.abc import Sequence
from importlib import import_module
from typing import Any

from rosio.drift import rules
from rosio.model import SEVERITY_ORDER, Atlas, Finding
from rosio.query import label, profile_name, site
from rosio.render import heading, joined, plural, rendered_from, table

NO_PROFILE = "not scoped to a profile"
WHOLE_ATLAS = "the whole atlas"
GATED_REASON = "a gate held it back"

FAMILY_NAMES = {
    "param": "Parameters",
    "plugin": "Plugins",
    "node": "Node identity",
    "topic": "Topics",
    "qos": "Quality of service",
    "type": "Interface types",
    "rate": "Rates",
    "frame": "Frames and transforms",
    "remap": "Remaps",
    "lifecycle": "Lifecycle",
    "build": "Build tree",
    "launch": "Launch files",
    "endpoint": "Endpoints",
    "resolution": "Coverage budget",
}

NEEDS_LAYER = {
    "topic.no_subscriber": "observed",
    "topic.no_publisher": "observed",
    "qos.differ": "observed",
    "type.differ": "observed",
    "rate.below_declared": "observed",
    "endpoint.observed_only": "observed",
    "node.declared_not_observed": "observed",
    "node.observed_only": "observed",
}
"""Rules that cannot speak until a live system has been probed. A rule outside this table
that produced nothing ran over the atlas and matched nothing, which is a different
statement and is written as one."""


def render_drift(atlas: Atlas) -> str:
    gates = _gates(atlas)
    lines: list[str] = []
    lines.extend(_header(atlas, gates))
    lines.extend(_families(atlas))
    lines.extend(_held_back(gates))
    lines.extend(_silences(atlas, gates))
    return "\n".join(lines).rstrip("\n") + "\n"


# ---------------------------------------------------------------------------
# header


def _findings(atlas: Atlas) -> list[Finding]:
    return sorted(atlas.findings, key=Finding.sort_key)


def _family_of(rule: str) -> str:
    return rule.split(".", 1)[0]


def _severity_counts(items: Sequence[Finding]) -> dict[str, int]:
    counts = dict.fromkeys(sorted(SEVERITY_ORDER, key=lambda name: SEVERITY_ORDER[name]), 0)
    for item in items:
        counts[item.severity] = counts.get(item.severity, 0) + 1
    return counts


def _summary(counts: dict[str, int]) -> str:
    return joined((f"{count} {name}" for name, count in counts.items() if count), "none")


def _drift_counters(atlas: Atlas) -> dict[str, Any]:
    return ((atlas.counters or {}).get("drift") or {}) if atlas.counters else {}


def _gates(atlas: Atlas) -> list[dict[str, str]]:
    """Every gate the rule families report, sorted, with the profile each one covers.

    A family that has nothing to declare has no `silences`, and a family that has one owns
    the wording: the renderer states where the reason came from and never rewrites it.
    """
    rows: list[dict[str, str]] = []
    for name in rules.FAMILIES:
        report = getattr(import_module(name), "silences", None)
        if report is None:
            continue
        for row in report(atlas):
            rows.append(
                {
                    "rule": str(row.get("rule") or ""),
                    "profile": str(row.get("profile") or ""),
                    "reason": str(row.get("reason") or ""),
                }
            )
    seen = {(row["rule"], row["profile"], row["reason"]): row for row in rows}
    return [seen[key] for key in sorted(seen)]


def _gate_scope(rows: Sequence[dict[str, str]]) -> dict[str, list[str]]:
    """The scopes each gated rule was held back over, by rule."""
    out: dict[str, list[str]] = {}
    for row in rows:
        scope = profile_name(row["profile"]) if row["profile"] else WHOLE_ATLAS
        found = out.setdefault(row["rule"], [])
        if scope not in found:
            found.append(scope)
    return {rule: sorted(scopes) for rule, scopes in out.items()}


def _header(atlas: Atlas, gates: Sequence[dict[str, str]]) -> list[str]:
    items = _findings(atlas)
    counts = _severity_counts(items)
    counters = _drift_counters(atlas)
    profiles = sorted({profile_name(name) for item in items for name in item.profiles})
    lines = [heading(1, "Drift"), ""]
    ran = counters.get("rules_run")
    lines.append(f"{plural(len(items), 'finding')}: {_summary(counts)}. {rendered_from(atlas)}.")
    lines.append(
        (
            f"{int(ran)} rules ran over this atlas"
            if ran is not None
            else "No drift run is recorded in this atlas"
        )
        + f" and {plural(len({item.rule for item in items}), 'rule')} produced something. "
        f"{plural(int(counters.get('waived', 0)), 'finding')} waived, "
        f"{plural(int(counters.get('rules_failed', 0)), 'rule')} failed."
    )
    if profiles:
        lines.append(
            f"Profiles carrying a finding: {joined(profiles)}. A finding with no profile holds "
            "regardless of which one runs."
        )
    scope = _gate_scope(gates)
    if scope:
        lines.append(
            f"A gate held {plural(len(scope), 'rule')} back over part of this atlas: "
            f"{joined(sorted(scope))}. Those rules state a lower bound, and what each gate "
            "kept out is the second table from the end of this file, which counts rows and "
            "not rules."
        )
    lines.append("")
    lines.append(
        "Each entry below carries the sentence to quote, the file and line to edit, why the "
        "statement is true and what to change. A finding id is the waiver key, so a rename of a "
        "rule never silently un waives anything."
    )
    by_rule: dict[str, list[Finding]] = {}
    for item in items:
        by_rule.setdefault(item.rule, []).append(item)
    rows = []
    for rule in sorted(by_rule):
        here = _severity_counts(by_rule[rule])
        rows.append(
            [
                rule,
                len(by_rule[rule]),
                here.get("error", 0),
                here.get("warn", 0),
                here.get("info", 0),
                joined(
                    sorted(
                        {profile_name(name) for item in by_rule[rule] for name in item.profiles}
                    ),
                    NO_PROFILE,
                ),
            ]
        )
    lines.append("")
    lines.extend(table(["rule", "findings", "error", "warn", "info", "profiles"], rows))
    return lines


# ---------------------------------------------------------------------------
# the findings


def _families(atlas: Atlas) -> list[str]:
    items = _findings(atlas)
    grouped: dict[str, list[Finding]] = {}
    for item in items:
        grouped.setdefault(_family_of(item.rule), []).append(item)
    order = sorted(
        grouped,
        key=lambda family: (
            min(SEVERITY_ORDER.get(item.severity, 9) for item in grouped[family]),
            -len(grouped[family]),
            family,
        ),
    )
    lines: list[str] = []
    for family in order:
        here = sorted(grouped[family], key=Finding.sort_key)
        lines.append("")
        lines.append(heading(2, f"{FAMILY_NAMES.get(family, family)}, {len(here)}"))
        lines.append("")
        spread = plural(len({item.rule for item in here}), "rule")
        lines.append(f"{_summary(_severity_counts(here))}, over {spread}.")
        # Info findings of one rule about one file are one section, so a launch file with
        # forty unresolved remaps does not bury the rest of the report.
        bundles: dict[tuple[str, str], list[Finding]] = {}
        for item in here:
            if item.severity == "info" and isinstance(item.subject, str):
                bundles.setdefault((item.rule, item.subject), []).append(item)
        done: set[tuple[str, str]] = set()
        for item in here:
            key = (item.rule, item.subject) if isinstance(item.subject, str) else None
            bundle = bundles.get(key) if key else None
            if bundle is None or len(bundle) == 1:
                lines.extend(_finding(item))
            elif key not in done:
                done.add(key)
                lines.extend(_bundle(bundle))
    return lines


def _bundle(items: Sequence[Finding]) -> list[str]:
    first = items[0]
    lines = ["", heading(3, f"{len(items)} {first.rule} findings about {label(first.subject)}")]
    lines.append("")
    scope = joined(
        sorted({profile_name(name) for item in items for name in item.profiles}), NO_PROFILE
    )
    lines.append(f"`{first.severity}` `{first.rule}`, {scope}")
    if first.fix:
        lines.append("")
        lines.append(f"Fix, for each: {first.fix}")
    lines.append("")
    for item in items:
        where = ", ".join(site(source, locator=True) for source in item.where)
        lines.append(f"* {item.spoken}" + (f" ({where})" if where else "") + f" `{item.id}`")
    return lines


def _finding(item: Finding) -> list[str]:
    lines = ["", heading(3, item.title or item.rule), ""]
    scope = joined((profile_name(name) for name in item.profiles), NO_PROFILE)
    lines.append(f"`{item.severity}` `{item.rule}`, {scope}")
    if item.say:
        lines.append("")
        lines.append(f"> {item.spoken}")
    if item.where:
        lines.append("")
        lines.append("Edit:")
        for source in item.where:
            lines.append(f"* {site(source, locator=True)}")
    if item.detail:
        lines.append("")
        lines.append(f"Why: {item.detail}")
    if item.fix:
        lines.append("")
        lines.append(f"Fix: {item.fix}")
    lines.append("")
    lines.append(f"About {_subject(item)}. Waiver key `{item.id}`.")
    return lines


def _subject(item: Finding) -> str:
    if isinstance(item.subject, str):
        return f"{item.subject.split(':', 1)[0]} {label(item.subject)}"
    return f"{label(item.subject.src)} {item.subject.kind} {label(item.subject.dst)}"


# ---------------------------------------------------------------------------
# the rules a gate held back, and the rules that said nothing


def _held_back(gates: Sequence[dict[str, str]]) -> list[str]:
    """The heading counts rules, the same unit the header sentence counts.

    One family emits a row per gated block per profile, so counting rows here printed a
    larger number than the header for one atlas, and the two read as a contradiction while
    both were true. The row count is stated in the paragraph, with its unit.
    """
    scope = _gate_scope(gates)
    lines = ["", heading(2, f"Rules a gate held back, {len(scope)}"), ""]
    if not gates:
        lines.append(
            "No gate held any rule back on this atlas, so every rule below either ran to the "
            "end or could not run at all, and the next table says which."
        )
        return lines
    lines.append(
        "A gate is a rule refusing to speak where it cannot know. The heading counts rules, "
        f"the table counts rows: {plural(len(scope), 'rule')} over "
        f"{plural(len(gates), 'row')}, one row per rule, profile and reason, so a rule gated "
        "twice is one rule and two rows. A rule named here may also carry findings above, "
        "from the part of the atlas the gate does not cover, so its findings are a lower "
        "bound and not a full count."
    )
    lines.append("")
    lines.extend(
        table(
            ["rule", "profile", "what the gate held back"],
            [
                [
                    row["rule"],
                    profile_name(row["profile"]) if row["profile"] else WHOLE_ATLAS,
                    row["reason"],
                ]
                for row in gates
            ],
        )
    )
    return lines


def _severity(atlas: Atlas, rule: str) -> str:
    """The severity the rule fires at on this atlas: the budget rule is info until a budget
    is stated."""
    if (
        rule == "resolution.below_budget"
        and (atlas.x.get("drift") or {}).get("resolution_budget") is None
    ):
        return rules.severity_of(rule, "unbudgeted")
    base = rules.SEVERITY.get(rule, "")
    # A rule that fires at another severity in some cases says both.
    other = {table[rule] for table in rules.VARIANT_SEVERITY.values() if rule in table}
    return (
        " or ".join(sorted({base} | other, key=SEVERITY_ORDER.__getitem__))
        if other - {base}
        else base
    )


def _silences(atlas: Atlas, gates: Sequence[dict[str, str]]) -> list[str]:
    fired = {item.rule for item in atlas.findings}
    counters = _drift_counters(atlas)
    failed = set(counters.get("failed") or [])
    scope = _gate_scope(gates)
    quiet = [rule for rule in rules.COMMITTED if rule not in fired]
    held = [rule for rule in quiet if rule in scope]
    lines = ["", heading(2, f"Rules that said nothing, {len(quiet)}"), ""]
    lines.append(
        "A silent rule is a claim too: it says this atlas holds no instance of that defect, or "
        "that the layer the rule reads is not in this atlas, or that a gate held it back. The "
        "third is not the first, so each row says which one it is."
    )
    present = {name for name, status in atlas.layers.items() if status.present}
    ran = int(counters.get("rules_run", 0))
    unrecorded = not ran
    rows = []
    for rule in quiet:
        needs = NEEDS_LAYER.get(rule)
        if rule == "rate.below_declared" and "recorded" in present:
            needs = None
        if rule in failed:
            reason = "the rule failed this run and its result was not trusted"
        elif unrecorded:
            reason = "no drift run is recorded in this atlas, so the rule never ran"
        elif rule in scope:
            reason = f"{GATED_REASON} over {joined(scope[rule])}, stated in the table above"
        elif needs and needs not in present:
            reason = f"needs the {needs} layer, which this atlas does not have"
        else:
            reason = "ran over this atlas and matched nothing"
        rows.append([rule, _severity(atlas, rule), reason])
    lines.append("")
    lines.extend(table(["rule", "severity when it fires", "why it is silent"], rows))
    if held:
        lines.append("")
        lines.append(
            f"A gate is why {plural(len(held), 'rule')} in this list said nothing: "
            f"{joined(sorted(held))}. Read those rows as an unanswered question and not as a "
            "clean result."
        )
    if unrecorded or 0 < ran < len(rules.COMMITTED):
        lines.append("")
        lines.append(
            f"{ran} of the {len(rules.COMMITTED)} committed rules ran, so this list also holds "
            "rules that were never loaded. A rule that does not run reads exactly like a clean "
            "stack, which is why the count is stated."
        )
    return lines


__all__ = ["render_drift"]
