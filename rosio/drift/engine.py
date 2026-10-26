"""The drift engine: it runs the rules and turns their output into the report.

A rule is a pure function from the joined atlas to a list of findings. Everything around
that belongs here: a fixed run order, isolation of a rule that raises, the closed
vocabulary check, deduplication by fingerprint, waivers, severity and profile filtering,
and the counters. The engine never reads a file, never re extracts and never mutates an
entity.

One registry. `rosio.drift.rules` owns the committed ids, the alias table, the severity
per rule and the map from an id to its implementation, and the engine takes its rules from
there. It used to keep a second registry of its own, with a second `register` and a second
`canonical`, reached from no production path and from one test, so somebody arriving to add
a rule met two of everything and could not tell which one ran.

Determinism. Rules run in sorted rule id order, so the order the family modules happen to
import in cannot reach the output. Findings sort by (severity, rule, subject,
discriminator). No clock, no ordinal and no iteration order enters a finding id, a
fingerprint or a counter, so two runs over one atlas produce identical findings.

Isolation. A rule that raises is caught, recorded in `failures` with its exception type and
message, and the run continues. A rule that produces a finding labelled with another rule's
id, a finding outside the committed vocabulary, or a finding whose prose carries a synthetic
id is refused the same way: the run keeps going and the report says which rule misbehaved.
"""

from __future__ import annotations

import dataclasses
import importlib
from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

from rosio.drift import rules as registry
from rosio.errors import ContractError, RosioError
from rosio.ids import KIND_SEP, SYNTHETIC
from rosio.model import SEVERITY_ORDER, Atlas, Finding

RuleFn = Callable[[Atlas], Iterable[Finding]]

_SYNTHETIC_SIGIL = KIND_SEP + SYNTHETIC


@dataclass(frozen=True)
class Rule:
    """One rule the engine runs: a committed rule id and the pure function behind it."""

    id: str
    fn: RuleFn

    def sort_key(self) -> str:
        return self.id


# ---------------------------------------------------------------------------
# vocabulary and discovery


def load_rules(family_names: Sequence[str] | None = None) -> tuple[list[Rule], list[RuleFailure]]:
    """Every implemented rule, sorted by rule id, with whatever could not be loaded.

    There is one registry and `rosio.drift.rules` owns it, so a contributor adding a rule
    meets one `register` and one `canonical`. The engine holds no rules of its own: it asks
    that module what is implemented and keeps the order, the isolation, the deduplication,
    the filtering and the counters. A caller with a rule of its own passes it to `run`.
    """
    failures: list[RuleFailure] = []
    if family_names is not None:
        failures.extend(_import_families(family_names))
    found: dict[str, Rule] = {}
    try:
        for name in registry.registered():
            found[name] = Rule(id=name, fn=_bound(name))
    except Exception as exc:  # a rule family that cannot import must not empty the run
        failures.append(_failure(registry.__name__, "load", exc))
    return [found[key] for key in sorted(found)], sorted(failures, key=RuleFailure.sort_key)


def _bound(rule_id: str) -> RuleFn:
    """One rule of the registry, called through the module's public surface."""

    def run_one(atlas: Atlas) -> Iterable[Finding]:
        return registry.findings_for(atlas, only=[rule_id])

    run_one.__name__ = f"rule_{rule_id.replace('.', '_')}"
    return run_one


def _import_families(family_names: Sequence[str]) -> list[RuleFailure]:
    failures: list[RuleFailure] = []
    for name in family_names:
        dotted = name if "." in name else f"rosio.drift.{name}"
        try:
            importlib.import_module(dotted)
        except ModuleNotFoundError as exc:
            if getattr(exc, "name", "") != dotted:
                failures.append(_failure(name, "load", exc))
        except Exception as exc:
            failures.append(_failure(name, "load", exc))
    return failures


def _committed() -> frozenset[str]:
    return frozenset(str(item) for item in registry.COMMITTED)


def _aliases() -> dict[str, str]:
    return {str(key): str(value) for key, value in (registry.ALIASES or {}).items()}


def _current(rule_id: str, aliases: Mapping[str, str]) -> str:
    """The id a rule is committed under today, following one rename hop.

    `rules.canonical` refuses an id outside the vocabulary, which is right where a rule
    registers itself and wrong here: the engine reports an invented id as one rule's failure
    and keeps running the others.
    """
    return aliases.get(rule_id, rule_id)


# ---------------------------------------------------------------------------
# results


@dataclass(frozen=True)
class RuleFailure:
    """A rule that could not be trusted for this run. Recorded, never fatal."""

    rule: str
    kind: str
    exc_type: str
    message: str

    def sort_key(self) -> tuple[str, str, str, str]:
        return (self.rule, self.kind, self.exc_type, self.message)

    def to_json(self) -> dict[str, str]:
        return {
            "rule": self.rule,
            "kind": self.kind,
            "exc_type": self.exc_type,
            "message": self.message,
        }


@dataclass
class DriftRun:
    """Everything one engine pass produced.

    `findings` is what the caller asked to see, so `min_severity` has already taken the
    quieter rows out of it. `gated` is the same run before that filter, and it is what the
    CI gate reads: the gate severity and the print threshold are two decisions, and a
    build that prints only errors must still fail on a warning when it was told to.
    """

    findings: list[Finding] = field(default_factory=list)
    gated: list[Finding] = field(default_factory=list)
    waived: list[str] = field(default_factory=list)
    failures: list[RuleFailure] = field(default_factory=list)
    counters: dict[str, Any] = field(default_factory=dict)
    rules: list[str] = field(default_factory=list)

    def by_severity(self, severity: str) -> list[Finding]:
        return [item for item in self.findings if item.severity == severity]

    def to_json(self) -> dict[str, Any]:
        return {
            "counters": dict(self.counters),
            "rules": list(self.rules),
            "failures": [item.to_json() for item in self.failures],
            "waived": list(self.waived),
            "findings": [item.to_json() for item in self.findings],
        }


def _failure(rule: str, kind: str, exc: BaseException) -> RuleFailure:
    return RuleFailure(
        rule=rule,
        kind=kind,
        exc_type=type(exc).__name__,
        message=" ".join(str(exc).split())[:400],
    )


# ---------------------------------------------------------------------------
# the run


def run(
    atlas: Atlas,
    *,
    rules: Sequence[Rule] | None = None,
    waivers: Iterable[str] = (),
    only: Iterable[str] = (),
    profiles: Iterable[str] = (),
    min_severity: str | None = None,
) -> DriftRun:
    """Run every registered rule over one joined atlas.

    `only` restricts the run to a set of rule ids, `profiles` drops findings that name a
    profile outside the set (a finding that names none holds in all of them),
    `min_severity` drops the quieter findings from `findings` and never from `gated`, and
    `waivers` holds finding ids or fingerprints that have been accepted.
    """
    known, aliases = _committed(), _aliases()
    if rules is None:
        loaded, failures = load_rules()
    else:
        loaded, failures = list(rules), []
    wanted = {str(item) for item in only}
    selected = [
        rule
        for rule in sorted(loaded, key=Rule.sort_key)
        if not wanted or _asked_for(rule, wanted, aliases)
    ]
    failures.extend(_unimplemented(wanted, loaded, aliases))

    produced: list[Finding] = []
    for rule in selected:
        if known and _current(rule.id, aliases) not in known:
            failures.append(
                RuleFailure(
                    rule=rule.id,
                    kind="vocabulary",
                    exc_type="ContractError",
                    message="rule id is not in the committed vocabulary",
                )
            )
            continue
        try:
            emitted = list(rule.fn(atlas))
        except Exception as exc:  # one bad rule must not empty the report
            failures.append(_failure(rule.id, "raised", exc))
            continue
        for item in emitted:
            try:
                problem = _reject(rule, item)
            except Exception as exc:  # a finding the engine cannot even inspect
                problem = _failure(rule.id, "malformed", exc)
            if problem is not None:
                failures.append(problem)
                continue
            produced.append(item)

    scoped = _scope(produced, profiles)
    ordered = sorted(scoped, key=Finding.sort_key)
    deduped, collisions = _dedupe(ordered)
    failures.extend(collisions)
    gated, matched = waive(deduped, waivers)

    result = DriftRun(
        findings=_above(gated, min_severity),
        gated=gated,
        waived=sorted(matched),
        failures=sorted(failures, key=RuleFailure.sort_key),
        rules=[rule.id for rule in selected],
    )
    result.counters = _counters(
        result, produced=len(produced), scoped=len(scoped), deduped=len(deduped)
    )
    return result


def _asked_for(rule: Rule, wanted: set[str], aliases: Mapping[str, str]) -> bool:
    return bool({rule.id, _current(rule.id, aliases)} & (wanted | _canonical_all(wanted, aliases)))


def _canonical_all(wanted: Iterable[str], aliases: Mapping[str, str]) -> set[str]:
    return {_current(item, aliases) for item in wanted}


def _unimplemented(
    wanted: set[str], loaded: Sequence[Rule], aliases: Mapping[str, str]
) -> list[RuleFailure]:
    """A rule id asked for by name that no rule implements.

    Silence here would be worse than a failure row: a run restricted to a mistyped rule id
    reports no drift, which reads exactly like a clean stack.
    """
    have = {rule.id for rule in loaded} | _canonical_all([rule.id for rule in loaded], aliases)
    return [
        RuleFailure(
            rule=name,
            kind="unimplemented",
            exc_type="UsageError",
            message="no rule with this id is implemented, so nothing was checked for it",
        )
        for name in sorted(wanted)
        if name not in have and _current(name, aliases) not in have
    ]


def _reject(rule: Rule, item: Any) -> RuleFailure | None:
    if not isinstance(item, Finding):
        return RuleFailure(
            rule=rule.id,
            kind="mislabelled",
            exc_type="ContractError",
            message=f"a rule must yield Finding objects, got {type(item).__name__}",
        )
    if item.rule != rule.id:
        return RuleFailure(
            rule=rule.id,
            kind="mislabelled",
            exc_type="ContractError",
            message=f"finding is labelled {item.rule}",
        )
    if _SYNTHETIC_SIGIL in item.say or _SYNTHETIC_SIGIL in item.title:
        return RuleFailure(
            rule=rule.id,
            kind="synthetic",
            exc_type="ContractError",
            message="say or title carries a synthetic id, use the label",
        )
    try:
        # An id that cannot be formed would otherwise raise later, outside the rule, and
        # take the whole run with it. A digest of eight digits is refused as an ordinal,
        # so a discriminator needs a prefix that is not a digit.
        _handles = (item.id, item.fingerprint)
    except RosioError as exc:
        return _failure(rule.id, "malformed", exc)
    return None


def _scope(findings: Sequence[Finding], profiles: Iterable[str]) -> list[Finding]:
    """The findings that belong to the profiles asked for. One that names no profile holds
    in every profile, so it stays."""
    wanted = {str(item) for item in profiles}
    if not wanted:
        return list(findings)
    return [item for item in findings if not item.profiles or wanted.intersection(item.profiles)]


def _above(findings: Sequence[Finding], min_severity: str | None) -> list[Finding]:
    """The print filter. It never reaches the gate, see `crosses_gate`."""
    if not min_severity:
        return list(findings)
    floor = SEVERITY_ORDER.get(min_severity, len(SEVERITY_ORDER))
    return [
        item for item in findings if SEVERITY_ORDER.get(item.severity, len(SEVERITY_ORDER)) <= floor
    ]


def _dedupe(ordered: Sequence[Finding]) -> tuple[list[Finding], list[RuleFailure]]:
    """One finding per fingerprint, and one finding per id.

    A rule that reaches one conclusion twice prints it once. The fingerprint covers the
    rule id, so this never folds two rules into one: two rules that agree about a subject
    both print, and separating them belongs to the vocabulary and not to the engine.
    Two findings that share an id while their fingerprints differ are a rule bug, because
    a waiver on that id would silence both, so the collision is recorded.
    """
    kept: list[Finding] = []
    seen_prints: set[str] = set()
    seen_ids: dict[str, Finding] = {}
    failures: list[RuleFailure] = []
    for item in ordered:
        if item.fingerprint in seen_prints:
            continue
        first = seen_ids.get(item.id)
        if first is not None:
            failures.append(
                RuleFailure(
                    rule=item.rule,
                    kind="duplicate",
                    exc_type="ContractError",
                    message=(
                        "two findings share one id with different content, "
                        "the discriminator does not separate them"
                    ),
                )
            )
            continue
        seen_prints.add(item.fingerprint)
        seen_ids[item.id] = item
        kept.append(item)
    return kept, failures


def waive(
    findings: Sequence[Finding], waivers: Iterable[str]
) -> tuple[list[Finding], dict[str, str]]:
    """The findings a gate still grades, and which waiver key accepted each of the rest.

    A waiver key is a finding id or a fingerprint. A waived finding is not deleted here: the
    caller keeps printing it and only the gate stops reading it, because a finding nobody can
    see is a finding nobody ever removes the waiver for.

    A rename is followed. Both keys carry the rule id, the id in its text and the fingerprint
    in its digest, so a rule that gains an `ALIASES` entry would otherwise un waive every
    stored waiver on it. Each finding therefore also answers to the keys it would have had
    under every retired id that now points at its rule.
    """
    accepted = {str(item) for item in waivers}
    if not accepted:
        return list(findings), {}
    retired = _retired_by_rule(_aliases())
    kept: list[Finding] = []
    matched: dict[str, str] = {}
    for item in findings:
        hit = next((key for key in _keys_of(item, retired) if key in accepted), "")
        if hit:
            matched[item.id] = hit
            continue
        kept.append(item)
    return kept, matched


def _retired_by_rule(aliases: Mapping[str, str]) -> dict[str, list[str]]:
    out: dict[str, list[str]] = {}
    for old, new in aliases.items():
        out.setdefault(new, []).append(old)
    return {key: sorted(value) for key, value in out.items()}


def _keys_of(item: Finding, retired: Mapping[str, Sequence[str]]) -> list[str]:
    keys = [item.id, item.fingerprint]
    for old in retired.get(item.rule, ()):
        under = dataclasses.replace(item, rule=old)
        keys.extend([under.id, under.fingerprint])
    return keys


def _counters(result: DriftRun, *, produced: int, scoped: int, deduped: int) -> dict[str, Any]:
    severities = dict.fromkeys(sorted(SEVERITY_ORDER), 0)
    by_rule: dict[str, int] = {}
    for item in result.findings:
        severities[item.severity] = severities.get(item.severity, 0) + 1
        by_rule[item.rule] = by_rule.get(item.rule, 0) + 1
    return {
        "rules_run": len(result.rules),
        "rules_failed": len({item.rule for item in result.failures}),
        "produced": produced,
        "filtered": (produced - scoped) + (len(result.gated) - len(result.findings)),
        "deduplicated": scoped - deduped,
        "waived": len(result.waived),
        "gated": len(result.gated),
        "findings": severities,
        "by_rule": dict(sorted(by_rule.items())),
    }


# ---------------------------------------------------------------------------
# writing back, and the gate


def apply(atlas: Atlas, result: DriftRun) -> Atlas:
    """Write the findings and the counters onto the atlas, in place."""
    atlas.findings = sorted(result.findings, key=Finding.sort_key)
    severities = dict.fromkeys(sorted(SEVERITY_ORDER), 0)
    for item in atlas.findings:
        severities[item.severity] = severities.get(item.severity, 0) + 1
    atlas.counters["findings"] = severities
    atlas.counters["drift"] = {
        key: value for key, value in result.counters.items() if key != "findings"
    }
    by_rule: dict[str, list[str]] = {}
    for item in atlas.findings:
        by_rule.setdefault(item.rule, []).append(item.id)
    atlas.indexes["findings_by_rule"] = {key: by_rule[key] for key in sorted(by_rule)}
    return atlas


def crosses_gate(result: DriftRun, fail_on: str = "never") -> bool:
    """Whether the run should fail the build.

    The gate severity and the print threshold are two decisions, so this reads the run
    before the print filter took anything out of it. A build told to print errors and to
    fail on warnings fails on a warning it never printed.
    """
    if fail_on == "never":
        return False
    floor = SEVERITY_ORDER.get(fail_on)
    if floor is None:
        raise ContractError(f"fail_on must be error, warn, info or never, got {fail_on!r}")
    return any(
        SEVERITY_ORDER.get(item.severity, len(SEVERITY_ORDER)) <= floor
        for item in (result.gated or result.findings)
    )


__all__ = [
    "DriftRun",
    "Rule",
    "RuleFailure",
    "apply",
    "crosses_gate",
    "load_rules",
    "run",
    "waive",
]
