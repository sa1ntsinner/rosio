"""The drift engine: run order, isolation, dedup, waivers, counters and determinism."""

from __future__ import annotations

import pytest

from rosio import ids, model
from rosio.drift import engine, rules
from rosio.errors import ContractError
from rosio.model import Finding, Source

REAL = "profile:nx_navigation"
SIM = "profile:nx_sim"


@pytest.fixture
def atlas() -> model.Atlas:
    builder = model.AtlasBuilder()
    builder.add_profile("nx_navigation")
    builder.add_profile("nx_sim")
    file_id = ids.build_file("nx_bringup", "config/nav2_params.yaml")
    builder.node(file_id)
    node_id = ids.build_node("/planner_server")
    builder.fact(node_id, "declared", profile=REAL, values={"pkg": "nav2_planner"})
    builder.node(node_id, name_source="literal")
    param_id = ids.build_param(node_id, "GridBased.cost_penalty")
    builder.set_param(
        param_id,
        state="effective",
        candidates=[
            model.Candidate(
                value=1.0,
                value_type="float",
                mechanism="yaml_overlay_merge",
                source=Source(file=file_id, locator="planner_server", line_hint=154),
                rank=0,
                wins=True,
                profile=REAL,
            )
        ],
    )
    return builder.build()


def a_finding(rule: str = "param.shadowed", **over) -> Finding:
    fields = {
        "rule": rule,
        "severity": "warn",
        "subject": ids.build_param(ids.build_node("/planner_server"), "GridBased.cost_penalty"),
        "title": "cost_penalty is set twice",
        "detail": "nav2_params.yaml states 2.0 and the overlay states 1.0",
        "fix": "delete the losing line",
        "say": "cost_penalty on planner_server is 1.0, set at nav2_overrides.yaml:154",
        "profiles": [REAL],
    }
    fields.update(over)
    return Finding(**fields)


def rule_of(rule_id: str, *findings: Finding) -> engine.Rule:
    """A rule handed straight to `run`, so a test never depends on which family modules the
    package happens to implement and never writes into the one registry."""

    def run_rule(_atlas):
        return list(findings)

    return engine.Rule(id=rule_id, fn=run_rule)


def run(atlas, *rules: engine.Rule, **over):
    return engine.run(atlas, rules=list(rules), **over)


# ---------------------------------------------------------------------------
# registration


def test_the_engine_keeps_no_registry_of_its_own():
    """One registry, and the vocabulary module owns it.

    The engine used to carry a second one, with a second `register` and a second
    `canonical`, that no production path reached. Somebody arriving to add a rule met both
    and could not tell which one ran.
    """
    gone = {"REGISTRY", "register", "unregister", "canonical", "vocabulary"}

    assert set(engine.__all__) & gone == set()
    assert [name for name in gone if callable(getattr(engine, name, None))] == []
    assert "register" in rules.__all__ and "canonical" in rules.__all__


def test_rules_run_in_sorted_id_order_whatever_the_registration_order(atlas):
    seen: list[str] = []
    names = ("topic.no_subscriber", "param.shadowed", "frame.no_publisher")
    rules = [
        engine.Rule(id=name, fn=(lambda label: lambda _atlas: seen.append(label) or [])(name))
        for name in names
    ]

    run(atlas, *rules)

    assert seen == ["frame.no_publisher", "param.shadowed", "topic.no_subscriber"]


def test_a_family_module_that_is_absent_is_not_an_error():
    _rules, failures = engine.load_rules(["a_family_that_does_not_exist"])

    assert failures == []


def test_every_discovered_rule_is_in_the_committed_vocabulary():
    loaded, failures = engine.load_rules()

    assert rules.COMMITTED, "the vocabulary module states the committed rule ids"
    assert [rule.id for rule in loaded] == sorted(rule.id for rule in loaded)
    assert [rule.id for rule in loaded if rule.id not in rules.COMMITTED] == []
    assert failures == []


def test_the_rules_the_engine_runs_are_the_ones_the_registry_holds():
    loaded, _failures = engine.load_rules()

    assert [rule.id for rule in loaded] == list(rules.registered())


# ---------------------------------------------------------------------------
# isolation


def test_a_rule_that_raises_is_recorded_and_the_run_continues(atlas):
    def explodes(_atlas):
        raise ZeroDivisionError("bad arithmetic in a rule")

    result = run(
        atlas,
        engine.Rule(id="topic.no_subscriber", fn=explodes),
        rule_of("param.shadowed", a_finding()),
    )

    assert [item.rule for item in result.findings] == ["param.shadowed"]
    assert [(row.rule, row.kind, row.exc_type) for row in result.failures] == [
        ("topic.no_subscriber", "raised", "ZeroDivisionError")
    ]
    assert result.counters["rules_failed"] == 1


def test_a_finding_labelled_with_another_rule_is_refused(atlas):
    result = run(atlas, rule_of("param.shadowed", a_finding(rule="topic.no_subscriber")))

    assert result.findings == []
    assert result.failures[0].kind == "mislabelled"


def test_a_synthetic_id_in_prose_is_refused(atlas):
    synthetic = ids.build_node_synthetic("nav2_controller", "controller_server", "/", "75ed3551")
    broken = rule_of(
        "node.unnamed", a_finding(rule="node.unnamed", subject=synthetic, say=f"see {synthetic}")
    )

    result = run(atlas, broken)

    assert result.findings == []
    assert result.failures[0].kind == "synthetic"


def test_a_rule_outside_the_committed_vocabulary_is_refused(atlas, monkeypatch):
    monkeypatch.setattr(rules, "COMMITTED", ("param.shadowed",))

    result = run(
        atlas,
        rule_of("param.invented", a_finding(rule="param.invented")),
        rule_of("param.shadowed", a_finding()),
    )

    assert [item.rule for item in result.findings] == ["param.shadowed"]
    assert [(row.rule, row.kind) for row in result.failures] == [("param.invented", "vocabulary")]


def test_a_deprecated_rule_id_passes_through_its_alias(atlas, monkeypatch):
    monkeypatch.setattr(rules, "COMMITTED", ("param.shadowed",))
    monkeypatch.setitem(rules.ALIASES, "param.overridden", "param.shadowed")
    result = run(atlas, rule_of("param.overridden", a_finding(rule="param.overridden")))

    assert [item.rule for item in result.findings] == ["param.overridden"]
    assert result.failures == []


# ---------------------------------------------------------------------------
# ordering, dedup, waivers, filters


def test_findings_sort_by_severity_then_rule_then_subject(atlas):
    node_id = ids.build_node("/planner_server")

    result = run(
        atlas,
        rule_of(
            "topic.no_subscriber",
            a_finding(rule="topic.no_subscriber", severity="info", subject=node_id),
        ),
        rule_of("param.shadowed", a_finding(severity="warn")),
        rule_of("node.unnamed", a_finding(rule="node.unnamed", severity="error", subject=node_id)),
    )

    assert [item.severity for item in result.findings] == ["error", "warn", "info"]


def test_two_rules_reaching_one_conclusion_report_it_once(atlas):
    result = run(atlas, rule_of("param.shadowed", a_finding(), a_finding()))

    assert len(result.findings) == 1
    assert result.counters["deduplicated"] == 1


def test_one_id_with_two_contents_is_a_collision_and_not_a_silent_drop(atlas):
    result = run(
        atlas, rule_of("param.shadowed", a_finding(), a_finding(detail="a different reason"))
    )

    assert len(result.findings) == 1
    assert [row.kind for row in result.failures] == ["duplicate"]


def test_a_waiver_matches_the_finding_id(atlas):
    finding = a_finding()

    result = run(atlas, rule_of("param.shadowed", finding), waivers=[finding.id])

    assert result.findings == []
    assert result.waived == [finding.id]
    assert result.counters["waived"] == 1


def test_a_waiver_matches_the_fingerprint(atlas):
    finding = a_finding()

    result = run(atlas, rule_of("param.shadowed", finding), waivers=[finding.fingerprint])

    assert result.findings == []


def test_a_stored_waiver_survives_a_rule_rename(monkeypatch):
    """AGENTS section 10.1: a rename never silently un waives a finding.

    A stored key holds the rule id, in its text for an id and in its digest for a
    fingerprint, so following the alias table backwards is the whole of the promise. The
    fingerprint half is the one the alias table cannot repair on its own.
    """
    stored = a_finding(rule="param.overridden")
    renamed = a_finding(rule="param.shadowed")
    assert stored.id != renamed.id and stored.fingerprint != renamed.fingerprint

    monkeypatch.setitem(rules.ALIASES, "param.overridden", "param.shadowed")

    by_id, matched_id = engine.waive([renamed], [stored.id])
    by_print, matched_print = engine.waive([renamed], [stored.fingerprint])

    assert by_id == [] and matched_id == {renamed.id: stored.id}
    assert by_print == [] and matched_print == {renamed.id: stored.fingerprint}


def test_a_waived_finding_is_still_a_finding(atlas):
    """The gate stops reading it and nothing else does.

    A finding nobody can see is a finding whose waiver nobody ever deletes, so `waive`
    hands back what a caller keeps printing rather than deleting a row.
    """
    finding = a_finding(severity="error")

    graded, matched = engine.waive([finding], [finding.id])

    assert graded == []
    assert matched == {finding.id: finding.id}
    assert engine.crosses_gate(engine.DriftRun(gated=graded), "error") is False
    assert engine.crosses_gate(engine.DriftRun(gated=[finding]), "error") is True


def test_a_finding_of_another_profile_is_dropped_when_a_profile_is_asked_for(atlas):
    rule = rule_of("param.shadowed", a_finding(profiles=[SIM]))

    assert run(atlas, rule, profiles=[REAL]).findings == []
    assert len(run(atlas, rule, profiles=[SIM]).findings) == 1
    assert len(run(atlas, rule).findings) == 1


def test_min_severity_drops_the_quieter_findings(atlas):
    node_id = ids.build_node("/planner_server")

    result = run(
        atlas,
        rule_of("param.shadowed", a_finding(severity="warn")),
        rule_of("node.unnamed", a_finding(rule="node.unnamed", severity="info", subject=node_id)),
        min_severity="warn",
    )

    assert [item.rule for item in result.findings] == ["param.shadowed"]
    assert result.counters["filtered"] == 1


def test_only_restricts_the_run_to_one_rule(atlas):
    result = run(
        atlas,
        rule_of("param.shadowed", a_finding()),
        rule_of(
            "topic.no_subscriber",
            a_finding(rule="topic.no_subscriber", subject=ids.build_node("/planner_server")),
        ),
        only=["param.shadowed"],
    )

    assert result.rules == ["param.shadowed"]
    assert [item.rule for item in result.findings] == ["param.shadowed"]


# ---------------------------------------------------------------------------
# determinism, counters and the gate


def test_two_runs_over_one_atlas_produce_identical_findings(atlas):
    rules = [
        rule_of("param.shadowed", a_finding()),
        rule_of(
            "topic.no_subscriber",
            a_finding(
                rule="topic.no_subscriber",
                severity="info",
                subject=ids.build_node("/planner_server"),
            ),
        ),
    ]

    first = run(atlas, *rules)
    second = run(atlas, *rules)

    assert [item.id for item in first.findings] == [item.id for item in second.findings]
    assert [item.fingerprint for item in first.findings] == [
        item.fingerprint for item in second.findings
    ]
    assert model.dumps(first.to_json()) == model.dumps(second.to_json())


def test_the_counters_say_what_happened(atlas):
    counters = run(atlas, rule_of("param.shadowed", a_finding(), a_finding())).counters

    assert counters["rules_run"] == 1
    assert counters["produced"] == 2
    assert counters["deduplicated"] == 1
    assert counters["findings"] == {"error": 0, "warn": 1, "info": 0}
    assert counters["by_rule"] == {"param.shadowed": 1}


def test_apply_writes_the_findings_and_the_counters_onto_the_atlas(atlas):
    result = run(atlas, rule_of("param.shadowed", a_finding()))

    engine.apply(atlas, result)

    assert [item.id for item in atlas.findings] == [item.id for item in result.findings]
    assert atlas.counters["findings"]["warn"] == 1
    assert atlas.indexes["findings_by_rule"]["param.shadowed"] == [result.findings[0].id]
    assert atlas.to_bytes()


def test_the_gate_reads_the_severity_and_not_the_print_filter(atlas):
    result = run(atlas, rule_of("param.shadowed", a_finding(severity="warn")))

    assert engine.crosses_gate(result, "never") is False
    assert engine.crosses_gate(result, "error") is False
    assert engine.crosses_gate(result, "warn") is True
    assert engine.crosses_gate(result, "info") is True
    with pytest.raises(ContractError):
        engine.crosses_gate(result, "loud")


def test_a_warning_the_print_filter_hid_still_fails_the_build(atlas):
    # --min-severity error prints errors only, --fail-on warn fails on a warning. One
    # flag must not carry both decisions, so the filtered warning still crosses the gate.
    result = run(atlas, rule_of("param.shadowed", a_finding(severity="warn")), min_severity="error")

    assert result.findings == []
    assert [item.severity for item in result.gated] == ["warn"]
    assert engine.crosses_gate(result, "warn") is True
    assert engine.crosses_gate(result, "error") is False


def test_a_waived_finding_does_not_cross_the_gate(atlas):
    finding = a_finding(severity="error")

    result = run(atlas, rule_of("param.shadowed", finding), waivers=[finding.id])

    assert result.gated == []
    assert engine.crosses_gate(result, "error") is False


def test_a_rule_id_asked_for_that_nothing_implements_is_recorded(atlas):
    # A run restricted to a mistyped rule id reports no drift, which reads exactly like a
    # clean stack unless the run says it checked nothing.
    result = run(atlas, rule_of("param.shadowed", a_finding()), only=["param.typo"])

    assert result.rules == []
    assert result.findings == []
    assert [(row.rule, row.kind) for row in result.failures] == [("param.typo", "unimplemented")]


def test_only_follows_the_alias_table(atlas, monkeypatch):
    monkeypatch.setattr(rules, "COMMITTED", ("param.shadowed",))
    monkeypatch.setitem(rules.ALIASES, "param.overridden", "param.shadowed")

    result = run(atlas, rule_of("param.shadowed", a_finding()), only=["param.overridden"])

    assert result.rules == ["param.shadowed"]
    assert result.failures == []


def test_a_finding_the_engine_cannot_inspect_is_recorded_and_not_fatal(atlas):
    finding = a_finding()
    finding.say = None

    result = run(atlas, rule_of("param.shadowed", finding))

    assert result.findings == []
    assert [row.kind for row in result.failures] == ["malformed"]


def test_apply_works_on_a_run_that_carries_no_counters(atlas):
    engine.apply(atlas, engine.DriftRun(findings=[a_finding()]))

    assert atlas.counters["findings"] == {"error": 0, "warn": 1, "info": 0}


def test_a_finding_whose_id_cannot_be_formed_is_recorded_and_not_fatal(atlas):
    # An eight digit digest is a legal digest and an illegal discriminator, and it must
    # not take the run down when it reaches the id.
    result = run(atlas, rule_of("param.shadowed", a_finding(discriminator="41490703")))

    assert result.findings == []
    assert [row.kind for row in result.failures] == ["malformed"]


def test_the_engine_runs_the_shipped_rules_over_an_atlas(atlas):
    result = engine.run(atlas)

    assert result.counters["rules_run"] == len(result.rules)
    assert [row for row in result.failures if row.kind == "raised"] == []
