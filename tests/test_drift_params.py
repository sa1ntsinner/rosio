"""The parameter and plugin rule families, and the registry they live in.

Every rule gets three tests: it fires on a crafted atlas, it stays silent on the near miss
that would make it a false positive, and its finding id is the same across two runs. The
crafted atlases are built through AtlasBuilder, so a rule can never be tested against a
shape the contract would refuse.
"""

from __future__ import annotations

import json
from typing import Any

import pytest

from rosio import ids, model, schema
from rosio.drift import params as params_rules
from rosio.drift import plugins as plugin_rules
from rosio.drift import rules
from rosio.errors import ContractError, UsageError

PROFILE = ids.build_profile("demo")
OTHER_PROFILE = ids.build_profile("other")
BASE = ids.build_file("demo_bringup", "config/params.yaml")
OVERLAY = ids.build_file("demo_bringup", "config/overrides.yaml")
LAUNCH = ids.build_file("demo_bringup", "launch/demo.launch.py")
WRITER = ids.build_file("@repo", "dx_app/launcher.py")
NODE = ids.build_node("/demo_server")

DECLARED = model.Layer.DECLARED.value
CODED = model.Layer.CODED.value


def _builder() -> model.AtlasBuilder:
    builder = model.AtlasBuilder()
    builder.node(PROFILE)
    builder.node(OTHER_PROFILE)
    for file_id in (BASE, OVERLAY, LAUNCH, WRITER):
        builder.node(file_id)
    builder.node(NODE, name_source=model.NameSource.LITERAL.value)
    return builder


def _source(file_id: str, locator: str, line: int | None = None) -> model.Source:
    return model.Source(file=file_id, locator=locator, line_hint=line)


def _param(
    builder: model.AtlasBuilder,
    name: str,
    candidates: list[model.Candidate],
    *,
    owner: str = NODE,
    state: str = model.ParamState.EFFECTIVE.value,
    profile: str = PROFILE,
    values: dict[str, Any] | None = None,
    evidence: list[model.Source] | None = None,
) -> str:
    param_id = ids.build_param(owner, name)
    row = {"name": name, "state": state}
    row.update(values or {})
    builder.fact(
        param_id,
        DECLARED,
        profile=profile,
        values=row,
        evidence=evidence or [candidates[0].source] if candidates else [],
    )
    if candidates:
        builder.set_param(param_id, state=state, candidates=candidates)
    return param_id


def _candidate(
    value: Any,
    mechanism: str,
    source: model.Source,
    *,
    rank: int = 0,
    wins: bool = False,
    shadow_kind: str | None = None,
    profile: str = PROFILE,
) -> model.Candidate:
    return model.Candidate(
        value=value,
        value_type=type(value).__name__,
        mechanism=mechanism,
        source=source,
        rank=rank,
        wins=wins,
        profile=profile,
        shadow_kind=shadow_kind,
        shadowed_by=0 if shadow_kind else None,
    )


def _noop(atlas: model.Atlas) -> list[model.Finding]:
    return []


OWNED = (
    "param.shadowed",
    "param.restated_identical",
    "param.set_undeclared",
    "param.declared_unread",
    "param.orphan_block",
    "param.type_changed",
    "param.owner_name_mismatch",
    "param.sim_time_mixed",
    "plugin.inert_by_design",
    "plugin.unused",
    "plugin.unresolved",
)


def _run(atlas: model.Atlas, rule: str) -> list[model.Finding]:
    return rules.findings_for(atlas, [rule])


def _ids(atlas: model.Atlas, rule: str) -> list[str]:
    return [finding.id for finding in _run(atlas, rule)]


# ---------------------------------------------------------------------------
# the registry


def test_the_committed_vocabulary_is_the_one_in_the_contract():
    assert len(rules.COMMITTED) == len(set(rules.COMMITTED)) == 48
    assert rules.COMMITTED[0] == "param.shadowed"
    assert set(rules.SEVERITY) == set(rules.COMMITTED)


def test_a_rule_id_outside_the_vocabulary_is_refused():
    with pytest.raises(ContractError):
        rules.register("param.made_up")
    with pytest.raises(ContractError):
        rules.canonical("param.made_up")


def test_an_alias_keeps_a_stored_finding_pointing_at_one_rule(monkeypatch):
    monkeypatch.setitem(rules.ALIASES, "param.overridden", "param.shadowed")
    assert rules.canonical("param.overridden") == "param.shadowed"
    assert rules.severity_of("param.overridden") == rules.severity_of("param.shadowed")


def test_every_rule_this_milestone_owns_has_exactly_one_implementation():
    assert set(OWNED) <= set(rules.registered())
    with pytest.raises(ContractError):
        rules.register("param.shadowed")(_noop)


def test_asking_for_a_rule_with_no_implementation_is_a_usage_error(monkeypatch):
    monkeypatch.setattr(rules, "_REGISTRY", {})
    monkeypatch.setattr(rules, "load", lambda: None)
    with pytest.raises(UsageError):
        rules.findings_for(_builder().atlas, ["param.shadowed"])


def test_a_severity_variant_is_named_data_and_not_a_number_in_a_rule():
    assert rules.severity_of("param.shadowed") == model.Severity.WARN.value
    assert rules.severity_of("param.shadowed", "self_contradiction") == model.Severity.ERROR.value
    assert rules.severity_of("param.orphan_block", "unproven") == model.Severity.INFO.value
    with pytest.raises(ContractError):
        rules.severity_of("param.shadowed", "invented")


# ---------------------------------------------------------------------------
# param.shadowed


def _shadowed_atlas() -> model.Atlas:
    builder = _builder()
    _param(
        builder,
        "max_speed",
        [
            _candidate(
                3.0,
                model.Mechanism.PYTHON_OVERRIDE.value,
                _source(WRITER, "build:demo_server.ros__parameters.max_speed", 88),
                rank=0,
                wins=True,
            ),
            _candidate(
                1.0,
                model.Mechanism.YAML_BASE.value,
                _source(BASE, "demo_server.ros__parameters.max_speed", 12),
                rank=1,
                shadow_kind=model.ShadowKind.VALUE_CHANGED.value,
            ),
        ],
    )
    return builder.recompute()


def test_param_shadowed_fires_when_a_config_line_never_takes_effect():
    found = _run(_shadowed_atlas(), "param.shadowed")
    assert len(found) == 1
    finding = found[0]
    assert finding.severity == model.Severity.WARN.value
    assert finding.subject == ids.build_param(NODE, "max_speed")
    assert "config/params.yaml:12" in finding.say
    assert "dx_app/launcher.py:88" in finding.say
    assert "3.0" in finding.say


def test_param_shadowed_is_silent_on_an_overlay_that_changes_its_own_base():
    builder = _builder()
    _param(
        builder,
        "max_speed",
        [
            _candidate(
                3.0,
                model.Mechanism.YAML_OVERLAY_MERGE.value,
                _source(OVERLAY, "demo_server.ros__parameters.max_speed", 4),
                rank=0,
                wins=True,
            ),
            _candidate(
                1.0,
                model.Mechanism.YAML_BASE.value,
                _source(BASE, "demo_server.ros__parameters.max_speed", 12),
                rank=1,
                shadow_kind=model.ShadowKind.VALUE_CHANGED.value,
            ),
        ],
    )
    assert _run(builder.recompute(), "param.shadowed") == []


def test_param_shadowed_is_silent_when_the_losing_site_is_an_inline_default():
    builder = _builder()
    _param(
        builder,
        "port",
        [
            _candidate(
                "/dev/ttyUSB0",
                model.Mechanism.YAML_WILDCARD.value,
                _source(BASE, "/**.ros__parameters.port", 5),
                rank=0,
                wins=True,
            ),
            _candidate(
                "/dev/ttyACM0",
                model.Mechanism.LAUNCH_INLINE_DICT.value,
                _source(LAUNCH, "parameters"),
                rank=1,
                shadow_kind=model.ShadowKind.VALUE_CHANGED.value,
            ),
        ],
    )
    assert _run(builder.recompute(), "param.shadowed") == []


def test_param_shadowed_is_silent_on_an_inert_block():
    builder = _builder()
    _param(
        builder,
        "FollowPath_OFF.max_speed",
        [
            _candidate(
                3.0,
                model.Mechanism.PYTHON_OVERRIDE.value,
                _source(WRITER, "build:x", 88),
                rank=0,
            ),
            _candidate(
                1.0,
                model.Mechanism.YAML_BASE.value,
                _source(BASE, "demo_server.ros__parameters.FollowPath_OFF.max_speed", 12),
                rank=1,
                shadow_kind=model.ShadowKind.VALUE_CHANGED.value,
            ),
        ],
        state=model.ParamState.INERT_BY_DESIGN.value,
    )
    assert _run(builder.recompute(), "param.shadowed") == []


def test_param_shadowed_is_silent_when_the_fact_confidence_is_unknown():
    builder = _builder()
    param_id = ids.build_param(NODE, "max_speed")
    builder.fact(
        param_id,
        DECLARED,
        profile=PROFILE,
        values={"name": "max_speed", "state": model.ParamState.EFFECTIVE.value},
        confidence=model.Confidence.UNKNOWN.value,
    )
    builder.set_param(
        param_id,
        state=model.ParamState.EFFECTIVE.value,
        candidates=[
            _candidate(
                3.0,
                model.Mechanism.PYTHON_OVERRIDE.value,
                _source(WRITER, "build:x", 88),
                rank=0,
                wins=True,
            ),
            _candidate(
                1.0,
                model.Mechanism.YAML_BASE.value,
                _source(BASE, "demo_server.ros__parameters.max_speed", 12),
                rank=1,
                shadow_kind=model.ShadowKind.VALUE_CHANGED.value,
            ),
        ],
    )
    assert _run(builder.recompute(), "param.shadowed") == []


def test_param_shadowed_is_silent_when_this_profile_does_not_select_the_block():
    """The entity keeps the strongest state of every profile, so a key that is live in the
    sim profile and inert in the real one reads as effective on the entity."""
    builder = _builder()
    param_id = ids.build_param(NODE, "FollowPath.weight")
    builder.fact(
        param_id,
        DECLARED,
        profile=PROFILE,
        values={"state": model.ParamState.INERT_BY_DESIGN.value, "block": "FollowPath"},
    )
    builder.fact(
        param_id,
        DECLARED,
        profile=OTHER_PROFILE,
        values={"state": model.ParamState.EFFECTIVE.value},
    )
    builder.set_param(
        param_id,
        state=model.ParamState.EFFECTIVE.value,
        candidates=[
            _candidate(
                3.0,
                model.Mechanism.PYTHON_OVERRIDE.value,
                _source(WRITER, "build:x", 88),
                wins=True,
            ),
            _candidate(
                1.0,
                model.Mechanism.YAML_BASE.value,
                _source(BASE, "demo_server.ros__parameters.FollowPath.weight", 12),
                rank=1,
                shadow_kind=model.ShadowKind.VALUE_CHANGED.value,
            ),
            _candidate(
                9.0,
                model.Mechanism.YAML_BASE.value,
                _source(OVERLAY, "demo_server.ros__parameters.FollowPath.weight", 30),
                wins=True,
                profile=OTHER_PROFILE,
            ),
        ],
    )
    assert _run(builder.recompute(), "param.shadowed") == []


def test_param_shadowed_is_silent_on_unknown_confidence_beside_a_layer_it_can_read():
    """A second layer that tracked the node says nothing about a value the declared layer
    did not track."""
    builder = _builder()
    param_id = ids.build_param(NODE, "max_speed")
    builder.fact(
        param_id,
        DECLARED,
        profile=PROFILE,
        values={"state": model.ParamState.EFFECTIVE.value},
        confidence=model.Confidence.UNKNOWN.value,
    )
    builder.fact(
        param_id,
        CODED,
        profile=PROFILE,
        values={"declared_in_code": True},
        confidence=model.Confidence.CERTAIN.value,
    )
    builder.set_param(
        param_id,
        state=model.ParamState.EFFECTIVE.value,
        candidates=[
            _candidate(
                3.0,
                model.Mechanism.PYTHON_OVERRIDE.value,
                _source(WRITER, "build:x", 88),
                wins=True,
            ),
            _candidate(
                1.0,
                model.Mechanism.YAML_BASE.value,
                _source(BASE, "demo_server.ros__parameters.max_speed", 12),
                rank=1,
                shadow_kind=model.ShadowKind.VALUE_CHANGED.value,
            ),
        ],
    )
    assert _run(builder.recompute(), "param.shadowed") == []


def test_param_shadowed_reports_one_file_contradicting_itself_at_error():
    builder = _builder()
    _param(
        builder,
        "max_speed",
        [
            _candidate(
                3.0,
                model.Mechanism.YAML_BASE.value,
                _source(BASE, "/**.ros__parameters.max_speed", 40),
                rank=0,
                wins=True,
            ),
            _candidate(
                1.0,
                model.Mechanism.YAML_BASE.value,
                _source(BASE, "demo_server.ros__parameters.max_speed", 12),
                rank=1,
                shadow_kind=model.ShadowKind.VALUE_CHANGED.value,
            ),
        ],
    )
    found = _run(builder.recompute(), "param.shadowed")
    assert [item.severity for item in found] == [model.Severity.ERROR.value]


def test_param_shadowed_groups_one_wildcard_line_into_one_finding():
    builder = _builder()
    for name in ("alpha", "beta", "gamma"):
        _param(
            builder,
            name,
            [
                _candidate(
                    2,
                    model.Mechanism.LAUNCH_PARAMS_FILE.value,
                    _source(OVERLAY, f"demo_server.ros__parameters.{name}", 7),
                    rank=0,
                    wins=True,
                ),
                _candidate(
                    1,
                    model.Mechanism.YAML_WILDCARD.value,
                    _source(BASE, "/**.ros__parameters", 3),
                    rank=1,
                    shadow_kind=model.ShadowKind.VALUE_CHANGED.value,
                ),
            ],
        )
    found = _run(builder.recompute(), "param.shadowed")
    assert len(found) == 1
    assert found[0].subject == BASE
    assert "3 parameters" in found[0].say


def test_param_shadowed_finding_id_is_stable_across_two_runs():
    atlas = _shadowed_atlas()
    assert _ids(atlas, "param.shadowed") == _ids(atlas, "param.shadowed")
    fingerprints = [item.fingerprint for item in _run(atlas, "param.shadowed")]
    assert fingerprints == [item.fingerprint for item in _run(_shadowed_atlas(), "param.shadowed")]


# ---------------------------------------------------------------------------
# param.restated_identical


def _restated_atlas() -> model.Atlas:
    builder = _builder()
    _param(
        builder,
        "use_sim_time",
        [
            _candidate(
                True,
                model.Mechanism.LAUNCH_INLINE_DICT.value,
                _source(LAUNCH, "parameters"),
                rank=0,
                wins=True,
            ),
            _candidate(
                True,
                model.Mechanism.YAML_BASE.value,
                _source(BASE, "demo_server.ros__parameters.use_sim_time", 9),
                rank=1,
                shadow_kind=model.ShadowKind.RESTATED_IDENTICAL.value,
            ),
        ],
    )
    return builder.recompute()


def test_param_restated_identical_names_the_line_to_delete():
    found = _run(_restated_atlas(), "param.restated_identical")
    assert len(found) == 1
    assert found[0].severity == model.Severity.INFO.value
    assert "config/params.yaml:9" in found[0].say
    assert "Delete" in found[0].say


def test_param_restated_identical_is_one_finding_for_one_line_in_every_profile():
    """Autoware: one restated line read by six profiles gave six findings with one heading."""
    builder = _builder()
    rows = []
    for profile in (PROFILE, OTHER_PROFILE):
        rows += [
            _candidate(
                True,
                model.Mechanism.LAUNCH_INLINE_DICT.value,
                _source(LAUNCH, "parameters"),
                rank=0,
                wins=True,
                profile=profile,
            ),
            _candidate(
                True,
                model.Mechanism.YAML_BASE.value,
                _source(BASE, "demo_server.ros__parameters.use_sim_time", 9),
                rank=1,
                shadow_kind=model.ShadowKind.RESTATED_IDENTICAL.value,
                profile=profile,
            ),
        ]
    param_id = _param(builder, "use_sim_time", rows)
    builder.fact(
        param_id,
        DECLARED,
        profile=OTHER_PROFILE,
        values={"name": "use_sim_time", "state": model.ParamState.EFFECTIVE.value},
        evidence=[rows[0].source],
    )
    found = _run(builder.recompute(), "param.restated_identical")
    assert len(found) == 1
    assert found[0].profiles == sorted([PROFILE, OTHER_PROFILE])
    assert "profiles " in found[0].say


def _restated_by_profile(sets: dict[str, dict[str, Any]]) -> model.Atlas:
    """One wildcard line restating, per profile, a value for each node that profile starts."""
    builder = _builder()
    owners = sorted({owner for chosen in sets.values() for owner in chosen})
    for owner in owners:
        node = ids.build_node(owner)
        builder.node(node, name_source=model.NameSource.LITERAL.value)
        rows = []
        for profile, chosen in sets.items():
            if owner not in chosen:
                continue
            rows += [
                _candidate(
                    chosen[owner],
                    model.Mechanism.YAML_BASE.value,
                    _source(OVERLAY, "/**.ros__parameters.use_sim_time"),
                    rank=0,
                    wins=True,
                    profile=profile,
                ),
                _candidate(
                    chosen[owner],
                    model.Mechanism.YAML_BASE.value,
                    _source(BASE, "/**.ros__parameters.use_sim_time", 3),
                    rank=1,
                    shadow_kind=model.ShadowKind.RESTATED_IDENTICAL.value,
                    profile=profile,
                ),
            ]
        profiles = sorted({row.profile for row in rows})
        param_id = _param(builder, "use_sim_time", rows, owner=node, profile=profiles[0])
        for profile in profiles[1:]:
            builder.fact(
                param_id,
                DECLARED,
                profile=profile,
                values={"name": "use_sim_time", "state": model.ParamState.EFFECTIVE.value},
                evidence=[rows[0].source],
            )
    return builder.recompute()


@pytest.mark.parametrize(
    "sets",
    [
        {PROFILE: {"/a": True, "/b": True}, OTHER_PROFILE: {"/a": True, "/c": True}},
        {PROFILE: {"/a": True}, OTHER_PROFILE: {"/a": False}},
    ],
)
def test_param_restated_identical_is_one_finding_when_profiles_restate_different_content(sets):
    """The line was merged per restated content and named by the line alone, so two profiles
    starting different node sets, or reading true in one and false in the other, wrote two
    findings under one id and the rule landed in counters.drift.failed."""
    found = _run(_restated_by_profile(sets), "param.restated_identical")
    assert len(found) == 1
    assert found[0].profiles == sorted([PROFILE, OTHER_PROFILE])
    assert "differs by profile" in found[0].detail


def test_param_restated_identical_is_silent_when_the_value_differs():
    builder = _builder()
    _param(
        builder,
        "use_sim_time",
        [
            _candidate(
                True,
                model.Mechanism.LAUNCH_INLINE_DICT.value,
                _source(LAUNCH, "parameters"),
                rank=0,
                wins=True,
            ),
            _candidate(
                False,
                model.Mechanism.YAML_BASE.value,
                _source(BASE, "demo_server.ros__parameters.use_sim_time", 9),
                rank=1,
                shadow_kind=model.ShadowKind.VALUE_CHANGED.value,
            ),
        ],
    )
    assert _run(builder.recompute(), "param.restated_identical") == []


def test_param_restated_identical_is_silent_when_no_line_can_be_named():
    builder = _builder()
    _param(
        builder,
        "use_sim_time",
        [
            _candidate(
                True,
                model.Mechanism.YAML_BASE.value,
                _source(BASE, "demo_server.ros__parameters.use_sim_time", 9),
                rank=0,
                wins=True,
            ),
            _candidate(
                True,
                model.Mechanism.LAUNCH_INLINE_DICT.value,
                _source(LAUNCH, "parameters"),
                rank=1,
                shadow_kind=model.ShadowKind.RESTATED_IDENTICAL.value,
            ),
        ],
    )
    assert _run(builder.recompute(), "param.restated_identical") == []


def _restated_pair(name: str, mechanism: str, source: model.Source) -> model.Atlas:
    """One yaml line restating the value the winning site already gives it."""
    builder = _builder()
    _param(
        builder,
        name,
        [
            _candidate("keep", mechanism, source, rank=0, wins=True),
            _candidate(
                "keep",
                model.Mechanism.YAML_BASE.value,
                _source(BASE, f"demo_server.ros__parameters.{name}", 9),
                rank=1,
                shadow_kind=model.ShadowKind.RESTATED_IDENTICAL.value,
            ),
        ],
    )
    return builder.recompute()


def test_param_restated_identical_is_a_warn_across_the_launcher_boundary():
    atlas = _restated_pair("device", model.Mechanism.LAUNCH_INLINE_DICT.value, _source(LAUNCH, "p"))
    found = _run(atlas, "param.restated_identical")
    assert [item.severity for item in found] == [model.Severity.WARN.value]

    written = _restated_pair("device", model.Mechanism.PYTHON_OVERRIDE.value, _source(WRITER, "w"))
    assert [item.severity for item in _run(written, "param.restated_identical")] == [
        model.Severity.WARN.value
    ]


def test_param_restated_identical_stays_info_between_two_parameter_documents():
    atlas = _restated_pair(
        "device",
        model.Mechanism.YAML_OVERLAY_MERGE.value,
        _source(OVERLAY, "demo_server.ros__parameters.device", 4),
    )
    found = _run(atlas, "param.restated_identical")
    assert [item.severity for item in found] == [model.Severity.INFO.value]


@pytest.mark.parametrize("name", ["use_sim_time", "qos_overrides./tf.publisher.depth"])
def test_param_restated_identical_stays_info_for_a_name_the_client_library_declares(name):
    atlas = _restated_pair(name, model.Mechanism.LAUNCH_INLINE_DICT.value, _source(LAUNCH, "p"))
    found = _run(atlas, "param.restated_identical")
    assert [item.severity for item in found] == [model.Severity.INFO.value]


def test_param_restated_identical_finding_id_is_stable_across_two_runs():
    assert _ids(_restated_atlas(), "param.restated_identical") == _ids(
        _restated_atlas(), "param.restated_identical"
    )


# ---------------------------------------------------------------------------
# param.set_undeclared


def _undeclared_atlas(
    name: str = "max_speed",
    declared: tuple[str, ...] = (),
    states_node: bool = True,
    auto_declare: bool | str = False,
    read: bool = False,
    plugins: int = 0,
    locator: str = "",
) -> model.Atlas:
    builder = _builder()
    for other in declared:
        builder.fact(
            ids.build_param(NODE, other),
            CODED,
            profile=PROFILE,
            values={"declared_in_code": True},
        )
    source_file = ids.build_file("demo_pkg", "src/demo_server.cpp")
    builder.node(source_file)
    if states_node:
        builder.atlas.entities[source_file].attrs["nodes"] = [{"name": "demo_server"}]
    values: dict = {"coded_sources": [source_file]}
    if plugins:
        values["exe"] = ids.build_exe("demo_pkg", "demo_server")
        for index in range(plugins):
            values["coded_sources"].append(ids.build_file("aa_solver", f"src/s{index}.cpp"))
    if auto_declare:
        values["auto_declare"] = auto_declare
    builder.fact(
        NODE,
        CODED,
        profile=PROFILE,
        values=values,
        attrs={"coded_params_known": True},
    )
    param_id = _param(
        builder,
        name,
        [
            _candidate(
                1.0,
                model.Mechanism.YAML_BASE.value,
                _source(BASE, locator or f"demo_server.ros__parameters.{name}", 12),
                rank=0,
                wins=True,
            )
        ],
    )
    if read:
        symbol = ids.build_sym("demo_pkg", "src/demo_server.cpp", "DemoServer::on_timer")
        builder.node(symbol)
        builder.edge(
            model.EdgeKind.READS_PARAM.value,
            symbol,
            param_id,
            key=ids.edge_key(profile=PROFILE, symbol=symbol, name=name, api="get_parameter"),
            layer=CODED,
            profile=PROFILE,
            values={"name": name, "api": "get_parameter", "symbol": symbol},
        )
    return builder.recompute()


def test_param_set_undeclared_fires_when_no_declaration_names_the_key():
    found = _run(_undeclared_atlas(), "param.set_undeclared")
    assert len(found) == 1
    assert found[0].severity == model.Severity.WARN.value
    assert "demo_pkg/src/demo_server.cpp" in found[0].say
    assert "config/params.yaml:12" in found[0].say


def test_param_set_undeclared_names_the_node_source_first_and_counts_in_plain_words():
    (found,) = _run(_undeclared_atlas(plugins=4), "param.set_undeclared")
    assert "demo_pkg/src/demo_server.cpp, aa_solver/src/s0.cpp, aa_solver/src/s1.cpp and " in (
        found.say
    )
    assert "and 2 more files declare no parameter" in found.say


def test_param_set_undeclared_is_silent_when_the_code_builds_a_name_at_run_time():
    """`declare_parameter(topic + "_config")` may be the declaration of this key."""
    atlas = _undeclared_atlas("odom0_config")
    owner = ids.build_sym("demo_pkg", "src/demo_server.cpp", "DemoServer::loadParams")
    atlas.entities[owner] = model.Node(id=owner, kind="sym", label=owner)
    atlas.unresolved.append(
        model.Unresolved(
            want=model.Want.PARAM_NAME.value,
            owner=owner,
            expr='topic_name + "_config"',
            reason="not_stated_in_source",
        )
    )
    assert _run(atlas, "param.set_undeclared") == []


def _with_blind(atlas: model.Atlas, expr: str) -> model.Atlas:
    owner = ids.build_sym("demo_pkg", "src/demo_server.cpp", "DemoServer::DemoServer")
    atlas.entities[owner] = model.Node(id=owner, kind="sym", label=owner)
    atlas.unresolved.append(
        model.Unresolved(
            want=model.Want.PARAM_NAME.value,
            owner=owner,
            expr=expr,
            reason="not_stated_in_source",
        )
    )
    return atlas


def test_param_set_undeclared_decides_per_key_when_a_name_is_built_at_run_time():
    """One blind declaration in slam_toolbox_common.cpp silenced min_laser_range, which no
    source names. A name built from `"sensor." + id` can never be max_speed."""
    atlas = _with_blind(_undeclared_atlas(), '"sensor." + sensor_id')
    found = _run(atlas, "param.set_undeclared")
    assert [item.subject for item in found] == [ids.build_param(NODE, "max_speed")]
    again = _run(_with_blind(_undeclared_atlas(), '"sensor." + sensor_id'), "param.set_undeclared")
    assert [item.id for item in again] == [item.id for item in found]


def test_a_key_only_a_name_built_at_run_time_could_carry_is_not_effective():
    """A constant with no literal text may name any key, so whether the node reads it is
    unknown, and calling it effective claimed the node reads it."""
    from rosio import join

    atlas = _with_blind(_undeclared_atlas(), "dep::msg::Constants::FLAG")
    assert _run(atlas, "param.set_undeclared") == []
    entity = _joined(atlas).get(ids.build_param(NODE, "max_speed"))
    assert entity.state == model.ParamState.OTHER.value
    assert not entity.effective
    assert [row.note for row in entity.candidates] == [join.BLIND_NOTE]


def test_a_name_built_at_run_time_matches_only_what_its_literal_text_allows():
    from rosio import join

    assert join.could_name('topic_name + "_config"', "odom0_config")
    assert join.could_name("prefix_ + name", "anything")
    assert join.could_name('f"{plugin}.inflation_radius"', "inflation.inflation_radius")
    assert not join.could_name('"sensor." + sensor_id', "max_speed")
    assert not join.could_name('"%s_rate" % name', "rate_limit")


def test_a_name_chosen_by_a_condition_or_a_lookup_could_be_any_name():
    """The literals of `use_fast_ ? "fast_gain" : "slow_gain"` are two whole names and the one
    of `names_.at("gain_key")` is a map key, so neither spells a piece of the name. Only a
    plain concatenation or a format string is read for its literal text."""
    from rosio import join

    for expr, name in [
        ('use_fast_ ? "fast_gain" : "slow_gain"', "fast_gain"),
        ('use_fast_ ? "fast_gain" : "slow_gain"', "slow_gain"),
        ('names_.at("gain_key")', "kp"),
        ('names_["gain_key"]', "kp"),
        ('names_.find("gain_key")->second', "kp"),
        ('self._names.get("gain_key")', "kp"),
        ('"fast_gain" if fast else "slow_gain"', "slow_gain"),
        ('lookup("gain_key") + "_x"', "kp_x"),
    ]:
        assert join.could_name(expr, name), expr
    for expr in [
        'std::string("sensor.") + sensor_id',
        'fmt::format("sensor.{}", sensor_id)',
        '"sensor.{}".format(sensor_id)',
        'f"sensor.{sensor_id}"',
        '"sensor.%s" % (sensor_id)',
    ]:
        assert not join.could_name(expr, "max_speed"), expr


@pytest.mark.parametrize("locator", ["/**.ros__parameters.wheel_base", "SetParameter"])
def test_param_set_undeclared_is_silent_on_a_shared_line_another_node_reads(locator):
    """vehicle_info.launch.py sets wheel_base with SetParameter for every node it starts, and
    a node that ignores it does not make the line deletable while the planner reads it."""
    atlas = _undeclared_atlas("wheel_base", locator=locator)
    assert len(_run(atlas, "param.set_undeclared")) == 1
    atlas = _undeclared_atlas("wheel_base", locator=locator)
    _planner_declares(atlas, "wheel_base", locator)
    assert _run(atlas, "param.set_undeclared") == []


def _planner_declares(atlas: model.Atlas, name: str, locator: str) -> None:
    planner = ids.build_param(ids.build_node("/planner"), name)
    atlas.entities[planner] = model.Node(
        id=planner,
        kind="param",
        label=name,
        facts=[model.LayerFact(layer=CODED, profile=PROFILE, values={"declared_in_code": True})],
        candidates=[
            _candidate(
                1.0, model.Mechanism.YAML_BASE.value, _source(BASE, locator, 12), rank=0, wins=True
            )
        ],
    )


def test_a_shared_key_only_this_node_is_handed_is_still_reported():
    """A `/**` block in a file handed to one node reaches that node alone. Another node
    declaring the name from another line does not make this line needed."""
    atlas = _undeclared_atlas("wheel_base", locator="/**.ros__parameters.wheel_base")
    _planner_declares(atlas, "wheel_base", "planner.ros__parameters.wheel_base")
    assert len(_run(atlas, "param.set_undeclared")) == 1


def test_a_name_another_profile_declares_is_still_undeclared_in_this_one():
    """A sim and a real profile can start one node name from two executables. The code the
    other profile runs declaring and reading max_speed says nothing about this profile."""
    atlas = _undeclared_atlas()
    param_id = ids.build_param(NODE, "max_speed")
    atlas.entities[param_id].facts.append(
        model.LayerFact(layer=CODED, profile=OTHER_PROFILE, values={"declared_in_code": True})
    )
    symbol = ids.build_sym("demo_sim", "src/sim_server.cpp", "SimServer::on_timer")
    atlas.entities[symbol] = model.Node(id=symbol, kind="sym", label=symbol)
    edge = model.Edge(
        kind=model.EdgeKind.READS_PARAM.value,
        src=symbol,
        dst=param_id,
        key=ids.edge_key(profile=OTHER_PROFILE, symbol=symbol, name="max_speed"),
        facts=[model.LayerFact(layer=CODED, profile=OTHER_PROFILE, values={})],
    )
    atlas.edges[edge.id] = edge
    found = _run(atlas, "param.set_undeclared")
    assert [(item.subject, item.profiles) for item in found] == [(param_id, [PROFILE])]


def test_param_set_undeclared_is_silent_for_a_name_a_condition_chooses():
    atlas = _with_blind(_undeclared_atlas(), 'use_fast_ ? "fast_gain" : "slow_gain"')
    assert _run(atlas, "param.set_undeclared") == []


def test_param_set_undeclared_is_silent_when_the_bound_source_only_runs_a_class():
    """A `main` constructing a class defined in another file read none of its declarations."""
    atlas = _undeclared_atlas(states_node=False)
    assert _run(atlas, "param.set_undeclared") == []
    entity = _joined(atlas).get(ids.build_param(NODE, "max_speed"))
    assert entity.state == model.ParamState.EFFECTIVE.value


def test_param_set_undeclared_is_silent_for_a_parameter_the_client_library_declares():
    assert _run(_undeclared_atlas("use_sim_time"), "param.set_undeclared") == []


def test_param_set_undeclared_is_silent_when_the_source_was_never_read():
    builder = _builder()
    _param(
        builder,
        "max_speed",
        [
            _candidate(
                1.0,
                model.Mechanism.YAML_BASE.value,
                _source(BASE, "demo_server.ros__parameters.max_speed", 12),
                rank=0,
                wins=True,
            )
        ],
    )
    assert _run(builder.recompute(), "param.set_undeclared") == []


def test_param_set_undeclared_is_silent_when_the_code_declares_the_key():
    builder = _builder()
    source_file = ids.build_file("demo_pkg", "src/demo_server.cpp")
    builder.node(source_file)
    builder.fact(
        NODE,
        CODED,
        profile=PROFILE,
        values={"coded_sources": [source_file]},
        attrs={"coded_params_known": True},
    )
    param_id = _param(
        builder,
        "max_speed",
        [
            _candidate(
                1.0,
                model.Mechanism.YAML_BASE.value,
                _source(BASE, "demo_server.ros__parameters.max_speed", 12),
                rank=0,
                wins=True,
            )
        ],
    )
    builder.fact(
        param_id,
        CODED,
        profile=PROFILE,
        values={"declared_in_code": True, "type": "double"},
        evidence=[_source(source_file, "DemoServer::DemoServer", 20)],
    )
    assert _run(builder.recompute(), "param.set_undeclared") == []


def _joined(atlas: model.Atlas) -> model.Atlas:
    from rosio import join

    builder = model.AtlasBuilder()
    builder.atlas = atlas
    join._ignore_undeclared(builder)
    return builder.recompute()


def test_a_parameter_the_node_never_declares_wins_nothing_and_is_undeclared():
    """`rosio param` called stop_distnce effective while drift said the node ignores it."""
    before = _run(_undeclared_atlas(), "param.set_undeclared")
    atlas = _joined(_undeclared_atlas())
    entity = atlas.get(ids.build_param(NODE, "max_speed"))
    assert entity.state == model.ParamState.UNDECLARED.value
    assert entity.effective == {}
    (row,) = entity.candidates
    assert not row.wins and "never declares" in row.note
    after = _run(atlas, "param.set_undeclared")
    assert [item.id for item in after] == [item.id for item in before]
    assert _joined(atlas).to_bytes() == atlas.to_bytes()


@pytest.mark.parametrize("auto_declare", [True, "unknown"])
def test_a_node_that_takes_undeclared_parameters_keeps_the_value(auto_declare):
    """`automatically_declare_parameters_from_overrides` declares what a file sets, and an
    option this reader could not read may do the same."""
    atlas = _joined(_undeclared_atlas(auto_declare=auto_declare))
    entity = atlas.get(ids.build_param(NODE, "max_speed"))
    assert entity.state == model.ParamState.EFFECTIVE.value
    assert entity.effective
    assert _run(atlas, "param.set_undeclared") == []


def test_a_parameter_the_code_reads_back_is_not_ignored():
    atlas = _joined(_undeclared_atlas(read=True))
    entity = atlas.get(ids.build_param(NODE, "max_speed"))
    assert entity.state == model.ParamState.EFFECTIVE.value
    assert all(not row.note for row in entity.candidates)
    assert _run(atlas, "param.set_undeclared") == []


def test_a_parameter_the_node_declares_stays_effective():
    atlas = _joined(_undeclared_atlas("max_speed", declared=("max_speed",)))
    entity = atlas.get(ids.build_param(NODE, "max_speed"))
    assert entity.state == model.ParamState.EFFECTIVE.value
    assert entity.effective


def test_param_set_undeclared_finding_id_is_stable_across_two_runs():
    assert _ids(_undeclared_atlas(), "param.set_undeclared") == _ids(
        _undeclared_atlas(), "param.set_undeclared"
    )


# ---------------------------------------------------------------------------
# param.declared_unread


def _unread_atlas(returns_value: bool = False) -> model.Atlas:
    builder = _builder()
    source_file = ids.build_file("demo_pkg", "src/demo_server.cpp")
    builder.node(source_file)
    param_id = ids.build_param(NODE, "unused_gain")
    builder.fact(
        param_id,
        CODED,
        profile=PROFILE,
        values={
            "declared_in_code": True,
            "read": False,
            "declaration_returns_value": returns_value,
        },
        evidence=[_source(source_file, "DemoServer::DemoServer", 31)],
    )
    return builder.recompute()


def test_param_declared_unread_fires_when_the_declaration_returns_nothing():
    found = _run(_unread_atlas(), "param.declared_unread")
    assert len(found) == 1
    assert found[0].severity == model.Severity.INFO.value
    assert "demo_pkg/src/demo_server.cpp:31" in found[0].say


def test_param_declared_unread_is_silent_when_the_declaration_returns_the_value():
    assert _run(_unread_atlas(returns_value=True), "param.declared_unread") == []


def test_param_declared_unread_is_silent_when_the_collector_says_nothing_about_the_return():
    builder = _builder()
    source_file = ids.build_file("demo_pkg", "src/demo_server.cpp")
    builder.node(source_file)
    builder.fact(
        ids.build_param(NODE, "unused_gain"),
        CODED,
        profile=PROFILE,
        values={"declared_in_code": True, "read": False},
        evidence=[_source(source_file, "DemoServer::DemoServer", 31)],
    )
    assert _run(builder.recompute(), "param.declared_unread") == []


def test_param_declared_unread_finding_id_is_stable_across_two_runs():
    assert _ids(_unread_atlas(), "param.declared_unread") == _ids(
        _unread_atlas(), "param.declared_unread"
    )


# ---------------------------------------------------------------------------
# param.type_changed


def _type_atlas(code_type: str = "int") -> model.Atlas:
    builder = _builder()
    source_file = ids.build_file("demo_pkg", "src/demo_server.cpp")
    builder.node(source_file)
    param_id = _param(
        builder,
        "queue_depth",
        [
            _candidate(
                2.5,
                model.Mechanism.YAML_BASE.value,
                _source(BASE, "demo_server.ros__parameters.queue_depth", 14),
                rank=0,
                wins=True,
            )
        ],
    )
    builder.fact(
        param_id,
        CODED,
        profile=PROFILE,
        values={"declared_in_code": True, "type": code_type},
        evidence=[_source(source_file, "DemoServer::DemoServer", 22)],
    )
    return builder.recompute()


def test_param_type_changed_fires_when_the_declaration_and_the_value_disagree():
    found = _run(_type_atlas(), "param.type_changed")
    assert len(found) == 1
    assert found[0].severity == model.Severity.ERROR.value
    assert "int" in found[0].say


def test_param_type_changed_is_silent_when_the_types_agree():
    assert _run(_type_atlas("double"), "param.type_changed") == []


def test_param_type_changed_is_silent_for_a_declaration_spelling_it_cannot_map():
    assert _run(_type_atlas("MyOwnScalar"), "param.type_changed") == []


def test_param_type_changed_ignores_a_losing_candidate_of_another_type():
    """A shadowed candidate never reaches the node. Comparing it named a declaration type no
    source states: a yaml string the inline bool replaced read as a str declaration."""
    builder = _builder()
    source_file = ids.build_file("demo_pkg", "src/demo_server.cpp")
    builder.node(source_file)
    param_id = _param(
        builder,
        "heartbeat",
        [
            _candidate(
                False,
                model.Mechanism.LAUNCH_PARAMS_FILE.value,
                _source(OVERLAY, "demo_server.ros__parameters.heartbeat", 3),
                rank=0,
                wins=True,
            ),
            _candidate(
                "$(var heartbeat)",
                model.Mechanism.YAML_BASE.value,
                _source(BASE, "demo_server.ros__parameters.heartbeat", 14),
                rank=1,
                shadow_kind=model.ShadowKind.TYPE_CHANGED.value,
            ),
        ],
    )
    builder.fact(
        param_id,
        CODED,
        profile=PROFILE,
        values={"declared_in_code": True, "type": "bool"},
        evidence=[_source(source_file, "DemoServer::DemoServer", 22)],
    )
    assert _run(builder.recompute(), "param.type_changed") == []


def test_param_type_changed_reads_a_whole_substitution_as_no_type():
    builder = _builder()
    source_file = ids.build_file("demo_pkg", "src/demo_server.cpp")
    builder.node(source_file)
    param_id = _param(
        builder,
        "heartbeat",
        [
            _candidate(
                "$(var heartbeat)",
                model.Mechanism.YAML_BASE.value,
                _source(BASE, "demo_server.ros__parameters.heartbeat", 14),
                rank=0,
                wins=True,
            )
        ],
    )
    builder.fact(
        param_id,
        CODED,
        profile=PROFILE,
        values={"declared_in_code": True, "type": "bool"},
        evidence=[_source(source_file, "DemoServer::DemoServer", 22)],
    )
    assert _run(builder.recompute(), "param.type_changed") == []


def test_param_type_changed_ignores_a_file_the_profile_displaced():
    """A displaced default file is provenance; the running node never receives its value."""
    builder = _builder()
    _param(
        builder,
        "queue_depth",
        [
            _candidate(
                2,
                model.Mechanism.LAUNCH_PARAMS_FILE.value,
                _source(OVERLAY, "demo_server.ros__parameters.queue_depth", 3),
                rank=0,
                wins=True,
            ),
            _candidate(
                "unused",
                model.Mechanism.YAML_BASE.value,
                _source(BASE, "demo_server.ros__parameters.queue_depth", 14),
                rank=1,
                shadow_kind=model.ShadowKind.NOT_READ_IN_PROFILE.value,
            ),
        ],
    )
    assert _run(builder.recompute(), "param.type_changed") == []


def test_param_type_changed_finding_id_is_stable_across_two_runs():
    assert _ids(_type_atlas(), "param.type_changed") == _ids(_type_atlas(), "param.type_changed")


# ---------------------------------------------------------------------------
# param.orphan_block and param.owner_name_mismatch


def _block_atlas(
    selector: str,
    *,
    state: str,
    keys: int = 2,
    unnamed: bool = False,
    block_line: int | None = 38,
):
    """One block of `keys` keys that no node owns.

    `block_line` is where the block key itself is written, which the parameter collector
    cites on the fact. `None` is an atlas built before it did, whose rows cite only the keys
    inside the block.
    """
    builder = _builder()
    if unnamed:
        builder.node(
            ids.build_node_synthetic("demo_pkg", "helper.py", "/", "a1b2c3d4"),
            name_source=model.NameSource.UNKNOWN.value,
        )
        builder.fact(
            ids.build_node_synthetic("demo_pkg", "helper.py", "/", "a1b2c3d4"),
            DECLARED,
            profile=PROFILE,
            values={"form": "process"},
        )
    builder.fact(NODE, DECLARED, profile=PROFILE, values={"form": "node"})
    for index in range(keys):
        name = f"key_{index}"
        param_id = ids.build_param(ids.build_node(f"/{selector}"), name)
        builder.fact(
            param_id,
            DECLARED,
            profile=PROFILE,
            values={"state": state, "selector": selector},
            evidence=([_source(BASE, selector, block_line)] if block_line is not None else ()),
        )
        builder.set_param(
            param_id,
            state=state,
            candidates=[
                _candidate(
                    index,
                    model.Mechanism.YAML_BASE.value,
                    _source(BASE, f"{selector}.ros__parameters.{name}", 40 + index),
                    rank=0,
                )
            ],
        )
    return builder.recompute()


def _mismatch_atlas() -> model.Atlas:
    return _block_atlas("demo", state=model.ParamState.OWNER_NAME_MISMATCH.value)


def test_param_owner_name_mismatch_fires_when_a_launched_node_is_one_suffix_away():
    found = _run(_mismatch_atlas(), "param.owner_name_mismatch")
    assert len(found) == 1
    assert found[0].subject == BASE
    assert "/demo_server" in found[0].say
    assert "config/params.yaml:38" in found[0].say


def test_param_owner_name_mismatch_is_silent_when_the_block_matches_no_node_at_all():
    atlas = _block_atlas("velocity_smoother", state=model.ParamState.OWNER_NAME_MISMATCH.value)
    assert _run(atlas, "param.owner_name_mismatch") == []


def test_param_owner_name_mismatch_holds_when_another_profile_reads_the_block():
    """The entity keeps the strongest state over all profiles, so a profile where the block
    is read must not hide the profile where it is ignored."""
    atlas = _mismatch_atlas()
    for node in atlas.entities.values():
        if node.kind == model.NodeKind.PARAM.value:
            node.state = model.ParamState.EFFECTIVE.value
    assert len(_run(atlas, "param.owner_name_mismatch")) == 1


def test_param_owner_name_mismatch_finding_id_is_stable_across_two_runs():
    assert _ids(_mismatch_atlas(), "param.owner_name_mismatch") == _ids(
        _mismatch_atlas(), "param.owner_name_mismatch"
    )


def _orphan_atlas() -> model.Atlas:
    return _block_atlas(
        "velocity_smoother", state=model.ParamState.OWNER_NAME_MISMATCH.value, keys=3
    )


def test_param_orphan_block_reports_the_block_once_and_not_once_per_key():
    found = _run(_orphan_atlas(), "param.orphan_block")
    assert len(found) == 1
    assert found[0].severity == model.Severity.WARN.value
    assert "3 keys" in found[0].say
    assert "config/params.yaml:38" in found[0].say


def test_param_orphan_block_is_silent_when_the_block_key_names_a_launched_node():
    builder = _builder()
    builder.fact(NODE, DECLARED, profile=PROFILE, values={"form": "node"})
    _param(
        builder,
        "max_speed",
        [
            _candidate(
                1.0,
                model.Mechanism.YAML_BASE.value,
                _source(BASE, "demo_server.ros__parameters.max_speed", 12),
                rank=0,
                wins=True,
            )
        ],
        values={"selector": "demo_server"},
    )
    assert _run(builder.recompute(), "param.orphan_block") == []


def test_param_orphan_block_drops_to_info_when_a_process_could_not_be_named():
    atlas = _block_atlas(
        "velocity_smoother",
        state=model.ParamState.UNRESOLVED_OWNER.value,
        unnamed=True,
    )
    found = _run(atlas, "param.orphan_block")
    assert len(found) == 1
    assert found[0].severity == model.Severity.INFO.value
    assert "carry no name" in found[0].say
    assert ids.SYNTHETIC not in found[0].say


def test_param_orphan_block_cites_the_block_key_and_not_the_first_key_in_it():
    """The finding tells a reader to delete the block, so it cites the line the block opens
    at. The keys inside it start two lines further down, one for the block key and one for
    `ros__parameters:`, and citing the first of them sent the reader to the wrong line. An
    atlas whose rows carry no block key falls back to the lowest line of the block."""
    found = _run(_orphan_atlas(), "param.orphan_block")
    assert len(found) == 1
    assert found[0].where[0].line_hint == 38
    assert "config/params.yaml:38" in found[0].say

    older = _block_atlas(
        "velocity_smoother",
        state=model.ParamState.OWNER_NAME_MISMATCH.value,
        keys=3,
        block_line=None,
    )
    fallback = _run(older, "param.orphan_block")
    assert len(fallback) == 1
    assert fallback[0].where[0].line_hint == 40


def test_one_block_name_in_two_files_is_two_blocks_and_neither_borrows_the_other():
    """A key written under the same block name in two files belongs to two blocks.

    A base parameter file and the overlay beside it both carrying a `velocity_smoother:`
    block is an ordinary shape, and a reader who deletes one still has the other. Grouping on
    the block key the fact cites folds the two into whichever file the collector happened to
    cite: the second finding disappears and its keys are added to the first, so the one that
    is left names a key count no single file carries.
    """
    builder = _builder()
    builder.fact(NODE, DECLARED, profile=PROFILE, values={"form": "node"})
    state = model.ParamState.OWNER_NAME_MISMATCH.value
    places = ((BASE, "base_key", 40), (OVERLAY, "overlay_key", 10))
    for file_id, name, key_line in places:
        param_id = ids.build_param(ids.build_node("/velocity_smoother"), name)
        builder.fact(
            param_id,
            DECLARED,
            profile=PROFILE,
            values={"state": state, "selector": "velocity_smoother"},
            evidence=[_source(file_id, "velocity_smoother", key_line - 2)],
        )
        builder.set_param(
            param_id,
            state=state,
            candidates=[
                _candidate(
                    1,
                    model.Mechanism.YAML_BASE.value,
                    _source(file_id, f"velocity_smoother.ros__parameters.{name}", key_line),
                    rank=0,
                )
            ],
        )
    found = _run(builder.recompute(), "param.orphan_block")
    assert len(found) == 2
    assert [item.subject for item in found] == sorted([BASE, OVERLAY])
    for item in found:
        assert "1 key" in item.say
    lines = {item.subject: item.where[0].line_hint for item in found}
    assert lines[BASE] == 38
    assert lines[OVERLAY] == 8


def _two_file_atlas(*, both_keys: bool = True) -> model.Atlas:
    """One key spelled under one orphan block name in a base file and in an overlay.

    `both_keys` false is the atlas the collector wrote before it cited a block key per file:
    one evidence row for a parameter that two blocks hold.
    """
    builder = _builder()
    builder.fact(NODE, DECLARED, profile=PROFILE, values={"form": "node"})
    state = model.ParamState.OWNER_NAME_MISMATCH.value
    param_id = ids.build_param(ids.build_node("/velocity_smoother"), "max_speed")
    blocks = [_source(BASE, "velocity_smoother", 38)]
    if both_keys:
        blocks.append(_source(OVERLAY, "velocity_smoother", 8))
    builder.fact(
        param_id,
        DECLARED,
        profile=PROFILE,
        values={"state": state, "selector": "velocity_smoother"},
        evidence=blocks,
    )
    builder.set_param(
        param_id,
        state=state,
        candidates=[
            _candidate(
                1.0,
                model.Mechanism.YAML_OVERLAY_MERGE.value,
                _source(OVERLAY, "velocity_smoother.ros__parameters.max_speed", 10),
                rank=0,
            ),
            _candidate(
                2.0,
                model.Mechanism.YAML_BASE.value,
                _source(BASE, "velocity_smoother.ros__parameters.max_speed", 40),
                rank=1,
            ),
        ],
    )
    return builder.recompute()


def test_one_key_in_two_orphan_blocks_cites_the_block_key_of_each_file():
    """The key an overlay restates is a key of both blocks, and each finding cites its own.

    Reading one write site per parameter answered the grouping question with the file that
    holds the winning candidate, so the losing file's block lost its only member and the
    finding over it fell back to the first key inside it, two lines past the block key it
    tells the reader to delete.
    """
    found = _run(_two_file_atlas(), "param.orphan_block")
    assert len(found) == 2
    assert [item.subject for item in found] == sorted([BASE, OVERLAY])
    lines = {item.subject: item.where[0].line_hint for item in found}
    assert lines[BASE] == 38
    assert lines[OVERLAY] == 8
    for item in found:
        assert "1 key" in item.say
    assert "config/params.yaml:38" in {item.subject: item.say for item in found}[BASE]


def test_a_block_with_no_key_of_its_own_file_still_cites_its_lowest_key():
    """An atlas written before the collector cited one block key per file still answers.

    The block whose key is missing has no line above its first key to offer, so the rule
    states the closest row it holds rather than a line out of another file.
    """
    found = _run(_two_file_atlas(both_keys=False), "param.orphan_block")
    assert len(found) == 2
    lines = {item.subject: item.where[0].line_hint for item in found}
    assert lines[BASE] == 38
    assert lines[OVERLAY] == 10
    assert {item.where[0].file for item in found} == {BASE, OVERLAY}


def _shared_block_atlas(*, read_elsewhere: bool) -> model.Atlas:
    """nav2 hands one parameters file to slam, localization and navigation launch files, so
    the slam profile carries a controller_server block another profile reads."""
    atlas = _block_atlas("velocity_smoother", state=model.ParamState.OWNER_NAME_MISMATCH.value)
    builder = model.AtlasBuilder()
    builder.atlas = atlas
    param_id = ids.build_param(ids.build_node("/velocity_smoother"), "key_0")
    source = _source(BASE, "velocity_smoother.ros__parameters.key_0", 40)
    state = model.ParamState.EFFECTIVE.value if read_elsewhere else "other"
    builder.fact(param_id, DECLARED, profile=OTHER_PROFILE, values={"state": state})
    winner = _candidate(
        0, model.Mechanism.YAML_BASE.value, source, wins=read_elsewhere, profile=OTHER_PROFILE
    )
    builder.set_param(param_id, state=state, candidates=[winner])
    return builder.build()


def test_param_orphan_block_is_a_note_when_another_profile_reads_the_block():
    (found,) = _run(_shared_block_atlas(read_elsewhere=True), "param.orphan_block")
    assert found.severity == model.Severity.INFO.value
    assert "The file is shared: profile other launches a node that reads this block" in found.say
    assert found.id == _run(_orphan_atlas(), "param.orphan_block")[0].id


def test_param_orphan_block_stays_a_warning_when_no_other_profile_reads_the_block():
    (found,) = _run(_shared_block_atlas(read_elsewhere=False), "param.orphan_block")
    assert found.severity == model.Severity.WARN.value
    assert "shared" not in found.say


def test_param_sim_time_mixed_skips_a_node_the_profile_never_starts():
    """A node behind a false launch condition kept a use_sim_time row with no node fact,
    and counted as a third rviz on wall time in nav2's multi robot simulation."""
    atlas = _sim_time_atlas({"/amcl": True, "/rviz": False})
    atlas.entities[ids.build_node("/rviz")].facts.clear()
    builder = model.AtlasBuilder()
    builder.atlas = atlas
    assert _run(builder.build(), "param.sim_time_mixed") == []


def test_node_text_tells_two_unnamed_nodes_apart_by_namespace():
    """Three rviz2 instances read as "the rviz2 rviz2 node" three times over, and a package
    named after its one executable is named once."""
    one = ids.build_node_synthetic("rviz2", "rviz2", "/robot1", "13cae34e")
    root = ids.build_node_synthetic("rviz2", "rviz2", "/", "b8cfd988")
    other = ids.build_node_synthetic("nav2_map_server", "map_server", "/", "b8cfd988")
    assert rules.node_text(one) == "the rviz2 node in /robot1"
    assert rules.node_text(root) == "the rviz2 node"
    assert rules.node_text(other) == "the nav2_map_server map_server node"


def _orphan_in_profiles(files: dict[str, str]) -> model.Atlas:
    """One dead block per profile, each read from the file the profile names."""
    builder = _builder()
    selector = "velocity_smoother"
    state = model.ParamState.OWNER_NAME_MISMATCH.value
    for profile, file_id in sorted(files.items()):
        builder.fact(NODE, DECLARED, profile=profile, values={"form": "node"})
        for index in range(2):
            param_id = ids.build_param(ids.build_node(f"/{selector}"), f"key_{index}")
            builder.fact(
                param_id,
                DECLARED,
                profile=profile,
                values={"state": state, "selector": selector},
                evidence=[_source(file_id, selector, 38)],
            )
            builder.set_param(
                param_id,
                state=state,
                candidates=[
                    _candidate(
                        index,
                        model.Mechanism.YAML_BASE.value,
                        _source(file_id, f"{selector}.ros__parameters.key_{index}", 40 + index),
                        rank=0,
                        profile=profile,
                    )
                ],
            )
    return builder.recompute()


def test_param_orphan_block_read_by_two_profiles_is_one_finding_naming_both():
    """nav2's shared parameters file made one deletable line one finding per profile."""
    atlas = _orphan_in_profiles({PROFILE: BASE, OTHER_PROFILE: BASE})
    (found,) = _run(atlas, "param.orphan_block")
    assert found.profiles == [PROFILE, OTHER_PROFILE]
    assert found.discriminator == "velocity_smoother"
    assert "profiles demo and other launch no node" in found.say
    again = _run(_orphan_in_profiles({PROFILE: BASE, OTHER_PROFILE: BASE}), "param.orphan_block")
    assert [item.id for item in again] == [found.id]


def test_param_orphan_block_in_two_files_stays_two_findings():
    atlas = _orphan_in_profiles({PROFILE: BASE, OTHER_PROFILE: OVERLAY})
    found = _run(atlas, "param.orphan_block")
    assert sorted(item.subject for item in found) == sorted([BASE, OVERLAY])


def test_param_orphan_block_finding_id_is_stable_across_two_runs():
    assert _ids(_orphan_atlas(), "param.orphan_block") == _ids(
        _orphan_atlas(), "param.orphan_block"
    )


# ---------------------------------------------------------------------------
# plugin.inert_by_design


def _inert_atlas(selector: str = "controller_plugins") -> model.Atlas:
    builder = _builder()
    for index, key in enumerate(("critic.weight", "critic.power")):
        param_id = ids.build_param(NODE, f"FollowPath_OFF.{key}")
        builder.fact(
            param_id,
            DECLARED,
            profile=PROFILE,
            values={
                "state": model.ParamState.INERT_BY_DESIGN.value,
                "block": "FollowPath_OFF",
                "owner": "/demo_server",
                "selector": selector,
            },
        )
        builder.set_param(
            param_id,
            state=model.ParamState.INERT_BY_DESIGN.value,
            candidates=[
                _candidate(
                    index,
                    model.Mechanism.YAML_BASE.value,
                    _source(BASE, f"demo_server.ros__parameters.FollowPath_OFF.{key}", 60 + index),
                    rank=0,
                )
            ],
        )
    return builder.recompute()


def test_plugin_inert_by_design_reports_one_line_for_the_whole_block():
    found = _run(_inert_atlas(), "plugin.inert_by_design")
    assert len(found) == 1
    assert found[0].severity == model.Severity.INFO.value
    assert "2 keys" in found[0].say
    assert "controller_plugins" in found[0].say
    assert "config/params.yaml:60" in found[0].say


def test_plugin_inert_by_design_is_silent_when_the_block_is_selected():
    builder = _builder()
    _param(
        builder,
        "FollowPath.critic.weight",
        [
            _candidate(
                1.0,
                model.Mechanism.YAML_BASE.value,
                _source(BASE, "demo_server.ros__parameters.FollowPath.critic.weight", 60),
                rank=0,
                wins=True,
            )
        ],
        values={"block": "FollowPath", "owner": "/demo_server", "selector": "controller_plugins"},
    )
    assert _run(builder.recompute(), "plugin.inert_by_design") == []


def test_plugin_inert_by_design_is_one_finding_for_a_block_inert_in_every_profile():
    """DRIFT.md printed six identical headings for one block, one per profile."""
    builder = _builder()
    for profile in (PROFILE, OTHER_PROFILE):
        param_id = ids.build_param(NODE, "FollowPath_OFF.critic.weight")
        builder.fact(
            param_id,
            DECLARED,
            profile=profile,
            values={
                "state": model.ParamState.INERT_BY_DESIGN.value,
                "block": "FollowPath_OFF",
                "owner": "/demo_server",
                "selector": "controller_plugins",
            },
        )
        builder.set_param(
            param_id,
            state=model.ParamState.INERT_BY_DESIGN.value,
            candidates=[
                _candidate(
                    1,
                    model.Mechanism.YAML_BASE.value,
                    _source(BASE, "demo_server.ros__parameters.FollowPath_OFF.critic.weight", 60),
                    rank=0,
                    profile=profile,
                )
            ],
        )
    (found,) = _run(builder.recompute(), "plugin.inert_by_design")
    assert found.profiles == sorted([PROFILE, OTHER_PROFILE])
    assert "profiles demo, other" in found.say


def test_plugin_inert_by_design_finding_id_is_stable_across_two_runs():
    assert _ids(_inert_atlas(), "plugin.inert_by_design") == _ids(
        _inert_atlas(), "plugin.inert_by_design"
    )


# ---------------------------------------------------------------------------
# plugin.unused and plugin.unresolved


def _plugin_atlas(
    *,
    selected: bool,
    exported: bool = True,
    loaded: bool = True,
    installed: bool = False,
    shared: bool = False,
) -> model.Atlas:
    builder = _builder()
    if shared:
        # The file that names the list is handed to a profile that never starts the node.
        builder.fact(NODE, DECLARED, profile=PROFILE, values={"name": "demo_server"})
        builder.fact(
            ids.build_param(NODE, "controller_plugins"),
            DECLARED,
            profile=OTHER_PROFILE,
            values={"value": ["FollowPath"]},
            evidence=[_source(BASE, "demo_server.ros__parameters.controller_plugins", 29)],
        )
    if loaded:
        # A controller server runs in the profile and selects its controllers from a list.
        builder.fact(
            ids.build_param(NODE, "controller_plugins"),
            DECLARED,
            profile=PROFILE,
            values={"value": ["FollowPath"]},
            evidence=[_source(BASE, "demo_server.ros__parameters.controller_plugins", 29)],
        )
    manifest = ids.build_file("demo_controller", "plugins.xml")
    builder.node(manifest)
    package = ids.build_pkg("demo_controller")
    builder.node(package)
    plugin_id = ids.build_plugin("nav2_core::Controller", "demo_controller/Spare")
    builder.fact(
        plugin_id,
        DECLARED,
        values={"base_class": "nav2_core::Controller", "lookup_name": "demo_controller/Spare"},
        confidence=model.Confidence.CERTAIN.value,
        evidence=[
            model.Source(
                file=manifest,
                locator="class[demo_controller/Spare]",
                root=model.Root.INSTALL.value if installed else model.Root.SRC.value,
            )
        ],
    )
    if exported:
        builder.edge(model.EdgeKind.PROVIDES_PLUGIN.value, package, plugin_id)
    if selected:
        instance = ids.build_pinst(NODE, "FollowPath")
        builder.fact(
            instance,
            DECLARED,
            profile=PROFILE,
            values={"plugin": "demo_controller/Spare", "selector": "controller_plugins"},
            evidence=[_source(BASE, "demo_server.ros__parameters.FollowPath.plugin", 30)],
        )
        builder.edge(model.EdgeKind.INSTANTIATES.value, instance, plugin_id, profile=PROFILE)
    return builder.recompute()


def test_plugin_unused_fires_for_a_class_this_workspace_builds_and_never_selects():
    found = _run(_plugin_atlas(selected=False), "plugin.unused")
    assert len(found) == 1
    assert found[0].severity == model.Severity.INFO.value
    assert "demo_controller/plugins.xml" in found[0].say
    assert "controller_plugins" in found[0].say


def test_plugin_unused_speaks_for_the_profiles_that_load_the_kind_only():
    """nav2: 61 infos covered profiles that never start a controller server, and told a
    reader to stop exporting controllers nav2 ships as alternatives."""
    (found,) = _run(_plugin_atlas(selected=False, shared=True), "plugin.unused")
    assert found.profiles == [PROFILE]
    assert "other" not in found.say
    assert "stop" not in found.fix + found.say


def test_plugin_unused_is_silent_when_one_profile_selects_it():
    assert _run(_plugin_atlas(selected=True), "plugin.unused") == []


def test_plugin_unused_is_silent_on_a_package_only_an_install_tree_holds():
    """23 infos on /opt/ros/jazzy told a reader to stop exporting an apt package's plugin."""
    assert _run(_plugin_atlas(selected=False, installed=True), "plugin.unused") == []


def test_plugin_unused_is_silent_when_no_profile_loads_that_kind_of_plugin():
    """A controller nothing could load is out of scope, not unused: a profile that runs no
    controller server says nothing about controllers."""
    assert _run(_plugin_atlas(selected=False, loaded=False), "plugin.unused") == []


def test_plugin_unused_is_silent_for_a_base_class_no_yaml_list_selects():
    builder = _builder()
    manifest = ids.build_file("demo_rviz", "plugins.xml")
    builder.node(manifest)
    package = ids.build_pkg("demo_rviz")
    builder.node(package)
    plugin_id = ids.build_plugin("rviz_common::Tool", "demo_rviz/Goal")
    builder.fact(
        plugin_id,
        DECLARED,
        values={"base_class": "rviz_common::Tool"},
        confidence=model.Confidence.CERTAIN.value,
        evidence=[_source(manifest, "class[demo_rviz/Goal]")],
    )
    builder.edge(model.EdgeKind.PROVIDES_PLUGIN.value, package, plugin_id)
    assert _run(builder.recompute(), "plugin.unused") == []


def test_plugin_unused_finding_id_is_stable_across_two_runs():
    atlas = _plugin_atlas(selected=False)
    assert _ids(atlas, "plugin.unused") == _ids(_plugin_atlas(selected=False), "plugin.unused")


def test_plugin_unresolved_fires_when_a_workspace_package_exports_no_such_class():
    found = _run(_plugin_atlas(selected=True, exported=False), "plugin.unresolved")
    assert len(found) == 1
    assert found[0].severity == model.Severity.ERROR.value
    assert "demo_controller/Spare" in found[0].say
    assert "config/params.yaml:30" in found[0].say


def test_plugin_unresolved_is_silent_for_a_package_this_atlas_never_read():
    builder = _builder()
    plugin_id = ids.build_plugin("nav2_core::Controller", "nav2_mppi_controller/MPPIController")
    instance = ids.build_pinst(NODE, "FollowPath")
    builder.fact(
        instance,
        DECLARED,
        profile=PROFILE,
        values={"plugin": "nav2_mppi_controller/MPPIController"},
        evidence=[_source(BASE, "demo_server.ros__parameters.FollowPath.plugin", 30)],
    )
    builder.fact(
        plugin_id, DECLARED, profile=PROFILE, values={"base_class": "nav2_core::Controller"}
    )
    builder.edge(model.EdgeKind.INSTANTIATES.value, instance, plugin_id, profile=PROFILE)
    assert _run(builder.recompute(), "plugin.unresolved") == []


def test_plugin_unresolved_tells_two_owners_of_one_instance_name_apart():
    """Two servers selecting one missing class as FollowPath are two lines to edit."""
    builder = _builder()
    package = ids.build_pkg("demo_controller")
    builder.node(package)
    builder.node(ids.build_file("demo_controller", "plugins.xml"))
    plugin_id = ids.build_plugin("nav2_core::Controller", "demo_controller/Missing")
    builder.fact(plugin_id, DECLARED, values={"base_class": "nav2_core::Controller"})
    for owner, line in (("/left/server", 30), ("/right/server", 60)):
        node = ids.build_node(owner)
        builder.node(node, name_source=model.NameSource.LITERAL.value)
        instance = ids.build_pinst(node, "FollowPath")
        builder.fact(
            instance,
            DECLARED,
            profile=PROFILE,
            values={"plugin": "demo_controller/Missing"},
            evidence=[_source(BASE, f"{owner[1:]}.ros__parameters.FollowPath.plugin", line)],
        )
        builder.edge(model.EdgeKind.INSTANTIATES.value, instance, plugin_id, profile=PROFILE)
    found = _run(builder.recompute(), "plugin.unresolved")
    assert len({item.id for item in found}) == 2


def test_plugin_unresolved_finding_id_is_stable_across_two_runs():
    atlas = _plugin_atlas(selected=True, exported=False)
    assert _ids(atlas, "plugin.unresolved") == _ids(
        _plugin_atlas(selected=True, exported=False), "plugin.unresolved"
    )


# ---------------------------------------------------------------------------
# what every finding of this family owes the reader


def test_every_finding_names_a_file_and_carries_no_id_in_its_prose():
    atlas = _shadowed_atlas()
    for finding in _run(atlas, "param.shadowed"):
        assert finding.say.endswith(".")
        assert "param:" not in finding.say
        assert "find:" not in finding.say
        assert finding.where


def test_findings_validate_against_the_schema_and_sort_by_severity():
    builder = model.AtlasBuilder()
    builder.atlas = _block_atlas(
        "velocity_smoother", state=model.ParamState.OWNER_NAME_MISMATCH.value
    )
    found = rules.findings_for(builder.atlas, OWNED)
    assert found
    for finding in found:
        builder.finding(finding)
    builder.recompute()
    assert schema.validate(json.loads(builder.atlas.to_bytes().decode("utf-8"))) == []


def test_the_families_expose_one_function_per_rule():
    assert callable(params_rules.shadowed)
    assert callable(plugin_rules.inert_by_design)


# ---------------------------------------------------------------------------
# coverage: what a rule may say when a launch tree or a source tree was not read


def _launchfile(
    builder: model.AtlasBuilder,
    stem: str,
    *,
    profile: str | None = PROFILE,
    loadable: bool = True,
    partial: bool = False,
) -> str:
    file_id = ids.build_launchfile("demo_bringup", f"launch/{stem}.launch.py")
    builder.node(file_id, attrs={"loadable": loadable})
    values: dict[str, Any] = {"loadable": loadable, "strategy": "executed"}
    if partial:
        values["partial"] = True
    builder.fact(file_id, DECLARED, profile=profile or model.ANY_PROFILE, values=values)
    return file_id


def _unused_atlas(build: Any) -> model.Atlas:
    builder = model.AtlasBuilder()
    builder.node(PROFILE)
    builder.node(OTHER_PROFILE)
    manifest = ids.build_file("demo_controller", "plugins.xml")
    builder.node(manifest)
    package = ids.build_pkg("demo_controller")
    builder.node(package)
    plugin_id = ids.build_plugin("nav2_core::Controller", "demo_controller/Spare")
    builder.fact(
        plugin_id,
        DECLARED,
        values={"base_class": "nav2_core::Controller"},
        confidence=model.Confidence.CERTAIN.value,
        evidence=[_source(manifest, "class[demo_controller/Spare]")],
    )
    builder.edge(model.EdgeKind.PROVIDES_PLUGIN.value, package, plugin_id)
    for profile in (PROFILE, OTHER_PROFILE):
        builder.fact(
            ids.build_param(NODE, "controller_plugins"),
            DECLARED,
            profile=profile,
            values={"value": ["FollowPath"]},
            evidence=[_source(BASE, "demo_server.ros__parameters.controller_plugins", 29)],
        )
    build(builder)
    return builder.recompute()


def test_plugin_unused_speaks_for_the_profile_whose_tree_was_read():
    """A refused launch file of one profile is no reason to stay quiet about another."""

    def build(builder: model.AtlasBuilder) -> None:
        _launchfile(builder, "demo", profile=PROFILE)
        _launchfile(builder, "other", profile=OTHER_PROFILE, loadable=False)

    found = _run(_unused_atlas(build), "plugin.unused")
    assert len(found) == 1
    assert found[0].profiles == [PROFILE]
    assert "profile demo" in found[0].say
    assert "other" in found[0].detail


def test_plugin_unused_is_silent_when_no_profile_tree_was_read():
    def build(builder: model.AtlasBuilder) -> None:
        _launchfile(builder, "demo", profile=PROFILE, loadable=False)
        _launchfile(builder, "other", profile=OTHER_PROFILE, loadable=False)

    assert _run(_unused_atlas(build), "plugin.unused") == []


def test_a_walk_that_stopped_part_way_counts_as_a_tree_that_was_not_read():
    def build(builder: model.AtlasBuilder) -> None:
        _launchfile(builder, "demo", profile=PROFILE, partial=True)
        _launchfile(builder, "other", profile=OTHER_PROFILE, loadable=False)

    assert _run(_unused_atlas(build), "plugin.unused") == []


def test_a_refusal_no_profile_claims_gates_every_profile():
    """A refusal nobody can attribute cannot be excluded from any profile."""

    def build(builder: model.AtlasBuilder) -> None:
        builder.node(
            ids.build_launchfile("demo_bringup", "launch/stray.launch.py"),
            attrs={"loadable": False},
        )

    assert _run(_unused_atlas(build), "plugin.unused") == []


def test_an_unresolved_include_gates_only_the_profile_that_owns_the_file():
    def build(builder: model.AtlasBuilder) -> None:
        _launchfile(builder, "demo", profile=PROFILE)
        other = _launchfile(builder, "other", profile=OTHER_PROFILE)
        builder.unresolved(
            model.Unresolved(
                want=model.Want.OTHER.value,
                owner=other,
                expr="source(launch_dir, 'child.launch.py')",
                reason="the included launch file path did not resolve",
                role="include",
            )
        )

    found = _run(_unused_atlas(build), "plugin.unused")
    assert len(found) == 1
    assert found[0].profiles == [PROFILE]


def test_the_plugin_family_reports_which_rule_it_kept_quiet_and_why():
    def build(builder: model.AtlasBuilder) -> None:
        _launchfile(builder, "demo", profile=PROFILE)
        _launchfile(builder, "other", profile=OTHER_PROFILE, loadable=False)

    rows = plugin_rules.silences(_unused_atlas(build))
    assert [row["rule"] for row in rows] == ["plugin.unused"]
    assert rows[0]["profile"] == OTHER_PROFILE
    assert "not read to the end" in rows[0]["reason"]
    assert "launch/other.launch.py" in rows[0]["reason"]


def _owner_atlas(
    selector: str,
    *,
    node_fqn: str = "/bt_demo",
    package: str = "demo_bt",
    sources_read: bool = False,
) -> model.Atlas:
    builder = model.AtlasBuilder()
    builder.node(PROFILE)
    builder.node(BASE)
    owner = ids.build_node(node_fqn)
    builder.node(owner, name_source=model.NameSource.LITERAL.value)
    builder.fact(owner, DECLARED, profile=PROFILE, values={"form": "node", "pkg": package})
    if sources_read:
        source_file = ids.build_file(package, "src/bt_demo.cpp")
        builder.node(source_file)
        builder.fact(source_file, CODED, profile=PROFILE, values={"symbols": 1})
    param_id = ids.build_param(ids.build_node("/" + selector.strip("/")), "timeout")
    builder.fact(
        param_id,
        DECLARED,
        profile=PROFILE,
        values={"state": model.ParamState.OWNER_NAME_MISMATCH.value, "selector": selector},
    )
    builder.set_param(
        param_id,
        state=model.ParamState.OWNER_NAME_MISMATCH.value,
        candidates=[
            _candidate(
                1.0,
                model.Mechanism.YAML_BASE.value,
                _source(BASE, f"{selector}.ros__parameters.timeout", 40),
                rank=0,
            )
        ],
    )
    return builder.recompute()


def test_a_block_key_that_extends_a_launched_name_is_not_called_dead_without_sources():
    """A node that appends a suffix to its own name and starts a second node is in its
    source and nowhere else, so an installed tree cannot tell that block from a dead one."""
    atlas = _owner_atlas("bt_demo_navigate_to_pose_rclcpp_node")
    assert _run(atlas, "param.owner_name_mismatch") == []
    assert _run(atlas, "param.orphan_block") == []
    assert "demo_bt" in params_rules.owner_gate(
        atlas, PROFILE, "bt_demo_navigate_to_pose_rclcpp_node"
    )


def test_the_same_block_key_is_reported_once_the_sources_were_read():
    atlas = _owner_atlas("bt_demo_navigate_to_pose_rclcpp_node", sources_read=True)
    assert params_rules.owner_gate(atlas, PROFILE, "bt_demo_navigate_to_pose_rclcpp_node") == ""
    found = _run(atlas, "param.owner_name_mismatch")
    assert len(found) == 1
    assert "/bt_demo" in found[0].say


def test_a_block_key_in_a_namespace_no_launch_file_creates_is_not_called_dead():
    atlas = _owner_atlas("local_costmap/local_costmap", node_fqn="/controller_server")
    assert _run(atlas, "param.orphan_block") == []
    assert "namespace" in params_rules.owner_gate(atlas, PROFILE, "local_costmap/local_costmap")


def test_a_block_key_that_truncates_a_launched_name_stays_reported():
    """A child extends its parent's name and never shortens it, so this key is a
    misspelling and not a node rosio could not see."""
    atlas = _owner_atlas("lifecycle_manager", node_fqn="/lifecycle_manager_navigation")
    assert params_rules.owner_gate(atlas, PROFILE, "lifecycle_manager") == ""
    found = _run(atlas, "param.owner_name_mismatch")
    assert len(found) == 1
    assert "/lifecycle_manager_navigation" in found[0].say


def test_a_plain_block_key_no_launched_name_relates_to_stays_reported():
    atlas = _owner_atlas("map_saver", node_fqn="/controller_server")
    assert params_rules.owner_gate(atlas, PROFILE, "map_saver") == ""
    assert len(_run(atlas, "param.orphan_block")) == 1


def test_the_block_rules_report_which_rule_they_kept_quiet_and_why():
    atlas = _owner_atlas("bt_demo_navigate_to_pose_rclcpp_node")
    rows = params_rules.silences(atlas)
    assert [row["rule"] for row in rows] == [
        "param.orphan_block",
        "param.owner_name_mismatch",
    ]
    for row in rows:
        assert row["profile"] == PROFILE
        assert "bt_demo_navigate_to_pose_rclcpp_node" in row["reason"]
        assert "no source of demo_bt was read" in row["reason"]


def test_a_block_of_a_profile_whose_launch_tree_collapsed_is_not_called_dead():
    builder = model.AtlasBuilder()
    builder.atlas = _owner_atlas("map_saver", node_fqn="/controller_server")
    _launchfile(builder, "demo", profile=PROFILE, loadable=False)
    atlas = builder.recompute()
    assert _run(atlas, "param.orphan_block") == []
    assert "lower bound" in params_rules.owner_gate(atlas, PROFILE, "map_saver")


def test_plugin_inert_by_design_says_when_no_selector_list_claims_the_block():
    found = _run(_inert_atlas(selector=""), "plugin.inert_by_design")
    assert len(found) == 1
    assert "no selector list of /demo_server names FollowPath_OFF" in found[0].say
    assert "controller_plugins" not in found[0].say


# ---------------------------------------------------------------------------
# what a finding tells a reader to do, and to which profile it applies


def test_param_orphan_block_scopes_the_instruction_to_the_profile_it_judged():
    """The title and the detail name one profile, so a bare delete is a wrong instruction:
    another profile of the same workspace launches the node and reads the block."""
    found = _run(_orphan_atlas(), "param.orphan_block")

    assert len(found) == 1
    assert found[0].profiles == [PROFILE]
    assert found[0].fix == (
        "Launch the node in profile demo, or delete the velocity_smoother block once no "
        "profile launches it."
    )
    assert "once no profile launches it" in found[0].say


def _selected_block_atlas(lookup: str, base_class: str) -> model.Atlas:
    """A block that names its own class, that no selector list holds, and a manifest that
    exports the class under one base class."""
    builder = _builder()
    manifest = ids.build_file("demo_controller", "plugins.xml")
    builder.node(manifest)
    plugin_id = ids.build_plugin(base_class, lookup)
    builder.fact(
        plugin_id,
        DECLARED,
        values={"base_class": base_class, "lookup": lookup},
        confidence=model.Confidence.CERTAIN.value,
        evidence=[_source(manifest, f"class[{lookup}]")],
    )
    for name, value in (("plugin", lookup), ("weight", 3)):
        param_id = ids.build_param(NODE, f"FollowPath_OFF.{name}")
        builder.fact(
            param_id,
            DECLARED,
            profile=PROFILE,
            values={
                "state": model.ParamState.INERT_BY_DESIGN.value,
                "block": "FollowPath_OFF",
                "owner": "/demo_server",
                "plugin": lookup if name == "plugin" else "",
            },
        )
        builder.set_param(
            param_id,
            state=model.ParamState.INERT_BY_DESIGN.value,
            candidates=[
                _candidate(
                    value,
                    model.Mechanism.YAML_BASE.value,
                    _source(BASE, f"demo_server.ros__parameters.FollowPath_OFF.{name}", 60),
                    rank=0,
                )
            ],
        )
    return builder.recompute()


def test_plugin_inert_by_design_recovers_the_key_the_block_class_determines():
    """The block names its own class, the class is exported under one base class, and one
    selector key spells that base class. Naming it is a lookup and not a guess."""
    found = _run(
        _selected_block_atlas("demo_controller/Spare", "nav2_core::Controller"),
        "plugin.inert_by_design",
    )

    assert len(found) == 1
    assert "controller_plugins does not name FollowPath_OFF" in found[0].say
    assert "Add FollowPath_OFF to controller_plugins" in found[0].fix


def test_plugin_inert_by_design_names_no_key_when_two_spellings_share_the_base_class():
    """A progress checker is spelled by two keys, and picking one of them is the guess this
    rule was told to stop making."""
    found = _run(
        _selected_block_atlas("demo_controller/Slow", "nav2_core::ProgressChecker"),
        "plugin.inert_by_design",
    )

    assert len(found) == 1
    assert "no selector list of /demo_server names FollowPath_OFF" in found[0].say
    assert "progress_checker_plugin" not in found[0].say


def test_plugin_unused_writes_profiles_in_the_plural_in_a_sentence_meant_to_be_quoted():
    def build(builder: model.AtlasBuilder) -> None:
        _launchfile(builder, "demo", profile=PROFILE)
        _launchfile(builder, "other", profile=OTHER_PROFILE)

    found = _run(_unused_atlas(build), "plugin.unused")

    assert len(found) == 1
    assert "profiles demo and other" in found[0].say
    assert "profile demo, other" not in found[0].say


def test_plugin_unused_states_one_profile_in_the_singular():
    def build(builder: model.AtlasBuilder) -> None:
        _launchfile(builder, "demo", profile=PROFILE)
        _launchfile(builder, "other", profile=OTHER_PROFILE, loadable=False)

    found = _run(_unused_atlas(build), "plugin.unused")

    assert "profile demo " in found[0].say
    assert "profiles" not in found[0].say


def test_the_plugin_family_states_one_silence_once():
    """A per profile row and an atlas wide row for one fact make a renderer that prints
    every row say it twice."""

    def build(builder: model.AtlasBuilder) -> None:
        _launchfile(builder, "demo", profile=PROFILE, loadable=False)
        _launchfile(builder, "other", profile=OTHER_PROFILE, loadable=False)

    rows = plugin_rules.silences(_unused_atlas(build))

    assert [row["profile"] for row in rows] == [PROFILE, OTHER_PROFILE]
    assert all(row["rule"] == "plugin.unused" for row in rows)


def test_an_atlas_with_no_profile_at_all_still_says_why_the_rule_was_quiet():
    rows = plugin_rules.silences(model.AtlasBuilder().build())

    assert [row["profile"] for row in rows] == [""]
    assert "holds no profile" in rows[0]["reason"]


def test_param_set_undeclared_names_the_declared_parameter_a_typo_meant():
    """`stop_distnce` in the yaml beside a node that declares `stop_distance` is a typo far more
    often than a new parameter, and the finding says which line to rename it on."""
    found = _run(_undeclared_atlas("max_sped", declared=("max_speed",)), "param.set_undeclared")
    assert len(found) == 1
    assert "declares max_speed, which is likely what was meant" in found[0].say
    assert found[0].fix.startswith("Rename max_sped to max_speed at")


def _sim_time_atlas(values: dict[str, bool | str], *, profile: str = PROFILE) -> model.Atlas:
    builder = _builder()
    for line, (node_name, value) in enumerate(sorted(values.items()), start=10):
        owner = ids.build_node(node_name)
        builder.node(owner, name_source=model.NameSource.LITERAL.value)
        builder.fact(owner, DECLARED, profile=profile, values={"name": node_name})
        source = _source(LAUNCH, f"{node_name}.use_sim_time", line)
        winner = _candidate(value, "launch_inline_dict", source, wins=True, profile=profile)
        _param(builder, "use_sim_time", [winner], owner=owner, profile=profile)
    return builder.build()


def test_param_sim_time_mixed_names_the_nodes_on_the_other_clock():
    atlas = _sim_time_atlas({"/amcl": True, "/planner": True, "/rviz": False})
    (found,) = _run(atlas, "param.sim_time_mixed")
    assert found.severity == model.Severity.WARN.value
    assert found.title == "use_sim_time is False for 1 and True for 2 nodes in profile demo"
    assert found.say.startswith("/rviz run with use_sim_time False while 2 other nodes")
    assert "demo_bringup/launch/demo.launch.py:12" in found.say
    assert found.subject == PROFILE


def test_param_sim_time_mixed_is_silent_when_every_stated_value_agrees():
    assert _run(_sim_time_atlas({"/amcl": True, "/planner": True}), "param.sim_time_mixed") == []


def test_param_sim_time_mixed_never_reads_a_value_that_is_not_a_bool():
    """A launch argument the walk could not fold stays a string, and a string is no clock."""
    atlas = _sim_time_atlas({"/amcl": True, "/rviz": "$(var use_sim_time)"})
    assert _run(atlas, "param.sim_time_mixed") == []


def test_param_sim_time_mixed_finding_id_is_stable_across_two_runs():
    values = {"/amcl": True, "/rviz": False}
    assert _ids(_sim_time_atlas(values), "param.sim_time_mixed") == _ids(
        _sim_time_atlas(values), "param.sim_time_mixed"
    )


def test_param_sim_time_mixed_skips_an_unreachable_owner():
    atlas = _sim_time_atlas({"/running": True, "/disabled": False})
    atlas.entities[ids.build_node("/disabled")].attrs["unreachable"] = True
    assert _run(atlas, "param.sim_time_mixed") == []


@pytest.mark.parametrize(
    ("outcomes", "mixed"),
    [(("false",), False), (("false", "true"), True), (("unknown",), True)],
)
def test_param_sim_time_mixed_uses_every_launch_site(outcomes, mixed):
    builder = model.AtlasBuilder()
    builder.atlas = _sim_time_atlas({"/running": True, "/disabled": False})
    for outcome in outcomes:
        site = ids.build_launchent(
            "demo_bringup", "launch/demo.launch.py", (ids.LaunchStep("Node", outcome),)
        )
        builder.fact(
            site,
            DECLARED,
            profile=PROFILE,
            values={"gates": [{"mechanism": "IfCondition", "outcome": outcome}]},
        )
        builder.edge(
            "launches",
            site,
            ids.build_node("/disabled"),
            key=ids.edge_key(launchent=site, profile=PROFILE),
            layer=DECLARED,
            profile=PROFILE,
        )
    assert bool(_run(builder.build(), "param.sim_time_mixed")) == mixed


def test_param_sim_time_mixed_reachability_is_per_profile():
    builder = model.AtlasBuilder()
    builder.atlas = _sim_time_atlas({"/running": True, "/disabled": False})
    for profile, outcome in ((PROFILE, "false"), (OTHER_PROFILE, "true")):
        site = ids.build_launchent(
            "demo_bringup", "launch/demo.launch.py", (ids.LaunchStep("Node", outcome),)
        )
        builder.fact(
            site,
            DECLARED,
            profile=profile,
            values={"gates": [{"mechanism": "IfCondition", "outcome": outcome}]},
        )
        builder.edge(
            "launches",
            site,
            ids.build_node("/disabled"),
            key=ids.edge_key(launchent=site, profile=profile),
            layer=DECLARED,
            profile=profile,
        )
    assert _run(builder.build(), "param.sim_time_mixed") == []


def test_param_sim_time_mixed_names_the_include_that_brought_the_value_in():
    """nav2 states rviz's use_sim_time as a default in rviz_launch.py, and the finding pointed
    there instead of at the include in the file that starts the profile."""
    atlas = _sim_time_atlas({"/amcl": True, "/planner": True, "/rviz": False})
    builder = model.AtlasBuilder()
    builder.atlas = atlas
    parent = ids.build_launchfile("demo_bringup", "launch/multi.launch.py")
    child = ids.build_launchfile("demo_bringup", "launch/demo.launch.py")
    site = _source("file:demo_bringup|launch/multi.launch.py", "IncludeLaunchDescription", 158)
    for node in (parent, child):
        builder.node(node)
    builder.edge("includes", parent, child, layer=DECLARED, profile=PROFILE, evidence=[site])
    (found,) = _run(builder.build(), "param.sim_time_mixed")
    assert (
        "gets it from demo_bringup/launch/demo.launch.py:12, included at "
        "demo_bringup/launch/multi.launch.py:158." in found.say
    )
    assert "passing use_sim_time at demo_bringup/launch/multi.launch.py:158" in found.fix
    assert site in found.where


def test_param_set_undeclared_offers_no_name_a_whole_word_away():
    """`min_laser_range` beside a declared `max_laser_range` is a different parameter, not a
    typo: the two differ in a word, and calling one the misspelling of the other misleads."""
    found = _run(
        _undeclared_atlas("min_laser_range", declared=("max_laser_range",)),
        "param.set_undeclared",
    )
    assert len(found) == 1
    assert "likely what was meant" not in found[0].say
    assert found[0].fix.startswith("Declare min_laser_range")


def test_param_set_undeclared_offers_a_name_one_slip_away():
    for typo, meant in (("stop_distnce", "stop_distance"), ("max_spede", "max_speed")):
        found = _run(_undeclared_atlas(typo, declared=(meant,)), "param.set_undeclared")
        assert f"declares {meant}, which is likely what was meant" in found[0].say


def test_param_set_undeclared_offers_a_name_with_an_inner_word_left_out():
    """nav2 main sets controller.simulation_step, and the dock controller declares
    controller.simulation_time_step: no hint, because no letter slipped."""
    found = _run(
        _undeclared_atlas(
            "controller.simulation_step", declared=("controller.simulation_time_step",)
        ),
        "param.set_undeclared",
    )
    assert "declares controller.simulation_time_step, which is likely what was meant" in (
        found[0].say
    )
    for name, other in (("range", "min_range"), ("min_range", "min_range_x")):
        found = _run(_undeclared_atlas(name, declared=(other,)), "param.set_undeclared")
        assert "likely what was meant" not in found[0].say, name


def test_param_set_undeclared_offers_no_name_whose_whole_word_changed():
    """`kd` beside `ki`, or `x_min` beside `y_min`, are other parameters: the slip replaced a
    word of one or two letters whole, and a short gain or axis name is never a typo of another."""
    for name, other in (("kd", "ki"), ("x_min", "y_min")):
        found = _run(_undeclared_atlas(name, declared=(other,)), "param.set_undeclared")
        assert "likely what was meant" not in found[0].say, name
