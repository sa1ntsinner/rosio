"""The argparse entry point: exit codes, the pinned encoder, the agent verbs.

The query verbs are wired here and implemented in `rosio/query.py`, so this suite asserts
the wiring and not the rendering: that every verb answers from an atlas file and never
extracts, that a question nothing answered exits with the usage code, and that the drift
gate is a different decision from the print filter.
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

import pytest

from rosio import ATLAS_SCHEMA_VERSION, __version__, cli, extra_command, ids, model
from rosio.cli import HANDLERS, IMPLEMENTED, VERBS, build_parser, commands, main
from rosio.errors import EXIT_DRIFT, EXIT_OK, EXIT_USAGE, UsageError
from rosio.model import Candidate, Finding, Source, Unresolved

PROFILE = "profile:nx_navigation"

PARAMS_FILE = ids.build_file("lx_bringup", "config/nav2_params.yaml")
OVERLAY_FILE = ids.build_file("lx_bringup", "config/nav2_overrides.yaml")
CONTROLLER = ids.build_node("/controller_server")
MONITOR = ids.build_node("/collision_monitor")
CMD_VEL = ids.build_topic("/cmd_vel")
COST_PENALTY = ids.build_param(CONTROLLER, "FollowPath.cost_penalty")


def _atlas() -> model.Atlas:
    """One small atlas holding what the verbs have to answer from: a candidate chain, an
    endpoint pair, a blind spot, and one finding of each severity the gate grades."""
    builder = model.AtlasBuilder(capabilities=["drift.findings", "layer.declared"])
    builder.add_profile("nx_navigation")
    for file_id in (PARAMS_FILE, OVERLAY_FILE):
        builder.node(file_id)
    builder.fact(CONTROLLER, "declared", profile=PROFILE, values={"pkg": "nav2_controller"})
    builder.node(CONTROLLER, name_source="literal")
    builder.fact(MONITOR, "declared", profile=PROFILE, values={"pkg": "nav2_collision_monitor"})
    builder.node(MONITOR, name_source="literal")
    builder.node(CMD_VEL)
    builder.edge(
        "publishes",
        CONTROLLER,
        CMD_VEL,
        key=ids.edge_key(profile=PROFILE),
        layer="declared",
        profile=PROFILE,
        values={"qos_depth": 10},
    )
    builder.edge(
        "subscribes",
        MONITOR,
        CMD_VEL,
        key=ids.edge_key(profile=PROFILE),
        layer="declared",
        profile=PROFILE,
        values={"qos_depth": 1},
    )
    builder.set_param(
        COST_PENALTY,
        state="effective",
        candidates=[
            Candidate(
                value=1.0,
                value_type="float",
                mechanism="yaml_overlay_merge",
                source=Source(file=OVERLAY_FILE, locator="FollowPath.cost_penalty", line_hint=154),
                rank=0,
                wins=True,
                profile=PROFILE,
            ),
            Candidate(
                value=2.0,
                value_type="float",
                mechanism="yaml_base",
                source=Source(file=PARAMS_FILE, locator="FollowPath.cost_penalty", line_hint=381),
                rank=1,
                wins=False,
                profile=PROFILE,
                shadow_kind="value_changed",
                shadowed_by=0,
            ),
        ],
    )
    builder.edge("has_param", CONTROLLER, COST_PENALTY)
    builder.unresolved(
        Unresolved(
            want="topic_name",
            owner=MONITOR,
            expr="self.get_parameter('cmd_vel_in_topic')",
            reason="parameter with no default",
            evidence=[Source(file=PARAMS_FILE, line_hint=430)],
        )
    )
    builder.finding(
        Finding(
            rule="param.type_changed",
            severity="error",
            subject=COST_PENALTY,
            title="cost_penalty changes type between the two files",
            detail="the overlay states a float and the base file states an int",
            fix="write 1.0 in both files",
            say="cost_penalty on /controller_server is a float in nav2_overrides.yaml:154",
            profiles=[PROFILE],
            where=[Source(file=OVERLAY_FILE, line_hint=154)],
        )
    )
    builder.finding(
        Finding(
            rule="param.shadowed",
            severity="warn",
            subject=COST_PENALTY,
            title="cost_penalty is set twice and one line does nothing",
            detail="the overlay states 1.0 and the base file states 2.0",
            fix="delete the base line",
            say="cost_penalty on /controller_server is 1.0 from nav2_overrides.yaml:154",
            discriminator="hbase",
            profiles=[PROFILE],
            where=[Source(file=PARAMS_FILE, line_hint=381)],
        )
    )
    return builder.build()


@pytest.fixture(scope="module")
def atlas_file(tmp_path_factory: pytest.TempPathFactory) -> Path:
    target = tmp_path_factory.mktemp("atlas") / "atlas.json"
    target.write_text(model.dumps(_atlas().to_json()), encoding="utf-8", newline="\n")
    return target


@pytest.fixture
def no_extraction(monkeypatch: pytest.MonkeyPatch):
    """Every query verb answers from the file. Extraction from one of them is a bug."""
    from rosio import extract

    def refuse(*args, **kwargs):
        raise AssertionError("a query verb re extracted instead of reading the atlas")

    monkeypatch.setattr(extract, "run", refuse)


# ---------------------------------------------------------------------------
# the shape of the command line


def test_version_json_states_the_schema_major_and_minor(capsys):
    assert main(["version", "--json"]) == 0
    text = capsys.readouterr().out
    assert text == model.dumps(
        {"version": __version__, "atlas_schema_version": model.SCHEMA_VERSION}
    )
    assert model.SCHEMA_VERSION.startswith(f"{ATLAS_SCHEMA_VERSION}.")


def test_version_plain_states_the_schema_it_writes(capsys):
    assert main(["version"]) == 0
    assert capsys.readouterr().out == f"rosio {__version__}, atlas schema {model.SCHEMA_VERSION}\n"


def test_the_version_flag_says_what_the_version_verb_says(capsys):
    with pytest.raises(SystemExit) as stopped:
        main(["--version"])
    assert stopped.value.code == 0
    assert capsys.readouterr().out == f"rosio {__version__}, atlas schema {model.SCHEMA_VERSION}\n"


def test_the_help_names_every_exit_code_a_verb_returns():
    from rosio.errors import EXIT_COLLECTOR, EXIT_CONTRACT, EXIT_ERROR

    for code in (EXIT_OK, EXIT_ERROR, EXIT_USAGE, EXIT_COLLECTOR, EXIT_CONTRACT, EXIT_DRIFT):
        assert f"{code} " in cli.EPILOG.split("Exit codes:", 1)[1]


def _verb(name: str):
    parser = build_parser()
    verbs = next(
        action.choices
        for action in parser._actions
        if isinstance(action, argparse._SubParsersAction)
    )
    return verbs[name]


def _flags(name: str) -> set[str]:
    return {option for action in _verb(name)._actions for option in action.option_strings}


def test_a_verb_offers_only_the_output_flags_it_reads():
    assert "--deterministic" in _flags("extract")
    for verb in ("version", "install", "uninstall"):
        assert "--deterministic" not in _flags(verb)
        assert "--out" not in _flags(verb)
    assert "--json" in _flags("version")
    assert not {"--json", "--quiet"} & _flags("install")
    assert {"--out", "--json", "--quiet"} <= _flags("drift")
    assert "--deterministic" not in _flags("drift")


def test_positional_arguments_and_their_choices_are_described():
    from rosio.agent.platforms import keys as platform_keys

    path_help = _verb("path").format_help()
    assert "where to start" in path_help and "where to end" in path_help
    assert "older atlas.json" in _verb("diff").format_help()
    install_help = "".join(_verb("install").format_help().split())
    for key in platform_keys():
        assert key in install_help
    want_help = "".join(_verb("unresolved").format_help().split())
    for want in model.Want:
        assert want.value in want_help


def test_the_help_starts_where_the_readme_starts():
    epilog = " ".join(cli.EPILOG.split())
    order = [epilog.index(f"rosio {verb}") for verb in ("init", "extract", "drift", "render")]
    assert order == sorted(order)


def test_no_command_prints_help_and_reports_usage(capsys):
    assert main([]) == EXIT_USAGE
    assert "atlas.json" in capsys.readouterr().out


@pytest.mark.parametrize(
    "argv, want",
    [
        (["extract"], None),
        (["--deterministic", "extract"], True),
        (["extract", "--deterministic"], True),
        (["--no-deterministic", "extract"], False),
        (["extract", "--no-deterministic"], False),
    ],
)
def test_a_global_flag_means_the_same_on_both_sides_of_the_verb(argv, want):
    """A subparser writes its own defaults over the namespace, so a flag whose subcommand
    copy carries a default means one thing before the verb and another after it."""
    assert build_parser().parse_args(argv).deterministic is want


def test_an_unset_flag_is_none_so_the_config_can_be_overridden():
    """False and not given have to stay distinguishable, or --no-deterministic cannot turn a
    configured true off."""
    args = build_parser().parse_args(["extract"])
    assert args.deterministic is None
    assert args.out is None
    for argv in (["--json", "extract"], ["extract", "--json"]):
        assert build_parser().parse_args(argv).json is True


def test_no_deterministic_turns_a_configured_true_off():
    from rosio.cli import _with_overrides
    from rosio.config import DEFAULT_OUT_DIR, Config, RootSpec

    parser = build_parser()
    for configured in (True, False):
        cfg = Config(
            roots=(RootSpec("src", "/does/not/matter"),),
            profiles=(),
            deterministic=configured,
        )
        merged = _with_overrides(cfg, parser.parse_args(["extract"]), DEFAULT_OUT_DIR)
        assert merged.deterministic is True
        for argv in (["--no-deterministic", "extract"], ["extract", "--no-deterministic"]):
            merged = _with_overrides(cfg, parser.parse_args(argv), DEFAULT_OUT_DIR)
            assert merged.deterministic is False


def test_every_verb_has_a_handler_and_a_parser():
    """The verb set is closed, so a new verb needs the table, the parser and the handler."""
    parser = build_parser()
    assert set(commands()) == set(VERBS)
    assert set(HANDLERS) == set(VERBS)
    for name in sorted(HANDLERS):
        assert parser.parse_args([name]).command == name


def test_every_verb_in_the_closed_set_does_the_work():
    """The three agent verbs used to print a milestone line while the code behind them
    existed, which shipped an MCP registration pointing at a command that exits 2."""
    assert set(VERBS) == IMPLEMENTED
    for name in ("install", "uninstall", "serve"):
        assert HANDLERS[name].__module__ == "rosio.cli"
        assert "not implemented" not in (HANDLERS[name].__doc__ or "")


# ---------------------------------------------------------------------------
# every verb answers from a file


@pytest.mark.parametrize(
    "argv",
    [
        ["drift"],
        ["explain", "/controller_server"],
        ["param", "/controller_server", "FollowPath.cost_penalty"],
        ["query", "which", "nodes", "publish", "cmd_vel"],
        ["path", "/controller_server", "/collision_monitor"],
        ["unresolved"],
        ["graph"],
        ["graph", "--format", "dot"],
    ],
)
def test_a_query_verb_answers_from_the_atlas_without_extracting(
    argv, atlas_file: Path, no_extraction, capsys
):
    assert main([*argv, "--atlas", str(atlas_file)]) == EXIT_OK
    assert capsys.readouterr().out.strip()


def test_diff_answers_from_two_files_without_extracting(atlas_file: Path, no_extraction, capsys):
    code = main(["diff", str(atlas_file), str(atlas_file)])
    assert code == EXIT_OK
    assert "no semantic difference" in capsys.readouterr().out


def test_diff_compares_two_profiles_of_one_atlas(atlas_file: Path, capsys):
    code = main(
        [
            "diff",
            str(atlas_file),
            "--profile",
            "nx_navigation",
            "--profile",
            "nx_navigation",
        ]
    )
    assert code == EXIT_OK
    assert "profile nx_navigation" in capsys.readouterr().out


def test_diff_needs_two_things_to_compare(atlas_file: Path, capsys):
    assert main(["diff", str(atlas_file)]) == EXIT_USAGE
    assert "two atlases" in capsys.readouterr().err
    assert main(["diff", str(atlas_file), "--profile", "nx_navigation"]) == EXIT_USAGE
    assert "exactly two" in capsys.readouterr().err


def test_the_atlas_is_found_under_out_when_it_is_not_named(atlas_file: Path, capsys):
    assert main(["drift", "--out", str(atlas_file.parent)]) == EXIT_OK
    assert "findings" in capsys.readouterr().out


def test_a_missing_atlas_names_the_path_it_looked_at(tmp_path: Path, capsys):
    assert main(["drift", "--out", str(tmp_path)]) == EXIT_USAGE
    err = capsys.readouterr().err
    assert str(tmp_path / "atlas.json") in err
    assert "extract" in err


def test_a_directory_is_accepted_wherever_a_file_is(atlas_file: Path, capsys):
    assert main(["unresolved", "--atlas", str(atlas_file.parent)]) == EXIT_OK
    assert "unresolved" in capsys.readouterr().out


@pytest.mark.parametrize("verb", ["explain", "param", "query"])
def test_a_verb_with_nothing_to_look_up_says_what_it_needs(verb, atlas_file: Path, capsys):
    assert main([verb, "--atlas", str(atlas_file)]) == EXIT_USAGE
    assert "for example" in capsys.readouterr().err


def test_path_needs_both_ends(atlas_file: Path, capsys):
    assert main(["path", "/controller_server", "--atlas", str(atlas_file)]) == EXIT_USAGE
    assert "both ends" in capsys.readouterr().err


# ---------------------------------------------------------------------------
# what the answers carry


def test_param_prints_the_winner_and_the_file_to_edit(atlas_file: Path, capsys):
    code = main(["param", "/controller_server FollowPath.cost_penalty", "--atlas", str(atlas_file)])
    printed = capsys.readouterr().out

    assert code == EXIT_OK
    assert "effective 1.0" in printed
    assert "nav2_overrides.yaml:154" in printed
    assert "nav2_params.yaml:381" in printed, "the shadowed line is the one an engineer deletes"


def test_explain_takes_a_finding_id_and_prints_its_evidence(atlas_file: Path, capsys):
    from rosio.model import Atlas

    atlas = Atlas.loads(atlas_file.read_text(encoding="utf-8"))
    finding = next(item for item in atlas.findings if item.rule == "param.shadowed")

    assert main(["explain", finding.id, "--atlas", str(atlas_file)]) == EXIT_OK
    printed = capsys.readouterr().out
    assert "param.shadowed" in printed
    assert "nav2_params.yaml:381" in printed
    assert "fix:" in printed


def test_a_question_nothing_answers_exits_with_the_usage_code(atlas_file: Path, capsys):
    code = main(["query", "octomap_server", "--atlas", str(atlas_file)])
    assert code == EXIT_USAGE
    assert "not answered" in capsys.readouterr().out


def test_an_ambiguous_selector_lists_the_candidates_and_does_not_pick_one(atlas_file: Path, capsys):
    code = main(["explain", "controller", "--atlas", str(atlas_file)])
    printed = capsys.readouterr().out
    assert code == EXIT_USAGE
    assert "pick one" in printed


def test_json_output_goes_through_the_pinned_encoder(atlas_file: Path, capsys):
    assert main(["drift", "--atlas", str(atlas_file), "--json"]) == EXIT_OK
    payload = json.loads(capsys.readouterr().out)
    assert payload["verb"] == "drift"
    assert payload["ok"] is True
    assert payload["data"]["by_severity"] == {"error": 1, "warn": 1, "info": 0}


# ---------------------------------------------------------------------------
# the drift gate


def test_the_gate_severity_and_the_print_threshold_are_two_decisions(atlas_file: Path, capsys):
    """AGENTS section 11. A build told to print errors and to fail on warnings fails on a
    warning it never printed, so the gate must not read the printed list."""
    code = main(
        ["drift", "--atlas", str(atlas_file), "--min-severity", "error", "--fail-on", "warn"]
    )
    captured = capsys.readouterr()

    assert code == EXIT_DRIFT
    assert "param.shadowed" not in captured.out, "the warn row was filtered out of the report"
    assert "2 findings at warn or above" in captured.err
    assert "param.shadowed" in captured.err, "the gate names the rule it tripped on"


def test_the_gate_is_open_unless_it_is_asked_for(atlas_file: Path):
    assert main(["drift", "--atlas", str(atlas_file), "--quiet"]) == EXIT_OK
    assert main(["drift", "--atlas", str(atlas_file), "--quiet", "--fail-on", "never"]) == EXIT_OK


@pytest.mark.parametrize(
    "fail_on, want",
    [("error", EXIT_DRIFT), ("warn", EXIT_DRIFT), ("info", EXIT_DRIFT), ("never", EXIT_OK)],
)
def test_the_gate_trips_at_every_severity_the_atlas_carries(
    fail_on, want, atlas_file: Path, capsys
):
    assert main(["drift", "--atlas", str(atlas_file), "--fail-on", fail_on]) == want


def test_a_rule_that_could_not_be_trusted_fails_the_gate_as_an_error(tmp_path: Path, capsys):
    """A rule that raised left no finding, and `--fail-on warn` read that as a clean stack."""
    atlas = _atlas()
    atlas.findings = []
    atlas.counters["drift"] = {"failed": ["launch.unloadable"]}
    target = tmp_path / "atlas.json"
    target.write_text(model.dumps(atlas.to_json()), encoding="utf-8", newline="\n")
    for fail_on in ("error", "warn", "info"):
        code = main(
            ["drift", "--atlas", str(target), "--only", "launch.unloadable", "--fail-on", fail_on]
        )
        captured = capsys.readouterr()
        assert code == EXIT_DRIFT
        assert "could not be trusted" in captured.out
        assert "launch.unloadable" in captured.err
    assert main(["drift", "--atlas", str(target), "--fail-on", "never", "--quiet"]) == EXIT_OK
    only = ["drift", "--atlas", str(target), "--only", "param.shadowed", "--fail-on", "error"]
    assert main(only) == EXIT_OK, "the failed rule is outside the rule that was asked about"


def test_only_scopes_the_report_and_the_gate_together(atlas_file: Path, capsys):
    code = main(
        ["drift", "--atlas", str(atlas_file), "--only", "param.shadowed", "--fail-on", "error"]
    )
    printed = capsys.readouterr().out

    assert code == EXIT_OK, "the error finding is outside the rule that was asked about"
    assert "param.shadowed" in printed
    assert "param.type_changed" not in printed


def test_only_refuses_a_rule_id_that_is_in_no_vocabulary(atlas_file: Path, capsys):
    """A mistyped rule id would report no drift, which reads exactly like a clean stack."""
    assert main(["drift", "--atlas", str(atlas_file), "--only", "param.shadowd"]) == EXIT_USAGE
    assert "unknown drift rule" in capsys.readouterr().err


# ---------------------------------------------------------------------------
# waivers


def _waiver_file(directory: Path, *rows: tuple[str, str]) -> Path:
    target = directory / "rosio-waivers.yaml"
    lines = ["waivers:"]
    for key, reason in rows:
        lines.append(f'  - key: "{key}"')
        lines.append(f"    reason: {reason}")
    target.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return target


def _finding(rule: str) -> Finding:
    return next(item for item in _atlas().findings if item.rule == rule)


def test_a_waived_finding_stops_failing_the_gate_and_stays_visible(
    atlas_file: Path, tmp_path: Path, capsys
):
    """AGENTS section 10.1, the whole point of the entry point.

    Without it a team with one known finding turns the gate off. With it the gate goes
    green while the finding, its id and the reason it was accepted stay on the page, so the
    waiver is something a reader can delete rather than something nobody can see.
    """
    error = _finding("param.type_changed")
    waivers = _waiver_file(tmp_path, (error.id, "the base file is generated, ticket 412"))

    before = main(["drift", "--atlas", str(atlas_file), "--fail-on", "error", "--quiet"])
    code = main(
        ["drift", "--atlas", str(atlas_file), "--fail-on", "error", "--waivers", str(waivers)]
    )
    printed = capsys.readouterr().out

    assert before == EXIT_DRIFT
    assert code == EXIT_OK
    assert "param.type_changed" in printed, "a waived finding is still reported"
    assert "1 of 1 waivers matched" in printed
    assert f"waived {error.id}" in printed
    assert "the base file is generated, ticket 412" in printed
    (line,) = [row for row in printed.splitlines() if "(param.type_changed)" in row]
    assert line.endswith("   waived"), "the finding is marked where it is listed"
    assert sum(row.endswith("   waived") for row in printed.splitlines()) == 1


def test_a_waiver_does_not_silence_the_findings_it_does_not_name(atlas_file: Path, tmp_path: Path):
    error = _finding("param.type_changed")
    waivers = _waiver_file(tmp_path, (error.id, "accepted"))

    code = main(
        ["drift", "--atlas", str(atlas_file), "--fail-on", "warn", "--waivers", str(waivers)]
    )

    assert code == EXIT_DRIFT, "the warn finding is not waived and still fails the gate"


def test_a_waiver_keys_on_the_fingerprint_as_well_as_the_id(atlas_file: Path, tmp_path: Path):
    error = _finding("param.type_changed")
    waivers = _waiver_file(tmp_path, (error.fingerprint, "accepted by fingerprint"))

    code = main(
        ["drift", "--atlas", str(atlas_file), "--fail-on", "error", "--waivers", str(waivers)]
    )

    assert code == EXIT_OK


def test_the_configuration_names_the_waiver_file_when_no_flag_does(
    atlas_file: Path, tmp_path: Path
):
    error = _finding("param.type_changed")
    _waiver_file(tmp_path, (error.id, "accepted in the configuration"))
    config = tmp_path / "rosio.yaml"
    config.write_text(
        "roots:\n  - kind: src\n    path: .\nwaivers: rosio-waivers.yaml\n", encoding="utf-8"
    )

    code = main(
        ["drift", "--atlas", str(atlas_file), "--fail-on", "error", "--config", str(config)]
    )

    assert code == EXIT_OK, "the key is resolved against the directory of the config file"


def test_a_waiver_key_that_matches_nothing_is_named(atlas_file: Path, tmp_path: Path, capsys):
    """A stored key rots when the finding it covers is fixed, and a silent waiver file only
    ever grows."""
    gone = "find:param.shadowed|param:/gone%7Ca.b|-"
    waivers = _waiver_file(tmp_path, (gone, "the finding this covered was fixed"))

    main(["drift", "--atlas", str(atlas_file), "--waivers", str(waivers)])
    printed = capsys.readouterr().out

    assert "0 of 1 waivers matched" in printed
    assert f"waiver matched nothing, delete it: {gone}" in printed


def test_a_waiver_needs_a_reason_and_a_key_that_is_one(atlas_file: Path, tmp_path: Path, capsys):
    """A rule id is the mistake this catches: it reads like a waiver and silences nothing."""
    no_reason = tmp_path / "no-reason.yaml"
    no_reason.write_text("waivers:\n  - key: sha256:0123456789abcdef\n", encoding="utf-8")
    rule_id = tmp_path / "rule-id.yaml"
    rule_id.write_text(
        "waivers:\n  - key: param.shadowed\n    reason: silence the rule\n", encoding="utf-8"
    )

    assert main(["drift", "--atlas", str(atlas_file), "--waivers", str(no_reason)]) == EXIT_USAGE
    assert "needs a reason" in capsys.readouterr().err
    assert main(["drift", "--atlas", str(atlas_file), "--waivers", str(rule_id)]) == EXIT_USAGE
    assert "neither a finding id nor a fingerprint" in capsys.readouterr().err


def test_a_waiver_never_reaches_the_report_as_a_missing_finding(
    atlas_file: Path, tmp_path: Path, capsys
):
    """The json answer keeps every finding and states the waiver beside them, so a consumer
    counting findings sees the same number with a waiver file and without one."""
    error = _finding("param.type_changed")
    waivers = _waiver_file(tmp_path, (error.id, "accepted"))

    main(["drift", "--atlas", str(atlas_file), "--json"])
    plain = json.loads(capsys.readouterr().out)
    main(["drift", "--atlas", str(atlas_file), "--json", "--waivers", str(waivers)])
    waived = json.loads(capsys.readouterr().out)

    assert plain["data"]["total"] == waived["data"]["total"]
    assert "waived" not in plain["data"]
    assert [row["finding"] for row in waived["data"]["waived"]] == [error.id]
    assert waived["data"]["waivers_matching_nothing"] == []


def test_an_atlas_the_rules_never_ran_over_is_refused_rather_than_called_clean(
    tmp_path: Path, capsys
):
    """AGENTS section 9a: test the capability rather than guessing from an absent key. An
    atlas with no findings and no probe is ungraded, and reporting none reads as clean."""
    atlas = _atlas()
    atlas.capabilities = ["layer.declared"]
    atlas.findings = []
    target = tmp_path / "atlas.json"
    target.write_text(model.dumps(atlas.to_json()), encoding="utf-8", newline="\n")

    assert main(["drift", "--atlas", str(target)]) == EXIT_USAGE
    err = capsys.readouterr().err
    assert "clean stack" in err
    assert "extract it again" in err


# ---------------------------------------------------------------------------
# render, which turns the atlas into the three files a person reads

WS = Path(__file__).parent / "fixtures" / "ws_launch"

OUTPUTS = ("atlas.json", "ATLAS_REPORT.md", "DRIFT.md", "atlas.html")


def _copied(atlas_file: Path, tmp_path: Path) -> Path:
    target = tmp_path / "atlas.json"
    target.write_text(atlas_file.read_text(encoding="utf-8"), encoding="utf-8")
    return target


def test_render_writes_the_three_files_beside_the_atlas(
    atlas_file: Path, tmp_path: Path, no_extraction, capsys
):
    """No --out, so the outputs land beside the atlas they describe and a report can never
    belong to a different run than the atlas next to it."""
    target = _copied(atlas_file, tmp_path)
    assert main(["render", "--atlas", str(target)]) == EXIT_OK

    printed = capsys.readouterr().out
    for name in OUTPUTS[1:]:
        assert (tmp_path / name).is_file()
        assert name in printed


@pytest.mark.parametrize(
    "fmt, want",
    [("md", {"ATLAS_REPORT.md", "DRIFT.md"}), ("html", {"atlas.html"})],
)
def test_render_format_selects_what_is_written(fmt, want, atlas_file: Path, tmp_path: Path):
    target = _copied(atlas_file, tmp_path)
    assert main(["render", "--atlas", str(target), "--format", fmt, "--quiet"]) == EXIT_OK
    assert {item.name for item in tmp_path.iterdir()} - {"atlas.json"} == want


def test_render_writes_where_out_says(atlas_file: Path, tmp_path: Path):
    assert (
        main(["render", "--atlas", str(atlas_file), "--out", str(tmp_path / "site"), "--quiet"])
        == EXIT_OK
    )
    assert sorted(item.name for item in (tmp_path / "site").iterdir()) == [
        "ATLAS_REPORT.md",
        "DRIFT.md",
        "atlas.html",
    ]


def test_render_reports_what_it_wrote_through_the_pinned_encoder(
    atlas_file: Path, tmp_path: Path, no_extraction, capsys
):
    argv = ["render", "--atlas", str(atlas_file), "--out", str(tmp_path / "site"), "--json"]
    assert main(argv) == EXIT_OK
    payload = json.loads(capsys.readouterr().out)
    assert payload["verb"] == "render"
    assert len(payload["data"]["written"]) == 3


def test_extract_then_render_fills_the_output_directory(tmp_path: Path, capsys):
    """What the documentation promises: one directory, four files, two commands."""
    out = tmp_path / "rosio-out"
    assert main(["extract", str(WS), "--out", str(out), "--quiet"]) == EXIT_OK
    assert main(["render", "--out", str(out), "--quiet"]) == EXIT_OK
    assert sorted(item.name for item in out.iterdir()) == sorted(OUTPUTS)


def test_extract_names_each_phase_on_stderr_unless_quiet(tmp_path: Path, capsys):
    out = tmp_path / "rosio-out"
    assert main(["extract", str(WS), "--out", str(out)]) == EXIT_OK
    first, *lines = capsys.readouterr().err.splitlines()
    assert first.startswith(("rosio extract: config ", "rosio extract: no rosio.yaml found"))
    assert lines[0].startswith("rosio extract: indexed ")
    assert any(line.startswith("rosio extract: join, ") for line in lines)
    assert lines[-1].startswith("rosio extract: write, ")
    assert all(line.endswith("s elapsed)") for line in lines)
    assert main(["extract", str(WS), "--out", str(out), "--quiet"]) == EXIT_OK
    assert capsys.readouterr().err == ""


def test_extract_collapses_the_per_profile_lines_when_stderr_is_not_a_terminal(
    tmp_path: Path, capsys
):
    """Sixty one profiles were sixty one progress lines in a log."""
    assert main(["extract", str(WS), "--out", str(tmp_path / "out")]) == EXIT_OK
    lines = capsys.readouterr().err.splitlines()
    per_profile = [line for line in lines if ", profile " in line]
    assert len(per_profile) == 1
    assert "\r" not in "".join(lines)


def test_extract_names_the_config_it_read_and_where_it_wrote(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys
):
    workspace = tmp_path / "ws"
    shutil.copytree(WS, workspace)
    here = tmp_path / "here"
    here.mkdir()
    (here / "rosio.yaml").write_text(
        f"roots:\n  - {{kind: src, path: {workspace / 'src'}}}\n"
        f"  - {{kind: install, path: {workspace / 'install'}}}\n"
        "profiles:\n  - slug: only\n",
        encoding="utf-8",
    )
    monkeypatch.chdir(here)
    assert main(["extract", str(workspace)]) == EXIT_OK
    captured = capsys.readouterr()
    assert f"rosio extract: config {here / 'rosio.yaml'}" in captured.err
    assert f"rosio extract: {here / 'rosio-out' / 'atlas.json'} (" in captured.out
    assert "profiles 1: only" in captured.out

    (here / "rosio.yaml").write_text("profiles:\n  - slug: only\n", encoding="utf-8")
    assert main(["extract", str(workspace), "--quiet"]) == EXIT_OK
    captured = capsys.readouterr()
    assert f"rosio extract: ignored {here / 'rosio.yaml'}" in captured.err


def test_a_configured_root_that_is_not_there_names_the_ways_out(tmp_path: Path, capsys):
    """The example rosio.yaml declared an install root nobody had built."""
    shutil.copytree(WS / "src", tmp_path / "src")
    config = tmp_path / "rosio.yaml"
    config.write_text(
        "roots:\n  - {kind: src, path: src}\n  - {kind: install, path: install}\n",
        encoding="utf-8",
    )
    assert main(["extract", str(tmp_path), "--out", str(tmp_path / "out")]) == EXIT_USAGE
    err = capsys.readouterr().err
    assert f"install root does not exist: {tmp_path / 'install'}" in err
    assert "colcon build" in err and f"delete that root from {config}" in err


def test_install_tree_becomes_a_root_and_reaches_the_setup_file(tmp_path: Path):
    """The flag was accepted, documented and never read: an install tree named only on the
    command line never became a root and never reached the ament index."""
    import shutil

    tree = tmp_path / "elsewhere"
    shutil.copytree(WS / "install", tree)
    (tree / "setup.bash").write_text("export FOO=1\n", encoding="utf-8")
    sources = tmp_path / "ws"
    shutil.copytree(WS / "src", sources / "src")

    out = tmp_path / "without"
    assert main(["extract", str(sources), "--out", str(out), "--quiet"]) == EXIT_OK
    plain = json.loads((out / "atlas.json").read_text(encoding="utf-8"))["generated"]
    assert [row["kind"] for row in plain["roots"]] == ["src"]

    out = tmp_path / "with"
    argv = ["extract", str(sources), "--install-tree", str(tree), "--out", str(out), "--quiet"]
    assert main(argv) == EXIT_OK
    named = json.loads((out / "atlas.json").read_text(encoding="utf-8"))["generated"]
    assert sorted(row["kind"] for row in named["roots"]) == ["install", "src"]

    # The named tree's setup file reaches the run, and AGENTS section 9.3 says how it is
    # written down: a file outside every root is stated by its name and the hash of its own
    # content, never by where this box holds it, or every clone at another prefix rewrites a
    # committed atlas.json on its first rebuild.
    added = [item for item in named["ros_setup"] if item not in plain["ros_setup"]]
    assert len(added) == 1, "the install tree contributes exactly its own setup file"
    assert added[0].startswith("@outside/setup.bash#")
    assert not [item for item in named["ros_setup"] if item.startswith("/")]


def test_install_tree_replaces_the_one_the_config_declares(tmp_path: Path):
    """The command line wins over the config file, so the flag is not a second install root
    beside the configured one: the ament index would then answer from two trees."""
    import shutil

    shutil.copytree(WS, tmp_path / "ws")
    tree = tmp_path / "elsewhere"
    shutil.copytree(WS / "install", tree)
    out = tmp_path / "out"
    argv = ["extract", str(tmp_path / "ws"), "--install-tree", str(tree), "--out", str(out)]
    assert main([*argv, "--quiet"]) == EXIT_OK
    rows = json.loads((out / "atlas.json").read_text(encoding="utf-8"))["generated"]["roots"]
    assert [row["kind"] for row in rows if row["kind"] == "install"] == ["install"]


def test_install_tree_that_is_not_there_is_refused_before_anything_is_read(tmp_path, capsys):
    argv = ["extract", str(WS), "--install-tree", str(tmp_path / "nowhere"), "--quiet"]
    assert main(argv) == EXIT_USAGE
    assert "install tree does not exist" in capsys.readouterr().err


# ---------------------------------------------------------------------------
# probe, the one verb that reads a running system
#
# Nothing here needs a middleware: the collector takes the system it writes. What is asserted
# is the wiring and the refusal, and `tests/test_probe.py` owns what the probe reports.


@pytest.fixture
def fake_probe(monkeypatch: pytest.MonkeyPatch) -> list[dict]:
    """One talker on the graph, and a process table with nothing ROS in it.

    Returns what the probe was asked for, so the flags can be followed to the child."""
    from rosio.collect import probe as probe_collect
    from rosio.collect import proc as proc_collect

    asked: list[dict] = []

    def system(**kwargs) -> probe_collect.ObservedSystem:
        asked.append(kwargs)
        return probe_collect.ObservedSystem(
            nodes=[probe_collect.ObservedNode(name="talker", namespace="/", fqn="/talker")],
            endpoints=[
                probe_collect.Endpoint(
                    kind="publisher", node="/talker", name="/chatter", type="std_msgs/msg/String"
                )
            ],
            domain_id="77",
            localhost_only="1",
        ).sort()

    monkeypatch.setattr(probe_collect, "probe", system)
    monkeypatch.setattr(proc_collect, "scan", lambda _root=proc_collect.PROC, domain=None: [])
    monkeypatch.setenv("ROS_DOMAIN_ID", "77")
    return asked


@pytest.fixture
def empty_probe(monkeypatch: pytest.MonkeyPatch, fake_probe):
    from rosio.collect import probe as probe_collect

    monkeypatch.setattr(
        probe_collect,
        "probe",
        lambda **_kwargs: probe_collect.ObservedSystem(domain_id="77", reached=True),
    )


def test_probe_refuses_to_run_without_a_domain(tmp_path: Path, monkeypatch, capsys):
    """The machine this runs on can be wired to a vehicle. The domain is never guessed."""
    monkeypatch.delenv("ROS_DOMAIN_ID", raising=False)
    assert main(["probe", str(WS), "--out", str(tmp_path / "out")]) == EXIT_USAGE
    err = capsys.readouterr().err
    assert "--domain-id" in err
    assert "vehicle" in err
    assert not (tmp_path / "out").exists(), "nothing was written and nothing was probed"


def test_probe_refuses_a_domain_that_is_not_a_number(tmp_path: Path, monkeypatch, capsys):
    monkeypatch.delenv("ROS_DOMAIN_ID", raising=False)
    argv = ["probe", str(WS), "--out", str(tmp_path / "out"), "--domain-id", "seventy seven"]
    assert main(argv) == EXIT_USAGE
    assert "a domain id is a number" in capsys.readouterr().err


def test_probe_pins_the_domain_the_child_will_attach_to(tmp_path: Path, monkeypatch, fake_probe):
    """The flag has to win over a workspace setup file, so it is exported before the child
    starts and the collector reads it from there."""
    import os

    from rosio.collect import probe as probe_collect

    monkeypatch.setenv("ROS_DOMAIN_ID", "3")
    monkeypatch.delenv("ROS_LOCALHOST_ONLY", raising=False)
    argv = [
        "probe",
        str(WS),
        "--out",
        str(tmp_path / "out"),
        "--domain-id",
        "77",
        "--localhost-only",
        "--quiet",
    ]
    assert main(argv) == EXIT_OK
    assert os.environ["ROS_DOMAIN_ID"] == "77"
    assert probe_collect.asked_env() == {"ROS_DOMAIN_ID": "77", "ROS_LOCALHOST_ONLY": "1"}


def test_probe_adds_the_observed_layer_to_an_atlas_that_is_already_on_disk(
    atlas_file: Path, tmp_path: Path, fake_probe, capsys
):
    from rosio.model import Atlas

    target = _copied(atlas_file, tmp_path)
    before = Atlas.loads(target.read_text(encoding="utf-8"))

    assert main(["probe", str(WS), "--atlas", str(target), "--duration", "0"]) == EXIT_OK

    printed = capsys.readouterr().out
    assert "domain 77" in printed
    assert "nodes 1" in printed
    after = Atlas.loads(target.read_text(encoding="utf-8"))
    assert set(before.entities) < set(after.entities), "the declared layer survived"
    assert after.entities[ids.build_node("/talker")].fact("observed") is not None
    assert "layer.observed" in after.capabilities
    assert after.generated["collectors"] == ["proc", "probe"]


def test_extract_refuses_to_select_the_live_probe(tmp_path: Path, capsys):
    """This box is attached to a vehicle, so the one verb that attaches to a live middleware
    is the one that takes the domain id. Reaching it through extract is refused before a
    workspace is read, and the refusal names the verb that does it."""
    argv = ["extract", str(WS), "--out", str(tmp_path / "out"), "--collector", "probe"]
    assert main(argv) == EXIT_USAGE

    err = capsys.readouterr().err
    assert "probe" in err
    assert not (tmp_path / "out" / "atlas.json").exists()


def test_probe_starts_a_new_atlas_when_there_is_none(tmp_path: Path, fake_probe):
    from rosio.model import Atlas

    out = tmp_path / "out"
    assert main(["probe", str(WS), "--out", str(out), "--quiet"]) == EXIT_OK

    atlas = Atlas.loads((out / "atlas.json").read_text(encoding="utf-8"))
    assert atlas.layers["observed"].present
    assert atlas.layers["declared"].present is False
    assert atlas.capabilities == ["drift.findings", "edge.timing", "layer.observed"]


def test_probe_refuses_to_write_a_second_observed_layer(
    atlas_file: Path, tmp_path: Path, fake_probe, capsys
):
    """A fact slot is written by exactly one run, so a second probe of a changed system would
    be a refusal from the builder. It is a refusal here, where it can name the way out."""
    target = _copied(atlas_file, tmp_path)
    assert main(["probe", str(WS), "--atlas", str(target), "--quiet"]) == EXIT_OK

    assert main(["probe", str(WS), "--atlas", str(target)]) == EXIT_USAGE
    err = capsys.readouterr().err
    assert "already carries the observed layer" in err
    assert "--out" in err


def test_a_probe_that_reached_nothing_says_so_and_still_writes_the_run(
    tmp_path: Path, empty_probe, capsys
):
    from rosio.errors import EXIT_COLLECTOR
    from rosio.model import Atlas

    out = tmp_path / "out"
    assert main(["probe", str(WS), "--out", str(out), "--quiet"]) == EXIT_COLLECTOR
    assert "the observed layer is empty" in capsys.readouterr().err

    atlas = Atlas.loads((out / "atlas.json").read_text(encoding="utf-8"))
    assert atlas.layers["observed"].present is False
    assert atlas.layers["observed"].reason, "the collector said why, and it reached the file"


@pytest.mark.parametrize("stamps", [False, True])
def test_probe_hands_the_window_and_the_setup_files_to_the_child(
    tmp_path: Path, fake_probe, stamps
):
    """The three flags the verb exists for have to reach the process that does the work."""
    argv = [
        "probe",
        str(WS),
        "--out",
        str(tmp_path / "out"),
        "--duration",
        "4",
        "--settle",
        "1",
        "--ros-setup",
        "/opt/ros/humble/setup.bash",
        "--quiet",
    ]
    if stamps:
        argv.append("--stamps")
    assert main(argv) == EXIT_OK

    asked = fake_probe[-1]
    assert asked["stamps"] is stamps
    assert asked["window"] == 4.0
    assert asked["settle"] == 1.0
    assert asked["ros_setup"] == ("/opt/ros/humble/setup.bash",)
    assert asked["env"]["ROS_DOMAIN_ID"] == "77", "the domain is exported after the setup files"


def test_probe_refuses_a_profile_the_config_does_not_declare(tmp_path: Path, fake_probe, capsys):
    argv = ["probe", str(WS), "--out", str(tmp_path / "out"), "--profile", "nope"]
    assert main(argv) == EXIT_USAGE
    assert "unknown profile" in capsys.readouterr().err


def test_probe_json_keeps_the_wall_time_out_of_the_atlas(tmp_path: Path, fake_probe, capsys):
    argv = ["probe", str(WS), "--out", str(tmp_path / "out"), "--duration", "0", "--json"]
    assert main(argv) == EXIT_OK

    payload = json.loads(capsys.readouterr().out)
    assert payload["observed"] == {
        "present": True,
        "reason": "",
        "endpoints": 1,
        "frames": 0,
        "nodes": 1,
        "parameters": 0,
        "topics": 1,
    }
    assert payload["volatile"]["wall_seconds"] >= 0.0
    assert payload["out"].endswith("atlas.json")
    assert "wall_seconds" not in (tmp_path / "out" / "atlas.json").read_text(encoding="utf-8")


def test_unresolved_restricts_to_one_want(atlas_file: Path, capsys):
    assert main(["unresolved", "--atlas", str(atlas_file), "--want", "topic_name"]) == EXIT_OK
    printed = capsys.readouterr().out
    assert "1 unresolved values, want topic_name" in printed
    assert main(["unresolved", "--atlas", str(atlas_file), "--want", "frame_id"]) == EXIT_OK
    assert "0 unresolved values" in capsys.readouterr().out


# ---------------------------------------------------------------------------
# the agent verbs: serve, install, uninstall


@pytest.fixture
def served(monkeypatch):
    """Capture the Options `serve` builds without opening a transport."""
    from rosio.serve import server as srv

    seen = {}
    monkeypatch.setattr(cli, "_require_mcp", lambda: None)
    monkeypatch.setattr(srv, "run", lambda options: seen.update(options=options))
    return seen


def test_serve_hands_the_server_every_flag_it_takes(served, tmp_path: Path):
    atlas = tmp_path / "atlas.json"
    atlas.write_text("{}", encoding="utf-8")
    argv = [
        "serve",
        "--atlas",
        str(atlas),
        "--http",
        "--host",
        "0.0.0.0",
        "--port",
        "9001",
        "--api-key",
        "shh",
        "--path",
        "/tools",
        "--stateless",
        "--workspace",
        str(tmp_path),
    ]
    assert main(argv) == EXIT_OK

    options = served["options"]
    assert options.atlas == str(atlas)
    assert (options.http, options.host, options.port) == (True, "0.0.0.0", 9001)
    assert (options.api_key, options.path, options.stateless) == ("shh", "/tools", True)
    assert options.workspace == str(tmp_path)


def test_serve_defaults_to_stdio_and_to_the_atlas_under_out(served, tmp_path: Path):
    (tmp_path / "atlas.json").write_text("{}", encoding="utf-8")
    assert main(["serve", "--out", str(tmp_path)]) == EXIT_OK

    options = served["options"]
    assert options.atlas == str(tmp_path / "atlas.json")
    assert options.http is False
    assert (options.host, options.port, options.path) == ("127.0.0.1", 8787, "/mcp")


def test_serve_prints_nothing_on_stdout_over_stdio(served, tmp_path: Path, capsys):
    (tmp_path / "atlas.json").write_text("{}", encoding="utf-8")
    assert main(["serve", "--out", str(tmp_path)]) == EXIT_OK
    assert capsys.readouterr().out == ""


def test_serve_refuses_a_public_bind_without_a_key(served, tmp_path: Path, capsys):
    argv = ["serve", "--out", str(tmp_path), "--http", "--host", "10.0.0.4"]
    assert main(argv) == EXIT_USAGE
    assert "--api-key" in capsys.readouterr().err
    assert "options" not in served


def test_serve_takes_one_transport(capsys):
    with pytest.raises(SystemExit):
        build_parser().parse_args(["serve", "--stdio", "--http"])


def test_the_http_defaults_are_the_servers_own():
    from rosio.serve import server as srv

    assert (cli.SERVE_PORT, cli.SERVE_PATH) == (srv.DEFAULT_PORT, srv.DEFAULT_PATH)


def _pyproject() -> str:
    return (Path(__file__).resolve().parents[1] / "pyproject.toml").read_text(encoding="utf-8")


def test_the_mcp_range_is_written_once_and_pyproject_restates_it():
    """AGENTS section 12: an extra's range lives in the packaging file and in the module
    that raises, and nowhere else. The cli spelled a third copy, so a bump could leave the
    sentence a user reads naming a range pip does not resolve."""
    from rosio.serve import server as srv

    assert f'"{srv.MCP_EXTRA}"' in _pyproject()
    assert not hasattr(cli, "MCP_EXTRA")
    assert srv.MCP_EXTRA in srv.MCP_INSTALL


def test_the_release_bounds_the_cli_refuses_on_are_the_range_itself():
    from rosio.serve import server as srv

    assert f"mcp>={srv.MCP_MIN[0]}.{srv.MCP_MIN[1]},<{srv.MCP_MAX[0]}" == srv.MCP_EXTRA


def test_the_uvicorn_pin_the_http_transport_names_is_the_one_pyproject_pins():
    """The http transport imports uvicorn itself, so its message names a range too, and an
    untested range is a range that drifts."""
    from rosio.serve import server as srv

    assert f'"{srv.UVICORN_EXTRA}"' in _pyproject()
    assert srv.UVICORN_EXTRA in srv.UVICORN_MISSING


def test_the_cpp_range_pyproject_pins_is_the_range_the_collector_names():
    """The collector tells a user to install a range. Resolving a different one from the
    extra hands them a tree-sitter the collector then refuses."""
    from rosio.collect.cpp_code import TREE_SITTER_PIN

    text = _pyproject()
    wanted = ", ".join(f'"{pin}"' for pin in TREE_SITTER_PIN)
    assert f"cpp = [{wanted}]" in text


@pytest.mark.parametrize(
    "found, refused",
    [("1.13.0", True), ("1.14.0", False), ("2.0.0", False), ("3.0.0", True), ("weird", False)],
)
def test_an_mcp_outside_the_range_is_refused_by_name(found, refused, monkeypatch):
    monkeypatch.setattr("importlib.metadata.version", lambda _name: found)
    if not refused:
        cli._require_mcp()
        return
    with pytest.raises(UsageError) as exc:
        cli._require_mcp()
    assert found in str(exc.value)
    assert extra_command("mcp") in str(exc.value)


def test_a_missing_mcp_names_the_exact_install_command(monkeypatch):
    from importlib.metadata import PackageNotFoundError

    def absent(name):
        raise PackageNotFoundError(name)

    monkeypatch.setattr("importlib.metadata.version", absent)
    with pytest.raises(UsageError) as exc:
        cli._require_mcp()
    assert f"{extra_command('mcp')}, which pins mcp>=1.14,<3" in str(exc.value)


def test_install_writes_the_files_and_uninstall_takes_them_back(tmp_path: Path, capsys):
    root = tmp_path / "project"
    root.mkdir()
    assert main(["install", "codex", "--root", str(root)]) == EXIT_OK
    assert "created" in capsys.readouterr().out
    written = sorted(p.relative_to(root).as_posix() for p in root.rglob("*") if p.is_file())
    assert written == [".agents/skills/rosio/coding.md", ".agents/skills/rosio/review.md", "AGENTS.md"]

    assert main(["uninstall", "codex", "--root", str(root)]) == EXIT_OK
    assert "removed" in capsys.readouterr().out
    assert [p for p in root.rglob("*") if p.is_file()] == []


def test_install_registers_the_command_the_serve_verb_answers(tmp_path: Path, capsys):
    home = tmp_path / "home"
    home.mkdir()
    assert main(["install", "codex", "--scope", "user", "--home", str(home)]) == EXIT_OK
    config = (home / ".codex/config.toml").read_text(encoding="utf-8")
    assert f"command = {json.dumps(sys.executable)}" in config
    assert 'args = ["-m", "rosio", "serve"]' in config
    assert "serve" in commands()


def test_install_dry_run_writes_nothing(tmp_path: Path, capsys):
    root = tmp_path / "project"
    root.mkdir()
    assert main(["install", "codex", "--root", str(root), "--dry-run"]) == EXIT_OK
    assert "would create " in capsys.readouterr().out
    assert [p for p in root.rglob("*") if p.is_file()] == []


def test_install_with_no_platform_prints_the_table_and_reports_usage(capsys):
    assert main(["install"]) == EXIT_USAGE
    assert "claude-code" in capsys.readouterr().out
    assert main(["install", "--list"]) == EXIT_OK
    assert "from the tool's documentation" in capsys.readouterr().out


def test_install_names_the_choices_when_the_platform_is_unknown(capsys):
    assert main(["install", "emacs"]) == EXIT_USAGE
    assert "codex" in capsys.readouterr().err


def test_severity_words_take_colour_on_a_terminal_only(monkeypatch):
    import io

    from rosio.cli import _paint

    class Tty(io.StringIO):
        def isatty(self):
            return True

    text = "2 findings\n  error qos.incompatible  x\n  warn  param.shadowed  y\nerror rule: z"
    monkeypatch.delenv("NO_COLOR", raising=False)
    monkeypatch.delenv("FORCE_COLOR", raising=False)
    monkeypatch.setenv("TERM", "xterm")
    assert _paint(text, io.StringIO()) == text
    painted = _paint(text, Tty())
    assert "  \x1b[1;31merror\x1b[0m qos.incompatible" in painted
    assert "  \x1b[33mwarn\x1b[0m  param.shadowed" in painted
    assert painted.endswith("\x1b[1;31merror\x1b[0m rule: z")
    monkeypatch.setenv("NO_COLOR", "1")
    assert _paint(text, Tty()) == text


def test_the_configured_resolution_budget_reaches_the_rule(tmp_path: Path):
    shutil.copytree(WS / "src", tmp_path / "src")
    config = tmp_path / "rosio.yaml"
    budgets = {}
    for budget in (0.0, 1.0):
        config.write_text(f"roots: [src]\nresolution_budget: {budget}\n", encoding="utf-8")
        out = tmp_path / f"out{budget}"
        assert main(["extract", str(tmp_path), "--out", str(out), "--quiet"]) == EXIT_OK
        atlas = model.Atlas.loads((out / "atlas.json").read_text(encoding="utf-8"))
        assert atlas.x["drift"]["resolution_budget"] == budget
        budgets[budget] = [f for f in atlas.findings if f.rule == "resolution.below_budget"]
    assert not budgets[0.0]
    assert budgets[1.0]


def test_the_summary_prints_one_line_per_distinct_problem_with_names_a_reader_knows():
    raised = {
        "path": "nav2_bringup/launch/tb3_simulation_launch.py",
        "exc_type": "UnrunProcessOutput",
        "message": "OpaqueFunction(fn125ecd34): FileNotFoundError on a file the process "
        "`xacro -o @output world.sdf.xacro` writes when it runs",
    }
    refused = {"path": "@collector/params", "exc_type": "ContractError", "message": "m"}

    shown = cli._problem_lines([raised, raised, refused])

    assert shown == [
        (
            "nav2_bringup/launch/tb3_simulation_launch.py UnrunProcessOutput: an OpaqueFunction: "
            "FileNotFoundError on a file the process `xacro -o <its output file> "
            "world.sdf.xacro` writes when it runs",
            2,
        ),
        ("the params collector: rosio hit an internal error here, please report it: m", 1),
    ]


def test_a_problem_line_names_paths_and_never_the_tokens_the_atlas_stores():
    atlas = model.Atlas(
        generated={
            "roots": [{"kind": "src", "label": "src"}],
            "packages": {"demo_ten": {"src": "rcd/example_10"}},
        }
    )
    rows = [
        {
            "path": "demo_ten/launch/a.launch.py",
            "exc_type": "FileNotFoundError",
            "message": "No such file or directory: @src/rcd/example_10/ros2_control/r.xacro, "
            "'file:demo_one|urdf/rrbot.urdf.xacro' in "
            'File "@outside/lib/python3.12/site-packages/xacro/__init__.py".',
        }
    ]

    ((text, _),) = cli._problem_lines(rows, atlas)

    assert "@src" not in text and "@outside" not in text and "file:" not in text
    assert "demo_ten/ros2_control/r.xacro," in text
    assert "'demo_one/urdf/rrbot.urdf.xacro'" in text
    assert '<outside the workspace>/lib/python3.12/site-packages/xacro/__init__.py".' in text


def test_a_collector_refusal_is_one_sentence_and_not_its_class_and_prefix():
    """`CollectorError: declared.urdf: xacro refused <the same path>: error: ...`"""
    path = "file:demo_10|description/urdf/rrbot.urdf.xacro"
    rows = [
        {
            "file": path,
            "exc_type": "CollectorError",
            "message": f"declared.urdf: xacro refused {path}: error: No such file or directory: "
            "file:demo_10|ros2_control/rrbot.ros2_control.xacro",
        }
    ]

    ((text, _),) = cli._problem_lines(rows)

    assert text == (
        "demo_10/description/urdf/rrbot.urdf.xacro: xacro refused it: error: No such file or "
        "directory: demo_10/ros2_control/rrbot.ros2_control.xacro"
    )


def test_a_name_joined_to_a_substitution_reads_as_one_path():
    from rosio.query import readable

    text = "$(find-pkg-share hls_lfcd_lds_driver)@outside/hlds_laser.launch.py did not resolve"

    assert readable(text) == (
        "$(find-pkg-share hls_lfcd_lds_driver)/hlds_laser.launch.py did not resolve"
    )


def test_a_phase_on_a_terminal_is_written_once(monkeypatch):
    import io

    class Tty(io.StringIO):
        def isatty(self):
            return True

    stream = Tty()
    monkeypatch.setattr(sys, "stderr", stream)
    say = cli._progress("extract", quiet=False)
    say("launch_declared, profile 1 of 1: rover", step=True)
    say("cpp_code")

    assert stream.getvalue().count("profile 1 of 1: rover") == 1


def test_extract_in_a_directory_with_no_package_and_no_launch_file_refuses(tmp_path: Path, capsys):
    """An atlas of nothing written with exit zero read as a clean workspace."""
    from rosio.errors import EXIT_COLLECTOR

    assert main(["extract", str(tmp_path)]) == EXIT_COLLECTOR
    err = capsys.readouterr().err
    assert f"no package.xml and no launch file under {tmp_path.absolute()}" in err
    assert not (tmp_path / "rosio-out").exists()


def test_extract_names_the_rendered_files_an_earlier_atlas_left_behind(tmp_path: Path, capsys):
    shutil.copytree(WS / "src", tmp_path / "src")
    out = tmp_path / "out"
    assert main(["extract", str(tmp_path), "--out", str(out)]) == EXIT_OK
    assert main(["render", "--atlas", str(out / "atlas.json"), "--format", "md"]) == EXIT_OK
    capsys.readouterr()
    assert main(["extract", str(tmp_path), "--out", str(out)]) == EXIT_OK
    assert "an earlier atlas" not in capsys.readouterr().out
    (out / "DRIFT.md").write_text("Rendered from atlas sha256:0000000000000000", encoding="utf-8")
    assert main(["extract", str(tmp_path), "--out", str(out)]) == EXIT_OK
    assert "DRIFT.md describes an earlier atlas, rosio render" in capsys.readouterr().out


def test_a_reader_that_closes_the_pipe_early_gets_no_traceback(tmp_path):
    """`rosio drift | head` closes stdout before rosio is done writing."""
    import subprocess

    script = (
        "import sys\n"
        "from rosio import cli\n"
        "cli.HANDLERS['version'] = lambda args: [print('x' * 80) for _ in range(100000)] and 0\n"
        "sys.exit(cli.main(['version']))\n"
    )
    done = subprocess.run(
        f'{sys.executable} -c "{script}" | head -n 1',
        shell=True,
        capture_output=True,
        text=True,
        check=False,
    )
    assert "Traceback" not in done.stderr
    assert "BrokenPipeError" not in done.stderr


def test_every_argument_of_every_verb_and_action_says_what_it_is():
    import argparse

    def walk(parser, name):
        if not parser.description:
            yield f"{name}: no description"
        for action in parser._actions:
            if isinstance(action, argparse._SubParsersAction):
                for child, sub in action.choices.items():
                    yield from walk(sub, f"{name} {child}")
            elif not action.help:
                yield f"{name} {'/'.join(action.option_strings) or action.metavar}"

    assert list(walk(cli.build_parser(), "rosio")) == []


def test_output_still_buffered_when_the_reader_leaves_ends_quietly():
    """Short output sits in the buffer until exit, so the pipe breaks at the final flush."""
    import subprocess

    script = (
        "import sys, time\n"
        "from rosio import cli\n"
        "def answer(args):\n"
        "    print('first'); print('x' * 2000)\n"
        "    time.sleep(0.5)\n"
        "    return 0\n"
        "cli.HANDLERS['version'] = answer\n"
        "sys.exit(cli.main(['version']))\n"
    )
    child = subprocess.Popen(
        [sys.executable, "-c", script], stdout=subprocess.PIPE, stderr=subprocess.PIPE
    )
    child.stdout.close()
    err = child.stderr.read().decode()
    assert child.wait(timeout=30) == 141
    assert "Traceback" not in err and "BrokenPipeError" not in err


def test_an_extra_is_installed_the_way_rosio_was(monkeypatch, tmp_path):
    """A checkout install in a uv environment has no pip and no PyPI release to name."""
    import importlib.metadata
    import json
    import sys

    class Found:
        def __init__(self, record):
            self.record = record

        def read_text(self, _name):
            return self.record

    checkout = json.dumps({"url": "file:///home/u/rosio", "dir_info": {}})
    monkeypatch.setattr(importlib.metadata, "distribution", lambda _name: Found(checkout))
    monkeypatch.setattr(sys, "prefix", str(tmp_path))
    assert extra_command("cpp") == "pip install '/home/u/rosio[cpp]'"
    (tmp_path / "pyvenv.cfg").write_text("home = /usr/bin\nuv = 0.9.0\n", encoding="utf-8")
    assert extra_command("cpp") == "uv pip install '/home/u/rosio[cpp]'"
    monkeypatch.setattr(importlib.metadata, "distribution", lambda _name: Found(None))
    assert extra_command("cpp") == "uv pip install 'rosio[cpp]'"


def test_init_comes_first_in_the_help_the_epilog_says_to_start_with(capsys):
    text = cli.build_parser().format_help()
    listed = text[text.index("command") :]
    assert listed.index("    init") < listed.index("    extract")


def test_bagmutate_says_what_each_flag_takes(capsys):
    from rosio.stamps import mutate

    with pytest.raises(SystemExit):
        mutate.main(["--help"])
    out = capsys.readouterr().out
    assert "the fault to put in" in out
    assert "where in the recording the fault starts" in out


def test_a_stored_install_line_is_shown_as_this_machine_runs_it():
    """The atlas keeps the machine independent cpp line, and the terminal said pip inside a
    uv environment while the mcp hint beside it said uv pip with the checkout path."""
    from rosio import extra_line, here

    stored = f"Install the extra: {extra_line('cpp')}, which pins tree-sitter"
    assert here(stored) == f"Install the extra: {extra_command('cpp')}, which pins tree-sitter"
    assert here("nothing to replace") == "nothing to replace"
