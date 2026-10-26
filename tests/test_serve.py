"""The agent surface: what a model with a small context window gets back, and what it is
refused. Every assertion here is about one of three things: the answer is correct, the
answer is small, or the answer says honestly that it cannot be given."""

from __future__ import annotations

import asyncio
import json
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from rosio import ids, model
from rosio.errors import ContractError, UsageError
from rosio.model import Candidate, Finding, Source, Unresolved
from rosio.serve import server, tools

REAL = "profile:nx_navigation"
SIM = "profile:nx_sim"

PARAMS_FILE = ids.build_file("nx_bringup", "config/nav2_params.yaml")
OVERLAY_FILE = ids.build_file("nx_bringup", "config/nav2_overrides.yaml")
LAUNCHER_FILE = ids.build_file(ids.REPO_PKG, "dx_app/dx_launcher/profiles.py")
CONTROLLER = ids.build_node("/controller_server")
MONITOR = ids.build_node("/collision_monitor")
CMD_VEL = ids.build_topic("/cmd_vel")
PLUGIN = ids.build_param(CONTROLLER, "FollowPath.plugin")
UNNAMED = ids.build_node_synthetic("nav2_controller", "controller_server", "/", "75ed3551")

WINNER = "nx_controller/DelayAwareController"
LOSER = "nx_planner/DockingController"


def build() -> model.Atlas:
    """A small atlas carrying every shape the tools have to render: two profiles, a
    candidate chain with a shadowed line, an endpoint pair, a blind spot and a finding."""
    builder = model.AtlasBuilder(capabilities=[tools.DRIFT_CAPABILITY])
    builder.declare_layer("declared", present=True)
    builder.declare_layer("coded", present=True)
    builder.add_profile("nx_navigation")
    builder.add_profile("nx_sim")
    for file_id in (PARAMS_FILE, OVERLAY_FILE, LAUNCHER_FILE):
        builder.node(file_id)
    symbol = ids.build_sym(ids.REPO_PKG, "dx_app/dx_launcher/profiles.py", "nav2_command")
    builder.node(symbol)

    builder.fact(
        CONTROLLER,
        "declared",
        profile=REAL,
        values={"pkg": "nav2_controller", "executable": "controller_server"},
        evidence=[Source(file=PARAMS_FILE, locator="controller_server", line_hint=140)],
    )
    builder.node(CONTROLLER, name_source="literal")
    builder.node(CMD_VEL)
    builder.edge(
        "publishes",
        CONTROLLER,
        CMD_VEL,
        key=ids.edge_key(symbol=symbol),
        layer="coded",
        profile=REAL,
        values={"qos_depth": 10},
    )
    builder.fact(MONITOR, "declared", profile=REAL, values={"pkg": "nav2_collision_monitor"})
    builder.node(MONITOR, name_source="literal")
    builder.edge(
        "subscribes",
        MONITOR,
        CMD_VEL,
        key=ids.edge_key(symbol=symbol),
        layer="coded",
        profile=REAL,
    )
    builder.node(UNNAMED, name_source="runtime_randomised")
    builder.edge(
        "publishes",
        UNNAMED,
        CMD_VEL,
        key=ids.edge_key(symbol=symbol, profile=SIM),
        layer="coded",
        profile=SIM,
    )
    builder.set_param(
        PLUGIN,
        state="effective",
        candidates=[
            Candidate(
                value=WINNER,
                value_type="str",
                mechanism="yaml_overlay_merge",
                source=Source(
                    file=OVERLAY_FILE,
                    locator="controller_server.ros__parameters.FollowPath.plugin",
                    line_hint=12,
                ),
                rank=0,
                wins=True,
                profile=REAL,
                applied_by=symbol,
            ),
            Candidate(
                value=LOSER,
                value_type="str",
                mechanism="yaml_base",
                source=Source(
                    file=PARAMS_FILE,
                    locator="controller_server.ros__parameters.FollowPath.plugin",
                    line_hint=149,
                ),
                rank=1,
                wins=False,
                profile=REAL,
                shadow_kind="value_changed",
                shadowed_by=0,
            ),
        ],
    )
    builder.edge("has_param", CONTROLLER, PLUGIN)
    builder.unresolved(
        Unresolved(
            want="topic_name",
            owner=symbol,
            expr="self.get_parameter('speed_limit_topic')",
            reason="parameter with no default",
            resolvable_by=["observed"],
            evidence=[Source(file=LAUNCHER_FILE, line_hint=430)],
        )
    )
    builder.finding(
        Finding(
            rule="param.shadowed",
            severity="warn",
            subject=PLUGIN,
            title="FollowPath.plugin is set twice and one line does nothing",
            detail=f"the overlay states {WINNER} and the base file states {LOSER}",
            fix="delete the base line",
            say=(
                f"FollowPath.plugin on /controller_server is {WINNER} from "
                "nx_bringup/config/nav2_overrides.yaml:12"
            ),
            profiles=[REAL],
            where=[Source(file=PARAMS_FILE, locator="controller_server", line_hint=149)],
        )
    )
    return builder.build()


@pytest.fixture
def atlas() -> model.Atlas:
    return build()


@pytest.fixture
def atlas_file(tmp_path: Path, atlas: model.Atlas) -> Path:
    target = tmp_path / "atlas.json"
    target.write_bytes(atlas.to_bytes())
    return target


def every_answer(atlas: model.Atlas) -> list[str]:
    return [
        tools.overview(atlas),
        tools.param_chain(atlas, "/controller_server", "FollowPath.plugin"),
        tools.drift(atlas),
        tools.explain(atlas, "/controller_server"),
        tools.query(atlas, "which nodes publish cmd_vel"),
        tools.path(atlas, "/controller_server", "/cmd_vel"),
        tools.neighbours(atlas, "/cmd_vel"),
        tools.graph(atlas),
        tools.graph(atlas, focus="/cmd_vel", depth=2),
        tools.unresolved(atlas),
    ]


# ---------------------------------------------------------------------------
# loading


def test_a_missing_atlas_names_the_command_that_writes_one(tmp_path: Path):
    with pytest.raises(UsageError) as caught:
        tools.load(tmp_path / "nothing.json")

    assert "rosio extract" in str(caught.value)


def test_a_directory_is_accepted_and_the_atlas_is_read_once(atlas_file: Path):
    source = tools.AtlasSource(atlas_file.parent)

    assert not source.loaded
    first = source.atlas()
    atlas_file.unlink()

    assert source.atlas() is first


def test_a_newer_schema_major_is_refused_with_a_way_out(tmp_path: Path, atlas: model.Atlas):
    payload = atlas.to_json()
    payload["schema_version"] = "9.0"
    target = tmp_path / "atlas.json"
    target.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ContractError) as caught:
        tools.load(target)

    assert "9.0" in str(caught.value)
    assert "extract this workspace again" in str(caught.value)


def test_a_file_that_is_not_json_is_a_usage_error(tmp_path: Path):
    target = tmp_path / "atlas.json"
    target.write_text("not json at all", encoding="utf-8")

    with pytest.raises(UsageError):
        tools.load(target)


# ---------------------------------------------------------------------------
# what every answer has to satisfy


def test_every_answer_fits_a_small_context_window(atlas: model.Atlas):
    for answer in every_answer(atlas):
        assert 0 < len(answer.encode("utf-8")) <= tools.MAX_BYTES


def test_no_answer_puts_a_synthetic_id_in_prose(atlas: model.Atlas):
    for answer in [*every_answer(atlas), tools.explain(atlas, UNNAMED)]:
        assert ":~" not in answer
        assert "%7C" not in answer


def test_every_answer_is_the_same_twice(atlas: model.Atlas):
    assert every_answer(atlas) == every_answer(atlas)


def test_a_capped_answer_says_it_was_capped_and_how_to_narrow():
    lines = [f"line {index} " + "x" * 200 for index in range(200)]

    text = tools._fit(lines, "narrow with rule=<rule>")

    assert len(text.encode("utf-8")) <= tools.MAX_BYTES
    assert "truncated after" in text
    assert "narrow with rule=<rule>" in text


# ---------------------------------------------------------------------------
# overview


def test_overview_states_profiles_layers_counts_coverage_and_the_worst_finding(atlas):
    text = tools.overview(atlas)

    assert "nx_navigation" in text and "nx_sim" in text
    assert "declared" in text and "coded" in text
    assert "entities" in text and "param" in text
    assert "unresolved" in text
    assert "param.shadowed" in text
    assert "param_chain" in text


def test_overview_says_when_the_rules_never_ran():
    plain = model.AtlasBuilder().build()

    text = tools.overview(plain)

    assert "never ran" in text


# ---------------------------------------------------------------------------
# param_chain, the highest value question


def test_param_chain_answers_value_type_winner_and_shadow_in_one_call(atlas):
    text = tools.param_chain(atlas, "/controller_server", "FollowPath.plugin")

    assert WINNER in text
    assert "(str)" in text
    assert "nav2_overrides.yaml:12" in text
    assert LOSER in text
    assert "nav2_params.yaml:149" in text
    assert "shadowed by rank 0, value_changed" in text
    assert "yaml_overlay_merge" in text


def test_param_chain_narrows_to_one_profile(atlas):
    text = tools.param_chain(atlas, "/controller_server", "FollowPath.plugin", profile="nx_sim")

    assert "not set in nx_sim" in text
    assert WINNER not in text


def test_param_chain_refuses_a_profile_that_does_not_exist(atlas):
    with pytest.raises(UsageError) as caught:
        tools.param_chain(atlas, "/controller_server", "FollowPath.plugin", profile="mapping")

    assert "nx_navigation" in str(caught.value)


def test_param_chain_says_so_when_the_parameter_is_not_there(atlas):
    text = tools.param_chain(atlas, "/controller_server", "no_such_parameter")

    assert "nothing in this atlas" in text
    assert WINNER not in text


def test_param_chain_needs_a_node(atlas):
    with pytest.raises(UsageError):
        tools.param_chain(atlas, "  ")


# ---------------------------------------------------------------------------
# drift


def test_drift_gives_the_sentence_to_quote_and_the_line_to_edit(atlas):
    text = tools.drift(atlas)

    assert "param.shadowed" in text
    assert f"say: FollowPath.plugin on /controller_server is {WINNER}" in text
    assert "edit: nx_bringup/config/nav2_params.yaml:149" in text


def test_drift_filters_by_severity_rule_and_profile(atlas):
    assert "param.shadowed" not in tools.drift(atlas, severity="error")
    assert "param.shadowed" in tools.drift(atlas, severity="warn")
    assert "param.shadowed" in tools.drift(atlas, profile="nx_navigation")
    assert "param.shadowed" not in tools.drift(atlas, profile="nx_sim")


def test_drift_refuses_a_severity_that_is_not_one(atlas):
    with pytest.raises(UsageError) as caught:
        tools.drift(atlas, severity="critical")

    assert "error, warn, info" in str(caught.value)


def test_drift_says_which_rules_fired_when_the_rule_asked_for_did_not(atlas):
    text = tools.drift(atlas, rule="qos.differ")

    assert "no finding of rule qos.differ" in text
    assert "param.shadowed 1" in text


def test_drift_says_when_the_rules_never_ran_instead_of_reporting_a_clean_stack():
    plain = model.AtlasBuilder().build()

    text = tools.drift(plain)

    assert "never ran" in text
    assert "extract" in text


# ---------------------------------------------------------------------------
# explain


def test_explain_states_layers_facts_provenance_and_the_next_call(atlas):
    text = tools.explain(atlas, "/controller_server")

    assert "node /controller_server" in text
    assert "layers declared" in text
    assert "nav2_params.yaml:140" in text
    assert "publishes /cmd_vel" in text
    assert "neighbours" in text


def test_explain_of_a_parameter_points_at_the_chain(atlas):
    text = tools.explain(atlas, "FollowPath.plugin")

    assert "state effective" in text
    assert "param_chain" in text


def test_explain_renders_a_finding_when_the_selector_is_one(atlas):
    handle = atlas.findings[0].id

    text = tools.explain(atlas, handle)

    assert "param.shadowed" in text
    assert "say:" in text


def test_explain_lists_the_candidates_when_a_name_is_ambiguous(atlas):
    text = tools.explain(atlas, "controller")

    assert "names 3 entities, ask again with one of these" in text
    assert "/controller_server" in text


def test_explain_says_so_when_nothing_carries_the_name(atlas):
    text = tools.explain(atlas, "/nothing_like_this")

    assert "nothing in this atlas" in text


# ---------------------------------------------------------------------------
# query


def test_query_answers_a_plain_question_with_a_scoped_subgraph(atlas):
    text = tools.query(atlas, "which nodes publish cmd_vel")

    assert "/cmd_vel" in text
    assert "publishes" in text


def test_query_says_when_it_did_not_understand(atlas):
    text = tools.query(atlas, "is it good")

    assert "not understood" in text or "nothing matched" in text


def test_query_admits_a_bare_name_match(atlas):
    text = tools.query(atlas, "controller_server")

    assert "matched the words by name only" in text


def test_query_needs_a_question(atlas):
    with pytest.raises(UsageError):
        tools.query(atlas, "")


# ---------------------------------------------------------------------------
# path


def test_path_states_every_hop_and_its_kind(atlas):
    text = tools.path(atlas, "/controller_server", "/collision_monitor")

    assert "hops" in text
    assert "publishes" in text
    assert "subscribes" in text


def test_path_says_when_there_is_none(atlas):
    text = tools.path(atlas, "/controller_server", "nx_bringup/config/nav2_params.yaml")

    assert "no path" in text


# ---------------------------------------------------------------------------
# neighbours


def test_neighbours_counts_both_directions_without_scanning(atlas):
    text = tools.neighbours(atlas, "/cmd_vel")

    assert "publishes 2" in text
    assert "subscribes 1" in text
    assert "/controller_server" in text


def test_neighbours_narrows_by_kind_and_direction(atlas):
    text = tools.neighbours(atlas, "/cmd_vel", kind="subscribes", direction="in")

    assert "/collision_monitor" in text
    assert "publishes" not in text


def test_neighbours_says_which_kinds_exist_when_the_kind_asked_for_does_not(atlas):
    text = tools.neighbours(atlas, "/cmd_vel", kind="calls")

    assert "no calls edge touches it" in text
    assert "publishes" in text


def test_neighbours_refuses_a_direction_that_is_not_one(atlas):
    with pytest.raises(UsageError):
        tools.neighbours(atlas, "/cmd_vel", direction="sideways")


# ---------------------------------------------------------------------------
# unresolved


def test_unresolved_states_the_blind_spots_with_their_site(atlas):
    text = tools.unresolved(atlas)

    assert "topic_name" in text
    assert "speed_limit_topic" in text
    assert "profiles.py:430" in text
    assert "resolvable by observed" in text


def test_unresolved_narrows_by_want_and_by_owner(atlas):
    owner = ids.build_sym(ids.REPO_PKG, "dx_app/dx_launcher/profiles.py", "nav2_command")

    assert "speed_limit_topic" in tools.unresolved(atlas, want="topic_name")
    assert "speed_limit_topic" in tools.unresolved(atlas, owner=owner)
    assert "nothing unresolved of want frame_id" in tools.unresolved(atlas, want="frame_id")


# ---------------------------------------------------------------------------
# the tool set as an interface


def test_the_tool_set_is_twelve_named_tools_with_a_description_each(atlas_file: Path):
    functions = server.bindings(tools.AtlasSource(atlas_file))

    names = [function.__name__ for function in functions]
    assert names == [
        "overview",
        "param_chain",
        "drift",
        "explain",
        "query",
        "path",
        "neighbours",
        "graph",
        "unresolved",
        "stamps",
        "transform",
        "migrate",
    ]
    for function in functions:
        assert function.__doc__ and len(function.__doc__.split()) >= 8


def test_every_tool_answers_from_the_bound_atlas(atlas_file: Path):
    functions = {item.__name__: item for item in server.bindings(tools.AtlasSource(atlas_file))}

    assert "nx_navigation" in functions["overview"]()
    assert WINNER in functions["param_chain"]("/controller_server", "FollowPath.plugin")
    assert "param.shadowed" in functions["drift"]()
    assert "node /controller_server" in functions["explain"]("/controller_server")
    assert "/cmd_vel" in functions["query"]("which topic does controller_server publish")
    assert "hops" in functions["path"]("/controller_server", "/collision_monitor")
    assert "publishes" in functions["neighbours"]("/cmd_vel")
    assert "published by /controller_server" in functions["graph"]()
    assert "topic_name" in functions["unresolved"]()


def test_only_the_first_call_reads_the_file(atlas_file: Path):
    functions = {item.__name__: item for item in server.bindings(tools.AtlasSource(atlas_file))}
    functions["overview"]()
    atlas_file.unlink()

    assert "param.shadowed" in functions["drift"]()


# ---------------------------------------------------------------------------
# transports and the refusal


def test_stdio_is_the_default_and_needs_no_key():
    assert server.Options(atlas="atlas.json").check().http is False


def test_an_open_bind_without_a_key_is_refused_and_not_warned_about():
    with pytest.raises(UsageError) as caught:
        server.Options(atlas="atlas.json", http=True, host="0.0.0.0").check()

    message = str(caught.value)
    assert "--api-key" in message
    assert "127.0.0.1" in message


def test_an_open_bind_with_a_key_is_allowed():
    options = server.Options(
        atlas="atlas.json", http=True, host="10.0.0.4", api_key="secret"
    ).check()

    assert options.host == "10.0.0.4"


def test_loopback_needs_no_key_and_a_bad_path_and_port_are_refused():
    server.Options(atlas="atlas.json", http=True, host="127.0.0.1").check()
    with pytest.raises(UsageError):
        server.Options(atlas="atlas.json", http=True, path="mcp").check()
    with pytest.raises(UsageError):
        server.Options(atlas="atlas.json", http=True, port=0).check()


def test_an_atlas_is_required():
    with pytest.raises(UsageError):
        server.Options(atlas="").check()


def test_the_command_line_maps_to_the_options(atlas_file: Path):
    from rosio.serve.__main__ import options

    chosen = options(
        [
            "--atlas",
            str(atlas_file),
            "--http",
            "--host",
            "0.0.0.0",
            "--port",
            "9000",
            "--api-key",
            "secret",
            "--path",
            "/tools",
            "--stateless",
        ]
    )

    assert chosen.http and chosen.stateless
    assert (chosen.host, chosen.port, chosen.path, chosen.api_key) == (
        "0.0.0.0",
        9000,
        "/tools",
        "secret",
    )


def test_the_command_line_refuses_an_open_bind_with_no_key(capsys):
    from rosio.serve.__main__ import main

    code = main(["--atlas", "atlas.json", "--http", "--host", "0.0.0.0"])

    assert code == 2
    assert "--api-key" in capsys.readouterr().err


# ---------------------------------------------------------------------------
# the key check in front of the http app


def drive(app, headers: list[tuple[bytes, bytes]], kind: str = "http") -> list[dict]:
    sent: list[dict] = []

    async def send(message: dict) -> None:
        sent.append(message)

    async def receive() -> dict:
        return {"type": "http.request", "body": b"", "more_body": False}

    asyncio.run(app({"type": kind, "headers": headers}, receive, send))
    return sent


async def served(scope, receive, send) -> None:
    await send({"type": "http.response.start", "status": 200, "headers": []})
    await send({"type": "http.response.body", "body": b"served"})


def test_without_a_key_the_app_is_served_unguarded():
    assert server.guarded(served, "") is served


def test_a_request_with_no_key_is_refused():
    sent = drive(server.guarded(served, "secret"), [])

    assert sent[0]["status"] == 401
    assert b"x-api-key" in sent[1]["body"]


def test_a_bearer_token_and_an_api_key_header_both_pass():
    for headers in (
        [(b"authorization", b"Bearer secret")],
        [(b"x-api-key", b"secret")],
        [(b"Authorization", b"bearer secret")],
    ):
        assert drive(server.guarded(served, "secret"), headers)[0]["status"] == 200


def test_a_wrong_key_is_refused():
    sent = drive(server.guarded(served, "secret"), [(b"x-api-key", b"guess")])

    assert sent[0]["status"] == 401


def test_a_lifespan_message_is_passed_through_untouched():
    seen: list[str] = []

    async def app(scope, receive, send) -> None:
        seen.append(scope["type"])

    drive(server.guarded(app, "secret"), [], kind="lifespan")

    assert seen == ["lifespan"]


# ---------------------------------------------------------------------------
# the optional extra


def test_the_missing_mcp_package_names_the_install_command(monkeypatch):
    monkeypatch.setitem(sys.modules, "mcp.server.mcpserver", None)
    monkeypatch.setitem(sys.modules, "mcp.server.fastmcp", None)

    with pytest.raises(UsageError) as caught:
        server._server_class()

    assert "pip install 'rosio[mcp]'" in str(caught.value)


def test_the_server_registers_every_tool_when_mcp_is_installed(atlas_file: Path):
    pytest.importorskip("mcp")

    built = server.build(tools.AtlasSource(atlas_file))
    listed = asyncio.run(built.list_tools())

    assert {tool.name for tool in listed} == {
        "overview",
        "param_chain",
        "drift",
        "explain",
        "query",
        "path",
        "neighbours",
        "graph",
        "unresolved",
        "stamps",
        "transform",
        "migrate",
    }


def test_the_http_app_carries_the_path_the_stateless_mode_and_the_key(atlas_file: Path):
    pytest.importorskip("mcp")

    built = server.build(tools.AtlasSource(atlas_file))
    options = server.Options(
        atlas=str(atlas_file),
        http=True,
        host="127.0.0.1",
        api_key="secret",
        path="/tools",
        stateless=True,
    ).check()

    app = server.http_app(built, options)

    assert app is not served
    assert callable(app)


# ---------------------------------------------------------------------------
# a file the server is willing to answer from


def test_a_layer_present_written_as_the_string_false_is_refused(tmp_path: Path, atlas: model.Atlas):
    """The wrong answer this replaces: `bool("false")` is True, so a hand edited file saying
    a layer is absent produced an overview that confidently listed it as present."""
    payload = atlas.to_json()
    payload["layers"]["coded"]["present"] = "false"
    target = tmp_path / "atlas.json"
    target.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ContractError) as caught:
        tools.load(target)

    assert "layers.coded.present" in str(caught.value)
    assert "not a boolean" in str(caught.value)


def test_a_truncated_atlas_is_refused_instead_of_answered(tmp_path: Path, atlas: model.Atlas):
    payload = atlas.to_json()
    del payload["entities"][CMD_VEL]
    target = tmp_path / "atlas.json"
    target.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ContractError) as caught:
        tools.load(target)

    assert "does not satisfy the atlas schema" in str(caught.value)
    assert "rosio extract" in str(caught.value)


def test_the_refusal_of_a_broken_file_is_capped(tmp_path: Path, atlas: model.Atlas):
    payload = atlas.to_json()
    payload["entities"] = {}
    target = tmp_path / "atlas.json"
    target.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ContractError) as caught:
        tools.load(target)

    assert len(str(caught.value).encode("utf-8")) <= tools.MAX_BYTES


def test_a_newer_minor_is_still_answered_from(tmp_path: Path, atlas: model.Atlas):
    """Section 9a: a reader tolerates the keys and the enum members a newer minor added, so
    the stricter check this build runs is not allowed to lock that reader out."""
    payload = atlas.to_json()
    payload["schema_version"] = f"{payload['schema_version'].split('.')[0]}.9"
    payload["entities"][CONTROLLER]["invented_by_a_later_build"] = True
    target = tmp_path / "atlas.json"
    target.write_text(json.dumps(payload), encoding="utf-8")

    loaded = tools.load(target)

    assert loaded.entities[CONTROLLER].extra["invented_by_a_later_build"] is True


# ---------------------------------------------------------------------------
# a refusal is an answer, and every answer is capped


def crowded(count: int = 1000) -> model.Atlas:
    builder = model.AtlasBuilder(capabilities=[tools.DRIFT_CAPABILITY])
    builder.declare_layer("declared", present=True)
    for index in range(count):
        builder.add_profile(f"nx_profile_{index:04d}")
    return builder.build()


def test_an_unknown_profile_is_refused_inside_the_answer_cap():
    """The wrong answer this replaces: the refusal listed every profile in the atlas, which
    was seventeen kilobytes in a surface that promises four."""
    with pytest.raises(UsageError) as caught:
        tools.param_chain(crowded(), "/controller_server", "rate", profile="nx_missing")

    message = str(caught.value)
    assert len(message.encode("utf-8")) <= tools.MAX_BYTES
    assert "nx_profile_0000" in message
    assert "more profiles" in message


def test_the_overview_of_a_crowded_atlas_still_says_something():
    answer = tools.overview(crowded())

    assert len(answer.encode("utf-8")) <= tools.MAX_BYTES
    assert "profiles (1000): nx_profile_0000" in answer
    assert "990 more profiles" in answer


# ---------------------------------------------------------------------------
# graph

ROVER = "profile:rover"
ROVER_SIM = "profile:rover_sim"
FILTER = ids.build_node("/scan_filter")
STOP = ids.build_node("/safety_stop")
SIM_NODE = ids.build_node_synthetic("sim_pkg", "sim_node", "/", "75ed3551")
FILTERED = ids.build_topic("/scan_filtered")
TWIST = ids.build_topic("/cmd_vel")
DIAGNOSTICS = ids.build_topic("/diagnostics")


def rover() -> model.Atlas:
    """The editor graph's own fixture: two profiles, a QoS mismatch with a finding, a
    library endpoint and an unnamed node in the second profile."""
    builder = model.AtlasBuilder(capabilities=[tools.DRIFT_CAPABILITY])
    builder.declare_layer("coded", present=True)
    builder.add_profile("rover")
    builder.add_profile("rover_sim")
    for node_id, profile in ((FILTER, ROVER), (STOP, ROVER), (SIM_NODE, ROVER_SIM)):
        builder.fact(node_id, "coded", profile=profile, values={"seen": True})
        builder.node(node_id, name_source="literal")
    for topic_id, profiles in (
        (FILTERED, (ROVER,)),
        (TWIST, (ROVER, ROVER_SIM)),
        (DIAGNOSTICS, (ROVER,)),
    ):
        for profile in profiles:
            builder.fact(topic_id, "coded", profile=profile, values={"seen": True})
    laser = ids.build_type("sensor_msgs", "msg", "LaserScan")
    builder.node(laser)
    builder.edge("uses_type", FILTERED, laser, layer="coded", profile=ROVER)

    def link(kind, src, dst, profile=ROVER, **values):
        builder.edge(
            kind,
            src,
            dst,
            key=ids.edge_key(profile=profile),
            layer="coded",
            profile=profile,
            values=values,
        )

    link("publishes", FILTER, FILTERED, qos={"preset": "sensor_data"})
    link("subscribes", STOP, FILTERED, qos={"reliability": "reliable", "depth": 10})
    link("publishes", STOP, TWIST)
    link("publishes", STOP, DIAGNOSTICS, via_known_wrapper=True)
    link("subscribes", SIM_NODE, TWIST, profile=ROVER_SIM)
    builder.finding(
        Finding(
            rule="qos.incompatible",
            severity="error",
            subject=FILTERED,
            title="a reliable subscription to a best effort publisher",
            say="/safety_stop asks /scan_filtered for reliable and /scan_filter offers best effort",
            profiles=[ROVER],
        )
    )
    return builder.build()


def test_graph_lists_each_channel_with_its_ends_and_the_stated_qos():
    answer = tools.graph(rover(), profile="rover")

    assert (
        answer.splitlines()[0]
        == "2 nodes, 2 topics, 0 services, 0 actions, 3 endpoints in profile rover"
    )
    assert answer.splitlines()[1] == (
        "/scan_filtered (sensor_msgs/msg/LaserScan) [qos.incompatible]: "
        "published by /scan_filter [sensor_data preset]; "
        "subscribed by /safety_stop [reliable, depth 10]"
    )
    assert "/cmd_vel: published by /safety_stop; no subscriber" in answer


def test_graph_keeps_to_one_profile_and_never_names_a_synthetic_id():
    atlas = rover()
    sim = tools.graph(atlas, profile="rover_sim")
    merged = tools.graph(atlas)

    assert "/scan_filtered" not in sim
    assert "/cmd_vel: no publisher; subscribed by sim_pkg/sim_node (unnamed by launch)" in sim
    assert "subscribed by sim_pkg/sim_node (unnamed by launch)" in merged
    assert "narrow with profile=<name>" in merged
    for answer in (sim, merged, tools.graph(atlas, focus="/cmd_vel", depth=3)):
        assert ":~" not in answer
        assert "%7C" not in answer


def test_graph_hides_library_endpoints_unless_asked():
    atlas = rover()
    hidden = tools.graph(atlas, profile="rover")
    shown = tools.graph(atlas, profile="rover", wrappers=True)

    assert "/diagnostics" not in hidden
    assert "1 library endpoint left out" in hidden
    assert "/diagnostics: published by /safety_stop; no subscriber" in shown
    assert "left out" not in shown


def test_graph_follows_a_focus_for_depth_hops():
    atlas = rover()
    one = tools.graph(atlas, profile="rover", focus="/scan_filter", depth=1)
    two = tools.graph(atlas, profile="rover", focus="/scan_filter")

    assert one.splitlines()[0].startswith("around node /scan_filter within 1 hop: 1 node, 1 topic")
    assert "published by /scan_filter [sensor_data preset]; no subscriber" in one
    assert "1 endpoint beyond 1 hop" in one
    assert "subscribed by /safety_stop [reliable, depth 10]" in two
    assert "/cmd_vel" not in two


def test_graph_stops_walking_when_nothing_is_left_to_reach():
    atlas = rover()
    whole = tools.graph(atlas, profile="rover", focus="/scan_filter", depth=50)

    assert (
        tools.graph(atlas, profile="rover", focus="/scan_filter", depth=10**9).splitlines()[1:]
        == whole.splitlines()[1:]
    )


def test_graph_answers_an_unknown_or_wrong_focus_like_the_other_tools():
    atlas = rover()

    assert "nothing in this atlas is named '/nobody'" in tools.graph(atlas, focus="/nobody")
    assert "is not a node, topic, service or action" in tools.graph(
        atlas, focus="type:sensor_msgs|msg|LaserScan"
    )
    assert "has no endpoint in profile rover_sim" in tools.graph(
        atlas, profile="rover_sim", focus="/scan_filter"
    )


def test_graph_is_capped_and_the_same_twice():
    atlas = rover()

    assert tools.graph(atlas) == tools.graph(atlas)
    capped = tools.graph(atlas, profile="rover", limit=1, wrappers=True)
    assert capped.splitlines()[1].startswith("/scan_filtered")
    assert "and 2 more channels, raise limit or pass focus" in capped
    assert len(capped.encode("utf-8")) <= tools.MAX_BYTES


# ---------------------------------------------------------------------------
# rosio graph, the same selection on the command line and drawn

ROVER_DOT = """\
digraph rosio {
  // 2 nodes, 3 topics, 0 services, 0 actions, 4 endpoints in profile rover
  rankdir=LR;
  t_31404efdc152 [label="/cmd_vel", shape=box, style=rounded];
  t_824d774fc200 [label="/diagnostics", shape=box, style=rounded];
  n_05f9e9f9291a [label="/safety_stop", shape=box];
  n_e882b3be290c [label="/scan_filter", shape=box];
  t_62abc172c749 [label="/scan_filtered\\nsensor_msgs/msg/LaserScan\\n[qos.incompatible]", \
shape=box, style=rounded, color="#d00000", fontcolor="#d00000", penwidth=2];
  n_05f9e9f9291a -> t_31404efdc152;
  n_05f9e9f9291a -> t_824d774fc200;
  n_e882b3be290c -> t_62abc172c749 [label="sensor_data preset"];
  t_62abc172c749 -> n_05f9e9f9291a [label="reliable, depth 10"];
}
"""

ROVER_MERMAID = """\
flowchart LR
  %% 2 nodes, 3 topics, 0 services, 0 actions, 4 endpoints in profile rover
  t_31404efdc152(["/cmd_vel"])
  t_824d774fc200(["/diagnostics"])
  n_05f9e9f9291a["/safety_stop"]
  n_e882b3be290c["/scan_filter"]
  t_62abc172c749(["/scan_filtered<br/>sensor_msgs/msg/LaserScan<br/>[qos.incompatible]"])
  n_05f9e9f9291a --> t_31404efdc152
  n_05f9e9f9291a --> t_824d774fc200
  n_e882b3be290c -->|"sensor_data preset"| t_62abc172c749
  t_62abc172c749 -->|"reliable, depth 10"| n_05f9e9f9291a
  classDef finding stroke:#d00000,stroke-width:2px,color:#d00000
  class t_62abc172c749 finding
"""


def rover_file(tmp_path: Path) -> Path:
    target = tmp_path / "atlas.json"
    target.write_text(model.dumps(rover().to_json()), encoding="utf-8", newline="\n")
    return target


def test_graph_on_the_command_line_prints_the_tool_answer(tmp_path, capsys):
    from rosio.cli import main

    path = rover_file(tmp_path)
    atlas = model.Atlas.loads(path.read_text(encoding="utf-8"))
    for flags, options in (
        ([], {}),
        (["--profile", "rover", "--wrappers"], {"profile": "rover", "wrappers": True}),
        (["--focus", "/scan_filter", "--depth", "1"], {"focus": "/scan_filter", "depth": 1}),
        (["--limit", "1"], {"limit": 1}),
    ):
        assert main(["graph", "--atlas", str(path), *flags]) == 0
        assert capsys.readouterr().out == tools.graph(atlas, **options) + "\n"


def test_graph_on_the_command_line_exits_with_the_usage_code_when_it_cannot_answer(
    tmp_path, capsys
):
    from rosio.cli import main

    path = str(rover_file(tmp_path))
    assert main(["graph", "--atlas", path, "--focus", "/nobody"]) == 2
    assert "nothing in this atlas is named '/nobody'" in capsys.readouterr().out
    assert main(["graph", "--atlas", path, "--focus", "/nobody", "--format", "dot"]) == 2
    assert capsys.readouterr().out == ""
    assert main(["graph", "--atlas", path, "--profile", "mars"]) == 2
    assert main(["graph", "--atlas", path, "--json"]) == 2


def test_graph_draws_dot_and_mermaid_with_data_flowing_left_to_right(tmp_path, capsys):
    from rosio.cli import main

    atlas = rover()
    assert tools.diagram(atlas, "dot", profile="rover", wrappers=True) == ROVER_DOT
    assert tools.diagram(atlas, "mermaid", profile="rover", wrappers=True) == ROVER_MERMAID
    path = str(rover_file(tmp_path))
    flags = ["--profile", "rover", "--wrappers", "--format", "mermaid"]
    assert main(["graph", "--atlas", path, *flags]) == 0
    first = capsys.readouterr().out
    assert main(["graph", "--atlas", path, *flags]) == 0
    assert capsys.readouterr().out == first == ROVER_MERMAID


def test_a_diagram_hides_library_endpoints_and_follows_a_focus():
    atlas = rover()
    hidden = tools.diagram(atlas, "dot", profile="rover")
    assert "/diagnostics" not in hidden
    assert "// 1 library endpoint left out" in hidden

    one = tools.diagram(atlas, "mermaid", profile="rover", focus="/scan_filter", depth=1)
    two = tools.diagram(atlas, "mermaid", profile="rover", focus="/scan_filter", depth=2)
    assert "/scan_filtered" in one and "/safety_stop" not in one
    assert "/safety_stop" in two and "/cmd_vel" not in two

    capped = tools.diagram(atlas, "dot", profile="rover", limit=1)
    assert "/scan_filtered" in capped and "/cmd_vel" not in capped
    assert "// 1 more channel not drawn" in capped


def test_a_diagram_never_names_a_synthetic_id_and_refuses_what_it_cannot_draw():
    atlas = rover()
    for fmt in tools.DIAGRAM_FORMATS:
        drawn = tools.diagram(atlas, fmt)
        assert "sim_pkg/sim_node (unnamed by launch)" in drawn
        assert ":~" not in drawn and "%7C" not in drawn
        with pytest.raises(UsageError, match="nothing in this atlas is named"):
            tools.diagram(atlas, fmt, focus="/nobody")
    with pytest.raises(UsageError, match="no diagram format"):
        tools.diagram(atlas, "svg")


def test_a_diagram_escapes_labels_for_each_format():
    assert tools._dot_text('a "b" \\N <c>|d\ne') == 'a \\"b\\" \\\\N <c>|d\\ne'
    assert tools._mermaid_text('a "b" #1 <c>|d&`e`\nf') == (
        "a #quot;b#quot; #35;1 #lt;c#gt;#124;d#amp;#96;e#96;<br/>f"
    )

    builder = model.AtlasBuilder()
    builder.declare_layer("coded", present=True)
    builder.add_profile("odd")
    node, topic = ids.build_node("/odd"), ids.build_topic("/odd_out")
    for entity_id in (node, topic):
        builder.fact(entity_id, "coded", profile="profile:odd", values={"seen": True})
    builder.node(node, name_source="literal")
    builder.edge(
        "publishes",
        node,
        topic,
        key=ids.edge_key(profile="profile:odd"),
        layer="coded",
        profile="profile:odd",
        values={"qos": {"preset": 'say "hi" <now>|x'}},
    )
    atlas = builder.build()
    assert '[label="say \\"hi\\" <now>|x preset"]' in tools.diagram(atlas, "dot")
    assert '-->|"say #quot;hi#quot; #lt;now#gt;#124;x preset"|' in tools.diagram(atlas, "mermaid")


def test_graphviz_reads_the_dot_it_is_given():
    dot = shutil.which("dot")
    if dot is None:
        pytest.skip("graphviz is not installed, so there is no dot to parse the output")
    atlas = rover()
    for options in ({}, {"profile": "rover", "wrappers": True}, {"focus": "/cmd_vel"}):
        done = subprocess.run(
            [dot, "-Tsvg"],
            input=tools.diagram(atlas, "dot", **options),
            capture_output=True,
            text=True,
            check=False,
        )
        assert done.returncode == 0, done.stderr
        assert "<svg" in done.stdout
