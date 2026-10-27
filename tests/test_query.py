"""The query surface: the seven answers, their provenance and their honesty."""

from __future__ import annotations

import pytest

from rosio import ids, model, query
from rosio.errors import UsageError
from rosio.model import Candidate, Finding, Source, Unresolved

REAL = "profile:nx_navigation"
SIM = "profile:nx_sim"

PARAMS_FILE = ids.build_file("nx_bringup", "config/nav2_params.yaml")
OVERLAY_FILE = ids.build_file("nx_bringup", "config/nav2_overrides.yaml")
LAUNCHER_FILE = ids.build_file(ids.REPO_PKG, "dx_app/dx_launcher/profiles.py")
CONTROLLER = ids.build_node("/controller_server")
MONITOR = ids.build_node("/collision_monitor")
CMD_VEL = ids.build_topic("/cmd_vel")
COST_PENALTY = ids.build_param(CONTROLLER, "FollowPath.cost_penalty")
UNNAMED = ids.build_node_synthetic("nav2_controller", "controller_server", "/", "75ed3551")


def build(*, penalty: float = 1.0, with_monitor: bool = True) -> model.Atlas:
    """A small atlas with the shapes the verbs have to render: two profiles, a candidate
    chain, an endpoint pair, a blind spot and a finding."""
    builder = model.AtlasBuilder()
    builder.add_profile("nx_navigation")
    builder.add_profile("nx_sim")
    for file_id in (PARAMS_FILE, OVERLAY_FILE, LAUNCHER_FILE):
        builder.node(file_id)
    sym = ids.build_sym(ids.REPO_PKG, "dx_app/dx_launcher/profiles.py", "nav2_command")
    builder.node(sym)

    builder.fact(
        CONTROLLER,
        "declared",
        profile=REAL,
        values={"pkg": "nav2_controller", "executable": "controller_server"},
        evidence=[Source(file=PARAMS_FILE, locator="controller_server", line_hint=140)],
    )
    builder.fact(CONTROLLER, "coded", profile=REAL, values={"pkg": "nav2_controller"})
    builder.node(CONTROLLER, name_source="literal")
    builder.node(CMD_VEL)
    builder.edge(
        "publishes",
        CONTROLLER,
        CMD_VEL,
        key=ids.edge_key(symbol=sym),
        layer="coded",
        profile=REAL,
        values={"qos_depth": 10, "type": "geometry_msgs/msg/Twist"},
    )
    if with_monitor:
        builder.fact(MONITOR, "declared", profile=REAL, values={"pkg": "nav2_collision_monitor"})
        builder.node(MONITOR, name_source="literal")
        builder.edge(
            "subscribes",
            MONITOR,
            CMD_VEL,
            key=ids.edge_key(symbol=sym),
            layer="coded",
            profile=REAL,
            values={"qos_depth": 1},
        )
    builder.set_param(
        COST_PENALTY,
        state="effective",
        candidates=[
            Candidate(
                value=penalty,
                value_type="float",
                mechanism="yaml_overlay_merge",
                source=Source(
                    file=OVERLAY_FILE,
                    locator="controller_server.ros__parameters.FollowPath.cost_penalty",
                    line_hint=154,
                ),
                rank=0,
                wins=True,
                profile=REAL,
                applied_by=sym,
            ),
            Candidate(
                value=2.0,
                value_type="float",
                mechanism="yaml_base",
                source=Source(
                    file=PARAMS_FILE,
                    locator="controller_server.ros__parameters.FollowPath.cost_penalty",
                    line_hint=381,
                ),
                rank=1,
                wins=False,
                profile=REAL,
                shadow_kind="value_changed",
                shadowed_by=0,
            ),
            Candidate(
                value=9.0,
                value_type="float",
                mechanism="yaml_base",
                source=Source(file=PARAMS_FILE, locator="controller_server", line_hint=381),
                rank=0,
                wins=True,
                profile=SIM,
            ),
        ],
    )
    builder.edge("has_param", CONTROLLER, COST_PENALTY)
    builder.unresolved(
        Unresolved(
            want="topic_name",
            owner=sym,
            expr="self.get_parameter('speed_limit_topic')",
            reason="parameter with no default",
            evidence=[Source(file=LAUNCHER_FILE, line_hint=430)],
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
            say="cost_penalty on controller_server is 1.0 from nav2_overrides.yaml:154",
            profiles=[REAL],
            where=[Source(file=PARAMS_FILE, locator="controller_server", line_hint=381)],
        )
    )
    return builder.build()


@pytest.fixture
def atlas() -> model.Atlas:
    return build()


def texts(answer: query.Answer) -> str:
    return answer.text()


# ---------------------------------------------------------------------------
# shared shaping


def test_a_synthetic_id_never_reaches_a_reader():
    name = query.label(UNNAMED)

    assert ":~" not in name
    assert "controller_server" in name
    assert "unnamed" in name


def test_a_site_reads_as_the_file_a_person_would_open():
    assert query.site(Source(file=PARAMS_FILE, line_hint=381)) == (
        "nx_bringup/config/nav2_params.yaml:381"
    )
    assert query.site(Source(file=LAUNCHER_FILE, line_hint=430)) == (
        "dx_app/dx_launcher/profiles.py:430"
    )


def test_a_selector_resolves_by_id_by_name_and_by_kind(atlas):
    assert query.resolve(atlas, CONTROLLER) == [CONTROLLER]
    assert query.resolve(atlas, "/controller_server") == [CONTROLLER]
    assert query.resolve(atlas, "topic:/cmd_vel") == [CMD_VEL]
    assert query.resolve(atlas, "nothing_here") == []
    with pytest.raises(UsageError):
        query.resolve(atlas, "  ")


# ---------------------------------------------------------------------------
# explain


def test_explain_states_the_layers_the_facts_and_the_provenance(atlas):
    answer = query.explain(atlas, "/controller_server")

    assert answer.ok
    assert answer.data["layers"] == ["declared", "coded"]
    assert answer.data["profiles"] == [REAL]
    assert answer.data["facts"][0]["where"] == ["nx_bringup/config/nav2_params.yaml:140"]
    body = texts(answer)
    assert "node /controller_server" in body
    assert "nav2_params.yaml:140" in body
    assert "publishes" in body


def test_explain_names_the_fields_two_layers_disagree_about():
    builder = model.AtlasBuilder()
    builder.add_profile("nx_navigation")
    builder.fact(CMD_VEL, "declared", profile=REAL, values={"type": "geometry_msgs/msg/Twist"})
    builder.fact(CMD_VEL, "coded", profile=REAL, values={"type": "geometry_msgs/msg/TwistStamped"})
    atlas = builder.build()

    answer = query.explain(atlas, "/cmd_vel")

    assert answer.data["conflicts"] == ["type"]
    assert "layers disagree about: type" in texts(answer)


def test_explain_of_a_finding_carries_the_say_and_the_site(atlas):
    handle = atlas.findings[0].id

    answer = query.explain(atlas, handle)

    assert answer.ok
    assert "delete the base line" in texts(answer)
    assert "nav2_params.yaml:381" in texts(answer)


def test_explain_of_an_unknown_name_says_so_instead_of_guessing(atlas):
    answer = query.explain(atlas, "/not_a_node")

    assert answer.ok is False
    assert "nothing in this atlas is named '/not_a_node', searched nodes" in texts(answer)
    assert "frame or a package" in texts(query.explain(atlas, "not_a_node"))


def test_explain_states_values_without_python_quoting_or_raw_ids():
    builder = model.AtlasBuilder()
    source = ids.build_file("rover_control", "rover_control/safety_stop.py")
    builder.node(source)
    builder.fact(
        "node:/safety_stop",
        "coded",
        values={"coded_sources": [source], "executable": "", "code_name": "safety_stop"},
    )

    body = texts(query.explain(builder.build(), "/safety_stop"))

    assert "coded_sources=[rover_control/rover_control/safety_stop.py]" in body
    assert "executable" not in body
    assert "'" not in body and "file:" not in body


def test_explain_of_an_ambiguous_name_lists_the_choices(atlas):
    answer = query.explain(atlas, "cost_penalty")

    assert answer.ok is False or answer.data.get("kind") == "param"


# ---------------------------------------------------------------------------
# param


def test_param_states_the_winner_with_its_file_and_the_shadowed_chain(atlas):
    answer = query.param(atlas, "/controller_server", "FollowPath.cost_penalty")

    row = answer.data["profiles"][REAL]
    assert row["value"] == 1.0
    assert row["value_type"] == "float"
    assert "nav2_overrides.yaml:154" in row["winner"]
    assert [item["rank"] for item in row["chain"]] == [0, 1]
    assert row["chain"][1]["shadow_kind"] == "value_changed"
    body = texts(answer)
    assert "effective 1.0 (float)" in body
    assert "nx_bringup/config/nav2_params.yaml:381" in body
    assert "shadowed by rank 0" in body


def test_param_keeps_the_profiles_apart(atlas):
    answer = query.param(atlas, "/controller_server", "FollowPath.cost_penalty", profile="nx_sim")

    assert list(answer.data["profiles"]) == [SIM]
    assert answer.data["profiles"][SIM]["value"] == 9.0


def test_param_refuses_a_profile_this_atlas_does_not_have(atlas):
    with pytest.raises(UsageError):
        query.param(atlas, "/controller_server", "FollowPath.cost_penalty", profile="nx_mapping")


def test_param_takes_the_node_and_the_name_in_one_selector(atlas):
    answer = query.param(atlas, "/controller_server FollowPath.cost_penalty")

    assert answer.data["profiles"][REAL]["value"] == 1.0


def _shared_name_atlas() -> model.Atlas:
    """Two nodes carrying a parameter of the same name, which is what one token names."""
    builder = model.AtlasBuilder()
    builder.add_profile("nx_navigation")
    builder.node(PARAMS_FILE)
    for owner in (CONTROLLER, MONITOR):
        builder.node(owner, name_source="literal")
        builder.set_param(
            ids.build_param(owner, "use_sim_time"),
            state="effective",
            candidates=[
                Candidate(
                    value=False,
                    value_type="bool",
                    mechanism="yaml_base",
                    source=Source(file=PARAMS_FILE, locator="use_sim_time", line_hint=114),
                    rank=0,
                    wins=True,
                    profile=REAL,
                )
            ],
        )
    return builder.build()


def test_a_count_is_printed_with_the_noun_it_takes():
    from rosio.render import plural

    assert [plural(1, "entity"), plural(2, "entity"), plural(1, "root"), plural(0, "key")] == [
        "1 entity",
        "2 entities",
        "1 root",
        "0 keys",
    ]


def test_param_says_which_half_of_the_node_and_the_name_found_nothing(atlas):
    """One string, '/nonexistent foo', read as one name nobody carries."""
    missing_node = texts(query.param(atlas, "/nonexistent", "foo"))
    missing_name = texts(query.param(atlas, "/controller_server", "no_such_key"))

    assert missing_node.startswith("no node /nonexistent in this atlas")
    assert "'/nonexistent foo'" not in missing_node
    assert missing_name.startswith("/controller_server carries no parameter no_such_key")


def test_param_with_one_token_names_the_nodes_that_carry_it():
    answer = query.param(_shared_name_atlas(), "use_sim_time")

    body = texts(answer)
    assert answer.ok is False
    assert "None" not in body
    assert "2 parameters match 'use_sim_time'" in body
    assert "param /controller_server use_sim_time" in body
    assert "param /collision_monitor use_sim_time" in body


def test_param_with_a_name_no_node_carries_says_so_and_names_the_form(atlas):
    answer = query.param(atlas, "max_vel_theta")

    body = texts(answer)
    assert answer.ok is False
    assert "None" not in body
    assert "no parameter named 'max_vel_theta'" in body
    assert "param takes a node and a name" in body


def test_param_given_only_a_node_asks_for_the_parameter(atlas):
    answer = query.param(atlas, "/controller_server")

    body = texts(answer)
    assert answer.ok is False
    assert "None" not in body
    assert "/controller_server is a node" in body


def test_param_says_why_a_block_that_is_not_read_has_no_value():
    builder = model.AtlasBuilder()
    builder.add_profile("nx_navigation")
    builder.node(PARAMS_FILE)
    builder.node(MONITOR, name_source="literal")
    orphan = ids.build_param(MONITOR, "width")
    builder.set_param(
        orphan,
        state="owner_name_mismatch",
        candidates=[
            Candidate(
                value=40,
                value_type="int",
                mechanism="yaml_base",
                source=Source(file=PARAMS_FILE, locator="map_saver", line_hint=452),
                rank=0,
                wins=False,
                profile=REAL,
            )
        ],
    )
    atlas = builder.build()

    answer = query.param(atlas, "/collision_monitor", "width")

    assert answer.data["state"] == "owner_name_mismatch"
    assert "matches no launched node" in texts(answer)
    assert "effective 40" not in texts(answer)


# ---------------------------------------------------------------------------
# query


def test_query_matches_by_name_and_returns_the_relations(atlas):
    answer = query.query(atlas, "who publishes cmd_vel")

    assert answer.ok
    assert CMD_VEL in [row["id"] for row in answer.data["matched"]]
    assert any(row["kind"] == "publishes" for row in answer.data["edges"])
    assert "/controller_server publishes /cmd_vel" in texts(answer)


def test_query_scoped_by_kind_falls_back_to_the_named_entity(atlas):
    answer = query.query(atlas, "which nodes publish cmd_vel")

    assert answer.ok
    assert answer.data["kinds"] == ["node"]
    assert answer.data["relations"] == ["publishes"]
    assert any(row["src"] == CONTROLLER for row in answer.data["edges"])


def test_query_says_honestly_when_it_did_not_understand(atlas):
    answer = query.query(atlas, "what is it that this does")

    assert answer.ok is False
    assert "not understood" in texts(answer)


def test_query_says_honestly_when_nothing_matched(atlas):
    answer = query.query(atlas, "topic wheel_odometry")

    assert answer.ok is False
    assert "nothing matched" in texts(answer)
    assert "this atlas holds" in texts(answer)


# ---------------------------------------------------------------------------
# path


def test_path_walks_through_the_topic_and_names_every_edge_kind(atlas):
    answer = query.path(atlas, "/controller_server", "/collision_monitor")

    assert answer.ok
    assert [hop["kind"] for hop in answer.data["hops"]] == ["publishes", "subscribes"]
    assert [hop["forward"] for hop in answer.data["hops"]] == [True, False]
    assert "-> publishes /cmd_vel" in texts(answer)
    assert "<- subscribes /collision_monitor" in texts(answer)


def test_path_from_one_entity_to_itself_says_so(atlas):
    answer = query.path(atlas, "/controller_server", "/controller_server")

    assert answer.ok
    assert answer.data["hops"] == []
    assert "no path" not in texts(answer)


def test_path_is_the_same_both_ways_and_the_shortest_one(atlas):
    assert len(query.path(atlas, "/collision_monitor", "/controller_server").data["hops"]) == 2


def test_path_says_there_is_none_rather_than_inventing_one():
    atlas = build(with_monitor=False)
    builder = model.AtlasBuilder()
    builder.fact(MONITOR, "declared", profile=REAL, values={"pkg": "nav2_collision_monitor"})
    atlas.entities[MONITOR] = builder.atlas.entities[MONITOR]

    answer = query.path(atlas, "/controller_server", "/collision_monitor")

    assert answer.ok is False
    assert "no path" in texts(answer)


# ---------------------------------------------------------------------------
# diff


def test_diff_of_one_atlas_against_itself_is_empty(atlas):
    answer = query.diff(atlas, build())

    assert answer.data["identical"] is True
    assert "no semantic difference" in texts(answer)


def test_diff_reports_the_changed_value_and_the_removed_node(atlas):
    answer = query.diff(atlas, build(penalty=3.0, with_monitor=False))

    assert answer.data["counts"]["entities"]["removed"] == 1
    assert answer.data["counts"]["entities"]["changed"] >= 1
    changed = {row["label"] for row in answer.data["entities"]["changed"]}
    assert any("cost_penalty" in name for name in changed)
    assert "subscribes" in texts(answer)


def test_diff_ignores_a_line_hint_that_moved():
    left = build()
    right = build()
    for candidate in right.entities[COST_PENALTY].candidates:
        candidate.source = Source(
            file=candidate.source.file,
            locator=candidate.source.locator,
            line_hint=(candidate.source.line_hint or 0) + 40,
            root=candidate.source.root,
        )

    assert query.diff(left, right).data["identical"] is True


def test_diff_compares_two_profiles_of_one_atlas(atlas):
    answer = query.diff(atlas, left_profile="nx_navigation", right_profile="nx_sim")

    assert answer.data["identical"] is False
    assert "profile nx_navigation against nx_sim" in texts(answer)


def test_diff_of_two_atlases_needs_two_atlases(atlas):
    with pytest.raises(UsageError):
        query.diff(atlas)
    with pytest.raises(UsageError):
        query.diff(atlas, left_profile="nx_navigation")


# ---------------------------------------------------------------------------
# diff: what changed, and does it matter


def test_the_value_that_moved_is_named_with_both_values_and_the_line(atlas):
    """The question a person asks is what changed, and the answer is a value and a line."""
    answer = query.diff(atlas, build(penalty=3.0))

    rows = answer.data["matters"]["values"]
    assert len(rows) == 1
    row = rows[0]
    assert row["before"] == 1.0
    assert row["after"] == 3.0
    assert row["profile"] == REAL
    assert "nav2_overrides.yaml:154" in row["locator"]
    body = texts(answer)
    assert "1.0 -> 3.0" in body
    assert "cost_penalty" in body


def test_what_matters_is_printed_before_the_ids_and_the_counts(atlas):
    lines = texts(query.diff(atlas, build(penalty=3.0))).splitlines()

    assert lines[1].startswith("  value ")
    assert any(line.startswith("  entities:") for line in lines)
    assert lines.index("  entities: 0 added, 0 removed, 1 changed") > 2


def test_a_node_that_vanished_is_named_and_the_topic_it_left_behind(atlas):
    answer = query.diff(atlas, build(with_monitor=False))

    matters = answer.data["matters"]
    assert [row["label"] for row in matters["nodes"]["vanished"]] == ["/collision_monitor"]
    assert matters["nodes"]["appeared"] == []
    assert [row["label"] for row in matters["topics"]["lost_subscriber"]] == ["/cmd_vel"]
    assert "has no subscriber now" in texts(answer)


def test_a_node_that_appeared_is_named(atlas):
    answer = query.diff(build(with_monitor=False), atlas)

    assert [row["label"] for row in answer.data["matters"]["nodes"]["appeared"]] == [
        "/collision_monitor"
    ]


def test_a_finding_that_is_new_or_gone_is_named_with_its_sentence(atlas):
    quiet = build()
    quiet.findings = []

    gone = query.diff(atlas, quiet).data["matters"]["findings"]
    new = query.diff(quiet, atlas).data["matters"]["findings"]

    assert [row["rule"] for row in gone["gone"]] == ["param.shadowed"]
    assert gone["new"] == []
    assert [row["rule"] for row in new["new"]] == ["param.shadowed"]
    assert "cost_penalty" in new["new"][0]["say"]


def test_a_parameter_that_stopped_being_effective_is_named():
    left = build()
    right = build()
    right.entities[COST_PENALTY].state = "owner_name_mismatch"

    rows = query.diff(left, right).data["matters"]["states"]

    assert rows[0]["before"] == "effective"
    assert rows[0]["after"] == "owner_name_mismatch"


def test_a_line_hint_that_moved_is_not_something_that_matters(atlas):
    right = build()
    for candidate in right.entities[COST_PENALTY].candidates:
        candidate.source = Source(
            file=candidate.source.file,
            locator=candidate.source.locator,
            line_hint=(candidate.source.line_hint or 0) + 40,
            root=candidate.source.root,
        )

    answer = query.diff(atlas, right)

    assert answer.data["matters_total"] == 0
    assert answer.data["identical"] is True


def test_two_atlases_that_differ_only_in_ids_say_that_nothing_that_matters_moved(atlas):
    right = build()
    right.entities[ids.build_file("nx_bringup", "config/extra.yaml")] = model.Node(
        id=ids.build_file("nx_bringup", "config/extra.yaml"), kind="file"
    )

    answer = query.diff(atlas, right)

    assert answer.data["matters_total"] == 0
    assert "nothing a reader asks about moved" in texts(answer)
    assert "added file" in texts(answer)


def test_two_profiles_of_one_atlas_are_grouped_the_same_way(atlas):
    """Simulation and the real robot are told apart by the same reading."""
    answer = query.diff(atlas, left_profile="nx_navigation", right_profile="nx_sim")

    rows = answer.data["matters"]["values"]
    assert [row["before"] for row in rows] == [1.0]
    assert [row["after"] for row in rows] == [9.0]
    assert "1.0 -> 9.0" in texts(answer)


def test_a_value_that_changed_type_is_a_change(atlas):
    right = build()
    node = right.entities[COST_PENALTY]
    for candidate in node.candidates:
        if candidate.wins and candidate.profile == REAL:
            candidate.value = 1
            candidate.value_type = "int"
    right = _recomputed(right)

    rows = query.diff(atlas, right).data["matters"]["values"]

    assert rows[0]["before"] == 1.0
    assert rows[0]["after"] == 1


def _recomputed(atlas: model.Atlas) -> model.Atlas:
    builder = model.AtlasBuilder()
    builder.atlas = atlas
    builder.recompute()
    return atlas


# ---------------------------------------------------------------------------
# diff: the build that looked, told apart from the system it looked at


RVIZ_LAUNCH = ids.build_launchfile("nav2_bringup", "launch/rviz_launch.py")


def _stamped(atlas: model.Atlas, input_digest: str) -> model.Atlas:
    """One atlas as a build of its own writes it: a run entity carrying the input digest,
    and every fact pointing at that run."""
    builder = model.AtlasBuilder()
    builder.atlas = atlas
    run = builder.add_run(
        "declared", profile=REAL, input_digest=input_digest, roots={"src": input_digest}
    )
    for item in [*atlas.entities.values(), *atlas.edges.values()]:
        for fact in item.facts:
            fact.run = run
    builder.recompute()
    return atlas


def _with_entry_point(atlas: model.Atlas) -> model.Atlas:
    builder = model.AtlasBuilder()
    builder.atlas = atlas
    builder.add_profile("rviz")
    builder.fact(RVIZ_LAUNCH, "declared", profile=ids.build_profile("rviz"), values={"nodes": 1})
    builder.recompute()
    return atlas


def test_a_rebuild_of_one_tree_moves_the_build_identity_and_nothing_else():
    """One byte anywhere renames every run, so two builds of one tree disagree about every
    run id and about the run every fact was written by, and about nothing else."""
    answer = query.diff(_stamped(build(), "aaaa1111"), _stamped(build(), "bbbb2222"))

    assert answer.data["matters_total"] == 0
    assert answer.data["counts"] == {
        "entities": {"added": 0, "removed": 0, "changed": 0},
        "edges": {"added": 0, "removed": 0, "changed": 0},
    }
    assert answer.data["build"]["entities"]["added"] == 1
    assert answer.data["build"]["entities"]["removed"] == 1
    assert answer.data["build"]["entities"]["changed"] > 0
    assert answer.data["identical"] is False
    body = texts(answer)
    assert "build identity" in body
    assert "only the build identity moved" in body


def test_a_value_that_moved_survives_a_rebuild_and_the_churn_does_not_join_it():
    answer = query.diff(_stamped(build(), "aaaa1111"), _stamped(build(penalty=3.0), "bbbb2222"))

    assert [row["after"] for row in answer.data["matters"]["values"]] == [3.0]
    assert [row["id"] for row in answer.data["entities"]["changed"]] == [COST_PENALTY]
    assert answer.data["counts"]["edges"]["changed"] == 0
    assert answer.data["build"]["entities"]["added"] == 1


def test_two_atlases_from_one_build_say_so():
    answer = query.diff(
        _stamped(build(), "aaaa1111"), _stamped(build(with_monitor=False), "aaaa1111")
    )

    assert answer.data["build"]["entities"] == {"added": 0, "removed": 0, "changed": 0}
    assert answer.data["build"]["edges"] == {"added": 0, "removed": 0, "changed": 0}
    assert "one build wrote both sides" in texts(answer)


def test_an_entry_point_and_a_profile_that_went_are_named_before_the_nodes():
    """Deleting a launch file takes a whole system with it, and the node that went with it
    is the consequence. Both are named, and the launch file first."""
    answer = query.diff(_with_entry_point(build()), build(with_monitor=False))

    matters = answer.data["matters"]
    assert [row["label"] for row in matters["launch"]["vanished"]] == [
        "nav2_bringup/launch/rviz_launch.py"
    ]
    assert [row["label"] for row in matters["profiles"]["vanished"]] == ["rviz"]
    lines = texts(answer).splitlines()
    launch = [index for index, line in enumerate(lines) if line.startswith("  launch file ")]
    nodes = [index for index, line in enumerate(lines) if line.startswith("  node vanished")]
    assert launch and nodes and launch[0] < nodes[0]
    assert "  profile vanished rviz" in lines


def _refused(atlas: model.Atlas, input_digest: str) -> model.Atlas:
    """One atlas whose extraction lost a pass, reported the way a real one reports it: the
    finding is about the run, so its own id carries that run's input digest."""
    builder = model.AtlasBuilder()
    builder.atlas = atlas
    run = builder.add_run("declared", input_digest=input_digest, roots={"src": input_digest})
    builder.finding(
        Finding(
            rule="extract.pass_incomplete",
            severity="error",
            subject=run,
            discriminator="h17a74252",
            title="the plugins collector stopped",
            detail="cannot read file:nav2_bringup|CMakeLists.txt",
            fix="make the file readable",
            say="the plugins collector stopped in the declared layer",
        )
    )
    builder.recompute()
    return atlas


def test_the_same_lost_pass_on_both_sides_is_not_a_finding_gone_and_a_finding_new():
    """A finding about a run holds that run's input digest in its own id, so one byte
    anywhere renamed it and the same refusal read as one gone and one new, in the head."""
    answer = query.diff(
        _refused(_stamped(build(), "aaaa1111"), "aaaa1111"),
        _refused(_stamped(build(), "bbbb2222"), "bbbb2222"),
    )

    assert answer.data["matters"]["findings"] == {"gone": [], "new": [], "restated": []}
    assert answer.data["matters_total"] == 0
    assert "only the build identity moved" in texts(answer)


def test_a_lost_pass_on_one_side_only_is_still_a_finding_that_appeared():
    answer = query.diff(
        _stamped(build(), "aaaa1111"),
        _refused(_stamped(build(), "bbbb2222"), "bbbb2222"),
    )

    rows = answer.data["matters"]["findings"]["new"]
    assert [row["rule"] for row in rows] == ["extract.pass_incomplete"]
    assert rows[0]["id"].startswith("find:extract.pass_incomplete|run:")
    assert answer.data["matters_total"] == 1


def test_an_entry_point_and_a_profile_that_arrived_are_named():
    matters = query.diff(build(), _with_entry_point(build())).data["matters"]

    assert [row["label"] for row in matters["launch"]["appeared"]] == [
        "nav2_bringup/launch/rviz_launch.py"
    ]
    assert [row["label"] for row in matters["profiles"]["appeared"]] == ["rviz"]
    assert matters["launch"]["vanished"] == []


# ---------------------------------------------------------------------------
# unresolved


def test_unresolved_groups_by_want_and_by_owner(atlas):
    answer = query.unresolved(atlas)

    assert answer.data["total"] == 1
    assert answer.data["by_want"]["topic_name"]["count"] == 1
    assert list(answer.data["by_owner"].values()) == [1]
    body = texts(answer)
    assert "topic_name: 1 unresolved" in body
    assert "speed_limit_topic" in body
    assert ":~" not in body


def test_unresolved_filters_by_want(atlas):
    assert query.unresolved(atlas, want="frame_id").data["total"] == 0


def test_unresolved_shows_as_many_sites_per_want_as_the_limit_and_counts_the_rest():
    builder = model.AtlasBuilder()
    builder.node(LAUNCHER_FILE)
    for index in range(7):
        builder.unresolved(
            Unresolved(
                want="topic_name",
                owner=LAUNCHER_FILE,
                expr=f"topic_{index}",
                reason="not_stated_in_source",
                evidence=[Source(file=LAUNCHER_FILE, line_hint=index + 1)],
            )
        )
    atlas = builder.build()

    few = texts(query.unresolved(atlas, limit=3))
    every = texts(query.unresolved(atlas, limit=100))

    assert (
        sum(line.strip()[:7] in {f"topic_{i}" for i in range(7)} for line in few.splitlines()) == 3
    )
    assert "and 4 more, --limit shows more" in few
    assert (
        sum(line.strip()[:7] in {f"topic_{i}" for i in range(7)} for line in every.splitlines())
        == 7
    )
    assert "more, --limit" not in every


# ---------------------------------------------------------------------------
# drift


def test_drift_lists_the_findings_with_the_say_and_the_site(atlas):
    answer = query.drift(atlas)

    assert answer.data["total"] == 1
    assert answer.data["by_severity"]["warn"] == 1
    body = texts(answer)
    assert "param.shadowed" in body
    assert "nav2_overrides.yaml:154" in body


def test_drift_filters_by_severity_and_by_rule(atlas):
    assert query.drift(atlas, min_severity="error").data["total"] == 0
    assert query.drift(atlas, rule="topic.no_subscriber").data["total"] == 0
    assert query.drift(atlas, rule="param.shadowed").data["total"] == 1


def test_drift_on_an_atlas_with_no_findings_says_so():
    builder = model.AtlasBuilder()
    builder.node(CMD_VEL)
    answer = query.drift(builder.build())

    assert "no drift reported" in texts(answer)


# ---------------------------------------------------------------------------
# the shape of every answer


def test_every_answer_is_small_enough_to_paste_and_carries_no_synthetic_id(atlas):
    answers = [
        query.explain(atlas, "/controller_server"),
        query.param(atlas, "/controller_server", "FollowPath.cost_penalty"),
        query.query(atlas, "who publishes cmd_vel"),
        query.path(atlas, "/controller_server", "/collision_monitor"),
        query.diff(atlas, build(penalty=3.0)),
        query.unresolved(atlas),
        query.drift(atlas),
    ]

    for answer in answers:
        body = answer.text()
        assert body, answer.verb
        assert ":~" not in body, answer.verb
        assert len(body.splitlines()) <= 80, answer.verb
        assert answer.to_json()["verb"] == answer.verb
        assert model.dumps(answer.to_json())


def test_two_runs_of_one_verb_render_the_same_text(atlas):
    assert query.explain(atlas, "/controller_server").text() == (
        query.explain(build(), "/controller_server").text()
    )


def test_a_profile_with_an_empty_effective_map_is_shown_no_value(atlas):
    """Empty and absent are different states. A profile whose effective map is stated and
    holds nothing is blind here, and the workspace wide entry of another scope is not its
    value. Falling back showed a blind profile a number it does not have."""
    node = atlas.entities[COST_PENALTY]
    node.effective = {
        model.ANY_PROFILE: {
            "value": model.EffectiveValue(value=7.0, layer="coded", confidence="resolved")
        },
        SIM: {},
    }
    view = query._profile_view(atlas, SIM)
    assert view["entities"][COST_PENALTY]["effective"] == {}
    other = query._profile_view(atlas, REAL)
    assert other["entities"][COST_PENALTY]["effective"]["value"]["value"] == 7.0


# ---------------------------------------------------------------------------
# selectors: an exact name first, a namespaced one second, a substring last and said so


ROBOT = "profile:nx_multi"
NAMESPACED = ids.build_node("/robot1/controller_server")
HEADER = ids.build_file("tb_node", "include/tb_node/turtlebot3_node.hpp")
ROBOT_LAUNCH = ids.build_launchfile("tb_bringup", "launch/robot.launch.py")


def _namespaced_atlas() -> model.Atlas:
    """One node at the root and the same node under a namespace in another profile, each
    with use_sim_time, plus a header whose path holds a node name and a launch that raised."""
    builder = model.AtlasBuilder()
    builder.add_profile("nx_navigation")
    builder.add_profile("nx_multi")
    builder.node(PARAMS_FILE)
    builder.node(HEADER)
    for owner, profile, value in ((CONTROLLER, REAL, False), (NAMESPACED, ROBOT, True)):
        builder.fact(owner, "declared", profile=profile, values={"pkg": "nav2_controller"})
        builder.node(owner, name_source="literal")
        builder.set_param(
            ids.build_param(owner, "use_sim_time"),
            state="effective",
            candidates=[
                Candidate(
                    value=value,
                    value_type="bool",
                    mechanism="yaml_base",
                    source=Source(file=PARAMS_FILE, locator="use_sim_time", line_hint=114),
                    rank=0,
                    wins=True,
                    profile=profile,
                )
            ],
        )
    builder.node(ROBOT_LAUNCH)
    builder.finding(
        Finding(
            rule="launch.raised",
            severity="error",
            subject=ROBOT_LAUNCH,
            title="robot.launch.py raised",
            detail="PackageNotFoundError",
            fix="install the package",
            say="robot.launch.py raised while building its description",
            profiles=["profile:robot"],
        )
    )
    return builder.build()


def test_an_exact_name_wins_over_the_same_name_under_a_namespace():
    atlas = _namespaced_atlas()

    assert query.resolve(atlas, "/controller_server") == [CONTROLLER]
    answer = query.param(atlas, "/controller_server", "use_sim_time")
    assert answer.ok
    assert answer.data["id"] == ids.build_param(CONTROLLER, "use_sim_time")
    assert answer.data["matched_by"] != "substring"


def test_a_name_under_a_namespace_is_found_at_a_slash_and_not_inside_a_word():
    atlas = _namespaced_atlas()

    assert query.match(atlas, "/robot1/controller_server").how == "label"
    assert query.match(atlas, "robot1/controller_server").ids == [NAMESPACED]
    assert query.match(atlas, "robot1/controller_server").how == "suffix"
    assert query.match(atlas, "troller_server").how == "substring"


def test_a_profile_narrows_before_an_ambiguity_is_reported():
    atlas = _namespaced_atlas()

    assert sorted(query.resolve(atlas, "controller_server")) == sorted([CONTROLLER, NAMESPACED])
    assert query.resolve(atlas, "controller_server", profile=ROBOT) == [NAMESPACED]
    answer = query.param(atlas, "controller_server", "use_sim_time", profile="nx_multi")
    assert answer.data["id"] == ids.build_param(NAMESPACED, "use_sim_time")
    assert answer.data["profiles"][ROBOT]["value"] is True


def test_an_ambiguous_selector_prints_the_id_to_paste():
    atlas = _namespaced_atlas()

    body = texts(query.explain(atlas, "controller_server use_sim_time"))

    assert "pick one and pass its id" in body
    assert "'param:/robot1/controller_server|use_sim_time'" in body


def test_a_parameter_found_by_substring_says_so(atlas):
    answer = query.param(atlas, "/controller_server", "FollowPath.cost_pen")

    assert answer.ok
    assert answer.data["matched_by"] == "substring"
    assert (
        "nothing is named '/controller_server FollowPath.cost_pen', so this answers for "
        "/controller_server FollowPath.cost_penalty" in texts(answer)
    )
    exact = query.param(atlas, "/controller_server", "FollowPath.cost_penalty")
    assert "nothing is named" not in texts(exact)


def test_a_leading_slash_never_answers_with_a_file_and_names_the_launch_that_raised():
    atlas = _namespaced_atlas()

    assert query.resolve(atlas, "/turtlebot3_node") == []
    assert query.resolve(atlas, "turtlebot3_node") == [HEADER]
    body = texts(query.explain(atlas, "/turtlebot3_node"))
    assert "searched nodes, topics, services, actions and parameters" in body
    assert "a launch file raised in robot" in body


# ---------------------------------------------------------------------------
# drift and explain: every finding printed with what explain takes


def test_drift_prints_the_fix_and_a_fingerprint_explain_takes(atlas):
    body = texts(query.drift(atlas))
    finding = atlas.findings[0]

    assert "fix: delete the base line" in body
    assert f"explain {finding.fingerprint}" in body
    assert query.explain(atlas, finding.fingerprint).ok


def test_explain_takes_a_rule_id_and_lists_its_findings(atlas):
    answer = query.explain(atlas, "param.shadowed")

    assert answer.ok
    assert answer.data["total"] == 1
    body = texts(answer)
    assert "param.shadowed: 1 finding" in body
    assert atlas.findings[0].fingerprint in body
    quiet = query.explain(atlas, "qos.incompatible")
    assert quiet.ok
    assert "no finding of this rule" in texts(quiet)


# ---------------------------------------------------------------------------
# path: how data gets there first


RELAY = ids.build_node("/relay")
CMD_VEL_OUT = ids.build_topic("/cmd_vel_out")
MANAGER = ids.build_node("/lifecycle_manager")


def _routes(*, with_data: bool = True) -> model.Atlas:
    """A lifecycle manager driving two nodes, which is two hops, and a four hop data route
    between the same two through a relay."""
    builder = model.AtlasBuilder()
    builder.add_profile("nx_navigation")
    for node in (CONTROLLER, MONITOR, RELAY, MANAGER):
        builder.fact(node, "declared", profile=REAL, values={"pkg": "nav2"})
        builder.node(node, name_source="literal")
    for topic in (CMD_VEL, CMD_VEL_OUT):
        builder.node(topic)
    for managed in (CONTROLLER, MONITOR):
        builder.edge(
            "manages_lifecycle",
            MANAGER,
            managed,
            key=ids.edge_key(profile=REAL),
            layer="declared",
            profile=REAL,
        )
    if with_data:
        for kind, src, dst in (
            ("publishes", CONTROLLER, CMD_VEL),
            ("subscribes", RELAY, CMD_VEL),
            ("publishes", RELAY, CMD_VEL_OUT),
            ("subscribes", MONITOR, CMD_VEL_OUT),
        ):
            builder.edge(
                kind, src, dst, key=ids.edge_key(profile=REAL), layer="declared", profile=REAL
            )
    return builder.build()


def test_path_takes_the_data_route_over_a_shorter_lifecycle_one():
    answer = query.path(_routes(), "/controller_server", "/collision_monitor")

    assert answer.data["data_route"] is True
    assert [hop["kind"] for hop in answer.data["hops"]] == [
        "publishes",
        "subscribes",
        "publishes",
        "subscribes",
    ]
    assert "manages_lifecycle" not in texts(answer)


def test_path_shows_a_lifecycle_route_only_when_no_data_route_exists_and_says_so():
    answer = query.path(_routes(with_data=False), "/controller_server", "/collision_monitor")

    assert answer.ok
    assert answer.data["data_route"] is False
    assert "no data route connects them" in texts(answer)
    assert "manages_lifecycle" in texts(answer)


# ---------------------------------------------------------------------------
# diff: an edit to a line another line shadows


def test_an_edit_to_a_shadowed_line_names_both_values_and_that_the_winner_stayed(atlas):
    right = build()
    for candidate in right.entities[COST_PENALTY].candidates:
        if candidate.profile == REAL and not candidate.wins:
            candidate.value = 2.5
    right = _recomputed(right)

    answer = query.diff(atlas, right)

    rows = answer.data["matters"]["candidates"]
    assert [(row["before"], row["after"], row["effective"]) for row in rows] == [(2.0, 2.5, 1.0)]
    assert answer.data["matters"]["values"] == []
    body = texts(answer)
    assert "2.0 -> 2.5" in body
    assert "effective stays 1.0" in body
    assert "nav2_params.yaml:381" in body


def test_the_build_identity_is_one_line_and_not_a_count_table():
    answer = query.diff(_stamped(build(), "aaaa1111"), _stamped(build(penalty=3.0), "bbbb2222"))

    build_lines = [line for line in texts(answer).splitlines() if "build identity" in line]
    assert len(build_lines) == 1
    assert "added" not in build_lines[0]
    assert "One byte anywhere" not in texts(answer)
    assert answer.data["build"]["entities"]["added"] == 1


# ---------------------------------------------------------------------------
# unresolved: whole words, and which rate is which


def test_a_reason_is_printed_whole_and_each_rate_says_what_it_covers():
    """A cut reason could only be read whole through --json."""
    builder = model.AtlasBuilder()
    builder.node(LAUNCHER_FILE)
    reason = "the remap resolves to /cmd_vel_nav, and nothing in the launch walk names " * 3
    builder.unresolved(
        Unresolved(
            want="other",
            owner=LAUNCHER_FILE,
            expr="cmd_vel:=cmd_vel_nav",
            reason=reason.strip(),
            evidence=[Source(file=LAUNCHER_FILE, line_hint=12)],
        )
    )
    builder.count_resolved("other", 3)
    atlas = builder.build()

    body = texts(query.unresolved(atlas))

    shown = next(line for line in body.splitlines() if "the remap resolves" in line)
    assert shown.endswith(reason.strip())
    assert "0.750 of every wanted value resolved" in body
    assert "other: 1 unresolved, 0.750 of this want resolved" in body
