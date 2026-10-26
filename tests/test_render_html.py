"""atlas.html: one self contained file, deterministic, no network at run time."""

from __future__ import annotations

import json
import re

import pytest

from rosio import ids, model
from rosio.errors import ContractError
from rosio.model import Atlas, AtlasBuilder, Candidate, Finding, Source
from rosio.render import html

LOCATOR = "planner_server.ros__parameters.GridBased.cost_penalty"


def _profile_run(builder: AtlasBuilder, slug: str) -> tuple[str, str]:
    profile = ids.build_profile(slug)
    run = builder.add_run("declared", profile=profile, input_digest="a1b2c3d4")
    builder.declare_layer("declared", present=True, run=run)
    builder.fact(profile, "declared", profile=profile, run=run, values={"declared_nodes": 1})
    return profile, run


@pytest.fixture()
def atlas() -> Atlas:
    builder = AtlasBuilder()
    profile, run = _profile_run(builder, "nx_navigation")
    other, other_run = _profile_run(builder, "nx_sim")

    package = ids.build_pkg("nx_bringup")
    base = ids.build_file("nx_bringup", "config/nav2_params.yaml")
    overlay = ids.build_file("nx_bringup", "config/nav2_overrides.yaml")
    node = ids.build_node("/planner_server")
    param = ids.build_param(node, "GridBased.cost_penalty")
    topic = ids.build_topic("/plan")

    builder.fact(package, "declared", profile=profile, run=run, values={"name": "nx_bringup"})
    for file_id in (base, overlay):
        builder.fact(file_id, "declared", profile=profile, run=run, values={"loadable": True})
        builder.edge("contains", package, file_id, layer="declared", profile=profile, run=run)
    where = {"executable": "planner_server"}
    builder.fact(node, "declared", profile=profile, run=run, values=where)
    builder.fact(node, "coded", profile=profile, run=run, values=where)
    builder.node(node).name_source = "literal"
    builder.fact(topic, "coded", profile=profile, run=run, values={"type": "nav_msgs/msg/Path"})
    builder.edge(
        "publishes",
        node,
        topic,
        key=ids.edge_key(site="plan"),
        layer="coded",
        profile=profile,
        run=run,
    )
    builder.fact(param, "declared", profile=profile, run=run, values={"value": 1.0})
    builder.edge("has_param", node, param, layer="declared", profile=profile, run=run)
    builder.fact(param, "declared", profile=other, run=other_run, values={"value": 1.3})
    builder.set_param(
        param,
        state="effective",
        candidates=[
            Candidate(
                value=1.0,
                value_type="float",
                mechanism="yaml_overlay_merge",
                source=Source(overlay, LOCATOR, 154),
                rank=0,
                wins=True,
                profile=profile,
            ),
            Candidate(
                value=2.0,
                value_type="float",
                mechanism="yaml_base",
                source=Source(base, LOCATOR, 381),
                rank=1,
                wins=False,
                profile=profile,
                shadow_kind="value_changed",
                shadowed_by=0,
            ),
        ],
    )
    builder.finding(
        Finding(
            rule="param.shadowed",
            severity="warn",
            subject=param,
            title="cost_penalty at nav2_params.yaml:381 is overridden",
            detail="the overlay sets 1.0",
            fix="delete the overlay line or the base line",
            say="In profile nx_navigation, /planner_server GridBased.cost_penalty is 1.0.",
            profiles=[profile],
            where=[Source(base, LOCATOR, 381)],
        )
    )
    return builder.build()


def _index_payload(page: str) -> dict:
    pattern = r'<script type="application/json" id="rosio-index">(.*?)</script>'
    match = re.search(pattern, page, re.S)
    assert match is not None
    return json.loads(match.group(1).replace("\\u003c", "<"))


def _detail_payload(page: str) -> dict:
    pattern = r'<script type="application/json" id="rosio-detail">(.*?)</script>'
    match = re.search(pattern, page, re.S)
    assert match is not None
    return json.loads(match.group(1).replace("\\u003c", "<"))


def _markup(page: str) -> str:
    """The page with the two data blocks removed. A url inside a parameter value is data,
    not a reference, and the reference test has to be able to tell the two apart."""
    return re.sub(r'<script type="application/json".*?</script>', "", page, flags=re.S)


def test_the_page_is_one_file_with_nothing_fetched(atlas: Atlas) -> None:
    page = html.render(atlas)
    assert page.startswith("<!doctype html>")
    assert page.rstrip().endswith("</body></html>")
    markup = _markup(page)
    for forbidden in ("http://", "https://", "//cdn", "fetch(", "XMLHttpRequest", "import("):
        assert forbidden not in markup
    assert page.count("<script") == page.count("</script>")
    assert "src=" not in markup.replace("data-src", "")


def test_a_url_in_the_data_stays_data(atlas: Atlas) -> None:
    """The reference stack configures an opc ua endpoint, so a url reaches the page as a
    parameter value. It must not become a reference the browser follows."""
    builder = AtlasBuilder()
    profile, run = _profile_run(builder, "nx_navigation")
    node = ids.build_node("/io_bridge")
    builder.fact(
        node,
        "declared",
        profile=profile,
        run=run,
        values={
            "endpoint": "opc.tcp://plant.example.invalid:4840",
            "doc": "http://example.invalid/x",
        },
    )
    page = html.render(builder.build())
    assert "http://" in page
    assert "http://" not in _markup(page)
    assert _detail_payload(page)["entities"][node]["facts"][0]["values"]["doc"] == (
        "http://example.invalid/x"
    )


def test_the_same_atlas_renders_the_same_bytes(atlas: Atlas) -> None:
    first = html.render(atlas).encode("utf-8")
    second = html.render(atlas).encode("utf-8")
    assert first == second


def test_the_index_carries_every_entity_and_every_edge(atlas: Atlas) -> None:
    index = _index_payload(html.render(atlas))
    assert len(index["entities"]["id"]) == len(atlas.entities)
    assert len(index["edges"]) == 4 * len(atlas.edges)
    assert index["entities"]["id"] == sorted(atlas.entities)
    for column in index["entities"].values():
        assert len(column) == len(atlas.entities)


def test_every_entity_has_a_coordinate_and_clusters_do_not_collapse(atlas: Atlas) -> None:
    index = _index_payload(html.render(atlas))
    points = set(zip(index["entities"]["x"], index["entities"]["y"], strict=True))
    assert len(points) > 1
    assert all(isinstance(value, (int, float)) for value in index["entities"]["x"])


def test_the_detail_block_holds_the_bulk_and_the_index_does_not(atlas: Atlas) -> None:
    page = html.render(atlas)
    index = _index_payload(page)
    detail = _detail_payload(page)
    param = ids.build_param(ids.build_node("/planner_server"), "GridBased.cost_penalty")
    assert "facts" not in index["entities"]
    assert detail["entities"][param]["candidates"][0]["wins"] is True
    assert detail["entities"][param]["facts"]
    assert "id" not in detail["entities"][param]


def test_the_candidate_chain_keeps_its_order_and_its_provenance(atlas: Atlas) -> None:
    detail = _detail_payload(html.render(atlas))
    param = ids.build_param(ids.build_node("/planner_server"), "GridBased.cost_penalty")
    chain = detail["entities"][param]["candidates"]
    assert [row["rank"] for row in chain] == [0, 1]
    assert chain[0]["wins"] is True
    assert chain[1]["shadow_kind"] == "value_changed"
    assert chain[1]["source"]["line_hint"] == 381
    assert chain[1]["source"]["file"].startswith("file:")


def test_findings_are_first_class_and_carry_their_subjects(atlas: Atlas) -> None:
    index = _index_payload(html.render(atlas))
    assert len(index["findings"]) == 1
    finding = index["findings"][0]
    assert finding["severity"] == "warn"
    assert finding["rule"] == "param.shadowed"
    assert finding["where"][0]["file"] == "nx_bringup/config/nav2_params.yaml"
    assert finding["where"][0]["line"] == 381
    subject = index["entities"]["id"][finding["subjects"][0]]
    assert subject.startswith("param:")


def test_an_entity_that_carries_a_finding_is_marked_by_severity(atlas: Atlas) -> None:
    index = _index_payload(html.render(atlas))
    param = ids.build_param(ids.build_node("/planner_server"), "GridBased.cost_penalty")
    position = index["entities"]["id"].index(param)
    assert index["entities"]["finding"][position] == html.SEVERITY_RANK["warn"]
    node = index["entities"]["id"].index(ids.build_node("/planner_server"))
    assert index["entities"]["finding"][node] == 0


def test_layer_presence_and_conflicts_reach_the_index(atlas: Atlas) -> None:
    index = _index_payload(html.render(atlas))
    names = index["layer_names"]
    position = index["entities"]["id"].index(ids.build_node("/planner_server"))
    mask = index["entities"]["layers"][position]
    assert all(mask & (1 << names.index(layer)) for layer in ("declared", "coded"))
    file_position = index["entities"]["id"].index(
        ids.build_file("nx_bringup", "config/nav2_params.yaml")
    )
    assert index["entities"]["layers"][file_position] == 1 << names.index("declared")


def test_a_parameter_carries_its_shadowed_flag_and_a_short_value(atlas: Atlas) -> None:
    index = _index_payload(html.render(atlas))
    param = ids.build_param(ids.build_node("/planner_server"), "GridBased.cost_penalty")
    position = index["entities"]["id"].index(param)
    assert index["entities"]["flags"][position] & 4
    assert index["entities"]["value"][position] == "1.0"


def _two_profile_param() -> tuple[Atlas, str, list[str]]:
    """One parameter, two profiles, two different winning values. This is the shape 104
    parameters on the reference stack have."""
    builder = AtlasBuilder()
    real, real_run = _profile_run(builder, "nx_navigation")
    sim, sim_run = _profile_run(builder, "nx_sim")
    node = ids.build_node("/collision_monitor")
    param = ids.build_param(node, "FootprintApproach.time_before_collision")
    base = ids.build_file("nx_bringup", "config/collision_monitor.yaml")
    sim_base = ids.build_file("nx_bringup", "config/params_sim.yaml")
    builder.fact(param, "declared", profile=real, run=real_run, values={"value": 2.5})
    builder.fact(param, "declared", profile=sim, run=sim_run, values={"value": 1.0})
    builder.set_param(
        param,
        state="effective",
        candidates=[
            Candidate(
                value=2.5,
                value_type="float",
                mechanism="launch_params_file",
                source=Source(base, "collision_monitor.ros__parameters", 79),
                rank=0,
                wins=True,
                profile=real,
            )
        ],
    )
    builder.set_param(
        param,
        state="effective",
        candidates=[
            Candidate(
                value=1.0,
                value_type="float",
                mechanism="yaml_base",
                source=Source(sim_base, "collision_monitor.ros__parameters", 595),
                rank=0,
                wins=True,
                profile=sim,
            ),
            Candidate(
                value=2.5,
                value_type="float",
                mechanism="launch_params_file",
                source=Source(base, "collision_monitor.ros__parameters", 79),
                rank=1,
                wins=False,
                profile=sim,
                shadow_kind="value_changed",
                shadowed_by=0,
            ),
        ],
    )
    return builder.build(), param, [sim, real]


def test_a_parameter_with_two_profile_values_names_neither_as_the_value() -> None:
    """Naming one of them is a wrong number on the profile the reader has selected."""
    built, param, profiles = _two_profile_param()
    index = _index_payload(html.render(built))
    position = index["entities"]["id"].index(param)
    assert index["entities"]["value"][position] == ""
    slots = index["value_by_profile"][str(position)]
    assert [row["id"] for row in index["profiles"]] == sorted(profiles)
    assert slots[[row["id"] for row in index["profiles"]].index(profiles[1])] == "2.5"
    assert slots[[row["id"] for row in index["profiles"]].index(profiles[0])] == "1.0"


def test_one_value_that_holds_in_every_profile_stays_in_the_column(atlas: Atlas) -> None:
    index = _index_payload(html.render(atlas))
    param = ids.build_param(ids.build_node("/planner_server"), "GridBased.cost_penalty")
    position = index["entities"]["id"].index(param)
    assert index["entities"]["value"][position] == "1.0"
    assert str(position) not in index["value_by_profile"]


def test_the_shadowed_mark_names_the_profiles_it_holds_in() -> None:
    built, param, profiles = _two_profile_param()
    index = _index_payload(html.render(built))
    position = index["entities"]["id"].index(param)
    names = [row["id"] for row in index["profiles"]]
    mask = index["entities"]["shadow"][position]
    assert mask & (1 << names.index(profiles[0]))
    assert not mask & (1 << names.index(profiles[1]))
    assert index["entities"]["flags"][position] & 4


def test_the_observed_and_recorded_layers_reach_the_page() -> None:
    """M5 adds two layers and two entity kinds. Both have to paint, and a bag has to own
    a cluster of its own rather than land on the origin with everything else."""
    builder = AtlasBuilder()
    profile, _run = _profile_run(builder, "nx_navigation")
    live = builder.add_run("observed", profile=profile, input_digest="b2c3d4e5")
    builder.declare_layer("observed", present=True, run=live)
    tape = builder.add_run("recorded", input_digest="c3d4e5f6")
    builder.declare_layer("recorded", present=True, run=tape)

    node = ids.build_node("/controller_server")
    topic = ids.build_topic("/cmd_vel_nav")
    bag = ids.build_bag("yard_20260813_1624")
    bagtopic = ids.build_bagtopic("yard_20260813_1624", "/cmd_vel_nav")
    builder.fact(node, "observed", profile=profile, run=live, values={"lifecycle": "active"})
    twist = {"type": "geometry_msgs/msg/Twist"}
    builder.fact(topic, "observed", profile=profile, run=live, values=twist)
    builder.fact(bag, "recorded", run=tape, values={"storage": "sqlite3"})
    builder.fact(
        bagtopic,
        "recorded",
        run=tape,
        values=dict(twist, volatile={"message_count": 12034}),
    )
    builder.edge("contains", bag, bagtopic, layer="recorded", run=tape)
    builder.edge("samples", bagtopic, topic, layer="recorded", run=tape)

    index = _index_payload(html.render(builder.build()))
    assert "observed" in index["layer_names"] and "recorded" in index["layer_names"]
    assert "bag" in index["kinds"] and "bagtopic" in index["kinds"]
    bag_position = index["entities"]["id"].index(bag)
    assert index["entities"]["radius"][bag_position] > 0
    points = set(zip(index["entities"]["x"], index["entities"]["y"], strict=True))
    assert len(points) == len(index["entities"]["id"])
    names = index["layer_names"]
    node_position = index["entities"]["id"].index(node)
    assert index["entities"]["layers"][node_position] & (1 << names.index("observed"))


def test_a_plugin_parameter_with_no_value_shows_no_value() -> None:
    """A `pparam` effective block carries `base_class`, `lookup` and `suffix` and no
    value, 266 of them on the reference stack. Any of those three in the value column
    reads as the parameter's value, which is a number the reader cannot act on."""
    builder = AtlasBuilder()
    profile, run = _profile_run(builder, "nx_navigation")
    plugin = ids.build_plugin("nav2_core::Controller", "nx_controller/DelayAware")
    pparam = ids.build_pparam(
        "nav2_core::Controller", "nx_controller/DelayAware", "allow_reversing"
    )
    builder.fact(plugin, "declared", profile=profile, run=run, values={"lookup": "DelayAware"})
    builder.fact(
        pparam,
        "coded",
        profile=profile,
        run=run,
        values={"base_class": "nav2_core::Controller", "suffix": "allow_reversing"},
    )
    builder.set_param(pparam, state="effective")
    index = _index_payload(html.render(builder.build()))
    position = index["entities"]["id"].index(pparam)
    assert index["entities"]["value"][position] == ""
    assert str(position) not in index["value_by_profile"]


def test_a_symbol_is_never_rendered_as_a_directory() -> None:
    """`applied_by` is a sym id, 918 candidate rows on the reference stack. Joining the
    qualified name onto the path with a slash names a directory that does not exist."""
    script = (html.ASSETS / "atlas.js").read_text(encoding="utf-8")
    assert "esc(symText(row.applied_by))" in script
    assert "fpath(row.applied_by)" not in script
    assert 'if (text.indexOf("sym:") !== 0 || parts.length < 3) return fpath(text);' in script


def test_a_fact_stated_under_every_profile_survives_the_profile_filter() -> None:
    """866 of the 2883 entities on the reference stack state their facts under `*`, every
    file, every symbol and every type among them. The edge filter already keeps an empty
    mask and a profile slice of the atlas keeps a `*` fact, so the entity filter has to."""
    script = (html.ASSETS / "atlas.js").read_text(encoding="utf-8")
    assert "profiles !== 0 && !bit(profiles, slot)" in script
    builder = AtlasBuilder()
    _profile, run = _profile_run(builder, "nx_navigation")
    shared = ids.build_type("nx_msgs", "msg", "RobotStatus")
    builder.fact(shared, "declared", profile=model.ANY_PROFILE, run=run, values={"fields": 4})
    index = _index_payload(html.render(builder.build()))
    position = index["entities"]["id"].index(shared)
    assert index["entities"]["profiles"][position] == 0


def test_profiles_and_states_are_named_once_and_referenced_by_slot(atlas: Atlas) -> None:
    index = _index_payload(html.render(atlas))
    assert [row["id"] for row in index["profiles"]] == sorted(
        entity for entity in atlas.entities if entity.startswith("profile:")
    )
    assert "effective" in index["states"]


def test_the_page_survives_an_empty_atlas() -> None:
    page = html.render(Atlas())
    assert "<canvas" in page
    index = _index_payload(page)
    assert index["entities"]["id"] == []
    assert index["edges"] == []


def test_write_puts_atlas_html_where_it_was_asked(atlas: Atlas, tmp_path) -> None:
    target = html.write(atlas, tmp_path / "out")
    assert target.name == "atlas.html"
    assert target.read_text(encoding="utf-8") == html.render(atlas)


def test_the_embedded_json_can_never_close_the_script_tag() -> None:
    builder = AtlasBuilder()
    profile, run = _profile_run(builder, "nx_navigation")
    node = ids.build_node("/tricky")
    builder.fact(node, "declared", profile=profile, run=run, values={"note": "</script><b>x"})
    page = html.render(builder.build())
    assert "</script><b>" not in page
    assert _detail_payload(page)["entities"][node]["facts"][0]["values"]["note"] == "</script><b>x"


def test_the_assets_are_inlined_not_referenced() -> None:
    page = html.render(Atlas())
    assert "<style>" in page and "atlas.css" not in page
    assert "canvas" in page and "atlas.js" not in page


def test_a_missing_asset_raises_a_rosio_error(monkeypatch) -> None:
    monkeypatch.setattr(html, "ASSETS", html.ASSETS.parent / "no-such-directory")
    with pytest.raises(ContractError):
        html.render(Atlas())


def test_the_compaction_preserves_the_document(atlas: Atlas) -> None:
    payload = atlas.to_json()
    assert json.loads(html._compact(model.dumps(payload))) == json.loads(model.dumps(payload))


def test_file_ids_read_as_paths() -> None:
    assert html._file_path(ids.build_file("nx_perception", "src/node.cpp")) == (
        "nx_perception/src/node.cpp"
    )
    assert html._file_path(ids.build_file("@repo", "dx_app/launcher.py")) == ("dx_app/launcher.py")
    assert html._file_path("sym:pkg|src/a.cpp|Klass::method").startswith("sym:")
