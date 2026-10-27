"""One executable that runs two nodes, each named by the class it constructs.

The fixture is `tests/fixtures/ws_multinode`: `pair` constructs `Alpha` (`Node("alpha")`) and
`Beta` (`Node("beta")`) and spins both in one executor. `pair.launch.py` starts it with no
name, so the process runs `/alpha` and `/beta`, and every parameter file of the launch site
reaches both. `named.launch.py` gives the site `name="pair"`, which rclcpp applies to every
node of the process, so the two collide under one name.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from rosio import extract, ids, join, model, schema
from rosio.config import Config, Entrypoint, ProfileSpec, RootSpec
from rosio.resolve import yaml_lines
from rosio.workspace import Workspace

FIXTURE = Path(__file__).parent / "fixtures" / "ws_multinode"
PROFILE = "profile:pair"
NAMED = "profile:named"


def _extract(tmp_path: Path, slug: str) -> model.Atlas:
    roots = (RootSpec(kind="src", path=str(FIXTURE / "src")),)
    cfg = Config(
        roots=roots,
        profiles=(
            ProfileSpec(
                slug=slug,
                entrypoints=(
                    Entrypoint(verb="launch", pkg="two_nodes", target=f"{slug}.launch.py"),
                ),
            ),
        ),
        out_dir=str(tmp_path / "out"),
        deterministic=True,
    )
    atlas = extract.run(cfg, Workspace.discover(roots))
    extract.drift(atlas)
    return atlas


def _edges(atlas: model.Atlas, node: str, kind: str) -> set[str]:
    return {edge.dst for edge in atlas.out_edges(node) if edge.kind == kind}


def _winner(atlas: model.Atlas, param: str) -> model.Candidate:
    entity = atlas.entities[param]
    assert entity.state == model.ParamState.EFFECTIVE.value
    return next(row for row in entity.candidates if row.profile == PROFILE and row.wins)


def test_two_mappings_under_one_key_are_one_mapping():
    document = yaml_lines.load_file(str(FIXTURE / "src/two_nodes/config/pair.yaml"))
    assert {block.selector for block in yaml_lines.param_blocks(document)} == {
        "/**/alpha",
        "/**/beta",
    }


def _split_atlas(nodes: list[dict]) -> model.AtlasBuilder:
    builder = model.AtlasBuilder()
    source = ids.build_file("p", "main.cpp")
    main = ids.build_sym("p", "main.cpp", "main")
    builder.node(source)
    builder.node(main)
    builder.edge("defines", source, main, layer="coded")
    builder.node("profile:p")
    builder.node("node:~p|pipe|/|0000aaaa", name_source="executable_default")
    builder.fact(
        "node:~p|pipe|/|0000aaaa",
        "declared",
        profile="profile:p",
        values={"pkg": "p", "executable": "pipe", "namespace": "/"},
    )
    builder.fact(
        main,
        "coded",
        values={
            "nodes": nodes,
            "endpoints": [
                {
                    "kind": "publisher",
                    "name": "one",
                    "construction": "a",
                    "owner_symbol": "Pipe::Pipe",
                },
                {
                    "kind": "publisher",
                    "name": "two",
                    "construction": "b",
                    "owner_symbol": "Pipe::Pipe",
                },
            ],
        },
    )
    builder.node(ids.build_exe("p", "pipe"))
    builder.edge("runs", ids.build_exe("p", "pipe"), main, layer="declared")
    return builder


def test_two_constructions_of_one_class_are_two_nodes_with_their_own_records():
    builder = _split_atlas(
        [
            {"name": "first", "construction": "a", "class": "Pipe", "name_source": "literal"},
            {"name": "second", "construction": "b", "class": "Pipe", "name_source": "literal"},
        ]
    )
    join.join(builder)
    atlas = builder.atlas
    assert _edges(atlas, "node:/first", "publishes") == {"topic:/one"}
    assert _edges(atlas, "node:/second", "publishes") == {"topic:/two"}
    assert not [key for key in atlas.entities if key.startswith("node:~")]
    assert not atlas.findings
    assert schema.validate(builder.build().to_json()) == []
    before = atlas.canonical_bytes()
    join.join(builder)
    assert builder.build().canonical_bytes() == before


def test_a_name_built_at_run_time_keeps_the_process_one_identity():
    builder = _split_atlas(
        [
            {"name": "first", "construction": "a", "class": "Pipe", "name_source": "literal"},
            {"name": "second", "construction": "b", "class": "Pipe", "name_source": "argv"},
        ]
    )
    join.join(builder)
    assert "node:~p|pipe|/|0000aaaa" in builder.atlas.entities
    assert "node:/first" not in builder.atlas.entities


@pytest.mark.ros
@pytest.mark.cpp
def test_an_unnamed_site_starts_one_node_per_name_the_source_states(tmp_path):
    atlas = _extract(tmp_path, "pair")
    nodes = sorted(key for key in atlas.entities if key.startswith("node:"))
    assert nodes == ["node:/alpha", "node:/beta"]
    for node in nodes:
        assert atlas.entities[node].name_source == model.NameSource.LITERAL.value
    assert _edges(atlas, "node:/alpha", "publishes") == {"topic:/chatter"}
    assert not _edges(atlas, "node:/alpha", "subscribes")
    assert _edges(atlas, "node:/beta", "subscribes") == {"topic:/chatter"}
    assert not _edges(atlas, "node:/beta", "publishes")
    assert _winner(atlas, "param:/alpha|rate").value == 20
    assert _winner(atlas, "param:/beta|gain").value == 2.5
    assert "param:/alpha|gain" not in atlas.entities
    assert "param:/beta|rate" not in atlas.entities
    assert not [item for item in atlas.findings if item.rule.startswith("node.")]
    assert not [item for item in atlas.findings if item.rule.startswith("param.")]
    assert not [row for row in atlas.unresolved if row.want == model.Want.NODE_NAME.value]


@pytest.mark.ros
@pytest.mark.cpp
def test_a_launch_name_on_that_process_is_a_collision(tmp_path):
    atlas = _extract(tmp_path, "named")
    assert "node:/pair" in atlas.entities
    found = [item for item in atlas.findings if item.rule == "node.ambiguous_identity"]
    assert len(found) == 1
    finding = found[0]
    assert finding.subject == "node:/pair"
    assert "alpha, beta" in finding.detail
    assert "alpha:__node:=" in finding.fix
    assert "name the node" not in finding.fix
    assert "launch/named.launch.py" in finding.say


@pytest.mark.ros
@pytest.mark.cpp
@pytest.mark.parametrize("slug", ["pair", "named"])
def test_the_fixture_extracts_twice_into_the_same_valid_atlas(tmp_path, slug):
    first = _extract(tmp_path / "first", slug)
    second = _extract(tmp_path / "second", slug)
    assert schema.validate(first.to_json()) == []
    assert first.canonical_bytes() == second.canonical_bytes()
