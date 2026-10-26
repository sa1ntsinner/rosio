import pytest

from rosio import ids, join, model


@pytest.mark.parametrize("other_profile", [False, True])
def test_join_retires_only_remap_blind_spots_it_bound_in_every_profile(other_profile):
    builder = model.AtlasBuilder()
    profile = builder.add_profile("read")
    node = ids.build_node("/worker")
    site = ids.build_launchent("fixture", "launch/run.py", (("Node", "worker"),))
    executable = ids.build_exe("fixture", "worker")
    file = ids.build_file("fixture", "worker.cpp")
    symbol = ids.build_sym("fixture", "worker.cpp", "Worker::start")
    builder.node(node, name_source="literal")
    builder.node(file, attrs={"language": "cpp"})
    builder.node(executable)
    builder.fact(
        symbol,
        "coded",
        values={
            "endpoints": [{"kind": "publisher", "name": "raw", "owner_symbol": "Worker::start"}]
        },
    )
    builder.edge("runs", executable, symbol, layer="declared")
    profiles = [profile]
    if other_profile:
        profiles.append(builder.add_profile("unread"))
    for scope in profiles:
        builder.fact(site, "declared", profile=scope, values={"remaps": [["raw", "resolved"]]})
        builder.edge(
            "launches", site, node, key=ids.edge_key(profile=scope), layer="declared", profile=scope
        )
    builder.fact(
        node,
        "declared",
        profile=profile,
        values={
            "pkg": "fixture",
            "executable": "worker",
            "name": "worker",
            "namespace": "/",
            "remaps": [["raw", "resolved"]],
        },
    )
    builder.unresolved(
        model.Unresolved(
            "other",
            site,
            "raw:=resolved",
            "kind_unknown",
            role="remap",
            evidence=[model.Source(file)],
        )
    )
    join.join(builder)
    assert len(builder.atlas.unresolved) == int(other_profile)
    assert any(
        edge.kind == "publishes" and edge.dst == ids.build_topic("/resolved")
        for edge in builder.atlas.edges.values()
    )
    before = builder.atlas.canonical_bytes()
    join.join(builder)
    assert builder.atlas.canonical_bytes() == before
