"""urdf and xacro: frames, joint geometry, gazebo plugin remaps and the refusals."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest

from rosio import ids, model, schema
from rosio.collect import CollectContext
from rosio.collect import urdf as urdf_collect
from rosio.config import Config, RootSpec
from rosio.errors import CollectorError
from rosio.workspace import Workspace

VARIANTS = {
    "real": "src/fx_desc/urdf/fx_robot.urdf",
    "sim": "src/fx_desc/urdf/fx_robot_sim.urdf",
    "sim_trailer": "src/fx_desc/urdf/fx_robot_sim_trailer.urdf",
    "broken": "src/fx_desc/urdf/fx_broken.urdf",
}


@pytest.fixture(scope="module")
def static_root(fixtures_dir: Path) -> Path:
    return fixtures_dir / "ws_static"


@pytest.fixture(scope="module")
def ws(static_root: Path) -> Workspace:
    return Workspace.discover([RootSpec("src", str(static_root / "src"))])


@pytest.fixture(scope="module")
def expected(fixtures_dir: Path):
    def _load(name: str) -> dict[str, Any]:
        return json.loads((fixtures_dir / "expected" / "static" / name).read_text(encoding="utf-8"))

    return _load


def _model(static_root: Path, ws: Workspace, variant: str) -> urdf_collect.UrdfModel:
    return urdf_collect.parse_urdf_file(
        str(static_root / VARIANTS[variant]), resolve_package=ws.share
    )


def _collect(static_root: Path, ws: Workspace) -> model.Atlas:
    roots = (RootSpec("src", str(static_root / "src")),)
    builder = model.AtlasBuilder()
    ctx = CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder)
    urdf_collect.UrdfCollector().collect(ctx)
    return builder.build()


# ---------------------------------------------------------------------------
# parsing


@pytest.mark.parametrize("variant", sorted(VARIANTS))
def test_frame_tree_matches_the_expected_file(
    static_root: Path, ws: Workspace, expected, variant: str
) -> None:
    want = expected("frames.json")["variants"][variant]
    got = _model(static_root, ws, variant).view(VARIANTS[variant])
    for key in sorted(want):
        assert got[key] == want[key], key


def test_the_three_variants_share_one_robot_name(static_root: Path, ws: Workspace) -> None:
    names = {variant: _model(static_root, ws, variant).robot_name for variant in VARIANTS}
    assert names["real"] == names["sim"] == names["sim_trailer"] == "fx_robot"


def test_variant_differences_match_the_expected_file(
    static_root: Path, ws: Workspace, expected
) -> None:
    want = expected("urdf_diff.json")
    models = {
        variant: _model(static_root, ws, variant) for variant in ("real", "sim", "sim_trailer")
    }
    got = urdf_collect.diff_models(models)
    for key in sorted(want):
        if key == "note":
            continue
        assert got[key] == want[key], key


def test_a_reparented_joint_is_not_compared_by_its_origin(static_root: Path, ws: Workspace) -> None:
    models = {
        variant: _model(static_root, ws, variant) for variant in ("real", "sim", "sim_trailer")
    }
    diff = urdf_collect.diff_models(models)
    assert "imu_joint" in diff["joint_reparented"]
    assert "imu_joint" not in diff["joint_origin_differs"]


def test_the_broken_model_reports_instead_of_inventing(static_root: Path, ws: Workspace) -> None:
    found = _model(static_root, ws, "broken")
    assert found.undeclared_parent_links() == ["ghost_link"]
    assert found.orphan_children() == ["probe_link"]
    assert found.roots() == ["base_footprint", "ghost_link"]
    assert "ghost_link" not in found.links


def test_a_missing_mesh_is_recorded(static_root: Path, ws: Workspace) -> None:
    meshes = {mesh.filename: mesh for mesh in _model(static_root, ws, "broken").meshes}
    assert list(meshes) == ["package://fx_desc/meshes/fx_absent_body.stl"]
    assert not next(iter(meshes.values())).exists
    assert _model(static_root, ws, "real").meshes[0].exists


def test_a_non_robot_document_is_refused() -> None:
    with pytest.raises(CollectorError):
        urdf_collect.parse_urdf_text("<sdf><world/></sdf>", path="x.urdf")


def test_discovery_filters_by_extension(static_root: Path, ws: Workspace, expected) -> None:
    want = expected("xacro.json")["discovery"]
    urdfs = [str(Path(path).relative_to(static_root)) for path in ws.source_files([".urdf"])]
    xacros = [str(Path(path).relative_to(static_root)) for path in ws.source_files([".xacro"])]
    assert urdfs == want["urdf_files"]
    assert xacros == want["xacro_files"]
    for ignored in want["ignored"]:
        assert ignored not in urdfs + xacros


# ---------------------------------------------------------------------------
# xacro


def test_xacro_static_scan_matches_the_expected_file(
    static_root: Path, ws: Workspace, expected
) -> None:
    files = expected("xacro.json")["files"]
    for relative, want in files.items():
        got = urdf_collect.scan_xacro(str(static_root / relative), resolve_package=ws.share).view(
            relative
        )
        for key in sorted(want):
            if key in ("expanded", "expanded_with", "needs_arguments_note"):
                continue
            if key == "includes":
                got[key] = [
                    dict(
                        row,
                        resolved=(
                            str(Path(row["resolved"]).relative_to(static_root))
                            if row["resolved"]
                            else None
                        ),
                    )
                    for row in got[key]
                ]
            assert got[key] == want[key], f"{relative} {key}"


def test_an_undeclared_xacro_argument_is_named(static_root: Path, ws: Workspace) -> None:
    scan = urdf_collect.scan_xacro(str(static_root / "src/fx_desc/xacro/fx_needs_arg.urdf.xacro"))
    assert scan.needs_arguments == ("sensor_height",)
    assert scan.kind == "model"


def test_a_missing_xacro_tool_refuses_with_a_reason(
    static_root: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(urdf_collect.shutil, "which", lambda _name: None)
    assert not urdf_collect.xacro_available()
    with pytest.raises(CollectorError) as caught:
        urdf_collect.expand_xacro(str(static_root / "src/fx_desc/xacro/fx_robot.urdf.xacro"))
    assert "xacro tool" in str(caught.value)


@pytest.mark.ros
def test_xacro_expansion_matches_the_expected_file(static_root: Path, expected) -> None:
    files = expected("xacro.json")["files"]
    relative = "src/fx_desc/xacro/fx_robot.urdf.xacro"
    want = files[relative]["expanded"]
    found = urdf_collect.parse_xacro_file(str(static_root / relative))
    assert found.strategy == "xacro"
    assert sorted(found.links) == want["links"]
    assert found.roots() == want["roots"]
    for name, joint in want["joints"].items():
        got = found.joint_map()[name].view()
        for key in sorted(joint):
            assert got[key] == joint[key], f"{name} {key}"


@pytest.mark.ros
def test_xacro_needs_its_argument_supplied(static_root: Path, expected) -> None:
    relative = "src/fx_desc/xacro/fx_needs_arg.urdf.xacro"
    want = expected("xacro.json")["files"][relative]["expanded_with"]
    with pytest.raises(CollectorError) as caught:
        urdf_collect.parse_xacro_file(str(static_root / relative))
    assert "sensor_height" in str(caught.value)
    args = tuple(want["args"].items())
    found = urdf_collect.parse_xacro_file(str(static_root / relative), args)
    assert sorted(found.links) == want["links"]
    assert found.joint_map()["sensor_joint"].xyz == tuple(want["joints"]["sensor_joint"]["xyz"])
    assert found.xacro_args == args


# ---------------------------------------------------------------------------
# the collector


def test_the_atlas_validates_and_is_stable(static_root: Path, ws: Workspace) -> None:
    first = _collect(static_root, ws)
    assert schema.validate(first.to_json()) == []
    second = _collect(static_root, Workspace.discover([RootSpec("src", str(static_root / "src"))]))
    assert first.canonical_bytes() == second.canonical_bytes()


def test_only_declared_links_become_frames(static_root: Path, ws: Workspace) -> None:
    atlas = _collect(static_root, ws)
    frames = {node.id for node in atlas.by_kind("frame")}
    assert ids.build_frame("base_link") in frames
    assert ids.build_frame("ghost_link") not in frames
    assert ids.build_frame("probe_link") in frames


def test_an_unresolved_parent_is_a_blind_spot(static_root: Path, ws: Workspace) -> None:
    atlas = _collect(static_root, ws)
    reasons = {entry.reason for entry in atlas.unresolved}
    assert "parent_link_never_declared" in reasons
    assert "parent_frame_unresolved" in reasons
    owner = ws.file_id(str(static_root / VARIANTS["broken"]))
    assert atlas.entities[owner].unresolved


def test_tf_parent_carries_the_model_in_its_key_and_values(
    static_root: Path, ws: Workspace
) -> None:
    atlas = _collect(static_root, ws)
    child = ids.build_frame("sensor_link")
    parent = ids.build_frame("base_link")
    models = {
        ws.file_id(str(static_root / VARIANTS[name])) for name in ("real", "sim", "sim_trailer")
    }
    edges = [
        edge
        for edge in atlas.out_edges(child)
        if edge.kind == model.EdgeKind.TF_PARENT.value
        and edge.dst == parent
        and edge.facts[0].values["model"] in models
    ]
    assert len(edges) == 3
    assert len({edge.key for edge in edges}) == 3
    origins = set()
    for edge in edges:
        fact = edge.facts[0]
        assert fact.values["joint"] == "sensor_joint"
        origins.add(tuple(fact.values["xyz"]))
    assert len(origins) == 3


def test_a_gazebo_plugin_block_is_a_remap_source(static_root: Path, ws: Workspace) -> None:
    atlas = _collect(static_root, ws)
    source = ws.file_id(str(static_root / VARIANTS["sim"]))
    remaps = [edge for edge in atlas.out_edges(source) if edge.kind == model.EdgeKind.REMAPS.value]
    targets = {edge.dst for edge in remaps}
    assert ids.build_topic("/fx/cmd_vel_sim") in targets
    assert ids.build_topic("/fx/points") in targets
    for edge in remaps:
        assert edge.facts[0].values["scope"] == "urdf_plugin"
        assert edge.key


def test_the_trailer_lidar_stamps_the_ray_frame(static_root: Path, ws: Workspace) -> None:
    plugins = {plugin.name: plugin for plugin in _model(static_root, ws, "sim_trailer").gazebo}
    assert plugins["fx_lidar_plugin"].frame_name == "sensor_ray"
    assert plugins["fx_lidar_plugin"].reference == "sensor_link"
    base = {plugin.name: plugin for plugin in _model(static_root, ws, "sim").gazebo}
    assert base["fx_lidar_plugin"].frame_name == "sensor_link"


def test_a_number_that_is_not_a_number_is_not_read_as_zero() -> None:
    text = (
        '<robot name="probe"><link name="a"/><link name="b"/>'
        '<joint name="j" type="revolute"><parent link="a"/><child link="b"/>'
        '<origin xyz="${offset} 0 0.5" rpy="0 0 0"/>'
        '<limit effort="${eff}" velocity="1.0"/></joint></robot>'
    )
    joint = urdf_collect.parse_urdf_text(text, path="probe.urdf").joints[0]
    assert dict(joint.unparsed) == {"limit.effort": "${eff}", "origin.xyz": "${offset} 0 0.5"}
    assert dict(joint.limit) == {"velocity": 1.0}
    assert joint.origin_view()["xyz"] == urdf_collect.UNKNOWN
    assert joint.origin_view()["rpy"] == [0.0, 0.0, 0.0]


@pytest.mark.parametrize("written", ["1 2", "1", "1 2 3 4", ""])
def test_a_vector_that_is_not_three_numbers_is_refused(written: str) -> None:
    """Padding a short vector with zero or dropping the tail of a long one publishes a
    certain looking origin for input that states something else."""
    text = (
        '<robot name="probe"><link name="a"/><link name="b"/>'
        '<joint name="j" type="fixed"><parent link="a"/><child link="b"/>'
        f'<origin xyz="{written}" rpy="0 0 0"/></joint></robot>'
    )
    joint = urdf_collect.parse_urdf_text(text, path="probe.urdf").joints[0]
    if not written:
        # An absent value is the urdf identity, which is a different statement.
        assert joint.unparsed == ()
        assert joint.origin_view()["xyz"] == [0.0, 0.0, 0.0]
        return
    assert dict(joint.unparsed) == {"origin.xyz": written}
    assert joint.origin_view()["xyz"] == urdf_collect.UNKNOWN


def test_a_broken_vector_reaches_the_blind_spot_list(tmp_path: Path) -> None:
    """The refusal has to be visible to a reader of the atlas, not only inside the record."""
    package = tmp_path / "src" / "short_pkg"
    package.mkdir(parents=True)
    (package / "package.xml").write_text(
        "<package format='3'><name>short_pkg</name></package>", encoding="utf-8"
    )
    (package / "short.urdf").write_text(
        '<robot name="short"><link name="a"/><link name="b"/>'
        '<joint name="j" type="fixed"><parent link="a"/><child link="b"/>'
        '<origin xyz="1 2"/></joint></robot>',
        encoding="utf-8",
    )
    roots = (RootSpec("src", str(tmp_path / "src")),)
    workspace = Workspace.discover(roots)
    builder = model.AtlasBuilder()
    urdf_collect.UrdfCollector().collect(
        CollectContext(ws=workspace, cfg=Config(roots=roots, profiles=()), builder=builder)
    )
    atlas = builder.build()
    entry = next(
        entry for entry in atlas.unresolved if entry.reason == "joint_attribute_unreadable"
    )
    assert 'origin.xyz="1 2"' in entry.expr
    edge = next(edge for edge in atlas.edges.values() if edge.kind == "tf_parent")
    assert "xyz" not in edge.facts[0].values
    assert edge.facts[0].values["unreadable"] == ["origin.xyz"]


def test_an_absent_origin_is_the_urdf_identity(static_root: Path, ws: Workspace) -> None:
    joint = _model(static_root, ws, "real").joint_map()["base_joint"]
    assert joint.unparsed == ()
    assert joint.origin_view()["rpy"] == [0.0, 0.0, 0.0]


def test_a_relative_remap_target_needs_a_declared_namespace(
    static_root: Path, ws: Workspace
) -> None:
    roots = (RootSpec("src", str(static_root / "src")),)
    builder = model.AtlasBuilder()
    ctx = CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder)
    collector = urdf_collect.UrdfCollector()
    plugin = urdf_collect.GazeboPlugin(
        name="drive",
        filename="libdrive.so",
        namespace=None,
        remappings=("cmd_vel:=cmd_vel", "odom:=/fx/odom"),
    )
    found = urdf_collect.UrdfModel(
        path=str(static_root / VARIANTS["real"]), robot_name="fx_robot", gazebo=(plugin,)
    )
    file_id = ws.file_id(found.path)
    builder.node(file_id)
    collector._emit_remaps(ctx, None, model.ANY_PROFILE, found.path, file_id, found)
    atlas = builder.build()
    assert [entry.reason for entry in atlas.unresolved] == ["remap_target_namespace_unknown"]
    edges = [edge for edge in atlas.out_edges(file_id) if edge.kind == "remaps"]
    assert [edge.dst for edge in edges] == [ids.build_topic("/fx/odom")]
    assert "namespace" not in edges[0].facts[0].values


def test_a_profile_scoped_run_carries_a_profile_id(static_root: Path, ws: Workspace) -> None:
    roots = (RootSpec("src", str(static_root / "src")),)
    builder = model.AtlasBuilder()
    ctx = CollectContext(
        ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder, profile="sim_trailer"
    )
    urdf_collect.UrdfCollector().collect(ctx)
    atlas = builder.build()
    frame = atlas.entities[ids.build_frame("base_link")]
    assert frame.facts[0].profile == ids.build_profile("sim_trailer")


def test_an_unreadable_xacro_file_does_not_empty_the_layer(
    static_root: Path, tmp_path: Path
) -> None:
    package = tmp_path / "src" / "fx_desc"
    (package / "xacro").mkdir(parents=True)
    (package / "package.xml").write_text(
        (static_root / "src/fx_desc/package.xml").read_text(encoding="utf-8"), encoding="utf-8"
    )
    (package / "xacro" / "torn.urdf.xacro").write_text("<robot name='torn'><link", encoding="utf-8")
    roots = (RootSpec("src", str(tmp_path / "src")),)
    ws = Workspace.discover(roots)
    builder = model.AtlasBuilder()
    urdf_collect.UrdfCollector().collect(
        CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder)
    )
    atlas = builder.build()
    refused = ws.file_id(str(package / "xacro" / "torn.urdf.xacro"))
    assert atlas.entities[refused].attrs["refused_because"] == "xacro_unreadable"
    assert [entry.reason for entry in atlas.unresolved] == ["xacro_unreadable"]
    assert ids.build_edge("contains", ids.build_pkg("fx_desc"), refused) in atlas.edges
    # The fix a refusal offers has to be the fix for that refusal.
    assert atlas.unresolved[0].resolvable_by == ["write the model as parsable xml"]


def test_a_urdf_that_is_not_parsable_xml_still_costs_its_entity(
    static_root: Path, tmp_path: Path
) -> None:
    """It used to leave nothing: no file entity, no blind spot, and the frame want gone
    from the counter, so the atlas read like a workspace with no model in it at all."""
    package = tmp_path / "src" / "fx_desc"
    (package / "urdf").mkdir(parents=True)
    (package / "package.xml").write_text(
        (static_root / "src/fx_desc/package.xml").read_text(encoding="utf-8"), encoding="utf-8"
    )
    (package / "urdf" / "fx_torn.urdf").write_text("<robot name='fx'><link", encoding="utf-8")
    roots = (RootSpec("src", str(tmp_path / "src")),)
    ws = Workspace.discover(roots)
    builder = model.AtlasBuilder()
    urdf_collect.UrdfCollector().collect(
        CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder)
    )
    atlas = builder.build()
    file_id = ws.file_id(str(package / "urdf" / "fx_torn.urdf"))
    entity = atlas.entities[file_id]
    assert entity.attrs["loadable"] is False
    assert entity.attrs["refused_because"] == "urdf_unreadable"
    assert "is not parsable xml" in entity.attrs["refused_detail"]
    assert ids.build_edge("contains", ids.build_pkg("fx_desc"), file_id) in atlas.edges
    entry = next(item for item in atlas.unresolved if item.owner == file_id)
    assert entry.want == model.Want.FRAME_ID.value
    assert entry.reason == "urdf_unreadable"
    assert entry.id in entity.unresolved
    # The want it would have contributed keeps its denominator instead of vanishing.
    assert atlas.counters["resolution_rate"]["frame_id"] == 0.0
    run = atlas.by_kind("run")[0]
    assert [row["file"] for row in run.attrs["exceptions"]] == [file_id]


def test_a_urdf_this_process_cannot_open_leaves_the_same_shape(
    static_root: Path, tmp_path: Path
) -> None:
    package = tmp_path / "src" / "fx_desc"
    (package / "urdf").mkdir(parents=True)
    (package / "package.xml").write_text(
        (static_root / "src/fx_desc/package.xml").read_text(encoding="utf-8"), encoding="utf-8"
    )
    locked = package / "urdf" / "fx_locked.urdf"
    locked.write_text("<robot name='fx'/>", encoding="utf-8")
    locked.chmod(0)
    roots = (RootSpec("src", str(tmp_path / "src")),)
    ws = Workspace.discover(roots)
    builder = model.AtlasBuilder()
    try:
        urdf_collect.UrdfCollector().collect(
            CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder)
        )
    finally:
        locked.chmod(0o600)
    atlas = builder.build()
    entity = atlas.entities[ws.file_id(str(locked))]
    assert entity.attrs["loadable"] is False
    assert entity.attrs["refused_because"] == "urdf_unreadable"
    assert [entry.reason for entry in atlas.unresolved] == ["urdf_unreadable"]


def test_a_refused_xacro_file_is_marked_unloadable(static_root: Path, ws: Workspace) -> None:
    atlas = _collect(static_root, ws)
    refused = ws.file_id(str(static_root / "src/fx_desc/xacro/fx_needs_arg.urdf.xacro"))
    assert atlas.entities[refused].attrs["loadable"] is False
    assert atlas.entities[refused].attrs["refused_because"] == "xacro_argument_missing"


def test_a_non_finite_origin_costs_itself_and_not_the_model(
    static_root: Path, tmp_path: Path
) -> None:
    """One written infinity used to end the whole urdf pass and leave a tf_parent edge with
    no facts, no geometry and frames that named no model."""
    package = tmp_path / "src" / "fx_desc"
    (package / "urdf").mkdir(parents=True)
    (package / "package.xml").write_text(
        (static_root / "src/fx_desc/package.xml").read_text(encoding="utf-8"), encoding="utf-8"
    )
    (package / "urdf" / "fx_inf.urdf").write_text(
        "<robot name='fx_inf'>"
        "<link name='base_link'/><link name='sensor_link'/>"
        "<joint name='base_to_sensor' type='fixed'>"
        "<parent link='base_link'/><child link='sensor_link'/>"
        "<origin xyz='inf 0.2 0.3' rpy='0 0 0'/>"
        "</joint></robot>",
        encoding="utf-8",
    )
    roots = (RootSpec("src", str(tmp_path / "src")),)
    ws = Workspace.discover(roots)
    builder = model.AtlasBuilder()
    urdf_collect.UrdfCollector().collect(
        CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder)
    )
    atlas = builder.build()
    file_id = ws.file_id(str(package / "urdf" / "fx_inf.urdf"))
    edge = next(item for item in atlas.edges.values() if item.kind == "tf_parent")
    values = edge.facts[0].values
    assert values["unreadable"] == ["origin.xyz"]
    assert "xyz" not in values
    assert values["rpy"] == [0.0, 0.0, 0.0]
    assert atlas.entities[ids.build_frame("base_link")].attrs["urdf_models"] == [file_id]
    entry = next(item for item in atlas.unresolved if item.owner == file_id)
    assert entry.expr == 'joint[base_to_sensor] origin.xyz="inf 0.2 0.3"'
    assert entry.reason == f"{model.NON_FINITE}: inf"
    assert entry.id in atlas.entities[file_id].unresolved


def test_an_origin_that_is_not_a_number_keeps_its_own_reason(tmp_path: Path) -> None:
    joint = urdf_collect.parse_urdf_text(
        "<robot name='fx'>"
        "<link name='a'/><link name='b'/>"
        "<joint name='j' type='fixed'><parent link='a'/><child link='b'/>"
        "<origin xyz='left 0 0'/></joint></robot>",
        path=str(tmp_path / "fx.urdf"),
    ).joints[0]
    assert dict(joint.unparsed) == {"origin.xyz": "left 0 0"}
    assert urdf_collect._unparsed_reason("left 0 0") == "joint_attribute_unreadable"
    assert urdf_collect._unparsed_reason("nan 0 0") == f"{model.NON_FINITE}: nan"


def test_a_model_no_package_installed_is_not_part_of_an_install_root(
    static_root: Path, tmp_path: Path
) -> None:
    """jazzy installs a physics engine's test models under `opt/` beside the ROS packages, and
    reading them reported orphan transform trees about robots nobody runs."""
    install = tmp_path / "install"
    share = install / "share" / "fx_desc"
    (share / "urdf").mkdir(parents=True)
    (share / "package.xml").write_text(
        (static_root / "src/fx_desc/package.xml").read_text(encoding="utf-8"), encoding="utf-8"
    )
    robot = (static_root / VARIANTS["real"]).read_text(encoding="utf-8")
    (share / "urdf" / "fx_robot.urdf").write_text(robot, encoding="utf-8")
    vendored = install / "opt" / "engine_vendor" / "share" / "doc" / "test"
    vendored.mkdir(parents=True)
    (vendored / "invalid.urdf").write_text("<robot name='x'><link", encoding="utf-8")
    loose = tmp_path / "src" / "descriptions"
    loose.mkdir(parents=True)
    (loose / "loose.urdf").write_text(robot, encoding="utf-8")
    roots = (RootSpec("install", str(install)), RootSpec("src", str(tmp_path / "src")))
    ws = Workspace.discover(roots)
    builder = model.AtlasBuilder()
    urdf_collect.UrdfCollector().collect(
        CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder)
    )
    files = {entity_id for entity_id in builder.build().entities if entity_id.startswith("file:")}
    assert ws.file_id(str(share / "urdf" / "fx_robot.urdf")) in files
    assert ws.file_id(str(loose / "loose.urdf")) in files
    assert not any("engine_vendor" in entity_id for entity_id in files)


def test_an_argument_an_included_file_declares_is_not_missing(tmp_path: Path) -> None:
    """jazzy's turtlebot4 model uses `$(arg gazebo)` that only the create3 file it includes
    declares, with a default, and binds xacro to the namespace spelled without `www`."""
    package = tmp_path / "src" / "fx_desc"
    (package / "urdf").mkdir(parents=True)
    (package / "package.xml").write_text(
        "<package format='3'><name>fx_desc</name><version>0.0.0</version>"
        "<description>d</description><maintainer email='a@b.c'>a</maintainer>"
        "<license>Apache-2.0</license></package>",
        encoding="utf-8",
    )
    (package / "urdf" / "base.urdf.xacro").write_text(
        '<robot name="base" xmlns:xacro="http://ros.org/wiki/xacro">'
        '<xacro:arg name="gazebo" default="ignition"/>'
        '<link name="base_link"/></robot>',
        encoding="utf-8",
    )
    (package / "urdf" / "robot.urdf.xacro").write_text(
        '<robot name="robot" xmlns:xacro="http://ros.org/wiki/xacro">'
        '<xacro:include filename="$(find fx_desc)/urdf/base.urdf.xacro"/>'
        '<link name="laser" type="$(arg gazebo)"/></robot>',
        encoding="utf-8",
    )
    ws = Workspace.discover([RootSpec("src", str(tmp_path / "src"))])
    base = urdf_collect.scan_xacro(
        str(package / "urdf" / "base.urdf.xacro"), resolve_package=ws.share
    )
    assert base.args == (("gazebo", "ignition"),)
    assert base.needs_arguments == ()
    robot = urdf_collect.scan_xacro(
        str(package / "urdf" / "robot.urdf.xacro"), resolve_package=ws.share
    )
    assert robot.needs_arguments == ("gazebo",)
    assert urdf_collect._declared_in_includes(robot, ws.share) == {"gazebo"}


# ---------------------------------------------------------------------------
# visual and collision geometry


def test_geometry_is_recorded_per_model_as_written(static_root: Path, ws: Workspace) -> None:
    atlas = _collect(static_root, ws)
    real = ws.file_id(str(static_root / VARIANTS["real"]))
    base = atlas.entities[ids.build_frame("base_link")].attrs["urdf_geometry"][real]
    assert base == {
        "collisions": [
            {
                "geometry": {"size": [0.6, 0.45, 0.2], "type": "box"},
                "origin": {"rpy": [0.0, 0.0, 0.0], "xyz": [0.25, 0.0, 0.1]},
            }
        ],
        "visuals": [
            {
                "geometry": {
                    "file": "file:fx_desc|meshes/fx_body.stl",
                    "filename": "package://fx_desc/meshes/fx_body.stl",
                    "scale": [1.0, 1.0, 1.0],
                    "type": "mesh",
                },
                "material": {"name": "fx_black", "rgba": [0.2, 0.2, 0.2, 1.0]},
                "origin": {"rpy": [0.0, 0.0, 0.0], "xyz": [0.25, 0.0, 0.1]},
            }
        ],
    }
    wheel = atlas.entities[ids.build_frame("drive_left_wheel")].attrs["urdf_geometry"][real]
    assert wheel["visuals"][0]["geometry"] == {"length": 0.04, "radius": 0.11, "type": "cylinder"}
    assert wheel["collisions"] == []
    sensor = atlas.entities[ids.build_frame("sensor_link")].attrs["urdf_geometry"][real]
    assert sensor["visuals"] == [
        {
            "geometry": {"radius": 0.05, "type": "sphere"},
            "material": {"name": "fx_orange", "rgba": [0.92, 0.4, 0.0, 1.0]},
            "name": "sensor_dome",
        }
    ]
    assert atlas.entities[ids.build_frame("tow_link")].attrs["urdf_geometry"] == {
        real: {"collisions": [], "visuals": []}
    }
    broken = ws.file_id(str(static_root / VARIANTS["broken"]))
    absent = [
        row["geometry"]
        for frame in atlas.by_kind("frame")
        for row in frame.attrs.get("urdf_geometry", {}).get(broken, {}).get("visuals", [])
        if row["geometry"]["type"] == "mesh"
    ]
    assert absent == [
        {
            "filename": "package://fx_desc/meshes/fx_absent_body.stl",
            "scale": [1.0, 1.0, 1.0],
            "type": "mesh",
        }
    ]


def test_a_link_points_at_its_own_element(static_root: Path, ws: Workspace) -> None:
    atlas = _collect(static_root, ws)
    real = ws.file_id(str(static_root / VARIANTS["real"]))
    text = (static_root / VARIANTS["real"]).read_text(encoding="utf-8").splitlines()
    rows = [
        row
        for fact in atlas.entities[ids.build_frame("base_link")].facts
        for row in fact.evidence
        if row.file == real
    ]
    assert len(rows) == 1 and rows[0].line_hint is not None
    assert '<link name="base_link">' in text[rows[0].line_hint - 1]


def test_shape_numbers_that_do_not_read_are_named_and_order_is_not_a_fact() -> None:
    visuals = (
        '<visual><geometry><box size="${w} 1 1"/></geometry>'
        '<material name="m"><color rgba="1 0 0"/></material></visual>'
        '<visual><origin xyz="0 0 1"/><geometry><cylinder radius="0.1"/></geometry></visual>'
    )
    first = urdf_collect.parse_urdf_text(f'<robot name="p"><link name="a">{visuals}</link></robot>')
    swapped = visuals.split("</visual>")
    swapped = f"{swapped[1]}</visual>{swapped[0]}</visual>"
    second = urdf_collect.parse_urdf_text(
        f'<robot name="p"><link name="a">{swapped}</link></robot>'
    )
    assert first.shapes == second.shapes
    rows = dict(first.shapes)["a"]["visuals"]
    box = next(row for row in rows if row["geometry"]["type"] == "box")
    assert box == {
        "geometry": {"type": "box"},
        "material": {"name": "m"},
        "unreadable": ["box.size", "material.rgba"],
    }
    cylinder = next(row for row in rows if row["geometry"]["type"] == "cylinder")
    assert cylinder == {
        "geometry": {"radius": 0.1, "type": "cylinder"},
        "origin": {"xyz": [0.0, 0.0, 1.0]},
    }


def _bot_install(install: Path) -> Path:
    share = install / "share" / "bot"
    (share / "meshes").mkdir(parents=True)
    (share / "package.xml").write_text(
        "<package format='3'><name>bot</name></package>", encoding="utf-8"
    )
    (share / "meshes" / "body.stl").write_text("solid body\nendsolid body\n", encoding="utf-8")
    return share


def test_a_mesh_named_by_its_absolute_path_is_stated_without_it(tmp_path: Path) -> None:
    """`$(find bot)` expands to where this machine installed the package, and that path sat in
    the geometry, so two clones wrote two atlases and the robot view could not draw the mesh."""
    install = tmp_path / "ws" / "install"
    share = _bot_install(install)
    elsewhere = tmp_path / "elsewhere" / "wheel.stl"
    elsewhere.parent.mkdir()
    elsewhere.write_text("solid wheel\nendsolid wheel\n", encoding="utf-8")
    (share / "urdf").mkdir()
    (share / "urdf" / "bot.urdf").write_text(
        '<robot name="bot"><link name="base">'
        f'<visual><geometry><mesh filename="file://{share}/meshes/body.stl"/></geometry>'
        f'<material name="skin"><texture filename="{share}/meshes/skin.png"/></material></visual>'
        f'<visual><geometry><mesh filename="{share}/meshes/gone.stl"/></geometry></visual>'
        f'<collision><geometry><mesh filename="file://{elsewhere}"/></geometry></collision>'
        "</link></robot>",
        encoding="utf-8",
    )
    roots = (RootSpec("install", str(install)),)
    ws = Workspace.discover(roots)
    builder = model.AtlasBuilder()
    urdf_collect.UrdfCollector().collect(
        CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder)
    )
    atlas = builder.build()
    model_id = ids.build_file("bot", "urdf/bot.urdf")
    shapes = atlas.entities[ids.build_frame("base")].attrs["urdf_geometry"][model_id]
    visuals = sorted(shapes["visuals"], key=lambda row: row["geometry"]["filename"])
    assert visuals[0]["geometry"] == {
        "file": ids.build_file("bot", "meshes/body.stl"),
        "filename": "package://bot/meshes/body.stl",
        "type": "mesh",
    }
    assert visuals[0]["material"]["texture"] == "package://bot/meshes/skin.png"
    assert visuals[1]["geometry"] == {"filename": "package://bot/meshes/gone.stl", "type": "mesh"}
    outside = shapes["collisions"][0]["geometry"]["filename"]
    assert outside.startswith("@outside/wheel.stl#")
    assert atlas.entities[model_id].attrs["missing_meshes"] == ["package://bot/meshes/gone.stl"]
    blind = [entry for entry in atlas.unresolved if entry.reason == "file_outside_roots"]
    assert [(entry.owner, entry.expr) for entry in blind] == [(model_id, outside)]
    assert str(tmp_path).encode() not in atlas.canonical_bytes()


def test_the_run_id_does_not_follow_where_the_roots_sort(tmp_path: Path) -> None:
    """The digest took the models in absolute path order, so a src root that sorted before
    the install root on one machine and after it on another gave one tree two run ids."""
    runs = []
    for src_at, install_at in (("a", "b"), ("b", "a")):
        src = tmp_path / src_at / "src"
        install = tmp_path / install_at / "x" / "install"
        _bot_install(install)
        (install / "share" / "bot" / "bot.urdf").write_text(
            '<robot name="bot"><link name="base"/></robot>', encoding="utf-8"
        )
        package = src / "loose"
        package.mkdir(parents=True)
        (package / "package.xml").write_text(
            "<package format='3'><name>loose</name></package>", encoding="utf-8"
        )
        (package / "loose.urdf").write_text(
            '<robot name="loose"><link name="l"/></robot>', encoding="utf-8"
        )
        roots = (RootSpec("src", str(src)), RootSpec("install", str(install)))
        builder = model.AtlasBuilder()
        urdf_collect.UrdfCollector().collect(
            CollectContext(
                ws=Workspace.discover(roots), cfg=Config(roots=roots, profiles=()), builder=builder
            )
        )
        runs.append(sorted(key for key in builder.build().entities if key.startswith("run:")))
    assert runs[0] == runs[1]


def test_a_missing_package_mesh_says_whether_the_source_holds_it(tmp_path: Path) -> None:
    """The source holding a mesh the install tree lacks is a missing install line, and no tree
    holding it is a wrong name, so `model.mesh_missing` grades the two apart."""
    install = tmp_path / "ws" / "install"
    share = install / "share" / "bot"
    (share / "urdf").mkdir(parents=True)
    (share / "package.xml").write_text(
        "<package format='3'><name>bot</name></package>", encoding="utf-8"
    )
    (share / "urdf" / "bot.urdf").write_text(
        '<robot name="bot"><link name="base">'
        '<visual><geometry><mesh filename="package://bot/meshes/body.stl"/></geometry></visual>'
        '<visual><geometry><mesh filename="package://bot/meshes/gone.stl"/></geometry></visual>'
        "</link></robot>",
        encoding="utf-8",
    )
    src = tmp_path / "ws" / "src" / "bot"
    (src / "meshes").mkdir(parents=True)
    (src / "package.xml").write_text(
        "<package format='3'><name>bot</name><export><build_type>ament_python</build_type>"
        "</export></package>",
        encoding="utf-8",
    )
    (src / "meshes" / "body.stl").write_text("solid body\nendsolid body\n", encoding="utf-8")
    roots = (RootSpec("src", str(src.parent)), RootSpec("install", str(install)))
    builder = model.AtlasBuilder()
    urdf_collect.UrdfCollector().collect(
        CollectContext(
            ws=Workspace.discover(roots), cfg=Config(roots=roots, profiles=()), builder=builder
        )
    )
    atlas = builder.build()
    attrs = atlas.entities[ids.build_file("bot", "urdf/bot.urdf")].attrs
    assert attrs["meshes_not_installed"] == ["package://bot/meshes/body.stl"]
    assert attrs["meshes_absent"] == ["package://bot/meshes/gone.stl"]
    assert atlas.entities[ids.build_pkg("bot")].attrs["build_type"] == "ament_python"
    assert "meshes_elsewhere" not in attrs


def test_a_missing_mesh_the_package_holds_elsewhere_names_where(tmp_path: Path) -> None:
    """nav2's tb3 model names models/tire.dae, which the package holds under
    models/turtlebot3_model/meshes."""
    share = tmp_path / "install" / "share" / "bot"
    (share / "urdf").mkdir(parents=True)
    (share / "models" / "bot_model" / "meshes").mkdir(parents=True)
    (share / "package.xml").write_text(
        "<package format='3'><name>bot</name></package>", encoding="utf-8"
    )
    (share / "models" / "bot_model" / "meshes" / "tire.dae").write_text("x", encoding="utf-8")
    (share / "urdf" / "bot.urdf").write_text(
        '<robot name="bot"><link name="wheel">'
        '<visual><geometry><mesh filename="package://bot/models/tire.dae"/></geometry></visual>'
        "</link></robot>",
        encoding="utf-8",
    )
    roots = (RootSpec("install", str(tmp_path / "install")),)
    builder = model.AtlasBuilder()
    urdf_collect.UrdfCollector().collect(
        CollectContext(
            ws=Workspace.discover(roots), cfg=Config(roots=roots, profiles=()), builder=builder
        )
    )
    attrs = builder.build().entities[ids.build_file("bot", "urdf/bot.urdf")].attrs
    assert attrs["meshes_absent"] == ["package://bot/models/tire.dae"]
    assert attrs["meshes_elsewhere"] == {
        "package://bot/models/tire.dae": ["package://bot/models/bot_model/meshes/tire.dae"]
    }


@pytest.mark.ros
def test_xacro_finds_a_package_an_unbuilt_source_root_holds(tmp_path: Path) -> None:
    """ros2_control_demos includes `$(find ros2_control_demo_description)/...` from every
    example, and an unbuilt checkout refused all of them with `package not found`."""
    if not urdf_collect.xacro_available():
        pytest.skip("the xacro tool is not on PATH")
    src = tmp_path / "src"
    for name in ("demo_description", "demo_robot"):
        (src / name / "urdf").mkdir(parents=True)
        (src / name / "package.xml").write_text(
            f"<package format='3'><name>{name}</name></package>", encoding="utf-8"
        )
    (src / "demo_description" / "urdf" / "arm.xacro").write_text(
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">'
        '<xacro:macro name="arm"><link name="arm_link"/></xacro:macro></robot>',
        encoding="utf-8",
    )
    (src / "demo_robot" / "urdf" / "robot.urdf.xacro").write_text(
        '<robot name="demo" xmlns:xacro="http://www.ros.org/wiki/xacro">'
        '<xacro:include filename="$(find demo_description)/urdf/arm.xacro"/>'
        '<link name="base_link"/><xacro:arm/></robot>',
        encoding="utf-8",
    )
    roots = (RootSpec("src", str(src)),)
    builder = model.AtlasBuilder()
    urdf_collect.UrdfCollector().collect(
        CollectContext(
            ws=Workspace.discover(roots), cfg=Config(roots=roots, profiles=()), builder=builder
        )
    )
    atlas = builder.build()
    robot = atlas.entities[ids.build_file("demo_robot", "urdf/robot.urdf.xacro")]
    assert "refused_because" not in robot.attrs
    assert ids.build_frame("arm_link") in atlas.entities
    with pytest.raises(CollectorError, match="not found") as raised:
        urdf_collect.expand_xacro(str(src / "demo_robot" / "urdf" / "robot.urdf.xacro"))
    assert "searching" not in str(raised.value)


@pytest.mark.ros
def test_a_xacro_include_only_a_build_puts_in_place_is_refused_as_such(tmp_path: Path) -> None:
    """ros2_control_demos installs description/ros2_control as ros2_control, so an unbuilt
    model reads `$(find pkg)/ros2_control/...` where the source holds no such directory."""
    if not urdf_collect.xacro_available():
        pytest.skip("the xacro tool is not on PATH")
    package = tmp_path / "src" / "demo_robot"
    (package / "description" / "urdf").mkdir(parents=True)
    (package / "description" / "ros2_control").mkdir()
    (package / "package.xml").write_text(
        "<package format='3'><name>demo_robot</name></package>", encoding="utf-8"
    )
    (package / "description" / "ros2_control" / "arm.ros2_control.xacro").write_text(
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"/>', encoding="utf-8"
    )
    (package / "description" / "urdf" / "robot.urdf.xacro").write_text(
        '<robot name="demo" xmlns:xacro="http://www.ros.org/wiki/xacro">'
        '<xacro:include filename="$(find demo_robot)/ros2_control/arm.ros2_control.xacro"/>'
        '<link name="base_link"/></robot>',
        encoding="utf-8",
    )
    roots = (RootSpec("src", str(tmp_path / "src")),)
    builder = model.AtlasBuilder()
    urdf_collect.UrdfCollector().collect(
        CollectContext(
            ws=Workspace.discover(roots), cfg=Config(roots=roots, profiles=()), builder=builder
        )
    )
    attrs = (
        builder.build()
        .entities[ids.build_file("demo_robot", "description/urdf/robot.urdf.xacro")]
        .attrs
    )
    assert attrs["refused_because"] == "file_after_build"
    assert attrs["needs"] == [ids.build_file("demo_robot", "ros2_control/arm.ros2_control.xacro")]
    assert attrs["found_elsewhere"] == ["description/ros2_control/arm.ros2_control.xacro"]


# ---------------------------------------------------------------------------
# a model named .urdf that holds xacro, and the profile that loads it with arguments

PREFIXED = (
    '<robot name="bot" xmlns:xacro="http://ros.org/wiki/xacro">'
    '<xacro:arg name="namespace" default=""/>'
    '<xacro:property name="namespace" value="$(arg namespace)"/>'
    '<link name="${namespace}base_link"/><link name="${namespace}base_scan"/>'
    '<joint name="${namespace}scan_joint" type="fixed">'
    '<parent link="${namespace}base_link"/><child link="${namespace}base_scan"/>'
    '<origin xyz="0.1 0 0.2" rpy="0 0 0"/></joint></robot>'
)


def _prefixed(tmp_path: Path) -> tuple[Workspace, Config, Path]:
    package = tmp_path / "src" / "bot"
    (package / "urdf").mkdir(parents=True)
    (package / "package.xml").write_text(
        "<package format='3'><name>bot</name><version>0.0.0</version>"
        "<description>d</description><maintainer email='a@b.c'>a</maintainer>"
        "<license>Apache-2.0</license></package>",
        encoding="utf-8",
    )
    path = package / "urdf" / "bot.urdf"
    path.write_text(PREFIXED, encoding="utf-8")
    roots = (RootSpec("src", str(tmp_path / "src")),)
    return Workspace.discover(list(roots)), Config(roots=roots, profiles=()), path


def test_xacro_markup_is_told_apart_from_a_namespace_declaration(tmp_path: Path) -> None:
    plain = tmp_path / "plain.urdf"
    plain.write_text(
        '<robot name="bot" xmlns:xacro="http://ros.org/wiki/xacro">'
        '<!-- <xacro:property name="r" value="${1}"/> --><link name="base_link"/></robot>',
        encoding="utf-8",
    )
    marked = tmp_path / "marked.urdf"
    marked.write_text(PREFIXED, encoding="utf-8")
    assert not urdf_collect.uses_xacro(str(plain))
    assert urdf_collect.uses_xacro(str(marked))


@pytest.mark.ros
def test_a_urdf_holding_xacro_is_expanded_and_never_names_the_expression(tmp_path: Path) -> None:
    ws, cfg, _path = _prefixed(tmp_path)
    builder = model.AtlasBuilder()
    urdf_collect.UrdfCollector().collect(CollectContext(ws=ws, cfg=cfg, builder=builder))
    atlas = builder.build()
    frames = sorted(item.id for item in atlas.by_kind("frame"))
    assert frames == [ids.build_frame("base_link"), ids.build_frame("base_scan")]
    edge = next(item for item in atlas.edges.values() if item.kind == "tf_parent")
    assert edge.facts[0].values["strategy"] == "xacro"
    assert "xacro_args" not in edge.facts[0].values


def _loaded(tmp_path: Path, args: tuple[tuple[str, str], ...]) -> tuple[model.Atlas, bool]:
    ws, cfg, path = _prefixed(tmp_path)
    builder = model.AtlasBuilder()
    urdf_collect.UrdfCollector().collect(CollectContext(ws=ws, cfg=cfg, builder=builder))
    builder.add_profile("tb3")
    run = builder.add_run("declared", profile="profile:tb3", input_digest="d", roots={})
    ctx = CollectContext(ws=ws, cfg=cfg, builder=builder)
    wrote = urdf_collect.emit_loaded(ctx, run, "profile:tb3", str(path), args)
    return builder.build(), wrote


@pytest.mark.ros
def test_a_profile_loading_the_model_with_arguments_states_the_frames_it_runs(
    tmp_path: Path,
) -> None:
    atlas, wrote = _loaded(tmp_path, (("namespace", "tb3/"),))
    assert wrote
    scan = atlas.entities[ids.build_frame("tb3/base_scan")]
    assert [fact.profile for fact in scan.facts] == ["profile:tb3"]
    assert scan.attrs["urdf_models"] == [ids.build_file("bot", "urdf/bot.urdf")]
    edge = next(
        item
        for item in atlas.edges.values()
        if item.kind == "tf_parent" and item.src == ids.build_frame("tb3/base_scan")
    )
    assert edge.dst == ids.build_frame("tb3/base_link")
    assert edge.facts[0].profile == "profile:tb3"
    assert edge.facts[0].values["xacro_args"] == [["namespace", "tb3/"]]
    assert edge.facts[0].values["xyz"] == [0.1, 0.0, 0.2]
    # the expansion with the defaults still stands for every profile
    assert ids.build_frame("base_scan") in atlas.entities
    assert schema.validate(atlas.to_json()) == []


@pytest.mark.ros
def test_arguments_that_give_the_default_model_add_nothing(tmp_path: Path) -> None:
    atlas, wrote = _loaded(tmp_path, (("namespace", ""),))
    assert not wrote
    assert sorted(item.id for item in atlas.by_kind("frame")) == [
        ids.build_frame("base_link"),
        ids.build_frame("base_scan"),
    ]


@pytest.mark.ros
def test_a_profile_expansion_is_the_same_twice(tmp_path: Path) -> None:
    first, _ = _loaded(tmp_path / "a", (("namespace", "tb3/"),))
    second, _ = _loaded(tmp_path / "b", (("namespace", "tb3/"),))
    assert first.canonical_bytes() == second.canonical_bytes()
