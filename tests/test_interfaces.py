"""msg, srv and action definitions: fields, arrays, constants, defaults and discovery."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest

from rosio import ids, model, schema
from rosio.collect import CollectContext
from rosio.collect import interfaces as interfaces_collect
from rosio.config import Config, RootSpec
from rosio.errors import CollectorError
from rosio.workspace import Workspace


@pytest.fixture(scope="module")
def static_root(fixtures_dir: Path) -> Path:
    return fixtures_dir / "ws_static"


@pytest.fixture(scope="module")
def ws(static_root: Path) -> Workspace:
    return Workspace.discover([RootSpec("src", str(static_root / "src"))])


@pytest.fixture(scope="module")
def expected(fixtures_dir: Path) -> dict[str, Any]:
    path = fixtures_dir / "expected" / "static" / "interfaces.json"
    return json.loads(path.read_text(encoding="utf-8"))


def _views(static_root: Path, ws: Workspace) -> dict[str, dict[str, Any]]:
    out: dict[str, dict[str, Any]] = {}
    for item in interfaces_collect.discover(ws):
        out[item.key] = item.view(str(Path(item.path).relative_to(static_root)))
    return out


def _collect(static_root: Path, ws: Workspace) -> model.Atlas:
    roots = (RootSpec("src", str(static_root / "src")),)
    builder = model.AtlasBuilder()
    ctx = CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder)
    interfaces_collect.InterfacesCollector().collect(ctx)
    return builder.build()


# ---------------------------------------------------------------------------
# parsing


def test_every_interface_matches_the_expected_file(
    static_root: Path, ws: Workspace, expected
) -> None:
    got = _views(static_root, ws)
    assert sorted(got) == sorted(expected["interfaces"])
    for key, want in expected["interfaces"].items():
        for section in sorted(want):
            assert got[key][section] == want[section], f"{key} {section}"


def test_packages_match_the_expected_file(static_root: Path, ws: Workspace, expected) -> None:
    got = interfaces_collect.package_view(interfaces_collect.discover(ws))
    for pkg, want in expected["packages"].items():
        for kind in ("msg", "srv", "action"):
            assert got[pkg][kind] == want[kind], f"{pkg} {kind}"
        assert str(Path(ws.packages[pkg].src_path).relative_to(static_root)) == want["path"]


def test_a_relative_field_type_resolves_to_the_declaring_package(
    static_root: Path, ws: Workspace
) -> None:
    state = next(item for item in interfaces_collect.discover(ws) if item.name == "FxState")
    fields = {item.name: item for item in state.sections[0].fields}
    assert fields["pose"].package == "fx_msgs"
    assert fields["pose"].written_relative is True
    assert fields["header"].package == "std_msgs"
    assert fields["header"].written_relative is False
    assert fields["tag"].package == "fx_other_msgs"
    assert fields["tag"].written_relative is False


def test_array_kinds_stay_distinct(static_root: Path, ws: Workspace) -> None:
    state = next(item for item in interfaces_collect.discover(ws) if item.name == "FxState")
    fields = {item.name: item.view() for item in state.sections[0].fields}
    assert fields["history"]["array"] == {"kind": "unbounded"}
    assert fields["corners"]["array"] == {"kind": "fixed", "size": 4}
    assert fields["weights"]["array"] == {"kind": "bounded", "size": 8}
    assert fields["pose"]["array"] is None


def test_default_values_keep_their_type(static_root: Path, ws: Workspace) -> None:
    state = next(item for item in interfaces_collect.discover(ws) if item.name == "FxState")
    fields = {item.name: item for item in state.sections[0].fields}
    assert fields["valid"].default is True
    assert isinstance(fields["confidence"].default, float)
    assert fields["confidence"].default == 0.5
    assert fields["child_frame_id"].default == "odom"
    constants = {item.name: item for item in state.sections[0].constants}
    assert constants["MAX_TRACKS"].value == 16
    assert isinstance(constants["MAX_TRACKS"].value, int)
    assert constants["DEFAULT_FRAME"].value == "odom"


def test_a_bounded_string_records_its_upper_bound() -> None:
    found = interfaces_collect.parse_interface_text(
        "string<=16 code\nstring<=8[<=2] tags\n",
        pkg="fx_msgs",
        kind="msg",
        name="Bounded",
    )
    fields = {item.name: item.view() for item in found.sections[0].fields}
    assert fields["code"]["string_upper_bound"] == 16
    assert fields["tags"]["string_upper_bound"] == 8
    assert fields["tags"]["array"] == {"kind": "bounded", "size": 2}


def test_action_sections_keep_their_order() -> None:
    text = "uint8 A=1\nstring goal_id\n---\nuint8 ERR_TIMEOUT=7\nbool ok\n---\nfloat32 progress\n"
    found = interfaces_collect.parse_interface_text(
        text, pkg="fx_msgs", kind="action", name="Ordered"
    )
    sections = {section.name: section for section in found.sections}
    assert [item.name for item in sections["goal"].constants] == ["A"]
    assert [item.name for item in sections["result"].constants] == ["ERR_TIMEOUT"]
    assert [item.name for item in sections["feedback"].fields] == ["progress"]


def test_too_many_sections_are_refused() -> None:
    with pytest.raises(CollectorError):
        interfaces_collect.parse_interface_text(
            "bool a\n---\nbool b\n", pkg="fx_msgs", kind="msg", name="Bad"
        )


@pytest.mark.parametrize(
    "kind, text",
    [
        ("srv", "int32 request\n"),
        ("action", "int32 goal\n---\nint32 result\n"),
        ("action", "int32 goal\n"),
    ],
)
def test_a_missing_section_is_refused_not_invented(kind: str, text: str) -> None:
    """A padded empty section reads as a service that answers nothing and an action that
    reports no feedback, which is a claim the file never made."""
    with pytest.raises(CollectorError):
        interfaces_collect.parse_interface_text(text, pkg="fx_msgs", kind=kind, name="Short")


def test_a_comma_inside_a_quoted_default_is_not_a_separator() -> None:
    found = interfaces_collect.parse_interface_text(
        "string[] labels [\"a,b\", 'c,d', e]\nint32[] sizes [1, 2]\n",
        pkg="fx_msgs",
        kind="msg",
        name="Labels",
    )
    fields = {item.name: item.default for item in found.sections[0].fields}
    assert fields["labels"] == ["a,b", "c,d", "e"]
    assert fields["sizes"] == [1, 2]


def test_an_unterminated_string_in_a_default_is_refused() -> None:
    with pytest.raises(CollectorError):
        interfaces_collect.parse_interface_text(
            'string[] labels ["a, b]\n', pkg="fx_msgs", kind="msg", name="Torn"
        )


def test_a_comment_inside_a_string_default_survives() -> None:
    found = interfaces_collect.parse_interface_text(
        'string tag "a#b"  # trailing note\n', pkg="fx_msgs", kind="msg", name="Hash"
    )
    assert found.sections[0].fields[0].default == "a#b"


def test_dependencies_are_named_as_full_types(static_root: Path, ws: Workspace) -> None:
    state = next(item for item in interfaces_collect.discover(ws) if item.name == "FxState")
    assert state.depends_on() == [
        "fx_msgs/msg/FxPoint",
        "fx_other_msgs/msg/FxTag",
        "std_msgs/msg/Header",
    ]


# ---------------------------------------------------------------------------
# discovery


def test_the_generator_list_is_the_discovery_key() -> None:
    text = (
        "rosidl_generate_interfaces(${PROJECT_NAME}\n"
        '  "msg/One.msg"\n'
        '  "${CMAKE_CURRENT_SOURCE_DIR}/msg_ros2:msg/BlockHeader.msg"\n'
        "  DEPENDENCIES std_msgs\n)\n"
    )
    assert interfaces_collect.generator_entries(text) == [
        "msg/One.msg",
        "${CMAKE_CURRENT_SOURCE_DIR}/msg_ros2:msg/BlockHeader.msg",
    ]


def test_discovery_is_sorted_and_stable(static_root: Path, ws: Workspace) -> None:
    first = [item.key for item in interfaces_collect.discover(ws)]
    second = [item.key for item in interfaces_collect.discover(ws)]
    assert first == sorted(first)
    assert first == second


def test_discovery_says_where_the_list_came_from(static_root: Path, ws: Workspace) -> None:
    for item in interfaces_collect.discover(ws):
        assert item.discovery == "generator_list"


# ---------------------------------------------------------------------------
# the collector


def test_the_atlas_validates_and_is_stable(static_root: Path, ws: Workspace) -> None:
    first = _collect(static_root, ws)
    assert schema.validate(first.to_json()) == []
    second = _collect(static_root, Workspace.discover([RootSpec("src", str(static_root / "src"))]))
    assert first.canonical_bytes() == second.canonical_bytes()


def test_type_entities_and_edges(static_root: Path, ws: Workspace) -> None:
    atlas = _collect(static_root, ws)
    type_id = ids.build_type("fx_msgs", "msg", "FxState")
    assert type_id in atlas.entities
    fact = atlas.entities[type_id].facts[0]
    assert fact.values["interface"] == "msg"
    assert "std_msgs/msg/Header" in fact.values["depends_on"]
    file_id = ids.build_file("fx_msgs", "msg/FxState.msg")
    kinds = {(edge.kind, edge.src) for edge in atlas.in_edges(type_id)}
    assert (model.EdgeKind.DEFINES.value, file_id) in kinds
    assert (model.EdgeKind.CONTAINS.value, ids.build_pkg("fx_msgs")) in kinds


def test_srv_and_action_sections_reach_the_atlas(static_root: Path, ws: Workspace) -> None:
    atlas = _collect(static_root, ws)
    srv = atlas.entities[ids.build_type("fx_msgs", "srv", "FxConfigure")].facts[0].values
    assert [row["name"] for row in srv["request.fields"]] == ["profile", "limits", "dry_run"]
    assert [row["name"] for row in srv["response.fields"]] == ["accepted", "reason"]
    action = atlas.entities[ids.build_type("fx_msgs", "action", "FxDock")].facts[0].values
    assert [row["name"] for row in action["goal.constants"]] == ["MODE_FRONT", "MODE_REAR"]
    assert [row["name"] for row in action["feedback.fields"]] == ["progress", "stage"]


def _one_msg_package(root: Path, text: str) -> Path:
    package = root / "src" / "zz_msgs"
    (package / "msg").mkdir(parents=True)
    (package / "package.xml").write_text(
        "<package format='3'><name>zz_msgs</name><version>0.0.1</version>"
        "<description>d</description><maintainer email='dev@example.invalid'>m</maintainer>"
        "<license>Apache-2.0</license><export><build_type>ament_cmake</build_type>"
        "</export></package>",
        encoding="utf-8",
    )
    (package / "CMakeLists.txt").write_text(
        "project(zz_msgs)\nrosidl_generate_interfaces(${PROJECT_NAME}\n"
        '  "msg/Ok.msg"\n  "msg/Torn.msg"\n)\n',
        encoding="utf-8",
    )
    (package / "msg" / "Ok.msg").write_text("int32 value\n", encoding="utf-8")
    (package / "msg" / "Torn.msg").write_text(text, encoding="utf-8")
    return package


def test_an_unreadable_definition_still_raises_for_a_direct_reader(tmp_path: Path) -> None:
    _one_msg_package(tmp_path, "nonsense\n")
    ws = Workspace.discover([RootSpec("src", str(tmp_path / "src"))])
    with pytest.raises(CollectorError):
        interfaces_collect.discover(ws)


def test_an_unreadable_definition_does_not_empty_the_layer(tmp_path: Path) -> None:
    package = _one_msg_package(tmp_path, "nonsense\n")
    roots = (RootSpec("src", str(tmp_path / "src")),)
    ws = Workspace.discover(roots)
    builder = model.AtlasBuilder()
    interfaces_collect.InterfacesCollector().collect(
        CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder)
    )
    atlas = builder.build()
    assert ids.build_type("zz_msgs", "msg", "Ok") in atlas.entities
    assert ids.build_type("zz_msgs", "msg", "Torn") not in atlas.entities
    torn = atlas.entities[ws.file_id(str(package / "msg" / "Torn.msg"))]
    assert torn.attrs["loadable"] is False
    assert [entry.reason for entry in atlas.unresolved] == ["interface_unreadable"]
    run = next(node for node in atlas.by_kind("run"))
    assert [row["exc_type"] for row in run.attrs["exceptions"]] == ["CollectorError"]
    torn_id = ws.file_id(str(package / "msg" / "Torn.msg"))
    assert torn.attrs["refused_because"] == "interface_unreadable"
    assert ids.build_edge("contains", ids.build_pkg("zz_msgs"), torn_id) in atlas.edges
    entry = atlas.unresolved[0]
    assert entry.owner == torn_id
    assert entry.want == model.Want.TYPE.value
    assert entry.id in torn.unresolved
    # One of the two definitions was read, so the want keeps both halves of its ratio.
    assert atlas.counters["resolution_rate"]["type"] == 0.5


def test_a_generator_list_this_process_cannot_read_is_a_refusal(tmp_path: Path) -> None:
    """The generator list is the discovery key, so a build file that cannot be opened is a
    refusal of its own and never a silent fall back to a directory scan."""
    package = _one_msg_package(tmp_path, "int32 value\n")
    cmake = package / "CMakeLists.txt"
    roots = (RootSpec("src", str(tmp_path / "src")),)
    ws = Workspace.discover(roots)
    cmake.chmod(0)
    builder = model.AtlasBuilder()
    try:
        interfaces_collect.InterfacesCollector().collect(
            CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder)
        )
    finally:
        cmake.chmod(0o600)
    atlas = builder.build()
    entity = atlas.entities[ws.file_id(str(cmake))]
    assert entity.attrs["loadable"] is False
    assert entity.attrs["refused_because"] == "generator_list_unreadable"
    entry = next(item for item in atlas.unresolved if item.owner == entity.id)
    assert entry.want == model.Want.TYPE.value
    assert entry.id in entity.unresolved


def test_a_profile_scoped_run_carries_a_profile_id(static_root: Path, ws: Workspace) -> None:
    roots = (RootSpec("src", str(static_root / "src")),)
    builder = model.AtlasBuilder()
    interfaces_collect.InterfacesCollector().collect(
        CollectContext(
            ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder, profile="sim_trailer"
        )
    )
    atlas = builder.build()
    fact = atlas.entities[ids.build_type("fx_msgs", "msg", "FxState")].facts[0]
    assert fact.profile == ids.build_profile("sim_trailer")


def test_no_type_is_invented_for_a_package_outside_the_workspace(
    static_root: Path, ws: Workspace
) -> None:
    atlas = _collect(static_root, ws)
    packages = {ids.parse_type(node.id).package for node in atlas.by_kind("type")}
    assert packages == {"fx_msgs", "fx_other_msgs"}


def _atlas_of(tmp_path: Path) -> tuple[Workspace, model.Atlas]:
    roots = (RootSpec("src", str(tmp_path / "src")),)
    ws = Workspace.discover(roots)
    builder = model.AtlasBuilder()
    interfaces_collect.InterfacesCollector().collect(
        CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder)
    )
    return ws, builder.build()


def test_a_constant_json_cannot_hold_costs_itself_and_not_the_type(tmp_path: Path) -> None:
    """One non finite constant used to end the interface pass, taking every edge of the
    type and one resolved count with it."""
    _one_msg_package(tmp_path, "float64 LIMIT=inf\nfloat64 rate\n")
    _ws, atlas = _atlas_of(tmp_path)
    type_id = ids.build_type("zz_msgs", "msg", "Torn")
    assert type_id in atlas.entities
    assert ids.build_type("zz_msgs", "msg", "Ok") in atlas.entities
    constant = atlas.entities[type_id].facts[0].values["constants"][0]
    assert constant["name"] == "LIMIT"
    assert constant["type"] == "float64"
    assert constant["value"] is None
    assert constant["value_state"] == "unresolved"
    assert constant["value_expr"] == "inf"
    entry = next(item for item in atlas.unresolved if item.owner == type_id)
    assert entry.expr == "LIMIT=inf"
    assert entry.reason == f"{model.NON_FINITE}: inf"
    assert entry.id in atlas.entities[type_id].unresolved
    assert entry.evidence[0].locator == "constants[LIMIT]"
    kinds = {item.kind for item in atlas.edges.values()}
    assert {"defines", "contains"} <= kinds
    assert schema.validate(atlas.to_json()) == []


def test_a_field_default_json_cannot_hold_keeps_the_field(tmp_path: Path) -> None:
    _one_msg_package(tmp_path, "float64 rate nan\n")
    _ws, atlas = _atlas_of(tmp_path)
    type_id = ids.build_type("zz_msgs", "msg", "Torn")
    field = atlas.entities[type_id].facts[0].values["fields"][0]
    assert field["name"] == "rate"
    assert field["type"] == "float64"
    assert field["default"] is None
    assert field["default_state"] == "unresolved"
    assert field["default_expr"] == "nan"
    entry = next(item for item in atlas.unresolved if item.owner == type_id)
    assert entry.expr == "rate nan"
    assert entry.reason == f"{model.NON_FINITE}: nan"


def test_a_finite_definition_carries_no_state_key(tmp_path: Path) -> None:
    """The keys above appear on a refused row and nowhere else."""
    _one_msg_package(tmp_path, "float64 LIMIT=1.5\nfloat64 rate 0.5\n")
    _ws, atlas = _atlas_of(tmp_path)
    values = atlas.entities[ids.build_type("zz_msgs", "msg", "Torn")].facts[0].values
    assert values["constants"][0] == {"name": "LIMIT", "type": "float64", "value": 1.5}
    assert "default_state" not in values["fields"][0]
    assert values["fields"][0]["default"] == 0.5
    assert atlas.unresolved == []
