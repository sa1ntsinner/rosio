"""The C++ collector, against the expectations committed in tests/fixtures/expected/code.

The per file expectations are the contract, so the comparison here is the whole document and
not a handful of assertions. Four keys are dropped before comparing: `line_hint` and the prose
keys (`note`, `detail`, the `*_reason` explanations), because AGENTS.md section 9 keeps a line
number out of every comparison and the prose is written for a human reading the fixture.
"""

from __future__ import annotations

import builtins
import json
import shutil
from pathlib import Path
from typing import Any

import pytest

from rosio import extract, ids, join, model, schema
from rosio.collect import CollectContext, bag, launch_declared, params, py_code
from rosio.collect import cpp_code as cpp
from rosio.config import Config, RootSpec
from rosio.errors import CollectorError
from rosio.resolve import records, wrappers
from rosio.workspace import Workspace

pytestmark = pytest.mark.cpp


@pytest.mark.parametrize("helper", ["member", "free"])
def test_nav2_declare_helpers_keep_literal_and_unknown_defaults(helper):
    if helper == "member":
        calls = """
        declare_or_get_parameter("frame", std::string("map"));
        this->declare_or_get_parameter<double>("gain", 1.5);
        declare_or_get_parameter("dynamic", runtime_value);
        """
    else:
        calls = """
        nav2_util::declare_parameter_if_not_declared(
          handle, "frame", rclcpp::ParameterValue("map"));
        nav2_util::declare_parameter_if_not_declared(
          handle, "gain", rclcpp::ParameterValue(1.5));
        nav2_util::declare_parameter_if_not_declared(
          handle, "dynamic", rclcpp::ParameterValue(runtime_value));
        """
    unit = cpp.parse_text("void Controller::configure() {" + calls + "}", path="helper.cpp")
    cpp.analyse([unit])
    rows = {row["name"]: row for row in cpp.file_report(unit)["parameters"]}
    assert set(rows) == {"frame", "gain", "dynamic"}
    assert rows["frame"]["default"] == "map"
    assert rows["gain"]["default"] == 1.5
    assert rows["dynamic"]["default"] is None
    assert rows["dynamic"]["default_state"] == "unresolved"
    assert all(row["owner_symbol"] == "Controller::configure" for row in rows.values())


def test_nav2_lifecycle_node_keeps_its_literal_name():
    unit = cpp.parse_text(
        'Controller::Controller() : nav2::LifecycleNode("controller_server", "", options) {}',
        path="controller.cpp",
    )
    cpp.analyse([unit])
    assert cpp.file_report(unit)["nodes"][0]["name"] == "controller_server"


@pytest.mark.parametrize(
    ("options", "stated"),
    [
        ("rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)", True),
        ("rclcpp::NodeOptions().allow_undeclared_parameters(flag)", "unknown"),
        ("rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(false)", None),
        ("options", None),
    ],
)
def test_a_node_built_to_take_undeclared_parameters_says_so(options, stated):
    unit = cpp.parse_text(
        f'Stop::Stop(bool flag) : rclcpp::Node("safety_stop", {options}) {{}}',
        path="stop.cpp",
    )
    cpp.analyse([unit])
    (row,) = cpp.file_report(unit)["nodes"]
    assert row.get("auto_declare") == stated


@pytest.mark.parametrize(
    "created",
    [
        'auto node = rclcpp::Node::make_shared("add_two_ints_client");',
        'auto node = std::make_shared<rclcpp::Node>("add_two_ints_client", options);',
        'rclcpp::Node node("add_two_ints_client");',
        'auto node = rclcpp_lifecycle::LifecycleNode::make_shared("add_two_ints_client");',
    ],
)
def test_a_node_a_free_function_creates_from_a_node_type_is_named_by_its_literal(created):
    """demo_nodes_cpp add_two_ints_client stayed unnamed by launch while main named it."""
    unit = cpp.parse_text(f"int main(int argc, char ** argv) {{\n  {created}\n}}\n", path="c.cpp")
    cpp.analyse([unit])
    (row,) = cpp.file_report(unit)["nodes"]
    assert (row["name"], row["name_source"], row["symbol"]) == (
        "add_two_ints_client",
        "literal",
        "main",
    )


def test_a_node_type_inside_a_class_or_named_at_run_time_states_no_literal():
    unit = cpp.parse_text(
        'void Host::helper() { auto node = rclcpp::Node::make_shared("helper"); }\n'
        "int main() { auto node = rclcpp::Node::make_shared(name); }\n",
        path="c.cpp",
    )
    cpp.analyse([unit])
    report = cpp.file_report(unit)
    (row,) = report["nodes"]
    assert (row["symbol"], row["name"], row["name_source"]) == ("main", None, "unknown")
    assert [item["reason"] for item in report["unresolved"] if item["want"] == "node_name"] == [
        "not_stated_in_source"
    ]


@pytest.mark.parametrize(
    ("options", "stated"),
    [
        ("rclcpp::NodeOptions().use_global_arguments(false)", False),
        ("rclcpp::NodeOptions().use_global_arguments(flag)", "unknown"),
        ("rclcpp::NodeOptions().use_global_arguments(true)", None),
        ("options", None),
    ],
)
def test_a_node_that_reads_no_global_argument_says_so(options, stated):
    unit = cpp.parse_text(
        f'Stop::Stop(bool flag) : rclcpp::Node("safety_stop", {options}) {{}}',
        path="stop.cpp",
    )
    cpp.analyse([unit])
    (row,) = cpp.file_report(unit)["nodes"]
    assert row.get("use_global_arguments") == stated


def test_global_arguments_are_decided_per_construction_in_main():
    unit = cpp.parse_text(
        "int main(int argc, char ** argv) {\n"
        "  auto options = rclcpp::NodeOptions();\n"
        "  options.use_global_arguments(false);\n"
        "  auto own = std::make_shared<Own>(options);\n"
        "  auto shared = std::make_shared<Shared>();\n"
        "}\n",
        path="main.cpp",
    )
    cpp.analyse([unit])
    report = cpp.file_report(unit)
    rows = {row["class"]: row.get("use_global_arguments") for row in report["constructions"]}
    assert rows == {"Own": False, "Shared": None}


def test_a_main_that_builds_the_options_marks_its_construction():
    unit = cpp.parse_text(
        "int main(int argc, char ** argv) {\n"
        "  auto options = rclcpp::NodeOptions();\n"
        "  options.automatically_declare_parameters_from_overrides(true);\n"
        "  rclcpp::spin(std::make_shared<Stop>(options));\n"
        "}\n",
        path="main.cpp",
    )
    cpp.analyse([unit])
    (row,) = cpp.file_report(unit)["constructions"]
    assert row["auto_declare"] is True


HELPER_MAIN = (
    "int main(int argc, char ** argv) {\n"
    "  auto client = std::make_shared<rclcpp::Node>(\n"
    '    "param_client", rclcpp::NodeOptions().allow_undeclared_parameters(true));\n'
    "  rclcpp::spin(std::make_shared<ScanFilter>());\n"
    "}\n"
)


def test_a_helper_node_built_with_its_own_options_speaks_only_for_itself():
    """A parameter client `main` builds to take any parameter made every node of the file
    take any, so none of the launched node's keys was ever undeclared."""
    unit = cpp.parse_text(
        'ScanFilter::ScanFilter() : rclcpp::Node("scan_filter") {}\n' + HELPER_MAIN,
        path="main.cpp",
    )
    cpp.analyse([unit])
    report = cpp.file_report(unit)
    constructed = {row["class"]: row.get("auto_declare") for row in report["constructions"]}
    assert constructed == {"rclcpp::Node": True, "ScanFilter": None}
    classes = {row["class"]: row.get("auto_declare") for row in report["nodes"]}
    assert classes["ScanFilter"] is None


def test_options_built_where_no_construction_can_be_told_apart_mark_the_file():
    unit = cpp.parse_text(
        "rclcpp::NodeOptions make_options() {\n"
        "  return rclcpp::NodeOptions().allow_undeclared_parameters(true);\n"
        "}\n"
        'ScanFilter::ScanFilter() : rclcpp::Node("scan_filter", make_options()) {}\n',
        path="filter.cpp",
    )
    cpp.analyse([unit])
    (row,) = cpp.file_report(unit)["nodes"]
    assert row["auto_declare"] is True


def test_a_refused_region_can_hide_a_nav2_declaration():
    unit = cpp.parse_text(
        'void f() { FOO(auto p = declare_or_get_parameter<double>("gain", 1.0)); }',
        path="helper.cpp",
    )
    assert not unit.parsed
    assert unit.hidden_regions >= 1


PROSE = frozenset(
    {
        "note",
        "detail",
        "line_hint",
        "node_id_reason",
        "topic_id_reason",
        "qos_reason",
        "lookup_name_source",
        "registered_executable_source",
        "effective_needs",
        "count_reason",
        "declared_in_header",
        "confidence",
        "parse",
    }
)


def _tree_sitter() -> bool:
    try:
        cpp.load_parser()
    except CollectorError:
        return False
    return True


needs_cpp = pytest.mark.skipif(not _tree_sitter(), reason="the cpp extra is not installed")


def _strip(payload: Any) -> Any:
    if isinstance(payload, dict):
        return {key: _strip(value) for key, value in payload.items() if key not in PROSE}
    if isinstance(payload, list):
        return [_strip(item) for item in payload]
    return payload


@pytest.fixture(scope="module")
def code_root(fixtures_dir: Path) -> Path:
    return fixtures_dir / "ws_code"


@pytest.fixture(scope="module")
def code_ws(code_root: Path) -> Workspace:
    return Workspace.discover([RootSpec("src", str(code_root / "src"))])


@pytest.fixture(scope="module")
def units(code_ws: Workspace) -> list[cpp.Unit]:
    parser = cpp.load_parser()
    out: list[cpp.Unit] = []
    for path in cpp.source_paths(code_ws):
        package, relpath = code_ws.rel(path)
        out.append(
            cpp.parse_file(path, pkg=package or ids.REPO_PKG, relpath=relpath, parser=parser)
        )
    cpp.analyse(out)
    return sorted(out, key=lambda item: item.path)


@pytest.fixture(scope="module")
def reports(units: list[cpp.Unit], code_ws: Workspace, code_root: Path) -> dict[str, Any]:
    lookups, components = cpp.package_tables(code_ws)
    return {
        unit.relpath: cpp.file_report(
            unit, workspace_root=str(code_root), lookups=lookups, components=components
        )
        for unit in units
    }


@pytest.fixture(scope="module")
def expected(fixtures_dir: Path):
    def _load(name: str) -> dict[str, Any]:
        return json.loads((fixtures_dir / "expected" / "code" / name).read_text(encoding="utf-8"))

    return _load


def _atlas(code_root: Path) -> model.Atlas:
    roots = (RootSpec("src", str(code_root / "src")),)
    ws = Workspace.discover(list(roots))
    builder = model.AtlasBuilder()
    ctx = CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder)
    cpp.CppCodeCollector().collect(ctx)
    return builder.build()


# ---------------------------------------------------------------------------
# the optional dependency


def test_missing_extra_names_the_install_line(monkeypatch: pytest.MonkeyPatch) -> None:
    real = builtins.__import__

    def refuse(name: str, *args: Any, **kwargs: Any) -> Any:
        if name.startswith("tree_sitter"):
            raise ImportError(f"No module named {name!r}", name=name)
        return real(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", refuse)
    with pytest.raises(CollectorError) as caught:
        cpp.load_parser()
    message = str(caught.value)
    assert cpp.INSTALL_HINT in message
    # No release is on PyPI yet, and the line lands in an atlas, so it names no machine path.
    assert "pip install '<checkout>[cpp]' from a rosio checkout, or 'rosio[cpp]'" in message
    assert "tree-sitter-cpp" in message
    assert message.startswith("coded.cpp_code:")


@needs_cpp
def test_a_tree_sitter_that_takes_no_language_is_a_named_mismatch(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """0.25 removed `set_language`, so a `Parser(language)` refusal there is a version
    mismatch. Reporting it as an AttributeError from the fallback names the wrong problem."""
    import tree_sitter

    class Refusing:
        def __init__(self, *args: Any) -> None:
            if args:
                raise TypeError("Parser() takes no arguments")

    monkeypatch.setattr(tree_sitter, "Parser", Refusing)
    with pytest.raises(CollectorError) as caught:
        cpp.load_parser()
    assert "tree-sitter" in str(caught.value)
    assert cpp.TREE_SITTER_PIN[0] in str(caught.value)


@needs_cpp
def test_the_pinned_pair_parses_a_real_translation_unit(code_root: Path) -> None:
    unit = cpp.parse_file(
        str(code_root / "src/cx_nodes/src/cx_literal_endpoints.cpp"),
        pkg="cx_nodes",
        relpath="src/cx_literal_endpoints.cpp",
    )
    assert unit.parsed
    assert [item.qualname for item in unit.classes] == ["cx_nodes::LiteralEndpoints"] * 2
    assert len(unit.endpoints) == 9


# ---------------------------------------------------------------------------
# the committed expectations


@pytest.mark.parametrize(
    ("relpath", "name"),
    [
        ("src/cx_component_node.cpp", "cx_component_node.json"),
        ("src/cx_layer_plugin.cpp", "cx_layer_plugin.json"),
        ("src/cx_lifecycle_node.cpp", "cx_lifecycle_node.json"),
        ("src/cx_literal_endpoints.cpp", "cx_literal_endpoints.json"),
        ("src/cx_param_endpoints.cpp", "cx_param_endpoints.json"),
        ("src/cx_tf_node.cpp", "cx_tf_node.json"),
        ("src/cx_wrappers.cpp", "cx_wrappers.json"),
    ],
)
@needs_cpp
def test_report_matches_the_expectation(
    reports: dict[str, Any], expected, relpath: str, name: str
) -> None:
    assert _strip(reports[relpath]) == _strip(expected(name))


@needs_cpp
def test_the_header_holds_no_endpoint_and_no_node_of_its_own(reports: dict[str, Any]) -> None:
    """The class is declared in the header and named in the source. After the join both
    files know the class, and only the file that defines the constructor reports the node."""
    header = reports["include/cx_nodes/cx_lifecycle_node.hpp"]
    assert header["nodes"] == []
    assert header["endpoints"] == []
    assert header["parameters"] == []


@needs_cpp
def test_totals_match_the_index(reports: dict[str, Any], expected) -> None:
    index = expected("index.json")["per_language"]["cpp"]
    with_content = [
        report
        for report in reports.values()
        if report["nodes"] or report["endpoints"] or report["parameters"]
    ]
    totals: dict[str, int] = {}
    for report in with_content:
        for key, value in report["counts"].items():
            totals[key] = totals.get(key, 0) + value
        totals["nodes"] = totals.get("nodes", 0) + len(report["nodes"])
    totals["files"] = len(with_content)
    for key, value in index.items():
        assert totals.get(key, 0) == value, key


@needs_cpp
def test_the_five_blind_spots_are_the_committed_ones(reports: dict[str, Any], expected) -> None:
    index = expected("index.json")
    wanted = sorted(
        (Path(row["file"]).name, row["want"], row["expr"], row["reason"])
        for row in index["unresolved_inventory"]
        if row["file"].endswith(".cpp")
    )
    got = sorted(
        (Path(relpath).name, row["want"], row["expr"], row["reason"])
        for relpath, report in reports.items()
        for row in report["unresolved"]
    )
    assert got == wanted


# ---------------------------------------------------------------------------
# the two rules that are specific to C++


@needs_cpp
def test_the_message_type_comes_from_the_template_argument(reports: dict[str, Any]) -> None:
    endpoints = reports["src/cx_literal_endpoints.cpp"]["endpoints"]
    by_name = {row["name"]: row for row in endpoints if row["name"]}
    assert by_name["/cx/odom"]["type"] == "nav_msgs/msg/Odometry"
    assert by_name["/cx/odom"]["type_id"] == "type:nav_msgs|msg|Odometry"
    assert by_name["/cx/reset"]["type"] == "std_srvs/srv/Trigger"
    # The type is never read from a string: no argument of these calls holds a type name.
    assert all(not str(row["raw"]).count("msg") for row in endpoints if row["kind"] != "timer")


@needs_cpp
def test_a_name_this_pass_cannot_follow_stays_unresolved(reports: dict[str, Any]) -> None:
    report = reports["src/cx_param_endpoints.cpp"]
    loop = [row for row in report["endpoints"] if row["name_state"] == "unresolved"]
    assert len(loop) == 1
    assert loop[0]["raw"] == "name"
    assert loop[0]["name"] is None
    assert loop[0]["count_state"] == "unknown"
    assert loop[0]["type"] == "sensor_msgs/msg/PointCloud2"


@needs_cpp
def test_a_relative_name_mints_no_topic_id(reports: dict[str, Any]) -> None:
    row = next(
        item
        for item in reports["src/cx_component_node.cpp"]["endpoints"]
        if item["name"] == "markers"
    )
    assert row["absolute"] is False
    assert row["topic_id"] is None
    atlas_names = {"topic:markers", "topic:/markers"}
    assert not atlas_names & set(_atlas(Path("tests/fixtures/ws_code")).entities)


@needs_cpp
def test_a_plugin_parameter_is_a_suffix_bound_to_the_plugin(reports: dict[str, Any]) -> None:
    rows = reports["src/cx_layer_plugin.cpp"]["plugin_parameters"]
    assert [row["suffix"] for row in rows] == ["decay_s", "enabled", "topic"]
    assert all(not row["suffix"].startswith(".") for row in rows)
    assert rows[2]["pparam_id"] == "pparam:cx_core::Layer|cx_nodes/DecayLayer|topic"
    assert rows[2]["asymmetric"] is True
    assert rows[2]["read_spelling"] == 'name_ + "." + "topic"'


@needs_cpp
def test_a_user_written_declare_wrapper_is_followed(reports: dict[str, Any]) -> None:
    report = reports["src/cx_lifecycle_node.cpp"]
    assert report["parameters"][0]["via_wrapper"] == "cx_nodes::declareOrGetParameter"
    assert report["parameters"][0]["default"] == "/cx/cmd_vel"
    subscriber = next(row for row in report["endpoints"] if row["kind"] == "subscriber")
    assert subscriber["name"] == "/cx/cmd_vel"
    assert subscriber["name_state"] == "resolved_from_parameter"


@needs_cpp
def test_a_constructor_that_takes_a_name_is_not_a_declare_wrapper(units) -> None:
    """`TfNode(const std::string & node_name)` declares four parameters and takes a name
    string. Treating it as a wrapper loses all four."""
    unit = next(item for item in units if item.relpath == "src/cx_tf_node.cpp")
    assert unit.wrappers == {}
    assert len(unit.params) == 4


# ---------------------------------------------------------------------------
# QoS is what the source states


@needs_cpp
def test_qos_records_only_the_properties_the_source_names() -> None:
    assert cpp.qos_of("10") == {
        "raw": "10",
        "form": "depth",
        "history": "keep_last",
        "depth": 10,
        "reliability": None,
        "durability": None,
        "preset": None,
    }
    preset = cpp.qos_of("rclcpp::SensorDataQoS()")
    assert preset["preset"] == "sensor_data"
    assert preset["depth"] is None and preset["reliability"] is None
    chained = cpp.qos_of("rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local()")
    assert chained["depth"] == 1
    assert chained["reliability"] == "reliable"
    assert chained["durability"] == "transient_local"


@needs_cpp
def test_a_qos_variable_is_followed_including_its_mutation(reports: dict[str, Any]) -> None:
    endpoints = reports["src/cx_literal_endpoints.cpp"]["endpoints"]
    estop = next(row for row in endpoints if row["name"] == "/cx/estop")
    assert estop["qos"]["form"] == "variable"
    assert estop["qos"]["mutated"] is True
    assert estop["qos"]["depth"] == 1
    assert estop["qos"]["durability"] == "transient_local"
    assert estop["options_position"] == 4


@needs_cpp
def test_a_wrapper_endpoint_carries_no_invented_qos(reports: dict[str, Any]) -> None:
    for row in reports["src/cx_wrappers.cpp"]["endpoints"]:
        assert row["via_known_wrapper"]
        assert row["qos"] is None


# ---------------------------------------------------------------------------
# the atlas the collector writes


@needs_cpp
def test_the_collector_output_validates(code_root: Path) -> None:
    atlas = _atlas(code_root)
    assert schema.validate(json.loads(atlas.to_bytes())) == []
    assert atlas.layers[model.Layer.CODED.value].present


@needs_cpp
def test_two_runs_give_identical_canonical_bytes(code_root: Path) -> None:
    assert _atlas(code_root).canonical_bytes() == _atlas(code_root).canonical_bytes()


@needs_cpp
def test_no_node_id_is_formed_from_source(code_root: Path) -> None:
    """The code states a name and never a namespace. An fqn with an assumed namespace is a
    fabricated identity, so the name rides on the symbol and the join supplies the rest."""
    atlas = _atlas(code_root)
    assert [entity for entity in atlas.entities if entity.startswith("node:")] == []
    sym = atlas.entities[
        "sym:cx_nodes|src/cx_literal_endpoints.cpp|cx_nodes::LiteralEndpoints::LiteralEndpoints"
    ]
    fact = sym.fact(model.Layer.CODED.value)
    assert fact is not None
    assert fact.values["nodes"][0]["name"] == "cx_literal"
    assert fact.values["nodes"][0]["node_id"] is None


@needs_cpp
def test_a_symbol_fact_carries_the_record_block_and_nothing_else(code_root: Path) -> None:
    """AGENTS section 2.2. The block is the whole of what a coded collector hands the join,
    so a key outside it is written for nobody: the join skips it and every later pass reads
    the join. The symbol fact spelled its node rows `node` and carried a `parameter_callbacks`
    list no pass has ever read, and both are gone."""
    atlas = _atlas(code_root)
    facts = [
        entity.fact(model.Layer.CODED.value)
        for key, entity in sorted(atlas.entities.items())
        if key.startswith("sym:")
    ]
    assert facts and all(fact is not None for fact in facts)
    written = {key for fact in facts for key in fact.values}
    assert written <= set(join.RECORD_KEYS), sorted(written - set(join.RECORD_KEYS))
    assert "nodes" in written


@needs_cpp
def test_no_absolute_path_and_no_line_number_reaches_an_id(code_root: Path) -> None:
    atlas = _atlas(code_root)
    for entity in atlas.entities:
        assert str(code_root) not in entity
        assert "/tmp/" not in entity
    for handle, edge in atlas.edges.items():
        assert str(code_root) not in handle
        assert edge.key == "" or ".cpp:" not in edge.key


@needs_cpp
def test_the_plugin_parameter_chain_is_a_code_default(code_root: Path) -> None:
    atlas = _atlas(code_root)
    entity = atlas.entities["pparam:cx_core::Layer|cx_nodes/DecayLayer|decay_s"]
    assert entity.state == model.ParamState.OTHER.value
    assert [row.mechanism for row in entity.candidates] == [model.Mechanism.CODE_DEFAULT.value]
    assert entity.candidates[0].value == 3.0
    assert entity.candidates[0].wins is False
    assert entity.effective == {}


@needs_cpp
def test_one_unreadable_file_does_not_empty_the_layer(code_root: Path, tmp_path: Path) -> None:
    """A file the reader cannot finish is recorded on the run and the other files still land.
    A generated header nested deep enough to exhaust the interpreter stack is the real case."""
    tree = tmp_path / "ws"
    shutil.copytree(code_root / "src", tree / "src")
    depth = 2000
    (tree / "src/cx_nodes/src/aaa_deep.cpp").write_text(
        "namespace fx {\nint x = " + "(" * depth + "1" + ")" * depth + ";\n}\n",
        encoding="utf-8",
    )
    roots = (RootSpec("src", str(tree / "src")),)
    ws = Workspace.discover(list(roots))
    builder = model.AtlasBuilder()
    cpp.CppCodeCollector().collect(
        CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder)
    )
    atlas = builder.build()
    assert len(atlas.entities) == len(_atlas(code_root).entities)
    recorded = [
        row
        for node in atlas.by_kind("run")
        for row in node.attrs.get("exceptions") or []
        if "aaa_deep" in row["path"]
    ]
    assert len(recorded) == 1
    assert recorded[0]["exc_type"] == "RecursionError"
    assert str(tree) not in recorded[0]["path"]


@needs_cpp
def test_the_wrapper_endpoints_come_from_the_one_shared_table() -> None:
    """`rosio/resolve/wrappers.py` owns what a construction creates. A second copy here
    would let the collector and the join disagree about the same library."""
    for symbol in cpp.KNOWN_WRAPPERS:
        assert wrappers.lookup(symbol, "cpp") is not None, symbol
        assert set(cpp.KNOWN_WRAPPERS[symbol]) <= {"reason", "broadcaster"}
    assert cpp.wrapper_endpoints("tf2_ros::TransformListener") == (
        ("subscriber", "/tf", "tf2_msgs/msg/TFMessage"),
        ("subscriber", "/tf_static", "tf2_msgs/msg/TFMessage"),
    )
    assert cpp.wrapper_endpoints("message_filters::Subscriber") == ()
    assert cpp.wrapper_takes_name("message_filters::Subscriber") is True
    assert cpp.wrapper_takes_name("diagnostic_updater::Updater") is False


@needs_cpp
def test_a_refused_region_costs_the_file_nothing_but_a_blind_spot(tmp_path: Path) -> None:
    """A macro that expands to a type is not parsable without the preprocessor. What is
    outside the region is still read, because the alternative is losing real endpoints."""
    source = tmp_path / "macro.cpp"
    source.write_text(
        "#include <rclcpp/rclcpp.hpp>\n"
        "namespace fx {\n"
        "class N : public rclcpp::Node {\n"
        " public:\n"
        '  N() : rclcpp::Node("fx") {\n'
        "    VF(4) pabcd;\n"
        '    pub_ = create_publisher<std_msgs::msg::Bool>("/fx/out", 10);\n'
        "  }\n"
        "};\n"
        "}\n",
        encoding="utf-8",
    )
    unit = cpp.parse_file(str(source), pkg="fx", relpath="src/macro.cpp")
    assert unit.parsed is False
    assert unit.error_lines
    assert [item.name.value for item in unit.endpoints] == ["/fx/out"]


def test_a_refused_region_is_a_blind_spot_only_when_it_holds_a_ros_call() -> None:
    """`"%" PRId64` in a format string and a visibility macro before a function are refused
    by the grammar and hide nothing; a refused region around a publisher hides the
    publisher."""
    harmless = cpp.parse_text(
        "#include <cinttypes>\n"
        'void f(int64_t v) { RCLCPP_INFO(get_logger(), "%" PRId64, v); }\n'
        "DEMO_LOCAL void g();\n",
        path="harmless.cpp",
    )
    assert harmless.parsed is False
    assert harmless.hidden_regions == 0
    hiding = cpp.parse_text(
        'void f() { FOO(auto p = create_publisher<B>("/out", 10)); }\n',
        path="hiding.cpp",
    )
    assert hiding.parsed is False
    assert hiding.hidden_regions >= 1


NON_FINITE_CPP = """
#include <rclcpp/rclcpp.hpp>

class N : public rclcpp::Node {
public:
  N() : rclcpp::Node("n") {
    this->declare_parameter("good_a", 1.0);
    this->declare_parameter("gain", 1e400);
    this->declare_parameter("good_b", 2.0);
    pub_ = create_publisher<std_msgs::msg::Bool>("/cx/out", 10);
  }
private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
};
"""

OTHER_CPP = """
#include <rclcpp/rclcpp.hpp>

class M : public rclcpp::Node {
public:
  M() : rclcpp::Node("m") {
    pub_ = create_publisher<std_msgs::msg::Bool>("/cx/other", 10);
  }
private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
};
"""


def _cpp_package(tmp_path: Path, files: dict[str, str], name: str = "cx_inf") -> Path:
    package = tmp_path / "src" / name
    (package / "src").mkdir(parents=True)
    (package / "package.xml").write_text(f'<package format="3"><name>{name}</name></package>')
    for relative, text in files.items():
        (package / "src" / relative).write_text(text)
    return tmp_path


@needs_cpp
def test_a_default_json_cannot_hold_costs_one_blind_spot_and_not_the_pass(
    tmp_path: Path,
) -> None:
    """The quietest of the four doors: this used to end the C++ pass with no finding, a
    resolution rate of 1.000 and a green error gate over the loss."""
    root = _cpp_package(tmp_path, {"n.cpp": NON_FINITE_CPP, "m.cpp": OTHER_CPP})
    atlas = _atlas(root)
    symbol = ids.build_sym("cx_inf", "src/n.cpp", "N::N")
    entry = next(item for item in atlas.unresolved if item.owner == symbol)
    assert entry.want == "param_value"
    assert "json has no infinity" in entry.reason
    assert atlas.entities[symbol].unresolved == [entry.id]
    assert "topic:/cx/out" in atlas.entities
    assert "topic:/cx/other" in atlas.entities
    run = next(entity for entity in atlas.entities.values() if entity.kind == "run")
    assert run.attrs["exceptions"] == []
    assert model.dumps(atlas.to_json())
    assert schema.validate(json.loads(atlas.to_bytes())) == []


TIMER_CPP = """
#include <rclcpp/rclcpp.hpp>

class T : public rclcpp::Node {
public:
  T() : rclcpp::Node("t") {
    good_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {});
    bad_ = create_wall_timer(std::chrono::duration<double>(1e400), [this]() {});
  }
private:
  rclcpp::TimerBase::SharedPtr good_;
  rclcpp::TimerBase::SharedPtr bad_;
};
"""


@needs_cpp
def test_a_timer_period_json_cannot_hold_records_a_blind_spot(tmp_path: Path) -> None:
    """This one left no trace at all: the rate resolved count dropped, no blind spot
    appeared, nothing raised, and the rate read 1.000 over the smaller denominator."""
    atlas = _atlas(_cpp_package(tmp_path, {"t.cpp": TIMER_CPP}))
    symbol = ids.build_sym("cx_inf", "src/t.cpp", "T::T")
    entry = next(item for item in atlas.unresolved if item.want == "rate")
    assert entry.owner == symbol
    assert entry.expr == "std::chrono::duration<double>(1e400)"
    assert entry.reason == f"{model.NON_FINITE}: inf"
    assert entry.id in atlas.entities[symbol].unresolved
    assert atlas.counters["resolution_rate"]["rate"] == 0.5
    run = next(entity for entity in atlas.entities.values() if entity.kind == "run")
    assert run.attrs["exceptions"] == []
    assert schema.validate(atlas.to_json()) == []


@needs_cpp
def test_a_timer_period_that_did_not_fold_is_not_a_refusal(tmp_path: Path) -> None:
    """An expression this layer cannot evaluate is a different answer from one it evaluated
    and cannot write, and only the second one names a reason."""
    text = TIMER_CPP.replace("std::chrono::duration<double>(1e400)", "period_from_config()")
    root = _cpp_package(tmp_path, {"t.cpp": text})
    unit = cpp.parse_file(
        str(root / "src" / "cx_inf" / "src" / "t.cpp"), pkg="cx_inf", relpath="src/t.cpp"
    )
    cpp.analyse([unit])
    report = cpp.file_report(unit)
    timers = [row for row in report["endpoints"] if row["kind"] == "timer"]
    states = sorted(row["period_state"] for row in timers)
    assert states == ["literal", "unresolved"]
    assert [row for row in report["unresolved"] if row["want"] == "rate"] == []


# ---------------------------------------------------------------------------
# inputs_present, the three states of a pass
#
# The four collectors below are the ones that answered `completed` where an absent input
# was the honest state. They are tested together because the contract they meet is one
# sentence of AGENTS section 13 point 8 and not four.


def _empty_ws(tmp_path: Path) -> tuple[Workspace, Config]:
    (tmp_path / "src" / "px").mkdir(parents=True)
    (tmp_path / "src" / "px" / "package.xml").write_text(
        '<package format="3"><name>px</name></package>'
    )
    roots = (RootSpec("src", str(tmp_path / "src")),)
    return Workspace.discover(list(roots)), Config(roots=roots, profiles=())


def test_a_workspace_with_no_cpp_is_not_attempted_and_never_loads_the_parser(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """An installed ROS tree holds zero C++ sources. On a machine without the extra this
    pass was recorded as refused and the completeness rule fired at error on a healthy
    workspace, because the parser was loaded before the input list was taken."""
    ws, cfg = _empty_ws(tmp_path)
    monkeypatch.setattr(cpp, "load_parser", _never)
    collector = cpp.CppCodeCollector()
    ctx = CollectContext(ws=ws, cfg=cfg, builder=model.AtlasBuilder())
    assert collector.inputs_present(ctx) == cpp.NO_SOURCE
    collector.collect(ctx)
    assert ctx.builder.build().entities == {}


def _never() -> Any:
    raise CollectorError("the cpp extra is not installed")


def test_a_workspace_holding_cpp_is_attempted(tmp_path: Path) -> None:
    root = _cpp_package(tmp_path, {"t.cpp": "int main() { return 0; }\n"})
    roots = (RootSpec("src", str(root / "src")),)
    ws = Workspace.discover(list(roots))
    ctx = CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=model.AtlasBuilder())
    assert cpp.CppCodeCollector().inputs_present(ctx) is True


def test_the_orchestrator_records_a_workspace_with_no_cpp_as_not_attempted(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    ws, cfg = _empty_ws(tmp_path)
    monkeypatch.setattr(cpp, "load_parser", _never)
    ctx = CollectContext(ws=ws, cfg=cfg, builder=model.AtlasBuilder())
    state, reason, failure = extract._collect_one(cpp.CppCodeCollector(), ctx)
    assert state == model.PassState.NOT_ATTEMPTED.value
    assert reason == cpp.NO_SOURCE
    assert failure is None


def test_a_recording_collector_with_no_root_and_with_an_empty_root(tmp_path: Path) -> None:
    """Recorded is the one layer a workspace may legitimately never have, and a bag root
    that turned out to hold nothing is a different sentence from no bag root at all."""
    ws, cfg = _empty_ws(tmp_path)
    collector = bag.BagCollector()
    bare = CollectContext(ws=ws, cfg=cfg, builder=model.AtlasBuilder())
    assert collector.inputs_present(bare) == bag.NO_ROOT
    empty = tmp_path / "bags"
    empty.mkdir()
    roots = (*cfg.roots, RootSpec("bag", str(empty)))
    with_root = Config(roots=roots, profiles=())
    ctx = CollectContext(ws=ws, cfg=with_root, builder=model.AtlasBuilder())
    assert collector.inputs_present(ctx) == bag.NO_RECORDING
    assert extract._collect_one(collector, ctx)[0] == model.PassState.NOT_ATTEMPTED.value


def test_the_launch_collector_with_no_profile_is_not_attempted(tmp_path: Path) -> None:
    ws, cfg = _empty_ws(tmp_path)
    ctx = CollectContext(ws=ws, cfg=cfg, builder=model.AtlasBuilder())
    collector = launch_declared.LaunchDeclaredCollector()
    assert collector.inputs_present(ctx) == launch_declared.NO_PROFILE
    assert extract._collect_one(collector, ctx)[0] == model.PassState.NOT_ATTEMPTED.value


def test_the_parameter_collector_with_no_walk_is_not_attempted(tmp_path: Path) -> None:
    """The worst of the three: a parameter pass handed no walk is indistinguishable from
    one that read every parameters file of the workspace, and the launch pass that died
    leaves exactly the same empty walk list behind."""
    ws, cfg = _empty_ws(tmp_path)
    ctx = CollectContext(ws=ws, cfg=cfg, builder=model.AtlasBuilder())
    collector = params.ParamsCollector()
    assert collector.inputs_present(ctx) == params.NO_WALK
    assert extract._collect_one(collector, ctx)[0] == model.PassState.NOT_ATTEMPTED.value
    assert model.completeness(ctx.builder.build())["refused"] == 0


# ---------------------------------------------------------------------------
# one shape for a refused entity
#
# A refused entity states `loadable`, `refused_because` and `refused_detail` in `attrs` and
# writes no fact. Two collectors used to write a `parsed false` fact instead, which put a
# layer in the entity's `layers`, so a consumer filtering entities by layer met those two
# refusals and none of the others.


def _py_package(tmp_path: Path, files: dict[str, str]) -> Path:
    package = tmp_path / "src" / "px"
    (package / "px").mkdir(parents=True)
    (package / "package.xml").write_text('<package format="3"><name>px</name></package>')
    for relative, text in files.items():
        (package / "px" / relative).write_text(text)
    return tmp_path


def _collected(tmp_path: Path, collector: Any) -> model.Atlas:
    roots = (RootSpec("src", str(tmp_path / "src")),)
    ws = Workspace.discover(list(roots))
    builder = model.AtlasBuilder()
    collector.collect(CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder))
    return builder.build()


def test_a_python_source_that_does_not_parse_states_its_refusal_in_attrs(
    tmp_path: Path,
) -> None:
    root = _py_package(tmp_path, {"good.py": "x = 1\n", "broken.py": "def broken(:\n"})
    atlas = _collected(root, py_code.PyCodeCollector())
    broken = atlas.entities[ids.build_file("px", "px/broken.py")]
    assert broken.attrs["loadable"] is False
    assert broken.attrs["refused_because"] == py_code.REFUSED_PY_SOURCE
    assert broken.attrs["refused_detail"]
    assert broken.facts == []
    assert broken.layers == []


def test_a_python_source_that_does_not_parse_keeps_both_denominators(tmp_path: Path) -> None:
    """The blind spot used to sit under `other`, so `node_name` and `topic_name` left the
    counter with the file and both read 1.000 over a smaller denominator."""
    root = _py_package(tmp_path, {"good.py": GOOD_PY, "broken.py": "def broken(:\n"})
    atlas = _collected(root, py_code.PyCodeCollector())
    broken = ids.build_file("px", "px/broken.py")
    wants = sorted(entry.want for entry in atlas.unresolved if entry.owner == broken)
    assert wants == ["node_name", "topic_name"]
    assert len(atlas.entities[broken].unresolved) == 2
    rates = atlas.counters["resolution_rate"]
    assert rates["node_name"] < 1.0
    assert rates["topic_name"] < 1.0
    assert schema.validate(atlas.to_json()) == []


GOOD_PY = """\
from rclpy.node import Node
from std_msgs.msg import Bool


class A(Node):
    def __init__(self):
        super().__init__('a')
        self.p = self.create_publisher(Bool, '/a/out', 1)
"""


def test_a_parameters_file_that_refuses_to_load_states_its_refusal_in_attrs(
    tmp_path: Path,
) -> None:
    package = tmp_path / "src" / "px"
    package.mkdir(parents=True)
    (package / "package.xml").write_text('<package format="3"><name>px</name></package>')
    torn = package / "torn.yaml"
    torn.write_text("a: [1, 2\n")
    entry = package / "px.launch.py"
    entry.write_text("def generate_launch_description():\n    pass\n")
    roots = (RootSpec("src", str(tmp_path / "src")),)
    ws = Workspace.discover(list(roots))
    walk = records.LaunchWalk(
        entry_path=str(entry),
        profile="nav",
        nodes=[
            records.ResolvedNode(
                pkg="px",
                executable="px_node",
                namespace="/",
                name="px_node",
                name_source="literal",
                params=(records.ParamSource(order=0, kind="repo_file", path=str(torn)),),
            )
        ],
    )
    builder = model.AtlasBuilder()
    ctx = CollectContext(
        ws=ws,
        cfg=Config(roots=roots, profiles=()),
        builder=builder,
        declared=records.DeclaredBundle(walks=[walk]),
        profile="nav",
    )
    collector = params.ParamsCollector()
    assert collector.inputs_present(ctx) is True
    collector.collect(ctx)
    atlas = builder.build()
    node = atlas.entities[ids.build_file("px", "torn.yaml")]
    assert node.attrs["loadable"] is False
    assert node.attrs["refused_because"] == params.REFUSED_PARAMS_FILE
    assert node.attrs["refused_detail"]
    assert node.facts == []
    assert node.layers == []
    assert [entry.owner for entry in atlas.unresolved] == [node.id]


IMAGE_NODE = """
DebayerNode::DebayerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("debayer_node", options)
{
  auto node_base = this->get_node_base_interface();
  image_topic_ = node_base->resolve_topic_or_service_name("image_raw", false);
  std::string mono_topic = node_base->resolve_topic_or_service_name("image_mono", false);
  image_transport::TransportHints hints(*this);
  sub_raw_ = image_transport::create_subscription(
    *this, image_topic_, [this](const Image::ConstSharedPtr & msg) {imageCb(msg);},
    hints.getTransport(), rclcpp::SensorDataQoS());
  pub_mono_ = image_transport::create_publisher(*this, mono_topic, rclcpp::SystemDefaultsQoS());
  pub_ = image_transport::create_camera_publisher(this, "left/image_rect");
  sub_ = image_transport::create_camera_subscription(this, "image", cb, "raw");
  image_transport::ImageTransport it(shared_from_this());
  auto legacy = it.advertiseCamera("/cam/image", 1);
  depth_sub_.subscribe(*this, "depth", hints.getTransport(), rclcpp::SystemDefaultsQoS());
}
"""


def test_image_transport_calls_bind_their_topic_and_never_the_node():
    """image_pipeline: every image_transport free function takes the node first."""
    unit = cpp.parse_text(IMAGE_NODE, path="debayer.cpp")
    cpp.analyse([unit])
    rows = {
        (row["kind"], row.get("name"), row.get("type"))
        for row in cpp.file_report(unit)["endpoints"]
    }
    image, info = "sensor_msgs/msg/Image", "sensor_msgs/msg/CameraInfo"
    assert rows == {
        ("subscriber", "image_raw", image),
        ("publisher", "image_mono", image),
        ("publisher", "left/image_rect", image),
        ("publisher", "left/camera_info", info),
        ("subscriber", "image", image),
        ("subscriber", "camera_info", info),
        ("publisher", "/cam/image", image),
        ("publisher", "/cam/camera_info", info),
        ("subscriber", "depth", image),
    }
    assert "*this" not in json.dumps(cpp.file_report(unit))


def test_a_rclcpp_member_publisher_keeps_its_first_argument_as_the_topic():
    unit = cpp.parse_text(
        'void N::start() { pub_ = create_publisher<std_msgs::msg::String>("chatter", 10); }',
        path="n.cpp",
    )
    cpp.analyse([unit])
    (row,) = cpp.file_report(unit)["endpoints"]
    assert (row["kind"], row["name"]) == ("publisher", "chatter")


def test_a_time_travelling_lookup_reads_the_source_from_the_third_argument():
    """pcl_ros asks `lookupTransform(target, target_time, source, source_time, fixed)`, and
    the time became the source frame."""
    calls = """
        buffer.lookupTransform("map", target_time, "laser", in.header.stamp, "odom");
        buffer.canTransform("map", now(), "camera", stamp, "odom", timeout);
        buffer.canTransform("map", "imu", stamp, timeout, &error);
        buffer.lookupTransform("map", "wheel", stamp);
        """
    unit = cpp.parse_text("void Filter::run() {" + calls + "}", path="f.cpp")
    cpp.analyse([unit])
    rows = cpp.file_report(unit)["tf"]["lookups"]
    assert sorted((row["target"], row["source"]) for row in rows) == [
        ("map", "camera"),
        ("map", "imu"),
        ("map", "laser"),
        ("map", "wheel"),
    ]


def test_a_descriptor_field_names_the_parameter_it_declares():
    """pcl_ros and image_view assign `desc.name` and declare by it, and every such
    declaration was a param_name blind spot."""
    body = """
        rcl_interfaces::msg::ParameterDescriptor negative_desc;
        negative_desc.name = "negative";
        negative_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
        declare_parameter(negative_desc.name, rclcpp::ParameterValue(false), negative_desc);
        """
    unit = cpp.parse_text("void CropBox::setup() {" + body + "}", path="crop_box.cpp")
    cpp.analyse([unit])
    rows = cpp.file_report(unit)["parameters"]
    assert [row["name"] for row in rows] == ["negative"]


def test_cast_and_empty_string_defaults_fold_and_a_local_is_named_as_one():
    body = """
        declare_parameter("rate", static_cast<double>(0));
        declare_parameter("count", static_cast<int>(3));
        declare_parameter("frame", std::string());
        declare_parameter("gain", tmp_val);
        """
    unit = cpp.parse_text("void Pub::setup() {" + body + "}", path="pub.cpp")
    cpp.analyse([unit])
    report = cpp.file_report(unit)
    rows = {row["name"]: row for row in report["parameters"]}
    assert rows["rate"]["default"] == 0.0 and isinstance(rows["rate"]["default"], float)
    assert rows["count"]["default"] == 3
    assert rows["frame"]["default"] == ""
    (blind,) = [row for row in report["unresolved"] if row["want"] == "param_value"]
    assert blind["reason"] == "local_variable"


def test_a_read_behind_has_parameter_is_a_checked_read():
    source = """
    void Node::configure() {
      if (this->has_parameter("gain")) { auto g = this->get_parameter("gain").as_double(); }
      auto w = this->get_parameter("watchdog").as_double();
    }
    """
    unit = cpp.parse_text(source, path="guard.cpp")
    cpp.analyse([unit])
    apis = {item.name.value: item.api for item in unit.reads}
    assert apis == {"gain": "get_parameter_checked", "watchdog": "get_parameter_accessor"}
