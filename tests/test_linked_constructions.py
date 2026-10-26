import shutil
from pathlib import Path

import pytest

from rosio import extract, join, model, schema
from rosio.collect import cpp_code as cpp
from rosio.config import Config, Entrypoint, ProfileSpec, RootSpec
from rosio.workspace import Workspace

pytestmark = pytest.mark.cpp


@pytest.mark.parametrize(
    "statement",
    [
        "std::make_shared<p::Filter<int>>(options)",
        "std::make_unique<p::Filter<int>>(options)",
        "p::Filter<int>(options)",
        "new p::Filter<int>(options)",
    ],
)
def test_main_construction_rows_keep_literal_types(statement):
    unit = cpp.parse_text(
        "namespace p { template<typename T> class Filter {}; } "
        f"int main() {{ auto node = {statement}; }}"
    )
    assert unit.constructions == [
        {"class": "p::Filter<int>", "owner_symbol": "main", "line_hint": 1}
    ]
    assert "main" in unit.symbols


def test_declaration_constructions_and_nonliteral_types():
    unit = cpp.parse_text("""
int main() {
  p::Filter<int> filter(1);
  auto unknown = value;
  auto other = decltype(value)(options);
  auto deferred = [] { return p::Hidden(); };
}
void helper() { p::Hidden hidden(options); }
""")
    assert [row["class"] for row in unit.constructions] == ["p::Filter<int>"]


@pytest.mark.parametrize(
    "declaration,target",
    [
        ("using Alias = Filter<int>;", "Filter<int>"),
        ("typedef Filter<int> Alias;", "Filter<int>"),
        ("using Alias = other::Filter<float>;", "other::Filter<float>"),
        ("typedef other::Filter<float> Alias;", "other::Filter<float>"),
    ],
)
def test_namespace_type_aliases(declaration, target):
    unit = cpp.parse_text(f"namespace p {{ namespace detail {{ {declaration} }} }}")
    assert unit.type_aliases == [
        {
            "name": "p::detail::Alias",
            "target": target,
            "namespace": "p::detail",
            "line_hint": 1,
        }
    ]


def test_local_aliases_do_not_escape_their_scope():
    unit = cpp.parse_text("void helper() { using A = Hidden; } class C { using A = Hidden; };")
    assert unit.type_aliases == []


def test_template_definitions_are_distinct_from_explicit_instantiations():
    unit = cpp.parse_text("""
namespace p {
template<typename T> class Filter {};
template<typename T> void Filter<T>::initialize() {
  create_publisher<std_msgs::msg::String>("filtered", 10);
}
}
template class p::Filter<int>;
""")
    assert unit.template_definitions == {"p::Filter": 1}
    assert not unit.template_specializations
    assert unit.endpoints[0].sym == "p::Filter<T>::initialize"


def test_forward_template_keeps_a_zero_definition_count():
    unit = cpp.parse_text("namespace p { template<class T> class Filter; }")
    assert unit.template_definitions == {"p::Filter": 0}


def test_template_binding_requires_one_definition():
    classes = {"p::Filter", "p::Filter<int>"}
    aliases = {"p::Alias": {("Filter<int>", "p")}}
    assert (
        join._resolve_class("p::Alias", "", classes, aliases, templates={"p::Filter": True})
        == "p::Filter"
    )
    assert join._resolve_class("p::Alias", "", classes, aliases, templates={}) == ""
    assert join._resolve_class("p::Filter<int>", "", classes, {}, templates={}) == ""
    assert join._resolve_class("p::Filter", "", classes, {}, templates={"p::Filter": False}) == ""


@pytest.mark.parametrize("namespace", ["p", "p::inner"])
def test_refusal_candidates_follow_relative_alias_targets(namespace):
    aliases = {
        f"{namespace}::Alias": {("detail::Impl", namespace)},
        "p::detail::Impl": {("Filter<int>", "p::detail")},
    }
    assert "Filter" in join._class_candidates(f"{namespace}::Alias", aliases)


def test_class_resolution_refuses_ambiguity_cycles_and_external_bases():
    classes = {"a::Filter", "b::Filter", "p::Filter"}
    aliases = {"p::Alias": {("Filter<int>", "p")}}
    assert join._resolve_class("p::Alias", "", classes, aliases) == "p::Filter"
    assert join._resolve_class("Filter", "", classes, aliases) == ""
    assert join._resolve_class("rclcpp::Node", "", classes, aliases) == ""
    assert join._resolve_class("T", "", classes, aliases) == ""
    aliases["p::Alias"] = {("Alias", "p")}
    assert join._resolve_class("p::Alias", "", classes, aliases) == ""


def test_constructor_calls_follow_types_in_other_package_headers():
    header = cpp.parse_text("namespace p { class Filter {}; using Alias = Filter; }", pkg="p")
    source = cpp.parse_text("int main() { auto filter = p::Alias(1); spin(filter); }", pkg="p")
    cpp.analyse([header, source])
    assert source.constructions == [{"class": "p::Alias", "owner_symbol": "main", "line_hint": 1}]


def test_dynamic_parameter_prefixes_are_not_literal_declarations():
    unit = cpp.parse_text(
        """
void load() {
  declare_parameter(prefix + std::string("_enabled"), true);
}
""",
        pkg="p",
        relpath="filter.cpp",
    )
    report = cpp.file_report(unit)
    assert not report["parameters"]
    assert any(row["want"] == model.Want.PARAM_NAME.value for row in report["unresolved"])


@pytest.mark.parametrize(
    "linked,construction,definitions,binds",
    [
        (False, "p::Filter<int>", 1, False),
        (True, "Filter<int>", 1, False),
        (True, "T", 1, False),
        (True, "rclcpp::Node", 1, False),
        (True, "p::Filter<int>", 1, True),
        (True, "p::Filter<int>", 2, False),
        (True, "p::Filter<int>", 0, False),
        (True, "p::Alias", 1, True),
        (True, "p::Alias", 2, False),
        (True, "p::Alias", 0, False),
    ],
)
def test_library_binding_obeys_definitions_and_linkage(linked, construction, definitions, binds):
    from rosio import ids

    builder = model.AtlasBuilder()
    main_file = ids.build_file("p", "main.cpp")
    method_file = ids.build_file("p", "filter.cpp")
    exe = ids.build_exe("p", "filter")
    run = builder.add_run("declared", input_digest="build")
    builder.fact(
        exe,
        "declared",
        run=run,
        values={
            "sources": [main_file],
            "linked_targets": ["library"] if linked else [],
        },
    )
    builder.fact(
        ids.build_file("p", "CMakeLists.txt"),
        "declared",
        run=run,
        values={
            "library_targets": [{"target": "library", "sources": [method_file]}],
        },
    )
    found = [
        join.Carrier(
            owner=ids.build_sym("p", "main.cpp", "main"),
            file=main_file,
            symbol=ids.build_sym("p", "main.cpp", "main"),
            constructions=({"class": construction},),
        ),
        join.Carrier(
            owner="header",
            file=ids.build_file("p", "nodes.hpp"),
            symbol="",
            language="cpp",
            classes=({"name": "p::Filter"}, {"name": "other::Filter"}),
            class_templates=(
                {"name": "p::Filter", "definitions": int(definitions > 0), "specialized": False},
            ),
            type_aliases=({"name": "p::Alias", "target": "Filter<int>", "namespace": "p"},),
        ),
        join.Carrier(
            owner="method",
            file=method_file,
            language="cpp",
            symbol=ids.build_sym("p", "filter.cpp", "p::Filter<T>::initialize"),
        ),
    ]
    if definitions == 2:
        found.append(
            join.Carrier(
                owner="other_header",
                file=ids.build_file("p", "other.hpp"),
                symbol="",
                language="cpp",
                classes=({"name": "p::Filter"},),
                class_templates=({"name": "p::Filter", "definitions": 1, "specialized": False},),
            )
        )
    builder.node(main_file)
    builder.node(method_file)
    by_exe = {}
    join._constructed_carriers(builder, found, by_exe, set())
    assert bool(by_exe) is binds
    if linked and not binds and construction not in {"T", "rclcpp::Node"}:
        assert len(builder.atlas.unresolved) == 1
        assert "no unique package definition" in builder.atlas.unresolved[0].reason
    before = builder.atlas.canonical_bytes()
    join._constructed_carriers(builder, found, by_exe, set())
    assert builder.atlas.canonical_bytes() == before


@pytest.mark.ros
@pytest.mark.parametrize(
    "defect", ["none", "duplicate_file", "duplicate_branch", "specialized", "missing", "alias"]
)
def test_linked_classes_bind_only_the_constructed_methods(tmp_path, monkeypatch, defect):
    original = join.join

    def twice(builder):
        result = original(builder)
        before = builder.atlas.canonical_bytes()
        original(builder)
        assert builder.atlas.canonical_bytes() == before
        return result

    monkeypatch.setattr(join, "join", twice)
    fixture = Path(__file__).parent / "fixtures" / "ws_linked" / "src"
    copied = tmp_path / "src"
    shutil.copytree(fixture, copied)
    fixture = copied
    header = fixture / "linked_nodes/include/nodes.hpp"
    source = fixture / "linked_nodes/src/filter.cpp"
    if defect == "duplicate_file":
        (header.parent / "other.hpp").write_text(
            "namespace linked_nodes { template<class T> class Filter {}; }"
        )
    elif defect == "duplicate_branch":
        header.write_text(
            header.read_text() + "\n#ifdef OTHER\nnamespace linked_nodes { "
            "template<class T> class Filter {}; }\n#endif\n"
        )
    elif defect == "specialized":
        source.write_text(
            source.read_text() + "\nnamespace linked_nodes { "
            "template<> void Filter<float>::initialize() { "
            'create_publisher<std_msgs::msg::String>("special", 10); } }'
        )
    elif defect == "missing":
        text = header.read_text()
        start = text.index("template<typename T>")
        end = text.index("class Navigation")
        header.write_text(text[:start] + text[end:])
    elif defect == "alias":
        header.write_text(
            header.read_text() + "\nnamespace linked_nodes { "
            "using SelectedFilter = Filter<float>; }"
        )
    roots = (RootSpec(kind="src", path=str(fixture)),)
    cfg = Config(
        roots=roots,
        profiles=(
            ProfileSpec(
                slug="linked",
                entrypoints=(
                    Entrypoint(
                        verb="launch",
                        pkg="linked_nodes",
                        target="nodes.launch.py",
                    ),
                ),
            ),
        ),
        out_dir=str(tmp_path / "out"),
        deterministic=True,
    )
    first = extract.run(cfg, Workspace.discover(roots))
    second = extract.run(cfg, Workspace.discover(roots))
    assert schema.validate(first.to_json()) == []
    assert first.canonical_bytes() == second.canonical_bytes()
    for node, topic, source in [
        ("selected_filter", "filtered_local", "filter"),
        ("selected_navigation", "position_local", "navigation"),
    ]:
        edges = [
            edge
            for edge in first.out_edges(f"node:/{node}")
            if edge.kind == model.EdgeKind.PUBLISHES.value
        ]
        if node == "selected_filter" and defect != "none":
            assert not edges
            assert any(
                row.expr == "linked_nodes::SelectedFilter" and "definition" in row.reason
                for row in first.unresolved
            )
            continue
        assert [edge.dst for edge in edges] == [f"topic:/{topic}"]
        fact = edges[0].fact("coded", "profile:linked")
        assert {row.file for row in fact.evidence} == {
            f"file:linked_nodes|src/{source}.cpp",
            f"file:linked_nodes|src/{source}_main.cpp",
        }
        assert any(row.locator == "main" and row.line_hint for row in fact.evidence)
    if defect == "none":
        assert not any(row.rule == "remap.unresolved" for row in first.findings)
        assert first.entities["param:/selected_filter|frequency"].fact("coded", "profile:linked")
