import shutil
from pathlib import Path

import pytest

from rosio import extract, model, schema
from rosio.collect import cpp_code, plugins, py_code
from rosio.config import Config, Entrypoint, ProfileSpec, RootSpec
from rosio.workspace import Workspace

FIXTURE = Path(__file__).parent / "fixtures" / "ws_names"


@pytest.mark.ros
@pytest.mark.cpp
@pytest.mark.parametrize("absolute", [False, True])
def test_one_source_with_different_target_include_paths_stays_unknown(tmp_path, absolute):
    fixture = tmp_path / "ws"
    shutil.copytree(FIXTURE, fixture)
    cmake = fixture / "src/argument_nodes/CMakeLists.txt"
    source = (
        str(fixture / "src/argument_nodes/src/header_main.cpp")
        if absolute
        else "src/header_main.cpp"
    )
    with cmake.open("a") as stream:
        stream.write(
            f"\nadd_executable(other {source})\n"
            "target_include_directories(other BEFORE PRIVATE other_headers)\n"
        )
    roots = (RootSpec(kind="src", path=str(fixture / "src")),)
    cfg = Config(
        roots=roots,
        profiles=(
            ProfileSpec(
                slug="header",
                entrypoints=(
                    Entrypoint(verb="run", pkg="argument_nodes", target="header_default"),
                ),
            ),
        ),
        deterministic=True,
    )
    atlas = extract.run(cfg, Workspace.discover(roots))
    assert "node:/header_default" not in atlas.entities
    assert any(
        row.want == "node_name" and row.reason == "argv_value" and "listener.hpp" in row.owner
        for row in atlas.unresolved
    )


@pytest.mark.cpp
@pytest.mark.parametrize(
    "argument,expected", [("", "default_name"), ('"override"', "override"), ("runtime()", None)]
)
def test_cpp_constructor_literal_default(argument, expected):
    unit = cpp_code.parse_text(
        f"""
class Listener : public rclcpp::Node {{
public:
  explicit Listener(const std::string & name = "default_name") : Node(name) {{}}
}};
int main() {{ auto node = std::make_shared<Listener>({argument}); }}
""",
        pkg="p",
        relpath="p.cpp",
    )
    assert cpp_code.file_report(unit)["nodes"][0]["name"] == expected


@pytest.mark.cpp
def test_cpp_constructor_argument_from_const_string():
    unit = cpp_code.parse_text(
        """
class Listener : public rclcpp::Node {
public:
  explicit Listener(const std::string & name) : Node(name) {}
};
int main() {
  const std::string name = "logger_service";
  auto node = std::make_shared<Listener>(name);
}
""",
        pkg="p",
        relpath="p.cpp",
    )
    assert cpp_code.file_report(unit)["nodes"][0]["name"] == "logger_service"


@pytest.mark.cpp
def test_cpp_constructor_does_not_read_a_shadowed_constant():
    unit = cpp_code.parse_text(
        """
class Listener : public rclcpp::Node {
public:
  Listener(const std::string & name) : Node(name) {}
};
int main() {
  const std::string name = "outer";
  if (std::string name = runtime(); !name.empty())
    std::make_shared<Listener>(name);
}
""",
        pkg="p",
        relpath="p.cpp",
    )
    assert cpp_code.file_report(unit)["nodes"][0]["name"] is None


def test_cmake_include_directories_keep_order_and_target_scope():
    scan = plugins.scan_cmake("""
include_directories(late)
add_executable(first first.cpp)
add_executable(second second.cpp)
include_directories(BEFORE early)
target_include_directories(first BEFORE PRIVATE first_headers)
target_include_directories(second PRIVATE second_headers INTERFACE consumer_headers)
""")
    assert scan.include_dirs == ("early", "late")
    assert scan.targets["first"].include_dirs == ("first_headers", "early", "late")
    assert scan.targets["second"].include_dirs == ("early", "late", "second_headers")


@pytest.mark.parametrize(
    "argument,expected",
    [("", "default_name"), ('"override"', "override"), ("runtime()", None), ("**options", None)],
)
def test_python_constructor_literal_default(argument, expected):
    source = (FIXTURE / "src/argument_nodes/argument_nodes/default_node.py").read_text()
    source = source.replace("DefaultNode()", f"DefaultNode({argument})")
    report = py_code.analyse_source(source, path="p.py", package="p", package_relative_path="p.py")
    assert report["nodes"][0]["name"] == expected


@pytest.mark.cpp
@pytest.mark.parametrize(
    "duplicate,included,expected",
    [(False, True, "header_default"), (True, True, None), (False, False, None)],
)
def test_header_constructor_binding_requires_a_unique_included_class(duplicate, included, expected):
    base = FIXTURE / "src/argument_nodes"
    header = cpp_code.parse_file(
        str(base / "include/argument_nodes/listener.hpp"),
        pkg="p",
        relpath="include/argument_nodes/listener.hpp",
    )
    main = cpp_code.parse_text(
        ('#include "argument_nodes/listener.hpp"\n' if included else "")
        + "int main() { auto node = std::make_shared<HeaderListener>(); }",
        pkg="p",
        relpath="src/header_main.cpp",
    )
    units = [header, main]
    if duplicate:
        units.append(
            cpp_code.parse_text(
                "class HeaderListener {};", pkg="p", relpath="include/duplicate.hpp"
            )
        )
    cpp_code.analyse(units)
    rows = cpp_code.file_report(main)["nodes"]
    assert (rows[0]["name"] if rows else None) == expected


@pytest.mark.cpp
@pytest.mark.parametrize(
    "argument,default,expected",
    [
        ("", '"header_default"', "header_default"),
        ('"override"', '"header_default"', "override"),
        ("", "runtime()", None),
    ],
)
def test_header_constructor_defaults_are_bound_per_main(argument, default, expected):
    text = (FIXTURE / "src/argument_nodes/include/argument_nodes/listener.hpp").read_text()
    header = cpp_code.parse_text(
        text.replace('"header_default"', default), pkg="p", relpath="headers/listener.hpp"
    )
    main = cpp_code.parse_text(
        '#include "listener.hpp"\n'
        f"int main() {{ auto node = std::make_shared<HeaderListener>({argument}); }}",
        pkg="p",
        relpath="main.cpp",
    )
    other = cpp_code.parse_text(
        '#include "listener.hpp"\n'
        'int main() { auto node = std::make_shared<HeaderListener>("other"); }',
        pkg="p",
        relpath="other.cpp",
    )
    scan = plugins.scan_cmake(
        'include_directories("$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/headers>")'
    )
    cpp_code.analyse([header, main, other], include_dirs={"p": list(scan.include_dirs)})
    report = cpp_code.file_report(main)
    assert report["nodes"][0]["name"] == expected
    assert cpp_code.file_report(other)["nodes"][0]["name"] == "other"
    assert bool(report["unresolved"]) == (expected is None)
    assert main.constructions[0]["definition_file"] == "file:p|headers/listener.hpp"


@pytest.mark.cpp
@pytest.mark.parametrize(
    "prefix,package",
    [('/*\n#include "listener.hpp"\n*/', "p"), ('#include "listener.hpp"', "other")],
)
def test_header_binding_does_not_guess_from_comments_or_another_package(prefix, package):
    header = cpp_code.parse_text(
        'class Listener : public rclcpp::Node { public: Listener() : Node("wrong") {} };',
        pkg=package,
        relpath="include/listener.hpp",
    )
    main = cpp_code.parse_text(
        prefix + "\nint main() { auto node = std::make_shared<Listener>(); }",
        pkg="p",
        relpath="main.cpp",
    )
    cpp_code.analyse([header, main])
    assert not cpp_code.file_report(main)["nodes"]
    assert "definition_file" not in main.constructions[0]


@pytest.mark.cpp
def test_header_binding_keeps_multiple_constructions_separate():
    text = (FIXTURE / "src/argument_nodes/include/argument_nodes/listener.hpp").read_text()
    header = cpp_code.parse_text(text, pkg="p", relpath="include/listener.hpp")
    main = cpp_code.parse_text(
        '#include "listener.hpp"\nint main() { auto a = std::make_shared<HeaderListener>();'
        ' auto b = std::make_shared<HeaderListener>("other"); }',
        pkg="p",
        relpath="main.cpp",
    )
    cpp_code.analyse([header, main])
    report = cpp_code.file_report(main)
    assert {row["name"] for row in report["nodes"]} == {"header_default", "other"}
    assert len({row["construction"] for row in report["nodes"]}) == 2
    assert not report["unresolved"]


@pytest.mark.parametrize("change", ["nonliteral", "duplicate"])
def test_python_constructor_default_near_misses(change):
    source = (FIXTURE / "src/argument_nodes/argument_nodes/default_node.py").read_text()
    if change == "nonliteral":
        source = source.replace('node_name="default_name"', "node_name=runtime()")
    else:
        source += "\nclass DefaultNode(Node):\n    pass\n"
    report = py_code.analyse_source(source, path="p.py", package="p", package_relative_path="p.py")
    assert report["nodes"][0]["name"] is None
    assert report["unresolved"][0]["want"] == "node_name"


@pytest.mark.cpp
@pytest.mark.parametrize(
    "construction",
    [
        'std::make_shared<Listener>(1, "listener")',
        'std::make_unique<Listener>(1, "listener")',
        'Listener(1, "listener")',
    ],
)
@pytest.mark.parametrize("base", ["rclcpp::Node", "rclcpp_lifecycle::LifecycleNode"])
def test_cpp_constructor_argument_is_bound_by_position(construction, base):
    source = f"""
class Listener : public {base} {{
public:
  Listener(int count, /* label */ const std::string & name) : {base}(name) {{}}
}};
int main() {{ auto node = {construction}; }}
"""
    unit = cpp_code.parse_text(source, pkg="p", relpath="src/listener.cpp")
    report = cpp_code.file_report(unit)
    row = report["nodes"][0]
    assert (row["name"], row["name_source"], row["symbol"]) == ("listener", "literal", "main")
    assert row["line_hint"] == 6
    assert not report["unresolved"]


@pytest.mark.cpp
@pytest.mark.parametrize(
    "calls",
    [
        'auto a = std::make_shared<Listener>("one"); auto b = Listener("two");',
        'Listener a("one"), b("two");',
        'auto a = Listener("one"); auto b = Listener("one");',
        'auto a = Listener("one"); auto b = Listener(argv[1]);',
    ],
)
def test_cpp_multiple_constructions_are_refused(calls):
    source = f"""
class Listener : public rclcpp::Node {{
public:
  Listener(const std::string & name) : Node(name) {{}}
}};
int main(int argc, char ** argv) {{ {calls} }}
"""
    unit = cpp_code.parse_text(source, pkg="p", relpath="p.cpp")
    report = cpp_code.file_report(unit)
    assert report["nodes"][0]["name"] is None
    assert report["unresolved"][0]["reason"] == "ambiguous_construction"


def test_component_registration_inside_a_cmake_function_keeps_its_class_and_sources():
    scan = plugins.scan_cmake("""
function(component plugin executable)
  cmake_parse_arguments(ARG "" "" "FILES;DEPENDENCIES" ${ARGN})
  set(library ${executable}_library)
  add_library(${library} SHARED ${ARG_FILES})
  target_link_libraries(${library} PRIVATE ${ARG_DEPENDENCIES})
  rclcpp_components_register_node(${library} PLUGIN ${plugin} EXECUTABLE ${executable})
endfunction()
component("p::Talker" talker FILES src/talker.cpp DEPENDENCIES rclcpp::rclcpp)
component("p::Listener" listener FILES src/listener.cpp DEPENDENCIES rclcpp::rclcpp)
""")
    assert [(row.class_name, row.executable, row.target) for row in scan.components] == [
        ("p::Talker", "talker", "talker_library"),
        ("p::Listener", "listener", "listener_library"),
    ]
    assert scan.targets["talker_library"].sources == ("src/talker.cpp",)
    assert scan.targets["listener_library"].sources == ("src/listener.cpp",)
    assert "rclcpp::rclcpp" in scan.targets["talker_library"].linked_targets


@pytest.mark.parametrize("construction", ['Talker(1, "talker")', 'Talker(1, name="talker")'])
def test_python_constructor_argument_keeps_the_callers_evidence(construction):
    source = f"""
from rclpy.lifecycle import LifecycleNode
class Talker(LifecycleNode):
    def __init__(self, count, name):
        super().__init__(name)
def main():
    {construction}
"""
    report = py_code.analyse_source(source, path="p.py", package="p", package_relative_path="p.py")
    row = report["nodes"][0]
    assert (row["name"], row["name_source"], row["symbol"]) == ("talker", "literal", "p.main")
    assert row["line_hint"] == 7
    assert not report["unresolved"]


def test_python_a_reassigned_argument_is_not_a_forwarded_literal():
    source = """
from rclpy.node import Node
class Talker(Node):
    def __init__(self, name):
        name = changed_at_runtime()
        super().__init__(name)
def main():
    Talker("talker")
"""
    report = py_code.analyse_source(source, path="p.py", package="p", package_relative_path="p.py")
    assert report["nodes"][0]["name"] is None
    assert report["unresolved"][0]["reason"] == "argv_value"


@pytest.mark.parametrize(
    "calls", ['Talker("one")\n    Talker("two")', 'Talker("one")\n    Talker("one")']
)
def test_python_multiple_constructions_are_refused(calls):
    source = f"""
from rclpy.node import Node
class Talker(Node):
    def __init__(self, name):
        super().__init__(name)
def main():
    {calls}
"""
    report = py_code.analyse_source(source, path="p.py", package="p", package_relative_path="p.py")
    assert report["nodes"][0]["name"] is None
    assert report["unresolved"][0]["reason"] == "ambiguous_construction"


@pytest.mark.ros
@pytest.mark.cpp
def test_constructor_names_extract_to_stable_valid_identities(tmp_path):
    roots = (RootSpec(kind="src", path=str(FIXTURE / "src")),)
    ws = Workspace.discover(roots)
    cfg = Config(
        roots=roots,
        profiles=(
            ProfileSpec(
                slug="names",
                entrypoints=(
                    Entrypoint(verb="launch", pkg="argument_nodes", target="names.launch.py"),
                ),
            ),
        ),
        out_dir=str(tmp_path / "out"),
        deterministic=True,
    )
    first = extract.run(cfg, ws)
    second = extract.run(cfg, ws)
    assert schema.validate(first.to_json()) == []
    assert first.canonical_bytes() == second.canonical_bytes()
    for name in (
        "argument_listener",
        "argument_talker",
        "component_talker",
        "header_default",
        "default_name",
    ):
        node = first.entities[f"node:/{name}"]
        assert node.name_source == "literal"
        assert node.aka
        fact = node.fact(model.Layer.CODED.value, "profile:names")
        assert fact.evidence
        if name not in {"component_talker", "header_default"}:
            assert all(source.locator.endswith("main") for source in fact.evidence)
    assert any(
        edge.dst == "topic:/header_output" for edge in first.out_edges("node:/header_default")
    )
    assert any(key.startswith("node:~argument_nodes|ambiguous|") for key in first.entities)
    assert any(row.reason == "multiple_constructed_nodes" for row in first.unresolved)
    assert not any(
        row.want == "node_name"
        and row.owner
        in {"node:/argument_listener", "node:/argument_talker", "node:/component_talker"}
        for row in first.unresolved
    )


@pytest.mark.ros
@pytest.mark.cpp
def test_one_unnamed_node_launched_by_two_profiles_resolves_once(tmp_path):
    """Two profiles running one unnamed executable share its synthetic id, so the second
    profile finds the name already resolved rather than taken by another node."""
    roots = (RootSpec(kind="src", path=str(FIXTURE / "src")),)
    entry = Entrypoint(verb="launch", pkg="argument_nodes", target="names.launch.py")
    cfg = Config(
        roots=roots,
        profiles=(
            ProfileSpec(slug="first", entrypoints=(entry,)),
            ProfileSpec(slug="second", entrypoints=(entry,)),
        ),
        out_dir=str(tmp_path / "out"),
        deterministic=True,
    )
    atlas = extract.run(cfg, Workspace.discover(roots))
    assert schema.validate(atlas.to_json()) == []
    node = atlas.entities["node:/component_talker"]
    assert node.profiles == ["profile:first", "profile:second"]
    assert not [
        row
        for row in atlas.findings
        if row.rule == "node.ambiguous_identity" and row.subject == node.id
    ]
    assert len([row for row in atlas.findings if row.rule == "node.ambiguous_identity"]) == 1
