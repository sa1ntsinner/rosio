"""pluginlib manifests, CMake and setup.py exports, and the executable to source mapping."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest

from rosio import ids, model, schema
from rosio.collect import CollectContext
from rosio.collect import plugins as plugins_collect
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
def expected(fixtures_dir: Path):
    def _load(name: str) -> dict[str, Any]:
        return json.loads((fixtures_dir / "expected" / "static" / name).read_text(encoding="utf-8"))

    return _load


@pytest.fixture(scope="module")
def builds(static_root: Path, ws: Workspace) -> list[plugins_collect.PackageBuild]:
    libraries = plugins_collect.library_targets(ws)
    return [
        plugins_collect.read_package(ws, pkg, libraries=libraries) for pkg in sorted(ws.packages)
    ]


def _rel(static_root: Path):
    return lambda path: str(Path(path).relative_to(static_root))


def _collect(static_root: Path, ws: Workspace) -> model.Atlas:
    roots = (RootSpec("src", str(static_root / "src")),)
    builder = model.AtlasBuilder()
    ctx = CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder)
    plugins_collect.PluginsCollector().collect(ctx)
    return builder.build()


# ---------------------------------------------------------------------------
# CMake and setup.py reading


def test_commands_survive_wrapping_comments_and_variables() -> None:
    text = (
        "project(demo)\n"
        "set(deps rclcpp\n  std_msgs)\n"
        "# add_executable(commented_out src/x.cpp)\n"
        "add_executable(demo_node\n  src/a.cpp\n  src/b.cpp)\n"
        "ament_target_dependencies(demo_node ${deps})\n"
        "install(TARGETS demo_node RUNTIME DESTINATION lib/${PROJECT_NAME})\n"
    )
    scan = plugins_collect.scan_cmake(text, pkg="demo")
    target = scan.targets["demo_node"]
    assert scan.project == "demo"
    assert "commented_out" not in scan.targets
    assert target.sources == ("src/a.cpp", "src/b.cpp")
    assert target.dependencies == ("rclcpp", "std_msgs")
    assert target.installed is True
    assert target.line_hint == 5


def test_an_ipp_file_is_a_source_of_its_target() -> None:
    scan = plugins_collect.scan_cmake(
        "add_library(demo src/a.cpp include/demo/a_impl.ipp include/demo/b.hxx)\n", pkg="demo"
    )
    assert scan.targets["demo"].sources == (
        "src/a.cpp",
        "include/demo/a_impl.ipp",
        "include/demo/b.hxx",
    )


def test_a_component_executable_is_not_an_add_executable_target() -> None:
    text = (
        "add_library(comp SHARED src/comp.cpp)\n"
        'rclcpp_components_register_node(comp\n  PLUGIN "demo::Comp"\n  EXECUTABLE comp_node)\n'
    )
    scan = plugins_collect.scan_cmake(text, pkg="demo")
    assert "comp_node" not in scan.targets
    assert scan.components[0].class_name == "demo::Comp"
    assert scan.components[0].executable == "comp_node"
    assert scan.components[0].target == "comp"


def test_console_script_names_and_modules_are_read_separately() -> None:
    text = (
        "setup(\n  entry_points={\n    'console_scripts': [\n"
        "      'health_monitor = pkg.watchdog:main',\n"
        "      'report = pkg.report_node:main',\n    ],\n  },\n)\n"
    )
    scripts = plugins_collect.scan_console_scripts(text)
    assert [(item.name, item.module, item.entry) for item in scripts] == [
        ("health_monitor", "pkg.watchdog", "main"),
        ("report", "pkg.report_node", "main"),
    ]


def test_export_macros_are_found_even_when_wrapped(static_root: Path) -> None:
    path = static_root / "src/fx_plugins/src/static_layer.cpp"
    found = plugins_collect.scan_export_macros(path.read_text(encoding="utf-8"), path=str(path))
    assert len(found) == 1
    assert found[0].type == "fx_plugins::StaticLayer"
    assert found[0].base_class == "fx_core::Layer"
    assert found[0].wrapped is True


# ---------------------------------------------------------------------------
# manifests


def test_the_plugin_table_matches_the_expected_file(static_root: Path, builds, expected) -> None:
    want = expected("plugins.json")
    build = next(item for item in builds if item.pkg == want["package"])
    got = plugins_collect.plugin_view(build, _rel(static_root))
    for key in ("manifests", "package_xml_exports", "cmake_exports", "source_export_macros"):
        assert got[key] == want[key], key


def test_the_expected_problems_are_reported(static_root: Path, builds, expected) -> None:
    want = expected("plugins.json")
    build = next(item for item in builds if item.pkg == want["package"])
    got = plugins_collect.plugin_view(build, _rel(static_root))["expected_problems"]
    assert sorted((row["kind"], row["subject"]) for row in got) == sorted(
        (row["kind"], row["subject"]) for row in want["expected_problems"]
    )


def test_a_lookup_name_falls_back_to_the_type() -> None:
    text = (
        '<library path="fx_plugin_layers">'
        '<class type="a::B" base_class_type="c::D"/>'
        '<class name="pkg/Named" type="a::E" base_class_type="c::D"/>'
        "</library>"
    )
    library, classes = plugins_collect.parse_manifest_text(text)
    assert library == "fx_plugin_layers"
    lookups = {item.lookup_name: item.name_attribute_present for item in classes}
    assert lookups == {"a::B": False, "pkg/Named": True}


def test_both_export_mechanisms_are_unioned(builds) -> None:
    build = next(item for item in builds if item.pkg == "fx_plugins")
    by_name = {Path(item.path).name: item for item in build.manifests}
    assert by_name["fx_layers_plugin.xml"].exported_by == ("cmake", "package.xml")
    assert by_name["fx_tools_plugin.xml"].exported_by == ("cmake",)
    assert by_name["fx_absent_plugin.xml"].exported_by == ("package.xml",)


# ---------------------------------------------------------------------------
# executables


def test_the_executable_table_matches_the_expected_file(
    static_root: Path, builds, expected
) -> None:
    want = expected("executables.json")
    got = plugins_collect.executable_view(builds, _rel(static_root))
    for key in ("executables", "components", "libraries"):
        assert sorted(got[key]) == sorted(want[key]), key
        for subject, row in want[key].items():
            for field in sorted(row):
                assert got[key][subject][field] == row[field], f"{key} {subject} {field}"
    assert got["python_modules_without_executable"] == want["python_modules_without_executable"]


def test_a_component_executable_has_no_main_and_is_still_named(builds) -> None:
    build = next(item for item in builds if item.pkg == "fx_cpp_nodes")
    gamma = next(item for item in build.executables if item.name == "fx_gamma_node")
    assert gamma.mechanism == plugins_collect.MECHANISM_COMPONENT_NODE
    assert gamma.has_main is False
    assert gamma.symbol == "fx_cpp_nodes::GammaNode"


def test_a_python_executable_name_need_not_match_its_module(builds) -> None:
    build = next(item for item in builds if item.pkg == "fx_py_nodes")
    watch = next(item for item in build.executables if item.name == "fx_watch")
    assert watch.module == "fx_py_nodes.watchdog"
    assert watch.entry == "main"
    assert watch.has_main is True


# ---------------------------------------------------------------------------
# the collector


def test_the_atlas_validates_and_is_stable(static_root: Path, ws: Workspace) -> None:
    first = _collect(static_root, ws)
    assert schema.validate(first.to_json()) == []
    second = _collect(static_root, Workspace.discover([RootSpec("src", str(static_root / "src"))]))
    assert first.canonical_bytes() == second.canonical_bytes()


def test_plugin_entities_and_provides_edges(static_root: Path, ws: Workspace) -> None:
    atlas = _collect(static_root, ws)
    pkg_id = ids.build_pkg("fx_plugins")
    plugin_ids = {node.id for node in atlas.by_kind("plugin")}
    assert plugin_ids == {
        ids.build_plugin("fx_core::Layer", "fx_plugins/DecayLayer"),
        ids.build_plugin("fx_core::Layer", "fx_plugins::StaticLayer"),
        ids.build_plugin("fx_tools::Tool", "fx_plugins/GoalTool"),
    }
    provided = {
        edge.dst
        for edge in atlas.out_edges(pkg_id)
        if edge.kind == model.EdgeKind.PROVIDES_PLUGIN.value
    }
    assert provided == plugin_ids


def test_a_plugin_carries_its_library_state(static_root: Path, ws: Workspace) -> None:
    atlas = _collect(static_root, ws)
    tool = atlas.entities[ids.build_plugin("fx_tools::Tool", "fx_plugins/GoalTool")]
    values = tool.facts[0].values
    assert values["library"] == "fx_missing_library"
    assert values["library_built_here"] is False
    reasons = {entry.reason for entry in atlas.unresolved}
    assert "plugin_library_not_built" in reasons
    assert "plugin_manifest_missing" in reasons


def test_a_missing_manifest_is_marked_unloadable(static_root: Path, ws: Workspace) -> None:
    atlas = _collect(static_root, ws)
    file_id = ids.build_file("fx_plugins", "fx_absent_plugin.xml")
    entity = atlas.entities[file_id]
    assert entity.attrs["exists"] is False
    assert entity.attrs["loadable"] is False


def test_a_source_defines_the_plugin_class(static_root: Path, ws: Workspace) -> None:
    atlas = _collect(static_root, ws)
    plugin_id = ids.build_plugin("fx_core::Layer", "fx_plugins::StaticLayer")
    sources = {
        edge.src for edge in atlas.in_edges(plugin_id) if edge.kind == model.EdgeKind.DEFINES.value
    }
    assert sources == {ids.build_file("fx_plugins", "src/static_layer.cpp")}


def test_an_executable_is_joinable_to_its_source(static_root: Path, ws: Workspace) -> None:
    atlas = _collect(static_root, ws)
    exe_id = ids.build_exe("fx_py_nodes", "fx_watch")
    symbols = [
        edge.dst for edge in atlas.out_edges(exe_id) if edge.kind == model.EdgeKind.RUNS.value
    ]
    assert symbols == [ids.build_sym("fx_py_nodes", "fx_py_nodes/watchdog.py", "main")]
    defining = {
        edge.src for edge in atlas.in_edges(symbols[0]) if edge.kind == model.EdgeKind.DEFINES.value
    }
    assert defining == {ids.build_file("fx_py_nodes", "fx_py_nodes/watchdog.py")}
    gamma = ids.build_exe("fx_cpp_nodes", "fx_gamma_node")
    assert [
        edge.dst for edge in atlas.out_edges(gamma) if edge.kind == model.EdgeKind.RUNS.value
    ] == [ids.build_sym("fx_cpp_nodes", "src/fx_gamma_node.cpp", "fx_cpp_nodes::GammaNode")]


def _package(root: Path, name: str, files: dict[str, str], export: str = "") -> Path:
    package = root / "src" / name
    package.mkdir(parents=True)
    (package / "package.xml").write_text(
        f"<package format='3'><name>{name}</name><version>0.0.1</version>"
        "<description>d</description><maintainer email='dev@example.invalid'>m</maintainer>"
        f"<license>Apache-2.0</license><export><build_type>ament_cmake</build_type>{export}"
        "</export></package>",
        encoding="utf-8",
    )
    for relative, text in files.items():
        target = package / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(text, encoding="utf-8")
    return package


def test_an_unreadable_manifest_is_refused_and_the_package_survives(tmp_path: Path) -> None:
    package = _package(
        tmp_path,
        "zz_plugins",
        {
            "CMakeLists.txt": "project(zz_plugins)\nadd_library(zz_lib src/x.cpp)\n"
            "pluginlib_export_plugin_description_file(rviz_common torn_plugin.xml)\n",
            "torn_plugin.xml": '<library path="libzz_lib.so"><class type="a::B"\n',
            "src/x.cpp": "int helper() { return 0; }\n",
        },
    )
    roots = (RootSpec("src", str(tmp_path / "src")),)
    ws = Workspace.discover(roots)
    builder = model.AtlasBuilder()
    plugins_collect.PluginsCollector().collect(
        CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder)
    )
    atlas = builder.build()
    file_id = ws.file_id(str(package / "torn_plugin.xml"))
    manifest = atlas.entities[file_id]
    assert manifest.attrs["refused_because"] == "plugin_manifest_unreadable"
    assert manifest.attrs["loadable"] is False
    assert "is not parsable xml" in manifest.attrs["refused_detail"]
    assert [entry.reason for entry in atlas.unresolved] == ["plugin_manifest_unreadable"]
    assert not list(atlas.by_kind("plugin"))
    # The blind spot is on the manifest, so a reader of that one entity meets it.
    entry = atlas.unresolved[0]
    assert entry.owner == file_id
    assert entry.id in manifest.unresolved
    assert atlas.entities[ids.build_pkg("zz_plugins")].unresolved == []
    edge = ids.build_edge("contains", ids.build_pkg("zz_plugins"), file_id)
    assert edge in atlas.edges


def test_a_missing_manifest_names_the_manifest_the_package_holds(tmp_path: Path) -> None:
    """nav2_theta_star_planner/package.xml exports global_planner_plugin.xml, and the file
    the package holds is theta_star_planner.xml."""
    package = _package(
        tmp_path,
        "zz_planner",
        {
            "CMakeLists.txt": "project(zz_planner)\nadd_library(zz_lib src/x.cpp)\n"
            "pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)\n",
            "theta_star_planner.xml": '<library path="zz_lib"><class type="a::B" '
            'base_class_type="nav2_core::GlobalPlanner"/></library>\n',
            "src/x.cpp": "int helper() { return 0; }\n",
        },
    )
    roots = (RootSpec("src", str(tmp_path / "src")),)
    ws = Workspace.discover(roots)
    builder = model.AtlasBuilder()
    plugins_collect.PluginsCollector().collect(
        CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder)
    )
    missing = builder.build().entities[ws.file_id(str(package / "global_planner_plugin.xml"))]
    assert missing.attrs["refused_because"] == "plugin_manifest_missing"
    assert missing.attrs["manifests_held"] == [ws.file_id(str(package / "theta_star_planner.xml"))]


def test_a_class_with_no_library_path_claims_no_build_state(tmp_path: Path) -> None:
    _package(
        tmp_path,
        "zz_nolib",
        {
            "CMakeLists.txt": "project(zz_nolib)\n"
            "pluginlib_export_plugin_description_file(rviz_common bare_plugin.xml)\n",
            "bare_plugin.xml": '<library path=""><class type="a::B" base_class_type="c::D"/>'
            "</library>",
        },
    )
    roots = (RootSpec("src", str(tmp_path / "src")),)
    ws = Workspace.discover(roots)
    builder = model.AtlasBuilder()
    plugins_collect.PluginsCollector().collect(
        CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder)
    )
    atlas = builder.build()
    values = atlas.entities[ids.build_plugin("c::D", "a::B")].facts[0].values
    assert "library_built_here" not in values
    assert "library" not in values


def test_a_console_script_entry_must_be_a_module_level_name(tmp_path: Path) -> None:
    _package(
        tmp_path,
        "zz_py",
        {
            "setup.py": "setup(entry_points={'console_scripts': ['worker = zz_py.worker:main']})\n",
            "zz_py/__init__.py": "",
            "zz_py/worker.py": "class Worker:\n    def main(self):\n        pass\n",
        },
    )
    ws = Workspace.discover([RootSpec("src", str(tmp_path / "src"))])
    build = plugins_collect.read_package(ws, "zz_py", libraries=plugins_collect.library_targets(ws))
    worker = next(item for item in build.executables if item.name == "worker")
    assert worker.has_main is False
    assert worker.symbol_file == ""


def test_a_profile_scoped_run_carries_a_profile_id(static_root: Path, ws: Workspace) -> None:
    roots = (RootSpec("src", str(static_root / "src")),)
    builder = model.AtlasBuilder()
    plugins_collect.PluginsCollector().collect(
        CollectContext(
            ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder, profile="sim_trailer"
        )
    )
    atlas = builder.build()
    fact = atlas.entities[ids.build_exe("fx_py_nodes", "fx_watch")].facts[0]
    assert fact.profile == ids.build_profile("sim_trailer")


def test_every_executable_is_contained_by_its_package(static_root: Path, ws: Workspace) -> None:
    atlas = _collect(static_root, ws)
    for node in atlas.by_kind("exe"):
        package = ids.parse_exe(node.id).package
        owners = {
            edge.src
            for edge in atlas.in_edges(node.id)
            if edge.kind == model.EdgeKind.CONTAINS.value
        }
        assert owners == {ids.build_pkg(package)}


# ---------------------------------------------------------------------------
# a target name this reader cannot expand


def test_a_target_named_by_a_variable_is_recorded_and_not_named() -> None:
    """`add_executable(${example_name} ${example_file})` inside a foreach. There is no
    honest name for the target, and an empty one cannot carry an id."""
    text = (
        "project(zz_examples)\n"
        "add_executable(real_node src/real.cpp)\n"
        "foreach(example_file ${examples})\n"
        "  add_executable(${example_name} ${example_file})\n"
        "  target_link_libraries(${example_name} zz_lib)\n"
        "  install(TARGETS ${example_name} DESTINATION lib/zz_examples)\n"
        "endforeach()\n"
    )
    scan = plugins_collect.scan_cmake(text, pkg="zz_examples")
    assert sorted(scan.targets) == ["real_node"]
    assert "" not in scan.targets
    assert [(row.command, row.expression) for row in scan.unresolved_targets] == [
        ("add_executable", "${example_name}")
    ]
    assert scan.installed == ()


def test_a_foreach_over_literal_names_builds_one_target_each() -> None:
    text = (
        "project(zz)\n"
        "foreach(node base_driver depth_monitor)\n"
        "  add_executable(${node} src/${node}.cpp)\n"
        "  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})\n"
        "endforeach()\n"
    )
    scan = plugins_collect.scan_cmake(text, pkg="zz")
    assert {name: row.sources for name, row in scan.targets.items()} == {
        "base_driver": ("src/base_driver.cpp",),
        "depth_monitor": ("src/depth_monitor.cpp",),
    }
    assert sorted(scan.installed) == ["base_driver", "depth_monitor"]


def test_usable_target_refuses_an_empty_and_an_unexpanded_name() -> None:
    assert plugins_collect.usable_target("real_node") is True
    assert plugins_collect.usable_target("") is False
    assert plugins_collect.usable_target("${example_name}") is False


def _two_packages(tmp_path: Path) -> tuple[Workspace, tuple[RootSpec, ...]]:
    """One package with an unexpandable target, one sorted after it with real content."""
    _package(
        tmp_path,
        "aa_utils",
        {
            "CMakeLists.txt": "project(aa_utils)\n"
            "foreach(example_file ${examples})\n"
            "  add_executable(${example_name} ${example_file})\n"
            "endforeach()\n",
            "src/util.cpp": "int helper() { return 0; }\n",
        },
    )
    _package(
        tmp_path,
        "zz_after",
        {
            "CMakeLists.txt": "project(zz_after)\nadd_executable(zz_node src/zz.cpp)\n"
            "add_library(zz_lib src/zz_layer.cpp)\n"
            "pluginlib_export_plugin_description_file(fx_core zz_plugins.xml)\n",
            "zz_plugins.xml": '<library path="zz_lib">'
            '<class type="zz::Layer" base_class_type="fx_core::Layer"/></library>',
            "src/zz.cpp": "int main() { return 0; }\n",
            "src/zz_layer.cpp": "// layer\n",
        },
    )
    roots = (RootSpec("src", str(tmp_path / "src")),)
    return Workspace.discover(roots), roots


def test_one_unreadable_target_does_not_take_the_rest_of_the_workspace(tmp_path: Path) -> None:
    """The defect this closes dropped 22 packages, 14 executables and 28 plugin classes,
    and said nothing anywhere."""
    ws, roots = _two_packages(tmp_path)
    builder = model.AtlasBuilder()
    plugins_collect.PluginsCollector().collect(
        CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder)
    )
    atlas = builder.build()
    assert ids.build_exe("zz_after", "zz_node") in atlas.entities
    assert ids.build_plugin("fx_core::Layer", "zz::Layer") in atlas.entities
    assert not [node for node in atlas.by_kind("exe") if ids.parse_exe(node.id).name == ""]
    assert schema.validate(atlas.to_json()) == []


def test_the_unreadable_target_is_a_blind_spot_with_its_line(tmp_path: Path) -> None:
    ws, roots = _two_packages(tmp_path)
    builder = model.AtlasBuilder()
    plugins_collect.PluginsCollector().collect(
        CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder)
    )
    atlas = builder.build()
    rows = [entry for entry in atlas.unresolved if entry.reason == "cmake_target_name_unresolved"]
    assert len(rows) == 1
    assert rows[0].owner == ids.build_pkg("aa_utils")
    assert rows[0].expr == "add_executable(${example_name})"
    assert rows[0].evidence[0].file == ids.build_file("aa_utils", "CMakeLists.txt")
    assert rows[0].evidence[0].line_hint == 3
    build = plugins_collect.read_package(ws, "aa_utils")
    assert [row["kind"] for row in plugins_collect.problems(build)] == ["target_name_unresolved"]


def test_a_file_the_reader_cannot_open_is_named_by_its_id(tmp_path: Path) -> None:
    """The refusal is recorded on a run, which is compared data, so it must hold no machine
    path: the same refusal on two boxes is otherwise two different atlases."""
    package = _package(tmp_path, "zz_locked", {"CMakeLists.txt": "project(zz_locked)\n"})
    (package / "CMakeLists.txt").chmod(0o000)
    ws = Workspace.discover([RootSpec("src", str(tmp_path / "src"))])

    with pytest.raises(CollectorError) as caught:
        plugins_collect.read_package(ws, "zz_locked")

    message = str(caught.value)
    assert ids.build_file("zz_locked", "CMakeLists.txt") in message
    assert str(tmp_path) not in message
    assert "Errno" in message


def test_the_refusal_of_one_workspace_is_the_refusal_of_its_copy(tmp_path: Path) -> None:
    """One workspace at two absolute paths refuses the same file with the same words."""
    messages = []
    for name in ("here", "a/much/deeper/place"):
        base = tmp_path / name
        package = _package(base, "zz_locked", {"CMakeLists.txt": "project(zz_locked)\n"})
        (package / "CMakeLists.txt").chmod(0o000)
        ws = Workspace.discover([RootSpec("src", str(base / "src"))])
        with pytest.raises(CollectorError) as caught:
            plugins_collect.read_package(ws, "zz_locked")
        messages.append(str(caught.value))
    assert messages[0] == messages[1]


def _installed(prefix: Path, name: str, manifest: str) -> None:
    """One package as an install tree holds it: no CMakeLists, the manifest under share."""
    share = prefix / "share" / name
    share.mkdir(parents=True)
    (share / "package.xml").write_text(
        f"<package format='3'><name>{name}</name><version>0.0.1</version>"
        "<description>d</description><maintainer email='dev@example.invalid'>m</maintainer>"
        "<license>Apache-2.0</license><export><build_type>ament_cmake</build_type>"
        '<rviz_common plugin="${prefix}/plugins.xml"/></export></package>',
        encoding="utf-8",
    )
    (share / "plugins.xml").write_text(manifest, encoding="utf-8")
    index = prefix / "share" / "ament_index" / "resource_index" / "packages"
    index.mkdir(parents=True, exist_ok=True)
    (index / name).write_text("", encoding="utf-8")


def test_an_installed_plugin_library_is_found_in_the_install_tree(tmp_path: Path) -> None:
    """An install tree has no CMakeLists, so a manifest there is checked against the
    libraries the tree holds, and one it cannot find is unknown rather than missing: a
    python plugin path is no file at all."""
    prefix = tmp_path / "install"
    _installed(
        prefix,
        "zz_cpp",
        '<library path="zz_cpp"><class type="a::B" base_class_type="c::D"/></library>',
    )
    _installed(
        prefix,
        "zz_py",
        '<library path="src"><class name="zz_py/Plugin" type="zz_py.Plugin" '
        'base_class_type="rqt_gui_py::Plugin"/></library>',
    )
    (prefix / "lib").mkdir()
    (prefix / "lib" / "libzz_cpp.so").write_bytes(b"")
    roots = (RootSpec("install", str(prefix)),)
    ws = Workspace.discover(roots)
    builder = model.AtlasBuilder()
    plugins_collect.PluginsCollector().collect(
        CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder)
    )
    atlas = builder.build()
    assert [entry.reason for entry in atlas.unresolved] == []
    built = atlas.entities[ids.build_plugin("c::D", "a::B")].facts[0].values
    assert built["library_built_here"] is True
    python = atlas.entities[ids.build_plugin("rqt_gui_py::Plugin", "zz_py/Plugin")]
    assert "library_built_here" not in python.facts[0].values


def test_an_ament_auto_executable_with_an_output_name_is_the_installed_name() -> None:
    """image_rotate builds image_rotate_bin with ament_auto and installs it as image_rotate,
    and the launch file that runs image_rotate bound to no source at all."""
    text = (
        "project(image_rotate)\n"
        "ament_auto_add_executable(image_rotate_bin src/image_rotate.cpp)\n"
        "set_target_properties(image_rotate_bin PROPERTIES OUTPUT_NAME ${PROJECT_NAME})\n"
    )
    scan = plugins_collect.scan_cmake(text, pkg="image_rotate")
    target = scan.targets["image_rotate_bin"]
    assert target.kind == "executable"
    assert target.sources == ("src/image_rotate.cpp",)
    assert target.output_name == "image_rotate"
    assert target.installed is True
