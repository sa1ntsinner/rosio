"""Release defaults retain only measured changes on known node classes."""

import pytest

from corpus.defaults.collect import collect_release, differences

pytestmark = pytest.mark.cpp
pytest.importorskip("tree_sitter")
pytest.importorskip("tree_sitter_cpp")


def test_defaults_corpus_excludes_tests_plugins_and_ambiguous_defaults(tmp_path):
    package = tmp_path / "nav2_demo"
    package.mkdir()
    (package / "package.xml").write_text("<package><name>nav2_demo</name></package>")
    code = """
    Demo::Demo() : nav2::LifecycleNode("demo", "", options) {
      declare_or_get_parameter("frame", "map");
      declare_or_get_parameter("gain", 2);
      declare_or_get_parameter("gain", 3);
      declare_or_get_parameter("plugin.key", 7);
      declare_or_get_parameter("dynamic", runtime_value);
    }
    """
    (package / "demo.cpp").write_text(code)
    (package / "demo_test.cpp").write_text(code.replace('"map"', '"bad"'))
    (package / "helper.cpp").write_text(code.replace('"map"', '"bad"'))
    (package / "CMakeLists.txt").write_text(
        "if(BUILD_TESTING)\nadd_library(helper SHARED helper.cpp)\nendif()\n"
    )
    current = collect_release(tmp_path, "jazzy")
    assert ("nav2_demo", "Demo", "frame") in current
    assert ("nav2_demo", "Demo", "gain") not in current
    assert ("nav2_demo", "Demo", "plugin.key") not in current
    key = ("nav2_demo", "Demo", "frame")
    old = {k: dict(v) for k, v in current.items()}
    old[key] = dict(old[key], default="odom")
    result = differences({"humble": old, "jazzy": current})
    assert [(r["node"], r["name"]) for r in result] == [("demo", "frame")]
    assert result[0]["releases"]["jazzy"]["source"] == {
        "branch": "jazzy",
        "path": "nav2_demo/demo.cpp",
        "line": 3,
    }


def test_a_declaration_moved_to_an_unbound_handler_is_not_removed(tmp_path):
    package = tmp_path / "demo"
    package.mkdir()
    (package / "package.xml").write_text("<package><name>demo</name></package>")
    path = package / "demo.cpp"
    path.write_text(
        'Demo::Demo() : nav2::LifecycleNode("demo", "", options) { declare_parameter("gain", 1); }'
    )
    old = collect_release(tmp_path, "humble")
    path.write_text(
        'Demo::Demo() : nav2::LifecycleNode("demo", "", options) {} '
        'void Handler::configure() { declare_or_get_parameter("gain", 1); }'
    )
    new = collect_release(tmp_path, "jazzy")
    assert differences({"humble": old, "jazzy": new}) == []


def test_component_executable_can_supply_a_verified_node_name(tmp_path):
    package = tmp_path / "nav2_map_server"
    package.mkdir()
    (package / "package.xml").write_text("<package><name>nav2_map_server</name></package>")
    (package / "CMakeLists.txt").write_text(
        'rclcpp_components_register_node(core PLUGIN "nav2_map_server::MapSaver" '
        "EXECUTABLE map_saver_server)\n"
    )
    (package / "node.cpp").write_text(
        "namespace nav2_map_server { MapSaver::MapSaver() : "
        'nav2::LifecycleNode(runtime_name, "", options) { declare_parameter("gain", 1); }}'
    )
    rows = collect_release(tmp_path, "humble")
    assert rows[("nav2_map_server", "nav2_map_server::MapSaver", "gain")]["node"] == "map_saver"
