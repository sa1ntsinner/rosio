"""rosio migrate: what a move between releases asks of a workspace, and the rewrites."""

from __future__ import annotations

import json
import subprocess
from pathlib import Path

import pytest

from rosio import migrate
from rosio.cli import main
from rosio.errors import UsageError
from rosio.migrate import distros
from rosio.migrate.rules import catalog
from rosio.migrate.scan import SourcePackage, blank_comments_and_strings, scan

MANIFEST = """<?xml version="1.0"?>
<package format="3">
  <name>{name}</name>
  <version>0.1.0</version>
  <description>d</description>
  <maintainer email="m@example.com">m</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
{depends}
  <export><build_type>ament_cmake</build_type></export>
</package>
"""


def write(path: Path, text: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        handle.write(text)
    return path


def package(root: Path, name: str, depends: list[str], files: dict[str, str]) -> Path:
    base = root / "src" / name
    tags = "\n".join(f"  <depend>{dep}</depend>" for dep in depends)
    write(base / "package.xml", MANIFEST.format(name=name, depends=tags))
    for relative, text in files.items():
        write(base / relative, text)
    return base


def source(root: Path) -> list[SourcePackage]:
    """The packages under root/src, read the way `migrate.packages` reads a workspace."""
    from rosio.migrate.scan import exports_targets, package_depends

    found = []
    for manifest in sorted((root / "src").glob("*/package.xml")):
        base = manifest.parent
        cmakelists = base / "CMakeLists.txt"
        found.append(
            SourcePackage(
                name=base.name,
                path=base,
                files=tuple(sorted(p for p in base.rglob("*") if p.is_file())),
                depends=package_depends(manifest.read_text(encoding="utf-8")),
                exports_targets=cmakelists.is_file()
                and exports_targets(cmakelists.read_text(encoding="utf-8")),
            )
        )
    return found


@pytest.mark.parametrize(
    "old,new,rule",
    [
        ("SequenceStarNode", "SequenceWithMemory", "bt.sequence_star"),
    ],
)
def test_bt_renames_are_dependency_gated(tmp_path, old, new, rule):
    package(tmp_path, "tree", ["behaviortree_cpp_v3"], {"tree.cpp": old + " value;\n"})
    package(tmp_path, "other", [], {"tree.cpp": old + " value;\n"})
    changes = [c for c in scan(source(tmp_path), "humble", "lyrical", tmp_path) if c.rule == rule]
    assert len(changes) == 1 and changes[0].package == "tree"
    migrate.apply(changes, tmp_path, force=True)
    assert (tmp_path / "src/tree/tree.cpp").read_text() == new + " value;\n"
    assert not [
        c for c in scan(source(tmp_path), "humble", "lyrical", tmp_path) if c.rule.startswith("bt.")
    ]


def test_bt_tick_root_requires_a_manual_choice(tmp_path):
    package(tmp_path, "tree", ["behaviortree_cpp_v3"], {"tree.cpp": "tree.tickRoot();\n"})
    package(tmp_path, "other", [], {"tree.cpp": "tree.tickRoot();\n"})
    changes = scan(source(tmp_path), "humble", "lyrical", tmp_path)
    changes = [c for c in changes if c.rule.startswith("bt.")]
    assert [(c.rule, c.package, c.automatic) for c in changes] == [("bt.tick_root", "tree", False)]
    assert "tickExactlyOnce" in changes[0].say


def test_bt_compatibility_aliases_do_not_report_removals(tmp_path):
    package(
        tmp_path,
        "tree",
        ["behaviortree_cpp_v3"],
        {
            "tree.cpp": (
                "BT::NodeConfiguration config; BT::AsyncActionNode * action; "
                "BT::Optional<int> value;\n"
                "BT::NodeConfig current; BT::ThreadedAction * thread; BT::Expected<int> result;\n"
                '// tree.tickRoot();\nconst char * text = "SequenceStarNode";\n'
            )
        },
    )
    assert not scan(source(tmp_path), "humble", "jazzy", tmp_path)
    assert not [
        c for c in scan(source(tmp_path), "humble", "lyrical", tmp_path) if c.rule.startswith("bt.")
    ]


def test_bt_v3_is_still_available_in_jazzy(tmp_path):
    package(
        tmp_path,
        "tree",
        ["behaviortree_cpp_v3"],
        {
            "tree.cpp": "#include <behaviortree_cpp_v3/bt_factory.h>\n"
            "BT::SequenceStarNode * control;\ntree.tickRoot();\n",
            "tree.xml": '<root main_tree_to_execute="Main"><SequenceStar/></root>',
        },
    )
    assert not scan(source(tmp_path), "humble", "jazzy", tmp_path)


def test_bt_includes_rewrite_only_measured_successors(tmp_path):
    text = (
        '#include "behaviortree_cpp_v3/bt_factory.h"\r\n'
        "#include <behaviortree_cpp_v3/action_node.h>\r\n"
        "#include <behaviortree_cpp_v3/controls/sequence_star_node.h>\r\n"
        "#include <behaviortree_cpp/bt_factory.h>\r\n"
        "// #include <behaviortree_cpp_v3/condition_node.h>\r\n"
    )
    base = package(tmp_path, "tree", ["behaviortree_cpp_v3"], {"tree.hpp": text})
    changes = scan(source(tmp_path), "humble", "lyrical", tmp_path)
    changes = [c for c in changes if c.rule.startswith("include.")]
    assert [(c.line, c.replacement) for c in changes] == [
        (1, "behaviortree_cpp/bt_factory.h"),
        (2, "behaviortree_cpp/action_node.h"),
        (3, None),
    ]
    migrate.apply(changes, tmp_path, force=True)
    assert (base / "tree.hpp").read_bytes() == text.replace(
        "behaviortree_cpp_v3/bt_factory.h", "behaviortree_cpp/bt_factory.h"
    ).replace("behaviortree_cpp_v3/action_node.h", "behaviortree_cpp/action_node.h").encode()


@pytest.mark.parametrize(
    "identifier",
    [
        "LocalParameterization",
        "AutoDiffLocalParameterization",
        "IdentityParameterization",
        "SubsetParameterization",
        "QuaternionParameterization",
        "EigenQuaternionParameterization",
        "HomogeneousVectorParameterization",
        "LineParameterization",
        "ProductParameterization",
        "SetParameterization",
        "GetParameterization",
        "local_parameterization_ownership",
        "ParameterBlockLocalSize",
    ],
)
def test_ceres_api_is_manual_and_dependency_gated(tmp_path, identifier):
    package(tmp_path, "solver", ["libceres-dev"], {"solver.cpp": identifier + " old;\n"})
    package(tmp_path, "other", [], {"solver.cpp": identifier + " old;\n"})
    changes = scan(source(tmp_path), "humble", "jazzy", tmp_path)
    assert [(c.rule, c.package, c.found, c.automatic) for c in changes] == [
        ("ceres.local_parameterization", "solver", identifier, False)
    ]
    write(tmp_path / "src/solver/solver.cpp", "ceres::Manifold * value;\n")
    assert not scan(source(tmp_path), "humble", "jazzy", tmp_path)
    assert not scan(source(tmp_path), "jazzy", "lyrical", tmp_path)


def test_ceres_header_is_measured_and_manual(tmp_path):
    package(
        tmp_path,
        "solver",
        ["libceres-dev"],
        {"solver.cpp": "#include <ceres/local_parameterization.h>\n#include <ceres/manifold.h>\n"},
    )
    changes = scan(source(tmp_path), "humble", "jazzy", tmp_path)
    assert [(c.rule, c.found, c.automatic) for c in changes] == [
        ("include.removed", "ceres/local_parameterization.h", False)
    ]
    assert "Manifold" in changes[0].say and "version_history" in changes[0].source


def test_pcl_version_guarded_includes_are_left_to_the_compiler(tmp_path):
    package(
        tmp_path,
        "cloud",
        ["libpcl-dev"],
        {
            "cloud.hpp": (
                "#if PCL_VERSION_COMPARE(>=, 1, 11, 0)\n"
                "#include <pcl/type_traits.h>\n#else\n#include <pcl/point_traits.h>\n#endif\n"
                "#if OTHER\n#if PCL_VERSION_COMPARE(>=, 1, 11, 0)\n"
                "#include <pcl/memory.h>\n#elif PCL_VERSION_COMPARE(>=, 1, 10, 0)\n"
                "#include <pcl/make_shared.h>\n#endif\n#endif\n"
                "#include <pcl/point_traits.h>\n"
            )
        },
    )
    changes = scan(source(tmp_path), "humble", "lyrical", tmp_path)
    assert [(c.rule, c.line, c.found) for c in changes] == [
        ("include.removed", 13, "pcl/point_traits.h")
    ]


@pytest.mark.parametrize(
    "old,new",
    [
        ("pcl/io/io.h", "pcl/common/io.h"),
        ("pcl/make_shared.h", "pcl/memory.h"),
        ("pcl/point_traits.h", "pcl/type_traits.h"),
    ],
)
def test_pcl_successors_are_cited_and_measured(tmp_path, old, new):
    base = package(tmp_path, "cloud", ["libpcl-dev"], {"cloud.hpp": f"#include <{old}>\n"})
    assert not scan(source(tmp_path), "humble", "jazzy", tmp_path)
    changes = scan(source(tmp_path), "humble", "lyrical", tmp_path)
    assert len(changes) == 1 and changes[0].replacement == new
    assert changes[0].source == "https://github.com/PointCloudLibrary/pcl/issues/6039"
    migrate.apply(changes, tmp_path, force=True)
    assert (base / "cloud.hpp").read_text() == f"#include <{new}>\n"
    assert not scan(source(tmp_path), "humble", "lyrical", tmp_path)


def test_bt_xml_only_reads_marked_behavior_trees(tmp_path):
    tree = (
        '<root main_tree_to_execute="Main"><BehaviorTree ID="Main">\n'
        "  <SequenceStar/>\n</BehaviorTree></root>"
    )
    base = package(
        tmp_path,
        "tree",
        ["behaviortree_cpp_v3"],
        {
            "tree.xml": tree,
            "plugin.xml": '<library main_tree_to_execute="Main"><SequenceStar/></library>',
            "unmarked.xml": "<root><SequenceStar/></root>",
            "comment.xml": '<root BTCPP_format="4"><!-- <SequenceStar/> --></root>',
            "bad.xml": '<root main_tree_to_execute="Main">',
        },
    )
    package(tmp_path, "other", [], {"tree.xml": tree})
    changes = scan(source(tmp_path), "humble", "lyrical", tmp_path)
    changes = [c for c in changes if c.rule.startswith("bt.")]
    assert [(c.rule, c.line, c.replacement) for c in changes] == [
        ("bt.xml_format", 1, None),
        ("bt.xml_sequence_star", 2, "SequenceWithMemory"),
    ]
    migrate.apply(changes, tmp_path, force=True)
    assert (base / "tree.xml").read_text() == tree.replace("SequenceStar", "SequenceWithMemory")


def test_bt_xml_rename_preserves_unicode_attributes_and_closing_tags(tmp_path):
    text = (
        '<root BTCPP_format="4"><BehaviorTree ID="M">\n'
        '<SequenceStar name="é">\n</SequenceStar>\n</BehaviorTree></root>'
    )
    base = package(tmp_path, "tree", ["behaviortree_cpp_v3"], {"tree.xml": text})
    changes = scan(source(tmp_path), "humble", "lyrical", tmp_path)
    changes = [c for c in changes if c.rule.startswith("bt.")]
    assert [(c.line, c.found) for c in changes] == [(2, "SequenceStar"), (3, "SequenceStar")]
    migrate.apply(changes, tmp_path, force=True)
    assert (base / "tree.xml").read_text() == text.replace("SequenceStar", "SequenceWithMemory")


def test_bt_xml_subtree_plus_is_manual(tmp_path):
    package(
        tmp_path,
        "tree",
        ["behaviortree_cpp_v3"],
        {"tree.xml": ('<root BTCPP_format="4"><SubTreePlus ID="M"/><SubTree ID="M"/></root>')},
    )
    changes = scan(source(tmp_path), "humble", "lyrical", tmp_path)
    changes = [c for c in changes if c.rule.startswith("bt.")]
    assert [(c.rule, c.found, c.automatic) for c in changes] == [
        ("bt.xml_subtree_plus", "SubTreePlus", False)
    ]


def test_bt_xml_self_closing_callbacks_do_not_edit_parent_tags_twice(tmp_path):
    text = (
        '<root BTCPP_format="4"><SequenceStar><SequenceStar/></SequenceStar>'
        "<SequenceStarCustom><SequenceStar/></SequenceStarCustom></root>"
    )
    base = package(tmp_path, "tree", ["behaviortree_cpp_v3"], {"tree.xml": text})
    changes = scan(source(tmp_path), "humble", "lyrical", tmp_path)
    changes = [c for c in changes if c.rule.startswith("bt.")]
    assert len(changes) == 4
    migrate.apply(changes, tmp_path, force=True)
    assert (base / "tree.xml").read_text() == (
        '<root BTCPP_format="4"><SequenceWithMemory><SequenceWithMemory/></SequenceWithMemory>'
        "<SequenceStarCustom><SequenceWithMemory/></SequenceStarCustom></root>"
    )


def test_bt_xml_entity_expansion_does_not_rewrite_the_reference(tmp_path):
    text = (
        '<!DOCTYPE root [<!ENTITY branch "<SequenceStar/>">]><root BTCPP_format="4">&branch;</root>'
    )
    package(tmp_path, "tree", ["behaviortree_cpp_v3"], {"tree.xml": text})
    assert not [
        c for c in scan(source(tmp_path), "humble", "lyrical", tmp_path) if c.rule.startswith("bt.")
    ]


def rules_of(changes) -> list[tuple[str, str, int]]:
    return [(c.rule, c.file.split("/", 2)[-1], c.line) for c in changes]


def test_yaml_defaults_are_manual_and_located_at_the_block_or_key(tmp_path, monkeypatch):
    import importlib

    scanner = importlib.import_module("rosio.migrate.scan")

    fixture = Path(__file__).parent / "fixtures/migrate_defaults.yaml"
    path = write(tmp_path / "params.yaml", fixture.read_text())

    def row(node, name, old, new):
        return {
            "package": "nav2_demo",
            "class": "Demo",
            "node": node,
            "name": name,
            "releases": {
                "humble": {"declared": True, "default_state": "known", "default": old},
                "jazzy": {
                    "declared": new is not None,
                    "default_state": "known",
                    "default": new,
                    "source": {"branch": "jazzy", "path": "demo.cpp", "line": 7},
                },
                "lyrical": {
                    "declared": new is not None,
                    "default_state": "known",
                    "default": new,
                    "source": {"branch": "lyrical", "path": "demo.cpp", "line": 7},
                },
            },
        }

    facts = {
        "checked": ["humble", "jazzy", "lyrical"],
        "repository": "https://github.com/ros-navigation/navigation2",
        "parameters": [
            row("behavior_server", "global_frame", "odom", "map"),
            row("behavior_server", "old_parameter", 4, None),
            row(
                "costmap_filter_info_server",
                "filter_info_topic",
                "costmap_filter_info",
                "filter_info",
            ),
            row("costmap_filter_info_server", "mask_topic", "filter_mask", "mask"),
        ],
    }
    monkeypatch.setattr(scanner, "_defaults", lambda: facts)
    source = [SourcePackage("robot", tmp_path, (path,))]
    changes = scan(source, "humble", "lyrical", tmp_path)
    assert [(c.rule, c.line, c.title, c.release) for c in changes] == [
        ("params.default_changed", 6, "costmap_filter_info_server.filter_info_topic", "jazzy"),
        ("params.not_declared", 11, "behavior_server.old_parameter", "jazzy"),
    ]
    assert all(not c.automatic and c.effect == "behaviour" for c in changes)
    assert 'set filter_info_topic: "costmap_filter_info"' in changes[0].say
    assert changes[1].found == "old_parameter"
    assert (
        changes[0].source == "https://github.com/ros-navigation/navigation2/blob/jazzy/demo.cpp#L7"
    )
    assert scan(source, "humble", "lyrical", tmp_path) == changes


def test_a_parameter_jazzy_renamed_names_its_replacement(tmp_path):
    path = write(
        tmp_path / "params.yaml",
        "controller_server:\n  ros__parameters:\n    progress_checker_plugin: progress_checker\n"
        "behavior_server:\n  ros__parameters:\n"
        "    costmap_topic: local_costmap/costmap_raw\n"
        "    footprint_topic: local_costmap/published_footprint\n",
    )
    packages = [SourcePackage("robot", tmp_path, (path,))]

    changes = {c.title: c for c in scan(packages, "humble", "jazzy", tmp_path)}

    checker = changes["controller_server.progress_checker_plugin"]
    assert "progress_checker_plugins" in checker.say and not checker.automatic
    topic = changes["behavior_server.costmap_topic"]
    assert topic.replacement == "local_costmap_topic"
    assert "reads local_costmap_topic instead" in topic.say
    assert changes["behavior_server.footprint_topic"].replacement == "local_footprint_topic"
    migrate.apply(list(changes.values()), tmp_path, force=True)
    text = path.read_text()
    assert "    local_costmap_topic: local_costmap/costmap_raw\n" in text
    assert "progress_checker_plugin: progress_checker" in text
    assert not any(c.automatic for c in scan(packages, "humble", "kilted", tmp_path))


def test_the_costmap_array_parser_names_the_header_that_took_it_over(tmp_path):
    package(
        tmp_path,
        "footprints",
        ["nav2_costmap_2d"],
        {"src/footprint.cpp": "#include <nav2_costmap_2d/array_parser.hpp>\n"},
    )
    (change,) = scan(source(tmp_path), "humble", "jazzy", tmp_path)

    assert not change.automatic
    assert "nav2_util/array_parser.hpp" in change.say and "nav2_util::parseVVF" in change.say


def test_measured_default_change_ignores_invalid_blocks_and_crossed_releases(tmp_path):
    path = write(
        tmp_path / "params.yaml",
        '"$(var namespace)":\n  ros__parameters: {gain: 1}\n'
        'bad_names:\n  ros__parameters: {".gain": 1}\n'
        "behavior_server:\n  ros__parameters: {}\n",
    )
    packages = [SourcePackage("robot", tmp_path, (path,))]
    changes = scan(packages, "humble", "lyrical", tmp_path)
    assert [(c.rule, c.line, c.title, c.release) for c in changes] == [
        ("params.default_changed", 5, "behavior_server.global_frame", "jazzy")
    ]
    assert not scan(packages, "jazzy", "lyrical", tmp_path)
    write(path, "{invalid")
    assert not scan(packages, "humble", "lyrical", tmp_path)


DRIVER_CMAKE = """cmake_minimum_required(VERSION 3.0.2)
project(driver)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gazebo_ros REQUIRED)
set(dependencies rclcpp std_msgs)

add_executable(driver src/driver.cpp)
ament_target_dependencies(driver rclcpp std_msgs)

add_library(core SHARED src/core.cpp)
ament_target_dependencies(core PUBLIC ${dependencies})

add_library(math SHARED src/math.cpp)
ament_target_dependencies(math Eigen3 rclcpp)

# ament_target_dependencies(old rclcpp)
ament_package()
"""

DRIVER_CPP = """#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "rclcpp/qos_event.hpp"
// #include <message_filters/subscriber.h>
/*
#include <rcutils/get_env.h>
*/
#include <rcppmath/clamp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const char * note = "rclcpp::get_typesupport_handle is gone";
  auto handle = rclcpp::get_typesupport_handle(type, "rosidl_typesupport_cpp", *library);
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  return 0;
}
"""


@pytest.fixture
def humble_ws(tmp_path: Path) -> Path:
    package(
        tmp_path,
        "driver",
        ["rclcpp", "std_msgs", "gazebo_ros", "behaviortree_cpp_v3", "hardware_interface"],
        {
            "CMakeLists.txt": DRIVER_CMAKE,
            "src/driver.cpp": DRIVER_CPP,
            "src/hardware.cpp": (
                "std::vector<hardware_interface::StateInterface> Robot::export_state_interfaces()\n"
                "{\n  return {};\n}\n"
            ),
            "scripts/setup_tool.py": "import os\nfrom distutils.core import setup\nimport imp\n",
        },
    )
    return tmp_path


def test_humble_to_lyrical_finds_each_change_once_at_its_line(humble_ws: Path) -> None:
    changes = scan(source(humble_ws), "humble", "lyrical", humble_ws)
    assert rules_of(changes) == [
        ("cmake.minimum_required", "CMakeLists.txt", 1),
        ("dependency.unreleased", "CMakeLists.txt", 7),
        ("cmake.ament_target_dependencies", "CMakeLists.txt", 11),
        ("cmake.ament_target_dependencies", "CMakeLists.txt", 14),
        ("cmake.ament_target_dependencies", "CMakeLists.txt", 17),
        ("dependency.unreleased", "package.xml", 11),
        ("dependency.unreleased", "package.xml", 12),
        ("python.removed_module", "scripts/setup_tool.py", 2),
        ("python.removed_module", "scripts/setup_tool.py", 3),
        ("include.removed", "src/driver.cpp", 2),
        ("include.removed", "src/driver.cpp", 3),
        ("include.removed", "src/driver.cpp", 8),
        ("cpp.get_typesupport_handle", "src/driver.cpp", 14),
        ("cpp.static_single_threaded_executor", "src/driver.cpp", 15),
        ("ros2_control.export_by_value", "src/hardware.cpp", 1),
    ]


def test_each_change_says_where_it_lands_and_why(humble_ws: Path) -> None:
    changes = {(c.rule, c.line): c for c in scan(source(humble_ws), "humble", "lyrical", humble_ws)}

    linked = changes[("cmake.ament_target_dependencies", 11)]
    assert linked.effect == "removed" and linked.release == "lyrical"
    assert linked.replacement is None
    assert linked.works_from is None
    assert "not measured" in linked.say
    assert linked.source == "https://github.com/ament/ament_cmake/pull/614"

    keyword = changes[("cmake.ament_target_dependencies", 14)]
    assert keyword.replacement is None

    eigen = changes[("cmake.ament_target_dependencies", 17)]
    assert eigen.replacement is None
    assert "Eigen3" in eigen.say and "not measured" in eigen.say

    header = changes[("include.removed", 2)]
    assert header.replacement == "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
    assert header.release == "jazzy" and header.works_from == "humble"

    event = changes[("include.removed", 3)]
    assert event.replacement == "rclcpp/event_handler.hpp"
    assert event.works_from == "iron"

    clamp = changes[("include.removed", 8)]
    assert clamp.replacement is None and "std::clamp" in clamp.say

    gazebo = next(
        c for c in changes.values() if c.found == "gazebo_ros" and c.file.endswith(".xml")
    )
    assert gazebo.release == "jazzy" and "Gazebo Classic reached end of life" in gazebo.say

    minimum = changes[("cmake.minimum_required", 1)]
    assert (minimum.found, minimum.replacement) == ("3.0.2", "3.8")

    typesupport = changes[("cpp.get_typesupport_handle", 14)]
    assert typesupport.replacement == "get_message_typesupport_handle"
    assert typesupport.works_from == "jazzy"

    distutils = changes[("python.removed_module", 2)]
    assert distutils.release == "jazzy" and "python 3.12" in distutils.say


def test_a_move_reports_only_what_it_crosses(humble_ws: Path) -> None:
    packages = source(humble_ws)
    to_iron = {c.rule for c in scan(packages, "humble", "iron", humble_ws)}
    assert "cmake.ament_target_dependencies" not in to_iron
    assert "cmake.minimum_required" not in to_iron

    to_kilted = {(c.rule, c.effect) for c in scan(packages, "jazzy", "kilted", humble_ws)}
    assert ("cmake.ament_target_dependencies", "deprecated") in to_kilted
    assert ("cpp.get_typesupport_handle", "removed") in to_kilted
    assert ("cpp.static_single_threaded_executor", "deprecated") in to_kilted


def test_a_dependency_missing_only_from_a_release_in_between_is_not_reported(
    tmp_path: Path,
) -> None:
    skipped = next(
        name
        for name, released in distros._facts()["packages"].items()
        if {"humble", "jazzy", "lyrical"} <= set(released) and "iron" not in released
    )
    package(tmp_path, "uses_it", [skipped], {})
    assert scan(source(tmp_path), "humble", "jazzy", tmp_path) == []
    assert scan(source(tmp_path), "humble", "iron", tmp_path)[0].release == "iron"


def test_a_package_of_the_workspace_is_never_an_unreleased_dependency(tmp_path: Path) -> None:
    package(tmp_path, "gazebo_ros", [], {})
    package(tmp_path, "user", ["gazebo_ros"], {})
    assert scan(source(tmp_path), "humble", "lyrical", tmp_path) == []


def test_a_workspace_library_without_exported_targets_is_linked_by_hand(tmp_path: Path) -> None:
    package(tmp_path, "mylib", [], {"CMakeLists.txt": "ament_export_libraries(mylib)\n"})
    package(tmp_path, "modern", [], {"CMakeLists.txt": "ament_export_targets(modernTargets)\n"})
    package(
        tmp_path,
        "app",
        ["mylib", "modern"],
        {
            "CMakeLists.txt": (
                "ament_target_dependencies(app modern)\nament_target_dependencies(app mylib)\n"
            )
        },
    )
    found = scan(source(tmp_path), "humble", "lyrical", tmp_path)
    assert [c.replacement for c in found] == ["target_link_libraries(app ${modern_TARGETS})", None]


def test_a_long_call_is_rewritten_one_target_per_line(tmp_path: Path) -> None:
    names = ["rclcpp", "rclcpp_components", "sensor_msgs", "geometry_msgs", "tf2_ros", "nav_msgs"]
    call = "  ament_target_dependencies(component_node\n    " + "\n    ".join(names) + ")\n"
    package(tmp_path, "app", names, {"CMakeLists.txt": call})
    for name in names:
        package(tmp_path, name, [], {"CMakeLists.txt": f"ament_export_targets({name}Targets)\n"})
    (change,) = scan(source(tmp_path), "humble", "lyrical", tmp_path)
    assert change.replacement == (
        "target_link_libraries(component_node\n"
        + "".join(f"    ${{{name}_TARGETS}}\n" for name in names)
        + "  )"
    )


def test_apply_rewrites_only_the_exact_changes_and_keeps_newlines(humble_ws: Path) -> None:
    cpp = humble_ws / "src" / "driver" / "src" / "driver.cpp"
    write(cpp, DRIVER_CPP.replace("\n", "\r\n"))
    changes = scan(source(humble_ws), "humble", "lyrical", humble_ws)
    migrate.apply(changes, humble_ws)
    text = cpp.read_bytes().decode()
    assert "\r\n" in text and "\n" not in text.replace("\r\n", "")
    assert "#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>" in text
    assert '#include "rclcpp/event_handler.hpp"' in text
    assert "rclcpp::executors::SingleThreadedExecutor executor;" in text
    assert '"rclcpp::get_typesupport_handle is gone"' in text
    assert "// #include <message_filters/subscriber.h>" in text
    assert (
        "ament_target_dependencies(driver rclcpp std_msgs)"
        in (humble_ws / "src/driver/CMakeLists.txt").read_text()
    )

    left = scan(source(humble_ws), "humble", "lyrical", humble_ws)
    assert all(not c.automatic for c in left)
    assert {c.rule for c in left} == {
        "cmake.ament_target_dependencies",
        "dependency.unreleased",
        "python.removed_module",
        "include.removed",
        "ros2_control.export_by_value",
    }


def test_apply_refuses_a_file_that_changed_since_the_plan(humble_ws: Path) -> None:
    changes = scan(source(humble_ws), "humble", "lyrical", humble_ws)
    cmake = humble_ws / "src" / "driver" / "CMakeLists.txt"
    write(cmake, "# edited\n" + DRIVER_CMAKE)
    with pytest.raises(UsageError, match="changed since it was planned"):
        migrate.apply(changes, humble_ws, force=True)


def test_apply_refuses_uncommitted_files_in_a_git_checkout(humble_ws: Path) -> None:
    git = ["git", "-C", str(humble_ws), "-c", "user.name=t", "-c", "user.email=t@example.com"]
    subprocess.run([*git, "init", "-q"], check=True)
    subprocess.run([*git, "add", "-A"], check=True)
    subprocess.run([*git, "commit", "-qm", "base"], check=True)
    cpp = humble_ws / "src" / "driver" / "src" / "driver.cpp"
    write(cpp, DRIVER_CPP + "// work in progress\n")
    changes = scan(source(humble_ws), "humble", "lyrical", humble_ws)
    with pytest.raises(UsageError, match=r"uncommitted changes in src/driver/src/driver\.cpp"):
        migrate.apply(changes, humble_ws)


def test_comments_and_strings_are_blanked_with_offsets_kept() -> None:
    text = 'a // x\nb /* y\nz */ "s\\"t" \'c\' R"d(raw)d" e'
    blanked = blank_comments_and_strings(text)
    assert len(blanked) == len(text)
    assert blanked == 'a     \nb     \n     "    " \' \' R"       " e'
    assert "raw" not in blank_comments_and_strings(text, strings=False)
    include = '#include "rcutils/get_env.h"\n'
    assert blank_comments_and_strings(include, strings=False) == include


def test_an_unknown_release_or_a_downgrade_is_a_usage_error(tmp_path: Path) -> None:
    with pytest.raises(UsageError, match="unknown distribution 'foxy'"):
        scan([], "foxy", "lyrical", tmp_path)
    with pytest.raises(UsageError, match="not an upgrade"):
        scan([], "lyrical", "humble", tmp_path)


def test_every_rule_and_successor_is_complete_and_measured() -> None:
    table = catalog()
    assert len({rule.id for rule in table.rules}) == len(table.rules)
    for rule in table.rules:
        assert rule.source.startswith("https://")
        if rule.removed and rule.deprecated:
            assert distros.index(rule.deprecated) < distros.index(rule.removed)
    for include, successor in table.headers.items():
        assert distros.header(include) is not None, include
        if successor.use:
            measured = distros.header(successor.use)
            assert measured is not None and measured.has("lyrical"), successor.use
    for name in table.packages:
        assert distros.released(name, "humble") and not distros.released(name, "lyrical"), name


def test_the_cli_plan_is_json_for_an_agent_and_text_for_a_person(
    humble_ws: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    assert (
        main(["migrate", "plan", str(humble_ws), "--from", "humble", "--to", "lyrical", "--json"])
        == 0
    )
    answer = json.loads(capsys.readouterr().out)
    data = answer["data"]
    assert data["summary"] == {
        "automatic": 5,
        "by_hand": 10,
        "changes": 15,
        "packages": 1,
        "removed": 14,
    }
    first = data["changes"][0]
    assert first == {
        "column": 32,
        "effect": "removed",
        "file": "src/driver/CMakeLists.txt",
        "found": "3.0.2",
        "line": 1,
        "package": "driver",
        "release": "lyrical",
        "replacement": "3.8",
        "rule": "cmake.minimum_required",
        "say": "CMake 4 (lyrical) refuses cmake_minimum_required below 3.5; ROS packages use 3.8",
        "source": "https://cmake.org/cmake/help/latest/release/4.0.html#deprecated-and-removed-features",
        "title": "cmake_minimum_required below 3.5",
        "works_from": "humble",
    }
    assert len(json.dumps(data)) < 8192

    assert main(["migrate", "plan", str(humble_ws), "--from", "humble", "--to", "lyrical"]) == 0
    text = capsys.readouterr().out
    assert text.startswith("rosio migrate humble -> lyrical: 15 changes in 1 package,")
    assert "  src/driver/CMakeLists.txt:11  removed in lyrical  ament_target_dependencies()" in text


CONTROL_LAUNCH = """from launch_ros.actions import Node

def generate_launch_description():
    # Ünïcode before the call, so a byte column is not a character column.
    control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        remappings=[("/diffbot_base_controller/cmd_vel", "/cmd_vel")],
    )
    other = Node(package="demo", executable="talker", remappings=[("a", "b")])
    return LaunchDescription([control, other])
"""


def test_the_control_and_executor_changes_of_lyrical(tmp_path: Path) -> None:
    package(
        tmp_path,
        "robot_control",
        ["hardware_interface", "controller_interface", "rclcpp"],
        {
            "src/robot.cpp": (
                "CallbackReturn Robot::on_init(const hardware_interface::HardwareInfo & info)\n"
                "{\n  options.memory_strategy = strategy;  // memory_strategy in a comment\n}\n"
            ),
            "src/chained.cpp": (
                "auto Chained::on_export_reference_interfaces()\n"
                "{ reference_interfaces_.resize(1); }\n"
            ),
            "launch/control.launch.py": CONTROL_LAUNCH,
        },
    )
    changes = scan(source(tmp_path), "humble", "lyrical", tmp_path)
    assert [(c.rule, c.file.split("/", 2)[-1], c.line, c.found) for c in changes] == [
        (
            "ros2_control.node_remappings",
            "launch/control.launch.py",
            8,
            '("/diffbot_base_controller/cmd_vel", "/cmd_vel")',
        ),
        (
            "ros2_control.chainable_exports",
            "src/chained.cpp",
            1,
            "on_export_reference_interfaces()",
        ),
        ("ros2_control.chainable_exports", "src/chained.cpp", 2, "reference_interfaces_"),
        (
            "ros2_control.on_init_hardware_info",
            "src/robot.cpp",
            1,
            "on_init(const hardware_interface::HardwareInfo",
        ),
        ("cpp.executor_memory_strategy", "src/robot.cpp", 3, ".memory_strategy"),
    ]
    remap = changes[0]
    assert remap.effect == "removed" and "--controller-ros-args" in remap.say
    assert scan(source(tmp_path), "humble", "kilted", tmp_path)[0].rule == (
        "ros2_control.chainable_exports"
    )


def test_control_remappings_are_reported_per_pair(tmp_path: Path) -> None:
    text = """from launch_ros.actions import Node
Node(executable="ros2_control_node", remappings=[
    ("~/robot_description", "/robot_description"),
    ("command", "/command"), ("state", destination),
])
Node(executable="ros2_control_node", remappings=remaps)
Node(executable="other", remappings=[("~/robot_description", "/robot_description")])
"""
    package(tmp_path, "control", [], {"launch/control.py": text})
    changes = scan(source(tmp_path), "humble", "lyrical", tmp_path)
    assert [(c.rule, c.line, c.found) for c in changes] == [
        (
            "ros2_control.robot_description_remap",
            3,
            '("~/robot_description", "/robot_description")',
        ),
        ("ros2_control.node_remappings", 4, '("command", "/command")'),
        ("ros2_control.node_remappings", 4, '("state", destination)'),
    ]
    assert changes == scan(source(tmp_path), "humble", "lyrical", tmp_path)
    description = changes[0]
    assert description.release == "jazzy"
    assert description.works_from == "jazzy"
    assert "delete" in description.say and "spawner" not in description.say
    assert "blob/jazzy/controller_manager/src/controller_manager.cpp#L" in description.source
    assert [c.rule for c in scan(source(tmp_path), "humble", "jazzy", tmp_path)] == [
        "ros2_control.robot_description_remap"
    ]
    assert [c.rule for c in scan(source(tmp_path), "jazzy", "lyrical", tmp_path)] == [
        "ros2_control.node_remappings",
        "ros2_control.node_remappings",
    ]


def test_the_agent_tool_hands_rewrites_to_apply_and_lists_the_rest(humble_ws: Path) -> None:
    from rosio.serve import tools

    answer = tools.migrate(str(humble_ws), "lyrical", start="humble")
    lines = answer.splitlines()
    assert lines[0] == "humble -> lyrical: 15 changes, 5 exact rewrites, 10 by hand"
    assert lines[1] == (
        "rewrites: run `rosio migrate apply --from humble --to lyrical` once, do not retype them"
    )
    assert "src/driver/package.xml:11 removed in jazzy: gazebo_ros is not released" in answer
    assert "src/driver/src/driver.cpp:3 apply rewrites rclcpp/qos_event.hpp to" in answer
    assert len(answer.encode("utf-8")) <= tools.MAX_BYTES
    assert "0 changes" in tools.migrate(str(humble_ws), "lyrical", start="humble", package="x")


def test_the_cli_apply_writes_the_diff_under_out(humble_ws: Path, capsys) -> None:
    out = humble_ws / "out"
    code = main(
        [
            "migrate",
            "apply",
            str(humble_ws),
            "--from",
            "humble",
            "--to",
            "lyrical",
            "--out",
            str(out),
        ]
    )
    assert code == 0
    diff = (out / "migrate" / "humble-to-lyrical.diff").read_text(encoding="utf-8")
    assert "-cmake_minimum_required(VERSION 3.0.2)" in diff
    assert "+cmake_minimum_required(VERSION 3.8)" in diff
