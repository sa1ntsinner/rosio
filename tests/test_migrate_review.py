from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from rosio import migrate
from rosio.errors import UsageError
from rosio.migrate.scan import SourcePackage, package_depends, scan


def _package(root: Path, filename: str, content: str | bytes) -> SourcePackage:
    base = root / "src/demo"
    base.mkdir(parents=True, exist_ok=True)
    (base / "package.xml").write_text(
        '<package format="3"><name>demo</name><version>0.1.0</version>'
        '<description>demo</description><maintainer email="m@example.com">m</maintainer>'
        "<license>Apache-2.0</license></package>",
        encoding="utf-8",
    )
    path = base / filename
    path.write_bytes(content.encode("utf-8") if isinstance(content, str) else content)
    return SourcePackage("demo", base, (path,), frozenset({"behaviortree_cpp_v3"}))


def test_plan_refuses_source_symlink_outside_workspace(tmp_path):
    root = tmp_path / "ws"
    package = _package(root, "node.cpp", "#include <rcutils/get_env.h>\n")
    outside = tmp_path / "outside.cpp"
    outside.write_bytes(package.files[0].read_bytes())
    package.files[0].unlink()
    package.files[0].symlink_to(outside)
    changes, planned_root = migrate.plan_workspace(root, "humble", "jazzy")
    assert len(changes) == 1
    assert changes[0].rule == "input.refused" and not changes[0].automatic
    assert "outside" in changes[0].say
    assert migrate.apply(changes, planned_root, force=True) == ""
    assert outside.read_text() == "#include <rcutils/get_env.h>\n"


def test_apply_rechecks_symlink_containment(tmp_path):
    root = tmp_path / "ws"
    package = _package(root, "node.cpp", "#include <rcutils/get_env.h>\n")
    changes = scan([package], "humble", "jazzy", root)
    outside = tmp_path / "outside.cpp"
    outside.write_bytes(package.files[0].read_bytes())
    package.files[0].unlink()
    package.files[0].symlink_to(outside)
    with pytest.raises(UsageError, match="outside"):
        migrate.apply(changes, root, force=True)
    assert outside.read_text() == "#include <rcutils/get_env.h>\n"


@pytest.mark.parametrize("relative", ["../outside.cpp", "absolute"])
def test_apply_refuses_paths_outside_workspace(tmp_path, relative):
    root = tmp_path / "ws"
    package = _package(root, "node.cpp", "#include <rcutils/get_env.h>\n")
    (change,) = scan([package], "humble", "jazzy", root)
    outside = tmp_path / "outside.cpp"
    outside.write_bytes(package.files[0].read_bytes())
    change = replace(change, file=str(outside) if relative == "absolute" else relative)
    with pytest.raises(UsageError, match="outside"):
        migrate.apply([change], root, force=True)
    assert outside.read_text() == "#include <rcutils/get_env.h>\n"


def test_unmeasured_dependency_exports_do_not_claim_compatibility(tmp_path):
    package = _package(tmp_path, "CMakeLists.txt", "ament_target_dependencies(demo nav2_core)\n")
    (change,) = scan([package], "humble", "lyrical", tmp_path)
    assert not change.automatic
    assert change.works_from is None
    assert "not measured" in change.say


@pytest.mark.parametrize("version", ["3.0...3.28", "3.0...3.5", "3.5...3.28"])
def test_cmake_policy_range_uses_maximum(tmp_path, version):
    package = _package(tmp_path, "CMakeLists.txt", f"cmake_minimum_required(VERSION {version})\n")
    assert scan([package], "humble", "lyrical", tmp_path) == []


def test_cmake_old_policy_range_keeps_range(tmp_path):
    package = _package(tmp_path, "CMakeLists.txt", "cmake_minimum_required(VERSION 3.0...3.4)\n")
    (change,) = scan([package], "humble", "lyrical", tmp_path)
    assert change.replacement == "3.0...3.8"


def test_cpp_literals_and_comments_do_not_create_migrations(tmp_path):
    text = (
        'const char *raw = R"doc(\n#include <rcutils/get_env.h>\n'
        'SequenceStarNode "quoted" tickRoot()\n)doc";\n'
        'const char *ordinary = "SequenceStarNode \\"tickRoot\\"";\n'
        "// #include <rcutils/get_env.h>\n"
        '/* SequenceStarNode */\n#include "rcutils/get_env.h"\nSequenceStarNode value;\n'
    )
    package = _package(tmp_path, "node.cpp", text)
    changes = scan([package], "humble", "lyrical", tmp_path)
    assert [(c.rule, c.line) for c in changes] == [("include.removed", 8), ("bt.sequence_star", 9)]
    migrate.apply(changes, tmp_path, force=True)
    assert package.files[0].read_text() == text.replace(
        '#include "rcutils/get_env.h"', '#include "rcutils/env.h"'
    ).replace("SequenceStarNode value", "SequenceWithMemory value")


@pytest.mark.parametrize(
    "directive",
    ['#include /* comment */ "rcutils/get_env.h"', '# /* comment */ include "rcutils/get_env.h"'],
)
def test_cpp_include_with_interleaved_comment_keeps_header_visible(tmp_path, directive):
    package = _package(tmp_path, "node.cpp", directive + "\n")
    changes = scan([package], "humble", "jazzy", tmp_path)
    assert len(changes) == 1
    change = changes[0]
    assert change.rule == "include.removed"
    migrate.apply([change], tmp_path, force=True)
    assert package.files[0].read_text() == directive.replace("get_env.h", "env.h") + "\n"


def test_manifest_dependencies_are_actual_xml_elements(tmp_path):
    text = (
        "<package><name>demo</name><!-- <depend>gazebo_ros</depend> -->"
        "<description><![CDATA[<depend>gazebo_ros</depend>]]></description>"
        '<depend condition="$ROS_VERSION == 2">rcutils</depend>'
        "<exec_depend> gazebo_ros </exec_depend></package>"
    )
    package = _package(tmp_path, "package.xml", text)
    assert package_depends(text) == frozenset({"rcutils", "gazebo_ros"})
    changes = scan([package], "humble", "jazzy", tmp_path)
    assert len(changes) == 1
    assert changes[0].span == (text.rindex("gazebo_ros"), text.rindex("gazebo_ros") + 10)


def test_xml_dependency_span_accepts_greater_than_in_attribute(tmp_path):
    text = (
        "<package><name>demo</name><description>naïve</description>"
        '<exec_depend condition="$ROS_VERSION > 1"> gazebo_ros </exec_depend></package>'
    )
    package = _package(tmp_path, "package.xml", text)
    assert package_depends(text) == frozenset({"gazebo_ros"})
    (change,) = scan([package], "humble", "jazzy", tmp_path)
    assert change.span == (text.index("gazebo_ros"), text.index("gazebo_ros") + 10)


def test_comment_only_xml_dependencies_do_not_enter_plan(tmp_path):
    text = "<package><name>demo</name><!-- <depend>gazebo_ros</depend> --></package>"
    package = _package(tmp_path, "package.xml", text)
    assert package_depends(text) == frozenset()
    assert scan([package], "humble", "jazzy", tmp_path) == []


@pytest.mark.parametrize("body", ["gazebo_&#114;os", "gazebo_<!-- note -->ros"])
def test_xml_dependency_values_use_parsed_text(tmp_path, body):
    text = f"<package><name>demo</name><depend> {body} </depend></package>"
    package = _package(tmp_path, "package.xml", text)
    assert package_depends(text) == frozenset({"gazebo_ros"})
    (change,) = scan([package], "humble", "jazzy", tmp_path)
    assert change.title == "gazebo_ros"
    assert change.found == body


@pytest.mark.parametrize(
    "other, warns", [("/behavior_server", True), ("/**/behavior_server", False), ("/**", False)]
)
def test_parameter_defaults_keep_full_node_selector(tmp_path, other, warns):
    text = (
        "/robot/behavior_server:\n  ros__parameters:\n    cycle_frequency: 10.0\n"
        f"{other}:\n  ros__parameters:\n    global_frame: odom\n"
    )
    package = _package(tmp_path, "params.yaml", text)
    changes = scan([package], "humble", "jazzy", tmp_path)
    changes = [c for c in changes if c.title == "behavior_server.global_frame"]
    assert bool(changes) is warns


def test_undecodable_input_remains_visible_in_plan(tmp_path):
    package = _package(tmp_path, "node.cpp", b"// \xff\n#include <rcutils/get_env.h>\n")
    changes = scan([package], "humble", "jazzy", tmp_path)
    assert len(changes) == 1
    change = changes[0]
    assert change.rule == "input.refused" and not change.automatic
    assert change.file == "src/demo/node.cpp"
    assert "UTF-8" in change.say
