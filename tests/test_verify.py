"""rosio verify: the parts that need no colcon."""

from __future__ import annotations

from pathlib import Path

from rosio import verify

BUILD = """Starting >>> rover_perception
--- stderr: rover_perception
src/rover_perception/src/scan_filter.cpp: In member function 'void ScanFilter::on_scan()':
src/rover_perception/src/scan_filter.cpp:35:5: error: 'retrun' was not declared in this scope
gmake[2]: *** [CMakeFiles/scan_filter.dir/build.make:76: scan_filter.o] Error 1
---
Failed   <<< rover_perception [3.1s, exited with code 2]
Finished <<< rover_control [0.9s]
"""

RESULT = """build/rover_control/pytest.xml: 2 tests, 0 errors, 1 failure, 0 skipped
- rover_control.test.test_speed test_speed
  <<< failure message
    AssertionError: speed limit wrong
    assert (1 + 1) == 3
  >>>

Summary: 2 tests, 0 errors, 1 failure, 0 skipped
"""


def test_a_changed_file_maps_to_the_package_holding_it(tmp_path: Path):
    package = tmp_path / "src" / "rover_perception"
    (package / "src").mkdir(parents=True)
    (package / "package.xml").write_text("<package><name>rover_perception</name></package>")
    (tmp_path / "build" / "x").mkdir(parents=True)
    (tmp_path / "build" / "x" / "package.xml").write_text("<package><name>x</name></package>")
    files = [package / "src" / "scan_filter.cpp", tmp_path / "build" / "x" / "a.cpp"]
    files.append(tmp_path / "README.md")
    assert verify.workspace_root(package / "src") == tmp_path.resolve()
    touched = verify.touched_packages(tmp_path, files)
    assert touched == {"rover_perception": "src/rover_perception"}


def test_build_and_test_output_reduce_to_the_decisive_lines():
    paths = {"rover_perception": "src/rover_perception", "rover_control": "src/rover_control"}
    built = verify.parse_build(BUILD, paths)
    assert built["rover_control"] == {"build": "built", "errors": []}
    assert built["rover_perception"]["build"] == "build failed"
    assert built["rover_perception"]["errors"] == [
        "src/rover_perception/src/scan_filter.cpp:35: 'retrun' was not declared in this scope"
    ]
    tests = verify.parse_tests(RESULT)
    assert (tests["total"], tests["failed"]) == (2, 1)
    assert tests["failures"] == [
        {
            "name": "rover_control.test.test_speed test_speed",
            "line": "AssertionError: speed limit wrong",
        }
    ]
    rows = [{"name": name, **built[name]} for name in sorted(built)]
    rows[0]["tests"] = tests
    assert verify.lines({"ok": False, "packages": rows}, cap=3) == [
        "rover_control: built, 1 tests failed",
        "  rover_control.test.test_speed test_speed: AssertionError: speed limit wrong",
        "... 2 lines cut, --json has them all",
    ]
