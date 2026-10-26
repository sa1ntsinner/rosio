"""Container build results and their correspondence to a migration plan."""

from __future__ import annotations

import importlib
import io
import json
import shutil
import subprocess
import tarfile
from copy import deepcopy
from pathlib import Path

import pytest

from rosio.cli import main
from rosio.errors import UsageError
from rosio.migrate.scan import Change

FIXTURES = Path(__file__).parent / "fixtures/migrate_verify"


def module():
    return importlib.import_module("rosio.migrate.verify")


def change(file="src/controller/node.cpp", line=10, found="removed(\n  argument,\n  other)"):
    return Change(
        rule="api.removed",
        package="controller",
        file=file,
        line=line,
        column=1,
        found=found,
        title="removed API",
        effect="removed",
        release="jazzy",
        say="Replace the removed API.",
        source="https://example.org/api",
    )


def test_parse_and_match_captured_colcon_output():
    verify = module()
    packages = verify.parse_output(
        (FIXTURES / "colcon.txt").read_text(),
        {
            "controller": "src/controller",
            "geometry": "src/geometry",
            "dependent": "src/dependent",
            "utility": "src/utility",
        },
    )
    before = deepcopy(packages)
    matched = verify.match_errors(packages, [change()])
    assert packages == before
    packages = matched
    assert [(p["name"], p["status"]) for p in packages] == [
        ("controller", "failed"),
        ("dependent", "skipped"),
        ("geometry", "failed"),
        ("utility", "finished"),
    ]
    errors = packages[0]["errors"]
    placed = [e for e in errors if e["file"]]
    assert [(e["file"], e["line"], e["status"]) for e in placed] == [
        ("src/controller/node.cpp", 12, "listed"),
        ("src/controller/node.cpp", 40, "not_listed"),
    ]
    assert placed[0]["matches"] == [{"rule": "api.removed", "say": "Replace the removed API."}]
    assert next(e for e in errors if e["file"] is None)["message"] == (
        "collect2: error: ld returned 1 exit status"
    )
    cmake = packages[2]["errors"][0]
    assert len(packages[2]["errors"]) == 1
    assert (cmake["file"], cmake["line"]) == ("src/geometry/CMakeLists.txt", 8)
    assert 'provided by "missing"' in cmake["message"]


def test_unknown_failure_is_kept_and_relative_cmake_is_package_relative():
    verify = module()
    packages = verify.parse_output(
        "--- stderr: alpha\nCMake Error at CMakeLists.txt:3 (message):\n"
        "  custom failure\n\n---\nFailed <<< alpha [1s]\n"
        "--- stderr: beta\ncustom build tool refused the input\n---\n"
        "Failed <<< beta [1s]\n",
        {"alpha": "src/alpha", "beta": "src/beta"},
    )
    assert packages[0]["errors"][0]["file"] == "src/alpha/CMakeLists.txt"
    assert packages[1]["errors"][0]["message"] == "custom build tool refused the input"


def test_matching_requires_package_file_and_nearby_line():
    verify = module()
    packages = verify.parse_output(
        "--- stderr: controller\n"
        "/rosio_ws/src/src/controller/node.cpp:13:1: error: next statement\n"
        "/rosio_ws/src/src/controller/other.cpp:10:1: error: other file\n"
        "---\nFailed <<< controller [1s]\n",
        {"controller": "src/controller"},
    )
    packages = verify.match_errors(packages, [change()])
    assert all(e["status"] == "not_listed" for e in packages[0]["errors"])


def test_external_paths_and_clocks_do_not_escape_into_results():
    verify = module()
    packages = verify.parse_output(
        "--- stderr: alpha\n/opt/ros/jazzy/include/x.hpp:4:1: error: "
        "cannot read /tmp/random-123/input at 12:30:45\n---\nFailed <<< alpha [1s]\n",
        {"alpha": "src/alpha"},
    )
    error = packages[0]["errors"][0]
    assert error["file"] is None
    assert "/opt/" not in error["message"] and "/tmp/" not in error["message"]
    assert "12:30:45" not in error["message"]


def test_diagnostic_text_keeps_relative_source_paths():
    packages = module().parse_output(
        "--- stderr: alpha\n/rosio_ws/src/src/alpha/node.cpp:4:1: error: "
        "cannot include src/alpha/api.hpp\n---\nFailed <<< alpha [1s]\n",
        {"alpha": "src/alpha"},
    )
    assert packages[0]["errors"][0]["message"] == "cannot include src/alpha/api.hpp"


def test_diagnostic_timing_does_not_change_the_report():
    output = (
        "--- stderr: alpha\n"
        "[0.127s] ERROR: task failed at 2026-10-24T12:30:45Z\n"
        "---\nFailed <<< alpha [1s]\n"
    )
    verify = module()
    first = verify.parse_output(output, {"alpha": "src/alpha"})
    second = verify.parse_output(
        output.replace("0.127s", "9.426s").replace("2026-10-24T12:30:45Z", "2026-10-25T13:30:46Z"),
        {"alpha": "src/alpha"},
    )
    assert first == second
    assert first[0]["errors"][0]["message"] == "[<duration>] ERROR: task failed at <time>"


def test_cohesive_output_and_stderr_restate_one_error():
    output = (
        "--- output: controller\n"
        "CMake Error at CMakeLists.txt:10 (removed):\n  Unknown command\n\n"
        "-- Configuring incomplete, errors occurred!\n---\n"
        "--- stderr: controller\n"
        "CMake Error at CMakeLists.txt:10 (removed):\n  Unknown command\n\n---\n"
        "Failed <<< controller [1s]\n"
    )
    verify = module()
    packages = verify.parse_output(output, {"controller": "src/controller"})
    packages = verify.match_errors(packages, [change("src/controller/CMakeLists.txt")])
    assert len(packages[0]["errors"]) == 1
    assert packages[0]["errors"][0]["status"] == "listed"
    assert packages[0]["all_errors_listed"] is True


def test_source_context_is_not_an_error_and_unknown_diagnostics_stay_unplaced():
    output = (
        "--- stderr: controller\n"
        "/rosio_ws/src/src/controller/node.cpp:10:1: error: removed API\n"
        '  10 | removed(std::runtime_error("failure"));\n'
        "     | ^~~~~~~\n"
        "custom compiler refused this translation unit\n---\nFailed <<< controller [1s]\n"
    )
    packages = module().parse_output(output, {"controller": "src/controller"})
    assert len(packages[0]["errors"]) == 2
    assert packages[0]["errors"][0]["message"] == "custom compiler refused this translation unit"


@pytest.mark.parametrize("missing", [True, False])
def test_docker_unavailable_is_a_usage_error(monkeypatch, missing):
    verify = module()

    def run(*args, **kwargs):
        if missing:
            raise FileNotFoundError("docker")
        return subprocess.CompletedProcess(args[0], 1, "", "daemon unavailable")

    monkeypatch.setattr(verify.subprocess, "run", run)
    with pytest.raises(UsageError, match="Docker must be installed and running\\."):
        verify.build([], Path(), "jazzy")


def test_cli_verify_reports_failure_and_keeps_workspace_unchanged(tmp_path, monkeypatch, capsys):
    verify = module()
    from test_migrate import package

    package(tmp_path, "alpha", [], {"CMakeLists.txt": "cmake_minimum_required(VERSION 3.8)\n"})
    before = {p.relative_to(tmp_path): p.read_bytes() for p in tmp_path.rglob("*") if p.is_file()}
    calls = []

    def run(argv, **kwargs):
        calls.append(argv)
        if argv[1] == "info":
            return subprocess.CompletedProcess(argv, 0, "", "")
        if argv[1:3] == ["image", "inspect"]:
            return subprocess.CompletedProcess(argv, 1, "", "")
        with tarfile.open(fileobj=io.BytesIO(kwargs["stdin"].read())) as archive:
            assert "src/src/alpha/package.xml" in archive.getnames()
        return subprocess.CompletedProcess(
            argv, 1, "--- stderr: alpha\nunknown failure\n---\nFailed <<< alpha [1s]\n", ""
        )

    monkeypatch.setattr(verify.subprocess, "run", run)
    assert (
        main(["migrate", "verify", str(tmp_path), "--from", "humble", "--to", "jazzy", "--json"])
        == 1
    )
    answer = json.loads(capsys.readouterr().out)
    assert answer["ok"] is False
    assert answer["data"]["summary"]["built"] == 0
    assert answer["data"]["summary"]["total"] == 1
    assert answer["data"]["packages"][0]["errors"][0]["status"] == "not_listed"
    assert "ros:jazzy-ros-base" in calls[-1]
    assert not any("--mount" in arg or "--volume" in arg for arg in calls[-1])
    assert before == {
        p.relative_to(tmp_path): p.read_bytes() for p in tmp_path.rglob("*") if p.is_file()
    }


@pytest.mark.slow
def test_verify_two_packages_in_jazzy_container(tmp_path):
    if shutil.which("docker") is None:
        pytest.skip("docker is absent")
    if subprocess.run(["docker", "info"], capture_output=True).returncode:
        pytest.skip("docker is not running")
    from test_migrate import package

    for name, depends in (("alpha", []), ("beta", ["alpha"])):
        package(
            tmp_path,
            name,
            depends,
            {
                "CMakeLists.txt": (
                    f"cmake_minimum_required(VERSION 3.8)\nproject({name})\n"
                    "find_package(ament_cmake REQUIRED)\nament_package()\n"
                )
            },
        )
    result = module().verify_workspace(tmp_path, "humble", "jazzy", image="ros:jazzy-ros-base")
    assert result["summary"]["built"] == result["summary"]["total"] == 2
    assert result["ok"] is True
    assert not (tmp_path / "build").exists()
