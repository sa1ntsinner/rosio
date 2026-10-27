"""Middleware facts from proc and rules over one observed profile."""

from __future__ import annotations

import contextlib
import os
import signal
import subprocess
import sys
import tempfile
import time
import uuid
from pathlib import Path

import pytest

from rosio import ids, model, schema
from rosio.collect import CollectContext, probe, proc
from rosio.config import Config, ProfileSpec, RootSpec
from rosio.drift import middleware, nodes, rules
from rosio.errors import CollectorError
from rosio.workspace import Workspace

PROFILE = ids.build_profile("live")
ZENOH = "rmw_zenoh_cpp"


def context(tmp_path: Path) -> CollectContext:
    src = tmp_path / "src"
    src.mkdir(exist_ok=True)
    ws = Workspace.discover([RootSpec("src", str(src))])
    builder = model.AtlasBuilder()
    builder.node(PROFILE)
    cfg = Config(roots=(RootSpec("src", str(src)),), profiles=(ProfileSpec(slug="live"),))
    return CollectContext(ws=ws, cfg=cfg, builder=builder, profile="live")


@pytest.fixture
def router_turn():
    """A router binds one port per machine and every suite on the machine reads one process
    table, so two checkouts tested at once saw each other's router: the second router could
    not bind and exited, and the router checks skipped. The live tests take turns."""
    fcntl = pytest.importorskip("fcntl")
    with (Path(tempfile.gettempdir()) / "rosio-zenoh-router.lock").open("a") as handle:
        fcntl.flock(handle, fcntl.LOCK_EX)
        yield


def _group_alive(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    return True


def stop_groups(children: list[subprocess.Popen]) -> None:
    """`ros2 run` can exit before the router it started, so wait for the whole group."""
    for child in children:
        with contextlib.suppress(ProcessLookupError):
            os.killpg(child.pid, signal.SIGTERM)
    deadline = time.monotonic() + 5
    for child in children:
        while (child.poll() is None or _group_alive(child.pid)) and time.monotonic() < deadline:
            time.sleep(0.05)
        with contextlib.suppress(ProcessLookupError):
            os.killpg(child.pid, signal.SIGKILL)
        child.wait(timeout=5)


def process(root: Path, pid: int, argv: list[str], env: bytes | None = b"") -> None:
    directory = root / str(pid)
    directory.mkdir(parents=True)
    (directory / "cmdline").write_bytes("\0".join(argv).encode() + b"\0")
    if env is not None:
        (directory / "environ").write_bytes(env)


def node_argv(name: str = "talker") -> list[str]:
    return ["/opt/ros/jazzy/lib/demo_nodes_cpp/talker", "--ros-args", "-r", f"__node:={name}"]


@pytest.mark.parametrize(
    ("env", "expected"),
    [
        (b"RMW_IMPLEMENTATION=rmw_zenoh_cpp\0", ZENOH),
        (b"HOME=/private\0", "default"),
        (None, "unknown"),
    ],
)
def test_process_environment(tmp_path, env, expected):
    root = tmp_path / "proc"
    process(root, 10, node_argv(), env)
    ctx = context(tmp_path)
    proc.ProcCollector(root=str(root)).collect(ctx)
    atlas = ctx.builder.build()
    assert (
        atlas.entities[ids.build_node("/talker")].fact("observed", PROFILE).values["rmw"]
        == expected
    )
    assert schema.validate(atlas.to_json()) == []


def test_permission_denied_is_unknown(tmp_path, monkeypatch):
    root = tmp_path / "proc"
    process(root, 10, node_argv())
    read = Path.read_bytes

    def denied(path):
        if path.name == "environ":
            raise PermissionError("denied")
        return read(path)

    monkeypatch.setattr(Path, "read_bytes", denied)
    assert proc.scan(str(root))[0].rmw == "unknown"


def test_zenoh_configuration_values_never_enter_compared_data(tmp_path):
    root = tmp_path / "proc"
    env = b"\0".join(f"{key}=/private/session/config.json5".encode() for key in proc.ZENOH_ENV)
    process(root, 10, node_argv(), env)
    ctx = context(tmp_path)
    proc.ProcCollector(root=str(root)).collect(ctx)
    atlas = ctx.builder.build()
    fact = atlas.entities[ids.build_node("/talker")].fact("observed", PROFILE)
    assert fact.values["zenoh_environment"] == dict.fromkeys(proc.ZENOH_ENV, "present")
    assert b"/private" not in atlas.canonical_bytes()


@pytest.mark.parametrize(
    "argv",
    [
        ["/opt/ros/jazzy/lib/rmw_zenoh_cpp/rmw_zenohd"],
        ["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
        ["/usr/bin/python3", "/opt/ros/jazzy/bin/ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
        ["ros2", "run", "other", "rmw_zenohd"],
        ["/usr/bin/sleep", "1"],
    ],
)
def test_router_presence(tmp_path, argv):
    root = tmp_path / "proc"
    process(root, 10, node_argv())
    process(root, 11, argv)
    ctx = context(tmp_path)
    proc.ProcCollector(root=str(root)).collect(ctx)
    run = ctx.builder.atlas.by_kind("run")[0]
    expected = "absent" if argv[0] == "/usr/bin/sleep" or "other" in argv else "present"
    assert run.fact("observed", PROFILE).values["zenoh_router"] == expected
    assert len(ctx.builder.atlas.by_kind("node")) == 1


def test_environment_and_router_are_run_inputs_and_pids_are_volatile(tmp_path):
    def collect(pid, rmw, router=False):
        root = tmp_path / f"proc{pid}{rmw}{router}"
        process(root, pid, node_argv(), f"RMW_IMPLEMENTATION={rmw}\0".encode())
        if router:
            process(root, pid + 1, ["rmw_zenohd"])
        ctx = context(tmp_path)
        proc.ProcCollector(root=str(root)).collect(ctx)
        return ctx.builder.build().canonical_bytes()

    first = collect(10, ZENOH)
    assert first == collect(20, ZENOH)
    assert first != collect(30, "rmw_fastrtps_cpp")
    assert first != collect(40, ZENOH, True)


def atlas(
    rmws=(ZENOH, "default"),
    router="absent",
    default="rmw_fastrtps_cpp",
    confidence="certain",
    digest="sample",
    extracted=True,
):
    builder = model.AtlasBuilder()
    if extracted:
        builder.node(PROFILE)
    run = builder.add_run("observed", profile=PROFILE, input_digest=digest)
    values = {"zenoh_router": router}
    if default:
        values["rmw_default"] = default
    builder.fact(run, "observed", profile=PROFILE, values=values, confidence=confidence)
    for name, rmw in zip(("talker", "listener"), rmws, strict=False):
        builder.fact(
            ids.build_node(f"/{name}"),
            "observed",
            profile=PROFILE,
            values={"rmw": rmw},
            confidence=confidence,
        )
    return builder.build()


def test_rules_fire_with_stable_ids_and_registry():
    first = middleware.check(atlas())
    assert [finding.rule for finding in first] == ["rmw.mixed", "zenoh.router_unreachable"]
    assert all(finding.severity == "error" and finding.subject_key == PROFILE for finding in first)
    assert [finding.id for finding in first] == [
        finding.id for finding in middleware.check(atlas(digest="another_run"))
    ]
    assert "/talker" in first[0].say and "/listener" in first[0].say
    assert "RMW_IMPLEMENTATION" in first[0].fix
    assert "ros2 run rmw_zenoh_cpp rmw_zenohd" in first[1].fix
    assert rules.findings_for(atlas(), only=[row.rule for row in first]) == first


@pytest.mark.parametrize("rmws", [(ZENOH, ZENOH), ("default", "default"), (ZENOH, "unknown")])
def test_mixed_silent_on_one_known_rmw(rmws):
    assert middleware.rmw_mixed(atlas(rmws)) == []


def test_default_is_collapsed_only_when_run_states_it():
    assert middleware.rmw_mixed(atlas((ZENOH, "default"), default=ZENOH)) == []
    assert middleware.rmw_mixed(atlas((ZENOH, "default"), default="rmw_fastrtps_cpp"))


def test_an_unknown_default_beside_one_named_implementation_is_unproven():
    """The default may be the very implementation the other node names."""
    found = middleware.rmw_mixed(atlas(("rmw_cyclonedds_cpp", "default"), default=None))
    assert [row.severity for row in found] == ["warn"]
    assert "unless the default middleware is rmw_cyclonedds_cpp" in found[0].say
    named = middleware.rmw_mixed(atlas((ZENOH, "rmw_cyclonedds_cpp"), default=None))
    assert [row.severity for row in named] == ["error"]


@pytest.mark.parametrize("router", ["present", "unknown", None])
def test_router_rule_requires_absence(router):
    assert middleware.zenoh_router_unreachable(atlas(router=router)) == []


def test_no_observed_layer_or_unknown_facts_stay_silent():
    assert middleware.check(model.AtlasBuilder().build()) == []
    assert middleware.check(atlas(confidence="unknown")) == []
    assert middleware.zenoh_router_unreachable(atlas(("default", "default"))) == []


def test_router_context_alone_does_not_claim_an_observed_node_set(tmp_path):
    root = tmp_path / "proc"
    root.mkdir()
    ctx = context(tmp_path)
    ctx.builder.fact(
        ids.build_node("/talker"), "declared", profile=PROFILE, values={"name": "talker"}
    )
    proc.ProcCollector(root=str(root)).collect(ctx)
    result = ctx.builder.build()
    assert not result.layers["observed"].present
    assert nodes.observed_profiles(result) == ()
    assert nodes.node_declared_not_observed(result) == []
    assert middleware.check(result) == []


def test_profiles_do_not_mix():
    builder = model.AtlasBuilder()
    builder.atlas = atlas((ZENOH,), router="present")
    other = ids.build_profile("other")
    builder.node(other)
    builder.fact(ids.build_node("/listener"), "observed", profile=other, values={"rmw": "default"})
    assert middleware.check(builder.build()) == []


def test_node_lists_are_capped():
    builder = model.AtlasBuilder()
    builder.atlas = atlas()
    for index in range(12):
        builder.fact(
            ids.build_node(f"/extra{index}"), "observed", profile=PROFILE, values={"rmw": ZENOH}
        )
    assert "5 more" in middleware.rmw_mixed(builder.build())[0].say


@pytest.mark.ros
@pytest.mark.parametrize("mixed", [True, False], ids=["mixed", "zenoh_without_router"])
def test_live_middleware(tmp_path, mixed, router_turn):
    if sys.platform != "linux":
        pytest.skip("the process table requires Linux")
    prefix = Path("/opt/ros") / os.environ.get("ROS_DISTRO", "jazzy")
    if not (prefix / "lib/librmw_zenoh_cpp.so").is_file():
        pytest.skip("rmw_zenoh_cpp is not installed")
    pytest.importorskip("rclpy")
    if not mixed and any(row.is_router for row in proc.scan()):
        pytest.skip("a Zenoh router is already running")
    script = tmp_path / "node.py"
    script.write_text(
        "import rclpy\nfrom pathlib import Path\nimport sys\n"
        "rclpy.init()\nnode = rclpy.create_node('middleware_test')\n"
        "Path(sys.argv[1]).write_text(node.get_name())\n"
        "try:\n    rclpy.spin(node)\n"
        "finally:\n    node.destroy_node()\n    rclpy.shutdown()\n",
        encoding="utf-8",
    )
    token = uuid.uuid4().hex
    domain = str(1 + int(token[:8], 16) % 220)
    children = []
    try:
        for index in range(2):
            env = dict(os.environ, ROS_DOMAIN_ID=domain, ROS_LOCALHOST_ONLY="1")
            for key in proc.ZENOH_ENV:
                env.pop(key, None)
            if index == 0 or not mixed:
                env["RMW_IMPLEMENTATION"] = ZENOH
                env["ZENOH_ROUTER_CHECK_ATTEMPTS"] = "1"
            else:
                env.pop("RMW_IMPLEMENTATION", None)
            ready = tmp_path / f"ready{index}"
            child = subprocess.Popen(
                [
                    sys.executable,
                    str(script),
                    str(ready),
                    "--ros-args",
                    "-r",
                    f"__node:=middleware_{token}_{index}",
                ],
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
            children.append(child)
            deadline = time.monotonic() + 20
            while not ready.exists() and child.poll() is None and time.monotonic() < deadline:
                time.sleep(0.05)
            assert ready.exists(), (
                child.communicate(timeout=2)[1].decode()
                if child.poll() is not None
                else "node did not initialise"
            )
        pids = {child.pid for child in children}
        rows = [row for row in proc.scan() if row.pid in pids or row.is_router]
        assert len([row for row in rows if row.pid in pids]) == 2
        ctx = context(tmp_path)
        proc.ProcCollector(processes=rows).collect(ctx)
        found = middleware.check(ctx.builder.build())
        expected = "rmw.mixed" if mixed else "zenoh.router_unreachable"
        assert expected in {finding.rule for finding in found}
        if not mixed:
            assert "rmw.mixed" not in {finding.rule for finding in found}
    finally:
        for child in children:
            if child.poll() is None:
                child.terminate()
            try:
                child.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.communicate(timeout=5)


def test_a_probe_only_atlas_names_the_run_it_holds():
    """`rosio probe` on a running system with no extraction behind it knows the profile by its
    slug only. A finding about the missing profile entity made the whole atlas unwritable."""
    probed = atlas(extracted=False)
    found = middleware.check(probed)
    assert [finding.rule for finding in found] == ["rmw.mixed", "zenoh.router_unreachable"]
    assert all(finding.subject_key in probed.entities for finding in found)
    assert all(finding.subject_key.startswith("run:observed") for finding in found)


@pytest.mark.parametrize("verb", ["run", "launch"])
@pytest.mark.parametrize(
    "environment,rmw",
    [(b"", "default"), (None, "unknown"), (b"RMW_IMPLEMENTATION=rmw_zenoh_cpp\0", ZENOH)],
)
def test_launcher_environment(tmp_path, verb, environment, rmw):
    root = tmp_path / "proc"
    process(
        root,
        10,
        ["/usr/bin/python3", "/opt/ros/jazzy/bin/ros2", verb, "demo_nodes_cpp", "talker"],
        environment,
    )
    ctx = context(tmp_path)
    proc.ProcCollector(root=str(root)).collect(ctx)
    result = ctx.builder.build()
    run = result.by_kind("run")[0]
    assert run.fact("observed", PROFILE).values["launchers"] == [
        {"command": f"ros2 {verb} demo_nodes_cpp talker", "rmw": rmw}
    ]
    assert result.by_kind("node") == []
    assert schema.validate(result.to_json()) == []
    assert b"/opt/ros" not in result.canonical_bytes()


@pytest.mark.parametrize("claimed", [False, True])
def test_launcher_parentage(tmp_path, claimed):
    root = tmp_path / "proc"
    process(root, 10, ["ros2", "run", "demo_nodes_cpp", "talker"])
    process(root, 11, node_argv() if claimed else ["talker"])
    (root / "11" / "stat").write_text("11 (talker with spaces) S 10 0 0")
    ctx = context(tmp_path)
    proc.ProcCollector(root=str(root)).collect(ctx)
    launchers = ctx.builder.atlas.by_kind("run")[0].fact("observed", PROFILE).values["launchers"]
    assert len(launchers) == (0 if claimed else 1)


def test_launcher_middleware_fires_without_claimed_nodes(tmp_path):
    root = tmp_path / "proc"
    process(
        root, 10, ["ros2", "run", "demo_nodes_cpp", "talker"], b"RMW_IMPLEMENTATION=rmw_zenoh_cpp\0"
    )
    process(root, 11, ["ros2", "run", "demo_nodes_cpp", "listener"])
    ctx = context(tmp_path)
    proc.ProcCollector(root=str(root)).collect(ctx)
    found = middleware.check(ctx.builder.build())
    assert {finding.rule for finding in found} == {"rmw.mixed", "zenoh.router_unreachable"}
    mixed = next(finding for finding in found if finding.rule == "rmw.mixed")
    assert "ros2 run demo_nodes_cpp talker" in mixed.say
    assert "ros2 run demo_nodes_cpp listener" in mixed.say


def test_named_launcher_is_deduplicated_when_parentage_is_unknown(tmp_path):
    root = tmp_path / "proc"
    process(
        root,
        10,
        ["ros2", "run", "demo_nodes_cpp", "talker", "--ros-args", "-r", "__node:=talker"],
        b"RMW_IMPLEMENTATION=rmw_zenoh_cpp\0",
    )
    process(root, 11, node_argv(), b"RMW_IMPLEMENTATION=rmw_zenoh_cpp\0")
    process(root, 12, ["ros2", "run", "demo_nodes_cpp", "listener"])
    ctx = context(tmp_path)
    proc.ProcCollector(root=str(root)).collect(ctx)
    result = ctx.builder.build()
    assert len(result.by_kind("run")[0].fact("observed", PROFILE).values["launchers"]) == 2
    mixed = middleware.rmw_mixed(result)[0]
    assert "/talker" in mixed.say
    assert "ros2 run demo_nodes_cpp talker" not in mixed.say
    assert "ros2 run demo_nodes_cpp listener" in mixed.say


def test_an_unnamed_launcher_is_its_executable_named_node(tmp_path):
    """`ros2 run demo_nodes_cpp talker` starts a node called talker. With the node on the
    same middleware and no parentage, the finding names it once, not as a node and again as
    the command that started it."""
    root = tmp_path / "proc"
    zenoh = b"RMW_IMPLEMENTATION=rmw_zenoh_cpp\0"
    process(root, 10, ["ros2", "run", "demo_nodes_cpp", "talker"], zenoh)
    process(root, 11, node_argv(), zenoh)
    process(root, 12, ["ros2", "run", "demo_nodes_cpp", "listener"])
    ctx = context(tmp_path)
    proc.ProcCollector(root=str(root)).collect(ctx)
    mixed = middleware.rmw_mixed(ctx.builder.build())[0]
    assert "/talker" in mixed.say
    assert "ros2 run demo_nodes_cpp talker" not in mixed.say
    assert "ros2 run demo_nodes_cpp listener" in mixed.say


@pytest.mark.parametrize(
    "values,forced,expected",
    [
        ([], None, ["default"]),
        (["unknown"], None, ["default"]),
        (["default", ZENOH, ZENOH, "unknown"], None, ["default", ZENOH]),
        ([ZENOH], ["default", "default"], ["default"]),
    ],
)
def test_middleware_pass_planning(values, forced, expected):
    rows = [
        proc.RosProcess(argv=("ros2", "run", "demo_nodes_cpp", "talker"), verb="run", rmw=value)
        for value in values
    ]
    assert probe.plan_passes(rows, forced) == expected


def test_missing_library_refuses_only_its_pass(tmp_path, monkeypatch):
    seen = []

    def fake(**kwargs):
        seen.append(kwargs["rmw"])
        if kwargs["rmw"] == ZENOH:
            return probe.ObservedSystem(reached=False, notes=["rmw_library_missing"])
        return probe.ObservedSystem(nodes=[probe.ObservedNode("talker", "/", "/talker")])

    monkeypatch.setattr(probe, "probe", fake)
    ctx = context(tmp_path)
    probe.ProbeCollector(rmws=[ZENOH, "default"], processes=[]).collect(ctx)
    result = ctx.builder.build()
    assert seen == ["default", ZENOH]
    assert result.entities[ids.build_node("/talker")].fact("observed", PROFILE).values[
        "seen_through"
    ] == ["default"]
    assert result.by_kind("run")[0].attrs["middleware_passes"][-1] == {
        "rmw": ZENOH,
        "state": "refused",
        "reason": "rmw_library_missing",
    }


def test_a_failed_child_does_not_stop_the_next_pass(tmp_path, monkeypatch):
    def fake(**kwargs):
        if kwargs["rmw"] == "default":
            raise CollectorError("child timed out")
        return probe.ObservedSystem(nodes=[probe.ObservedNode("talker", "/", "/talker")])

    monkeypatch.setattr(probe, "probe", fake)
    ctx = context(tmp_path)
    probe.ProbeCollector(rmws=["default", ZENOH], processes=[]).collect(ctx)
    result = ctx.builder.build()
    assert result.by_kind("node")[0].fact("observed", PROFILE).values["seen_through"] == [ZENOH]
    assert result.by_kind("run")[0].attrs["middleware_passes"][0]["state"] == "refused"


@pytest.mark.parametrize("rmw", ["default", ZENOH])
def test_pass_environment_overrides_setup_and_caller(monkeypatch, rmw):
    monkeypatch.setenv("RMW_IMPLEMENTATION", "caller_rmw")
    seen = {}

    def run(argv, **kwargs):
        seen.update(kwargs)
        seen["script"] = argv[-1]
        return subprocess.CompletedProcess(argv, 0, probe.MARKER + model.dumps({}), "")

    monkeypatch.setattr(subprocess, "run", run)
    probe.probe(rmw=rmw, ros_setup=["setup.bash"])
    expected = (
        "unset RMW_IMPLEMENTATION" if rmw == "default" else f"export RMW_IMPLEMENTATION={rmw}"
    )
    assert seen["script"].index(". setup.bash") < seen["script"].index(expected)
    assert seen["env"].get("RMW_IMPLEMENTATION") == (None if rmw == "default" else rmw)


def test_two_passes_merge_one_node_and_keep_disagreement(tmp_path, monkeypatch):
    def fake(**kwargs):
        return probe.ObservedSystem(
            nodes=[
                probe.ObservedNode(
                    "talker",
                    "/",
                    "/talker",
                    lifecycle_state=kwargs["rmw"],
                    parameters=(probe.Parameter("value", kwargs["rmw"], "string"),),
                )
            ]
        )

    monkeypatch.setattr(probe, "probe", fake)
    ctx = context(tmp_path)
    probe.ProbeCollector(rmws=["default", ZENOH], processes=[]).collect(ctx)
    result = ctx.builder.build()
    assert len(result.by_kind("node")) == 1
    values = result.by_kind("node")[0].fact("observed", PROFILE).values
    assert values["seen_through"] == ["default", ZENOH]
    assert "lifecycle_state" not in values
    observations = result.by_kind("run")[0].attrs["observations"]
    assert observations[ZENOH]["nodes"][0]["lifecycle_state"] == ZENOH
    assert observations["default"]["nodes"][0]["parameters"][0]["value"] == "default"
    assert schema.validate(result.to_json()) == []


@pytest.mark.ros
@pytest.mark.parametrize("with_router", [True, False], ids=["router", "no_router"])
def test_live_plain_ros2_run(tmp_path, monkeypatch, with_router, router_turn):
    if sys.platform != "linux":
        pytest.skip("the process table requires Linux")
    pytest.importorskip("rclpy")
    packages = pytest.importorskip("ament_index_python.packages")
    for package in (ZENOH, "demo_nodes_cpp"):
        try:
            packages.get_package_prefix(package)
        except packages.PackageNotFoundError:
            pytest.skip(f"{package} is not installed")
    if any(row.is_router for row in proc.scan()):
        pytest.skip("a Zenoh router is already running")
    domain = str(1 + int(uuid.uuid4().hex[:8], 16) % 220)
    monkeypatch.setenv("ROS_DOMAIN_ID", domain)
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ZENOH_ROUTER_CHECK_ATTEMPTS", "1")
    children = []

    def start(package, executable, rmw):
        env = dict(os.environ)
        for key in proc.ZENOH_ENV:
            env.pop(key, None)
        env["ZENOH_ROUTER_CHECK_ATTEMPTS"] = "1"
        if rmw:
            env["RMW_IMPLEMENTATION"] = rmw
        else:
            env.pop("RMW_IMPLEMENTATION", None)
        child = subprocess.Popen(
            ["ros2", "run", package, executable],
            env=env,
            start_new_session=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        children.append(child)
        return child

    try:
        if with_router:
            start(ZENOH, "rmw_zenohd", None)
            time.sleep(2)
        start("demo_nodes_cpp", "talker", ZENOH)
        start("demo_nodes_cpp", "listener", None)
        time.sleep(3)
        assert all(child.poll() is None for child in children)
        pids = {child.pid for child in children}
        rows = [row for row in proc.scan() if row.pid in pids or row.is_router]
        ctx = context(tmp_path)
        proc.ProcCollector(processes=rows).collect(ctx)
        probe.ProbeCollector(window=1, settle=8, processes=rows).collect(ctx)
        result = ctx.builder.build()
        found = middleware.check(result)
        mixed = next(finding for finding in found if finding.rule == "rmw.mixed")
        for name in ("talker", "listener"):
            # Named once: as the node when the probe saw it, else as the command.
            as_node = f"/{name}" in mixed.say
            assert as_node != (f"ros2 run demo_nodes_cpp {name}" in mixed.say), mixed.say
        if with_router:
            for name, rmw in (("talker", ZENOH), ("listener", "default")):
                fact = result.entities[ids.build_node(f"/{name}")].fact("observed", PROFILE)
                assert fact.values["seen_through"] == [rmw]
            assert not any(finding.rule == "zenoh.router_unreachable" for finding in found)
        else:
            assert any(finding.rule == "zenoh.router_unreachable" for finding in found)
        assert schema.validate(result.to_json()) == []
    finally:
        stop_groups(children)


MIDDLEWARE_KEYS = frozenset({"rmw", "zenoh_environment", "seen_through", "volatile", "run"})
"""What a fact may say differently when only the middleware changed."""

DDS_BLIND_QOS = ("depth", "history")
"""DDS discovery does not carry history or depth, so a DDS probe states them unknown where a
Zenoh probe reads them. Unknown against known is a narrower view, not a disagreement."""


def _middleware_view(atlas: model.Atlas) -> dict:
    def strip(value):
        if isinstance(value, dict):
            return {
                key: strip(item)
                for key, item in value.items()
                if key not in MIDDLEWARE_KEYS
                and not (key.startswith("qos.") and key[4:] in DDS_BLIND_QOS)
            }
        if isinstance(value, list):
            return [strip(item) for item in value]
        return value

    def qos(value):
        if isinstance(value, dict):
            if "qos" in value and isinstance(value["qos"], dict):
                value = {
                    **value,
                    "qos": {k: v for k, v in value["qos"].items() if k not in DDS_BLIND_QOS},
                }
            return {key: qos(item) for key, item in value.items()}
        if isinstance(value, list):
            return [qos(item) for item in value]
        return value

    data = atlas.to_json()
    return qos(
        strip(
            {
                "entities": {
                    key: value
                    for key, value in data["entities"].items()
                    if not key.startswith("run:")
                },
                "edges": data["edges"],
                "findings": sorted(finding.id for finding in atlas.findings),
            }
        )
    )


@pytest.mark.ros
@pytest.mark.slow
def test_one_system_probed_on_dds_and_on_zenoh_is_one_atlas(tmp_path, monkeypatch, router_turn):
    """Acceptance gate G6: the middleware a system runs on is a fact about it, not a second
    system. The same talker and listener probed on Fast DDS and on Zenoh give the same nodes,
    endpoints, parameters and findings, and differ only in the middleware facts and in the
    two QoS policies DDS discovery never reports."""
    if sys.platform != "linux":
        pytest.skip("the process table requires Linux")
    pytest.importorskip("rclpy")
    packages = pytest.importorskip("ament_index_python.packages")
    for package in (ZENOH, "rmw_fastrtps_cpp", "demo_nodes_cpp"):
        try:
            packages.get_package_prefix(package)
        except packages.PackageNotFoundError:
            pytest.skip(f"{package} is not installed")
    if any(row.is_router for row in proc.scan()):
        pytest.skip("a Zenoh router is already running")
    monkeypatch.setenv("ROS_DOMAIN_ID", str(1 + int(uuid.uuid4().hex[:8], 16) % 220))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    for key in proc.ZENOH_ENV:
        monkeypatch.delenv(key, raising=False)

    def probed(rmw: str) -> model.Atlas:
        env = dict(os.environ, RMW_IMPLEMENTATION=rmw)
        commands = [["ros2", "run", "demo_nodes_cpp", name] for name in ("talker", "listener")]
        if rmw == ZENOH:
            commands.insert(0, ["ros2", "run", ZENOH, "rmw_zenohd"])
        children = []
        try:
            for command in commands:
                children.append(
                    subprocess.Popen(
                        command,
                        env=env,
                        start_new_session=True,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                )
                time.sleep(2 if command[-1] == "rmw_zenohd" else 0)
            time.sleep(3)
            assert all(child.poll() is None for child in children)
            pids = {child.pid for child in children}
            rows = [row for row in proc.scan() if row.pid in pids]
            (tmp_path / rmw).mkdir()
            ctx = context(tmp_path / rmw)
            proc.ProcCollector(processes=rows).collect(ctx)
            probe.ProbeCollector(window=1, settle=8, processes=rows).collect(ctx)
            built = ctx.builder.build()
            built.findings = nodes.check(built) + middleware.check(built)
            return built
        finally:
            stop_groups(children)

    dds = probed("rmw_fastrtps_cpp")
    zenoh = probed(ZENOH)
    for name in ("/talker", "/listener"):
        node = ids.build_node(name)
        assert dds.entities[node].fact("observed", PROFILE).values["rmw"] == "rmw_fastrtps_cpp"
        assert zenoh.entities[node].fact("observed", PROFILE).values["rmw"] == ZENOH
    view = _middleware_view(dds)
    assert any(
        edge["kind"] == "publishes" and edge["dst"] == ids.build_topic("/chatter")
        for edge in view["edges"].values()
    )
    assert view == _middleware_view(zenoh)


def test_a_process_on_another_domain_is_not_part_of_the_probed_system(tmp_path):
    root = tmp_path / "proc"
    process(root, 10, node_argv("here"), b"ROS_DOMAIN_ID=91\0")
    process(
        root, 11, node_argv("elsewhere"), b"ROS_DOMAIN_ID=7\0RMW_IMPLEMENTATION=rmw_zenoh_cpp\0"
    )
    process(root, 12, node_argv("unset"), b"HOME=/private\0")
    process(root, 13, node_argv("unreadable"), None)
    process(root, 14, ["/opt/ros/jazzy/lib/rmw_zenoh_cpp/rmw_zenohd"], b"ROS_DOMAIN_ID=7\0")
    kept = [row.node_name or row.executable for row in proc.scan(str(root), domain="91")]
    # unreadable stays, because nothing proves it is elsewhere; a router serves every domain
    assert sorted(kept) == ["here", "rmw_zenohd", "unreadable"]
    assert "unset" in [row.node_name for row in proc.scan(str(root), domain="0")]
    assert len(proc.scan(str(root))) == 5


def test_a_domain_is_compared_as_the_number_rcl_reads(tmp_path):
    root = tmp_path / "proc"
    process(root, 10, node_argv("padded"), b"ROS_DOMAIN_ID=07\0")
    assert [row.node_name for row in proc.scan(str(root), domain="7")] == ["padded"]
    assert proc.scan(str(root), domain="8") == []
