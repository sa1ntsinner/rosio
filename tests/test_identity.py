"""Node identity, and the machine dependent values that used to leak into it.

One runtime node must have one id no matter which collector met it first, and nothing inside
an id or inside compared data may name a host path, a line number or a temp file. Both
statements were false before, and each one below is the case that proved it.
"""

from __future__ import annotations

import tempfile
import types

import pytest

from rosio import ids
from rosio.collect import launch_declared
from rosio.config import Entrypoint, _entrypoint_from_command
from rosio.errors import UsageError
from rosio.resolve import _launch_child, identity, records, substitutions

TEMP = tempfile.gettempdir()


def worker(**over) -> records.ResolvedNode:
    """An unnamed node of a package with no name table row: the shape that splits."""
    fields = {
        "pkg": "acme",
        "executable": "worker",
        "namespace": "/",
        "name": None,
        "name_source": "executable_default",
        "argv": ("--frame-id", "base_link"),
        "remaps": (records.Remap(frm="points", to="/points_raw"),),
        "params": (
            records.ParamSource(order=0, kind="repo_file", path="/ws/src/acme/config/worker.yaml"),
        ),
    }
    fields.update(over)
    return records.ResolvedNode(**fields)


# ---------------------------------------------------------------------------
# one builder


def test_the_collector_uses_the_shared_builder() -> None:
    """No module keeps a second role digest. Two builders is what made one node two ids."""
    assert launch_declared._role is identity.role_digest
    assert launch_declared._identity is identity.identify
    assert launch_declared._stated_name is identity.stated_name


def test_one_unnamed_node_gets_one_id() -> None:
    node = worker()
    assert identity.node_id(node) == launch_declared._identity(node).node_id
    assert ids.is_synthetic(identity.node_id(node))


def test_the_digest_is_a_function_of_the_content_and_not_of_the_walk() -> None:
    """A record rebuilt from the same content digests the same, whichever collector holds it.

    The launch walk fills `role_content` where the roots are known, and a collector that
    computes it later from the same fields has to land on the same tuple.
    """
    node = worker()
    prebuilt = worker(role_content=identity.role_content(node.argv, node.remaps, node.params))
    assert identity.node_id(prebuilt) == identity.node_id(node)


def test_two_unnamed_siblings_differ_by_their_own_content() -> None:
    left = worker(argv=("--frame-id", "base_link", "--child-frame-id", "os_sensor"))
    right = worker(argv=("--frame-id", "base_link", "--child-frame-id", "gps"))
    assert identity.node_id(left) != identity.node_id(right)


def test_a_repeated_invocation_is_one_id_and_a_changed_one_is_another() -> None:
    """Three `ros2 run` invocations of one executable are three ids, not six.

    They differ by their argv, which is where a run entry point keeps its whole identity, and
    a node that genuinely repeats joins on that content rather than on iteration order.
    """
    invocations = [
        worker(form="process", argv=("--ros-args", "-p", "port:=1")),
        worker(form="process", argv=("--ros-args", "-p", "port:=2")),
        worker(form="process", argv=("--ros-args", "-p", "port:=3")),
    ]
    minted = [identity.node_id(node) for node in invocations]
    assert len(set(minted)) == 3
    again = identity.node_id(worker(form="process", argv=("--ros-args", "-p", "port:=1")))
    assert again == minted[0]


def test_a_named_node_joins_on_its_name() -> None:
    node = worker(name="worker_a", name_source="literal")
    decided = identity.identify(node)
    assert decided.node_id == ids.build_node("/worker_a")
    assert decided.fqn == "/worker_a"


def test_a_randomised_family_never_joins_on_a_name() -> None:
    node = worker(pkg="tf2_ros", executable="static_transform_publisher")
    decided = identity.identify(node)
    assert decided.fqn is None
    assert decided.name_source == "runtime_randomised"


# ---------------------------------------------------------------------------
# nothing machine specific inside an id


def test_a_temp_parameters_file_digests_the_same_twice() -> None:
    def merged(name: str) -> records.ResolvedNode:
        source = records.ParamSource(order=0, kind="external_file", path=f"{TEMP}/{name}")
        return worker(params=(source,))

    first = merged("a.yaml")
    second = merged("b.yaml")
    assert identity.node_id(first) == identity.node_id(second)
    assert identity.EXTERNAL_TOKEN in identity.content_of(first)[-1]


def test_a_mount_point_does_not_change_an_id() -> None:
    left = worker(argv=("--config", "/ssd/ws/src/acme/cfg/a.yaml"))
    right = worker(argv=("--config", "/mnt/other/ws/src/acme/cfg/a.yaml"))
    assert identity.node_id(left) == identity.node_id(right)


def test_a_ros_name_in_argv_is_left_alone() -> None:
    assert identity.role_token("/ouster/points") == "/ouster/points"
    assert identity.role_token("/opt/ros/humble/share/nav2/params.yaml") == "params.yaml"
    assert identity.role_token(f"{TEMP}/x_merged.yaml") == identity.EXTERNAL_TOKEN
    assert identity.role_token("plain") == "plain"


def test_a_temp_path_in_an_inline_value_digests_the_same_twice() -> None:
    """The same hole as a temp parameters file, one field further in. A launch file that
    names the merged file inside an inline dict used to put the random name in the digest."""

    def named(name: str) -> records.ResolvedNode:
        source = records.ParamSource(
            order=0, kind="inline_dict", values=(("yaml_filename", f"{TEMP}/{name}"),)
        )
        return worker(params=(source,))

    assert identity.node_id(named("a.yaml")) == identity.node_id(named("b.yaml"))
    assert identity.EXTERNAL_TOKEN in identity.content_of(named("a.yaml"))[-1]


def test_a_mount_point_in_an_inline_value_does_not_change_an_id() -> None:
    def under(root: str) -> records.ResolvedNode:
        source = records.ParamSource(
            order=0, kind="inline_dict", values=(("yaml_filename", f"{root}/maps/yard.yaml"),)
        )
        return worker(params=(source,), role_content=())

    left = identity.role_digest(under("/ssd/ws/src"), ["/ssd/ws/src"])
    right = identity.role_digest(under("/mnt/other/ws/src"), ["/mnt/other/ws/src"])
    assert left == right


def test_a_ros_name_in_an_inline_value_is_left_alone() -> None:
    source = records.ParamSource(
        order=0, kind="inline_dict", values=(("cmd_vel_in_topic", "/cmd_vel_raw"),)
    )
    node = worker(argv=(), remaps=(), params=(source,))
    assert identity.content_of(node) == ("set:cmd_vel_in_topic=/cmd_vel_raw",)


def test_two_parameter_files_with_one_basename_are_two_nodes() -> None:
    roots = ["/ws/src"]

    def reading(path: str) -> records.ResolvedNode:
        source = records.ParamSource(order=0, kind="repo_file", path=path)
        return worker(params=(source,), role_content=())

    left = identity.role_digest(reading("/ws/src/acme/left/config.yaml"), roots)
    right = identity.role_digest(reading("/ws/src/acme/right/config.yaml"), roots)
    assert left != right
    moved = identity.role_digest(reading("/mnt/b/src/acme/left/config.yaml"), ["/mnt/b/src"])
    assert moved == left


def test_two_argv_paths_with_one_basename_are_two_nodes() -> None:
    roots = ["/ws/src"]
    assert identity.role_token("/ws/src/a/cfg.yaml", roots) != identity.role_token(
        "/ws/src/b/cfg.yaml", roots
    )


def test_an_inline_value_keeps_its_type() -> None:
    def setting(value: object) -> records.ResolvedNode:
        source = records.ParamSource(order=0, kind="inline_dict", values=(("value", value),))
        return worker(params=(source,))

    assert identity.node_id(setting(1)) != identity.node_id(setting("1"))
    assert identity.node_id(setting(1)) != identity.node_id(setting(1.0))
    assert identity.node_id(setting(True)) != identity.node_id(setting("True"))


def test_a_path_under_a_root_is_stated_relative_to_it() -> None:
    assert identity.relative_path("/ssd/ws/src/maps/site.posegraph", ["/ssd/ws/src"]) == (
        "maps/site.posegraph"
    )
    assert identity.relative_path("/mnt/b/ws/src/maps/site.posegraph", ["/mnt/b/ws/src"]) == (
        "maps/site.posegraph"
    )
    assert identity.relative_path("/elsewhere/site.posegraph", ["/ssd/ws/src"]).endswith(
        "/site.posegraph"
    )
    assert "/elsewhere" not in identity.relative_path("/elsewhere/site.posegraph", ["/ssd/ws/src"])


# ---------------------------------------------------------------------------
# bug 2, an OpaqueFunction discriminator


def _lambda_from(filename: str):
    code = compile("f = lambda context: []\n", filename, "exec")
    scope: dict = {}
    exec(code, scope)
    return scope["f"]


def test_a_callback_discriminator_carries_no_path_and_no_line() -> None:
    roots = ["/home/a/ws", "/mnt/b/ws"]
    left = substitutions.function_name(_lambda_from("/home/a/ws/launch/x.launch.py"), roots)
    right = substitutions.function_name(_lambda_from("/mnt/b/ws/launch/x.launch.py"), roots)
    assert left == right
    assert "/" not in left and ":" not in left


def test_a_named_callback_uses_its_qualified_name() -> None:
    def build_nodes(context):
        return []

    name = substitutions.function_name(build_nodes)
    assert name.endswith("build_nodes")
    assert "<locals>" not in name and "/" not in name


def test_two_callbacks_in_two_modules_stay_two_discriminators() -> None:
    roots = ["/ws"]
    left = substitutions.function_name(_lambda_from("/ws/launch/one.launch.py"), roots)
    right = substitutions.function_name(_lambda_from("/ws/launch/two.launch.py"), roots)
    assert left != right


def test_two_callbacks_in_one_module_stay_two_discriminators() -> None:
    """Dropping the line number must not collapse two OpaqueFunctions of one launch file
    into one launch entity id."""
    scope: dict = {}
    source = "a = lambda c: nodes(1)\nb = lambda c: other(2)\n"
    exec(compile(source, "/ws/x.launch.py", "exec"), scope)
    roots = ["/ws"]
    assert substitutions.function_name(scope["a"], roots) != substitutions.function_name(
        scope["b"], roots
    )


def test_a_callback_discriminator_carries_no_nested_code_repr() -> None:
    scope: dict = {}
    exec(compile("f = lambda c: [n for n in c]\n", "/ws/x.launch.py", "exec"), scope)
    content = substitutions.code_content(scope["f"].__code__)
    assert not any("code object" in item or "/ws" in item for item in content)


# ---------------------------------------------------------------------------
# bugs 8 and 9, what a probe records


def _probe_env(value, key: str = "NX_STANDBY_DELAY_S") -> dict[str, bool]:
    probes = _launch_child._Probes()
    wrapped = probes._wrap_env(lambda *_args: value)
    wrapped(key)
    return probes.seen


def test_an_environment_probe_keeps_the_value_that_decided_the_branch() -> None:
    assert _probe_env("5") == {"env:NX_STANDBY_DELAY_S=5": True}
    assert _probe_env("600") == {"env:NX_STANDBY_DELAY_S=600": True}
    assert _probe_env(None) == {"env:NX_STANDBY_DELAY_S=": False}


def test_a_long_environment_value_is_capped() -> None:
    seen = _probe_env("x" * 500)
    key = next(iter(seen))
    assert len(key) < 120
    assert key.endswith("...")


def test_a_filesystem_probe_key_is_workspace_relative() -> None:
    probes = _launch_child._Probes(["/srv/nx_ws/src"])
    wrapped = probes._wrap_path("exists", lambda *_args: True)
    wrapped("/srv/nx_ws/src/maps/site.posegraph")
    assert probes.seen == {"exists:maps/site.posegraph": True}


def test_the_same_probe_on_another_mount_gives_the_same_key() -> None:
    left = _launch_child._Probes(["/ssd/ws"])
    right = _launch_child._Probes(["/mnt/ws"])
    left._wrap_path("exists", lambda *_args: True)("/ssd/ws/map.posegraph")
    right._wrap_path("exists", lambda *_args: True)("/mnt/ws/map.posegraph")
    assert list(left.seen) == list(right.seen) == ["exists:map.posegraph"]


# ---------------------------------------------------------------------------
# bug 11, the entry point tokeniser in config.py


def test_a_quoted_argument_value_survives_tokenising() -> None:
    entry = _entrypoint_from_command('ros2 launch pkg file.launch.py path:="/a b" x:=1')
    assert isinstance(entry, Entrypoint)
    assert entry.args == (("path", "/a b"), ("x", "1"))
    assert entry.argv == ()


def test_an_unbalanced_quote_is_a_usage_error() -> None:
    with pytest.raises(UsageError):
        _entrypoint_from_command('ros2 launch pkg file.launch.py path:="/a b')


def test_a_run_entry_point_keeps_its_argv() -> None:
    entry = _entrypoint_from_command("ros2 run pkg node --ros-args -p name:='a b'")
    assert entry.verb == "run"
    assert entry.argv == ("--ros-args", "-p")
    assert entry.args == (("name", "a b"),)


# ---------------------------------------------------------------------------
# the module is importable with nothing optional installed


def test_identity_needs_no_optional_import() -> None:
    module = types.ModuleType("probe")
    exec("from rosio.resolve import identity", module.__dict__)
    assert module.__dict__["identity"] is identity
