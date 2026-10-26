"""The parameter candidate chain, against the expected chains in tests/fixtures/expected.

The launch walk is not executed here. The records the walk has to produce are built by hand
from the fixture launch files, so a chain is tested for what it says about yaml, order,
wildcards, plugin blocks and provenance, and not for the launch resolver.

One expectation in the fixtures is not reproduced on purpose, see KNOWN_DEVIATIONS.
"""

from __future__ import annotations

import json
import shutil
from pathlib import Path

import pytest
import yaml

from rosio import ids, model
from rosio.collect import CollectContext, entrypoints
from rosio.collect import params as params_collect
from rosio.config import Config, Entrypoint, ProfileSpec, RootSpec
from rosio.errors import CollectorError
from rosio.resolve import _launch_child as launch_child
from rosio.resolve import identity, py_pipeline, records, rewritten_yaml, ros_args, yaml_lines
from rosio.workspace import Workspace

EXPECTED = Path(__file__).parent / "fixtures" / "expected" / "params"
WS = Path(__file__).parent / "fixtures" / "ws_launch"


def test_a_command_name_cannot_match_a_guessed_yaml_owner(ws):
    entry = str(WS / "src/lx_bringup/launch/param_order.launch.py")
    path = str(WS / "src/lx_bringup/config/order_base.yaml")
    owner = records.ResolvedNode(
        pkg="lx_bringup",
        executable="inline_wins",
        namespace="/",
        command_blinds=(("name", "printf actual_name"),),
        params=(
            records.ParamSource(order=0, kind="repo_file", path=path),
            records.ParamSource(order=1, kind="inline_dict", values=(("known", 42),)),
        ),
    )
    walk = records.LaunchWalk(entry_path=entry, nodes=[owner])
    rows = params_collect.chain_for(walk, owner, ws=ws)
    assert [(row.name, row.effective, row.owner_fqn) for row in rows] == [("known", 42, "")]
    orphans = params_collect.orphan_rows(walk, ws=ws)
    assert orphans
    assert all(row.state == "unresolved_owner" for row in orphans)


MECHANISMS = {
    "yaml_file": {
        model.Mechanism.LAUNCH_PARAMS_FILE.value,
        model.Mechanism.YAML_WILDCARD.value,
        model.Mechanism.YAML_BASE.value,
        model.Mechanism.YAML_OVERLAY_MERGE.value,
    },
    "launch_inline": {model.Mechanism.LAUNCH_INLINE_DICT.value},
    "ros_args_assign": {model.Mechanism.ROS_ARGS_P.value},
    "ros_args_file": {model.Mechanism.ROS_ARGS_PARAMS_FILE.value},
    "rewritten_yaml": {model.Mechanism.LAUNCH_REWRITTEN_YAML.value},
    "python_override": {model.Mechanism.PYTHON_OVERRIDE.value},
}

SHADOW_KINDS = {
    None: None,
    "overridden": model.ShadowKind.VALUE_CHANGED.value,
    "restated_identical": model.ShadowKind.RESTATED_IDENTICAL.value,
    "type_changed": model.ShadowKind.TYPE_CHANGED.value,
    "inert": model.ShadowKind.INERT_PLUGIN_BLOCK.value,
}

KNOWN_DEVIATIONS = {
    # argv_chains.json expects the block key `cropper:` in crop.yaml to configure the node
    # /prep/cropper. It does not: verified by running a node under humble with
    # `-r __node:=cropper -r __ns:=/prep` and that exact file, the parameter stays unset,
    # and with the key `/prep/cropper` it arrives. ids.param_key_matches is the only
    # matching rule and it agrees with the runtime, so these two rows keep the runtime
    # answer and the fixture note is wrong.
    ("/prep/cropper", "input_topic"),
    ("/prep/cropper", "rate_hz"),
}


# ---------------------------------------------------------------------------
# fixtures


@pytest.fixture(scope="module")
def ws() -> Workspace:
    return Workspace.discover(
        [
            RootSpec(kind="src", path=str(WS / "src")),
            RootSpec(kind="install", path=str(WS / "install")),
        ]
    )


def cfg(path: str) -> str:
    return str(WS / "src" / "lx_bringup" / "config" / path)


def expected(name: str) -> dict:
    return json.loads((EXPECTED / f"{name}.json").read_text())


def repo_file(order: int, path: str) -> records.ParamSource:
    return records.ParamSource(order=order, kind="repo_file", path=path)


def inline(order: int, values: dict, locator: str, entry: str) -> records.ParamSource:
    return records.ParamSource(
        order=order,
        kind="inline_dict",
        values=tuple(values.items()),
        site=records.Site(path=entry, locator=locator),
    )


def node(**kwargs) -> records.ResolvedNode:
    kwargs.setdefault("namespace", "/")
    kwargs.setdefault("name_source", "literal" if kwargs.get("name") else "executable_default")
    return records.ResolvedNode(**kwargs)


def zoo_walk() -> records.LaunchWalk:
    """The node set of nav_stack.launch.py, with its parameters lists in order."""
    entry = str(WS / "src" / "lx_bringup" / "launch" / "nav_stack.launch.py")
    base = cfg("zoo_base.yaml")
    overlay = cfg("zoo_overlay.yaml")
    wildcard = cfg("zoo_wildcard.yaml")
    return records.LaunchWalk(
        entry_path=entry,
        profile="navigation_zoo",
        entry_pkg="lx_bringup",
        nodes=[
            node(
                pkg="lx_bringup",
                executable="lx_controller",
                name="controller_server",
                namespace="/",
                params=(
                    repo_file(0, base),
                    repo_file(1, overlay),
                    inline(
                        2,
                        {"use_sim_time": False},
                        "Node(controller_server).parameters[2]",
                        entry,
                    ),
                ),
            ),
            node(
                pkg="lx_bringup",
                executable="lx_controller",
                name="local_costmap",
                namespace="local_costmap",
                params=(repo_file(0, base), repo_file(1, overlay)),
            ),
            node(
                pkg="lx_sensors",
                executable="health_monitor",
                name="map_server",
                namespace="/",
                params=(repo_file(0, base),),
            ),
            node(
                pkg="lx_sensors",
                executable="health_monitor",
                name="lifecycle_manager_navigation",
                namespace="/",
                params=(
                    inline(
                        0,
                        {"autostart": True, "node_names": ["controller_server"]},
                        "Node(lifecycle_manager_navigation).parameters[0]",
                        entry,
                    ),
                ),
            ),
            node(
                pkg="lx_sensors",
                executable="lidar_driver",
                name="os_driver",
                namespace="ouster",
                params=(repo_file(0, cfg("zoo_node_wildcard.yaml")), repo_file(1, wildcard)),
            ),
            node(
                pkg="lx_sensors",
                executable="scan_relay",
                name="scan_relay",
                namespace="/",
                params=(repo_file(0, cfg("zoo_dotted.yaml")), repo_file(1, wildcard)),
            ),
        ],
    )


def argv_walk() -> records.LaunchWalk:
    """exec_process.launch.py, whose node carries its whole configuration in its argv."""
    entry = str(WS / "src" / "lx_bringup" / "launch" / "exec_process.launch.py")
    site = records.Site(path=entry, locator="ExecuteProcess(cropper_proc).cmd")
    return records.LaunchWalk(
        entry_path=entry,
        profile="argv_zoo",
        entry_pkg="lx_bringup",
        nodes=[
            node(
                pkg="lx_bringup",
                executable="crop_cloud.py",
                form="process",
                name="cropper",
                name_source="argv",
                namespace="/prep",
                argv=("--ros-args", "-p", "rate_hz:=20.0"),
                params=(
                    records.ParamSource(
                        order=0,
                        kind="ros_args_assign",
                        values=(("rate_hz", "20.0"), ("decimate_rings", "2")),
                        site=site.with_locator(
                            "ExecuteProcess(cropper_proc).cmd[-p decimate_rings]"
                        ),
                    ),
                    records.ParamSource(order=1, kind="ros_args_file", path=cfg("crop.yaml")),
                ),
            )
        ],
    )


def rows_of(walk: records.LaunchWalk, ws: Workspace) -> dict[tuple[str, str], object]:
    out: dict[tuple[str, str], object] = {}
    for entry in walk.nodes:
        for row in params_collect.chain_for(walk, entry, ws=ws):
            out[(row.owner_fqn, row.name)] = row
    for row in params_collect.orphan_rows(walk, ws=ws):
        out[(row.selector_key, row.name)] = row
    return out


def relative(path: str) -> str:
    return str(Path(path).relative_to(WS))


# ---------------------------------------------------------------------------
# shape detection


def test_shape_detection_matches_the_expected_inventory(ws: Workspace):
    want = expected("shape_detection")
    for relpath in want["parameter_files"]:
        document = yaml_lines.load_file(str(WS / relpath))
        assert document.is_parameter_file(), relpath
    for row in want["refused"]:
        document = yaml_lines.load_file(str(WS / row["path"]))
        assert not document.is_parameter_file(), row["path"]
        assert yaml_lines.refuse_reason(document.data)


def test_workspace_parameter_files_are_the_same_set(ws: Workspace):
    want = set(expected("shape_detection")["parameter_files"])
    found = {relative(path) for path in ws.param_files()}
    assert {row for row in found if row.startswith("src/")} == want


def test_lines_are_recorded_for_every_key_path():
    document = yaml_lines.load_file(cfg("zoo_base.yaml"))
    assert document.line(("controller_server",)) == 4
    assert document.line(("controller_server", "ros__parameters", "controller_frequency")) == 6
    assert (
        document.locator(("controller_server", "ros__parameters", "FollowPath", "plugin"))
        == "controller_server.ros__parameters.FollowPath.plugin"
    )
    assert document.line(("local_costmap", "local_costmap", "ros__parameters", "width")) == 28


def test_a_dotless_exponent_is_a_float_because_rcl_reads_it_as_one(tmp_path: Path):
    """Verified against humble: 5e3 arrives as a float from a file and from a -p flag."""
    target = tmp_path / "sci.yaml"
    target.write_text("/relay:\n  ros__parameters:\n    gradient_tol: 5e-3\n    dotted: 1.0e-6\n")
    document = yaml_lines.load_file(str(target))
    values = document.data["/relay"]["ros__parameters"]
    assert values["gradient_tol"] == 0.005
    assert yaml_lines.type_name(values["gradient_tol"]) == "float"
    assert yaml_lines.type_name(values["dotted"]) == "float"
    assert ros_args.parse_value("1e3") == 1000.0
    assert ros_args.parse_value("20") == 20


def test_one_scalar_rule_types_an_argv_assignment(ws: Workspace):
    """rcl reads a `-p` value as an rcl scalar, and its boolean list is an exact case list.

    Measured one value per process against an rclpy node on humble: `on`, `off`, `yes`, `no`
    and the `y`, `Y`, `n`, `N` pair arrive as `Type.BOOL`, so keeping them strings fabricates
    a type change against the running system. `null` and `~` are yaml 1.1 None spellings that
    rcl never produces, and they stay strings.
    """
    for text in ("null", "~", "TrUe", "yEs"):
        assert ros_args.parse_value(text) == text
    for text in ("on", "yes", "y", "Y"):
        assert ros_args.parse_value(text) is True
    for text in ("off", "no", "n", "N"):
        assert ros_args.parse_value(text) is False
    assert isinstance(ros_args.parse_value("20"), int)
    assert ros_args.parse_value("20.0") == 20.0
    assert ros_args.parse_value("false") is False
    assert ros_args.parse_value("os_sensor") == "os_sensor"
    walk = argv_walk()
    entry = walk.nodes[0]
    walk.nodes[0] = records.ResolvedNode(
        pkg=entry.pkg,
        executable=entry.executable,
        form=entry.form,
        name=entry.name,
        name_source=entry.name_source,
        namespace=entry.namespace,
        argv=entry.argv,
        params=(
            records.ParamSource(
                order=0,
                kind="ros_args_assign",
                values=(("latch", "off"), ("depth", "20"), ("mode", "null")),
                site=entry.params[0].site,
            ),
        ),
    )
    rows = {row.name: row for row in params_collect.chain_for(walk, walk.nodes[0], ws=ws)}
    assert rows["latch"].effective is False
    assert rows["latch"].value_type == "bool"
    assert rows["depth"].effective == 20
    assert rows["depth"].value_type == "int"
    assert rows["mode"].effective == "null"


def test_a_mixed_type_argv_value_names_no_value_and_leaves_a_blind_spot(ws: Workspace):
    """rcl refuses the whole argument list over one bad `-p` value, so the process never
    starts. Measured one value per process on humble: `-p k0:=[1, x]` and `-p k0:=[true, 1]`
    both end `rcl_init` with "Sequence should be of same type". A candidate carrying a value
    for such a process is a fact about nothing, so the chain names no winner and the raw
    text stays as the blind spot.
    """
    walk = argv_walk()
    entry = walk.nodes[0]
    walk.nodes[0] = records.ResolvedNode(
        pkg=entry.pkg,
        executable=entry.executable,
        form=entry.form,
        name=entry.name,
        name_source=entry.name_source,
        namespace=entry.namespace,
        argv=entry.argv,
        params=(
            records.ParamSource(
                order=0,
                kind="ros_args_assign",
                values=(
                    ("rings", "[1, x]"),
                    ("flags", "[true, 1]"),
                    ("gaps", "[, 1]"),
                    ("good", "[1, 2]"),
                ),
                site=entry.params[0].site,
            ),
        ),
    )
    rows = {row.name: row for row in params_collect.chain_for(walk, walk.nodes[0], ws=ws)}

    assert rows["good"].effective == [1, 2]
    assert not rows["good"].unresolved

    for name, text, reason in (
        ("rings", "[1, x]", ros_args.SEQUENCE_MIXED_TYPES),
        ("flags", "[true, 1]", ros_args.SEQUENCE_MIXED_TYPES),
        ("gaps", "[, 1]", ros_args.VALUE_UNPARSABLE),
    ):
        row = rows[name]
        assert row.state != model.ParamState.EFFECTIVE.value, name
        assert row.effective is None, name
        assert not any(candidate.wins for candidate in row.candidates), name
        blinds = [blind for blind in row.unresolved if blind.reason == reason]
        assert blinds, name
        assert blinds[0].expr == f"-p {name}:={text}", name


def _argv_walk_with(values: tuple[tuple[str, str], ...]) -> records.LaunchWalk:
    walk = argv_walk()
    entry = walk.nodes[0]
    walk.nodes[0] = records.ResolvedNode(
        pkg=entry.pkg,
        executable=entry.executable,
        form=entry.form,
        name=entry.name,
        name_source=entry.name_source,
        namespace=entry.namespace,
        argv=entry.argv,
        params=(
            records.ParamSource(
                order=0,
                kind="ros_args_assign",
                values=values,
                site=entry.params[0].site,
            ),
        ),
    )
    return walk


def test_an_argv_value_that_sets_nothing_names_no_value(ws: Workspace):
    """An input that starts the node and leaves the parameter with no value at all.

    Measured on humble, one value per process, against an rclpy node with
    `automatically_declare_parameters_from_overrides`: `-p k0:=' '` starts the process and
    the parameter comes back `Type.NOT_SET` with value `None`, and so do a comment only value
    and an empty flow sequence. Naming the text as the effective value states a value no node
    holds, so the chain names none, and the input stays readable as the blind spot.
    """
    cases = (
        ("blank", " "),
        ("blank_wide", "   "),
        ("comment", "#c"),
        ("empty_seq", "[]"),
        ("nested_empty", "[[]]"),
        ("good", "20.0"),
    )
    walk = _argv_walk_with(cases)
    rows = {row.name: row for row in params_collect.chain_for(walk, walk.nodes[0], ws=ws)}
    assert rows["good"].effective == 20.0
    assert not rows["good"].unresolved
    for name, text in cases[:-1]:
        row = rows[name]
        assert row.state != model.ParamState.EFFECTIVE.value, name
        assert row.effective is None, name
        assert not any(candidate.wins for candidate in row.candidates), name
        blinds = [blind for blind in row.unresolved if blind.reason == ros_args.SETS_NOTHING]
        assert blinds, name
        assert blinds[0].expr == f"-p {name}:={text}", name


def test_a_nested_argv_sequence_names_no_value_and_records_a_blind_spot(ws: Workspace):
    """rcl takes a nested sequence, and what the node then holds is not in the text.

    Measured against a node on humble, one value per process: `-p k0:=[[1, 2], [3]]` comes
    back `[3]` and `-p k0:=[[1, 2], [a]]` comes back `[1, 2]`. Measured against
    `rcl_parse_yaml_value` directly, both leave more than one slot of `rcl_variant_t` filled,
    and which one the parameter takes is the order the client library reads the slots in.
    Only rclpy was measured, so the chain names no value and keeps the text.
    """
    cases = (("rings", "[[1, 2], [3]]"), ("gaps", "[[1, 2], [a]]"), ("good", "[1, 2]"))
    walk = _argv_walk_with(cases)
    rows = {row.name: row for row in params_collect.chain_for(walk, walk.nodes[0], ws=ws)}
    assert rows["good"].effective == [1, 2]
    for name, text in cases[:-1]:
        row = rows[name]
        assert row.state != model.ParamState.EFFECTIVE.value, name
        assert row.effective is None, name
        assert not any(candidate.wins for candidate in row.candidates), name
        blinds = [blind for blind in row.unresolved if blind.reason == ros_args.SEQUENCE_NESTED]
        assert blinds, name
        assert blinds[0].expr == f"-p {name}:={text}", name


def test_a_key_spelled_twice_cites_the_line_that_won(tmp_path: Path):
    target = tmp_path / "twice.yaml"
    target.write_text("/relay:\n  ros__parameters:\n    depth: 1\n    depth: 2\n")
    document = yaml_lines.load_file(str(target))
    assert document.data["/relay"]["ros__parameters"]["depth"] == 2
    assert document.line(("/relay", "ros__parameters", "depth")) == 4


def test_a_commented_value_is_not_a_key():
    document = yaml_lines.load_file(cfg("zoo_base.yaml"))
    params = document.data["controller_server"]["ros__parameters"]
    assert params["controller_frequency"] == 20.0
    assert len([key for key in params if key == "controller_frequency"]) == 1


def test_dotted_keys_and_nested_dicts_fold_to_one_name():
    document = yaml_lines.load_file(cfg("zoo_dotted.yaml"))
    block = yaml_lines.param_blocks(document)[0]
    names = {leaf.name for leaf in yaml_lines.leaves(block.values)}
    assert names == {
        "ground.bins",
        "ground.ceiling_m",
        "ground.max_slope",
        "obstacle.cluster_tolerance",
        "obstacle.voxel",
    }


def test_two_level_costmap_key_is_one_selector():
    document = yaml_lines.load_file(cfg("zoo_base.yaml"))
    selectors = {block.selector for block in yaml_lines.param_blocks(document)}
    assert "local_costmap/local_costmap" in selectors
    assert ids.param_key_matches("local_costmap/local_costmap", "/local_costmap/local_costmap")


# ---------------------------------------------------------------------------
# the zoo chains


def test_zoo_chains_match_the_expected_file(ws: Workspace):
    want = expected("zoo_chains")
    rows = rows_of(zoo_walk(), ws)
    checked = 0
    for entry in want["parameters"]:
        key = (entry["owner"] or entry["block_key"], entry["name"])
        assert key in rows, key
        _check_row(rows[key], entry)
        checked += 1
    assert checked == len(want["parameters"])


def test_zoo_owner_sources_are_the_recorded_order(ws: Workspace):
    want = {row["fqn"]: row["sources"] for row in expected("zoo_chains")["owners"]}
    walk = zoo_walk()
    for entry in walk.nodes:
        fqn = params_collect.owner_name(entry)
        files = [
            relative(source.path)
            for source in sorted(entry.params, key=lambda item: item.order)
            if source.path
        ]
        assert files == [row for row in want[fqn] if not row.startswith("<")]


def _check_row(row, entry: dict) -> None:
    assert row.state == entry["state"], entry["id"] or entry["name"]
    if entry["state"] == "effective":
        assert row.effective == entry["effective"]
        assert type(row.effective) is type(entry["effective"])
    else:
        assert row.effective is None
    assert row.value_type == entry["type"]
    if entry["id"]:
        assert row.param_id == entry["id"]
    assert len(row.candidates) == len(entry["candidates"]), row.name
    for candidate, want in zip(row.candidates, entry["candidates"], strict=True):
        assert candidate.rank == want["rank"]
        assert candidate.wins is want["wins"]
        assert relative(candidate.site.path) == want["file"]
        assert candidate.site.locator == want["locator"]
        assert candidate.value == want.get("value_as_yaml", want["value"])
        assert candidate.mechanism in MECHANISMS[want["mechanism"]]
        assert candidate.shadow_kind == SHADOW_KINDS[want["shadow_kind"]]
        if want["shadowed_by"] is None:
            assert candidate.shadowed_by is None
        else:
            assert candidate.shadowed_by == 0
            assert relative(row.candidates[0].site.path) == want["shadowed_by"]


def test_a_wildcard_line_owns_one_parameter_per_node_with_one_locator(ws: Workspace):
    rows = rows_of(zoo_walk(), ws)
    left = rows[("/ouster/os_driver", "diagnostics.window_sec")]
    right = rows[("/scan_relay", "diagnostics.window_sec")]
    assert left.candidates[0].site.locator == right.candidates[0].site.locator
    assert left.candidates[0].site.locator == "/**.ros__parameters.diagnostics.window_sec"
    assert left.candidates[0].mechanism == model.Mechanism.YAML_WILDCARD.value


def test_an_int_stays_an_int(ws: Workspace):
    rows = rows_of(zoo_walk(), ws)
    row = rows[("/local_costmap/local_costmap", "width")]
    assert row.effective == 40
    assert type(row.effective) is int
    assert row.value_type == "int"


def test_the_inert_block_has_a_candidate_and_no_effective_value(ws: Workspace):
    rows = rows_of(zoo_walk(), ws)
    row = rows[("/controller_server", "FollowPath_DISABLED.plugin")]
    assert row.state == model.ParamState.INERT_BY_DESIGN.value
    assert row.candidates[0].wins is False
    assert row.effective is None
    assert row.block == "FollowPath_DISABLED"


def test_a_block_no_list_names_records_no_selector_key(ws: Workspace):
    """The first selector key at that scope is a guess as soon as a node holds two lists,
    and a guess inside a recorded fact is worse than an absent field."""
    rows = rows_of(zoo_walk(), ws)
    assert rows[("/controller_server", "FollowPath_DISABLED.plugin")].selector_key == ""
    assert rows[("/controller_server", "FollowPath.plugin")].selector_key == "controller_plugins"


def test_the_selected_plugin_block_stays_effective(ws: Workspace):
    rows = rows_of(zoo_walk(), ws)
    assert rows[("/controller_server", "FollowPath.plugin")].state == "effective"
    assert rows[("/controller_server", "FollowPath.desired_linear_vel")].block == ""


def test_a_plain_nested_group_is_not_a_plugin_block(ws: Workspace):
    rows = rows_of(zoo_walk(), ws)
    row = rows[("/controller_server", "goal_checker.xy_goal_tolerance")]
    assert row.state == "effective"
    assert row.block == ""


def test_a_block_key_no_node_matches_is_owner_name_mismatch(ws: Workspace):
    rows = rows_of(zoo_walk(), ws)
    row = rows[("lifecycle_manager", "node_names")]
    assert row.state == model.ParamState.OWNER_NAME_MISMATCH.value
    assert row.effective is None
    assert row.owner_fqn == ""
    assert row.param_id == "param:/lifecycle_manager|node_names"
    assert row.candidates[0].wins is False
    assert relative(row.candidates[0].site.path) == "src/lx_bringup/config/zoo_base.yaml"
    assert row.candidates[0].site.line_hint == 49


def test_an_orphan_row_cites_the_block_key_and_not_the_first_key_in_it(ws: Workspace):
    """The finding over these rows tells a reader to delete the block, so it has to cite the
    line the block opens at. Every candidate cites one key inside the block, and in
    zoo_base.yaml the first of them is two lines past `lifecycle_manager:`, one for the key
    and one for `ros__parameters:`."""
    rows = rows_of(zoo_walk(), ws)
    for name in ("autostart", "node_names"):
        row = rows[("lifecycle_manager", name)]
        assert row.block_site is not None
        assert row.block_site.locator == "lifecycle_manager"
        assert row.block_site.line_hint == 46
        assert row.candidates[0].site.line_hint > 46
    assert rows[("lifecycle_manager", "autostart")].candidates[0].site.line_hint == 48
    assert rows[("/controller_server", "FollowPath.plugin")].block_site is None


def test_the_launched_manager_gets_its_inline_list_instead(ws: Workspace):
    rows = rows_of(zoo_walk(), ws)
    row = rows[("/lifecycle_manager_navigation", "node_names")]
    assert row.state == "effective"
    assert row.effective == ["controller_server"]


def test_an_explicit_empty_string_is_a_value(ws: Workspace):
    rows = rows_of(zoo_walk(), ws)
    row = rows[("/map_server", "yaml_filename")]
    assert row.effective == ""
    assert row.value_type == "str"


def test_a_bare_key_with_no_value_is_null(ws: Workspace):
    rows = rows_of(zoo_walk(), ws)
    row = rows[("/scan_relay", "ground.ceiling_m")]
    assert row.effective is None
    assert row.state == "effective"
    assert row.value_type == "null"


# ---------------------------------------------------------------------------
# argv chains


def test_argv_chains_match_the_expected_file(ws: Workspace):
    want = expected("argv_chains")
    rows = rows_of(argv_walk(), ws)
    for entry in want["parameters"]:
        key = (want["owner"], entry["name"])
        if key in KNOWN_DEVIATIONS:
            continue
        assert key in rows, key
        _check_row(rows[key], entry)


def test_an_argv_assignment_keeps_the_text_and_the_typed_value(ws: Workspace):
    rows = rows_of(argv_walk(), ws)
    row = rows[("/prep/cropper", "decimate_rings")]
    assert row.effective == 2
    assert row.value_type == "int"
    assert row.candidates[0].raw == "2"
    assert row.candidates[0].mechanism == model.Mechanism.ROS_ARGS_P.value


def test_a_bare_node_key_does_not_configure_a_namespaced_node(ws: Workspace):
    """Verified against humble: `cropper:` does not reach /prep/cropper.

    This is the one place the expected file is not reproduced. crop.yaml keys on `cropper`
    and the node runs at /prep/cropper, so the file contributes nothing: the parameter the
    file alone would set does not exist, and the key becomes a mismatched block instead.
    """
    rows = rows_of(argv_walk(), ws)
    assert not ids.param_key_matches("cropper", "/prep/cropper")
    assert ids.param_key_matches("/prep/cropper", "/prep/cropper")
    assert ("/prep/cropper", "input_topic") not in rows
    orphan = rows[("cropper", "input_topic")]
    assert orphan.state == model.ParamState.OWNER_NAME_MISMATCH.value
    rate = rows[("/prep/cropper", "rate_hz")]
    assert len(rate.candidates) == 1
    assert rate.effective == 20.0
    assert rate.candidates[0].mechanism == model.Mechanism.ROS_ARGS_P.value


def test_the_same_file_reaches_the_node_in_the_root_namespace(ws: Workspace):
    walk = records.LaunchWalk(
        entry_path=str(WS / "src" / "lx_bringup" / "launch" / "exec_process.launch.py"),
        profile="argv_zoo",
        nodes=[
            node(
                pkg="lx_bringup",
                executable="crop_cloud.py",
                form="process",
                name="cropper",
                name_source="argv",
                namespace="/",
                params=(records.ParamSource(order=0, kind="ros_args_file", path=cfg("crop.yaml")),),
            )
        ],
    )
    rows = rows_of(walk, ws)
    row = rows[("/cropper", "rate_hz")]
    assert row.effective == 5.0
    assert row.candidates[0].mechanism == model.Mechanism.ROS_ARGS_PARAMS_FILE.value


# ---------------------------------------------------------------------------
# order, wildcards and one file rules


def test_the_parameters_list_order_decides_the_winner(ws: Workspace):
    order_file = cfg("order_base.yaml")
    entry = str(WS / "src" / "lx_bringup" / "launch" / "param_order.launch.py")
    walk = records.LaunchWalk(
        entry_path=entry,
        profile="order_zoo",
        nodes=[
            node(
                pkg="lx_bringup",
                executable="lx_controller",
                name="inline_wins",
                params=(
                    repo_file(0, order_file),
                    inline(1, {"width": 40}, "Node(inline_wins).parameters[1]", entry),
                ),
            ),
            node(
                pkg="lx_bringup",
                executable="lx_controller",
                name="file_wins",
                params=(
                    inline(0, {"width": 40}, "Node(file_wins).parameters[0]", entry),
                    repo_file(1, order_file),
                ),
            ),
        ],
    )
    rows = rows_of(walk, ws)
    assert rows[("/inline_wins", "width")].effective == 40
    assert rows[("/inline_wins", "width")].candidates[0].mechanism == "launch_inline_dict"
    assert rows[("/file_wins", "width")].effective == 20
    assert rows[("/file_wins", "width")].candidates[0].mechanism == "launch_params_file"
    assert rows[("/file_wins", "width")].candidates[1].shadow_kind == "value_changed"


def test_inside_one_file_the_last_matching_key_wins(ws: Workspace, tmp_path: Path):
    """Verified against humble by running a node against both key orders."""
    target = tmp_path / "both.yaml"
    target.write_text(
        yaml.safe_dump(
            {
                "/relay": {"ros__parameters": {"topic": "from_exact"}},
                "/**": {"ros__parameters": {"topic": "from_wildcard"}},
            },
            sort_keys=False,
        )
    )
    document = yaml_lines.load_file(str(target))
    blocks = yaml_lines.param_blocks(document)
    assert [block.selector for block in blocks] == ["/relay", "/**"]
    assert [block.order for block in blocks] == [0, 1]


def scratch_ws(tmp_path: Path, name: str, text: str) -> tuple[Workspace, tuple, str]:
    pkg = tmp_path / "src" / "lx_odd"
    (pkg / "config").mkdir(parents=True, exist_ok=True)
    (pkg / "package.xml").write_text(
        "<package format='3'><name>lx_odd</name>"
        "<export><build_type>ament_cmake</build_type></export></package>\n"
    )
    target = pkg / "config" / name
    target.write_text(text)
    roots = (RootSpec(kind="src", path=str(tmp_path / "src")),)
    return Workspace.discover(list(roots)), roots, str(target)


def test_a_verified_third_party_name_owns_the_block_the_executable_does_not(tmp_path: Path):
    """nav2_map_server/map_saver_server runs as map_saver, per known_names.

    Reading the executable as the node name puts the parameters on a node that does not
    exist and turns the block that does configure it into a mismatch.
    """
    odd_ws, roots, target = scratch_ws(
        tmp_path,
        "saver.yaml",
        "map_saver:\n  ros__parameters:\n    save_map_timeout: 5.0\n",
    )
    saver = records.ResolvedNode(
        pkg="nav2_map_server",
        executable="map_saver_server",
        namespace="/",
        params=(records.ParamSource(order=0, kind="repo_file", path=target),),
    )
    assert params_collect.owner_name(saver) == "/map_saver"
    assert params_collect.owner_node_id(saver) == "node:/map_saver"
    walk = records.LaunchWalk(
        entry_path=str(tmp_path / "src" / "lx_odd" / "launch" / "saver.launch.py"),
        profile="saver_zoo",
        nodes=[saver],
    )
    rows = {row.name: row for row in params_collect.chain_for(walk, saver, ws=odd_ws)}
    assert rows["save_map_timeout"].effective == 5.0
    assert params_collect.orphan_rows(walk, ws=odd_ws) == []
    builder = model.AtlasBuilder()
    params_collect.ParamsCollector().collect(
        CollectContext(
            ws=odd_ws,
            cfg=Config(roots=roots, profiles=()),
            builder=builder,
            declared=records.DeclaredBundle(walks=[walk]),
        )
    )
    atlas = builder.build()
    assert atlas.get("param:/map_saver|save_map_timeout") is not None
    assert atlas.get("node:/map_saver").name_source == "executable_default"


def test_a_block_key_that_cannot_be_a_ros_name_costs_only_that_block(tmp_path: Path):
    """One unusable yaml key must not empty the node's chain or abort the collector."""
    pkg = tmp_path / "src" / "lx_odd"
    (pkg / "config").mkdir(parents=True)
    (pkg / "package.xml").write_text(
        "<package format='3'><name>lx_odd</name>"
        "<export><build_type>ament_cmake</build_type></export></package>\n"
    )
    target = pkg / "config" / "odd.yaml"
    target.write_text(
        "my-node:\n  ros__parameters:\n    depth: 1\n/relay:\n  ros__parameters:\n    depth: 2\n"
    )
    roots = (RootSpec(kind="src", path=str(tmp_path / "src")),)
    odd_ws = Workspace.discover(list(roots))
    walk = records.LaunchWalk(
        entry_path=str(pkg / "launch" / "odd.launch.py"),
        profile="odd_zoo",
        nodes=[
            node(
                pkg="lx_odd",
                executable="lx_relay",
                name="relay",
                params=(records.ParamSource(order=0, kind="repo_file", path=str(target)),),
            )
        ],
    )
    rows = {row.name: row for row in params_collect.chain_for(walk, walk.nodes[0], ws=odd_ws)}
    assert rows["depth"].effective == 2
    orphans = params_collect.orphan_rows(walk, ws=odd_ws)
    assert [(row.selector_key, row.param_id) for row in orphans] == [("my-node", None)]
    builder = model.AtlasBuilder()
    params_collect.ParamsCollector().collect(
        CollectContext(
            ws=odd_ws,
            cfg=Config(roots=roots, profiles=()),
            builder=builder,
            declared=records.DeclaredBundle(walks=[walk]),
        )
    )
    atlas = builder.build()
    assert atlas.get("param:/relay|depth") is not None


def test_one_orphan_block_key_in_two_files_cites_the_block_key_of_each(tmp_path: Path):
    """A key spelled under one orphan block name in a base file and again in an overlay.

    The fact carries the block key a reader deleting the block starts at, and one site per
    parameter names only the block of whichever file the collector read last. Both files hold
    a block, a reader has to delete both, and each one has to be cited at its own first line
    rather than two lines into it.
    """
    pkg = tmp_path / "src" / "lx_odd"
    (pkg / "config").mkdir(parents=True)
    (pkg / "package.xml").write_text(
        "<package format='3'><name>lx_odd</name>"
        "<export><build_type>ament_cmake</build_type></export></package>\n"
    )
    base = pkg / "config" / "base.yaml"
    base.write_text(
        "relay:\n"
        "  ros__parameters:\n"
        "    depth: 1\n"
        "\n"
        "velocity_smoother:\n"
        "  ros__parameters:\n"
        "    max_speed: 1.0\n"
    )
    overlay = pkg / "config" / "overlay.yaml"
    overlay.write_text(
        "# Overlay half of the pair.\nvelocity_smoother:\n  ros__parameters:\n    max_speed: 2.0\n"
    )
    roots = (RootSpec(kind="src", path=str(tmp_path / "src")),)
    odd_ws = Workspace.discover(list(roots))
    walk = records.LaunchWalk(
        entry_path=str(pkg / "launch" / "odd.launch.py"),
        profile="odd_zoo",
        nodes=[
            node(
                pkg="lx_odd",
                executable="lx_relay",
                name="relay",
                params=(
                    records.ParamSource(order=0, kind="repo_file", path=str(base)),
                    records.ParamSource(order=1, kind="repo_file", path=str(overlay)),
                ),
            )
        ],
    )
    sites = {
        row.block_site.path: row.block_site.line_hint
        for row in params_collect.orphan_rows(walk, ws=odd_ws)
        if row.block_site is not None
    }
    assert sites == {str(base): 5, str(overlay): 2}
    builder = model.AtlasBuilder()
    params_collect.ParamsCollector().collect(
        CollectContext(
            ws=odd_ws,
            cfg=Config(roots=roots, profiles=()),
            builder=builder,
            declared=records.DeclaredBundle(walks=[walk]),
        )
    )
    entity = builder.build().get("param:/velocity_smoother|max_speed")
    assert entity is not None
    facts = [fact for fact in entity.facts if fact.layer == "declared"]
    assert len(facts) == 1
    assert {source.locator for source in facts[0].evidence} == {"velocity_smoother"}
    assert {source.file: source.line_hint for source in facts[0].evidence} == {
        "file:lx_odd|config/base.yaml": 5,
        "file:lx_odd|config/overlay.yaml": 2,
    }


def test_a_process_with_no_usable_name_costs_only_itself(tmp_path: Path):
    """An ExecuteProcess whose executable is a script name emptied the whole profile."""
    pkg = tmp_path / "src" / "lx_script"
    (pkg / "config").mkdir(parents=True)
    (pkg / "package.xml").write_text(
        "<package format='3'><name>lx_script</name>"
        "<export><build_type>ament_cmake</build_type></export></package>\n"
    )
    target = pkg / "config" / "script.yaml"
    target.write_text(
        "relay:\n  ros__parameters:\n    depth: 3\nspare:\n  ros__parameters:\n    a: 1\n"
    )
    roots = (RootSpec(kind="src", path=str(tmp_path / "src")),)
    script_ws = Workspace.discover(list(roots))
    source = records.ParamSource(order=0, kind="repo_file", path=str(target))
    walk = records.LaunchWalk(
        entry_path=str(pkg / "launch" / "script.launch.py"),
        profile="script_zoo",
        nodes=[
            node(pkg="lx_script", executable="crop_scan.py", form="process", params=(source,)),
            node(pkg="lx_script", executable="lx_relay", name="relay", params=(source,)),
        ],
    )
    orphans = params_collect.orphan_rows(walk, ws=script_ws)
    assert [row.state for row in orphans] == ["unresolved_owner"]
    builder = model.AtlasBuilder()
    params_collect.ParamsCollector().collect(
        CollectContext(
            ws=script_ws,
            cfg=Config(roots=roots, profiles=()),
            builder=builder,
            declared=records.DeclaredBundle(walks=[walk]),
        )
    )
    atlas = builder.build()
    assert atlas.get("param:/relay|depth").effective["profile:script_zoo"]["value"].value == 3
    assert atlas.get("param:/spare|a").state == "unresolved_owner"


def test_a_node_whose_condition_is_false_owns_no_parameter(ws: Workspace):
    """nav2's multi robot sim includes `rviz_launch.py` twice with `use_namespace` true, so
    both copies of the `UnlessCondition(use_namespace)` rviz never start. Their inline line
    became a parameter of a node no profile runs, stated as two candidates."""
    entry = str(WS / "src" / "lx_bringup" / "launch" / "nav_stack.launch.py")
    off = node(
        pkg="rviz2",
        executable="rviz2",
        params=(inline(0, {"use_sim_time": False}, "parameters", entry),),
        attrs={"unreachable": True},
    )
    on = node(
        pkg="lx_bringup",
        executable="lx_relay",
        name="relay",
        params=(inline(0, {"use_sim_time": True}, "parameters", entry),),
    )
    walk = records.LaunchWalk(entry_path=entry, profile="gated", nodes=[off, off, on])
    builder = model.AtlasBuilder()
    params_collect.ParamsCollector().collect(
        CollectContext(
            ws=ws,
            cfg=Config(roots=tuple(RootSpec(s.kind, s.path) for s in ws.roots), profiles=()),
            builder=builder,
            declared=records.DeclaredBundle(walks=[walk]),
        )
    )
    atlas = builder.build()
    assert [entity.id for entity in atlas.by_kind("param")] == ["param:/relay|use_sim_time"]
    assert [entity.id for entity in atlas.by_kind("node")] == ["node:/relay"]


def test_a_vanished_temp_parameters_file_is_one_stable_blind_spot(ws: Workspace):
    """A launch file that writes a parameters file under /tmp at evaluation time names a
    random file. The sandbox throws it away, and its name in an unresolved entry gave two
    extractions of one tree two different atlases."""
    entry = str(WS / "src" / "lx_bringup" / "launch" / "nav_stack.launch.py")

    def atlas_for(name: str) -> model.Atlas:
        source = records.ParamSource(
            order=0,
            kind="rewritten_file",
            source_path=f"/tmp/{name}",
            site=records.Site(path=entry, locator="parameters"),
        )
        reader = node(
            pkg="lx_bringup",
            executable="lx_relay",
            name="relay",
            params=(source,),
            site=records.Site(path=entry, locator="Node(relay)", line_hint=4),
        )
        walk = records.LaunchWalk(entry_path=entry, profile="temp", nodes=[reader])
        builder = model.AtlasBuilder()
        params_collect.ParamsCollector().collect(
            CollectContext(
                ws=ws,
                cfg=Config(roots=tuple(RootSpec(s.kind, s.path) for s in ws.roots), profiles=()),
                builder=builder,
                declared=records.DeclaredBundle(walks=[walk]),
            )
        )
        return builder.build()

    first, second = atlas_for("tmpw8hfp171"), atlas_for("tmpatlcxeuf")
    assert first.canonical_bytes() == second.canonical_bytes()
    text = model.dumps(first.to_json())
    assert "tmpw8hfp171" not in text and "/tmp" not in text
    assert [entry.role for entry in first.unresolved] == ["parameters_file"]


def test_two_vanished_temp_parameters_files_of_one_node_stay_two_stable_blind_spots(
    ws: Workspace,
):
    """The two files are told apart by the item each launch file spells, never by position,
    so a group scoped SetParameter in front of them moves neither id."""
    entry = str(WS / "src" / "lx_bringup" / "launch" / "nav_stack.launch.py")
    texts = ("LaunchConfiguration('base_file')", "LaunchConfiguration('tuning_file')")

    def atlas_for(names: tuple[str, str], with_global: bool = False) -> model.Atlas:
        globals_ = (
            (
                records.ParamSource(
                    order=0,
                    kind="inline_dict",
                    values=(("use_sim_time", True),),
                    note="SetParameter, group scoped",
                    site=records.Site(path=entry, locator="SetParameter"),
                ),
            )
            if with_global
            else ()
        )
        sources = tuple(
            records.ParamSource(
                order=len(globals_) + order,
                kind="rewritten_file",
                source_path=f"/tmp/{name}",
                text=f"RewrittenYaml({text})",
                site=records.Site(path=entry, locator="parameters"),
            )
            for order, (name, text) in enumerate(zip(names, texts, strict=True))
        )
        reader = node(
            pkg="lx_bringup",
            executable="lx_relay",
            name="relay",
            params=(*globals_, *sources),
            site=records.Site(path=entry, locator="Node(relay)", line_hint=4),
        )
        walk = records.LaunchWalk(entry_path=entry, profile="temp", nodes=[reader])
        builder = model.AtlasBuilder()
        params_collect.ParamsCollector().collect(
            CollectContext(
                ws=ws,
                cfg=Config(roots=tuple(RootSpec(s.kind, s.path) for s in ws.roots), profiles=()),
                builder=builder,
                declared=records.DeclaredBundle(walks=[walk]),
            )
        )
        return builder.build()

    def blind(atlas: model.Atlas) -> list[str]:
        return sorted(entry.id for entry in atlas.unresolved if entry.role == "parameters_file")

    first = atlas_for(("tmpaaaa1111", "tmpbbbb2222"))
    second = atlas_for(("tmpzzzz9999", "tmpcccc3333"))
    assert first.canonical_bytes() == second.canonical_bytes()
    assert len(blind(first)) == 2
    assert "/tmp" not in model.dumps(first.to_json())
    assert blind(atlas_for(("tmpaaaa1111", "tmpbbbb2222"), with_global=True)) == blind(first)
    assert not [entry for entry in first.unresolved if "item 0" in entry.expr]


def test_a_rewrite_nobody_could_perform_names_no_winner(ws: Workspace):
    """The file value is not the effective one when launch rewrites the key to the unknown."""
    entry = str(WS / "src" / "lx_bringup" / "launch" / "rewritten.launch.py")
    walk = records.LaunchWalk(
        entry_path=entry,
        profile="rewrite_zoo",
        nodes=[
            node(
                pkg="lx_bringup",
                executable="lx_controller",
                name="rewritten_node",
                params=(
                    records.ParamSource(
                        order=0,
                        kind="rewritten_file",
                        source_path=cfg("rewrite_source.yaml"),
                        rewrites=(),
                        note="convert_types, unresolved rewrites: autostart",
                    ),
                ),
            )
        ],
    )
    rows = {row.name: row for row in params_collect.chain_for(walk, walk.nodes[0], ws=ws)}
    assert rows["autostart"].state == "other"
    assert rows["autostart"].effective is None
    assert [blind.reason for blind in rows["autostart"].unresolved] == ["rewrite_unresolved"]
    assert rows["nested.autostart"].state == "other"
    assert rows["controller_frequency"].state == "effective"


def test_a_root_key_nobody_could_perform_names_no_winner(ws: Workspace):
    """An unknown root key moves every block of the file, so no value of it is knowable."""
    entry = str(WS / "src" / "lx_bringup" / "launch" / "rewritten.launch.py")
    walk = records.LaunchWalk(
        entry_path=entry,
        profile="rewrite_zoo",
        nodes=[
            node(
                pkg="lx_bringup",
                executable="lx_controller",
                name="rewritten_node",
                params=(
                    records.ParamSource(
                        order=0,
                        kind="rewritten_file",
                        source_path=cfg("rewrite_source.yaml"),
                        note="convert_types, unresolved root_key",
                    ),
                ),
            )
        ],
    )
    rows = {row.name: row for row in params_collect.chain_for(walk, walk.nodes[0], ws=ws)}
    assert rows
    assert {row.state for row in rows.values()} == {"other"}
    assert {blind.reason for row in rows.values() for blind in row.unresolved} == {
        "root_key_unresolved"
    }


# ---------------------------------------------------------------------------
# RewrittenYaml


class _Rewritten:
    """Same duck type as nav2_common RewrittenYaml, name mangled privates included."""

    def __init__(self, source_file, param_rewrites, root_key="", convert_types=True):
        self._RewrittenYaml__source_file = source_file
        self._RewrittenYaml__param_rewrites = dict(param_rewrites)
        self._RewrittenYaml__root_key = root_key
        self._RewrittenYaml__convert_types = convert_types


class _ParameterFile:
    """Same duck type as launch_ros ParameterFile."""

    def __init__(self, param_file, allow_substs=True):
        self._ParameterFile__param_file = param_file
        self._ParameterFile__allow_substs = allow_substs


def test_describe_terminates_at_the_source_file():
    item = _ParameterFile(
        _Rewritten(cfg("rewrite_source.yaml"), {"autostart": "false", "use_sim_time": "true"})
    )
    source = rewritten_yaml.describe(item)
    assert source is not None
    assert source.kind == "rewritten_file"
    assert source.source_path == cfg("rewrite_source.yaml")
    assert source.rewrites == (("autostart", "false"), ("use_sim_time", "true"))
    assert "convert_types" in source.note
    assert source.path is None


def test_describe_reports_a_plain_file_as_a_repo_file():
    source = rewritten_yaml.describe(_ParameterFile(cfg("zoo_base.yaml")))
    assert source is not None
    assert source.kind == "repo_file"
    assert source.path == cfg("zoo_base.yaml")


def test_describe_gives_up_rather_than_guessing():
    assert rewritten_yaml.describe(_ParameterFile(_Rewritten(object(), {}))) is None


def test_a_rewrite_of_an_absent_key_is_a_silent_no_op():
    document = yaml_lines.load_file(cfg("rewrite_source.yaml"))
    rewrites = (("use_sim_time", "true"),)
    assert rewritten_yaml.rewritten_keys(document.data, rewrites) == []
    assert rewritten_yaml.apply_rewrites(document.data, "", rewrites) == document.data


def test_a_rewrite_lands_at_every_depth():
    document = yaml_lines.load_file(cfg("rewrite_source.yaml"))
    out = rewritten_yaml.apply_rewrites(document.data, "", (("autostart", "false"),))
    params = out["rewritten_node"]["ros__parameters"]
    assert params["autostart"] is False
    assert params["nested"]["autostart"] is False


def test_a_rewritten_bool_is_a_bool_whether_or_not_convert_types_is_set():
    """nav2_common converts true and false outside the convert_types guard."""
    assert rewritten_yaml.convert_value("true", convert_types=False) is True
    assert rewritten_yaml.convert_value("false", convert_types=False) is False
    assert rewritten_yaml.convert_value("true", convert_types=True) is True
    assert rewritten_yaml.convert_value("2", convert_types=False) == "2"
    assert rewritten_yaml.convert_value("2", convert_types=True) == 2


def test_convert_types_changes_the_type_of_a_rewritten_string():
    document = yaml_lines.load_file(cfg("rewrite_source.yaml"))
    out = rewritten_yaml.apply_rewrites(document.data, "", (("anchor_tag", "2.5"),))
    assert out["rewritten_node"]["ros__parameters"]["anchor_tag"] == 2.5
    kept = rewritten_yaml.apply_rewrites(
        document.data, "", (("anchor_tag", "2.5"),), convert_types=False
    )
    assert kept["rewritten_node"]["ros__parameters"]["anchor_tag"] == "2.5"


class _Substitution:
    """Same duck type as a launch substitution: text only through a resolver."""

    def __init__(self, text: str):
        self.text = text


class _Evaluated:
    """A ParameterFile that has already been performed once.

    The public property then answers with the temp path it produced, which is the shape that
    silently replaced the real provenance for every node after the first.
    """

    def __init__(self, param_file, temp: str):
        self._ParameterFile__param_file = param_file
        self._ParameterFile__allow_substs = True
        self.temp = temp

    @property
    def param_file(self):
        return self.temp


def resolver(value):
    if isinstance(value, _Substitution):
        return value.text
    if isinstance(value, (list, tuple)):
        parts = [resolver(item) for item in value]
        return "".join(parts) if all(isinstance(part, str) for part in parts) else None
    return None


def test_unwrap_descends_a_one_element_substitution_list():
    """launch_ros normalises everything that is not a dict into a list of substitutions."""
    inner = _Rewritten([_Substitution(cfg("rewrite_source.yaml"))], {})
    wrapper = rewritten_yaml.unwrap(_ParameterFile([inner]), resolve=resolver)
    assert wrapper.kind == "rewritten_file"
    assert wrapper.wrapped_in_parameter_file
    source = rewritten_yaml.describe(_ParameterFile([inner]), resolve=resolver)
    assert source is not None
    assert source.source_path == cfg("rewrite_source.yaml")


def test_describing_never_reads_the_evaluated_temp_path():
    item = _Evaluated(
        [_Rewritten(cfg("rewrite_source.yaml"), {"autostart": "false"})],
        temp="/tmp/launch_params_9x8",
    )
    source = rewritten_yaml.describe(item, resolve=resolver)
    assert source is not None
    assert source.source_path == cfg("rewrite_source.yaml")
    assert "/tmp/" not in str(source.source_path)


def test_a_rewritten_yaml_is_never_performed_to_find_its_path():
    """Performing one writes a new temp yaml, which is the provenance being described."""

    def refuse(_value):
        raise AssertionError("describing must not perform a RewrittenYaml")

    inner = _Rewritten(cfg("rewrite_source.yaml"), {"autostart": "false"})
    assert rewritten_yaml.describe(_ParameterFile([inner]), resolve=refuse) is not None


def test_an_empty_root_key_is_not_an_absent_one():
    absent = rewritten_yaml.unwrap(_Rewritten(cfg("rewrite_source.yaml"), {}, root_key=None))
    empty = rewritten_yaml.unwrap(
        _Rewritten(cfg("rewrite_source.yaml"), {}, root_key=[_Substitution("")]),
        resolve=resolver,
    )
    assert absent.root_key is None
    assert empty.root_key == ""
    plain = _Rewritten(cfg("rewrite_source.yaml"), {}, root_key=None)
    assert rewritten_yaml.describe(plain).root_key is None


def test_a_rewrite_value_that_is_a_substitution_resolves_through_the_callback():
    item = _Rewritten(
        cfg("rewrite_source.yaml"),
        {"autostart": [_Substitution("false")], "use_sim_time": _Substitution("true")},
    )
    source = rewritten_yaml.describe(item, resolve=resolver)
    assert source.rewrites == (("autostart", "false"), ("use_sim_time", "true"))
    assert "unresolved rewrites" not in source.note
    blind = rewritten_yaml.describe(item)
    assert blind.rewrites == ()
    assert "unresolved rewrites: autostart, use_sim_time" in blind.note


def test_a_root_key_reshapes_every_node_key(ws: Workspace):
    entry = str(WS / "src" / "lx_bringup" / "launch" / "rewritten.launch.py")
    source = records.ParamSource(
        order=0,
        kind="rewritten_file",
        source_path=cfg("rewrite_source.yaml"),
        root_key="robot1",
        rewrites=(("autostart", "false"),),
        note="convert_types",
    )
    walk = records.LaunchWalk(
        entry_path=entry,
        profile="rewritten_zoo",
        nodes=[
            node(
                pkg="lx_bringup",
                executable="lx_controller",
                name="rewritten_node",
                namespace="robot1",
                params=(source,),
            )
        ],
    )
    rows = rows_of(walk, ws)
    row = rows[("/robot1/rewritten_node", "autostart")]
    assert row.effective is False
    assert row.candidates[0].mechanism == model.Mechanism.LAUNCH_REWRITTEN_YAML.value
    assert relative(row.candidates[0].site.path) == "src/lx_bringup/config/rewrite_source.yaml"
    assert row.candidates[0].site.locator == "rewritten_node.ros__parameters.autostart"
    assert row.candidates[0].site.line_hint == 6


def rewritten_walk(root_key: str) -> records.LaunchWalk:
    entry = str(WS / "src" / "lx_bringup" / "launch" / "rewritten.launch.py")
    return records.LaunchWalk(
        entry_path=entry,
        profile="rewritten_zoo",
        nodes=[
            node(
                pkg="lx_bringup",
                executable="lx_controller",
                name="rewritten_node",
                namespace=root_key or "/",
                params=(
                    records.ParamSource(
                        order=0,
                        kind="rewritten_file",
                        source_path=cfg("rewrite_source.yaml"),
                        root_key=root_key or None,
                        rewrites=(("autostart", "false"),),
                        note="convert_types",
                    ),
                ),
            )
        ],
    )


def test_the_orphan_pass_reads_the_reshaped_document(ws: Workspace):
    """A root key wraps every node key, so the orphan pass has to wrap them too.

    Reading the file as it sits on disk makes the one block that does configure the node
    look like a block that configures nothing, and mints a second parameter id for it.
    """
    walk = rewritten_walk("robot1")
    assert params_collect.orphan_rows(walk, ws=ws) == []
    atlas = build(ws, [walk])
    assert atlas.get("param:/robot1/rewritten_node|autostart") is not None
    assert atlas.get("param:/rewritten_node|autostart") is None


# ---------------------------------------------------------------------------
# a params file written outside the tracked trees


def merged_params_file(target: Path) -> str:
    """What tools/launcher.py does: merge the pair, write a temp file, keep no record."""
    base = yaml.safe_load(Path(cfg("zoo_base.yaml")).read_text())
    overlay = yaml.safe_load(Path(cfg("zoo_overlay.yaml")).read_text())
    from rosio.resolve import merge

    merged = merge.deep_merge(base, overlay)
    merged["controller_server"]["ros__parameters"]["FollowPath"]["desired_linear_vel"] = 1.0
    target.write_text(yaml.safe_dump(merged, sort_keys=True))
    return str(target)


def external_walk(temp_params: str) -> records.LaunchWalk:
    writer = ids.build_sym(
        ids.REPO_PKG,
        "tests/fixtures/ws_launch/tools/launcher.py",
        "merged_params",
    )
    return records.LaunchWalk(
        entry_path=str(WS / "src" / "lx_bringup" / "launch" / "nav_stack.launch.py"),
        profile="navigation_zoo",
        nodes=[
            node(
                pkg="lx_bringup",
                executable="lx_controller",
                name="controller_server",
                params=(
                    records.ParamSource(
                        order=0,
                        kind="external_file",
                        path=temp_params,
                        produced_by=writer,
                        note="mechanism=python_override",
                    ),
                ),
            )
        ],
    )


def test_an_external_params_file_keeps_its_values_and_loses_its_path(ws: Workspace, tmp_path: Path):
    temp = merged_params_file(tmp_path / "merged_navigation_zoo.yaml")
    rows = rows_of(external_walk(temp), ws)
    row = rows[("/controller_server", "FollowPath.desired_linear_vel")]
    assert row.effective == 1.0
    candidate = row.candidates[0]
    assert candidate.mechanism == model.Mechanism.PYTHON_OVERRIDE.value
    assert relative(candidate.site.path) == "tools/launcher.py"
    assert candidate.applied_by is not None
    assert ids.parse(candidate.applied_by).kind == "sym"
    for other in rows.values():
        for row_candidate in other.candidates:
            assert str(tmp_path) not in row_candidate.site.path


def test_a_rewritten_document_outside_every_root_loses_its_path(ws: Workspace, tmp_path: Path):
    """A launch file that wraps a temp document in a RewrittenYaml still cites no temp path."""
    temp = merged_params_file(tmp_path / "merged_navigation_zoo.yaml")
    entry = str(WS / "src" / "lx_bringup" / "launch" / "nav_stack.launch.py")
    walk = records.LaunchWalk(
        entry_path=entry,
        profile="navigation_zoo",
        nodes=[
            node(
                pkg="lx_bringup",
                executable="lx_controller",
                name="controller_server",
                params=(
                    records.ParamSource(
                        order=0,
                        kind="rewritten_file",
                        source_path=temp,
                        rewrites=(("desired_linear_vel", "2.0"),),
                        note="convert_types",
                    ),
                ),
            )
        ],
    )
    rows = rows_of(walk, ws)
    row = rows[("/controller_server", "FollowPath.desired_linear_vel")]
    assert row.effective == 2.0
    for other in rows.values():
        for candidate in other.candidates:
            assert str(tmp_path) not in candidate.site.path
            assert candidate.site.path == entry
    builder = model.AtlasBuilder()
    params_collect.ParamsCollector().collect(
        CollectContext(
            ws=ws,
            cfg=Config(
                roots=(RootSpec(kind="src", path=str(WS / "src")),),
                profiles=(),
            ),
            builder=builder,
            declared=records.DeclaredBundle(walks=[walk]),
        )
    )
    assert builder.build().get("param:/controller_server|FollowPath.desired_linear_vel") is not None


# ---------------------------------------------------------------------------
# child nodes, from the committed table


def costmap_owner(params: tuple) -> records.ResolvedNode:
    """controller_server as the reference stack launches it: no name, one params file."""
    return records.ResolvedNode(
        pkg="nav2_controller",
        executable="controller_server",
        namespace="/",
        name_source="executable_default",
        params=params,
    )


def costmap_walk(params: tuple) -> records.LaunchWalk:
    return records.LaunchWalk(
        entry_path=str(WS / "src" / "lx_bringup" / "launch" / "nav_stack.launch.py"),
        profile="navigation_zoo",
        nodes=[costmap_owner(params)],
    )


def test_the_child_node_table_names_the_costmap(ws: Workspace):
    children = params_collect.child_nodes(costmap_owner((repo_file(0, cfg("zoo_base.yaml")),)))
    assert [params_collect.owner_name(child) for child in children] == [
        "/local_costmap/local_costmap"
    ]
    assert children[0].name_source == "executable_default"


def test_the_two_level_block_configures_the_child_and_not_nobody(ws: Workspace):
    """116 parameters on the reference stack hang on this: the block has a real owner."""
    walk = costmap_walk((repo_file(0, cfg("zoo_base.yaml")),))
    child = params_collect.child_nodes(walk.nodes[0])[0]
    rows = {row.name: row for row in params_collect.chain_for(walk, child, ws=ws)}
    width = rows["width"]
    assert width.state == "effective"
    assert width.effective == 20
    assert type(width.effective) is int
    assert width.owner_fqn == "/local_costmap/local_costmap"
    assert width.param_id == "param:/local_costmap/local_costmap|width"
    orphans = {row.selector_key for row in params_collect.orphan_rows(walk, ws=ws)}
    assert "local_costmap/local_costmap" not in orphans


def test_an_inline_dict_reaches_the_child_only_when_launch_stated_no_name(ws: Workspace):
    """launch keys an inline dict on the parent's fully specified name, else on `/**`."""
    entry = str(WS / "src" / "lx_bringup" / "launch" / "nav_stack.launch.py")
    dict_source = inline(1, {"use_sim_time": True}, "Node(controller_server).parameters[1]", entry)
    unnamed = costmap_owner((repo_file(0, cfg("zoo_base.yaml")), dict_source))
    named = records.ResolvedNode(
        pkg="nav2_controller",
        executable="controller_server",
        namespace="/",
        name="controller_server",
        name_source="literal",
        params=(repo_file(0, cfg("zoo_base.yaml")), dict_source),
    )
    assert [source.kind for source in params_collect.child_nodes(unnamed)[0].params] == [
        "repo_file",
        "inline_dict",
    ]
    assert [source.kind for source in params_collect.child_nodes(named)[0].params] == ["repo_file"]


# ---------------------------------------------------------------------------
# a parameters file assembled outside launch


def nav_pipeline(**kwargs) -> records.ParamPipeline:
    """What the analyser recovers from a launcher that merges a pair and writes two keys."""
    site = records.Site(
        path=str(WS / "tools" / "launcher.py"), locator="merged_params", line_hint=41
    )
    fields = {
        "output_token": "params_file",
        "base": cfg("zoo_base.yaml"),
        "overlays": (cfg("zoo_overlay.yaml"),),
        "overrides": (
            records.ParamOverride(
                key_path=("controller_server", "ros__parameters", "FollowPath", "state_timeout"),
                value=0.9,
                expr="float(STATE_TIMEOUT)",
                site=records.Site(
                    path=str(WS / "tools" / "launcher.py"),
                    locator="merged_params",
                    line_hint=44,
                ),
            ),
        ),
        "applied_by": "launcher.merged_params",
        "site": site,
    }
    fields.update(kwargs)
    return records.ParamPipeline(**fields)


def pipeline_walk(source: records.ParamSource) -> records.LaunchWalk:
    """The entry point the profile names, with the base file standing in for the temp path."""
    return records.LaunchWalk(
        entry_path=str(WS / "src" / "lx_bringup" / "launch" / "nav_stack.launch.py"),
        profile="navigation_zoo",
        args=(("params_file", cfg("zoo_base.yaml")),),
        nodes=[costmap_owner((source,))],
    )


def test_the_pipeline_puts_the_overlay_and_the_python_write_in_the_chain(ws: Workspace):
    walk = pipeline_walk(repo_file(0, cfg("zoo_base.yaml")))
    pipelines = [nav_pipeline()]
    rows = {
        row.name: row
        for row in params_collect.chain_for(walk, walk.nodes[0], ws=ws, pipelines=pipelines)
    }
    plugin = rows["FollowPath.plugin"]
    assert plugin.effective == "lx_bringup/OrbitFollower"
    assert [candidate.mechanism for candidate in plugin.candidates] == [
        model.Mechanism.YAML_OVERLAY_MERGE.value,
        model.Mechanism.YAML_BASE.value,
    ]
    assert relative(plugin.candidates[0].site.path) == "src/lx_bringup/config/zoo_overlay.yaml"
    assert plugin.candidates[0].site.line_hint == 10
    assert plugin.candidates[1].site.line_hint == 12
    assert plugin.candidates[1].shadow_kind == "value_changed"
    for candidate in plugin.candidates:
        assert ids.parse(candidate.applied_by).kind == "sym"

    written = rows["FollowPath.state_timeout"]
    assert written.effective == 0.9
    assert written.candidates[0].mechanism == model.Mechanism.PYTHON_OVERRIDE.value
    assert relative(written.candidates[0].site.path) == "tools/launcher.py"
    assert written.candidates[0].site.line_hint == 44
    assert written.candidates[1].value == 0.4
    assert written.candidates[1].shadow_kind == "value_changed"


def test_the_child_node_gets_the_overlay_value_too(ws: Workspace):
    walk = pipeline_walk(repo_file(0, cfg("zoo_base.yaml")))
    child = params_collect.child_nodes(walk.nodes[0])[0]
    rows = {
        row.name: row
        for row in params_collect.chain_for(walk, child, ws=ws, pipelines=[nav_pipeline()])
    }
    assert rows["width"].effective == 40
    assert type(rows["width"].effective) is int
    assert rows["width"].candidates[0].mechanism == model.Mechanism.YAML_OVERLAY_MERGE.value
    assert rows["width"].candidates[1].value == 20


def test_provenance_never_points_at_the_temp_file(ws: Workspace, tmp_path: Path):
    temp = str(tmp_path / "tmp_navigation_zoo.yaml")
    Path(temp).write_text("controller_server:\n  ros__parameters:\n    controller_frequency: 1.0\n")
    source = records.ParamSource(order=0, kind="external_file", path=temp)
    walk = pipeline_walk(source)
    pipelines = [nav_pipeline(output_token=temp)]
    rows = params_collect.chain_for(walk, walk.nodes[0], ws=ws, pipelines=pipelines)
    assert rows
    for row in rows:
        for candidate in row.candidates:
            assert str(tmp_path) not in candidate.site.path


def test_a_rewritten_pipeline_keeps_its_root_key_and_its_rewrites(ws: Workspace):
    source = records.ParamSource(
        order=0,
        kind="rewritten_file",
        source_path=cfg("zoo_base.yaml"),
        root_key="",
        rewrites=(("controller_frequency", "5.0"),),
        note="convert_types",
        site=records.Site(
            path=str(WS / "src" / "lx_bringup" / "launch" / "nav_stack.launch.py"),
            locator="parameters",
        ),
    )
    walk = pipeline_walk(source)
    rows = {
        row.name: row
        for row in params_collect.chain_for(walk, walk.nodes[0], ws=ws, pipelines=[nav_pipeline()])
    }
    assert rows["controller_frequency"].effective == 5.0
    assert rows["FollowPath.plugin"].effective == "lx_bringup/OrbitFollower"
    assert rows["FollowPath.plugin"].candidates[0].mechanism == (
        model.Mechanism.YAML_OVERLAY_MERGE.value
    )


def test_a_rewrite_of_a_written_key_wins_and_the_write_stays_visible(ws: Workspace):
    source = records.ParamSource(
        order=0,
        kind="rewritten_file",
        source_path=cfg("zoo_base.yaml"),
        rewrites=(("state_timeout", "1.5"),),
        note="convert_types",
        site=records.Site(
            path=str(WS / "src" / "lx_bringup" / "launch" / "nav_stack.launch.py"),
            locator="parameters",
        ),
    )
    walk = pipeline_walk(source)
    rows = {
        row.name: row
        for row in params_collect.chain_for(walk, walk.nodes[0], ws=ws, pipelines=[nav_pipeline()])
    }
    row = rows["FollowPath.state_timeout"]
    assert row.effective == 1.5
    assert row.candidates[0].mechanism == model.Mechanism.LAUNCH_REWRITTEN_YAML.value
    assert row.candidates[1].mechanism == model.Mechanism.PYTHON_OVERRIDE.value
    assert row.candidates[1].value == 0.9
    assert row.candidates[1].wins is False


def test_a_file_the_argument_does_not_name_is_not_fed_by_the_pipeline(ws: Workspace):
    walk = records.LaunchWalk(
        entry_path=str(WS / "src" / "lx_bringup" / "launch" / "nav_stack.launch.py"),
        profile="navigation_zoo",
        nodes=[costmap_owner((repo_file(0, cfg("zoo_base.yaml")),))],
    )
    rows = {
        row.name: row
        for row in params_collect.chain_for(walk, walk.nodes[0], ws=ws, pipelines=[nav_pipeline()])
    }
    assert rows["FollowPath.plugin"].effective == "lx_bringup/PathFollower"
    assert len(rows["FollowPath.plugin"].candidates) == 1


def test_an_incomplete_pipeline_names_no_winner(ws: Workspace):
    walk = pipeline_walk(repo_file(0, cfg("zoo_base.yaml")))
    pipelines = [nav_pipeline(complete=False, notes=("the document is never dumped",))]
    rows = {
        row.name: row
        for row in params_collect.chain_for(walk, walk.nodes[0], ws=ws, pipelines=pipelines)
    }
    row = rows["FollowPath.plugin"]
    assert row.state == model.ParamState.OTHER.value
    assert row.effective is None
    assert len(row.candidates) == 2
    assert not any(candidate.wins for candidate in row.candidates)
    assert row.candidates[1].shadow_kind == "value_changed"
    assert [blind.reason for blind in row.unresolved] == ["pipeline_not_followed"]
    atlas = build(ws, [walk], pipelines=pipelines)
    entity = atlas.get("param:/controller_server|FollowPath.plugin")
    assert entity.state == model.ParamState.OTHER.value
    assert entity.effective == {}
    assert any(
        entry.owner == entity.id and entry.want == "param_value" for entry in atlas.unresolved
    )


def test_two_pipelines_on_one_argument_name_no_winner(ws: Workspace):
    """Two launcher functions build one argument name from one base for two entry points."""
    walk = pipeline_walk(repo_file(0, cfg("zoo_base.yaml")))
    pipelines = [nav_pipeline(), nav_pipeline(applied_by="launcher.legacy_params")]
    rows = {
        row.name: row
        for row in params_collect.chain_for(walk, walk.nodes[0], ws=ws, pipelines=pipelines)
    }
    row = rows["FollowPath.plugin"]
    assert row.state == model.ParamState.OTHER.value
    assert row.effective is None
    assert [blind.reason for blind in row.unresolved] == ["pipeline_ambiguous"]
    assert row.candidates[0].value == "lx_bringup/PathFollower"


def test_the_bound_entry_point_tells_two_pipelines_apart(ws: Workspace):
    """Two launcher functions, one argument name, one base document, two commands.

    The command is the only thing that differs, so it is what binds a pipeline to a walk.
    Without it both feed both entry points and every parameter they touch goes blind.
    """
    walk = pipeline_walk(repo_file(0, cfg("zoo_base.yaml")))
    pipelines = [
        nav_pipeline(notes=(f"{py_pipeline.NOTE_SINK_TARGET}lx_bringup nav_stack.launch.py",)),
        nav_pipeline(
            applied_by="launcher.legacy_params",
            notes=(f"{py_pipeline.NOTE_SINK_TARGET}lx_bringup param_order.launch.py",),
        ),
    ]
    rows = {
        row.name: row
        for row in params_collect.chain_for(walk, walk.nodes[0], ws=ws, pipelines=pipelines)
    }
    assert rows["FollowPath.plugin"].effective == "lx_bringup/OrbitFollower"
    assert rows["FollowPath.plugin"].unresolved == ()
    pipelines = [
        nav_pipeline(
            notes=(f"{entrypoints.NOTE_ENTRYPOINT}ros2 launch lx_bringup nav_stack.launch.py",)
        ),
        nav_pipeline(
            applied_by="launcher.legacy_params",
            notes=(f"{entrypoints.NOTE_ENTRYPOINT}ros2 launch lx_bringup param_order.launch.py",),
        ),
    ]
    rows = {
        row.name: row
        for row in params_collect.chain_for(walk, walk.nodes[0], ws=ws, pipelines=pipelines)
    }
    assert rows["FollowPath.plugin"].effective == "lx_bringup/OrbitFollower"
    assert rows["FollowPath.plugin"].unresolved == ()


def test_an_argument_named_by_the_output_token_feeds_the_pipeline(ws: Workspace, tmp_path: Path):
    """The launcher writes a temp file and spells it into `params_file:=<temp>`."""
    temp = str(tmp_path / "tmp_navigation_zoo.yaml")
    Path(temp).write_text("controller_server:\n  ros__parameters:\n    controller_frequency: 1.0\n")
    walk = records.LaunchWalk(
        entry_path=str(WS / "src" / "lx_bringup" / "launch" / "nav_stack.launch.py"),
        profile="navigation_zoo",
        args=(("params_file", temp),),
        nodes=[costmap_owner((records.ParamSource(order=0, kind="external_file", path=temp),))],
    )
    rows = {
        row.name: row
        for row in params_collect.chain_for(walk, walk.nodes[0], ws=ws, pipelines=[nav_pipeline()])
    }
    assert rows["FollowPath.plugin"].effective == "lx_bringup/OrbitFollower"
    assert all(str(tmp_path) not in row.candidates[0].site.path for row in rows.values())


def test_an_unresolved_python_write_marks_only_its_own_parameter(ws: Workspace):
    override = records.ParamOverride(
        key_path=("controller_server", "ros__parameters", "FollowPath", "desired_linear_vel"),
        resolved=False,
        expr="self._limit()",
        site=records.Site(path=str(WS / "tools" / "launcher.py"), locator="merged_params"),
    )
    walk = pipeline_walk(repo_file(0, cfg("zoo_base.yaml")))
    pipelines = [nav_pipeline(overrides=(override,))]
    rows = {
        row.name: row
        for row in params_collect.chain_for(walk, walk.nodes[0], ws=ws, pipelines=pipelines)
    }
    blind = rows["FollowPath.desired_linear_vel"]
    assert blind.state == model.ParamState.OTHER.value
    assert blind.effective is None
    assert blind.candidates[0].value == 1.2
    assert not any(candidate.wins for candidate in blind.candidates)
    assert [item.expr for item in blind.unresolved] == ["self._limit()"]
    assert rows["FollowPath.plugin"].state == "effective"


def test_the_collector_reads_the_pipelines_from_the_bundle(ws: Workspace):
    walk = pipeline_walk(repo_file(0, cfg("zoo_base.yaml")))
    atlas = build(ws, [walk], pipelines=[nav_pipeline()])
    from rosio import schema

    assert schema.validate(atlas.to_json()) == []
    width = atlas.get("param:/local_costmap/local_costmap|width")
    assert width is not None
    assert width.state == "effective"
    profile = ids.build_profile("navigation_zoo")
    assert width.effective[profile]["value"].value == 40
    assert atlas.get("node:/local_costmap/local_costmap").name_source == "executable_default"
    written = atlas.get("param:/controller_server|FollowPath.state_timeout")
    winner = next(candidate for candidate in written.candidates if candidate.wins)
    assert winner.mechanism == model.Mechanism.PYTHON_OVERRIDE.value
    assert winner.source.file == "file:@repo|tests/fixtures/ws_launch/tools/launcher.py"
    assert ids.parse(winner.applied_by).kind == "sym"


def launch_pipeline(**kwargs) -> records.ParamPipeline:
    """What the analyser recovers from a launch file that merges its own document.

    There is no command string here: the launch file hands the temp path to an include of
    itself, so the sink target is the file and the temp suffix is what names the document.
    """
    entry = str(WS / "src" / "lx_bringup" / "launch" / "nav_stack.launch.py")
    fields = {
        "output_token": "_nav_setup:params_file",
        "base": cfg("zoo_base.yaml"),
        "overlays": (cfg("zoo_overlay.yaml"),),
        "applied_by": "nav_stack._nav_setup",
        "site": records.Site(path=entry, locator="_nav_setup:params", line_hint=30),
        "notes": (
            f"{py_pipeline.NOTE_LAUNCH_ARG}params_file",
            f"{py_pipeline.NOTE_SINK_TARGET}nav_stack.launch.py",
            f"{py_pipeline.NOTE_TEMP_SUFFIX}_nav_zoo.yaml",
        ),
    }
    fields.update(kwargs)
    return records.ParamPipeline(**fields)


def launch_pipeline_walk(temp: str) -> records.LaunchWalk:
    """The walk of that launch file, holding the temp path the merge produced."""
    return records.LaunchWalk(
        entry_path=str(WS / "src" / "lx_bringup" / "launch" / "nav_stack.launch.py"),
        profile="navigation_zoo",
        args=(),
        nodes=[costmap_owner((records.ParamSource(order=0, kind="external_file", path=temp),))],
    )


def test_a_merge_inside_a_launch_file_is_decomposed_like_a_launcher_one(
    ws: Workspace, tmp_path: Path
):
    temp = str(tmp_path / "abc123_nav_zoo.yaml")
    Path(temp).write_text("controller_server:\n  ros__parameters:\n    controller_frequency: 1.0\n")
    walk = launch_pipeline_walk(temp)
    pipelines = [launch_pipeline()]
    rows = {
        row.name: row
        for row in params_collect.chain_for(walk, walk.nodes[0], ws=ws, pipelines=pipelines)
    }
    plugin = rows["FollowPath.plugin"]
    assert plugin.effective == "lx_bringup/OrbitFollower"
    assert [candidate.mechanism for candidate in plugin.candidates] == [
        model.Mechanism.YAML_OVERLAY_MERGE.value,
        model.Mechanism.YAML_BASE.value,
    ]
    assert relative(plugin.candidates[0].site.path) == "src/lx_bringup/config/zoo_overlay.yaml"
    assert plugin.candidates[0].site.line_hint == 10
    assert plugin.candidates[1].site.line_hint == 12
    for row in rows.values():
        for candidate in row.candidates:
            assert str(tmp_path) not in candidate.site.path


def test_the_temp_suffix_tells_two_documents_of_one_launch_file_apart(
    ws: Workspace, tmp_path: Path
):
    """One launch file assembles a nav document and a slam document. Neither may take the
    other's provenance, and the random temp name is the only thing that arrives at launch."""
    temp = str(tmp_path / "abc123_slam_zoo.yaml")
    Path(temp).write_text("controller_server:\n  ros__parameters:\n    controller_frequency: 1.0\n")
    walk = launch_pipeline_walk(temp)
    rows = params_collect.chain_for(walk, walk.nodes[0], ws=ws, pipelines=[launch_pipeline()])
    assert [row.name for row in rows] == ["controller_frequency"]
    assert rows[0].candidates[0].mechanism == model.Mechanism.LAUNCH_PARAMS_FILE.value


def test_the_pipeline_chain_is_deterministic(ws: Workspace):
    walk = pipeline_walk(repo_file(0, cfg("zoo_base.yaml")))
    left = build(ws, [walk], pipelines=[nav_pipeline()])
    right = build(ws, [walk], pipelines=[nav_pipeline()])
    assert left.canonical_bytes() == right.canonical_bytes()


# ---------------------------------------------------------------------------
# the collector


def build(
    ws: Workspace,
    walks: list[records.LaunchWalk],
    pipelines: list[records.ParamPipeline] | None = None,
) -> model.Atlas:
    builder = model.AtlasBuilder()
    ctx = CollectContext(
        ws=ws,
        cfg=Config(
            roots=(
                RootSpec(kind="src", path=str(WS / "src")),
                RootSpec(kind="install", path=str(WS / "install")),
            ),
            profiles=(),
        ),
        builder=builder,
        declared=records.DeclaredBundle(walks=walks, pipelines=list(pipelines or [])),
    )
    params_collect.ParamsCollector().collect(ctx)
    return builder.build()


def test_the_collector_produces_a_valid_atlas(ws: Workspace):
    atlas = build(ws, [zoo_walk()])
    from rosio import schema

    assert schema.validate(atlas.to_json()) == []
    entity = atlas.get("param:/local_costmap/local_costmap|width")
    assert entity is not None
    assert entity.state == "effective"
    profile = ids.build_profile("navigation_zoo")
    assert entity.effective[profile]["value"].value == 40
    assert [candidate.rank for candidate in entity.candidates] == [0, 1]


def test_the_node_name_source_is_the_strongest_offered_whatever_the_order(ws: Workspace):
    """Two collectors reach one node and each has an answer. Writing the field flat let the
    pass that ran last own it, so reordering the passes moved the atlas. The precedence and
    the reason are in `ids.NAME_SOURCE_ORDER`.
    """
    from rosio.collect import launch_declared

    node_id = ids.build_node("/prep/cropper")
    written = []
    for literal_first in (True, False):
        builder = model.AtlasBuilder()
        ctx = CollectContext(
            ws=ws,
            cfg=Config(
                roots=(
                    RootSpec(kind="src", path=str(WS / "src")),
                    RootSpec(kind="install", path=str(WS / "install")),
                ),
                profiles=(),
            ),
            builder=builder,
            declared=records.DeclaredBundle(walks=[argv_walk()], pipelines=[]),
        )
        if literal_first:
            launch_declared._name_node(builder, node_id, model.NameSource.LITERAL.value)
        params_collect.ParamsCollector().collect(ctx)
        if not literal_first:
            launch_declared._name_node(builder, node_id, model.NameSource.LITERAL.value)
        atlas = builder.build()
        written.append((atlas.entities[node_id].name_source, atlas.canonical_bytes()))

    assert written[0][0] == written[1][0] == model.NameSource.LITERAL.value
    assert written[0][1] == written[1][1]


def test_the_collector_is_deterministic(ws: Workspace):
    left = build(ws, [zoo_walk()])
    right = build(ws, [zoo_walk()])
    assert left.canonical_bytes() == right.canonical_bytes()


def test_the_collector_never_marks_a_mismatched_block_effective(ws: Workspace):
    atlas = build(ws, [zoo_walk()])
    entity = atlas.get("param:/lifecycle_manager|node_names")
    assert entity is not None
    assert entity.state == model.ParamState.OWNER_NAME_MISMATCH.value
    assert entity.effective == {}
    assert all(not candidate.wins for candidate in entity.candidates)


def test_the_collector_records_the_selected_plugin_instance(ws: Workspace):
    atlas = build(ws, [zoo_walk()])
    pinst = "pinst:/controller_server|FollowPath"
    assert atlas.get(pinst) is not None
    plugin = ids.build_plugin("nav2_core::Controller", "lx_bringup/OrbitFollower")
    assert atlas.get(plugin) is not None
    assert any(
        edge.kind == "instantiates" and edge.src == pinst and edge.dst == plugin
        for edge in atlas.edges.values()
    )
    assert atlas.get("pinst:/controller_server|FollowPath_DISABLED") is None


def two_selector_walk(tmp_path: Path) -> tuple[Workspace, records.LaunchWalk, tuple[RootSpec, ...]]:
    """One node holding two selector lists that name two different instances."""
    pkg = tmp_path / "src" / "lx_two"
    (pkg / "config").mkdir(parents=True)
    (pkg / "package.xml").write_text(
        "<package format='3'><name>lx_two</name>"
        "<export><build_type>ament_cmake</build_type></export></package>\n"
    )
    target = pkg / "config" / "two.yaml"
    target.write_text(
        "controller_server:\n"
        "  ros__parameters:\n"
        '    controller_plugins: ["FollowPath"]\n'
        '    goal_checker_plugins: ["general_goal_checker"]\n'
        "    FollowPath:\n"
        '      plugin: "lx_two/PathFollower"\n'
        "    general_goal_checker:\n"
        '      plugin: "lx_two/SimpleGoalChecker"\n'
        "      xy_goal_tolerance: 0.25\n"
    )
    roots = (RootSpec(kind="src", path=str(tmp_path / "src")),)
    walk = records.LaunchWalk(
        entry_path=str(pkg / "launch" / "two.launch.py"),
        profile="two_zoo",
        entry_pkg="lx_two",
        nodes=[
            node(
                pkg="lx_two",
                executable="lx_controller",
                name="controller_server",
                params=(records.ParamSource(order=0, kind="repo_file", path=str(target)),),
            )
        ],
    )
    return Workspace.discover(list(roots)), walk, roots


def test_an_instance_takes_the_base_class_of_the_list_that_names_it(tmp_path: Path):
    """Two selector lists on one node name two instances, and neither may take the other's
    base class. Binding by the first list the node holds mints a plugin id that is exported
    under a class nothing selects it as, which is an error severity finding and is false."""
    two_ws, walk, roots = two_selector_walk(tmp_path)
    builder = model.AtlasBuilder()
    params_collect.ParamsCollector().collect(
        CollectContext(
            ws=two_ws,
            cfg=Config(roots=roots, profiles=()),
            builder=builder,
            declared=records.DeclaredBundle(walks=[walk]),
        )
    )
    atlas = builder.build()
    bound = {}
    for instance in ("FollowPath", "general_goal_checker"):
        entity = atlas.get(ids.build_pinst(ids.build_node("/controller_server"), instance))
        assert entity is not None
        values = entity.facts[0].values
        bound[instance] = (values["selector"], values["base_class"])
    assert bound == {
        "FollowPath": ("controller_plugins", "nav2_core::Controller"),
        "general_goal_checker": ("goal_checker_plugins", "nav2_core::GoalChecker"),
    }
    assert atlas.get(ids.build_plugin("nav2_core::GoalChecker", "lx_two/SimpleGoalChecker"))
    wrong = ids.build_plugin("nav2_core::Controller", "lx_two/SimpleGoalChecker")
    assert atlas.get(wrong) is None
    assert atlas.get(ids.build_plugin("nav2_core::GoalChecker", "lx_two/PathFollower")) is None


def test_the_collector_names_plugin_scoped_parameters(ws: Workspace):
    atlas = build(ws, [zoo_walk()])
    pparam = ids.build_pparam("nav2_core::Controller", "lx_bringup/OrbitFollower", "state_timeout")
    assert atlas.get(pparam) is not None
    assert atlas.get(pparam).state == "effective"


def test_the_collector_cites_the_yaml_line_of_the_winner(ws: Workspace):
    atlas = build(ws, [zoo_walk()])
    entity = atlas.get("param:/controller_server|FollowPath.plugin")
    winner = next(candidate for candidate in entity.candidates if candidate.wins)
    assert winner.source.file == "file:lx_bringup|config/zoo_overlay.yaml"
    assert winner.source.locator == "controller_server.ros__parameters.FollowPath.plugin"
    assert winner.source.line_hint == 10
    loser = next(candidate for candidate in entity.candidates if not candidate.wins)
    assert loser.source.file == "file:lx_bringup|config/zoo_base.yaml"
    assert loser.source.line_hint == 12
    assert loser.shadow_kind == "value_changed"


def second_entrypoint() -> records.LaunchWalk:
    """A second entry point of the same profile handing one node the same file.

    A profile on this stack is composed of about twenty five entry points, so two of them
    naming one node and one file is the normal case, not a corner.
    """
    return records.LaunchWalk(
        entry_path=str(WS / "src" / "lx_bringup" / "launch" / "param_order.launch.py"),
        profile="navigation_zoo",
        nodes=[
            node(
                pkg="lx_bringup",
                executable="lx_controller",
                name="controller_server",
                params=(repo_file(0, cfg("zoo_base.yaml")),),
            )
        ],
    )


def test_two_entrypoints_of_one_profile_share_one_write_site(ws: Workspace):
    atlas = build(ws, [zoo_walk(), second_entrypoint()])
    entity = atlas.get("param:/controller_server|controller_frequency")
    assert entity is not None
    profile = ids.build_profile("navigation_zoo")
    ranks = sorted(candidate.rank for candidate in entity.candidates)
    assert ranks == list(range(len(entity.candidates)))
    assert sum(candidate.wins for candidate in entity.candidates) == 1
    assert entity.fact("declared", profile) is not None
    for candidate in entity.candidates:
        assert candidate.profile == profile
        assert candidate.source.file
        assert candidate.source.locator


def _inf_walk() -> records.LaunchWalk:
    walk = argv_walk()
    entry = walk.nodes[0]
    walk.nodes[0] = records.ResolvedNode(
        pkg=entry.pkg,
        executable=entry.executable,
        form=entry.form,
        name=entry.name,
        name_source=entry.name_source,
        namespace=entry.namespace,
        argv=entry.argv,
        params=(
            records.ParamSource(
                order=0,
                kind="ros_args_assign",
                values=(("good_a", "1"), ("gain", ".inf"), ("good_b", "2")),
                site=entry.params[0].site,
            ),
        ),
    )
    return walk


def test_a_non_finite_value_costs_its_own_entry_and_not_the_pass(ws: Workspace):
    """json has no infinity, so the value cannot be written, and the pass keeps going.

    Refusing it inside the value encoder ended the collector and left the profile with zero
    parameters, which reads as a node that declares nothing.
    """
    atlas = build(ws, [_inf_walk()])
    params = {
        ids.parse_param(entity.id).name: entity
        for entity in atlas.entities.values()
        if entity.kind == "param"
    }
    assert set(params) >= {"good_a", "gain", "good_b"}
    profile = ids.build_profile("argv_zoo")
    assert params["good_a"].effective[profile]["value"].value == 1
    assert params["good_b"].effective[profile]["value"].value == 2
    assert params["gain"].state == model.ParamState.OTHER.value
    assert params["gain"].effective == {}
    blind = [entry for entry in atlas.unresolved if entry.owner == params["gain"].id]
    assert [entry.expr for entry in blind] == [".inf"]
    assert "json has no infinity" in blind[0].reason
    assert model.dumps(atlas.to_json())


def test_a_non_finite_value_in_a_list_is_the_same_case(ws: Workspace):
    walk = _inf_walk()
    node = walk.nodes[0]
    walk.nodes[0] = records.ResolvedNode(
        pkg=node.pkg,
        executable=node.executable,
        form=node.form,
        name=node.name,
        name_source=node.name_source,
        namespace=node.namespace,
        argv=node.argv,
        params=(
            records.ParamSource(
                order=0,
                kind="ros_args_assign",
                values=(("good_a", "1"), ("bounds", "[1.0, .inf]")),
                site=node.params[0].site,
            ),
        ),
    )
    atlas = build(ws, [walk])
    params = {
        ids.parse_param(entity.id).name: entity
        for entity in atlas.entities.values()
        if entity.kind == "param"
    }
    assert params["good_a"].state == model.ParamState.EFFECTIVE.value
    assert params["bounds"].state == model.ParamState.OTHER.value
    assert model.dumps(atlas.to_json())


def _run_digests(ws: Workspace, args: tuple[tuple[str, str], ...]) -> list[str]:
    walk = zoo_walk()
    walk.args = args
    atlas = build(ws, [walk])
    return sorted(
        entity.attrs["input_digest"]
        for entity in atlas.entities.values()
        if entity.kind == "run" and entity.attrs.get("input_digest")
    )


def test_an_argument_holding_a_space_is_not_two_arguments(ws: Workspace):
    """The run input digest frames its members, so the two argument sets stay apart.

    Space joining `key=value` pairs first gave ('a', 'b c=d') and ('a', 'b'), ('c', 'd') one
    digest, which is one recorded input for two entry points.
    """
    one = _run_digests(ws, (("a", "b c=d"),))
    two = _run_digests(ws, (("a", "b"), ("c", "d")))
    assert one != two
    assert one == _run_digests(ws, (("a", "b c=d"),))


def test_one_run_per_profile_covers_every_entrypoint(ws: Workspace):
    walk = zoo_walk()
    walk.predicates = {"map.posegraph": True}
    other = second_entrypoint()
    other.predicates = {"map.pbstream": False}
    atlas = build(ws, [walk, other])
    runs = [
        entity
        for entity in atlas.entities.values()
        if entity.kind == "run" and entity.attrs["profile"] == ids.build_profile("navigation_zoo")
    ]
    assert len(runs) == 1
    assert runs[0].attrs["predicates"] == {"map.pbstream": False, "map.posegraph": True}
    alone = build(ws, [zoo_walk()])
    only = [entity for entity in alone.entities.values() if entity.kind == "run"]
    assert only[0].attrs["input_digest"] != runs[0].attrs["input_digest"]


def test_two_profiles_keep_their_own_winner(ws: Workspace):
    walk = zoo_walk()
    other = records.LaunchWalk(
        entry_path=walk.entry_path,
        profile="navigation_base",
        nodes=[
            node(
                pkg="lx_bringup",
                executable="lx_controller",
                name="controller_server",
                params=(repo_file(0, cfg("zoo_base.yaml")),),
            )
        ],
    )
    atlas = build(ws, [walk, other])
    entity = atlas.get("param:/controller_server|FollowPath.plugin")
    winners = {
        candidate.profile: candidate.value for candidate in entity.candidates if candidate.wins
    }
    assert winners == {
        ids.build_profile("navigation_base"): "lx_bringup/PathFollower",
        ids.build_profile("navigation_zoo"): "lx_bringup/OrbitFollower",
    }


def profile_build(
    ws: Workspace,
    walks: list[records.LaunchWalk],
    spec: ProfileSpec,
) -> model.Atlas:
    """The collector over one profile whose entry point order is the configured one."""
    builder = model.AtlasBuilder()
    ctx = CollectContext(
        ws=ws,
        cfg=Config(
            roots=(
                RootSpec(kind="src", path=str(WS / "src")),
                RootSpec(kind="install", path=str(WS / "install")),
            ),
            profiles=(spec,),
        ),
        builder=builder,
        declared=records.DeclaredBundle(walks=walks),
        profile=spec.slug,
    )
    params_collect.ParamsCollector().collect(ctx)
    return builder.build()


def ordered_walk(target: str, params_file: str) -> records.LaunchWalk:
    return records.LaunchWalk(
        entry_path=str(WS / "src" / "lx_bringup" / "launch" / target),
        profile="ordering",
        entry_pkg="lx_bringup",
        nodes=[
            node(
                pkg="lx_bringup",
                executable="lx_controller",
                name="controller_server",
                params=(repo_file(0, cfg(params_file)),),
            )
        ],
    )


def ordering_spec(*targets: str) -> ProfileSpec:
    return ProfileSpec(
        slug="ordering",
        entrypoints=tuple(
            Entrypoint(verb="launch", pkg="lx_bringup", target=target) for target in targets
        ),
    )


def test_precedence_follows_the_configured_entry_point_order(ws: Workspace):
    """The profile runs its entry points in the order it declares them, and that is the
    precedence. Sorting the walks by launch file path instead lets the entry point declared
    first override the one declared last, which is a wrong answer stated with confidence."""
    walks = [
        ordered_walk("param_order.launch.py", "zoo_overlay.yaml"),
        ordered_walk("nav_stack.launch.py", "zoo_base.yaml"),
    ]
    profile = ids.build_profile("ordering")
    late = profile_build(ws, walks, ordering_spec("param_order.launch.py", "nav_stack.launch.py"))
    entity = late.get("param:/controller_server|FollowPath.plugin")
    assert entity.effective[profile]["value"].value == "lx_bringup/PathFollower"
    early = profile_build(ws, walks, ordering_spec("nav_stack.launch.py", "param_order.launch.py"))
    entity = early.get("param:/controller_server|FollowPath.plugin")
    assert entity.effective[profile]["value"].value == "lx_bringup/OrbitFollower"


def test_the_run_digest_holds_no_absolute_path(ws: Workspace, tmp_path: Path):
    """Two boxes with one repository at two mount points are one run, not two."""
    copy = tmp_path / "elsewhere"
    shutil.copytree(WS, copy)
    other = Workspace.discover(
        [
            RootSpec(kind="src", path=str(copy / "src")),
            RootSpec(kind="install", path=str(copy / "install")),
        ]
    )
    here = build(ws, [zoo_walk()])
    moved = build(other, [_rebased_walk(zoo_walk(), copy)])
    digests = {
        entity.attrs["input_digest"] for atlas in (here, moved) for entity in atlas.by_kind("run")
    }
    assert len(digests) == 1


def _rebased_walk(walk: records.LaunchWalk, root: Path) -> records.LaunchWalk:
    def moved(path: str) -> str:
        return str(root / Path(path).relative_to(WS))

    walk.entry_path = moved(walk.entry_path)
    walk.nodes = [
        records.ResolvedNode(
            pkg=item.pkg,
            executable=item.executable,
            namespace=item.namespace,
            name=item.name,
            name_source=item.name_source,
            params=tuple(
                records.ParamSource(
                    order=source.order,
                    kind=source.kind,
                    path=moved(source.path) if source.path else None,
                    values=source.values,
                    site=(
                        None
                        if source.site is None
                        else records.Site(
                            path=moved(source.site.path),
                            locator=source.site.locator,
                            line_hint=source.site.line_hint,
                            root=source.site.root,
                        )
                    ),
                )
                for source in item.params
            ),
        )
        for item in walk.nodes
    ]
    return walk


def test_the_later_spelling_of_one_parameter_wins(ws: Workspace, tmp_path: Path):
    """A dotted key and a nested mapping fold to one name, so only document order tells
    them apart. Verified against humble: the later key is the one the node reads."""
    for text, want in (
        ("    ground.max_slope: 1.0\n    ground:\n      max_slope: 2.0\n", 2.0),
        ("    ground:\n      max_slope: 2.0\n    ground.max_slope: 1.0\n", 1.0),
    ):
        target = tmp_path / f"spelling_{want}.yaml"
        target.write_text("/relay:\n  ros__parameters:\n" + text)
        walk = records.LaunchWalk(
            entry_path=str(WS / "src" / "lx_bringup" / "launch" / "nav_stack.launch.py"),
            profile="spelling",
            nodes=[
                node(
                    pkg="lx_bringup",
                    executable="scan_relay",
                    name="relay",
                    params=(repo_file(0, str(target)),),
                )
            ],
        )
        rows = {row.name: row for row in params_collect.chain_for(walk, walk.nodes[0], ws=ws)}
        row = rows["ground.max_slope"]
        assert row.effective == want
        assert [candidate.value for candidate in row.candidates] == [want, 3.0 - want]


# ---------------------------------------------------------------------------
# identity, provenance and hostile input


def unnamed(**kwargs) -> records.ResolvedNode:
    kwargs.setdefault("pkg", "acme")
    kwargs.setdefault("executable", "worker")
    kwargs.setdefault("namespace", "/")
    kwargs.setdefault("name_source", model.NameSource.UNKNOWN.value)
    return records.ResolvedNode(**kwargs)


def test_the_owner_id_is_the_one_resolve_identity_decides():
    """Two collectors meeting one unnamed node have to reach one id. Deciding it here as
    well split the node in two and left every parameter on an owner in no layer."""
    item = unnamed(
        argv=("--ros-args", "-p", "rate:=5"),
        remaps=(records.Remap(frm="in", to="/bus"),),
        params=(repo_file(0, "/ws/src/acme/config/worker.yaml"),),
    )
    assert params_collect.owner_node_id(item) == identity.node_id(item)


def test_a_temp_parameters_path_never_reaches_the_owner_id(tmp_path: Path):
    """The merged file a launcher hands to launch has a random name, so hashing it mints a
    new node on every run."""

    def merged(name: str) -> records.ResolvedNode:
        source = records.ParamSource(order=0, kind="external_file", path=f"/tmp/{name}")
        return unnamed(params=(source,))

    assert params_collect.owner_node_id(merged("a.yaml")) == params_collect.owner_node_id(
        merged("b.yaml")
    )


def test_a_mount_point_never_reaches_the_owner_id():
    def under(root: str) -> records.ResolvedNode:
        return unnamed(params=(repo_file(0, f"{root}/src/acme/config/worker.yaml"),))

    assert params_collect.owner_node_id(under("/data/ws")) == params_collect.owner_node_id(
        under("/mnt/other/ws")
    )


def test_a_line_hint_never_survives_a_move_to_another_file(ws: Workspace, tmp_path: Path):
    """A document assembled outside the tracked trees keeps its content and loses its path.
    The line belongs to that document, so it cannot travel to the file the evidence cites:
    the reader would land on an unrelated line, and on a shorter file on no line at all."""
    merged = tmp_path / "merged.yaml"
    padding = "\n".join(["# padding"] * 40)
    merged.write_text(f"{padding}\nworker:\n  ros__parameters:\n    speed: 9.0\n")
    entry = WS / "src" / "lx_bringup" / "launch" / "nav_stack.launch.py"
    source = records.ParamSource(order=0, kind="external_file", path=str(merged))
    item = node(pkg="lx_bringup", executable="scan_relay", name="worker", params=(source,))
    walk = records.LaunchWalk(entry_path=str(entry), profile="merged", nodes=[item])
    rows = params_collect.chain_for(walk, item, ws=ws)
    row = next(row for row in rows if row.name == "speed")
    site = row.candidates[0].site
    assert site.path == str(entry)
    assert site.locator.startswith("external params file:")
    assert site.line_hint is None


def test_a_line_hint_survives_when_the_evidence_cites_the_file_it_read(ws: Workspace):
    walk = records.LaunchWalk(
        entry_path=str(WS / "src" / "lx_bringup" / "launch" / "nav_stack.launch.py"),
        profile="plain",
        nodes=[
            node(
                pkg="lx_bringup",
                executable="scan_relay",
                name="controller_server",
                params=(repo_file(0, cfg("zoo_base.yaml")),),
            )
        ],
    )
    rows = params_collect.chain_for(walk, walk.nodes[0], ws=ws)
    assert any(row.candidates[0].site.line_hint for row in rows)


@pytest.mark.parametrize(
    "text",
    [
        "a: &a {x: *a}\n",
        "? [1, 2]\n: v\n",
        "? {a: 1}\n: v\n",
        "a:\n\t- 1\n",
    ],
)
def test_hostile_yaml_is_refused_as_a_collector_error(text: str):
    """A yaml file in a workspace is hostile input. An exception that is not a CollectorError
    empties the whole layer instead of one file."""
    with pytest.raises(CollectorError):
        yaml_lines.load(text, path="hostile.yaml")


def test_a_hostile_parameters_file_does_not_empty_the_chain(ws: Workspace, tmp_path: Path):
    target = tmp_path / "cycle.yaml"
    target.write_text("a: &a {x: *a}\n")
    walk = records.LaunchWalk(
        entry_path=str(WS / "src" / "lx_bringup" / "launch" / "nav_stack.launch.py"),
        profile="hostile",
        nodes=[
            node(
                pkg="lx_bringup",
                executable="scan_relay",
                name="controller_server",
                params=(repo_file(0, str(target)), repo_file(1, cfg("zoo_base.yaml"))),
            )
        ],
    )
    rows = params_collect.chain_for(walk, walk.nodes[0], ws=ws)
    assert rows, "one unreadable file emptied the whole node"


# ---------------------------------------------------------------------------
# what the chain no longer restates, see AGENTS section 11


def test_the_parameter_fact_restates_nothing_the_chain_already_holds(ws: Workspace):
    """The chain holds the file, the locator, the mechanism and the profile of every write
    site. A per site edge, a fact evidence row, the parameter name and the candidate count
    are the same four fields under a second name, and together they were 27 percent of the
    atlas on the reference stack."""
    atlas = build(ws, [zoo_walk()])
    entity = atlas.get("param:/controller_server|controller_frequency")
    assert entity is not None
    fact = entity.fact("declared", ids.build_profile("navigation_zoo"))
    assert fact is not None
    assert fact.evidence == []
    assert "name" not in fact.values
    assert "candidates" not in fact.values
    assert not [edge for edge in atlas.edges.values() if edge.kind == "sets_param"]
    assert entity.label == "controller_frequency"
    assert len(entity.candidates) >= 1
    for candidate in entity.candidates:
        assert candidate.source.file
        assert candidate.source.locator
        assert candidate.mechanism


def test_every_candidate_source_file_is_an_entity(ws: Workspace):
    """Provenance that cites a file the atlas does not hold is unfollowable."""
    atlas = build(ws, [zoo_walk()])
    for entity in atlas.entities.values():
        for candidate in entity.candidates:
            assert candidate.source.file in atlas.entities


# ---------------------------------------------------------------------------
# a python write whose guard did not fold


def guarded_pipeline(value, expr: str) -> records.ParamPipeline:
    return nav_pipeline(
        overrides=(
            records.ParamOverride(
                key_path=("controller_server", "ros__parameters", "controller_frequency"),
                value=value,
                resolved=False,
                expr=expr,
                site=records.Site(
                    path=str(WS / "tools" / "launcher.py"), locator="merged_params", line_hint=52
                ),
            ),
        ),
    )


def guarded_rows(ws: Workspace, value, expr: str) -> dict:
    walk = pipeline_walk(repo_file(0, cfg("zoo_base.yaml")))
    walk.nodes = [
        node(
            pkg="lx_bringup",
            executable="lx_controller",
            name="controller_server",
            params=(repo_file(0, cfg("zoo_base.yaml")),),
        )
    ]
    rows = params_collect.chain_for(
        walk, walk.nodes[0], ws=ws, pipelines=[guarded_pipeline(value, expr)]
    )
    return {row.name: row for row in rows}


def test_a_write_whose_value_folded_but_whose_guard_did_not_stays_in_the_chain(ws: Workspace):
    """Dropping it hides the only line an engineer can edit, and letting it win names a
    value that is wrong on the other branch."""
    expr = "5.0 if os.path.exists(base + '.posegraph')"
    row = guarded_rows(ws, 5.0, expr)["controller_frequency"]
    assert row.state == model.ParamState.OTHER.value
    assert row.effective is None
    assert row.unresolved, "the guard is still a blind spot"
    top = row.candidates[0]
    assert top.value == 5.0
    assert top.guard == expr
    assert top.mechanism == model.Mechanism.PYTHON_OVERRIDE.value
    assert not any(candidate.wins for candidate in row.candidates)
    assert [candidate.value for candidate in row.candidates[1:]] == [20.0, 20.0]


def test_a_write_whose_value_did_not_fold_adds_no_candidate(ws: Workspace):
    """The near miss: nothing is known about the value, so nothing is written down."""
    row = guarded_rows(ws, None, "cfg.frequency if enabled")["controller_frequency"]
    assert row.state == model.ParamState.OTHER.value
    assert row.unresolved
    assert model.Mechanism.PYTHON_OVERRIDE.value not in [
        candidate.mechanism for candidate in row.candidates
    ]
    assert not any(candidate.guard for candidate in row.candidates)


def test_the_guarded_candidate_never_becomes_an_effective_value(ws: Workspace):
    atlas = build(
        ws,
        [pipeline_walk(repo_file(0, cfg("zoo_base.yaml")))],
        [guarded_pipeline(5.0, "5.0 if enabled")],
    )
    entity = atlas.get("param:/controller_server|controller_frequency")
    assert entity is not None
    assert entity.state == model.ParamState.OTHER.value
    assert entity.effective == {}


# ---------------------------------------------------------------------------
# a parameters file a launch argument default named and the profile displaced


def displaced_walk(default: str, supplied: str) -> records.LaunchWalk:
    entry = str(WS / "src" / "lx_bringup" / "launch" / "nav_stack.launch.py")
    return records.LaunchWalk(
        entry_path=entry,
        profile="navigation_zoo",
        entry_pkg="lx_bringup",
        args=(("params_file", supplied),),
        arg_declarations=[
            records.ArgDeclaration(
                name="params_file",
                default=default,
                site=records.Site(path=entry, locator="DeclareLaunchArgument(params_file)"),
            )
        ],
        nodes=[
            node(
                pkg="lx_bringup",
                executable="lx_controller",
                name="controller_server",
                params=(repo_file(0, supplied),),
            )
        ],
    )


def displaced_rows(ws: Workspace, default: str, supplied: str) -> dict:
    walk = displaced_walk(default, supplied)
    return {row.name: row for row in params_collect.chain_for(walk, walk.nodes[0], ws=ws)}


def test_a_displaced_parameters_file_stays_a_candidate(ws: Workspace):
    """The argument overrode the default, so the file is not read here. It is still the file
    an engineer finds when they read the launch file, and a chain that omits it says the file
    contributes nothing anywhere rather than nothing in this profile."""
    row = displaced_rows(ws, cfg("zoo_overlay.yaml"), cfg("zoo_base.yaml"))["FollowPath.plugin"]
    kinds = [candidate.shadow_kind for candidate in row.candidates]
    assert model.ShadowKind.NOT_READ_IN_PROFILE.value in kinds
    displaced = row.candidates[-1]
    assert displaced.shadow_kind == model.ShadowKind.NOT_READ_IN_PROFILE.value
    assert not displaced.wins
    assert displaced.value == "lx_bringup/OrbitFollower"
    assert displaced.site.path.endswith("zoo_overlay.yaml")
    assert displaced.site.line_hint
    assert row.candidates[0].value == "lx_bringup/PathFollower"
    assert row.candidates[0].wins
    assert row.effective == "lx_bringup/PathFollower"


def test_a_displaced_file_mints_no_parameter_of_its_own(ws: Workspace):
    """A key only the displaced file spells is not configured in this profile at all, and
    minting it would invent configuration."""
    rows = displaced_rows(ws, cfg("zoo_overlay.yaml"), cfg("zoo_base.yaml"))
    assert "FollowPath.state_timeout" not in rows
    assert "FollowPath.plugin" in rows


def test_an_argument_the_profile_did_not_displace_is_silent(ws: Workspace):
    """The near miss: the profile supplied the default, so nothing was displaced."""
    same = cfg("zoo_base.yaml")
    rows = displaced_rows(ws, same, same)
    for row in rows.values():
        for candidate in row.candidates:
            assert candidate.shadow_kind != model.ShadowKind.NOT_READ_IN_PROFILE.value


def test_a_displaced_default_that_is_not_a_parameters_file_is_silent(ws: Workspace):
    rows = displaced_rows(ws, cfg("not_params_map.yaml"), cfg("zoo_base.yaml"))
    for row in rows.values():
        for candidate in row.candidates:
            assert candidate.shadow_kind != model.ShadowKind.NOT_READ_IN_PROFILE.value


def test_a_displaced_default_that_is_not_a_path_is_silent(ws: Workspace):
    rows = displaced_rows(ws, "false", cfg("zoo_base.yaml"))
    for row in rows.values():
        for candidate in row.candidates:
            assert candidate.shadow_kind != model.ShadowKind.NOT_READ_IN_PROFILE.value


def displaced_walk_named(entry_name: str, default: str, supplied: str) -> records.LaunchWalk:
    walk = displaced_walk(default, supplied)
    walk.entry_path = str(WS / "src" / "lx_bringup" / "launch" / entry_name)
    return walk


def plain_walk(path: str) -> records.LaunchWalk:
    walk = displaced_walk(path, path)
    walk.arg_declarations = []
    walk.args = ()
    return walk


def test_a_displaced_file_another_entry_point_reads_is_not_displaced(ws: Workspace):
    """A profile is an ordered set of entry points. The second one overrode the file the
    first one reads, so saying `not_read_in_profile` about that line would be false."""
    atlas = build(
        ws,
        [
            plain_walk(cfg("zoo_overlay.yaml")),
            displaced_walk_named("extra.launch.py", cfg("zoo_overlay.yaml"), cfg("zoo_base.yaml")),
        ],
    )
    entity = atlas.get("param:/controller_server|FollowPath.plugin")
    assert entity is not None
    kinds = [candidate.shadow_kind for candidate in entity.candidates]
    assert model.ShadowKind.NOT_READ_IN_PROFILE.value not in kinds
    winners = [candidate for candidate in entity.candidates if candidate.wins]
    assert len(winners) == 1
    assert winners[0].value == "lx_bringup/PathFollower"


def test_a_displaced_file_no_entry_point_reads_stays_a_candidate(ws: Workspace):
    """The near miss of the case above: nothing else reads the file, so the row stays."""
    atlas = build(
        ws,
        [
            plain_walk(cfg("zoo_base.yaml")),
            displaced_walk_named("extra.launch.py", cfg("zoo_overlay.yaml"), cfg("zoo_base.yaml")),
        ],
    )
    entity = atlas.get("param:/controller_server|FollowPath.plugin")
    assert entity is not None
    displaced = [
        candidate
        for candidate in entity.candidates
        if candidate.shadow_kind == model.ShadowKind.NOT_READ_IN_PROFILE.value
    ]
    assert len(displaced) == 1
    assert not displaced[0].wins


def test_two_arguments_naming_one_displaced_file_are_one_candidate(ws: Workspace):
    """One deletable yaml line is one candidate, not one per argument that named the file."""
    walk = displaced_walk(cfg("zoo_overlay.yaml"), cfg("zoo_base.yaml"))
    walk.args = (("params_file", cfg("zoo_base.yaml")), ("nav_params_file", cfg("zoo_base.yaml")))
    walk.arg_declarations = [
        walk.arg_declarations[0],
        records.ArgDeclaration(
            name="nav_params_file",
            default=cfg("zoo_overlay.yaml"),
            site=records.Site(
                path=walk.entry_path, locator="DeclareLaunchArgument(nav_params_file)"
            ),
        ),
    ]
    rows = {row.name: row for row in params_collect.chain_for(walk, walk.nodes[0], ws=ws)}
    displaced = [
        candidate
        for candidate in rows["FollowPath.plugin"].candidates
        if candidate.shadow_kind == model.ShadowKind.NOT_READ_IN_PROFILE.value
    ]
    assert len(displaced) == 1


def test_a_displaced_default_that_does_not_parse_leaves_the_chain_intact(
    ws: Workspace, tmp_path: Path
):
    broken = WS / "src" / "lx_bringup" / "config" / "broken_default.yaml"
    broken.write_text("controller_server:\n  ros__parameters:\n    a: [1, 2\n")
    try:
        rows = displaced_rows(ws, str(broken), cfg("zoo_base.yaml"))
    finally:
        broken.unlink()
    assert rows["FollowPath.plugin"].effective == "lx_bringup/PathFollower"
    for row in rows.values():
        for candidate in row.candidates:
            assert candidate.shadow_kind != model.ShadowKind.NOT_READ_IN_PROFILE.value


def test_a_displaced_default_that_is_not_on_disk_is_silent(ws: Workspace):
    rows = displaced_rows(ws, cfg("no_such_defaults.yaml"), cfg("zoo_base.yaml"))
    assert rows["FollowPath.plugin"].effective == "lx_bringup/PathFollower"
    for row in rows.values():
        for candidate in row.candidates:
            assert candidate.shadow_kind != model.ShadowKind.NOT_READ_IN_PROFILE.value


# ---------------------------------------------------------------------------
# the line an inline dict is written on


PARENT_LAUNCH = """\
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription('child.launch.py')])
"""

CHILD_LAUNCH = """\
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _setup(context, *args, **kwargs):
    return [Node(
        package='lx_disp',
        executable='lx_relay',
        name='relay',
        parameters=[LaunchConfiguration('params_file'), {
            'in_topic': LaunchConfiguration('in_topic'),
            'out_topic': LaunchConfiguration('out_topic'),
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('lx_disp'), 'config', 'child.yaml'])),
        DeclareLaunchArgument(
            'in_topic', default_value='cmd_vel',
            description='what the relay reads.'),
        DeclareLaunchArgument(
            'out_topic', default_value='cmd_vel_safe',
            description='what the relay writes.'),
    ])
"""

CHILD_YAML = """\
relay:
  ros__parameters:
    in_topic: "cmd_vel"
    out_topic: "cmd_vel_safe"
    depth: 3
"""

SUPPLIED_YAML = """\
relay:
  ros__parameters:
    in_topic: "smoothed"
    out_topic: "safe"
    depth: 7
"""


def line_of(text: str, needle: str) -> int:
    for index, line in enumerate(text.splitlines(), start=1):
        if needle in line:
            return index
    raise AssertionError(f"{needle!r} is not in this file")


def disp_ws(tmp_path: Path, files: dict[str, str]) -> tuple[Workspace, Path]:
    """A one package workspace holding the launch and config files a test names."""
    pkg = tmp_path / "src" / "lx_disp"
    (pkg / "launch").mkdir(parents=True, exist_ok=True)
    (pkg / "config").mkdir(parents=True, exist_ok=True)
    (pkg / "package.xml").write_text(
        "<package format='3'><name>lx_disp</name>"
        "<export><build_type>ament_cmake</build_type></export></package>\n"
    )
    for name, text in files.items():
        (pkg / name).write_text(text)
    disp = Workspace.discover([RootSpec(kind="src", path=str(tmp_path / "src"))])
    return disp, pkg


def relay_node(pkg: Path, params: tuple) -> records.ResolvedNode:
    return node(pkg="lx_disp", executable="lx_relay", name="relay", params=params)


def include_walk(pkg: Path, supplied: str) -> records.LaunchWalk:
    """The shape acceptance 6 has: an entry point that includes a launch file, passes it a
    parameters file and one topic argument, and the included file declares both."""
    child = str(pkg / "launch" / "child.launch.py")
    return records.LaunchWalk(
        entry_path=str(pkg / "launch" / "parent.launch.py"),
        profile="disp_zoo",
        entry_pkg="lx_disp",
        includes=[
            records.LaunchInclude(
                path=child,
                args=(("params_file", supplied), ("in_topic", "raw"), ("out_topic", "safe")),
            )
        ],
        nodes=[
            relay_node(
                pkg,
                (
                    records.ParamSource(order=0, kind="repo_file", path=supplied),
                    records.ParamSource(
                        order=1,
                        kind="inline_dict",
                        values=(("in_topic", "raw"), ("out_topic", "safe")),
                        site=records.Site(path=child, locator="parameters"),
                    ),
                ),
            )
        ],
    )


def chain_rows(walk: records.LaunchWalk, disp: Workspace) -> dict:
    return {row.name: row for row in params_collect.chain_for(walk, walk.nodes[0], ws=disp)}


def test_an_inline_dict_cites_the_line_it_is_written_on(tmp_path: Path):
    """The launch action names the attribute the dict came from and not a line, so the chain
    stopped at the file while the value sits on a line an engineer can open."""
    disp, pkg = disp_ws(tmp_path, {"launch/child.launch.py": CHILD_LAUNCH})
    walk = records.LaunchWalk(
        entry_path=str(pkg / "launch" / "child.launch.py"),
        profile="disp_zoo",
        entry_pkg="lx_disp",
        nodes=[
            relay_node(
                pkg,
                (
                    records.ParamSource(
                        order=0,
                        kind="inline_dict",
                        values=(("in_topic", "raw"), ("out_topic", "safe")),
                        site=records.Site(
                            path=str(pkg / "launch" / "child.launch.py"), locator="parameters"
                        ),
                    ),
                ),
            )
        ],
    )
    rows = chain_rows(walk, disp)
    assert rows["in_topic"].candidates[0].site.line_hint == line_of(CHILD_LAUNCH, "'in_topic': Lau")
    assert rows["out_topic"].candidates[0].site.line_hint == line_of(
        CHILD_LAUNCH, "'out_topic': Lau"
    )


def test_two_inline_dicts_with_one_key_set_cite_no_line(tmp_path: Path):
    """Two dicts a key set cannot tell apart state nothing rather than the wrong line."""
    twice = CHILD_LAUNCH.replace(
        "    )]\n",
        "    ), Node(package='lx_disp', executable='lx_other', parameters=[{\n"
        "        'in_topic': 'a',\n"
        "        'out_topic': 'b',\n"
        "    }])]\n",
        1,
    )
    disp, pkg = disp_ws(tmp_path, {"launch/child.launch.py": twice})
    walk = records.LaunchWalk(
        entry_path=str(pkg / "launch" / "child.launch.py"),
        profile="disp_zoo",
        entry_pkg="lx_disp",
        nodes=[
            relay_node(
                pkg,
                (
                    records.ParamSource(
                        order=0,
                        kind="inline_dict",
                        values=(("in_topic", "raw"), ("out_topic", "safe")),
                        site=records.Site(
                            path=str(pkg / "launch" / "child.launch.py"), locator="parameters"
                        ),
                    ),
                ),
            )
        ],
    )
    assert chain_rows(walk, disp)["in_topic"].candidates[0].site.line_hint is None


def test_a_stated_line_is_never_replaced(tmp_path: Path):
    disp, pkg = disp_ws(tmp_path, {"launch/child.launch.py": CHILD_LAUNCH})
    walk = records.LaunchWalk(
        entry_path=str(pkg / "launch" / "child.launch.py"),
        profile="disp_zoo",
        entry_pkg="lx_disp",
        nodes=[
            relay_node(
                pkg,
                (
                    records.ParamSource(
                        order=0,
                        kind="inline_dict",
                        values=(("in_topic", "raw"),),
                        site=records.Site(
                            path=str(pkg / "launch" / "child.launch.py"),
                            locator="parameters",
                            line_hint=3,
                        ),
                    ),
                ),
            )
        ],
    )
    assert chain_rows(walk, disp)["in_topic"].candidates[0].site.line_hint == 3


# ---------------------------------------------------------------------------
# the defaults an included launch file declares


def test_a_parameters_file_default_of_an_included_file_is_a_candidate(tmp_path: Path):
    """The entry point passed the file down, so the default it displaced is declared in the
    included file and not in the entry point. The default names its file through the package
    share, which is a workspace lookup the syntax walk cannot perform."""
    disp, pkg = disp_ws(
        tmp_path,
        {
            "launch/parent.launch.py": PARENT_LAUNCH,
            "launch/child.launch.py": CHILD_LAUNCH,
            "config/child.yaml": CHILD_YAML,
            "config/supplied.yaml": SUPPLIED_YAML,
        },
    )
    rows = chain_rows(include_walk(pkg, str(pkg / "config" / "supplied.yaml")), disp)
    displaced = [
        candidate
        for candidate in rows["in_topic"].candidates
        if candidate.shadow_kind == model.ShadowKind.NOT_READ_IN_PROFILE.value
        and candidate.site.path.endswith("child.yaml")
    ]
    assert len(displaced) == 1
    assert displaced[0].value == "cmd_vel"
    assert displaced[0].site.line_hint == line_of(CHILD_YAML, "in_topic")
    assert not displaced[0].wins
    assert rows["in_topic"].effective == "raw"


def test_a_displaced_argument_default_an_inline_dict_carries_is_a_candidate(tmp_path: Path):
    """The value is the link. The profile supplied it, one argument of the declaring file was
    supplied with it, and one inline dict of that file delivers it, so the default is what
    stands at that site when nobody supplies the argument."""
    disp, pkg = disp_ws(
        tmp_path,
        {
            "launch/parent.launch.py": PARENT_LAUNCH,
            "launch/child.launch.py": CHILD_LAUNCH,
            "config/child.yaml": CHILD_YAML,
            "config/supplied.yaml": SUPPLIED_YAML,
        },
    )
    rows = chain_rows(include_walk(pkg, str(pkg / "config" / "supplied.yaml")), disp)
    displaced = [
        candidate
        for candidate in rows["in_topic"].candidates
        if candidate.shadow_kind == model.ShadowKind.NOT_READ_IN_PROFILE.value
        and candidate.site.path.endswith("child.launch.py")
    ]
    assert len(displaced) == 1
    assert displaced[0].value == "cmd_vel"
    assert displaced[0].mechanism == model.Mechanism.LAUNCH_INLINE_DICT.value
    assert displaced[0].site.locator == "DeclareLaunchArgument(in_topic)"
    assert displaced[0].site.line_hint == line_of(CHILD_LAUNCH, "'in_topic', default_value=")
    assert not displaced[0].wins
    assert displaced[0].rank > 0


def test_the_full_chain_of_the_acceptance_shape_is_ordered(tmp_path: Path):
    """One winner and three shadowed rows, each with a file, a line and a value."""
    disp, pkg = disp_ws(
        tmp_path,
        {
            "launch/parent.launch.py": PARENT_LAUNCH,
            "launch/child.launch.py": CHILD_LAUNCH,
            "config/child.yaml": CHILD_YAML,
            "config/supplied.yaml": SUPPLIED_YAML,
        },
    )
    row = chain_rows(include_walk(pkg, str(pkg / "config" / "supplied.yaml")), disp)["in_topic"]
    assert [candidate.rank for candidate in row.candidates] == [0, 1, 2, 3]
    assert [candidate.wins for candidate in row.candidates] == [True, False, False, False]
    assert [candidate.shadow_kind for candidate in row.candidates[1:]] == [
        model.ShadowKind.VALUE_CHANGED.value,
        model.ShadowKind.NOT_READ_IN_PROFILE.value,
        model.ShadowKind.NOT_READ_IN_PROFILE.value,
    ]
    for candidate in row.candidates:
        assert candidate.site.line_hint is not None
        assert candidate.value is not None


def test_an_included_argument_the_profile_did_not_displace_is_silent(tmp_path: Path):
    """The near miss: the profile supplied the default, so nothing was displaced."""
    disp, pkg = disp_ws(
        tmp_path,
        {
            "launch/parent.launch.py": PARENT_LAUNCH,
            "launch/child.launch.py": CHILD_LAUNCH,
            "config/child.yaml": CHILD_YAML,
        },
    )
    walk = include_walk(pkg, str(pkg / "config" / "child.yaml"))
    walk.includes = [
        records.LaunchInclude(
            path=str(pkg / "launch" / "child.launch.py"),
            args=(
                ("params_file", str(pkg / "config" / "child.yaml")),
                ("in_topic", "cmd_vel"),
                ("out_topic", "cmd_vel_safe"),
            ),
        )
    ]
    walk.nodes[0] = relay_node(
        pkg,
        (
            records.ParamSource(order=0, kind="repo_file", path=str(pkg / "config" / "child.yaml")),
            records.ParamSource(
                order=1,
                kind="inline_dict",
                values=(("in_topic", "cmd_vel"), ("out_topic", "cmd_vel_safe")),
                site=records.Site(
                    path=str(pkg / "launch" / "child.launch.py"), locator="parameters"
                ),
            ),
        ),
    )
    for row in chain_rows(walk, disp).values():
        for candidate in row.candidates:
            assert candidate.shadow_kind != model.ShadowKind.NOT_READ_IN_PROFILE.value


def test_a_value_two_arguments_share_states_nothing(tmp_path: Path):
    """Two supplied arguments carrying one value cannot say which default stood there."""
    disp, pkg = disp_ws(
        tmp_path,
        {
            "launch/parent.launch.py": PARENT_LAUNCH,
            "launch/child.launch.py": CHILD_LAUNCH,
            "config/child.yaml": CHILD_YAML,
            "config/supplied.yaml": SUPPLIED_YAML,
        },
    )
    walk = include_walk(pkg, str(pkg / "config" / "supplied.yaml"))
    walk.includes = [
        records.LaunchInclude(
            path=str(pkg / "launch" / "child.launch.py"),
            args=(
                ("params_file", str(pkg / "config" / "supplied.yaml")),
                ("in_topic", "raw"),
                ("out_topic", "raw"),
            ),
        )
    ]
    walk.nodes[0] = relay_node(
        pkg,
        (
            records.ParamSource(
                order=0, kind="repo_file", path=str(pkg / "config" / "supplied.yaml")
            ),
            records.ParamSource(
                order=1,
                kind="inline_dict",
                values=(("in_topic", "raw"), ("out_topic", "raw")),
                site=records.Site(
                    path=str(pkg / "launch" / "child.launch.py"), locator="parameters"
                ),
            ),
        ),
    )
    rows = chain_rows(walk, disp)
    for name in ("in_topic", "out_topic"):
        for candidate in rows[name].candidates:
            if candidate.shadow_kind == model.ShadowKind.NOT_READ_IN_PROFILE.value:
                assert candidate.site.path.endswith(".yaml")


LITERAL_CHILD = CHILD_LAUNCH.replace(
    "            'out_topic': LaunchConfiguration('out_topic'),\n",
    "            'out_topic': 'raw',\n",
    1,
)


def test_a_literal_that_reads_no_argument_never_takes_a_default(tmp_path: Path):
    """`out_topic` is a literal at the delivering site. It carries the value the profile
    supplied for another argument, and that argument's default was never on this key."""
    disp, pkg = disp_ws(
        tmp_path,
        {
            "launch/parent.launch.py": PARENT_LAUNCH,
            "launch/child.launch.py": LITERAL_CHILD,
            "config/child.yaml": CHILD_YAML,
            "config/supplied.yaml": SUPPLIED_YAML,
        },
    )
    walk = include_walk(pkg, str(pkg / "config" / "supplied.yaml"))
    walk.includes = [
        records.LaunchInclude(
            path=str(pkg / "launch" / "child.launch.py"),
            args=(
                ("params_file", str(pkg / "config" / "supplied.yaml")),
                ("in_topic", "raw"),
            ),
        )
    ]
    walk.nodes[0] = relay_node(
        pkg,
        (
            records.ParamSource(
                order=0, kind="repo_file", path=str(pkg / "config" / "supplied.yaml")
            ),
            records.ParamSource(
                order=1,
                kind="inline_dict",
                values=(("in_topic", "raw"), ("out_topic", "raw")),
                site=records.Site(
                    path=str(pkg / "launch" / "child.launch.py"), locator="parameters"
                ),
            ),
        ),
    )
    rows = chain_rows(walk, disp)
    for candidate in rows["out_topic"].candidates:
        if candidate.shadow_kind == model.ShadowKind.NOT_READ_IN_PROFILE.value:
            assert candidate.site.path.endswith(".yaml")
    assert any(
        candidate.shadow_kind == model.ShadowKind.NOT_READ_IN_PROFILE.value
        and candidate.site.path.endswith("child.launch.py")
        for candidate in rows["in_topic"].candidates
    )


def test_a_value_the_launch_file_computed_never_takes_a_default(tmp_path: Path):
    """The delivering dict is not written in the source, so nothing states that the default
    would stand there. On the reference stack this is the slam_toolbox map_file_name shape,
    where the guard the default disables is what decides whether the key is written at all."""
    disp, pkg = disp_ws(
        tmp_path,
        {
            "launch/parent.launch.py": PARENT_LAUNCH,
            "launch/child.launch.py": CHILD_LAUNCH,
            "config/child.yaml": CHILD_YAML,
            "config/supplied.yaml": SUPPLIED_YAML,
        },
    )
    walk = include_walk(pkg, str(pkg / "config" / "supplied.yaml"))
    walk.nodes[0] = relay_node(
        pkg,
        (
            records.ParamSource(
                order=0, kind="repo_file", path=str(pkg / "config" / "supplied.yaml")
            ),
            records.ParamSource(
                order=1,
                kind="inline_dict",
                values=(("depth", "raw"),),
                site=records.Site(
                    path=str(pkg / "launch" / "child.launch.py"), locator="parameters"
                ),
            ),
        ),
    )
    for candidate in chain_rows(walk, disp)["depth"].candidates:
        if candidate.shadow_kind == model.ShadowKind.NOT_READ_IN_PROFILE.value:
            assert candidate.site.path.endswith(".yaml")


def test_a_default_of_another_file_is_never_attached_to_this_site(tmp_path: Path):
    """The declaring file and the delivering site have to be the same file, or the row would
    put a default of one launch file on a dict written in another."""
    disp, pkg = disp_ws(
        tmp_path,
        {
            "launch/parent.launch.py": PARENT_LAUNCH,
            "launch/child.launch.py": CHILD_LAUNCH,
            "config/child.yaml": CHILD_YAML,
            "config/supplied.yaml": SUPPLIED_YAML,
        },
    )
    walk = include_walk(pkg, str(pkg / "config" / "supplied.yaml"))
    walk.nodes[0] = relay_node(
        pkg,
        (
            records.ParamSource(
                order=0, kind="repo_file", path=str(pkg / "config" / "supplied.yaml")
            ),
            records.ParamSource(
                order=1,
                kind="inline_dict",
                values=(("in_topic", "raw"), ("out_topic", "safe")),
                site=records.Site(
                    path=str(pkg / "launch" / "parent.launch.py"), locator="parameters"
                ),
            ),
        ),
    )
    for candidate in chain_rows(walk, disp)["in_topic"].candidates:
        if candidate.shadow_kind == model.ShadowKind.NOT_READ_IN_PROFILE.value:
            assert candidate.site.path.endswith(".yaml")


def test_a_displaced_default_that_only_the_included_file_spells_mints_nothing(tmp_path: Path):
    """A key only the displaced file spells is not configured in this profile at all."""
    disp, pkg = disp_ws(
        tmp_path,
        {
            "launch/parent.launch.py": PARENT_LAUNCH,
            "launch/child.launch.py": CHILD_LAUNCH,
            "config/child.yaml": CHILD_YAML + "    only_here: 1\n",
            "config/supplied.yaml": SUPPLIED_YAML,
        },
    )
    rows = chain_rows(include_walk(pkg, str(pkg / "config" / "supplied.yaml")), disp)
    assert "only_here" not in rows
    assert "depth" in rows


def test_the_chain_is_the_same_over_two_reads(tmp_path: Path):
    disp, pkg = disp_ws(
        tmp_path,
        {
            "launch/parent.launch.py": PARENT_LAUNCH,
            "launch/child.launch.py": CHILD_LAUNCH,
            "config/child.yaml": CHILD_YAML,
            "config/supplied.yaml": SUPPLIED_YAML,
        },
    )
    walk = include_walk(pkg, str(pkg / "config" / "supplied.yaml"))
    first = chain_rows(walk, disp)["in_topic"].candidates
    second = chain_rows(walk, disp)["in_topic"].candidates
    assert first == second


# ---------------------------------------------------------------------------
# an inline dict a launch argument fills, one file up


def delivered_walk(supplied: str = "30.0", typed: bool = True) -> records.LaunchWalk:
    """include_parent passing child_rate down to the inline dict of include_child."""
    parent = str(WS / "src" / "lx_bringup" / "launch" / "include_parent.launch.py")
    child = str(WS / "src" / "lx_bringup" / "launch" / "include_child.launch.py")
    value = float(supplied) if typed else supplied
    return records.LaunchWalk(
        entry_path=parent,
        profile="include_zoo",
        entry_pkg="lx_bringup",
        nodes=[
            node(
                pkg="lx_sensors",
                executable="scan_relay",
                name="relay_child",
                params=(
                    records.ParamSource(
                        order=0,
                        kind="inline_dict",
                        values=(("rate_hz", value),),
                        site=records.Site(path=child, locator="parameters"),
                    ),
                ),
            )
        ],
        includes=[
            records.LaunchInclude(
                path=child,
                args=(("child_name", "relay_child"), ("child_rate", supplied)),
                site=records.Site(path=parent, locator="IncludeLaunchDescription"),
            )
        ],
    )


@pytest.mark.parametrize("include_line", [None, 19])
def test_a_value_an_include_wrote_is_cited_where_it_is_written(ws: Workspace, include_line):
    from dataclasses import replace

    walk = delivered_walk()
    include = walk.includes[0]
    walk.includes[0] = replace(include, site=replace(include.site, line_hint=include_line))
    row = rows_of(walk, ws)[("/relay_child", "rate_hz")]
    winner = row.candidates[0]
    assert row.effective == 30.0
    assert relative(winner.site.path) == "src/lx_bringup/launch/include_parent.launch.py"
    assert winner.site.locator == "launch_arguments(child_rate)"
    assert winner.site.line_hint == 23
    assert winner.mechanism == model.Mechanism.LAUNCH_INLINE_DICT.value
    assert "include_child.launch.py" in winner.note


def test_the_default_the_include_displaced_is_still_a_candidate(ws: Workspace):
    row = rows_of(delivered_walk(typed=False), ws)[("/relay_child", "rate_hz")]
    displaced = [item for item in row.candidates if item.shadow_kind is not None]
    assert len(displaced) == 1
    assert relative(displaced[0].site.path) == "src/lx_bringup/launch/include_child.launch.py"
    assert displaced[0].value == "10.0"
    assert displaced[0].shadow_kind == model.ShadowKind.NOT_READ_IN_PROFILE.value
    assert displaced[0].wins is False
    assert row.candidates[0].site.line_hint == 23


def test_a_value_the_include_did_not_write_keeps_the_line_that_reads_it(ws: Workspace):
    walk = delivered_walk()
    walk.includes = []
    winner = rows_of(walk, ws)[("/relay_child", "rate_hz")].candidates[0]
    assert relative(winner.site.path) == "src/lx_bringup/launch/include_child.launch.py"
    assert winner.site.line_hint == 23
    assert winner.note == ""


# ---------------------------------------------------------------------------
# the selector key list, against what parameter documents actually spell


def test_both_spellings_of_a_selector_key_are_held():
    """One ROS 2 humble install writes `progress_checker_plugin` in one shipped parameter
    file and `progress_checker_plugins` in another. Holding one of the two reports the block
    the other selects as configured and never loaded."""
    assert "progress_checker_plugin" in params_collect.SELECTOR_KEYS
    assert "progress_checker_plugins" in params_collect.SELECTOR_KEYS
    assert "waypoint_task_executor_plugin" in params_collect.SELECTOR_KEYS


def test_every_selector_key_with_a_base_class_is_a_selector_key():
    missing = set(params_collect.SELECTOR_BASE_CLASS) - set(params_collect.SELECTOR_KEYS)
    assert missing == set()


def test_two_spellings_of_one_key_carry_one_base_class():
    table = params_collect.SELECTOR_BASE_CLASS
    assert table["progress_checker_plugin"] == table["progress_checker_plugins"]


def test_a_singular_selector_key_selects_its_block():
    winners = {
        "progress_checker_plugin": "progress_checker",
        "progress_checker.plugin": "nav2_controller::SimpleProgressChecker",
        "controller_plugins": ["FollowPath"],
        "FollowPath.plugin": "dwb_core::DWBLocalPlanner",
    }
    selected = params_collect._selected_instances(winners)
    assert selected["progress_checker"] == "progress_checker_plugin"
    block, selector = params_collect._governing_block("progress_checker.plugin", winners, selected)
    assert (block, selector) == ("", "progress_checker_plugin")


def _label_walk() -> records.LaunchWalk:
    """A `ros2 run` entry point whose executable is not in the workspace.

    The walk then carries the command label where a path belongs, which is what the
    provenance builder used to raise on.
    """
    label = "ros2 run lx_missing lx_relay"
    return records.LaunchWalk(
        entry_path=label,
        profile="run_only",
        nodes=[
            node(
                pkg="lx_missing",
                executable="lx_relay",
                form="process",
                name="relay",
                name_source="argv",
                namespace="/",
                argv=("--ros-args", "-p", "good_a:=1"),
                params=(
                    records.ParamSource(
                        order=0,
                        kind="ros_args_assign",
                        values=(("good_a", "1"), ("gain", "3.5"), ("good_b", "2")),
                        site=records.Site(path=label, locator=label),
                    ),
                ),
            )
        ],
    )


def test_a_write_site_no_root_can_place_costs_its_candidate_and_not_the_pass(ws: Workspace):
    """Every value here is finite and the profile still produced nothing, because the
    provenance builder raised inside the emit loop on a label that is not a path."""
    atlas = build(ws, [_label_walk()])
    params = {
        ids.parse_param(entity.id).name: entity
        for entity in atlas.entities.values()
        if entity.kind == "param"
    }
    assert sorted(params) == ["gain", "good_a", "good_b"]
    for name, entity in params.items():
        assert entity.state == model.ParamState.OTHER.value, name
        assert entity.effective == {}
        blind = [item for item in atlas.unresolved if item.owner == entity.id]
        assert len(blind) == 1
        assert blind[0].reason.endswith("ros2 run lx_missing lx_relay")
        assert blind[0].evidence == []
        assert entity.unresolved == [blind[0].id]
    assert model.dumps(atlas.to_json())


def test_the_placeable_half_of_a_chain_still_wins(ws: Workspace):
    """One entry point whose site cannot be cited must not cost another one its answer."""
    atlas = build(ws, [zoo_walk(), _label_walk()])
    entity = atlas.get("param:/local_costmap/local_costmap|width")
    assert entity is not None
    assert entity.state == model.ParamState.EFFECTIVE.value
    assert entity.effective[ids.build_profile("navigation_zoo")]["value"].value == 40


def test_an_inline_value_the_sandbox_could_not_carry_is_one_blind_spot(ws: Workspace):
    """The launch child marks what the process boundary refuses instead of dying on it.

    An inline dict holding one such value used to kill the walk of a launch file that real
    ROS 2 loads, and the walk came back as two error findings, one of them false.
    """
    walk = zoo_walk()
    entry = walk.nodes[0]
    site = entry.params[2].site
    walk.nodes[0] = records.ResolvedNode(
        pkg=entry.pkg,
        executable=entry.executable,
        form=entry.form,
        name=entry.name,
        name_source=entry.name_source,
        namespace=entry.namespace,
        argv=entry.argv,
        params=(
            records.ParamSource(
                order=0,
                kind="inline_dict",
                values=(
                    ("good_a", 1),
                    ("gain", launch_child.uncarried("1e400", model.NON_FINITE + ": inf")),
                    ("good_b", 2),
                ),
                site=site,
            ),
        ),
    )
    atlas = build(ws, [walk])
    params = {
        ids.parse_param(entity.id).name: entity
        for entity in atlas.entities.values()
        if entity.kind == "param" and entity.id.startswith("param:/controller_server|")
    }
    profile = ids.build_profile("navigation_zoo")
    assert params["good_a"].effective[profile]["value"].value == 1
    assert params["good_b"].effective[profile]["value"].value == 2
    assert params["gain"].state == model.ParamState.OTHER.value
    assert params["gain"].effective == {}
    blind = [item for item in atlas.unresolved if item.owner == params["gain"].id]
    assert [item.expr for item in blind] == ["1e400"]
    assert "json has no infinity" in blind[0].reason
    assert "uncarried" not in atlas.to_bytes().decode("utf-8")


def test_a_refused_value_inside_a_sequence_is_a_blind_spot_and_not_a_value(ws: Workspace):
    """A sequence is one parameter value, so a mark inside it refuses the whole value.

    Reading only the top of the dict let the mark itself through as the effective value of
    a node, which is a value no source states, and it fabricated a type change finding.
    """
    walk = zoo_walk()
    entry = walk.nodes[0]
    site = entry.params[2].site
    walk.nodes[0] = records.ResolvedNode(
        pkg=entry.pkg,
        executable=entry.executable,
        form=entry.form,
        name=entry.name,
        name_source=entry.name_source,
        namespace=entry.namespace,
        argv=entry.argv,
        params=(
            records.ParamSource(
                order=0,
                kind="inline_dict",
                values=(
                    ("good_a", 1),
                    ("gain", [1.0, launch_child.uncarried("inf", model.NON_FINITE + ": inf")]),
                ),
                site=site,
            ),
        ),
    )
    atlas = build(ws, [walk])
    gain = atlas.get("param:/controller_server|gain")
    assert gain is not None
    assert gain.state == model.ParamState.OTHER.value
    assert gain.effective == {}
    blind = [item for item in atlas.unresolved if item.owner == gain.id]
    assert [item.expr for item in blind] == ["[1.0, inf]"]
    assert atlas.get("param:/controller_server|good_a").effective
    assert "uncarried" not in atlas.to_bytes().decode("utf-8")


def test_a_parameters_file_that_cannot_be_read_says_so(ws: Workspace):
    """A file the collector refused used to leave no entity, no exception and no blind spot,
    so an atlas missing a whole file read exactly like an atlas that had it."""
    missing = cfg("lx_absent.yaml")
    walk = records.LaunchWalk(
        entry_path=str(WS / "src" / "lx_bringup" / "launch" / "nav_stack.launch.py"),
        profile="navigation_zoo",
        entry_pkg="lx_bringup",
        nodes=[
            node(
                pkg="lx_bringup",
                executable="lx_controller",
                name="controller_server",
                namespace="/",
                params=(repo_file(0, cfg("zoo_base.yaml")), repo_file(1, missing)),
            )
        ],
    )
    atlas = build(ws, [walk])
    file_id = ws.file_id(missing)
    assert atlas.get(file_id) is not None
    blind = [item for item in atlas.unresolved if item.owner == file_id]
    assert len(blind) == 1
    assert blind[0].expr == "lx_absent.yaml"
    assert blind[0].reason.startswith("cannot read lx_absent.yaml")
    assert str(WS) not in blind[0].reason


@pytest.mark.parametrize(
    ("relative", "role"),
    (("rviz/missing.rviz", "rviz_configuration"), ("urdf/missing.xacro", "robot_model")),
)
def test_a_missing_launch_input_without_a_source_twin_is_refused(ws, relative, role):
    entry = str(WS / "src/lx_bringup/launch/nav_stack.launch.py")
    missing = str(WS / "install/lx_bringup/share/lx_bringup" / relative)
    site = records.Site(path=entry, locator="Node(view)", line_hint=12)
    walk = records.LaunchWalk(
        entry_path=entry,
        profile="navigation_zoo",
        nodes=[
            node(
                pkg="rviz2",
                executable="rviz2",
                name="view",
                site=site,
                file_reads=(records.FileRead(path=missing, role=role, site=site),),
            )
        ],
    )
    atlas = build(ws, [walk])
    file_id = ws.file_id(missing)
    refused = atlas.get(file_id)
    assert refused is not None
    assert refused.attrs["loadable"] is False
    assert refused.attrs["refused_because"] == "file_missing"
    assert not refused.facts
    blind = [item for item in atlas.unresolved if item.owner == file_id]
    assert len(blind) == 1
    assert blind[0].reason == "file_missing"
    assert blind[0].role == role
    assert blind[0].evidence[0].file == "file:lx_bringup|launch/nav_stack.launch.py"
    assert str(WS) not in atlas.canonical_bytes().decode("utf-8")


def test_generated_rviz_paths_outside_workspace_leave_no_missing_file_refusal(ws, tmp_path):
    entry = str(WS / "src/lx_bringup/launch/nav_stack.launch.py")
    site = records.Site(path=entry, locator="Node(view)")
    atlases = []
    for filename in ("tmp_first", "tmp_second"):
        walk = records.LaunchWalk(
            entry_path=entry,
            profile="navigation_zoo",
            nodes=[
                node(
                    pkg="rviz2",
                    executable="rviz2",
                    name="view",
                    site=site,
                    file_reads=(
                        records.FileRead(str(tmp_path / filename), "rviz_configuration", site),
                    ),
                )
            ],
        )
        atlases.append(build(ws, [walk]))
    assert not atlases[0].unresolved
    assert atlases[0].canonical_bytes() == atlases[1].canonical_bytes()


def test_an_argv_block_sequence_reaches_the_chain_as_the_array_it_is(ws: Workspace):
    """A `-p` value written as a block sequence is an array, not the text of one.

    Measured on humble, one value per process, against an rclpy node with
    `automatically_declare_parameters_from_overrides`: `-p k0:=- a` and a break and `- b`
    comes back `Type.STRING_ARRAY ['a', 'b']`, `- 1` and a break and `- 2` comes back
    `Type.INTEGER_ARRAY [1, 2]`, and `- 1` and a break and `- 2.0` takes `rcl_init` down for
    mixed types exactly as `[1, 2.0]` does.
    """
    cases = (
        ("rings", "- 1\n- 2"),
        ("names", "- a\n- b"),
        ("one", "- a"),
        ("folded", "a\nb"),
        ("mixed", "- 1\n- 2.0"),
        ("empty_item", "- a\n-"),
        ("marker", "--- a"),
    )
    walk = _argv_walk_with(cases)
    rows = {row.name: row for row in params_collect.chain_for(walk, walk.nodes[0], ws=ws)}
    assert rows["rings"].effective == [1, 2]
    assert rows["names"].effective == ["a", "b"]
    assert rows["one"].effective == ["a"]
    assert rows["folded"].effective == "a b"
    for name, reason in (
        ("mixed", ros_args.SEQUENCE_MIXED_TYPES),
        ("empty_item", ros_args.VALUE_EMPTY),
        ("marker", ros_args.VALUE_UNMODELLED),
    ):
        row = rows[name]
        assert row.state != model.ParamState.EFFECTIVE.value, name
        assert row.effective is None, name
        assert not any(candidate.wins for candidate in row.candidates), name
        assert [blind for blind in row.unresolved if blind.reason == reason], name


def test_an_argv_value_this_parser_does_not_model_names_no_value(ws: Workspace):
    """An anchor, a tag, a block scalar header, a directive and a document marker are refused
    rather than modelled, and the code they carry claims nothing about whether the process
    started, because the same construct does both: measured one value per process on humble,
    `-p k0:=&x 1` runs as the integer 1 while `-p k0:=&x` takes `rcl_init` down.
    """
    cases = (
        ("anchor", "&x 1"),
        ("tag", "!!str 1"),
        ("block", "|\n a"),
        ("directive", "%YAML 1.1"),
        ("marker", "---"),
        ("good", "20.0"),
    )
    walk = _argv_walk_with(cases)
    rows = {row.name: row for row in params_collect.chain_for(walk, walk.nodes[0], ws=ws)}
    assert rows["good"].effective == 20.0
    for name, text in cases[:-1]:
        row = rows[name]
        assert row.state != model.ParamState.EFFECTIVE.value, name
        assert row.effective is None, name
        assert not any(candidate.wins for candidate in row.candidates), name
        blinds = [blind for blind in row.unresolved if blind.reason == ros_args.VALUE_UNMODELLED]
        assert blinds, name
        assert blinds[0].expr == f"-p {name}:={text}", name
