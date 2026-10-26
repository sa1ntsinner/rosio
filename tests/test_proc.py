"""The process collector: what argv says, and the two digests taken over it."""

from __future__ import annotations

import itertools
import shlex

import pytest

from rosio import ids, model
from rosio.collect import CollectContext, proc
from rosio.config import Config, ProfileSpec, RootSpec
from rosio.errors import CollectorError
from rosio.workspace import Workspace

TALKER = "/opt/ros/humble/lib/demo_nodes_cpp/talker"

# One value carrying a space, spelled as one token and as two. The text of the two command
# lines is identical, the command lines are not.
NAMED_ONE_TOKEN = (TALKER, "--ros-args", "-r", "__node:=talker", "-p", "note:=one two")
NAMED_TWO_TOKENS = (TALKER, "--ros-args", "-r", "__node:=talker", "-p", "note:=one", "two")

BLIND_ONE_TOKEN = (TALKER, "--ros-args", "-p", "note:=one two")
BLIND_TWO_TOKENS = (TALKER, "--ros-args", "-p", "note:=one", "two")


@pytest.fixture()
def context(tmp_path):
    def build():
        pkg = tmp_path / "src" / "fx_pkg"
        pkg.mkdir(parents=True, exist_ok=True)
        (pkg / "package.xml").write_text("<package><name>fx_pkg</name></package>", encoding="utf-8")
        ws = Workspace.discover([RootSpec("src", str(tmp_path / "src"))])
        cfg = Config(
            roots=tuple(RootSpec(kind=spec.kind, path=spec.path) for spec in ws.roots),
            profiles=(ProfileSpec(slug="live"),),
        )
        builder = model.AtlasBuilder()
        return builder, CollectContext(ws=ws, cfg=cfg, builder=builder, profile="")

    return build


def collect(context, argv_rows):
    builder, ctx = context()
    rows = [proc.parse(argv, pid=index + 1) for index, argv in enumerate(argv_rows)]
    proc.ProcCollector(processes=rows).collect(ctx)
    return builder


def run_id(builder) -> str:
    return next(key for key in builder.atlas.entities if key.startswith("run:"))


# ---------------------------------------------------------------------------
# parsing


def test_a_command_line_without_ros_args_is_not_claimed():
    assert proc.parse((TALKER, "--rate", "10")) is None


def test_a_named_process_carries_its_fully_qualified_name():
    item = proc.parse((TALKER, "--ros-args", "-r", "__ns:=/prep", "-r", "__node:=talker"))
    assert item is not None
    assert item.fqn == "/prep/talker"
    assert item.is_node
    assert not item.is_launcher


def test_a_launcher_is_a_launcher_and_not_a_node():
    item = proc.parse(("/usr/bin/python3", "/opt/ros/humble/bin/ros2", "launch", "fx_pkg", "a.py"))
    assert item is not None
    assert item.is_launcher
    assert item.verb == "launch"
    assert item.target == "fx_pkg a.py"
    assert item.fqn is None


def test_no_process_table_is_not_an_empty_process_table(tmp_path):
    with pytest.raises(CollectorError):
        proc.read_table(str(tmp_path / "absent"))


def test_match_reads_the_name_off_argv():
    rows = [
        proc.parse(NAMED_ONE_TOKEN),
        proc.parse((TALKER, "--ros-args", "-p", "gain:=1")),
    ]
    assert [item.fqn for item in proc.match(rows, fqn="/talker")] == ["/talker"]
    assert len(proc.match(rows, executable="talker")) == 2


# ---------------------------------------------------------------------------
# the two digests over argv


def test_two_processes_that_differ_only_in_a_token_boundary_are_two_runs(context):
    """The run input digest used to space join each argv, so a value holding a space
    answered for two tokens and two boxes really running different command lines showed no
    input difference. `-p note:="one two"` is an ordinary parameter value."""
    one = run_id(collect(context, [NAMED_ONE_TOKEN]))
    two = run_id(collect(context, [NAMED_TWO_TOKENS]))
    assert one != two


def test_the_run_digest_is_the_same_for_the_same_processes(context):
    first = run_id(collect(context, [NAMED_ONE_TOKEN, BLIND_TWO_TOKENS]))
    second = run_id(collect(context, [NAMED_ONE_TOKEN, BLIND_TWO_TOKENS]))
    assert first == second


def test_the_run_digest_does_not_depend_on_the_order_processes_are_read_in(context):
    first = run_id(collect(context, [NAMED_ONE_TOKEN, BLIND_TWO_TOKENS]))
    second = run_id(collect(context, [BLIND_TWO_TOKENS, NAMED_ONE_TOKEN]))
    assert first == second


def test_two_blind_processes_are_two_blind_spots(context):
    """An unresolved entry is keyed over its owner, its expression and what was wanted, so
    the expression is the only thing holding two unnamed processes apart."""
    builder = collect(context, [BLIND_ONE_TOKEN, BLIND_TWO_TOKENS])
    entries = [item for item in builder.atlas.unresolved if item.want == model.Want.NODE_NAME.value]
    assert len(entries) == 2
    assert len({item.id for item in entries}) == 2


def test_a_blind_spot_expression_is_a_command_line_a_human_can_paste(context):
    builder = collect(context, [BLIND_ONE_TOKEN])
    entry = builder.atlas.unresolved[0]
    assert shlex.split(entry.expr)[-2:] == ["-p", "note:=one two"]
    assert entry.reason.startswith("the command line carries no __node assignment")


def test_shell_quoting_holds_a_token_list_apart():
    """The frame under the expression, swept rather than argued: every token list of length
    1 to 3 over a hostile alphabet is its own line."""
    alphabet = ["", "a", " ", "'", '"', "\\", "\n", "\x00", "a b", "'a'", "|", "%", "~"]
    lines = set()
    total = 0
    for width in (1, 2, 3):
        for combo in itertools.product(alphabet, repeat=width):
            lines.add(shlex.join(combo))
            total += 1
    assert total == 2379
    assert len(lines) == total


# ---------------------------------------------------------------------------
# the fact


def test_the_named_process_fact_keeps_the_order_argv_states(context):
    argv = (
        TALKER,
        "--ros-args",
        "-r",
        "__node:=talker",
        "--params-file",
        "/ws/z.yaml",
        "--params-file",
        "/ws/a.yaml",
        "-p",
        "gain:=2",
    )
    builder = collect(context, [argv])
    values = builder.atlas.entities[ids.build_node("/talker")].facts[0].values
    assert [item.rsplit("/", 1)[-1] for item in values["params_files"]] == ["z.yaml", "a.yaml"]
    assert [pair[0] for pair in values["assignments"]] == ["gain"]
    assert values["argv"][0] != TALKER


def test_the_node_carries_argv_as_its_name_source(context):
    builder = collect(context, [NAMED_ONE_TOKEN])
    node = builder.atlas.entities[ids.build_node("/talker")]
    assert node.name_source == model.NameSource.ARGV.value


def test_the_pid_never_reaches_compared_data(context):
    builder = collect(context, [NAMED_ONE_TOKEN])
    assert b"pid" not in builder.atlas.canonical_bytes()
