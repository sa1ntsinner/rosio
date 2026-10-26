"""The second half of the observed layer: what the system was actually started with.

A probe says what a node is. The process table says how it was started, and those are two
different truths. The argv holds every parameters file path, every remap and every node name
assignment that rcl was handed, so reading it catches the three cases nothing else catches:
a generated parameters file that is older than the config it was generated from, a hand
edited override on a running process, and a node an operator started outside the launcher.

Two rules keep this honest.

* A process is claimed as a ROS node only when its own argv says so. `--ros-args` is proof.
  A `ros2 launch` or `ros2 run` command line is proof of a launcher, not of a node. Anything
  else is not claimed, because matching by executable name alone would attach a node identity
  to any binary that shares a name.
* A process is matched to a node by the name in its argv, and otherwise it is unmatched and
  recorded as a blind spot. An executable with no `__node` assignment runs under a name only
  its own source states, and guessing it is how a process gets attributed to the wrong node.

Paths in an argv are machine paths, so nothing reaches the atlas raw: a workspace file
becomes its file id, a temp file becomes the temp marker with its basename, and anything
else becomes the outside marker. The pid is volatile and never enters an id or compared data.

The environment states the middleware implementation, or `default` when no implementation
was exported. Unreadable environments stay `unknown`. Zenoh configuration variables carry
presence only. Router presence is a fact on the run, not on any node.

This reads `/proc`, which is Linux. On a system without it the collector raises
`CollectorError` rather than reporting an empty process table, because no processes and no
process table are different statements.
"""

from __future__ import annotations

import shlex
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

from rosio import ids, model
from rosio.collect import CollectContext, register
from rosio.errors import CollectorError
from rosio.resolve import identity, ros_args

LAYER = model.Layer.OBSERVED.value
NAME = "proc"

PROC = "/proc"

ROS_ARGS = ros_args.ROS_ARGS

CLI = "ros2"
CLI_VERBS = ("launch", "run")

MAX_ARGV = 4096
"""How much of one command line is read. A generated command line on this stack is long and
bounded, and an unbounded read would put a whole file into a fact."""

ZENOH_ENV = (
    "ZENOH_ROUTER_CHECK_ATTEMPTS",
    "ZENOH_CONFIG_OVERRIDE",
    "ZENOH_SESSION_CONFIG_URI",
    "ZENOH_ROUTER_CONFIG_URI",
)


@dataclass(frozen=True)
class RosProcess:
    """One running process that says what it is.

    `fqn` is present only when argv named the node. `pid` is volatile: it is carried so a
    human can find the process and it never enters an id, a fact value or a digest.
    """

    argv: tuple[str, ...]
    program: str = ""
    executable: str = ""
    node_name: str | None = None
    namespace: str | None = None
    fqn: str | None = None
    params_files: tuple[str, ...] = ()
    remaps: tuple[tuple[str, str], ...] = ()
    assignments: tuple[tuple[str, str], ...] = ()
    verb: str = ""
    target: str = ""
    pid: int = 0
    ppid: int = 0
    rmw: str = "unknown"
    zenoh_environment: tuple[tuple[str, str], ...] = ()

    @property
    def is_router(self) -> bool:
        return self.executable == "rmw_zenohd" or (
            self.verb == "run" and self.target == "rmw_zenoh_cpp rmw_zenohd"
        )

    @property
    def is_node(self) -> bool:
        return ROS_ARGS in self.argv

    @property
    def is_launcher(self) -> bool:
        return bool(self.verb)


# ---------------------------------------------------------------------------
# reading the table


def read_table(root: str = PROC) -> list[tuple[int, tuple[str, ...]]]:
    """Every readable command line, by pid. A process that exits mid read is skipped."""
    base = Path(root)
    if not base.is_dir():
        raise CollectorError(f"no process table at {root}", LAYER, NAME)
    out: list[tuple[int, tuple[str, ...]]] = []
    for entry in sorted(base.iterdir(), key=lambda item: item.name):
        if not entry.name.isdigit():
            continue
        try:
            raw = (entry / "cmdline").read_bytes()[:MAX_ARGV]
        except (OSError, ValueError):
            continue
        argv = tuple(part for part in raw.decode("utf-8", "replace").split("\0") if part)
        if argv:
            out.append((int(entry.name), argv))
    return out


def parse(argv: Sequence[str], pid: int = 0) -> RosProcess | None:
    """One command line as a ROS process, or None when nothing in it says it is one."""
    tokens = [str(item) for item in argv]
    if not tokens:
        return None
    program = tokens[0]
    executable = program.rsplit("/", 1)[-1]
    verb, target = _cli(tokens, executable)
    if ROS_ARGS not in tokens and not verb and executable != "rmw_zenohd":
        return None
    parsed = ros_args.parse_ros_args(tokens)
    namespace = parsed.namespace
    name = parsed.node_name
    fqn = None
    if name:
        fqn = ids.normalise_ros_name(name, ids.normalise_namespace(namespace))
    return RosProcess(
        argv=tuple(tokens),
        program=program,
        executable=executable,
        node_name=name,
        namespace=namespace,
        fqn=fqn,
        params_files=tuple(parsed.params_files),
        remaps=tuple(parsed.remaps),
        assignments=tuple(parsed.assignments),
        verb=verb,
        target=target,
        pid=pid,
    )


def _cli(tokens: Sequence[str], executable: str) -> tuple[str, str]:
    """The `ros2 launch` or `ros2 run` invocation a command line carries, if it is one."""
    head = list(tokens)
    if executable.startswith("python") and len(head) > 1:
        # The ros2 entry point carries a shebang, so /proc shows the interpreter first and
        # the four tokens that name the command start one later.
        head = head[1:]
        executable = head[0].rsplit("/", 1)[-1]
    head = head[:4]
    if executable != CLI or len(head) < 3:
        return "", ""
    verb = head[1]
    if verb not in CLI_VERBS:
        return "", ""
    return verb, " ".join(head[2:])


def scan(root: str = PROC) -> list[RosProcess]:
    """ROS nodes, launchers and Zenoh routers, with environments, sorted by argv."""
    table = read_table(root)
    parents = {pid: _parent(root, pid) for pid, _argv in table}
    found = [item for pid, argv in table if (item := parse(argv, pid)) is not None]
    recognised = {item.pid for item in found}

    def parent_of(pid: int) -> int:
        parent = parents.get(pid, 0)
        visited = {pid}
        while parent and parent not in recognised and parent not in visited:
            visited.add(parent)
            parent = parents.get(parent, 0)
        return parent if parent not in visited else 0

    rows = [
        replace(item, ppid=parent_of(item.pid), **_environment(root, item.pid)) for item in found
    ]
    rows.sort(key=lambda item: (item.fqn or "", item.executable, item.argv))
    return rows


def _parent(root: str, pid: int) -> int:
    try:
        text = (Path(root) / str(pid) / "stat").read_text()
        return int(text.rsplit(")", 1)[1].split()[1])
    except (OSError, ValueError, IndexError):
        return 0


def unclaimed_launchers(rows: Sequence[RosProcess]) -> list[RosProcess]:
    """Keep a launcher unless parentage proves a claimed node represents it."""
    parents = {row.pid: row.ppid for row in rows if row.pid}
    claimed = set()
    for row in rows:
        if not row.is_node or row.is_launcher:
            continue
        parent = row.ppid
        visited = set()
        while parent and parent not in visited:
            claimed.add(parent)
            visited.add(parent)
            parent = parents.get(parent, 0)
    return [row for row in rows if row.is_launcher and not row.is_router and row.pid not in claimed]


def _environment(root: str, pid: int) -> dict[str, Any]:
    """Read only middleware identity and configuration presence, never URI values."""
    try:
        raw = (Path(root) / str(pid) / "environ").read_bytes()
    except (OSError, ValueError):
        return {
            "rmw": "unknown",
            "zenoh_environment": tuple((key, "unknown") for key in ZENOH_ENV),
        }
    env = dict(
        part.split("=", 1) for part in raw.decode("utf-8", "replace").split("\0") if "=" in part
    )
    return {
        "rmw": env.get("RMW_IMPLEMENTATION", "default"),
        "zenoh_environment": tuple(
            (key, "present" if key in env else "absent") for key in ZENOH_ENV
        ),
    }


def match(
    processes: Sequence[RosProcess], *, fqn: str = "", executable: str = ""
) -> list[RosProcess]:
    """The processes that answer to a name or run an executable, both read off argv."""
    out = []
    for item in processes:
        by_name = bool(fqn) and item.fqn == fqn
        by_exe = not fqn and bool(executable) and item.executable == executable
        if by_name or by_exe:
            out.append(item)
    return out


# ---------------------------------------------------------------------------
# the collector


class ProcCollector:
    """Write process arguments, middleware and router presence into the observed layer."""

    name = NAME
    layer = LAYER

    def __init__(self, *, root: str = PROC, processes: Sequence[RosProcess] | None = None) -> None:
        self.root = root
        self.processes = processes

    def collect(self, ctx: CollectContext) -> None:
        rows = list(self.processes) if self.processes is not None else scan(self.root)
        builder = ctx.builder
        profile = ids.build_profile(ctx.profile) if ctx.profile else model.ANY_PROFILE
        named = [item for item in rows if item.fqn and item.is_node and not item.is_launcher]
        router = "present" if any(item.is_router for item in rows) else "absent"
        run = builder.add_run(
            LAYER,
            profile=profile if ctx.profile else None,
            input_digest=ids.digest(
                NAME,
                router,
                *sorted(
                    ids.digest_fields(
                        {
                            "argv": _token_argv(ctx, item),
                            "rmw": item.rmw,
                            **dict(item.zenoh_environment),
                        }
                    )
                    for item in rows
                ),
            ),
            roots=ctx.ws.root_hashes(),
            volatile={"processes": len(rows), "pids": sorted(item.pid for item in rows)},
        )
        builder.node(
            run,
            attrs={"commands": sorted(_command(ctx, item) for item in rows if item.is_launcher)},
        )
        builder.fact(
            run,
            LAYER,
            profile=profile,
            run=run,
            values={
                "zenoh_router": router,
                "launchers": sorted(
                    [
                        {"command": _launcher_command(ctx, item), "rmw": item.rmw}
                        for item in unclaimed_launchers(rows)
                    ],
                    key=lambda row: (row["command"], row["rmw"]),
                ),
            },
            confidence=model.Confidence.CERTAIN.value,
        )
        if not named:
            builder.declare_layer(
                LAYER,
                present=bool(rows),
                run=run,
                reason=("no running process names a node in its argv" if rows else ""),
                root=model.Root.RUNTIME.value,
            )
        else:
            builder.declare_layer(LAYER, present=True, run=run, root=model.Root.RUNTIME.value)
        for item in named:
            self._node(ctx, run, profile, item)
        for item in rows:
            if item.fqn or item.is_launcher or not item.is_node:
                # A launcher is not a node that lost its name, it is the command that starts
                # them, and its own argv tail is the tail it passes on.
                continue
            builder.unresolved(
                model.Unresolved(
                    want=model.Want.NODE_NAME.value,
                    owner=run,
                    expr=_argv_text(ctx, item),
                    reason="the command line carries no __node assignment, so the name it runs "
                    "under is stated only by its own source",
                    role="process",
                    resolvable_by=["read the name from the source", "probe the running system"],
                )
            )

    def _node(self, ctx: CollectContext, run: str, profile: str, item: RosProcess) -> None:
        builder = ctx.builder
        node_id = ids.build_node(str(item.fqn))
        existing = builder.atlas.entities.get(node_id)
        held = existing is not None and existing.fact(LAYER, profile) is not None
        builder.node(node_id, name_source=model.NameSource.ARGV.value)
        values: dict[str, Any] = {
            "name": str(item.fqn).rsplit("/", 1)[-1],
            "namespace": ids.normalise_namespace(item.namespace),
            "executable": item.executable,
            "argv": _token_argv(ctx, item),
            "rmw": item.rmw,
            "zenoh_environment": dict(item.zenoh_environment),
        }
        # Argv order is the precedence rcl applied, so it is data and not presentation:
        # a later parameters file wins the keys it shares with an earlier one. Sorting
        # these made the fact contradict the argv kept beside it. Determinism is two runs
        # producing the same sequence, which argv order already is.
        if item.params_files:
            values["params_files"] = [_path_token(ctx, path) for path in item.params_files]
        if item.remaps:
            values["remaps"] = [list(pair) for pair in item.remaps]
        if item.assignments:
            values["assignments"] = [
                [key, ctx.ws.stated_value(value)] for key, value in item.assignments
            ]
        # One slot per layer and profile, written by one run. When the probe already filled
        # it, these keys are added to it and the run that owns the slot stays the probe's:
        # the two halves describe one running node and must not become two facts.
        builder.fact(
            node_id,
            LAYER,
            profile=profile,
            run=None if held else run,
            values=values,
            confidence=model.Confidence.CERTAIN.value,
        )
        builder.count_resolved(model.Want.NODE_NAME.value)


def available(root: str = PROC) -> bool:
    """Whether a process table can be read here."""
    return Path(root).is_dir()


def summarise(processes: Sequence[RosProcess]) -> Mapping[str, Any]:
    """Counts for a human reading a scan. Every number here is volatile."""
    return {
        "processes": len(processes),
        "named": len([item for item in processes if item.fqn]),
        "launchers": len([item for item in processes if item.is_launcher]),
        "params_files": len({path for item in processes for path in item.params_files}),
    }


def _looks_like_path(text: str) -> bool:
    """A ROS name also starts with a slash, so a token is a path only when it names one."""
    tail = text.rsplit("/", 1)[-1]
    if "." in tail:
        return True
    try:
        return Path(text).exists()
    except OSError:
        return False


def _path_token(ctx: CollectContext, value: str) -> str:
    """A token the argv itself says is a path: the program, or a parameters file."""
    text = str(value)
    if not text.startswith("/"):
        return text
    try:
        token = ctx.ws.token(text)
    except Exception:
        token = text
    if token != text:
        return token
    return identity.relative_path(text, [spec.path for spec in ctx.ws.roots])


def _token(ctx: CollectContext, value: str) -> str:
    """One argv token as compared data. A machine path never reaches a fact.

    A ROS name also starts with a slash, so a bare token is reduced only when it names a
    file on disk or carries a suffix. The tokens the argv itself says are paths go through
    `_path_token` instead, where no such judgement is needed.
    """
    text = str(value)
    if not text.startswith("/"):
        return ctx.ws.stated_value(text)
    if identity.under_temp(text, [spec.path for spec in ctx.ws.roots]) or _looks_like_path(text):
        return _path_token(ctx, text)
    try:
        token = ctx.ws.token(text)
    except Exception:
        token = text
    return token


def _command(ctx: CollectContext, item: RosProcess) -> str:
    """One launcher invocation as compared data. A launch file named by path is a machine
    path, so it is reduced the same way every other argv token is."""
    target = " ".join(_token(ctx, token) for token in item.target.split(" ") if token)
    return f"{item.verb} {target}".strip()


def _token_argv(ctx: CollectContext, item: RosProcess) -> list[str]:
    paths = {item.program, *item.params_files}
    return [
        _path_token(ctx, token) if token in paths else _token(ctx, token) for token in item.argv
    ]


def _launcher_command(ctx: CollectContext, item: RosProcess) -> str:
    tokens = _token_argv(ctx, item)
    offset = 1 if item.executable == CLI else 2
    return shlex.join([CLI, *tokens[offset:]])


def _argv_text(ctx: CollectContext, item: RosProcess) -> str:
    """One argv as the line an operator can paste back into a shell.

    An unresolved entry is keyed by a digest over its owner, its expression and what was
    wanted, so this text is the only thing holding two blind processes apart. A token may
    hold a space, and `-p note:="one two"` is an ordinary parameter value, so joining with a
    space made two different command lines one entry. Shell quoting is the frame here rather
    than `ids.digest_fields`, because the expression is read by a human and has to stay a
    command line: it round trips through `shlex.split`, and over an exhaustive sweep of
    every token list of length 1 to 3 from a hostile alphabet of thirteen members the 2,379
    lists are 2,379 distinct strings.
    """
    return shlex.join(_token_argv(ctx, item))


def _factory() -> ProcCollector:
    return ProcCollector()


register(NAME, _factory)


__all__ = [
    "MAX_ARGV",
    "PROC",
    "ZENOH_ENV",
    "ProcCollector",
    "RosProcess",
    "available",
    "match",
    "parse",
    "read_table",
    "scan",
    "summarise",
]
