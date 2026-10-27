"""The MCP server: one tool set over two transports.

stdio is the default and serves one developer from one process. Streamable HTTP serves a
team from one process, and it refuses to listen anywhere but loopback unless a key is set,
because an atlas names every file, node and parameter of a workspace.

The mcp package is an optional extra. Nothing in this module imports it at import time, so
the tool set stays testable, and the error names the command that installs it.
"""

from __future__ import annotations

import hmac
import inspect
from dataclasses import dataclass, field
from typing import Any

from rosio import __version__
from rosio.errors import RosioError, UsageError
from rosio.model import SEVERITY_ORDER, Want
from rosio.serve import tools

SERVER_NAME = "rosio"
DEFAULT_PORT = 8787
DEFAULT_PATH = "/mcp"
LOOPBACK = frozenset({"127.0.0.1", "::1", "localhost", "::ffff:127.0.0.1"})

MCP_EXTRA = "mcp>=1.14,<3"
"""The one place the mcp range is written in code. pyproject restates it and a test holds
the two equal, and the cli reads this name rather than spelling the range again."""


def _bounds(spec: str) -> tuple[tuple[int, int], tuple[int, int]]:
    """The two comparable releases a `name>=low,<high` range names."""
    low, high = spec.split(",", 1)
    return _release(low.split(">=", 1)[1]), _release(high.split("<", 1)[1])


def _release(text: str) -> tuple[int, int]:
    parts = [*text.strip().split("."), "0"][:2]
    return int(parts[0]), int(parts[1])


MCP_MIN, MCP_MAX = _bounds(MCP_EXTRA)
"""The range as two comparable releases, derived so a bump of the string cannot leave the
version check the cli runs behind."""

MCP_INSTALL = f"pip install 'rosio[mcp]', which pins {MCP_EXTRA}"


def mcp_problem() -> str:
    """Why this interpreter cannot serve, in one sentence, or nothing when it can.

    An mcp that is present but outside the range is the case the import guard reads as
    absent, and telling a user to install something they already have wastes the one
    message they get. Read from the installed metadata, so nothing is imported."""
    from importlib.metadata import PackageNotFoundError, version

    try:
        found = version("mcp")
    except PackageNotFoundError:
        return f"rosio serve needs the mcp package, which is not installed. {MCP_INSTALL}"
    parts = found.split(".")[:2]
    if len(parts) < 2 or not all(part.isdigit() for part in parts):
        return ""
    if MCP_MIN <= (int(parts[0]), int(parts[1])) < MCP_MAX:
        return ""
    return (
        f"mcp {found} is installed and rosio serve is not exercised against it. "
        f"the tools are registered from plain functions with postponed annotations, which "
        f"mcp resolves from {MCP_MIN[0]}.{MCP_MIN[1]} on. {MCP_INSTALL}"
    )


MCP_MISSING = (
    "the mcp package is not installed, so there is no server to run. "
    f"install it with: {MCP_INSTALL}"
)

UVICORN_EXTRA = "uvicorn>=0.31.1"
"""The one place the uvicorn range is written in code, held equal to pyproject by a test."""

UVICORN_MISSING = (
    "the http transport needs uvicorn, which is not installed. "
    f"install it with: pip install 'rosio[mcp]', which pins {UVICORN_EXTRA}"
)

INSTRUCTIONS = (
    "rosio answers questions about a ROS 2 workspace from a precomputed atlas of four "
    "layers: what the repository declares, what the code does, what a live system reported "
    "and what a recording holds. Call overview first. Every answer is deterministic, states "
    "the file and line to edit, and says so when the atlas cannot support an answer."
)


@dataclass
class Options:
    """What the two transports need, and the one refusal that is not a warning."""

    atlas: str
    http: bool = False
    host: str = "127.0.0.1"
    port: int = DEFAULT_PORT
    api_key: str = ""
    path: str = DEFAULT_PATH
    stateless: bool = False
    workspace: str = ""
    extra: dict[str, Any] = field(default_factory=dict)

    def check(self) -> Options:
        if not str(self.atlas).strip():
            raise UsageError("an atlas is needed: --atlas <dir>/atlas.json")
        if not self.http:
            return self
        if not self.path.startswith("/"):
            raise UsageError(f"the http path must start with a slash, not {self.path!r}")
        if self.host not in LOOPBACK and not self.api_key:
            raise UsageError(
                f"binding to {self.host} serves the atlas to the network and no key is set. "
                "pass --api-key <secret>, or bind 127.0.0.1 and put a proxy in front. "
                "an atlas names every file, node and parameter of the workspace"
            )
        if not 1 <= int(self.port) <= 65535:
            raise UsageError(f"port {self.port} is not a port number")
        return self


# ---------------------------------------------------------------------------
# the tool set, bound to one atlas


def bindings(source: tools.AtlasSource) -> list[Any]:
    """The twelve functions the server exposes. The signature is the tool schema and the
    docstring is what a model reads before choosing, so both are part of the product."""

    def overview() -> str:
        """Start here. What this atlas holds: profiles, layers, entity counts, coverage and
        the worst drift findings, in one screen."""
        return tools.overview(source.atlas())

    def param_chain(node: str, name: str = "", profile: str = "") -> str:
        """The effective value of one parameter, its type, the file and line that set it,
        and every candidate that line shadows. node is a node name such as
        /controller_server, name is the dotted parameter name such as FollowPath.plugin,
        profile narrows to one of the systems this atlas covers, named by its slug."""
        return tools.param_chain(source.atlas(), node, name, profile=profile)

    def drift(min_severity: str = "", only: str = "", profile: str = "", limit: int = 5) -> str:
        """Findings ready to quote, each with one sentence, the line to edit and the
        fingerprint explain takes."""
        return tools.drift(
            source.atlas(), min_severity=min_severity, rule=only, profile=profile, limit=limit
        )

    def explain(selector: str, limit: int = 8) -> str:
        """One entity in full: its layers, facts, conflicts, effective values and
        provenance. Or one finding, or every finding of one rule."""
        return tools.explain(source.atlas(), selector, limit=limit)

    def query(question: str, limit: int = 8) -> str:
        """A scoped subgraph for a question in plain words, for example "which nodes
        publish cmd_vel". Matched by name and kind, with no language model, and it says so
        when it did not understand."""
        return tools.query(source.atlas(), question, limit=limit)

    def path(source: str, target: str) -> str:
        """How data gets from one entity to another: the shortest route through topics,
        services, remaps and TF, with the kind and direction of every hop. A route through
        lifecycle management or launch is shown only when no data route exists."""
        return tools.path(store.atlas(), source, target)

    def neighbours(selector: str, kind: str = "", direction: str = "both") -> str:
        """What one entity is connected to, counted by edge kind."""
        return tools.neighbours(source.atlas(), selector, kind=kind, direction=direction)

    def graph(
        profile: str = "", focus: str = "", depth: int = 2, wrappers: bool = False, limit: int = 8
    ) -> str:
        """Who talks to what: one line per topic, service or action with the nodes that
        publish, subscribe, serve or call it, the QoS the atlas states and the rule ids of
        its findings. focus is a node or channel such as /scan_filter, depth how many hops
        around it, wrappers lists library endpoints such as tf2_ros listeners."""
        return tools.graph(
            source.atlas(),
            profile=profile,
            focus=focus,
            depth=depth,
            wrappers=wrappers,
            limit=limit,
        )

    def unresolved(want: str = "", owner: str = "") -> str:
        """The blind spots: values this atlas could not resolve, so a reader can tell a
        hollow answer from a complete one."""
        return tools.unresolved(source.atlas(), want=want, owner=owner)

    def stamps(bag: str = "", limit: int = 8) -> str:
        """The header stamp verdict of every recording in the atlas, with the topics whose
        clocks step backwards, freeze, jump or sit years off. bag narrows to one recording."""
        return tools.stamps(source.atlas(), bag=bag, limit=limit)

    def transform(source: str, target: str, profile: str = "", model: str = "") -> str:
        """Where one frame sits relative to another, composed from the robot model joints
        and the static_transform_publisher nodes launch files start: the pose of source in
        the target frame, xyz in metres, rpy and quaternion, then the chain link by link with
        file and line. A frame such as laser_link or base_link.
        It stops at a transform a node broadcasts at run time, and answers once per robot
        model when models disagree; model names one model file."""
        return tools.transform(store.atlas(), source, target, profile=profile, model=model)

    def migrate(to: str, start: str = "", package: str = "", workspace: str = ".") -> str:
        """Everything moving this workspace to another ROS 2 release asks, from measured
        release data and cited rules, never from memory of release notes. to is the target
        release such as lyrical, start the current one (from is accepted too). Lists the by
        hand changes with file and line, the exact rewrites and the one command that
        performs them. Over http, workspace stays inside the served workspace."""
        within = store.workspace() if store.confined else None
        return tools.migrate(workspace, to, start=start, package=package, within=within)

    store = source
    return [
        overview,
        param_chain,
        drift,
        explain,
        query,
        path,
        neighbours,
        graph,
        unresolved,
        stamps,
        transform,
        migrate,
    ]


ARGUMENTS = {
    "node": "a node name such as /controller_server",
    "name": "the dotted parameter name such as FollowPath.plugin",
    "profile": "one system this atlas covers, by its slug, as overview lists them",
    "min_severity": f"the lowest severity shown: {', '.join(SEVERITY_ORDER)}",
    "only": "one drift rule id such as param.shadowed",
    "limit": "how many rows to return",
    "selector": "a name such as /planner_server, /scan or base_link, a full id, a finding "
    "fingerprint or id, or a rule id",
    "question": 'plain words, for example "which nodes publish cmd_vel"',
    "source": "where to start: a name such as /scan_filter, or a full id",
    "target": "where to end: a name such as /cmd_vel, or a full id",
    "kind": "one edge kind such as publishes or has_param",
    "direction": "out, in or both",
    "focus": "a node or channel such as /scan_filter, and only its neighbourhood is shown",
    "depth": "how many hops around focus",
    "wrappers": "also list library endpoints such as tf2_ros listeners",
    "want": f"what was wanted: {', '.join(item.value for item in Want)}",
    "owner": "one node, symbol or file the blind spots belong to",
    "bag": "one recording, by its directory name",
    "model": "one robot model file, such as urdf/robot.urdf",
    "to": "the target release, such as lyrical",
    "start": "the current release, also accepted as from. default ROS_DISTRO, then the one "
    "release installed under /opt/ros",
    "package": "one package of the workspace",
    "workspace": "the workspace directory, default the server's working directory",
}
"""What each tool argument means, written into the tool schema. One argument name has one
meaning across the tools, and it is the command line's word: `only` is `--only`, `source`
and `target` are the two ends `rosio path` takes. `--from` is the one word a python name
cannot spell, so `migrate` takes `start` and accepts `from` through `SPELLINGS`."""

SPELLINGS = {"migrate": {"from": "start"}}
"""Second spellings a tool accepts for one of its arguments, renamed before validation."""

TRANSFORM_ARGUMENTS = {
    "source": "the frame whose pose is wanted, such as laser_link",
    "target": "the frame the pose is stated in, such as base_link",
}


def described(function: Any) -> Any:
    """The function with each argument's description in its annotation, which is where the
    mcp package reads a schema description from. pydantic arrives with mcp, so it is
    imported here and never at module import time."""
    from typing import Annotated

    from pydantic import Field

    words = {**ARGUMENTS, **TRANSFORM_ARGUMENTS} if function.__name__ == "transform" else ARGUMENTS
    hints = {"str": str, "int": int, "bool": bool}
    annotations = {}
    for name, hint in function.__annotations__.items():
        kind = hints.get(hint, hint) if isinstance(hint, str) else hint
        if name in words:
            kind = Annotated[kind, Field(description=words[name])]
        annotations[name] = kind
    function.__annotations__ = annotations
    return function


def _takes(tool: str, takes: list[str]) -> str:
    second = {name: spelled for spelled, name in SPELLINGS.get(tool, {}).items()}
    named = [f"{name} (or {second[name]})" if name in second else name for name in takes]
    return f"{tool} takes {', '.join(named) or 'no arguments'}"


def doubled(tool: str, arguments: dict) -> str:
    """The refusal for one argument given under both of its spellings, or nothing. Renaming
    both would keep whichever the caller's JSON happened to put last."""
    twice = [
        f"{name} was given twice, as {name} and as {spelled}"
        for spelled, name in sorted(SPELLINGS.get(tool, {}).items())
        if spelled in arguments and name in arguments
    ]
    return f"{tool} was not run: {', '.join(twice)}" if twice else ""


def strangers(tool: str, arguments: dict, takes: dict[str, bool]) -> str:
    """The refusal for arguments a tool does not take, or nothing. `takes` maps each argument
    to whether it is required. A stray argument the schema would drop is refused instead,
    because a filter that was silently ignored answers a different question."""
    unknown = sorted(set(arguments) - set(takes))
    if not unknown:
        return ""
    missing = [
        f"{name} is missing" for name, needed in takes.items() if needed and name not in arguments
    ]
    head = f"{', '.join(missing)}, and it" if missing else "it"
    return f"{tool} was not run: {head} takes no {', '.join(unknown)}. {_takes(tool, list(takes))}"


def plain(tool: str, error: Exception, arguments: dict, takes: list[str]) -> str:
    """One sentence for arguments the schema refused, in place of the validator's report."""
    problems = []
    for row in error.errors():
        field_name = ".".join(str(part) for part in row.get("loc", ())) or "the arguments"
        if row.get("type") == "missing":
            problems.append(f"{field_name} is missing")
        else:
            problems.append(f"{field_name} {str(row.get('msg', 'is not valid')).lower()}")
    unknown = sorted(set(arguments) - set(takes))
    stranger = f", and it takes no {', '.join(unknown)}" if unknown else ""
    return f"{tool} was not run: {', '.join(problems)}{stranger}. {_takes(tool, takes)}"


# ---------------------------------------------------------------------------
# the mcp package, imported only where it is needed


def _server_class() -> Any:
    try:
        from mcp.server.mcpserver import MCPServer

        return MCPServer
    except ImportError:
        pass
    try:
        from mcp.server.fastmcp import FastMCP
    except ImportError as exc:
        raise UsageError(_mcp_note()) from exc
    return FastMCP


def _mcp_note() -> str:
    """Which of the two happened: no mcp on the path, or one that carries no server class.

    Telling somebody to install what they already have costs them the one message they get,
    so the version that is present is read and stated with the range that would work.
    """
    from importlib.metadata import PackageNotFoundError, version

    try:
        found = version("mcp")
    except PackageNotFoundError:
        return MCP_MISSING
    return (
        f"mcp {found} is installed and exposes neither MCPServer nor FastMCP, so there is no "
        f"server to build. rosio needs {MCP_EXTRA}. "
        "move it into that range with: pip install 'rosio[mcp]'"
    )


def _tool_error() -> type[Exception]:
    try:
        from mcp.server.mcpserver.exceptions import ToolError
    except ImportError:
        from mcp.server.fastmcp.exceptions import ToolError
    return ToolError


def build(source: tools.AtlasSource) -> Any:
    """The mcp server with the tool set registered.

    An atlas that is there is read here, so a broken file fails before a transport is
    opened. One that is not there yet does not stop the server: every tool that needs it
    answers with the command that builds it, and the next call after that build is answered
    from it, because nothing about the server depends on the file until a tool reads it.

    A refusal reaches the caller as its own sentence. A tool that raises one of ours, and
    arguments the schema refuses, would otherwise come back as the package's generic line
    or as the validator's multi line report with a link in it.
    """
    if source.present:
        source.atlas()
    factory = _server_class()
    refusal = _tool_error()
    functions = [described(function) for function in bindings(source)]
    takes = {item.__name__: list(inspect.signature(item).parameters) for item in functions}
    needed = {
        item.__name__: {
            name: row.default is inspect.Parameter.empty
            for name, row in inspect.signature(item).parameters.items()
        }
        for item in functions
    }

    class Server(factory):
        async def call_tool(self, name: str, arguments: dict, *rest: Any, **named: Any) -> Any:
            if name in needed:
                refused = doubled(name, arguments or {})
                if refused:
                    raise refusal(refused)
                spelled = SPELLINGS.get(name, {})
                arguments = {
                    spelled.get(key, key): value for key, value in (arguments or {}).items()
                }
                refused = strangers(name, arguments, needed[name])
                if refused:
                    raise refusal(refused)
            try:
                return await super().call_tool(name, arguments, *rest, **named)
            except Exception as exc:
                cause = exc.__cause__
                if isinstance(cause, RosioError):
                    raise refusal(str(cause)) from cause
                if cause is not None and hasattr(cause, "errors") and name in takes:
                    raise refusal(plain(name, cause, arguments or {}, takes[name])) from cause
                raise

    wanted = {"name": SERVER_NAME, "instructions": INSTRUCTIONS, "version": __version__}
    accepted = inspect.signature(factory).parameters
    server = Server(**{key: value for key, value in wanted.items() if key in accepted})
    # the text is the answer, and a structured copy of it beside it doubles what a small
    # context window has to hold
    structured = "structured_output" in inspect.signature(server.add_tool).parameters
    for function in functions:
        extra = {"structured_output": False} if structured else {}
        server.add_tool(function, name=function.__name__, description=_summary(function), **extra)
    return server


def _summary(function: Any) -> str:
    return " ".join((function.__doc__ or "").split())


# ---------------------------------------------------------------------------
# transports


def run(options: Options) -> None:
    """Serve until the caller stops it. Nothing is printed on stdout, which the stdio
    transport owns."""
    options.check()
    server = build(
        tools.AtlasSource(options.atlas, confined=options.http, workspace=options.workspace)
    )
    if not options.http:
        _run_async(server.run_stdio_async)
        return
    try:
        import uvicorn
    except ImportError as exc:
        raise UsageError(UVICORN_MISSING) from exc

    config = uvicorn.Config(
        http_app(server, options), host=options.host, port=options.port, log_level="warning"
    )
    uvicorn.Server(config).run()


def _run_async(entry: Any) -> None:
    import anyio

    anyio.run(entry)


def http_app(server: Any, options: Options) -> Any:
    """The streamable HTTP app, with the key check in front of it.

    The transport options moved between mcp releases, so they are passed to whichever of the
    two places accepts them rather than assumed.
    """
    wanted = {
        "streamable_http_path": options.path,
        "stateless_http": options.stateless,
        "host": options.host,
        "port": options.port,
    }
    accepted = inspect.signature(server.streamable_http_app).parameters
    settings = getattr(server, "settings", None)
    for name, value in wanted.items():
        if name not in accepted and settings is not None and hasattr(settings, name):
            setattr(settings, name, value)
    app = server.streamable_http_app(**{k: v for k, v in wanted.items() if k in accepted})
    return guarded(app, options.api_key)


def guarded(app: Any, key: str) -> Any:
    """An ASGI wrapper that refuses a request without the key. A bearer token and an
    x-api-key header are both accepted, because clients disagree about which one to send."""
    if not key:
        return app

    async def gate(scope: dict, receive: Any, send: Any) -> None:
        if scope.get("type") != "http":
            await app(scope, receive, send)
            return
        if presented(scope.get("headers") or [], key):
            await app(scope, receive, send)
            return
        await send(
            {
                "type": "http.response.start",
                "status": 401,
                "headers": [
                    (b"content-type", b"text/plain; charset=utf-8"),
                    (b"www-authenticate", b'Bearer realm="rosio"'),
                ],
            }
        )
        await send(
            {
                "type": "http.response.body",
                "body": b"rosio: send the key as Authorization: Bearer <key> or x-api-key\n",
            }
        )

    return gate


def presented(headers: Any, key: str) -> bool:
    offered: list[str] = []
    for name, value in headers:
        header = name.decode("latin-1").lower() if isinstance(name, bytes) else str(name).lower()
        text = value.decode("latin-1") if isinstance(value, bytes) else str(value)
        if header == "authorization":
            head, _, token = text.partition(" ")
            offered.append(token.strip() if head.lower() == "bearer" else text.strip())
        elif header in ("x-api-key", "api-key"):
            offered.append(text.strip())
    return any(hmac.compare_digest(item, key) for item in offered)


__all__ = [
    "DEFAULT_PATH",
    "DEFAULT_PORT",
    "INSTRUCTIONS",
    "LOOPBACK",
    "MCP_EXTRA",
    "MCP_INSTALL",
    "MCP_MAX",
    "MCP_MIN",
    "MCP_MISSING",
    "SPELLINGS",
    "UVICORN_EXTRA",
    "Options",
    "bindings",
    "build",
    "doubled",
    "guarded",
    "http_app",
    "mcp_problem",
    "presented",
    "run",
    "strangers",
]
