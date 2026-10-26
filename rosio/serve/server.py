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
from rosio.errors import UsageError
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

    def drift(severity: str = "", rule: str = "", profile: str = "", limit: int = 5) -> str:
        """Findings ready to quote, each with one sentence and the line to edit. severity is
        error, warn or info and means that level or worse. rule is one rule id such as
        param.shadowed. profile narrows to one system."""
        return tools.drift(
            source.atlas(), severity=severity, rule=rule, profile=profile, limit=limit
        )

    def explain(entity: str, limit: int = 8) -> str:
        """One entity in full: its layers, facts, conflicts, effective values and
        provenance. entity is a name such as /planner_server, /scan or base_link, or a
        finding id."""
        return tools.explain(source.atlas(), entity, limit=limit)

    def query(question: str, budget: int = 8) -> str:
        """A scoped subgraph for a question in plain words, for example "which nodes
        publish cmd_vel". Matched by name and kind, with no language model, and it says so
        when it did not understand. budget is how many entities to return."""
        return tools.query(source.atlas(), question, budget=budget)

    def path(a: str, b: str) -> str:
        """The shortest path between two entities, with the kind and direction of every hop,
        for example from a node to a frame."""
        return tools.path(source.atlas(), a, b)

    def neighbours(entity: str, kind: str = "", direction: str = "both") -> str:
        """What one entity is connected to, counted by edge kind. kind narrows to one edge
        kind such as publishes or has_param, direction is out, in or both."""
        return tools.neighbours(source.atlas(), entity, kind=kind, direction=direction)

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
        hollow answer from a complete one. want is param_value, topic_name, frame_id,
        node_name or other. owner narrows to one node, symbol or file."""
        return tools.unresolved(source.atlas(), want=want, owner=owner)

    def stamps(bag: str = "", limit: int = 8) -> str:
        """The header stamp verdict of every recording in the atlas, with the topics whose
        clocks step backwards, freeze, jump or sit years off. bag narrows to one recording."""
        return tools.stamps(source.atlas(), bag=bag, limit=limit)

    store = source

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
        release such as lyrical, start the current one (default ROS_DISTRO). Lists the by hand
        changes with file and line, and the one command that performs the exact rewrites."""
        return tools.migrate(workspace, to, start=start, package=package)

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


def build(source: tools.AtlasSource) -> Any:
    """The mcp server with the tool set registered. The atlas is read here, so a broken
    path fails before a transport is opened."""
    source.atlas()
    factory = _server_class()
    wanted = {"name": SERVER_NAME, "instructions": INSTRUCTIONS, "version": __version__}
    accepted = inspect.signature(factory).parameters
    server = factory(**{key: value for key, value in wanted.items() if key in accepted})
    # the text is the answer, and a structured copy of it beside it doubles what a small
    # context window has to hold
    plain = "structured_output" in inspect.signature(server.add_tool).parameters
    for function in bindings(source):
        extra = {"structured_output": False} if plain else {}
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
    server = build(tools.AtlasSource(options.atlas))
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
    "UVICORN_EXTRA",
    "Options",
    "bindings",
    "build",
    "guarded",
    "http_app",
    "presented",
    "run",
]
