"""Intermediate records of the declared layer.

A record is what one source said before identity was decided, so nothing here holds an
atlas id. The launch walk produces these, the parameter chain and the collectors consume
them. Keeping the shape in one place is what lets the launch resolver and the parameter
resolver be written against the same contract.

Closed vocabularies are validated on construction. A collector that needs a new member
adds it here first, so a typo cannot travel silently into the atlas.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from rosio.errors import ContractError

ROOTS = ("src", "install", "runtime", "bag")

NODE_FORMS = ("node", "lifecycle_node", "composable_node", "process")
"""A composable node has no executable. A process is an ExecuteProcess that turns out to
be a ros node, which is common enough on real stacks to need its own form: its name,
namespace and parameters live in argv rather than in launch keywords."""

PARAM_SOURCE_KINDS = (
    "repo_file",
    "external_file",
    "inline_dict",
    "rewritten_file",
    "ros_args_file",
    "ros_args_assign",
)
"""`external_file` is a parameters file produced by a process outside the tracked trees,
typically a temp file written by a launcher script. Its content is real and its path is
worthless, so provenance has to terminate at whatever wrote it."""

GATE_MECHANISMS = (
    "OnProcessExit",
    "OnProcessStart",
    "OnStateTransition",
    "OnShutdown",
    "TimerAction",
    "IfCondition",
    "UnlessCondition",
    "OpaqueFunction",
    "filesystem_probe",
    "environment_probe",
)

REMAP_SCOPES = ("node", "global", "urdf_plugin")

WALK_PHASES = ("import", "describe", "visit", "argument", "include")


def _check(value: str, allowed: tuple[str, ...], what: str) -> str:
    if value not in allowed:
        raise ContractError(f"{what} {value!r} is not one of {allowed}")
    return value


@dataclass(frozen=True)
class Site:
    """Where a record was read. `locator` is a stable handle inside the file, a yaml key
    path or an action path, never an ordinal. `line_hint` is advisory and never compared."""

    path: str
    locator: str = ""
    line_hint: int | None = None
    root: str = "src"

    def __post_init__(self) -> None:
        _check(self.root, ROOTS, "site root")

    def with_locator(self, locator: str) -> Site:
        return Site(self.path, locator, self.line_hint, self.root)


@dataclass(frozen=True)
class Remap:
    frm: str
    to: str
    scope: str = "node"
    site: Site | None = None

    def __post_init__(self) -> None:
        _check(self.scope, REMAP_SCOPES, "remap scope")


@dataclass(frozen=True)
class ParamSource:
    """One item of a node's ordered parameter inputs. Order is load bearing: several
    launch files on real stacks append an inline dict after a file precisely so it wins."""

    order: int
    kind: str
    path: str | None = None
    source_path: str | None = None
    root_key: Any = None
    rewrites: tuple[tuple[str, Any], ...] = ()
    values: tuple[tuple[str, Any], ...] = ()
    produced_by: str | None = None
    site: Site | None = None
    note: str = ""
    text: str = ""
    """The item as the launch file spells it, which tells two temp files of one node apart."""

    def __post_init__(self) -> None:
        _check(self.kind, PARAM_SOURCE_KINDS, "parameter source kind")
        if self.kind in ("repo_file", "external_file", "ros_args_file") and not self.path:
            raise ContractError(f"parameter source {self.kind} needs a path")
        if self.kind == "rewritten_file" and not self.source_path:
            raise ContractError("a rewritten parameters file needs its source_path")


@dataclass(frozen=True)
class Gate:
    """One reason a node may not be reachable even though it is declared. Reachability is
    a trail of these, never a boolean: two nodes that both exist can still never coexist."""

    mechanism: str
    expression: str = ""
    period: float | None = None
    target: str = ""
    outcome: str = ""
    commands: tuple[str, ...] = ()
    command_role: str = "condition"
    site: Site | None = None

    def __post_init__(self) -> None:
        _check(self.mechanism, GATE_MECHANISMS, "gate mechanism")


@dataclass(frozen=True)
class FileRead:
    path: str
    role: str
    site: Site | None = None


@dataclass(frozen=True)
class ResolvedNode:
    """One node as the executed launch description described it.

    `name` is None when launch did not state one, which is the common case for third party
    nodes, and `name_source` says why. `entity_path` is the ordered action trail used to
    build the launchent id, so it must contain only attributes of the actions themselves.
    """

    pkg: str
    executable: str
    namespace: str
    form: str = "node"
    name: str | None = None
    name_source: str = "unknown"
    container: str | None = None
    plugin_class: str | None = None
    remaps: tuple[Remap, ...] = ()
    params: tuple[ParamSource, ...] = ()
    argv: tuple[str, ...] = ()
    file_reads: tuple[FileRead, ...] = ()
    command_blinds: tuple[tuple[str, str], ...] = ()
    entity_path: tuple[str, ...] = ()
    gates: tuple[Gate, ...] = ()
    conditions: tuple[str, ...] = ()
    role_content: tuple[str, ...] = ()
    site: Site | None = None
    attrs: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        _check(self.form, NODE_FORMS, "node form")
        if self.form == "composable_node" and self.executable:
            raise ContractError("a composable node has no executable")


@dataclass(frozen=True)
class LaunchInclude:
    path: str
    args: tuple[tuple[str, str], ...] = ()
    site: Site | None = None
    resolved: bool = True
    reason: str = ""
    entity_path: tuple[str, ...] = ()


@dataclass(frozen=True)
class WalkException:
    """A launch file that raised. Recorded per file and carried on, because one entry
    point that raises at description build time must not empty the whole layer.

    `kept` names the action the walk still recorded without the part that failed, such as
    `Node(rviz2)` whose `-d` argument names an unbuilt package. Empty means the failure
    dropped whatever the action would have started."""

    path: str
    phase: str
    exc_type: str
    message: str
    kept: str = ""

    def __post_init__(self) -> None:
        _check(self.phase, WALK_PHASES, "walk phase")


@dataclass(frozen=True)
class ArgDeclaration:
    name: str
    default: str | None = None
    description: str = ""
    site: Site | None = None


@dataclass
class LaunchWalk:
    """The result of executing one entry point with one argument set.

    `predicates` records the machine dependent inputs that decided a branch, for example
    whether a serialised map existed. Two boxes that disagree only about a file on disk
    must show an input difference and not drift.
    `sandboxed` is parent execution metadata, outside the child protocol.
    """

    entry_path: str
    profile: str = ""
    entry_pkg: str | None = None
    args: tuple[tuple[str, str], ...] = ()
    env: tuple[tuple[str, str], ...] = ()
    nodes: list[ResolvedNode] = field(default_factory=list)
    includes: list[LaunchInclude] = field(default_factory=list)
    arg_declarations: list[ArgDeclaration] = field(default_factory=list)
    exceptions: list[WalkException] = field(default_factory=list)
    predicates: dict[str, bool] = field(default_factory=dict)
    loadable: bool = True
    strategy: str = "executed"
    notes: list[str] = field(default_factory=list)
    sandboxed: bool = field(default=False, compare=False, repr=False)

    def __post_init__(self) -> None:
        _check(self.strategy, ("executed", "ast", "refused"), "walk strategy")


@dataclass(frozen=True)
class RosArgs:
    """What a `--ros-args` argv tail said. ExecuteProcess nodes carry their whole identity
    here, so an argv that is not parsed becomes a node that looks observed only."""

    node_name: str | None = None
    namespace: str | None = None
    params_files: tuple[str, ...] = ()
    assignments: tuple[tuple[str, str], ...] = ()
    remaps: tuple[tuple[str, str], ...] = ()
    log_level: str | None = None
    enclave: str | None = None
    leftover: tuple[str, ...] = ()
    inputs: tuple[tuple[str, Any], ...] = ()
    """Parameter inputs in argv order, ('params_file', path) and ('assign', (name, value))
    interleaved, so precedence is read off the argv instead of reconstructed from the two
    flat tuples above."""


@dataclass(frozen=True)
class ScrapedEntrypoint:
    """A `ros2 launch` or `ros2 run` invocation found in a script, a unit file or launcher
    source. This is how a real profile is recovered: no single launch file describes a
    running system."""

    verb: str
    pkg: str
    target: str
    args: tuple[tuple[str, str], ...] = ()
    argv: tuple[str, ...] = ()
    site: Site | None = None
    interpolated: tuple[str, ...] = ()
    confidence: str = "resolved"

    def __post_init__(self) -> None:
        _check(self.verb, ("launch", "run"), "scraped verb")


@dataclass(frozen=True)
class ParamOverride:
    """One key written into a parameter document by code rather than by yaml."""

    key_path: tuple[str, ...]
    value: Any = None
    resolved: bool = True
    expr: str = ""
    site: Site | None = None


@dataclass(frozen=True)
class ParamPipeline:
    """A parameters file assembled outside launch, by a launcher module or a script, and
    then handed to launch as a path.

    This is how real robots are configured and it is invisible to every existing tool: the
    file the running stack was given is a temp file that exists in no repository, so
    provenance has to terminate at the base document, the overlays and the symbol that
    performed the writes.
    """

    output_token: str
    base: str | None = None
    overlays: tuple[str, ...] = ()
    overrides: tuple[ParamOverride, ...] = ()
    applied_by: str = ""
    site: Site | None = None
    complete: bool = True
    notes: tuple[str, ...] = ()


@dataclass
class DeclaredBundle:
    """Carried by the collect context so the declared collectors can hand each other
    records without one of them reading another layer's atlas output."""

    walks: list[LaunchWalk] = field(default_factory=list)
    scraped: list[ScrapedEntrypoint] = field(default_factory=list)
    pipelines: list[ParamPipeline] = field(default_factory=list)
