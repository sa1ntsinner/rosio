"""The coded layer for rclpy sources, read with the stdlib ast module.

What this module claims is what one python file says, and nothing else. It never executes
the source and never asks a running system. Three rules shape every record it produces.

* A name resolves or it does not. A literal resolves, and so does a local or a member bound
  to a parameter default, an f string over such a value, and a module level constant.
  Anything else (an environment variable, a message field, a constructor argument) becomes
  an unresolved entry carrying the raw source text and what was wanted. A topic is never
  invented.
* The coded layer does not know the fully qualified node name, because the namespace lives
  in the launch file. Facts hang off the symbol and the executable, and a relative endpoint
  name is carried raw for the join to bind.
* QoS is what the source states. A depth the source never wrote is not a default here, it is
  a null, because a fabricated depth of 10 manufactures drift against the observed layer.

Endpoints created inside a known wrapper (diagnostic_updater, message_filters, tf2_ros) are
marked with the wrapper symbol rather than resolved away, so the join can mark them
via_known_wrapper instead of reporting a node that publishes nothing. The rows come from
rosio/resolve/wrappers.py, which is the single table, so nothing here restates one.
"""

from __future__ import annotations

import ast
import operator as _op
import re
from collections.abc import Sequence
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

from rosio import ids, model, workspace
from rosio.collect import CollectContext, register
from rosio.errors import CollectorError, ContractError
from rosio.resolve import parse_python, wrappers

LANGUAGE = "python"
LAYER = model.Layer.CODED.value
NAME = "py_code"
SUFFIXES = (".py",)

WANTS = (
    "topic_name",
    "service_name",
    "action_name",
    "node_name",
    "param_name",
    "param_value",
    "frame_id",
    "other",
)

REFUSED_PY_SOURCE = "py_parse_error"

REFUSED_WANTS = (model.Want.NODE_NAME.value, model.Want.TOPIC_NAME.value)
"""What a python source this reader could not parse took out of the rate with it. A
refusal recorded under `other` left both of these wants with a perfect rate over a smaller
denominator, which is the shape AGENTS section 13 point 3 refuses."""

_NODE_BASES = {
    "rclpy.node.Node": "node",
    "rclpy.lifecycle.LifecycleNode": "lifecycle_node",
    "rclpy.lifecycle.Node": "lifecycle_node",
    "rclpy.lifecycle.node.LifecycleNode": "lifecycle_node",
}

_QOS_PROFILE_DEFAULTS = {
    "history": "keep_last",
    "reliability": "reliable",
    "durability": "volatile",
}
"""What `rclpy.qos.QoSProfile(...)` holds for a policy it is not given: history keep_last, the
rest from `rmw_qos_profile_default` (rclpy/qos.py, `QoSProfile.__init__`)."""

_QOS_PRESETS = {
    "qos_profile_sensor_data": "sensor_data",
    "qos_profile_system_default": "system_default",
    "qos_profile_services_default": "services_default",
    "qos_profile_parameters": "parameters",
    "qos_profile_parameter_events": "parameter_events",
    "qos_profile_action_status_default": "action_status_default",
    "SENSOR_DATA": "sensor_data",
    "SYSTEM_DEFAULT": "system_default",
    "SERVICES_DEFAULT": "services_default",
    "PARAMETERS": "parameters",
    "PARAMETER_EVENTS": "parameter_events",
    "ACTION_STATUS_DEFAULT": "action_status_default",
}

_POLICY_FIELDS = {
    "ReliabilityPolicy": "reliability",
    "QoSReliabilityPolicy": "reliability",
    "DurabilityPolicy": "durability",
    "QoSDurabilityPolicy": "durability",
    "HistoryPolicy": "history",
    "QoSHistoryPolicy": "history",
    "LivelinessPolicy": "liveliness",
    "QoSLivelinessPolicy": "liveliness",
}

_PARAM_TYPES = {
    "BOOL": "bool",
    "INTEGER": "int",
    "DOUBLE": "double",
    "STRING": "string",
    "BYTE_ARRAY": "byte_array",
    "BOOL_ARRAY": "bool_array",
    "INTEGER_ARRAY": "int_array",
    "DOUBLE_ARRAY": "double_array",
    "STRING_ARRAY": "string_array",
    "NOT_SET": None,
}

_ENDPOINT_CALLS = {
    "create_publisher": ("publisher", 1, 0, 2, "topic", "msg_type"),
    "create_lifecycle_publisher": ("publisher", 1, 0, 2, "topic", "msg_type"),
    "create_subscription": ("subscriber", 1, 0, 3, "topic", "msg_type"),
    "create_service": ("service_server", 1, 0, None, "srv_name", "srv_type"),
    "create_client": ("service_client", 1, 0, None, "srv_name", "srv_type"),
}
"""call name -> (endpoint kind, name index, type index, qos index, name keyword, type keyword).
The service APIs spell the name `srv_name` and the type `srv_type`, and take their qos by
keyword only."""

_ACTION_CALLS = {
    "ActionServer": ("action_server", 2, 1),
    "ActionClient": ("action_client", 2, 1),
    "create_action_server": ("action_server", 2, 1),
    "create_action_client": ("action_client", 2, 1),
}

_RCLPY_ACTIONS = frozenset({"rclpy.action.ActionClient", "rclpy.action.ActionServer"})
"""Read as one action endpoint with its type, as the C++ reader reads `rclcpp_action`, and
not as the five endpoints rcl_action derives from the name."""

_WRAPPER_KEYWORDS = {
    "message_filters.Subscriber": ("topic", "msg_type", "qos_profile"),
}
"""(name, type, qos) keyword of every python wrapper row whose caller supplies the name. A
keyword argument is the same argument, and read positionally it states none. Signature on
humble: `message_filters/__init__.py:61`. An rclpy action is read by `_action`, never here."""

_LOOKUP_CALLS = (
    "lookup_transform",
    "can_transform",
    "lookup_transform_async",
    "wait_for_transform_async",
    "lookup_transform_full",
    "can_transform_full",
    "lookup_transform_full_async",
    "wait_for_transform_full_async",
)
"""tf2_ros.Buffer reads. The `_full` forms take (target, target_time, source, source_time,
fixed_frame), so the source is the third argument."""

_NODE_ARGUMENTS = frozenset({"self", "node", "parent", "parent_node"})
_NODE_USERS = frozenset(
    {
        "spin",
        "spin_once",
        "spin_until_future_complete",
        "add_node",
        "remove_node",
        "destroy_node",
        "isinstance",
        "issubclass",
        "super",
        "print",
        "id",
        "getattr",
        "setattr",
        "hasattr",
        "type",
        "repr",
        "str",
        "ref",
    }
)
"""Calls that take a node and create nothing on it, so handing it to them hides no name."""

_PLAIN_BASES = frozenset({"abc", "typing", "enum", "dataclasses", "object", "rclpy"})

NAME_LITERAL = re.compile(r"[~/]?[A-Za-z_]\w*(?:/[A-Za-z_]\w*)*/?", re.ASCII)
"""A string literal shaped like a ROS name, which a remap could be written against."""

_FILESYSTEM_PATH = re.compile(
    r"/(bin|boot|dev|etc|home|lib|media|mnt|opt|proc|root|run|srv|sys|tmp|usr|var)/."
)


def is_name_literal(value: str) -> bool:
    """A string literal shaped like a ROS name. A path under a filesystem root (`/tmp/pose`,
    `/dev/ttyUSB0`, a home directory) is a file, and would name the machine in compared data."""
    return (
        len(value) <= 200
        and NAME_LITERAL.fullmatch(value) is not None
        and not _FILESYSTEM_PATH.match(value)
    )


_CREATE_NODE = "rclpy.create_node"
AUTO_DECLARE_OPTIONS = frozenset(
    {"automatically_declare_parameters_from_overrides", "allow_undeclared_parameters"}
)
"""The functional form of a node. Eight sites on the reference stack build a node this way,
and without it the literal name the call states is lost and the endpoints made on the returned
object belong to nothing."""

_ALIASABLE = frozenset(
    {
        *_ENDPOINT_CALLS,
        *_LOOKUP_CALLS,
        "create_timer",
        "declare_parameter",
        "declare_parameters",
        "get_parameter",
        "get_parameter_or",
        "sendTransform",
    }
)
"""APIs that real sources bind to a short local name, `d = self.declare_parameter`."""

_BINOPS = {
    ast.Add: _op.add,
    ast.Sub: _op.sub,
    ast.Mult: _op.mul,
    ast.Div: _op.truediv,
    ast.FloorDiv: _op.floordiv,
    ast.Mod: _op.mod,
    ast.Pow: _op.pow,
}

_STR_METHODS = frozenset(
    {"strip", "lstrip", "rstrip", "lower", "upper", "replace", "removeprefix", "removesuffix"}
)
"""Pure string methods, applied when the receiver and every argument are resolved."""

_CAST_CALLS = {"str": str, "int": int, "float": float, "bool": bool, "list": list, "tuple": tuple}

_RESOLVABLE_BY = {
    "branch_dependent": ("observed",),
    "runtime_attribute_default": ("observed",),
    "environment_variable": ("observed",),
    "message_field": ("observed", "recorded"),
    "argv_value": ("declared", "observed"),
    "call_result": ("observed",),
    "loop_variable": ("observed",),
    "unbound_name": ("observed",),
}

_DETAILS = {
    "frame_empty": "the literal names no frame, so tf2 refuses the transform at run time",
    "branch_dependent": (
        "the two arms of a branch bind this name to different values, so the source states"
        " one value per branch and no value that holds either way"
    ),
    "runtime_attribute_default": (
        "the default is an attribute of an object built at run time, so neither the value nor"
        " the parameter type is stated. The name is still resolved"
    ),
    "environment_variable": (
        "the name comes from the process environment, and the endpoint exists only when that"
        " variable is set and not empty. Neither the name nor the existence is a property of"
        " the source"
    ),
    "message_field": (
        "the frame is whatever arrived on the topic, so the code states no frame. Only a live"
        " system or the publisher of that topic can name it"
    ),
    "argv_value": (
        "the value is a constructor argument filled by the caller, so it is not a property of"
        " the source. The identity has to come from the declared or observed layer"
    ),
    "call_result": "the value is the result of a call this reader does not evaluate",
    "loop_variable": "the value is an element of an iterable that does not resolve statically",
    "unbound_name": "the name is not bound anywhere this reader can see",
}

_PROSE_KEYS = frozenset(
    {
        "note",
        "detail",
        "count_detail",
        "node_id_reason",
        "topic_id_reason",
        "qos_reason",
        "line_hint",
    }
)
"""Keys that carry advisory text or an advisory line. They never enter a comparison."""


# ---------------------------------------------------------------------------
# values


@dataclass
class Val:
    """One expression, after the reader tried to resolve it.

    `state` is the vocabulary the records use: literal, resolved_from_parameter,
    resolved_from_fstring, resolved_from_constant, resolved_from_module_literal, unresolved.
    """

    expr: str = ""
    known: bool = False
    value: Any = None
    state: str = "unresolved"
    from_parameter: str | None = None
    parameter_suffix: str = ""
    from_constant: str | None = None
    binding: str | None = None
    reason: str = ""
    env_var: str | None = None
    wrapper: str | None = None
    condition: str | None = None
    guard: tuple[str, bool] | None = None
    qos: dict[str, Any] | None = None
    qos_target: str | None = None
    node: ast.expr | None = None

    def carry(self, expr: str) -> Val:
        return replace(self, expr=expr)


_UNKNOWN = Val()


@dataclass
class Ctx:
    """Where the reader currently is: which symbol, under which condition, in which scope."""

    symbol: str
    scope: dict[str, Val] = field(default_factory=dict)
    fields: dict[str, dict[str, Val]] = field(default_factory=dict)
    condition: str | None = None
    args: frozenset[str] = frozenset()
    guard: tuple[str, bool] | None = None
    """The parameter and the value it needs, when `condition` is exactly one parameter."""

    def under(self, condition: str | None, guard: tuple[str, bool] | None = None) -> Ctx:
        if not condition:
            return Ctx(self.symbol, self.scope, self.fields, self.condition, self.args, self.guard)
        return Ctx(self.symbol, self.scope, self.fields, condition, self.args, guard)


# ---------------------------------------------------------------------------
# helpers


def _frame_val(val: Val) -> Val:
    """A literal that names no frame (`''`, `'/'`) is a blind spot, never `frame:`."""
    if val.known and isinstance(val.value, str) and not val.value.strip().lstrip("/"):
        return Val(expr=val.expr or repr(val.value), reason="frame_empty")
    return val


def _guard_view(guard: tuple[str, bool] | None) -> dict[str, Any]:
    if guard is None:
        return {}
    return {"guard_parameters": [{"name": guard[0], "value": guard[1]}]}


def _is_docstring(node: ast.stmt) -> bool:
    return isinstance(node, ast.Expr) and isinstance(node.value, ast.Constant)


def base_key(package: str, qualname: str) -> str:
    """The workspace key of one node base class, package scoped."""
    return f"{package}|{qualname}"


def _dotted(node: ast.AST) -> str:
    """The dotted spelling of a Name or Attribute chain, empty for anything else."""
    parts: list[str] = []
    current = node
    while isinstance(current, ast.Attribute):
        parts.append(current.attr)
        current = current.value
    if isinstance(current, ast.Name):
        parts.append(current.id)
    elif isinstance(current, ast.Call):
        parts.append("()")
    else:
        return ""
    return ".".join(reversed(parts))


def _line(node: ast.AST) -> int | None:
    return getattr(node, "lineno", None)


def _period_form(period: ast.expr) -> dict[str, Any]:
    """How a timer period follows the parameter it is read from: `1.0 / rate` is the inverse
    scaled by the literal numerator, `period` itself is direct, anything else is not stated."""
    if (
        isinstance(period, ast.BinOp)
        and isinstance(period.op, ast.Div)
        and isinstance(period.left, ast.Constant)
        and isinstance(period.left.value, int | float)
        and not isinstance(period.left.value, bool)
    ):
        return {"period_form": "inverse", "period_numerator": float(period.left.value)}
    if isinstance(period, ast.Name | ast.Attribute | ast.Call | ast.Subscript):
        return {"period_form": "direct"}
    return {}


def _checked(condition: str | None, name: str) -> bool:
    """Whether a guard asks `has_parameter` for this name first."""
    return bool(condition) and "has_parameter(" in condition and repr(name)[1:-1] in condition


def _is_header_frame(node: ast.AST) -> bool:
    """`<message>.header.frame_id` as an assignment target."""
    return (
        isinstance(node, ast.Attribute)
        and node.attr == "frame_id"
        and isinstance(node.value, ast.Attribute)
        and node.value.attr == "header"
    )


def _is_self(node: ast.AST) -> bool:
    return isinstance(node, ast.Name) and node.id == "self"


def _sorted_key(value: str | None) -> tuple[int, str]:
    """Sort key that puts a missing name last without comparing None to a string."""
    return (1, "") if value is None else (0, value)


_FileRead = tuple[dict[str, Any], dict[str, int], dict[str, str], list[dict[str, Any]]]


# ---------------------------------------------------------------------------
# the per file reader


class PyFile:
    """Reads one rclpy source file into the record shape the join phase consumes."""

    def __init__(
        self,
        source: str,
        *,
        path: str,
        package: str,
        package_relative_path: str,
        executables: tuple[str, ...] = (),
        extra_wrappers: dict[str, wrappers.Wrapper] | None = None,
        extra_bases: dict[str, str] | None = None,
        constructions: dict[str, list[dict[str, Any]]] | None = None,
        declarers: dict[str, tuple[int, int | None]] | None = None,
    ) -> None:
        self.source = source
        self.extra = dict(extra_wrappers or {})
        self.declarers = dict(declarers or {})
        self.bases = dict(extra_bases or {})
        self.constructions = dict(constructions or {})
        self.sites: list[dict[str, Any]] = []
        self.context: tuple[str, dict[str, Val]] | None = None
        self.construction = ""
        self.path = path
        self.package = package
        self.relpath = package_relative_path
        self.executables = tuple(sorted(executables))
        self.module = self.relpath[:-3].replace("/", ".") if self.relpath.endswith(".py") else ""
        try:
            self.tree = parse_python(source)
        except (SyntaxError, ValueError, RecursionError) as exc:
            raise CollectorError(
                f"{path} is not parseable python: {exc}", layer="coded", collector="py_code"
            ) from exc

        self.imports: dict[str, str] = {}
        self.constants: dict[str, Val] = {}
        self.node_classes: dict[str, str] = {}
        self.defined_bases: dict[str, str] = {}
        self.accessors: set[str] = set()
        self.aliases: dict[str, str] = {}
        self.members: dict[str, Val] = {}
        self.class_members: dict[str, dict[str, Val]] = {}
        self.nodes: list[dict[str, Any]] = []
        self.endpoints: list[dict[str, Any]] = []
        self.params: dict[tuple[str, str, str], dict[str, Any]] = {}
        self.param_reads: list[dict[str, Any]] = []
        self.prefix_reads: list[tuple[str, str, dict[str, Any]]] = []
        self.callbacks: list[dict[str, Any]] = []
        self.broadcasts: list[dict[str, Any]] = []
        self.lookups: list[dict[str, Any]] = []
        self.wrappers: dict[str, dict[str, Any]] = {}
        self.constructs: dict[str, list[str]] = {}
        self.node_arguments: dict[str, str] = {}
        self.class_lines: dict[str, int | None] = {}
        self.unresolved: list[dict[str, Any]] = []
        self.handoffs: set[str] = set()
        self.resolved_counts: dict[str, int] = dict.fromkeys(WANTS, 0)
        self._seen: dict[int, Val] = {}
        self._text: dict[int, str] = {}
        self._lines = [line.encode("utf-8") for line in source.splitlines(keepends=True)]
        self._target: ast.expr | None = None

    # -- naming ------------------------------------------------------------

    def _qual(self, *parts: str) -> str:
        return ".".join(part for part in (self.module, *parts) if part)

    def _resolve_dotted(self, name: str) -> str:
        """Rewrite a local spelling through the import table."""
        if not name:
            return ""
        head, _, tail = name.partition(".")
        origin = self.imports.get(head)
        if origin is None:
            return name
        return f"{origin}.{tail}" if tail else origin

    def _wrapper_of(self, dotted: str) -> wrappers.Wrapper | None:
        """The wrapper row a construction matches, from the one shared table.

        A workspace level row discovered by `wrappers.discover` is consulted first, and a bare
        spelling reaches one only inside the module that defines it, so two same named classes
        in two packages can never be confused for each other.
        """
        full = self._resolve_dotted(dotted)
        if not full:
            return None
        discovered = self.extra.get(full)
        if discovered is None and "." not in full and self.module:
            discovered = self.extra.get(f"{self.module}.{full}")
        if discovered is not None:
            return discovered
        parts = full.split(".")
        for candidate in (full, f"{parts[0]}.{parts[-1]}"):
            row = wrappers.lookup(candidate, LANGUAGE)
            # The library a row names has to be the one this spelling imports. A bare tail
            # would hand a first party class called Subscriber the endpoints of message_filters.
            if row is not None and not row.implicit and row.symbol.split(".")[0] == parts[0]:
                return row
        return None

    def _type_of(
        self, node: ast.AST | None, ctx: Ctx | None = None
    ) -> tuple[str | None, str | None]:
        if node is None:
            return None, None
        if ctx is not None and isinstance(node, ast.Name):
            held = ctx.scope.get(node.id)
            if held is not None and held.node is not None and held.node is not node:
                return self._type_of(held.node)
        if ctx is not None and not _dotted(node):
            # a message class taken out of a table row, `cfg["msg_type"]`
            found = self._eval(node, ctx)
            if found.node is not None and found.node is not node:
                return self._type_of(found.node)
        full = self._resolve_dotted(_dotted(node))
        parts = full.split(".")
        if len(parts) >= 3 and parts[-2] in ids.INTERFACE_KINDS:
            return "/".join(parts[-3:]), ids.build_type(parts[-3], parts[-2], parts[-1])
        return None, None

    def _src(self, node: ast.AST) -> str:
        """The exact source text of a node, whitespace folded, cached per node."""
        cached = self._text.get(id(node))
        if cached is not None:
            return cached
        text = self._segment(node)
        folded = " ".join(text.split())
        self._text[id(node)] = folded
        return folded

    def _segment(self, node: ast.AST) -> str:
        start = getattr(node, "lineno", None)
        end = getattr(node, "end_lineno", None)
        if start is None or end is None or end > len(self._lines):
            return ast.unparse(node) if isinstance(node, ast.AST) else ""
        first, last = start - 1, end - 1
        left, right = node.col_offset, node.end_col_offset
        if first == last:
            return self._lines[first][left:right].decode("utf-8", "replace")
        parts = [
            self._lines[first][left:],
            *self._lines[first + 1 : last],
            self._lines[last][:right],
        ]
        return b"".join(parts).decode("utf-8", "replace")

    # -- entry point -------------------------------------------------------

    def read(self) -> dict[str, Any]:
        self._collect_imports(self.tree.body)
        self._collect_constants()
        self._collect_classes()
        self._collect_accessors()
        self._collect_sites()
        self.declarers.update(self.own_declarers())
        self._module_body()
        for node in self._nested(self.tree.body):
            if isinstance(node, ast.ClassDef):
                contexts = self._contexts(node)
                if contexts:
                    self._class_in_contexts(node, contexts)
                else:
                    self._class(node)
            else:
                self._function(node, self._qual(node.name))
        return self._record()

    # -- module level ------------------------------------------------------

    def _module_body(self) -> None:
        """The statements of the module itself, which a script writes its whole node in.

        Walking only the definitions loses every endpoint, parameter and node name of a file
        that calls `rclpy.create_node` at module level, and such a file then reads as a source
        with no ros content at all rather than as one this reader could not follow.
        """
        if not self.module:
            return
        body = [
            stmt
            for stmt in self.tree.body
            if not isinstance(stmt, ast.FunctionDef | ast.AsyncFunctionDef | ast.ClassDef)
        ]
        self._stmts(body, Ctx(self.module))
        # The module pass caches a value per expression, and the constant table wraps the
        # same expressions with where they came from. Keeping the cache would hand a later
        # read the bare value and lose the constant it is spelled with.
        self._seen.clear()

    def _collect_imports(self, body: list[ast.stmt]) -> None:
        for node in ast.walk(ast.Module(body=body, type_ignores=[])):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    # `import a.b` binds `a` to `a`, and `import a.b as c` binds `c` to `a.b`
                    if alias.asname:
                        self.imports[alias.asname] = alias.name
                    else:
                        head = alias.name.split(".")[0]
                        self.imports[head] = head
            elif isinstance(node, ast.ImportFrom):
                base = self._import_base(node)
                if not base:
                    continue
                for alias in node.names:
                    self.imports[alias.asname or alias.name] = f"{base}.{alias.name}"

    def _import_base(self, node: ast.ImportFrom) -> str:
        """The module an import names, with a relative one spelled out from this module.

        `from .helpers import Reporter` has to reach the same symbol the absolute spelling
        reaches, or one wrapper is known in one file and unknown in its neighbour.
        """
        if not node.level:
            return node.module or ""
        parts = self.module.split(".")[: -node.level] if self.module else []
        if not parts:
            return ""
        return ".".join([*parts, node.module]) if node.module else ".".join(parts)

    def _collect_constants(self) -> None:
        """Module level constants, kept with their ast so a table of rows stays readable."""
        for node in self.tree.body:
            if not isinstance(node, ast.Assign) or len(node.targets) != 1:
                continue
            target = node.targets[0]
            if not isinstance(target, ast.Name):
                continue
            val = self._static(node.value)
            if val.known:
                self.constants[target.id] = replace(
                    val, state="resolved_from_constant", from_constant=self._qual(target.id)
                )
            elif val.node is not None:
                self.constants[target.id] = val

    def _literal(self, node: ast.AST) -> Val:
        try:
            value = ast.literal_eval(node)
        except (ValueError, SyntaxError, TypeError):
            return Val(expr=self._src(node))
        return Val(expr=self._src(node), known=True, value=value, state="literal")

    def _static(self, node: ast.expr) -> Val:
        """Resolve a module level expression without recording anything."""
        text = self._src(node)
        if isinstance(node, ast.Constant):
            return Val(expr=text, known=True, value=node.value, state="literal")
        if isinstance(node, ast.Name):
            held = self.constants.get(node.id)
            return held.carry(text) if held is not None else Val(expr=text, reason="unbound_name")
        if isinstance(node, ast.Dict):
            keys = [self._static(item) if item is not None else _UNKNOWN for item in node.keys]
            values = [self._static(item) for item in node.values]
            known = all(item.known for item in keys + values)
            value = {k.value: v.value for k, v in zip(keys, values, strict=True)} if known else None
            return Val(expr=text, known=known, value=value, state="literal", node=node)
        if isinstance(node, ast.List | ast.Tuple | ast.Set):
            items = [self._static(item) for item in node.elts]
            known = all(item.known for item in items)
            value = [item.value for item in items] if known else None
            if known and isinstance(node, ast.Tuple):
                value = tuple(value)
            return Val(expr=text, known=known, value=value, state="literal", node=node)
        return self._literal(node)

    def own_declarers(self) -> dict[str, tuple[int, int | None]]:
        """Module level functions that declare a parameter under a name they are handed,
        `def parameter(node, name, default)`, as the position of the name and of the default.
        Each call of one declares the name its caller passes."""
        out: dict[str, tuple[int, int | None]] = {}
        for stmt in self.tree.body:
            if not isinstance(stmt, ast.FunctionDef):
                continue
            params = [arg.arg for arg in stmt.args.args]
            for call in ast.walk(stmt):
                if not (
                    isinstance(call, ast.Call)
                    and isinstance(call.func, ast.Attribute)
                    and call.func.attr == "declare_parameter"
                    and isinstance(call.func.value, ast.Name)
                    and call.func.value.id in params
                    and call.args
                    and isinstance(call.args[0], ast.Name)
                    and call.args[0].id in params
                ):
                    continue
                default = call.args[1] if len(call.args) > 1 else None
                out[self._qual(stmt.name)] = (
                    params.index(call.args[0].id),
                    params.index(default.id)
                    if isinstance(default, ast.Name) and default.id in params
                    else None,
                )
                break
        return out

    def _collect_classes(self) -> None:
        pending = [node for node in ast.walk(self.tree) if isinstance(node, ast.ClassDef)]
        for node in pending:
            for base in node.bases:
                form = self._base_form(base)
                if form:
                    self.node_classes[node.name] = form
                    break
        changed = True
        while changed:
            changed = False
            for node in pending:
                if node.name in self.node_classes:
                    continue
                for base in node.bases:
                    parent = self.node_classes.get(_dotted(base))
                    if parent:
                        self.node_classes[node.name] = parent
                        changed = True
                        break
        for node in pending:
            if node.name not in self.node_classes:
                continue
            for base in node.bases:
                full = self._defined_name(_dotted(base))
                if (
                    full
                    and full not in _NODE_BASES
                    and full.partition(".")[0] not in _PLAIN_BASES
                    and full.rpartition(".")[0] != self.module
                ):
                    # a node base or mixin another module defines may create any endpoint
                    self.handoffs.add(full)
        for node in self.tree.body:
            if isinstance(node, ast.ClassDef) and node.name in self.node_classes:
                self.defined_bases[base_key(self.package, self._qual(node.name))] = (
                    self.node_classes[node.name]
                )

    def _base_form(self, base: ast.expr) -> str:
        """The node form a base class carries, rclpy's own or one this workspace defines.

        A class whose base lives in another module of the same workspace is a node class as
        much as one that spells `rclpy.node.Node` here, and without the second table its own
        name never reaches the atlas. The table is built by the first pass over the tree.
        """
        full = self._resolve_dotted(_dotted(base))
        if not full:
            return ""
        form = _NODE_BASES.get(full) or self._stated_base(full)
        if form is None and "." not in full and self.module:
            form = self._stated_base(f"{self.module}.{full}")
        return form or ""

    def _stated_base(self, qualname: str) -> str | None:
        """A node base this workspace defines, from the package entitled to answer for it.

        A module path is package relative, so `scripts.base.Helper` and `test.util.Helper`
        are spelled by several packages of one workspace. Answering from any of them turns a
        plain class of one package into a node because another package defines that path.
        """
        for package in (self.package, qualname.partition(".")[0]):
            form = self.bases.get(base_key(package, qualname))
            if form:
                return form
        return None

    def _collect_accessors(self) -> None:
        """One hop parameter accessors, the dominant idiom on real stacks.

        `def gp(n): return self.get_parameter(n).value` is not indirection this reader has
        to guess at: the call site states the parameter name, so the read resolves.
        """
        for node in ast.walk(self.tree):
            if not isinstance(node, ast.FunctionDef | ast.AsyncFunctionDef):
                continue
            body = [item for item in node.body if not _is_docstring(item)]
            if len(body) != 1 or not isinstance(body[0], ast.Return):
                continue
            names = [arg.arg for arg in node.args.args if arg.arg != "self"]
            if len(names) != 1:
                continue
            returned = body[0].value
            if isinstance(returned, ast.Attribute) and returned.attr == "value":
                returned = returned.value
            if not isinstance(returned, ast.Call):
                continue
            reader = _dotted(returned.func).rsplit(".", 1)[-1]
            if reader not in ("get_parameter", "get_parameter_or"):
                continue
            args = [arg for arg in returned.args if not isinstance(arg, ast.Starred)]
            if len(args) == 1 and isinstance(args[0], ast.Name) and args[0].id == names[0]:
                self.accessors.add(node.name)

    def _collect_sites(self) -> None:
        """Every use of a class here, with what a call in `main` passes it.

        Any other use, a call elsewhere, a subclass or the class handed on as a value, is a
        construction this reader cannot follow, and the workspace pass then binds nothing.
        """
        called: dict[int, tuple[ast.Call, ast.FunctionDef | ast.AsyncFunctionDef]] = {}
        for main in self._nested(self.tree.body):
            if not isinstance(main, ast.FunctionDef | ast.AsyncFunctionDef) or main.name != "main":
                continue
            pending: list[ast.AST] = list(main.body)
            while pending:
                item = pending.pop()
                if isinstance(item, ast.FunctionDef | ast.AsyncFunctionDef | ast.Lambda):
                    continue
                if isinstance(item, ast.Call):
                    called[id(item.func)] = (item, main)
                pending.extend(ast.iter_child_nodes(item))
        for item in ast.walk(self.tree):
            if not isinstance(item, ast.Name | ast.Attribute) or not isinstance(item.ctx, ast.Load):
                continue
            dotted = _dotted(item)
            if not dotted or not dotted.rsplit(".", 1)[-1][:1].isupper():
                continue
            full = self._resolve_dotted(dotted)
            if full == dotted and "." not in dotted:
                full = self._qual(dotted)
            site: dict[str, Any] = {"class": full, "main": id(item) in called}
            if site["main"]:
                call, main = called[id(item)]
                site.update(
                    file=ids.build_file(self.package, self.relpath),
                    opaque=any(isinstance(arg, ast.Starred) for arg in call.args)
                    or any(keyword.arg is None for keyword in call.keywords),
                    arguments=[
                        replace(self._main_value(arg, call, main), node=None) for arg in call.args
                    ],
                    keywords={
                        keyword.arg: replace(self._main_value(keyword.value, call, main), node=None)
                        for keyword in call.keywords
                        if keyword.arg
                    },
                )
            self.sites.append(site)

    def _main_value(
        self, argument: ast.expr, site: ast.Call, main: ast.FunctionDef | ast.AsyncFunctionDef
    ) -> Val:
        """What an argument of a call in `main` holds, a literal or a constant written once."""
        value = self._static(argument)
        if isinstance(argument, ast.Name):
            for statement in main.body:
                if statement.lineno >= site.lineno:
                    break
                if any(
                    isinstance(item, ast.Name)
                    and item.id == argument.id
                    and isinstance(item.ctx, ast.Store)
                    for item in ast.walk(statement)
                ):
                    value = Val(expr=argument.id, reason="branch_dependent")
                    target = (
                        statement.target
                        if isinstance(statement, ast.AnnAssign)
                        else statement.targets[0]
                        if isinstance(statement, ast.Assign) and len(statement.targets) == 1
                        else None
                    )
                    if (
                        isinstance(target, ast.Name)
                        and target.id == argument.id
                        and statement.value is not None
                    ):
                        value = self._static(statement.value)
        return value

    def _contexts(self, node: ast.ClassDef) -> list[tuple[str, dict[str, Val], bool]]:
        """One binding of the constructor arguments per construction in a `main`.

        Only when every use of the class is such a construction, and not for a single one in
        this same file, which `_constructor_name` already reads without a context.
        """
        sites = self.constructions.get(self._qual(node.name), [])
        init = next(
            (
                item
                for item in node.body
                if isinstance(item, ast.FunctionDef) and item.name == "__init__"
            ),
            None,
        )
        if (
            init is None
            or node.name not in self.node_classes
            or not sites
            or not all(site["main"] for site in sites)
            or (len(sites) == 1 and sites[0]["file"] == ids.build_file(self.package, self.relpath))
        ):
            return []
        positional = [arg.arg for arg in init.args.posonlyargs + init.args.args][1:]
        names = positional + [arg.arg for arg in init.args.kwonlyargs]
        defaults = dict(zip(positional[::-1], init.args.defaults[::-1], strict=False))
        defaults.update(
            (arg.arg, default)
            for arg, default in zip(init.args.kwonlyargs, init.args.kw_defaults, strict=True)
            if default is not None
        )
        found: dict[str, tuple[dict[str, Val], int]] = {}
        for site in sites:
            binding: dict[str, Val] = {}
            spelled: list[str] = []
            for index, name in enumerate(names):
                val: Val | None = None
                mark = ""
                if site["opaque"]:
                    mark = "unknown"
                elif index < len(positional) and index < len(site["arguments"]):
                    val = site["arguments"][index]
                elif name in site["keywords"]:
                    val = site["keywords"][name]
                elif name in defaults:
                    val, mark = self._static(defaults[name]), "default"
                known = val is not None and val.known and val.value is not None
                if known:
                    binding[name] = val
                spelled.append(mark or (f"known:{val.value!r}" if known else "unknown"))
            context = ids.digest_fields(
                {"class": self._qual(node.name), "file": site["file"], "arguments": spelled}
            )
            held = found.get(context)
            found[context] = (binding, held[1] + 1 if held else 1)
        return [(context, binding, count > 1) for context, (binding, count) in found.items()]

    def _class_in_contexts(
        self, node: ast.ClassDef, contexts: list[tuple[str, dict[str, Val], bool]]
    ) -> None:
        """The class read once per construction, every record it states carrying that context."""
        rows = ("nodes", "endpoints", "param_reads", "callbacks", "broadcasts", "lookups")
        members: list[dict[str, Val]] = []
        for context, binding, ambiguous in contexts:
            start = {name: len(getattr(self, name)) for name in rows}
            self._seen.clear()
            self.context, self.construction = (node.name, binding), context
            self._class(node)
            for name in rows:
                for row in getattr(self, name)[start[name] :]:
                    row["construction"] = context
                    if ambiguous and name == "nodes":
                        row["construction_ambiguous"] = True
            for key, row in self.params.items():
                if key[2] == context:
                    row["construction"] = context
            members.append(self.class_members[node.name])
        self.context, self.construction = None, ""
        self._seen.clear()
        self.class_members[node.name] = {
            name: val
            for name, val in members[0].items()
            if all(other.get(name) == val for other in members)
        }

    # -- classes and functions --------------------------------------------

    def _class(self, node: ast.ClassDef, prefix: str = "") -> None:
        owner = f"{prefix}.{node.name}" if prefix else self._qual(node.name)
        methods = [
            item for item in node.body if isinstance(item, ast.FunctionDef | ast.AsyncFunctionDef)
        ]
        methods.sort(key=lambda item: (item.name != "__init__", 0))
        # Members belong to the class that assigns them. Sharing one map across a file lends
        # a topic one class states to a second class that states none.
        outer = self.members
        self.members = self._base_members(node)
        for method in methods:
            self._function(method, f"{owner}.{method.name}", class_name=node.name)
        self.class_members[node.name] = self.members
        for item in node.body:
            if isinstance(item, ast.ClassDef):
                self._class(item, owner)
        self.members = outer

    def _base_members(self, node: ast.ClassDef) -> dict[str, Val]:
        """What a class inherits from the base classes this file defines."""
        found: dict[str, Val] = {}
        for base in node.bases:
            held = self.class_members.get(_dotted(base).rsplit(".", 1)[-1])
            if held:
                found.update(held)
        return found

    def _function(
        self, node: ast.FunctionDef | ast.AsyncFunctionDef, symbol: str, **kw: str
    ) -> None:
        class_name = kw.get("class_name", "")
        names = {arg.arg for arg in node.args.args + node.args.kwonlyargs}
        ctx = Ctx(symbol, args=frozenset(names))
        for arg in sorted(names - {"self"}):
            ctx.scope[arg] = Val(expr=arg, reason="argv_value")
        if class_name and node.name == "__init__":
            for arg in sorted(names - {"self"}):
                value, _, _ = self._constructor_name(node, class_name, arg)
                if value is not None and value.known:
                    ctx.scope[arg] = value
            if self.context is not None and self.context[0] == class_name:
                ctx.scope.update(self.context[1])
            owner = symbol.rsplit(".", 1)[0]
            self.node_arguments[owner] = self._node_argument(node)
            self.class_lines[owner] = _line(node)
        if class_name and class_name in self.node_classes and node.name == "__init__":
            self._node_name(node, ctx, class_name, symbol)
        self._stmts(node.body, ctx)
        for item in self._nested(node.body):
            if isinstance(item, ast.ClassDef):
                self._class(item, symbol)
            else:
                self._function(item, f"{symbol}.{item.name}")

    @staticmethod
    def _nested(
        body: list[ast.stmt],
    ) -> list[ast.FunctionDef | ast.AsyncFunctionDef | ast.ClassDef]:
        """Definitions written inside a function body, including under an if or a try.

        A node class defined inside `def main()` with function local imports is a real idiom
        on the reference stack, and walking only the module body loses the whole node.
        """
        found: list[ast.FunctionDef | ast.AsyncFunctionDef | ast.ClassDef] = []
        pending = list(body)
        while pending:
            item = pending.pop(0)
            if isinstance(item, ast.FunctionDef | ast.AsyncFunctionDef | ast.ClassDef):
                found.append(item)
            elif isinstance(item, ast.If | ast.For | ast.AsyncFor | ast.While | ast.Try):
                pending = [*item.body, *item.orelse, *pending]
                pending.extend(getattr(item, "finalbody", []))
                for handler in getattr(item, "handlers", []):
                    pending.extend(handler.body)
            elif isinstance(item, ast.With | ast.AsyncWith):
                pending = [*item.body, *pending]
        return found

    def _node_name(
        self, node: ast.FunctionDef | ast.AsyncFunctionDef, ctx: Ctx, class_name: str, symbol: str
    ) -> None:
        call = None
        for item in ast.walk(node):
            if not isinstance(item, ast.Call):
                continue
            dotted = _dotted(item.func)
            if dotted in ("().__init__", f"{class_name}.__init__") or dotted.endswith(
                "Node.__init__"
            ):
                call = item
                break
        record: dict[str, Any] = {
            "name": None,
            "name_source": "unknown",
            "node_id": None,
            "node_id_reason": "the code states no name, so no fqn is formed here",
            "form": self.node_classes[class_name],
            "base": "rclpy.lifecycle.LifecycleNode"
            if self.node_classes[class_name] == "lifecycle_node"
            else "rclpy.node.Node",
            "class": symbol.rsplit(".", 1)[0],
            "symbol": symbol,
            "symbol_id": ids.build_sym(self.package, self.relpath, symbol),
            "line_hint": _line(node),
        }
        if call is None:
            self.nodes.append(record)
            return
        self._auto_declare(call, ctx, record)
        expr = self._name_argument(call)
        if expr is None:
            self.nodes.append(record)
            return
        record["line_hint"] = _line(call)
        self._namespace(call, ctx, record)
        val = self._eval(expr, ctx)
        if isinstance(expr, ast.Name) and expr.id in ctx.args:
            bound, site, main = self._constructor_name(node, class_name, expr.id)
            if bound is not None:
                val = bound
            if site is not None and val.known:
                record["line_hint"] = _line(site)
                record["symbol"] = main
                record["symbol_id"] = ids.build_sym(self.package, self.relpath, main)
        if val.known and isinstance(val.value, str):
            record["name"] = val.value
            record["name_source"] = "literal"
            record["node_id_reason"] = _named_reason(record)
            self.resolved_counts["node_name"] += 1
        else:
            source = self._src(expr)
            in_args = isinstance(expr, ast.Name) and expr.id in ctx.args
            record["name_source"] = "argv" if in_args else "unknown"
            record["node_id_reason"] = (
                "super().__init__ is handed a constructor argument, so the code states no name"
                if in_args
                else "the name is not a literal in this source"
            )
            record["unresolved_ref"] = f"node_name:{source}"
            reason = (
                val.reason
                if val.reason == "ambiguous_construction"
                else ("argv_value" if in_args else val.reason)
            )
            self._blind("node_name", source, reason, symbol, expr)
        self.nodes.append(record)

    def _auto_declare(self, call: ast.Call, ctx: Ctx, record: dict[str, Any]) -> None:
        """A node built to take parameters it never declares: True for a literal True, and
        `unknown` when the keyword is not a literal or arrives through `**`."""
        found: bool | str = False
        for keyword in call.keywords:
            if keyword.arg is None:
                found = found or "unknown"
            elif keyword.arg in AUTO_DECLARE_OPTIONS:
                val = self._eval(keyword.value, ctx)
                if val.known and val.value is True:
                    found = True
                elif not (val.known and val.value is False) and found is not True:
                    found = "unknown"
        if found:
            record["auto_declare"] = found

    def _constructor_name(
        self, constructor: ast.FunctionDef | ast.AsyncFunctionDef, class_name: str, parameter: str
    ) -> tuple[Val | None, ast.Call | None, str]:
        """Only one construction in main can supply a class's forwarded name."""
        if self.context is not None and self.context[0] == class_name:
            return None, None, ""
        parameters = [arg.arg for arg in constructor.args.posonlyargs + constructor.args.args]
        parameters = [name for name in parameters if name != "self"]
        if parameter not in parameters:
            return None, None, ""
        if any(
            isinstance(item, ast.Name) and item.id == parameter and isinstance(item.ctx, ast.Store)
            for item in ast.walk(constructor)
        ):
            return None, None, ""
        if (
            sum(
                isinstance(item, ast.ClassDef) and item.name == class_name
                for item in ast.walk(self.tree)
            )
            != 1
        ):
            return None, None, ""
        sites: list[tuple[ast.Call, str]] = []
        for main in self._nested(self.tree.body):
            if not isinstance(main, ast.FunctionDef | ast.AsyncFunctionDef) or main.name != "main":
                continue
            pending: list[ast.AST] = list(main.body)
            while pending:
                item = pending.pop()
                if isinstance(item, ast.FunctionDef | ast.AsyncFunctionDef | ast.Lambda):
                    continue
                if isinstance(item, ast.Call) and _dotted(item.func) == class_name:
                    sites.append((item, self._qual(main.name)))
                pending.extend(ast.iter_child_nodes(item))
        if len(sites) > 1:
            return Val(reason="ambiguous_construction"), None, ""
        if not sites:
            return None, None, ""
        site, main_symbol = sites[0]
        if any(isinstance(arg, ast.Starred) for arg in site.args):
            return None, None, ""
        position = parameters.index(parameter)
        argument = (
            site.args[position] if position < len(site.args) else self._keyword(site, parameter)
        )
        if argument is None:
            if any(keyword.arg is None for keyword in site.keywords):
                return None, None, ""
            signature = constructor.args.posonlyargs + constructor.args.args
            defaults = dict(
                zip(
                    [arg.arg for arg in signature][-len(constructor.args.defaults) :],
                    constructor.args.defaults,
                    strict=False,
                )
            )
            argument = defaults.get(parameter)
            if not isinstance(argument, ast.Constant) or not isinstance(argument.value, str):
                return None, None, ""
        main = next(
            item
            for item in self._nested(self.tree.body)
            if isinstance(item, ast.FunctionDef | ast.AsyncFunctionDef)
            and self._qual(item.name) == main_symbol
        )
        return self._main_value(argument, site, main), site, main_symbol

    def _namespace(self, call: ast.Call, ctx: Ctx, record: dict[str, Any]) -> None:
        """The namespace the constructor states, which is a fact about the source.

        A launch `namespace=` still overrides it, so this never forms an fqn here, but
        dropping it hands the join a node the code placed under `/x` as if the code had
        said nothing at all.
        """
        stated = self._keyword(call, "namespace")
        if stated is None:
            return
        val = self._eval(stated, ctx)
        if val.known and isinstance(val.value, str) and val.value:
            record["namespace"] = val.value
            record["namespace_source"] = "literal"
        else:
            record["namespace_source"] = "unresolved"
            record["namespace_expr"] = self._src(stated)

    @staticmethod
    def _node_argument(node: ast.FunctionDef | ast.AsyncFunctionDef) -> str:
        """The constructor argument that is a node, empty when the signature states none."""
        for arg in node.args.args + node.args.kwonlyargs:
            if arg.arg == "self":
                continue
            if arg.arg in wrappers.NODE_ARGUMENT_NAMES:
                return arg.arg
            stated = arg.annotation
            text = ""
            if isinstance(stated, ast.Constant) and isinstance(stated.value, str):
                text = stated.value
            elif stated is not None:
                text = _dotted(stated)
            if text.rsplit(".", 1)[-1] in ("Node", "LifecycleNode"):
                return arg.arg
        return ""

    def _note_construction(self, dotted: str, ctx: Ctx) -> None:
        """A class built inside another class constructor, for the workspace wrapper pass."""
        if not ctx.symbol.endswith(".__init__"):
            return
        tail = dotted.rsplit(".", 1)[-1]
        if not tail[:1].isupper() or tail in _CAST_CALLS:
            return
        full = self._resolve_dotted(dotted)
        if not full:
            return
        held = self.constructs.setdefault(ctx.symbol[: -len(".__init__")], [])
        if full not in held:
            held.append(full)

    def _created_node(self, node: ast.Call, ctx: Ctx, text: str) -> Val:
        """`rclpy.create_node("name")`, which states a node with no class to hang it on."""
        args = self._positional(node)
        expr = args[0] if args else self._keyword(node, "node_name")
        record: dict[str, Any] = {
            "name": None,
            "name_source": "unknown",
            "node_id": None,
            "node_id_reason": "the code states no name, so no fqn is formed here",
            "form": "node",
            "base": "rclpy.node.Node",
            "class": None,
            "api": _CREATE_NODE,
            "symbol": ctx.symbol,
            "symbol_id": ids.build_sym(self.package, self.relpath, ctx.symbol),
            "line_hint": _line(node),
        }
        self._auto_declare(node, ctx, record)
        if expr is None:
            self.nodes.append(record)
            return Val(expr=text, reason="call_result")
        self._namespace(node, ctx, record)
        val = self._eval(expr, ctx)
        if val.known and isinstance(val.value, str):
            record["name"] = val.value
            record["name_source"] = "literal"
            record["node_id_reason"] = _named_reason(record)
            self.resolved_counts["node_name"] += 1
        else:
            source = self._src(expr)
            in_args = isinstance(expr, ast.Name) and expr.id in ctx.args
            record["name_source"] = "argv" if in_args else "unknown"
            record["node_id_reason"] = (
                "the name handed to rclpy.create_node is a call argument, so the code states none"
                if in_args
                else "the name is not a literal in this source"
            )
            record["unresolved_ref"] = f"node_name:{source}"
            self._blind(
                "node_name", source, "argv_value" if in_args else val.reason, ctx.symbol, expr
            )
        if ctx.condition:
            record["condition"] = ctx.condition
        self.nodes.append(record)
        return Val(expr=text, reason="call_result")

    @staticmethod
    def _name_argument(call: ast.Call) -> ast.expr | None:
        for keyword in call.keywords:
            if keyword.arg == "node_name":
                return keyword.value
        positional = [arg for arg in call.args if not isinstance(arg, ast.Starred)]
        if positional and _dotted(call.func) != "().__init__":
            positional = positional[1:] if _is_self(call.args[0]) else positional
        return positional[0] if positional else None

    # -- statements --------------------------------------------------------

    def _stmts(self, body: list[ast.stmt], ctx: Ctx) -> None:
        for stmt in body:
            self._stmt(stmt, ctx)

    def _stmt(self, stmt: ast.stmt, ctx: Ctx) -> None:
        if isinstance(stmt, ast.Assign):
            target = stmt.targets[0] if len(stmt.targets) == 1 else None
            self._alias(target, stmt.value)
            self._target = target
            val = self._eval(stmt.value, ctx, target=target)
            self._target = None
            for item in stmt.targets:
                self._bind(item, val, ctx)
            if isinstance(target, ast.Name) and isinstance(stmt.value, ast.Call):
                self._message_fields(target.id, stmt.value, ctx)
        elif isinstance(stmt, ast.AnnAssign) and stmt.value is not None:
            self._target = stmt.target
            val = self._eval(stmt.value, ctx, target=stmt.target)
            self._target = None
            self._bind(stmt.target, val, ctx)
        elif isinstance(stmt, ast.AugAssign | ast.Expr):
            self._eval(stmt.value, ctx)
        elif isinstance(stmt, ast.If):
            condition = self._condition(stmt.test, ctx)
            self._eval(stmt.test, ctx)
            self._branches(stmt, ctx, condition)
        elif isinstance(stmt, ast.For | ast.AsyncFor):
            self._loop(stmt, ctx)
        elif isinstance(stmt, ast.While):
            self._eval(stmt.test, ctx)
            self._stmts(stmt.body, ctx)
            self._stmts(stmt.orelse, ctx)
        elif isinstance(stmt, ast.With | ast.AsyncWith):
            for item in stmt.items:
                self._eval(item.context_expr, ctx)
            self._stmts(stmt.body, ctx)
        elif isinstance(stmt, ast.Try):
            self._stmts(stmt.body, ctx)
            for handler in stmt.handlers:
                self._stmts(handler.body, ctx)
            self._stmts(stmt.orelse, ctx)
            self._stmts(stmt.finalbody, ctx)
        elif isinstance(stmt, ast.Return | ast.Await) and stmt.value is not None:
            self._eval(stmt.value, ctx)

    def _branches(self, stmt: ast.If, ctx: Ctx, condition: str | None) -> None:
        """Read each arm in its own scope and keep only what both arms agree on.

        A name the two arms bind to different values is not the else arm's value: it is one
        value per branch, so it stays a blind spot carrying the condition. Letting the last
        arm win states a name that is wrong whenever the other arm runs.
        """
        other = f"not {condition}" if condition else None
        scope_before, members_before = dict(ctx.scope), dict(self.members)
        taken = ctx.under(condition, self._guard(stmt.test, ctx, True))
        taken.scope = dict(scope_before)
        self._stmts(stmt.body, taken)
        taken_members, self.members = dict(self.members), dict(members_before)
        skipped = ctx.under(other, self._guard(stmt.test, ctx, False))
        skipped.scope = dict(scope_before)
        self._stmts(stmt.orelse, skipped)
        skipped_members = dict(self.members)
        for name in sorted(set(taken.scope) | set(skipped.scope)):
            ctx.scope[name] = self._merged(
                name, taken.scope.get(name), skipped.scope.get(name), condition
            )
        self.members = dict(members_before)
        for name in sorted(set(taken_members) | set(skipped_members)):
            self.members[name] = self._merged(
                f"self.{name}", taken_members.get(name), skipped_members.get(name), condition
            )

    def _merged(self, expr: str, taken: Val | None, skipped: Val | None, cond: str | None) -> Val:
        if taken is None or skipped is None:
            return taken or skipped or _UNKNOWN
        if taken is skipped:
            return taken
        agree = (
            taken.known
            and skipped.known
            and type(taken.value) is type(skipped.value)
            and taken.value == skipped.value
        )
        if agree or not (taken.known or skipped.known):
            return taken
        return Val(expr=expr, reason="branch_dependent", condition=cond or "")

    def _alias(self, target: ast.expr | None, value: ast.expr) -> None:
        if not isinstance(target, ast.Name) or not isinstance(value, ast.Attribute):
            return
        if value.attr in _ALIASABLE or value.attr in self.accessors:
            self.aliases[target.id] = value.attr

    def _loop(self, stmt: ast.For | ast.AsyncFor, ctx: Ctx) -> None:
        source = self._eval(stmt.iter, ctx)
        elements = self._iter_elements(stmt.iter, ctx)
        if elements is not None:
            for element in elements:
                inner = replace(ctx, scope=dict(ctx.scope))
                self._bind_element(stmt.target, element, inner, source)
                self._seen.clear()
                self._stmts(stmt.body, inner)
            self._seen.clear()
        else:
            if isinstance(stmt.target, ast.Name):
                ctx.scope[stmt.target.id] = Val(expr=stmt.target.id, reason="loop_variable")
            self._stmts(stmt.body, ctx)
        self._stmts(stmt.orelse, ctx)

    def _iter_elements(
        self, node: ast.expr, ctx: Ctx
    ) -> list[ast.expr | tuple[ast.expr, ...]] | None:
        """The elements of an iterable that is a literal in this source, as ast nodes.

        Working on the ast rather than on the evaluated value keeps the message class of a
        (topic, type, latched) row readable, and it covers `for name, value in DEFAULTS.items()`
        which is how most parameter blocks on a real stack are declared.
        """
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            view = node.func.attr
            if view in ("items", "keys", "values"):
                base = self._eval(node.func.value, ctx)
                if isinstance(base.node, ast.Dict) and None not in base.node.keys:
                    pairs = list(zip(base.node.keys, base.node.values, strict=True))
                    if view == "keys":
                        return [key for key, _ in pairs]
                    if view == "values":
                        return [value for _, value in pairs]
                    return [(key, value) for key, value in pairs]
            return None
        held = self._eval(node, ctx)
        if isinstance(held.node, ast.List | ast.Tuple):
            return list(held.node.elts)
        return None

    def _bind_element(
        self, target: ast.expr, element: ast.expr | tuple[ast.expr, ...], ctx: Ctx, source: Val
    ) -> None:
        if isinstance(target, ast.Tuple | ast.List):
            parts: tuple[ast.expr, ...] = ()
            if isinstance(element, tuple):
                parts = element
            elif isinstance(element, ast.Tuple | ast.List):
                parts = tuple(element.elts)
            if len(parts) != len(target.elts):
                for item in target.elts:
                    self._bind(item, Val(expr=self._src(item), reason="loop_variable"), ctx)
                return
            for item, part in zip(target.elts, parts, strict=True):
                self._bind_element(item, part, ctx, source)
            return
        if isinstance(element, tuple):
            self._bind(target, Val(expr=self._src(target), reason="loop_variable"), ctx)
            return
        val = replace(
            self._eval(element, ctx),
            node=element,
            binding="loop",
            from_parameter=source.from_parameter,
        )
        self._bind(target, val, ctx)

    def _message_fields(self, name: str, call: ast.Call, ctx: Ctx) -> None:
        """The frame fields a message built by keyword states,
        `TransformStamped(header=odom.header, child_frame_id='base_link')`."""
        fields: dict[str, Val] = {}
        for keyword in call.keywords:
            if keyword.arg == "child_frame_id":
                value = keyword.value
                held = None
                if isinstance(value, ast.Attribute) and isinstance(value.value, ast.Name):
                    held = ctx.fields.get(value.value.id, {}).get(value.attr)
                fields["child_frame_id"] = held or self._eval(value, ctx)
            elif keyword.arg == "header" and isinstance(keyword.value, ast.Attribute):
                source = keyword.value.value
                if isinstance(source, ast.Name) and keyword.value.attr == "header":
                    held = ctx.fields.get(source.id, {}).get("header.frame_id")
                    if held is not None:
                        fields["header.frame_id"] = held
        if fields:
            ctx.fields[name] = fields

    def _bind(self, target: ast.expr, val: Val, ctx: Ctx) -> None:
        if isinstance(target, ast.Name):
            ctx.scope[target.id] = replace(val, expr=target.id, binding=val.binding or "local")
        elif isinstance(target, ast.Attribute):
            if _is_self(target.value):
                self.members[target.attr] = replace(
                    val, expr=f"self.{target.attr}", binding=val.binding or "member"
                )
                return
            root = target
            chain: list[str] = []
            while isinstance(root, ast.Attribute):
                chain.append(root.attr)
                root = root.value
            chain.reverse()
            if not isinstance(root, ast.Name):
                return
            held = ctx.scope.get(root.id)
            if held is not None and held.qos is not None and len(chain) == 1:
                held.qos[chain[0]] = val.value if val.known else None
                held.qos["mutated"] = True
                return
            ctx.fields.setdefault(root.id, {})[".".join(chain)] = val
        elif isinstance(target, ast.Tuple | ast.List):
            for item in target.elts:
                self._bind(item, _UNKNOWN, ctx)

    # -- expressions -------------------------------------------------------

    def _guard(self, test: ast.expr, ctx: Ctx, value: bool) -> tuple[str, bool] | None:
        parameter = self._eval(test, ctx).from_parameter
        return (parameter, value) if parameter else None

    def _condition(self, test: ast.expr, ctx: Ctx) -> str:
        val = self._eval(test, ctx)
        if val.from_parameter:
            return val.from_parameter
        if val.env_var:
            return f"{val.env_var} is set and not empty"
        return self._src(test)

    def _eval(self, node: ast.expr, ctx: Ctx, target: ast.expr | None = None) -> Val:
        cached = self._seen.get(id(node))
        if cached is not None:
            return cached
        val = self._eval_uncached(node, ctx, target)
        self._seen[id(node)] = val
        return val

    def _eval_uncached(self, node: ast.expr, ctx: Ctx, target: ast.expr | None) -> Val:
        if isinstance(node, ast.Constant):
            return Val(expr=self._src(node), known=True, value=node.value, state="literal")
        if isinstance(node, ast.Name):
            return self._eval_name(node, ctx)
        if isinstance(node, ast.Attribute):
            return self._eval_attribute(node, ctx)
        if isinstance(node, ast.JoinedStr):
            return self._eval_fstring(node, ctx)
        if isinstance(node, ast.BinOp):
            return self._eval_binop(node, ctx)
        if isinstance(node, ast.Subscript):
            return self._eval_subscript(node, ctx)
        if isinstance(node, ast.Call):
            return self._eval_call(node, ctx, target)
        if isinstance(node, ast.IfExp):
            condition = self._condition(node.test, ctx)
            taken = ctx.under(condition, self._guard(node.test, ctx, True))
            body = self._eval(node.body, taken)
            skipped = ctx.under(f"not {condition}", self._guard(node.test, ctx, False))
            self._eval(node.orelse, skipped)
            return replace(body, condition=condition)
        if isinstance(node, ast.ListComp | ast.SetComp | ast.GeneratorExp | ast.DictComp):
            return self._eval_comprehension(node, ctx)
        if isinstance(node, ast.List | ast.Tuple | ast.Set | ast.Dict):
            return self._eval_container(node, ctx)
        if isinstance(node, ast.UnaryOp | ast.BoolOp | ast.Compare):
            for child in ast.iter_child_nodes(node):
                if isinstance(child, ast.expr):
                    self._eval(child, ctx)
            return self._literal(node)
        return Val(expr=self._src(node), reason="call_result")

    def _eval_container(self, node: ast.expr, ctx: Ctx) -> Val:
        """A list, tuple or dict literal, resolved element wise.

        One unreadable element (a message class, a callable) must not throw away the
        literal topic names next to it, which is the shape of every source table on a real
        stack.
        """
        text = self._src(node)
        if isinstance(node, ast.Dict):
            keys = [self._eval(item, ctx) if item is not None else _UNKNOWN for item in node.keys]
            values = [self._eval(item, ctx) for item in node.values]
            known = all(item.known for item in keys + values)
            value = {k.value: v.value for k, v in zip(keys, values, strict=True)} if known else None
        else:
            items = [self._eval(item, ctx) for item in node.elts]
            known = all(item.known for item in items)
            value = [item.value for item in items] if known else None
            if known and isinstance(node, ast.Tuple):
                value = tuple(value)
        return Val(expr=text, known=known, value=value, state="literal", node=node)

    def _eval_name(self, node: ast.Name, ctx: Ctx) -> Val:
        held = ctx.scope.get(node.id)
        if held is not None:
            return held.carry(node.id)
        constant = self.constants.get(node.id)
        if constant is not None:
            return constant.carry(node.id)
        if node.id in ctx.args:
            return Val(expr=node.id, reason="argv_value")
        return Val(expr=node.id, reason="unbound_name")

    def _eval_attribute(self, node: ast.Attribute, ctx: Ctx) -> Val:
        text = self._src(node)
        if node.attr == "value" and isinstance(node.value, ast.Call):
            inner = self._eval_call(node.value, ctx, None, want_value=True)
            return inner.carry(text)
        if _is_self(node.value):
            held = self.members.get(node.attr)
            return held.carry(text) if held is not None else Val(expr=text, reason="unbound_name")
        preset = _QOS_PRESETS.get(node.attr)
        dotted = _dotted(node)
        if preset and any(part in dotted for part in ("QoSPresetProfiles", "qos")):
            return Val(expr=text, known=True, value=preset, state="literal")
        owner = dotted.split(".")[-2] if "." in dotted else ""
        if owner in _POLICY_FIELDS:
            return Val(expr=text, known=True, value=node.attr.lower(), state="literal")
        if dotted.startswith("Parameter.Type.") or dotted.endswith(f"Type.{node.attr}"):
            mapped = _PARAM_TYPES.get(node.attr)
            if mapped is not None or node.attr in _PARAM_TYPES:
                return Val(expr=text, known=True, value=("__type__", mapped), state="literal")
        root = node
        chain = 0
        while isinstance(root, ast.Attribute):
            chain += 1
            root = root.value
        if isinstance(root, ast.Name):
            if _is_self(root):
                return Val(expr=text, reason="runtime_attribute_default")
            if root.id in ctx.args and chain >= 2:
                return Val(expr=text, reason="message_field")
            held = ctx.scope.get(root.id) or self.constants.get(root.id)
            if held is not None and not held.known:
                return Val(expr=text, reason=held.reason or "runtime_attribute_default")
        return Val(expr=text, reason="call_result")

    def _eval_fstring(self, node: ast.JoinedStr, ctx: Ctx) -> Val:
        text = self._src(node)
        parts: list[str] = []
        carried = Val(expr=text)
        for piece in node.values:
            if isinstance(piece, ast.Constant):
                inner = Val(expr=text, known=True, value=str(piece.value))
            elif isinstance(piece, ast.FormattedValue):
                inner = self._eval(piece.value, ctx)
                if not inner.known:
                    return Val(expr=text, reason=inner.reason or "call_result")
            else:
                return Val(expr=text, reason="call_result")
            if inner.from_parameter:
                if parts or carried.from_parameter is not None:
                    # The join reads a parameter per profile and appends the literal rest,
                    # so only a name that starts with the one parameter can follow it.
                    return Val(expr=text, reason="parameter_inside_name")
                carried = replace(
                    carried,
                    from_parameter=inner.from_parameter,
                    binding=inner.binding,
                    parameter_suffix=inner.parameter_suffix,
                )
            elif carried.from_parameter is not None:
                carried = replace(
                    carried, parameter_suffix=carried.parameter_suffix + str(inner.value)
                )
            parts.append(str(inner.value))
            if carried.from_constant is None and inner.from_constant:
                carried = replace(carried, from_constant=inner.from_constant)
        return replace(carried, known=True, value="".join(parts), state="resolved_from_fstring")

    def _eval_binop(self, node: ast.BinOp, ctx: Ctx) -> Val:
        left = self._eval(node.left, ctx)
        right = self._eval(node.right, ctx)
        text = self._src(node)
        if not (left.known and right.known):
            unknown = left if not left.known else right
            return Val(expr=text, reason=unknown.reason or "call_result")
        apply = _BINOPS.get(type(node.op))
        if apply is None:
            return Val(expr=text, reason="call_result")
        try:
            value = apply(left.value, right.value)
        except (TypeError, ValueError, ZeroDivisionError):
            return Val(expr=text, reason="call_result")
        carried = left if left.state != "literal" else right
        state = carried.state if carried.state != "literal" else "literal"
        suffix = ""
        if (
            isinstance(node.op, ast.Add)
            and carried is left
            and left.from_parameter
            and right.state == "literal"
            and isinstance(right.value, str)
        ):
            # `prefix + "odom"`: the join reads the parameter per profile and adds the rest
            suffix = left.parameter_suffix + right.value
        return Val(
            expr=text,
            known=True,
            value=value,
            state=state,
            from_parameter=carried.from_parameter,
            parameter_suffix=suffix,
            from_constant=carried.from_constant,
            binding=carried.binding,
        )

    def _eval_subscript(self, node: ast.Subscript, ctx: Ctx) -> Val:
        base = self._eval(node.value, ctx)
        key = self._eval(node.slice, ctx) if isinstance(node.slice, ast.expr) else _UNKNOWN
        text = self._src(node)
        element = self._element(base, key, ctx)
        if element is not None:
            if element.known and base.from_constant:
                return replace(element, expr=text, state="resolved_from_module_literal")
            return replace(element, expr=text)
        if not (base.known and key.known):
            return Val(expr=text, reason=base.reason or key.reason or "call_result")
        try:
            value = base.value[key.value]
        except (KeyError, IndexError, TypeError):
            return Val(expr=text, reason="call_result")
        state = (
            "resolved_from_module_literal"
            if base.from_constant
            else (base.state if base.state != "literal" else "literal")
        )
        return Val(
            expr=text,
            known=True,
            value=value,
            state=state,
            from_parameter=base.from_parameter,
            binding=base.binding,
        )

    def _element(self, base: Val, key: Val, ctx: Ctx) -> Val | None:
        """One element of a container literal, taken from its ast rather than its value."""
        if base.node is None or not key.known:
            return None
        if isinstance(base.node, ast.Dict):
            for item, value in zip(base.node.keys, base.node.values, strict=True):
                if item is None:
                    continue
                found = self._eval(item, ctx)
                if found.known and found.value == key.value:
                    return replace(self._eval(value, ctx), node=value)
            return None
        if isinstance(base.node, ast.List | ast.Tuple) and isinstance(key.value, int):
            elements = base.node.elts
            if -len(elements) <= key.value < len(elements):
                picked = elements[key.value]
                return replace(self._eval(picked, ctx), node=picked)
        return None

    def _eval_comprehension(self, node: ast.expr, ctx: Ctx) -> Val:
        generators = node.generators
        elements = [node.key, node.value] if isinstance(node, ast.DictComp) else [node.elt]
        if len(generators) != 1 or not isinstance(generators[0].target, ast.Name):
            for element in elements:
                self._eval(element, ctx)
            return Val(expr=self._src(node), reason="call_result")
        source = self._eval(generators[0].iter, ctx)
        if not (source.known and isinstance(source.value, list | tuple)):
            for element in elements:
                self._eval(element, ctx)
            return Val(expr=self._src(node), reason="call_result")
        for item in source.value:
            inner = replace(ctx, scope=dict(ctx.scope))
            inner.scope[generators[0].target.id] = Val(
                expr=generators[0].target.id,
                known=True,
                value=item,
                state="resolved_from_parameter" if source.from_parameter else source.state,
                from_parameter=source.from_parameter,
                binding="comprehension",
            )
            self._seen.clear()
            for element in elements:
                self._eval(element, inner)
        self._seen.clear()
        if isinstance(node, ast.DictComp):
            return Val(expr=self._src(node), reason="call_result")
        return Val(expr=self._src(node), known=True, value=list(source.value), state=source.state)

    # -- calls -------------------------------------------------------------

    def _eval_call(
        self, node: ast.Call, ctx: Ctx, target: ast.expr | None, want_value: bool = False
    ) -> Val:
        dotted = _dotted(node.func)
        tail = self.aliases.get(dotted, dotted.rsplit(".", 1)[-1])
        text = self._src(node)
        self._note_construction(dotted, ctx)

        if tail == "create_node" and self._resolve_dotted(dotted) == _CREATE_NODE:
            return self._created_node(node, ctx, text)

        if tail in _STR_METHODS and isinstance(node.func, ast.Attribute):
            receiver = self._eval(node.func.value, ctx)
            arguments = [self._eval(arg, ctx) for arg in self._positional(node)]
            if (
                receiver.known
                and isinstance(receiver.value, str)
                and all(item.known for item in arguments)
            ):
                try:
                    value = getattr(receiver.value, tail)(*[item.value for item in arguments])
                except (TypeError, ValueError):
                    return Val(expr=text, reason="call_result")
                return replace(receiver, expr=text, value=value, node=None)
            return Val(expr=text, reason=receiver.reason or "call_result")

        if tail in _CAST_CALLS and node.args:
            inner = self._eval(node.args[0], ctx)
            if inner.known:
                try:
                    return replace(inner, expr=text, value=_CAST_CALLS[tail](inner.value))
                except (TypeError, ValueError):
                    return inner.carry(text)
            return inner.carry(text)

        if dotted in ("os.environ.get", "os.getenv") and node.args:
            name = self._eval(node.args[0], ctx)
            variable = name.value if name.known and isinstance(name.value, str) else None
            return Val(expr=text, reason="environment_variable", env_var=variable)
        if dotted == "os.environ" or (
            isinstance(node.func, ast.Subscript) and "environ" in self._src(node.func)
        ):
            return Val(expr=text, reason="environment_variable")

        if tail in ("declare_parameter", "declare_parameters"):
            return self._declare(node, ctx, tail, text)
        if tail in ("get_parameter", "get_parameter_or", "has_parameter"):
            return self._read_param(node, ctx, tail, target, want_value, text)
        if tail == "get_parameters_by_prefix":
            # every parameter under the prefix is read back; a prefix the reader cannot
            # follow could be any, so it reads every parameter of the node
            args = self._positional(node)
            prefix = self._eval(args[0], ctx) if args else Val(expr=text)
            spelled = prefix.value if prefix.known and isinstance(prefix.value, str) else ""
            read = {"api": tail, "target": None, "line_hint": _line(node)}
            self.prefix_reads.append((spelled, self.construction, read))
            return Val(expr=text, reason="call_result")
        if tail in self.accessors and len(node.args) == 1:
            return self._read_param(node, ctx, tail, target, True, text)
        if tail == "add_on_set_parameters_callback":
            self._callback(node, ctx)
            return Val(expr=text, reason="call_result")
        if tail in _ENDPOINT_CALLS:
            self._endpoint(node, ctx, tail)
            return Val(expr=text, reason="call_result")
        if tail == "create_timer":
            self._timer(node, ctx)
            return Val(expr=text, reason="call_result")
        if tail == "sendTransform":
            self._broadcast(node, ctx)
            return Val(expr=text, reason="call_result")
        if tail in _LOOKUP_CALLS:
            self._lookup(node, ctx, tail)
            return Val(expr=text, reason="call_result")

        row = self._wrapper_of(dotted)
        if row is not None and row.symbol in _RCLPY_ACTIONS:
            self._action(node, ctx, row.symbol.rpartition(".")[2])
            return Val(expr=text, reason="call_result")
        if row is not None:
            return self._wrapper(node, ctx, row, text)
        if tail in _ACTION_CALLS and self._looks_like_action(node):
            self._action(node, ctx, tail)
            return Val(expr=text, reason="call_result")

        if tail == "QoSProfile":
            return self._qos_profile(node, ctx, text)

        declarer = self.declarers.get(self._defined_name(dotted))
        if declarer is not None and len(node.args) > declarer[0]:
            name_index, default_index = declarer
            default = (
                node.args[default_index]
                if default_index is not None and default_index < len(node.args)
                else None
            )
            name = self._eval(node.args[name_index], ctx)
            record, val = self._param_record(
                name, node.args[name_index], default, ctx, "declare_parameter"
            )
            if record is not None:
                self._keep(record, ctx)
                return replace(val, expr=text)
            return Val(expr=text, reason="call_result")

        for arg in node.args:
            self._eval(arg, ctx)
        for keyword in node.keywords:
            self._eval(keyword.value, ctx)
        if tail not in _NODE_USERS and any(
            isinstance(arg, ast.Name) and arg.id in _NODE_ARGUMENTS
            for arg in [*node.args, *(keyword.value for keyword in node.keywords)]
        ):
            self.handoffs.add(self._defined_name(dotted) or text.partition("(")[0])
        return Val(expr=text, reason="call_result")

    def _defined_name(self, dotted: str) -> str:
        """A call or base spelled here, as the module that defines it would spell it."""
        head = dotted.partition(".")[0]
        if head in self.imports:
            return self._resolve_dotted(dotted)
        top = {
            stmt.name
            for stmt in self.tree.body
            if isinstance(stmt, ast.ClassDef | ast.FunctionDef | ast.AsyncFunctionDef)
        }
        return self._qual(dotted) if head in top else dotted

    def _looks_like_action(self, node: ast.Call) -> bool:
        return len(self._action_arguments(node)) >= 3

    def _action_arguments(self, node: ast.Call) -> list[ast.expr]:
        """The node, type and name of an rclpy action call, positional or by keyword:
        `ActionClient(node=self, action_type=Fibonacci, action_name='fib')`."""
        args = self._positional(node)
        for key in ("node", "action_type", "action_name")[len(args) :]:
            value = self._keyword(node, key)
            if value is None:
                break
            args.append(value)
        return args

    # -- parameters --------------------------------------------------------

    def _declare(self, node: ast.Call, ctx: Ctx, api: str, text: str) -> Val:
        if api == "declare_parameters":
            self._declare_many(node, ctx)
            return Val(expr=text, reason="call_result")
        args = self._positional(node)
        if not args:
            return Val(expr=text, reason="call_result")
        if (
            ctx.symbol in self.declarers
            and isinstance(args[0], ast.Name)
            and args[0].id in ctx.args
        ):
            # the wrapper itself: each call site declares the name it passes
            return Val(expr=text, reason="call_result")
        name = self._eval(args[0], ctx)
        default = args[1] if len(args) > 1 else self._keyword(node, "value")
        record, val = self._param_record(name, args[0], default, ctx, "declare_parameter")
        if record is None:
            return Val(expr=text, reason="call_result")
        self._keep(record, ctx)
        return replace(val, expr=text)

    def _declare_many(self, node: ast.Call, ctx: Ctx) -> None:
        namespace = self._keyword(node, "namespace") or (
            self._positional(node)[0] if self._positional(node) else None
        )
        prefix = ""
        if namespace is not None:
            resolved = self._eval(namespace, ctx)
            prefix = str(resolved.value) if resolved.known and resolved.value else ""
        entries = self._keyword(node, "parameters")
        if entries is None:
            positional = self._positional(node)
            entries = positional[1] if len(positional) > 1 else None
        if isinstance(entries, ast.Dict):
            rows = list(zip(entries.keys, entries.values, strict=True))
        elif isinstance(entries, ast.List | ast.Tuple):
            rows = [
                (item.elts[0], item.elts[1] if len(item.elts) > 1 else None)
                for item in entries.elts
                if isinstance(item, ast.Tuple | ast.List) and item.elts
            ]
        else:
            return
        for name_node, default in rows:
            if name_node is None:
                # A ** spread names no parameter here, so nothing about it is known.
                self._blind("other", self._src(default), "call_result", ctx.symbol, default)
                continue
            name = self._eval(name_node, ctx)
            record, _ = self._param_record(
                name, name_node, default, ctx, "declare_parameters", prefix=prefix
            )
            if record is None:
                continue
            record["name_state"] = "map_key"
            if prefix:
                record["map_namespace"] = prefix
            self._keep(record, ctx)

    @staticmethod
    def _scope(symbol: str) -> str:
        """The class or function a declaration belongs to, so two node classes in one file
        keep their own defaults instead of the first one winning for both."""
        return symbol.rsplit(".", 1)[0]

    def _keep(self, record: dict[str, Any], ctx: Ctx) -> None:
        self.params.setdefault((self._scope(ctx.symbol), record["name"], self.construction), record)

    def _declared(self, name: str, ctx: Ctx) -> dict[str, Any] | None:
        held = self.params.get((self._scope(ctx.symbol), name, self.construction))
        if held is not None:
            return held
        found = [
            row
            for key, row in self.params.items()
            if key[1] == name and key[2] == self.construction
        ]
        return found[0] if len(found) == 1 else None

    def _param_record(
        self,
        name: Val,
        name_node: ast.expr,
        default: ast.expr | None,
        ctx: Ctx,
        api: str,
        prefix: str = "",
    ) -> tuple[dict[str, Any] | None, Val]:
        if not (name.known and isinstance(name.value, str)):
            self._blind("param_name", self._src(name_node), name.reason, ctx.symbol, name_node)
            return None, Val(expr=name.expr, reason=name.reason or "call_result")
        self.resolved_counts["param_name"] += 1
        full = f"{prefix}.{name.value}" if prefix else name.value
        record: dict[str, Any] = {
            "name": full,
            "name_state": "literal_dotted" if "." in name.value else "literal",
            "type": None,
            "has_default": default is not None,
            "default": None,
            "api": api,
            "owner_symbol": ctx.symbol,
            "reads": [],
            "line_hint": _line(name_node),
        }
        if name.state == "resolved_from_fstring":
            record["name_state"] = "resolved_from_fstring"
            record["name_expr"] = self._src(name_node)
        value = Val(expr=full, state="resolved_from_parameter", from_parameter=full)
        if default is None:
            record["default_state"] = "absent"
            self.resolved_counts["param_value"] += 1
            return record, value
        resolved = self._eval(default, ctx)
        if (
            resolved.known
            and isinstance(resolved.value, tuple)
            and resolved.value[:1] == ("__type__",)
        ):
            record["type"] = resolved.value[1]
            record["type_state"] = "declared_type_only"
            record["has_default"] = False
            record["default_state"] = "absent"
            record["declared_as"] = self._src(default)
            self.resolved_counts["param_value"] += 1
            return record, value
        if not resolved.known:
            record["type_state"] = "unresolved"
            record["default_state"] = "unresolved"
            record["default_expr"] = self._src(default)
            self._blind(
                "param_value",
                self._src(default),
                resolved.reason or "call_result",
                ctx.symbol,
                default,
            )
            return record, value
        refused = model.unrepresentable(resolved.value)
        if refused:
            # The value folded and the atlas cannot hold it. It costs its own blind spot
            # and nothing else: raising here ended the pass and dropped every other file.
            record["type_state"] = "unresolved"
            record["default_state"] = "unresolved"
            record["default_expr"] = self._src(default)
            self._blind("param_value", self._src(default), refused, ctx.symbol, default)
            return record, value
        record["type"] = _param_type(resolved.value)
        record["default"] = resolved.value
        if resolved.state != "literal":
            record["default_state"] = resolved.state
            record["default_expr"] = self._src(default)
        self.resolved_counts["param_value"] += 1
        return record, replace(value, known=True, value=resolved.value)

    def _read_param(
        self,
        node: ast.Call,
        ctx: Ctx,
        api: str,
        target: ast.expr | None,
        want_value: bool,
        text: str,
    ) -> Val:
        args = self._positional(node)
        if not args:
            return Val(expr=text, reason="call_result")
        name = self._eval(args[0], ctx)
        if not (name.known and isinstance(name.value, str)):
            return Val(expr=text, reason=name.reason or "call_result")
        target = target if target is not None else self._target
        if api == "get_parameter" and _checked(ctx.condition, name.value):
            # a read under `if self.has_parameter(name)` never meets an undeclared name
            api = "get_parameter_checked"
        read = {
            "api": "get_parameter_value" if want_value and api == "get_parameter" else api,
            **({"via": api} if api in self.accessors else {}),
            "target": self._src(target) if target is not None else None,
            "line_hint": _line(node),
        }
        declared = self._declared(name.value, ctx)
        if declared is not None:
            declared["reads"].append(read)
        else:
            self.param_reads.append({"name": name.value, "owner_symbol": ctx.symbol, **read})
        if api == "has_parameter":
            # whether the name is declared, never its value
            return Val(expr=text, reason="call_result")
        value = Val(expr=text, state="resolved_from_parameter", from_parameter=name.value)
        if declared is not None and declared.get("default") is not None:
            value = replace(value, known=True, value=declared["default"])
        return value

    def _callback(self, node: ast.Call, ctx: Ctx) -> None:
        args = self._positional(node)
        handler = args[0] if args else None
        record: dict[str, Any] = {
            "api": "add_on_set_parameters_callback",
            "form": "unknown",
            "owner_symbol": ctx.symbol,
            "handler": None,
            "line_hint": _line(node),
        }
        if isinstance(handler, ast.Attribute) and _is_self(handler.value):
            record["form"] = "bound_method"
            owner = ctx.symbol.rsplit(".", 1)[0]
            record["handler"] = f"{owner}.{handler.attr}"
        elif isinstance(handler, ast.Name):
            record["form"] = "function"
            record["handler"] = self._qual(handler.id)
        elif isinstance(handler, ast.Lambda):
            record["form"] = "lambda"
        self.callbacks.append(record)

    # -- endpoints ---------------------------------------------------------

    def _endpoint(self, node: ast.Call, ctx: Ctx, call: str) -> None:
        kind, name_index, type_index, qos_index, name_key, type_key = _ENDPOINT_CALLS[call]
        args = self._positional(node)
        name_node = args[name_index] if len(args) > name_index else self._keyword(node, name_key)
        type_node = args[type_index] if len(args) > type_index else self._keyword(node, type_key)
        if qos_index is not None and len(args) > qos_index:
            qos_node = args[qos_index]
        else:
            qos_node = self._keyword(node, "qos_profile") or self._keyword(node, "qos")
        want = "service_name" if kind.startswith("service") else "topic_name"
        record = self._named_endpoint(kind, want, name_node, type_node, ctx, node)
        record["qos"] = self._qos(qos_node, ctx) if qos_node is not None else None
        if qos_node is None:
            record["qos_reason"] = "the call states no qos argument"
        target = self._target
        if kind == "publisher" and isinstance(target, ast.Attribute) and _is_self(target.value):
            record["attribute"] = target.attr
        self.endpoints.append(record)

    def _action(self, node: ast.Call, ctx: Ctx, call: str) -> None:
        kind, name_index, type_index = _ACTION_CALLS[call]
        args = self._action_arguments(node)
        name = args[name_index] if len(args) > name_index else self._keyword(node, "action_name")
        if name is None:
            return
        kind_of = args[type_index] if len(args) > type_index else self._keyword(node, "action_type")
        record = self._named_endpoint(kind, "action_name", name, kind_of, ctx, node)
        record["qos"] = None
        self.endpoints.append(record)

    def _named_endpoint(
        self,
        kind: str,
        want: str,
        name_node: ast.expr | None,
        type_node: ast.expr | None,
        ctx: Ctx,
        call: ast.Call,
        override: str = "",
    ) -> dict[str, Any]:
        type_name, type_id = self._type_of(type_node, ctx)
        record: dict[str, Any] = {
            "kind": kind,
            "raw": self._src(name_node) if name_node is not None else "",
            "name": None,
            "name_state": "unresolved",
            "absolute": None,
            "type": type_name,
            "type_id": type_id,
            "owner_symbol": ctx.symbol,
            "line_hint": _line(call),
        }
        if name_node is None:
            self._blind(want, "", "call_result", ctx.symbol, call)
            return record
        val = self._eval(name_node, ctx)
        if val.known and isinstance(val.value, str):
            stated = override or val.value
            absolute = stated.startswith("/")
            record["name"] = stated
            record["absolute"] = absolute
            if val.state == "literal":
                record["name_state"] = "literal_absolute" if absolute else "literal_relative"
            else:
                record["name_state"] = val.state
            if not absolute:
                record["topic_id"] = None
                record["topic_id_reason"] = (
                    "a relative name mints no topic id, it stays raw until a remap or a"
                    " namespace binds it"
                )
            if val.from_parameter:
                record["from_parameter"] = val.from_parameter
                if val.parameter_suffix:
                    record["parameter_suffix"] = val.parameter_suffix
                record["binding"] = val.binding
                if val.binding in ("comprehension", "loop"):
                    record["count_state"] = "parameter_dependent"
                    record["count_detail"] = (
                        f"one endpoint per element of the {val.from_parameter} parameter. The"
                        " count is the length of the code side default, not a fact about the"
                        " running system"
                    )
            if val.from_constant:
                record["from_constant"] = val.from_constant
            self.resolved_counts[want] += 1
        else:
            record["unresolved_ref"] = f"{want}:{self._src(name_node)}"
            self._blind(
                want, self._src(name_node), val.reason, ctx.symbol, name_node, val.condition or ""
            )
        condition = ctx.condition or val.condition
        if condition:
            record["condition"] = condition
            record.update(_guard_view(ctx.guard if ctx.condition else None))
        return record

    def _timer(self, node: ast.Call, ctx: Ctx) -> None:
        args = self._positional(node)
        period = args[0] if args else self._keyword(node, "timer_period_sec")
        record: dict[str, Any] = {
            "kind": "timer",
            "raw": self._src(period) if period is not None else "",
            "name": None,
            "name_state": "not_applicable",
            "period_s": None,
            "period_state": "unresolved",
            "owner_symbol": ctx.symbol,
            "line_hint": _line(node),
        }
        if period is not None:
            val = self._eval(period, ctx)
            refused = model.unrepresentable(float(val.value)) if _numeric(val) else ""
            if refused:
                record["unresolved_ref"] = f"other:{self._src(period)}"
                self._blind("other", self._src(period), refused, ctx.symbol, period)
            elif val.known and isinstance(val.value, int | float):
                record["period_s"] = float(val.value)
                record["period_state"] = "literal" if val.state == "literal" else val.state
                self.resolved_counts["other"] += 1
                if val.from_parameter:
                    record["from_parameter"] = val.from_parameter
                    record.update(_period_form(period))
            else:
                record["unresolved_ref"] = f"other:{self._src(period)}"
                self._blind("other", self._src(period), val.reason, ctx.symbol, period)
        callback = args[1] if len(args) > 1 else self._keyword(node, "callback")
        if isinstance(callback, ast.Attribute) and _is_self(callback.value):
            record["callback"] = f"{ctx.symbol.rpartition('.')[0]}.{callback.attr}"
        if ctx.condition:
            record["condition"] = ctx.condition
            record.update(_guard_view(ctx.guard))
        self.endpoints.append(record)

    # -- qos ---------------------------------------------------------------

    def _qos(self, node: ast.expr, ctx: Ctx) -> dict[str, Any]:
        text = self._src(node)
        blank = {
            "raw": text,
            "form": "expression",
            "history": None,
            "depth": None,
            "reliability": None,
            "durability": None,
            "preset": None,
        }
        val = self._eval(node, ctx)
        if isinstance(node, ast.Constant) and isinstance(node.value, int):
            # rclpy turns a bare depth into QoSProfile(depth=n), which takes the defaults
            return {
                **blank,
                **_QOS_PROFILE_DEFAULTS,
                "form": "depth",
                "depth": node.value,
            }
        if isinstance(node, ast.Call) and _dotted(node.func).endswith("QoSProfile"):
            return {**self._qos_profile(node, ctx, text).qos, "raw": text, "form": "profile"}
        if val.known and isinstance(val.value, str) and val.value in _QOS_PRESETS.values():
            return {**blank, "form": "preset", "preset": val.value}
        preset = _QOS_PRESETS.get(_dotted(node).rsplit(".", 1)[-1])
        if preset:
            return {**blank, "form": "preset", "preset": preset}
        if val.qos is not None:
            stated = dict(val.qos)
            mutated = bool(stated.pop("mutated", False))
            return {
                "raw": text,
                "form": "variable",
                "variable": val.qos_target or text,
                "mutated": mutated,
                **stated,
            }
        return blank

    def _qos_profile(self, node: ast.Call, ctx: Ctx, text: str) -> Val:
        stated: dict[str, Any] = {
            "history": None,
            "depth": None,
            "reliability": None,
            "durability": None,
            "preset": None,
            "mutated": False,
        }
        given = {keyword.arg for keyword in node.keywords}
        for keyword in node.keywords:
            if keyword.arg is None:
                continue
            val = self._eval(keyword.value, ctx)
            if keyword.arg == "depth":
                stated["depth"] = val.value if val.known else None
            elif keyword.arg in ("reliability", "durability", "history", "liveliness"):
                stated[keyword.arg] = val.value if val.known else None
        if None not in given and not node.args:
            # rclpy fills a policy the constructor is not given from rmw_qos_profile_default.
            for key, value in _QOS_PROFILE_DEFAULTS.items():
                if key not in given:
                    stated[key] = value
        return Val(expr=text, known=False, qos=stated, reason="call_result")

    # -- wrappers ----------------------------------------------------------

    def _wrapper(self, node: ast.Call, ctx: Ctx, row: wrappers.Wrapper, text: str) -> Val:
        """One construction of a known wrapper, expanded through the shared table."""
        line = _line(node)
        args = self._positional(node)
        index = row.argument if row.argument is not None else 0
        name_key, type_key, qos_key = _WRAPPER_KEYWORDS.get(row.symbol, ("", "", ""))
        name_node = None
        type_node = None
        if row.takes_name:
            name_node = args[index] if len(args) > index else self._keyword(node, name_key)
            if index >= 1:
                type_node = (
                    args[index - 1] if len(args) > index - 1 else self._keyword(node, type_key)
                )
        val = self._eval(name_node, ctx) if name_node is not None else _UNKNOWN
        base = str(val.value) if val.known and isinstance(val.value, str) else ""
        produced = row.resolved(base) if (base or not row.takes_name) else ()
        entry = self.wrappers.setdefault(
            row.symbol,
            {
                "symbol": row.symbol,
                "kind": "known_wrapper",
                "detail": row.note or row.evidence,
                "line_hint": line,
            },
        )
        qos_node = None
        if row.takes_name and len(row.endpoints) == 1:
            qos_node = args[index + 1] if len(args) > index + 1 else self._keyword(node, qos_key)

        records: list[dict[str, Any]] = []
        if produced:
            for kind, name, type_name in produced:
                if name_node is None:
                    records.append(self._fixed_endpoint(kind, name, type_name, ctx, text, line))
                    continue
                derived = name != base
                record = self._named_endpoint(
                    kind,
                    "topic_name",
                    name_node,
                    None if derived else type_node,
                    ctx,
                    node,
                    override=name,
                )
                if type_name:
                    package, interface, short = type_name.split("/")
                    record["type"] = type_name
                    record["type_id"] = ids.build_type(package, interface, short)
                elif derived:
                    # /x/_action/send_goal is not of the action type, it is one of the five
                    # interfaces rcl_action derives from it, and this reader does not name them.
                    outer, outer_id = self._type_of(type_node, ctx)
                    if outer:
                        record["action_type"] = outer
                        record["action_type_id"] = outer_id
                record["qos"] = self._qos(qos_node, ctx) if qos_node is not None else None
                if qos_node is None:
                    record["qos_reason"] = (
                        f"{row.symbol} was given no qos argument, so the source states none"
                    )
                records.append(record)
        elif row.takes_name:
            for endpoint in row.endpoints:
                record = self._named_endpoint(
                    endpoint.kind, "topic_name", name_node, type_node, ctx, node
                )
                record["qos"] = None
                records.append(record)
        for record in records:
            record["via_known_wrapper"] = row.symbol
            if row.library == "tf2_ros" and "condition" not in record:
                record["condition"] = ctx.condition
                record.update(_guard_view(ctx.guard if ctx.condition else None))
            self.endpoints.append(record)
            if row.library != "tf2_ros" and record["name"]:
                bucket = "publishes" if record["kind"] == "publisher" else "subscribes"
                names = entry.setdefault(bucket, [])
                if record["name"] not in names:
                    names.append(record["name"])
        if row.library == "tf2_ros":
            self.wrappers.pop(row.symbol, None)
        return Val(
            expr=text,
            reason="call_result",
            wrapper=row.symbol,
            condition=ctx.condition,
            guard=ctx.guard if ctx.condition else None,
        )

    def _fixed_endpoint(
        self, kind: str, name: str, type_name: str, ctx: Ctx, text: str, line: int | None
    ) -> dict[str, Any]:
        record: dict[str, Any] = {
            "kind": kind,
            "raw": text,
            "name": name,
            "name_state": "wrapper_constant",
            "absolute": name.startswith("/"),
            "type": type_name or None,
            "type_id": None,
            "qos": None,
            "qos_reason": "the profile is chosen inside the wrapper, not in this source",
            "owner_symbol": ctx.symbol,
            "line_hint": line,
        }
        if type_name:
            package, interface, short = type_name.split("/")
            record["type_id"] = ids.build_type(package, interface, short)
        self.resolved_counts["topic_name"] += 1
        return record

    # -- tf ----------------------------------------------------------------

    def _broadcast(self, node: ast.Call, ctx: Ctx) -> None:
        holder = _UNKNOWN
        if isinstance(node.func, ast.Attribute):
            holder = self._eval(node.func.value, ctx)
        wrapper = holder.wrapper or ""
        kind = "static" if "Static" in wrapper else "dynamic"
        args = self._positional(node)
        pairs = self._transform_pairs(args[0] if args else None, ctx)
        if args and not pairs and not isinstance(args[0], ast.List | ast.Tuple):
            # transforms this reader cannot follow are lost, so they are a blind spot
            self._blind("frame_id", self._src(args[0]), "", ctx.symbol, node)
            return
        for parent, child in pairs:
            parent, child = _frame_val(parent), _frame_val(child)
            record: dict[str, Any] = {
                "kind": kind,
                "parent": parent.value if parent.known else None,
                "child": child.value if child.known else None,
                "parent_state": parent.state if parent.known else "unresolved",
                "child_state": child.state if child.known else "unresolved",
                "broadcaster": wrapper or None,
                "condition": holder.condition,
                **_guard_view(holder.guard),
                "owner_symbol": ctx.symbol,
                "frame_ids": [],
                "line_hint": _line(node),
            }
            for role, val in (("parent", parent), ("child", child)):
                if val.known:
                    record["frame_ids"].append(ids.build_frame(str(val.value)))
                    self.resolved_counts["frame_id"] += 1
                    if val.from_parameter:
                        record[f"{role}_from_parameter"] = val.from_parameter
                        if val.parameter_suffix:
                            record[f"{role}_parameter_suffix"] = val.parameter_suffix
                elif val.expr:
                    record["unresolved_ref"] = f"frame_id:{val.expr}"
                    self._blind("frame_id", val.expr, val.reason, ctx.symbol, node)
            self.broadcasts.append(record)

    def _transform_pairs(self, node: ast.expr | None, ctx: Ctx) -> list[tuple[Val, Val]]:
        """(parent, child) per transform one `sendTransform` argument holds."""
        if node is None:
            return [(_UNKNOWN, _UNKNOWN)]
        if isinstance(node, ast.List | ast.Tuple):
            pairs: list[tuple[Val, Val]] = []
            for item in node.elts:
                found = self._transform_pairs(item, ctx)
                if not found:
                    self._blind("frame_id", self._src(item), "", ctx.symbol, item)
                pairs.extend(found)
            return pairs
        if isinstance(node, ast.Name):
            fields = ctx.fields.get(node.id, {})
            return [
                (fields.get("header.frame_id", _UNKNOWN), fields.get("child_frame_id", _UNKNOWN))
            ]
        if isinstance(node, ast.Call):
            method = self._own_method(node, ctx)
            if method is not None:
                return self._returned_transforms(method, ctx)
            parent = self._keyword(node, "frame_id")
            child = self._keyword(node, "child_frame_id")
            if (
                parent is None
                and child is None
                and not _dotted(node.func).endswith("TransformStamped")
            ):
                return []
            return [
                (
                    self._eval(parent, ctx) if parent is not None else _UNKNOWN,
                    self._eval(child, ctx) if child is not None else _UNKNOWN,
                )
            ]
        return []

    def _own_method(self, node: ast.Call, ctx: Ctx) -> ast.FunctionDef | None:
        """The one method `self.<name>()` with no arguments calls, defined by the class of
        the calling method."""
        func = node.func
        if (
            node.args
            or node.keywords
            or not isinstance(func, ast.Attribute)
            or not isinstance(func.value, ast.Name)
            or func.value.id != "self"
        ):
            return None
        owner = ctx.symbol.rsplit(".", 1)[0].rsplit(".", 1)[-1]
        found = [
            item
            for klass in ast.walk(self.tree)
            if isinstance(klass, ast.ClassDef) and klass.name == owner
            for item in klass.body
            if isinstance(item, ast.FunctionDef) and item.name == func.attr
        ]
        return found[0] if len(found) == 1 else None

    def _returned_transforms(self, method: ast.FunctionDef, ctx: Ctx) -> list[tuple[Val, Val]]:
        """The transforms a method builds field by field and returns, `return (a, b)`.

        Only a returned name whose frame fields the method assigns from expressions this
        reader evaluates counts, so anything else stays a blind spot of the caller.
        """
        fields: dict[str, dict[str, ast.expr]] = {}
        returned: ast.expr | None = None
        # the method's own locals, never the caller's
        own = Ctx(ctx.symbol)
        assigned: dict[str, int] = {}
        for item in ast.walk(method):
            if isinstance(item, ast.Assign | ast.AnnAssign | ast.AugAssign | ast.For):
                targets = item.targets if isinstance(item, ast.Assign) else [item.target]
                for target in targets:
                    for name in ast.walk(target):
                        if isinstance(name, ast.Name):
                            assigned[name.id] = assigned.get(name.id, 0) + 1
        for item in method.body:
            if (
                isinstance(item, ast.Assign)
                and len(item.targets) == 1
                and isinstance(item.targets[0], ast.Name)
                and assigned.get(item.targets[0].id) == 1
            ):
                own.scope[item.targets[0].id] = self._eval(item.value, own)
        for item in ast.walk(method):
            if isinstance(item, ast.Return):
                if returned is not None:
                    return []
                returned = item.value
            elif isinstance(item, ast.Assign) and len(item.targets) == 1:
                path = _dotted(item.targets[0]).split(".", 1)
                if len(path) == 2 and path[1] in ("header.frame_id", "child_frame_id"):
                    fields.setdefault(path[0], {})[path[1]] = item.value
        names = (
            returned.elts
            if isinstance(returned, ast.List | ast.Tuple)
            else [returned]
            if returned is not None
            else []
        )
        pairs: list[tuple[Val, Val]] = []
        for name in names:
            held = fields.get(name.id) if isinstance(name, ast.Name) else None
            if not held or len(held) != 2:
                return []
            pairs.append(
                (self._eval(held["header.frame_id"], own), self._eval(held["child_frame_id"], own))
            )
        return pairs

    def _lookup(self, node: ast.Call, ctx: Ctx, call: str) -> None:
        args = self._positional(node)
        source_at = 2 if "_full" in call else 1
        target_node = args[0] if args else self._keyword(node, "target_frame")
        source_node = (
            args[source_at] if len(args) > source_at else self._keyword(node, "source_frame")
        )
        if target_node is None or source_node is None:
            return
        target = _frame_val(self._eval(target_node, ctx))
        source = _frame_val(self._eval(source_node, ctx))
        record: dict[str, Any] = {
            "api": call,
            "target": target.value if target.known else None,
            "source": source.value if source.known else None,
            "target_state": target.state if target.known else "unresolved",
            "source_state": source.state if source.known else "unresolved",
            "owner_symbol": ctx.symbol,
            "line_hint": _line(node),
        }
        for role, val, expr_node in (
            ("target", target, target_node),
            ("source", source, source_node),
        ):
            if val.known:
                self.resolved_counts["frame_id"] += 1
                if val.from_parameter:
                    record[f"{role}_from_parameter"] = val.from_parameter
                    if val.parameter_suffix:
                        record[f"{role}_parameter_suffix"] = val.parameter_suffix
            else:
                record["unresolved_ref"] = f"frame_id:{self._src(expr_node)}"
                self._blind("frame_id", self._src(expr_node), val.reason, ctx.symbol, expr_node)
        if ctx.condition:
            record["condition"] = ctx.condition
        self.lookups.append(record)

    # -- blind spots -------------------------------------------------------

    def _blind(
        self,
        want: str,
        expr: str,
        reason: str,
        symbol: str,
        node: ast.AST,
        condition: str = "",
    ) -> None:
        reason = reason or "call_result"
        entry: dict[str, Any] = {
            "want": want,
            "expr": expr,
            "reason": reason,
            "detail": _DETAILS.get(reason, "the expression does not resolve statically"),
            "resolvable_by": list(_RESOLVABLE_BY.get(reason, ("observed",))),
            "owner_symbol": symbol,
            "line_hint": _line(node),
        }
        if condition:
            entry["condition"] = condition
        for existing in self.unresolved:
            if (existing["want"], existing["expr"], existing["owner_symbol"]) == (
                want,
                expr,
                symbol,
            ):
                return
        self.unresolved.append(entry)

    # -- argument helpers --------------------------------------------------

    @staticmethod
    def _positional(node: ast.Call) -> list[ast.expr]:
        return [arg for arg in node.args if not isinstance(arg, ast.Starred)]

    @staticmethod
    def _keyword(node: ast.Call, name: str) -> ast.expr | None:
        for keyword in node.keywords:
            if keyword.arg == name:
                return keyword.value
        return None

    # -- output ------------------------------------------------------------

    def _publishing(self) -> dict[str, set[str]]:
        """The methods that call `self.<attribute>.publish(...)`, by attribute."""
        out: dict[str, set[str]] = {}
        for klass in self.tree.body:
            if not isinstance(klass, ast.ClassDef):
                continue
            for method in klass.body:
                if not isinstance(method, ast.FunctionDef | ast.AsyncFunctionDef):
                    continue
                for call in ast.walk(method):
                    func = call.func if isinstance(call, ast.Call) else None
                    if (
                        isinstance(func, ast.Attribute)
                        and func.attr == "publish"
                        and isinstance(func.value, ast.Attribute)
                        and _is_self(func.value.value)
                    ):
                        out.setdefault(func.value.attr, set()).add(
                            self._qual(klass.name, method.name)
                        )
        return out

    def _candidates(self) -> list[dict[str, Any]]:
        """Classes that might be wrappers, for the workspace pass to rule on.

        A node subclass is never one: it is the thing a wrapper is given, and promoting it
        would hand every construction of a node class the endpoints of its own base.
        """
        out: list[dict[str, Any]] = []
        for symbol in sorted(self.constructs):
            argument = self.node_arguments.get(symbol, "")
            if not argument or symbol.rsplit(".", 1)[-1] in self.node_classes:
                continue
            out.append(
                {
                    "symbol": symbol,
                    "node_argument": argument,
                    "constructs": sorted(self.constructs[symbol]),
                    "line_hint": self.class_lines.get(symbol),
                }
            )
        return out

    def _record(self) -> dict[str, Any]:
        endpoints = sorted(
            self.endpoints,
            key=lambda item: (item["kind"], _sorted_key(item.get("name")), item.get("raw", "")),
        )
        counts = {
            "endpoints": len(endpoints),
            "publishers": sum(1 for item in endpoints if item["kind"] == "publisher"),
            "subscribers": sum(1 for item in endpoints if item["kind"] == "subscriber"),
            "service_servers": sum(1 for item in endpoints if item["kind"] == "service_server"),
            "service_clients": sum(1 for item in endpoints if item["kind"] == "service_client"),
            "timers": sum(1 for item in endpoints if item["kind"] == "timer"),
            "parameters": len(self.params),
            "unresolved": len(self.unresolved),
        }
        for kind in ("action_server", "action_client"):
            total = sum(1 for item in endpoints if item["kind"] == kind)
            if total:
                counts[f"{kind}s"] = total
        if self.broadcasts:
            counts["tf_broadcasts"] = len(self.broadcasts)
        if self.lookups:
            counts["tf_lookups"] = len(self.lookups)
        wrapped = sum(1 for item in endpoints if item.get("via_known_wrapper"))
        if wrapped:
            counts["via_known_wrapper"] = wrapped

        record: dict[str, Any] = {
            "package": self.package,
            "path": self.path,
            "package_relative_path": self.relpath,
            "language": LANGUAGE,
            "file_id": ids.build_file(self.package, self.relpath),
            "executables": list(self.executables),
            "nodes": sorted(
                self.nodes, key=lambda item: (item["symbol"], _sorted_key(item.get("name")))
            ),
            "endpoints": endpoints,
            "parameters": sorted(
                self.params.values(), key=lambda item: (item["name"], item["owner_symbol"])
            ),
            "parameter_callbacks": sorted(
                self.callbacks,
                key=lambda item: (item["owner_symbol"], item["form"], _sorted_key(item["handler"])),
            ),
            "plugin_parameters": [],
            "tf": {
                "broadcasts": sorted(
                    self.broadcasts,
                    key=lambda item: (_sorted_key(item["parent"]), _sorted_key(item["child"])),
                ),
                "lookups": sorted(
                    self.lookups,
                    key=lambda item: (
                        item["api"],
                        _sorted_key(item["target"]),
                        _sorted_key(item["source"]),
                    ),
                ),
            },
            "unresolved": sorted(self.unresolved, key=lambda item: (item["want"], item["expr"])),
            "counts": counts,
        }
        if self.wrappers:
            record["wrappers"] = sorted(self.wrappers.values(), key=lambda item: item["symbol"])
        candidates = self._candidates()
        if candidates:
            record["wrapper_candidates"] = candidates
        if self.handoffs:
            record["node_handoffs"] = sorted(self.handoffs)
        own = self.own_declarers()
        if own:
            record["declarers"] = own
        literals = {
            node.value
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Constant)
            and isinstance(node.value, str)
            and is_name_literal(node.value)
        }
        if literals:
            record["name_literals"] = sorted(literals)
        publishing = self._publishing()
        for row in self.endpoints:
            if row.get("attribute") in publishing:
                row["published_from"] = sorted(publishing[row["attribute"]])
        stamped = {
            (node.value.value, _line(node))
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Assign)
            and isinstance(node.value, ast.Constant)
            and isinstance(node.value.value, str)
            and any(_is_header_frame(target) for target in node.targets)
        }
        if stamped:
            record["stamped_frames"] = [
                {"frame": frame, "line_hint": line} for frame, line in sorted(stamped)
            ]
        for prefix, construction, read in self.prefix_reads:
            for (_scope, name, held), row in self.params.items():
                under = not prefix or (isinstance(name, str) and name.startswith(prefix + "."))
                if held == construction and under:
                    row["reads"].append(read)
        if self.param_reads:
            record["parameter_reads"] = sorted(
                self.param_reads, key=lambda item: (item["name"], item["owner_symbol"])
            )
        return record


def _named_reason(record: dict[str, Any]) -> str:
    if record.get("namespace_source") == "literal":
        return (
            "the code states the name and the namespace, and a launch namespace overrides the"
            " second, so no fqn is formed here"
        )
    return (
        "the code states the name, the namespace comes from the declared layer, so no fqn is"
        " formed here"
    )


def _numeric(val: Val) -> bool:
    return val.known and isinstance(val.value, int | float)


def _param_type(value: Any) -> str | None:
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int):
        return "int"
    if isinstance(value, float):
        return "double"
    if isinstance(value, str):
        return "string"
    if isinstance(value, list | tuple):
        if not value:
            return None
        first = value[0]
        if isinstance(first, bool):
            return "bool_array"
        if isinstance(first, int):
            return "int_array"
        if isinstance(first, float):
            return "double_array"
        if isinstance(first, str):
            return "string_array"
    return None


# ---------------------------------------------------------------------------
# workspace level reading


def analyse_source(
    source: str,
    *,
    path: str,
    package: str,
    package_relative_path: str,
    executables: tuple[str, ...] = (),
    extra_wrappers: dict[str, wrappers.Wrapper] | None = None,
    extra_bases: dict[str, str] | None = None,
) -> dict[str, Any]:
    """Read one python source into the coded layer record shape."""
    reader = PyFile(
        source,
        path=path,
        package=package,
        package_relative_path=package_relative_path,
        executables=executables,
        extra_wrappers=extra_wrappers,
        extra_bases=extra_bases,
    )
    return reader.read()


def analyse_file(
    file: Path,
    root: Path,
    extra_wrappers: dict[str, wrappers.Wrapper] | None = None,
    extra_bases: dict[str, str] | None = None,
) -> dict[str, Any]:
    """Read one file on disk. `root` is the workspace root every path is relative to."""
    return _read_file(file, root, extra_wrappers, extra_bases)[0]


def _read_file(
    file: Path,
    root: Path,
    extra_wrappers: dict[str, wrappers.Wrapper] | None = None,
    extra_bases: dict[str, str] | None = None,
    constructions: dict[str, list[dict[str, Any]]] | None = None,
) -> _FileRead:
    package, package_dir = package_of(file)
    relative = file.relative_to(package_dir) if package_dir else file.relative_to(root)
    module = str(relative)[:-3].replace("/", ".") if str(relative).endswith(".py") else ""
    reader = PyFile(
        file.read_text(encoding="utf-8", errors="replace"),
        path=str(file.relative_to(root)),
        package=package,
        package_relative_path=str(relative),
        executables=entry_points(package_dir).get(module, ()),
        extra_wrappers=extra_wrappers,
        extra_bases=extra_bases,
        constructions=constructions,
    )
    return reader.read(), reader.resolved_counts, reader.defined_bases, reader.sites


def package_of(file: Path) -> tuple[str, Path | None]:
    """The ament package a file belongs to, and the package directory."""
    for parent in file.parents:
        manifest = parent / "package.xml"
        if manifest.is_file():
            return _package_name(manifest), parent
    return ids.REPO_PKG, None


def _package_name(manifest: Path) -> str:
    text = manifest.read_text(encoding="utf-8", errors="replace")
    start = text.find("<name>")
    end = text.find("</name>")
    if start == -1 or end == -1:
        return ids.REPO_PKG
    return text[start + len("<name>") : end].strip()


_ENTRY_CACHE: dict[str, dict[str, tuple[str, ...]]] = {}


def entry_points(package_dir: Path | None) -> dict[str, tuple[str, ...]]:
    """console_scripts of an ament_python package, as module dotted path to executables."""
    if package_dir is None:
        return {}
    cached = _ENTRY_CACHE.get(str(package_dir))
    if cached is not None:
        return cached
    found: dict[str, list[str]] = {}
    setup = package_dir / "setup.py"
    if setup.is_file():
        try:
            tree = parse_python(setup.read_text(encoding="utf-8", errors="replace"))
        except SyntaxError:
            tree = None
        if tree is not None:
            for node in ast.walk(tree):
                if not isinstance(node, ast.Call):
                    continue
                if _dotted(node.func).rsplit(".", 1)[-1] != "setup":
                    continue
                for keyword in node.keywords:
                    if keyword.arg != "entry_points":
                        continue
                    for entry in _console_scripts(keyword.value):
                        name, _, target = entry.partition("=")
                        module = target.split(":")[0].strip()
                        found.setdefault(module, []).append(name.strip())
    table = {module: tuple(sorted(names)) for module, names in found.items()}
    _ENTRY_CACHE[str(package_dir)] = table
    return table


def _console_scripts(node: ast.expr) -> list[str]:
    if not isinstance(node, ast.Dict):
        return []
    out: list[str] = []
    for key, value in zip(node.keys, node.values, strict=True):
        if not isinstance(key, ast.Constant) or key.value != "console_scripts":
            continue
        if isinstance(value, ast.List | ast.Tuple):
            out.extend(
                item.value
                for item in value.elts
                if isinstance(item, ast.Constant) and isinstance(item.value, str)
            )
    return out


def candidates(records: Sequence[dict[str, Any]]) -> list[wrappers.Candidate]:
    """The wrapper candidates a set of records offers, in the shape the table rules on."""
    out: list[wrappers.Candidate] = []
    for record in records:
        for row in record.get("wrapper_candidates") or ():
            out.append(
                wrappers.Candidate(
                    symbol=row["symbol"],
                    constructs=tuple(row["constructs"]),
                    takes_node=bool(row["node_argument"]),
                    language=LANGUAGE,
                    file=record.get("package_relative_path", ""),
                    line_hint=row.get("line_hint"),
                )
            )
    return sorted(out, key=lambda item: item.symbol)


def analyse_workspace(root: Path, subdir: str = "src", discover: bool = True) -> dict[str, Any]:
    """Read every python file under a workspace, returning the records and the counters.

    Two passes when discovery is on: the first states which first party classes wrap a known
    wrapper, the second reads the files again knowing them, because the construction of such a
    class is an endpoint site and a per file reader cannot see the class's own module.
    """
    base = root / subdir if subdir else root
    files = sorted(path for path in base.rglob("*.py") if path.is_file())
    found: dict[str, wrappers.Wrapper] = {}
    bases: dict[str, str] = {}
    constructions: dict[str, list[dict[str, Any]]] = {}
    if discover:
        first, _ = _read_all(files, root, {}, {})
        found = wrappers.index(wrappers.discover(candidates([row[0] for row in first])))
        bases = {name: form for row in first for name, form in row[2].items()}
        constructions = construction_table([(row[0], row[3]) for row in first])
    read, failures = _read_all(files, root, found, bases, constructions)
    resolved: dict[str, int] = dict.fromkeys(WANTS, 0)
    unresolved: dict[str, int] = dict.fromkeys(WANTS, 0)
    for record, counts, _, _ in read:
        for want, total in counts.items():
            resolved[want] += total
        for entry in record["unresolved"]:
            unresolved[entry["want"]] += 1
    return {
        "root": str(root),
        "files": [row[0] for row in read],
        "failures": failures,
        "wrappers": [row.symbol for row in sorted(found.values(), key=lambda row: row.symbol)],
        "resolution": {
            want: {"resolved": resolved[want], "unresolved": unresolved[want]} for want in WANTS
        },
    }


def _read_all(
    files: Sequence[Path],
    root: Path,
    found: dict[str, wrappers.Wrapper],
    bases: dict[str, str],
    constructions: dict[str, list[dict[str, Any]]] | None = None,
) -> tuple[list[_FileRead], list[dict[str, str]]]:
    read: list[_FileRead] = []
    failures: list[dict[str, str]] = []
    for file in files:
        try:
            read.append(_read_file(file, root, found, bases, constructions))
        except (CollectorError, OSError, RecursionError, ValueError) as exc:
            failures.append({"path": str(file.relative_to(root)), "error": str(exc)})
    return read, failures


def construction_table(
    reads: Sequence[tuple[dict[str, Any], list[dict[str, Any]]]],
) -> dict[str, list[dict[str, Any]]]:
    """Every use of a node class across the workspace, keyed by the class's module path.

    A module path two files define cannot say which of them a use reaches, so it is left out.
    """
    defined: dict[str, set[str]] = {}
    for record, _ in reads:
        for row in record["nodes"]:
            if row.get("class"):
                defined.setdefault(str(row["class"]), set()).add(record["file_id"])
    table: dict[str, list[dict[str, Any]]] = {}
    for _, sites in reads:
        for site in sites:
            if len(defined.get(site["class"], ())) == 1:
                table.setdefault(site["class"], []).append(site)
    return table


# ---------------------------------------------------------------------------
# the collector


def source_paths(ws: Any) -> list[str]:
    """The python this layer reads: the editable trees only.

    The install tree of an ament_python package holds copies of the same modules, so reading
    it would state every endpoint twice under a second file id while an editor can only change
    the src twin.
    """
    return [path for path in ws.source_files(SUFFIXES) if ws.root_of(path) == model.Root.SRC.value]


def package_directory(ws: Any, package: str | None) -> Path | None:
    found = ws.packages.get(package or "")
    source = getattr(found, "src_path", None) if found is not None else None
    return Path(source) if source else None


def read_file(
    ws: Any,
    path: str,
    found: dict[str, wrappers.Wrapper] | None = None,
    bases: dict[str, str] | None = None,
    constructions: dict[str, list[dict[str, Any]]] | None = None,
    declarers: dict[str, tuple[int, int | None]] | None = None,
) -> _FileRead:
    """One file of a discovered workspace, cited by file id rather than by machine path."""
    package, relpath = ws.rel(path)
    module = relpath[:-3].replace("/", ".") if relpath.endswith(".py") else ""
    reader = PyFile(
        Path(path).read_text(encoding="utf-8", errors="replace"),
        path=ws.file_id(path),
        package=package or ids.REPO_PKG,
        package_relative_path=relpath,
        executables=entry_points(package_directory(ws, package)).get(module, ()),
        extra_wrappers=found,
        extra_bases=bases,
        constructions=constructions,
        declarers=declarers,
    )
    return reader.read(), reader.resolved_counts, reader.defined_bases, reader.sites


_TARGETS = {
    "publisher": ids.build_topic,
    "subscriber": ids.build_topic,
    "service_server": ids.build_service,
    "service_client": ids.build_service,
    "action_server": ids.build_action,
    "action_client": ids.build_action,
}

_SYMBOL_KEYS = (
    ("endpoints", "endpoints"),
    ("parameters", "parameters"),
    ("parameter_callbacks", "parameter_callbacks"),
    ("parameter_reads", "parameter_reads"),
)


@dataclass
class _Read:
    path: str
    package: str
    relpath: str
    record: dict[str, Any]
    counts: dict[str, int]
    bases: dict[str, str] = field(default_factory=dict)
    sites: list[dict[str, Any]] = field(default_factory=list)


@dataclass
class PyCodeCollector:
    """The coded layer for rclpy sources.

    Two passes over the tree, for one reason stated in `wrappers.discover`: a class of this
    workspace that wraps a known wrapper is an endpoint site, and the file that constructs it
    cannot see the class's own module. The first pass offers the candidates, the table rules on
    them, the second pass reads the files knowing the answer. Nothing else differs between the
    two, so the second pass alone is what is written.
    """

    name: str = NAME
    layer: str = LAYER
    discovered: tuple[wrappers.Wrapper, ...] = ()
    node_bases: dict[str, str] = field(default_factory=dict)

    def collect(self, ctx: CollectContext) -> None:
        ws = ctx.ws
        builder = ctx.builder
        paths = source_paths(ws)
        read, problems = self._read(ws, paths, {}, {})
        self.discovered = wrappers.discover(candidates([item.record for item in read]))
        self.node_bases = {name: form for item in read for name, form in sorted(item.bases.items())}
        constructions = construction_table([(item.record, item.sites) for item in read])
        declarers = {
            name: position
            for item in read
            for name, position in sorted((item.record.get("declarers") or {}).items())
        }
        if self.discovered or self.node_bases or constructions or declarers:
            read, problems = self._read(
                ws,
                paths,
                wrappers.index(self.discovered),
                self.node_bases,
                constructions,
                declarers,
            )
        run = builder.add_run(
            LAYER,
            input_digest=ids.digest(
                NAME,
                *sorted(ws.file_id(item.path) for item in read),
                *[row.symbol for row in self.discovered],
                *sorted(self.node_bases),
                *[f"{key}={value}" for key, value in sorted(ws.root_hashes().items())],
            ),
            roots=ws.root_hashes(),
            exceptions=[
                {
                    "path": ws.file_id(path),
                    "phase": "describe",
                    "exc_type": exc_type,
                    "message": message,
                }
                for path, exc_type, message in sorted(problems)
            ]
            + ws.undecodable(),
        )
        workspace.record_undecodable(builder, ws)
        builder.declare_layer(LAYER, present=True, run=run)
        late = builder.atlas.entities[run].attrs["exceptions"]
        for item in read:
            try:
                self._emit(ctx, run, item)
            except ContractError as exc:
                self._refused(ctx, item.path, str(exc), late)
        for path, _exc_type, message in sorted(problems):
            self._unreadable(ctx, path, message)
        late.sort(key=lambda row: (str(row.get("path", "")), str(row.get("phase", ""))))

    def _refused(
        self, ctx: CollectContext, path: str, message: str, late: list[dict[str, Any]]
    ) -> None:
        """One file the atlas refused, recorded on that file and nowhere else.

        Letting it out of the loop ended the pass, and every other source of the workspace
        left the atlas with no finding, a resolution rate of 1.000 and a green gate.
        """
        ws = ctx.ws
        file_id = ws.file_id(path)
        late.append(
            {
                "path": file_id,
                "phase": "describe",
                "exc_type": ContractError.__name__,
                "message": message,
            }
        )
        ctx.builder.unresolved(
            model.Unresolved(
                want=model.Want.OTHER.value,
                owner=file_id,
                expr=message,
                reason="py_refused_by_atlas",
                role="py_source",
                resolvable_by=["the value or the attribute this file states has to be writable"],
                evidence=[ws.source(ws.site(path))],
            )
        )

    def _unreadable(self, ctx: CollectContext, path: str, message: str) -> None:
        """A file this reader could not parse is still a file of its package, and says so.

        Dropping it would state a package whose sources this layer read in full while one of
        them was never opened. The refusal is attributes and no fact, which is the one shape
        a refused entity takes, and the blind spots sit under the wants this file would have
        answered, so a refusal costs the rate its denominator instead of leaving the counter.
        """
        ws = ctx.ws
        builder = ctx.builder
        package, _relpath = ws.rel(path)
        file_id = ws.file_id(path)
        builder.node(
            file_id,
            attrs={
                "language": LANGUAGE,
                "loadable": False,
                "refused_because": REFUSED_PY_SOURCE,
                "refused_detail": message.replace(path, file_id),
            },
        )
        if package and package != ids.REPO_PKG:
            pkg_id = ids.build_pkg(package)
            builder.node(pkg_id)
            builder.edge(model.EdgeKind.CONTAINS.value, pkg_id, file_id)
        for want in REFUSED_WANTS:
            builder.unresolved(
                model.Unresolved(
                    want=want,
                    owner=file_id,
                    expr=message.replace(path, file_id),
                    reason=REFUSED_PY_SOURCE,
                    role="py_source",
                    resolvable_by=["the file has to parse as python 3.10"],
                    evidence=[ws.source(ws.site(path, "parse"))],
                )
            )

    # reading --------------------------------------------------------------

    def _read(
        self,
        ws: Any,
        paths: Sequence[str],
        found: dict[str, wrappers.Wrapper],
        bases: dict[str, str],
        constructions: dict[str, list[dict[str, Any]]] | None = None,
        declarers: dict[str, tuple[int, int | None]] | None = None,
    ) -> tuple[list[_Read], list[tuple[str, str, str]]]:
        read: list[_Read] = []
        problems: list[tuple[str, str, str]] = []
        for path in paths:
            package, relpath = ws.rel(path)
            try:
                record, counts, defined, sites = read_file(
                    ws, path, found, bases, constructions, declarers
                )
            except CollectorError as exc:
                problems.append((path, CollectorError.__name__, str(exc)))
            except Exception as exc:
                # One unreadable file must never cost the layer every other file. A source
                # with a null byte reaches here as a ValueError from the parser.
                problems.append(
                    (
                        path,
                        type(exc).__name__,
                        f"{LAYER}.{NAME}: reading {ws.file_id(path)} raised "
                        f"{type(exc).__name__}: {exc}",
                    )
                )
            else:
                read.append(
                    _Read(
                        path=path,
                        package=package or ids.REPO_PKG,
                        relpath=relpath,
                        record=record,
                        counts=counts,
                        bases=defined,
                        sites=sites,
                    )
                )
        return read, problems

    # emission -------------------------------------------------------------

    def _sym_id(self, item: _Read, symbol: str) -> str:
        return ids.build_sym(item.package, item.relpath, symbol) if symbol else ""

    def _emit(self, ctx: CollectContext, run: str, item: _Read) -> None:
        ws = ctx.ws
        builder = ctx.builder
        record = item.record
        file_id = ws.file_id(item.path)
        # A test file shaped like a launch file is refused by the launch walk and read here,
        # and the refusal stands, as every other writer of the attribute keeps it.
        builder.node(file_id, attrs={"language": LANGUAGE}).attrs.setdefault("loadable", True)
        if item.package != ids.REPO_PKG:
            pkg_id = ids.build_pkg(item.package)
            builder.node(pkg_id)
            builder.edge(model.EdgeKind.CONTAINS.value, pkg_id, file_id)
        values: dict[str, Any] = {
            "language": LANGUAGE,
            "parsed": True,
            "nodes": record["nodes"],
            "counts": record["counts"],
            "executables": record["executables"],
            "wrappers": record.get("wrappers", []),
            **{
                key: record[key]
                for key in ("node_handoffs", "name_literals", "stamped_frames")
                if key in record
            },
        }
        builder.fact(
            file_id,
            LAYER,
            run=run,
            values=values,
            confidence=model.Confidence.CERTAIN.value,
            evidence=[ws.source(ws.site(item.path))],
        )
        for want, total in sorted(item.counts.items()):
            if total:
                builder.count_resolved(want, total)
        for symbol in self._symbols(record):
            self._symbol(ctx, run, item, symbol, file_id)
        self._blind_spots(ctx, item, file_id)

    @staticmethod
    def _symbols(record: dict[str, Any]) -> list[str]:
        found = {str(row["symbol"]) for row in record["nodes"]}
        for key, _label in _SYMBOL_KEYS:
            for row in record.get(key) or ():
                found.add(str(row.get("owner_symbol") or ""))
        for key in ("broadcasts", "lookups"):
            for row in record["tf"][key]:
                found.add(str(row.get("owner_symbol") or ""))
        # A symbol whose only statement is a blind spot still gets its entity, so the entry
        # is mirrored onto the function an engineer would open rather than onto the file.
        for row in record["unresolved"]:
            found.add(str(row.get("owner_symbol") or ""))
        return sorted(name for name in found if name)

    def _symbol(
        self, ctx: CollectContext, run: str, item: _Read, symbol: str, file_id: str
    ) -> None:
        ws = ctx.ws
        builder = ctx.builder
        record = item.record
        sym_id = self._sym_id(item, symbol)
        builder.node(sym_id, attrs={"language": LANGUAGE})
        builder.edge(model.EdgeKind.DEFINES.value, file_id, sym_id)

        def slice_of(rows: Any) -> list[dict[str, Any]]:
            return [row for row in rows or () if row.get("owner_symbol") == symbol]

        endpoints = slice_of(record["endpoints"])
        broadcasts = slice_of(record["tf"]["broadcasts"])
        builder.fact(
            sym_id,
            LAYER,
            run=run,
            values={
                "nodes": [row for row in record["nodes"] if row["symbol"] == symbol],
                "endpoints": endpoints,
                "parameters": slice_of(record["parameters"]),
                "parameter_reads": slice_of(record.get("parameter_reads")),
                "tf_broadcasts": broadcasts,
                "tf_lookups": slice_of(record["tf"]["lookups"]),
            },
            confidence=model.Confidence.CERTAIN.value,
            evidence=[ws.source(ws.site(item.path, symbol))],
        )
        for row in endpoints:
            self._endpoint(ctx, row)
        for row in broadcasts:
            self._broadcast(ctx, row)
        for row in slice_of(record["tf"]["lookups"]):
            for name in (row["target"], row["source"]):
                if name:
                    builder.node(ids.build_frame(str(name)))

    def _endpoint(self, ctx: CollectContext, row: dict[str, Any]) -> None:
        """The entities an absolute code name states. A relative name mints nothing: it is the
        join that binds it, through the namespace and the remaps of an instance."""
        build = _TARGETS.get(str(row["kind"]))
        if build is None or not row.get("absolute") or not row.get("name"):
            return
        endpoint_id = build(str(row["name"]))
        ctx.builder.node(endpoint_id)
        type_id = row.get("type_id")
        if type_id:
            ctx.builder.node(str(type_id))
            ctx.builder.edge(model.EdgeKind.USES_TYPE.value, endpoint_id, str(type_id))
            ctx.builder.count_resolved(model.Want.TYPE.value)

    def _broadcast(self, ctx: CollectContext, row: dict[str, Any]) -> None:
        """The frames a broadcast names, and nothing else.

        The tf_parent edge belongs to the join and to no one else. This layer cannot say
        which profile a broadcast runs in, and a second unscoped edge over the same pair
        makes every rule that counts the parents of a child count one site twice.
        """
        for name in (row["parent"], row["child"]):
            if name:
                ctx.builder.node(ids.build_frame(str(name)))

    def _blind_spots(self, ctx: CollectContext, item: _Read, file_id: str) -> None:
        ws = ctx.ws
        for entry in item.record["unresolved"]:
            owner = self._sym_id(item, str(entry.get("owner_symbol") or "")) or file_id
            ctx.builder.unresolved(
                model.Unresolved(
                    want=entry["want"],
                    owner=owner,
                    expr=entry["expr"],
                    reason=entry["reason"],
                    role="py_source",
                    resolvable_by=list(entry.get("resolvable_by") or []),
                    evidence=[ws.source(ws.site(item.path, entry["reason"], entry["line_hint"]))],
                )
            )


def counts(records: Sequence[dict[str, Any]]) -> dict[str, int]:
    """The totals the bench report quotes."""
    out = dict.fromkeys(
        (
            "files",
            "nodes",
            "named_nodes",
            "publisher",
            "subscriber",
            "service_server",
            "service_client",
            "action_server",
            "action_client",
            "timer",
            "parameters",
            "broadcasts",
            "lookups",
            "via_known_wrapper",
            "unresolved",
        ),
        0,
    )
    for record in records:
        out["files"] += 1
        out["nodes"] += len(record["nodes"])
        out["named_nodes"] += sum(1 for row in record["nodes"] if row["name"])
        out["parameters"] += len(record["parameters"])
        out["broadcasts"] += len(record["tf"]["broadcasts"])
        out["lookups"] += len(record["tf"]["lookups"])
        out["unresolved"] += len(record["unresolved"])
        for row in record["endpoints"]:
            if row["kind"] in out:
                out[row["kind"]] += 1
            if row.get("via_known_wrapper"):
                out["via_known_wrapper"] += 1
    return out


register(NAME, PyCodeCollector)


def comparable(value: Any) -> Any:
    """Drop the advisory keys, so two records compare on facts only."""
    if isinstance(value, dict):
        return {key: comparable(item) for key, item in value.items() if key not in _PROSE_KEYS}
    if isinstance(value, list):
        return [comparable(item) for item in value]
    return value


__all__ = [
    "LANGUAGE",
    "LAYER",
    "NAME",
    "REFUSED_PY_SOURCE",
    "REFUSED_WANTS",
    "SUFFIXES",
    "PyCodeCollector",
    "PyFile",
    "analyse_file",
    "analyse_source",
    "analyse_workspace",
    "base_key",
    "candidates",
    "comparable",
    "counts",
    "entry_points",
    "package_directory",
    "package_of",
    "read_file",
    "source_paths",
]
