"""C++ sources: endpoints, parameters, TF, node names and registrations.

The declared layer knows a node was launched with a remap. This layer knows what the
translation unit actually asks for. Neither half is the topic graph, and the join of the two
is the reason M2 exists, so the rule here is that a name is reported exactly as the source
states it: absolute when the source is absolute, raw when the source is relative, and
unresolved when the source assembles it from something this pass cannot follow.

tree-sitter is an optional extra. `load_parser` imports it inside the function and raises
`CollectorError` naming the install line when it is missing, so an extraction without the
extra records one exception and keeps every other layer.

Three decisions are worth stating before the code.

* No node id is formed here. The source states a node name and never a namespace, and an
  fqn built by assuming the root namespace is a fabricated identity that splits every
  namespaced node in two. The name and its `name_source` ride on the symbol that passed it
  to the base constructor, and the declared layer supplies the namespace.
* The message type rides in the template argument, `create_publisher<pkg::msg::Type>(...)`,
  so it is read from the type and never from a string. A template argument that is not a
  three part interface name (a local alias, a template parameter) yields no type entity.
* A name assembled from a member, a constant, a macro or a concatenation is followed as far
  as one function plus the file scope allows, because on this stack most non literal names
  are a `std::string` initialised from `declare_parameter<std::string>` a few lines earlier,
  and how it was followed is recorded in `name_state`. What is left is a blind spot with the
  raw expression: a loop over a descriptor file, a frame that arrives in a message header, a
  plugin prefix that only the yaml layer can supply.

QoS is reported as the properties the expression names and nothing more. A depth of 10 or a
reliability the source never stated, written as fact, manufactures drift against a live
system, which is the fastest way to make a drift report get ignored.
"""

from __future__ import annotations

import copy
import math
import posixpath
import re
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

from rosio import extra_line, ids, model, workspace
from rosio.collect import CollectContext, py_code, register
from rosio.errors import CollectorError, ContractError
from rosio.resolve import wrappers

LAYER = model.Layer.CODED.value
NAME = "cpp_code"

SUFFIXES = (".cpp", ".cc", ".cxx", ".c", ".hpp", ".hh", ".hxx", ".h", ".ipp")

TREE_SITTER_PIN = ("tree-sitter>=0.23,<0.27", "tree-sitter-cpp>=0.23,<0.24")
"""The pair this collector is written against. The python binding changed shape more than
once: 0.21 wanted a compiled library path plus `Parser.set_language`, 0.22 introduced the
`Language(PyCapsule)` constructor the grammar packages ship today, and 0.25 moved query
execution behind `QueryCursor`. Only the constructor and plain node walking are used here,
which is the part that has been stable since 0.22, and `load_parser` accepts both the
`Parser(language)` and the `set_language` spelling, so the whole 0.22 to 0.26 range works."""

INSTALL_HINT = extra_line("cpp")

NO_SOURCE = "the workspace holds no C++ source under an editable root"
"""What `inputs_present` says when there is nothing here for this collector to read."""

# ---------------------------------------------------------------------------
# what the reader recognises

PUBLISH_CALLS = frozenset(
    {"create_publisher", "create_lifecycle_publisher", "create_generic_publisher"}
)
SUBSCRIBE_CALLS = frozenset({"create_subscription", "create_generic_subscription"})
GENERIC_CALLS = frozenset({"create_generic_publisher", "create_generic_subscription"})
"""Generic endpoints take the type as a string after the name and the qos after the type."""
SERVICE_CALLS = frozenset({"create_service"})
CLIENT_CALLS = frozenset({"create_client"})
PLUGIN_SETUP_METHODS = frozenset({"configure", "initialize"})
PLUGIN_NAME = "plugin_name"
PLUGIN_NAME_ARGUMENT = "<plugin instance name>"
"""A construction argument that hands a helper the instance name of the plugin building it."""
"""The origin of a name that holds a plugin's instance name, which only the yaml states."""
ACTION_FACTORY_CALLS = {
    "create_action_server": "action_server",
    "create_action_client": "action_client",
}
"""nav2 `LifecycleNode::create_action_server<T>(name, ...)`, and the namespace qualified free
functions of `nav2::interfaces` that take the node first."""
TIMER_CALLS = frozenset({"create_wall_timer", "create_timer"})
CAMERA_CALLS = frozenset({"create_camera_publisher", "create_camera_subscription"})
IMAGE_CALLS = frozenset({"create_publisher", "create_subscription"}) | CAMERA_CALLS
IMAGE_MEMBER_CALLS = frozenset({"advertise", "advertiseCamera", "subscribeCamera"})
"""image_transport takes the node first and the base topic second. The rows in
`rosio/resolve/wrappers.py` say which topics each form creates."""
IMAGE_QOS_POSITION = {
    "image_transport::create_publisher": 2,
    "image_transport::create_camera_publisher": 2,
    "image_transport::create_subscription": 4,
    "image_transport::create_camera_subscription": 4,
    "image_transport::SubscriberFilter": 3,
}
CALLBACK_CALLS = frozenset({"add_on_set_parameters_callback", "add_post_set_parameters_callback"})

DECLARE_CALLS = frozenset(
    {
        "declare_parameter",
        "declare_parameter_if_not_declared",
        "declare_or_get_parameter",
        "get_or_declare_parameter",
    }
)
NODE_FIRST_DECLARE_CALLS = frozenset(
    {"declare_parameter_if_not_declared", "get_or_declare_parameter"}
)
"""Free functions handed the node first: nav2 `declare_parameter_if_not_declared(node, name,
value)` and `autoware_utils::get_or_declare_parameter<T>(node, name)`, whose definition is
not in a workspace that only depends on it."""
DECLARE_MAP_CALLS = frozenset({"declare_parameters"})
PLUGIN_DECLARE_CALLS = frozenset({"declareParameter"})
"""`nav2_costmap_2d::Layer::declareParameter` prefixes the layer instance name itself, so
the source states a suffix and the full name only exists once the yaml names the layer."""
READ_CALLS = frozenset({"get_parameter", "get_parameter_or"})
VALUE_CALLS = frozenset(
    {
        "string",
        "basic_string",
        "to_string",
        "c_str",
        "data",
        "ParameterValue",
        "resolve_topic_or_service_name",
        "resolve_topic_name",
    }
)
"""Calls whose value is their first argument as far as a name goes. A node resolves a topic
with its own remaps and namespace, which the join applies to the raw name the same way."""

BROADCAST_CALLS = frozenset({"sendTransform"})
LOOKUP_CALLS = frozenset({"lookupTransform", "canTransform"})

NODE_BASES = {
    "rclcpp::Node": "node",
    "Node": "node",
    "rclcpp_lifecycle::LifecycleNode": "lifecycle_node",
    "LifecycleNode": "lifecycle_node",
    "nav2_util::LifecycleNode": "lifecycle_node",
    "nav2::LifecycleNode": "lifecycle_node",
}

PLAIN_NODE_TYPES = {name: form for name, form in NODE_BASES.items() if "::" in name}
"""The node types a free function creates by value: `rclcpp::Node::make_shared("talker")`,
`std::make_shared<rclcpp::Node>("talker")` or `rclcpp::Node node("talker")`. Only the
qualified spellings, because a bare `Node` there is as likely a class of the package."""

NODE_INTERFACE = re.compile(r"(?:(?:this|node_?|parent)->)?get_node_\w+_interface\(\)")

NODE_USERS = frozenset(
    {"bind", "ref", "cref", "move", "forward", "make_pair", "make_tuple", "spin", "spin_some"}
    | {"spin_until_future_complete", "static_pointer_cast", "dynamic_pointer_cast"}
    | {"create_server", "create_client"}
)
"""Free functions handed a node that create nothing on it, or nothing this reader misses."""

MACROS = frozenset(
    {
        "PLUGINLIB_EXPORT_CLASS",
        "RCLCPP_COMPONENTS_REGISTER_NODE",
        "GZ_REGISTER_MODEL_PLUGIN",
        "GZ_REGISTER_SENSOR_PLUGIN",
        "GZ_REGISTER_WORLD_PLUGIN",
        "BT_REGISTER_NODES",
    }
)

KNOWN_WRAPPERS: dict[str, dict[str, str]] = {
    "tf2_ros::TransformBroadcaster": {
        "reason": "the profile is chosen inside tf2_ros, not in this source",
        "broadcaster": "dynamic",
    },
    "tf2_ros::StaticTransformBroadcaster": {
        "reason": "the profile is chosen inside tf2_ros, not in this source",
        "broadcaster": "static",
    },
    "tf2_ros::TransformListener": {
        "reason": "the profile is chosen inside tf2_ros, not in this source",
    },
    "nav2::TransformBroadcaster": {
        "reason": "the profile is chosen inside tf2_ros, not in this source",
        "broadcaster": "dynamic",
    },
    "nav2::StaticTransformBroadcaster": {
        "reason": "the profile is chosen inside tf2_ros, not in this source",
        "broadcaster": "static",
    },
    "nav2::TransformListener": {
        "reason": "the profile is chosen inside tf2_ros, not in this source",
    },
    "diagnostic_updater::Updater": {
        "reason": "the profile is chosen inside diagnostic_updater, not in this source",
    },
    "message_filters::Subscriber": {
        "reason": (
            "message_filters::Subscriber::subscribe was given no qos argument, "
            "so the source states none"
        ),
    },
    "image_transport::SubscriberFilter": {"reason": ""},
    "nav2_util::TwistPublisher": {"reason": "the qos is a constructor argument of the wrapper"},
    "nav2_util::TwistSubscriber": {"reason": "the qos is a constructor argument of the wrapper"},
    "autoware_utils_rclcpp::InterProcessPollingSubscriber": {
        "reason": "the qos is a constructor argument of the wrapper"
    },
    "autoware_utils::InterProcessPollingSubscriber": {
        "reason": "the qos is a constructor argument of the wrapper"
    },
    "autoware::universe_utils::InterProcessPollingSubscriber": {
        "reason": "the qos is a constructor argument of the wrapper"
    },
}
"""Types this reader recognises as creating a real endpoint no `create_publisher` or
`create_subscription` call names, with the C++ specific note each one needs. What each
construction publishes or subscribes is not restated here: `rosio/resolve/wrappers.py`
owns that table and both this collector and the join read it, so the two can never drift."""


def wrapper_endpoints(symbol: str) -> tuple[tuple[str, str, str], ...]:
    """(kind, name, type) per endpoint one C++ construction creates, from the shared table."""
    return wrappers.endpoints_for(symbol, language="cpp")


def wrapper_takes_name(symbol: str) -> bool:
    """True when the caller supplies the name, so the construction alone states no endpoint."""
    row = wrappers.lookup(symbol, "cpp")
    return row is not None and row.takes_name


INTERFACE_DIRS = ("msg", "srv", "action")

PRESETS = {
    "SensorDataQoS": "sensor_data",
    "SystemDefaultsQoS": "system_default",
    "ServicesQoS": "services",
    "ParametersQoS": "parameters",
    "ParameterEventsQoS": "parameter_events",
    "ClockQoS": "clock",
    "RosoutQoS": "rosout",
}
RELIABILITY = {"reliable": "reliable", "best_effort": "best_effort"}
DURABILITY = {"transient_local": "transient_local", "durability_volatile": "volatile"}

SCALAR_TYPES = {
    "std::string": "string",
    "string": "string",
    "double": "double",
    "float": "double",
    "int": "int",
    "int64_t": "int",
    "bool": "bool",
}

_CHRONO = {
    "nanoseconds": 1e-9,
    "microseconds": 1e-6,
    "milliseconds": 1e-3,
    "seconds": 1.0,
    "minutes": 60.0,
    "hours": 3600.0,
}
_DURATION_SUFFIX = re.compile(r"^(\d+(?:\.\d+)?)(ns|us|ms|s|min|h)$")
_SUFFIX_SCALE = {"ns": 1e-9, "us": 1e-6, "ms": 1e-3, "s": 1.0, "min": 60.0, "h": 3600.0}
_ROS_CALL = re.compile(
    r"create_(?:publisher|subscription|service|client|wall_timer|timer|server)"
    r"|declare_parameters?|get_parameters?|sendTransform|lookupTransform|canTransform"
    r"|rclcpp_action|rclcpp::Node|LifecycleNode"
)
"""What a refused region has to contain to hide something this collector reads. A format
macro inside a string or a visibility macro before a function hides nothing."""

_DEFINE = re.compile(r'^[ \t]*#[ \t]*define[ \t]+([A-Za-z_]\w*)[ \t]+"((?:[^"\\]|\\.)*)"', re.M)

_TRANSITIONS = (
    "on_configure",
    "on_activate",
    "on_deactivate",
    "on_cleanup",
    "on_shutdown",
    "on_error",
)

_LOOPS = ("for_statement", "for_range_loop", "while_statement", "do_statement")

LITERAL = "literal"
SUFFIX = "suffix"
UNRESOLVED = "unresolved"

REFUSED = "refused"
"""A default the source states and json cannot hold. The declaration keeps its name,
its type and its line, and the value alone becomes a blind spot."""

STATE_BY_ORIGIN = {
    "parameter": "resolved_from_parameter",
    "plugin_parameter": "resolved_from_plugin_parameter_default",
    "constant": "resolved_from_constant",
    "macro": "resolved_from_macro",
    "concat": "resolved_from_concat",
    "wrapper": "wrapper_constant",
}


# ---------------------------------------------------------------------------
# the optional dependency


def load_parser() -> Any:
    """A tree-sitter parser for C++, or a CollectorError naming the install line."""
    try:
        import tree_sitter
        import tree_sitter_cpp
    except ImportError as exc:
        raise CollectorError(
            f"the C++ collector needs tree-sitter and tree-sitter-cpp ({exc.name} is missing). "
            f"Install the extra: {INSTALL_HINT}, which pins {' and '.join(TREE_SITTER_PIN)}",
            LAYER,
            NAME,
        ) from exc
    try:
        language = tree_sitter.Language(tree_sitter_cpp.language())
    except (TypeError, ValueError, AttributeError) as exc:
        raise CollectorError(
            f"the installed tree-sitter and tree-sitter-cpp do not match: {exc}. "
            f"Install the pair the extra pins: {' and '.join(TREE_SITTER_PIN)}",
            LAYER,
            NAME,
        ) from exc
    try:
        return tree_sitter.Parser(language)
    except TypeError as exc:
        # 0.22 and 0.23 also accept the older two step spelling. From 0.25 there is no
        # set_language at all, so a refusal here is a version mismatch and not a fallback.
        setter = getattr(tree_sitter.Parser, "set_language", None)
        if setter is None:
            raise CollectorError(
                f"the installed tree-sitter does not take a language: {exc}. "
                f"Install the pair the extra pins: {' and '.join(TREE_SITTER_PIN)}",
                LAYER,
                NAME,
            ) from exc
        parser = tree_sitter.Parser()
        setter(parser, language)
        return parser


# ---------------------------------------------------------------------------
# records


@dataclass(frozen=True)
class Ref:
    """One name the source states, and how far it could be followed.

    `status` is `literal` when the string is known, `suffix` when the source states only the
    tail of a plugin scoped parameter, and `unresolved` otherwise. `origin` says how it was
    followed and becomes the `name_state` of the report. `member` keeps a pending member read
    so the second pass can finish a name whose value is written in another file.
    """

    status: str
    value: str = ""
    expr: str = ""
    origin: str = ""
    source_name: str = ""
    binding: str = ""
    member: str = ""
    owner: str = ""
    parameter_suffix: str = ""
    built: str = ""
    """The expression a local was assigned from, kept when it could not be read."""

    @property
    def resolved(self) -> bool:
        return self.status in (LITERAL, SUFFIX)

    @property
    def absolute(self) -> bool:
        return self.status == LITERAL and self.value.startswith("/")

    def state(self) -> str:
        if self.status == UNRESOLVED:
            return UNRESOLVED
        if self.origin in STATE_BY_ORIGIN:
            return STATE_BY_ORIGIN[self.origin]
        return "literal_absolute" if self.absolute else "literal_relative"


@dataclass(frozen=True)
class Endpoint:
    kind: str
    name: Ref
    sym: str
    owner_class: str
    line: int
    api: str = ""
    type_text: str = ""
    qos: dict[str, Any] | None = None
    qos_reason: str = ""
    options_argument: str = ""
    options_position: int = 0
    via_known_wrapper: str = ""
    condition: str = ""
    guard: tuple[tuple[str, bool], ...] = ()
    publisher_kind: str = ""
    period_s: float | None = None
    period_state: str = ""
    period_from: str = ""
    period_expr: str = ""
    period_reason: str = ""
    """Why a period that folded cannot be written. Empty when nothing folded, which is a
    different answer and gets a different state."""
    unknown_count: str = ""
    callback: str = ""
    period_form: str = ""
    period_numerator: float | None = None
    attribute: str = ""
    lifecycle: bool = False
    construction: str = ""
    conditional_site: bool = False
    """A construction that a guard could have wrapped. A member initialiser cannot be
    conditional, so there is no condition to report for one."""


@dataclass(frozen=True)
class ParamSite:
    """One declaration. `scope` is `node` for a plain name and `plugin` for a suffix that
    only the yaml plugin selector can complete."""

    name: Ref
    sym: str
    owner_class: str
    line: int
    scope: str = "node"
    api: str = "declare_parameter"
    value_type: str = ""
    default: Any = None
    default_ref: Ref | None = None
    default_expr: str = ""
    default_state: str = "known"
    default_reason: str = ""
    via_wrapper: str = ""
    map_namespace: str = ""
    spelling: str = ""
    default_site: tuple[str, str, int] | None = None
    construction: str = ""


@dataclass(frozen=True)
class ParamRead:
    name: Ref
    sym: str
    line: int
    api: str = "get_parameter_out"
    target: str = ""
    accessor: str = ""
    spelling: str = ""
    construction: str = ""


@dataclass(frozen=True)
class ParamCallback:
    api: str
    form: str
    handler: str
    sym: str
    line: int


@dataclass(frozen=True)
class TfBroadcast:
    parent: Ref
    child: Ref
    sym: str
    line: int
    kind: str = "dynamic"
    broadcaster: str = ""
    condition: str = ""
    guard: tuple[tuple[str, bool], ...] = ()
    construction: str = ""


@dataclass(frozen=True)
class TfLookup:
    api: str
    target: Ref
    source: Ref
    sym: str
    line: int
    construction: str = ""


@dataclass(frozen=True)
class NodeClass:
    qualname: str
    base: str = ""
    form: str = "node"
    stated_name: str = ""
    name_source: str = "unknown"
    name_expr: str = ""
    symbol: str = ""
    takes_node_options: bool = False
    name_line: int | None = None
    name_position: int | None = None
    name_reason: str = ""
    name_default: str | None = None
    construction: str = ""
    construction_ambiguous: bool = False


@dataclass(frozen=True)
class Registration:
    macro: str
    type: str
    base_class: str = ""
    names: tuple[str, ...] = ()
    line: int = 0
    wrapped: bool = False


@dataclass(frozen=True)
class WrapperUse:
    symbol: str
    kind: str
    line: int
    topics: tuple[str, ...] = ()


@dataclass
class Unit:
    """One translation unit as read. `parsed` false means the grammar refused a region, and
    what is outside the region is still read, because a macro that expands to a type must not
    cost a file its endpoints."""

    path: str
    pkg: str = ""
    relpath: str = ""
    parsed: bool = True
    reason: str = ""
    error_lines: tuple[int, ...] = ()
    missing_tokens: int = 0
    hidden_regions: int = 0
    classes: list[NodeClass] = field(default_factory=list)
    plain_nodes: list[NodeClass] = field(default_factory=list)
    """Nodes a free function creates from a node type itself, named by the type."""
    members: dict[tuple[str, str], Ref] = field(default_factory=dict)
    endpoints: list[Endpoint] = field(default_factory=list)
    publishing: dict[tuple[str, str], set[str]] = field(default_factory=dict)
    params: list[ParamSite] = field(default_factory=list)
    reads: list[ParamRead] = field(default_factory=list)
    callbacks: list[ParamCallback] = field(default_factory=list)
    broadcasts: list[TfBroadcast] = field(default_factory=list)
    lookups: list[TfLookup] = field(default_factory=list)
    registrations: list[Registration] = field(default_factory=list)
    plugin_of_helper: Registration | None = None
    """The plugin that builds this file's helper class and hands it the instance name."""
    plugin_name_locals: set[str] = field(default_factory=set)
    """The string parameter of `configure` or `initialize`, the instance name of a plugin."""
    wrappers: dict[str, tuple[int, int]] = field(default_factory=dict)
    shared_wrappers: dict[str, tuple[int, int]] = field(default_factory=dict)
    member_wrappers: dict[tuple[str, str], str] = field(default_factory=dict)
    shared_members: dict[tuple[str, str], str] = field(default_factory=dict)
    """Known wrapper members another file of the package declares, by short class name and
    member, so `diagnostics_(this)` in a source constructs what its header declares."""
    plugin_loaders: list[str] = field(default_factory=list)
    lambda_wrappers: set[str] = field(default_factory=set)
    """Declare wrappers another file of the package defines, a template in a header."""
    wrapper_uses: list[WrapperUse] = field(default_factory=list)
    methods: list[str] = field(default_factory=list)
    empty_bodies: list[str] = field(default_factory=list)
    constructions: list[dict[str, Any]] = field(default_factory=list)
    type_aliases: list[dict[str, Any]] = field(default_factory=list)
    class_names: list[str] = field(default_factory=list)
    class_bases: dict[str, list[str]] = field(default_factory=dict)
    class_helpers: dict[str, set[str]] = field(default_factory=dict)
    node_handoffs: dict[str, set[str]] = field(default_factory=dict)
    """Class to the classes it constructs, and the free functions (`name()`) it calls,
    handing them a node: its own `this` or a node handle it was given."""
    field_types: dict[tuple[str, str], str] = field(default_factory=dict)
    member_uses: list[tuple[str, str, bool, tuple[tuple[str, str], ...] | None]] = field(
        default_factory=list
    )
    """`fitter_(this)` in a member initializer list: the class, the member, whether a node
    was handed over and the handles, resolved once the field's declared type is known."""
    class_node_plugins: dict[str, set[str]] = field(default_factory=dict)
    helper_types: dict[tuple[str, str], set[str]] = field(default_factory=dict)
    helper_plugins: dict[tuple[str, str], set[str]] = field(default_factory=dict)
    node_handles: dict[tuple[str, str], set[bool]] = field(default_factory=dict)
    handle_uses: list[tuple[str, tuple[tuple[str, str], ...], tuple[str, ...]]] = field(
        default_factory=list
    )
    template_definitions: dict[str, int] = field(default_factory=dict)
    template_specializations: set[str] = field(default_factory=set)
    construction_calls: list[dict[str, Any]] = field(default_factory=list)
    construction_sites: list[tuple[str, tuple[str | None, ...], int]] = field(default_factory=list)
    includes: list[tuple[str, bool]] = field(default_factory=list)
    constructed_classes: set[str] = field(default_factory=set)
    source: bytes = b""
    tree: Any = None
    signatures: dict[str, list[tuple[int, list[str]]]] | None = None
    string_maps: dict[str, tuple[str, int]] = field(default_factory=dict)
    map_mutations: set[str] = field(default_factory=set)
    auto_declare: bool | str = False
    auto_declare_classes: dict[str, bool | str] = field(default_factory=dict)
    local_arguments: bool | str = False
    local_arguments_classes: dict[str, bool | str] = field(default_factory=dict)

    @property
    def syntax(self) -> Any:
        """A fresh root per walk. A held root keeps every node a walk visited, with the
        children list the binding caches on each, which is gigabytes over a large tree."""
        return None if self.tree is None else self.tree.root_node

    @property
    def symbols(self) -> list[str]:
        found = {item.sym for item in self.endpoints}
        found.update(item.sym for item in self.params)
        found.update(item.sym for item in self.reads)
        found.update(item.sym for item in self.callbacks)
        found.update(item.sym for item in self.broadcasts)
        found.update(item.sym for item in self.lookups)
        found.update(item.symbol for item in self.classes if item.symbol)
        found.update(item.symbol for item in self.plain_nodes)
        if self.constructions:
            found.add("main")
        return sorted(item for item in found if item)


@dataclass
class CodeIndex:
    """Every unit after the cross file joins: node class names, member values."""

    units: list[Unit] = field(default_factory=list)
    classes: dict[str, NodeClass] = field(default_factory=dict)
    members: dict[tuple[str, str], Ref] = field(default_factory=dict)


# ---------------------------------------------------------------------------
# small helpers over the tree


def _text(node: Any, src: bytes) -> str:
    return src[node.start_byte : node.end_byte].decode("utf-8", "replace")


def _flat(node: Any, src: bytes) -> str:
    return " ".join(_text(node, src).split())


_SCOPE_SPACE = re.compile(r"\s*::\s*")


def _line(node: Any) -> int:
    return int(node.start_point[0]) + 1


def _child(node: Any, name: str) -> Any:
    return node.child_by_field_name(name)


SPACED = {"binary_expression", "assignment_expression", "conditional_expression"}
VERBATIM = {
    "string_literal",
    "raw_string_literal",
    "char_literal",
    "concatenated_string",
    "user_defined_literal",
}
OPERANDS = {
    "identifier",
    "field_identifier",
    "qualified_identifier",
    "field_expression",
    "call_expression",
    "subscript_expression",
    "parenthesized_expression",
    "template_function",
    "this",
    "true",
    "false",
    "null",
    "nullptr",
    "number_literal",
    *VERBATIM,
}


def _canonical(node: Any, src: bytes) -> str:
    """Guard text with one spelling per expression: one space around a binary, assignment or
    ternary operator and after a comma, none elsewhere unless two words would merge."""
    if node.child_count == 0 or node.type in VERBATIM:
        return _flat(node, src)
    out = ""
    for child in node.children:
        if child.type == "comment":
            continue
        part = _canonical(child, src)
        if not child.is_named and node.type in SPACED:
            out = f"{out} {part} "
        elif part == ",":
            out += ", "
        elif out and _word(out[-1]) and _word(part[0]):
            out = f"{out} {part}"
        else:
            out += part
    return out.strip()


def _word(char: str) -> bool:
    return char.isalnum() or char == "_"


def _operand(node: Any) -> bool:
    """True when the expression needs no parentheses beside `&&`."""
    if node.type == "unary_expression":
        argument = _child(node, "argument")
        return argument is not None and _operand(argument)
    return node.type in OPERANDS


Guard = tuple[str, str, tuple[str, bool] | None]
"""(alone, grouped, term): the text of one guard, and the parameter with the value the guard
needs when the guard is exactly one parameter of the node."""


def _joined(guards: Sequence[Guard]) -> str:
    if len(guards) == 1:
        return guards[0][0]
    return " && ".join(grouped for _alone, grouped, _term in guards)


def _terms(guards: Sequence[Guard]) -> tuple[tuple[str, bool], ...]:
    """The parameter terms of a conjunction, or nothing when any term is source text."""
    terms = tuple(term for _alone, _grouped, term in guards if term is not None)
    return terms if len(terms) == len(guards) else ()


def _args(node: Any) -> list[Any]:
    arguments = _child(node, "arguments")
    if arguments is None:
        return []
    return [item for item in arguments.named_children if item.type != "comment"]


def _string_of(node: Any, src: bytes) -> str | None:
    if node.type == "string_literal":
        parts = [_text(item, src) for item in node.named_children if item.type == "string_content"]
        return _unescape("".join(parts))
    if node.type == "concatenated_string":
        pieces = []
        for item in node.named_children:
            piece = _string_of(item, src)
            if piece is None:
                return None
            pieces.append(piece)
        return "".join(pieces)
    return None


def _unescape(text: str) -> str:
    out: list[str] = []
    index = 0
    while index < len(text):
        char = text[index]
        if char == "\\" and index + 1 < len(text):
            index += 1
            out.append({"n": "\n", "t": "\t", "r": "\r", "0": "\0"}.get(text[index], text[index]))
        else:
            out.append(char)
        index += 1
    return "".join(out)


def _call_parts(node: Any, src: bytes) -> tuple[str, Any, Any]:
    """(bare name, receiver node, template argument list) of a call expression."""
    function = _child(node, "function")
    if function is None:
        return "", None, None
    template: Any = None
    receiver: Any = None
    target = function
    if target.type == "field_expression":
        receiver = _child(target, "argument")
        target = _child(target, "field") or target
    if target.type == "dependent_name" and target.named_children:
        # `node->template declare_parameter<double>(...)`
        target = target.named_children[0]
    if target.type in ("template_function", "template_method"):
        template = _child(target, "arguments")
        target = _child(target, "name") or target
    if target.type == "qualified_identifier":
        while target.type == "qualified_identifier":
            nxt = _child(target, "name")
            if nxt is None:
                break
            target = nxt
        if target.type in ("template_function", "template_method"):
            template = _child(target, "arguments")
            target = _child(target, "name") or target
    return _text(target, src), receiver, template


def _scope_text(node: Any, src: bytes) -> str:
    function = _child(node, "function")
    if function is None:
        return ""
    if function.type in ("template_function", "template_method"):
        function = _child(function, "name") or function
    if function.type == "qualified_identifier":
        return _text(function, src).rpartition("::")[0]
    if function.type == "field_expression":
        argument = _child(function, "argument")
        return _text(argument, src) if argument is not None else ""
    return ""


def _template_types(template: Any, src: bytes) -> list[str]:
    if template is None:
        return []
    return [
        _flat(item, src)
        for item in template.named_children
        if item.type in ("type_descriptor", "qualified_identifier", "type_identifier")
    ]


def interface_parts(text: str) -> tuple[str, str, str] | None:
    """`pkg::msg::Type` or `pkg/msg/Type` split for a `type` id."""
    parts = [item for item in re.split(r"::|/", text.strip()) if item]
    if len(parts) != 3 or parts[1] not in INTERFACE_DIRS:
        return None
    if not parts[0].isidentifier() or not parts[2].isidentifier():
        return None
    return (parts[0], parts[1], parts[2])


def interface_name(text: str) -> str:
    parts = interface_parts(text)
    return "/".join(parts) if parts else text


def qos_of(raw: str, *, variable: str = "", mutated: bool = False) -> dict[str, Any]:
    """The QoS properties an expression names, and nothing else.

    Every key is present so a reader never has to guess whether a property was absent or
    unread, and a property the source did not state is null rather than the rmw default.
    """
    text = " ".join(raw.split())
    out: dict[str, Any] = {
        "raw": variable or text,
        "form": "",
        "history": None,
        "depth": None,
        "reliability": None,
        "durability": None,
        "preset": None,
    }
    if variable:
        out["form"] = "variable"
        out["variable"] = variable
        out["mutated"] = mutated
    for name, preset in PRESETS.items():
        if name in text:
            out["preset"] = preset
            out["form"] = out["form"] or "preset"
    for name, value in RELIABILITY.items():
        if f".{name}(" in text:
            out["reliability"] = value
    for name, value in DURABILITY.items():
        if f".{name}(" in text:
            out["durability"] = value
    if "KeepAll" in text:
        out["history"] = "keep_all"
    else:
        depth = re.search(r"KeepLast\(\s*(\d+)\s*\)", text)
        if depth is None and not out["preset"]:
            depth = re.search(r"QoS\(\s*(\d+)\s*\)", text)
        if depth is None and re.fullmatch(r"\d+", text):
            depth = re.fullmatch(r"(\d+)", text)
            out["form"] = out["form"] or "depth"
        if depth is not None:
            out["history"] = "keep_last"
            out["depth"] = int(depth.group(1))
    if not out["form"]:
        out["form"] = "qos_object" if "QoS" in text else "expression"
    if out["form"] == "" or (out["form"] == "expression" and out["preset"]):
        out["form"] = "preset"
    if out["form"] not in ("variable", "depth", "preset") and "QoS" in text:
        out["form"] = "qos_object"
    return out


# ---------------------------------------------------------------------------
# reading one file


class _Reader:
    """One pass over one tree, carrying the scope chain that resolves a name."""

    def __init__(self, src: bytes, unit: Unit, defines: dict[str, str]) -> None:
        self.src = src
        self.unit = unit
        self.defines = defines
        self.namespaces: list[str] = []
        self.classes: list[str] = []
        self.functions: list[str] = []
        self.scopes: list[dict[str, Ref]] = [{}]
        self.frames: dict[str, dict[str, Ref]] = {}
        self.numbers: dict[str, tuple[float, str]] = {}
        self.direct_numbers: set[str] = set()
        self.defaults: dict[str, Ref] = {}
        self.exprs: dict[str, tuple[str, bool]] = {}
        self.member_types: dict[str, str] = {}
        self.conditions: list[Guard] = []
        self.member_conditions: dict[str, list[Guard]] = {}
        self.ctor_params: set[str] = set()
        self.parameter_positions: dict[str, int] = {}
        self.parameter_defaults: dict[str, str | None] = {}
        self.consumed: set[int] = set()
        self.node_selectors: dict[tuple[str, str], str] = {}
        self.constructor_arguments: dict[str, list[tuple[str | None, ...]]] = {}
        self.assigned: dict[tuple[str, str], int] = {}
        self.bound_under: dict[tuple[str, str], tuple[Guard, ...]] = {}

    # scope ---------------------------------------------------------------

    @property
    def klass(self) -> str:
        return self.classes[-1] if self.classes else ""

    @property
    def sym(self) -> str:
        if self.functions:
            return self.functions[-1]
        return "::".join(self.namespaces) if self.namespaces else ""

    def _inside_wrapper(self) -> bool:
        if not self.functions:
            return False
        tail = self.functions[-1].rpartition("::")[2]
        return any(
            symbol.rpartition("::")[2] == tail
            for symbol in self.unit.wrappers
            if symbol not in self.unit.lambda_wrappers
        )

    def bind(self, name: str, ref: Ref) -> None:
        if not name:
            return
        key = (self.klass, name) if name.endswith("_") else (self.sym, name)
        self.assigned[key] = self.assigned.get(key, 0) + 1
        self.bound_under[key] = tuple(self.conditions)
        if name.endswith("_") and self.klass:
            self.unit.members[(self.klass, name)] = ref
            return
        self.scopes[-1][name] = ref

    def lookup(self, name: str) -> Ref | None:
        for scope in reversed(self.scopes):
            if name in scope:
                return scope[name]
        return self.unit.members.get((self.klass, name))

    # resolution ----------------------------------------------------------

    def resolve(self, node: Any) -> Ref:
        expr = _flat(node, self.src)
        literal = _string_of(node, self.src)
        if literal is not None:
            return Ref(LITERAL, literal, expr, origin=LITERAL)
        if node.type in ("identifier", "field_identifier", "type_identifier"):
            found = self.lookup(expr)
            if found is not None:
                binding = found.binding or ("member" if expr.endswith("_") else "local")
                built = found.built or (found.expr if found.status == UNRESOLVED else "")
                key = (self.klass, expr) if expr.endswith("_") else (self.sym, expr)
                under = self.bound_under.get(key, ())
                if self.assigned.get(key, 0) > 1 and tuple(self.conditions[: len(under)]) != under:
                    # the last value was written in a branch this read is outside of, an if
                    # and an else among them, so the name is any of the values assigned
                    return Ref(UNRESOLVED, "", expr, binding=binding)
                return replace(found, expr=expr, binding=binding, built=built)
            if expr in self.defines:
                return Ref(LITERAL, self.defines[expr], expr, origin="macro", source_name=expr)
            if expr.endswith("_") and self.klass:
                return Ref(UNRESOLVED, "", expr, member=expr, owner=self.klass)
            return Ref(UNRESOLVED, "", expr)
        if node.type == "field_expression":
            argument = _child(node, "argument")
            member = _child(node, "field")
            if argument is not None and member is not None and argument.type == "this":
                return self.resolve(member)
            found = next((scope[expr] for scope in reversed(self.scopes) if expr in scope), None)
            if found is not None:
                return replace(found, expr=expr, binding="local")
            return Ref(UNRESOLVED, "", expr)
        if node.type == "parenthesized_expression" and node.named_child_count == 1:
            return replace(self.resolve(node.named_children[0]), expr=expr)
        if node.type == "binary_expression":
            return self._concat(node, expr)
        if node.type == "call_expression":
            return self._call_value(node, expr)
        return Ref(UNRESOLVED, "", expr)

    def _concat(self, node: Any, expr: str) -> Ref:
        operator = _child(node, "operator")
        left = _child(node, "left")
        right = _child(node, "right")
        if operator is None or left is None or right is None:
            return Ref(UNRESOLVED, "", expr)
        if _text(operator, self.src) != "+":
            return Ref(UNRESOLVED, "", expr)
        first = self.resolve(left)
        second = self.resolve(right)
        if second.status == LITERAL and (
            first.origin == "parameter"
            or (first.member and not (_plugin_prefix(first) and second.value.startswith(".")))
        ):
            return replace(
                first,
                expr=expr,
                value=first.value + second.value if first.status == LITERAL else "",
                parameter_suffix=first.parameter_suffix + second.value,
            )
        if first.status == LITERAL and second.status == LITERAL:
            carrier = first if first.origin not in ("", LITERAL) else second
            return Ref(
                LITERAL,
                first.value + second.value,
                expr,
                origin="concat",
                source_name=carrier.source_name,
                binding=carrier.binding,
            )
        if second.status == LITERAL and (
            first.status == SUFFIX or (_plugin_prefix(first) and second.value.startswith("."))
        ):
            # The head is the plugin instance name the yaml supplies, so what the source
            # states is the tail, and the separating dot belongs to neither half.
            tail = (first.value if first.status == SUFFIX else "") + second.value
            return Ref(SUFFIX, tail.lstrip("."), expr, origin="plugin_parameter")
        return Ref(UNRESOLVED, "", expr)

    def _call_value(self, node: Any, expr: str) -> Ref:
        name, _receiver, template = _call_parts(node, self.src)
        arguments = _args(node)
        if name == "as_string" and _receiver is not None and _receiver.type == "call_expression":
            reader, _, _ = _call_parts(_receiver, self.src)
            values = _args(_receiver)
            if reader in READ_CALLS and len(values) == 1:
                parameter = self.resolve(values[0])
                if parameter.status == LITERAL:
                    return Ref(
                        UNRESOLVED, "", expr, origin="parameter", source_name=parameter.value
                    )
        if name in VALUE_CALLS:
            if arguments:
                return replace(self.resolve(arguments[0]), expr=expr)
            return Ref(UNRESOLVED, "", expr)
        if name == "getName" and not arguments and self.klass:
            return Ref(UNRESOLVED, "", expr, origin=PLUGIN_NAME)
        declare = self._declare_shape(name, arguments, template)
        if declare is not None and declare[1] is not None:
            parameter = self.resolve(declare[0])
            origin = "plugin_parameter" if name in PLUGIN_DECLARE_CALLS else "parameter"
            return replace(
                self.resolve(declare[1]),
                expr=expr,
                origin=origin,
                source_name=parameter.value if parameter.resolved else "",
            )
        return Ref(UNRESOLVED, "", expr)

    def _declare_shape(
        self, name: str, arguments: Sequence[Any], template: Any
    ) -> tuple[Any, Any, list[str], str] | None:
        """(name node, default node, template types, wrapper symbol) for any declare
        spelling, or None.

        The project wrappers are found by their own body, not by a list of names: a function
        whose body declares a parameter and whose signature takes a name string is a declare
        wrapper, and two of them exist on the reference stack.
        """
        types = _template_types(template, self.src)
        if name in DECLARE_CALLS:
            values = (
                list(arguments[1:])
                if name in NODE_FIRST_DECLARE_CALLS
                else [item for item in arguments if not _is_node_argument(item, self.src)]
            )
            if not values:
                return None
            return (values[0], values[1] if len(values) > 1 else None, types, "")
        qualified = self._wrapper_symbol(name)
        if name in PLUGIN_DECLARE_CALLS and arguments and not qualified:
            return (arguments[0], arguments[1] if len(arguments) > 1 else None, types, "")
        wrapper = (
            (self.unit.wrappers.get(qualified) or self.unit.shared_wrappers.get(qualified))
            if qualified
            else None
        )
        if wrapper is not None:
            name_index, default_index = wrapper
            if name_index < len(arguments):
                default = arguments[default_index] if default_index < len(arguments) else None
                return (arguments[name_index], default, types, qualified)
        return None

    def _wrapper_symbol(self, name: str) -> str:
        for symbol in [*self.unit.wrappers, *self.unit.shared_wrappers]:
            if symbol.rpartition("::")[2] == name:
                return symbol
        return ""

    # walking -------------------------------------------------------------

    def walk(self, node: Any) -> None:
        kind = node.type
        if kind == "preproc_include":
            path = _child(node, "path")
            if path is not None and path.type in {"string_literal", "system_lib_string"}:
                self.unit.includes.append(
                    (_text(path, self.src)[1:-1], path.type == "string_literal")
                )
            return
        if kind == "namespace_definition":
            name = _child(node, "name")
            self.namespaces.append(_text(name, self.src) if name is not None else "(anonymous)")
            self._children(node)
            self.namespaces.pop()
            return
        if kind in ("class_specifier", "struct_specifier"):
            self._class(node)
            return
        if kind == "function_definition":
            self._function(node)
            return
        if kind == "declaration":
            self._declaration(node)
            return
        if kind == "assignment_expression":
            self._assignment(node)
            return
        if kind == "field_declaration":
            self._field_declaration(node)
            return
        if kind == "if_statement":
            self._if(node)
            return
        if kind == "case_statement" or kind in _LOOPS:
            # A switch arm or a loop body may not run, which states no guard but is a branch.
            self.conditions.append(("", str(node.start_byte), None))
            self._children(node)
            self.conditions.pop()
            return
        if kind == "call_expression":
            self._call(node)
            self._children(node)
            return
        if kind == "new_expression":
            type_node = _child(node, "type")
            if type_node is not None:
                self._node_helper("new", None, [_text(type_node, self.src)], _args(node))
        self._children(node)

    def _children(self, node: Any) -> None:
        for child in node.named_children:
            if child.type != "comment":
                self.walk(child)

    def _if(self, node: Any) -> None:
        """A guard on a parameter is stated by the parameter's name. Any other guard is a
        runtime test this pass cannot evaluate, so it is stated as written. Either is
        negated on the else arm, and a site under it never reads as unconditional."""
        condition = _child(node, "condition")
        guard: Guard = ("", "", None)
        negated = guard
        if condition is not None:
            inner = condition
            while inner.type in ("condition_clause", "parenthesized_expression"):
                if inner.named_child_count != 1:
                    break
                inner = inner.named_children[0]
            text = _canonical(inner, self.src)
            found = self.lookup(text)
            if found is not None and found.source_name:
                name = found.source_name
                parameter = found.origin == "parameter"
                guard = (name, name, (name, True) if parameter else None)
                negated = (f"!{name}", f"!{name}", (name, False) if parameter else None)
            else:
                guard = (text, text if _operand(inner) else f"({text})", None)
                negated = (f"!({text})", f"!({text})", None)
        alternative = _child(node, "alternative")
        self.conditions.append(guard)
        for child in node.named_children:
            if child.type != "comment" and child != alternative:
                self.walk(child)
        if alternative is not None:
            self.conditions[-1] = negated
            self.walk(alternative)
        self.conditions.pop()

    @property
    def guards(self) -> list[Guard]:
        return [item for item in self.conditions if item[0]]

    @property
    def condition(self) -> str:
        return _joined(self.guards)

    @property
    def guard(self) -> tuple[tuple[str, bool], ...]:
        return _terms(self.guards)

    def _member_guards(self, member: str) -> list[Guard]:
        """The guard the member was built under, outside the guard of this use."""
        outer = self.member_conditions.get(member, [])
        return [*outer, *(item for item in self.guards if item not in outer)]

    def guarded(self, member: str) -> str:
        return _joined(self._member_guards(member))

    def guard_of(self, member: str) -> tuple[tuple[str, bool], ...]:
        return _terms(self._member_guards(member))

    def _class(self, node: Any) -> None:
        name = _child(node, "name")
        label = _text(name, self.src) if name is not None else ""
        qualified = "::".join([*self.namespaces, label]) if label else ""
        if qualified:
            self.unit.class_names.append(qualified)
            if node.parent is not None and node.parent.type == "template_declaration":
                self.unit.template_definitions[qualified] = self.unit.template_definitions.get(
                    qualified, 0
                ) + int(_child(node, "body") is not None)
                if name.type != "type_identifier":
                    self.unit.template_specializations.add(qualified)
        base = ""
        bases: list[str] = []
        for child in node.named_children:
            if child.type != "base_class_clause":
                continue
            for candidate in child.named_children:
                if candidate.type == "access_specifier":
                    continue
                text = _text(candidate, self.src)
                bases.append(text)
                if text in NODE_BASES:
                    base = text
        if qualified and bases:
            self.unit.class_bases[qualified] = sorted(set(bases))
        if qualified and base:
            self.unit.classes.append(
                NodeClass(qualname=qualified, base=base, form=NODE_BASES[base])
            )
        self.classes.append(qualified)
        self.scopes.append({})
        body = _child(node, "body")
        if body is not None:
            # Fields first: a member initialised in the constructor is declared below it.
            for child in body.named_children:
                if child.type == "field_declaration":
                    self._field_declaration(child)
            for child in body.named_children:
                if child.type not in ("comment", "field_declaration"):
                    self.walk(child)
        self.scopes.pop()
        self.classes.pop()

    def _field_declaration(self, node: Any) -> None:
        default = _child(node, "default_value")
        type_node = _child(node, "type")
        type_text = _flat(type_node, self.src) if type_node is not None else ""
        # `Subscriber<CameraInfo> sub_depth_info_, sub_rgb_info_;` declares two members.
        for declarator in node.children_by_field_name("declarator") if self.klass else ():
            member = _text(declarator, self.src).strip("&* ")
            if declarator.type == "field_identifier" and type_text and "<" not in type_text:
                self.unit.field_types[(self.klass, member)] = type_text
            if default is not None:
                self.unit.members[(self.klass, member)] = replace(
                    self.resolve(default), origin="constant", source_name=member
                )
            wrapper = _wrapper_of(type_text)
            if wrapper:
                self.member_types[member] = wrapper
                self.unit.member_wrappers[(self.klass, member)] = wrapper
                head = _template_head(type_text, wrapper)
                inner = re.search(r"<(.+)>", type_text)
                if head or inner is not None:
                    self.member_types[f"{member}::type"] = head or inner.group(1).strip()
                if default is not None and default.type == "initializer_list":
                    # `odom_sub_{this, "~/input/odometry"}` is built by every constructor.
                    constructor = f"{self.klass}::{self.klass.rpartition('::')[2]}"
                    self._named_wrapper_endpoints(member, wrapper, default, sym=constructor)
        self._children(node)

    def _function(self, node: Any) -> None:
        declarator = _child(node, "declarator")
        qualified, owner = self._function_name(declarator)
        body = _child(node, "body")
        parent = node.parent
        if parent is not None and parent.type == "template_declaration" and "<" in owner:
            parameters = _child(parent, "parameters")
            if parameters is not None and not parameters.named_child_count:
                self.unit.template_specializations.add(owner)
        self.functions.append(qualified)
        pushed = False
        if owner and owner != self.klass:
            self.classes.append(owner)
            pushed = True
        self.scopes.append({})
        options = self._parameters(declarator)
        self._plugin_name_parameter(declarator, qualified)
        if qualified:
            self.unit.methods.append(qualified)
            if body is not None and not body.named_child_count:
                self.unit.empty_bodies.append(qualified)
        for child in node.named_children:
            if child.type == "field_initializer_list":
                self._base_initializer(child, qualified, options)
            elif child.type != "comment":
                self.walk(child)
        self.scopes.pop()
        if pushed:
            self.classes.pop()
        self.functions.pop()

    def _plugin_name_parameter(self, declarator: Any, qualified: str) -> None:
        """The one string a plugin's `configure` or `initialize` takes is its instance name,
        which the yaml supplies, so a parameter named from it is a suffix of that name."""
        if not self.klass or qualified.rpartition("::")[2] not in PLUGIN_SETUP_METHODS:
            return
        target = declarator
        while target is not None and target.type != "function_declarator":
            target = _child(target, "declarator")
        parameters = _child(target, "parameters") if target is not None else None
        if parameters is None:
            return
        names = []
        for child in parameters.named_children:
            type_node = _child(child, "type")
            inner = _child(child, "declarator")
            if type_node is None or inner is None:
                continue
            if _flat(type_node, self.src).removeprefix("const ") == "std::string":
                names.append(_text(inner, self.src).strip("&* "))
        if len(names) == 1 and names[0]:
            self.scopes[-1][names[0]] = Ref(UNRESOLVED, "", names[0], origin=PLUGIN_NAME)
            self.unit.plugin_name_locals.add(names[0])

    def _function_name(self, declarator: Any) -> tuple[str, str]:
        target = declarator
        while target is not None and target.type != "function_declarator":
            target = _child(target, "declarator")
        if target is None:
            return "", ""
        inner = _child(target, "declarator")
        if inner is None:
            return "", ""
        # `Foo<T>::` and `Foo` on two lines are one name however a formatter wraps them
        text = _SCOPE_SPACE.sub("::", _flat(inner, self.src))
        if inner.type == "qualified_identifier":
            scope = (
                _SCOPE_SPACE.sub("::", _flat(_child(inner, "scope"), self.src))
                if _child(inner, "scope")
                else ""
            )
            owner = "::".join([*self.namespaces, scope]) if scope else ""
            return "::".join([*self.namespaces, text]), owner
        prefix = self.classes or self.namespaces
        return "::".join([*prefix[-1:], text]) if prefix else text, self.klass

    def _parameters(self, declarator: Any) -> bool:
        target = declarator
        while target is not None and target.type != "function_declarator":
            target = _child(target, "declarator")
        if target is None:
            return False
        parameters = _child(target, "parameters")
        if parameters is None:
            return False
        options = False
        self.ctor_params = set()
        self.parameter_positions = {}
        self.parameter_defaults = {}
        arguments = [child for child in parameters.named_children if child.type != "comment"]
        # an overload that cannot take the call's arguments is not the one the call runs
        required = sum(child.type == "parameter_declaration" for child in arguments)
        accepted = (
            len(arguments)
            if not any(child.type.startswith("variadic") for child in arguments)
            else None
        )
        for position, child in enumerate(arguments):
            if child.type not in ("parameter_declaration", "optional_parameter_declaration"):
                continue
            type_node = _child(child, "type")
            if type_node is not None and "NodeOptions" in _text(type_node, self.src):
                options = True
            inner = _child(child, "declarator")
            if inner is None:
                continue
            text = _text(inner, self.src).strip("&* ")
            if text:
                self.scopes[-1][text] = Ref(UNRESOLVED, "", text, origin="argv")
                self.ctor_params.add(text)
                self.parameter_positions[text] = position
                sites = self.constructor_arguments.get(self.klass, [])
                if (
                    self.sym.rsplit("::", 1)[-1] == self.klass.rsplit("::", 1)[-1]
                    and len(sites) == 1
                    and position < len(sites[0])
                    and sites[0][position] is not None
                    and required <= len(sites[0]) <= (accepted or len(sites[0]))
                ):
                    self.scopes[-1][text] = (
                        Ref(UNRESOLVED, "", text, origin=PLUGIN_NAME)
                        if sites[0][position] == PLUGIN_NAME_ARGUMENT
                        else Ref(LITERAL, sites[0][position], text, origin="constructor_argument")
                    )
                default = _child(child, "default_value")
                if default is not None:
                    self.parameter_defaults[text] = _string_of(default, self.src)
        return options

    def _base_initializer(self, node: Any, symbol: str, options: bool) -> None:
        for child in node.named_children:
            if child.type != "field_initializer":
                continue
            named = child.named_children
            if len(named) < 2:
                continue
            base = _text(named[0], self.src)
            if base not in NODE_BASES:
                values = named[1].named_children
                if len(values) == 1:
                    value = self.resolve(values[0])
                    if value.status == LITERAL:
                        self.bind(base, value)
                self._member_wrapper(base, named[1])
                if self.klass:
                    arguments = [item for item in values if item.type != "comment"]
                    self.unit.member_uses.append(
                        (
                            self.klass,
                            base,
                            any(_is_node_argument(item, self.src) for item in arguments),
                            self._handles(arguments),
                        )
                    )
                # `encoding_(declare_parameter<std::string>("encoding"))`
                self.walk(named[1])
                continue
            values = [item for item in named[1].named_children if item.type != "comment"]
            if not values:
                continue
            ref = self.resolve(values[0])
            argv = _flat(values[0], self.src) in self.ctor_params
            owner = self.klass
            if not owner:
                continue
            self.unit.classes.append(
                NodeClass(
                    qualname=owner,
                    base=base,
                    form=NODE_BASES[base],
                    stated_name=ref.value if ref.status == LITERAL and not argv else "",
                    name_source=model.NameSource.LITERAL.value
                    if ref.status == LITERAL and not argv
                    else (model.NameSource.ARGV.value if argv else model.NameSource.UNKNOWN.value),
                    name_expr=ref.expr,
                    symbol=symbol,
                    takes_node_options=options,
                    name_line=_line(values[0]),
                    name_position=self.parameter_positions.get(_flat(values[0], self.src)),
                    name_default=self.parameter_defaults.get(_flat(values[0], self.src)),
                )
            )

    def _member_wrapper(self, member: str, arguments: Any) -> None:
        wrapper = self.member_types.get(member) or self.unit.shared_members.get(
            (self.klass.rpartition("::")[2], member)
        )
        if not wrapper:
            return
        self._wrapper_endpoints(
            wrapper, f"{wrapper}({_flat(arguments, self.src)[1:-1]})", arguments, conditional=False
        )
        # `odom_sub_(this, "~/input/localization", qos)` names its topic like a declaration.
        self._named_wrapper_endpoints(member, wrapper, arguments)
        listed = [item for item in arguments.named_children if item.type != "comment"]
        if wrapper == "message_filters::Subscriber" and len(listed) > 1:
            qos = listed[2] if len(listed) > 2 else None
            self._filter_endpoint(member, listed[1], arguments, "Subscriber", qos)

    def _declaration(self, node: Any) -> None:
        type_node = _child(node, "type")
        type_text = _flat(type_node, self.src) if type_node is not None else ""
        for child in node.named_children:
            if child.type != "init_declarator":
                continue
            target = _child(child, "declarator")
            value = _child(child, "value")
            if target is None or value is None:
                continue
            name = _text(target, self.src).strip("&* ").removesuffix("[]")
            if value.type == "argument_list":
                # `rclcpp::QoS latched_qos(1);` declares a variable, and the type is half of
                # what the variable holds.
                self.exprs[name] = (f"{type_text}{_flat(value, self.src)}", False)
                self._plain_node(
                    type_text, [item for item in value.named_children if item.type != "comment"]
                )
                self._track_wrapper(name, type_text, value)
                self.walk(value)
                continue
            self._track_helper(name, value)
            ref = self.resolve(value)
            if "constexpr" in _flat(node, self.src).partition("=")[0] and ref.origin == LITERAL:
                qualified = "::".join([*self.namespaces, name]) if self.namespaces else name
                ref = replace(ref, origin="constant", source_name=qualified)
            self.bind(name, ref)
            self.exprs[name] = (_flat(value, self.src), False)
            self._track_number(name, value)
            self._track_wrapper(name, type_text, value)
            self.walk(value)
        for child in node.named_children:
            if child.type not in (
                "init_declarator",
                "identifier",
                "pointer_declarator",
                "reference_declarator",
                "array_declarator",
            ):
                self.walk(child)

    def _assignment(self, node: Any) -> None:
        left = _child(node, "left")
        right = _child(node, "right")
        if left is None or right is None:
            self._children(node)
            return
        text = _flat(left, self.src)
        self._track_helper(text.removeprefix("this->"), right)
        if left.type == "field_expression":
            self._frame_assignment(left, right)
            argument = _child(left, "argument")
            if argument is not None and argument.type == "this":
                member = _child(left, "field")
                if member is not None:
                    self._track_number(_flat(member, self.src), right)
        elif left.type == "identifier":
            operator = _child(node, "operator")
            value = self.resolve(right)
            expr = _flat(right, self.src)
            if operator is not None and _text(operator, self.src) != "=":
                # `ns += ".speed_calculation"` extends ns, it does not replace it
                expr, value = self._extended(left, right, _text(operator, self.src) == "+=")
            self.bind(text, value)
            self.exprs[text] = (expr, False)
            self._track_number(text, right)
            self._track_wrapper(text, "", right)
        self.walk(right)

    def _extended(self, left: Any, right: Any, appends: bool = True) -> tuple[str, Ref]:
        """What `left += right` leaves in left. A literal head is spelled as the literal, so a
        name built from it still says which keys it could be."""
        held, value = self.resolve(left), self.resolve(right)
        head = f'"{held.value}"' if held.status == LITERAL else _flat(left, self.src)
        expr = f"{head} + {_flat(right, self.src)}"
        if appends and held.status == LITERAL and value.status == LITERAL:
            return expr, Ref(LITERAL, held.value + value.value, expr)
        return expr, Ref(UNRESOLVED, "", expr)

    def _append(self, receiver: Any, arguments: Sequence[Any]) -> None:
        """`key.append(id)` extends key the way `key += id` does."""
        if len(arguments) != 1 or self.resolve(receiver).status != LITERAL:
            return
        text = _flat(receiver, self.src)
        expr, value = self._extended(receiver, arguments[0])
        self.bind(text, value)
        self.exprs[text] = (expr, False)

    def _track_number(self, name: str, value: Any) -> None:
        number, source = self._number(value)
        direct = self._number_direct(value)
        self.numbers.pop(name, None)
        self.direct_numbers.discard(name)
        parent = value.parent
        operator = _child(parent, "operator") if parent is not None else None
        if operator is not None and _text(operator, self.src) != "=":
            return
        if number is not None:
            self.numbers[name] = (number, source)
            if source and direct:
                self.direct_numbers.add(name)

    def _number_direct(self, node: Any) -> bool:
        if node is None:
            return False
        if node.type == "parenthesized_expression" and node.named_child_count == 1:
            return self._number_direct(node.named_children[0])
        if node.type == "call_expression":
            name, receiver, template = _call_parts(node, self.src)
            if name.startswith("as_"):
                return self._number_direct(receiver)
            return (
                name in READ_CALLS or self._declare_shape(name, _args(node), template) is not None
            )
        return _flat(node, self.src).removeprefix("this->") in self.direct_numbers

    def _number(self, node: Any) -> tuple[float | None, str]:
        """A numeric value and the parameter it came from, following one function.

        A result json cannot hold is not a number this layer can state, so it leaves by the
        same door as an expression that did not fold."""
        number, source = self._number_value(node)
        if number is not None and not math.isfinite(number):
            return None, ""
        return number, source

    def _number_value(self, node: Any) -> tuple[float | None, str]:
        if node is None:
            return None, ""
        literal = _literal_value(node, self.src)
        if isinstance(literal, (int, float)) and not isinstance(literal, bool):
            return float(literal), ""
        text = _flat(node, self.src)
        if node.type in ("identifier", "field_identifier"):
            found = self.numbers.get(text)
            return found if found is not None else (None, "")
        if node.type == "field_expression":
            argument = _child(node, "argument")
            if argument is not None and argument.type == "this":
                member = _child(node, "field")
                return self._number(member) if member is not None else (None, "")
            return None, ""
        if node.type == "parenthesized_expression" and node.named_child_count == 1:
            return self._number(node.named_children[0])
        if node.type == "binary_expression":
            left, right = _child(node, "left"), _child(node, "right")
            operator = _child(node, "operator")
            if left is None or right is None or operator is None:
                return None, ""
            first, first_source = self._number(left)
            second, second_source = self._number(right)
            if first is None or second is None:
                return None, ""
            symbol = _text(operator, self.src)
            source = first_source or second_source
            try:
                if symbol == "/":
                    return first / second, source
                if symbol == "*":
                    return first * second, source
                if symbol == "+":
                    return first + second, source
                if symbol == "-":
                    return first - second, source
            except ZeroDivisionError:
                return None, ""
        if node.type == "call_expression":
            arguments = _args(node)
            name, receiver, template = _call_parts(node, self.src)
            declare = self._declare_shape(name, arguments, template)
            if declare is not None:
                parameter = self.resolve(declare[0])
                value, _ = self._number(declare[1])
                return value, parameter.value if parameter.resolved else ""
            if name.startswith("as_") and receiver is not None:
                return self._number(receiver)
            if name in READ_CALLS and arguments:
                parameter = self.resolve(arguments[0])
                return self.numbers.get(parameter.value, (None, ""))
            if name in ("duration", "double") and arguments:
                return self._number(arguments[0])
        return None, ""

    def _track_wrapper(self, name: str, type_text: str, value: Any) -> None:
        wrapper = _wrapper_of(type_text) or _wrapper_of(_flat(value, self.src))
        if not wrapper:
            return
        self.member_types[name] = wrapper
        # Built at several sites, only the guards every site shares hold.
        built = self.member_conditions.get(name)
        self.member_conditions[name] = [
            item for item in self.guards if built is None or item in built
        ]
        raw = f"{wrapper}({_flat(_construction_args(value), self.src)[1:-1]})"
        self._wrapper_endpoints(wrapper, raw, value)
        self._named_wrapper_endpoints(name, wrapper, value)
        if wrapper == "message_filters::Subscriber":
            message = _template_head(_flat(value, self.src), wrapper) or _template_head(
                type_text, wrapper
            )
            if message:
                self.member_types[f"{name}::type"] = message
            arguments = _construction_args(value)
            listed = [
                item
                for item in (arguments.named_children if arguments is not None else [])
                if item.type != "comment"
            ]
            if arguments is not None and arguments.type == "argument_list" and len(listed) > 1:
                qos = listed[2] if len(listed) > 2 else None
                self._filter_endpoint(name, listed[1], value, "Subscriber", qos)

    def _named_wrapper_endpoints(
        self, member: str, wrapper: str, value: Any, sym: str = ""
    ) -> None:
        """The endpoints a wrapper class creates under the name its constructor is handed,
        `TwistPublisher(node, "cmd_vel")`. message_filters and image_transport have readers
        of their own."""
        row = wrappers.lookup(wrapper, "cpp")
        if row is None or row.argument is None or wrapper in _OWN_READERS:
            return
        arguments = _construction_args(value)
        listed = [
            item
            for item in (arguments.named_children if arguments is not None else [])
            if item.type != "comment"
        ]
        if (
            arguments is None
            or arguments.type not in ("argument_list", "initializer_list")
            or len(listed) <= row.argument
        ):
            return
        ref = self.resolve(listed[row.argument])
        sym = sym or self.sym
        for item in row.endpoints:
            if item.name_source != "argument":
                continue
            self.unit.endpoints.append(
                Endpoint(
                    kind=item.kind,
                    name=ref,
                    sym=sym,
                    owner_class=self.klass,
                    line=_line(value),
                    api=wrapper,
                    type_text=item.type,
                    qos=None,
                    qos_reason=str(KNOWN_WRAPPERS[wrapper]["reason"]),
                    condition=self.guarded(member),
                    guard=self.guard_of(member),
                )
            )

    def _wrapper_endpoints(
        self, wrapper: str, raw: str, site: Any, *, conditional: bool = True
    ) -> None:
        """The endpoints a known wrapper creates that no create_ call in this file names."""
        reason = KNOWN_WRAPPERS[wrapper].get("reason", "")
        topics: list[str] = []
        for kind, topic, message in wrapper_endpoints(wrapper):
            topics.append(topic)
            self.unit.endpoints.append(
                Endpoint(
                    kind=kind,
                    name=Ref(LITERAL, topic, raw, origin="wrapper"),
                    sym=self.sym,
                    owner_class=self.klass,
                    line=_line(site),
                    api=wrapper,
                    type_text=message,
                    qos=None,
                    qos_reason=reason,
                    via_known_wrapper=wrapper,
                    condition=self.condition,
                    guard=self.guard,
                    conditional_site=conditional,
                )
            )
        if topics or wrapper_takes_name(wrapper):
            self.unit.wrapper_uses.append(
                WrapperUse(
                    symbol=wrapper, kind="known_wrapper", line=_line(site), topics=tuple(topics)
                )
            )

    def _frame_assignment(self, left: Any, right: Any) -> None:
        """`msg.header.frame_id = "odom"` a few lines above a `sendTransform`."""
        member = _child(left, "field")
        name = _text(member, self.src) if member is not None else ""
        if name not in ("frame_id", "child_frame_id"):
            argument = _child(left, "argument")
            if argument is not None and argument.type == "this":
                self.bind(name, self.resolve(right))
            elif argument is not None and argument.type == "identifier":
                # `desc.name = "negative"; declare_parameter(desc.name, ...)`
                self.scopes[-1][_flat(left, self.src)] = self.resolve(right)
            return
        base = left
        while base.type == "field_expression":
            argument = _child(base, "argument")
            if argument is None:
                break
            base = argument
        holder = _text(base, self.src)
        slot = "parent" if name == "frame_id" else "child"
        self.frames.setdefault(holder, {})[slot] = self.resolve(right)

    # calls ---------------------------------------------------------------

    def _call(self, node: Any) -> None:
        name, receiver, template = _call_parts(node, self.src)
        if not name:
            return
        arguments = _args(node)
        types = _template_types(template, self.src)
        if name == "publish" and receiver is not None and self.klass:
            member = _flat(receiver, self.src).removeprefix("this->")
            if re.fullmatch(r"\w+", member):
                symbol = self.sym
                parent = node.parent
                while parent is not None and parent.type != "function_definition":
                    if parent.type == "lambda_expression":
                        symbol = self._lambda_symbol(parent)
                        break
                    parent = parent.parent
                self.unit.publishing.setdefault((self.klass, member), set()).add(symbol)
        if receiver is None and name[:1].isupper():
            # `VehicleInfoUtils(*this).getVehicleInfo()` constructs a temporary helper
            scope = _scope_text(node, self.src)
            self._node_helper("new", None, [f"{scope}::{name}" if scope else name], arguments)
        else:
            scope = _scope_text(node, self.src) if receiver is None else ""
            if scope.rpartition("::")[2][:1].isupper():
                # `NodeParam::init(*this)` hands the node to a static method of that class
                self._node_helper("new", None, [scope], arguments)
            self._node_helper(name, receiver, types, arguments)
        if name in ("make_shared", "make_unique"):
            self._plain_node(types[0] if types else _scope_text(node, self.src), arguments)
        if name in MACROS:
            self._macro(name, arguments, node)
            return
        if name in TIMER_CALLS:
            self._timer(name, arguments, node)
            return
        if name in BROADCAST_CALLS:
            self._broadcast(receiver, arguments, node)
            return
        if name in LOOKUP_CALLS:
            self._lookup(name, arguments, node)
            return
        if name in CALLBACK_CALLS:
            self._callback(name, arguments, node)
            return
        if name in IMAGE_MEMBER_CALLS and receiver is not None:
            self._image_endpoints(f"image_transport::ImageTransport::{name}", arguments, node)
            return
        if name == "subscribe" and receiver is not None:
            self._filter_subscribe(receiver, arguments, node)
            return
        if name.startswith("as_") and receiver is not None and receiver.type == "call_expression":
            self._accessor_read(name, receiver, node)
            return
        if self._endpoint(name, node, template, arguments):
            return
        if name == "append" and receiver is not None and receiver.type == "identifier":
            self._append(receiver, arguments)
        self._mutation(name, receiver)
        self._parameter(name, arguments, template, node)

    def _helper_key(self, name: str) -> tuple[str, str]:
        return (self.klass if name.endswith("_") else self.sym, name)

    def _selector(self, node: Any) -> str:
        if node.type == "identifier":
            return self.node_selectors.get((self.sym, _flat(node, self.src)), "")
        if node.type != "call_expression":
            return ""
        name, receiver, _template = _call_parts(node, self.src)
        if name.startswith("as_") and receiver is not None:
            return self._selector(receiver)
        arguments = _args(node)
        if name in READ_CALLS | DECLARE_CALLS and arguments:
            return _string_of(arguments[0], self.src) or ""
        return ""

    def _track_helper(self, name: str, node: Any) -> None:
        if not self.klass or not name.isidentifier():
            return
        selector = self._selector(node)
        self.node_selectors[(self.sym, name)] = selector
        key = self._helper_key(name)
        self.unit.node_handles.setdefault(key, set()).add(_self_handle(node, self.src))
        if node.type != "call_expression":
            if key in self.unit.helper_types:
                self.unit.helper_types[key].add("")
            if key in self.unit.helper_plugins:
                self.unit.helper_plugins[key].add("")
            return
        called, _receiver, template = _call_parts(node, self.src)
        arguments = _args(node)
        types = _template_types(template, self.src)
        if called in ("make_unique", "make_shared") and types:
            self.unit.helper_types.setdefault(key, set()).add(types[0])
        elif called in ("createSharedInstance", "createUniqueInstance") and arguments:
            parameter = self._selector(arguments[0])
            self.unit.helper_plugins.setdefault(key, set()).add(parameter)
        else:
            if key in self.unit.helper_types:
                self.unit.helper_types[key].add("")
            if key in self.unit.helper_plugins:
                self.unit.helper_plugins[key].add("")

    def _plain_node(self, created: str, arguments: Sequence[Any]) -> None:
        """A node a free function creates from a node type, named by its first argument."""
        form = PLAIN_NODE_TYPES.get(created.removeprefix("::"))
        if form is None or not arguments or self.klass or not self.functions:
            return
        ref = self.resolve(arguments[0])
        literal = ref.status == LITERAL
        self.unit.plain_nodes.append(
            NodeClass(
                qualname=created.removeprefix("::"),
                base=created.removeprefix("::"),
                form=form,
                stated_name=ref.value if literal else "",
                name_source=(
                    model.NameSource.LITERAL.value if literal else model.NameSource.UNKNOWN.value
                ),
                name_expr=ref.expr,
                symbol=self.sym,
                name_line=_line(arguments[0]),
                name_reason="" if literal else "not_stated_in_source",
            )
        )

    def _node_helper(
        self, name: str, receiver: Any, types: Sequence[str], arguments: Sequence[Any]
    ) -> None:
        """A class this node class hands itself to, directly or through a variable that holds
        it, which `_finish_node_helpers` decides once every assignment is read."""
        if not self.klass:
            return
        if any(_is_node_handle(argument, self.src) for argument in arguments):
            member = _flat(receiver, self.src).removeprefix("this->") if receiver else ""
            if member.isidentifier():
                # `smoother_.initialize(this)` hands the node to the class the field holds
                self.unit.member_uses.append((self.klass, member, True, None))
            if name in ("make_unique", "make_shared", "new") and types:
                self.unit.node_handoffs.setdefault(self.klass, set()).add(types[0])
            elif (
                receiver is None
                and name.isidentifier()
                and name not in NODE_USERS | PUBLISH_CALLS | SUBSCRIBE_CALLS
                and name not in SERVICE_CALLS | TIMER_CALLS | CAMERA_CALLS | DECLARE_CALLS
                and name not in READ_CALLS | ACTION_FACTORY_CALLS.keys()
            ):
                self.unit.node_handoffs.setdefault(self.klass, set()).add(name + "()")
        handles = self._handles(arguments)
        if handles is None:
            return
        if name in ("make_unique", "make_shared", "new") and types:
            self.unit.handle_uses.append((self.klass, handles, ("type", types[0])))
        elif receiver is not None:
            target = _flat(receiver, self.src).removeprefix("this->")
            if target.isidentifier():
                self.unit.handle_uses.append(
                    (self.klass, handles, ("receiver", *self._helper_key(target)))
                )

    def _handles(self, arguments: Sequence[Any]) -> tuple[tuple[str, str], ...] | None:
        """No handle for `this`, the variables that may hold the node, or None for neither."""
        texts = [_flat(argument, self.src).removeprefix("this->") for argument in arguments]
        if any(text in _SELF_HANDLES for text in texts):
            return ()
        handles = tuple(self._helper_key(text) for text in texts if text.isidentifier())
        return handles or None

    def _endpoint(self, name: str, node: Any, template: Any, arguments: Sequence[Any]) -> bool:
        function = _child(node, "function")
        head = _text(function, self.src) if function is not None else ""
        if name in CAMERA_CALLS or (name in IMAGE_CALLS and head.startswith("image_transport::")):
            self._image_endpoints(f"image_transport::{name}", arguments, node)
            return True
        action = head.startswith("rclcpp_action::")
        factory = name in ACTION_FACTORY_CALLS
        if action and name in ("create_client", "create_server"):
            kind = "action_client" if name == "create_client" else "action_server"
        elif factory:
            kind = ACTION_FACTORY_CALLS[name]
        elif name in PUBLISH_CALLS:
            kind = "publisher"
        elif name in SUBSCRIBE_CALLS:
            kind = "subscriber"
        elif name in SERVICE_CALLS:
            kind = "service_server"
        elif name in CLIENT_CALLS:
            kind = "service_client"
        else:
            return False
        if not arguments:
            return True
        if (
            not action
            and not factory
            and len(arguments) > 1
            and _is_node_handle(arguments[0], self.src)
        ):
            # The free function spelling, rclcpp::create_publisher(node, topic, qos).
            arguments = arguments[1:]
        types = _template_types(template, self.src)
        generic = name in GENERIC_CALLS
        if generic and len(arguments) > 1 and not types:
            spelled = self.resolve(arguments[1])
            if spelled.status == LITERAL and isinstance(spelled.value, str):
                types = [spelled.value.replace("/", "::")]
        qos = None
        options = ""
        position = 0
        at = 2 if generic else 1
        if kind in ("publisher", "subscriber") and len(arguments) > at:
            qos = self._qos(arguments[at])
        if kind == "subscriber" and len(arguments) > at + 2:
            options = _flat(arguments[at + 2], self.src)
            position = at + 3
        interface_arity = 8 if name == "create_server" else 5
        name_position = (4 if len(arguments) >= interface_arity else 1) if action else 0
        if factory and function is not None and function.type == "qualified_identifier":
            # `nav2::interfaces::create_action_client<T>(node, name)` takes the node first,
            # whatever the handle is called, and the member form takes the name first
            name_position = 1
        if name_position >= len(arguments):
            return True
        ref = self.resolve(arguments[name_position])
        self.unit.endpoints.append(
            Endpoint(
                kind=kind,
                name=ref,
                sym=self.sym,
                owner_class=self.klass,
                line=_line(node),
                api=name,
                type_text=types[0] if types else "",
                qos=qos,
                options_argument=options,
                options_position=position,
                condition=self.condition,
                guard=self.guard,
                unknown_count="loop" if self._in_loop(node) else "",
                attribute=self._publisher_member(node) if kind == "publisher" else "",
            )
        )
        return True

    def _publisher_member(self, node: Any) -> str:
        parent = node.parent
        if parent is None or not self.klass:
            return ""
        target = _child(parent, "left") if parent.type == "assignment_expression" else None
        member = _flat(target, self.src).removeprefix("this->") if target is not None else ""
        return member if re.fullmatch(r"\w+", member) else ""

    def _lambda_symbol(self, node: Any) -> str:
        return f"{self.sym}::lambda_{ids.digest(_flat(node, self.src))}"

    def _image_endpoints(self, symbol: str, arguments: Sequence[Any], node: Any) -> None:
        row = wrappers.lookup(symbol, "cpp")
        if row is None or row.argument is None or row.argument >= len(arguments):
            return
        ref = self.resolve(arguments[row.argument])
        position = IMAGE_QOS_POSITION.get(symbol, len(arguments))
        qos = self._qos(arguments[position]) if position < len(arguments) else None
        for item in row.endpoints:
            if ref.status == LITERAL:
                name = replace(ref, value=item.resolve(ref.value))
            elif item.name_source == "argument":
                name = ref
            else:
                # The sibling of a name nobody can read: the base topic is the blind spot.
                continue
            self.unit.endpoints.append(
                Endpoint(
                    kind=item.kind,
                    name=name,
                    sym=self.sym,
                    owner_class=self.klass,
                    line=_line(node),
                    api=symbol,
                    type_text=item.type,
                    qos=qos,
                    condition=self.condition,
                    guard=self.guard,
                    unknown_count="loop" if self._in_loop(node) else "",
                )
            )

    def _in_loop(self, node: Any) -> bool:
        parent = node.parent
        while parent is not None:
            if parent.type in _LOOPS:
                return True
            if parent.type == "function_definition":
                return False
            parent = parent.parent
        return False

    def _qos(self, node: Any) -> dict[str, Any]:
        text = _flat(node, self.src)
        if node.type == "identifier" and text in self.exprs:
            expression, mutated = self.exprs[text]
            return qos_of(expression, variable=text, mutated=mutated)
        return qos_of(text)

    def _mutation(self, name: str, receiver: Any) -> None:
        """`rclcpp::QoS latched_qos(1); latched_qos.transient_local();` is one profile written
        over two statements, so the mutation is appended to what the variable holds."""
        if receiver is None or receiver.type != "identifier":
            return
        target = _text(receiver, self.src)
        current = self.exprs.get(target)
        if current is None or (name not in RELIABILITY and name not in DURABILITY):
            return
        self.exprs[target] = (f"{current[0]}.{name}()", True)

    def _filter_subscribe(self, receiver: Any, arguments: Sequence[Any], node: Any) -> None:
        member = _text(receiver, self.src)
        wrapper = self.member_types.get(member)
        if wrapper is None and len(arguments) > 1:
            # A member declared in a header this file does not walk, told apart by its
            # arguments: ImageTransport::subscribe(topic, queue_size, ...),
            # SubscriberFilter::subscribe(node, topic, transport, ...) and
            # message_filters::Subscriber::subscribe(node, topic, qos).
            # ponytail: a spelling heuristic, a walk of the class header would know the type.
            if arguments[1].type == "number_literal":
                symbol = "image_transport::ImageTransport::subscribe"
                self._image_endpoints(symbol, arguments, node)
                return
            third = _flat(arguments[2], self.src) if len(arguments) > 2 else ""
            if _is_node_argument(arguments[0], self.src) and (
                third.startswith('"') or "Transport" in third
            ):
                wrapper = "image_transport::SubscriberFilter"
            elif _is_node_argument(arguments[0], self.src) and "qos" in third.lower():
                wrapper = "message_filters::Subscriber"
        if wrapper == "image_transport::SubscriberFilter":
            self._image_endpoints(wrapper, arguments, node)
            return
        if wrapper != "message_filters::Subscriber" or len(arguments) < 2:
            return
        qos = arguments[2] if len(arguments) > 2 else None
        self._filter_endpoint(member, arguments[1], node, "subscribe", qos)

    def _filter_endpoint(
        self, member: str, topic: Any, node: Any, api: str, qos: Any = None
    ) -> None:
        """The subscription a message_filters::Subscriber makes, by `subscribe(node, topic,
        qos)` or by the constructor taking the same arguments. Without a qos argument the
        profile is the one message_filters picks, which this source does not state."""
        wrapper = "message_filters::Subscriber"
        self.unit.endpoints.append(
            Endpoint(
                kind="subscriber",
                name=self.resolve(topic),
                sym=self.sym,
                owner_class=self.klass,
                line=_line(node),
                api=api,
                type_text=self.member_types.get(f"{member}::type", ""),
                qos=self._qos(qos) if qos is not None else None,
                qos_reason="" if qos is not None else str(KNOWN_WRAPPERS[wrapper]["reason"]),
                via_known_wrapper=wrapper,
                condition=self.guarded(member),
                guard=self.guard_of(member),
            )
        )

    def _accessor_read(self, name: str, receiver: Any, node: Any) -> None:
        inner, _receiver, _template = _call_parts(receiver, self.src)
        if inner not in READ_CALLS:
            return
        self.consumed.add(receiver.id)
        arguments = [item for item in _args(receiver) if not _is_node_argument(item, self.src)]
        if not arguments:
            return
        ref = self.resolve(arguments[0])
        api = "get_parameter_accessor" if inner == "get_parameter" else "get_parameter_or_accessor"
        if inner == "get_parameter" and ref.resolved and _checked(self.condition, str(ref.value)):
            api = "get_parameter_checked"
        self.unit.reads.append(
            ParamRead(
                name=ref,
                sym=self.sym,
                line=_line(node),
                api=api,
                accessor=name,
                spelling=_flat(arguments[0], self.src),
            )
        )

    def _callback(self, name: str, arguments: Sequence[Any], node: Any) -> None:
        form = "expression"
        handler = ""
        if arguments:
            argument = arguments[0]
            text = _flat(argument, self.src)
            if argument.type == "lambda_expression":
                form = "lambda"
                handler = self._lambda_handler(argument)
            elif text.startswith("&"):
                form = "bound_method"
                handler = text.lstrip("&")
            else:
                form = "variable"
                handler = text
        self.unit.callbacks.append(
            ParamCallback(api=name, form=form, handler=handler, sym=self.sym, line=_line(node))
        )

    def _lambda_handler(self, node: Any) -> str:
        stack = [node]
        while stack:
            current = stack.pop()
            for child in current.named_children:
                stack.append(child)
            if current.type != "call_expression":
                continue
            name, receiver, _template = _call_parts(current, self.src)
            if not name or receiver is not None:
                continue
            if self.klass and name not in ("std", "move"):
                return f"{self.klass}::{name}"
        return ""

    def _parameter(self, name: str, arguments: Sequence[Any], template: Any, node: Any) -> None:
        if (
            node.id in self.consumed
            or self._inside_wrapper()
            or _in_wrapper_lambda(node, self.src, self.unit.wrappers)
        ):
            # The body of a declare wrapper states its own signature, not a parameter name.
            return
        if name in DECLARE_MAP_CALLS:
            self._declare_map(arguments, template, node)
            return
        declare = self._declare_shape(name, arguments, template)
        if declare is not None:
            self._declare(name, declare, node)
            return
        if name not in READ_CALLS or not arguments:
            return
        values = [item for item in arguments if not _is_node_argument(item, self.src)]
        if not values:
            return
        ref = self.resolve(values[0])
        target = ""
        if len(values) > 1 and values[1].type in ("identifier", "field_expression"):
            target = _flat(values[1], self.src)
            # The out parameter form writes the value into a variable, and that variable is
            # the topic or the frame a few lines later.
            known = self.defaults.get(ref.value) if ref.resolved else None
            if known is not None and known.status == LITERAL:
                self.bind(target.strip("&* "), known)
            number = self.numbers.get(ref.value) if ref.resolved else None
            if number is not None:
                self.numbers[target.strip("&* ")] = number
                self.direct_numbers.add(target.strip("&* ").removeprefix("this->"))
        api = "get_parameter_out" if target else name
        if api == "get_parameter" and ref.resolved and _checked(self.condition, str(ref.value)):
            # a read under `if (has_parameter(name))` never meets an undeclared name
            api = "get_parameter_checked"
        self.unit.reads.append(
            ParamRead(
                name=ref,
                sym=self.sym,
                line=_line(node),
                api=api,
                target=target,
                spelling=_flat(values[0], self.src),
            )
        )

    def _declare(self, name: str, declare: tuple[Any, Any, list[str], str], node: Any) -> None:
        name_node, default_node, types, wrapper = declare
        if default_node is not None:
            type_name = _flat(default_node, self.src)
            if re.fullmatch(
                r"rclcpp::(?:ParameterType::)?PARAMETER_(?:BOOL|INTEGER|DOUBLE|STRING|BYTE_ARRAY|BOOL_ARRAY|INTEGER_ARRAY|DOUBLE_ARRAY|STRING_ARRAY)",
                type_name,
            ):
                parameter_type = type_name.rpartition("PARAMETER_")[2].lower()
                types = [
                    {"integer": "int", "integer_array": "int_array"}.get(
                        parameter_type, parameter_type
                    )
                ]
                default_node = None
        ref = self.resolve(name_node)
        scope = "plugin" if name in PLUGIN_DECLARE_CALLS or ref.status == SUFFIX else "node"
        default = self.resolve(default_node) if default_node is not None else None
        literal = _default_value(default_node, self.src) if default_node is not None else None
        blocked = literal is _DEFAULT_BLOCKED
        if blocked:
            literal = None
        refused = model.unrepresentable(literal) if literal is not None else ""
        if default_node is None:
            state = "absent"
        elif refused:
            state = REFUSED
        elif literal is not None:
            state = "known"
        else:
            state = "unknown"
        self.unit.params.append(
            ParamSite(
                name=ref,
                sym=self.sym,
                owner_class=self.klass,
                line=_line(node),
                scope=scope,
                api=name if name in PLUGIN_DECLARE_CALLS else "declare_parameter",
                value_type=_scalar_type(types) or _type_of(literal),
                default=None if refused else literal,
                default_ref=default
                if not blocked
                and default is not None
                and (default.member or default.origin == "macro")
                else None,
                default_expr=default.expr if default is not None else "",
                default_state=state,
                default_reason=refused or "",
                via_wrapper=wrapper,
                spelling=_flat(name_node, self.src),
            )
        )
        if ref.resolved and not refused:
            origin = "plugin_parameter" if scope == "plugin" else "parameter"
            if literal is not None and isinstance(literal, str):
                self.defaults[ref.value] = Ref(
                    LITERAL, literal, "", origin=origin, source_name=ref.value
                )
            elif default is not None and default.status == LITERAL:
                self.defaults[ref.value] = replace(
                    default, origin=origin, source_name=ref.value, expr=""
                )
            number = _as_number(literal)
            if number is not None:
                self.defaults.setdefault(
                    ref.value, Ref(UNRESOLVED, "", "", origin=origin, source_name=ref.value)
                )
                self.numbers.setdefault(ref.value, (number, ref.value))

    def _declare_map(self, arguments: Sequence[Any], template: Any, node: Any) -> None:
        """`declare_parameters<T>(namespace, {{"key", value}, ...})` declares one parameter
        per map entry, and the dotted name exists nowhere as a literal."""
        values = [item for item in arguments if not _is_node_argument(item, self.src)]
        if len(values) < 2:
            return
        prefix = self.resolve(values[0])
        types = _template_types(template, self.src)
        for key, value in _pairs(values[1], self.src):
            dotted = f"{prefix.value}.{key}" if prefix.value else key
            refused = model.unrepresentable(value) if value is not None else ""
            self.unit.params.append(
                ParamSite(
                    name=Ref(LITERAL, dotted, _flat(values[0], self.src), origin="map_key"),
                    sym=self.sym,
                    owner_class=self.klass,
                    line=_line(node),
                    api="declare_parameters",
                    value_type=_scalar_type(types),
                    default=None if refused else value,
                    default_state=_map_state(value, refused),
                    default_reason=refused or "",
                    map_namespace=prefix.value,
                )
            )

    def _timer_callback(self, node: Any) -> str:
        """The callback a timer argument names: a lambda, or `std::bind(&Class::method, this)`."""
        if node.type == "lambda_expression":
            return self._lambda_symbol(node)
        if node.type != "call_expression":
            return ""
        function, _, _ = _call_parts(node, self.src)
        bound = _args(node)
        if function != "bind" or len(bound) != 2 or _flat(bound[1], self.src) != "this":
            return ""
        method = _flat(bound[0], self.src)
        if not re.fullmatch(r"&[\w:]+", method):
            return ""
        short = self.klass.rpartition("::")[2]
        if self.klass and method[1:].startswith(short + "::"):
            return self.klass + method[1 + len(short) :]
        return method[1:]

    def _period_form(self, node: Any, source: str) -> tuple[str, float | None]:
        """How a period read from a parameter is built: `direct` for `duration<double>(rate)`,
        `inverse` with its numerator for `duration<double>(1.0 / rate)`."""
        if node.type != "call_expression":
            return "", None
        function, _, _ = _call_parts(node, self.src)
        values = _args(node)
        if function != "duration" or not values:
            return "", None
        value = values[0]
        while value.type == "parenthesized_expression" and value.named_child_count == 1:
            value = value.named_children[0]
        if value.type != "binary_expression":
            return ("direct", None) if self._number_direct(value) else ("", None)
        operator = _child(value, "operator")
        right = _child(value, "right")
        first, first_source = self._number(_child(value, "left"))
        _, second_source = self._number(right)
        if (
            operator is not None
            and _text(operator, self.src) == "/"
            and first is not None
            and not first_source
            and second_source == source
            and right is not None
            and right.type in ("identifier", "field_identifier", "field_expression")
            and self._number_direct(right)
        ):
            return "inverse", first
        return "", None

    def _timer(self, name: str, arguments: Sequence[Any], node: Any) -> None:
        period: float | None = None
        expr = ""
        source = ""
        reason = ""
        period_form = ""
        numerator = None
        callback = next(filter(None, (self._timer_callback(item) for item in arguments)), "")
        for item in arguments:
            seconds, text, from_parameter, refused = self._duration(item)
            if text:
                period, expr, source, reason = seconds, text, from_parameter, refused
                if source:
                    period_form, numerator = self._period_form(item, source)
                break
        if reason:
            state = REFUSED
        elif period is None:
            state = UNRESOLVED
        elif source:
            state = "resolved_from_parameter"
        else:
            state = LITERAL
        self.unit.endpoints.append(
            Endpoint(
                kind="timer",
                name=Ref(UNRESOLVED, "", expr),
                sym=self.sym,
                owner_class=self.klass,
                line=_line(node),
                api=name,
                period_s=period,
                period_state=state,
                period_from=source,
                period_expr=expr,
                period_reason=reason,
                callback=callback,
                period_form=period_form,
                period_numerator=numerator,
                condition=self.condition,
                guard=self.guard,
            )
        )

    def _duration(self, node: Any) -> tuple[float | None, str, str, str]:
        """Seconds, the text as written, the parameter it came from, and the refusal when
        the number folded and json cannot hold it."""
        text = _flat(node, self.src)
        if node.type == "user_defined_literal":
            found = _DURATION_SUFFIX.match(text)
            if found:
                return float(found.group(1)) * _SUFFIX_SCALE[found.group(2)], text, "", ""
            return None, text, "", ""
        if node.type == "call_expression":
            name, _receiver, template = _call_parts(node, self.src)
            arguments = _args(node)
            if name == "duration" and len(_template_types(template, self.src)) > 1:
                return None, text, "", ""
            if name in _CHRONO:
                value, source = self._number(arguments[0]) if arguments else (None, "")
                if value is not None:
                    return value * _CHRONO[name], text, source, ""
                return None, text, "", self._refused_number(arguments[0] if arguments else None)
            if "chrono" in text or name == "duration":
                value, source = self._number(arguments[0]) if arguments else (None, "")
                refused = (
                    ""
                    if value is not None
                    else self._refused_number(arguments[0] if arguments else None)
                )
                return value, text, source, refused
            return None, "", "", ""
        if node.type == "number_literal":
            value, source = self._number(node)
            return value, text, source, "" if value is not None else self._refused_number(node)
        return None, "", "", ""

    def _refused_number(self, node: Any) -> str:
        """Why a number that folded cannot be written, empty when nothing folded."""
        if node is None:
            return ""
        value, _source = self._number_value(node)
        return model.unrepresentable(value) if value is not None else ""

    def _broadcast(self, receiver: Any, arguments: Sequence[Any], node: Any) -> None:
        if not arguments:
            return
        holder = _text(arguments[0], self.src)
        pair = self.frames.get(holder, {})
        unknown = Ref(UNRESOLVED, "", holder)
        member = _flat(receiver, self.src).lstrip("*&") if receiver is not None else ""
        wrapper = self.member_types.get(member, "")
        self.unit.broadcasts.append(
            TfBroadcast(
                parent=pair.get("parent", unknown),
                child=pair.get("child", unknown),
                sym=self.sym,
                line=_line(node),
                kind=str(KNOWN_WRAPPERS.get(wrapper, {}).get("broadcaster", "dynamic")),
                broadcaster=wrapper,
                condition=self.guarded(member),
                guard=self.guard_of(member),
            )
        )

    def _lookup(self, name: str, arguments: Sequence[Any], node: Any) -> None:
        if len(arguments) < 2:
            return
        # The time travelling overloads take (target, target_time, source, source_time,
        # fixed_frame, ...), so the second argument is a time and the source is the third.
        travels = len(arguments) >= 5 and (
            name == "lookupTransform" or _TIME_TEXT.search(_flat(arguments[1], self.src))
        )
        self.unit.lookups.append(
            TfLookup(
                api=name,
                target=self.resolve(arguments[0]),
                source=self.resolve(arguments[2] if travels else arguments[1]),
                sym=self.sym,
                line=_line(node),
            )
        )

    def _macro(self, name: str, arguments: Sequence[Any], node: Any) -> None:
        texts = [_flat(item, self.src) for item in arguments]
        wrapped = "\n" in _text(node, self.src)
        if name == "PLUGINLIB_EXPORT_CLASS" and len(texts) >= 2:
            self.unit.registrations.append(
                Registration(
                    macro=name,
                    type=texts[0],
                    base_class=texts[1],
                    line=_line(node),
                    wrapped=wrapped,
                )
            )
            return
        if name == "BT_REGISTER_NODES":
            return
        if texts:
            self.unit.registrations.append(
                Registration(macro=name, type=texts[0], line=_line(node), wrapped=wrapped)
            )


_TIME_TEXT = re.compile(r"(?i)time|stamp|now\(\)|zero")


def _construction_args(node: Any) -> Any:
    """The argument list of a construction, through make_unique and make_shared."""
    if node is None or node.type != "call_expression":
        return node
    arguments = _child(node, "arguments")
    return arguments if arguments is not None else node


def _template_head(text: str, symbol: str) -> str:
    """The first template argument after `symbol`, so `Subscriber<LaserScan, LifecycleNode>`
    names the message and not the node type beside it."""
    at = text.find(symbol + "<")
    if at < 0:
        return ""
    depth = 0
    start = at + len(symbol) + 1
    for index in range(start, len(text)):
        char = text[index]
        if char == "<":
            depth += 1
        elif char == ">" and depth:
            depth -= 1
        elif char in ",>" and not depth:
            return " ".join(text[start:index].split())
    return ""


_OWN_READERS = frozenset({"message_filters::Subscriber", "image_transport::SubscriberFilter"})


WRAPPER_FACTORIES = {
    "create_transform_broadcaster": "nav2::TransformBroadcaster",
    "create_static_transform_broadcaster": "nav2::StaticTransformBroadcaster",
    "create_transform_listener": "nav2::TransformListener",
}
"""Free functions that return a known wrapper (`nav2_ros_common/tf2_factories.hpp`), so
`tf_broadcaster_ = nav2::create_transform_broadcaster(this)` builds one."""


def _wrapper_of(text: str) -> str:
    for name in KNOWN_WRAPPERS:
        if name in text:
            return name
    for name, wrapper in WRAPPER_FACTORIES.items():
        if re.search(rf"\b{name}\s*\(", text):
            return wrapper
    return ""


def _type_of(value: Any) -> str:
    """The parameter type a literal default states, for an API with no template argument."""
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int):
        return "int"
    if isinstance(value, float):
        return "double"
    if isinstance(value, str):
        return "string"
    return ""


def _scalar_type(types: Sequence[str]) -> str:
    if not types:
        return ""
    return SCALAR_TYPES.get(types[0], types[0])


def _as_number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


def _pairs(node: Any, src: bytes) -> list[tuple[str, Any]]:
    """`{{"max_speed", 3.0}, {"max_accel", 1.0}}` as ordered key value pairs."""
    out: list[tuple[str, Any]] = []
    stack = [node]
    while stack:
        current = stack.pop()
        for child in current.named_children:
            if child.type != "initializer_list":
                continue
            items = [item for item in child.named_children if item.type != "comment"]
            if len(items) == 2:
                key = _string_of(items[0], src)
                if key is not None:
                    out.append((key, _literal_value(items[1], src)))
                    continue
            stack.append(child)
    return sorted(out)


_SELF_HANDLES = frozenset({"this", "*this", "shared_from_this()", "this->shared_from_this()"})
_SHARED_THIS = re.compile(r"(?:::)?[\w:]*(?:shared_ptr\s*<[^;]*>|SharedPtr)\s*\(\s*this\s*[,)]")


def _self_handle(node: Any, src: bytes) -> bool:
    """`this`, `shared_from_this()`, or a shared pointer built over `this` with a no op
    deleter, which is how a node class hands itself to a helper as a `Node::SharedPtr`."""
    text = _flat(node, src)
    return text in _SELF_HANDLES or bool(_SHARED_THIS.match(text))


def _is_node_argument(node: Any, src: bytes) -> bool:
    """A leading node handle in a nav2 style free function, not the parameter name."""
    if node.type == "this":
        return True
    text = _text(node, src).strip().lstrip("*&")
    return text in (
        "this",
        "node",
        "node_",
        "parent",
        "parent_node",
        "shared_from_this()",
        "this->shared_from_this()",
    )


def _is_node_handle(node: Any, src: bytes) -> bool:
    """A node handed to a call: the node itself, or one of its interfaces."""
    text = _text(node, src).strip().lstrip("*&")
    return (
        _is_node_argument(node, src)
        or text in ("weak_from_this()", "this->weak_from_this()")
        or NODE_INTERFACE.fullmatch(text) is not None
    )


def name_literals(source: bytes) -> list[str]:
    """Every string literal of a source shaped like a ROS name, a remap could name it. A
    comment and a character literal are no string literal."""
    text = source.decode("utf-8", "replace")
    return sorted(
        {value for value in _STRING_LITERAL.findall(text) if py_code.is_name_literal(value)}
    )


def _checked(condition: str, name: str) -> bool:
    """Whether a guard asks `has_parameter` for this name first."""
    return "has_parameter(" in condition and f'"{name}"' in condition


def stamped_frames(source: bytes) -> list[dict[str, Any]]:
    """The literal frames a source writes into a message header, `msg.header.frame_id =
    "camera_link"`, with their lines. A comment is no assignment."""
    text = _COMMENT.sub(
        lambda match: re.sub(r"[^\n]", " ", match.group()), source.decode("utf-8", "replace")
    )
    return sorted(
        (
            {"frame": match.group(1), "line_hint": text.count("\n", 0, match.start()) + 1}
            for match in _HEADER_FRAME.finditer(text)
        ),
        key=lambda row: (row["frame"], row["line_hint"]),
    )


_COMMENT = re.compile(r"//[^\n]*|/\*.*?\*/", re.S)
_HEADER_FRAME = re.compile(r'(?:\.|->)header\.frame_id\s*=\s*"([^"\\\n]*)"')


_STRING_LITERAL = re.compile(
    r'"((?:[^"\\\n]|\\.)*)"|\'(?:[^\'\\\n]|\\.)*\'|//[^\n]*|/\*.*?\*/', re.S
)


_DEFAULT_BLOCKED = object()


def _default_value(node: Any, src: bytes, depth: int = 0) -> Any:
    if node is None or depth > 32:
        return None
    if node.type == "declaration":
        type_node = _child(node, "type")
        spelling = _flat(type_node, src) if type_node is not None else ""
        if spelling == "std::string":
            return ""
        if spelling.startswith("std::vector<"):
            return []
        return None
    literal = _literal_value(node, src)
    if literal is not None:
        if model.unrepresentable(literal):
            return literal
        if node.parent is not None and node.parent.type == "init_declarator":
            declaration = node.parent.parent
            type_node = _child(declaration, "type") if declaration is not None else None
            scalar = SCALAR_TYPES.get(_flat(type_node, src), "") if type_node is not None else ""
            if isinstance(literal, (int, float, bool)):
                if scalar == "double":
                    return float(literal)
                if scalar == "int":
                    return int(literal)
                if scalar == "bool":
                    return bool(literal)
        return literal
    if node.type == "call_expression":
        name, _receiver, _template = _call_parts(node, src)
        arguments = _args(node)
        if name in ("ParameterValue", "string", "basic_string") and arguments:
            return _default_value(arguments[0], src, depth + 1)
        return None
    name = _flat(node, src).removeprefix("this->")
    if node.type not in ("identifier", "field_identifier", "field_expression"):
        return None
    if node.type == "field_expression" and not _flat(node, src).startswith("this->"):
        return None
    current = node
    while current.parent is not None:
        parent = current.parent
        if parent.type == "compound_statement":
            for previous in reversed(parent.named_children):
                if previous.end_byte > current.start_byte:
                    continue
                found, value = _default_write(previous, name, src)
                if found:
                    result = _default_value(value, src, depth + 1)
                    if result is None:
                        return _DEFAULT_BLOCKED
                    scalar = _default_local_type(node, name, src)
                    if isinstance(result, (int, float, bool)) and not model.unrepresentable(result):
                        if scalar == "double":
                            return float(result)
                        if scalar == "int":
                            return int(result)
                        if scalar == "bool":
                            return bool(result)
                    return result
        if parent.type == "function_definition":
            for initializers in parent.named_children:
                if initializers.type != "field_initializer_list":
                    continue
                for initializer in initializers.named_children:
                    children = initializer.named_children
                    if len(children) == 2 and _flat(children[0], src) == name:
                        values = children[1].named_children
                        value = (
                            _default_value(values[0], src, depth + 1) if len(values) == 1 else None
                        )
                        return value if value is not None else _DEFAULT_BLOCKED
            return None
        current = parent
    return None


def _default_local_type(node: Any, name: str, src: bytes) -> str:
    current = node
    while current.parent is not None:
        parent = current.parent
        if parent.type == "compound_statement":
            for previous in reversed(parent.named_children):
                if previous.end_byte > current.start_byte or previous.type != "declaration":
                    continue
                for child in previous.named_children:
                    target = (
                        _child(child, "declarator") if child.type == "init_declarator" else child
                    )
                    if _flat(target, src).strip("&* ") == name:
                        type_node = _child(previous, "type")
                        return (
                            SCALAR_TYPES.get(_flat(type_node, src), "")
                            if type_node is not None
                            else ""
                        )
        if parent.type == "function_definition":
            return ""
        current = parent
    return ""


def _default_write(node: Any, name: str, src: bytes) -> tuple[bool, Any]:
    if node.type in ("function_definition", "lambda_expression"):
        return False, None
    if node.type == "declaration":
        for child in node.named_children:
            target = _child(child, "declarator") if child.type == "init_declarator" else child
            if _flat(target, src).strip("&* ") == name:
                return True, _child(child, "value") if child.type == "init_declarator" else (
                    node if target.type == "identifier" else None
                )
            value = _child(child, "value")
            if (
                target.type in ("reference_declarator", "pointer_declarator")
                and value is not None
                and _flat(value, src).lstrip("&").removeprefix("this->") == name
            ):
                return True, None
    if node.type == "assignment_expression":
        left = _child(node, "left")
        if left is not None and _flat(left, src).removeprefix("this->") == name:
            operator = _child(node, "operator")
            value = _child(node, "right")
            return True, value if operator is not None and _text(operator, src) == "=" else None
    if node.type == "update_expression" and any(
        _flat(child, src).removeprefix("this->") == name for child in node.named_children
    ):
        return True, None
    if node.type == "call_expression":
        api, _receiver, _template = _call_parts(node, src)
        arguments = _args(node)
        if api in READ_CALLS:
            arguments = [item for item in arguments if not _is_node_argument(item, src)][1:]
        elif api in DECLARE_CALLS | PLUGIN_DECLARE_CALLS | {"has_parameter"}:
            arguments = []
        for argument in arguments:
            if _flat(argument, src).lstrip("&").removeprefix("this->") == name:
                return True, None
    children = list(node.named_children)
    if node.type == "compound_statement":
        for position, child in enumerate(children):
            if child.type == "declaration" and _default_write(child, name, src)[0]:
                children = children[:position]
                break
    for child in reversed(children):
        found, value = _default_write(child, name, src)
        if found:
            if node.type in (
                "if_statement",
                "switch_statement",
                "for_statement",
                "while_statement",
                "do_statement",
                "try_statement",
            ):
                value = None
            return True, value
    return False, None


CAST_INTEGERS = frozenset(
    {"int", "long", "int64_t", "std::int64_t", "int32_t", "std::int32_t", "size_t", "std::size_t"}
)


def _literal_value(node: Any, src: bytes) -> Any:
    """A C++ literal as a python value, or None when the expression is not a literal."""
    if node is None:
        return None
    if node.type == "call_expression":
        arguments = _args(node)
        name, _receiver, template = _call_parts(node, src)
        if name in ("ParameterValue", "string", "basic_string") and arguments:
            return _literal_value(arguments[0], src)
        if name in ("string", "basic_string") and not arguments:
            return ""
        if name == "static_cast" and len(arguments) == 1 and template is not None:
            inner = _literal_value(arguments[0], src)
            target = _flat(template, src).strip("<> ")
            if isinstance(inner, (int, float)) and not isinstance(inner, bool):
                if target in ("double", "float"):
                    return float(inner)
                if target in CAST_INTEGERS:
                    return int(inner)
            return None
        if name == "vector" and not arguments and _flat(node, src).startswith("std::vector<"):
            return []
        return None
    if node.type == "compound_literal_expression":
        type_node = _child(node, "type")
        value = _child(node, "value")
        if type_node is not None and _flat(type_node, src).startswith("std::vector<"):
            return _literal_value(value, src)
    if node.type in ("string_literal", "concatenated_string"):
        return _string_of(node, src)
    if node.type in ("true", "false"):
        return node.type == "true"
    if node.type == "number_literal":
        text = _text(node, src).replace("'", "").rstrip("fFuUlL")
        try:
            return int(text, 0)
        except ValueError:
            pass
        try:
            return float(text)
        except ValueError:
            return None
    if node.type == "unary_expression" and node.named_child_count == 1:
        inner = _literal_value(node.named_children[0], src)
        if isinstance(inner, (int, float)) and _text(node, src).startswith("-"):
            return -inner
        return inner
    if node.type == "initializer_list":
        out = []
        for child in node.named_children:
            value = _literal_value(child, src)
            if value is None:
                return None
            out.append(value)
        return out
    return None


def _wrappers(root: Any, src: bytes, *, lambdas: bool = True) -> dict[str, tuple[int, int]]:
    """Project local declare wrappers: a function, or a lambda a variable holds, that
    declares a parameter and takes the name as a string. Two thirds of the parameter named
    topics on the reference stack go through one of these rather than through the rclcpp API."""
    out: dict[str, tuple[int, int]] = {}
    if not any(call.encode() in src for call in DECLARE_CALLS):
        return out
    stack = [root]
    while stack:
        node = stack.pop()
        for child in node.named_children:
            stack.append(child)
        lambda_name = _lambda_name(node, src) if lambdas else ""
        if lambda_name:
            # `auto dp = [&](const std::string & s) { return node.declare_parameter<double>(s); };`
            name_types: tuple[str, ...] = ("std::string", "string", "auto")
            declarator = _child(node, "declarator")
            inner = None
        elif node.type == "function_definition" and _child(node, "type") is not None:
            name_types = ("std::string", "string")
            declarator = _child(node, "declarator")
            while declarator is not None and declarator.type != "function_declarator":
                declarator = _child(declarator, "declarator")
            inner = _child(declarator, "declarator") if declarator is not None else None
            if inner is None:
                continue
        else:
            continue
        body = _child(node, "body")
        if body is None or not any(call in _text(body, src) for call in DECLARE_CALLS):
            continue
        parameters = _child(declarator, "parameters") if declarator is not None else None
        if parameters is None:
            continue
        name = lambda_name or _text(inner, src)
        if not name.isidentifier():
            continue
        if not lambda_name:
            name = "::".join([*_namespaces_of(node, src), name])
        index = None
        for position, child in enumerate(
            [item for item in parameters.named_children if item.type == "parameter_declaration"]
        ):
            type_node = _child(child, "type")
            if type_node is None or _flat(type_node, src) not in name_types:
                continue
            inner_name = _child(child, "declarator")
            if inner_name is None:
                continue
            # The declare call must be handed this very parameter as the name. Without that
            # test a constructor that takes a node name and declares parameters of its own
            # reads as a wrapper, and every one of its declarations disappears.
            if not _declares_with(body, src, _text(inner_name, src).strip("&* ")):
                continue
            index = position
            break
        if index is None:
            continue
        default = index + 1
        if lambda_name:
            # `load(name, covariance)` hands the lambda an out variable, not a default
            declared = [
                _text(_child(item, "declarator"), src).strip("&* ")
                for item in parameters.named_children
                if item.type == "parameter_declaration" and _child(item, "declarator") is not None
            ]
            given = _declared_default(body, src, declared[index])
            default = declared.index(given) if given in declared else len(declared)
        out[name] = (index, default)
    return out


def _declared_default(body: Any, src: bytes, parameter: str) -> str:
    """The text a declare call taking `parameter` as its name passes as the default."""
    stack = [body]
    while stack:
        node = stack.pop()
        stack.extend(node.named_children)
        if node.type != "call_expression" or _call_parts(node, src)[0] not in DECLARE_CALLS:
            continue
        values = [item for item in _args(node) if not _is_node_argument(item, src)]
        if values and _flat(values[0], src).strip("&* ") == parameter:
            return _flat(values[1], src).strip("&* ") if len(values) > 1 else ""
    return ""


def _lambda_name(node: Any, src: bytes) -> str:
    """The variable a lambda is assigned to where it is declared, or nothing."""
    if node.type != "lambda_expression" or node.parent is None:
        return ""
    if node.parent.type != "init_declarator":
        return ""
    target = _child(node.parent, "declarator")
    return _text(target, src) if target is not None and target.type == "identifier" else ""


def _in_wrapper_lambda(node: Any, src: bytes, wrappers: dict[str, tuple[int, int]]) -> bool:
    parent = node.parent
    while parent is not None:
        if parent.type == "lambda_expression" and _lambda_name(parent, src) in wrappers:
            return True
        parent = parent.parent
    return False


def _namespaces_of(node: Any, src: bytes) -> list[str]:
    out: list[str] = []
    parent = node.parent
    while parent is not None:
        if parent.type == "namespace_definition":
            name = _child(parent, "name")
            if name is not None:
                out.append(_text(name, src))
        parent = parent.parent
    return list(reversed(out))


AUTO_DECLARE_OPTIONS = frozenset(
    {"automatically_declare_parameters_from_overrides", "allow_undeclared_parameters"}
)


def _stronger(left: bool | str, right: bool | str) -> bool | str:
    return True if True in (left, right) else ("unknown" if "unknown" in (left, right) else False)


def _auto_declare(
    root: Any, src: bytes, options: frozenset[str] = AUTO_DECLARE_OPTIONS, literal: str = "true"
) -> tuple[bool | str, dict[int, bool | str], dict[str, bool | str]]:
    """Which node a `NodeOptions` chain lets take parameters it never declares: True for a
    literal `true`, `unknown` when the value is not a literal. With `use_global_arguments`
    and the literal `false` it is the node that reads no launch argument instead.

    A chain is decided per node construction where the code allows: the construction in
    `main` whose arguments hold it, directly or through the options variable it sets, or the
    class whose base initializer takes it. A chain this cannot trace to one construction (a
    helper function building the options, say) falls back to the whole file, which is the
    only case where one helper node built to take any parameter still speaks for every node
    of the file. Returns the file fallback, the value per construction site by its first
    byte, and the value per class by its unqualified name.
    """
    fallback: bool | str = False
    sites: dict[int, bool | str] = {}
    classes: dict[str, bool | str] = {}
    variables: dict[str, bool | str] = {}
    constructions = _main_constructions(root, src)
    stack = [root]
    while stack:
        node = stack.pop()
        stack.extend(node.named_children)
        if node.type != "call_expression":
            continue
        name, receiver, _template = _call_parts(node, src)
        if name not in options or receiver is None:
            continue
        arguments = _args(node)
        flat = _flat(arguments[0], src) if len(arguments) == 1 else ""
        if flat in ("true", "false") and flat != literal:
            continue
        value: bool | str = True if flat == literal else "unknown"
        holders = [
            site
            for _name, _arguments, site in constructions
            if site.start_byte <= node.start_byte and node.end_byte <= site.end_byte
        ]
        if holders:
            site = min(holders, key=lambda item: item.end_byte - item.start_byte)
            sites[site.start_byte] = _stronger(sites.get(site.start_byte, False), value)
            continue
        owner = _initialized_class(node, src)
        variable = "" if owner else _options_variable(node, src)
        if owner:
            classes[owner] = _stronger(classes.get(owner, False), value)
        elif variable:
            variables[variable] = _stronger(variables.get(variable, False), value)
        else:
            fallback = _stronger(fallback, value)
    used: set[str] = set()
    for _name, arguments, site in constructions:
        for argument in arguments:
            variable = _flat(argument, src).strip("&* ")
            if variable in variables:
                used.add(variable)
                sites[site.start_byte] = _stronger(
                    sites.get(site.start_byte, False), variables[variable]
                )
    for variable, value in variables.items():
        if variable not in used:
            fallback = _stronger(fallback, value)
    return fallback, sites, classes


def _chain_top(node: Any) -> Any:
    while node.parent is not None and node.parent.type in ("call_expression", "field_expression"):
        node = node.parent
    return node


def _initialized_class(node: Any, src: bytes) -> str:
    """The class whose base initializer takes this options chain as an argument."""
    holder = _chain_top(node).parent
    if holder is None or holder.type != "argument_list":
        return ""
    if holder.parent is None or holder.parent.type != "field_initializer":
        return ""
    parent = holder.parent
    while parent is not None:
        if parent.type in ("class_specifier", "struct_specifier"):
            name = _child(parent, "name")
            return _text(name, src).rpartition("::")[2] if name is not None else ""
        if parent.type == "function_definition":
            declarator = _child(parent, "declarator")
            while declarator is not None and declarator.type != "function_declarator":
                declarator = _child(declarator, "declarator")
            name = _child(declarator, "declarator") if declarator is not None else None
            qualified = _text(name, src) if name is not None else ""
            if "::" in qualified:
                return qualified.rpartition("::")[0].rpartition("::")[2]
        parent = parent.parent
    return ""


def _options_variable(node: Any, src: bytes) -> str:
    """The variable an options chain initialises or sets, `options` in both
    `auto options = rclcpp::NodeOptions().allow_undeclared_parameters(true);` and
    `options.allow_undeclared_parameters(true);`."""
    holder = _chain_top(node).parent
    if holder is not None and holder.type == "init_declarator":
        declarator = _child(holder, "declarator")
        return _text(declarator, src).strip("&* ") if declarator is not None else ""
    base = node
    while base is not None and base.type in ("call_expression", "field_expression"):
        base = _child(base, "function" if base.type == "call_expression" else "argument")
    return _text(base, src) if base is not None and base.type == "identifier" else ""


GLOBAL_ARGUMENTS_OPTIONS = frozenset({"use_global_arguments"})


def _global_arguments(local: bool | str) -> dict[str, Any]:
    """`use_global_arguments` as a record states it: False for a literal false only."""
    return {"use_global_arguments": False if local is True else local} if local else {}


def _constructions(unit: Unit, rows: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    out = []
    for row in rows:
        if unit.auto_declare:
            row = {
                **row,
                "auto_declare": _stronger(row.get("auto_declare", False), unit.auto_declare),
            }
        if unit.local_arguments:
            literal = row.get("use_global_arguments") is False or unit.local_arguments is True
            row = {**row, "use_global_arguments": False if literal else "unknown"}
        out.append(row)
    return out


def _declares_with(body: Any, src: bytes, parameter: str) -> bool:
    stack = [body]
    while stack:
        node = stack.pop()
        for child in node.named_children:
            stack.append(child)
        if node.type != "call_expression":
            continue
        name, _receiver, _template = _call_parts(node, src)
        if name not in DECLARE_CALLS:
            continue
        for argument in _args(node):
            if _flat(argument, src).strip("&* ") == parameter:
                return True
    return False


_EXPORT_MACRO = re.compile(
    rb"(\b(?:class|struct)[ \t]+)([A-Z][A-Z0-9_]*)"
    rb"(?=[ \t]+(?!final\b)[A-Za-z_]\w*\s*(?::(?!:)|\{[ \t]*(?:\r?\n|\Z)|final\b))"
)
"""`class FOO_PUBLIC Controller : public rclcpp::Node`: the grammar takes the export macro for
the class name, so it is blanked to spaces of its own length, which keeps every offset. A
brace that does not end its line opens an initializer, `struct POINT origin{0, 0};`."""


def parse_text(
    text: str,
    *,
    path: str = "",
    pkg: str = "",
    relpath: str = "",
    parser: Any = None,
    shared_wrappers: dict[str, tuple[int, int]] | None = None,
    shared_members: dict[tuple[str, str], str] | None = None,
) -> Unit:
    """One translation unit. A file the grammar refuses is marked, never dropped."""
    engine = parser if parser is not None else load_parser()
    unit = Unit(
        path=path,
        pkg=pkg,
        relpath=relpath,
        shared_wrappers=dict(shared_wrappers or {}),
        shared_members=dict(shared_members or {}),
    )
    src = _EXPORT_MACRO.sub(
        lambda match: match.group(1) + b" " * len(match.group(2)), text.encode("utf-8", "replace")
    )
    unit.tree = engine.parse(src)
    root = unit.tree.root_node
    unit.source = src
    errors: list[int] = []
    missing = 0
    # a subtree without an error holds neither an ERROR region nor a missing token
    stack = [root] if root.has_error else []
    while stack:
        node = stack.pop()
        if node.type == "ERROR":
            errors.append(_line(node))
            if _ROS_CALL.search(src[node.start_byte : node.end_byte].decode("utf-8", "replace")):
                unit.hidden_regions += 1
            continue
        if node.is_missing:
            missing += 1
            continue
        stack.extend(child for child in node.children if child.has_error)
    unit.error_lines = tuple(sorted(errors))
    unit.missing_tokens = missing
    unit.parsed = not errors
    if errors:
        # Almost always a preprocessor construct: a macro that expands to a type, or an
        # `#if __has_include` pair. What is outside the region is still read.
        unit.reason = f"the grammar refused {len(errors)} region(s)"
    unit.wrappers = _wrappers(root, src)
    if unit.wrappers:
        unit.lambda_wrappers = set(unit.wrappers) - set(_wrappers(root, src, lambdas=False))
    reader = _Reader(src, unit, dict(_DEFINE.findall(text)))
    unit.signatures = _signatures(root, src)
    sites = _construction_values(root, src, reader.defines, unit.signatures)
    for name, values, _site in sites:
        reader.constructor_arguments.setdefault(name, []).append(values)
    reader.walk(root)
    unit.endpoints = _aliased_interfaces(root, src, unit.endpoints)
    unit.type_aliases = _type_aliases(root, src)
    unit.plugin_loaders = sorted(set(_PLUGIN_LOADER.findall(text)))
    unit.auto_declare, auto_sites, unit.auto_declare_classes = _auto_declare(root, src)
    unit.local_arguments, local_sites, unit.local_arguments_classes = (
        _auto_declare(root, src, GLOBAL_ARGUMENTS_OPTIONS, "false")
        if b"use_global_arguments" in src
        else (False, {}, {})
    )
    # the mutations are a second, whole tree walk that only a map default ever asks for
    unit.string_maps = _string_map_records(root, src, mutations_wanted=False)[0]
    for name, values, site in sites:
        unit.construction_sites.append((name, values, _line(site)))
        row = {"class": name, "owner_symbol": "main", "line_hint": _line(site)}
        if auto_sites.get(site.start_byte):
            row["auto_declare"] = auto_sites[site.start_byte]
        row.update(_global_arguments(local_sites.get(site.start_byte, False)))
        if site.type == "call_expression" and _call_parts(site, src)[0] not in {
            "make_shared",
            "make_unique",
        }:
            unit.construction_calls.append(row)
        else:
            unit.constructions.append(row)
    _resolve_construction_calls([unit])
    _construction_names(unit)
    unit.methods = sorted(set(unit.methods))
    return unit


def _construction_values(
    root: Any,
    src: bytes,
    defines: dict[str, str],
    signatures: dict[str, list[tuple[int, list[str]]]],
    constants: dict[str, str] | None = None,
) -> list[tuple[str, tuple[str | None, ...], Any]]:
    return [
        (
            name,
            tuple(
                _construction_value(argument, site, src, defines, signatures, constants)
                for argument in arguments
            ),
            site,
        )
        for name, arguments, site in _main_constructions(root, src)
    ]


def _construction_value(
    argument: Any,
    site: Any,
    src: bytes,
    defines: dict[str, str],
    signatures: dict[str, list[tuple[int, list[str]]]],
    constants: dict[str, str] | None = None,
) -> str | None:
    value = _string_of(argument, src)
    if value is not None:
        return value
    name = _flat(argument, src)
    if name in defines:
        return defines[name]
    statement = site
    while statement.parent is not None and statement.parent.type != "compound_statement":
        statement = statement.parent
    if statement.type not in {"declaration", "expression_statement"}:
        return None
    # A construction inside an `if` branch sees every write before the branch, in each
    # enclosing block up to the body of main. Loops are refused: a later write in the
    # loop reaches the next pass.
    while True:
        body = statement.parent
        if body is None or body.type != "compound_statement" or body.parent is None:
            return None
        for previous in reversed(body.named_children):
            if previous.end_byte > statement.start_byte:
                continue
            if _readonly_construction(previous, name, src, signatures):
                continue
            found, written = _default_write(previous, name, src)
            if found:
                if (
                    written is not None
                    and written.type == "argument_list"
                    and written.named_child_count == 1
                    and previous.type == "declaration"
                    and _flat(_child(previous, "type"), src) in {"std::string", "string"}
                ):
                    written = written.named_children[0]
                value = _default_value(written, src)
                return value if isinstance(value, str) else None
        if body.parent.type == "function_definition":
            # Nothing in main writes the name, so it is a namespace scope constant or unknown.
            return (constants or {}).get(name.removeprefix("::"))
        statement = body.parent
        while statement is not None and statement.type in {"else_clause", "if_statement"}:
            condition = _child(statement, "condition")
            if condition is not None and _default_write(condition, name, src)[0]:
                return None
            if statement.parent is None or statement.parent.type not in {
                "else_clause",
                "if_statement",
            }:
                break
            statement = statement.parent
        if statement is None or statement.type != "if_statement":
            return None


def _signatures(root: Any, src: bytes) -> dict[str, list[tuple[int, list[str]]]]:
    """Every function a tree defines or declares, by its last name segment.

    A row is the number of required parameters and the text of every parameter, so a call
    that leaves the defaulted tail out still finds the declaration a header states.
    """
    out: dict[str, list[tuple[int, list[str]]]] = {}
    pending = [root]
    while pending:
        node = pending.pop()
        pending.extend(node.named_children)
        if node.type not in {"function_definition", "declaration", "field_declaration"}:
            continue
        declarator = _child(node, "declarator")
        while declarator is not None and declarator.type != "function_declarator":
            declarator = _child(declarator, "declarator")
        function = _child(declarator, "declarator") if declarator is not None else None
        parameters = _child(declarator, "parameters") if declarator is not None else None
        if function is None or parameters is None:
            continue
        rows = [
            item
            for item in parameters.named_children
            if item.type in {"parameter_declaration", "optional_parameter_declaration"}
        ]
        out.setdefault(_flat(function, src).rsplit("::", 1)[-1], []).append(
            (
                sum(item.type == "parameter_declaration" for item in rows),
                [_flat(item, src) for item in rows],
            )
        )
    return out


def _readonly_construction(
    statement: Any, name: str, src: bytes, signatures: dict[str, list[tuple[int, list[str]]]]
) -> bool:
    if statement.type != "declaration":
        return False
    calls = [child for child in statement.named_children if child.type == "init_declarator"]
    if len(calls) != 1:
        return False
    call = _child(calls[0], "value")
    target = _child(calls[0], "declarator")
    if call is None or call.type != "call_expression" or _flat(target, src) == name:
        return False
    api, _, template = _call_parts(call, src)
    types = _template_types(template, src)
    if api not in {"make_shared", "make_unique"} or not types:
        return False
    arguments = _args(call)
    positions = [position for position, arg in enumerate(arguments) if _flat(arg, src) == name]
    if not positions:
        return False
    matching = [
        texts
        for required, texts in signatures.get(types[0].rsplit("::", 1)[-1], [])
        if required <= len(arguments) <= len(texts)
    ]
    return bool(matching) and all(
        "const" in texts[position].split()
        or ("&" not in texts[position] and "*" not in texts[position])
        for texts in matching
        for position in positions
    )


def _bound_constructor(
    item: NodeClass, sites: list[tuple[tuple[str | None, ...], int]]
) -> NodeClass:
    if len(sites) > 1 and item.name_position is not None:
        return replace(
            item,
            stated_name="",
            name_reason="ambiguous_construction",
        )
    if len(sites) != 1:
        return item
    arguments, line = sites[0]
    value = item.stated_name or None
    if item.name_position is not None:
        value = (
            arguments[item.name_position]
            if item.name_position < len(arguments)
            else item.name_default
        )
    if value is None:
        return item
    return replace(
        item,
        stated_name=value or "",
        name_source="literal",
        symbol="main",
        name_line=line,
        name_expr="",
    )


def _construction_names(unit: Unit) -> None:
    """A single caller in main can state the name a constructor forwards unchanged."""
    classes: dict[str, NodeClass] = {}
    for item in unit.classes:
        classes[item.qualname] = _merge_class(classes.get(item.qualname), item)
    sites: dict[str, list[tuple[tuple[str | None, ...], int]]] = {}
    for construction, arguments, line in unit.construction_sites:
        matches = [
            name
            for name in classes
            if construction == name
            or ("::" not in construction and name.split("::")[-1] == construction)
        ]
        if len(matches) == 1:
            sites.setdefault(matches[0], []).append((arguments, line))
    for name, item in classes.items():
        if item.name_position is None or item.stated_name:
            continue
        classes[name] = _bound_constructor(item, sites.get(name, []))
    unit.classes = [
        classes[item.qualname] if item.name_position is not None else item for item in unit.classes
    ]


def _main_constructions(root: Any, src: bytes) -> list[tuple[str, list[Any], Any]]:
    out: list[tuple[str, list[Any], Any]] = []
    if b"main" not in src:
        return out
    pending = [(root, False)]
    while pending:
        node, in_main = pending.pop()
        if node.type == "function_definition":
            declarator = _child(node, "declarator")
            while declarator is not None and declarator.type != "function_declarator":
                declarator = _child(declarator, "declarator")
            name = _child(declarator, "declarator") if declarator is not None else None
            in_main = name is not None and _text(name, src) == "main"
        if node.type == "lambda_expression":
            continue
        if in_main and node.type == "call_expression":
            name, _, template = _call_parts(node, src)
            types = _template_types(template, src)
            construction = types[0] if name in ("make_shared", "make_unique") and types else ""
            if not construction:
                function = _child(node, "function")
                construction = _text(function, src) if function is not None else ""
            if _literal_type(construction):
                out.append((construction, _args(node), node))
        elif in_main and node.type == "new_expression":
            type_node = _child(node, "type")
            construction = _text(type_node, src) if type_node is not None else ""
            arguments = _child(node, "arguments")
            if _literal_type(construction):
                out.append(
                    (construction, list(arguments.named_children) if arguments else [], node)
                )
        elif in_main and node.type == "declaration":
            type_node = _child(node, "type")
            for child in node.named_children:
                value = _child(child, "value")
                if (
                    child.type == "init_declarator"
                    and value is not None
                    and value.type == "argument_list"
                    and type_node is not None
                ):
                    construction = _text(type_node, src)
                    if _literal_type(construction):
                        out.append((construction, list(value.named_children), child))
        pending.extend((child, in_main) for child in reversed(node.named_children))
    return out


def _literal_type(text: str) -> bool:
    return bool(
        re.fullmatch(r"(?:::)?[A-Za-z_]\w*(?:::[A-Za-z_]\w*)*(?:<[^;{}()]+>)?", text)
    ) and text not in {"auto", "decltype"}


def _resolve_construction_calls(units: Sequence[Unit]) -> None:
    """A call expression alone cannot distinguish a function from a constructor."""
    types: dict[str, set[str]] = {}
    for unit in units:
        types.setdefault(unit.pkg, set()).update(unit.class_names)
        types[unit.pkg].update(row["name"] for row in unit.type_aliases)
    for unit in units:
        for row in unit.construction_calls:
            name = re.sub(r"<.*>", "", row["class"]).removeprefix("::")
            matches = {
                item
                for item in types[unit.pkg]
                if item == name or ("::" not in name and item.rsplit("::", 1)[-1] == name)
            }
            if len(matches) == 1 and row not in unit.constructions:
                unit.constructions.append(row)
        unit.constructions.sort(key=lambda row: (row["class"], row["line_hint"]))


def _aliased_interfaces(root: Any, src: bytes, endpoints: list[Endpoint]) -> list[Endpoint]:
    """An endpoint typed by an alias the file states for one interface, class scope included
    (`using Dock = fleet_msgs::action::Dock;` then `create_server<Dock>`), states the
    interface. A name two aliases of the file spell differently stays as written."""
    if not any(item.type_text and not interface_parts(item.type_text) for item in endpoints):
        return endpoints
    targets: dict[str, set[str]] = {}
    pending = [root]
    while pending:
        node = pending.pop()
        if node.type in {"alias_declaration", "type_definition"}:
            name = _child(node, "name") or _child(node, "declarator")
            target = _child(node, "type")
            if name is not None and target is not None:
                targets.setdefault(_text(name, src), set()).add(_text(target, src).strip())
        pending.extend(node.named_children)
    out = []
    for item in endpoints:
        spelled = targets.get(item.type_text.rpartition("::")[2], set())
        if item.type_text and not interface_parts(item.type_text) and len(spelled) == 1:
            target = next(iter(spelled))
            if interface_parts(target):
                item = replace(item, type_text=target)
        out.append(item)
    return out


def _type_aliases(root: Any, src: bytes) -> list[dict[str, Any]]:
    """Namespace aliases stay qualified so unrelated package classes cannot collide."""
    out: list[dict[str, Any]] = []
    pending = [root]
    while pending:
        node = pending.pop()
        if node.type in {"function_definition", "class_specifier", "struct_specifier"}:
            continue
        if node.type in {"alias_declaration", "type_definition"}:
            name = _child(node, "name") or _child(node, "declarator")
            target = _child(node, "type")
            if name is not None and target is not None:
                spelling = _text(target, src)
                if _literal_type(spelling):
                    namespace = "::".join(_namespaces_of(node, src))
                    alias = _text(name, src)
                    if not alias.isidentifier():
                        continue
                    out.append(
                        {
                            "name": f"{namespace}::{alias}" if namespace else alias,
                            "target": spelling,
                            "namespace": namespace,
                            "line_hint": _line(node),
                        }
                    )
        pending.extend(reversed(node.named_children))
    return sorted(out, key=lambda row: (row["name"], row["target"]))


_PLUGIN_LOADER = re.compile(r"\bClassLoader\s*<\s*([A-Za-z_][\w:]*)\s*>")
"""`pluginlib::ClassLoader<Base>`: the file loads plugins of that base class by a name only
the run knows, so any class a manifest registers for it may run in the node."""


def parse_file(path: str, *, pkg: str = "", relpath: str = "", parser: Any = None) -> Unit:
    try:
        text = Path(path).read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        raise CollectorError(f"cannot read {path}: {exc}", LAYER, NAME) from exc
    return parse_text(text, path=path, pkg=pkg, relpath=relpath, parser=parser)


def _called(name: str) -> bytes:
    return rf"\b{re.escape(name)}\s*[<(]".encode()


def share_wrappers(units: list[Unit], parser: Any) -> list[Unit]:
    """Read again each file that calls a declare wrapper another file of its package
    defines, `utils::declareParameter<double>(parameters, "v_ratio")` against a template in
    a header, so the call states the name it declares. A lambda stays in its own file. A file
    that initializes a member another file declares as a known wrapper (`diagnostics_(this)`
    against `diagnostic_updater::Updater diagnostics_;`) is read again the same way."""
    shared: dict[str, dict[str, tuple[int, int]]] = {}
    held: dict[str, dict[tuple[str, str], set[str]]] = {}
    for unit in sorted(units, key=lambda item: item.relpath):
        if unit.syntax is not None and unit.wrappers:
            for symbol, shape in _wrappers(unit.syntax, unit.source, lambdas=False).items():
                shared.setdefault(unit.pkg, {}).setdefault(symbol, shape)
        for (klass, member), wrapper in unit.member_wrappers.items():
            held.setdefault(unit.pkg, {}).setdefault(
                (klass.rpartition("::")[2], member), set()
            ).add(wrapper)
    out: list[Unit] = []
    for unit in units:
        extra = {
            symbol: shape
            for symbol, shape in shared.get(unit.pkg, {}).items()
            if symbol not in unit.wrappers
            and re.search(_called(symbol.rpartition("::")[2]), unit.source)
        }
        local = {(klass.rpartition("::")[2], member) for klass, member in unit.member_wrappers}
        members = {
            key: next(iter(found))
            for key, found in held.get(unit.pkg, {}).items()
            if len(found) == 1
            and key not in local
            and re.search(rf"\b{re.escape(key[1])}\s*[({{]".encode(), unit.source)
        }
        if not extra and not members:
            out.append(unit)
            continue
        out.append(
            parse_text(
                unit.source.decode("utf-8", "replace"),
                path=unit.path,
                pkg=unit.pkg,
                relpath=unit.relpath,
                parser=parser,
                shared_wrappers=extra,
                shared_members=members,
            )
        )
    return out


# ---------------------------------------------------------------------------
# the cross file join


def analyse(
    units: Sequence[Unit], *, include_dirs: dict[str, list[str]] | None = None
) -> CodeIndex:
    """Fold the units into one index and finish the names a second file completes.

    A node class states its base in the header and its name in the source, and a member that
    holds a topic is written in one file and read in another, so a per file pass alone
    reports a blind spot the tree does not have.
    """
    _resolve_construction_calls(units)
    _package_construction_values(units, include_dirs or {})
    _helper_arguments(units)
    definitions = {id(unit): list(unit.classes) for unit in units}
    index = CodeIndex(units=list(units))
    for unit in units:
        for item in unit.classes:
            if item.construction:
                continue
            current = index.classes.get(item.qualname)
            index.classes[item.qualname] = _merge_class(current, item)
        for key, value in unit.members.items():
            current_member = index.members.get(key)
            if current_member is None or (
                current_member.origin != "parameter"
                and (
                    (value.origin == "parameter" and value.source_name)
                    or (current_member.status != LITERAL and value.status == LITERAL)
                )
            ):
                index.members[key] = value
    for unit in units:
        unit.endpoints = [
            replace(
                item,
                name=_finish(item.name, index),
                lifecycle=_is_lifecycle(index, item.owner_class),
            )
            for item in unit.endpoints
        ]
        unit.params = [_finish_param(item, index) for item in unit.params]
        unit.reads = [replace(item, name=_finish(item.name, index)) for item in unit.reads]
        unit.broadcasts = [
            replace(item, parent=_finish(item.parent, index), child=_finish(item.child, index))
            for item in unit.broadcasts
        ]
        unit.lookups = [
            replace(item, target=_finish(item.target, index), source=_finish(item.source, index))
            for item in unit.lookups
        ]
        for position, item in enumerate(unit.classes):
            if not item.construction:
                unit.classes[position] = _merge_class(index.classes.get(item.qualname), item)
    _plugin_suffixes(units)
    _finish_node_helpers(units)
    _member_constructions(units)
    _local_constructions(units)
    _header_constructions(units, include_dirs or {}, definitions)
    _map_defaults(units, include_dirs or {})
    _member_defaults(units)
    publishing: dict[tuple[str, str, str], set[str]] = {}
    for unit in units:
        for (owner, member), symbols in unit.publishing.items():
            publishing.setdefault((unit.pkg, owner, member), set()).update(symbols)
    for unit in units:
        for item in unit.endpoints:
            if item.attribute:
                unit.publishing[(item.owner_class, item.attribute)] = publishing.get(
                    (unit.pkg, item.owner_class, item.attribute), set()
                )
    return index


def _member_constructions(units: Sequence[Unit]) -> None:
    """A wrapper member a header builds where it declares it, `odom_sub_{this, "topic"}`, is
    built by the constructor, which a source file of the package defines."""
    headers = (".h", ".hpp", ".hh", ".hxx")
    defined: dict[tuple[str, str], Unit] = {}
    for unit in units:
        if not unit.relpath.endswith(headers):
            for method in unit.methods:
                defined.setdefault((unit.pkg, method), unit)
    for unit in units:
        if not unit.relpath.endswith(headers):
            continue
        moved = [
            item
            for item in unit.endpoints
            if item.api in KNOWN_WRAPPERS and (unit.pkg, item.sym) in defined
        ]
        unit.endpoints = [item for item in unit.endpoints if item not in moved]
        for item in moved:
            # the line is the header's, and the evidence of the unit it moves to cites the source
            defined[(unit.pkg, item.sym)].endpoints.append(replace(item, line=None))


_MEMBER_TYPES = {"double": float, "int": int, "int64_t": int, "bool": bool, "std::string": str}


def _member_defaults(units: Sequence[Unit]) -> None:
    """A default that names a member, read outside the constructor, is the literal the
    constructor or the class body gives it, when the declaration that reads it is the only
    write to that member anywhere in the package."""
    for unit in units:
        pending = [
            (position, item)
            for position, item in enumerate(unit.params)
            if item.default_state == "unknown"
            and item.default_ref is not None
            and item.default_ref.member
            and item.owner_class
        ]
        for position, item in pending:
            value = _member_initial(units, unit.pkg, item.owner_class, item.default_ref.member)
            if value is None or _member_written(units, unit, item.default_ref.member, item):
                continue
            unit.params[position] = replace(
                item,
                default=value,
                default_state="known",
                value_type=item.value_type or _type_of(value),
            )


def _type_tail(name: str) -> str:
    return re.sub(r"<.*>", "", name).rsplit("::", 1)[-1]


def _member_initial(units: Sequence[Unit], pkg: str, owner: str, member: str) -> Any:
    tail = _type_tail(owner)
    types: set[str] = set()
    in_class: list[Any] = []
    constructors: list[Any] = []
    for unit in units:
        if unit.pkg != pkg or unit.syntax is None:
            continue
        src = unit.source
        stack = [unit.syntax]
        while stack:
            node = stack.pop()
            stack.extend(node.named_children)
            if node.type == "field_declaration":
                parent = node.parent.parent if node.parent is not None else None
                label = _child(parent, "name") if parent is not None else None
                if label is None or _type_tail(_flat(label, src)) != tail:
                    continue
                for child in node.named_children:
                    if child.type == "field_identifier" and _flat(child, src) == member:
                        types.add(_flat(_child(node, "type"), src))
                        in_class.append((_child(node, "default_value"), src))
            elif node.type == "function_definition":
                declarator = _child(node, "declarator")
                while declarator is not None and declarator.type != "function_declarator":
                    declarator = _child(declarator, "declarator")
                function = _child(declarator, "declarator") if declarator is not None else None
                if function is None:
                    continue
                parts = [_type_tail(part) for part in _flat(function, src).split("::")]
                enclosing = node.parent.parent if node.parent is not None else None
                label = _child(enclosing, "name") if enclosing is not None else None
                if parts[-1] != tail or (
                    parts[-2:-1] != [tail]
                    and (label is None or _type_tail(_flat(label, src)) != tail)
                ):
                    continue
                stated = None
                for initializers in node.named_children:
                    if initializers.type != "field_initializer_list":
                        continue
                    for initializer in initializers.named_children:
                        named = initializer.named_children
                        if len(named) == 2 and _flat(named[0], src) == member:
                            stated = (named[1], src)
                constructors.append(stated)
    if len(types) != 1 or len(in_class) != 1 or len(constructors) != 1:
        return None
    kind = _MEMBER_TYPES.get(next(iter(types)))
    stated = constructors[0]
    if stated is not None:
        arguments, src = stated
        if arguments.named_child_count != 1:
            return None
        value = _literal_value(arguments.named_children[0], src)
    else:
        default, src = in_class[0]
        if default is None:
            return None
        if default.type == "initializer_list":
            if default.named_child_count != 1:
                return None
            default = default.named_children[0]
        value = _literal_value(default, src)
    if kind is None or value is None or isinstance(value, list):
        return None
    if kind is float and isinstance(value, (int, float)) and not isinstance(value, bool):
        return float(value)
    return value if type(value) is kind else None


def _names_member(text: str, member: str) -> bool:
    """Whether an expression is the member, through `this`, another object or a pointer."""
    tail = re.split(r"->|\.|::", text.lstrip("&*( ").rstrip(") "))[-1]
    return tail == member


def _member_written(units: Sequence[Unit], owner: Unit, member: str, site: ParamSite) -> bool:
    """Whether anything but the assignment of this declaration can write the member.

    A name is matched in every file of the package and on any object, so a write through
    another pointer to the same node counts too.
    """
    for unit in units:
        if unit.pkg != owner.pkg or unit.syntax is None:
            continue
        src = unit.source
        stack = [unit.syntax]
        while stack:
            node = stack.pop()
            stack.extend(node.named_children)
            if node.type == "assignment_expression":
                left = _child(node, "left")
                right = _child(node, "right")
                if left is None or not _names_member(_flat(left, src), member):
                    continue
                if (
                    unit is owner
                    and right is not None
                    and right.type == "call_expression"
                    and _call_parts(right, src)[0] in DECLARE_CALLS
                    and _line(right) == site.line
                ):
                    continue
                return True
            if node.type == "update_expression" and any(
                _names_member(_flat(child, src), member) for child in node.named_children
            ):
                return True
            if node.type == "binary_expression":
                operator = _child(node, "operator")
                right = _child(node, "right")
                if (
                    operator is not None
                    and _text(operator, src) == ">>"
                    and right is not None
                    and _names_member(_flat(right, src), member)
                ):
                    return True
            if node.type == "init_declarator":
                target = _child(node, "declarator")
                value = _child(node, "value")
                if (
                    target is not None
                    and value is not None
                    and _names_member(_flat(value, src), member)
                    and (target.type == "reference_declarator" or _flat(value, src).startswith("&"))
                ):
                    return True
            if node.type == "call_expression":
                api, receiver, _ = _call_parts(node, src)
                if receiver is not None and _names_member(_flat(receiver, src), member):
                    return True
                if api in DECLARE_CALLS | PLUGIN_DECLARE_CALLS | {"has_parameter"}:
                    continue
                arguments = _args(node)
                if api in READ_CALLS:
                    arguments = [item for item in arguments if not _is_node_argument(item, src)]
                    arguments = arguments[1:]
                if any(_names_member(_flat(argument, src), member) for argument in arguments):
                    return True
    return False


def _package_construction_values(units: Sequence[Unit], include_dirs: dict[str, list[str]]) -> None:
    """A constructor another file of the package declares says whether a call writes its
    argument, and a header main includes can state the constant main passes, so a value
    main passes on is known only once those files are read too."""
    tables: dict[str, dict[str, list[tuple[int, list[str]]]]] = {}
    files: dict[str, set[str]] = {}
    for unit in units:
        files.setdefault(unit.pkg, set()).add(unit.relpath)
        if unit.syntax is None:
            continue
        table = tables.setdefault(unit.pkg, {})
        found = unit.signatures
        if found is None:
            found = _signatures(unit.syntax, unit.source)
        for key, rows in found.items():
            table.setdefault(key, []).extend(rows)
    for unit in units:
        if unit.syntax is None or not unit.construction_sites:
            continue
        paths = _included_paths(unit, files.get(unit.pkg, set()), include_dirs)
        again = _construction_values(
            unit.syntax,
            unit.source,
            dict(_DEFINE.findall(unit.source.decode("utf-8", "replace"))),
            tables[unit.pkg],
            _visible_constants(unit, units, paths),
        )
        unit.construction_sites = [
            (
                name,
                tuple(
                    old if old is not None else new for old, new in zip(values, fresh, strict=True)
                ),
                line,
            )
            for (name, values, line), (_, fresh, _) in zip(
                unit.construction_sites, again, strict=True
            )
        ]


def _helper_constructions(root: Any, src: bytes) -> list[tuple[str, list[Any]]]:
    out: list[tuple[str, list[Any]]] = []
    pending = [root] if b"make_" in src or b"new" in src else []
    while pending:
        node = pending.pop()
        pending.extend(node.named_children)
        if node.type == "new_expression":
            type_node = _child(node, "type")
            if type_node is not None:
                out.append((_text(type_node, src), _args(node)))
        elif node.type == "call_expression":
            name, _receiver, template = _call_parts(node, src)
            types = _template_types(template, src)
            if name in ("make_unique", "make_shared") and types:
                out.append((types[0], _args(node)))
    return out


def _enclosing_method(node: Any, src: bytes) -> str:
    while node is not None and node.type != "function_definition":
        node = node.parent
    target = _child(node, "declarator") if node is not None else None
    while target is not None and target.type != "function_declarator":
        target = _child(target, "declarator")
    inner = _child(target, "declarator") if target is not None else None
    return _flat(inner, src).rpartition("::")[2] if inner is not None else ""


def _helper_arguments(units: Sequence[Unit]) -> None:
    """A class that is not a node and that its package constructs at one site takes the
    literal arguments of that site into its constructor, as a node class main constructs once
    already does. `new sensors::Imu(node_handle_, "imu")` states the topic the constructor
    publishes on. Two sites in the package, or two classes it names alike, state nothing, and
    the constructor keeps its blind spots."""
    names: dict[str, set[str]] = {}
    nodes: set[tuple[str, str]] = set()
    for unit in units:
        names.setdefault(unit.pkg, set()).update(unit.class_names)
        nodes.update((unit.pkg, item.qualname) for item in unit.classes)

    def match(pkg: str, spelling: str) -> str:
        spelling = re.sub(r"<.*", "", spelling).strip().removeprefix("::")
        found = {
            name
            for name in names.get(pkg, set())
            if spelling and (name == spelling or name.endswith("::" + spelling))
        }
        return next(iter(found)) if len(found) == 1 else ""

    sites: dict[tuple[str, str], list[tuple[str | None, ...]]] = {}
    builders: dict[tuple[str, str], Unit] = {}
    for unit in units:
        if unit.syntax is None:
            continue
        defines = dict(_DEFINE.findall(unit.source.decode("utf-8", "replace")))
        # only a plugin hands a helper its instance name
        plugin = any(item.macro == "PLUGINLIB_EXPORT_CLASS" for item in unit.registrations)
        names_held = (
            {member for (_klass, member), ref in unit.members.items() if ref.origin == PLUGIN_NAME}
            | {"getName()"}
            if plugin
            else set()
        )
        for spelling, arguments in _helper_constructions(unit.syntax, unit.source):
            values = [_string_of(argument, unit.source) for argument in arguments]
            texts = [_flat(argument, unit.source).removeprefix("this->") for argument in arguments]
            if plugin and unit.plugin_name_locals:
                texts = [
                    PLUGIN_NAME_ARGUMENT
                    if text in unit.plugin_name_locals
                    and _enclosing_method(argument, unit.source) in PLUGIN_SETUP_METHODS
                    else text
                    for text, argument in zip(texts, arguments, strict=True)
                ]
                names_held.add(PLUGIN_NAME_ARGUMENT)
            key = (unit.pkg, match(unit.pkg, spelling))
            sites.setdefault(key, []).append(
                tuple(
                    value
                    if value is not None
                    else PLUGIN_NAME_ARGUMENT
                    if text in names_held
                    else defines.get(text)
                    for text, value in zip(texts, values, strict=True)
                )
            )
            builders[key] = unit
    for unit in units:
        if unit.syntax is None or unit.construction_sites:
            continue
        for klass in sorted(
            {
                method.rpartition("::")[0]
                for method in unit.methods
                if method.rpartition("::")[2] == method.rpartition("::")[0].rpartition("::")[2]
            }
        ):
            qualname = match(unit.pkg, klass)
            if (
                not qualname
                or len(sites.get((unit.pkg, qualname), [])) != 1
                or any(
                    (unit.pkg, name) in nodes
                    for name in (qualname, klass, qualname.rpartition("::")[2])
                )
            ):
                continue
            values = sites[(unit.pkg, qualname)]
            fresh = Unit(
                path=unit.path,
                pkg=unit.pkg,
                relpath=unit.relpath,
                wrappers=unit.wrappers.copy(),
                shared_wrappers=unit.shared_wrappers,
                shared_members=unit.shared_members,
                lambda_wrappers=unit.lambda_wrappers,
            )
            reader = _Reader(
                unit.source, fresh, dict(_DEFINE.findall(unit.source.decode("utf-8", "replace")))
            )
            reader.constructor_arguments[klass] = values
            reader.walk(unit.syntax)
            if PLUGIN_NAME_ARGUMENT in values[0]:
                unit.plugin_of_helper = next(
                    (
                        item
                        for item in builders[(unit.pkg, qualname)].registrations
                        if item.macro == "PLUGINLIB_EXPORT_CLASS"
                    ),
                    None,
                )
            constructor = f"{klass}::{klass.rpartition('::')[2]}"
            for record_key in ("endpoints", "params", "reads", "broadcasts", "lookups"):
                setattr(
                    unit,
                    record_key,
                    [item for item in getattr(unit, record_key) if item.sym != constructor]
                    + [item for item in getattr(fresh, record_key) if item.sym == constructor],
                )
            for member, value in fresh.members.items():
                held = unit.members.get(member)
                if (
                    member[0] == klass
                    and (value.status == LITERAL or value.origin == PLUGIN_NAME)
                    and (held is None or held.status != LITERAL)
                ):
                    unit.members[member] = value


def _finish_node_helpers(units: Sequence[Unit]) -> None:
    fields: dict[tuple[str, str, str], set[str]] = {}
    for unit in units:
        for (klass, member), spelled in unit.field_types.items():
            fields.setdefault((unit.pkg, klass.rpartition("::")[2], member), set()).add(spelled)
    for unit in units:
        for klass, member, handed, held in unit.member_uses:
            found = fields.get((unit.pkg, klass.rpartition("::")[2], member), set())
            if len(found) != 1:
                continue
            spelled = next(iter(found))
            if handed:
                unit.node_handoffs.setdefault(klass, set()).add(spelled)
            if held is not None:
                unit.handle_uses.append((klass, held, ("type", spelled)))
    types: dict[tuple[str, str, str], set[str]] = {}
    plugins: dict[tuple[str, str, str], set[str]] = {}
    helpers: dict[tuple[str, str], set[str]] = {}
    selectors: dict[tuple[str, str], set[str]] = {}
    nodes = {
        (unit.pkg, item.qualname)
        for unit in units
        for item in unit.classes
        if item.base in NODE_BASES
    }
    bases: dict[tuple[str, str], set[str]] = {}
    for unit in units:
        for name, values in unit.class_bases.items():
            bases.setdefault((unit.pkg, name), set()).update(values)
    changed = True
    while changed:
        changed = False
        for key, values in sorted(bases.items()):
            if key in nodes:
                continue
            namespace = key[1].rpartition("::")[0]
            for base in values:
                candidates = {(key[0], base), (key[0], f"{namespace}::{base}")}
                if candidates & nodes:
                    nodes.add(key)
                    changed = True
                    break
    handles: dict[tuple[str, str, str], set[bool]] = {}
    for unit in units:
        for (scope, name), values in unit.helper_types.items():
            types.setdefault((unit.pkg, scope, name), set()).update(values)
        for (scope, name), values in unit.helper_plugins.items():
            plugins.setdefault((unit.pkg, scope, name), set()).update(values)
        for (scope, name), held in unit.node_handles.items():
            handles.setdefault((unit.pkg, scope, name), set()).update(held)

    def node_class(pkg: str, name: str) -> str:
        # `using ns::Host;` then `Host::start()` spells the class without its namespace
        found = {
            qualname
            for package, qualname in nodes
            if package == pkg
            and (qualname == name or ("::" not in name and qualname.rpartition("::")[2] == name))
        }
        found = {qualname for qualname in found if "::" in qualname} or found
        return next(iter(found)) if len(found) == 1 else ""

    for unit in units:
        for klass, keys, use in unit.handle_uses:
            owner = node_class(unit.pkg, klass)
            if not owner or (
                keys and not any(handles.get((unit.pkg, *key)) == {True} for key in keys)
            ):
                continue
            if use[0] == "type":
                helpers.setdefault((unit.pkg, owner), set()).add(use[1])
                continue
            scope, member = use[1], use[2]
            values = types.get((unit.pkg, scope, member), set())
            if len(values) == 1 and next(iter(values)):
                helpers.setdefault((unit.pkg, owner), set()).update(values)
            values = plugins.get((unit.pkg, scope, member), set())
            if len(values) == 1 and next(iter(values)):
                selectors.setdefault((unit.pkg, owner), set()).update(values)
    handoffs: dict[tuple[str, str], set[str]] = {}
    for unit in units:
        for name, values in unit.node_handoffs.items():
            handoffs.setdefault((unit.pkg, name.rpartition("::")[2]), set()).update(values)
    defined = {(unit.pkg, name.rpartition("::")[2]) for unit in units for name in unit.class_names}
    for unit in units:
        # the class row sits in the file that defines the class, often a header, and in the
        # file that uses it when no file this reader parsed defines it
        unit.node_handoffs = {
            name: handoffs[(unit.pkg, name.rpartition("::")[2])]
            for name in [
                *unit.class_names,
                *(
                    name
                    for name in unit.node_handoffs
                    if (unit.pkg, name.rpartition("::")[2]) not in defined
                ),
            ]
            if (unit.pkg, name.rpartition("::")[2]) in handoffs
        }
    for unit in units:
        unit.class_helpers.clear()
        unit.class_node_plugins.clear()
        for name in unit.class_names:
            if (unit.pkg, name) in helpers:
                unit.class_helpers[name] = helpers[(unit.pkg, name)]
            if (unit.pkg, name) in selectors:
                unit.class_node_plugins[name] = selectors[(unit.pkg, name)]


def _map_identifier(node: Any, src: bytes) -> str:
    text = _flat(node, src).lstrip("&*") if node is not None else ""
    return text if re.fullmatch(r"[A-Za-z_]\w*(?:::[A-Za-z_]\w*)*", text) else ""


def _string_map_records(
    root: Any, src: bytes, *, mutations_wanted: bool = True
) -> tuple[dict[str, tuple[str, int]], set[str]]:
    """The namespace scope string maps, and the names something may mutate. A map is only
    collected outside a function or class body, so the maps alone skip every body."""
    maps: dict[str, tuple[str, int]] = {}
    mutations: set[str] = set()
    pending = [(root, False)]
    reads = {
        "begin",
        "end",
        "cbegin",
        "cend",
        "find",
        "at",
        "size",
        "empty",
        "count",
        "contains",
        "lower_bound",
        "upper_bound",
        "equal_range",
    }
    while pending:
        node, local = pending.pop()
        local = local or node.type in {"function_definition", "class_specifier", "struct_specifier"}
        if local and not mutations_wanted:
            continue
        if node.type == "declaration" and not local:
            type_node = _child(node, "type")
            spelling = re.sub(r"\s+", "", _flat(type_node, src)) if type_node is not None else ""
            bounded = any(
                child.type in {"storage_class_specifier", "type_qualifier"}
                and _flat(child, src) in {"static", "const"}
                for child in node.named_children
            )
            if bounded and re.fullmatch(r"std::map<std::string,[^,<>]+>", spelling):
                for child in node.named_children:
                    target, value = _child(child, "declarator"), _child(child, "value")
                    if (
                        target is None
                        or target.type != "identifier"
                        or value is None
                        or value.type != "initializer_list"
                    ):
                        continue
                    keys = []
                    for pair in value.named_children:
                        if pair.type == "comment":
                            continue
                        entries = [
                            entry for entry in pair.named_children if entry.type != "comment"
                        ]
                        key = _string_of(entries[0], src) if len(entries) == 2 else None
                        if key is None or not key.isascii() or chr(92) in _text(entries[0], src):
                            keys = []
                            break
                        keys.append(key)
                    if keys:
                        name = "::".join([*_namespaces_of(node, src), _text(target, src)])
                        maps[name] = (min(keys), _line(target))
        if not mutations_wanted:
            pending.extend((child, local) for child in node.named_children)
            continue
        if node.type == "call_expression":
            api, receiver, _template = _call_parts(node, src)
            if receiver is not None and api not in reads:
                mutations.add(_map_identifier(receiver, src))
            mutations.update(_map_identifier(argument, src) for argument in _args(node))
        if node.type in {"assignment_expression", "update_expression"}:
            target = (
                _child(node, "left")
                if node.type == "assignment_expression"
                else _child(node, "argument")
            )
            while target is not None and target.type in {
                "subscript_expression",
                "field_expression",
            }:
                target = _child(target, "argument")
            mutations.add(_map_identifier(target, src))
        if node.type == "init_declarator":
            target, value = _child(node, "declarator"), _child(node, "value")
            if target is not None and target.type in {"reference_declarator", "pointer_declarator"}:
                mutations.add(_map_identifier(value, src))
        if node.type in {
            "argument_list",
            "parameter_list",
            "initializer_list",
            "return_statement",
            "binary_expression",
        }:
            mutations.update(_map_identifier(child, src) for child in node.named_children)
        if node.type == "pointer_expression" and _flat(node, src).startswith("&"):
            mutations.add(_map_identifier(_child(node, "argument"), src))
        if node.type == "ERROR":
            mutations.update(re.findall(r"[A-Za-z_]\w*", _text(node, src)))
        pending.extend((child, local) for child in node.named_children)
    return maps, mutations - {""}


def _map_defaults(units: Sequence[Unit], include_dirs: dict[str, list[str]]) -> None:
    files = {(unit.pkg, unit.relpath): unit for unit in units}
    mutations: dict[str, set[str]] = {}

    def mutated(pkg: str) -> set[str]:
        if pkg not in mutations:
            found = mutations[pkg] = set()
            for unit in units:
                if unit.pkg != pkg:
                    continue
                if unit.tree is not None and not unit.map_mutations:
                    unit.map_mutations = _string_map_records(unit.syntax, unit.source)[1]
                found.update(name.rsplit("::", 1)[-1] for name in unit.map_mutations)
        return mutations[pkg]

    for unit in units:
        visible = {unit.relpath}
        search_dirs = (
            include_dirs.get(ids.build_file(unit.pkg, unit.relpath), include_dirs.get(unit.pkg, []))
            if unit.pkg and unit.relpath
            else []
        )
        for include, quoted in unit.includes:
            directories = ([posixpath.dirname(unit.relpath)] if quoted else []) + [
                *search_dirs,
                "include",
                f"include/{unit.pkg}",
            ]
            for directory in directories:
                path = posixpath.normpath(posixpath.join(directory, include))
                if (unit.pkg, path) in files:
                    visible.add(path)
                    break
        for position, parameter in enumerate(unit.params):
            match = re.fullmatch(
                r"([A-Za-z_]\w*(?:::[A-Za-z_]\w*)*)\.begin\(\)->first",
                re.sub(r"\s+", "", parameter.default_expr),
            )
            if match is None or parameter.default_state != "unknown":
                continue
            spelling = match.group(1)
            choices = []
            for path in sorted(visible):
                definition = files[(unit.pkg, path)]
                for name, (value, line) in definition.string_maps.items():
                    namespace, _, tail = name.rpartition("::")
                    if tail in mutated(unit.pkg):
                        continue
                    if spelling == name or (
                        spelling == tail
                        and (not namespace or parameter.sym.startswith(namespace + "::"))
                    ):
                        choices.append((definition, name, value, line))
            if len(choices) == 1:
                definition, name, value, line = choices[0]
                unit.params[position] = replace(
                    parameter,
                    default=value,
                    default_state="known",
                    value_type="string",
                    default_site=(definition.path, name, line),
                )


def _local_constructions(units: Sequence[Unit]) -> None:
    for unit in units:
        classes = {item.qualname for item in unit.classes if item.symbol and not item.construction}
        selected: dict[str, list[tuple[tuple[str | None, ...], int]]] = {}
        for spelling, arguments, line in unit.construction_sites:
            matches = [
                name
                for name in classes
                if name == spelling
                or ("::" not in spelling and name.rsplit("::", 1)[-1] == spelling)
            ]
            if len(matches) == 1:
                selected.setdefault(matches[0], []).append((arguments, line))
        if sum(len(sites) for sites in selected.values()) > 1:
            for name, sites in selected.items():
                _construction_instances(unit, unit, name, sites, units)


def _construction_instances(
    main: Unit,
    definition: Unit,
    name: str,
    sites: list[tuple[tuple[str | None, ...], int]],
    units: Sequence[Unit],
    defaults: dict[int, tuple[str | None, ...]] | None = None,
) -> None:
    if definition.syntax is None:
        return
    # A registered component is also loaded without this main, with no context to select
    # records by, so its unscoped records stay beside the scoped ones.
    component = name.rsplit("::", 1)[-1] in _components(units).get(definition.pkg, set())
    if not component:
        definition.constructed_classes.add(name)
        main.constructed_classes.add(name)
    sources = [
        unit
        for unit in units
        if unit.pkg == definition.pkg
        and (unit is definition or any(symbol.startswith(name + "::") for symbol in unit.methods))
    ]
    fields = ("endpoints", "params", "reads", "broadcasts", "lookups")
    for source in [] if component else sources:
        for record_key in fields:
            setattr(
                source,
                record_key,
                [
                    item
                    for item in getattr(source, record_key)
                    if not item.sym.startswith(name + "::") or item.construction
                ],
            )
    existing = {item.construction for item in main.classes if item.construction}
    for arguments, line in sites:
        context = ids.digest_fields(
            {
                "class": name,
                "file": ids.build_file(
                    main.pkg or ids.REPO_PKG, main.relpath or Path(main.path).name
                ),
                "arguments": ["unknown" if arg is None else "known:" + arg for arg in arguments],
            }
        )
        if context in existing:
            continue
        unit = Unit(
            path=definition.path,
            pkg=definition.pkg,
            relpath=definition.relpath,
            wrappers=definition.wrappers.copy(),
            shared_wrappers=definition.shared_wrappers,
            shared_members=definition.shared_members,
            lambda_wrappers=definition.lambda_wrappers,
        )
        reader = _Reader(definition.source, unit, dict(_DEFINE.findall(definition.source.decode())))
        # A parameter the call leaves out takes the default its declaration states, the same
        # for every call, so it stays out of the context digest.
        reader.constructor_arguments[name] = [
            tuple(arguments) + (defaults or {}).get(len(arguments), ())
        ]
        reader.walk(definition.syntax)
        item = next((item for item in unit.classes if item.qualname == name and item.symbol), None)
        if item is None:
            continue
        main.classes.append(
            replace(
                _bound_constructor(item, [(arguments, line)]),
                symbol="main",
                name_line=line,
                construction=context,
                construction_ambiguous=sum(values == arguments for values, _ in sites) > 1,
            )
        )
        members = CodeIndex(members=unit.members.copy())
        scoped_units = []
        for source in sources:
            if source is definition:
                scoped = unit
            else:
                scoped = Unit(
                    path=source.path,
                    pkg=source.pkg,
                    relpath=source.relpath,
                    members=unit.members.copy(),
                    wrappers=source.wrappers.copy(),
                    shared_wrappers=source.shared_wrappers,
                    shared_members=source.shared_members,
                    lambda_wrappers=source.lambda_wrappers,
                )
                _Reader(source.source, scoped, dict(_DEFINE.findall(source.source.decode()))).walk(
                    source.syntax
                )
                members.members.update(
                    {
                        key: value
                        for key, value in scoped.members.items()
                        if value != unit.members.get(key)
                    }
                )
            scoped_units.append((source, scoped))
        for source, scoped in scoped_units:
            source.endpoints.extend(
                replace(
                    endpoint,
                    name=_finish(endpoint.name, members),
                    construction=context,
                    lifecycle=item.form == "lifecycle_node",
                )
                for endpoint in scoped.endpoints
                if endpoint.owner_class == name
            )
            source.params.extend(
                replace(
                    _finish_param(parameter, members),
                    construction=context,
                )
                for parameter in scoped.params
                if parameter.owner_class == name
            )
            source.reads.extend(
                replace(
                    read,
                    name=_finish(read.name, members),
                    construction=context,
                )
                for read in scoped.reads
                if read.sym.startswith(name + "::")
            )
            source.broadcasts.extend(
                replace(
                    broadcast,
                    parent=_finish(broadcast.parent, members),
                    child=_finish(broadcast.child, members),
                    construction=context,
                )
                for broadcast in scoped.broadcasts
                if broadcast.sym.startswith(name + "::")
            )
            source.lookups.extend(
                replace(
                    lookup,
                    target=_finish(lookup.target, members),
                    source=_finish(lookup.source, members),
                    construction=context,
                )
                for lookup in scoped.lookups
                if lookup.sym.startswith(name + "::")
            )
        existing.add(context)


def _components(units: Sequence[Unit]) -> dict[str, set[str]]:
    """Package to the unqualified class names its registration macros export."""
    out: dict[str, set[str]] = {}
    for unit in units:
        for registration in unit.registrations:
            if registration.macro == "RCLCPP_COMPONENTS_REGISTER_NODE":
                out.setdefault(unit.pkg, set()).add(registration.type.rsplit("::", 1)[-1])
    return out


def _header_constructions(
    units: Sequence[Unit],
    include_dirs: dict[str, list[str]],
    definitions: dict[int, list[NodeClass]],
) -> None:
    declarations: dict[tuple[str, str], list[Unit]] = {}
    files: dict[str, set[str]] = {}
    for unit in units:
        files.setdefault(unit.pkg, set()).add(unit.relpath)
        for name in unit.class_names:
            declarations.setdefault((unit.pkg, name), []).append(unit)
    components = _components(units)
    plans: list[
        tuple[
            Unit,
            str,
            Unit,
            Unit,
            list[tuple[tuple[str | None, ...], int]],
            dict[int, tuple[str | None, ...]],
            list[tuple[str, int]],
        ]
    ] = []
    for main in units:
        paths = _included_paths(main, files.get(main.pkg, set()), include_dirs)
        selected: dict[str, tuple[Unit, NodeClass, list[tuple[tuple[str | None, ...], int]]]] = {}
        declared: dict[str, Unit] = {}
        spellings: dict[str, list[tuple[str, int]]] = {}
        for spelling, arguments, line in main.construction_sites:
            matches = [
                key
                for key in declarations
                if key[0] == main.pkg
                and (
                    key[1] == spelling
                    or ("::" not in spelling and key[1].rsplit("::", 1)[-1] == spelling)
                )
            ]
            if len(matches) != 1 or len(declarations[matches[0]]) != 1:
                continue
            header = declarations[matches[0]][0]
            name = matches[0][1]
            if (
                header is main
                or header.relpath not in paths
                or not header.relpath.endswith((".h", ".hpp", ".hh", ".hxx", ".ipp"))
            ):
                continue
            # The constructor is defined in the header or in exactly one other file.
            found = [
                (unit, item)
                for unit in units
                if unit.pkg == main.pkg and unit is not main
                for item in definitions[id(unit)]
                if item.qualname == name and item.symbol
            ]
            if len({id(unit) for unit, _ in found}) != 1:
                continue
            definition, item = found[0]
            if definition is not header and name.rsplit("::", 1)[-1] in components.get(
                main.pkg, set()
            ):
                # A component is also loaded without this main, and a construction context
                # would hide its records from every load that has none.
                continue
            selected.setdefault(name, (definition, item, []))[2].append((arguments, line))
            declared[name] = header
            spellings.setdefault(name, []).append((spelling, line))
        visible = _visible_constants(main, units, paths)
        for name, (definition, _item, sites) in selected.items():
            header = declared[name]
            filled = {
                len(arguments): _declared_defaults(header, name, len(arguments), visible)
                for arguments, _ in sites
            }
            plans.append((main, name, definition, header, sites, filled, spellings[name]))
    # A class whose constructor is outside its header takes contexts only when some main
    # passes it a known value. Otherwise a context would only rename the records.
    informed = {
        (main.pkg, name)
        for main, name, definition, header, sites, filled, _ in plans
        if definition is header
        or any(
            value is not None
            for arguments, _ in sites
            for value in (*arguments, *filled[len(arguments)])
        )
    }
    for main, name, definition, header, sites, filled, spelled in plans:
        if (main.pkg, name) not in informed:
            continue
        for spelling, line in spelled:
            for row in main.constructions:
                if row["class"] == spelling and row["line_hint"] == line:
                    row["definition_file"] = ids.build_file(header.pkg, header.relpath)
        _construction_instances(main, definition, name, sites, units, filled)


def _included_paths(main: Unit, files: set[str], include_dirs: dict[str, list[str]]) -> set[str]:
    """The files of main's own package that main includes directly."""
    paths = set()
    search_dirs = include_dirs.get(main.pkg, [])
    if main.pkg and main.relpath:
        search_dirs = include_dirs.get(ids.build_file(main.pkg, main.relpath), search_dirs)
    for include, quoted in main.includes:
        directories = ([posixpath.dirname(main.relpath)] if quoted else []) + [
            *search_dirs,
            "include",
            f"include/{main.pkg}",
        ]
        for directory in directories:
            path = posixpath.normpath(posixpath.join(directory, include))
            if path in files:
                paths.add(path)
                break
    return paths


def _constants(root: Any, src: bytes) -> dict[str, str]:
    """Namespace scope string constants by qualified name: a `const` or `constexpr` array or
    object initialised by a literal. A pointer to const can be reassigned and is left out."""
    out: dict[str, str] = {}
    pending: list[tuple[Any, tuple[str, ...]]] = [(root, ())]
    while pending:
        node, scope = pending.pop()
        if node.type == "namespace_definition":
            name = _child(node, "name")
            body = _child(node, "body")
            if body is not None:
                inner = (_flat(name, src),) if name is not None else ()
                pending.extend((child, scope + inner) for child in body.named_children)
            continue
        if node.type in {
            "function_definition",
            "class_specifier",
            "struct_specifier",
            "template_declaration",
            "compound_statement",
        }:
            continue
        if node.type != "declaration":
            pending.extend((child, scope) for child in node.named_children)
            continue
        if not {"const", "constexpr"} & set(_flat(node, src).partition("=")[0].split()):
            continue
        for child in node.named_children:
            if child.type != "init_declarator":
                continue
            target = _child(child, "declarator")
            value = _child(child, "value")
            if target is None or value is None:
                continue
            if target.type not in {"identifier", "array_declarator"}:
                continue
            literal = _string_of(value, src)
            if literal is not None:
                name = _flat(target, src).partition("[")[0].strip()
                out["::".join([*scope, name])] = literal
    return out


def _visible_constants(main: Unit, units: Sequence[Unit], paths: set[str]) -> dict[str, str]:
    """The string constants the headers main includes state, minus any name two state."""
    found: dict[str, set[str]] = {}
    for unit in units:
        if unit.pkg != main.pkg or unit.relpath not in paths or unit.syntax is None:
            continue
        for key, value in _constants(unit.syntax, unit.source).items():
            found.setdefault(key, set()).add(value)
    return {key: next(iter(values)) for key, values in found.items() if len(values) == 1}


def _declared_defaults(
    header: Unit, name: str, count: int, constants: dict[str, str]
) -> tuple[str | None, ...]:
    """The values the constructor declared in the class body gives the parameters a call
    with `count` arguments leaves out, when exactly one declaration takes that call."""
    if header.syntax is None:
        return ()
    src = header.source
    tail = name.rsplit("::", 1)[-1]
    candidates = []
    pending: list[tuple[Any, tuple[str, ...]]] = [(header.syntax, ())]
    while pending:
        node, scope = pending.pop()
        if node.type == "namespace_definition":
            label = _child(node, "name")
            inner = (_flat(label, src),) if label is not None else ()
            pending.extend((child, scope + inner) for child in node.named_children)
            continue
        if node.type in {"class_specifier", "struct_specifier"}:
            label = _child(node, "name")
            body = _child(node, "body")
            if label is None or body is None or "::".join([*scope, _flat(label, src)]) != name:
                continue
            for member in body.named_children:
                declarator = _child(member, "declarator")
                while declarator is not None and declarator.type != "function_declarator":
                    declarator = _child(declarator, "declarator")
                if declarator is None:
                    continue
                function = _child(declarator, "declarator")
                parameters = _child(declarator, "parameters")
                if function is None or parameters is None or _flat(function, src) != tail:
                    continue
                rows = [
                    item
                    for item in parameters.named_children
                    if item.type in {"parameter_declaration", "optional_parameter_declaration"}
                ]
                required = sum(item.type == "parameter_declaration" for item in rows)
                if required <= count <= len(rows):
                    candidates.append((scope, rows))
            continue
        pending.extend((child, scope) for child in node.named_children)
    if len(candidates) != 1:
        return ()
    scope, rows = candidates[0]
    out: list[str | None] = []
    for item in rows[count:]:
        default = _child(item, "default_value")
        value = _string_of(default, src) if default is not None else None
        if value is None and default is not None:
            # Looked up from the namespace of the class outwards, as the compiler does.
            spelled = _flat(default, src).removeprefix("::")
            for depth in range(len(scope), -1, -1):
                value = constants.get("::".join([*scope[:depth], spelled]))
                if value is not None:
                    break
        out.append(value)
    return tuple(out)


def _is_lifecycle(index: CodeIndex, owner: str) -> bool:
    found = index.classes.get(owner)
    return found is not None and found.form == "lifecycle_node"


def _merge_class(current: NodeClass | None, item: NodeClass) -> NodeClass:
    if current is None:
        return item
    named = current if current.stated_name or current.name_expr else item
    return NodeClass(
        qualname=item.qualname,
        base=current.base or item.base,
        form=current.form if current.base else item.form,
        stated_name=named.stated_name,
        name_source=named.name_source
        if named.name_source != model.NameSource.UNKNOWN.value
        else item.name_source,
        name_expr=named.name_expr,
        symbol=current.symbol or item.symbol,
        takes_node_options=current.takes_node_options or item.takes_node_options,
        name_line=named.name_line,
        name_position=named.name_position,
        name_reason=named.name_reason,
        name_default=named.name_default,
    )


def _map_state(value: Any, refused: str) -> str:
    if refused:
        return REFUSED
    return "known" if value is not None else "unknown"


def _finish_param(item: ParamSite, index: CodeIndex) -> ParamSite:
    name = _finish(item.name, index)
    if name.status == SUFFIX:
        item = replace(item, scope="plugin")
    if item.default_state != "unknown" or item.default_ref is None:
        return replace(item, name=name)
    default = _finish(item.default_ref, index)
    if default.status != LITERAL:
        return replace(item, name=name)
    return replace(item, name=name, default=default.value, default_state="known")


def _short_class(name: str) -> str:
    return re.sub(r"<.*", "", name).strip().rpartition("::")[2]


def _plugin_suffixes(units: Sequence[Unit]) -> None:
    """A suffix names a parameter of a plugin instance only in a class its package registers
    with `PLUGINLIB_EXPORT_CLASS`, through an alias when the class is a template, or in a
    helper a registered plugin hands its name. Anywhere else the instance name is not a
    plugin's, so the suffix is a `param_name` blind spot and never a node parameter."""
    aliases: dict[str, dict[str, str]] = {}
    for unit in units:
        for row in unit.type_aliases:
            aliases.setdefault(unit.pkg, {})[_short_class(row["name"])] = _short_class(
                row["target"]
            )
    registered: dict[str, dict[str, set[Registration]]] = {}
    for unit in units:
        for item in unit.registrations:
            if item.macro != "PLUGINLIB_EXPORT_CLASS":
                continue
            table = registered.setdefault(unit.pkg, {})
            short = _short_class(item.type)
            for name in {short, aliases.get(unit.pkg, {}).get(short, short)}:
                table.setdefault(name, set()).add(item)
    for unit in units:
        table = registered.get(unit.pkg, {})
        own = next(
            (item for item in unit.registrations if item.macro == "PLUGINLIB_EXPORT_CLASS"), None
        )
        helper = unit.plugin_of_helper

        def plugin_of(
            owner_class: str,
            helper: Registration | None = helper,
            table: dict[str, set[Registration]] = table,
            own: Registration | None = own,
        ) -> Registration | None:
            if helper is not None:
                return helper
            found = table.get(_short_class(owner_class), set())
            if len(found) == 1:
                return next(iter(found))
            return own if not owner_class else None

        owners = [item.owner_class for item in unit.params if item.name.status == SUFFIX]
        owners += [
            item.sym.rpartition("::")[0] for item in unit.reads if item.name.status == SUFFIX
        ]
        found = {plugin_of(owner) for owner in owners} - {None}
        plugin = own or helper or (next(iter(found)) if len(found) == 1 else None)
        if own is None and helper is None and plugin is not None:
            # the file states the parameters of a plugin another file of the package registers
            unit.plugin_of_helper = plugin
        unit.params = [
            replace(item, scope="node")
            if item.name.status == SUFFIX
            and (plugin is None or plugin_of(item.owner_class) is not plugin)
            else item
            for item in unit.params
        ]
        unit.reads = [
            replace(item, name=Ref(UNRESOLVED, "", item.name.expr))
            if item.name.status == SUFFIX
            and (plugin is None or plugin_of(item.sym.rpartition("::")[0]) is not plugin)
            else item
            for item in unit.reads
        ]


def _plugin_prefix(ref: Ref) -> bool:
    return ref.expr in {"name_", "this->name_"} or ref.origin == PLUGIN_NAME


def _finish(ref: Ref, index: CodeIndex) -> Ref:
    if ref.status != UNRESOLVED or not ref.member:
        return ref
    found = index.members.get((ref.owner, ref.member))
    if found is not None and found.origin == PLUGIN_NAME and ref.parameter_suffix[:1] == ".":
        return Ref(SUFFIX, ref.parameter_suffix[1:], ref.expr, origin="plugin_parameter")
    if found is None or (
        found.status == UNRESOLVED and not (found.origin == "parameter" and found.source_name)
    ):
        return ref
    return replace(
        found,
        value=found.value + ref.parameter_suffix if found.status == LITERAL else found.value,
        expr=ref.expr,
        binding="member",
        parameter_suffix=found.parameter_suffix + ref.parameter_suffix,
    )


# ---------------------------------------------------------------------------
# the per file report


UNRESOLVED_REASONS = {
    "message_field": (["observed", "recorded"], ("->header.", ".header.")),
    "argv_value": (["declared", "observed"], ()),
    "plugin_instance_prefix": (["declared"], ()),
    "runtime_attribute_default": (["observed"], ()),
    "external_descriptor_file": (["observed"], ()),
    "not_stated_in_source": (["declared", "observed"], ()),
    "frame_empty": (["observed"], ()),
}
EMPTY_FRAME = "frame_empty"
_STRING_PIECE = re.compile(r'"[^"]+"')


def _reason(expr: str, want: str, *, loop: bool = False) -> str:
    if want == model.Want.NODE_NAME.value:
        return "argv_value"
    if "header." in expr:
        return "message_field"
    if loop:
        return "external_descriptor_file"
    if want == model.Want.PARAM_VALUE.value:
        return "runtime_attribute_default"
    return "not_stated_in_source"


def _guard_view(guard: tuple[tuple[str, bool], ...]) -> dict[str, Any]:
    if not guard:
        return {}
    return {"guard_parameters": [{"name": name, "value": value} for name, value in guard]}


def _endpoint_view(item: Endpoint) -> dict[str, Any]:
    if item.kind == "timer":
        return {
            "kind": "timer",
            **({"construction": item.construction} if item.construction else {}),
            "raw": item.period_expr,
            "name": None,
            "name_state": "not_applicable",
            "period_s": item.period_s,
            "period_state": item.period_state,
            **({"unresolved_ref": f"rate:{item.period_expr}"} if item.period_reason else {}),
            **({"from_parameter": item.period_from} if item.period_from else {}),
            **({"callback": item.callback} if item.callback else {}),
            **({"period_form": item.period_form} if item.period_form else {}),
            **(
                {"period_numerator": item.period_numerator}
                if item.period_numerator is not None
                else {}
            ),
            **({"condition": item.condition} if item.condition else {}),
            **_guard_view(item.guard),
            "owner_symbol": item.sym,
            "line_hint": item.line,
        }
    state = item.name.state()
    out: dict[str, Any] = {
        "kind": item.kind,
        "raw": item.name.expr,
        "name": item.name.value if item.name.resolved else None,
        "name_state": state,
        "absolute": item.name.absolute if item.name.resolved else None,
    }
    if state == UNRESOLVED:
        out["unresolved_ref"] = f"{_want_of(item.kind)}:{item.name.expr}"
        if item.unknown_count:
            out["count_state"] = "unknown"
    if item.name.source_name:
        key = {
            "resolved_from_plugin_parameter_default": "from_plugin_parameter",
            "resolved_from_constant": "from_constant",
            "resolved_from_macro": "from_macro",
        }.get(state, "from_parameter")
        out[key] = item.name.source_name
        if item.name.binding and key == "from_parameter":
            out["binding"] = item.name.binding
    if item.name.parameter_suffix:
        out["parameter_suffix"] = item.name.parameter_suffix
    if item.name.resolved and not item.name.absolute:
        out["topic_id"] = None
    if item.type_text:
        out["type"] = interface_name(item.type_text)
        parts = interface_parts(item.type_text)
        out["type_id"] = ids.build_type(*parts) if parts else None
    out["qos"] = item.qos
    if item.qos_reason:
        out["qos_reason"] = item.qos_reason
    if item.options_argument:
        out["options_argument"] = item.options_argument
        out["options_position"] = item.options_position
    if item.via_known_wrapper:
        out["via_known_wrapper"] = item.via_known_wrapper
    if item.condition or (item.via_known_wrapper and item.conditional_site):
        out["condition"] = item.condition or None
    out.update(_guard_view(item.guard))
    if item.publisher_kind:
        out["publisher_kind"] = item.publisher_kind
    if item.attribute:
        out["attribute"] = item.attribute
    if item.api in PUBLISH_CALLS and item.lifecycle:
        out["publisher_kind"] = "lifecycle"
    if item.construction:
        out["construction"] = item.construction
    out["owner_symbol"] = item.sym
    out["line_hint"] = item.line
    return out


def _want_of(kind: str) -> str:
    if kind in ("service_server", "service_client"):
        return model.Want.SERVICE_NAME.value
    if kind in ("action_server", "action_client"):
        return model.Want.ACTION_NAME.value
    return model.Want.TOPIC_NAME.value


def _param_view(item: ParamSite, reads: Sequence[ParamRead]) -> dict[str, Any]:
    out: dict[str, Any] = {
        **({"construction": item.construction} if item.construction else {}),
        "name": item.name.value,
        "name_state": "map_key"
        if item.api == "declare_parameters"
        else (LITERAL if item.name.origin in ("", LITERAL) else item.name.state()),
    }
    if item.name.origin == "concat":
        out["name_expr"] = item.name.expr
    out["type"] = item.value_type
    out["has_default"] = item.default_state != "absent"
    out["default"] = item.default if item.default_state == "known" else None
    if item.default_state != "known":
        out["default_state"] = "absent" if item.default_state == "absent" else UNRESOLVED
        if item.default_expr:
            out["default_expr"] = item.default_expr
    out["api"] = item.api
    if item.via_wrapper:
        out["via_wrapper"] = item.via_wrapper
        out["wrapper_kind"] = "user_declare_wrapper"
    if item.map_namespace:
        out["map_namespace"] = item.map_namespace
    out["owner_symbol"] = item.sym
    out["reads"] = [_read_view(read) for read in reads]
    out["line_hint"] = item.line
    return out


def _read_view(read: ParamRead) -> dict[str, Any]:
    if read.accessor:
        return {"api": read.api, "accessor": read.accessor, "line_hint": read.line}
    return {"api": read.api, "target": read.target, "line_hint": read.line}


def _plugin_param_view(
    item: ParamSite, reads: Sequence[ParamRead], base: str, lookup: str
) -> dict[str, Any]:
    read = reads[0] if reads else None
    return {
        "suffix": item.name.value,
        "type": item.value_type,
        "has_default": item.default_state == "known",
        "default": item.default if item.default_state == "known" else None,
        "base_class": base,
        "lookup_name": lookup,
        "pparam_id": ids.build_pparam(base, lookup, item.name.value),
        "declare_api": item.api,
        "declare_spelling": item.spelling,
        "read_spelling": read.spelling if read is not None else None,
        "asymmetric": read is not None and read.spelling != item.spelling,
        "owner_symbol": item.sym,
        "reads": [_read_view(row) for row in reads],
        "line_hint": item.line,
    }


def _frame_ref(ref: Ref) -> Ref:
    """A literal that names no frame (`""`, `"/"`) is a blind spot, not a frame id."""
    if ref.status == LITERAL and not ref.value.strip().lstrip("/"):
        expr = ref.expr or f'"{ref.value}"'
        return replace(ref, status=UNRESOLVED, expr=expr, origin=EMPTY_FRAME)
    return ref


def _frame_view(ref: Ref) -> tuple[str | None, str]:
    """A frame has no relative form, so its state is `literal` or how it was followed."""
    if ref.status != LITERAL:
        return None, UNRESOLVED
    state = ref.state()
    if state.startswith(LITERAL):
        state = LITERAL
    return ids.normalise_frame(ref.value), state


def file_report(
    unit: Unit,
    *,
    workspace_root: str = "",
    lookups: dict[str, tuple[str, str]] | None = None,
    components: dict[str, str] | None = None,
) -> dict[str, Any]:
    """Everything one translation unit says, in one sorted document.

    This is the shape the fixture expectations are written in, so a disagreement between an
    implementation and the contract is one diff and not an argument.
    """
    lookups = lookups or {}
    components = components or {}
    path = unit.path
    if workspace_root:
        try:
            path = str(Path(unit.path).resolve().relative_to(Path(workspace_root).resolve()))
        except ValueError:
            path = unit.path
    file_id = ids.build_file(unit.pkg or ids.REPO_PKG, unit.relpath or Path(unit.path).name)
    unresolved: list[dict[str, Any]] = []

    nodes: list[dict[str, Any]] = []
    seen_classes: set[tuple[str, str]] = set()
    for item in sorted(unit.classes, key=lambda row: (row.qualname, row.construction)):
        # The symbol has to be defined here: a header states the base class and the source
        # states the name, and after the join both files carry the merged class.
        if (
            not item.symbol
            or item.symbol not in unit.methods
            or (item.qualname, item.construction) in seen_classes
            or (item.qualname in unit.constructed_classes and not item.construction)
        ):
            continue
        seen_classes.add((item.qualname, item.construction))
        row: dict[str, Any] = {
            "name": item.stated_name or None,
            "name_source": item.name_source,
            "node_id": None,
            "form": "composable_node"
            if item.qualname in components
            else ("lifecycle_node" if item.form == "lifecycle_node" else "node"),
            "base": item.base,
            "class": item.qualname,
            "symbol": item.symbol,
            "symbol_id": ids.build_sym(
                unit.pkg or ids.REPO_PKG, unit.relpath or Path(unit.path).name, item.symbol
            ),
        }
        if item.construction:
            row["construction"] = item.construction
        if item.construction_ambiguous:
            row["construction_ambiguous"] = True
        if item.takes_node_options:
            row["takes_node_options"] = True
        auto = _stronger(
            unit.auto_declare,
            unit.auto_declare_classes.get(item.qualname.rpartition("::")[2], False),
        )
        if auto:
            row["auto_declare"] = auto
        row.update(
            _global_arguments(
                _stronger(
                    unit.local_arguments,
                    unit.local_arguments_classes.get(item.qualname.rpartition("::")[2], False),
                )
            )
        )
        if not item.stated_name:
            row["unresolved_ref"] = f"node_name:{item.name_expr}"
            unresolved.append(
                {
                    "want": model.Want.NODE_NAME.value,
                    "expr": item.name_expr,
                    "reason": item.name_reason or "argv_value",
                    "resolvable_by": UNRESOLVED_REASONS["argv_value"][0],
                    **({"construction": item.construction} if item.construction else {}),
                    "owner_symbol": item.symbol,
                    "line_hint": item.name_line,
                }
            )
        row["line_hint"] = item.name_line
        nodes.append(row)
    for item in sorted(unit.plain_nodes, key=lambda row: (row.symbol, row.name_line or 0)):
        row = {
            "name": item.stated_name or None,
            "name_source": item.name_source,
            "node_id": None,
            "form": item.form,
            "base": item.base,
            "class": item.qualname,
            "symbol": item.symbol,
            "symbol_id": ids.build_sym(
                unit.pkg or ids.REPO_PKG, unit.relpath or Path(unit.path).name, item.symbol
            ),
            "line_hint": item.name_line,
        }
        if not item.stated_name:
            row["unresolved_ref"] = f"node_name:{item.name_expr}"
            unresolved.append(
                {
                    "want": model.Want.NODE_NAME.value,
                    "expr": item.name_expr,
                    "reason": item.name_reason,
                    "resolvable_by": UNRESOLVED_REASONS[item.name_reason][0],
                    "owner_symbol": item.symbol,
                    "line_hint": item.name_line,
                }
            )
        nodes.append(row)

    endpoints = [_endpoint_view(item) for item in unit.endpoints]
    for item, row in zip(unit.endpoints, endpoints, strict=True):
        if item.attribute:
            row["published_from"] = sorted(
                unit.publishing.get((item.owner_class, item.attribute), ())
            )
    endpoints.sort(
        key=lambda row: (
            row["kind"],
            row["name"] is None,
            row["name"] or "",
            row["raw"],
            row.get("construction", ""),
        )
    )
    for item in unit.endpoints:
        if item.kind == "timer":
            if item.period_reason:
                unresolved.append(
                    {
                        "want": model.Want.RATE.value,
                        "expr": item.period_expr,
                        "reason": item.period_reason,
                        "resolvable_by": ["observed"],
                        **({"construction": item.construction} if item.construction else {}),
                        "owner_symbol": item.sym,
                        "line_hint": item.line,
                    }
                )
            continue
        if item.name.resolved:
            continue
        unresolved.append(
            {
                "want": _want_of(item.kind),
                "expr": item.name.expr,
                "reason": _reason(
                    item.name.expr, _want_of(item.kind), loop=bool(item.unknown_count)
                ),
                "resolvable_by": ["observed"],
                **({"construction": item.construction} if item.construction else {}),
                "owner_symbol": item.sym,
                "line_hint": item.line,
            }
        )

    reads_by_name: dict[tuple[str, str], list[ParamRead]] = {}
    for read in unit.reads:
        if read.name.resolved:
            reads_by_name.setdefault((read.construction, read.name.value), []).append(read)

    parameters: list[dict[str, Any]] = []
    plugin_parameters: list[dict[str, Any]] = []
    plugin = next(
        (item for item in unit.registrations if item.macro == "PLUGINLIB_EXPORT_CLASS"),
        unit.plugin_of_helper,
    )
    base_class = plugin.base_class if plugin else ""
    lookup_name = lookups.get(plugin.type, (base_class, plugin.type))[1] if plugin else ""
    for item in sorted(unit.params, key=lambda row: (row.scope, row.name.value, row.construction)):
        reads = sorted(
            reads_by_name.get((item.construction, item.name.value), []), key=lambda row: row.line
        )
        if not item.name.resolved:
            unresolved.append(
                {
                    "want": model.Want.PARAM_NAME.value,
                    # a local holding `prefix + ".value"` builds only names with that tail
                    "expr": (
                        item.name.built if _STRING_PIECE.search(item.name.built) else item.spelling
                    ),
                    "reason": "not_stated_in_source",
                    "resolvable_by": ["observed"],
                    **({"construction": item.construction} if item.construction else {}),
                    "owner_symbol": item.sym,
                    "line_hint": item.line,
                }
            )
            if re.fullmatch(
                r"(?:::)?[A-Za-z_]\w*::(?:msg|srv|action)::[A-Za-z_]\w*::[A-Za-z_]\w*",
                item.spelling,
            ):
                row = _param_view(item, reads)
                row.update(name=None, name_expr=item.spelling, name_state=UNRESOLVED)
                parameters.append(row)
            continue
        if item.scope == "plugin" and plugin is not None:
            plugin_parameters.append(_plugin_param_view(item, reads, base_class, lookup_name))
        elif item.name.status == SUFFIX:
            # a suffix of an instance name no registration of the package owns
            unresolved.append(
                {
                    "want": model.Want.PARAM_NAME.value,
                    "expr": item.spelling,
                    "reason": "plugin_instance_prefix",
                    "resolvable_by": ["observed"],
                    **({"construction": item.construction} if item.construction else {}),
                    "owner_symbol": item.sym,
                    "line_hint": item.line,
                }
            )
            continue
        else:
            parameters.append(_param_view(item, reads))
        if item.default_state == "unknown":
            local = re.fullmatch(r"[A-Za-z_]\w*", item.default_expr or "") and not (
                item.default_expr.endswith("_")
            )
            unresolved.append(
                {
                    "want": model.Want.PARAM_VALUE.value,
                    "expr": item.default_expr,
                    "reason": "local_variable" if local else "runtime_attribute_default",
                    "resolvable_by": ["observed"],
                    **({"construction": item.construction} if item.construction else {}),
                    "owner_symbol": item.sym,
                    "line_hint": item.line,
                }
            )
        elif item.default_state == REFUSED:
            unresolved.append(
                {
                    "want": model.Want.PARAM_VALUE.value,
                    "expr": item.default_expr or item.spelling,
                    "reason": item.default_reason,
                    "resolvable_by": ["observed"],
                    **({"construction": item.construction} if item.construction else {}),
                    "owner_symbol": item.sym,
                    "line_hint": item.line,
                }
            )
    parameters.sort(key=lambda row: (row["name"] is None, row["name"] or row.get("name_expr", "")))
    plugin_parameters.sort(key=lambda row: row["suffix"])

    prefix_sites = [
        read
        for read in unit.reads
        if read.name.status == SUFFIX and read.spelling.startswith("name_")
    ]
    if prefix_sites:
        unresolved.append(
            {
                "want": model.Want.OTHER.value,
                "expr": 'name_ + "."',
                "reason": "plugin_instance_prefix",
                "resolvable_by": ["declared"],
                "owner_symbol": prefix_sites[0].sym,
                "line_hint": prefix_sites[0].line,
            }
        )

    broadcasts: list[dict[str, Any]] = []
    for item in unit.broadcasts:
        item = replace(item, parent=_frame_ref(item.parent), child=_frame_ref(item.child))
        parent, parent_state = _frame_view(item.parent)
        child, child_state = _frame_view(item.child)
        row = {
            "kind": item.kind,
            "parent": parent,
            "child": child,
            "parent_state": parent_state,
            "child_state": child_state,
            "broadcaster": item.broadcaster,
            "condition": item.condition or None,
            **_guard_view(item.guard),
            **({"construction": item.construction} if item.construction else {}),
            "owner_symbol": item.sym,
            "frame_ids": [ids.build_frame(name) for name in (parent, child) if name],
            "line_hint": item.line,
        }
        if item.parent.source_name:
            row["parent_from_parameter"] = item.parent.source_name
        if item.child.source_name:
            row["child_from_parameter"] = item.child.source_name
        for role, ref in (("parent", item.parent), ("child", item.child)):
            if ref.origin == "parameter":
                row[f"{role}_expr"] = ref.expr
                if ref.parameter_suffix:
                    row[f"{role}_parameter_suffix"] = ref.parameter_suffix
        broadcasts.append(row)
        for ref in (item.parent, item.child):
            if ref.status != LITERAL:
                unresolved.append(_frame_unresolved(ref, item.sym, item.line, item.construction))
    broadcasts.sort(
        key=lambda row: (
            row["parent"] is None,
            row["parent"] or "",
            row["child"] is None,
            row["child"] or "",
            row.get("construction", ""),
        )
    )

    lookup_rows: list[dict[str, Any]] = []
    for item in unit.lookups:
        item = replace(item, target=_frame_ref(item.target), source=_frame_ref(item.source))
        target, target_state = _frame_view(item.target)
        source, source_state = _frame_view(item.source)
        row = {
            "api": item.api,
            "target": target,
            "source": source,
            "target_state": target_state,
            "source_state": source_state,
            **({"construction": item.construction} if item.construction else {}),
            "owner_symbol": item.sym,
            "line_hint": item.line,
        }
        if item.target.source_name:
            row["target_from_parameter"] = item.target.source_name
        if item.source.source_name:
            row["source_from_parameter"] = item.source.source_name
        for role, ref in (("target", item.target), ("source", item.source)):
            if ref.origin == "parameter":
                row[f"{role}_expr"] = ref.expr
                if ref.parameter_suffix:
                    row[f"{role}_parameter_suffix"] = ref.parameter_suffix
        for ref, label in ((item.target, "target"), (item.source, "source")):
            if ref.status != LITERAL:
                row["unresolved_ref"] = f"frame_id:{ref.expr}"
                unresolved.append(_frame_unresolved(ref, item.sym, item.line, item.construction))
                del label
        lookup_rows.append(row)
    lookup_rows.sort(
        key=lambda row: (
            row["api"],
            row["target"] is None,
            row["target"] or "",
            row["source"] is None,
            row["source"] or "",
            row.get("construction", ""),
        )
    )

    report: dict[str, Any] = {
        "package": unit.pkg,
        "path": path,
        "package_relative_path": unit.relpath,
        "language": "cpp",
        "file_id": file_id,
        "nodes": nodes,
    }
    if unit.constructions:
        report["constructions"] = _constructions(unit, unit.constructions)
    if unit.type_aliases:
        report["type_aliases"] = unit.type_aliases
    if plugin is not None and plugin is not unit.plugin_of_helper:
        report["plugins"] = [
            {
                "macro": plugin.macro,
                "macro_wrapped_lines": plugin.wrapped,
                "type": plugin.type,
                "base_class": base_class,
                "lookup_name": lookup_name,
                "plugin_id": ids.build_plugin(base_class, lookup_name),
                "line_hint": plugin.line,
            }
        ]
    registered = [
        item for item in unit.registrations if item.macro == "RCLCPP_COMPONENTS_REGISTER_NODE"
    ]
    if registered:
        report["components"] = [
            {
                "macro": item.macro,
                "class": item.type,
                "registered_executable": components.get(item.type),
                "has_main": "main" in unit.methods,
                "line_hint": item.line,
            }
            for item in registered
        ]
    report["endpoints"] = endpoints
    report["parameters"] = parameters
    declared = {(item.construction, item.name.value) for item in unit.params if item.name.resolved}
    report["parameter_reads"] = sorted(
        (
            {
                "name": read.name.value,
                **_read_view(read),
                **({"construction": read.construction} if read.construction else {}),
                "owner_symbol": read.sym,
            }
            for read in unit.reads
            if read.name.resolved
            and read.name.status != SUFFIX
            and (read.construction, read.name.value) not in declared
        ),
        key=lambda row: (row["name"], row["owner_symbol"], row["line_hint"]),
    )
    report["parameter_callbacks"] = [
        {
            "api": item.api,
            "form": item.form,
            "owner_symbol": item.sym,
            "handler": item.handler,
            "line_hint": item.line,
        }
        for item in sorted(unit.callbacks, key=lambda row: (row.api, row.line))
    ]
    report["plugin_parameters"] = plugin_parameters
    lifecycle = _lifecycle(unit)
    if lifecycle:
        report["lifecycle"] = lifecycle
    wrappers = _wrapper_view(unit)
    if wrappers:
        report["wrappers"] = wrappers
    report["tf"] = {"broadcasts": broadcasts, "lookups": lookup_rows}
    seen: set[tuple[str, str, str, str]] = set()
    unique: list[dict[str, Any]] = []
    for row in sorted(
        unresolved,
        key=lambda item: (
            item["want"],
            item["expr"],
            item.get("construction", ""),
            item.get("owner_symbol") or "",
        ),
    ):
        key = (
            row["want"],
            row["expr"],
            row.get("construction", ""),
            row.get("owner_symbol") or "",
        )
        if key in seen:
            continue
        seen.add(key)
        unique.append(row)
    report["unresolved"] = unique
    report["counts"] = _counts(unit, endpoints, parameters, plugin_parameters, unique)
    if not unit.parsed:
        report["parse"] = {"parsed": False, "reason": unit.reason}
    return report


def _frame_unresolved(ref: Ref, sym: str, line: int, construction: str = "") -> dict[str, Any]:
    reason = (
        EMPTY_FRAME if ref.origin == EMPTY_FRAME else _reason(ref.expr, model.Want.FRAME_ID.value)
    )
    return {
        "want": model.Want.FRAME_ID.value,
        "expr": ref.expr,
        "reason": reason,
        "resolvable_by": UNRESOLVED_REASONS[reason][0],
        **({"construction": construction} if construction else {}),
        "owner_symbol": sym,
        "line_hint": line,
    }


def _lifecycle(unit: Unit) -> dict[str, Any]:
    transitions = sorted(
        {
            item.rpartition("::")[2]
            for item in unit.methods
            if item.rpartition("::")[2] in _TRANSITIONS
        }
    )
    if not transitions:
        return {}
    created: dict[str, list[str]] = {}
    for item in unit.endpoints:
        where = item.sym.rpartition("::")[2]
        if where not in transitions:
            continue
        created.setdefault(where, []).append(
            "timer" if item.kind == "timer" else (item.name.value or item.name.expr)
        )
    for value in created.values():
        value.sort()
    empty = [item for item in unit.empty_bodies if item.rpartition("::")[2] == _class_tail(item)]
    return {
        "transitions_implemented": transitions,
        "endpoints_created_in": created,
        "constructor_body_empty": bool(empty),
    }


def _class_tail(qualname: str) -> str:
    parts = qualname.split("::")
    return parts[-2] if len(parts) > 1 else ""


def _wrapper_view(unit: Unit) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for name, (index, _default) in sorted(unit.wrappers.items()):
        del index
        out.append(
            {
                "symbol": name,
                "kind": "user_declare_wrapper",
                "line_hint": 0,
            }
        )
    grouped: dict[str, dict[str, Any]] = {}
    for item in unit.wrapper_uses:
        if item.symbol.startswith("tf2_ros::"):
            continue
        row = grouped.setdefault(item.symbol, {"symbol": item.symbol, "kind": item.kind})
        row.setdefault("line_hint", item.line)
    for item in unit.endpoints:
        if not item.via_known_wrapper or item.via_known_wrapper.startswith("tf2_ros::"):
            continue
        row = grouped.setdefault(
            item.via_known_wrapper, {"symbol": item.via_known_wrapper, "kind": "known_wrapper"}
        )
        row.setdefault("line_hint", item.line)
        key = "publishes" if item.kind == "publisher" else "subscribes"
        row.setdefault(key, [])
        if item.name.value not in row[key]:
            row[key].append(item.name.value)
    for row in grouped.values():
        for key in ("publishes", "subscribes"):
            if key in row:
                row[key] = sorted(row[key])
    out.extend(grouped[key] for key in sorted(grouped))
    return out


def _counts(
    unit: Unit,
    endpoints: Sequence[dict[str, Any]],
    parameters: Sequence[dict[str, Any]],
    plugin_parameters: Sequence[dict[str, Any]],
    unresolved: Sequence[dict[str, Any]],
) -> dict[str, int]:
    out = {
        "endpoints": len([row for row in endpoints if row["kind"] != "timer"]),
        "publishers": len([row for row in endpoints if row["kind"] == "publisher"]),
        "subscribers": len([row for row in endpoints if row["kind"] == "subscriber"]),
        "service_servers": len([row for row in endpoints if row["kind"] == "service_server"]),
        "service_clients": len([row for row in endpoints if row["kind"] == "service_client"]),
        "timers": len([row for row in endpoints if row["kind"] == "timer"]),
        "parameters": len(parameters),
        "unresolved": len(unresolved),
    }
    out["endpoints"] = len(endpoints)
    if plugin_parameters:
        out["plugin_parameters"] = len(plugin_parameters)
    if unit.broadcasts:
        out["tf_broadcasts"] = len(unit.broadcasts)
    if unit.lookups:
        out["tf_lookups"] = len(unit.lookups)
    wrapped = len([row for row in endpoints if row.get("via_known_wrapper")])
    if wrapped:
        out["via_known_wrapper"] = wrapped
    return out


# ---------------------------------------------------------------------------
# the collector


def source_paths(ws: Any) -> list[str]:
    """The C++ this layer reads: the editable tree only.

    The install tree holds generated rosidl headers for every interface package, hundreds of
    files that define no node and no endpoint and are written in preprocessor heavy C the
    grammar cannot always follow. Reading them would report parse failures for content
    nobody edits, and every fact would still have to cite the src twin.
    """
    return [path for path in ws.source_files(SUFFIXES) if ws.root_of(path) == model.Root.SRC.value]


def package_tables(ws: Any) -> tuple[dict[str, tuple[str, str]], dict[str, str]]:
    """(C++ type to (base class, pluginlib lookup name), C++ type to component executable).

    Both come from the one implementation that owns manifests and CMake, because a lookup
    name that differs from the type and an executable that exists only in an `EXECUTABLE`
    clause are exactly the two joins a second reader would get wrong.
    """
    from rosio.collect import plugins

    lookups: dict[str, tuple[str, str]] = {}
    components: dict[str, str] = {}
    for name in sorted(ws.packages):
        try:
            build = plugins.read_package(ws, name)
        except Exception:
            continue
        for manifest in build.manifests:
            for item in manifest.classes:
                lookups.setdefault(item.type, (item.base_class, item.lookup_name))
        for component in build.components:
            if component.executable:
                components.setdefault(component.class_name, component.executable)
    return lookups, components


@dataclass
class _Pending:
    """Every code side statement about one plugin parameter, gathered before it is written.

    One parameter is declared and read from several symbols, so the chain and the fact are
    assembled once. Writing them per site would either collide on the single `(coded, *)`
    fact slot or hand one profile two candidates at rank 0.
    """

    declares: list[tuple[model.Source, ParamSite, str]] = field(default_factory=list)
    reads: list[str] = field(default_factory=list)


class _Overlay(dict):  # type: ignore[type-arg]
    """The entries of `base`, each copied on first touch, so a file the atlas refuses leaves
    `base` as it was."""

    def __init__(self, base: dict[str, Any]) -> None:
        super().__init__()
        self.base = base

    def get(self, key: str, default: Any = None) -> Any:
        if not dict.__contains__(self, key):
            if key not in self.base:
                return default
            dict.__setitem__(self, key, copy.deepcopy(self.base[key]))
        return dict.__getitem__(self, key)

    def __contains__(self, key: object) -> bool:
        return dict.__contains__(self, key) or key in self.base

    def __getitem__(self, key: str) -> Any:
        if key not in self:
            raise KeyError(key)
        return self.get(key)


def _scratch(builder: model.AtlasBuilder) -> model.AtlasBuilder:
    """A builder over one translation unit, written into `builder` only once it is whole."""
    scratch = model.AtlasBuilder()
    scratch.atlas.entities = _Overlay(builder.atlas.entities)
    scratch.atlas.edges = _Overlay(builder.atlas.edges)
    return scratch


def _commit(builder: model.AtlasBuilder, scratch: model.AtlasBuilder) -> None:
    builder.atlas.entities.update(dict.items(scratch.atlas.entities))
    builder.atlas.edges.update(dict.items(scratch.atlas.edges))
    for entry in scratch.atlas.unresolved:
        builder.unresolved(entry)
    for want, count in sorted(scratch._resolved_counts.items()):
        builder.count_resolved(want, count)


@dataclass
class CppCodeCollector:
    name: str = NAME
    layer: str = LAYER
    pending: dict[str, _Pending] = field(default_factory=dict)

    def inputs_present(self, ctx: CollectContext) -> bool | str:
        """Whether there is any C++ here to read.

        Asked before the pass is attempted, so a workspace with no C++ on a machine without
        the optional extra is recorded as not attempted rather than as a pass that stopped
        with data still to read. An installed ROS tree is exactly that workspace.
        """
        return True if source_paths(ctx.ws) else NO_SOURCE

    def collect(self, ctx: CollectContext) -> None:
        ws = ctx.ws
        builder = ctx.builder
        self.pending = {}
        sources = source_paths(ws)
        if not sources:
            return  # nothing to read, which `inputs_present` states to the orchestrator
        parser = load_parser()
        units: list[Unit] = []
        problems: list[tuple[str, str, str]] = []
        for path in sources:
            package, relpath = ws.rel(path)
            try:
                units.append(
                    parse_file(path, pkg=package or ids.REPO_PKG, relpath=relpath, parser=parser)
                )
            except CollectorError as exc:
                problems.append((path, CollectorError.__name__, str(exc)))
            except Exception as exc:
                # One unreadable file must never cost the layer every other file. A
                # header nested deep enough to exhaust the interpreter stack is the case
                # that reaches here, and it is a property of that file.
                problems.append(
                    (
                        path,
                        type(exc).__name__,
                        f"{LAYER}.{NAME}: reading this file raised {type(exc).__name__}: {exc}",
                    )
                )
        units = share_wrappers(units, parser)
        from rosio.collect import plugins

        include_dirs = {}
        for name in sorted({unit.pkg for unit in units} & ws.packages.keys()):
            base = ws.packages[name].src_path
            if base and (Path(base) / "CMakeLists.txt").is_file():
                path = str(Path(base) / "CMakeLists.txt")
                try:
                    scan = plugins.scan_cmake(plugins._read(ws, path), pkg=name, base=str(base))
                    include_dirs[name] = list(scan.include_dirs)
                    source_dirs: dict[str, set[tuple[str, ...]]] = {}
                    for target in scan.targets.values():
                        for source in target.sources:
                            source_dirs.setdefault(str((Path(base) / source).resolve()), set()).add(
                                target.include_dirs
                            )
                    for unit in units:
                        if unit.pkg != name:
                            continue
                        file_id = ids.build_file(name, unit.relpath)
                        choices = source_dirs.get(str(Path(unit.path).resolve()), set())
                        if len(choices) == 1:
                            include_dirs[file_id] = list(next(iter(choices)))
                        elif len(choices) > 1:
                            unit.includes.clear()
                except CollectorError as exc:
                    problems.append((path, CollectorError.__name__, str(exc)))
        analyse(units, include_dirs=include_dirs)
        lookups, components = package_tables(ws)
        run = builder.add_run(
            LAYER,
            input_digest=ids.digest(
                NAME,
                *sorted(ws.file_id(unit.path) for unit in units),
                *[f"{key}={value}" for key, value in sorted(ws.root_hashes().items())],
            ),
            roots=ws.root_hashes(),
            exceptions=[
                {
                    "path": ws.file_id(path),
                    "phase": "describe",
                    "exc_type": exc_type,
                    "message": message.replace(path, ws.file_id(path)),
                }
                for path, exc_type, message in sorted(problems)
            ]
            + ws.undecodable(),
        )
        workspace.record_undecodable(builder, ws)
        builder.declare_layer(LAYER, present=True, run=run)
        late = builder.atlas.entities[run].attrs["exceptions"]
        for unit in sorted(units, key=lambda item: item.path):
            held, self.pending = self.pending, {}
            scratch = _scratch(builder)
            ctx.builder = scratch
            try:
                self._emit(ctx, run, unit, lookups, components)
            except ContractError as exc:
                ctx.builder, self.pending = builder, held
                self._refused(ctx, unit.path, str(exc), late)
                continue
            finally:
                ctx.builder = builder
            _commit(builder, scratch)
            for param_id, row in self.pending.items():
                into = held.setdefault(param_id, _Pending())
                into.declares.extend(row.declares)
                into.reads.extend(row.reads)
            self.pending = held
        self._flush(ctx, run)
        late.sort(key=lambda row: (str(row.get("path", "")), str(row.get("phase", ""))))

    def _refused(
        self, ctx: CollectContext, path: str, message: str, late: list[dict[str, Any]]
    ) -> None:
        """One translation unit the atlas refused, recorded on that file and nowhere else.

        Letting it out of the loop ended the C++ pass, and the atlas that came out carried
        no finding, a resolution rate of 1.000 and a green error gate over the loss.
        """
        ws = ctx.ws
        file_id = ws.file_id(path)
        ctx.builder.node(
            file_id,
            attrs={
                "language": "cpp",
                "loadable": False,
                "refused_because": "cpp_refused_by_atlas",
                "refused_detail": message.replace(path, file_id),
            },
        )
        package, _relpath = ws.rel(path)
        if package and package != ids.REPO_PKG:
            ctx.builder.node(ids.build_pkg(package))
            ctx.builder.edge(model.EdgeKind.CONTAINS.value, ids.build_pkg(package), file_id)
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
                reason="cpp_refused_by_atlas",
                role="cpp_source",
                resolvable_by=["the value this file states has to be one json can hold"],
                evidence=[ws.source(ws.site(path))],
            )
        )

    # emission ------------------------------------------------------------

    def _emit(
        self,
        ctx: CollectContext,
        run: str,
        unit: Unit,
        lookups: dict[str, tuple[str, str]],
        components: dict[str, str],
    ) -> None:
        ws = ctx.ws
        builder = ctx.builder
        report = file_report(unit, lookups=lookups, components=components)
        file_id = ws.file_id(unit.path)
        literals = name_literals(unit.source)
        stamped = stamped_frames(unit.source)
        builder.node(file_id, attrs={"language": "cpp", "loadable": unit.parsed})
        if unit.pkg and unit.pkg != ids.REPO_PKG:
            pkg_id = ids.build_pkg(unit.pkg)
            builder.node(pkg_id)
            builder.edge(model.EdgeKind.CONTAINS.value, pkg_id, file_id)
        builder.fact(
            file_id,
            LAYER,
            run=run,
            values={
                "language": "cpp",
                "parsed": unit.parsed,
                "nodes": report["nodes"],
                "counts": report["counts"],
                "registrations": [
                    {"macro": item.macro, "type": item.type, "base_class": item.base_class}
                    for item in sorted(unit.registrations, key=lambda row: (row.macro, row.type))
                ],
                "wrappers": report.get("wrappers", []),
                "type_aliases": unit.type_aliases,
                **({"name_literals": literals} if literals else {}),
                **({"stamped_frames": stamped} if stamped else {}),
                **(
                    {"plugin_loaders": [{"base_class": base} for base in unit.plugin_loaders]}
                    if unit.plugin_loaders
                    else {}
                ),
                "classes": [
                    {
                        "name": name,
                        **({"bases": unit.class_bases[name]} if name in unit.class_bases else {}),
                        **(
                            {"node_helpers": sorted(unit.class_helpers[name])}
                            if name in unit.class_helpers
                            else {}
                        ),
                        **(
                            {"node_handoffs": sorted(unit.node_handoffs[name])}
                            if name in unit.node_handoffs
                            else {}
                        ),
                        **(
                            {"node_plugin_parameters": sorted(unit.class_node_plugins[name])}
                            if name in unit.class_node_plugins
                            else {}
                        ),
                    }
                    for name in sorted(set(unit.class_names) | set(unit.node_handoffs))
                ],
                "class_templates": [
                    {
                        "name": name,
                        "definitions": unit.template_definitions.get(name, 0),
                        "specialized": name in unit.template_specializations,
                    }
                    for name in sorted(
                        unit.template_definitions.keys() | unit.template_specializations
                    )
                ],
            },
            confidence=model.Confidence.CERTAIN.value,
            evidence=[ws.source(ws.site(unit.path))],
        )
        if unit.hidden_regions:
            # Only a region that holds a ROS call loses anything. The rest are macros the
            # grammar cannot expand, and the code around them is read.
            builder.unresolved(
                model.Unresolved(
                    want=model.Want.OTHER.value,
                    owner=file_id,
                    expr=f"the grammar refused {unit.hidden_regions} region(s) holding ROS calls",
                    reason="cpp_parse_error",
                    role="cpp_source",
                    resolvable_by=["the region needs the preprocessor to be readable"],
                    evidence=[ws.source(ws.site(unit.path, "parse"))],
                )
            )
        self._count(builder, report)
        plugin_id = self._plugin(ctx, run, unit, report, file_id)
        rows = report.get("plugin_parameters") or []
        if not plugin_id and rows:
            plugin_id = ids.build_plugin(rows[0]["base_class"], rows[0]["lookup_name"])
            builder.node(plugin_id)
        for qualname in unit.symbols:
            self._symbol(ctx, run, unit, report, qualname, file_id, plugin_id)
        for entry in report["unresolved"]:
            owner = self._sym_id(unit, entry.get("owner_symbol") or "") or file_id
            builder.unresolved(
                model.Unresolved(
                    want=entry["want"],
                    owner=owner,
                    expr=entry["expr"],
                    reason=entry["reason"],
                    role="cpp_source",
                    construction=str(entry.get("construction") or ""),
                    resolvable_by=list(entry.get("resolvable_by") or []),
                    evidence=[ws.source(ws.site(unit.path, entry["reason"], entry["line_hint"]))],
                )
            )

    def _count(self, builder: model.AtlasBuilder, report: dict[str, Any]) -> None:
        """The denominator of every rate, taken once from the report."""
        for row in report["nodes"]:
            if row["name"]:
                builder.count_resolved(model.Want.NODE_NAME.value)
        for row in report["endpoints"]:
            if row["kind"] == "timer":
                if row["period_s"] is not None:
                    builder.count_resolved(model.Want.RATE.value)
                continue
            if row["name"] is not None:
                builder.count_resolved(_want_of(row["kind"]))
        for row in [*report["parameters"], *report["plugin_parameters"]]:
            if row.get("name") is not None or row.get("suffix"):
                builder.count_resolved(model.Want.PARAM_NAME.value)
            if row["has_default"] and row.get("default_state") != UNRESOLVED:
                builder.count_resolved(model.Want.PARAM_VALUE.value)
        for row in report["tf"]["broadcasts"]:
            for key in ("parent", "child"):
                if row[key]:
                    builder.count_resolved(model.Want.FRAME_ID.value)
        for row in report["tf"]["lookups"]:
            for key in ("target", "source"):
                if row[key]:
                    builder.count_resolved(model.Want.FRAME_ID.value)

    def _sym_id(self, unit: Unit, qualname: str) -> str:
        if not qualname:
            return ""
        return ids.build_sym(unit.pkg or ids.REPO_PKG, unit.relpath, qualname)

    def _plugin(
        self, ctx: CollectContext, run: str, unit: Unit, report: dict[str, Any], file_id: str
    ) -> str:
        rows = report.get("plugins") or []
        if not rows:
            return ""
        row = rows[0]
        plugin_id = str(row["plugin_id"])
        ctx.builder.node(plugin_id)
        ctx.builder.fact(
            plugin_id,
            LAYER,
            run=run,
            values={
                "type": row["type"],
                "base_class": row["base_class"],
                "macro": row["macro"],
                "macro_wrapped_lines": row["macro_wrapped_lines"],
            },
            confidence=model.Confidence.CERTAIN.value,
            evidence=[ctx.ws.source(ctx.ws.site(unit.path, row["macro"], row["line_hint"]))],
        )
        ctx.builder.edge(model.EdgeKind.DEFINES.value, file_id, plugin_id)
        return plugin_id

    def _symbol(
        self,
        ctx: CollectContext,
        run: str,
        unit: Unit,
        report: dict[str, Any],
        qualname: str,
        file_id: str,
        plugin_id: str,
    ) -> None:
        ws = ctx.ws
        builder = ctx.builder
        sym_id = self._sym_id(unit, qualname)
        builder.node(sym_id, attrs={"language": "cpp"})
        builder.edge(model.EdgeKind.DEFINES.value, file_id, sym_id)

        def slice_of(rows: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
            return [row for row in rows if row.get("owner_symbol") == qualname]

        evidence = [ws.source(ws.site(unit.path, qualname))]
        for parameter in unit.params:
            if parameter.sym == qualname and parameter.default_site is not None:
                path, locator, line = parameter.default_site
                builder.node(ws.file_id(path))
                evidence.append(ws.source(ws.site(path, locator, line)))
        builder.fact(
            sym_id,
            LAYER,
            run=run,
            values={
                "nodes": [row for row in report["nodes"] if row["symbol"] == qualname],
                "constructions": _constructions(unit, slice_of(unit.constructions)),
                "endpoints": slice_of(report["endpoints"]),
                "parameters": slice_of(report["parameters"]),
                "plugin_parameters": slice_of(report["plugin_parameters"]),
                "parameter_reads": slice_of(report["parameter_reads"]),
                "tf_broadcasts": slice_of(report["tf"]["broadcasts"]),
                "tf_lookups": slice_of(report["tf"]["lookups"]),
            },
            confidence=model.Confidence.CERTAIN.value,
            evidence=evidence,
        )
        for row in slice_of(report["endpoints"]):
            self._endpoint(ctx, run, unit, row, sym_id)
        for row in slice_of(report["tf"]["broadcasts"]):
            self._broadcast(ctx, run, unit, row, sym_id)
        for row in slice_of(report["tf"]["lookups"]):
            for role in ("target", "source"):
                if row[role] and not row.get(f"{role}_from_parameter"):
                    builder.node(ids.build_frame(row[role]))
        for item in sorted(
            unit.params, key=lambda row: (row.scope, row.name.value, row.construction)
        ):
            if item.sym != qualname or item.scope != "plugin" or not plugin_id:
                continue
            self._plugin_param(ctx, run, unit, item, sym_id, plugin_id)
        for item in unit.reads:
            if item.sym != qualname or item.name.status != SUFFIX or not plugin_id:
                continue
            self._plugin_read(ctx, run, unit, item, sym_id, plugin_id)

    def _endpoint(
        self, ctx: CollectContext, run: str, unit: Unit, row: dict[str, Any], sym_id: str
    ) -> None:
        ws = ctx.ws
        builder = ctx.builder
        if row["kind"] == "timer" or row["name"] is None:
            return
        if not row["absolute"]:
            return
        build = (
            ids.build_action
            if row["kind"].startswith("action")
            else ids.build_service
            if row["kind"].startswith("service")
            else ids.build_topic
        )
        target = build(row["name"])
        builder.node(target)
        type_id = row.get("type_id")
        if type_id:
            builder.node(type_id)
            builder.edge(model.EdgeKind.USES_TYPE.value, target, type_id)
            builder.count_resolved(model.Want.TYPE.value)
        del ws, unit, run, sym_id

    def _broadcast(
        self, ctx: CollectContext, run: str, unit: Unit, row: dict[str, Any], sym_id: str
    ) -> None:
        builder = ctx.builder
        ws = ctx.ws
        # A frame read from a parameter is the code default, which a profile may replace, so
        # only the join states it, once per profile.
        if row.get("parent_from_parameter") or row.get("child_from_parameter"):
            return
        for name in (row["parent"], row["child"]):
            if name:
                builder.node(ids.build_frame(name))
        if not row["parent"] or not row["child"]:
            return
        parent, child = ids.build_frame(row["parent"]), ids.build_frame(row["child"])
        values = {
            "source": "cpp_broadcast",
            "symbol": sym_id,
            "broadcaster": row["broadcaster"],
            "condition": row["condition"],
        }
        # Two sites of one symbol stating the same content fold into one edge, as
        # `join._site_edge` folds them, and every other site has a key of its own.
        key = ids.edge_key(
            sym=sym_id,
            pair=f"{row['parent']}>{row['child']}",
            content=ids.digest_fields({name: str(value) for name, value in values.items()}),
        )
        builder.edge(
            model.EdgeKind.TF_PARENT.value,
            child,
            parent,
            key=key,
            layer=LAYER,
            run=run,
            values=values,
            confidence=model.Confidence.CERTAIN.value,
            evidence=[ws.source(ws.site(unit.path, row["owner_symbol"], row["line_hint"]))],
            fold=True,
        )

    def _plugin_param(
        self,
        ctx: CollectContext,
        run: str,
        unit: Unit,
        item: ParamSite,
        sym_id: str,
        plugin_id: str,
    ) -> None:
        ws = ctx.ws
        builder = ctx.builder
        plugin = ids.parse_plugin(plugin_id)
        param_id = ids.build_pparam(plugin.base_class, plugin.lookup, item.name.value)
        builder.node(param_id)
        site = ws.site(unit.path, f"{item.sym}:{item.api}", item.line)
        self.pending.setdefault(param_id, _Pending()).declares.append(
            (ws.source(site), item, sym_id)
        )
        builder.edge(
            model.EdgeKind.DECLARES_PARAM.value,
            plugin_id,
            param_id,
            key=ids.edge_key(sym=sym_id, api=item.api, name=item.name.value),
            layer=LAYER,
            run=run,
            values={"api": item.api, "symbol": sym_id, "spelling": item.spelling},
            confidence=model.Confidence.CERTAIN.value,
            evidence=[ws.source(site)],
        )

    def _plugin_read(
        self,
        ctx: CollectContext,
        run: str,
        unit: Unit,
        item: ParamRead,
        sym_id: str,
        plugin_id: str,
    ) -> None:
        ws = ctx.ws
        builder = ctx.builder
        plugin = ids.parse_plugin(plugin_id)
        param_id = ids.build_pparam(plugin.base_class, plugin.lookup, item.name.value)
        builder.node(param_id)
        self.pending.setdefault(param_id, _Pending()).reads.append(sym_id)
        site = ws.site(unit.path, f"{item.sym}:{item.api}", item.line)
        builder.edge(
            model.EdgeKind.READS_PARAM.value,
            sym_id,
            param_id,
            key=ids.edge_key(sym=sym_id, api=item.api, name=item.name.value),
            layer=LAYER,
            run=run,
            values={"api": item.api, "symbol": sym_id, "spelling": item.spelling},
            confidence=model.Confidence.CERTAIN.value,
            evidence=[ws.source(site)],
        )

    def _flush(self, ctx: CollectContext, run: str) -> None:
        """The plugin parameter chain, one write per parameter."""
        builder = ctx.builder
        for param_id in sorted(self.pending):
            pending = self.pending[param_id]
            rows = sorted(pending.declares, key=lambda row: (row[0].sort_key(), row[2]))
            candidates = [
                model.Candidate(
                    value=site.default if site.default_state == "known" else None,
                    value_type=site.value_type,
                    mechanism=model.Mechanism.CODE_DEFAULT.value,
                    source=source,
                    rank=rank,
                    wins=False,
                    note="" if site.default_state == "known" else "the default is not a literal",
                )
                for rank, (source, site, _sym) in enumerate(rows)
            ]
            builder.set_param(param_id, state=model.ParamState.OTHER.value, candidates=candidates)
            if not candidates:
                continue
            builder.fact(
                param_id,
                LAYER,
                run=run,
                values={
                    "code_default": rows[0][1].default
                    if rows[0][1].default_state == "known"
                    else None,
                    "default_known": rows[0][1].default_state == "known",
                    "value_type": rows[0][1].value_type,
                    "declared_by": sorted({sym for _source, _site, sym in rows}),
                    "read_by": sorted(set(pending.reads)),
                },
                confidence=model.Confidence.CERTAIN.value,
                evidence=[source for source, _site, _sym in rows],
            )


def counts(units: Iterable[Unit]) -> dict[str, int]:
    """The totals the bench report quotes."""
    out = {
        "files": 0,
        "parse_failures": 0,
        "node_classes": 0,
        "publisher": 0,
        "subscriber": 0,
        "service_server": 0,
        "service_client": 0,
        "timer": 0,
        "declares": 0,
        "reads": 0,
        "broadcasts": 0,
        "lookups": 0,
        "resolved": 0,
        "unresolved": 0,
    }
    for unit in units:
        out["files"] += 1
        out["parse_failures"] += 0 if unit.parsed else 1
        out["node_classes"] += len({item.qualname for item in unit.classes})
        out["declares"] += len(unit.params)
        out["reads"] += len(unit.reads)
        out["broadcasts"] += len(unit.broadcasts)
        out["lookups"] += len(unit.lookups)
        for endpoint in unit.endpoints:
            out[endpoint.kind] += 1
            if endpoint.kind == "timer":
                continue
            out["resolved" if endpoint.name.resolved else "unresolved"] += 1
    return out


register(NAME, CppCodeCollector)


__all__ = [
    "INSTALL_HINT",
    "KNOWN_WRAPPERS",
    "MACROS",
    "NODE_BASES",
    "NO_SOURCE",
    "SUFFIXES",
    "TREE_SITTER_PIN",
    "CodeIndex",
    "CppCodeCollector",
    "Endpoint",
    "NodeClass",
    "ParamRead",
    "ParamSite",
    "Ref",
    "Registration",
    "TfBroadcast",
    "TfLookup",
    "Unit",
    "analyse",
    "counts",
    "file_report",
    "interface_name",
    "interface_parts",
    "load_parser",
    "package_tables",
    "parse_file",
    "parse_text",
    "qos_of",
    "source_paths",
    "wrapper_endpoints",
    "wrapper_takes_name",
]
