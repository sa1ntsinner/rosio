"""Libraries that create endpoints on a node's behalf, as committed data.

A node that constructs `diagnostic_updater.Updater` publishes `/diagnostics` with no
`create_publisher` call anywhere in its source. A node that constructs
`tf2_ros.TransformListener` subscribes `/tf` and `/tf_static` the same way. On the reference
stack that is twelve `/diagnostics` publishers, sixteen `/tf` subscriptions and two
`PointCloud2` subscriptions hidden behind `message_filters.Subscriber`. Comparing the coded
layer with a running system without this table reports every one of them as an endpoint that
exists only at runtime, which is exactly the noise that makes a drift report get ignored.

Every row is data with a citation, checked against ROS humble under `DISTRO_ROOT`. A row
states three things: the symbol a source constructs, the endpoints that construction creates,
and where the endpoint name comes from. Three name sources exist and they are not
interchangeable:

* `fixed`, the library hardcodes an absolute name (`/tf`, `/diagnostics`)
* `argument`, the caller passes the name and the library only wraps it
  (`message_filters.Subscriber`), so the endpoint is real and its name is the caller's
* `expanded`, the caller passes one name and the library derives several from it, which is
  how one action name becomes three services and two topics

`implicit` rows are the endpoints every ros node gets from its base class. They are not
materialised by the join, because the source constructs nothing: they exist so a later rule
can recognise `/rosout`, `/parameter_events` and the six parameter services rather than
reporting them as observed only.

`discover` is the second half of the same idea and the reason this table is not enough on its
own. A first party class that constructs a known wrapper and takes a node as a constructor
argument is a wrapper too, and the twelve `/diagnostics` publishers on the reference stack are
behind exactly one such class. A per file reader cannot see it, because the construction is in
another module. The rule is deliberately narrow: only the endpoints whose name the library
fixes are inherited, so a discovered wrapper can never invent a name, and a class that takes no
node is never promoted however suggestive its own name is.

QoS is a short readable note, never a number this table invented. A wrapper whose default
depends on a constructor argument says so.
"""

from __future__ import annotations

import re
from collections.abc import Iterable
from dataclasses import dataclass

from rosio.errors import ContractError

DISTRO_ROOT = "/opt/ros/humble"
"""Where every row was checked. Recorded so a row can be rechecked on another distro."""

LANGUAGES = ("python", "cpp")

ENDPOINT_KINDS = ("publisher", "subscriber", "service_server", "service_client")

NAME_SOURCES = ("fixed", "argument", "expanded")

ACTION_SERVICE_SUFFIXES = ("_action/send_goal", "_action/cancel_goal", "_action/get_result")
ACTION_TOPIC_SUFFIXES = ("_action/feedback", "_action/status")
"""The five names rcl_action derives from one action name. Checked in the string table of
`lib/librcl_action.so`, which holds `%s/_action/send_goal`, `%s/_action/cancel_goal`,
`%s/_action/get_result`, `%s/_action/feedback` and `%s/_action/status`, all formatted in
`src/rcl_action/action_client.c`."""


@dataclass(frozen=True)
class WrapperEndpoint:
    """One endpoint a wrapper construction creates."""

    kind: str
    name: str = ""
    type: str = ""
    name_source: str = "fixed"
    suffix: str = ""
    qos: str = ""

    def __post_init__(self) -> None:
        if self.kind not in ENDPOINT_KINDS:
            raise ContractError(
                f"wrapper endpoint kind {self.kind!r} is not one of {ENDPOINT_KINDS}"
            )
        if self.name_source not in NAME_SOURCES:
            raise ContractError(f"wrapper name source {self.name_source!r} is unknown")
        if self.name_source == "fixed" and not self.name:
            raise ContractError("a fixed wrapper endpoint needs its name")
        if self.name_source != "fixed" and self.name:
            raise ContractError("a wrapper endpoint taking its name from the call states none")

    def resolve(self, name: str = "") -> str:
        """The endpoint name for one construction, empty when the caller name is unknown."""
        if self.name_source == "fixed":
            return self.name
        if not name:
            return ""
        if self.name_source == "argument":
            return name
        return f"{name.rstrip('/')}/{self.suffix}"


@dataclass(frozen=True)
class Wrapper:
    """One library symbol that creates endpoints for the node that constructs it."""

    symbol: str
    library: str
    language: str
    endpoints: tuple[WrapperEndpoint, ...]
    argument: int | None = None
    evidence: str = ""
    implicit: bool = False
    note: str = ""
    discovered: bool = False
    via: tuple[str, ...] = ()
    """Set by `discover` only: a first party class is not committed data, it is a workspace
    reading, and `via` names the committed rows it inherited its endpoints from."""

    def __post_init__(self) -> None:
        if self.language not in LANGUAGES:
            raise ContractError(f"wrapper language {self.language!r} is not one of {LANGUAGES}")
        if not self.symbol or not self.evidence:
            raise ContractError("a wrapper row needs a symbol and its evidence")
        if not self.endpoints:
            raise ContractError(f"wrapper {self.symbol} states no endpoint")

    @property
    def takes_name(self) -> bool:
        return any(item.name_source != "fixed" for item in self.endpoints)

    def resolved(self, name: str = "") -> tuple[tuple[str, str, str], ...]:
        """(kind, name, type) per endpoint. An endpoint whose name the caller supplies and
        that was not supplied one is left out, because a wrapper never invents a name."""
        out = []
        for item in self.endpoints:
            resolved = item.resolve(name)
            if resolved:
                out.append((item.kind, resolved, item.type))
        return tuple(out)


_TF = "tf2_msgs/msg/TFMessage"

TABLE: tuple[Wrapper, ...] = (
    Wrapper(
        symbol="tf2_ros.TransformListener",
        library="tf2_ros",
        language="python",
        endpoints=(
            WrapperEndpoint("subscriber", "/tf", _TF, qos="depth 100, volatile"),
            WrapperEndpoint("subscriber", "/tf_static", _TF, qos="depth 100, transient local"),
        ),
        evidence="lib/python3.10/site-packages/tf2_ros/transform_listener.py:98-101 creates "
        "both subscriptions, with the two default profiles built at :69-80",
        note="The dominant tf idiom on the reference stack: sixteen listener pairs, none of "
        "which names a topic in the node source.",
    ),
    Wrapper(
        symbol="tf2_ros::TransformListener",
        library="tf2_ros",
        language="cpp",
        endpoints=(
            WrapperEndpoint("subscriber", "/tf", _TF, qos="DynamicListenerQoS, depth 100"),
            WrapperEndpoint(
                "subscriber", "/tf_static", _TF, qos="StaticListenerQoS, depth 100 transient local"
            ),
        ),
        evidence="include/tf2_ros/tf2_ros/transform_listener.hpp:141-143 and :153-155 create "
        "both subscriptions; the two profiles are tf2_ros/qos.hpp:39-61",
    ),
    Wrapper(
        symbol="tf2_ros.TransformBroadcaster",
        library="tf2_ros",
        language="python",
        endpoints=(WrapperEndpoint("publisher", "/tf", _TF, qos="depth 100"),),
        evidence="lib/python3.10/site-packages/tf2_ros/transform_broadcaster.py:61, default "
        "profile at :60",
    ),
    Wrapper(
        symbol="tf2_ros::TransformBroadcaster",
        library="tf2_ros",
        language="cpp",
        endpoints=(
            WrapperEndpoint("publisher", "/tf", _TF, qos="DynamicBroadcasterQoS, depth 100"),
        ),
        evidence="include/tf2_ros/tf2_ros/transform_broadcaster.hpp:74, profile at qos.hpp:46",
    ),
    Wrapper(
        symbol="tf2_ros.StaticTransformBroadcaster",
        library="tf2_ros",
        language="python",
        endpoints=(
            WrapperEndpoint("publisher", "/tf_static", _TF, qos="depth 1, transient local"),
        ),
        evidence="lib/python3.10/site-packages/tf2_ros/static_transform_broadcaster.py:62, "
        "default profile at :56-61",
    ),
    Wrapper(
        symbol="tf2_ros::StaticTransformBroadcaster",
        library="tf2_ros",
        language="cpp",
        endpoints=(
            WrapperEndpoint(
                "publisher", "/tf_static", _TF, qos="StaticBroadcasterQoS, depth 1 transient local"
            ),
        ),
        evidence="include/tf2_ros/tf2_ros/static_transform_broadcaster.hpp:82, profile at "
        "qos.hpp:63",
        note="The first party tree constructs this nowhere. Every static transform on the "
        "reference stack comes from a launched static_transform_publisher or from the urdf.",
    ),
    Wrapper(
        symbol="diagnostic_updater.Updater",
        library="diagnostic_updater",
        language="python",
        endpoints=(
            WrapperEndpoint(
                "publisher", "/diagnostics", "diagnostic_msgs/msg/DiagnosticArray", qos="depth 1"
            ),
        ),
        evidence="local/lib/python3.10/dist-packages/diagnostic_updater/"
        "_diagnostic_updater.py:239-240 creates the publisher and :244 the update timer",
        note="A first party class that constructs the updater and holds it as an attribute "
        "hides the publisher one level further, so the topic appears in no node source at "
        "all and every node that uses such a class is a publisher of it.",
    ),
    Wrapper(
        symbol="diagnostic_updater::Updater",
        library="diagnostic_updater",
        language="cpp",
        endpoints=(
            WrapperEndpoint(
                "publisher", "/diagnostics", "diagnostic_msgs/msg/DiagnosticArray", qos="depth 1"
            ),
        ),
        evidence="strings lib/libdiagnostic_updater.so holds the exact literal /diagnostics; "
        "the constructor taking the node interfaces is declared at "
        "include/diagnostic_updater/diagnostic_updater.hpp:385-393",
    ),
    Wrapper(
        symbol="message_filters.Subscriber",
        library="message_filters",
        language="python",
        endpoints=(WrapperEndpoint("subscriber", name_source="argument", qos="depth 10 default"),),
        argument=2,
        evidence="local/lib/python3.10/dist-packages/message_filters/__init__.py:84-90 calls "
        "node.create_subscription with the topic from its third positional argument, declared "
        "at :61-67",
        note="A node that time synchronises two sensor streams subscribes only this way, so "
        "the topics reach no create_subscription call and exist in the source nowhere else.",
    ),
    Wrapper(
        symbol="message_filters::Subscriber",
        library="message_filters",
        language="cpp",
        endpoints=(WrapperEndpoint("subscriber", name_source="argument"),),
        argument=1,
        evidence="include/message_filters/message_filters/subscriber.h:64 and :75 declare "
        "subscribe(node, topic, qos), so the topic is the second argument of the call and the "
        "first after the node",
    ),
    Wrapper(
        symbol="rclpy.action.ActionClient",
        library="rclpy",
        language="python",
        endpoints=(
            WrapperEndpoint(
                "service_client", name_source="expanded", suffix=ACTION_SERVICE_SUFFIXES[0]
            ),
            WrapperEndpoint(
                "service_client", name_source="expanded", suffix=ACTION_SERVICE_SUFFIXES[1]
            ),
            WrapperEndpoint(
                "service_client", name_source="expanded", suffix=ACTION_SERVICE_SUFFIXES[2]
            ),
            WrapperEndpoint(
                "subscriber",
                type="action_msgs/msg/GoalStatusArray",
                name_source="expanded",
                suffix=ACTION_TOPIC_SUFFIXES[1],
            ),
            WrapperEndpoint("subscriber", name_source="expanded", suffix=ACTION_TOPIC_SUFFIXES[0]),
        ),
        argument=2,
        evidence="strings lib/librcl_action.so holds %s/_action/send_goal, "
        "%s/_action/cancel_goal, %s/_action/get_result, %s/_action/feedback and "
        "%s/_action/status, formatted in src/rcl_action/action_client.c",
        note="The reference stack also subscribes /navigate_to_pose/_action/status as a plain "
        "topic, so the same name arrives from two directions and must not become two ids.",
    ),
    Wrapper(
        symbol="rclpy.action.ActionServer",
        library="rclpy",
        language="python",
        endpoints=(
            WrapperEndpoint(
                "service_server", name_source="expanded", suffix=ACTION_SERVICE_SUFFIXES[0]
            ),
            WrapperEndpoint(
                "service_server", name_source="expanded", suffix=ACTION_SERVICE_SUFFIXES[1]
            ),
            WrapperEndpoint(
                "service_server", name_source="expanded", suffix=ACTION_SERVICE_SUFFIXES[2]
            ),
            WrapperEndpoint(
                "publisher",
                type="action_msgs/msg/GoalStatusArray",
                name_source="expanded",
                suffix=ACTION_TOPIC_SUFFIXES[1],
            ),
            WrapperEndpoint("publisher", name_source="expanded", suffix=ACTION_TOPIC_SUFFIXES[0]),
        ),
        argument=2,
        evidence="same rcl_action expansion as the client, src/rcl_action/action_server.c, "
        "with the three services served and the two topics published",
    ),
    Wrapper(
        symbol="rclcpp_action::create_client",
        library="rclcpp_action",
        language="cpp",
        endpoints=(
            WrapperEndpoint(
                "service_client", name_source="expanded", suffix=ACTION_SERVICE_SUFFIXES[0]
            ),
            WrapperEndpoint(
                "service_client", name_source="expanded", suffix=ACTION_SERVICE_SUFFIXES[1]
            ),
            WrapperEndpoint(
                "service_client", name_source="expanded", suffix=ACTION_SERVICE_SUFFIXES[2]
            ),
            WrapperEndpoint(
                "subscriber",
                type="action_msgs/msg/GoalStatusArray",
                name_source="expanded",
                suffix=ACTION_TOPIC_SUFFIXES[1],
            ),
            WrapperEndpoint("subscriber", name_source="expanded", suffix=ACTION_TOPIC_SUFFIXES[0]),
        ),
        argument=1,
        evidence="include/rclcpp_action/rclcpp_action/create_client.hpp takes the action name "
        "and hands it to rcl_action, whose expansion is in lib/librcl_action.so",
        note="Unused by the reference stack: it has no C++ action client.",
    ),
    Wrapper(
        symbol="rclcpp_action::create_server",
        library="rclcpp_action",
        language="cpp",
        endpoints=(
            WrapperEndpoint(
                "service_server", name_source="expanded", suffix=ACTION_SERVICE_SUFFIXES[0]
            ),
            WrapperEndpoint(
                "service_server", name_source="expanded", suffix=ACTION_SERVICE_SUFFIXES[1]
            ),
            WrapperEndpoint(
                "service_server", name_source="expanded", suffix=ACTION_SERVICE_SUFFIXES[2]
            ),
            WrapperEndpoint(
                "publisher",
                type="action_msgs/msg/GoalStatusArray",
                name_source="expanded",
                suffix=ACTION_TOPIC_SUFFIXES[1],
            ),
            WrapperEndpoint("publisher", name_source="expanded", suffix=ACTION_TOPIC_SUFFIXES[0]),
        ),
        argument=1,
        evidence="include/rclcpp_action/rclcpp_action/create_server.hpp, same rcl_action "
        "expansion as the client",
        note="Unused by the reference stack.",
    ),
    Wrapper(
        symbol="image_transport::ImageTransport::advertise",
        library="image_transport",
        language="cpp",
        endpoints=(WrapperEndpoint("publisher", name_source="argument"),),
        argument=0,
        evidence="include/image_transport/image_transport/image_transport.hpp:120 and :126 "
        "advertise(base_topic, ...)",
        note="The base topic is real. The per transport siblings (compressed, theora) are not "
        "in this table because which ones exist depends on the transport plugins installed, "
        "which no source states. Unused by the reference stack.",
    ),
    Wrapper(
        symbol="image_transport::ImageTransport::subscribe",
        library="image_transport",
        language="cpp",
        endpoints=(WrapperEndpoint("subscriber", name_source="argument"),),
        argument=0,
        evidence="include/image_transport/image_transport/image_transport.hpp declares "
        "subscribe(base_topic, ...) beside advertise",
        note="Unused by the reference stack.",
    ),
    Wrapper(
        symbol="rclpy.node.Node",
        library="rclpy",
        language="python",
        implicit=True,
        endpoints=(
            WrapperEndpoint("publisher", "/rosout", "rcl_interfaces/msg/Log"),
            WrapperEndpoint("publisher", "/parameter_events", "rcl_interfaces/msg/ParameterEvent"),
            WrapperEndpoint("service_server", "~/describe_parameters"),
            WrapperEndpoint("service_server", "~/get_parameter_types"),
            WrapperEndpoint("service_server", "~/get_parameters"),
            WrapperEndpoint("service_server", "~/list_parameters"),
            WrapperEndpoint("service_server", "~/set_parameters"),
            WrapperEndpoint("service_server", "~/set_parameters_atomically"),
        ),
        evidence="local/lib/python3.10/dist-packages/rclpy/node.py:200 creates the "
        "/parameter_events publisher; strings lib/librcl.so holds /rosout and "
        "lib/librclcpp.so holds /parameter_events and the six parameter service names",
        note="Implicit: every node has these and no source constructs them, so the join "
        "materialises nothing here and a later rule uses the row to stay quiet about them. "
        "The base class publishes /parameter_events and does not subscribe it: the subscriber "
        "belongs to ParameterEventHandler, which a source constructs by hand.",
    ),
    Wrapper(
        symbol="rclcpp::Node",
        library="rclcpp",
        language="cpp",
        implicit=True,
        endpoints=(
            WrapperEndpoint("publisher", "/rosout", "rcl_interfaces/msg/Log"),
            WrapperEndpoint("publisher", "/parameter_events", "rcl_interfaces/msg/ParameterEvent"),
            WrapperEndpoint("service_server", "~/describe_parameters"),
            WrapperEndpoint("service_server", "~/get_parameter_types"),
            WrapperEndpoint("service_server", "~/get_parameters"),
            WrapperEndpoint("service_server", "~/list_parameters"),
            WrapperEndpoint("service_server", "~/set_parameters"),
            WrapperEndpoint("service_server", "~/set_parameters_atomically"),
        ),
        evidence="strings lib/librcl.so holds /rosout, lib/librclcpp.so holds "
        "/parameter_events and the six parameter service names",
    ),
)


INDEX: dict[str, Wrapper] = {row.symbol: row for row in TABLE}


def rows() -> tuple[Wrapper, ...]:
    """The table sorted by symbol, which is how it is compared."""
    return tuple(sorted(TABLE, key=lambda row: row.symbol))


def symbols() -> tuple[str, ...]:
    return tuple(sorted(INDEX))


def canonical(symbol: str) -> str:
    """One spelling for a symbol, so `a::B` and `a.B` reach the same row."""
    return str(symbol).replace("::", ".").strip()


def lookup(symbol: str, language: str = "") -> Wrapper | None:
    """The row for a construction, or None.

    A bare class name is accepted only when every row it could mean states the same
    endpoints, because the two language spellings of one wrapper are the same wrapper and
    two different wrappers must never be guessed apart.
    """
    if not symbol:
        return None
    exact = INDEX.get(str(symbol))
    if exact is not None and (not language or exact.language == language):
        return exact
    wanted = canonical(symbol)
    found = [
        row
        for row in rows()
        if (not language or row.language == language)
        and (canonical(row.symbol) == wanted or canonical(row.symbol).endswith("." + wanted))
    ]
    if not found:
        return None
    shapes = {tuple((item.kind, item.name, item.suffix) for item in row.endpoints) for row in found}
    if len(shapes) > 1:
        return None
    return found[0]


def is_known(symbol: str, language: str = "") -> bool:
    return lookup(symbol, language) is not None


# ---------------------------------------------------------------------------
# first party wrappers, discovered from a workspace


NODE_ARGUMENT_NAMES = frozenset({"node", "parent", "parent_node", "ros_node", "rosnode"})
"""Constructor argument names that mean a node was handed over. A reader may also decide the
question from the annotation or from what the construction is given, and says so on the
candidate: this set is the last resort, not the rule."""


@dataclass(frozen=True)
class Candidate:
    """One class a reader offers to `discover`, with the two facts the rule needs.

    `constructs` holds every class the constructor builds, spelled as the reader resolved it
    through the import table. Nothing is filtered there, because the rule below is what decides
    which of them matter, and a chain of two first party classes needs the second name to be
    present before the first one is known to be a wrapper.
    """

    symbol: str
    constructs: tuple[str, ...] = ()
    takes_node: bool = False
    language: str = "python"
    file: str = ""
    line_hint: int | None = None

    def __post_init__(self) -> None:
        if self.language not in LANGUAGES:
            raise ContractError(f"candidate language {self.language!r} is not one of {LANGUAGES}")
        if not self.symbol:
            raise ContractError("a wrapper candidate needs its symbol")


def _inherited(
    candidate: Candidate, found: dict[str, Wrapper]
) -> tuple[tuple[WrapperEndpoint, ...], tuple[str, ...]]:
    """The endpoints a candidate inherits, and the rows they came from.

    Only fixed name endpoints are inherited. A library that takes its name from the call
    (`message_filters.Subscriber`) states nothing about the wrapper's own signature, and
    guessing which argument of the wrapper carries that name would invent a topic.
    """
    kept: dict[tuple[str, str, str], WrapperEndpoint] = {}
    via: list[str] = []
    for symbol in sorted(dict.fromkeys(candidate.constructs)):
        row = found.get(symbol) or lookup(symbol, candidate.language)
        if row is None or row.implicit:
            continue
        fixed = [item for item in row.endpoints if item.name_source == "fixed"]
        if not fixed:
            continue
        via.append(row.symbol)
        for item in fixed:
            kept.setdefault((item.kind, item.name, item.type), item)
    return tuple(kept[key] for key in sorted(kept)), tuple(sorted(dict.fromkeys(via)))


def discover(candidates: Iterable[Candidate]) -> tuple[Wrapper, ...]:
    """The first party wrapper classes a workspace defines, as rows of this same table.

    A class qualifies when it takes a node and constructs a wrapper that is either committed
    here or already discovered. Every round reconsiders every candidate, including the ones
    already promoted, because a wrapper over a wrapper otherwise keeps whichever half of its
    endpoints the read order happened to offer first. What a row states only grows, so the
    fixpoint terminates, and the result is a pure function of the candidates.
    """
    rows = sorted({item.symbol: item for item in candidates}.values(), key=lambda item: item.symbol)
    found: dict[str, Wrapper] = {}
    changed = True
    while changed:
        changed = False
        for candidate in rows:
            if not candidate.takes_node:
                continue
            endpoints, via = _inherited(candidate, found)
            held = found.get(candidate.symbol)
            if not endpoints or (held is not None and held.endpoints == endpoints):
                continue
            libraries = {
                (found.get(symbol) or lookup(symbol, candidate.language)).library  # type: ignore[union-attr]
                for symbol in via
            }
            found[candidate.symbol] = Wrapper(
                symbol=candidate.symbol,
                library=libraries.pop() if len(libraries) == 1 else "first_party",
                language=candidate.language,
                endpoints=endpoints,
                evidence=(
                    f"{candidate.file or candidate.symbol} constructs "
                    + ", ".join(via)
                    + " and takes a node as a constructor argument, so every construction of it "
                    "creates those endpoints on the caller"
                ),
                discovered=True,
                via=via,
            )
            changed = True
    return tuple(sorted(found.values(), key=lambda row: row.symbol))


def index(rows: Iterable[Wrapper]) -> dict[str, Wrapper]:
    """Discovered rows keyed by symbol, which is how a reader consults them."""
    return {row.symbol: row for row in sorted(rows, key=lambda item: item.symbol)}


def endpoints_for(
    symbol: str, name: str = "", language: str = ""
) -> tuple[tuple[str, str, str], ...]:
    """(kind, name, type) per endpoint one construction creates, empty for an unknown row."""
    row = lookup(symbol, language)
    if row is None or row.implicit:
        return ()
    return row.resolved(name)


def implicit_endpoints(language: str = "") -> tuple[tuple[str, str, str], ...]:
    """The endpoints every node gets from its base class, for a rule that has to stay quiet
    about them. Never materialised by the join: no source constructs them."""
    out: list[tuple[str, str, str]] = []
    for row in rows():
        if not row.implicit or (language and row.language != language):
            continue
        out.extend(row.resolved())
    return tuple(sorted(dict.fromkeys(out)))


def libraries() -> tuple[str, ...]:
    return tuple(sorted({row.library for row in TABLE}))


@dataclass(frozen=True)
class WrapperNode:
    """A whole node a library creates on behalf of the node that constructed it.

    `pattern` is matched against the full node name. `{parent}` in it stands for the name of
    another running node in the same namespace, so a row that needs one cannot match a node
    that merely ends the same way.
    """

    symbol: str
    pattern: str
    evidence: str = ""

    def matches(self, name: str, siblings: Iterable[str]) -> bool:
        if "{parent}" not in self.pattern:
            return re.fullmatch(self.pattern, name) is not None
        return any(
            re.fullmatch(self.pattern.replace("{parent}", re.escape(parent)), name) is not None
            for parent in siblings
            if parent != name
        )


NODES: tuple[WrapperNode, ...] = (
    WrapperNode(
        symbol="tf2_ros::TransformListener",
        pattern="transform_listener_impl_[0-9a-f]+",
        evidence="/opt/ros/jazzy/lib/libtf2_ros.so holds the format transform_listener_impl_%zx, "
        "the node a listener with its own spin thread creates; lib/python3.12/site-packages/"
        "tf2_ros/transform_listener.py:87 formats the python one the same way",
    ),
    WrapperNode(
        symbol="nav2_behavior_tree::BtActionServer",
        pattern="{parent}_[A-Za-z0-9_]+_rclcpp_node",
        evidence="/opt/ros/jazzy/include/nav2_behavior_tree/bt_action_server_impl.hpp:121-129 "
        "names the client node it creates <node name>_<action name>_rclcpp_node",
    ),
)
"""Nodes a library creates beside the one that constructed it. Checked against ROS jazzy.
A probe sees them as running nodes that no launch file starts, and without this table each
one reads as a node that will not come back after a restart, which is false: the node that
created it brings it back."""


def node_wrapper(name: str, siblings: Iterable[str] = ()) -> str:
    """The symbol of the library that created a node of this name, or an empty string."""
    peers = tuple(siblings)
    return next((row.symbol for row in NODES if row.matches(name, peers)), "")


__all__ = [
    "ACTION_SERVICE_SUFFIXES",
    "ACTION_TOPIC_SUFFIXES",
    "DISTRO_ROOT",
    "ENDPOINT_KINDS",
    "INDEX",
    "LANGUAGES",
    "NAME_SOURCES",
    "NODES",
    "NODE_ARGUMENT_NAMES",
    "TABLE",
    "Candidate",
    "Wrapper",
    "WrapperEndpoint",
    "WrapperNode",
    "canonical",
    "discover",
    "endpoints_for",
    "implicit_endpoints",
    "index",
    "is_known",
    "libraries",
    "lookup",
    "node_wrapper",
    "rows",
    "symbols",
]
