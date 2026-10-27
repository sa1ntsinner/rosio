"""The child that attaches to a running system and reports what it says.

Run as a script with one json request on stdin and one json response on stdout, behind a
marker so a shell banner or a chatty middleware cannot corrupt the payload. The parent never
imports rclpy, so a stuck discovery or an rmw that aborts costs one probe.

The node this creates has rosout and its own parameter services turned off, creates no
publisher of its own and calls only `list_parameters`, `get_parameters` and `get_state`.
The `/parameter_events` publisher rclpy gives every node is the one publisher it holds, and
it never sends, because nothing sets a parameter on this node. Sampling is bounded by a
window given in the request, and the counts it produces are volatile.

Everything is read in one pass with a fixed order: settle until the graph stops changing,
read the graph, subscribe, sample
for the window, then ask each node for its parameters and its lifecycle state. A read that
raises is recorded against the thing it was about and the rest of the pass carries on, so one
node that stopped answering does not empty the layer.
"""

from __future__ import annotations

import contextlib
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Any

_REPO = str(Path(__file__).resolve().parents[2])
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

from rosio import model  # noqa: E402
from rosio.collect import probe  # noqa: E402

PROBE_NODE = "rosio_probe"

TF_TOPICS = ("/tf", "/tf_static")

TF_TYPE = "tf2_msgs/msg/TFMessage"

LIFECYCLE_GET_STATE = "lifecycle_msgs/srv/GetState"

SPIN_SLICE = 0.05

DISCOVERY_QUIET = 3.0
"""Seconds the graph has to stay unchanged before it is read. Discovery pauses between
bursts, so a shorter quiet time ends the wait inside a pause."""

DISCOVERY_STEP = 0.25

DISCOVERY_LIMIT = 30.0
"""Seconds after the settle time a graph that keeps changing is waited for."""

_HISTORY = {1: "keep_last", 2: "keep_all"}
_RELIABILITY = {1: "reliable", 2: "best_effort"}
_DURABILITY = {1: "transient_local", 2: "volatile"}
_LIVELINESS = {1: "automatic", 3: "manual_by_topic"}
"""Only the members that state something. System default and unknown are not values, they
are the absence of one, and writing an rmw default in their place manufactures drift."""

_PARAM_TYPES = {
    1: ("bool", "bool_value"),
    2: ("integer", "integer_value"),
    3: ("double", "double_value"),
    4: ("string", "string_value"),
    5: ("byte_array", "byte_array_value"),
    6: ("bool_array", "bool_array_value"),
    7: ("integer_array", "integer_array_value"),
    8: ("double_array", "double_array_value"),
    9: ("string_array", "string_array_value"),
}


class Probe:
    def __init__(self, request: dict[str, Any]) -> None:
        self.stamps = bool(request.get("stamps", False))
        self.window = max(0.0, float(request.get("window", probe.DEFAULT_WINDOW)))
        self.settle = max(0.0, float(request.get("settle", probe.DEFAULT_SETTLE)))
        self.service_timeout = max(0.1, float(request.get("service_timeout", 2.0)))
        self.system = probe.ObservedSystem(
            window_s=self.window,
            settle_s=self.settle,
            domain_id=os.environ.get("ROS_DOMAIN_ID", ""),
            localhost_only=os.environ.get("ROS_LOCALHOST_ONLY", ""),
        )
        self.node: Any = None
        self.counts: dict[str, int] = {}
        self.tf_seen: dict[tuple[str, str, bool], str | None] = {}
        self.tf_publishers: dict[str, list[str]] = {}

    # -----------------------------------------------------------------
    def run(self) -> probe.ObservedSystem:
        import rclpy
        from ament_index_python.packages import PackageNotFoundError, get_package_prefix
        from rclpy.utilities import get_rmw_implementation_identifier

        rmw = os.environ.get("RMW_IMPLEMENTATION")
        if rmw:
            try:
                get_package_prefix(rmw)
            except PackageNotFoundError:
                self.system.reached = False
                self.system.notes.append("rmw_library_missing")
                return self.system
        rclpy.init(args=None)
        self.system.rmw = get_rmw_implementation_identifier()
        try:
            self._pass()
        finally:
            with contextlib.suppress(Exception):
                if self.node is not None:
                    self.node.destroy_node()
                rclpy.shutdown()
        return self.system.sort()

    def _pass(self) -> None:
        from rclpy.node import Node

        self.node = Node(
            PROBE_NODE,
            start_parameter_services=False,
            enable_rosout=False,
        )
        self._discover()
        graph = self._graph()
        subscriptions = self._subscribe(graph)
        sampled = self._sample()
        for handle in subscriptions:
            with contextlib.suppress(Exception):
                self.node.destroy_subscription(handle)
        self._late(graph)
        self._rates(sampled)
        self._tf_links()
        self._details(graph)

    # -----------------------------------------------------------------
    # graph

    def _discover(self) -> None:
        """Spin through the settle time, then until the graph stops growing.

        Discovery arrives in bursts: a container holding many nodes is one participant, and
        on a loaded machine it can answer a fresh probe after the settle time is over. A
        fixed settle then read the graph before the burst, two probes of one unchanged system
        described two different systems, and the second reported every node it missed as
        launched and not running. A settle of zero still means no waiting at all.
        """
        self._spin(self.settle)
        if self.settle <= 0:
            return
        deadline = time.monotonic() + DISCOVERY_LIMIT
        last = self._census()
        quiet_since = time.monotonic()
        while time.monotonic() - quiet_since < DISCOVERY_QUIET:
            if time.monotonic() >= deadline:
                self.system.notes.append(
                    f"the graph was still changing {DISCOVERY_LIMIT:g} seconds after the "
                    "settle time, so it was read while it changed"
                )
                return
            self._spin(DISCOVERY_STEP)
            now = self._census()
            if now != last:
                last, quiet_since = now, time.monotonic()

    def _foreign(self, found: list[tuple[str, str]]) -> list[tuple[str, str]]:
        """The graph without the probe's own node. Its endpoints, its services and the
        parameter events topic every rclpy node opens are this tool, not the system, and on
        an empty domain they were the whole observed layer."""
        own = (self.node.get_name(), self.node.get_namespace())
        return [row for row in found if tuple(row) != own]

    def _census(self) -> tuple[Any, ...]:
        """What discovery has delivered so far: node names, and every topic and service
        with how many ends each has."""
        try:
            nodes = tuple(sorted(self._foreign(self.node.get_node_names_and_namespaces())))
            topics = tuple(
                (
                    topic,
                    self.node.count_publishers(topic),
                    self.node.count_subscribers(topic),
                )
                for topic, _types in sorted(self.node.get_topic_names_and_types())
            )
            services = tuple(sorted(name for name, _ in self.node.get_service_names_and_types()))
        except Exception as exc:
            self._failed("graph", exc, "census")
            return ()
        return nodes, topics, services

    def _graph(self) -> dict[str, dict[str, Any]]:
        """Every node with its endpoints. One read, so one picture of one moment."""
        from rclpy.action import (
            get_action_client_names_and_types_by_node,
            get_action_server_names_and_types_by_node,
        )

        out: dict[str, dict[str, Any]] = {}
        try:
            found = self._foreign(self.node.get_node_names_and_namespaces())
        except Exception as exc:
            self._failed("graph", exc, "get_node_names_and_namespaces")
            return out
        for name, namespace in sorted(found):
            fqn = _join(namespace, name)
            if fqn in out:
                # Two running nodes answer to one name. The graph reads by name, so the
                # endpoints of one of them are not reachable and the record says so.
                self.system.notes.append(
                    f"{found.count((name, namespace))} running nodes answer to {fqn}, so the "
                    "endpoints and the parameters of one of them were read"
                )
                continue
            readers = (
                ("publisher", self.node.get_publisher_names_and_types_by_node),
                ("subscriber", self.node.get_subscriber_names_and_types_by_node),
                ("service_server", self.node.get_service_names_and_types_by_node),
                ("service_client", self.node.get_client_names_and_types_by_node),
            )
            rows: list[tuple[str, str, str]] = []
            complete = True
            for kind, reader in readers:
                try:
                    for endpoint, types in reader(name, namespace):
                        for type_name in sorted(types) or [""]:
                            rows.append((kind, endpoint, type_name))
                except Exception as exc:
                    complete = False
                    self._failed("graph", exc, f"{fqn} {kind}")
            for kind, reader in (
                ("action_server", get_action_server_names_and_types_by_node),
                ("action_client", get_action_client_names_and_types_by_node),
            ):
                try:
                    for endpoint, types in reader(self.node, name, namespace):
                        for type_name in sorted(types) or [""]:
                            rows.append((kind, endpoint, type_name))
                except Exception as exc:
                    complete = False
                    self._failed("graph", exc, f"{fqn} {kind}")
            out[fqn] = {
                "name": name,
                "namespace": namespace,
                "rows": rows,
                "complete": complete,
                "services": {row[1] for row in rows if row[0] == "service_server"},
                "service_types": {row[1]: row[2] for row in rows if row[0] == "service_server"},
            }
        self._endpoints(out)
        return out

    def _endpoints(self, graph: dict[str, dict[str, Any]]) -> None:
        """Endpoint records, with the QoS the middleware states for each topic end."""
        info: dict[tuple[str, str], list[dict[str, Any]]] = {}
        topics = sorted({row[1] for entry in graph.values() for row in entry["rows"]})
        for kind, reader in (
            ("publisher", "get_publishers_info_by_topic"),
            ("subscriber", "get_subscriptions_info_by_topic"),
        ):
            for topic in topics:
                try:
                    found = getattr(self.node, reader)(topic)
                except Exception:
                    continue
                for item in found:
                    owner = _join(item.node_namespace, item.node_name)
                    info.setdefault((kind, f"{owner}|{topic}"), []).append(
                        {"qos": _qos(item.qos_profile), "type": str(item.topic_type or "")}
                    )
        for fqn in sorted(graph):
            for kind, endpoint, type_name in sorted(graph[fqn]["rows"]):
                ends = info.get((kind, f"{fqn}|{endpoint}"), [])
                stated = ends[0] if ends else None
                qos = stated["qos"] if stated else None
                reason = ""
                if len({repr(end["qos"]) for end in ends}) > 1:
                    # The graph names an endpoint by node and topic only, so which policy
                    # belongs to which source site cannot be read from it.
                    qos = None
                    reason = f"this node has {len(ends)} {kind}s on this topic with different qos"
                elif qos is None and kind in ("publisher", "subscriber"):
                    reason = "the middleware reported no endpoint information for this end"
                elif qos is None:
                    reason = "rmw states no qos for a service or an action endpoint"
                self.system.endpoints.append(
                    probe.Endpoint(
                        kind=kind,
                        node=fqn,
                        name=endpoint,
                        type=type_name or (stated or {}).get("type") or None,
                        qos=qos,
                        qos_reason=reason,
                    )
                )
        for topic in TF_TOPICS:
            self.tf_publishers[topic] = sorted(
                {
                    item.node
                    for item in self.system.endpoints
                    if item.kind == "publisher" and item.name == topic
                }
            )

    def _late(self, graph: dict[str, dict[str, Any]]) -> None:
        """Nodes the graph gained after it was read.

        Discovery is not instant, so a graph read once is a graph read at one moment, and a
        settle window that was too short reads as a smaller system with nothing to say about
        it. The list is read a second time and the difference is stated.
        """
        try:
            found = self._foreign(self.node.get_node_names_and_namespaces())
        except Exception as exc:
            self._failed("graph", exc, "get_node_names_and_namespaces")
            return
        late = sorted(
            {
                (_join(namespace, name), name, namespace)
                for name, namespace in found
                if _join(namespace, name) not in graph
            }
        )
        if late:
            self.system.notes.append(
                "these nodes answered after the graph was read, so their endpoints and "
                "their parameters were not read: " + ", ".join(row[0] for row in late)
            )
        # A late node still runs. Leaving it out reported it as launched and not running.
        for fqn, name, namespace in late:
            self.system.nodes.append(
                probe.ObservedNode(
                    name=name,
                    namespace=namespace,
                    fqn=fqn,
                    parameters_read=False,
                    parameters_reason="the node answered after the graph was read",
                    internal=probe.internal_name(name),
                )
            )

    # -----------------------------------------------------------------
    # sampling

    def _subscribe(self, graph: dict[str, dict[str, Any]]) -> list[Any]:
        """One raw counting subscription per topic, plus the two tf topics read for content.

        Best effort and volatile on the counting side, because that pair is compatible with
        every publisher, and a stricter request would silently receive nothing.
        """
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

        handles: list[Any] = []
        counting = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        types: dict[str, str] = {}
        for entry in graph.values():
            for kind, endpoint, type_name in entry["rows"]:
                if kind in ("publisher", "subscriber") and type_name:
                    types.setdefault(endpoint, type_name)
        for topic in sorted(types):
            message = self._message(types[topic], topic)
            if message is None:
                continue
            self.counts.setdefault(topic, 0)
            samples = None
            if self.stamps:
                fields = message.get_fields_and_field_types()
                if next(iter(fields.items()), None) in (
                    ("header", "std_msgs/Header"),
                    ("header", "std_msgs/msg/Header"),
                ):
                    samples = probe.TopicStamps(topic)
                    self.system.stamps.append(samples)
            try:
                handles.append(
                    self.node.create_subscription(
                        message,
                        topic,
                        _counter(self.counts, topic, samples),
                        counting,
                        raw=True,
                    )
                )
            except Exception as exc:
                self.counts.pop(topic, None)
                self._failed("sample", exc, topic)
        handles.extend(self._tf_subscriptions())
        if len(handles) > 200:
            self.system.notes.append(f"{len(handles)} subscriptions were opened for one window")
        return handles

    def _tf_subscriptions(self) -> list[Any]:
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

        message = self._message(TF_TYPE, "/tf")
        if message is None:
            return []
        out = []
        for topic in TF_TOPICS:
            static = topic == "/tf_static"
            profile = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=100,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=(
                    DurabilityPolicy.TRANSIENT_LOCAL if static else DurabilityPolicy.VOLATILE
                ),
            )
            try:
                out.append(
                    self.node.create_subscription(
                        message, topic, _tf_reader(self.tf_seen, static), profile
                    )
                )
            except Exception as exc:
                self._failed("sample", exc, topic)
        return out

    def _message(self, type_name: str, topic: str) -> Any:
        try:
            from rosidl_runtime_py.utilities import get_message

            return get_message(type_name)
        except Exception as exc:
            self.system.rates.append(
                probe.TopicRate(
                    topic=topic,
                    reason=f"the type {type_name} is not on this path: {type(exc).__name__}",
                )
            )
            return None

    def _sample(self) -> float:
        started = time.monotonic()
        self._spin(self.window)
        return time.monotonic() - started

    def _rates(self, elapsed: float) -> None:
        window = elapsed if elapsed > 0 else self.window
        for topic in sorted(self.counts):
            count = self.counts[topic]
            self.system.rates.append(
                probe.TopicRate(
                    topic=topic,
                    count=count,
                    window_s=round(window, 3),
                    hz=round(count / window, 3) if window > 0 else None,
                    reason="" if window > 0 else "the sampling window was zero",
                )
            )

    def _tf_links(self) -> None:
        for (parent, child, static), _seen in sorted(self.tf_seen.items()):
            topic = "/tf_static" if static else "/tf"
            senders = self.tf_publishers.get(topic, [])
            broadcaster = senders[0] if len(senders) == 1 else None
            reason = ""
            if broadcaster is None:
                reason = (
                    f"{len(senders)} nodes publish {topic}, so the sender of one message is "
                    "not knowable from the message"
                )
            self.system.tf.append(
                probe.TfLink(
                    parent=parent,
                    child=child,
                    static=static,
                    broadcaster=broadcaster,
                    broadcaster_reason=reason,
                )
            )

    # -----------------------------------------------------------------
    # per node detail

    def _details(self, graph: dict[str, dict[str, Any]]) -> None:
        for fqn in sorted(graph):
            entry = graph[fqn]
            name = entry["name"]
            internal = probe.internal_name(name)
            state = None if internal else self._lifecycle(fqn, entry)
            values, read, reason = ((), True, "") if internal else self._parameters(fqn)
            if internal:
                reason = "tooling attached to the graph, not read"
                read = False
            self.system.nodes.append(
                probe.ObservedNode(
                    name=name,
                    namespace=entry["namespace"],
                    fqn=fqn,
                    lifecycle_state=state,
                    parameters=tuple(values),
                    parameters_read=read,
                    parameters_reason=reason,
                    internal=internal,
                )
            )
            if not entry["complete"]:
                self.system.notes.append(f"the endpoints of {fqn} were read only in part")

    def _parameters(self, fqn: str) -> tuple[list[Any], bool, str]:
        from rcl_interfaces.srv import GetParameters, ListParameters

        listed = self._call(ListParameters, f"{fqn}/list_parameters", ListParameters.Request())
        if listed is None:
            return [], False, "the node did not answer list_parameters within the timeout"
        names = sorted(listed.result.names)
        if not names:
            return [], True, ""
        request = GetParameters.Request()
        request.names = names
        got = self._call(GetParameters, f"{fqn}/get_parameters", request)
        if got is None:
            return [], False, "the node listed its parameters and did not answer get_parameters"
        values = list(got.values)
        if len(values) != len(names):
            # rclcpp answers a request naming one declared and uninitialised parameter with no
            # values at all, so one such parameter hid every other value of the node and the
            # node read as one with no parameters. Asked one by one, only that one is unread.
            values = [self._one_parameter(fqn, name) for name in names]
        out = []
        for name, value in zip(names, values, strict=True):
            typed, plain = ("", None) if value is None else _parameter_value(value)
            out.append(
                probe.Parameter(
                    name=name,
                    value=plain,
                    type=typed,
                    read=typed != "",
                )
            )
        return out, True, ""

    def _one_parameter(self, fqn: str, name: str) -> Any:
        from rcl_interfaces.srv import GetParameters

        request = GetParameters.Request()
        request.names = [name]
        got = self._call(GetParameters, f"{fqn}/get_parameters", request)
        return got.values[0] if got is not None and len(got.values) == 1 else None

    def _lifecycle(self, fqn: str, entry: dict[str, Any]) -> str | None:
        service = f"{fqn}/get_state"
        if entry["service_types"].get(service) != LIFECYCLE_GET_STATE:
            return None
        from lifecycle_msgs.srv import GetState

        answer = self._call(GetState, service, GetState.Request())
        if answer is None:
            return None
        return str(answer.current_state.label)

    def _call(self, service_type: Any, name: str, request: Any) -> Any:
        """One read only service call, bounded twice: waiting for it and waiting on it."""
        import rclpy

        client = None
        try:
            client = self.node.create_client(service_type, name)
            if not client.wait_for_service(timeout_sec=self.service_timeout):
                return None
            future = client.call_async(request)
            rclpy.spin_until_future_complete(
                self.node, future, timeout_sec=self.service_timeout * 2
            )
            return future.result() if future.done() else None
        except Exception as exc:
            self._failed("read", exc, name)
            return None
        finally:
            with contextlib.suppress(Exception):
                if client is not None:
                    self.node.destroy_client(client)

    # -----------------------------------------------------------------

    def _spin(self, seconds: float) -> None:
        import rclpy

        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            try:
                rclpy.spin_once(self.node, timeout_sec=SPIN_SLICE)
            except Exception as exc:
                self._failed("sample", exc, "spin")
                return

    def _failed(self, phase: str, exc: BaseException, about: str = "") -> None:
        self.system.exceptions.append(
            probe.ProbeException(
                phase=phase,
                exc_type=type(exc).__name__,
                message=str(exc) or type(exc).__name__,
                about=about,
            )
        )


def header_stamp(raw: bytes) -> int | None:
    """Read only CDR encodings whose first field is known to be a Header."""
    if len(raw) < 12 or raw[:2] not in (b"\x00\x00", b"\x00\x01"):
        return None
    order = "little" if raw[1] == 1 else "big"
    sec = int.from_bytes(raw[4:8], order, signed=True)
    nanosec = int.from_bytes(raw[8:12], order)
    return sec * 1_000_000_000 + nanosec


def _counter(counts: dict[str, int], topic: str, samples: probe.TopicStamps | None = None):
    def _seen(raw: bytes) -> None:
        counts[topic] = counts.get(topic, 0) + 1
        if samples is not None and len(samples.stamp_ns) < probe.STAMP_SAMPLES:
            received = time.time_ns()
            stamp = header_stamp(raw)
            if stamp is not None:
                samples.receive_ns.append(received)
                samples.stamp_ns.append(stamp)

    return _seen


def _tf_reader(seen: dict[tuple[str, str, bool], str | None], static: bool):
    def _read(message: Any) -> None:
        for transform in message.transforms:
            parent = str(transform.header.frame_id).lstrip("/")
            child = str(transform.child_frame_id).lstrip("/")
            if parent and child:
                seen.setdefault((parent, child, static), None)

    return _read


def _qos(profile: Any) -> probe.Qos | None:
    history = _HISTORY.get(int(getattr(profile.history, "value", profile.history)))
    reliability = _RELIABILITY.get(int(getattr(profile.reliability, "value", profile.reliability)))
    durability = _DURABILITY.get(int(getattr(profile.durability, "value", profile.durability)))
    liveliness = _LIVELINESS.get(int(getattr(profile.liveliness, "value", profile.liveliness)))
    depth = int(getattr(profile, "depth", 0) or 0)
    found = probe.Qos(
        history=history,
        depth=depth if history == "keep_last" and depth > 0 else None,
        reliability=reliability,
        durability=durability,
        liveliness=liveliness,
    )
    return found if found.stated() else None


def _parameter_value(value: Any) -> tuple[str, Any]:
    """A ParameterValue as a python value and the type name rcl gave it.

    An unset parameter has no value, so it reports no type and the caller marks it unread.
    """
    known = _PARAM_TYPES.get(int(value.type))
    if known is None:
        return "", None
    name, attribute = known
    plain = _plain(getattr(value, attribute))
    if plain is None and name != "string":
        return "", None
    return name, plain


def _plain(value: Any) -> Any:
    if isinstance(value, bool) or value is None:
        return value
    if isinstance(value, int):
        return int(value)
    if isinstance(value, float):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, str):
        return value
    if isinstance(value, (bytes, bytearray)):
        return [int(item) for item in value]
    if isinstance(value, (list, tuple)) or hasattr(value, "tolist"):
        items = list(value.tolist()) if hasattr(value, "tolist") else list(value)
        out = [_plain(item) for item in items]
        return None if any(item is None for item in out) else out
    return str(value)


def _join(namespace: str, name: str) -> str:
    text = namespace or "/"
    if not text.startswith("/"):
        text = "/" + text
    return name if not name else (text.rstrip("/") + "/" + name)


def main() -> int:
    request = json.loads(sys.stdin.read() or "{}")
    runner = Probe(request)
    try:
        system = runner.run()
    except BaseException as exc:
        system = runner.system
        system.reached = False
        runner._failed("attach", exc)
        system.sort()
    sys.stdout.write("\n" + probe.MARKER + "\n")
    sys.stdout.write(model.dumps(probe.encode_system(system)))
    sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
