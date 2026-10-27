"""The observed layer: what one running system reports about itself.

The parent process never imports rclpy. It runs `_probe_child.py` under a sourced ROS
environment and reads one json object back, exactly the way the launch resolver runs a
launch description, so the core keeps pyyaml as its only runtime dependency and a middleware
that hangs or aborts costs one probe instead of the run.

What the child reports, and what it refuses to report
-----------------------------------------------------

Every node with its namespace, every topic, service and action endpoint with the type and
the full QoS profile the middleware states for it, the effective parameter values of every
node that answers its parameter services, the frames seen on `/tf` and `/tf_static` with
their parents, the lifecycle state of every node that has the service, and a sampled rate per
topic over a bounded window.

Three refusals shape the record:

* A property the middleware reports as unknown stays null. An rmw default written in as a
  value is manufactured drift, and QoS is where that is easiest to do by accident.
* A frame is attributed to a broadcaster only when one node publishes the carrying topic.
  With two publishers on `/tf` nothing can say which of them sent a message, so the
  broadcaster is unknown and no edge is drawn.
* A node that did not answer its parameter services is recorded as unread, never as a node
  with no parameters.

Rates, message counts and window durations are volatile. They sit under `volatile` in every
fact, so two probes of the same healthy system produce identical canonical bytes.

What the probe does to the system: nothing. It creates one node with rosout and the
parameter services turned off, subscribes, reads the graph, and calls only
`list_parameters`, `get_parameters` and `get_state`, which are the three read only services
a ROS node offers. It creates no publisher, sends no goal and calls no service that changes
state. One publisher exists that it did not ask for, the `/parameter_events` publisher every
rclpy node carries, and it never sends a message because nothing sets a parameter on the
probe node. The probe therefore appears in its own record as a publisher on that topic.

Where this layer stops. An observed endpoint edge is keyed over the running system, and the
coded edge for the same endpoint is keyed over the source symbol and the raw name that
created it, which no probe can produce. Binding the two onto one edge is a join step and
`rosio/join.py` owns it. Until it lands, `qos.differ`, `type.differ` and
`rate.below_declared` see one layer per edge and stay silent, and `endpoint.observed_only`
would report every endpoint of a node whose source was also read.
"""

from __future__ import annotations

import json
import os
import shlex
import subprocess
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from rosio import ids, model
from rosio.collect import CollectContext, proc, register
from rosio.errors import CollectorError
from rosio.resolve import wrappers, yaml_lines

LAYER = model.Layer.OBSERVED.value
NAME = "probe"

CHILD = str(Path(__file__).with_name("_probe_child.py"))

MARKER = "__rosio_probe__"
"""Sentinel before the json object. A login shell prints banners and a middleware writes to
stdout, so the payload is found rather than assumed to be all of stdout."""

DEFAULT_WINDOW = 5.0
"""Seconds of sampling. Long enough to measure a 1 Hz publisher, short enough to attach to
a running robot without becoming an event."""

DEFAULT_SETTLE = 2.0
"""Seconds of discovery before anything is read. A graph read at zero is half a graph."""

DEFAULT_TIMEOUT = 120.0

DEFAULT_SERVICE_TIMEOUT = 2.0

STAMP_SAMPLES = 10000
"""Header stamps kept per topic. A camera at 30 Hz fills it in five minutes, which no probe
window reaches, and a 1 kHz IMU keeps ten seconds of evidence instead of a growing list."""

ENDPOINT_KINDS = (
    "publisher",
    "subscriber",
    "service_server",
    "service_client",
    "action_server",
    "action_client",
)

QOS_KEYS = ("history", "depth", "reliability", "durability")
"""The shape the coded layer produces, so one comparison reads both. A property nobody
stated is null, never an rmw default."""

_EDGES: dict[str, tuple[str, str]] = {
    "publisher": (model.EdgeKind.PUBLISHES.value, model.NodeKind.TOPIC.value),
    "subscriber": (model.EdgeKind.SUBSCRIBES.value, model.NodeKind.TOPIC.value),
    "service_server": (model.EdgeKind.SERVES.value, model.NodeKind.SERVICE.value),
    "service_client": (model.EdgeKind.CALLS.value, model.NodeKind.SERVICE.value),
    "action_server": (model.EdgeKind.SERVES.value, model.NodeKind.ACTION.value),
    "action_client": (model.EdgeKind.CALLS.value, model.NodeKind.ACTION.value),
}

_BUILDERS = {
    model.NodeKind.TOPIC.value: ids.build_topic,
    model.NodeKind.SERVICE.value: ids.build_service,
    model.NodeKind.ACTION.value: ids.build_action,
}

_WANTS = {
    model.NodeKind.TOPIC.value: model.Want.TOPIC_NAME.value,
    model.NodeKind.SERVICE.value: model.Want.SERVICE_NAME.value,
    model.NodeKind.ACTION.value: model.Want.ACTION_NAME.value,
}

INTERNAL_PREFIXES = ("_ros2cli_", "ros2cli_", "rosio_probe")
"""Names of tooling that attached itself to the graph. Reported, marked, never drift."""


# ---------------------------------------------------------------------------
# records


@dataclass(frozen=True)
class Qos:
    """One endpoint's quality of service, as the middleware states it."""

    history: str | None = None
    depth: int | None = None
    reliability: str | None = None
    durability: str | None = None
    liveliness: str | None = None

    def values(self) -> dict[str, Any]:
        out: dict[str, Any] = {key: getattr(self, key) for key in QOS_KEYS}
        if self.liveliness is not None:
            out["liveliness"] = self.liveliness
        return out

    def stated(self) -> bool:
        return any(value is not None for value in self.values().values())

    def compared(self) -> bool:
        """Whether the middleware stated anything the qos comparison reads. Liveliness alone
        is not it: an endpoint with only that stated is a blind spot for every other key."""
        return any(getattr(self, key) is not None for key in QOS_KEYS)


@dataclass(frozen=True)
class Endpoint:
    """One end of one connection, owned by one running node."""

    kind: str
    node: str
    name: str
    type: str | None = None
    qos: Qos | None = None
    qos_reason: str = ""


@dataclass(frozen=True)
class Parameter:
    name: str
    value: Any = None
    type: str = ""
    read: bool = True


@dataclass(frozen=True)
class ObservedNode:
    """One running node. `fqn` is what the middleware answers to, so it is identity."""

    name: str
    namespace: str
    fqn: str
    lifecycle_state: str | None = None
    parameters: tuple[Parameter, ...] = ()
    parameters_read: bool = True
    parameters_reason: str = ""
    internal: bool = False


@dataclass(frozen=True)
class TfLink:
    """One parent to child pair seen on the tf topics, with who sent it when that is
    knowable. Two publishers on one topic make the sender unknowable, and unknown is then
    the only honest answer."""

    parent: str
    child: str
    static: bool = False
    broadcaster: str | None = None
    broadcaster_reason: str = ""


@dataclass(frozen=True)
class TopicRate:
    """A sampled rate. Volatile by construction, so it never enters compared data."""

    topic: str
    count: int = 0
    window_s: float = 0.0
    hz: float | None = None
    reason: str = ""


@dataclass
class TopicStamps:
    """Bounded raw samples, retained only between the child and collector."""

    topic: str
    receive_ns: list[int] = field(default_factory=list)
    stamp_ns: list[int] = field(default_factory=list)


@dataclass(frozen=True)
class ProbeException:
    phase: str
    exc_type: str
    message: str
    about: str = ""


@dataclass
class ObservedSystem:
    """Everything one probe of one running system saw."""

    nodes: list[ObservedNode] = field(default_factory=list)
    endpoints: list[Endpoint] = field(default_factory=list)
    tf: list[TfLink] = field(default_factory=list)
    rates: list[TopicRate] = field(default_factory=list)
    stamps: list[TopicStamps] = field(default_factory=list)
    exceptions: list[ProbeException] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    window_s: float = DEFAULT_WINDOW
    settle_s: float = DEFAULT_SETTLE
    domain_id: str = ""
    localhost_only: str = ""
    reached: bool = True
    rmw: str = ""

    def sort(self) -> ObservedSystem:
        self.nodes.sort(key=lambda item: (item.fqn, item.name))
        self.endpoints.sort(key=lambda item: (item.node, item.kind, item.name, item.type or ""))
        self.tf.sort(key=lambda item: (item.child, item.parent, item.static))
        self.rates.sort(key=lambda item: item.topic)
        self.stamps.sort(key=lambda item: item.topic)
        self.exceptions.sort(key=lambda item: (item.phase, item.exc_type, item.about, item.message))
        self.notes = sorted(dict.fromkeys(self.notes))
        return self

    def node_names(self) -> list[str]:
        return sorted(item.fqn for item in self.nodes)


# ---------------------------------------------------------------------------
# running the child


def plan_passes(
    processes: Sequence[proc.RosProcess], forced: Sequence[str] | None = None
) -> list[str]:
    """Unknown environments cannot select a library. With no evidence, use the default."""
    if forced is not None:
        return sorted(set(forced))
    known = {
        row.rmw
        for row in processes
        if (row.is_node or row.is_launcher) and row.rmw not in ("", "unknown")
    }
    return sorted(known or {"default"})


def merge_observations(observations: Mapping[str, Mapping[str, Any]]) -> ObservedSystem:
    """Project only agreed values into the graph, keeping each full view on the run."""
    payloads = [observations[rmw] for rmw in sorted(observations)]
    if not payloads:
        return ObservedSystem(reached=False)
    merged = dict(payloads[0])
    keys = {
        "nodes": ("fqn",),
        "endpoints": ("node", "kind", "name"),
        "tf": ("parent", "child", "static"),
    }
    for collection, identity in keys.items():
        groups: dict[tuple[Any, ...], list[Mapping[str, Any]]] = {}
        for payload in payloads:
            for row in payload[collection]:
                groups.setdefault(tuple(row[key] for key in identity), []).append(row)
        rows = []
        for group in groups.values():
            row = dict(group[0])
            for key in row:
                if any(other.get(key) != row[key] for other in group[1:]):
                    row[key] = None
            if collection == "nodes" and row.get("parameters") is None:
                row["parameters"] = []
                row["parameters_read"] = False
                row["parameters_reason"] = "middleware passes returned different parameters"
            rows.append(row)
        merged[collection] = rows
    for collection in ("rates", "stamps"):
        first = {}
        for payload in payloads:
            for row in payload.get(collection, []):
                first.setdefault(row["topic"], row)
        merged[collection] = [first[topic] for topic in sorted(first)]
    merged["exceptions"] = [row for payload in payloads for row in payload["exceptions"]]
    merged["notes"] = [note for payload in payloads for note in payload["notes"]]
    return decode_system(merged)


def _observation_record(payload: Mapping[str, Any]) -> dict[str, Any]:
    return {
        **{
            key: value
            for key, value in payload.items()
            if key not in ("rates", "stamps", "window_s", "settle_s")
        },
        "volatile": {key: payload[key] for key in ("rates", "window_s", "settle_s")},
    }


def available(ros_setup: Sequence[str] = ()) -> bool:
    """Whether a child sourcing these setup files can import rclpy."""
    script = "python3 -c 'import rclpy'"
    try:
        done = subprocess.run(
            ["bash", "-lc", _sourced(ros_setup, script)],
            capture_output=True,
            timeout=120,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return False
    return done.returncode == 0


def probe(
    *,
    window: float = DEFAULT_WINDOW,
    settle: float = DEFAULT_SETTLE,
    ros_setup: Sequence[str] = (),
    timeout: float | None = None,
    env: Mapping[str, str] | None = None,
    service_timeout: float = DEFAULT_SERVICE_TIMEOUT,
    rmw: str | None = None,
    stamps: bool = False,
) -> ObservedSystem:
    """Attach to the running system for one bounded window and report what it says."""
    window = max(0.0, float(window))
    settle = max(0.0, float(settle))
    budget = float(timeout) if timeout else window + settle + DEFAULT_TIMEOUT
    request = {
        "stamps": stamps,
        "window": window,
        "settle": settle,
        "service_timeout": max(0.1, float(service_timeout)),
    }
    asked = {str(key): str(value) for key, value in (env or {}).items()}
    child_env = dict(os.environ)
    child_env.update(asked)
    if rmw == "default":
        child_env.pop("RMW_IMPLEMENTATION", None)
    elif rmw is not None:
        child_env["RMW_IMPLEMENTATION"] = rmw
    script = f"exec python3 {shlex.quote(CHILD)}"
    if rmw is not None:
        script = (
            "unset RMW_IMPLEMENTATION && " + script
            if rmw == "default"
            else f"export RMW_IMPLEMENTATION={shlex.quote(rmw)} && " + script
        )
    try:
        done = subprocess.run(
            ["bash", "-lc", _sourced(ros_setup, script, asked)],
            input=model.dumps(request),
            capture_output=True,
            text=True,
            timeout=budget,
            env=child_env,
            check=False,
        )
    except subprocess.TimeoutExpired:
        raise CollectorError(
            f"the probe child did not finish within {budget:g} seconds",
            LAYER,
            NAME,
        ) from None
    except (OSError, subprocess.SubprocessError) as exc:
        raise CollectorError(f"the probe child could not be started: {exc}", LAYER, NAME) from exc
    payload = _payload(done.stdout)
    if payload is None:
        tail = " ".join((done.stderr or done.stdout or "").strip().splitlines()[-3:])
        raise CollectorError(
            tail or f"the probe child exited with {done.returncode} and said nothing",
            LAYER,
            NAME,
        )
    return decode_system(payload)


def _sourced(ros_setup: Sequence[str], script: str, env: Mapping[str, str] | None = None) -> str:
    """The child command line. The caller's environment is exported after the setup files,
    because this runs under a login shell and both the profile and a workspace setup file
    can export `ROS_DOMAIN_ID`. A probe attaching to a domain the caller did not ask for is
    the worst thing this collector can do, so an explicit request wins over both."""
    parts = [f". {shlex.quote(str(path))}" for path in ros_setup]
    for key, value in sorted((env or {}).items()):
        if key.isidentifier():
            parts.append(f"export {key}={shlex.quote(value)}")
    parts.append(script)
    return " && ".join(parts)


def asked_env() -> dict[str, str]:
    """The domain and the localhost setting the caller is on.

    Both are passed to the child explicitly, because a workspace setup file that exports
    `ROS_DOMAIN_ID` would otherwise move the probe onto a domain nobody asked about, and on
    a machine attached to a vehicle that is the worst thing this collector can do.
    """
    keys = ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY")
    return {key: os.environ[key] for key in keys if os.environ.get(key)}


def _payload(stdout: str) -> dict[str, Any] | None:
    if MARKER not in stdout:
        return None
    try:
        loaded = json.loads(stdout.rsplit(MARKER, 1)[1])
    except json.JSONDecodeError:
        return None
    return loaded if isinstance(loaded, dict) else None


# ---------------------------------------------------------------------------
# json codec, shared with the child


def encode_system(system: ObservedSystem) -> dict[str, Any]:
    return {
        "nodes": [
            {
                "name": item.name,
                "namespace": item.namespace,
                "fqn": item.fqn,
                "lifecycle_state": item.lifecycle_state,
                "parameters": [
                    {
                        "name": param.name,
                        "value": param.value,
                        "type": param.type,
                        "read": param.read,
                    }
                    for param in item.parameters
                ],
                "parameters_read": item.parameters_read,
                "parameters_reason": item.parameters_reason,
                "internal": item.internal,
            }
            for item in system.nodes
        ],
        "endpoints": [
            {
                "kind": item.kind,
                "node": item.node,
                "name": item.name,
                "type": item.type,
                "qos": None if item.qos is None else item.qos.values(),
                "qos_reason": item.qos_reason,
            }
            for item in system.endpoints
        ],
        "tf": [
            {
                "parent": item.parent,
                "child": item.child,
                "static": item.static,
                "broadcaster": item.broadcaster,
                "broadcaster_reason": item.broadcaster_reason,
            }
            for item in system.tf
        ],
        "rates": [
            {
                "topic": item.topic,
                "count": item.count,
                "window_s": item.window_s,
                "hz": item.hz,
                "reason": item.reason,
            }
            for item in system.rates
        ],
        "stamps": [
            {"topic": item.topic, "receive_ns": item.receive_ns, "stamp_ns": item.stamp_ns}
            for item in system.stamps
        ],
        "exceptions": [
            {
                "phase": item.phase,
                "exc_type": item.exc_type,
                "message": item.message,
                "about": item.about,
            }
            for item in system.exceptions
        ],
        "notes": list(system.notes),
        "window_s": system.window_s,
        "settle_s": system.settle_s,
        "domain_id": system.domain_id,
        "localhost_only": system.localhost_only,
        "reached": system.reached,
        "rmw": system.rmw,
    }


def decode_system(payload: Mapping[str, Any]) -> ObservedSystem:
    system = ObservedSystem(
        nodes=[
            ObservedNode(
                name=str(item.get("name", "")),
                namespace=str(item.get("namespace", "/")),
                fqn=str(item.get("fqn", "")),
                lifecycle_state=item.get("lifecycle_state"),
                parameters=tuple(
                    Parameter(
                        name=str(row.get("name", "")),
                        value=row.get("value"),
                        type=str(row.get("type", "")),
                        read=bool(row.get("read", True)),
                    )
                    for row in item.get("parameters", ())
                ),
                parameters_read=bool(item.get("parameters_read", True)),
                parameters_reason=str(item.get("parameters_reason", "")),
                internal=bool(item.get("internal", False)),
            )
            for item in payload.get("nodes", ())
        ],
        endpoints=[
            Endpoint(
                kind=str(item.get("kind", "")),
                node=str(item.get("node", "")),
                name=str(item.get("name", "")),
                type=item.get("type"),
                qos=_decode_qos(item.get("qos")),
                qos_reason=str(item.get("qos_reason", "")),
            )
            for item in payload.get("endpoints", ())
        ],
        tf=[
            TfLink(
                parent=str(item.get("parent", "")),
                child=str(item.get("child", "")),
                static=bool(item.get("static", False)),
                broadcaster=item.get("broadcaster"),
                broadcaster_reason=str(item.get("broadcaster_reason", "")),
            )
            for item in payload.get("tf", ())
        ],
        rates=[
            TopicRate(
                topic=str(item.get("topic", "")),
                count=int(item.get("count", 0) or 0),
                window_s=float(item.get("window_s", 0.0) or 0.0),
                hz=None if item.get("hz") is None else float(item["hz"]),
                reason=str(item.get("reason", "")),
            )
            for item in payload.get("rates", ())
        ],
        stamps=[
            TopicStamps(str(item["topic"]), list(item["receive_ns"]), list(item["stamp_ns"]))
            for item in payload.get("stamps", ())
        ],
        exceptions=[
            ProbeException(
                phase=str(item.get("phase", "read")),
                exc_type=str(item.get("exc_type", "Exception")),
                message=str(item.get("message", "")),
                about=str(item.get("about", "")),
            )
            for item in payload.get("exceptions", ())
        ],
        notes=[str(note) for note in payload.get("notes", ())],
        window_s=float(payload.get("window_s", DEFAULT_WINDOW) or 0.0),
        settle_s=float(payload.get("settle_s", DEFAULT_SETTLE) or 0.0),
        domain_id=str(payload.get("domain_id", "")),
        localhost_only=str(payload.get("localhost_only", "")),
        reached=bool(payload.get("reached", True)),
        rmw=str(payload.get("rmw", "")),
    )
    return system.sort()


def _decode_qos(payload: Mapping[str, Any] | None) -> Qos | None:
    if not payload:
        return None
    depth = payload.get("depth")
    return Qos(
        history=payload.get("history"),
        depth=None if depth is None else int(depth),
        reliability=payload.get("reliability"),
        durability=payload.get("durability"),
        liveliness=payload.get("liveliness"),
    )


# ---------------------------------------------------------------------------
# the collector


class ProbeCollector:
    """Write one observed system into the atlas.

    `system` is injectable so the writing half is testable without a middleware, which is
    the same split the launch collector uses between the walk and the atlas.
    """

    name = NAME
    layer = LAYER

    def __init__(
        self,
        *,
        window: float = DEFAULT_WINDOW,
        settle: float = DEFAULT_SETTLE,
        system: ObservedSystem | None = None,
        rmws: Sequence[str] | None = None,
        processes: Sequence[proc.RosProcess] | None = None,
        stamps: bool = False,
    ) -> None:
        self.stamps = stamps
        self.window = window
        self.settle = settle
        self.system = system
        self.rmws = rmws
        self.processes = processes

    def collect(self, ctx: CollectContext) -> None:
        passes = []
        observations = {}
        seen: dict[str, list[str]] = {}
        if self.system is not None:
            system = self.system
        else:
            rows = self.processes
            if rows is None:
                rows = proc.scan(domain=proc.probed_domain()) if proc.available() else []
            for rmw in plan_passes(rows, self.rmws):
                try:
                    observed = probe(
                        window=self.window,
                        settle=self.settle,
                        ros_setup=ctx.cfg.ros_setup,
                        env=asked_env(),
                        rmw=rmw,
                        stamps=self.stamps,
                    )
                except CollectorError:
                    passes.append({"rmw": rmw, "state": "refused", "reason": "child_failed"})
                    continue
                if not observed.reached:
                    reason = (
                        "rmw_library_missing"
                        if "rmw_library_missing" in observed.notes
                        else "attach_failed"
                    )
                    passes.append({"rmw": rmw, "state": "refused", "reason": reason})
                    continue
                passes.append({"rmw": rmw, "state": "completed"})
                observations[rmw] = encode_system(observed)
                for node in observed.nodes:
                    seen.setdefault(node.fqn, []).append(rmw)
            system = merge_observations(observations)
        system.sort()
        owners = node_ids(system)
        stable = {
            fqn: node_id if ids.is_synthetic(node_id) else fqn for fqn, node_id in owners.items()
        }
        builder = ctx.builder
        profile = ids.build_profile(ctx.profile) if ctx.profile else model.ANY_PROFILE
        run = builder.add_run(
            LAYER,
            profile=profile if ctx.profile else None,
            input_digest=ids.digest(
                NAME,
                *[ids.digest_fields(row) for row in passes],
                system.domain_id,
                *sorted(set(stable.values())),
                *sorted(
                    f"{stable.get(item.node, item.node)}|{item.kind}|{item.name}"
                    for item in system.endpoints
                ),
            ),
            roots=ctx.ws.root_hashes(),
            env={
                "ROS_DOMAIN_ID": system.domain_id,
                "ROS_LOCALHOST_ONLY": system.localhost_only,
            },
            predicates={"reached": system.reached},
            exceptions=[
                {
                    "phase": item.phase,
                    "exc_type": item.exc_type,
                    "message": item.message,
                    "about": item.about,
                }
                for item in system.exceptions
            ],
            volatile={
                "window_s": system.window_s,
                "settle_s": system.settle_s,
                "nodes": len(system.nodes),
                "endpoints": len(system.endpoints),
            },
        )
        if passes:
            builder.node(
                run,
                attrs={
                    "middleware_passes": passes,
                    "observations": {
                        rmw: _observation_record(payload) for rmw, payload in observations.items()
                    },
                },
            )
        default_rmw = observations.get("default", {}).get("rmw")
        if default_rmw:
            builder.fact(run, LAYER, profile=profile, run=run, values={"rmw_default": default_rmw})
        if system.notes:
            # Everything the probe knows it did not read. A note that stays in the record
            # and never reaches the atlas is a blind spot nobody can find.
            builder.node(run, attrs={"notes": list(system.notes)})
        self._stamp_checks(ctx, run, profile, system)
        if all(item.internal for item in system.nodes):
            builder.declare_layer(
                LAYER,
                present=False,
                run=run,
                reason=_absent_reason(system),
                root=model.Root.RUNTIME.value,
            )
            return
        builder.declare_layer(LAYER, present=True, run=run, root=model.Root.RUNTIME.value)
        through = _seen_through(owners, seen)
        self._nodes(ctx, run, profile, system, owners, through)
        for node_id, middlewares in sorted(through.items()):
            if not middlewares:
                continue
            fact = builder.atlas.entities[node_id].fact(LAYER, profile)
            values: dict[str, Any] = {"seen_through": middlewares}
            if "rmw" not in fact.values:
                values["rmw"] = middlewares[0] if len(middlewares) == 1 else "unknown"
            builder.fact(node_id, LAYER, profile=profile, values=values)
        self._endpoints(ctx, run, profile, system, owners)
        self._parameters(ctx, run, profile, system, owners)
        self._frames(ctx, run, profile, system, owners)

    # nodes ---------------------------------------------------------------
    def _nodes(
        self,
        ctx: CollectContext,
        run: str,
        profile: str,
        system: ObservedSystem,
        owners: Mapping[str, str],
        through: Mapping[str, list[str]],
    ) -> None:
        builder = ctx.builder
        siblings = _siblings(system)
        for item in system.nodes:
            node_id = owners[item.fqn]
            randomised = ids.is_synthetic(node_id)
            existing = builder.atlas.entities.get(node_id)
            held = existing is not None and existing.fact(LAYER, profile) is not None
            stated = existing is not None and existing.name_source not in (
                None,
                model.NameSource.UNKNOWN.value,
            )
            # The probe observed the name, it did not decide it. A layer that can say where
            # the name comes from keeps that answer, and the process table is the other half
            # of this layer, so the slot belongs to whichever half reached it first.
            # A name one process start decided is not stated at all, or two starts of one
            # system would be two atlases.
            if randomised:
                source = model.NameSource.RUNTIME_RANDOMISED.value
            else:
                source = None if stated else model.NameSource.OBSERVED.value
            builder.node(node_id, name_source=source)
            values: dict[str, Any] = {
                "namespace": ids.normalise_namespace(item.namespace),
                "seen_through": through[node_id],
            }
            if not randomised:
                values["name"] = item.name
            if item.internal:
                values["internal"] = True
            wrapper = wrappers.node_wrapper(item.name, siblings[item.namespace])
            if wrapper:
                values["via_known_wrapper"] = wrapper
            if item.lifecycle_state is not None:
                values["lifecycle_state"] = item.lifecycle_state
            builder.fact(
                node_id,
                LAYER,
                profile=profile,
                run=None if held else run,
                values=values,
                confidence=model.Confidence.CERTAIN.value,
            )
            builder.count_resolved(model.Want.NODE_NAME.value)

    # endpoints -----------------------------------------------------------
    def _endpoints(
        self,
        ctx: CollectContext,
        run: str,
        profile: str,
        system: ObservedSystem,
        owners: Mapping[str, str],
    ) -> None:
        builder = ctx.builder
        rates = {item.topic: item for item in system.rates}
        for item in system.endpoints:
            node_id = owners.get(item.node)
            if node_id is None:
                continue
            binding = _EDGES.get(item.kind)
            if binding is None:
                builder.unresolved(
                    model.Unresolved(
                        want=model.Want.OTHER.value,
                        owner=node_id,
                        expr=f"{item.kind} {item.name}",
                        reason="this endpoint kind is not in the table, so no relation is drawn",
                        role="endpoint",
                        resolvable_by=["add the kind to the endpoint table of this collector"],
                    )
                )
                continue
            edge_kind, entity_kind = binding
            endpoint_id = _BUILDERS[entity_kind](item.name)
            builder.node(endpoint_id)
            builder.fact(
                endpoint_id,
                LAYER,
                profile=profile,
                run=run,
                values={"name": item.name},
                confidence=model.Confidence.CERTAIN.value,
            )
            builder.count_resolved(_WANTS[entity_kind])
            values: dict[str, Any] = {"raw_name": item.name}
            if item.type:
                values["type"] = item.type
                builder.count_resolved(model.Want.TYPE.value)
            if item.qos is not None and item.qos.stated():
                values["qos"] = item.qos.values()
            if item.qos is not None and item.qos.compared():
                builder.count_resolved(model.Want.QOS.value)
            elif entity_kind == model.NodeKind.TOPIC.value:
                # Only a topic end. rmw states no qos for a service or an action in humble,
                # which is a limit of the api and not a blind spot in the running system.
                builder.unresolved(
                    model.Unresolved(
                        want=model.Want.QOS.value,
                        owner=node_id,
                        expr=f"{item.kind} {item.name}",
                        reason=item.qos_reason
                        or "the middleware stated no history, depth, reliability or durability "
                        "for this end",
                        role=item.kind,
                        resolvable_by=["read the endpoint through a middleware that states it"],
                    )
                )
            measured = rates.get(item.name) if item.kind == "publisher" else None
            if measured is not None and measured.hz is not None:
                values["volatile"] = {
                    "rate_hz": measured.hz,
                    "samples": measured.count,
                    "window_s": measured.window_s,
                }
            builder.edge(
                edge_kind,
                node_id,
                endpoint_id,
                key=ids.edge_key(layer=LAYER, profile=profile, kind=item.kind, name=item.name),
                layer=LAYER,
                profile=profile,
                run=run,
                values=values,
                confidence=model.Confidence.CERTAIN.value,
            )

    def _stamp_checks(
        self, ctx: CollectContext, run: str, profile: str, system: ObservedSystem
    ) -> None:
        if not self.stamps and not system.stamps:
            return

        from rosio.collect.stamps import analyze, check_rows, rounded
        from rosio.stamps.model import TopicSeries

        series = {
            row.topic: TopicSeries(
                row.topic,
                "",
                [stamp * 1e-9 for stamp in row.stamp_ns],
                [received * 1e-9 for received in row.receive_ns],
            )
            for row in system.stamps
            if row.stamp_ns
        }
        results, verdict = analyze(series)
        ctx.builder.atlas.entities[run].attrs["volatile"]["stamp_verdict"] = rounded(verdict)
        for topic, entry in sorted(series.items()):
            ctx.builder.fact(
                ids.build_topic(topic),
                LAYER,
                profile=profile,
                run=run,
                values={
                    "volatile": {
                        "stamps": rounded(
                            {
                                "messages": len(entry.stamps),
                                "checks": check_rows(results, topic),
                            }
                        )
                    }
                },
            )

    # parameters ----------------------------------------------------------
    def _parameters(
        self,
        ctx: CollectContext,
        run: str,
        profile: str,
        system: ObservedSystem,
        owners: Mapping[str, str],
    ) -> None:
        builder = ctx.builder
        for item in system.nodes:
            node_id = owners[item.fqn]
            if item.internal:
                # Tooling that attached itself to the graph is not read on purpose, and a
                # decision is not a blind spot.
                continue
            if not item.parameters_read:
                builder.unresolved(
                    model.Unresolved(
                        want=model.Want.PARAM_VALUE.value,
                        owner=node_id,
                        expr="list_parameters",
                        reason=item.parameters_reason
                        or "the node did not answer its parameter services",
                        role="parameters",
                        resolvable_by=["start the node with its parameter services enabled"],
                    )
                )
                continue
            for param in item.parameters:
                param_id = ids.build_param(node_id, param.name)
                entity = builder.node(param_id)
                if entity.state is None:
                    # Only when nothing has stated one: a candidate chain the declared layer
                    # wrote owns the state, and the observed value never overrides it. A
                    # parameter the node holds no value for is a blind spot, not live.
                    state = model.ParamState.EFFECTIVE if param.read else model.ParamState.OTHER
                    builder.set_param(param_id, state=state.value)
                if not param.read:
                    builder.unresolved(
                        model.Unresolved(
                            want=model.Want.PARAM_VALUE.value,
                            owner=param_id,
                            expr=param.name,
                            reason="the node listed the parameter and did not return a value",
                            role="parameter",
                            resolvable_by=["read the parameter while the node is up"],
                        )
                    )
                    continue
                builder.fact(
                    param_id,
                    LAYER,
                    profile=profile,
                    run=run,
                    values={
                        # The declared chain states a path by its root label, and the two
                        # layers must spell one value one way or they disagree about nothing.
                        "value": ctx.ws.stated_value(param.value),
                        # Spelled the way the candidate chain spells it. rcl says double where
                        # the chain says float, and two spellings of one type are a conflict
                        # on every parameter both layers state.
                        "value_type": yaml_lines.type_name(param.value),
                        "name": param.name,
                    },
                    confidence=model.Confidence.CERTAIN.value,
                )
                builder.count_resolved(model.Want.PARAM_VALUE.value)
                builder.edge(
                    model.EdgeKind.HAS_PARAM.value,
                    node_id,
                    param_id,
                    layer=LAYER,
                    profile=profile,
                    run=run,
                    confidence=model.Confidence.CERTAIN.value,
                )

    # frames --------------------------------------------------------------
    def _frames(
        self,
        ctx: CollectContext,
        run: str,
        profile: str,
        system: ObservedSystem,
        owners: Mapping[str, str],
    ) -> None:
        builder = ctx.builder
        for link in system.tf:
            parent = ids.build_frame(link.parent)
            child = ids.build_frame(link.child)
            builder.node(parent)
            builder.node(child)
            topic = "/tf_static" if link.static else "/tf"
            values: dict[str, Any] = {
                "source": "tf_message",
                "parent": link.parent,
                "child": link.child,
                "static": link.static,
                "topic": topic,
            }
            if link.broadcaster:
                values["broadcaster"] = link.broadcaster
            builder.edge(
                model.EdgeKind.TF_PARENT.value,
                child,
                parent,
                key=ids.edge_key(
                    layer=LAYER,
                    profile=profile,
                    topic=topic,
                    pair=f"{link.parent}>{link.child}",
                ),
                layer=LAYER,
                profile=profile,
                run=run,
                values=values,
                confidence=model.Confidence.CERTAIN.value,
            )
            builder.count_resolved(model.Want.FRAME_ID.value)
            node_id = owners.get(link.broadcaster or "")
            if node_id is None:
                builder.unresolved(
                    model.Unresolved(
                        want=model.Want.NODE_NAME.value,
                        owner=child,
                        expr=f"{link.parent} > {link.child} on {topic}",
                        reason=link.broadcaster_reason
                        or "more than one node publishes this topic, so the sender of one "
                        "message is not knowable",
                        role="broadcaster",
                        resolvable_by=["read the frame from the node that states it in source"],
                    )
                )
                continue
            builder.edge(
                model.EdgeKind.BROADCASTS_TF.value,
                node_id,
                child,
                key=ids.edge_key(
                    layer=LAYER,
                    profile=profile,
                    topic=topic,
                    pair=f"{link.parent}>{link.child}",
                ),
                layer=LAYER,
                profile=profile,
                run=run,
                values={"parent": link.parent, "child": link.child, "topic": topic},
                confidence=model.Confidence.CERTAIN.value,
            )


def _absent_reason(system: ObservedSystem) -> str:
    """Why the layer holds nothing. A probe that never attached and a probe that attached to
    an empty graph are two different statements, and only one of them is about the system.
    A graph of tooling alone is an empty one: written as a layer, it reported every node a
    profile launches as launched and not running."""
    if system.reached:
        tooling = " but tooling such as a ros2cli daemon" if system.nodes else ""
        return (
            f"no node{tooling} answered on domain {system.domain_id or 'unset'}, so nothing "
            "of this system is running there. check ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY "
            "against the shell that started it"
        )
    attach = [item for item in system.exceptions if item.phase == "attach"]
    if attach:
        return f"the probe never attached: {attach[0].exc_type}: {attach[0].message}"
    return "the probe never attached to the middleware"


LAUNCH_NODE_PREFIX = "launch_ros_"
"""`launch_ros/ros_adapters.py:61` names the node a running `ros2 launch` holds
`launch_ros_` plus its process id. It is the launcher itself, not a node it starts."""


def internal_name(name: str) -> bool:
    """Whether a node name belongs to tooling that attached itself to the graph."""
    if name.startswith(LAUNCH_NODE_PREFIX) and name[len(LAUNCH_NODE_PREFIX) :].isdigit():
        return True
    return any(name.startswith(prefix) for prefix in INTERNAL_PREFIXES)


def randomised_family(name: str, siblings: Sequence[str] = ()) -> tuple[str, str]:
    """The package and name stem of a node whose name one process start decides, or two
    empty strings.

    `ros2 launch` names its node after its process id, ros2cli after its own, and a transform
    listener with its own spin thread after a pointer. Such a name is never identity (AGENTS
    3.4 rule 2): the node is synthetic over the stem and its namespace, so a restart of the
    system writes the same atlas, and two of one family in one namespace are one node.
    """
    if internal_name(name):
        prefix = next(
            item for item in (LAUNCH_NODE_PREFIX, *INTERNAL_PREFIXES) if name.startswith(item)
        )
        stem = prefix.strip("_")
        return stem, stem
    row = next((row for row in wrappers.NODES if row.stem and row.matches(name, siblings)), None)
    return (row.symbol.split("::")[0], row.stem) if row is not None else ("", "")


def node_ids(system: ObservedSystem) -> dict[str, str]:
    """The node id of every observed name."""
    siblings = _siblings(system)
    out: dict[str, str] = {}
    for item in system.nodes:
        package, stem = randomised_family(item.name, siblings[item.namespace])
        out[item.fqn] = (
            ids.build_node_synthetic(package, stem, item.namespace, ids.role_digest(stem=stem))
            if stem
            else ids.build_node(item.fqn)
        )
    return out


def _siblings(system: ObservedSystem) -> dict[str, list[str]]:
    out: dict[str, list[str]] = {}
    for item in system.nodes:
        out.setdefault(item.namespace, []).append(item.name)
    return out


def _seen_through(owners: Mapping[str, str], seen: Mapping[str, list[str]]) -> dict[str, list[str]]:
    """The middlewares each node was seen through, one row per node id, since the names of
    one randomised family share one."""
    out: dict[str, set[str]] = {}
    for fqn, node_id in owners.items():
        out.setdefault(node_id, set()).update(seen.get(fqn, []))
    return {node_id: sorted(items) for node_id, items in out.items()}


def summary(system: ObservedSystem) -> dict[str, Any]:
    """Counts for a human reading a probe. Every number here is volatile."""
    kinds: dict[str, int] = {}
    for item in system.endpoints:
        kinds[item.kind] = kinds.get(item.kind, 0) + 1
    measured = [item for item in system.rates if item.hz is not None]
    return {
        "nodes": len(system.nodes),
        "endpoints": dict(sorted(kinds.items())),
        "frames": len({item.child for item in system.tf}),
        "topics_sampled": len(measured),
        "parameters": sum(len(item.parameters) for item in system.nodes),
        "exceptions": len(system.exceptions),
    }


def _factory() -> ProbeCollector:
    return ProbeCollector()


register(NAME, _factory)


__all__ = [
    "CHILD",
    "DEFAULT_SETTLE",
    "DEFAULT_WINDOW",
    "ENDPOINT_KINDS",
    "MARKER",
    "QOS_KEYS",
    "Endpoint",
    "ObservedNode",
    "ObservedSystem",
    "Parameter",
    "ProbeCollector",
    "ProbeException",
    "Qos",
    "TfLink",
    "TopicRate",
    "TopicStamps",
    "asked_env",
    "available",
    "decode_system",
    "encode_system",
    "internal_name",
    "node_ids",
    "probe",
    "randomised_family",
    "summary",
]
