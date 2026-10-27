"""The declared layer, launch side: launch walks turned into entities, facts and edges.

The unit of truth here is the profile, not the launch file. A workspace wide union of launch
files describes a system that never runs, so every node set this collector claims carries the
profile it belongs to and the entry point it came from, and the profile entity names those
entry points.

What this module does not do: it never decides a parameter value. It records the ordered
parameter inputs on the walk records and leaves the candidate chain to the parameter
collector, which reads the same walks off `ctx.declared.walks`.

Three shapes are load bearing and easy to get wrong:

* A node whose name launch did not state, or whose name the runtime randomises, gets a
  synthetic id built over a role digest of its own content. Two unnamed nodes in one
  namespace therefore stay two nodes.
* A remap target that does not resolve to an absolute name mints nothing. It stays as the
  raw name on the launch entity and becomes one unresolved entry.
* Lifecycle management arrives two ways: a manager node holding a `node_names` list, and an
  in launch EmitEvent plus OnStateTransition pair with no manager node at all. Both produce
  manages_lifecycle edges, and the mechanism goes in the edge values.

A profile is also composed of entry points with the verb `run`, which are nodes whose whole
identity is in their argv, and this collector reads those too. It owns the
`launch.unloadable` finding, one per file it could not load. A file that raised is a run row
with the note `launch.raised`, and the drift rule of that name builds the finding.
"""

from __future__ import annotations

import dataclasses
import os
import re
from collections.abc import Sequence
from concurrent.futures import Future, ThreadPoolExecutor, wait
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

from rosio import ids, model
from rosio.collect import BUILT_ONLY, CollectContext, register, same_name_in_package, urdf
from rosio.config import Entrypoint, ProfileSpec
from rosio.errors import CollectorError, ContractError
from rosio.resolve import identity, records, ros_args, substitutions
from rosio.workspace import OUTSIDE, Workspace

NAME = "launch_declared"
LAYER = model.Layer.DECLARED.value

UNSPECIFIED_NAME = identity.UNSPECIFIED_NAME
UNSPECIFIED_NAMESPACE = identity.UNSPECIFIED_NAMESPACE
EXTERNAL_TOKEN = identity.EXTERNAL_TOKEN
UNKNOWN_SEGMENT = identity.UNKNOWN_SEGMENT
RANDOMISED_EXECUTABLES = identity.RANDOMISED_EXECUTABLES
USABLE_NAME_SOURCES = identity.USABLE_NAME_SOURCES

FORM_ACTIONS = {
    "node": "Node",
    "lifecycle_node": "LifecycleNode",
    "composable_node": "ComposableNode",
    "process": "ExecuteProcess",
}

LIFECYCLE_LIST_PARAM = "node_names"

RULE_UNLOADABLE = "launch.unloadable"
RULE_RAISED = "launch.raised"
"""The two collector owned rule ids from the committed vocabulary. A file that was refused
by shape and a file that raised while building its description are different repairs, so
they stay two rules."""

REFUSED_ROS1 = "ros1_xml"
REFUSED_MARKUP = "markup_frontend"
REFUSED_UNAVAILABLE = "launch_unavailable"
REFUSED_MISSING = "launch_file_missing"
"""Why a file was not read, recorded beside the message so a rule states what is true about
the file instead of one sentence for three different causes."""

NO_PROFILE = "the configuration declares no profile, so no entry point names a launch file"
"""What `inputs_present` says when nothing tells this collector which system to describe.
A workspace wide union of launch files is refused by contract, so with no profile there is
no input at all, which is an absence and not a pass that stopped."""

LAUNCH_UNAVAILABLE = "LaunchUnavailable"
"""Exception type of the one row that says the layer did not happen. A collapse of the whole
launch machinery is not a per file failure, and a run that reports it per file, or not at
all, hands back a thin atlas that reads as a complete one."""

INCLUDE_UNRESOLVED = "IncludeUnresolved"

_REPR_MARK = " object at 0x"

ACTION_SERVICE_SUFFIXES = ("/_action/send_goal", "/_action/cancel_goal", "/_action/get_result")
ACTION_TOPIC_SUFFIXES = ("/_action/feedback", "/_action/status")
NODE_SERVICE_SUFFIXES = (
    "/describe_parameters",
    "/get_parameter_types",
    "/get_parameters",
    "/list_parameters",
    "/set_parameters",
    "/set_parameters_atomically",
)
BUILTIN_TOPICS = ("/clock", "/diagnostics", "/parameter_events", "/rosout", "/tf", "/tf_static")
"""The endpoint kinds a launch file can be read to state. A remap rule in ROS 2 applies to
every interface kind at once, so the rule itself says nothing about what it renames: only
the two action sub interface families, the per node parameter services and the built in
topics are decidable here. Anything else stays unresolved with its raw name until the coded
or the observed layer says which kind it is. Calling an unknown endpoint a topic is how a
service remap ends up with the wrong entity kind, and a wrong kind is worse than a gap."""


_Identity = identity.Identity


@dataclass(frozen=True)
class _Placed:
    """One resolved node after its launch file, launch entity and identity are known."""

    node: records.ResolvedNode
    launchfile_id: str
    launchent_id: str
    locator: str
    identity: _Identity
    source: model.Source
    confidence: str = model.Confidence.RESOLVED.value
    entry: str = ""


@dataclass(frozen=True)
class _RunNode:
    """One `ros2 run` entry point, with its identity read out of its argv."""

    node: records.ResolvedNode
    identity: _Identity
    entry: str
    source: model.Source | None = None
    exe_path: str | None = None


@dataclass(frozen=True)
class _Refusal:
    """A launch file this collector refused to load, ready to become a finding."""

    file_id: str
    label: str
    message: str
    profile: str
    source: model.Source
    refused_because: str = ""


@dataclass
class _LaunchLayer:
    """The one decision about whether the launch machinery can be used at all.

    Asked once per run and not once per file: an entry point that could not be executed
    because nothing on this box can execute one is a property of the run, and the answer
    also has to reach every entry point, so it is carried rather than asked again.
    """

    ros_setup: tuple[str, ...]
    report: Any = None
    collapsed: bool = False

    def state(self) -> Any:
        if self.report is None:
            self.report = _launch_exec().probe(self.ros_setup)
        return self.report


WALKERS = 4
"""Launch children running at once, never more than the processors that run them. Each
walk is its own process and states nothing about another. The child's timeout is wall time,
so a walk that timed out beside others is walked again alone before a pass reads it, and the
number changes how long a run takes and never what it writes."""


@dataclass
class _Ahead:
    """The walks started ahead of their passes, stopped with the run whichever way it ends."""

    pool: ThreadPoolExecutor
    children: Any

    def stop(self) -> None:
        self.pool.shutdown(wait=False, cancel_futures=True)
        self.children.stop()


def _timed_out(walk: records.LaunchWalk) -> bool:
    return any(failure.exc_type == "TimeoutExpired" for failure in walk.exceptions)


def _started_key(spec: ProfileSpec, entry: Entrypoint, path: str) -> tuple[Any, ...]:
    return (spec.slug, path, tuple(entry.args), tuple(spec.env))


def _walk_options(ctx: CollectContext, spec: ProfileSpec, entry: Entrypoint) -> dict[str, Any]:
    return {
        "args": tuple(entry.args),
        "env": tuple(spec.env),
        "ros_setup": tuple(ctx.cfg.ros_setup),
        "timeout": ctx.cfg.launch_timeout,
        "workspace": ctx.ws,
        "launch_sandbox": ctx.cfg.launch_sandbox,
        "sandbox_command": ctx.launch_sandbox.get("bwrap", ""),
    }


def _start_walks(ctx: CollectContext, layer: _LaunchLayer) -> None:
    """Start the executed walk of every launch entry point of the run, once per run.

    A run with many profiles spends nearly all of this pass waiting on one child at a time.
    Each pass still takes its own walks in profile order, so only the waiting overlaps.
    """
    if "started" in ctx.launch_walks or len(ctx.cfg.profiles) < 2 or not layer.state().ok:
        return
    pool = ThreadPoolExecutor(max_workers=max(1, min(WALKERS, os.cpu_count() or 1)))
    children = _launch_exec().Children()
    ctx.launch_walks["started"] = _Ahead(pool, children)
    index = _launch_index(ctx.ws)
    aliases = _aliases(ctx.ws)
    for spec in sorted(ctx.cfg.profiles, key=lambda item: item.slug):
        for entry in spec.entrypoints:
            if entry.verb != "launch":
                continue
            path = _locate(index, entry.pkg, entry.target, aliases)
            key = None if path is None else _started_key(spec, entry, path)
            if key is None or key in ctx.launch_walks:
                continue
            options = _walk_options(ctx, spec, entry)
            ctx.launch_walks[key] = pool.submit(
                _launch_exec().walk, path, children=children, **options
            )


class LaunchDeclaredCollector:
    """Registered as `launch_declared`. One run and one profile entity per profile."""

    name = NAME
    layer = LAYER

    def inputs_present(self, ctx: CollectContext) -> bool | str:
        """Whether a profile names anything for this pass to walk."""
        return True if ctx.profile or ctx.cfg.profiles else NO_PROFILE

    def collect(self, ctx: CollectContext) -> None:
        sandbox = ""
        if ctx.cfg.launch_sandbox != "off":
            if "bwrap" not in ctx.launch_sandbox:
                ctx.launch_sandbox["bwrap"] = _launch_exec().probe_sandbox(ctx.cfg.ros_setup)
            sandbox = ctx.launch_sandbox["bwrap"]
        if ctx.run:
            ctx.builder.atlas.entities[ctx.run].attrs["predicates"]["launch_sandbox:bwrap"] = False
        if not sandbox and ctx.cfg.launch_sandbox == "required":
            raise CollectorError(_launch_exec().SANDBOX_REQUIRED)
        specs = [ctx.cfg.profile(ctx.profile)] if ctx.profile else list(ctx.cfg.profiles)
        refusals: list[_Refusal] = []
        layer = _LaunchLayer(ros_setup=tuple(ctx.cfg.ros_setup))
        _start_walks(ctx, layer)
        for spec in specs:
            self._profile(ctx, spec, refusals, layer)
        _emit_refusals(ctx, refusals)

    # ------------------------------------------------------------------
    # profile level

    def _profile(
        self,
        ctx: CollectContext,
        spec: ProfileSpec,
        refusals: list[_Refusal],
        layer: _LaunchLayer,
    ) -> None:
        profile_id = ids.build_profile(spec.slug)
        index = _launch_index(ctx.ws)
        aliases = _aliases(ctx.ws)
        walks: list[records.LaunchWalk] = []
        entry_files: list[str] = []
        problems: list[dict[str, Any]] = []
        missing: list[str] = []
        other_verbs: list[str] = []
        runs: list[_RunNode] = []

        for entry in spec.entrypoints:
            if entry.verb == "run":
                other_verbs.append(entry.label)
                runs.append(_run_entry(ctx, spec, entry))
                continue
            if entry.verb != "launch":
                other_verbs.append(entry.label)
                continue
            path = _locate(index, entry.pkg, entry.target, aliases)
            if path is None:
                missing.append(entry.label)
                problems.append(
                    {
                        "path": f"{entry.pkg}/{entry.target}",
                        "phase": "import",
                        "exc_type": "MissingLaunchFile",
                        "message": f"no launch file in the workspace matches {entry.label}",
                        "note": RULE_UNLOADABLE,
                        "refused_because": REFUSED_MISSING,
                    }
                )
                continue
            walk = self._walk(ctx, spec, entry, path, layer)
            if walk is None:
                continue
            entry_files.append(path)
            walks.append(walk)
            problems.extend(_include_problems(ctx.ws, walk))
            for failure in walk.exceptions:
                row = {
                    "path": _rel_or_path(ctx.ws, failure.path, ctx.cfg.ros_setup),
                    "phase": failure.phase,
                    "exc_type": failure.exc_type,
                    "message": relabel_searched(ctx.ws, failure.message, ctx.cfg.ros_setup),
                }
                if failure.exc_type == "ModuleNotFoundError":
                    row["message"] = relabel_module(row["message"], ctx.cfg.ros_setup)
                if failure.kept:
                    row["kept"] = failure.kept
                if failure.env:
                    row["env"] = _unset_variables(failure, spec, ctx.cfg.ros_setup)
                absent = _missing_file(ctx.ws, failure)
                if absent:
                    row["missing"] = absent
                if _unrun(failure):
                    # A file only a process writes is the walk's blind spot, stated by the
                    # unresolved entry of this file, and nothing the launch file can repair.
                    problems.append(row)
                    continue
                problems.append({**row, "note": RULE_RAISED})
            if not walk.loadable:
                problems.append(
                    {
                        "path": _rel_or_path(ctx.ws, walk.entry_path),
                        "phase": "import",
                        "exc_type": "Unloadable",
                        # The same sentence the finding carries. Two wordings for one
                        # refusal gave the collector and the rule two fingerprints.
                        "message": ", ".join(walk.notes)
                        or "the file is not a ROS 2 loadable launch file",
                        "note": RULE_UNLOADABLE,
                        "refused_because": _refused_because(walk),
                    }
                )
                refusals.append(
                    _Refusal(
                        file_id=_launchfile_id(ctx.ws, walk.entry_path),
                        label=_rel_or_path(ctx.ws, walk.entry_path),
                        message=", ".join(walk.notes)
                        or "the file is not a ROS 2 loadable launch file",
                        profile=profile_id,
                        source=_source(ctx.ws, walk.entry_path, "import"),
                        refused_because=_refused_because(walk),
                    )
                )

        if layer.collapsed:
            problems.append(
                {
                    "path": _LAUNCH_LAYER_PATH,
                    "phase": "import",
                    "exc_type": LAUNCH_UNAVAILABLE,
                    "message": layer.state().summary,
                }
            )
        launchfile_ids = [_launchfile_id(ctx.ws, path) for path in entry_files]
        predicates = _predicates(ctx.ws, walks)
        input_digest = _input_digest(ctx, spec, walks, launchfile_ids, predicates)
        # Sandbox availability is an input difference, never an identity component.
        sandbox_used = any(walk.sandboxed for walk in walks)
        predicates["launch_sandbox:bwrap"] = sandbox_used
        if ctx.run:
            run_predicates = ctx.builder.atlas.entities[ctx.run].attrs["predicates"]
            run_predicates["launch_sandbox:bwrap"] |= sandbox_used
        run = ctx.builder.add_run(
            LAYER,
            profile=profile_id,
            input_digest=input_digest,
            roots=ctx.ws.root_hashes(),
            env=ctx.ws.stated_value(dict(spec.env)),
            predicates=predicates,
            exceptions=sorted(problems, key=_problem_key),
        )
        ctx.builder.declare_layer(LAYER, present=True, run=run)
        late = ctx.builder.atlas.entities[run].attrs["exceptions"]
        for walk in walks:
            for package, share in sorted(walk.underlay.items()):
                held = ctx.ws.packages.get(package)
                attrs: dict[str, Any] = {
                    "share_read_from": share_tree(ctx.ws, share, ctx.cfg.ros_setup)
                }
                if held is not None and held.share_path:
                    attrs["install_not_sourced"] = True
                built = ctx.ws.unnamed_build(share)
                if built:
                    attrs["share_build"] = built
                ctx.builder.node(ids.build_pkg(package), attrs=attrs)

        ctx.builder.add_profile(
            spec.slug,
            label=spec.label or spec.slug,
            entry=sorted(dict.fromkeys(launchfile_ids)),
            entry_kind="launch" if not other_verbs else "mixed",
            inputs={
                "args": ctx.ws.stated_value(
                    [[key, value] for entry in spec.entrypoints for key, value in entry.args]
                ),
                "env": ctx.ws.stated_value([[key, value] for key, value in spec.env]),
                "ros_setup": ctx.ws.stated_by_content(ctx.cfg.ros_setup),
                "run_entrypoints": sorted(other_verbs),
            },
        )

        for label in missing:
            # A profile that names a file no package holds read nothing for that entry point,
            # and the rate has to fall with it rather than stay perfect over what was read.
            ctx.builder.unresolved(
                model.Unresolved(
                    want=model.Want.OTHER.value,
                    owner=profile_id,
                    expr=label,
                    reason=REFUSED_MISSING,
                    role="entrypoint",
                    resolvable_by=["name a launch file a package in the roots holds"],
                )
            )

        placed: list[_Placed] = []
        for walk in walks:
            try:
                placed.extend(self._emit_file(ctx, spec, walk, run, late))
            except ContractError as exc:
                late.append(
                    {
                        "path": _rel_or_path(ctx.ws, walk.entry_path),
                        "phase": "visit",
                        "exc_type": type(exc).__name__,
                        "message": str(exc),
                    }
                )
        for row in placed:
            self._emit_node(ctx, spec, run, row, late)
        for entry_node in runs:
            self._emit_run_node(ctx, spec, run, entry_node, late)
        self._emit_lifecycle(ctx, spec, run, placed, late)
        self._emit_models(ctx, spec, run, placed, late)

        ctx.builder.fact(
            profile_id,
            LAYER,
            profile=profile_id,
            run=run,
            values={
                "entrypoints": [entry.label for entry in spec.entrypoints],
                "launch_files": sorted(dict.fromkeys(launchfile_ids)),
                "declared_nodes": len(placed) + len(runs),
                "run_nodes": len(runs),
                "missing_targets": sorted(missing),
                "unloadable": sorted(
                    _rel_or_path(ctx.ws, walk.entry_path) for walk in walks if not walk.loadable
                ),
            },
            confidence=model.Confidence.RESOLVED.value,
            evidence=[_source(ctx.ws, path) for path in sorted(dict.fromkeys(entry_files))],
        )
        # Every row of this run passes here, and a message names whatever path it raised on.
        for row in late:
            row["message"] = ctx.ws.stated_message(row["message"], ctx.cfg.ros_setup)
        late.sort(key=_problem_key)

    def _emit_models(
        self,
        ctx: CollectContext,
        spec: ProfileSpec,
        run: str,
        placed: Sequence[_Placed],
        late: list[dict[str, Any]],
    ) -> None:
        """The robot models this profile hands its nodes through a xacro command.

        The urdf collector expands every model with its own defaults, for every profile at
        once. A launch line that passes arguments changes the model the profile runs:
        turtlebot3 passes `namespace:=tb3/` and its state publisher then names `tb3/base_scan`,
        not `base_scan`. So the profile states the model it loads in its own slot. An argument
        the walk could not perform is a blind spot, and the frames it would name are never
        guessed.
        """
        profile_id = ids.build_profile(spec.slug)
        loaded: dict[tuple[str, tuple[tuple[str, str], ...]], model.Source] = {}
        for row in placed:
            # a site whose condition read false never loads its model in this profile
            if row.node.attrs.get("unreachable") or any(
                gate.outcome == "false" for gate in row.node.gates
            ):
                continue
            for read in row.node.file_reads:
                if read.role != "robot_model" or not _in_workspace(ctx.ws, read.path):
                    continue
                if read.blind:
                    ctx.builder.unresolved(
                        model.Unresolved(
                            want=model.Want.FRAME_ID.value,
                            owner=ctx.ws.file_id(read.path),
                            expr=ctx.ws.stated_value(read.blind, ctx.cfg.ros_setup),
                            reason="xacro_argument_unresolved",
                            role="xacro",
                            resolvable_by=["state the launch argument the command reads"],
                            evidence=[row.source],
                        )
                    )
                elif read.xacro_args:
                    loaded.setdefault((read.path, read.xacro_args), row.source)
        for (path, args), source in sorted(loaded.items(), key=lambda item: item[0]):
            try:
                urdf.emit_loaded(ctx, run, profile_id, path, args)
            except (CollectorError, ContractError) as exc:
                file_id = ctx.ws.file_id(path)
                message = ctx.ws.stated_message(str(exc).replace(path, file_id), ctx.cfg.ros_setup)
                late.append(
                    {
                        "path": _problem_path(source.file),
                        "phase": "visit",
                        "exc_type": type(exc).__name__,
                        "message": message,
                    }
                )
                # the frames this profile runs are unknown, and the blind spot says so
                after_build = (
                    not Path(path).exists() and bool(same_name_in_package(ctx.ws, path))
                ) or bool(urdf.after_build(ctx.ws, str(exc)))
                ctx.builder.unresolved(
                    model.Unresolved(
                        want=model.Want.FRAME_ID.value,
                        owner=file_id,
                        expr=ctx.ws.stated_value(
                            " ".join(["xacro", file_id, *(f"{k}:={v}" for k, v in args)]),
                            ctx.cfg.ros_setup,
                        ),
                        reason=BUILT_ONLY if after_build else "xacro_failed",
                        role="xacro",
                        resolvable_by=[
                            "build the workspace and add its install tree as a root"
                            if after_build
                            else "fix what the xacro tool refused"
                        ],
                        evidence=[source],
                    )
                )

    # ------------------------------------------------------------------
    # walks

    def _walk(
        self,
        ctx: CollectContext,
        spec: ProfileSpec,
        entry: Entrypoint,
        path: str,
        layer: _LaunchLayer,
    ) -> records.LaunchWalk | None:
        wanted = (spec.slug, path, tuple(entry.args))
        for existing in ctx.declared.walks:
            # An argument set is part of the walk: a walk made with other arguments is a
            # different resolution of the same file, never a reusable one.
            if (existing.profile, existing.entry_path, existing.args) == wanted:
                return existing
        walk = self._resolve(ctx, spec, entry, path, layer)
        # A tool message names where this machine holds the roots, and it reaches the run
        # rows, the findings and the blind spots, so the mount point is stated by its label.
        walk.exceptions = [
            dataclasses.replace(
                failure, message=ctx.ws.stated_message(failure.message, ctx.cfg.ros_setup)
            )
            for failure in walk.exceptions
        ]
        ctx.declared.walks.append(walk)
        return walk

    def _resolve(
        self,
        ctx: CollectContext,
        spec: ProfileSpec,
        entry: Entrypoint,
        path: str,
        layer: _LaunchLayer,
    ) -> records.LaunchWalk:
        package, _relative = ctx.ws.rel(path)
        common = {
            "entry_path": path,
            "profile": spec.slug,
            "entry_pkg": package,
            "args": tuple(entry.args),
            "env": tuple(spec.env),
        }
        # A `.launch` suffix is not ROS 1: ros_gz ships ROS 2 XML under it. The walker decides
        # by shape.
        walker = _launch_exec()
        state = layer.state()
        try:
            started = ctx.launch_walks.pop(_started_key(spec, entry, path), None)
            walk = started.result() if state.ok and started is not None else None
            if state.ok and (walk is None or _timed_out(walk)):
                if walk is not None:
                    # Alone: the child's timeout is wall time, so the walks still queued
                    # finish first and only this one competes for the box.
                    wait([item for item in ctx.launch_walks.values() if isinstance(item, Future)])
                walk = walker.walk(path, **_walk_options(ctx, spec, entry))
            elif not state.ok:
                # Nothing on this box can build a launch description, so the source reader
                # is a lower bound over a file that was never loaded. Saying so on every
                # entry point is the difference between a thin atlas and a silent one.
                layer.collapsed = True
                walk = walker.walk_ast(path, args=tuple(entry.args), workspace=ctx.ws)
                walk.loadable = False
                walk.notes.append(state.note)
        except Exception as exc:  # one entry point that raises must not empty the layer
            return records.LaunchWalk(
                strategy="refused",
                exceptions=[
                    records.WalkException(
                        path=path,
                        phase="describe",
                        exc_type=type(exc).__name__,
                        message=str(exc),
                    )
                ],
                **common,
            )
        if not walk.profile:
            walk.profile = spec.slug
        return walk

    # ------------------------------------------------------------------
    # launch files

    def _emit_file(
        self,
        ctx: CollectContext,
        spec: ProfileSpec,
        walk: records.LaunchWalk,
        run: str,
        late: list[dict[str, Any]],
    ) -> list[_Placed]:
        profile_id = ids.build_profile(spec.slug)
        launchfile_id = _launchfile_id(ctx.ws, walk.entry_path)
        package, relative = ctx.ws.rel(walk.entry_path)
        pkg_id = _pkg_entity(ctx, package)
        file_id = _file_entity(ctx, walk.entry_path, loadable=walk.loadable)
        ctx.builder.node(launchfile_id, label=relative, attrs={})
        _attr_once(ctx.builder.node(launchfile_id), "loadable", walk.loadable)
        ctx.builder.edge(model.EdgeKind.CONTAINS.value, pkg_id, launchfile_id, layer=LAYER)
        ctx.builder.edge(model.EdgeKind.CONTAINS.value, pkg_id, file_id, layer=LAYER)
        values: dict[str, Any] = {"strategy": walk.strategy, "loadable": walk.loadable}
        if _complete(walk):
            values.update(
                {
                    "args": ctx.ws.stated_value([[key, value] for key, value in walk.args]),
                    "node_count": len(walk.nodes),
                    "raised": [failure.exc_type for failure in walk.exceptions],
                    "notes": list(walk.notes),
                    "arg_declarations": sorted(item.name for item in walk.arg_declarations),
                    "required_args": sorted(
                        item.name for item in walk.arg_declarations if item.default is None
                    ),
                }
            )
        else:
            # A walk that stopped early saw fewer declarations and fewer nodes than one that
            # finished. Every list it could produce is a lower bound, and the slot is written
            # once, so it states only what it knows completely: that it stopped, and where.
            # The reason and the argument that stopped it are on the run's exception rows.
            values["partial"] = True
            values["stopped_at"] = sorted({failure.phase for failure in walk.exceptions}) or [
                walk.strategy
            ]
        ctx.builder.fact(
            launchfile_id,
            LAYER,
            profile=profile_id,
            run=run,
            values=values,
            confidence=model.Confidence.RESOLVED.value,
            evidence=[_source(ctx.ws, walk.entry_path)],
        )
        self._emit_includes(ctx, spec, walk, run, launchfile_id)
        if walk.strategy == "refused" or walk.exceptions:
            ctx.builder.unresolved(
                model.Unresolved(
                    want=model.Want.OTHER.value,
                    owner=launchfile_id,
                    expr=f"{relative} {walk.strategy}",
                    reason=_refusal_reason(walk),
                    role="entrypoint",
                    resolvable_by=[
                        "probe the running system, where the process has written the file"
                        if walk.exceptions and all(_unrun(item) for item in walk.exceptions)
                        else "supply the arguments and the environment the file needs"
                    ],
                    evidence=[_source(ctx.ws, walk.entry_path)],
                )
            )
        else:
            ctx.builder.count_resolved(model.Want.OTHER.value)

        placed: list[_Placed] = []
        ordered = sorted(walk.nodes, key=_node_order)
        for node, steps in zip(ordered, _distinct_steps(ordered), strict=True):
            locator = "/".join(f"{step.action}({step.disc})" for step in steps)
            launchent_id = ids.build_launchent(package or ids.REPO_PKG, relative, steps)
            source = _held_source(ctx.ws, walk, node.site, node.entity_path, locator)
            try:
                identity = _identity(node, executed=walk.strategy == "executed")
            except ContractError as exc:
                late.append(
                    {
                        "path": _problem_path(source.file),
                        "phase": "visit",
                        "exc_type": type(exc).__name__,
                        "message": str(exc),
                    }
                )
                continue
            placed.append(
                _Placed(
                    node=node,
                    launchfile_id=launchfile_id,
                    launchent_id=launchent_id,
                    locator=locator,
                    identity=identity,
                    source=source,
                    confidence=_confidence(walk),
                )
            )
        return placed

    def _emit_includes(
        self,
        ctx: CollectContext,
        spec: ProfileSpec,
        walk: records.LaunchWalk,
        run: str,
        launchfile_id: str,
    ) -> None:
        profile_id = ids.build_profile(spec.slug)
        paths: dict[tuple[str, ...], set[str]] = {}
        # One file included at one trail with two argument sets is two sites: nav2's multi
        # robot sim includes a bringup per robot, and gz_sim is included as server and client.
        arg_sets: dict[tuple[tuple[str, ...], str], set[str]] = {}
        child_sets: dict[str, set[str]] = {}
        for include in walk.includes:
            paths.setdefault(include.entity_path, set()).add(include.path)
            stated = model.dumps(_include_args(ctx.ws, include))
            arg_sets.setdefault((include.entity_path, include.path), set()).add(stated)
            child_sets.setdefault(include.path, set()).add(stated)
        for include in walk.includes:
            site = include.site or records.Site(path=walk.entry_path)
            cited = _held_source(
                ctx.ws,
                walk,
                site,
                include.entity_path,
                site.locator if _in_workspace(ctx.ws, site.path) else "/".join(include.entity_path),
            )
            if not include.resolved or not include.path:
                ctx.builder.unresolved(
                    model.Unresolved(
                        want=model.Want.OTHER.value,
                        owner=launchfile_id,
                        expr=substitutions.command_text(_include_expr(include), ctx.ws.token)
                        if include.reason == "command_substitution"
                        else _include_expr(include),
                        reason=include.reason or "the included launch file path did not resolve",
                        role="include",
                        resolvable_by=["install the included package", "supply the argument"],
                        evidence=[cited],
                    )
                )
                continue
            try:
                child_id = _launchfile_id(ctx.ws, include.path)
            except ContractError:
                ctx.builder.unresolved(
                    model.Unresolved(
                        want=model.Want.OTHER.value,
                        owner=launchfile_id,
                        expr=_outside_include(ctx, include.path),
                        reason="the included launch file is outside every configured root",
                        role="include",
                        resolvable_by=["add the package to the roots"],
                        evidence=[cited],
                    )
                )
                continue
            if include.entity_path:
                package, relative = ctx.ws.rel(walk.entry_path)
                steps = tuple(_step(text) for text in include.entity_path)
                args = _include_args(ctx.ws, include)
                values = {"included_file": child_id, "args": args}
                if len(arg_sets[(include.entity_path, include.path)]) > 1:
                    leaf = steps[-1]
                    tag = ids.digest(
                        child_id, *(part for k, v in args for part in (k, ctx.ws.token(str(v))))
                    )
                    steps = (*steps[:-1], ids.LaunchStep(leaf.action, f"{leaf.disc}@{tag}"))
                    values["entity_path"] = list(include.entity_path)
                elif len(paths[include.entity_path]) > 1:
                    leaf = steps[-1]
                    steps = (
                        *steps[:-1],
                        ids.LaunchStep(leaf.action, f"{leaf.disc}@{ids.digest(child_id)}"),
                    )
                    values["entity_path"] = list(include.entity_path)
                entity_id = ids.build_launchent(package or ids.REPO_PKG, relative, steps)
                ctx.builder.node(entity_id, label="/".join(include.entity_path))
                from rosio.resolve._launch_child import _command_text

                for name, value in include.args:
                    command = _command_text(value)
                    if command is not None:
                        ctx.builder.unresolved(
                            model.Unresolved(
                                want=model.Want.OTHER.value,
                                owner=entity_id,
                                expr=substitutions.command_text(command, ctx.ws.token),
                                reason="command_substitution",
                                role=f"include_argument:{name}",
                                evidence=[cited],
                            )
                        )
                ctx.builder.edge("contains", launchfile_id, entity_id, layer=LAYER)
                ctx.builder.fact(
                    entity_id,
                    LAYER,
                    profile=profile_id,
                    run=run,
                    values=values,
                    confidence=model.Confidence.RESOLVED.value,
                    evidence=[cited],
                )
            ctx.builder.node(child_id)
            ctx.builder.fact(
                child_id,
                LAYER,
                profile=profile_id,
                run=run,
                values={"included": True},
                confidence=model.Confidence.RESOLVED.value,
                evidence=[_source(ctx.ws, include.path)],
            )
            ctx.builder.edge(
                model.EdgeKind.INCLUDES.value,
                launchfile_id,
                child_id,
                layer=LAYER,
                profile=profile_id,
                run=run,
                # The edge is one per file pair, so several argument sets are stated only by
                # the launch entity of each site.
                values=(
                    {"args": _include_args(ctx.ws, include)}
                    if len(child_sets[include.path]) == 1
                    else {}
                ),
                evidence=[cited],
            )

    # ------------------------------------------------------------------
    # nodes

    def _emit_node(
        self,
        ctx: CollectContext,
        spec: ProfileSpec,
        run: str,
        row: _Placed,
        late: list[dict[str, Any]],
    ) -> None:
        try:
            self._emit_node_inner(ctx, spec, run, row)
        except ContractError as exc:
            late.append(
                {
                    "path": _problem_path(row.source.file),
                    "phase": "visit",
                    "exc_type": type(exc).__name__,
                    "message": str(exc),
                }
            )

    def _emit_node_inner(
        self,
        ctx: CollectContext,
        spec: ProfileSpec,
        run: str,
        row: _Placed,
    ) -> None:
        profile_id = ids.build_profile(spec.slug)
        node = row.node
        identity = row.identity
        builder = ctx.builder

        builder.node(row.launchent_id, label=row.locator)
        builder.edge(
            model.EdgeKind.CONTAINS.value, row.launchfile_id, row.launchent_id, layer=LAYER
        )
        builder.fact(
            row.launchent_id,
            LAYER,
            profile=profile_id,
            run=run,
            values={
                "form": node.form,
                "pkg": node.pkg,
                "executable": node.executable,
                "plugin_class": node.plugin_class,
                "container": node.container,
                "conditions": list(node.conditions),
                "gates": [_gate_values(gate) for gate in node.gates],
                "parameter_inputs": [
                    _param_values(ctx.ws, item, ctx.cfg.ros_setup) for item in node.params
                ],
                "file_reads": _read_values(ctx, node),
                "namespace": identity.namespace,
                "namespace_stated": identity.namespace_stated,
                # The rules themselves, so a remap whose endpoint kind nothing states is
                # still readable here instead of living only in an unresolved entry.
                "remaps": [[item.frm, item.to, item.scope] for item in node.remaps],
            },
            confidence=row.confidence,
            evidence=[row.source],
        )
        for field, expr in node.command_blinds:
            builder.unresolved(
                model.Unresolved(
                    want=model.Want.NODE_NAME.value
                    if field in ("name", "namespace")
                    else model.Want.OTHER.value,
                    owner=row.launchent_id,
                    expr=substitutions.command_text(expr, ctx.ws.token),
                    reason="command_substitution",
                    role=field,
                    evidence=[row.source],
                )
            )

        values: dict[str, Any] = {
            "pkg": node.pkg,
            "executable": node.executable,
            "form": node.form,
        }
        if identity.name:
            values["name"] = identity.name
        if identity.namespace:
            values["namespace"] = identity.namespace
        if identity.known_name:
            values["known_name"] = identity.known_name
        if node.plugin_class:
            values["plugin_class"] = node.plugin_class
        if node.container:
            values["container"] = node.container
        if node.argv and not any(field == "arguments" for field, _expr in node.command_blinds):
            values["argv"] = _argv_values(ctx.ws, node.argv)
        evidence = [row.source]
        if identity.known_name:
            evidence.append(_table_source(node.pkg, node.executable))
            _file_entity_id(ctx, _TABLE_FILE)
        _name_node(builder, identity.node_id, identity.name_source)
        builder.fact(
            identity.node_id,
            LAYER,
            profile=profile_id,
            run=run,
            values=values,
            confidence=row.confidence,
            evidence=evidence,
        )
        builder.edge(
            model.EdgeKind.LAUNCHES.value,
            row.launchent_id,
            identity.node_id,
            key=ids.edge_key(launchent=row.launchent_id, profile=spec.slug),
            layer=LAYER,
            profile=profile_id,
            run=run,
            values={"form": node.form},
            evidence=[row.source],
        )
        builder.edge(
            model.EdgeKind.LAUNCHES.value,
            profile_id,
            row.launchent_id,
            key=ids.edge_key(profile=spec.slug, launchent=row.launchent_id),
            layer=LAYER,
            profile=profile_id,
            run=run,
            values={"entry": row.launchfile_id},
            evidence=[row.source],
        )
        if identity.fqn:
            builder.count_resolved(model.Want.NODE_NAME.value)
        else:
            builder.unresolved(
                model.Unresolved(
                    want=model.Want.NODE_NAME.value,
                    owner=identity.node_id,
                    expr=f"{node.pkg or UNKNOWN_SEGMENT}/{node.executable or node.plugin_class}",
                    reason=_name_reason(node, identity, ctx.ws),
                    role=node.form,
                    resolvable_by=["probe the running system", "name the node in the launch file"],
                    evidence=[row.source],
                )
            )
        self._emit_gates(ctx, spec, run, row)
        self._emit_remaps(ctx, spec, run, row)

    def _emit_run_node(
        self,
        ctx: CollectContext,
        spec: ProfileSpec,
        run: str,
        row: _RunNode,
        late: list[dict[str, Any]],
    ) -> None:
        """A `ros2 run` entry point as a node of the profile.

        There is no launch entity here and no remap edge: a remap edge is keyed on the launch
        site that produced it, and this node has none, so the remap rules stay on the fact
        where they can be read without being claimed as a resolved endpoint.
        """
        profile_id = ids.build_profile(spec.slug)
        node = row.node
        identity = row.identity
        builder = ctx.builder
        values: dict[str, Any] = {
            "pkg": node.pkg,
            "executable": node.executable,
            "form": node.form,
            "entrypoint": row.entry,
            "argv": _argv_values(ctx.ws, node.argv),
            "parameter_inputs": [
                _param_values(ctx.ws, item, ctx.cfg.ros_setup) for item in node.params
            ],
        }
        if identity.name:
            values["name"] = identity.name
        if identity.namespace:
            values["namespace"] = identity.namespace
        if identity.known_name:
            values["known_name"] = identity.known_name
        if node.remaps:
            values["remaps"] = [[item.frm, item.to] for item in node.remaps]
        evidence = [row.source] if row.source else []
        if row.exe_path:
            builder.node(ctx.ws.file_id(row.exe_path))
        for item in node.params:
            if item.site and _in_workspace(ctx.ws, item.site.path):
                builder.node(ctx.ws.file_id(item.site.path))
        if identity.known_name:
            evidence.append(_table_source(node.pkg, node.executable))
            _file_entity_id(ctx, _TABLE_FILE)
        try:
            _name_node(builder, identity.node_id, identity.name_source)
            builder.fact(
                identity.node_id,
                LAYER,
                profile=profile_id,
                run=run,
                values=values,
                confidence=model.Confidence.RESOLVED.value,
                evidence=evidence,
            )
            builder.edge(
                model.EdgeKind.LAUNCHES.value,
                profile_id,
                identity.node_id,
                key=ids.edge_key(profile=spec.slug, entry=row.entry),
                layer=LAYER,
                profile=profile_id,
                run=run,
                values={"form": node.form, "entrypoint": row.entry},
                evidence=evidence,
            )
            if row.exe_path:
                exe_id = ids.build_exe(node.pkg, node.executable)
                builder.node(exe_id, label=node.executable)
                builder.edge(
                    model.EdgeKind.CONTAINS.value,
                    _pkg_entity(ctx, node.pkg),
                    exe_id,
                    layer=LAYER,
                )
        except ContractError as exc:
            late.append(
                {
                    "path": row.entry,
                    "phase": "visit",
                    "exc_type": type(exc).__name__,
                    "message": str(exc),
                }
            )
            return
        if identity.fqn:
            builder.count_resolved(model.Want.NODE_NAME.value)
            return
        builder.unresolved(
            model.Unresolved(
                want=model.Want.NODE_NAME.value,
                owner=identity.node_id,
                expr=row.entry,
                reason=_name_reason(node, identity, ctx.ws),
                role=node.form,
                resolvable_by=["probe the running system", "name the node with __node in argv"],
                evidence=evidence,
            )
        )

    def _emit_gates(self, ctx: CollectContext, spec: ProfileSpec, run: str, row: _Placed) -> None:
        profile_id = ids.build_profile(spec.slug)
        trail: list[str] = []
        for gate in row.node.gates:
            trail.append(f"{gate.mechanism}({gate.target})")
            ctx.builder.edge(
                model.EdgeKind.GATES.value,
                row.launchent_id,
                row.identity.node_id,
                key=ids.edge_key(
                    launchent=row.launchent_id,
                    profile=spec.slug,
                    mechanism=gate.mechanism,
                    target=gate.target,
                    expression=gate.expression,
                    period=gate.period,
                    trail=trail[:-1],
                ),
                layer=LAYER,
                profile=profile_id,
                run=run,
                values=_gate_values(gate),
                evidence=[row.source],
            )

    def _emit_remaps(self, ctx: CollectContext, spec: ProfileSpec, run: str, row: _Placed) -> None:
        profile_id = ids.build_profile(spec.slug)
        node = row.node
        for remap in node.remaps:
            source = _remap_source(ctx.ws, remap) or row.source
            written_in = _written_outside(ctx, remap.site)
            resolved = _resolve_endpoint(remap.to, row.identity)
            if resolved is None:
                ctx.builder.unresolved(
                    model.Unresolved(
                        want=model.Want.TOPIC_NAME.value,
                        owner=row.launchent_id,
                        expr=f"{remap.frm}:={remap.to}",
                        reason="the remap target is not an absolute name and the namespace "
                        "the launch file would resolve it in is unknown",
                        role="remap",
                        resolvable_by=["state the namespace", "probe the running system"],
                        evidence=[source],
                        written_in=written_in,
                    )
                )
                continue
            kind = _endpoint_kind(remap.frm, resolved)
            if kind is None:
                # A ROS 2 remap rule renames a topic, a service and an action at once, so the
                # rule states no kind. Minting a topic here is how a service remap acquires
                # the wrong entity kind, and the wrong kind is worse than the gap.
                ctx.builder.unresolved(
                    model.Unresolved(
                        want=model.Want.OTHER.value,
                        owner=row.launchent_id,
                        expr=f"{remap.frm}:={remap.to}",
                        reason=f"the remap resolves to {resolved}, and nothing in the launch "
                        "file or the interface table says whether that name is a topic, a "
                        "service or an action",
                        role="remap",
                        resolvable_by=[
                            "read the endpoint in the node source",
                            "probe the running system",
                        ],
                        evidence=[source],
                        written_in=written_in,
                    )
                )
                continue
            endpoint_id = _endpoint_id(resolved, kind)
            ctx.builder.node(endpoint_id)
            ctx.builder.fact(
                endpoint_id,
                LAYER,
                profile=profile_id,
                run=run,
                values={"name": resolved},
                confidence=model.Confidence.RESOLVED.value,
                evidence=[source],
            )
            ctx.builder.edge(
                model.EdgeKind.REMAPS.value,
                row.launchent_id,
                endpoint_id,
                key=ids.edge_key(
                    launchent=row.launchent_id,
                    profile=spec.slug,
                    frm=remap.frm,
                    to=remap.to,
                    scope=remap.scope,
                ),
                layer=LAYER,
                profile=profile_id,
                run=run,
                values={"raw_name": remap.frm, "to": resolved, "scope": remap.scope},
                evidence=[source],
            )
            ctx.builder.count_resolved(model.Want.TOPIC_NAME.value)

    # ------------------------------------------------------------------
    # lifecycle

    def _emit_lifecycle(
        self,
        ctx: CollectContext,
        spec: ProfileSpec,
        run: str,
        placed: list[_Placed],
        late: list[dict[str, Any]],
    ) -> None:
        profile_id = ids.build_profile(spec.slug)
        by_fqn = {row.identity.fqn: row for row in placed if row.identity.fqn}
        at_root = _root_index(placed)
        for row in placed:
            for name in _lifecycle_names(row.node):
                target = self._managed_target(ctx, spec, run, row, name, by_fqn, at_root)
                if target is None:
                    continue
                try:
                    ctx.builder.edge(
                        model.EdgeKind.MANAGES_LIFECYCLE.value,
                        row.identity.node_id,
                        target,
                        key=ids.edge_key(
                            profile=spec.slug,
                            launchent=row.launchent_id,
                            mechanism=LIFECYCLE_LIST_PARAM,
                            target=target,
                            raw_name=name,
                        ),
                        layer=LAYER,
                        profile=profile_id,
                        run=run,
                        values={"mechanism": LIFECYCLE_LIST_PARAM, "raw_name": name},
                        evidence=[row.source],
                    )
                except ContractError as exc:
                    late.append(
                        {
                            "path": _problem_path(row.source.file),
                            "phase": "visit",
                            "exc_type": type(exc).__name__,
                            "message": str(exc),
                        }
                    )
            if not _in_launch_transition(row.node):
                continue
            ctx.builder.edge(
                model.EdgeKind.MANAGES_LIFECYCLE.value,
                row.launchent_id,
                row.identity.node_id,
                key=ids.edge_key(
                    profile=spec.slug,
                    launchent=row.launchent_id,
                    mechanism="OnStateTransition",
                    target=row.identity.node_id,
                ),
                layer=LAYER,
                profile=profile_id,
                run=run,
                values={
                    "mechanism": "OnStateTransition",
                    "in_launch": True,
                    "transitions": _transitions(row.node),
                },
                evidence=[row.source],
            )

    def _managed_target(
        self,
        ctx: CollectContext,
        spec: ProfileSpec,
        run: str,
        row: _Placed,
        name: str,
        by_fqn: dict[str | None, _Placed],
        at_root: dict[str, str | None],
    ) -> str | None:
        stated = bool(row.identity.namespace)
        base = row.identity.namespace or "/"
        try:
            fqn = ids.normalise_ros_name(name, base, row.identity.name)
        except ContractError:
            self._unmanaged(
                ctx,
                row,
                name,
                "a managed node name in the node_names list is not a usable ROS name",
            )
            return None
        known = by_fqn.get(fqn)
        if known is not None:
            return known.identity.node_id
        if not stated and (ids.is_relative(name) or ids.is_private(name)):
            # The manager states no namespace, so the namespace this relative name resolves in
            # is not tracked. A node the walk launched under the same name is the answer, and
            # nothing is minted when there is none: a guessed root namespace is false drift.
            sibling = at_root.get(fqn)
            if sibling is not None:
                return sibling
            self._unmanaged(
                ctx,
                row,
                name,
                "the manager states no namespace, so the namespace this managed name "
                "resolves in is unknown and no launched node carries it",
            )
            return None
        target = ids.build_node(fqn)
        _name_node(ctx.builder, target, model.NameSource.LITERAL.value)
        ctx.builder.fact(
            target,
            LAYER,
            profile=ids.build_profile(spec.slug),
            run=run,
            values={"name": ids.split_ros_name(fqn)[1], "namespace": ids.split_ros_name(fqn)[0]},
            confidence=model.Confidence.INFERRED.value,
            evidence=[row.source],
        )
        return target

    def _unmanaged(self, ctx: CollectContext, row: _Placed, name: str, reason: str) -> None:
        ctx.builder.unresolved(
            model.Unresolved(
                want=model.Want.NODE_NAME.value,
                owner=row.launchent_id,
                expr=f"{LIFECYCLE_LIST_PARAM}={name}",
                reason=reason,
                role="lifecycle",
                resolvable_by=["probe the running system", "state the manager namespace"],
                evidence=[row.source],
            )
        )


def _name_node(builder: model.AtlasBuilder, node_id: str, offered: str | None) -> None:
    """Register a node with the strongest name source anyone has offered for it.

    Writing the field flat let whichever pass ran last own it, so the atlas moved when the
    passes were reordered. `ids.NAME_SOURCE_ORDER` states the precedence and why.
    """
    held = builder.atlas.entities.get(node_id)
    builder.node(
        node_id,
        name_source=ids.strongest_name_source(
            held.name_source if held is not None else None, offered
        ),
    )


# ---------------------------------------------------------------------------
# run entry points


def _run_entry(ctx: CollectContext, spec: ProfileSpec, entry: Entrypoint) -> _RunNode:
    """A `ros2 run` invocation read as the node it starts.

    A run entry point carries its whole identity in argv, so nothing here is guessed: the
    name and the namespace are taken only when argv states them, the parameter inputs keep
    their argv order because that order is their precedence, and the walk record is handed
    on so the parameter chain sees the same inputs the launch side sees.
    """
    exe_path = _exe_path(ctx.ws, entry.pkg, entry.target)
    site = ctx.ws.site(exe_path, entry.label) if exe_path else None
    parsed = ros_args.parse_ros_args(entry.argv)
    namespace = ""
    if parsed.namespace:
        try:
            namespace = ids.normalise_namespace(parsed.namespace)
        except ContractError:
            namespace = ""
    node = records.ResolvedNode(
        pkg=entry.pkg,
        executable=entry.target,
        namespace=namespace,
        form="process",
        name=parsed.node_name,
        name_source=(
            model.NameSource.ARGV.value if parsed.node_name else model.NameSource.UNKNOWN.value
        ),
        remaps=tuple(records.Remap(frm=frm, to=to, site=site) for frm, to in parsed.remaps),
        params=_run_params(ctx, parsed, entry.argv, site),
        argv=tuple(entry.argv),
        site=site,
    )
    _hand_on(
        ctx,
        records.LaunchWalk(
            entry_path=exe_path or entry.label,
            profile=spec.slug,
            entry_pkg=entry.pkg if entry.pkg in ctx.ws.packages else None,
            args=tuple(entry.args),
            env=tuple(spec.env),
            nodes=[node],
            strategy="ast",
            notes=[f"{entry.label}, identity read from argv"],
        ),
    )
    return _RunNode(
        node=node,
        identity=_identity(node),
        entry=entry.label,
        source=ctx.ws.source(site) if site else None,
        exe_path=exe_path,
    )


def _hand_on(ctx: CollectContext, walk: records.LaunchWalk) -> None:
    """Leave the run entry point's record on the bundle, so the parameter chain reads the
    same ordered inputs the launch side hands it.

    A run entry point states its identity in argv, so two invocations of one executable that
    differ there are two records. Comparing the entry point alone would drop the parameters
    of the second one while its node stays in the atlas.
    """
    for existing in ctx.declared.walks:
        if _walk_key(existing) == _walk_key(walk):
            return
    ctx.declared.walks.append(walk)


def _walk_key(walk: records.LaunchWalk) -> tuple[Any, ...]:
    return (walk.profile, walk.entry_path, walk.args, tuple(node.argv for node in walk.nodes))


def _run_params(
    ctx: CollectContext,
    parsed: records.RosArgs,
    argv: tuple[str, ...],
    site: records.Site | None,
) -> tuple[records.ParamSource, ...]:
    out: list[records.ParamSource] = []
    for order, (kind, payload) in enumerate(_ordered_inputs(parsed, argv)):
        if kind == ros_args.INPUT_FILE:
            path = str(payload)
            where = ctx.ws.site(path, "--params-file") if _in_workspace(ctx.ws, path) else site
            out.append(
                records.ParamSource(order=order, kind="ros_args_file", path=path, site=where)
            )
            continue
        name, text = payload
        out.append(
            records.ParamSource(
                order=order,
                kind="ros_args_assign",
                values=((name, text),),
                site=site.with_locator(f"-p {name}") if site else None,
            )
        )
    return tuple(out)


def _ordered_inputs(parsed: records.RosArgs, argv: tuple[str, ...]) -> list[tuple[str, Any]]:
    """The parameter inputs in argv order, as (kind, payload) with payload a path or a
    (name, text) pair.

    `RosArgs.inputs` is the field that carries the interleaving. Its two spellings of the
    file kind are both accepted, and an argv parser that does not fill it at all is read
    again through `param_inputs`, which answers the same question from the raw argv.
    """
    rows = list(parsed.inputs)
    if not rows:
        rows = list(ros_args.param_inputs(argv))
    out: list[tuple[str, Any]] = []
    for kind, payload in rows:
        if kind == ros_args.INPUT_ASSIGN:
            if isinstance(payload, str):
                name, _, text = payload.partition(ros_args.ASSIGN)
                out.append((ros_args.INPUT_ASSIGN, (name, text)))
            else:
                name, text = payload
                out.append((ros_args.INPUT_ASSIGN, (str(name), str(text))))
            continue
        out.append((ros_args.INPUT_FILE, str(payload)))
    return out


def _exe_path(ws: Workspace, pkg: str, executable: str) -> str | None:
    """The installed executable, so a run entry point can cite a file that exists.

    Only an exact `lib/<pkg>/<executable>` hit counts. A name match anywhere else would cite
    a source file that may not be the thing that runs.
    """
    tail = f"/lib/{pkg}/{executable}"
    for path in ws.files():
        if path.endswith(tail):
            return path
    return None


def _in_workspace(ws: Workspace, path: str) -> bool:
    try:
        ws.file_id(path)
    except ContractError:
        return False
    return True


_MISSING_PACKAGE = re.compile(r"package '([^']+)' not found")
_SEARCHING = re.compile(r", searching: \[([^\]]*)\]")
_SEARCHED_DISTRO = re.compile(r"searched: [^\"]*?\bROS ([A-Za-z0-9_]+)")
_DISTRO_PREFIX = re.compile(r"/opt/ros/([A-Za-z0-9_]+)/?")


def missing_package(message: str) -> tuple[str, str] | None:
    """The package a `PackageNotFoundError` names and the ROS release it searched, when the
    message says which. `relabel_searched` wrote the second half."""
    found = _MISSING_PACKAGE.search(message)
    if found is None:
        return None
    distro = _SEARCHED_DISTRO.search(message)
    return found.group(1), distro.group(1) if distro else ""


_MISSING_MODULE = re.compile(r"No module named '([A-Za-z0-9_]+)")


def missing_module(message: str) -> tuple[str, str] | None:
    """The top level module a `ModuleNotFoundError` names and the ROS release that was
    sourced, when `relabel_module` wrote it."""
    found = _MISSING_MODULE.search(message)
    if found is None:
        return None
    distro = _SEARCHED_DISTRO.search(message)
    return found.group(1), distro.group(1) if distro else ""


def relabel_module(message: str, ros_setup: Sequence[str] = ()) -> str:
    """A missing module, with the ROS release whose python path was searched for it."""
    distros = [
        found.group(1)
        for found in (_DISTRO_PREFIX.fullmatch(str(Path(setup).parent)) for setup in ros_setup)
        if found
    ]
    if not distros or "searched:" in message:
        return message
    return f"{message}, searched: ROS {distros[0]}"


def relabel_searched(ws: Workspace, message: str, ros_setup: Sequence[str] = ()) -> str:
    """`searching: ['@outside']` as the trees a person configured, by label.

    The walk states the prefixes it was handed (`@install`, `@outside` for the directory of a
    setup file), which names nothing a reader can act on, so each one is named by its root,
    or by the ROS release that installed it."""
    kinds = {str(spec.kind) for spec in ws.roots}

    def label(prefix: str) -> str:
        head = prefix[1:].split("/", 1)[0] if prefix.startswith("@") else ""
        if head in kinds:
            kind, paths = head, [str(spec.path) for spec in ws.roots if spec.kind == head]
        elif prefix == OUTSIDE:
            kind, paths = "", [str(Path(setup).parent) for setup in ros_setup]
        else:
            inside = [spec for spec in ws.roots if _under_path(prefix, str(spec.path))]
            kind, paths = (str(inside[0].kind) if inside else ""), [prefix]
        distros = [found.group(1) for found in map(_DISTRO_PREFIX.fullmatch, paths) if found]
        if distros:
            return f"ROS {distros[0]}" + (f" ({kind} root)" if kind else "")
        if kind:
            return f"the {kind} root"
        return "the sourced setup files" if prefix == OUTSIDE else "a prefix outside every root"

    def searched(match: re.Match[str]) -> str:
        prefixes = re.findall(r"'([^']*)'", match.group(1))
        labels = list(dict.fromkeys(label(prefix) for prefix in prefixes))
        return f", searched: {', '.join(labels) or 'nothing'}"

    return _SEARCHING.sub(searched, message)


def _under_path(path: str, root: str) -> bool:
    return (path.rstrip("/") + "/").startswith(root.rstrip("/") + "/")


def _unset_variables(
    failure: records.WalkException, spec: ProfileSpec, ros_setup: Sequence[str]
) -> str:
    """The variable the walk stopped at, then every other one the same file reads with no
    default that neither the profile nor the sourced environment of the walk sets, so one
    extract names them all. TurtleBot3 needs LDS_MODEL as well as TURTLEBOT3_MODEL, and the
    second used to surface only after a second extract with the first one set.

    The shell that ran extract is not the walk's environment, and the row is compared data,
    so an environment that could not be read names the one variable the walk stopped at."""
    environment = _launch_exec().sourced_environment(ros_setup)
    if environment is None:
        return failure.env
    try:
        source = Path(failure.path).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return failure.env
    stated = {key for key, _value in spec.env}
    others = [
        name
        for name in _launch_exec().environment_reads(source)
        if name != failure.env and name not in stated and name not in environment
    ]
    return " ".join([failure.env, *others])


def unloadable_title(label: str, because: str = "") -> str:
    """One line for a file that produced no node, in the wording its own cause deserves."""
    if because == REFUSED_UNAVAILABLE:
        return f"{label} was never loaded: the launch machinery could not be imported"
    if because == REFUSED_MARKUP:
        return f"{label} is a launch frontend this walk does not execute"
    if because == REFUSED_MISSING:
        return f"no package in the roots holds the launch file {label}"
    return f"{label} is not a loadable ROS 2 launch file"


def unloadable_fix(because: str = "") -> str:
    if because == REFUSED_UNAVAILABLE:
        return (
            "make the interpreter that runs rosio able to import launch and launch_ros, "
            "then extract again: the run record names what failed and how to fix it"
        )
    if because == REFUSED_MARKUP:
        return (
            "name a python launch file that includes this one in the profile, or write this "
            "entry point as a python launch description"
        )
    if because == REFUSED_MISSING:
        return (
            "correct the package or the file name of the entry point in rosio.yaml, or add the "
            "root that holds the package"
        )
    if because == REFUSED_ROS1:
        return "port the file to the ROS 2 launch format or drop it from the profile that names it"
    return "make the file build a launch description, or drop it from the profile that names it"


def unloadable_say(label: str, because: str = "") -> str:
    """The sentence a small model may quote. XML is a supported ROS 2 launch format, so a
    ROS 2 XML file that this walk declines to execute must not be called a ROS 1 file."""
    if because == REFUSED_UNAVAILABLE:
        return (
            f"{label} was not executed because this environment cannot import the launch "
            "package, so every node and parameter it composes is missing from this atlas."
        )
    if because == REFUSED_MARKUP:
        return (
            f"{label} is an XML or YAML ROS 2 launch file and this walk executes python "
            "launch descriptions only, so none of its nodes are in this atlas."
        )
    if because == REFUSED_ROS1:
        return (
            f"{label} is a ROS 1 style launch file, so nothing in it was read and none of "
            "its nodes are in this atlas."
        )
    if because == REFUSED_MISSING:
        return (
            f"the profile names the launch file {label}, which no package in the roots holds, "
            "so that entry point added nothing to this atlas."
        )
    return f"{label} could not be loaded, so none of its nodes are in this atlas."


def _emit_refusals(ctx: CollectContext, refusals: list[_Refusal]) -> None:
    """One `launch.unloadable` finding per refused file, merged over the profiles that hit it.

    A raise is only a run row here: `launch.raised` is built by its drift rule from the rows.
    """
    grouped: dict[str, list[_Refusal]] = {}
    for item in refusals:
        grouped.setdefault(item.file_id, []).append(item)
    for file_id in sorted(grouped):
        rows = grouped[file_id]
        first = rows[0]
        if file_id not in ctx.builder.atlas.entities:
            continue
        ctx.builder.finding(
            model.Finding(
                rule=RULE_UNLOADABLE,
                severity=model.Severity.WARN.value,
                subject=file_id,
                title=unloadable_title(first.label, first.refused_because),
                detail=f"{first.message}. The file was recorded and never parsed, so it "
                "contributes no node to any profile.",
                fix=unloadable_fix(first.refused_because),
                say=unloadable_say(first.label, first.refused_because),
                discriminator="",
                profiles=sorted({row.profile for row in rows}),
                layers=[LAYER],
                # the phase is in the detail, and as a locator it read as a word after the path
                where=sorted(
                    {replace(row.source, locator="") for row in rows},
                    key=model.Source.sort_key,
                ),
            )
        )


# ---------------------------------------------------------------------------
# identity


_identity = identity.identify
_stated_name = identity.stated_name
_role = identity.role_digest
_role_namespace = identity.role_namespace
_under_temp = identity.under_temp
"""Identity is decided in one place for every collector. A second role digest here is what
gave one unnamed node two ids and split its edges between them."""


def _complete(walk: records.LaunchWalk) -> bool:
    """Whether the walk reached the end of the file it was given.

    Anything that raised leaves a subtree unread, so a list this walk produced is a lower
    bound and not the answer.
    """
    return walk.loadable and walk.strategy != "refused" and not walk.exceptions


def _confidence(walk: records.LaunchWalk) -> str:
    """An executed description is resolved. A source read is inferred: an AST walk cannot see
    what an OpaqueFunction or an event handler payload builds."""
    return (
        model.Confidence.RESOLVED.value
        if walk.strategy == "executed"
        else model.Confidence.INFERRED.value
    )


def _missing_file(ws: Workspace, failure: records.WalkException) -> str:
    """The file id of a workspace file a walk step did not find, or an empty string. The
    message names it by package once stated, and `build.unbuilt` owns the ones only a build
    puts in place."""
    if failure.exc_type != "FileNotFoundError":
        return ""
    found = _NO_SUCH_FILE.search(failure.message)
    package, _, relative = (found.group(1) if found else "").partition("/")
    if package not in ws.packages or not relative:
        return ""
    try:
        return ids.build_file(package, relative)
    except ContractError:
        return ""


_NO_SUCH_FILE = re.compile(r"No such file or directory: '([^']+)'")


def _unrun(failure: records.WalkException) -> bool:
    return failure.exc_type == _launch_exec().UNRUN_OUTPUT


def _refusal_reason(walk: records.LaunchWalk) -> str:
    if not walk.loadable:
        return "the file is not a ROS 2 loadable launch file, so its node set is unknown"
    if walk.exceptions and all(_unrun(failure) for failure in walk.exceptions):
        stated = "; ".join(sorted({failure.message for failure in walk.exceptions}))
        return f"{stated}, so the node set is incomplete"
    if walk.exceptions and all(failure.kept for failure in walk.exceptions):
        kept = ", ".join(sorted({failure.kept for failure in walk.exceptions}))
        return f"{kept} could not expand every value, so the values that failed are missing"
    if walk.exceptions:
        return "building the launch description raised, so the node set is incomplete"
    return "the entry point produced no launch description"


def _name_reason(node: records.ResolvedNode, identity: _Identity, ws: Workspace) -> str:
    """Why the name is unknown, naming the cause a reader can act on. The name table only
    answers for an executable whose source is not in the roots."""
    if identity.name_source == model.NameSource.RUNTIME_RANDOMISED.value:
        return "the runtime appends or randomises this executable's node name"
    package = ws.packages.get(node.pkg or "")
    if package is not None and package.src_path:
        unnamed = f"no source of {node.pkg} that rosio read states a node name for it"
    else:
        unnamed = f"{node.pkg or 'its package'} is not in the roots and not in the name table"
    if node.form == "process" and not identity.name:
        return f"the argv states no name, and {unnamed}"
    if not node.name:
        return f"launch states no name, and {unnamed}"
    return "launch states no namespace, so the fully qualified name cannot be formed"


# ---------------------------------------------------------------------------
# the third party name table, loaded lazily and treated as data


_TABLE_FILE = "rosio/resolve/known_names.py"


_known_table = identity.known_table
_known_name = identity.known_name


def _table_source(pkg: str, executable: str) -> model.Source:
    """Evidence row for a name that came out of the committed table. The table is data with
    a citation per row, so the row itself is the provenance."""
    return model.Source(
        file=ids.build_file(ids.REPO_PKG, _TABLE_FILE),
        locator=f"{pkg}/{executable}",
        root=model.Root.SRC.value,
    )


# ---------------------------------------------------------------------------
# launch entity steps


def _steps(node: records.ResolvedNode) -> tuple[ids.LaunchStep, ...]:
    if node.entity_path:
        trail = [_step(text) for text in node.entity_path]
        leaf = trail[-1]
        return (*trail[:-1], ids.LaunchStep(action=leaf.action, disc=_node_disc(node)))
    steps: list[ids.LaunchStep] = []
    for gate in node.gates:
        disc = gate.target or _disc(gate.mechanism, gate.expression, gate.period)
        steps.append(ids.LaunchStep(action=_token(gate.mechanism), disc=disc))
    steps.append(
        ids.LaunchStep(
            action=FORM_ACTIONS.get(node.form, "Node"),
            disc=_node_disc(node),
        )
    )
    return tuple(steps)


def _distinct_steps(
    nodes: Sequence[records.ResolvedNode],
) -> list[tuple[ids.LaunchStep, ...]]:
    """One step trail per node, unique within one walk.

    Two actions can state the same name on the same trail when conditions choose between
    them: jazzy's `localization_launch.py` loads `map_server` twice, once with an inline
    `yaml_filename` and once without, gated on whether a map was given. Only the colliding
    leaves grow, by the role digest and then by the gate trail, so every other id keeps its
    readable form.
    """
    trails = [_steps(node) for node in nodes]
    seen: dict[tuple[ids.LaunchStep, ...], int] = {}
    for trail in trails:
        seen[trail] = seen.get(trail, 0) + 1
    out: list[tuple[ids.LaunchStep, ...]] = []
    for node, trail in zip(nodes, trails, strict=True):
        if seen[trail] == 1:
            out.append(trail)
            continue
        gates = [part for g in node.gates for part in (g.mechanism, g.expression, g.period)]
        leaf = trail[-1]
        disc = f"{leaf.disc}@{_role(node)}@{_disc(*node.conditions, *gates)}"
        out.append((*trail[:-1], ids.LaunchStep(action=leaf.action, disc=disc)))
    return out


def _node_disc(node: records.ResolvedNode) -> str:
    """The action's own discriminator: its declared name, else its executable plus a role
    digest. The namespace is included when launch stated one, because two nodes in one file
    can carry the same name under two namespaces.

    The leaf of a resolver supplied trail is replaced by this, so four unnamed siblings in one
    file stay four launch entities instead of collapsing into one.
    """
    stated = _stated_name(node)[0]
    base = stated or f"{node.executable or node.plugin_class or 'node'}@{_role(node)}"
    namespace = _role_namespace(node).strip("/")
    return f"{namespace}/{base}" if namespace else base


def _step(text: str) -> ids.LaunchStep:
    head, sep, tail = text.partition("(")
    if sep and tail.endswith(")"):
        return ids.LaunchStep(action=_token(head), disc=tail[:-1] or _disc(text))
    return ids.LaunchStep(action=_token(text), disc=_disc(text))


def _token(text: str) -> str:
    cleaned = "".join(char if char.isalnum() or char == "_" else "" for char in text.strip())
    if not cleaned or cleaned[0].isdigit():
        return f"Step{cleaned}"
    return cleaned


def _disc(*parts: object) -> str:
    """A digest discriminator, prefixed so it can never read as an ordinal."""
    return "h" + ids.digest(*parts)


# ---------------------------------------------------------------------------
# endpoints, gates, parameters


def _resolve_endpoint(name: str, identity: _Identity) -> str | None:
    text = name.strip()
    if not text:
        return None
    if not text.startswith("/") and not identity.namespace:
        return None
    if ids.is_private(text) and not identity.name:
        return None
    try:
        return ids.normalise_ros_name(text, identity.namespace or "/", identity.name)
    except ContractError:
        return None


def _endpoint_id(resolved: str, kind: str) -> str:
    if kind == model.NodeKind.SERVICE.value:
        return ids.build_service(resolved)
    if kind == model.NodeKind.ACTION.value:
        return ids.build_action(resolved)
    return ids.build_topic(resolved)


def _endpoint_kind(source: str, resolved: str) -> str | None:
    """The entity kind of a remap endpoint, or None when nothing here states it.

    Both names are read, because a remap never changes an interface kind: the name the code
    uses decides it just as well as the name it is renamed to.
    """
    for name in (resolved, source):
        text = name.strip()
        if text.endswith(ACTION_SERVICE_SUFFIXES) or text.endswith(NODE_SERVICE_SUFFIXES):
            return model.NodeKind.SERVICE.value
        if text.endswith(ACTION_TOPIC_SUFFIXES):
            return model.NodeKind.TOPIC.value
        if text.rsplit("/", 1)[-1] and "/" + text.rsplit("/", 1)[-1] in BUILTIN_TOPICS:
            return model.NodeKind.TOPIC.value
    return None


def _gate_values(gate: records.Gate) -> dict[str, Any]:
    return {
        "mechanism": gate.mechanism,
        "expression": gate.expression,
        "period": gate.period,
        "target": gate.target,
        "outcome": gate.outcome,
    }


def _include_args(ws: Workspace, include: records.LaunchInclude) -> list[list[Any]]:
    """The arguments an include was given, with a temp path folded like a parameter input.

    The reference launcher hands a merged parameters file to an include as an argument, and
    its name is random per run, so leaving it here makes two identical extracts differ.
    """
    from rosio.resolve._launch_child import _command_text

    out: list[list[Any]] = []
    for key, value in include.args:
        out.append(
            [
                key,
                None
                if _command_text(value) is not None
                else EXTERNAL_TOKEN
                if _under_temp(value, _root_paths(ws))
                else ws.stated_value(value),
            ]
        )
    return out


def _read_values(ctx: CollectContext, node: records.ResolvedNode) -> list[dict[str, Any]]:
    out: dict[tuple[str, str], dict[str, Any]] = {}
    for read in node.file_reads:
        if not _in_workspace(ctx.ws, read.path):
            continue
        file_id = ctx.ws.file_id(read.path)
        ctx.builder.node(file_id)
        row: dict[str, Any] = {"file": file_id, "role": read.role}
        if read.xacro_args:
            row["xacro_args"] = ctx.ws.stated_value([list(pair) for pair in read.xacro_args])
        # A model read both as text and through its command is one read, with its arguments.
        if len(row) >= len(out.get((file_id, read.role), {})):
            out[(file_id, read.role)] = row
    return [out[key] for key in sorted(out)]


def _param_values(
    ws: Workspace, item: records.ParamSource, ros_setup: Sequence[str] = ()
) -> dict[str, Any]:
    """The parameter input as the launch walk saw it. Provenance points at the source file
    and the rewrite list, never at the temp file a rewrite materialises."""
    payload: dict[str, Any] = {"order": item.order, "kind": item.kind}
    for key, value in (("path", item.path), ("source_path", item.source_path)):
        if not value:
            continue
        if _under_temp(value, _root_paths(ws)):
            # The name of a file written under the temp directory is random per run. It is
            # not provenance, and putting it in a fact makes two identical runs differ.
            payload[key] = EXTERNAL_TOKEN
            payload["external"] = True
            continue
        payload[key] = _rel_or_path(ws, value, ros_setup)
    from rosio.resolve._launch_child import uncarried_reason

    if item.root_key is not None:
        payload["root_key"] = None if uncarried_reason(item.root_key) else item.root_key
    if item.rewrites:
        payload["rewrites"] = [
            [key, None if uncarried_reason(value) else value] for key, value in item.rewrites
        ]
    if item.values:
        payload["keys"] = sorted(str(key) for key, _value in item.values)
    if item.note:
        payload["note"] = item.note
    return payload


def _root_index(placed: list[_Placed]) -> dict[str, str | None]:
    """The nodes a walk launched with no stated namespace, keyed by the name they carry at the
    root namespace. Used to point a managed name at a node this profile really launched, never
    to mint an id. An ambiguous name maps to None, so two candidates produce no edge.
    """
    out: dict[str, str | None] = {}
    for row in placed:
        identity = row.identity
        if row.node.namespace.strip() or not identity.name:
            continue
        if identity.name_source not in USABLE_NAME_SOURCES:
            continue
        try:
            fqn = ids.normalise_ros_name(identity.name, "/")
        except ContractError:
            continue
        out[fqn] = None if fqn in out else identity.node_id
    return out


def _lifecycle_names(node: records.ResolvedNode) -> list[str]:
    """The `node_names` list of a lifecycle manager, in order, from its inline parameters.

    Reading only the launch actions misses the manager driven majority on a real stack, so
    the parameter value itself is the edge source.
    """
    out: list[str] = []
    for item in node.params:
        for key, value in item.values:
            if ids.normalise_param_name(key) != LIFECYCLE_LIST_PARAM:
                continue
            if isinstance(value, (list, tuple)):
                out.extend(str(entry) for entry in value if str(entry).strip())
            elif isinstance(value, str) and value.strip():
                out.append(value.strip())
    return out


def _in_launch_transition(node: records.ResolvedNode) -> bool:
    if any(gate.mechanism == "OnStateTransition" for gate in node.gates):
        return True
    return bool(node.attrs.get("lifecycle_transitions") or node.attrs.get("emit_events"))


def _transitions(node: records.ResolvedNode) -> list[str]:
    raw = node.attrs.get("lifecycle_transitions") or node.attrs.get("emit_events") or []
    stated = [str(item) for item in raw] if isinstance(raw, (list, tuple)) else [str(raw)]
    stated.extend(
        gate.outcome or gate.target
        for gate in node.gates
        if gate.mechanism == "OnStateTransition" and (gate.outcome or gate.target)
    )
    return sorted(dict.fromkeys(item for item in stated if item))


# ---------------------------------------------------------------------------
# workspace plumbing


def _launch_exec() -> Any:
    try:
        from rosio.resolve import launch_exec
    except ImportError as exc:
        raise CollectorError(
            "the launch resolver is not available, install the launch extra or supply "
            "prewalked launch records",
            layer=LAYER,
            collector=NAME,
        ) from exc
    return launch_exec


def _launch_index(ws: Workspace) -> dict[tuple[str, str], str]:
    """Entry point files by package. Once a package is built `ros2 launch` runs its installed
    copy, so the walk does too, and a src copy edited since the build is a `build.stale`
    finding instead of a graph that describes neither tree."""
    index: dict[tuple[str, str], str] = {}
    for path in ws.launch_files():
        package, relative = ws.rel(path)
        found = ws.packages.get(package or "")
        installed = Path(found.share_path, relative) if found and found.share_path else None
        if installed is not None and installed.is_file():
            path = str(installed)
        index.setdefault((package or ids.REPO_PKG, relative), path)
    return index


def _aliases(ws: Workspace) -> dict[str, str]:
    """Directory name to package.xml name, for the directories that disagree.

    A package is its manifest name, and the directory holding it is only an alias: a
    directory spelled in capitals can hold a package whose manifest name is lower case, and
    an entry point written with either spelling names the same launch files. An alias that
    two packages claim is dropped rather than guessed.
    """
    claims: dict[str, set[str]] = {}
    for package in ws.packages.values():
        for base in (package.src_path, package.share_path):
            if not base:
                continue
            alias = Path(base).name
            if alias != package.name:
                claims.setdefault(alias, set()).add(package.name)
    return {
        alias: sorted(names)[0]
        for alias, names in sorted(claims.items())
        if len(names) == 1 and alias not in ws.packages
    }


def _locate(
    index: dict[tuple[str, str], str],
    pkg: str,
    target: str,
    aliases: dict[str, str] | None = None,
) -> str | None:
    cleaned = target.strip().lstrip("/")
    owners = [pkg]
    resolved = (aliases or {}).get(pkg)
    if resolved:
        owners.append(resolved)
    for owner in owners:
        for candidate in (cleaned, f"launch/{cleaned}"):
            hit = index.get((owner, candidate))
            if hit is not None:
                return hit
    base = Path(cleaned).name
    for owner in owners:
        for (holder, relative), path in sorted(index.items()):
            if holder == owner and Path(relative).name == base:
                return path
    return None


def _launchfile_id(ws: Workspace, path: str) -> str:
    package, relative = ws.rel(path)
    return ids.build_launchfile(package or ids.REPO_PKG, relative)


def _pkg_entity(ctx: CollectContext, package: str | None) -> str:
    name = package or ids.REPO_PKG
    pkg_id = ids.build_pkg(name)
    found = ctx.ws.packages.get(name)
    node = ctx.builder.node(pkg_id)
    if found is not None:
        _attr_once(node, "build_type", found.build_type)
    return pkg_id


def _file_entity(ctx: CollectContext, path: str, loadable: bool = True) -> str:
    file_id = ctx.ws.file_id(path)
    node = ctx.builder.node(file_id)
    _attr_once(node, "loadable", loadable)
    return file_id


def _file_entity_id(ctx: CollectContext, relative: str) -> str:
    """Entity for a file that is part of rosio itself, so a table citation has a target."""
    file_id = ids.build_file(ids.REPO_PKG, relative)
    node = ctx.builder.node(file_id)
    _attr_once(node, "vendored_table", True)
    return file_id


def _attr_once(node: model.Node, key: str, value: Any) -> None:
    """Set an attribute the first time. A later disagreement about loadability keeps the
    refusal, because a file that refused once is not loadable everywhere."""
    if key not in node.attrs:
        node.attrs[key] = value
        return
    if isinstance(value, bool) and isinstance(node.attrs[key], bool):
        node.attrs[key] = node.attrs[key] and value


def _source(
    ws: Workspace, path: str, locator: str = "", line_hint: int | None = None
) -> model.Source:
    return ws.source(ws.site(path, locator, line_hint))


def _held_source(
    ws: Workspace,
    walk: records.LaunchWalk,
    site: records.Site | None,
    trail: tuple[str, ...],
    locator: str,
) -> model.Source:
    """A site as a line the workspace holds.

    A node or an include stated in a file outside every root, nav2_bringup from the ROS
    installation under turtlebot3, is cited at the deepest include on its trail whose line a
    root holds, which is the line an engineer edits. The locator is the action trail, so it
    still names every outside file between that line and the site.

    The include steps of a trail name a file by its basename, so two includes of
    `robot.launch.py` from one file share one step. The include that reached the site is
    therefore followed by path, file to the include that read it, up to a line a root holds;
    the deepest include on the trail answers only when that chain breaks."""
    if site is not None and _in_workspace(ws, site.path):
        return _source(ws, site.path, locator, site.line_hint)
    on_trail = [
        include
        for include in walk.includes
        if include.site is not None
        and include.entity_path
        and trail[: len(include.entity_path)] == include.entity_path
    ]
    path, depth = (site.path if site is not None else ""), len(trail)
    while path:
        reached = [
            include
            for include in on_trail
            if include.path == path and len(include.entity_path) <= depth
        ]
        if not reached:
            break
        include = max(reached, key=lambda item: len(item.entity_path))
        cite = include.site
        if cite is None:
            break
        if _in_workspace(ws, cite.path):
            return _source(ws, cite.path, locator, cite.line_hint)
        path, depth = cite.path, len(include.entity_path) - 1
    held = [
        (len(include.entity_path), include.site)
        for include in walk.includes
        if include.site is not None
        and include.entity_path
        and trail[: len(include.entity_path)] == include.entity_path
        and _in_workspace(ws, include.site.path)
    ]
    if held:
        _depth, site = max(held, key=lambda item: item[0])
        return _source(ws, site.path, locator, site.line_hint)
    return _source(ws, walk.entry_path, locator)


def _remap_source(ws: Workspace, remap: records.Remap) -> model.Source | None:
    """A group scoped rule is written where it was pushed, often above an include."""
    site = remap.site
    if remap.scope != "global" or site is None or not _in_workspace(ws, site.path):
        return None
    return _source(ws, site.path, site.locator, site.line_hint)


def _written_outside(ctx: CollectContext, site: records.Site | None) -> str:
    """The file a site sits in when no root holds it, which its citation cannot name."""
    if site is None or not site.path or _in_workspace(ctx.ws, site.path):
        return ""
    return ctx.ws.stated_path(site.path, ctx.cfg.ros_setup)


def _outside_include(ctx: CollectContext, path: str) -> str:
    """An included file no root holds, as its blind spot states it.

    `stated_path` names a file outside the checkout and every setup directory by its name
    alone, and the blind spot id is a digest over that expression, so two includes of
    `robot.launch.py` from two directories were one blind spot. That form is stated by its
    content as well, the way `Workspace.stated_by_content` states a setup file."""
    stated = ctx.ws.stated_path(path, ctx.cfg.ros_setup)
    if stated.startswith(f"{OUTSIDE}/") and stated.count("/") == 1:
        return ctx.ws.stated_by_content([path])[0]
    return stated


def _problem_path(file_id: str) -> str:
    """The `<pkg>/<relpath>` form an exception row carries, from a file id."""
    parsed = ids.parse_file(file_id)
    if parsed.package == ids.REPO_PKG:
        return parsed.relpath
    return f"{parsed.package}/{parsed.relpath}"


def _root_paths(ws: Workspace) -> list[str]:
    return [str(spec.path) for spec in ws.roots]


def _rel_or_path(ws: Workspace, path: str, ros_setup: Sequence[str] = ()) -> str:
    return ws.stated_path(path, ros_setup)


def _predicates(ws: Workspace, walks: list[records.LaunchWalk]) -> dict[str, bool]:
    """Filesystem predicates, namespaced by the file that probed them.

    Two boxes that disagree only about a file on disk have to show an input difference, and
    two entry points that probe the same path must not overwrite each other.
    """
    out: dict[str, bool] = {}
    for walk in walks:
        prefix = _rel_or_path(ws, walk.entry_path)
        for key in sorted(walk.predicates):
            out[f"{prefix}:{key}"] = bool(walk.predicates[key])
        for package in walk.underlay:
            out[f"{UNDERLAY_PREDICATE}{package}"] = True
    if ws.install_unnamed():
        out[INSTALL_UNNAMED] = True
    return dict(sorted(out.items()))


UNDERLAY_PREDICATE = "underlay:"
"""A package a src root holds whose share the ament index answered from another tree in this
profile, so the walk read the released copy and not the source."""

INSTALL_UNNAMED = "workspace:install_unnamed"
"""An install tree a build made sits beside a src root and no root names it."""


OUTSIDE_TREE = "a tree outside this workspace"


def share_tree(ws: Workspace, share: str, ros_setup: Sequence[str] = ()) -> str:
    """The tree a share directory sits in, as a reader names it: `ROS jazzy` for a release
    under `/opt/ros`, else the prefix as compared data may state it inside the workspace.
    Outside it the prefix is a property of one machine and its last segment names nothing a
    reader can find, so the tree is said to be outside, and sourced when `ros_setup` holds it.
    """
    prefix = str(Path(share).parent.parent)
    found = _DISTRO_PREFIX.fullmatch(prefix)
    if found:
        return f"ROS {found.group(1)}"
    stated = ws.stated_path(prefix, ros_setup)
    if not stated.startswith(OUTSIDE):
        return stated
    if any(Path(prefix).is_relative_to(Path(setup).parent) for setup in ros_setup):
        return f"{OUTSIDE_TREE} that ros_setup sources"
    return OUTSIDE_TREE


def _input_digest(
    ctx: CollectContext,
    spec: ProfileSpec,
    walks: list[records.LaunchWalk],
    launchfile_ids: list[str],
    predicates: dict[str, bool],
) -> str:
    """The inputs this walk depended on.

    A setup file enters as the place it holds in the checkout, or by its name and the hash
    of its content when it sits outside one, the same form the generation block and the
    profile attributes state, because the run id built from this digest is compared data and
    the distribution is mounted at a different prefix on the next box.
    """
    parts: list[str] = [spec.slug]
    for entry in spec.entrypoints:
        parts.append(entry.label)
        parts.extend(f"{key}:={ctx.ws.stated_value(value)}" for key, value in entry.args)
        parts.extend(ctx.ws.stated_value(list(entry.argv)))
    parts.extend(f"{key}={ctx.ws.stated_value(value)}" for key, value in spec.env)
    parts.extend(ctx.ws.stated_by_content(ctx.cfg.ros_setup))
    parts.extend(sorted(launchfile_ids))
    parts.extend(f"{key}={value}" for key, value in sorted(predicates.items()))
    for name, value in sorted(ctx.ws.root_hashes().items()):
        parts.append(f"{name}={value}")
    for walk in walks:
        parts.append(f"{_rel_or_path(ctx.ws, walk.entry_path)}={walk.strategy}={walk.loadable}")
    return ids.digest(*parts)


def _node_order(node: records.ResolvedNode) -> tuple[str, str, str, str, str]:
    return (
        node.namespace,
        node.name or "",
        node.pkg,
        node.executable,
        node.plugin_class or "",
    )


_LAUNCH_LAYER_PATH = "@launch"
"""Subject of the row that reports the layer itself rather than one of its files. It is not
a file id and never becomes one: nothing on disk failed here."""


def _refused_because(walk: records.LaunchWalk) -> str:
    """Why the file was not read, from the note the walk recorded about it."""
    unavailable = _launch_exec().NOTE_UNAVAILABLE
    for note in walk.notes:
        if note.startswith(unavailable):
            return REFUSED_UNAVAILABLE
    text = " ".join(walk.notes)
    if "ROS 1" in text:
        return REFUSED_ROS1
    if "launch frontend" in text:
        return REFUSED_MARKUP
    return ""


def _include_problems(ws: Workspace, walk: records.LaunchWalk) -> list[dict[str, Any]]:
    """One row per include that did not resolve, so a partial collapse is countable.

    Three collapsed `IncludeLaunchDescription` calls take the whole node set of a stack with
    them, and a run that reports that as one blind spot buried in the unresolved list reads
    like a run that read everything.
    """
    out: list[dict[str, Any]] = []
    for include in walk.includes:
        if include.resolved:
            continue
        out.append(
            {
                "path": _rel_or_path(ws, walk.entry_path),
                "phase": "include",
                "exc_type": INCLUDE_UNRESOLVED,
                "message": (
                    f"{ws.stated_value(_include_expr(include))} did not resolve to a launch "
                    "file, so the nodes it includes are not in this profile"
                ),
            }
        )
    return out


def _include_expr(include: records.LaunchInclude) -> str:
    """The include expression as compared data may carry it.

    A launch object that did not stringify carries a memory address in its repr, and this
    text keys the blind spot, so the address can never reach it.
    """
    text = include.path or "IncludeLaunchDescription"
    if _REPR_MARK not in text:
        return text
    head, _sep, _tail = text.partition(_REPR_MARK)
    return f"{head}>"


def _argv_values(ws: Workspace, argv: Sequence[str]) -> list[str]:
    """Argv as a fact carries it, with a temp path folded like a parameter input.

    A launch time rewrite of an rviz configuration or a parameters file materialises a file
    whose name is random per run. Leaving it here makes two extracts of one unchanged
    workspace differ inside canonical bytes.
    """
    roots = _root_paths(ws)
    return [EXTERNAL_TOKEN if _under_temp(item, roots) else ws.token(item) for item in argv]


def _problem_key(problem: dict[str, Any]) -> tuple[str, str, str, str]:
    return (
        str(problem.get("path", "")),
        str(problem.get("phase", "")),
        str(problem.get("exc_type", "")),
        str(problem.get("message", "")),
    )


register(NAME, LaunchDeclaredCollector)


__all__ = [
    "LAYER",
    "NAME",
    "NO_PROFILE",
    "REFUSED_MARKUP",
    "REFUSED_MISSING",
    "REFUSED_ROS1",
    "REFUSED_UNAVAILABLE",
    "LaunchDeclaredCollector",
    "missing_module",
    "missing_package",
    "relabel_module",
    "relabel_searched",
    "unloadable_fix",
    "unloadable_say",
    "unloadable_title",
]
