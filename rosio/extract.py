"""Orchestration: run the collectors once, in one committed order, and write the atlas.

Four decisions live here and nowhere else.

Order. `ORDER` is the selection as well as the order: a run that names no collector runs
exactly that tuple, not whatever the registry happens to hold. The registry fills up by
import, so defaulting to it makes the atlas depend on which module was imported first, which
is a property of the caller and not of the workspace. Only one dependency exists between the
declared collectors: the parameter chain reads the launch walks that the launch collector
recorded on the shared bundle, so the launch collector runs first. Everything else is
independent, and the order between them is committed only so that the output cannot move.

Scope. A collector either describes the repository (interface definitions, plugin manifests,
urdf models, scraped entry points, the C++ and python sources) or one running composition
(the launch walk and the parameter chain that depends on it). The first kind runs once and
its facts carry no profile. The second runs once per profile, because a workspace wide union
of launch files describes a system that never runs.

Failure. One input that raises must not empty a layer, so every collector call is wrapped and
the exception is recorded on the run it belongs to. A layer is therefore declared present
after its collectors ran and only when one of them produced a fact: declaring it up front
says the layer holds a truth even when the collector that would have produced it raised, and
a reader cannot tell that apart from a layer that is genuinely empty.

Completeness. The run also records, per collector, whether that pass finished, because no
ratio can. `counters.resolution_rate` is resolved over resolved plus unresolved per want, so
a pass that died contributes neither and its want either keeps a perfect rate over a smaller
denominator or leaves the counter entirely. A pass that stopped is therefore recorded as a
fact of its own, `counters.passes` sums it, and `extract.pass_incomplete` grades it. A
collector that has nothing to read is complete and never in it: it may say so through an
optional `inputs_present(ctx)`, which returns True, or False or the sentence naming what is
absent, and a collector that does not implement it is attempted.

Product. The atlas is the input to the drift engine, not the output of the tool, so the
rules run here and their findings are written onto the atlas before it is validated. The
join and the launch collector put findings on it first, and one rule reads them so that it
never repeats a subject the join already reported, so the two sets are merged and never
replaced.

Growth. A run may extend an atlas that is already on disk instead of starting an empty one,
which is what `rosio probe` does: the observed layer is collected against a repository the
static layers already described, and re extracting to reach it would throw that work away and
move every id that depends on a file on disk. The resolved counters are carried over with the
atlas, because the denominator of the resolution rate belongs to the atlas and not to the
pass that happened to write last.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
from importlib import import_module
from pathlib import Path
from typing import Any

from rosio import collect, ids, model, schema
from rosio.config import Config, ProfileSpec
from rosio.errors import ContractError, UsageError
from rosio.workspace import Workspace

ATLAS_NAME = "atlas.json"

PROBE_VERB = "rosio probe"
"""The verb that owns attaching to a live system, named in the refusal so the reader is not
left looking for a flag that does not exist."""

ALWAYS = ("drift.findings",)
"""Declared by every run, because the rules run here and an atlas without the mark reads
exactly like a clean stack."""

LAYER_CAPABILITIES = {
    model.Layer.DECLARED.value: ("layer.declared", "param.candidates", "profile.entrypoints"),
    model.Layer.CODED.value: ("layer.coded",),
    model.Layer.OBSERVED.value: ("edge.timing", "layer.observed"),
    model.Layer.RECORDED.value: ("bagtopic.stats", "edge.timing", "layer.recorded"),
}
"""The feature probe a consumer tests instead of guessing from an absent key, per layer, so
an atlas states what it holds and never what the tool can do. AGENTS section 9a names the
three members the recorded layer appends."""

CAPABILITIES = tuple(
    sorted(
        set(ALWAYS)
        | set(LAYER_CAPABILITIES[model.Layer.DECLARED.value])
        | set(LAYER_CAPABILITIES[model.Layer.CODED.value])
    )
)
"""What a static extraction of both repository layers declares."""

ALL_CAPABILITIES = tuple(
    sorted(set(ALWAYS) | {mark for members in LAYER_CAPABILITIES.values() for mark in members})
)
"""The whole vocabulary this version can declare, which is the closed list a consumer may
test against. The runtime layers append `layer.observed`, `layer.recorded`, `edge.timing` and
`bagtopic.stats`, and an atlas states them only once the layer that carries them said
something, per AGENTS section 9a: the mark is a feature probe and not a statement about what
the tool can do."""

MODULES = (
    "rosio.collect.bag",
    "rosio.collect.stamps",
    "rosio.collect.cpp_code",
    "rosio.collect.entrypoints",
    "rosio.collect.interfaces",
    "rosio.collect.launch_declared",
    "rosio.collect.params",
    "rosio.collect.plugins",
    "rosio.collect.probe",
    "rosio.collect.proc",
    "rosio.collect.py_code",
    "rosio.collect.urdf",
)
"""Imported for their registration. Import is deferred to `run` so that importing this
module stays cheap and no optional dependency is touched by an import."""

STATIC = (
    "interfaces",
    "plugins",
    "urdf",
    "entrypoints",
    "launch_declared",
    "params",
    "cpp_code",
    "py_code",
)
"""The committed selection of a run that reads the repository, which is what `extract` with
no further request calls. `params` follows `launch_declared` because it consumes the walks
that one records. The rest is fixed so that two runs cannot differ, and a collector that is
registered but not named here never runs unless a caller asks for it by name."""

RUNTIME = ("bag", "stamps", "proc", "probe")
"""The collectors that read something other than the repository. They are in the committed
order so their output cannot move, and never in a default selection: a bag is named on the
command line or by a root, and attaching to a live system is its own verb."""

ORDER = STATIC + RUNTIME
"""Every committed collector, in the order it runs."""

OPT_IN = frozenset({"probe", "proc"})
"""Collectors that report a live system, whether by attaching to the middleware or by reading
the process table. A run reaches one only when the caller named it in `opt_in`, which the
probe verb does after it has a domain id to attach to. Naming one with `--collector` is
refused: this box is attached to a vehicle, a probe that discovers the wrong domain is the
worst thing the tool can do, and an atlas that declares the observed layer has to come from
the verb that says it went looking for a running system."""

RULE_PACKAGE = "rosio.drift"
"""Where the rule families live. They are discovered rather than listed, so a family that
lands after this file was last touched still runs."""

PROFILE_SCOPED = frozenset({"launch_declared", "params"})
"""Collectors that describe one running composition. Everything else runs once, with no
profile, and says so by carrying `*` on its facts."""

PROFILE_BOUND = frozenset({"probe", "proc"})
"""Collectors that describe the one system that is running now. They run once, and their
facts carry a profile only when the caller named exactly one: a probe cannot tell which
composition it attached to, and a fact filed against every profile is read against each of
them, which is the honest answer when the caller did not say."""

SELF_DECLARED = frozenset({model.Layer.OBSERVED.value, model.Layer.RECORDED.value})
"""Layers whose collectors state their own presence, reason and root. A probe that never
attached, a probe that attached to a system with no node, and a recording that refused to be
read are three statements, and only the collector knows which one happened. The orchestrator
declares such a layer only when its collectors said nothing at all."""

X_NAMESPACE = "extract"
X_NOT_FROM_A_RULE = "findings_no_rule_produced"
"""Where the ids of the findings the collectors and the join wrote are kept, under
`atlas.x`. The join writes `node.ambiguous_identity` and the launch collector writes its
refusals, and neither is derivable from the atlas afterwards, so a run that extends the file
keeps exactly these and drops the rule output it is about to compute again."""

CONTRIBUTES = {
    "bag": "the recordings, their topics and their per topic statistics",
    "stamps": "header stamp checks and clock diagnoses for recorded topics",
    "cpp_code": "the endpoints, parameter defaults, frames and rates the C++ sources state",
    "entrypoints": "the entry points that compose a profile outside launch",
    "interfaces": "the message, service and action definitions of the workspace",
    "launch_declared": "the nodes, remaps and parameters an entry point launches",
    "params": "the parameter candidate chains of the yaml profiles",
    "plugins": "the pluginlib manifests and the plugins they export",
    "probe": "the nodes, endpoints and effective values of the live system",
    "proc": "the command lines of the processes that are running",
    "py_code": "the endpoints, parameter defaults and frames the python sources state",
    "urdf": "the urdf models, the frames they declare and their geometry",
}
"""What each collector is the only source of, in the words the finding quotes. A pass that
stopped costs exactly this, and the sentence a reader can act on has to name it."""

NO_INPUT = "the collector reported that it has nothing to read here"
"""What a pass that was not attempted says when the collector stated no reason of its own."""

LAYER_ROOTS = {
    model.Layer.OBSERVED.value: model.Root.RUNTIME.value,
    model.Layer.RECORDED.value: model.Root.BAG.value,
}


@dataclass(frozen=True)
class Pass:
    """One collector group, on one layer, for one profile or for the repository."""

    layer: str
    profile: str
    names: tuple[str, ...]


def run(
    cfg: Config,
    ws: Workspace,
    *,
    collectors: Sequence[str] | None = None,
    instances: Mapping[str, Any] | None = None,
    atlas: model.Atlas | None = None,
    opt_in: Sequence[str] = (),
    progress: Callable[[str], None] | None = None,
) -> model.Atlas:
    """Collect every selected layer, recompute once, validate, then write the atlas.

    `instances` hands one built collector to the pass that would otherwise ask the registry
    for it, which is how a sampling window or a list of recordings reaches a collector the
    registry builds with its defaults. `atlas` extends an atlas that already exists instead
    of starting an empty one. `opt_in` names the collectors in `OPT_IN` this caller is
    allowed to reach, and nothing else can reach them. `progress` is told the name of each
    phase as it ends, and nothing it hears reaches the atlas.
    """
    say = progress or (lambda _label: None)
    _register()
    selected = _select(collectors, opt_in)
    builder = _builder(cfg, ws, selected, atlas)
    ctx = collect.CollectContext(ws=ws, cfg=cfg, builder=builder)
    roots = ws.root_hashes()
    say("root hashes")
    steps = _passes(cfg, selected)
    scoped = sum(1 for step in steps if step.profile)
    passes = []
    done = 0
    try:
        for step in steps:
            each = None if step.profile else say
            passes.append((step.layer, _run_pass(ctx, cfg, step, roots, instances or {}, each)))
            if step.profile:
                done += 1
                say(f"{', '.join(step.names)} for profile {step.profile} ({done}/{scoped})")
    finally:
        ctx.close()
    _declare_layers(builder, passes)
    if any(name in STATIC for name in selected):
        _join(builder)
        _record_share_reads(builder, ws)
        say("join")
    _declare_capabilities(builder)
    built = builder.build()
    drift(built)
    say("drift")
    payload = built.to_json()
    problems = schema.validate(payload)
    if problems:
        raise ContractError(
            f"the extracted atlas does not satisfy the schema ({len(problems)} violations): "
            + "; ".join(problems[:5])
        )
    _write(cfg, payload)
    say("write")
    return built


def _builder(
    cfg: Config,
    ws: Workspace,
    selected: Sequence[str],
    atlas: model.Atlas | None,
) -> model.AtlasBuilder:
    """An empty atlas, or the one on disk with its counters and its own findings back.

    The rule output of the earlier run is dropped here rather than merged: this run computes
    it again over more layers than that one had, and a finding the new answer contradicts
    would otherwise stay in the report forever.
    """
    if atlas is None:
        return model.AtlasBuilder(generated=_generated(cfg, ws, selected))
    builder = model.AtlasBuilder()
    builder.atlas = atlas
    atlas.generated = _generated(cfg, ws, selected, previous=atlas.generated)
    for want, count in sorted((atlas.counters.get("resolved") or {}).items()):
        builder.count_resolved(str(want), int(count))
    keep = set((atlas.x.get(X_NAMESPACE) or {}).get(X_NOT_FROM_A_RULE) or [])
    atlas.findings = [item for item in atlas.findings if item.id in keep]
    return builder


def carries_layer(atlas: model.Atlas, layer: str) -> bool:
    """Whether one layer already stated something in this atlas.

    Read from the facts and not from `atlas.layers`, which states an intention: a pass that
    declared a layer and collected nothing must not read as a layer with content.
    """
    holders = [*atlas.entities.values(), *atlas.edges.values()]
    return any(fact.layer == layer for holder in holders for fact in holder.facts)


def out_path(cfg: Config) -> Path:
    return Path(cfg.out_dir) / ATLAS_NAME


def available(layer: str = "") -> list[str]:
    """Every registered collector name, in the order `run` would call them."""
    _register()
    names = collect.names(layer)
    return sorted(names, key=_order_key)


def default_collectors(layer: str = "") -> list[str]:
    """The committed selection, which is what a run with no `--collector` calls.

    This is `STATIC`, not the registry, so a collector another module registered on import
    cannot join a default run and the selection is the same whatever was imported first. The
    runtime collectors are outside it on purpose: a default extract reads the repository and
    neither attaches to a live system nor opens a recording nobody named.
    """
    _register()
    missing = [name for name in ORDER if name not in collect.REGISTRY]
    if missing:
        raise ContractError(
            "the committed collector order names collectors nothing registers: "
            + ", ".join(missing)
        )
    return [name for name in STATIC if not layer or _layer_of(name) == layer]


def collectors_for(layer: str) -> list[str]:
    """Every committed collector of one layer, in the order it runs."""
    _register()
    return [name for name in ORDER if _layer_of(name) == layer]


def summary(atlas: model.Atlas) -> dict[str, Any]:
    """The counts that matter, for the human and the machine readable report.

    No wall clock and no path is in here: the caller adds the timing, so the summary itself
    stays a function of the atlas.
    """
    counters = atlas.counters
    resolved = dict(counters.get("resolved") or {})
    unresolved = dict(counters.get("unresolved") or {})
    total_resolved = sum(resolved.values())
    total_unresolved = sum(unresolved.values())
    denominator = total_resolved + total_unresolved
    return {
        "profiles": [ids.parse_profile(node.id) for node in atlas.by_kind("profile")],
        "layers": list(counters.get("layers_present") or []),
        "entities": counters.get("entities", 0),
        "entities_by_kind": dict(counters.get("entities_by_kind") or {}),
        "edges": counters.get("edges", 0),
        "edges_by_kind": dict(counters.get("edges_by_kind") or {}),
        "unresolved": total_unresolved,
        "unresolved_by_want": unresolved,
        "resolved": total_resolved,
        "resolution_rate": round(total_resolved / denominator, 6) if denominator else 1.0,
        "resolution_rate_by_want": dict(counters.get("resolution_rate") or {}),
        "passes": dict(counters.get("passes") or {}),
        "findings": dict(counters.get("findings") or {}),
        "findings_by_rule": dict((counters.get("drift") or {}).get("by_rule") or {}),
        "rules_failed": sorted((counters.get("drift") or {}).get("failed") or []),
        "exceptions": exceptions(atlas),
    }


def exceptions(atlas: model.Atlas) -> list[dict[str, Any]]:
    """Every problem the runs recorded, sorted, whatever collector recorded it."""
    rows: list[dict[str, Any]] = []
    for node in atlas.by_kind("run"):
        for row in node.attrs.get("exceptions") or []:
            entry = dict(row)
            entry["run"] = node.id
            rows.append(entry)
    return sorted(rows, key=_problem_key)


# ---------------------------------------------------------------------------
# passes


def _passes(cfg: Config, selected: Sequence[str]) -> list[Pass]:
    out: list[Pass] = []
    for layer in _layers(selected):
        names = [name for name in selected if _layer_of(name) == layer]
        shared = tuple(
            name for name in names if name not in PROFILE_SCOPED and name not in PROFILE_BOUND
        )
        bound = tuple(name for name in names if name in PROFILE_BOUND)
        scoped = tuple(name for name in names if name in PROFILE_SCOPED)
        if shared:
            out.append(Pass(layer=layer, profile="", names=shared))
        if bound:
            out.append(Pass(layer=layer, profile=_sole_profile(cfg), names=bound))
        if not scoped:
            continue
        for spec in sorted(cfg.profiles, key=lambda item: item.slug):
            out.append(Pass(layer=layer, profile=spec.slug, names=scoped))
    return out


def _sole_profile(cfg: Config) -> str:
    return cfg.profiles[0].slug if len(cfg.profiles) == 1 else ""


def _run_pass(
    ctx: collect.CollectContext,
    cfg: Config,
    step: Pass,
    roots: Mapping[str, str],
    instances: Mapping[str, Any],
    say: Callable[[str], None] | None = None,
) -> str:
    spec = cfg.profile(step.profile) if step.profile else None
    profile_id = ids.build_profile(step.profile) if step.profile else None
    run_id = ctx.builder.add_run(
        step.layer,
        profile=profile_id,
        input_digest=_input_digest(cfg, ctx.ws, step, spec, roots),
        roots=dict(roots),
        env=dict(spec.env) if spec else {},
    )
    problems = ctx.builder.atlas.entities[run_id].attrs["exceptions"]
    scoped = ctx.for_profile(step.profile)
    scoped.run = run_id
    for name in step.names:
        collector = instances.get(name) or collect.get(name)
        state, reason, failure = _collect_one(collector, scoped)
        if failure is not None:  # one collector must never empty the layer
            problems.append(
                {
                    "path": f"@collector/{name}",
                    "phase": "collect",
                    "exc_type": type(failure).__name__,
                    "message": str(failure),
                }
            )
        ctx.builder.record_pass(
            run_id,
            name,
            state=state,
            reason=reason,
            # only a pass that stopped costs the atlas anything: a pass with no inputs read
            # everything there was, and naming what it would have read reads as a loss
            missing=contribution(name) if state == model.PassState.REFUSED.value else "",
        )
        if say is not None:
            say(name)
    problems.sort(key=_problem_key)
    return run_id


def contribution(name: str) -> str:
    """What the atlas loses when this collector does not finish."""
    return CONTRIBUTES.get(name, f"whatever the {name} collector reads")


def _collect_one(collector: Any, ctx: collect.CollectContext) -> tuple[str, str, Exception | None]:
    """Run one collector and say how it ended: completed, refused, or never attempted."""
    try:
        absent = _absent(collector, ctx)
    except Exception as exc:
        return (model.PassState.REFUSED.value, _reason(exc), exc)
    if absent:
        return (model.PassState.NOT_ATTEMPTED.value, absent, None)
    try:
        collector.collect(ctx)
    except Exception as exc:
        return (model.PassState.REFUSED.value, _reason(exc), exc)
    return (model.PassState.COMPLETED.value, "", None)


def _absent(collector: Any, ctx: collect.CollectContext) -> str:
    """Why this collector has nothing to read, empty when it has something.

    Optional and duck typed: a collector states it by implementing `inputs_present(ctx)`,
    returning True, or False or the sentence naming what is absent. A collector that does
    not implement it is attempted, and one that ran over an empty tree is complete rather
    than failed, so the rule cannot cry wolf on a small repository.
    """
    probe = getattr(collector, "inputs_present", None)
    if probe is None:
        return ""
    answer = probe(ctx)
    if answer is True:
        return ""
    if answer is False or answer is None:
        return NO_INPUT
    return str(answer) or NO_INPUT


def _reason(exc: Exception) -> str:
    text = " ".join(str(exc).split())
    return f"{type(exc).__name__}: {text}" if text else type(exc).__name__


def _declare_layers(builder: model.AtlasBuilder, passes: Sequence[tuple[str, str]]) -> None:
    """Presence per layer, decided after the collectors ran rather than before.

    A layer is present when a collector for it produced a fact. A collector that raised
    still leaves its run behind with the exception on it, so a reader sees that the layer
    was attempted and holds nothing, which is a different statement from an absent layer.

    A runtime layer states itself, because its own collector is the only place that knows
    whether it reached the middleware or the recording at all. This writes such a layer only
    when nothing was said about it, which is what a collector that raised leaves behind.
    """
    with_facts = _layers_with_facts(builder.atlas)
    for layer, run_id in passes:
        status = builder.atlas.layers.get(layer)
        if layer in SELF_DECLARED and status is not None and (status.present or status.reason):
            continue
        present = layer in with_facts
        builder.declare_layer(
            layer,
            present=present,
            run=run_id,
            reason="" if present else "no collector for this layer produced a fact",
            root=LAYER_ROOTS.get(layer, model.Root.SRC.value),
        )


def _declare_capabilities(builder: model.AtlasBuilder) -> None:
    """The feature probe, from the layers the atlas holds after the collectors ran."""
    atlas = builder.atlas
    marks = set(atlas.capabilities) | set(ALWAYS)
    for layer, members in LAYER_CAPABILITIES.items():
        if carries_layer(atlas, layer):
            marks.update(members)
    atlas.capabilities = sorted(marks)


def _layers_with_facts(atlas: model.Atlas) -> set[str]:
    seen: set[str] = set()
    for node in atlas.entities.values():
        for fact in node.facts:
            seen.add(fact.layer)
    for edge in atlas.edges.values():
        for fact in edge.facts:
            seen.add(fact.layer)
    return seen


def _join(builder: model.AtlasBuilder) -> None:
    """One join pass, only when both halves are there to join."""
    from rosio.join import join

    layers = builder.atlas.layers
    if not all(
        getattr(layers.get(name), "present", False)
        for name in (model.Layer.CODED.value, model.Layer.DECLARED.value)
    ):
        return
    join(builder)


def drift(atlas: model.Atlas) -> Any:
    """Run every rule over the built atlas and write the findings onto it.

    Returns the engine run, whose `failures` name every rule that could not be trusted this
    time. A rule that raises is recorded there and never empties the report.
    """
    from rosio.drift import engine

    loaded, failures = engine.load_rules(rule_families())
    result = engine.run(atlas, rules=loaded)
    result.failures = sorted([*failures, *result.failures], key=engine.RuleFailure.sort_key)
    carried = _carried(atlas.findings, result.findings)
    result.findings = sorted([*result.findings, *carried], key=model.Finding.sort_key)
    result.gated = list(result.findings)
    result.counters["carried"] = len(carried)
    result.counters["gated"] = len(result.gated)
    result.counters["rules_failed"] = len({item.rule for item in result.failures})
    result.counters["failed"] = sorted({item.rule for item in result.failures})
    engine.apply(atlas, result)
    # What no rule produced, so a later run over the same file can tell the two apart. A run
    # that extends an atlas re runs every rule over more layers than the run before it, and
    # keeping the earlier output would leave a finding the new answer contradicts.
    atlas.x.setdefault(X_NAMESPACE, {})[X_NOT_FROM_A_RULE] = sorted(item.id for item in carried)
    return result


def rule_families() -> list[str]:
    """The rule family modules under `RULE_PACKAGE`, discovered and sorted.

    Discovery rather than a list, because a list here goes stale the moment a family lands
    and a family that never loads reports nothing while reading exactly like a clean stack.
    """
    import pkgutil

    package = import_module(RULE_PACKAGE)
    skip = {"engine", "rules"}
    return sorted(
        module.name
        for module in pkgutil.iter_modules(package.__path__)
        if module.name not in skip and not module.name.startswith("_")
    )


def _carried(existing: Sequence[model.Finding], produced: Sequence[model.Finding]) -> list:
    """The findings already on the atlas that no rule restated.

    The join writes `node.ambiguous_identity` and the launch collector writes its refusals,
    and the rule that covers the runtime half of the identity case deliberately skips a
    subject the join already reported. Replacing the array instead of merging it would
    delete exactly those. A rule wins an id collision, because the rule owns the text.
    """
    ids_seen = {item.id for item in produced}
    prints = {item.fingerprint for item in produced}
    return [item for item in existing if item.id not in ids_seen and item.fingerprint not in prints]


def _input_digest(
    cfg: Config,
    ws: Workspace,
    step: Pass,
    spec: ProfileSpec | None,
    roots: Mapping[str, str],
) -> str:
    """The inputs this pass depended on, so two boxes that differ show an input difference.

    The collector names are in the digest because a run over a subset of the collectors is a
    different pass over the same tree, not the same one.

    Every argument names its entry point, the way the launch collector anchors its own list.
    Flattened without that anchor, one argument moving from the first entry point to the
    second left the flat list unchanged, and two profiles that really differ showed no input
    difference.

    A setup file inside the checkout enters as the place it holds in the checkout, the same
    form the generation block states, because the run id built from this digest is compared
    data: with the absolute path in it, one workspace at two paths was two atlases and every
    clone rewrote the committed file. What that tree holds is still in the digest, under
    `roots`, which is a content hash and not a path.
    """
    entrypoints = [entry.label for entry in spec.entrypoints] if spec else []
    args = (
        [
            f"{ids.escape_segment(entry.label)}|{key}={ws.stated_value(value)}"
            for entry in spec.entrypoints
            for key, value in entry.args
        ]
        if spec
        else []
    )
    return ids.digest_fields(
        {
            "orchestrator": "extract",
            "layer": step.layer,
            "collectors": list(step.names),
            "profile": step.profile,
            "entrypoints": entrypoints,
            "args": args,
            "env": [f"{key}={ws.stated_value(value)}" for key, value in (spec.env if spec else ())],
            "ignore": list(cfg.ignore),
            "ros_setup": ws.stated_by_content(cfg.ros_setup),
            "roots": [f"{key}={value}" for key, value in sorted(roots.items())],
        }
    )


def _record_share_reads(builder: model.AtlasBuilder, ws: Workspace) -> None:
    """Mark the files that a launch file reached through the package share.

    The workspace, not a collector, is what knows that two trees hold the same package
    relative path, so the annotation is written once here rather than once per collector. A
    file whose install copy still matches src is cited as src and only carries the note. A
    file whose copies differ is the build staleness case, and `build.stale` reads the mark
    and states it once on the file instead of once per value read from it.

    A twin is a file at the same package relative path, which is evidence and not proof that
    it is the source of the installed copy: a package that keeps a ROS 1 and a ROS 2 spelling
    of one message and installs the second leaves a different file under src at the same
    relative path, with nothing stale about the build. That is why the mark says what was
    measured and the rule leaves the cause to the reader.
    """
    for read in ws.share_reads():
        node = builder.atlas.entities.get(read.file_id)
        if node is None:
            continue
        node.attrs["via_package_share"] = True
        if not read.identical:
            node.attrs["install_copy_differs"] = True


# ---------------------------------------------------------------------------
# selection


def _register() -> None:
    for module in MODULES:
        import_module(module)


def _select(collectors: Sequence[str] | None, opt_in: Sequence[str] = ()) -> tuple[str, ...]:
    """The selection, sorted into the committed order. An unknown name is a usage error."""
    wanted = list(collectors) if collectors is not None else default_collectors()
    for name in wanted:
        collect.get(name)
    allowed = set(opt_in)
    refused = sorted(name for name in wanted if name in OPT_IN and name not in allowed)
    if refused:
        raise UsageError(
            "these collectors attach to a live system and are not reachable from extract: "
            + ", ".join(refused)
            + f". run {PROBE_VERB} instead, which takes the domain id to attach to"
        )
    unique = dict.fromkeys(wanted)
    return tuple(sorted(unique, key=_order_key))


def _order_key(name: str) -> tuple[int, str]:
    return (ORDER.index(name) if name in ORDER else len(ORDER), name)


def _layer_of(name: str) -> str:
    return collect.get(name).layer


def _layers(selected: Iterable[str]) -> list[str]:
    seen = {_layer_of(name) for name in selected}
    return sorted(seen, key=lambda layer: model.LAYER_ORDER.index(layer))


# ---------------------------------------------------------------------------
# output


def _generated(
    cfg: Config,
    ws: Workspace,
    selected: Sequence[str],
    previous: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """The generation block, dropped from `canonical_bytes` and therefore never compared.

    What it may state about where the atlas was built is decided here: the identity of each
    root, which is its kind, its label, the hash of its content and its place in the
    checkout, and never where the checkout sits on this machine. The atlas is committed and
    read by clones, so an absolute path here is rewritten by the first rebuild in every clone
    and `git merge` then refuses to start on a dirty work tree. The path a person debugging
    one machine wants is the one they passed in, and it is in the configuration and in the
    command, both of which stay local.

    A setup file of the ROS installation sits outside every root, and it is stated the way
    `Workspace.stated_by_content` states such a file, by its name and the hash of its content
    with its own prefix normalised out. `stated` handed back the machine path, which is what
    made two clones at two prefixes rewrite 29 bytes of a committed atlas on their first
    rebuild. The same rule already answers for the input digest one level in.

    A wall clock appears here only when the caller asked for a non deterministic run. The
    duration of the run is never written at all: it belongs to the caller's report. A run
    that extended an atlas keeps the collectors the earlier run named, so the block states
    everything the file was built from and not only the last pass over it.
    """
    earlier = list((previous or {}).get("collectors") or [])
    payload: dict[str, Any] = {
        "collectors": list(dict.fromkeys([*earlier, *selected])),
        "profiles": [spec.slug for spec in sorted(cfg.profiles, key=lambda item: item.slug)],
        "packages": ws.package_records(),
        "roots": ws.root_records(),
        "ros_setup": ws.stated_by_content(cfg.ros_setup),
    }
    if not cfg.deterministic:
        payload["at"] = datetime.now(timezone.utc).isoformat(timespec="seconds")
    return payload


def _write(cfg: Config, payload: Mapping[str, Any]) -> Path:
    target = out_path(cfg)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(model.dumps(payload), encoding="utf-8", newline="\n")
    return target


def _problem_key(row: Mapping[str, Any]) -> tuple[str, str, str, str]:
    return (
        str(row.get("path") or row.get("file") or ""),
        str(row.get("phase", "")),
        str(row.get("exc_type", "")),
        str(row.get("message", "")),
    )


__all__ = [
    "ALL_CAPABILITIES",
    "ATLAS_NAME",
    "CAPABILITIES",
    "CONTRIBUTES",
    "LAYER_CAPABILITIES",
    "NO_INPUT",
    "OPT_IN",
    "ORDER",
    "PROFILE_SCOPED",
    "RULE_PACKAGE",
    "RUNTIME",
    "SELF_DECLARED",
    "STATIC",
    "Pass",
    "available",
    "carries_layer",
    "collectors_for",
    "contribution",
    "default_collectors",
    "drift",
    "exceptions",
    "out_path",
    "rule_families",
    "run",
    "summary",
]
