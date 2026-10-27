"""Node identity rules, and the shared gating the graph rules use.

Five rules live here: `node.declared_not_observed`, `node.observed_only`,
`node.ambiguous_identity`, `node.identity_regressed` and `node.unnamed`. Each is a pure
function from the joined atlas to a list of findings. Nothing here re extracts, mutates an
entity or reads a file.

When each rule may speak
------------------------

Three of the five need a running system, and the observed layer arrives in M5. They are
scoped to the profiles that actually carry an observed fact, which is computed from the
facts and not from `atlas.layers`, so an atlas with the layer declared present and nothing
collected is treated as an atlas without it. On an atlas with no observed fact anywhere they
return nothing at all. A finding that fires because a layer was not collected is the worst
kind of false positive, so absence of evidence is never evidence here.

`node.identity_regressed` and `node.unnamed` read only the declared and coded layers and
speak today.

What is skipped, and why
------------------------

* a fact whose confidence is `unknown`, everywhere, per AGENTS section 6
* a synthetic identity, for `node.declared_not_observed`: its runtime name is not known, so
  its absence from a probe is a statement about rosio and not about the stack
* a node whose declared fact says `gated`, or whose attrs say `unreachable`: a condition or
  an event handler that kept a node out of one run is configuration, not drift
* a runtime randomised or executable default name in the ambiguity groups, because those are
  exactly the families that must join on the role digest and never on the name
* an observed node the probe marked `via_known_wrapper` or `internal`, and the ros2cli
  private names, so tooling that attached itself to the graph is never reported as drift
* an observed node whose `attrs.child_of` names a parent the profile launches, for
  `node.observed_only`: the parent creates it, so a restart brings it back

The observed contract this reads
--------------------------------

One fact per node per profile, layer `observed`, values `name`, `namespace`, and optionally
`via_known_wrapper`, `internal` and `lifecycle_state`. Nothing else is required and nothing
is assumed: a probe that states less produces fewer findings, never wrong ones.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable
from dataclasses import dataclass
from typing import Any

from rosio import ids, model
from rosio.drift import rules

DECLARED = model.Layer.DECLARED.value
CODED = model.Layer.CODED.value
OBSERVED = model.Layer.OBSERVED.value
ANY = model.ANY_PROFILE

NODE = model.NodeKind.NODE.value
UNKNOWN = model.Confidence.UNKNOWN.value

NAME_NOT_IDENTITY = (
    model.NameSource.RUNTIME_RANDOMISED.value,
    model.NameSource.UNKNOWN.value,
)
"""Name sources that cannot be matched against a runtime name."""


# ---------------------------------------------------------------------------
# shared reading helpers, used by this module and by rosio.drift.topics


def observed_profiles(atlas: model.Atlas) -> tuple[str, ...]:
    """Profiles that carry at least one observed fact, from the facts themselves.

    `atlas.layers` states an intention. A run that declared the layer present and collected
    nothing would let every observed rule fire on its own blind spot.
    """
    seen: set[str] = set()
    for entity in atlas.entities.values():
        if entity.kind == model.NodeKind.RUN.value:
            continue
        for fact in entity.facts:
            if fact.layer == OBSERVED and fact.confidence != UNKNOWN:
                seen.add(fact.profile)
    for edge in atlas.edges.values():
        for fact in edge.facts:
            if fact.layer == OBSERVED and fact.confidence != UNKNOWN:
                seen.add(fact.profile)
    if ANY in seen:
        seen.discard(ANY)
        seen.update(profile_ids(atlas))
    return tuple(sorted(seen))


def profile_ids(atlas: model.Atlas) -> tuple[str, ...]:
    return tuple(sorted(entity.id for entity in atlas.by_kind(model.NodeKind.PROFILE.value)))


def stated(holder: Any, layer: str, profile: str) -> model.LayerFact | None:
    """The fact one layer stated for one profile, or None.

    A fact filed against every profile counts for each of them. `rules.usable` decides what
    counts as a statement, so unknown confidence and a known wrapper are skipped here rather
    than in eleven places.
    """
    for candidate in (profile, ANY):
        fact = holder.fact(layer, candidate)
        if rules.usable(fact):
            return fact
    return None


def says_anything(holder: Any, layer: str, profile: str) -> bool:
    return stated(holder, layer, profile) is not None


def evidence_of(*facts: model.LayerFact | None) -> list[model.Source]:
    rows: list[model.Source] = []
    for fact in facts:
        if fact is not None:
            rows.extend(fact.evidence)
    return rules.sources(rows)


def slug(profile: str) -> str:
    return rules.slug_of(profile) if profile and profile != ANY else "every profile"


def disc_for(profile: str, *parts: object) -> str:
    """A discriminator over sorted content. Never an ordinal, never an iteration index."""
    tail = sorted(str(part) for part in parts)
    if not tail:
        return ids.digest(slug(profile))
    return ids.digest(slug(profile), *tail)


def human(entity_id: str) -> str:
    """A name for prose. A synthetic id never reaches a reader, so `rules.node_text` names
    the node by what launched it."""
    if ids.kind_of(entity_id) == NODE:
        return rules.node_text(entity_id)
    return ids.label_for(entity_id)


def node_entities(atlas: model.Atlas) -> list[model.Node]:
    return atlas.by_kind(NODE)


def already_reported(atlas: model.Atlas, rule: str, subject: str) -> bool:
    """The join reports the coded side of an ambiguity while it is deciding not to
    supersede. One rule, one report: this rule stays off a subject that already carries
    that rule."""
    return any(
        finding.rule == rule and finding.subject_key == subject for finding in atlas.findings
    )


# ---------------------------------------------------------------------------
# ambiguity, computed once and reused by three rules


@dataclass(frozen=True)
class Ambiguity:
    """A running node that more than one declared instance could be."""

    profile: str
    observed: str
    candidates: tuple[str, ...]


def runtime_name(entity: model.Node, fact: model.LayerFact) -> tuple[str, str]:
    """The namespace and name the probe reported for one node."""
    name = str(fact.values.get("name") or "")
    namespace = str(fact.values.get("namespace") or "")
    if not name and not ids.is_synthetic(entity.id):
        namespace, name = ids.split_ros_name(ids.parse_node(entity.id).fqn or "")
    return ids.normalise_namespace(namespace), name


def _could_be(entity_id: str, namespace: str, name: str) -> bool:
    """Whether one declared instance could be the running node.

    A randomised family appends a runtime suffix to the executable name, so the executable
    is the only thing that can be compared, and comparing it is what makes the case
    ambiguous rather than resolved.
    """
    parsed = ids.parse_node(entity_id)
    if not parsed.synthetic:
        return False
    if ids.normalise_namespace(parsed.namespace) != namespace:
        return False
    return name == parsed.executable or name.startswith(parsed.executable + "_")


def ambiguities(atlas: model.Atlas) -> tuple[Ambiguity, ...]:
    out: list[Ambiguity] = []
    entities = node_entities(atlas)
    for profile in observed_profiles(atlas):
        declared = [
            entity
            for entity in entities
            if ids.is_synthetic(entity.id) and says_anything(entity, DECLARED, profile)
        ]
        if len(declared) < 2:
            continue
        for entity in entities:
            fact = stated(entity, OBSERVED, profile)
            if fact is None or says_anything(entity, DECLARED, profile):
                continue
            namespace, name = runtime_name(entity, fact)
            if not name:
                continue
            hits = tuple(
                sorted(item.id for item in declared if _could_be(item.id, namespace, name))
            )
            if len(hits) > 1:
                out.append(Ambiguity(profile=profile, observed=entity.id, candidates=hits))
    return tuple(out)


def _gated(atlas: model.Atlas, node_id: str, profile: str) -> bool:
    """Every site that starts the node in this profile waits on a gate, so the node may not
    be up yet. The gates live on the launch entity, where one site is one entity."""
    sites = [
        atlas.entities[edge.src]
        for edge in atlas.in_edges(node_id)
        if edge.kind == model.EdgeKind.LAUNCHES.value
        and edge.src in atlas.entities
        and edge.fact(DECLARED, profile) is not None
    ]

    def gates(site: model.Node) -> Any:
        fact = site.fact(DECLARED, profile) or site.fact(DECLARED, model.ANY_PROFILE)
        return fact.values.get("gates") if fact is not None else None

    return bool(sites) and all(gates(site) for site in sites)


def _ambiguous_ids(rows: Iterable[Ambiguity]) -> set[str]:
    out: set[str] = set()
    for row in rows:
        out.add(row.observed)
        out.update(row.candidates)
    return out


# ---------------------------------------------------------------------------
# rules


@rules.register("node.declared_not_observed")
def node_declared_not_observed(atlas: model.Atlas) -> list[model.Finding]:
    """A node the repository launches that the running system does not report."""
    rows = ambiguities(atlas)
    skip = _ambiguous_ids(rows)
    out: list[model.Finding] = []
    for profile in observed_profiles(atlas):
        for entity in node_entities(atlas):
            if entity.id in skip or ids.is_synthetic(entity.id):
                continue
            if entity.name_source in NAME_NOT_IDENTITY:
                continue
            declared = stated(entity, DECLARED, profile)
            if declared is None or says_anything(entity, OBSERVED, profile):
                continue
            if _gated(atlas, entity.id, profile) or entity.attrs.get("unreachable"):
                continue
            where = slug(profile)
            name = human(entity.id)
            pkg = str(declared.values.get("pkg") or "")
            executable = str(declared.values.get("executable") or "")
            target = f"{pkg}/{executable}".strip("/") or name
            out.append(
                model.Finding(
                    rule="node.declared_not_observed",
                    severity=rules.severity_of("node.declared_not_observed"),
                    subject=entity.id,
                    title=f"{name} is launched but not running",
                    detail=(
                        f"the launch description for {where} starts {target} as {name}, and the "
                        "probe of the running system reported no node with that name"
                    ),
                    fix=(
                        "start the node, or remove it from the launch description if it is no "
                        "longer part of this profile"
                    ),
                    say=(
                        f"{name} is launched by {where} but was not running when the system was "
                        f"probed, so whatever depends on {name} has no source of data"
                    ),
                    discriminator=disc_for(profile),
                    profiles=[profile],
                    layers=[DECLARED, OBSERVED],
                    where=evidence_of(declared),
                )
            )
    return out


@rules.register("node.observed_only")
def node_observed_only(atlas: model.Atlas) -> list[model.Finding]:
    """A node the running system reports that no launch description starts."""
    skip = _ambiguous_ids(ambiguities(atlas))
    out: list[model.Finding] = []
    for profile in observed_profiles(atlas):
        for entity in node_entities(atlas):
            if entity.id in skip:
                continue
            fact = stated(entity, OBSERVED, profile)
            if fact is None:
                continue
            if fact.values.get("via_known_wrapper") or fact.values.get("internal"):
                continue
            if says_anything(entity, DECLARED, profile) or says_anything(entity, CODED, profile):
                continue
            if started_by_parent(atlas, entity, profile):
                continue
            namespace, name = runtime_name(entity, fact)
            if is_tooling(namespace, name):
                continue
            where = slug(profile)
            label = human(entity.id)
            out.append(
                model.Finding(
                    rule="node.observed_only",
                    severity=rules.severity_of("node.observed_only"),
                    subject=entity.id,
                    title=f"{label} runs and nothing in the repository starts it",
                    detail=(
                        f"the probe reported {label} in {where}, and no launch description, "
                        "command line entry point or source in this workspace names it"
                    ),
                    fix=(
                        "add the node to the launch description of this profile, or stop it if "
                        "it is left over from another run"
                    ),
                    say=(
                        f"{label} was running during {where} and no launch file in this "
                        "repository starts it, so it will not come back after a restart"
                    ),
                    discriminator=disc_for(profile),
                    profiles=[profile],
                    layers=[OBSERVED],
                    where=evidence_of(fact),
                )
            )
    return out


def started_by_parent(atlas: model.Atlas, entity: model.Node, profile: str) -> bool:
    """Whether a node the profile launches creates this one for itself.

    A costmap is a full node that no launch file names: the server that owns it constructs
    it, so it comes back whenever that server does. `attrs.child_of` names the parents the
    committed child table gave it, and one of them has to be launched in this profile.
    """
    parents = entity.attrs.get("child_of")
    if not isinstance(parents, dict):
        return False
    for parent in sorted(parents):
        held = atlas.entities.get(parent)
        if held is not None and says_anything(held, DECLARED, profile):
            return True
    return False


def is_tooling(namespace: str, name: str) -> bool:
    """ros2cli and the launch machinery attach private nodes to a live graph. The leading
    underscore is the convention they all follow."""
    return name.startswith("_") or namespace.startswith("/_")


@rules.register("node.ambiguous_identity")
def node_ambiguous_identity(atlas: model.Atlas) -> list[model.Finding]:
    """A running node that more than one declared instance could be.

    The join reports the coded side of this: two sources behind one executable stating two
    node names. This is the runtime side, and it never repeats a subject the join already
    reported.
    """
    out: list[model.Finding] = []
    for row in ambiguities(atlas):
        if already_reported(atlas, "node.ambiguous_identity", row.observed):
            continue
        entity = atlas.entities[row.observed]
        fact = stated(entity, OBSERVED, row.profile)
        label = human(row.observed)
        candidates = ", ".join(sorted(human(item) for item in row.candidates))
        where = slug(row.profile)
        facts = [stated(atlas.entities[item], DECLARED, row.profile) for item in row.candidates]
        out.append(
            model.Finding(
                rule="node.ambiguous_identity",
                severity=rules.severity_of("node.ambiguous_identity"),
                subject=row.observed,
                title=f"{label} could be any of {len(row.candidates)} launched instances",
                detail=(
                    f"{where} launches {candidates} without a name, and the running node "
                    f"{label} matches all of them, so rosio kept them apart instead of "
                    "attaching the runtime facts to one of them"
                ),
                fix="give each instance a name in the launch description",
                say=(
                    f"{where} starts {candidates} with no node name, so the running node "
                    f"{label} cannot be matched to one of them, and its parameters and "
                    "endpoints stay unattached until each instance is named"
                ),
                discriminator=disc_for(row.profile, *row.candidates),
                profiles=[row.profile],
                layers=[DECLARED, OBSERVED],
                where=evidence_of(fact, *facts),
            )
        )
    return out


@rules.register("node.identity_regressed")
def node_identity_regressed(atlas: model.Atlas) -> list[model.Finding]:
    """An identity that was resolved and is now synthetic again.

    Reads `aka`, which is where supersession records what an id used to be. A resolved id
    superseded into a synthetic one loses a name a reader had, and every derived parameter
    and plugin instance id moves with it, so it is an error and never a silent id change.
    """
    out: list[model.Finding] = []
    for entity in node_entities(atlas):
        if not ids.is_synthetic(entity.id):
            continue
        lost = sorted(
            item for item in entity.aka if ids.kind_of(item) == NODE and not ids.is_synthetic(item)
        )
        if not lost:
            continue
        names = ", ".join(ids.label_for(item) for item in lost)
        label = human(entity.id)
        out.append(
            model.Finding(
                rule="node.identity_regressed",
                severity=rules.severity_of("node.identity_regressed"),
                subject=entity.id,
                title=f"{label} lost the name {names}",
                detail=(
                    f"this node was known as {names} and is now carried under a synthetic "
                    "identity, so every parameter and plugin instance id derived from it moved "
                    "with it and any stored finding or waiver on the old id is orphaned"
                ),
                fix=(
                    "restore the name the node had, either the launch name= or the literal the "
                    "source passes to the node constructor"
                ),
                say=(
                    f"{label} used to be identified as {names} and no longer is, so waivers and "
                    "stored findings that name it no longer match"
                ),
                discriminator=ids.digest(*lost),
                profiles=list(entity.profiles),
                layers=list(entity.layers),
                where=evidence_of(*entity.facts),
            )
        )
    return out


@rules.register("node.unnamed")
def node_unnamed(atlas: model.Atlas) -> list[model.Finding]:
    """A launched node whose name nothing states, on a node whose source was read.

    `executable_default` and `runtime_randomised` are not reported: the first has a name from
    the committed table and the second cannot have one by design. `unknown` is left, and it is
    not on its own a statement about the repository. Per AGENTS section 3.4 it is the bucket
    for a name rosio could not determine, so a node the coded layer never read has an
    unknown name source whether or not its source names it. A first party executable that
    passes a literal to `super().__init__` is named in its own source, and reporting it while
    the coded layer is absent would report a gap in the reader as a gap in the stack.

    So the rule needs both halves: the launch side gives no name, and the source was read in
    this profile and gives none either. When the source does name the node the join supersedes
    the synthetic id and this rule never sees it.
    """
    out: list[model.Finding] = []
    for entity in node_entities(atlas):
        if entity.name_source != model.NameSource.UNKNOWN.value:
            continue
        for profile in sorted(entity.profiles) or [ANY]:
            declared = stated(entity, DECLARED, profile)
            if declared is None:
                continue
            coded = stated(entity, CODED, profile)
            if coded is None or coded.values.get("name"):
                continue
            form = str(declared.values.get("form") or "node")
            pkg = str(declared.values.get("pkg") or "")
            executable = str(declared.values.get("executable") or "")
            target = f"{pkg}/{executable}".strip("/") or human(entity.id)
            where = slug(profile)
            fix = (
                "pass --ros-args -r __node:=<name> in the command"
                if form == "process"
                else "set name= on the launch action"
            )
            out.append(
                model.Finding(
                    rule="node.unnamed",
                    severity=rules.severity_of("node.unnamed"),
                    subject=entity.id,
                    title=f"{target} runs without a stated name",
                    detail=(
                        f"{where} starts {target}, the source of {target} was read and passes "
                        "no name to the node constructor, and the launch description sets none "
                        "either, so its runtime name is whatever the executable defaults to"
                    ),
                    fix=fix,
                    say=(
                        f"{where} starts {target} without a node name, so no yaml block, remap "
                        f"or parameter can address it by name, and the repair is to {fix}"
                    ),
                    discriminator=disc_for(profile),
                    profiles=[profile] if profile != ANY else [],
                    layers=[CODED, DECLARED],
                    where=evidence_of(declared, coded),
                )
            )
    return out


RULES: tuple[tuple[str, Callable[[model.Atlas], list[model.Finding]]], ...] = (
    ("node.ambiguous_identity", node_ambiguous_identity),
    ("node.declared_not_observed", node_declared_not_observed),
    ("node.identity_regressed", node_identity_regressed),
    ("node.observed_only", node_observed_only),
    ("node.unnamed", node_unnamed),
)
"""Rule id to implementation, in rule id order, for a loader that reads a module list."""


def check(atlas: model.Atlas) -> list[model.Finding]:
    out: list[model.Finding] = []
    for _rule, function in RULES:
        out.extend(function(atlas))
    return sorted(out, key=model.Finding.sort_key)


__all__ = [
    "RULES",
    "Ambiguity",
    "ambiguities",
    "check",
    "disc_for",
    "evidence_of",
    "human",
    "is_tooling",
    "node_ambiguous_identity",
    "node_declared_not_observed",
    "node_identity_regressed",
    "node_observed_only",
    "node_unnamed",
    "observed_profiles",
    "profile_ids",
    "runtime_name",
    "says_anything",
    "slug",
    "stated",
]
