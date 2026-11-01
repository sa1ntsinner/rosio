"""The closed rule vocabulary, the severity per rule, and the registry rule modules use.

A finding id is a waiver key, so a rule id that is not in `COMMITTED` is a contract
violation and not a warning: an invented id makes a stored waiver stop matching without
anyone noticing. A rename goes through `ALIASES`, never through a replacement, and
`canonical` is what a waiver reader calls before it compares.

Severity is data here rather than a decision inside a rule, so the whole gate can be read
in one place. Some rules report at a second severity when the evidence is stronger or
weaker than the usual case, and both variants are named in `VARIANT_SEVERITY`.

This module also owns the guards and the prose helpers every family shares, because a
second spelling of "skip a fact rosio is not sure about" or of "name the file an
engineer edits" is how one family starts reporting what another family suppresses.
"""

from __future__ import annotations

import re
from collections.abc import Callable, Iterable, Sequence
from importlib import import_module
from typing import Any

from rosio import ids, model
from rosio.errors import ContractError, RosioError, UsageError

COMMITTED = (
    "param.shadowed",
    "param.restated_identical",
    "param.set_undeclared",
    "param.declared_unread",
    "param.orphan_block",
    "param.type_changed",
    "param.owner_name_mismatch",
    "plugin.inert_by_design",
    "plugin.unused",
    "plugin.unresolved",
    "node.declared_not_observed",
    "node.observed_only",
    "node.ambiguous_identity",
    "node.identity_regressed",
    "node.unnamed",
    "topic.no_subscriber",
    "topic.no_publisher",
    "qos.differ",
    "qos.incompatible",
    "type.differ",
    "rate.below_declared",
    "frame.no_publisher",
    "frame.orphan_root",
    "frame.not_in_urdf",
    "frame.parent_differs",
    "remap.unresolved",
    "lifecycle.unmanaged",
    "build.stale",
    "launch.unloadable",
    "launch.raised",
    "endpoint.observed_only",
    "resolution.below_budget",
    "extract.pass_incomplete",
    "extract.input_refused",
    "time.backward",
    "time.epoch_offset",
    "time.topic_skew",
    "time.frozen",
    "time.zero",
    "time.gap",
    "time.drift",
    "rmw.mixed",
    "zenoh.router_unreachable",
    "param.sim_time_mixed",
    "launch.file_not_installed",
    "model.mesh_missing",
    "build.unbuilt",
    "build.underlay",
    "remap.matches_nothing",
    "type.incompatible",
    "service.no_server",
    "launch.argument_undeclared",
    "frame.broadcast_twice",
    "param.read_undeclared",
    "controller.undefined",
    "controller.joint_missing",
    "topic.namespace_escape",
)
"""AGENTS section 10, in the order that file lists them."""

ALIASES: dict[str, str] = {}
"""Retired rule id to the id that replaced it. Empty because nothing has been renamed yet,
and the lookup is used anyway so the first rename cannot forget it."""

ERROR = model.Severity.ERROR.value
WARN = model.Severity.WARN.value
INFO = model.Severity.INFO.value

SEVERITY: dict[str, str] = {
    "param.shadowed": WARN,
    "param.restated_identical": INFO,
    "param.set_undeclared": WARN,
    "param.declared_unread": INFO,
    "param.orphan_block": WARN,
    "param.type_changed": ERROR,
    "param.owner_name_mismatch": WARN,
    "plugin.inert_by_design": INFO,
    "plugin.unused": INFO,
    "plugin.unresolved": ERROR,
    "node.declared_not_observed": WARN,
    "node.observed_only": WARN,
    "node.ambiguous_identity": WARN,
    "node.identity_regressed": ERROR,
    "node.unnamed": INFO,
    "topic.no_subscriber": INFO,
    "topic.no_publisher": WARN,
    "qos.differ": WARN,
    "qos.incompatible": ERROR,
    "type.differ": ERROR,
    "rate.below_declared": WARN,
    "frame.no_publisher": ERROR,
    "frame.orphan_root": WARN,
    "frame.not_in_urdf": INFO,
    "frame.parent_differs": ERROR,
    "remap.unresolved": INFO,
    "lifecycle.unmanaged": WARN,
    "build.stale": WARN,
    "launch.unloadable": ERROR,
    "launch.raised": ERROR,
    "endpoint.observed_only": INFO,
    "resolution.below_budget": WARN,
    "extract.pass_incomplete": ERROR,
    "extract.input_refused": WARN,
    "time.backward": ERROR,
    "time.epoch_offset": ERROR,
    "time.topic_skew": WARN,
    "time.frozen": ERROR,
    "time.zero": ERROR,
    "time.gap": WARN,
    "time.drift": WARN,
    "rmw.mixed": ERROR,
    "zenoh.router_unreachable": ERROR,
    "param.sim_time_mixed": WARN,
    "launch.file_not_installed": ERROR,
    "model.mesh_missing": ERROR,
    "build.unbuilt": WARN,
    "build.underlay": WARN,
    "remap.matches_nothing": WARN,
    "type.incompatible": ERROR,
    "service.no_server": WARN,
    "launch.argument_undeclared": WARN,
    "frame.broadcast_twice": WARN,
    "param.read_undeclared": WARN,
    "controller.undefined": ERROR,
    "controller.joint_missing": ERROR,
    "topic.namespace_escape": WARN,
}

VARIANT_SEVERITY: dict[str, dict[str, str]] = {
    "self_contradiction": {"param.shadowed": ERROR},
    "unproven": {"param.orphan_block": INFO, "lifecycle.unmanaged": INFO, "rmw.mixed": WARN},
    "unclassified": {"time.epoch_offset": WARN},
    "strong": {"time.topic_skew": ERROR, "time.drift": ERROR},
    "looked_up": {"frame.not_in_urdf": WARN},
    "renamed": {"frame.not_in_urdf": WARN},
    "stamped": {"frame.not_in_urdf": WARN},
    "shared": {"param.orphan_block": INFO},
    "unbudgeted": {"resolution.below_budget": INFO},
    "absent": {"model.mesh_missing": WARN},
}
"""`self_contradiction`: one file that sets one parameter twice with two values is a
stronger claim than one file losing to another file. `unproven`: a profile that holds a
process whose runtime name rosio could not form cannot prove a block owns nothing, so
the same rule drops to info instead of asserting a dead block, and a lifecycle node in a
profile with no manager drops to info when another process of it runs code rosio did not read,
because that code can drive the transitions through the node's services, and a node with
RMW_IMPLEMENTATION unset beside one named implementation is mixed middleware at warn when no
probe pass established the default. `unclassified` lowers an
epoch offset to warn, and `strong` raises measured topic skew or clock drift to error.
`looked_up` raises a frame the code looks up by name to warn: a lookup of a frame nothing
states fails on every call. `renamed` raises a frame the code broadcasts under a link of the
loaded model to warn when the model holds a child of that link with a close name: the robot
then has two frames for one part. `shared` lowers an orphan block to info when another profile
launches a node that reads it from the same file, which is how nav2 shares one parameters
file between its launch files. `absent` lowers a missing mesh to warn when no tree holds it,
because then the path is wrong or the file was never committed and rosio cannot say which,
while a mesh only the source holds is a missing install line."""

FAMILIES = (
    "rosio.drift.build",
    "rosio.drift.controllers",
    "rosio.drift.frames",
    "rosio.drift.nodes",
    "rosio.drift.middleware",
    "rosio.drift.params",
    "rosio.drift.plugins",
    "rosio.drift.topics",
    "rosio.drift.time",
)
"""Families this module loads by itself, imported by `load` and never at import time here.
`rosio/extract.py` discovers the family modules instead of naming them and hands that list
to the engine, and a family registers on import, so `registered` reports whatever has been
loaded by either route."""

RuleFn = Callable[[model.Atlas], list[model.Finding]]

_REGISTRY: dict[str, RuleFn] = {}
"""The one rule registry. `rosio/drift/engine.py` reads it through `registered` and
`findings_for` and keeps none of its own."""


def canonical(rule: str) -> str:
    """The current id of a rule, following one rename hop."""
    name = ALIASES.get(rule, rule)
    if name not in COMMITTED:
        raise ContractError(f"rule id {rule!r} is not in the committed vocabulary")
    return name


def severity_of(rule: str, variant: str = "") -> str:
    name = canonical(rule)
    if variant:
        table = VARIANT_SEVERITY.get(variant)
        if table is None:
            raise ContractError(f"unknown severity variant {variant!r}")
        if name in table:
            return table[name]
    return SEVERITY[name]


def register(rule: str) -> Callable[[RuleFn], RuleFn]:
    """Bind one rule id to one implementation."""
    name = canonical(rule)

    def bind(function: RuleFn) -> RuleFn:
        if name in _REGISTRY and _REGISTRY[name] is not function:
            raise ContractError(f"rule {name} already has an implementation")
        _REGISTRY[name] = function
        return function

    return bind


def load() -> None:
    for module in FAMILIES:
        import_module(module)


def registered() -> tuple[str, ...]:
    load()
    return tuple(sorted(_REGISTRY))


def findings_for(atlas: model.Atlas, only: Iterable[str] = ()) -> list[model.Finding]:
    """Every finding the registered rules produce, sorted and with unique ids."""
    load()
    wanted = [canonical(name) for name in only] if only else sorted(_REGISTRY)
    missing = [name for name in wanted if name not in _REGISTRY]
    if missing:
        raise UsageError(f"no implementation for {', '.join(sorted(missing))}")
    out: list[model.Finding] = []
    seen: dict[str, str] = {}
    for name in sorted(wanted):
        for finding in _REGISTRY[name](atlas):
            if finding.rule != name:
                raise ContractError(f"rule {name} produced a finding for {finding.rule}")
            if finding.id in seen:
                raise ContractError(f"duplicate finding id {finding.id}")
            seen[finding.id] = name
            finding.where = [row for row in finding.where if row.file not in TOOL_FILES]
            out.append(plain_finding(finding))
    return sorted(out, key=model.Finding.sort_key)


TOOL_FILES = frozenset({ids.build_file(ids.REPO_PKG, "rosio/resolve/known_names.py")})
"""Files of rosio itself that a fact may cite as evidence: the committed name table is data
with a citation per row (AGENTS section 3.4). A finding is read as a pointer into the user's
tree, so it never cites one of them."""


# ---------------------------------------------------------------------------
# guards every family uses

WRAPPER_KEY = "via_known_wrapper"


def usable(fact: model.LayerFact | None) -> bool:
    """A fact a rule is allowed to compare.

    Unknown confidence means the layer did not track the value, and a wrapper endpoint is
    a known blind spot of the source reader, not a missing endpoint.
    """
    if fact is None:
        return False
    if fact.confidence == model.Confidence.UNKNOWN.value:
        return False
    return not fact.values.get(WRAPPER_KEY)


def site(entity: model.Node, profile: str, fact: model.LayerFact) -> model.Source | None:
    """Where one profile writes a parameter.

    The candidate chain is the provenance of a parameter, so a fact carries no evidence row
    of its own. A collector that still writes one is read first, and a rule that asks for a
    site before there is one gets nothing rather than a guess.
    """
    if fact.evidence:
        return fact.evidence[0]
    return chain_site(entity, profile)


def chain_site(entity: model.Node, profile: str) -> model.Source | None:
    """The same answer read off the candidate chain alone.

    One parameter can be written in two files, and then a fact carries one evidence row and
    the chain carries both. A rule that groups per file asks for this one, because grouping
    on the evidence folds two files into whichever the collector happened to cite.
    """
    rows = [row for row in entity.candidates if row.profile == profile and row.source]
    if not rows:
        return None
    return min(rows, key=lambda row: (row.rank, row.source.file, row.source.locator)).source


def site_live(site: model.Node, profile: str) -> bool:
    """False when a launch site never runs in this profile: a condition read false, or the
    walk marked it unreachable."""
    if site.attrs.get("unreachable"):
        return False
    fact = site.fact(model.Layer.DECLARED.value, profile) or site.fact(
        model.Layer.DECLARED.value, model.ANY_PROFILE
    )
    gates = fact.values.get("gates", []) if usable(fact) else []
    return not any(gate.get("outcome") == "false" for gate in gates)


def profiles_of(atlas: model.Atlas) -> list[str]:
    return sorted(
        entity.id
        for entity in atlas.entities.values()
        if entity.kind == model.NodeKind.PROFILE.value
    )


def in_profile(entity: model.Node, profile: str) -> bool:
    return profile in entity.profiles


def blind_files(atlas: model.Atlas, want: str) -> set[str]:
    """Files holding a site whose `want` could not be read. One definition, in the join."""
    from rosio import join

    return join.blind_files(atlas, want)


def coded_sources(atlas: model.Atlas, node_id: str, profile: str) -> set[str]:
    """The source files the join bound to a node in one profile."""
    node = atlas.entities.get(node_id)
    fact = node.fact(model.Layer.CODED.value, profile) if node else None
    return set(fact.values.get("coded_sources") or []) if fact else set()


# ---------------------------------------------------------------------------
# prose, shared so two families cannot spell one file two ways

_VALUE_LIMIT = 72


def path_of(file_id: str) -> str:
    """A file id as an engineer types it, package qualified unless it is repository wide."""
    parsed = ids.parse_file(file_id)
    if parsed.package == ids.REPO_PKG:
        return parsed.relpath
    return f"{parsed.package}/{parsed.relpath}"


_FILE_TOKEN = re.compile(r"file:[^\s|'\"]+\|[^\s'\",;)]+")
_ROOT_TOKEN = re.compile(r"@([A-Za-z][\w.-]*)/")
_DROPPED_LABELS = frozenset({"outside", "checkout"})


def plain(text: str) -> str:
    """Prose without the spellings compared data uses for a path.

    A file id becomes the package relative path an engineer types, a root label the name of
    that tree, and a path outside every root or under the checkout the path alone. A reader
    met `@src/...` and `file:pkg|path` in a sentence and took them for files rosio opened.
    """

    def file_path(match: re.Match[str]) -> str:
        try:
            return path_of(match.group(0))
        except RosioError:
            return match.group(0)

    text = _FILE_TOKEN.sub(file_path, text)
    return _ROOT_TOKEN.sub(
        lambda hit: "" if hit.group(1) in _DROPPED_LABELS else f"{hit.group(1)}/", text
    )


def plain_finding(finding: model.Finding) -> model.Finding:
    """The finding with every sentence in `plain` words. Ids and evidence stay as they are."""
    finding.title = plain(finding.title)
    finding.detail = plain(finding.detail)
    finding.fix = plain(finding.fix)
    finding.say = plain(finding.say)
    return finding


def where(source: model.Source | None) -> str:
    if source is None:
        return "an unrecorded site"
    text = path_of(source.file)
    return f"{text}:{source.line_hint}" if source.line_hint else text


def edit(source: model.Source, find: str, replace: str) -> dict[str, Any]:
    """One exact replacement a fix makes on the line a source cites. A reader applies it only
    where the line holds `find` exactly once, so a moved line is left alone."""
    row: dict[str, Any] = {"file": source.file, "root": source.root, "find": find, "replace": replace}
    if source.line_hint:
        row["line_hint"] = source.line_hint
    return row


def node_text(node_id: str) -> str:
    """A node in prose. A synthetic identity is named by what launched it, never by its id."""
    parsed = ids.parse_node(node_id)
    if parsed.synthetic:
        where = f" in {parsed.namespace}" if parsed.namespace not in ("", "/") else ""
        name = (
            parsed.executable
            if parsed.executable == parsed.package
            else f"{parsed.package} {parsed.executable}"
        )
        return f"the {name} node{where}"
    return parsed.fqn or node_id


def value_text(value: Any) -> str:
    text = repr(value)
    if len(text) > _VALUE_LIMIT:
        text = text[: _VALUE_LIMIT - 3] + "..."
    return text.replace(ids.KIND_SEP + ids.SYNTHETIC, ids.KIND_SEP + " " + ids.SYNTHETIC)


def slug_of(profile: str) -> str:
    return ids.parse_profile(profile)


def sources(rows: Sequence[model.Source]) -> list[model.Source]:
    return list(dict.fromkeys(rows))


__all__ = [
    "ALIASES",
    "COMMITTED",
    "FAMILIES",
    "SEVERITY",
    "VARIANT_SEVERITY",
    "canonical",
    "chain_site",
    "findings_for",
    "in_profile",
    "load",
    "node_text",
    "path_of",
    "plain",
    "plain_finding",
    "profiles_of",
    "register",
    "registered",
    "severity_of",
    "site",
    "site_live",
    "slug_of",
    "sources",
    "usable",
    "value_text",
    "where",
]
