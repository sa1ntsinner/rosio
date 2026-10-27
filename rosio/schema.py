"""The atlas.json schema, as a python dict, plus a dependency free validator.

ATLAS_SCHEMA is a JSON Schema document (2020-12 vocabulary) published for external
tooling and for documentation. validate() does not use it: it runs hand written checks
so that rosio needs no third party validator at run time, and so it can check the
things JSON Schema cannot express, which are the ones that actually matter here:

  * an entity key equals its id and its kind equals its id prefix
  * every edge endpoint exists, and the (source kind, destination kind) pair is allowed
  * repeatable edge kinds carry a key, and no two sites collapse into one edge
  * a parameter always has a state, and has an effective value only when the state is
    effective, so an orphan yaml block can never be read as live configuration
  * counters and indexes agree with the body, because they are the CI gate surface
  * findings are sorted by severity and carry a quotable sentence with no synthetic id
"""

from __future__ import annotations

import re
from collections.abc import Mapping, Sequence
from typing import Any

from rosio import ids
from rosio.errors import ContractError
from rosio.model import (
    ANY_PROFILE,
    EDGE_DOMAINS,
    KEYED_EDGE_KINDS,
    LAYER_ORDER,
    SCHEMA_VERSION,
    SEVERITY_ORDER,
    Agreement,
    Confidence,
    EdgeKind,
    Mechanism,
    NameSource,
    NodeKind,
    ParamState,
    Root,
    Severity,
    ShadowKind,
    Want,
    canonical_value,
    dumps,
)

_VERSION_RE = re.compile(r"^\d+\.\d+$")

_LAYERS = sorted(LAYER_ORDER)
_NODE_KINDS = sorted(item.value for item in NodeKind)
_EDGE_KINDS = sorted(item.value for item in EdgeKind)
_CONFIDENCE = sorted(item.value for item in Confidence)
_SEVERITY = sorted(item.value for item in Severity)
_AGREEMENT = sorted(item.value for item in Agreement)
_PARAM_STATES = sorted(item.value for item in ParamState)
_MECHANISMS = sorted(item.value for item in Mechanism)
_SHADOW_KINDS = sorted(item.value for item in ShadowKind)
_NAME_SOURCES = sorted(item.value for item in NameSource)
_ROOTS = sorted(item.value for item in Root)
_WANTS = sorted(item.value for item in Want)

_SOURCE_SCHEMA: dict[str, Any] = {
    "type": "object",
    "required": ["file", "root"],
    "properties": {
        "file": {"type": "string", "pattern": "^file:"},
        "locator": {"type": "string"},
        "line_hint": {"type": "integer", "minimum": 1},
        "root": {"enum": _ROOTS},
    },
    "additionalProperties": False,
}

_FACT_SCHEMA: dict[str, Any] = {
    "type": "object",
    "required": ["layer", "profile", "confidence", "values", "evidence"],
    "properties": {
        "layer": {"enum": _LAYERS},
        "profile": {"type": "string"},
        "run": {"type": "string", "pattern": "^run:"},
        "confidence": {"enum": _CONFIDENCE},
        "values": {"type": "object"},
        "evidence": {"type": "array", "items": _SOURCE_SCHEMA},
    },
    "additionalProperties": False,
}

_EFFECTIVE_SCHEMA: dict[str, Any] = {
    "type": "object",
    "description": "profile id or * mapped to key mapped to the precomputed answer",
    "additionalProperties": {
        "type": "object",
        "additionalProperties": {
            "type": "object",
            "required": ["value", "layer", "confidence"],
            "properties": {
                "value": {},
                "layer": {"enum": _LAYERS},
                "confidence": {"enum": _CONFIDENCE},
                "value_type": {"type": "string"},
                "from_candidate": {"type": "integer"},
                "source": _SOURCE_SCHEMA,
            },
            "additionalProperties": False,
        },
    },
}

_CANDIDATE_SCHEMA: dict[str, Any] = {
    "type": "object",
    "required": ["rank", "wins", "value", "value_type", "mechanism", "profile", "source"],
    "properties": {
        "rank": {"type": "integer", "minimum": 0},
        "wins": {"type": "boolean"},
        "value": {},
        "value_type": {"type": "string"},
        "mechanism": {"enum": _MECHANISMS},
        "profile": {"type": "string"},
        "source": _SOURCE_SCHEMA,
        "shadow_kind": {"enum": _SHADOW_KINDS},
        "shadowed_by": {"type": "integer"},
        "applied_by": {"type": "string"},
        "guard": {"type": "string"},
        "note": {"type": "string"},
    },
    "additionalProperties": False,
}

_ENTITY_SCHEMA: dict[str, Any] = {
    "type": "object",
    "required": [
        "id",
        "kind",
        "label",
        "layers",
        "profiles",
        "agreement",
        "conflicts",
        "attrs",
        "facts",
    ],
    "properties": {
        "id": {"type": "string"},
        "kind": {"enum": _NODE_KINDS},
        "label": {"type": "string"},
        "layers": {"type": "array", "items": {"enum": _LAYERS}},
        "profiles": {"type": "array", "items": {"type": "string"}},
        "agreement": {"enum": _AGREEMENT},
        "conflicts": {"type": "array", "items": {"type": "string"}},
        "attrs": {"type": "object"},
        "facts": {"type": "array", "items": _FACT_SCHEMA},
        "aka": {"type": "array", "items": {"type": "string"}},
        "effective": _EFFECTIVE_SCHEMA,
        "state": {"enum": _PARAM_STATES},
        "candidates": {"type": "array", "items": _CANDIDATE_SCHEMA},
        "name_source": {"enum": _NAME_SOURCES},
        "unresolved": {"type": "array", "items": {"type": "string", "pattern": "^unres:"}},
    },
}

_EDGE_SCHEMA: dict[str, Any] = {
    "type": "object",
    "required": [
        "id",
        "kind",
        "src",
        "dst",
        "key",
        "layers",
        "profiles",
        "agreement",
        "conflicts",
        "attrs",
        "facts",
    ],
    "properties": {
        "id": {"type": "string", "pattern": "^edge:"},
        "kind": {"enum": _EDGE_KINDS},
        "src": {"type": "string"},
        "dst": {"type": "string"},
        "key": {"type": "string"},
        "layers": {"type": "array", "items": {"enum": _LAYERS}},
        "profiles": {"type": "array", "items": {"type": "string"}},
        "agreement": {"enum": _AGREEMENT},
        "conflicts": {"type": "array", "items": {"type": "string"}},
        "attrs": {"type": "object"},
        "facts": {"type": "array", "items": _FACT_SCHEMA},
        "effective": _EFFECTIVE_SCHEMA,
    },
}

_FINDING_SCHEMA: dict[str, Any] = {
    "type": "object",
    "required": [
        "id",
        "rule",
        "severity",
        "title",
        "detail",
        "fix",
        "say",
        "fingerprint",
        "about",
        "profiles",
        "layers",
        "where",
    ],
    "properties": {
        "id": {"type": "string", "pattern": "^find:"},
        "rule": {"type": "string", "pattern": "^[a-z][a-z0-9_.]*$"},
        "severity": {"enum": _SEVERITY},
        "title": {"type": "string"},
        "detail": {"type": "string"},
        "fix": {"type": "string"},
        "say": {"type": "string"},
        "fingerprint": {"type": "string", "pattern": "^sha256:[0-9a-f]+$"},
        "discriminator": {"type": "string"},
        "about": {
            "type": "object",
            "oneOf": [
                {"required": ["entity"]},
                {"required": ["edge"]},
            ],
            "properties": {
                "entity": {"type": "string"},
                "edge": {
                    "type": "object",
                    "required": ["kind", "src", "dst", "key"],
                    "properties": {
                        "kind": {"enum": _EDGE_KINDS},
                        "src": {"type": "string"},
                        "dst": {"type": "string"},
                        "key": {"type": "string"},
                    },
                    "additionalProperties": False,
                },
            },
            "additionalProperties": False,
        },
        "profiles": {"type": "array", "items": {"type": "string"}},
        "layers": {"type": "array", "items": {"enum": _LAYERS}},
        "where": {"type": "array", "items": _SOURCE_SCHEMA},
        "volatile": {"type": "object"},
    },
}

_UNRESOLVED_SCHEMA: dict[str, Any] = {
    "type": "object",
    "required": ["id", "want", "owner", "expr", "reason", "resolvable_by", "evidence"],
    "properties": {
        "id": {"type": "string", "pattern": "^unres:"},
        "want": {"enum": _WANTS},
        "owner": {"type": "string"},
        "expr": {"type": "string"},
        "reason": {"type": "string"},
        "role": {"type": "string"},
        "construction": {"type": "string"},
        "written_in": {"type": "string"},
        "resolvable_by": {"type": "array", "items": {"type": "string"}},
        "evidence": {"type": "array", "items": _SOURCE_SCHEMA},
    },
    "additionalProperties": False,
}

ATLAS_SCHEMA: dict[str, Any] = {
    "$schema": "https://json-schema.org/draft/2020-12/schema",
    "$id": "https://rosio.tools/schema/atlas-1.json",
    "title": "rosio atlas",
    "description": (
        "Four layer atlas of a ROS 2 system. One entity per id, one fact per "
        "(layer, profile), every answer precomputed on the entity."
    ),
    "type": "object",
    "required": [
        "schema_version",
        "generator",
        "capabilities",
        "layers",
        "counters",
        "indexes",
        "entities",
        "edges",
        "findings",
        "unresolved",
    ],
    "properties": {
        "schema_version": {"type": "string", "pattern": r"^\d+\.\d+$"},
        "generator": {
            "type": "object",
            "required": ["name", "version"],
            "properties": {
                "name": {"const": "rosio"},
                "version": {"type": "string"},
                "determinism": {"type": "string"},
            },
        },
        "capabilities": {"type": "array", "items": {"type": "string"}, "uniqueItems": True},
        "generated": {
            "type": "object",
            "description": "excluded from canonical_bytes, never compared",
        },
        "layers": {
            "type": "object",
            "additionalProperties": {
                "type": "object",
                "required": ["present", "runs", "root"],
                "properties": {
                    "present": {"type": "boolean"},
                    "runs": {"type": "array", "items": {"type": "string"}},
                    "reason": {"type": "string"},
                    "root": {"enum": _ROOTS},
                },
                "additionalProperties": False,
            },
        },
        "counters": {"type": "object"},
        "indexes": {"type": "object"},
        "entities": {"type": "object", "additionalProperties": _ENTITY_SCHEMA},
        "edges": {"type": "object", "additionalProperties": _EDGE_SCHEMA},
        "findings": {"type": "array", "items": _FINDING_SCHEMA},
        "unresolved": {"type": "array", "items": _UNRESOLVED_SCHEMA},
        "x": {"type": "object"},
    },
}


def validate(atlas_dict: Mapping[str, Any]) -> list[str]:
    """Check one atlas document. Returns a sorted list of human readable violations.

    An empty list means the document satisfies the contract. Unknown enum members and
    unknown keys from a newer minor schema are reported as violations by this build,
    but a reader must still tolerate them, which is why loading does not call validate.
    """
    problems: list[str] = []
    if not isinstance(atlas_dict, Mapping):
        return ["atlas: document is not an object"]

    _check_header(atlas_dict, problems)
    entities = atlas_dict.get("entities")
    edges = atlas_dict.get("edges")
    if not isinstance(entities, Mapping):
        problems.append("entities: missing or not an object")
        entities = {}
    if not isinstance(edges, Mapping):
        problems.append("edges: missing or not an object")
        edges = {}

    for key in sorted(entities):
        _check_entity(str(key), entities[key], problems)
    for key in sorted(edges):
        _check_edge(str(key), edges[key], entities, problems)

    _check_findings(atlas_dict, entities, edges, problems)
    _check_unresolved(atlas_dict, entities, problems)
    _check_counters(atlas_dict, entities, edges, problems)
    _check_indexes(atlas_dict, entities, edges, problems)
    return sorted(dict.fromkeys(problems))


def validate_or_raise(atlas_dict: Mapping[str, Any]) -> None:
    problems = validate(atlas_dict)
    if problems:
        raise ContractError("atlas does not satisfy the schema: " + "; ".join(problems[:5]))


# ---------------------------------------------------------------------------


def _check_header(atlas_dict: Mapping[str, Any], problems: list[str]) -> None:
    for key in ATLAS_SCHEMA["required"]:
        if key not in atlas_dict:
            problems.append(f"atlas: required key {key} is missing")
    version = atlas_dict.get("schema_version")
    if not isinstance(version, str) or not _VERSION_RE.match(version):
        problems.append("schema_version: must look like 1.0")
    elif version.split(".", 1)[0] != SCHEMA_VERSION.split(".", 1)[0]:
        problems.append(
            f"schema_version: major {version} is not readable by this build "
            f"(expected {SCHEMA_VERSION})"
        )
    generator = atlas_dict.get("generator")
    if not isinstance(generator, Mapping) or generator.get("name") != "rosio":
        problems.append("generator: must be an object with name rosio")
    capabilities = atlas_dict.get("capabilities")
    if isinstance(capabilities, list):
        if capabilities != sorted(capabilities):
            problems.append("capabilities: must be sorted")
        if len(set(capabilities)) != len(capabilities):
            problems.append("capabilities: must be unique")
    else:
        problems.append("capabilities: must be an array")
    layers = atlas_dict.get("layers")
    if isinstance(layers, Mapping):
        for name in sorted(layers):
            if name not in LAYER_ORDER:
                problems.append(f"layers.{name}: unknown layer")
            row = layers[name]
            if not isinstance(row, Mapping) or not isinstance(row.get("present"), bool):
                problems.append(f"layers.{name}: needs a boolean present")
                continue
            runs = row.get("runs") or []
            if list(runs) != sorted(runs):
                problems.append(f"layers.{name}.runs: must be sorted")
            for run in runs:
                _check_id(f"layers.{name}.runs", str(run), "run", problems)
    else:
        problems.append("layers: must be an object")


def _check_id(where: str, value: str, kind: str | None, problems: list[str]) -> str | None:
    try:
        parsed = ids.parse(value)
    except ContractError as exc:
        problems.append(f"{where}: {exc}")
        return None
    if kind is not None and parsed.kind != kind:
        problems.append(f"{where}: expected a {kind} id, got {value}")
        return None
    return parsed.kind


def _check_entity(key: str, node: Any, problems: list[str]) -> None:
    where = f"entities.{key}"
    if not isinstance(node, Mapping):
        problems.append(f"{where}: not an object")
        return
    for required in _ENTITY_SCHEMA["required"]:
        if required not in node:
            problems.append(f"{where}: required key {required} is missing")
    if node.get("id") != key:
        problems.append(f"{where}: id {node.get('id')!r} does not match its key")
    kind = _check_id(where, key, None, problems)
    if kind is None:
        return
    if kind not in _NODE_KINDS:
        problems.append(f"{where}: unknown entity kind {kind}")
    if node.get("kind") != kind:
        problems.append(f"{where}: kind {node.get('kind')!r} does not match the id prefix {kind}")
    if node.get("agreement") not in _AGREEMENT:
        problems.append(f"{where}.agreement: unknown value {node.get('agreement')!r}")
    conflicts = node.get("conflicts") or []
    if list(conflicts) != sorted(conflicts):
        problems.append(f"{where}.conflicts: must be sorted")
    _check_facts(where, node.get("facts") or [], problems)
    layers = node.get("layers") or []
    seen = sorted({fact.get("layer") for fact in node.get("facts") or []})
    if sorted(layers) != seen:
        problems.append(f"{where}.layers: {sorted(layers)} does not match the facts {seen}")
    if kind in (NodeKind.PARAM.value, NodeKind.PPARAM.value):
        _check_param(where, node, problems)
    elif node.get("state") is not None:
        problems.append(f"{where}.state: only a parameter carries a state")
    name_source = node.get("name_source")
    if name_source is not None and name_source not in _NAME_SOURCES:
        problems.append(f"{where}.name_source: unknown value {name_source!r}")
    if kind == NodeKind.NODE.value and not name_source:
        problems.append(f"{where}: every node must record its name_source")
    for entry in node.get("unresolved") or []:
        _check_id(f"{where}.unresolved", str(entry), "unres", problems)


def _check_param(where: str, node: Mapping[str, Any], problems: list[str]) -> None:
    state = node.get("state")
    if state is None:
        problems.append(f"{where}.state: a parameter must always carry a state")
        return
    if state not in _PARAM_STATES:
        problems.append(f"{where}.state: unknown value {state!r}")
    effective = node.get("effective") or {}
    if state != ParamState.EFFECTIVE.value and effective:
        problems.append(
            f"{where}: state is {state} but an effective value is present, "
            "effective must be empty unless the state is effective"
        )
    ranks: set[tuple[str, int]] = set()
    winners: dict[str, int] = {}
    previous: tuple[str, int, str, str] | None = None
    for index, candidate in enumerate(node.get("candidates") or []):
        spot = f"{where}.candidates[{index}]"
        if not isinstance(candidate, Mapping):
            problems.append(f"{spot}: not an object")
            continue
        for required in _CANDIDATE_SCHEMA["required"]:
            if required not in candidate:
                problems.append(f"{spot}: required key {required} is missing")
        if candidate.get("mechanism") not in _MECHANISMS:
            problems.append(f"{spot}.mechanism: unknown value {candidate.get('mechanism')!r}")
        shadow = candidate.get("shadow_kind")
        if shadow is not None and shadow not in _SHADOW_KINDS:
            problems.append(f"{spot}.shadow_kind: unknown value {shadow!r}")
        profile = str(candidate.get("profile", ANY_PROFILE))
        rank = candidate.get("rank")
        if not isinstance(rank, int):
            problems.append(f"{spot}.rank: must be an integer")
            continue
        if (profile, rank) in ranks:
            problems.append(f"{spot}: rank {rank} repeats in profile {profile}")
        ranks.add((profile, rank))
        if candidate.get("wins"):
            winners[profile] = winners.get(profile, 0) + 1
        source = candidate.get("source") or {}
        _check_source(f"{spot}.source", source, problems)
        order = (profile, rank, str(source.get("file", "")), str(source.get("locator", "")))
        if previous is not None and order < previous:
            problems.append(f"{spot}: candidates must be sorted by profile then rank")
        previous = order
    for profile, count in sorted(winners.items()):
        if count > 1:
            problems.append(f"{where}: {count} winning candidates in profile {profile}")
    if state == ParamState.EFFECTIVE.value and node.get("candidates") and not winners:
        problems.append(f"{where}: state is effective but no candidate wins")


def _check_source(where: str, source: Any, problems: list[str]) -> None:
    if not isinstance(source, Mapping):
        problems.append(f"{where}: not an object")
        return
    _check_id(where, str(source.get("file", "")), "file", problems)
    if source.get("root") not in _ROOTS:
        problems.append(f"{where}.root: unknown value {source.get('root')!r}")
    for key in source:
        if key not in _SOURCE_SCHEMA["properties"]:
            problems.append(f"{where}: unknown key {key}")


def _check_facts(where: str, facts: Sequence[Any], problems: list[str]) -> None:
    slots: set[tuple[str, str]] = set()
    previous: tuple[int, str, str] | None = None
    for index, fact in enumerate(facts):
        spot = f"{where}.facts[{index}]"
        if not isinstance(fact, Mapping):
            problems.append(f"{spot}: not an object")
            continue
        for required in _FACT_SCHEMA["required"]:
            if required not in fact:
                problems.append(f"{spot}: required key {required} is missing")
        layer = str(fact.get("layer"))
        profile = str(fact.get("profile", ANY_PROFILE))
        if layer not in LAYER_ORDER:
            problems.append(f"{spot}.layer: unknown layer {layer!r}")
        if fact.get("confidence") not in _CONFIDENCE:
            problems.append(f"{spot}.confidence: unknown value {fact.get('confidence')!r}")
        if profile != ANY_PROFILE:
            _check_id(spot, profile, "profile", problems)
        if fact.get("run") is not None:
            _check_id(spot, str(fact["run"]), "run", problems)
        if (layer, profile) in slots:
            problems.append(f"{spot}: a second fact in slot ({layer}, {profile})")
        slots.add((layer, profile))
        order = (
            LAYER_ORDER.index(layer) if layer in LAYER_ORDER else len(LAYER_ORDER),
            layer,
            profile,
        )
        if previous is not None and order < previous:
            problems.append(f"{spot}: facts must be sorted by layer then profile")
        previous = order
        for source_index, source in enumerate(fact.get("evidence") or []):
            _check_source(f"{spot}.evidence[{source_index}]", source, problems)
        values = fact.get("values")
        if not isinstance(values, Mapping):
            problems.append(f"{spot}.values: must be an object")
            continue
        try:
            canonical_value(dict(values))
        except ContractError as exc:
            problems.append(f"{spot}.values: {exc}")


def _check_edge(key: str, edge: Any, entities: Mapping[str, Any], problems: list[str]) -> None:
    where = f"edges.{key}"
    if not isinstance(edge, Mapping):
        problems.append(f"{where}: not an object")
        return
    for required in _EDGE_SCHEMA["required"]:
        if required not in edge:
            problems.append(f"{where}: required key {required} is missing")
    if edge.get("id") != key:
        problems.append(f"{where}: id {edge.get('id')!r} does not match its key")
    kind = str(edge.get("kind", ""))
    src = str(edge.get("src", ""))
    dst = str(edge.get("dst", ""))
    edge_key = str(edge.get("key", ""))
    try:
        parsed = ids.parse_edge(key)
    except ContractError as exc:
        problems.append(f"{where}: {exc}")
        return
    if (parsed.kind, parsed.src, parsed.dst, parsed.key) != (kind, src, dst, edge_key):
        problems.append(f"{where}: the handle does not encode kind, src, dst and key")
    if kind not in _EDGE_KINDS:
        problems.append(f"{where}.kind: unknown edge kind {kind!r}")
        return
    if kind in KEYED_EDGE_KINDS and not edge_key:
        problems.append(f"{where}: edge kind {kind} needs a key over its producing site")
    for endpoint in (src, dst):
        if endpoint not in entities:
            problems.append(f"{where}: endpoint {endpoint} is not an entity")
    if src in entities and dst in entities:
        src_kinds, dst_kinds = EDGE_DOMAINS[kind]
        src_kind = str(entities[src].get("kind"))
        dst_kind = str(entities[dst].get("kind"))
        if src_kind not in src_kinds or dst_kind not in dst_kinds:
            problems.append(f"{where}: {kind} is not allowed from {src_kind} to {dst_kind}")
    if edge.get("agreement") not in _AGREEMENT:
        problems.append(f"{where}.agreement: unknown value {edge.get('agreement')!r}")
    _check_facts(where, edge.get("facts") or [], problems)


def _check_findings(
    atlas_dict: Mapping[str, Any],
    entities: Mapping[str, Any],
    edges: Mapping[str, Any],
    problems: list[str],
) -> None:
    findings = atlas_dict.get("findings")
    if not isinstance(findings, list):
        problems.append("findings: must be an array")
        return
    seen: set[str] = set()
    previous: tuple[int, str, str, str] | None = None
    for index, finding in enumerate(findings):
        where = f"findings[{index}]"
        if not isinstance(finding, Mapping):
            problems.append(f"{where}: not an object")
            continue
        for required in _FINDING_SCHEMA["required"]:
            if required not in finding:
                problems.append(f"{where}: required key {required} is missing")
        rule = str(finding.get("rule", ""))
        severity = str(finding.get("severity", ""))
        if severity not in _SEVERITY:
            problems.append(f"{where}.severity: unknown value {severity!r}")
        finding_id = str(finding.get("id", ""))
        if finding_id in seen:
            problems.append(f"{where}: duplicate finding id {finding_id}")
        seen.add(finding_id)
        about = finding.get("about") or {}
        subject = ""
        if "entity" in about:
            subject = str(about["entity"])
            if subject not in entities:
                problems.append(f"{where}: subject {subject} is not an entity")
        elif "edge" in about:
            ref = about["edge"]
            if not isinstance(ref, Mapping):
                problems.append(f"{where}.about.edge: not an object")
            else:
                subject = ids.build_edge(
                    str(ref.get("kind")),
                    str(ref.get("src")),
                    str(ref.get("dst")),
                    str(ref.get("key", "")),
                )
                if subject not in edges:
                    problems.append(f"{where}: subject edge is not in the atlas")
        else:
            problems.append(f"{where}.about: needs an entity or an edge")
        disc = str(finding.get("discriminator", ""))
        if subject:
            expected = ids.build_finding(rule, subject, disc)
            if finding_id != expected:
                problems.append(f"{where}.id: expected {expected}")
        say = str(finding.get("say", ""))
        if not say:
            problems.append(f"{where}.say: a finding needs one quotable sentence")
        sigil = ids.KIND_SEP + ids.SYNTHETIC
        if sigil in say or sigil in str(finding.get("title", "")):
            problems.append(f"{where}: say and title must not contain a synthetic id")
        for source_index, source in enumerate(finding.get("where") or []):
            _check_source(f"{where}.where[{source_index}]", source, problems)
        order = (
            SEVERITY_ORDER.get(severity, len(SEVERITY_ORDER)),
            rule,
            subject,
            disc,
        )
        if previous is not None and order < previous:
            problems.append(f"{where}: findings must be sorted by severity, rule then subject")
        previous = order


def _check_unresolved(
    atlas_dict: Mapping[str, Any], entities: Mapping[str, Any], problems: list[str]
) -> None:
    entries = atlas_dict.get("unresolved")
    if not isinstance(entries, list):
        problems.append("unresolved: must be an array")
        return
    seen: set[str] = set()
    for index, entry in enumerate(entries):
        where = f"unresolved[{index}]"
        if not isinstance(entry, Mapping):
            problems.append(f"{where}: not an object")
            continue
        for required in _UNRESOLVED_SCHEMA["required"]:
            if required not in entry:
                problems.append(f"{where}: required key {required} is missing")
        entry_id = str(entry.get("id", ""))
        if entry_id in seen:
            problems.append(f"{where}: duplicate unresolved id {entry_id}")
        seen.add(entry_id)
        if entry.get("want") not in _WANTS:
            problems.append(f"{where}.want: unknown value {entry.get('want')!r}")
        owner = str(entry.get("owner", ""))
        expected = ids.build_unresolved(
            owner,
            str(entry.get("expr", "")),
            str(entry.get("want")),
            construction=str(entry.get("construction", "")),
        )
        if entry_id != expected:
            problems.append(f"{where}.id: expected {expected}")
        if owner not in entities:
            problems.append(f"{where}: owner {owner} is not an entity")
        else:
            mirror = entities[owner].get("unresolved") or []
            if entry_id not in mirror:
                problems.append(
                    f"{where}: not mirrored on {owner}, a reader of one entity would see "
                    "a complete extraction"
                )
        for source_index, source in enumerate(entry.get("evidence") or []):
            _check_source(f"{where}.evidence[{source_index}]", source, problems)
    if [str(entry.get("id", "")) for entry in entries if isinstance(entry, Mapping)] != sorted(
        str(entry.get("id", "")) for entry in entries if isinstance(entry, Mapping)
    ):
        problems.append("unresolved: must be sorted by id")


def _check_counters(
    atlas_dict: Mapping[str, Any],
    entities: Mapping[str, Any],
    edges: Mapping[str, Any],
    problems: list[str],
) -> None:
    counters = atlas_dict.get("counters")
    if not isinstance(counters, Mapping):
        problems.append("counters: must be an object")
        return
    if counters.get("entities") != len(entities):
        problems.append(f"counters.entities: {counters.get('entities')!r} != {len(entities)}")
    if counters.get("edges") != len(edges):
        problems.append(f"counters.edges: {counters.get('edges')!r} != {len(edges)}")
    severities: dict[str, int] = dict.fromkeys(SEVERITY_ORDER, 0)
    for finding in atlas_dict.get("findings") or []:
        if isinstance(finding, Mapping):
            name = str(finding.get("severity"))
            severities[name] = severities.get(name, 0) + 1
    if dict(counters.get("findings") or {}) != severities:
        problems.append("counters.findings: does not match the findings array")
    wants: dict[str, int] = {}
    for entry in atlas_dict.get("unresolved") or []:
        if isinstance(entry, Mapping):
            name = str(entry.get("want"))
            wants[name] = wants.get(name, 0) + 1
    if dict(counters.get("unresolved") or {}) != wants:
        problems.append("counters.unresolved: does not match the unresolved array")
    if "resolution_rate" not in counters:
        problems.append("counters.resolution_rate: missing, it is the CI gate surface")


def _check_indexes(
    atlas_dict: Mapping[str, Any],
    entities: Mapping[str, Any],
    edges: Mapping[str, Any],
    problems: list[str],
) -> None:
    indexes = atlas_dict.get("indexes")
    if not isinstance(indexes, Mapping):
        problems.append("indexes: must be an object")
        return
    by_kind: dict[str, list[str]] = {}
    for key in sorted(entities):
        node = entities[key]
        if isinstance(node, Mapping):
            by_kind.setdefault(str(node.get("kind")), []).append(str(key))
    if _as_index(indexes.get("by_kind")) != by_kind:
        problems.append("indexes.by_kind: does not match the entities table")
    out: dict[str, list[str]] = {}
    into: dict[str, list[str]] = {}
    for key in sorted(edges):
        edge = edges[key]
        if not isinstance(edge, Mapping):
            continue
        out.setdefault(str(edge.get("src")), []).append(str(key))
        into.setdefault(str(edge.get("dst")), []).append(str(key))
    if _as_index(indexes.get("out")) != out:
        problems.append("indexes.out: does not match the edges table")
    if _as_index(indexes.get("in")) != into:
        problems.append("indexes.in: does not match the edges table")
    differ = sorted(
        [
            str(key)
            for key, node in entities.items()
            if isinstance(node, Mapping) and node.get("agreement") == Agreement.DIFFER.value
        ]
        + [
            str(key)
            for key, edge in edges.items()
            if isinstance(edge, Mapping) and edge.get("agreement") == Agreement.DIFFER.value
        ]
    )
    if list(indexes.get("differ") or []) != differ:
        problems.append("indexes.differ: does not match the bodies marked differ")


def _as_index(value: Any) -> dict[str, list[str]]:
    if not isinstance(value, Mapping):
        return {}
    return {str(key): [str(item) for item in (value[key] or [])] for key in value}


def schema_json() -> str:
    """The schema as pinned JSON text, for publishing next to the atlas."""
    return dumps(ATLAS_SCHEMA)


__all__ = ["ATLAS_SCHEMA", "schema_json", "validate", "validate_or_raise"]
