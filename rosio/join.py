"""The join phase: one pass, after every collector, idempotent.

The declared layer knows that a node was launched from a package executable with a remap
`cmd_vel:=cmd_vel_smoothed`. The coded layer knows that the source of that
executable publishes the literal `cmd_vel`. Neither half is the answer, and no existing tool
produces the join: rqt shows a running graph with no source, a code graph tool shows symbols
with no remaps. This module produces it.

Six responsibilities, in order:

1. Bind an executable to the node instances the declared layer launched from it. One
   executable produces several instances with different namespaces and different remaps, so
   the binding is one to many and every derived fact carries the profile it belongs to. A
   composable node has no executable at all, so it reaches its code through the plugin lookup
   name, then the registered component class, then the symbol, which is the order AGENTS
   section 3.4 states. A process a launch file starts by path has neither: it names a script
   the workspace holds, and its argv is the only statement of which one, so the file the argv
   names is the last table asked. Without it the endpoints of a first party script are read
   and never attached to anything, and a rule that asks who publishes a name reads that
   absence as ignorance.
2. Apply the remaps. A coded endpoint's raw name, the node's namespace and the node's ordered
   remap rules give the absolute topic, service or action. A name that still does not resolve
   becomes an unresolved entry with the raw expression, never a guessed topic.
3. Supersede identity when the code states what the launch file did not. A node the declared
   layer named from the executable table, whose source states a literal node name, becomes
   the real node id. An ambiguous match never supersedes: both entities stay and exactly one
   `node.ambiguous_identity` finding is emitted. A process whose source names each of its
   nodes with a literal of its own, started by a site that names none, splits into one node
   per name, and a site that does name it gives all of them that one name, which is reported.
4. Attach the coded parameter declarations to the parameter entities the declared layer
   built, so a parameter set in yaml that no code declares and a parameter declared in code
   that no yaml sets are both readable in one pass.
5. Bind the frames. A broadcast states a parent and a child, a lookup states a target and a
   source, and either side can arrive as a parameter value rather than as a literal. The node
   is the declared layer's, the frames are the source's, and only the join owns both, so
   without this pass no `broadcasts_tf` and no `listens_tf` edge exists anywhere.
6. Recompute once at the end.

What this module reads is the record block, and AGENTS section 2.2 is the contract, not this
docstring. It has one key per record kind and one spelling per key: `endpoints`, `nodes`,
`parameters`, `parameter_reads`, `plugin_parameters`, `tf_broadcasts`, `tf_lookups`,
`classes`, `class_templates`, `constructions` and `type_aliases`, each
a list of flat rows, carried in a `coded` fact's `values` or in the entity's `attrs` for a
collector that records an inventory rather than a fact. The carrier entity is an `exe`, a
`sym` or a `file`, and the executables it belongs to are found through the `runs` and
`defines` edges plus the `sources` and `linked_targets` the build files declare, so a symbol
with no `runs` edge of its own still binds. There is one spelling per key and no fallback:
a key outside the block reaches nothing.

Idempotence. Running the join twice changes nothing: every write is a merge of identical
content, a supersession whose old id is gone is skipped, a finding that exists is not
repeated, an edge key is a digest over the content the edge carries rather than over an
ordinal, and the resolution counters are contributed once, guarded by the marker the first
pass leaves in `atlas.x`. A frame that comes from a parameter is read off the candidate
chain and not off the precomputed `effective` block, because that block is written by the
recompute this pass ends with and would otherwise differ between the two passes.
"""

from __future__ import annotations

import re
from collections import Counter
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field, replace
from typing import Any

from rosio import ids, model
from rosio.errors import ContractError
from rosio.resolve import known_names, ros_args, wrappers, yaml_lines

CODED = model.Layer.CODED.value
DECLARED = model.Layer.DECLARED.value
MARKER = "join"

ENDPOINT_EDGES: dict[str, tuple[str, str]] = {
    "publisher": (model.EdgeKind.PUBLISHES.value, "topic"),
    "subscriber": (model.EdgeKind.SUBSCRIBES.value, "topic"),
    "service_server": (model.EdgeKind.SERVES.value, "service"),
    "service_client": (model.EdgeKind.CALLS.value, "service"),
    "action_server": (model.EdgeKind.SERVES.value, "action"),
    "action_client": (model.EdgeKind.CALLS.value, "action"),
}

WANTS = {
    "topic": model.Want.TOPIC_NAME.value,
    "service": model.Want.SERVICE_NAME.value,
    "action": model.Want.ACTION_NAME.value,
}

BUILDERS = {
    "topic": ids.build_topic,
    "service": ids.build_service,
    "action": ids.build_action,
}

RECORD_KEYS = (
    "classes",
    "class_templates",
    "constructions",
    "type_aliases",
    "endpoints",
    "nodes",
    "parameters",
    "parameter_reads",
    "plugin_parameters",
    "tf_broadcasts",
    "tf_lookups",
    "plugin_loaders",
)
"""The record block, which is the whole of what a coded collector may hand the join. One
key per record kind, one spelling per key, and AGENTS section 2.2 states the rows. A key
outside this tuple is not a record and is not read."""

SCRIPT_SUFFIX = ".py"
"""What a launch file starts by path on this stack. A process whose argv names a file the
coded layer read is bound to that file, and the argv of every other form names none."""

COMPONENT_MACRO = "COMPONENTS_REGISTER_NODE"
"""The registration macro that turns a class into a loadable component. A composable node
has no executable, so this registration is the only thing that ties the launched plugin
class to the file that implements it."""

_BROADCAST_SOURCES = {"python": "py_broadcast", "cpp": "cpp_broadcast"}
"""What a `tf_parent` edge names as the statement it came from. The frame rules read the
direction of the pair off this value, so it has to name the language of the broadcast and
not the pass that wrote the edge."""

WRAPPER_KEYS = ("via_known_wrapper", "wrapper", "via_wrapper")
"""Where a record states the library that created the endpoint. `via_known_wrapper` is the
spelling both collectors emit and the flag AGENTS section 6 requires on the edge."""


# ---------------------------------------------------------------------------
# what the coded layer left behind


@dataclass(frozen=True)
class Carrier:
    """One entity holding coded records, with the file its evidence cites."""

    owner: str
    file: str
    symbol: str
    nodes: tuple[dict[str, Any], ...] = ()
    endpoints: tuple[dict[str, Any], ...] = ()
    parameters: tuple[dict[str, Any], ...] = ()
    parameter_reads: tuple[dict[str, Any], ...] = ()
    plugin_parameters: tuple[dict[str, Any], ...] = ()
    tf_broadcasts: tuple[dict[str, Any], ...] = ()
    tf_lookups: tuple[dict[str, Any], ...] = ()
    language: str = ""
    constructions: tuple[dict[str, Any], ...] = ()
    type_aliases: tuple[dict[str, Any], ...] = ()
    construction_sources: tuple[model.Source, ...] = ()
    classes: tuple[dict[str, Any], ...] = ()
    class_templates: tuple[dict[str, Any], ...] = ()
    plugin_loaders: tuple[dict[str, Any], ...] = ()

    def declarations(self) -> tuple[dict[str, Any], ...]:
        """Every parameter this carrier declares, node scoped and plugin scoped together.

        Both are read at one call site because the row itself decides which it is: a row
        carrying a base class or a suffix is the plugin's and never becomes a node
        parameter. They are two keys and not one because they are two record kinds, and the
        list that merged them said the opposite."""
        return self.parameters + self.plugin_parameters


@dataclass(frozen=True)
class Instance:
    """One node instance the declared layer launched, in one profile."""

    node_id: str
    profile: str
    pkg: str
    executable: str
    namespace: str
    name: str
    name_source: str
    known_name: str
    remaps: tuple[tuple[str, str], ...]
    plugin_class: str = ""
    namespace_stated: bool = False
    argv: tuple[str, ...] = ()


@dataclass(frozen=True)
class Bound:
    """A raw code name after the namespace and the remap rules were applied."""

    name: str
    rule: tuple[str, str] | None = None


@dataclass
class JoinReport:
    """What the join did, for the milestone report. Not stored in the atlas."""

    instances: int = 0
    executables_bound: int = 0
    components_joined: int = 0
    endpoints_bound: int = 0
    endpoints_remapped: int = 0
    endpoints_unresolved: int = 0
    wrapper_endpoints: int = 0
    tf_edges: int = 0
    tf_unresolved: int = 0
    tf_from_parameter: int = 0
    superseded: int = 0
    ambiguous: int = 0
    parameters_attached: int = 0
    parameters_code_only: int = 0
    parameter_reads_bound: int = 0
    scripts_bound: int = 0
    unbound_carriers: int = 0
    guarded_off: int = 0
    remap_cases: list[tuple[str, str, str, str]] = field(default_factory=list)
    contradictions: list[tuple[str, str, str, str]] = field(default_factory=list)
    shared_sources: list[tuple[str, tuple[str, ...]]] = field(default_factory=list)
    resolved_sites: set[tuple[str, str, str, str]] = field(default_factory=set)
    blocked_sites: set[tuple[str, str, str, str]] = field(default_factory=set)
    tf_cases: list[tuple[str, str, str, str]] = field(default_factory=list)
    coded_blind: set[tuple[str, str, str, int | None]] = field(default_factory=set)

    def summary(self) -> dict[str, Any]:
        return {
            "components_joined": self.components_joined,
            "endpoints_bound": self.endpoints_bound,
            "endpoints_remapped": self.endpoints_remapped,
            "endpoints_unresolved": self.endpoints_unresolved,
            "guarded_off": self.guarded_off,
            "instances": self.instances,
            "parameter_reads_bound": self.parameter_reads_bound,
            "parameters_attached": self.parameters_attached,
            "parameters_code_only": self.parameters_code_only,
            "scripts_bound": self.scripts_bound,
            "superseded": self.superseded,
            "tf_edges": self.tf_edges,
            "wrapper_endpoints": self.wrapper_endpoints,
        }


def _records(values: Any) -> dict[str, tuple[dict[str, Any], ...]]:
    if not isinstance(values, dict):
        return {}
    out: dict[str, list[dict[str, Any]]] = {}

    def take(key: str, rows: Any) -> None:
        if not isinstance(rows, list):
            return
        kept = [row for row in rows if isinstance(row, dict)]
        if kept:
            out.setdefault(key, []).extend(kept)

    for key in RECORD_KEYS:
        take(key, values.get(key))
    return {key: tuple(rows) for key, rows in out.items()}


def _file_of(entity_id: str) -> str:
    kind = ids.kind_of(entity_id)
    if kind == "file":
        return entity_id
    if kind == "sym":
        parsed = ids.parse_sym(entity_id)
        return ids.build_file(parsed.package, parsed.relpath)
    return ""


def carriers(atlas: model.Atlas) -> list[Carrier]:
    """Every entity holding coded records, sorted by id."""
    found: list[Carrier] = []
    for entity_id in sorted(atlas.entities):
        entity = atlas.entities[entity_id]
        if entity.kind not in ("exe", "sym", "file"):
            continue
        merged: dict[str, list[dict[str, Any]]] = {key: [] for key in RECORD_KEYS}
        seen: set[str] = set()
        sources: list[Any] = [entity.attrs]
        sources.extend(fact.values for fact in entity.facts if fact.layer == CODED)
        for values in sources:
            for key, rows in _records(values).items():
                for row in rows:
                    handle = ids.digest(key, ids.digest_fields({k: str(v) for k, v in row.items()}))
                    if handle in seen:
                        continue
                    seen.add(handle)
                    merged[key].append(row)
        if not any(merged.values()):
            continue
        file_id = _file_of(entity_id)
        found.append(
            Carrier(
                owner=entity_id,
                file=file_id,
                symbol=entity_id if entity.kind == "sym" else "",
                nodes=tuple(merged["nodes"]),
                endpoints=tuple(merged["endpoints"]),
                parameters=tuple(merged["parameters"]),
                parameter_reads=tuple(merged["parameter_reads"]),
                plugin_parameters=tuple(merged["plugin_parameters"]),
                tf_broadcasts=tuple(merged["tf_broadcasts"]),
                tf_lookups=tuple(merged["tf_lookups"]),
                language=_language_of(atlas, file_id),
                constructions=tuple(merged["constructions"]),
                type_aliases=tuple(merged["type_aliases"]),
                classes=tuple(merged["classes"]),
                class_templates=tuple(merged["class_templates"]),
                plugin_loaders=tuple(merged["plugin_loaders"]),
            )
        )
    return found


def _language_of(atlas: model.Atlas, file_id: str) -> str:
    entity = atlas.entities.get(file_id)
    if entity is None:
        return ""
    stated = str(entity.attrs.get("language") or "")
    if stated:
        return stated
    for fact in entity.facts:
        if fact.layer == CODED and fact.values.get("language"):
            return str(fact.values["language"])
    return ""


def executables_of(
    atlas: model.Atlas, entity_id: str, builds: dict[str, dict[str, Any]] | None = None
) -> tuple[str, ...]:
    """The executables a coded carrier belongs to.

    Two paths, both of them the build's own statement. The entry point path is the `runs`
    edge onto a symbol the file defines. The source path is the `sources` list the build
    files declare for an executable, plus the executable that links the target those sources
    build, which is the only way a class compiled into a component library also reaches the
    plain executable that links it. When both paths name an executable, every one of them is
    kept: one source file really does back two executables on this stack, and picking one
    binds a node to code it never runs.
    """
    kind = ids.kind_of(entity_id)
    if kind == "exe":
        return (entity_id,)
    symbols = {entity_id} if kind == "sym" else set()
    owner = _file_of(entity_id)
    if owner:
        symbols.update(
            edge.dst
            for edge in atlas.out_edges(owner)
            if edge.kind == model.EdgeKind.DEFINES.value and ids.kind_of(edge.dst) == "sym"
        )
    found = set()
    for symbol in symbols:
        for edge in atlas.in_edges(symbol):
            if edge.kind == model.EdgeKind.RUNS.value and ids.kind_of(edge.src) == "exe":
                found.add(edge.src)
    found.update(_declaring_executables(atlas, owner, builds))
    return tuple(sorted(found))


def _build_rows(atlas: model.Atlas) -> dict[str, dict[str, Any]]:
    """What every executable states about its own build, merged over its declared facts."""
    out: dict[str, dict[str, Any]] = {}
    for entity_id in sorted(atlas.entities):
        entity = atlas.entities[entity_id]
        if entity.kind != "exe":
            continue
        sources: set[str] = set()
        targets: set[str] = set()
        linked: set[str] = set()
        for fact in entity.facts:
            if fact.layer != DECLARED:
                continue
            sources.update(str(item) for item in _string_list(fact.values.get("sources")))
            target = str(fact.values.get("target") or "")
            if target:
                targets.add(target)
            linked.update(_string_list(fact.values.get("linked_targets")))
        if sources or targets or linked:
            out[entity_id] = {"sources": sources, "targets": targets, "linked": linked}
    return out


def _string_list(raw: Any) -> list[str]:
    if not isinstance(raw, list):
        return []
    return [str(item) for item in raw if isinstance(item, str)]


def _class_type(spelling: str) -> str:
    """Template instances run the methods of their template class."""
    while "<" in spelling:
        reduced = re.sub(r"<[^<>]*>", "", spelling)
        if reduced == spelling:
            return ""
        spelling = reduced
    return spelling.strip().removeprefix("::")


def _resolve_class(
    spelling: str,
    namespace: str,
    classes: set[str],
    aliases: dict[str, set[tuple[str, str]]],
    seen: frozenset[str] = frozenset(),
    *,
    templates: dict[str, bool] | None = None,
) -> str:
    name = _class_type(spelling)
    if not name or name in seen or name in {"rclcpp::Node", "auto", "decltype"}:
        return ""
    choices = classes | aliases.keys()
    matches: set[str] = set()
    scope = namespace if not spelling.startswith("::") else ""
    while scope:
        scoped = f"{scope}::{name}"
        if scoped in choices:
            matches = {scoped}
            break
        scope = scope.rpartition("::")[0]
    if not matches and name in choices:
        matches = {name}
    elif not matches and "::" not in name and not spelling.startswith("::"):
        matches = {candidate for candidate in choices if candidate.rsplit("::", 1)[-1] == name}
    if len(matches) != 1:
        return ""
    selected = next(iter(matches))
    if selected in aliases:
        targets = aliases[selected]
        if len(targets) != 1 or selected in classes:
            return ""
        target, scope = next(iter(targets))
        return _resolve_class(target, scope, classes, aliases, seen | {name}, templates=templates)
    if (
        templates is not None
        and ("<" in spelling or selected in templates)
        and not templates.get(selected, False)
    ):
        return ""
    return selected


def _class_candidates(spelling: str, aliases: dict[str, set[tuple[str, str]]]) -> set[str]:
    pending = [spelling]
    found: set[str] = set()
    while pending:
        name = _class_type(pending.pop())
        if name in found:
            continue
        found.add(name)
        for alias, targets in aliases.items():
            if name == alias or ("::" not in name and alias.rsplit("::", 1)[-1] == name):
                for target, scope in targets:
                    pending.append(target)
                    if not target.startswith("::"):
                        while scope:
                            pending.append(f"{scope}::{target}")
                            scope = scope.rpartition("::")[0]
    return found


def _library_sources(atlas: model.Atlas) -> dict[tuple[str, str], set[str]]:
    out: dict[tuple[str, str], set[str]] = {}
    dependencies: dict[tuple[str, str], set[tuple[str, str]]] = {}
    for entity in atlas.entities.values():
        if entity.kind != "file":
            continue
        pkg = ids.parse_file(entity.id).package
        for fact in entity.facts:
            if fact.layer != DECLARED:
                continue
            for row in fact.values.get("library_targets", []):
                key = (pkg, row["target"])
                out.setdefault(key, set()).update(row["sources"])
                dependencies.setdefault(key, set()).update(
                    (pkg, target) for target in row.get("linked_targets", []) if target
                )
    changed = True
    while changed:
        changed = False
        for key in sorted(out):
            before = len(out[key])
            for target in dependencies.get(key, set()):
                out[key].update(out.get(target, set()))
            changed |= len(out[key]) != before
    return out


def _constructed_carriers(
    builder: model.AtlasBuilder,
    found: list[Carrier],
    by_exe: dict[str, list[Carrier]],
    bound: set[str],
) -> None:
    """A link makes code available, only a construction selects the class that runs."""
    aliases: dict[str, dict[str, set[tuple[str, str]]]] = {}
    atlas = builder.atlas
    templates: dict[str, dict[str, dict[tuple[str, str], dict[str, Any]]]] = {}
    classes: dict[str, set[str]] = {}
    bases: dict[str, dict[str, set[str]]] = {}
    methods: dict[str, dict[str, list[Carrier]]] = {}
    libraries = _library_sources(atlas)
    for carrier in found:
        if not carrier.file or carrier.language != "cpp":
            continue
        pkg = ids.parse_file(carrier.file).package
        classes.setdefault(pkg, set()).update(row["name"] for row in carrier.classes)
        for row in carrier.classes:
            bases.setdefault(pkg, {}).setdefault(row["name"], set()).update(
                [*row.get("bases", []), *row.get("node_helpers", [])]
            )
        for row in carrier.class_templates:
            templates.setdefault(pkg, {}).setdefault(_class_type(row["name"]), {})[
                (carrier.file, row["name"])
            ] = row
        for row in carrier.type_aliases:
            aliases.setdefault(pkg, {}).setdefault(row["name"], set()).add(
                (row["target"], row["namespace"])
            )
        if carrier.symbol:
            name = _class_type(ids.parse_sym(carrier.symbol).qualname).rpartition("::")[0]
            if name and name != "rclcpp":
                methods.setdefault(pkg, {}).setdefault(name, []).append(carrier)
    builds = _build_rows(atlas)
    unique_templates = {
        pkg: {
            name: sum(row["definitions"] for row in rows.values()) == 1
            and not any(row["specialized"] for row in rows.values())
            for name, rows in families.items()
        }
        for pkg, families in templates.items()
    }
    for main in found:
        if not main.file or not main.constructions:
            continue
        pkg = ids.parse_file(main.file).package
        table = methods.get(pkg, {})
        for exe in executables_of(atlas, main.owner, builds):
            sources: set[str] = set()
            for target in builds.get(exe, {}).get("linked", set()):
                sources.update(libraries.get((pkg, target), set()))
            for row in main.constructions:
                spelling = str(row.get("class") or "")
                name = _resolve_class(
                    spelling,
                    "",
                    classes.get(pkg, set()),
                    aliases.get(pkg, {}),
                    templates=unique_templates.get(pkg, {}),
                )
                if not name:
                    candidates = _class_candidates(spelling, aliases.get(pkg, {}))
                    if any(
                        candidate == owner
                        or ("::" not in candidate and owner.rsplit("::", 1)[-1] == candidate)
                        for owner in table
                        for candidate in candidates
                    ):
                        builder.unresolved(
                            model.Unresolved(
                                want=model.Want.OTHER.value,
                                owner=main.owner,
                                expr=spelling,
                                reason=(
                                    "the constructed class or alias has no unique "
                                    "package definition without specializations; a template "
                                    "requires exactly one class template definition"
                                ),
                                evidence=_source(main, row),
                            )
                        )
                selected: set[str] = set()
                pending = [name] if name else []
                while pending:
                    current = pending.pop()
                    if current in selected:
                        continue
                    selected.add(current)
                    for base in sorted(bases.get(pkg, {}).get(current, set())):
                        resolved = _resolve_class(
                            base,
                            current.rpartition("::")[0],
                            classes.get(pkg, set()),
                            aliases.get(pkg, {}),
                        )
                        if resolved:
                            pending.append(resolved)
                for selected_by, carrier in [
                    (selected_name, item)
                    for selected_name in sorted(selected)
                    for item in _methods_of(table, selected_name, classes.get(pkg, set()))
                ]:
                    if (
                        carrier.file not in sources
                        and carrier.file != row.get("definition_file")
                        # a class the constructed one inherits or hands itself to, defined
                        # in a header of its package, is compiled wherever it is included
                        and (selected_by == name or not _header(carrier.file))
                    ):
                        continue
                    rows = by_exe.setdefault(exe, [])
                    existing = next((item for item in rows if item.owner == carrier.owner), None)
                    if existing is not None and not existing.construction_sources:
                        continue
                    evidence = tuple(_source(main, row))
                    if existing is not None:
                        rows.remove(existing)
                        evidence = tuple(
                            sorted(
                                set(existing.construction_sources + evidence),
                                key=model.Source.sort_key,
                            )
                        )
                    rows.append(replace(carrier, construction_sources=evidence))
                    bound.add(carrier.owner)


def _header(file_id: str) -> bool:
    return file_id.endswith((".h", ".hpp", ".hh", ".hxx"))


def _methods_of(table: dict[str, list[Carrier]], name: str, classes: set[str]) -> list[Carrier]:
    """The methods of one class. A source that brings the class in with
    `using robotis::turtlebot3::TurtleBot3;` defines `TurtleBot3::TurtleBot3` outside its
    namespace, so the bare name stands for the class when the package has one of that name."""
    if name in table:
        return table[name]
    short = name.rpartition("::")[2]
    if short == name or sum(1 for item in classes if item.rpartition("::")[2] == short) != 1:
        return []
    return table.get(short, [])


def _declaring_executables(
    atlas: model.Atlas, file_id: str, builds: dict[str, dict[str, Any]] | None = None
) -> set[str]:
    if not file_id:
        return set()
    rows = _build_rows(atlas) if builds is None else builds
    direct = {exe for exe, row in rows.items() if file_id in row["sources"]}
    targets: set[str] = set()
    for exe in direct:
        targets.update(rows[exe]["targets"])
    linked = {exe for exe, row in rows.items() if row["linked"] & targets}
    return direct | linked


# ---------------------------------------------------------------------------
# what the declared layer launched


def _remap_rules(atlas: model.Atlas, node_id: str, profile: str) -> tuple[tuple[str, str], ...]:
    rules: list[tuple[str, str]] = []
    for edge in atlas.in_edges(node_id):
        if edge.kind != model.EdgeKind.LAUNCHES.value or ids.kind_of(edge.src) != "launchent":
            continue
        if edge.fact(DECLARED, profile) is None:
            continue
        owner = atlas.entities.get(edge.src)
        fact = owner.fact(DECLARED, profile) if owner else None
        if fact is None:
            continue
        rules.extend(_rule_rows(fact.values.get("remaps")))
    node = atlas.entities.get(node_id)
    fact = node.fact(DECLARED, profile) if node else None
    if fact is not None:
        rules.extend(_rule_rows(fact.values.get("remaps")))
    return tuple(dict.fromkeys(rules))


def _rule_rows(raw: Any) -> list[tuple[str, str]]:
    out: list[tuple[str, str]] = []
    if not isinstance(raw, list):
        return out
    for row in raw:
        if isinstance(row, (list, tuple)) and len(row) >= 2:
            out.append((str(row[0]), str(row[1])))
    return out


def instances(atlas: model.Atlas) -> list[Instance]:
    """Every node instance the declared layer placed, one row per profile.

    A node whose every launch site is off in a profile, a condition read false, is not an
    instance there: no code runs under it, so no record binds to it.
    """
    found: list[Instance] = []
    for entity_id in sorted(atlas.entities):
        entity = atlas.entities[entity_id]
        if entity.kind != "node":
            continue
        parsed = ids.parse_node(entity_id)
        for fact in sorted(entity.facts, key=model.LayerFact.sort_key):
            if fact.layer != DECLARED or fact.profile == model.ANY_PROFILE:
                continue
            values = fact.values
            pkg = str(values.get("pkg") or parsed.package or "")
            executable = str(values.get("executable") or parsed.executable or "")
            namespace = str(values.get("namespace") or "")
            name = str(values.get("name") or "")
            if parsed.fqn:
                head, base = ids.split_ros_name(parsed.fqn)
                namespace = namespace or head
                name = name or base
            namespace = namespace or parsed.namespace or ""
            found.append(
                Instance(
                    node_id=entity_id,
                    profile=fact.profile,
                    pkg=pkg,
                    executable=executable,
                    namespace=namespace,
                    name=name,
                    name_source=entity.name_source or model.NameSource.UNKNOWN.value,
                    known_name=str(values.get("known_name") or ""),
                    remaps=_remap_rules(atlas, entity_id, fact.profile),
                    plugin_class=str(values.get("plugin_class") or ""),
                    namespace_stated=bool(values.get("namespace")),
                    argv=tuple(_string_list(values.get("argv"))),
                )
            )
    return [item for item in found if _launched(atlas, item)]


# ---------------------------------------------------------------------------
# the remap engine


def _expand(name: str, namespace: str, node_name: str) -> str:
    try:
        return ids.normalise_ros_name(name, namespace or "/", node_name or None)
    except ContractError:
        return ""


def apply_remaps(
    raw: str,
    namespace: str = "/",
    node_name: str = "",
    rules: tuple[tuple[str, str], ...] = (),
) -> Bound | None:
    """Bind one raw code name to an absolute name, ROS 2 rules, in order.

    A rule matches when its left side is the raw name as written or expands to the same
    absolute name, and the first matching rule wins. A rule may carry a node prefix
    (`node:from:=to`), which applies only to that node. The argv rules `__node`, `__ns` and
    friends are not remaps of a name and are left to the declared layer. None means the name
    could not be resolved, which is an unresolved entry and never a guessed topic.
    """
    if not raw:
        return None
    expanded = _expand(raw, namespace, node_name)
    for frm, to in rules:
        target, matched = _matches(frm, raw, expanded, namespace, node_name)
        if not matched:
            continue
        resolved = _expand(to, namespace, node_name)
        if not resolved:
            return None
        return Bound(name=resolved, rule=(target, to))
    if not expanded:
        return None
    return Bound(name=expanded, rule=None)


def _matches(frm: str, raw: str, expanded: str, namespace: str, node_name: str) -> tuple[str, bool]:
    rule = frm.strip()
    if rule.startswith("__"):
        return rule, False
    if ":" in rule and "://" not in rule:
        prefix, _, tail = rule.partition(":")
        if prefix and not prefix.startswith("/") and tail:
            if prefix != node_name:
                return rule, False
            rule = tail
    if rule == raw:
        return rule, True
    return rule, bool(expanded) and _expand(rule, namespace, node_name) == expanded


# ---------------------------------------------------------------------------
# the join


@dataclass(frozen=True)
class Index:
    """Every way a launched instance can reach the code behind it.

    A node names an executable. A composable node names a class, and AGENTS section 3.4 fixes
    the order it is followed in: the pluginlib lookup name, then the registered component
    class, then the symbol that spells the class. A process started by path names neither, so
    the script its argv holds is asked last, keyed by package and package relative path.
    """

    by_exe: dict[str, tuple[Carrier, ...]]
    by_plugin: dict[str, tuple[Carrier, ...]]
    by_component: dict[str, tuple[Carrier, ...]]
    by_symbol: dict[str, tuple[Carrier, ...]]
    by_script: dict[str, tuple[Carrier, ...]] = field(default_factory=dict)

    def of(self, item: Instance) -> tuple[tuple[Carrier, ...], str]:
        if item.executable:
            found = self.by_exe.get(_exe_id(item), ())
            if found:
                return found, "executable"
        found = self.script_of(item)
        if found:
            return found, "script_path"
        for table, via in (
            (self.by_plugin, "plugin_lookup"),
            (self.by_component, "registered_class"),
            (self.by_symbol, "symbol"),
        ):
            found = table.get(item.plugin_class)
            if found:
                return found, via
        return (), ""

    def script_of(self, item: Instance) -> tuple[Carrier, ...]:
        """The carriers of the script this instance's argv names.

        The argv word has to end at the package relative path of a file the coded layer read,
        under its own package directory, so a bare basename can never pick a file and the
        install copy of a path resolves to the same entry as the source copy. Nothing about
        the absolute path is kept: it decides a table lookup and never reaches a value.
        """
        for word in item.argv:
            for key in self.by_script:
                if word == key or word.endswith(f"/{key}"):
                    return self.by_script[key]
        return ()


def join(builder: model.AtlasBuilder) -> JoinReport:
    """Run the join once. Running it again changes nothing."""
    atlas = builder.atlas
    first = MARKER not in atlas.x
    _resolve_interface_constants(builder)
    report = JoinReport()
    report.coded_blind = {
        (entry.want, entry.expr, source.file, source.line_hint)
        for entry in atlas.unresolved
        if ids.kind_of(entry.owner) in ("sym", "file")
        for source in entry.evidence
    }
    index = index_of(builder, report)
    _supersede(builder, index, report)
    rows: list[tuple[Instance, tuple[Carrier, ...], str]] = []
    for item in instances(atlas):
        found, via = index.of(item)
        if found:
            rows.append((_own_arguments(atlas, item, found), found, via))
    report.instances = len(rows)
    report.executables_bound = len({_exe_id(item) for item, _, via in rows if via == "executable"})
    report.components_joined = len(
        {item.node_id for item, _, via in rows if via not in ("executable", "script_path")}
    )
    report.scripts_bound = len({item.node_id for item, _, via in rows if via == "script_path"})
    for item, found, via in rows:
        _bind_instance(builder, item, found, via, report, count=first)
    _retire_bound_remaps(builder, report)
    _retire_resolved_sites(builder, report)
    _ignore_undeclared(builder)
    if first:
        atlas.x[MARKER] = report.summary()
    builder.recompute()
    return report


AUTO_DECLARED = frozenset({"use_sim_time"})
AUTO_DECLARED_PREFIXES = ("qos_overrides.",)
"""Names the client library declares for every node, so no source states them."""


def auto_declared(name: str) -> bool:
    return name in AUTO_DECLARED or name.startswith(AUTO_DECLARED_PREFIXES)


def blind_names(atlas: model.Atlas, want: str) -> dict[str, list[str]]:
    """Files holding a site whose `want` could not be read, with the expression of each site.

    A claim that something is absent from a node's code cannot be made when that code holds
    such a site and its expression could build the missing name at run time.
    """
    out: dict[str, list[str]] = {}
    for entry in atlas.unresolved:
        if entry.want != want:
            continue
        kind = ids.kind_of(entry.owner)
        if kind == "file":
            out.setdefault(entry.owner, []).append(entry.expr)
        elif kind == "sym":
            parsed = ids.parse_sym(entry.owner)
            out.setdefault(ids.build_file(parsed.package, parsed.relpath), []).append(entry.expr)
    return out


def blind_files(atlas: model.Atlas, want: str) -> set[str]:
    return set(blind_names(atlas, want))


_STRING = re.compile(r"\"((?:[^\"\\]|\\.)*)\"|'((?:[^'\\]|\\.)*)'")
_PLACEHOLDER = re.compile(r"\{[^{}]*\}|%[-+ #0-9.]*[a-z]")


_CHOICE = re.compile(r"[?\[]|\|\||&&|\b(?:if|else|or|and)\b")
_FORMATTERS = frozenset({"", "std::string", "string", "std::format", "fmt::format", "str"})
_CALLEE = re.compile(r"([\w:]*)\s*$")


def could_name(expr: str, name: str) -> bool:
    """Whether a name built at run time from `expr` can be `name`.

    The text the expression spells as string literals has to appear in the name in that
    order: `"sensor." + id` builds `sensor.front` and never `min_range`. An expression with
    no literal, a constant or a variable, could build any name. Literals are pieces of the
    name only in a plain concatenation or a format string: a condition choosing between two
    names, a subscript, or a call taking a literal (`names_.at("gain_key")`) could build any.
    """
    if not _spells_pieces(expr):
        return True
    pieces = [
        piece
        for found in _STRING.finditer(expr)
        for piece in _PLACEHOLDER.split(found.group(1) or found.group(2) or "")
        if piece
    ]
    position = 0
    for piece in pieces:
        found = name.find(piece, position)
        if found < 0:
            return False
        position = found + len(piece)
    return True


def _spells_pieces(expr: str) -> bool:
    blank = _STRING.sub(lambda found: '"' + " " * (len(found.group(0)) - 2) + '"', expr)
    if _CHOICE.search(blank):
        return False
    for found in _STRING.finditer(blank):
        before = re.sub(r"[fFrRbBuU]{1,2}$", "", blank[: found.start()]).rstrip()
        if before.endswith(","):
            depth = 0
            for at in range(len(before) - 1, -1, -1):
                depth += {")": 1, "(": -1}.get(before[at], 0)
                if depth < 0:
                    before = before[: at + 1]
                    break
            else:
                return False
        if before.endswith("(") and _CALLEE.search(before[:-1]).group(1) not in _FORMATTERS:
            return False
    return True


def _row_class(row: dict[str, Any], carrier: Carrier, classes: set[str]) -> str:
    """The short name of the class a record row belongs to, or `name()` for a free function,
    `classes` being the short class names its package defines."""
    owner = str(row.get("owner_symbol") or row.get("symbol") or "")
    if owner.startswith("sym:"):
        owner = ids.parse_sym(owner).qualname
    if not owner and carrier.symbol:
        owner = ids.parse_sym(carrier.symbol).qualname
    scope, _, function = _class_type(owner).rpartition("::")
    return _short(scope) if _short(scope) in classes else function + "()"


class _Reach:
    """The classes a node's code runs, by package and short class name.

    The node's own classes reach what they inherit and what they hand themselves to, in
    their package (`bases`, `node_helpers`) and in another one when the helper is spelled
    with a namespace that package defines (`make_shared<mpc::Controller>(*this)`). A package
    that holds a `pluginlib::ClassLoader<Base>` reaches every class a plugin manifest
    registers for that base, because the class it loads is a name only the run knows.
    """

    def __init__(self, atlas: model.Atlas, found: Sequence[Carrier]) -> None:
        self.graph: dict[tuple[str, str], set[tuple[str, str]]] = {}
        self.loads: dict[str, set[tuple[str, str]]] = {}
        defined: dict[str, set[tuple[str, str]]] = {}
        for carrier in found:
            if carrier.file:
                pkg = ids.parse_file(carrier.file).package
                for row in carrier.classes:
                    name = _class_type(str(row.get("name") or ""))
                    defined.setdefault(_short(name), set()).add((name, pkg))
        by_base: dict[str, set[tuple[str, str]]] = {}
        for entity in atlas.by_kind("plugin"):
            base = ids.parse_plugin(entity.id).base_class
            for fact in entity.facts:
                implementation = fact.values.get("type")
                for source in fact.evidence:
                    if isinstance(implementation, str) and ids.kind_of(source.file) == "file":
                        by_base.setdefault(base, set()).add(
                            (ids.parse_file(source.file).package, _short(implementation))
                        )
        for carrier in found:
            if not carrier.file:
                continue
            pkg = ids.parse_file(carrier.file).package
            for row in carrier.plugin_loaders:
                spelled = str(row.get("base_class") or "")
                for base, classes in by_base.items():
                    if spelled and (base == spelled or base.endswith("::" + spelled)):
                        self.loads.setdefault(pkg, set()).update(classes)
            for row in carrier.classes:
                key = (pkg, _short(_class_type(str(row.get("name") or ""))))
                for other in row.get("node_handoffs", []):
                    if str(other).endswith("()"):
                        self.graph.setdefault(key, set()).add((pkg, _short(str(other))))
                for other in [
                    *row.get("bases", []),
                    *row.get("node_helpers", []),
                    *(item for item in row.get("node_handoffs", []) if not item.endswith("()")),
                ]:
                    spelled = _class_type(str(other))
                    matches = {
                        (owner, _short(name))
                        for name, owner in defined.get(_short(spelled), set())
                        if spelled and (name == spelled or name.endswith("::" + spelled))
                    }
                    local = {item for item in matches if item[0] == pkg}
                    self.graph.setdefault(key, set()).update(
                        local or matches or {(pkg, _short(spelled))}
                    )
        self.cache: dict[frozenset[tuple[str, str]], frozenset[tuple[str, str]]] = {}

    def of(self, start: set[tuple[str, str]]) -> frozenset[tuple[str, str]]:
        key = frozenset(start)
        if key not in self.cache:
            reached: set[tuple[str, str]] = set()
            pending = sorted(start)
            while pending:
                current = pending.pop()
                if current in reached:
                    continue
                reached.add(current)
                pending.extend(self.graph.get(current, ()))
                pending.extend(self.loads.get(current[0], ()))
            self.cache[key] = frozenset(reached)
        return self.cache[key]


def parameter_libraries(atlas: model.Atlas) -> dict[str, tuple[set[str], list[str]]]:
    """Source file id to what the `generate_parameter_library` targets its build links
    declare: the parameter names, and the expressions of the names a mapped group builds at
    run time. A definition the reader could not load is one expression that could build
    any name, so the node it serves is never called deaf to a key."""
    libraries: dict[tuple[str, str], dict[str, Any]] = {}
    for entity in atlas.by_kind("file"):
        pkg = ids.parse_file(entity.id).package
        for fact in entity.facts:
            if fact.layer != DECLARED:
                continue
            for row in fact.values.get("library_targets", []):
                libraries.setdefault((pkg, str(row.get("target") or "")), row)
    if not any("definition" in row for row in libraries.values()):
        return {}
    targets = [
        (ids.parse_exe(exe_id).package, row["sources"], row["linked"])
        for exe_id, row in _build_rows(atlas).items()
    ]
    targets.extend(
        (pkg, row.get("sources", []), row.get("linked_targets", []))
        for (pkg, _), row in sorted(libraries.items())
    )
    out: dict[str, tuple[set[str], list[str]]] = {}
    for pkg, sources, linked in targets:
        names: set[str] = set()
        patterns: set[str] = set()
        seen: set[tuple[str, str]] = set()
        pending = [(pkg, str(target)) for target in linked]
        while pending:
            key = pending.pop()
            row = libraries.get(key)
            if key in seen or row is None:
                continue
            seen.add(key)
            names.update(ids.normalise_param_name(str(name)) for name in row.get("parameters", []))
            patterns.update(str(item) for item in row.get("parameter_patterns", []))
            if row.get("parameters_unread"):
                patterns.add(str(row.get("definition") or ""))
            pending.extend((pkg, str(target)) for target in row.get("linked_targets", []))
        if not names and not patterns:
            continue
        for source in sources:
            held = out.setdefault(str(source), (set(), []))
            held[0].update(names)
            held[1].extend(sorted(patterns - set(held[1])))
    return out


def parameter_declarations(
    atlas: model.Atlas,
) -> tuple[dict[str, list[str]], dict[tuple[str, str], set[str]]]:
    """The `param_name` blind sites and the declared names `reading` takes.

    A node declares what its bound code declares, what the generated parameter libraries its
    build links declare, and what the code it reaches declares without the join binding it:
    a class it inherits or hands itself to, a plugin class its package may load, and a helper
    whose handle the reader could not trace (`node_handle_untraced`) in its own packages.
    In the node's own packages only the classes it reaches count; another package it reaches
    counts whole, less the code that states a node of its own, because a second executable
    declaring a name says nothing about this node. A name that code builds at run time is a
    blind spot of the node, keyed by the node id.
    """
    found = carriers(atlas)
    blind = blind_names(atlas, model.Want.PARAM_NAME.value)
    declared = declared_names(atlas)
    generated = parameter_libraries(atlas)
    reach = _Reach(atlas, found)
    classes: dict[str, set[str]] = {}
    for carrier in found:
        if carrier.file:
            classes.setdefault(ids.parse_file(carrier.file).package, set()).update(
                _short(_class_type(str(row.get("name") or ""))) for row in carrier.classes
            )
    names: dict[tuple[str, str], set[str]] = {}
    standalone: set[tuple[str, str]] = set()
    for carrier in found:
        if not carrier.file:
            continue
        pkg = ids.parse_file(carrier.file).package
        for row in (*carrier.parameters, *carrier.parameter_reads):
            try:
                name = ids.normalise_param_name(str(row.get("name") or ""))
            except ContractError:
                continue
            key = (pkg, _row_class(row, carrier, classes.get(pkg, set())))
            names.setdefault(key, set()).add(name)
            if carrier.nodes:
                standalone.add(key)
    exprs: dict[tuple[str, str], list[str]] = {}
    untraced: dict[str, set[str]] = {}
    for entry in atlas.unresolved:
        if ids.kind_of(entry.owner) != "sym":
            continue
        parsed = ids.parse_sym(entry.owner)
        klass = _row_class(
            {"owner_symbol": parsed.qualname},
            Carrier(owner="", file="", symbol=""),
            classes.get(parsed.package, set()),
        )
        if entry.want == model.Want.PARAM_NAME.value:
            exprs.setdefault((parsed.package, klass), []).append(entry.expr)
        elif entry.reason == "node_handle_untraced":
            untraced.setdefault(parsed.package, set()).add(klass)
    for node in atlas.by_kind(model.NodeKind.NODE.value):
        for fact in node.facts:
            if fact.layer != CODED:
                continue
            sources = sorted(fact.values.get("coded_sources") or [])
            start: set[tuple[str, str]] = set()
            for carrier in found:
                if carrier.file in sources:
                    pkg = ids.parse_file(carrier.file).package
                    # the node's classes, and every class whose code the bound sources hold
                    start.update(
                        (pkg, _short(_class_type(str(row.get("class") or ""))))
                        for row in carrier.nodes
                        if row.get("class")
                    )
                    start.update(
                        (pkg, _short(_class_type(str(row.get("name") or ""))))
                        for row in carrier.classes
                    )
                    if carrier.symbol:
                        start.add(
                            (
                                pkg,
                                _row_class(
                                    {"owner_symbol": carrier.symbol},
                                    carrier,
                                    classes.get(pkg, set()),
                                ),
                            )
                        )
            packages = {ids.parse_file(source).package for source in sources}
            reached = set(reach.of(start)) | {
                (pkg, klass) for pkg in packages for klass in untraced.get(pkg, ())
            }
            held = declared.setdefault((node.id, REACHED), set())
            pending: list[str] = []
            # Another package is reached whole: a plugin builds its own parameter structs
            # from the node it is handed, which no class edge of this reader states.
            others = {pkg for pkg, _ in reached} - packages
            for key in sorted(set(names) | set(exprs)):
                if key in reached or (key[0] in others and key not in standalone):
                    held.update(names.get(key, ()))
                    pending.extend(exprs.get(key, ()))
            for source in sources:
                generated_names, patterns = generated.get(source, (set(), []))
                declared.setdefault((node.id, fact.profile), set()).update(generated_names)
                pending.extend(patterns)
            if pending:
                blind[node.id] = sorted(set(blind.get(node.id, [])) | set(pending))
    return blind, declared


def declared_names(atlas: model.Atlas) -> dict[tuple[str, str], set[str]]:
    """Owner node id and profile to the parameter names its source declares or reads back.

    A read of a name no declaration carries is the code using the value all the same, through
    a declaration this reader could not follow or a node that takes undeclared parameters.
    The profile is the one the code ran in, because a sim and a real profile can start one
    node name from two executables; `declared_in` adds what holds in every profile.
    """
    out: dict[tuple[str, str], set[str]] = {}
    for node in atlas.by_kind(model.NodeKind.PARAM.value):
        parsed = ids.parse_param(node.id)
        for fact in node.facts:
            if fact.layer == CODED and fact.values.get("declared_in_code"):
                out.setdefault((parsed.owner, fact.profile), set()).add(parsed.name)
    for edge in atlas.edges.values():
        if edge.kind == model.EdgeKind.READS_PARAM.value:
            parsed = ids.parse_param(edge.dst)
            for fact in edge.facts:
                out.setdefault((parsed.owner, fact.profile), set()).add(parsed.name)
    return out


REACHED = "~reached"
"""The profile slot of `parameter_declarations` holding what a node's code declares through
code the join did not bind to it, in any profile it runs in. It never answers whether the
code was read at all."""


def declared_in(declared: Mapping[tuple[str, str], set[str]], owner: str, profile: str) -> set[str]:
    return declared.get((owner, profile), set()) | declared.get((owner, model.ANY_PROFILE), set())


DECLARED_STATE = "declared"
UNDECLARED_STATE = "undeclared"
UNKNOWN_STATE = "unknown"


def reading(
    atlas: model.Atlas,
    node: model.Node,
    profile: str,
    blind: Mapping[str, Sequence[str]],
    declared: Mapping[tuple[str, str], set[str]],
) -> str:
    """Whether the node a parameter belongs to declares it in this profile, decided per key.

    `undeclared` when the node runs code, in this profile, that never declares it, so the
    value a file sets is not live. `unknown` when the only declarations that could carry the
    name build it at run time. `declared` when the code names it, and an empty string when
    the code cannot answer at all.

    Only a node whose code the join bound and read can say so: a `main` that only constructs
    a class defined in another file, with no declaration bound, read none of them. A node
    built to take undeclared parameters, or one whose value it cannot tell, uses them. A
    declaration whose name is built at run time hides only the keys its literal text allows,
    so one such line no longer exempts every key of the file. What the node reaches without
    the join binding it is in `declared` and `blind` under the node id
    (`parameter_declarations`).
    """
    parsed = ids.parse_param(node.id)
    owner = atlas.entities.get(parsed.owner)
    if owner is None or auto_declared(parsed.name):
        return ""
    coded = owner.fact(CODED, profile)
    if (
        coded is None
        or coded.confidence == model.Confidence.UNKNOWN.value
        or coded.values.get("via_known_wrapper")
        or coded.values.get("auto_declare")
    ):
        return ""
    sources = set(coded.values.get("coded_sources") or [])
    names = declared_in(declared, parsed.owner, profile)
    if not names and not any(_states_node(atlas, s) for s in sources):
        return ""
    own = node.fact(CODED, profile)
    if parsed.name in names or (own is not None and own.values.get("declared_in_code")):
        return DECLARED_STATE
    if parsed.name in declared.get((parsed.owner, REACHED), set()):
        return ""
    if any(
        could_name(expr, parsed.name)
        for item in sorted(sources | {parsed.owner})
        for expr in blind.get(item, ())
    ):
        return UNKNOWN_STATE
    return UNDECLARED_STATE


def ignored(
    atlas: model.Atlas,
    node: model.Node,
    profile: str,
    blind: Mapping[str, Sequence[str]],
    declared: Mapping[tuple[str, str], set[str]],
) -> bool:
    """Whether the node never declares the parameter in this profile, so it ignores it."""
    return reading(atlas, node, profile, blind, declared) == UNDECLARED_STATE


def _states_node(atlas: model.Atlas, source: str) -> bool:
    """Whether a bound source names the node it runs."""
    entity = atlas.entities.get(source)
    if entity is None:
        return False
    return bool(entity.attrs.get("nodes")) or any(
        fact.values.get("nodes") for fact in entity.facts if fact.layer == CODED
    )


IGNORED_NOTE = "the node never declares this parameter, so it ignores the value"
BLIND_NOTE = (
    "the node declares a parameter whose name it builds at run time, so whether it reads "
    "this one is unknown"
)


def _ignore_undeclared(builder: model.AtlasBuilder) -> None:
    """A parameter a file sets and the node's code never declares wins nothing.

    The chain still states which line would win, and nothing does: the profile's candidates
    lose their win, and a parameter that wins in no profile any more is `undeclared` with no
    effective value (AGENTS section 7), which is what the drift rule says of it. A key only a
    declaration built at run time could carry wins nothing either, because calling it
    effective claims the node reads it, and the parameter is `other` with the note saying why.
    """
    atlas = builder.atlas
    blind, declared = parameter_declarations(atlas)
    for node in atlas.by_kind(model.NodeKind.PARAM.value):
        if node.state != model.ParamState.EFFECTIVE.value:
            continue
        dropped = {
            row.profile: reading(atlas, node, row.profile, blind, declared)
            for row in node.candidates
            if row.wins
        }
        notes = {UNDECLARED_STATE: IGNORED_NOTE, UNKNOWN_STATE: BLIND_NOTE}
        dropped = {profile: state for profile, state in dropped.items() if state in notes}
        if not dropped:
            continue
        node.candidates = [
            replace(row, wins=False, note=notes[dropped[row.profile]])
            if row.profile in dropped and row.wins
            else row
            for row in node.candidates
        ]
        if not any(row.wins for row in node.candidates):
            node.state = (
                model.ParamState.UNDECLARED.value
                if set(dropped.values()) == {UNDECLARED_STATE}
                else model.ParamState.OTHER.value
            )


def _resolve_interface_constants(builder: model.AtlasBuilder) -> None:
    atlas = builder.atlas
    constants: dict[str, set[str]] = {}
    for entity in atlas.entities.values():
        if entity.kind != "type":
            continue
        parsed = ids.parse_type(entity.id)
        for fact in entity.facts:
            if fact.confidence == model.Confidence.UNKNOWN.value:
                continue
            for row in fact.values.get("constants", []):
                if isinstance(row.get("value"), str):
                    expr = f"{parsed.package}::{parsed.interface}::{parsed.name}::{row['name']}"
                    constants.setdefault(expr, set()).add(row["value"])
    resolved: set[tuple[str, str]] = set()
    for entity in atlas.entities.values():
        for fact in entity.facts:
            if fact.layer != CODED:
                continue
            for row in fact.values.get("parameters", []):
                expr = str(row.get("name_expr") or "")
                known = known_names.INTERFACE_CONSTANTS.get(expr.removeprefix("::"))
                values = constants.get(expr.removeprefix("::"), {known} if known else set())
                if row.get("name") is not None or len(values) != 1:
                    continue
                row.update(name=next(iter(values)), name_state="interface_constant")
                owner = entity.id
                if entity.kind == "file" and row.get("owner_symbol"):
                    parsed = ids.parse_file(entity.id)
                    owner = ids.build_sym(parsed.package, parsed.relpath, row["owner_symbol"])
                resolved.add((owner, expr))
    remaining = []
    for entry in atlas.unresolved:
        if entry.want == model.Want.PARAM_NAME.value and (entry.owner, entry.expr) in resolved:
            builder.count_resolved(entry.want)
        else:
            remaining.append(entry)
    atlas.unresolved = remaining


def _retire_bound_remaps(builder: model.AtlasBuilder, report: JoinReport) -> None:
    atlas = builder.atlas
    launched: dict[str, list[model.Edge]] = {}
    for edge in atlas.edges.values():
        if edge.kind == model.EdgeKind.LAUNCHES.value:
            launched.setdefault(edge.src, []).append(edge)
    scopes = {(item.profile, item.node_id): item for item in instances(atlas)}
    covered: dict[tuple[str, str, str], set[str]] = {}
    for profile, node_id, raw, target in report.remap_cases:
        item = scopes.get((profile, node_id))
        if item is not None:
            covered.setdefault((node_id, profile, target), set()).add(
                _expand(raw, item.namespace, item.name)
            )
    remaining = []
    for entry in atlas.unresolved:
        source, separator, target = entry.expr.partition(":=")
        owner = atlas.entities.get(entry.owner)
        profiles = (
            {fact.profile for fact in owner.facts if fact.layer == DECLARED}
            if owner is not None
            else set()
        )
        matched = set()
        if entry.role == "remap" and separator:
            for edge in launched.get(entry.owner, []):
                for profile in profiles:
                    item = scopes.get((profile, edge.dst))
                    if item is None:
                        continue
                    destination = _expand(target, item.namespace, item.name)
                    origin = _expand(source, item.namespace, item.name)
                    if origin and origin in covered.get((edge.dst, profile, destination), set()):
                        matched.add(profile)
        if profiles and profiles <= matched:
            builder.count_resolved(entry.want)
        else:
            remaining.append(entry)
    atlas.unresolved = remaining


def _exe_id(item: Instance) -> str:
    if not item.pkg or not item.executable:
        return ""
    try:
        return ids.build_exe(item.pkg, item.executable)
    except ContractError:
        return ""


def index_of(builder: model.AtlasBuilder, report: JoinReport) -> Index:
    atlas = builder.atlas
    found = carriers(atlas)
    by_file: dict[str, list[Carrier]] = {}
    for carrier in found:
        if carrier.file:
            by_file.setdefault(carrier.file, []).append(carrier)
    by_exe: dict[str, list[Carrier]] = {}
    bound: set[str] = set()
    builds = _build_rows(atlas)
    for carrier in found:
        exes = executables_of(atlas, carrier.owner, builds)
        if len(exes) > 1 and carrier.file:
            # Two executables really are built from this source, so both are recorded and
            # the instance decides which one it launched.
            builder.node(carrier.file, attrs={"executables": list(exes)})
            row = (carrier.file, exes)
            if row not in report.shared_sources:
                report.shared_sources.append(row)
        for exe in exes:
            by_exe.setdefault(exe, []).append(carrier)
            bound.add(carrier.owner)
    by_plugin = _class_table(by_file, _plugin_files(atlas))
    by_component = _class_table(by_file, _component_files(atlas))
    by_symbol: dict[str, list[Carrier]] = {}
    for carrier in found:
        if not carrier.symbol:
            continue
        qualname = ids.parse_sym(carrier.symbol).qualname
        for name in _class_names(qualname):
            by_symbol.setdefault(name, []).append(carrier)
    for entity in atlas.entities.values():
        if entity.kind != "exe":
            continue
        classes = {
            str(fact.values["component_class"])
            for fact in entity.facts
            if fact.layer == DECLARED and fact.values.get("component_class")
        }
        if len(classes) == 1:
            selected = by_symbol.get(next(iter(classes)), [])
            if selected:
                by_exe[entity.id] = selected
    _constructed_carriers(builder, found, by_exe, bound)
    _untraced_helpers(builder, found, bound, (by_plugin, by_component))
    report.unbound_carriers = len(
        [
            carrier
            for carrier in found
            if carrier.owner not in bound
            and not _in_table(carrier, by_plugin)
            and not _in_table(carrier, by_component)
        ]
    )
    return Index(
        by_exe=_frozen(by_exe),
        by_plugin=_frozen(by_plugin),
        by_component=_frozen(by_component),
        by_symbol=_frozen(by_symbol),
        by_script=_frozen(_script_table(by_file)),
    )


def _untraced_helpers(
    builder: model.AtlasBuilder,
    found: list[Carrier],
    bound: set[str],
    tables: tuple[dict[str, list[Carrier]], ...],
) -> None:
    """Records of a C++ class that no node class inherits or hands itself to, in a source a
    target of the package builds, once the join followed the package's executable code. The
    class reaches a node through a handle this reader could not trace to one, so each such
    method is a blind spot rather than nothing."""
    followed = {ids.parse_file(item.file).package for item in found if item.owner in bound}
    built = {path for paths in _library_sources(builder.atlas).values() for path in paths}
    built.update(path for row in _build_rows(builder.atlas).values() for path in row["sources"])
    reached: dict[str, set[str]] = {}
    for carrier in found:
        if (
            carrier.owner in bound
            or not carrier.symbol
            or carrier.language != "cpp"
            or not (carrier.endpoints or carrier.parameters or carrier.tf_broadcasts)
            or any(_in_table(carrier, table) for table in tables)
        ):
            continue
        pkg = ids.parse_file(carrier.file).package
        klass = _class_type(ids.parse_sym(carrier.symbol).qualname).rpartition("::")[0]
        if pkg not in followed or not klass or carrier.file not in built:
            continue
        if pkg not in reached:
            graph = _class_graph(builder.atlas, pkg)
            nodes = {
                _short(str(row.get("class") or ""))
                for item in found
                if item.file and ids.parse_file(item.file).package == pkg
                for row in item.nodes
            } | {_short(_class_type(name)) for table in tables for name in table}
            reached[pkg] = {name for node in nodes for name in _reach(graph, node)}
        if _short(klass) in reached[pkg]:
            continue
        row = next(iter(carrier.endpoints or carrier.parameters or carrier.tf_broadcasts))
        builder.unresolved(
            model.Unresolved(
                want=model.Want.OTHER.value,
                owner=carrier.symbol,
                expr=klass,
                reason="node_handle_untraced",
                role="helper",
                resolvable_by=[
                    "construct the class from the node class, handing it this or shared_from_this()"
                ],
                evidence=_source(carrier, row),
            )
        )


def _script_table(by_file: dict[str, list[Carrier]]) -> dict[str, list[Carrier]]:
    out: dict[str, list[Carrier]] = {}
    for file_id, rows in by_file.items():
        key = _script_key(file_id)
        if key:
            out.setdefault(key, []).extend(rows)
    return out


def _script_key(file_id: str) -> str:
    """The path an argv word has to end at for this file, empty when the file is not one."""
    try:
        parsed = ids.parse_file(file_id)
    except ContractError:
        return ""
    if not parsed.relpath.endswith(SCRIPT_SUFFIX):
        return ""
    if parsed.package.startswith("@"):
        return parsed.relpath
    return f"{parsed.package}/{parsed.relpath}"


def _frozen(table: dict[str, list[Carrier]]) -> dict[str, tuple[Carrier, ...]]:
    return {key: tuple(value) for key, value in sorted(table.items())}


def _in_table(carrier: Carrier, table: dict[str, list[Carrier]]) -> bool:
    return any(carrier in rows for rows in table.values())


def _class_names(qualname: str) -> list[str]:
    """The class a symbol belongs to, and the symbol itself when it is the class."""
    out = [qualname]
    for separator in ("::", "."):
        head, found, _ = qualname.rpartition(separator)
        if found and head:
            out.append(head)
    return sorted(set(out))


def _class_table(
    by_file: dict[str, list[Carrier]],
    files: dict[str, set[str]],
) -> dict[str, list[Carrier]]:
    out: dict[str, list[Carrier]] = {}
    for name in sorted(files):
        for file_id in sorted(files[name]):
            for carrier in by_file.get(file_id, ()):
                out.setdefault(name, []).append(carrier)
    return out


def _plugin_files(atlas: model.Atlas) -> dict[str, set[str]]:
    """Pluginlib lookup name to the files that define the plugin."""
    out: dict[str, set[str]] = {}
    for entity_id in sorted(atlas.entities):
        if ids.kind_of(entity_id) != "plugin":
            continue
        lookup = ids.parse_plugin(entity_id).lookup
        for edge in atlas.in_edges(entity_id):
            if edge.kind == model.EdgeKind.DEFINES.value and ids.kind_of(edge.src) == "file":
                out.setdefault(lookup, set()).add(edge.src)
    return out


def _component_files(atlas: model.Atlas) -> dict[str, set[str]]:
    """Registered component class to the files that hold the registration macro."""
    out: dict[str, set[str]] = {}
    for entity_id in sorted(atlas.entities):
        entity = atlas.entities[entity_id]
        if entity.kind != "file":
            continue
        sources: list[Any] = [entity.attrs]
        sources.extend(fact.values for fact in entity.facts if fact.layer == CODED)
        for values in sources:
            if not isinstance(values, dict):
                continue
            rows = values.get("registrations")
            rows = rows if isinstance(rows, list) else []
            extra = values.get("components")
            rows = rows + (extra if isinstance(extra, list) else [])
            for row in rows:
                if not isinstance(row, dict):
                    continue
                if COMPONENT_MACRO not in str(row.get("macro") or ""):
                    continue
                name = str(row.get("type") or row.get("class") or "")
                if name:
                    out.setdefault(name, set()).add(entity_id)
    return out


# ---------------------------------------------------------------------------
# identity


def auto_declare_of(name: str, found: tuple[Carrier, ...]) -> bool | str | None:
    """Whether the node an instance runs is built to take parameters it never declares.

    The collector marks the construction or the class whose options say so, so a helper node
    `main` builds with options of its own speaks only for its class. The node rows named like
    the instance pick its classes when the code names several, and every row counts when it
    names none of them.
    """
    auto = {row.get("auto_declare") for row in _option_rows(name, found) if row.get("auto_declare")}
    if not auto:
        return None
    return True if True in auto else "unknown"


def ignores_global_arguments(name: str, found: tuple[Carrier, ...]) -> bool:
    """Whether the node is built with `use_global_arguments(false)`, a literal, so it takes no
    launch name, namespace, remap or parameter file."""
    return any(row.get("use_global_arguments") is False for row in _option_rows(name, found))


def _option_rows(name: str, found: tuple[Carrier, ...]) -> list[dict[str, Any]]:
    nodes = [row for carrier in found for row in carrier.nodes]
    mine = [row for row in nodes if name and row.get("name") == name] or nodes
    classes = {str(row.get("class") or "").rpartition("::")[2] for row in mine}
    return [
        *mine,
        *(
            row
            for carrier in found
            for row in carrier.constructions
            if not mine or str(row.get("class") or "").rpartition("::")[2] in classes
        ),
    ]


def _code_names(found: tuple[Carrier, ...]) -> list[str]:
    names = set()
    for carrier in found:
        for row in carrier.nodes:
            if str(row.get("name_source") or "literal") != "literal":
                continue
            name = str(row.get("name") or "").strip()
            if name:
                names.add(name)
    return sorted(names)


def _code_namespace(found: tuple[Carrier, ...]) -> str:
    """The namespace the sources state, absolute, empty when they state none or disagree."""
    stated = {_row_namespace(row) for carrier in found for row in carrier.nodes} - {""}
    return stated.pop() if len(stated) == 1 else ""


def _row_namespace(row: dict[str, Any]) -> str:
    if str(row.get("namespace_source") or "") != "literal":
        return ""
    name = str(row.get("namespace") or "").strip()
    return "" if not name else name if name.startswith("/") else f"/{name}"


def _placed(item: Instance, stated: str) -> Instance:
    """The instance, under the namespace its own source states.

    A launch `namespace=` is an argv override and wins whenever it exists, so the code side
    namespace is used only where the declared layer states none. Dropping it puts every
    relative name of that node at the root, which is a topic the running system never has.
    """
    if not stated or stated == "/" or item.namespace_stated:
        return item
    return replace(item, namespace=stated)


def _supersede(
    builder: model.AtlasBuilder,
    index: Index,
    report: JoinReport,
) -> None:
    atlas = builder.atlas
    rows: list[tuple[Instance, list[str], str]] = []
    splits: dict[str, tuple[Instance, dict[str, dict[str, Any]], tuple[Carrier, ...]]] = {}
    for item in instances(atlas):
        found, _ = index.of(item)
        if not found or item.node_id not in atlas.entities:
            continue
        names = _code_names(found)
        nodes = _split_names(found)
        if nodes:
            if atlas.entities[item.node_id].attrs.get("name_from_code"):
                continue
            taking = sorted(name for name in nodes if not ignores_global_arguments(name, found))
            if item.name_source in UNNAMED:
                splits.setdefault(item.node_id, (item, nodes, found))
            elif len(taking) > 1:
                _collision(builder, item, taking, report)
            continue
        contexts = {
            row["construction"]
            for carrier in found
            for row in carrier.nodes
            if row.get("construction")
        }
        if len(contexts) > 1 or any(
            row.get("construction_ambiguous") for carrier in found for row in carrier.nodes
        ):
            _ambiguous(builder, item, names, report, reason="constructions")
            _starts_several(builder, item, names)
            continue
        if not names:
            continue
        item = _placed(item, _code_namespace(found))
        if item.known_name and item.known_name not in names:
            report.contradictions.append((item.pkg, item.executable, item.known_name, names[0]))
        if item.name_source not in UNNAMED:
            continue
        if len(names) > 1:
            _ambiguous(builder, item, names, report, one_process=True)
            _starts_several(builder, item, names)
            continue
        new_id = _target_id(item, names[0])
        if new_id and new_id != item.node_id:
            rows.append((item, names, new_id))
    for item, nodes, found in splits.values():
        _split(builder, item, nodes, found, report)
    # One profile launching this executable twice states two runtime nodes for one code name,
    # so neither of them may take it.
    crowded = {
        key
        for key, count in Counter((item.profile, new_id) for item, _, new_id in rows).items()
        if count > 1
    }
    reported: set[tuple[str, str]] = set()
    for item, names, new_id in rows:
        if item.node_id not in atlas.entities:
            # Profiles launching one unnamed process share its synthetic id, and the first
            # row already carried every profile's facts over.
            continue
        if (item.profile, new_id) in crowded:
            if (item.profile, new_id) not in reported:
                reported.add((item.profile, new_id))
                _ambiguous(builder, item, [names[0]], report, reason="instances")
            continue
        if _taken(atlas, new_id, item):
            _ambiguous(builder, item, [names[0], item.name or item.executable], report)
            continue
        _yield_to_named(builder, item.node_id, new_id)
        builder.supersede(item.node_id, new_id)
        node = builder.node(new_id)
        node.name_source = model.NameSource.LITERAL.value
        node.attrs.setdefault("name_from_code", names[0])
        _settle_params(builder, new_id)
        remaining = []
        for entry in atlas.unresolved:
            if entry.owner == new_id and entry.want == "node_name" and entry.role == "node":
                builder.count_resolved(entry.want)
            else:
                remaining.append(entry)
        atlas.unresolved = remaining
        report.superseded += 1


def _own_arguments(atlas: model.Atlas, item: Instance, found: tuple[Carrier, ...]) -> Instance:
    """A node a split made that reads no global argument binds without the site's remaps and
    namespace."""
    entity = atlas.entities.get(item.node_id)
    name = str(entity.attrs.get("name_from_code") or "") if entity is not None else ""
    fqn = ids.parse_node(item.node_id).fqn
    if not fqn or name not in _split_names(found) or not ignores_global_arguments(name, found):
        return item
    return replace(item, remaps=(), namespace=ids.split_ros_name(fqn)[0])


UNNAMED = (model.NameSource.EXECUTABLE_DEFAULT.value, model.NameSource.UNKNOWN.value)
"""Name sources of a launch site that states no name, so the source decides it."""


def _split_names(found: tuple[Carrier, ...]) -> dict[str, dict[str, Any]]:
    """The node row of each node one process runs, by name, when the source names each one.

    Empty unless there are several, every one a literal of its own: a name built at run time,
    two nodes under one name or two constructions nothing tells apart keep the process one
    identity, which is a blind spot and never a guess.
    """
    out: dict[str, dict[str, Any]] = {}
    for carrier in found:
        for row in carrier.nodes:
            name = str(row.get("name") or "").strip()
            if (
                not name
                or row.get("construction_ambiguous")
                or str(row.get("name_source") or "literal") != "literal"
            ):
                return {}
            held = out.setdefault(name, row)
            if not _context_of(row) or _context_of(held) != _context_of(row):
                return {}
    contexts = {_context_of(row) for row in out.values()}
    return out if len(out) > 1 and len(contexts) == len(out) else {}


def _context_of(row: dict[str, Any]) -> str:
    return str(row.get("construction") or "") or _short(str(row.get("class") or ""))


def _short(name: str) -> str:
    return name.rpartition("::")[2]


def _split(
    builder: model.AtlasBuilder,
    item: Instance,
    nodes: dict[str, dict[str, Any]],
    found: tuple[Carrier, ...],
    report: JoinReport,
) -> None:
    """One process a launch site starts unnamed, whose source names each node it runs.

    rclcpp gives each node of the process the name its own source states, so the site starts
    one node per name, in the namespace the site gives unless the source states one. Each
    takes the launch site, its arguments, its remaps and the parameter files, which reach
    every node of the process, and the bind keeps for each the records of its own
    construction. A node built with `use_global_arguments(false)` reads none of those
    arguments, so it keeps its own namespace and no parameter the site hands it.
    """
    atlas = builder.atlas
    names = sorted(nodes)
    local = {name for name in names if ignores_global_arguments(name, found)}
    targets = {
        name: _target_id(
            replace(item, namespace=_row_namespace(row) or "/")
            if name in local
            else _placed(item, _row_namespace(row)),
            name,
        )
        for name, row in nodes.items()
    }
    profiles = {fact.profile for fact in atlas.entities[item.node_id].facts}
    held = [atlas.entities.get(target) for target in targets.values()]
    if (
        not all(targets.values())
        or len(set(targets.values())) < len(targets)
        or any(
            other is not None
            and any(other.fact(DECLARED, profile) is not None for profile in profiles)
            for other in held
        )
    ):
        _ambiguous(builder, item, names, report, one_process=True)
        _starts_several(builder, item, names)
        return
    builder.split(item.node_id, sorted(targets.values()))
    for name in names:
        node = builder.node(targets[name])
        node.name_source = model.NameSource.LITERAL.value
        node.attrs.setdefault("name_from_code", name)
        _settle_params(builder, targets[name])
        if name in local:
            _drop_launch_params(builder, targets[name])
    remaining = []
    for entry in atlas.unresolved:
        if (
            entry.owner in targets.values()
            and entry.want == model.Want.NODE_NAME.value
            and entry.role == "node"
        ):
            builder.count_resolved(entry.want)
        else:
            remaining.append(entry)
    atlas.unresolved = remaining
    report.superseded += 1


YAML_MECHANISMS = frozenset(
    {
        model.Mechanism.YAML_BASE.value,
        model.Mechanism.YAML_OVERLAY_MERGE.value,
        model.Mechanism.YAML_WILDCARD.value,
        model.Mechanism.LAUNCH_PARAMS_FILE.value,
        model.Mechanism.LAUNCH_REWRITTEN_YAML.value,
        model.Mechanism.ROS_ARGS_PARAMS_FILE.value,
    }
)
KEEP_SHADOW = frozenset({model.ShadowKind.NOT_READ_IN_PROFILE.value, model.ShadowKind.OTHER.value})


def _settle_params(builder: model.AtlasBuilder, node_id: str) -> None:
    """The chains of a node a split made, read for the name it has now.

    The parameter collector followed a block keyed on this name as a chain of its own, which
    states this name as its owner and matched no launched node. One is launched now, so its
    first row wins. A chain copied from the process was followed under the executable's name,
    and loses every row of a block keyed on another name. A chain left with no row in a
    profile was configuration of another node, and goes with its fact.
    """
    atlas = builder.atlas
    fqn = ids.parse_node(node_id).fqn or ""
    for entity in list(atlas.by_kind(model.NodeKind.PARAM.value)):
        if ids.parse_param(entity.id).owner != node_id:
            continue
        rows: list[model.Candidate] = []
        for profile in sorted({row.profile for row in entity.candidates}):
            chain = sorted(
                (row for row in entity.candidates if row.profile == profile),
                key=lambda row: row.rank,
            )
            fact = entity.fact(DECLARED, profile)
            owner = str(fact.values.get("owner") or "") if fact is not None else ""
            if (
                owner == fqn
                and fact is not None
                and fact.values.get("state") == model.ParamState.OWNER_NAME_MISMATCH.value
            ):
                chain = _ranked(chain, wins=True)
                builder.edge(
                    model.EdgeKind.HAS_PARAM.value,
                    node_id,
                    entity.id,
                    layer=CODED,
                    profile=profile,
                )
            elif owner and owner != fqn:
                kept = [row for row in chain if _reaches(row, fqn)]
                if len(kept) < len(chain):
                    chain = _ranked(kept, wins=any(row.wins for row in chain))
                if not chain:
                    entity.facts = [
                        item
                        for item in entity.facts
                        if not (item.layer == DECLARED and item.profile == profile)
                    ]
            rows.extend(chain)
        entity.candidates = sorted(rows, key=model.Candidate.sort_key)
        if any(row.wins for row in rows):
            entity.state = model.ParamState.EFFECTIVE.value
        if not entity.candidates and not entity.facts:
            _drop(builder, entity.id)


LAUNCH_MECHANISMS = YAML_MECHANISMS | {
    model.Mechanism.LAUNCH_INLINE_DICT.value,
    model.Mechanism.ROS_ARGS_P.value,
    model.Mechanism.PYTHON_OVERRIDE.value,
}
"""The candidates a process receives as its global `--ros-args`."""


def _drop_launch_params(builder: model.AtlasBuilder, node_id: str) -> None:
    """The candidates a split copied from the launch site, off a node that reads none of them."""
    atlas = builder.atlas
    for entity in list(atlas.by_kind(model.NodeKind.PARAM.value)):
        if ids.parse_param(entity.id).owner != node_id:
            continue
        kept = [row for row in entity.candidates if row.mechanism not in LAUNCH_MECHANISMS]
        if len(kept) == len(entity.candidates):
            continue
        gone = {row.profile for row in entity.candidates} - {row.profile for row in kept}
        entity.facts = [
            fact for fact in entity.facts if not (fact.layer == DECLARED and fact.profile in gone)
        ]
        rows: list[model.Candidate] = []
        for profile in sorted({row.profile for row in kept}):
            chain = [row for row in kept if row.profile == profile]
            rows.extend(_ranked(chain, wins=any(row.wins for row in chain)))
        entity.candidates = rows
        if not rows and not entity.facts:
            _drop(builder, entity.id)
        elif not any(row.wins for row in rows):
            entity.state = model.ParamState.OTHER.value


def _yield_to_named(builder: model.AtlasBuilder, old_id: str, new_id: str) -> None:
    """Drop the process chain of every profile the chain followed under the new name states.

    That chain was followed for this exact name, so it is the one to keep, and two chains for
    one profile cannot be merged by a supersession.
    """
    atlas = builder.atlas
    for entity in list(atlas.by_kind(model.NodeKind.PARAM.value)):
        owner, name = ids.parse_param(entity.id)
        named = atlas.entities.get(ids.build_param(new_id, name)) if owner == old_id else None
        if named is None:
            continue
        stated = {row.profile for row in named.candidates} | {
            fact.profile for fact in named.facts if fact.layer == DECLARED
        }
        entity.candidates = [row for row in entity.candidates if row.profile not in stated]
        entity.facts = [
            fact for fact in entity.facts if not (fact.layer == DECLARED and fact.profile in stated)
        ]
        if not entity.candidates and not entity.facts:
            _drop(builder, entity.id)


def _ranked(chain: list[model.Candidate], *, wins: bool) -> list[model.Candidate]:
    out: list[model.Candidate] = []
    for index, row in enumerate(chain):
        if index == 0:
            out.append(replace(row, rank=0, wins=wins, shadow_kind=None, shadowed_by=None))
            continue
        kind = row.shadow_kind
        if wins and kind not in KEEP_SHADOW:
            kind = _shadow_of(chain[0].value, row.value)
        out.append(replace(row, rank=index, wins=False, shadow_kind=kind, shadowed_by=0))
    return out


def _reaches(row: model.Candidate, fqn: str) -> bool:
    """Whether the yaml block a candidate was read from selects this node.

    The locator is the key path, so the block key is what precedes `ros__parameters`. A row
    that is not a yaml key, or whose block key cannot be read, reached the process and stays.
    """
    if row.mechanism not in YAML_MECHANISMS or row.source is None:
        return True
    head, found, _rest = row.source.locator.partition(".ros__parameters.")
    if not found:
        return True
    try:
        return ids.param_key_matches(yaml_lines.selector_of(head.split(".")), fqn)
    except ContractError:
        return True


def _drop(builder: model.AtlasBuilder, entity_id: str) -> None:
    atlas = builder.atlas
    del atlas.entities[entity_id]
    for handle in sorted(atlas.edges):
        edge = atlas.edges[handle]
        if entity_id in (edge.src, edge.dst):
            del atlas.edges[handle]
    atlas.unresolved = [entry for entry in atlas.unresolved if entry.owner != entity_id]


def _collision(
    builder: model.AtlasBuilder,
    item: Instance,
    names: list[str],
    report: JoinReport,
) -> None:
    """A launch name on a process whose source runs several nodes gives all of them that name.

    `name=` and an argv `__node` rule remap every node of the process, so the nodes the
    source names apart all start under the one name, and nothing can tell them apart.
    """
    fact = builder.atlas.entities[item.node_id].fact(DECLARED, item.profile)
    where = list(fact.evidence) if fact is not None else []
    stated = ids.parse_file(where[0].file) if where else None
    site = f"{stated.package}/{stated.relpath}" if stated else "the launch file"
    fqn = ids.parse_node(item.node_id).fqn or item.name
    listed = ", ".join(names)
    finding = model.Finding(
        rule="node.ambiguous_identity",
        severity=model.Severity.WARN.value,
        subject=item.node_id,
        title=f"{item.pkg}/{item.executable} runs {len(names)} nodes under the one name {fqn}",
        detail=(
            f"the launch site names the process {fqn}, and a launch name is a __node remap "
            f"rclcpp applies to every node of the process, so the nodes its source names "
            f"{listed} all run as {fqn}"
        ),
        fix=(
            "remove the name from the launch site, or rename one node at a time with a remap "
            f"written <node>:__node:=<new name>, such as {names[0]}:__node:=<new name>"
        ),
        say=(
            f"{site} names {item.pkg}/{item.executable} {fqn}, which renames each of its "
            f"nodes {listed} to {fqn}, so they share one name; drop the name, or remap one "
            "node at a time with <node>:__node:=<new name>"
        ),
        discriminator=ids.digest("collision", *names),
        profiles=[item.profile],
        layers=[DECLARED, CODED],
        where=where,
    )
    if any(existing.id == finding.id for existing in builder.atlas.findings):
        return
    builder.finding(finding)
    report.ambiguous += 1


def _launched(atlas: model.Atlas, item: Instance) -> bool:
    from rosio.drift.rules import site_live

    sites = [
        atlas.entities[edge.src]
        for edge in atlas.in_edges(item.node_id)
        if edge.kind == model.EdgeKind.LAUNCHES.value
        and edge.src in atlas.entities
        and edge.fact(DECLARED, item.profile) is not None
    ]
    return not sites or any(site_live(site, item.profile) for site in sites)


def _starts_several(builder: model.AtlasBuilder, item: Instance, names: list[str]) -> None:
    """The name blind spot of an unnamed launch site whose executable starts several nodes
    says so, rather than blaming a table the name was never going to come from."""
    if len(names) < 2:
        return
    for entry in builder.atlas.unresolved:
        if entry.owner == item.node_id and entry.want == model.Want.NODE_NAME.value:
            entry.reason = (
                f"{item.pkg}/{item.executable} starts the nodes {', '.join(names)} and "
                "launch names none of them"
            )


def _target_id(item: Instance, name: str) -> str:
    fqn = _expand(name, item.namespace or "/", name)
    if not fqn:
        return ""
    try:
        return ids.build_node(fqn)
    except ContractError:
        return ""


def _taken(atlas: model.Atlas, new_id: str, item: Instance) -> bool:
    """True when another instance already owns the name the code states.

    One profile launching the same executable twice is two runtime nodes, so the second one
    must not be folded into the first: merging them would hand one entity both instances'
    remaps and publish the union of two topic sets. Across profiles the same name is the same
    node and the merge is right, which is why the profile is part of the question.
    """
    other = atlas.entities.get(new_id)
    if other is None:
        return False
    return other.fact(DECLARED, item.profile) is not None


def _ambiguous(
    builder: model.AtlasBuilder,
    item: Instance,
    names: list[str],
    report: JoinReport,
    reason: str = "names",
    *,
    one_process: bool = False,
) -> None:
    disc = ids.digest(reason, *sorted(names))
    where = ids.parse_profile(item.profile) if item.profile else "this profile"
    if reason == "constructions":
        detail = (
            "the executable constructs multiple node contexts, so no single context "
            "identifies this launch instance"
        )
        say = (
            f"{item.pkg}/{item.executable} constructs multiple nodes, so rosio kept "
            "the declared identity unresolved"
        )
    elif reason == "instances":
        detail = (
            f"{where} launches this executable more than once and its source states one node "
            f"name, {names[0]}, so the instances cannot be told apart by that name"
        )
        say = (
            f"{where} launches {item.pkg}/{item.executable} more than once and the source names "
            f"the node {names[0]}, so rosio kept the instances apart instead of merging them"
        )
    else:
        detail = (
            "the sources behind this executable state "
            + ", ".join(sorted(names))
            + ", so the name the declared layer recorded cannot be replaced by one of them"
        )
        say = (
            f"{item.pkg}/{item.executable} states the node names "
            + ", ".join(sorted(names))
            + ", so rosio kept both identities instead of merging them"
        )
    fix = "name the node in the launch file, or keep one node name literal in the source"
    if one_process or reason == "constructions":
        # A name= in the launch file is a __node remap, which renames every node in the process.
        detail += (
            ". The nodes run in one process, so a name, a remap or a parameter file given to "
            "this launch site reaches all of them, and rosio cannot tell which one it is meant for"
        )
        if one_process:
            say = (
                f"{item.pkg}/{item.executable} runs the nodes {', '.join(sorted(names))} in one "
                "process, so rosio cannot tell which of them a remap or parameter of this "
                "launch site is meant for"
            )
        fix = (
            "leave the launch file name alone, it would rename every node in the process; "
            "address one node by its own name instead, a yaml block under that name and a "
            "remap written <node>:<from>:=<to>"
        )
    finding = model.Finding(
        rule="node.ambiguous_identity",
        severity=model.Severity.WARN.value,
        subject=item.node_id,
        title=f"{item.pkg}/{item.executable} could be more than one node",
        detail=detail,
        fix=fix,
        say=say,
        discriminator=disc,
        profiles=[item.profile],
        layers=[DECLARED, CODED],
    )
    if any(existing.id == finding.id for existing in builder.atlas.findings):
        return
    builder.finding(finding)
    report.ambiguous += 1


# ---------------------------------------------------------------------------
# endpoints and parameters


def _source(carrier: Carrier, row: dict[str, Any]) -> list[model.Source]:
    if not carrier.file:
        return []
    locator = str(row.get("owner_symbol") or row.get("symbol") or "")
    line = row.get("line_hint")
    return [
        model.Source(
            file=carrier.file,
            locator=locator,
            line_hint=int(line) if isinstance(line, int) else None,
        ),
        *carrier.construction_sources,
    ]


def _symbol_of(carrier: Carrier, row: dict[str, Any]) -> str:
    """The site of one coded row, as a sym id, which is the one shape this module states.

    A record spells its owner as a qualified name and a symbol fact carries the id, and a
    rule that has to accept both spellings for one field is a rule written against two
    products. Everything that reaches an edge value or an edge key goes through here.
    """
    name = str(row.get("owner_symbol") or row.get("symbol") or "")
    if name and carrier.file:
        try:
            parsed = ids.parse_file(carrier.file)
            return ids.build_sym(parsed.package, parsed.relpath, name)
        except ContractError:
            pass
    return carrier.symbol or carrier.owner


def _with_node_plugins(
    atlas: model.Atlas, item: Instance, found: tuple[Carrier, ...]
) -> tuple[Carrier, ...]:
    owners = {
        _class_type(ids.parse_sym(carrier.symbol).qualname).rpartition("::")[0]
        for carrier in found
        if carrier.symbol
    }
    all_carriers = carriers(atlas)
    selectors = {
        selector
        for carrier in all_carriers
        if carrier.file and ids.parse_file(carrier.file).package == item.pkg
        for row in carrier.classes
        if row["name"] in owners
        for selector in row.get("node_plugin_parameters", [])
    }
    if not selectors:
        return found
    by_file: dict[str, list[Carrier]] = {}
    for carrier in all_carriers:
        by_file.setdefault(carrier.file, []).append(carrier)
    table = _class_table(by_file, _plugin_files(atlas))
    implementations: dict[str, set[str]] = {}
    for entity in atlas.entities.values():
        if entity.kind == "plugin":
            for fact in entity.facts:
                if (
                    isinstance(fact.values.get("type"), str)
                    and fact.confidence != model.Confidence.UNKNOWN.value
                ):
                    implementations.setdefault(ids.parse_plugin(entity.id).lookup, set()).add(
                        fact.values["type"]
                    )
    selected = {carrier.owner: carrier for carrier in found}
    for selector in sorted(selectors):
        value = _param_value(atlas, item, selector)
        if value is None:
            configured = atlas.entities.get(ids.build_param(item.node_id, selector))
            if configured is not None and (
                any(
                    fact.layer == DECLARED and fact.profile == item.profile
                    for fact in configured.facts
                )
                or any(
                    candidate.profile == item.profile
                    and candidate.mechanism != model.Mechanism.CODE_DEFAULT.value
                    for candidate in configured.candidates
                )
            ):
                continue
            defaults = {
                row.get("default")
                for carrier in found
                for row in carrier.parameters
                if row.get("name") == selector
                and isinstance(row.get("default"), str)
                and row.get("has_default", True)
                and row.get("default_state") not in ("unknown", "unresolved")
            }
            value = next(iter(defaults)) if len(defaults) == 1 else None
        if isinstance(value, str):
            classes = implementations.get(value, set())
            if len(classes) != 1:
                continue
            implementation = next(iter(classes))
            for carrier in table.get(value, []):
                rows = {
                    field: tuple(
                        row for row in getattr(carrier, field) if _row_of_class(row, implementation)
                    )
                    for field in (
                        "nodes",
                        "endpoints",
                        "parameters",
                        "parameter_reads",
                        "plugin_parameters",
                        "tf_broadcasts",
                        "tf_lookups",
                    )
                }
                if any(rows.values()):
                    selected.setdefault(carrier.owner, replace(carrier, **rows))
    return tuple(selected[key] for key in sorted(selected))


SCOPED_FIELDS = (
    "endpoints",
    "parameters",
    "parameter_reads",
    "plugin_parameters",
    "tf_broadcasts",
    "tf_lookups",
)


def _own(atlas: model.Atlas, item: Instance, found: tuple[Carrier, ...]) -> tuple[Carrier, ...]:
    """The records of the one node a split gave this id, out of a process that runs several.

    A record of another node's class or of another construction is that node's, and so is a
    record of a class only another node inherits or hands itself to. A record of a class no
    node reaches that way, or that several reach, or of a free function, stays with every node.
    """
    entity = atlas.entities.get(item.node_id)
    name = str(entity.attrs.get("name_from_code") or "") if entity is not None else ""
    nodes = _split_names(found)
    if name not in nodes:
        return found
    mine = nodes[name]
    construction = str(mine.get("construction") or "")
    graph = _class_graph(atlas, item.pkg)
    own = _reach(graph, _short(str(mine.get("class") or "")))
    others = {
        reached
        for row in nodes.values()
        if row is not mine
        for reached in _reach(graph, _short(str(row.get("class") or "")))
    } - own

    def kept(row: dict[str, Any]) -> bool:
        if construction and row.get("construction") and row["construction"] != construction:
            return False
        owner = str(row.get("owner_symbol") or row.get("symbol") or "")
        if owner.startswith("sym:"):
            owner = ids.parse_sym(owner).qualname
        return _short(_class_type(owner).rpartition("::")[0]) not in others

    return tuple(
        replace(
            carrier,
            nodes=tuple(row for row in carrier.nodes if row.get("name") == name),
            **{field: tuple(filter(kept, getattr(carrier, field))) for field in SCOPED_FIELDS},
        )
        for carrier in found
    )


def _class_graph(atlas: model.Atlas, pkg: str) -> dict[str, set[str]]:
    """Each class of a package, by short name, to the classes it inherits or hands itself to."""
    graph: dict[str, set[str]] = {}
    for entity in atlas.by_kind("file"):
        if ids.parse_file(entity.id).package != pkg:
            continue
        sources = [entity.attrs, *(fact.values for fact in entity.facts if fact.layer == CODED)]
        for values in sources:
            for row in _records(values).get("classes", ()):
                graph.setdefault(_short(_class_type(str(row.get("name") or ""))), set()).update(
                    _short(_class_type(str(other)))
                    for other in [*row.get("bases", []), *row.get("node_helpers", [])]
                )
    return graph


def _reach(graph: dict[str, set[str]], name: str) -> set[str]:
    reached: set[str] = set()
    pending = [name] if name else []
    while pending:
        current = pending.pop()
        if current and current not in reached:
            reached.add(current)
            pending.extend(graph.get(current, ()))
    return reached


def _row_of_class(row: dict[str, Any], implementation: str) -> bool:
    owner = str(row.get("owner_symbol") or row.get("symbol") or "")
    if owner.startswith("sym:"):
        owner = ids.parse_sym(owner).qualname
    return (
        row.get("class") == implementation
        or _class_type(owner).rpartition("::")[0] == implementation
    )


def _bind_instance(
    builder: model.AtlasBuilder,
    item: Instance,
    found: tuple[Carrier, ...],
    via: str,
    report: JoinReport,
    count: bool,
) -> None:
    found = _own(builder.atlas, item, _with_node_plugins(builder.atlas, item, found))
    contexts = {
        str(row["construction"])
        for carrier in found
        for row in carrier.nodes
        if row.get("construction")
    }
    if len(contexts) > 1 or any(
        row.get("construction_ambiguous") for carrier in found for row in carrier.nodes
    ):
        for carrier in found:
            for row in carrier.endpoints:
                if row.get("from_parameter") and row.get("kind") in ENDPOINT_EDGES:
                    want = WANTS[ENDPOINT_EDGES[row["kind"]][1]]
                    report.blocked_sites.add(
                        (
                            want,
                            _symbol_of(carrier, row),
                            str(row.get("raw") or row.get("name") or ""),
                            str(row.get("construction") or ""),
                        )
                    )
            for row in carrier.tf_broadcasts:
                for role in ("parent", "child"):
                    _frame_site(report, carrier, row, role, False)
            for row in carrier.tf_lookups:
                for role in ("target", "source"):
                    _frame_site(report, carrier, row, role, False)
        builder.unresolved(
            model.Unresolved(
                want=model.Want.OTHER.value,
                owner=item.node_id,
                expr=f"{item.pkg}/{item.executable}",
                reason="multiple_constructed_nodes",
                role="coded_constructions",
                resolvable_by=[
                    "launch each node as a separate component or inspect its source contexts"
                ],
                evidence=[
                    source
                    for carrier in found
                    for row in carrier.nodes
                    for source in _source(carrier, row)
                ],
            )
        )
        return
    scoped_fields = (
        "endpoints",
        "parameters",
        "parameter_reads",
        "plugin_parameters",
        "tf_broadcasts",
        "tf_lookups",
    )
    scoped = any(
        row.get("construction")
        for carrier in found
        for field in scoped_fields
        for row in getattr(carrier, field)
    )
    unscoped = {
        str(row.get("class") or "")
        for carrier in found
        for row in carrier.nodes
        if not row.get("construction")
    } - {""}
    if not contexts and any(
        row.get("construction")
        and not any(str(row.get("owner_symbol") or "").startswith(cls + "::") for cls in unscoped)
        for carrier in found
        for field in scoped_fields
        for row in getattr(carrier, field)
    ):
        builder.unresolved(
            model.Unresolved(
                want=model.Want.OTHER.value,
                owner=item.node_id,
                expr=f"{item.pkg}/{item.executable}",
                reason="construction_context_unavailable",
                role="coded_constructions",
                resolvable_by=["supply the construction context for this executable or component"],
                evidence=[
                    source
                    for carrier in found
                    for row in carrier.endpoints
                    if row.get("construction")
                    for source in _source(carrier, row)
                ],
            )
        )
    if contexts or scoped:
        # A registered component keeps unscoped records for loads without a main, and a
        # selected context replaces them.
        constructed = {
            str(row.get("class") or "")
            for carrier in found
            for row in carrier.nodes
            if row.get("construction") in contexts
        } - {""}

        def kept(row: dict[str, Any], owner: str) -> bool:
            if row.get("construction"):
                return row["construction"] in contexts
            name = str(row.get(owner) or "")
            return not any(name == cls or name.startswith(cls + "::") for cls in constructed)

        found = tuple(
            replace(
                carrier,
                nodes=tuple(row for row in carrier.nodes if kept(row, "class")),
                **{
                    field: tuple(
                        row for row in getattr(carrier, field) if kept(row, "owner_symbol")
                    )
                    for field in scoped_fields
                },
            )
            for carrier in found
        )
    files = sorted({carrier.file for carrier in found if carrier.file})
    values: dict[str, Any] = {"coded_sources": files, "bound_through": via}
    exe = _exe_id(item)
    if exe:
        values["exe"] = exe
    if item.plugin_class:
        values["plugin_class"] = item.plugin_class
    names = _code_names(found)
    if len(names) == 1:
        values["code_name"] = names[0]
    stated = _code_namespace(found)
    if stated:
        values["code_namespace"] = stated
    auto = auto_declare_of(item.name.rpartition("/")[2], found)
    if auto:
        values["auto_declare"] = auto
    item = _placed(item, stated)
    reads = _read_names(found)
    assigned = _argv_assigned(item)
    builder.fact(
        item.node_id,
        CODED,
        profile=item.profile,
        values=values,
        confidence=model.Confidence.RESOLVED.value,
        attrs={"coded_params_known": True},
        evidence=[
            source
            for carrier in found
            for row in carrier.nodes
            if row.get("name_source") == "literal"
            for source in _source(carrier, row)
        ],
    )
    for carrier in found:
        for row in carrier.declarations():
            _bind_parameter(builder, item, carrier, row, reads, assigned, report)
    for carrier in found:
        for row in carrier.endpoints:
            _bind_endpoint(builder, item, carrier, row, report, count)
    # The reads come after every declaration of every carrier. A read that runs first mints
    # the parameter with the note that no declaration was found, and a declaration in a
    # later file then leaves that sentence on a parameter it does declare.
    for carrier in found:
        for row in carrier.parameter_reads:
            _bind_read(builder, item, carrier, row, report)
    # The frames come last: a frame name can be a parameter value, and the parameter has to
    # carry its chain before its value can be read.
    for carrier in found:
        for row in carrier.tf_broadcasts:
            _bind_broadcast(builder, item, carrier, row, report)
        for row in carrier.tf_lookups:
            _bind_lookup(builder, item, carrier, row, report)


def _endpoint_targets(row: dict[str, Any]) -> list[tuple[str, str, str, bool]]:
    """(kind, raw name, type, via wrapper) per endpoint a record produces.

    A record that already carries the name the wrapper creates is taken as it stands. The
    table expands only what the collector left to expand, so one construction never becomes
    two sets of endpoints.
    """
    kind = str(row.get("kind") or "")
    raw = str(row.get("name") or "")
    wrapper = _wrapper_of(row)
    if wrapper and not raw:
        expanded = wrappers.endpoints_for(wrapper, raw)
        if expanded:
            return [
                (item[0], item[1], item[2] or str(row.get("type") or ""), True) for item in expanded
            ]
    if kind not in ENDPOINT_EDGES:
        return []
    return [(kind, raw, str(row.get("type") or ""), bool(wrapper))]


def _wrapper_of(row: dict[str, Any]) -> str:
    for key in WRAPPER_KEYS:
        value = row.get(key)
        if isinstance(value, str) and value:
            return value
    return ""


def _bind_endpoint(
    builder: model.AtlasBuilder,
    item: Instance,
    carrier: Carrier,
    row: dict[str, Any],
    report: JoinReport,
    count: bool,
) -> None:
    if _guard_off(builder.atlas, item, row):
        report.guarded_off += 1
        return
    evidence = _source(carrier, row)
    resolved_row = row
    parameter = str(row.get("from_parameter") or "")
    if parameter and row.get("kind") != "timer":
        name, _ = _parameter_name(
            builder.atlas,
            item,
            parameter,
            str(row.get("parameter_suffix") or ""),
            str(row.get("name") or ""),
        )
        resolved_row = {**row, "name": name or None}
    for kind, raw, type_name, via_wrapper in _endpoint_targets(resolved_row):
        edge_kind, entity_kind = ENDPOINT_EDGES[kind]
        bound = apply_remaps(raw, item.namespace, item.name, item.remaps)
        if parameter:
            expr = str(row.get("raw") or row.get("name") or "")
            sites = report.resolved_sites if bound is not None else report.blocked_sites
            sites.add(
                (
                    WANTS[entity_kind],
                    _symbol_of(carrier, row),
                    expr,
                    str(row.get("construction") or ""),
                )
            )
            sites.add((WANTS[entity_kind], item.node_id, expr, str(row.get("construction") or "")))
        if bound is None:
            report.endpoints_unresolved += 1
            expr = str(row.get("raw") or raw or row.get("expr") or kind)
            if any(
                (WANTS[entity_kind], expr, source.file, source.line_hint) in report.coded_blind
                for source in evidence
            ):
                # The collector already owns this blind spot at the symbol that states the
                # site, and a second entry under the node counted one line twice.
                continue
            builder.unresolved(
                model.Unresolved(
                    want=WANTS[entity_kind],
                    owner=item.node_id,
                    expr=expr,
                    reason=str(
                        row.get("reason")
                        or "the source does not state this name and no remap resolves it"
                    ),
                    role=kind,
                    construction=str(row.get("construction") or ""),
                    resolvable_by=["remap the name in the launch file", "probe the running system"],
                    evidence=evidence,
                )
            )
            continue
        endpoint_id = BUILDERS[entity_kind](bound.name)
        builder.node(endpoint_id)
        builder.fact(
            endpoint_id,
            CODED,
            profile=item.profile,
            values={"name": bound.name},
            confidence=model.Confidence.RESOLVED.value,
            evidence=evidence,
        )
        edge_values: dict[str, Any] = {
            "raw_name": raw,
            "remapped": bound.rule is not None,
            "via_known_wrapper": via_wrapper,
        }
        if bound.rule is not None:
            edge_values["remap"] = list(bound.rule)
        if type_name:
            edge_values["type"] = type_name
        if row.get("qos") is not None:
            edge_values["qos"] = row.get("qos")
        if row.get("condition"):
            edge_values["condition"] = row.get("condition")
        _site_edge(
            builder,
            edge_kind,
            item.node_id,
            endpoint_id,
            ids.edge_key(
                profile=item.profile,
                symbol=_symbol_of(carrier, row),
                raw_name=raw,
                kind=kind,
                # Two mutually exclusive create_subscription calls on one member name,
                # one per message type, are two sites and not one.
                type=type_name,
                condition=str(row.get("condition") or ""),
                **({"construction": row["construction"]} if row.get("construction") else {}),
            ),
            profile=item.profile,
            values=edge_values,
            evidence=evidence,
        )
        _attach_type(builder, endpoint_id, row, type_name, item.profile)
        report.endpoints_bound += 1
        if via_wrapper:
            report.wrapper_endpoints += 1
        if bound.rule is not None:
            report.endpoints_remapped += 1
            report.remap_cases.append((item.profile, item.node_id, raw, bound.name))
        if count:
            builder.count_resolved(WANTS[entity_kind])


def _type_id(row: dict[str, Any], type_name: str) -> str:
    stated = str(row.get("type_id") or "")
    if stated:
        try:
            ids.parse_type(stated)
        except ContractError:
            return ""
        return stated
    parts = type_name.split("/")
    if len(parts) != 3 or parts[1] not in ids.INTERFACE_KINDS:
        return ""
    try:
        return ids.build_type(parts[0], parts[1], parts[2])
    except ContractError:
        return ""


def _attach_type(
    builder: model.AtlasBuilder,
    endpoint_id: str,
    row: dict[str, Any],
    type_name: str,
    profile: str,
) -> None:
    type_id = _type_id(row, type_name)
    if not type_id:
        return
    builder.node(type_id)
    builder.edge(
        model.EdgeKind.USES_TYPE.value,
        endpoint_id,
        type_id,
        layer=CODED,
        profile=profile,
        values={"name": type_name or ids.label_for(type_id)},
        confidence=model.Confidence.RESOLVED.value,
    )


# ---------------------------------------------------------------------------
# frames


def frame_id(name: str, namespace: str = "/") -> str:
    """The frame entity id for a name one source states.

    A tf2 frame is global and carries no namespace, so a plain name keeps the spelling the
    source gives it. Prefixing every frame with the launch namespace would mint frames that
    no urdf and no other node ever names, which is a fabricated finding in every frame rule.
    The namespace is applied to the one form that really is node relative, a private `~`
    name, and it is recorded on the edge either way.
    """
    text = name.strip()
    if not text:
        return ""
    if text.startswith("~"):
        head = (namespace or "/").strip("/")
        text = text.lstrip("~").lstrip("/")
        text = f"{head}/{text}" if head else text
    try:
        return ids.build_frame(text)
    except ContractError:
        return ""


def _param_value(atlas: model.Atlas, item: Instance, name: str) -> Any:
    """The effective value of one parameter of this instance, in this profile.

    The candidate chain is read rather than the precomputed `effective` block, because
    `recompute` runs after this pass and a chain written in this pass would otherwise be
    visible only to the second run.
    """
    try:
        param_id = ids.build_param(item.node_id, ids.normalise_param_name(name))
    except ContractError:
        return None
    entity = atlas.entities.get(param_id)
    if entity is None or entity.state not in (
        model.ParamState.EFFECTIVE.value,
        model.ParamState.UNDECLARED.value,
    ):
        return None
    # A line the node ignores is read as the first run read it, so a second run binds the
    # same records.
    for candidate in sorted(entity.candidates, key=model.Candidate.sort_key):
        if (candidate.wins or candidate.note == IGNORED_NOTE) and candidate.profile == item.profile:
            return candidate.value
    return None


def _guard_off(atlas: model.Atlas, item: Instance, row: dict[str, Any]) -> bool:
    """True when a parameter the record's guard needs reads the other bool in this profile.

    The collector states `guard_parameters` only when every term of the guard is a parameter
    of the node, so one term read false makes the whole conjunction false. A value that is
    not a bool, or that no candidate wins, is not evaluated and the record binds as before.
    """
    for term in row.get("guard_parameters") or ():
        value = _param_value(atlas, item, str(term.get("name") or ""))
        wanted = term.get("value")
        if isinstance(value, bool) and isinstance(wanted, bool) and value != wanted:
            return True
    return False


def _parameter_name(
    atlas: model.Atlas, item: Instance, parameter: str, suffix: str, fallback: str
) -> tuple[str, str]:
    if parameter:
        value = _param_value(atlas, item, parameter)
        if isinstance(value, str) and value.strip():
            return value.strip() + suffix, "parameter"
        try:
            entity = atlas.entities.get(ids.build_param(item.node_id, parameter))
        except ContractError:
            return "", ""
        if entity is not None and (
            any(candidate.profile == item.profile for candidate in entity.candidates)
            or any(fact.layer == DECLARED and fact.profile == item.profile for fact in entity.facts)
        ):
            return "", ""
    return fallback, "literal" if fallback else ""


def _frame_site(
    report: JoinReport, carrier: Carrier, row: dict[str, Any], role: str, resolved: bool
) -> None:
    expr = str(row.get(f"{role}_expr") or "")
    if expr and row.get(f"{role}_from_parameter"):
        sites = report.resolved_sites if resolved else report.blocked_sites
        sites.add(
            (
                model.Want.FRAME_ID.value,
                _symbol_of(carrier, row),
                expr,
                str(row.get("construction") or ""),
            )
        )


def _retire_resolved_sites(builder: model.AtlasBuilder, report: JoinReport) -> None:
    resolved = report.resolved_sites - report.blocked_sites
    remaining = []
    for entry in builder.atlas.unresolved:
        if (entry.want, entry.owner, entry.expr, entry.construction) in resolved:
            builder.count_resolved(entry.want)
        else:
            remaining.append(entry)
    builder.atlas.unresolved = remaining


def _frame_of(
    atlas: model.Atlas, item: Instance, row: dict[str, Any], role: str
) -> tuple[str, str, str]:
    """(frame id, frame name, origin) for one side of a tf record.

    A frame follows this profile's parameter chain. An unreadable configured value never
    falls back to the source default.
    """
    name, origin = _parameter_name(
        atlas,
        item,
        str(row.get(f"{role}_from_parameter") or ""),
        str(row.get(f"{role}_parameter_suffix") or ""),
        str(row.get(role) or "").strip(),
    )
    return frame_id(name, item.namespace), name, origin


def _tf_key(profile: str, symbol: str, values: dict[str, Any]) -> str:
    """The key of one tf edge, over the site and the content it carries.

    Everything that would make two rows different edges is in the digest, so a second site
    can never overwrite the first and two identical sites can never become two edges. Line
    numbers stay out of it, which is what keeps the handle stable when a file grows a line.
    """
    content = ids.digest_fields({key: str(value) for key, value in values.items()})
    return ids.edge_key(profile=profile, symbol=symbol, content=content)


def _site_edge(
    builder: model.AtlasBuilder,
    kind: str,
    src: str,
    dst: str,
    key: str,
    *,
    profile: str,
    values: dict[str, Any],
    evidence: list[model.Source],
    confidence: str = model.Confidence.RESOLVED.value,
) -> None:
    """One coded edge over a site of one symbol.

    Two sites in one symbol can state the same content: two constructor overloads declaring
    one name, an `if` and an `else if` asking for one transform. No key may carry a line, so
    those fold into one edge holding both evidence rows. A site whose content differs from
    the edge its key already names takes its content into the key, so it never stands for
    the first.
    """
    edge = builder.atlas.edges.get(ids.build_edge(kind, src, dst, key))
    slot = None
    if edge is not None:
        slot = next((row for row in edge.facts if row.slot == (CODED, profile)), None)
    if slot is not None and (
        slot.confidence != confidence
        or any(
            name not in slot.values or slot.values[name] != value for name, value in values.items()
        )
    ):
        content = ids.digest_fields({name: str(value) for name, value in sorted(values.items())})
        key = ids.edge_key(site=key, content=content)
    builder.edge(
        kind,
        src,
        dst,
        key=key,
        layer=CODED,
        profile=profile,
        values=values,
        confidence=confidence,
        evidence=evidence,
        fold=True,
    )


def _tf_values(item: Instance, row: dict[str, Any], origins: dict[str, str]) -> dict[str, Any]:
    values: dict[str, Any] = {"namespace": item.namespace or "/"}
    if row.get("condition"):
        values["condition"] = row["condition"]
    for role, origin in sorted(origins.items()):
        if origin == "parameter":
            values[f"{role}_from_parameter"] = str(row.get(f"{role}_from_parameter") or "")
    return values


def _bind_broadcast(
    builder: model.AtlasBuilder,
    item: Instance,
    carrier: Carrier,
    row: dict[str, Any],
    report: JoinReport,
) -> None:
    atlas = builder.atlas
    if _guard_off(atlas, item, row):
        report.guarded_off += 1
        return
    parent_id, parent, parent_origin = _frame_of(atlas, item, row, "parent")
    child_id, child, child_origin = _frame_of(atlas, item, row, "child")
    _frame_site(report, carrier, row, "parent", bool(parent_id))
    _frame_site(report, carrier, row, "child", bool(child_id))
    if not child_id:
        # A broadcast defines its child, so a child nothing states is a blind spot the
        # collector already recorded and never a frame this pass invents.
        report.tf_unresolved += 1
        return
    origins = {"parent": parent_origin, "child": child_origin}
    if "parameter" in origins.values():
        report.tf_from_parameter += 1
    evidence = _source(carrier, row)
    symbol = _symbol_of(carrier, row)
    values = _tf_values(item, row, origins)
    values.update(
        {
            "parent": parent or None,
            "child": child,
            "static": str(row.get("kind") or "") == "static",
            "broadcaster": row.get("broadcaster") or None,
        }
    )
    builder.node(child_id)
    _site_edge(
        builder,
        model.EdgeKind.BROADCASTS_TF.value,
        item.node_id,
        child_id,
        _tf_key(item.profile, symbol, values),
        profile=item.profile,
        values=values,
        evidence=evidence,
    )
    report.tf_edges += 1
    report.tf_cases.append((item.profile, item.node_id, parent, child))
    if not parent_id:
        report.tf_unresolved += 1
        return
    parent_values = _tf_values(item, row, origins)
    parent_values.update(
        {
            "source": _BROADCAST_SOURCES.get(carrier.language, "coded_broadcast"),
            "node": item.node_id,
            "symbol": symbol,
            "static": values["static"],
        }
    )
    builder.node(parent_id)
    _site_edge(
        builder,
        model.EdgeKind.TF_PARENT.value,
        parent_id,
        child_id,
        # The geometry of one pair differs between profiles, so the profile and the node that
        # broadcasts it are both in the key.
        _tf_key(item.profile, symbol, parent_values),
        profile=item.profile,
        values=parent_values,
        evidence=evidence,
    )
    report.tf_edges += 1


def _bind_lookup(
    builder: model.AtlasBuilder,
    item: Instance,
    carrier: Carrier,
    row: dict[str, Any],
    report: JoinReport,
) -> None:
    atlas = builder.atlas
    evidence = _source(carrier, row)
    symbol = _symbol_of(carrier, row)
    api = str(row.get("api") or "")
    resolved = {role: _frame_of(atlas, item, row, role) for role in ("target", "source")}
    origins = {role: found[2] for role, found in resolved.items()}
    if "parameter" in origins.values():
        report.tf_from_parameter += 1
    for role in ("source", "target"):
        entity_id = resolved[role][0]
        _frame_site(report, carrier, row, role, bool(entity_id))
        if not entity_id:
            report.tf_unresolved += 1
            continue
        values = _tf_values(item, row, origins)
        values.update(
            {
                "api": api,
                "role": role,
                "target": resolved["target"][1] or None,
                "source": resolved["source"][1] or None,
            }
        )
        builder.node(entity_id)
        _site_edge(
            builder,
            model.EdgeKind.LISTENS_TF.value,
            item.node_id,
            entity_id,
            _tf_key(item.profile, symbol, values),
            profile=item.profile,
            values=values,
            evidence=evidence,
        )
        report.tf_edges += 1


# ---------------------------------------------------------------------------
# parameters


def _read_names(found: tuple[Carrier, ...]) -> frozenset[str]:
    """The parameter names the sources read back, whatever declared them.

    A read that names a parameter this symbol did not declare is the only evidence that the
    value reaches the running code, and until it is read here the rule that reports a
    parameter nobody reads has nothing to look at.
    """
    names = set()
    for carrier in found:
        for row in carrier.parameter_reads:
            name = str(row.get("name") or "")
            if name:
                names.add(name)
    return frozenset(names)


def _bind_read(
    builder: model.AtlasBuilder,
    item: Instance,
    carrier: Carrier,
    row: dict[str, Any],
    report: JoinReport,
) -> None:
    """One `get_parameter` site, as an edge from the symbol that reads to the parameter."""
    if not carrier.file:
        return
    try:
        name = ids.normalise_param_name(str(row.get("name") or ""))
        param_id = ids.build_param(item.node_id, name)
    except ContractError:
        return
    symbol = _symbol_of(carrier, row)
    if symbol not in builder.atlas.entities:
        # An edge endpoint has to exist, and the symbol a row names is one the collector
        # emits. A record that names a symbol nothing emitted falls back to its carrier.
        symbol = carrier.symbol or carrier.owner
    if ids.kind_of(symbol) not in ("sym", "node"):
        return
    known = builder.atlas.entities.get(param_id)
    if known is None or not known.state:
        # A read states that the node has this parameter and nothing about its value, and
        # the declaration can sit in a table this reader cannot follow, so the state is
        # `other` and never `undeclared`. The row carries no fact either: with no candidate
        # chain behind it, a fact would become this parameter's effective value.
        builder.set_param(
            param_id,
            state=model.ParamState.OTHER.value,
            note="read in code, no declaration this reader could follow",
        )
    _site_edge(
        builder,
        model.EdgeKind.READS_PARAM.value,
        symbol,
        param_id,
        ids.edge_key(
            profile=item.profile,
            symbol=symbol,
            name=name,
            api=str(row.get("api") or ""),
        ),
        profile=item.profile,
        values={"name": name, "api": str(row.get("api") or ""), "symbol": symbol},
        evidence=_source(carrier, row),
    )
    report.parameter_reads_bound += 1


def _argv_assigned(item: Instance) -> dict[str, str]:
    """The parameter names this instance's own command line sets, with the text it sets.

    A process started by path carries its parameters in argv, and the declared layer states
    no chain for it when it could not name the owner. A code default is then the only row
    left, and letting it win would publish the value of the other branch as configuration.
    The owner is known here, so the argv row is written as the candidate that wins and the
    code default becomes the line it shadows.
    """
    out: dict[str, str] = {}
    for name, text in ros_args.parse_ros_args(item.argv).assignments:
        try:
            out[ids.normalise_param_name(name)] = text
        except ContractError:
            continue
    return out


def _bind_parameter(
    builder: model.AtlasBuilder,
    item: Instance,
    carrier: Carrier,
    row: dict[str, Any],
    reads: frozenset[str],
    assigned: Mapping[str, str],
    report: JoinReport,
) -> None:
    if not carrier.file:
        # A parameter candidate carries provenance, and provenance cites a file entity.
        return
    if row.get("base_class") or row.get("suffix"):
        # A suffix only declaration is the plugin's, and the prefix is the yaml instance. It
        # never becomes a node parameter, because the name would be missing its prefix.
        _bind_plugin_parameter(builder, carrier, row, item.profile)
        return
    raw = row.get("name")
    if raw is None:
        return
    try:
        name = ids.normalise_param_name(raw)
        param_id = ids.build_param(item.node_id, name)
    except ContractError:
        return
    values: dict[str, Any] = {"declared_in_code": True}
    if row.get("type"):
        values["type"] = str(row["type"])
    read = row.get("read", row.get("reads"))
    if read is not None or str(row.get("name") or "") in reads:
        values["read"] = bool(read) or str(row.get("name") or "") in reads
    default, resolved = _default(row)
    if resolved:
        values["default"] = default
    builder.fact(
        param_id,
        CODED,
        profile=item.profile,
        values=values,
        confidence=model.Confidence.RESOLVED.value,
        evidence=_source(carrier, row),
    )
    # The chain is per profile, and one binary carries the same default into every profile
    # that runs it, so the row is written for each profile whose chain is still empty.
    node = builder.atlas.entities[param_id]
    if not any(candidate.profile == item.profile for candidate in node.candidates):
        _state_for(
            builder,
            param_id,
            item,
            carrier,
            row,
            default,
            resolved,
            wins=resolved and name not in assigned,
            argv_text=assigned.get(name),
        )
        report.parameters_code_only += 1
    builder.edge(
        model.EdgeKind.HAS_PARAM.value,
        item.node_id,
        param_id,
        layer=CODED,
        profile=item.profile,
    )
    owner = carrier.symbol or item.node_id
    if ids.kind_of(owner) in ("sym", "node"):
        _site_edge(
            builder,
            model.EdgeKind.DECLARES_PARAM.value,
            owner,
            param_id,
            ids.edge_key(profile=item.profile, symbol=owner, name=name),
            profile=item.profile,
            values={"name": name},
            evidence=_source(carrier, row),
        )
    report.parameters_attached += 1


def _default(row: dict[str, Any]) -> tuple[Any, bool]:
    state = str(row.get("default_state") or "")
    if state and state not in ("literal", "resolved"):
        return None, False
    if "default" not in row:
        return None, False
    value = row.get("default")
    if value is None:
        return None, False
    try:
        return model.canonical_value(value), True
    except ContractError:
        return None, False


def _state_for(
    builder: model.AtlasBuilder,
    param_id: str,
    item: Instance,
    carrier: Carrier,
    row: dict[str, Any],
    default: Any,
    resolved: bool,
    wins: bool,
    argv_text: str | None = None,
) -> None:
    """A parameter no file sets takes the default its declaration states.

    A declaration whose default is not a literal still gets its row, with no value and no
    win, so the parameter is visible and nothing reads a value rosio never saw. A default
    the command line of this process overrides keeps its value and loses to the argv row,
    because the value that runs is the one in argv and this row is not it. When the argv row
    cannot cite a file the site becomes a blind spot instead, since a chain where nothing
    wins and nothing is recorded reads as a parameter no one configured.
    """
    site = _source(carrier, row)[0]
    code_row = model.Candidate(
        value=default if resolved else None,
        value_type=type(default).__name__ if resolved else "unknown",
        mechanism=model.Mechanism.CODE_DEFAULT.value,
        source=site,
        rank=0,
        wins=wins,
        profile=item.profile,
        note=_default_note(row, resolved, wins),
    )
    state = model.ParamState.EFFECTIVE.value if wins else model.ParamState.OTHER.value
    if argv_text is None:
        builder.set_param(param_id, state=state, candidates=[code_row])
        return
    name = ids.parse_param(param_id).name
    where = _argv_site(builder, item, name)
    if where is None:
        builder.set_param(param_id, state=state, candidates=[code_row])
        builder.unresolved(
            model.Unresolved(
                want=model.Want.PARAM_VALUE.value,
                owner=param_id,
                expr=f"-p {name}:={argv_text}",
                reason="the command line of this process sets it and states no file to cite",
                evidence=[code_row.source],
            )
        )
        return
    value = ros_args.parse_value(argv_text)
    builder.set_param(
        param_id,
        state=model.ParamState.EFFECTIVE.value,
        candidates=[
            model.Candidate(
                value=value,
                value_type=type(value).__name__,
                mechanism=model.Mechanism.ROS_ARGS_P.value,
                source=where,
                rank=0,
                wins=True,
                profile=item.profile,
            ),
            replace(
                code_row,
                rank=1,
                wins=False,
                shadow_kind=_shadow_of(value, default) if resolved else None,
                shadowed_by=0,
            ),
        ],
    )


def _shadow_of(winner: Any, loser: Any) -> str:
    if type(winner) is not type(loser):
        return model.ShadowKind.TYPE_CHANGED.value
    if winner == loser:
        return model.ShadowKind.RESTATED_IDENTICAL.value
    return model.ShadowKind.VALUE_CHANGED.value


def _argv_site(builder: model.AtlasBuilder, item: Instance, name: str) -> model.Source | None:
    """Where this instance's command line was written, for an argv candidate to cite."""
    entity = builder.atlas.entities.get(item.node_id)
    if entity is None:
        return None
    for fact in sorted(entity.facts, key=model.LayerFact.sort_key):
        if fact.layer != DECLARED or fact.profile != item.profile or not fact.evidence:
            continue
        where = fact.evidence[0]
        locator = f"{where.locator} -p {name}" if where.locator else f"-p {name}"
        return replace(where, locator=locator)
    return None


def _default_note(row: dict[str, Any], resolved: bool, wins: bool) -> str:
    if wins:
        return ""
    if resolved:
        return "the command line of this process sets this parameter, so this default loses"
    return str(row.get("default_state") or "no static default")


def _lookup_of(row: dict[str, Any]) -> str:
    """The pluginlib lookup name of a suffix only declaration, either spelling."""
    return str(row.get("lookup") or row.get("lookup_name") or "")


def _bind_plugin_parameter(
    builder: model.AtlasBuilder, carrier: Carrier, row: dict[str, Any], profile: str
) -> None:
    """A suffix only declaration belongs to the plugin, not to a node. The full name needs
    the plugin instance from the yaml, which is the drift engine's work."""
    suffix = str(row.get("name", row.get("suffix")) or "")
    lookup = _lookup_of(row)
    base = str(row.get("base_class") or "")
    if not base or not lookup or not suffix:
        return
    try:
        pparam_id = ids.build_pparam(base, lookup, ids.normalise_param_name(suffix))
        plugin_id = ids.build_plugin(base, lookup)
    except ContractError:
        return
    if plugin_id not in builder.atlas.entities:
        return
    values: dict[str, Any] = {"declared_in_code": True}
    default, resolved = _default(row)
    if resolved:
        values["default"] = default
    builder.fact(
        pparam_id,
        CODED,
        profile=profile,
        values=values,
        confidence=model.Confidence.RESOLVED.value,
        evidence=_source(carrier, row),
    )
    entity = builder.atlas.entities[pparam_id]
    if not any(candidate.profile == profile for candidate in entity.candidates):
        # The full name needs the plugin instance from the yaml, so nothing wins here.
        builder.set_param(
            pparam_id,
            state=model.ParamState.OTHER.value,
            candidates=[
                model.Candidate(
                    value=default if resolved else None,
                    value_type=type(default).__name__ if resolved else "unknown",
                    mechanism=model.Mechanism.CODE_DEFAULT.value,
                    source=_source(carrier, row)[0],
                    rank=0,
                    wins=False,
                    profile=profile,
                    note="suffix only, the prefix is the plugin instance from the yaml",
                )
            ],
        )
    _site_edge(
        builder,
        model.EdgeKind.DECLARES_PARAM.value,
        plugin_id,
        pparam_id,
        ids.edge_key(profile=profile, plugin=plugin_id, name=suffix),
        profile=profile,
        values={"suffix": suffix},
        evidence=_source(carrier, row),
    )


def known_name_contradictions(report: JoinReport) -> list[str]:
    """Rows of the committed name table the first party sources contradict."""
    out = []
    for pkg, executable, table_name, code_name in sorted(set(report.contradictions)):
        row = known_names.lookup(pkg, executable)
        if row is None:
            continue
        out.append(
            f"{pkg}/{executable}: the table states {table_name}, the source states {code_name}"
        )
    return out


__all__ = [
    "BUILDERS",
    "CODED",
    "COMPONENT_MACRO",
    "DECLARED",
    "ENDPOINT_EDGES",
    "MARKER",
    "SCRIPT_SUFFIX",
    "WANTS",
    "Bound",
    "Carrier",
    "Index",
    "Instance",
    "JoinReport",
    "apply_remaps",
    "carriers",
    "executables_of",
    "frame_id",
    "index_of",
    "instances",
    "join",
    "known_name_contradictions",
]
