"""The parameter candidate chain: who sets a parameter, in what order, and who wins.

This is the single most valuable fact in the product, so the rules are spelled out here and
implemented once.

  * Order is load bearing. A node's `parameters=` list is a precedence list, later entries
    win, and the winner is rank 0. Nothing here encodes a hard file over inline rule.
    Verified on humble: an inline dict listed after a parameters file wins, and a later
    file wins over an earlier one even when the earlier key is the more specific one.
  * Inside one file the last matching block wins, not the most specific one. Verified by
    running a node against a file holding both a `/**` key and an exact node key in both
    orders. Specificity is still recorded, because it names the mechanism. Wildcard
    matching is `ids.param_key_matches` only.
  * Nested mappings and dotted keys fold through `ids.normalise_param_name` before anything
    compares them, so `ground: {max_slope: 8.0}` and `ground.max_slope: 8.0` are one name.
  * A wildcard key expands one source line into one parameter per node. The locator stays the
    key itself, so one deletable line is one finding and not N.
  * Provenance never points at a temp path. A parameters file produced outside the tracked
    trees keeps its content, because the values are real, and terminates at the symbol that
    wrote it, recorded as `applied_by`.
  * A yaml block whose key matches no launched node in the profile is `owner_name_mismatch`
    and has no effective value. A plugin block that no selector list names is
    `inert_by_design`. Both cases exist on the reference stack and both are silently ignored
    by the stack itself today.
  * A parameters file assembled outside launch, by a launcher module that loads a base
    document, merges overlays and writes single keys in python, is followed key by key
    (`records.ParamPipeline`). The chain then carries the base file, each overlay and each
    python write as its own candidate with its own file and line, and the launcher symbol as
    `applied_by`. A pipeline the analyser could not follow to the end marks every parameter
    it feeds as unresolved instead of naming a winner.
  * A node can own child nodes, whose fully qualified name is `<child>/<child>` under the
    parent's namespace. Which executable does that is data in `resolve/known_names.py`, never
    a package name spelled here: without it the yaml blocks that configure them read as
    blocks that configure nothing.
  * A parameters file that a launch argument default names and the profile displaced is still
    a candidate, marked `not_read_in_profile`. It never wins and it never mints a parameter of
    its own: a key only the displaced file spells is not configured in this profile at all.
    The declarations of the files an entry point includes count as well, because an argument
    the entry point passes down displaces a default declared in the included file and not in
    the entry point. A default that carries a value rather than a file is the same case, with
    one more thing to hold: the delivering key has to read that argument and nothing else, or
    the default is put on a key it never reached.
  * A python write whose value folded but whose guard did not is a candidate carrying the
    guard text, and still a blind spot. Dropping it entirely hides the only line an engineer
    can edit, and letting it win names a value that is wrong on the other branch.

Nothing here restates the candidate chain anywhere else in the atlas. The chain already holds
the file, the locator, the mechanism and the profile of every write site, so a per site edge,
a fact evidence row and a fact copy of the parameter name would each be the same four fields
under a second name. See AGENTS section 11.

Only pyyaml is imported. Nothing in this module needs launch, launch_ros or rclpy: it reads
the records the launch walk produced, and the launch file text through the syntax reader in
`resolve/launch_exec.py`, which imports nothing optional either.
"""

from __future__ import annotations

import ast
import difflib
import functools
import re
import weakref
from collections.abc import Iterator, Mapping, Sequence
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

from rosio import ids, model
from rosio.collect import (
    BUILT_ONLY,
    CollectContext,
    entrypoints,
    plugins,
    register,
    same_name_in_package,
)
from rosio.errors import CollectorError, ContractError, UsageError
from rosio.resolve import (
    identity,
    launch_exec,
    parse_python,
    py_pipeline,
    records,
    rewritten_yaml,
    ros_args,
    substitutions,
    yaml_lines,
)
from rosio.resolve._launch_child import uncarried_reason
from rosio.workspace import Workspace

LAYER = model.Layer.DECLARED.value

NO_WALK = "no launch walk reached this pass, so no profile names a parameters file"

REFUSED_PARAMS_FILE = "params_file_unreadable"
"""Why a parameters file gave nothing. It is an attribute on the file and not a fact,
which is the one shape a refused entity takes, stated in AGENTS section 4."""

NOT_INSTALLED = "file_not_installed"
"""A parameters file the launch file reads from the package share, which the build never
installed while the package source holds it. The fix is in the build file, not the yaml."""

MISSING_FILE = "file_missing"


_REFUSAL_ORDER = (NOT_INSTALLED, BUILT_ONLY, MISSING_FILE, REFUSED_PARAMS_FILE)
"""Which refusal of one file wins when two readers state it differently, strongest first.
Two profiles reaching one file have to leave one refusal on it in either order."""

SELECTOR_KEYS = (
    "behavior_plugins",
    "controller_plugins",
    "dock_plugins",
    "edge_cost_functions",
    "filters",
    "goal_checker_plugins",
    "navigators",
    "observation_sources",
    "operations",
    "path_handler_plugins",
    "planner_plugins",
    "plugins",
    "polygons",
    "progress_checker_plugin",
    "progress_checker_plugins",
    "smoother_plugins",
    "waypoint_task_executor_plugin",
)
"""The yaml keys that select which sub blocks of a node are live. Data, cited as evidence:
the reference stack keeps two fully configured blocks deliberately out of these lists.

Both spellings of a key are held where both are in use, and the singular ones are not typos.
`progress_checker_plugin` is what `nav2_bringup/params/nav2_params.yaml` writes and
`progress_checker_plugins` is what `nav2_bringup/params/nav2_multirobot_params_all.yaml`
writes, both in one ROS 2 humble install, and `waypoint_task_executor_plugin` has only ever
been singular. Holding one spelling of a key in use reports the block the other spelling
selects as configured and not selected, which is the loudest wrong sentence this family can
produce. Checked against every `*_plugin` and `*_plugins` key of the parameter files ROS 2
humble installs: the two above and the seven plural keys here are the whole set, and
`solver_plugin` is deliberately absent because its value is a class name and not the name of
a block, so it selects nothing that this list gates.

ROS 2 jazzy adds three that do not end in `_plugins`: `operations` and `edge_cost_functions` on
`route_server`, and `dock_plugins` on `docking_server`. Found by walking every yaml file jazzy
installs for a list whose items name sibling blocks that carry a `plugin` key; without them
every route operation and every dock read as configured and not selected."""

SELECTOR_BASE_CLASS = {
    "behavior_plugins": "nav2_core::Behavior",
    "controller_plugins": "nav2_core::Controller",
    "dock_plugins": "opennav_docking_core::ChargingDock",
    "edge_cost_functions": "nav2_route::EdgeCostFunction",
    "filters": "nav2_costmap_2d::Layer",
    "goal_checker_plugins": "nav2_core::GoalChecker",
    "navigators": "nav2_core::NavigatorBase",
    "operations": "nav2_route::RouteOperation",
    "path_handler_plugins": "nav2_core::PathHandler",
    "planner_plugins": "nav2_core::GlobalPlanner",
    "plugins": "nav2_costmap_2d::Layer",
    "progress_checker_plugin": "nav2_core::ProgressChecker",
    "progress_checker_plugins": "nav2_core::ProgressChecker",
    "smoother_plugins": "nav2_core::Smoother",
    "waypoint_task_executor_plugin": "nav2_core::WaypointTaskExecutor",
}
"""Base class per selector key, so a plugin scoped parameter can be named before the code
layer has read a single pluginlib manifest. A selector with no entry mints no plugin id. Two
spellings of one key carry one base class, because the spelling is the document's and the
base class is the code's."""

PLUGIN_KEY = "plugin"

_FILE_KINDS = ("repo_file", "external_file", "ros_args_file", "rewritten_file")

_PRODUCED_KINDS = ("external_file", "rewritten_file")
"""The kinds a document assembled outside the yaml files can arrive as. A launch file that
wraps its own temp document in a RewrittenYaml hands over the same path under the other
kind, so a rule that names only one of them decomposes half the profile."""


@dataclass(frozen=True)
class CandidateRow:
    """One write site for one parameter, before it becomes an atlas candidate."""

    value: Any
    value_type: str
    mechanism: str
    site: records.Site
    order: int
    group: int = 0
    rank: int = 0
    wins: bool = False
    shadow_kind: str | None = None
    shadowed_by: int | None = None
    applied_by: str | None = None
    raw: str | None = None
    selector: str = ""
    specificity: int = 0
    block_order: int = 0
    key_order: int = 0
    guard: str = ""
    note: str = ""

    def order_key(self) -> tuple[int, int, int, int, str, str]:
        """Winner first: the latest group, then the latest source, then the later block in
        the file, then the later key inside the block, then the locator so two equal
        candidates still sort the same way.

        Inside one file the last matching key wins, not the most specific one. Verified by
        running a node against a file holding both a `/**` key and an exact node key in both
        orders. The key position is what tells two spellings of one parameter apart, since a
        dotted key and a nested mapping fold to the same name and the same locator."""
        return (
            -self.group,
            -self.order,
            -self.block_order,
            -self.key_order,
            self.site.path,
            self.site.locator,
        )

    def to_candidate(self, ws: Workspace, profile: str) -> model.Candidate:
        note = self.note
        if self.raw is not None and self.raw != self.value:
            note = ", ".join(part for part in (note, f"argv text {self.raw!r}") if part)
        return model.Candidate(
            value=ws.stated_value(self.value),
            value_type=self.value_type,
            mechanism=self.mechanism,
            source=ws.source(self.site),
            rank=self.rank,
            wins=self.wins,
            profile=profile,
            shadow_kind=self.shadow_kind,
            shadowed_by=self.shadowed_by,
            applied_by=self.applied_by,
            guard=self.guard,
            note=note,
        )


@dataclass(frozen=True)
class Blind:
    """One value on a parameter's path that rosio will not name.

    Either it could not be resolved, or the input resolves to no value at all, which is what
    `ros_args.SETS_NOTHING` says. A parameter carrying one of these never names a winner. The
    chain stays readable, so a reader sees every write site that was found and the expression
    that was not."""

    expr: str
    reason: str
    site: records.Site | None = None

    def key(self) -> tuple[str, str, str, str]:
        site = self.site
        return (
            self.expr,
            self.reason,
            site.path if site else "",
            site.locator if site else "",
        )


@dataclass(frozen=True)
class ParamCandidateRow:
    """One parameter of one owner, with its ordered chain.

    `owner_fqn` is empty only for a yaml block that matched no launched node. `param_id` is
    still formed from the block key in that case, because the key is a literal in the file,
    but no owner node entity is created and the parameter can never be read as live
    configuration.
    """

    name: str
    owner_fqn: str
    param_id: str | None
    state: str
    candidates: tuple[CandidateRow, ...]
    effective: Any = None
    value_type: str = ""
    block: str = ""
    plugin: str = ""
    selector_key: str = ""
    block_site: records.Site | None = None
    """Where the block key itself is written, for a row of a block no node owns. The
    candidates cite one key inside the block, and a reader told to delete the block from
    there deletes from two lines past its first line."""
    note: str = ""
    unresolved: tuple[Blind, ...] = ()

    @property
    def winner(self) -> CandidateRow | None:
        for candidate in self.candidates:
            if candidate.rank == 0:
                return candidate
        return None


_UNPLACEABLE = "the write site is not in any configured root"


def _cite(ws: Workspace, site: records.Site | None) -> model.Source | None:
    """Provenance for a site, or None when no configured root can place it."""
    if site is None:
        return None
    try:
        return ws.source(site)
    except ContractError:
        return None


def _placeable(ws: Workspace, path: str, memo: dict[str, bool]) -> bool:
    """Whether a write site names a file of this workspace that is there to be cited.

    A command label joined onto the working directory can land under a root and name
    nothing, and citing it would mint a file entity for a file that does not exist.
    """
    known = memo.get(path)
    if known is None:
        known = _readable(ws, path) and Path(ws.localise(path)).exists()
        memo[path] = known
    return known


def _without_unplaceable(
    row: ParamCandidateRow, ws: Workspace, memo: dict[str, bool]
) -> ParamCandidateRow:
    """Move every candidate whose write site no root can place onto the blind spot list.

    A `ros2 run` entry point whose executable is not in the workspace carries its label
    where a path belongs, and provenance cannot cite a label. The value costs its own
    candidate and keeps its text in an unresolved entry. Raising here instead ended the
    parameter pass for the profile with every value finite.
    """
    kept: list[CandidateRow] = []
    blinds = list(row.unresolved)
    for candidate in row.candidates:
        if _placeable(ws, candidate.site.path, memo):
            kept.append(candidate)
            continue
        blind = Blind(
            expr=candidate.raw if candidate.raw is not None else repr(candidate.value),
            reason=f"{_UNPLACEABLE}: {Path(candidate.site.path).name}",
        )
        if blind not in blinds:
            blinds.append(blind)
    if len(kept) == len(row.candidates):
        return row
    return replace(
        row,
        state=model.ParamState.OTHER.value,
        candidates=tuple(kept),
        effective=None,
        value_type="",
        unresolved=tuple(blinds),
    )


def _without_unwritable(row: ParamCandidateRow) -> ParamCandidateRow:
    """Move every candidate the atlas cannot hold onto the blind spot list.

    A `-p gain:=.inf` and a yaml `gain: .inf` both produce a non finite float, which json
    has no spelling for, so the value cannot be written. It costs its own candidate and
    keeps its raw text in an unresolved entry. Raising here instead ended the pass and took
    every other parameter of the profile with it.
    """
    kept: list[CandidateRow] = []
    blinds = list(row.unresolved)
    for candidate in row.candidates:
        reason = model.unrepresentable(candidate.value)
        if reason is None:
            kept.append(candidate)
            continue
        blind = Blind(
            expr=candidate.raw if candidate.raw is not None else repr(candidate.value),
            reason=reason,
            site=candidate.site,
        )
        if blind not in blinds:
            blinds.append(blind)
    effective = model.unrepresentable(row.effective)
    if effective is None and len(kept) == len(row.candidates):
        return row
    if effective is not None and not blinds:
        blinds.append(Blind(expr=repr(row.effective), reason=effective))
    return replace(
        row,
        state=model.ParamState.OTHER.value,
        candidates=tuple(kept),
        effective=None if effective is not None else row.effective,
        value_type="" if effective is not None else row.value_type,
        unresolved=tuple(blinds),
    )


@dataclass(frozen=True)
class _Input:
    name: str
    value: Any
    site: records.Site
    mechanism: str
    order: int
    selector: str = ""
    specificity: int = 0
    block_order: int = 0
    key_order: int = 0
    raw: str | None = None
    applied_by: str | None = None
    guard: str = ""
    shadow_kind: str | None = None
    note: str = ""
    blind: Blind | None = None
    blind_only: bool = False
    """A site that has a blind spot and no value: it adds no candidate and never wins."""
    reads_arg: str = ""
    """The launch argument an inline dict value reads, where the source spells one."""
    delivered_at: records.Site | None = None
    """The site that delivered a value written one file up, when the two differ."""


@dataclass(frozen=True)
class _Step:
    """One item of a node's parameter inputs after the pipelines are expanded.

    Either a parameter source, or one python write of a pipeline, which needs the owner name
    before it can say which parameter it writes."""

    order: int
    source: records.ParamSource | None = None
    override: records.ParamOverride | None = None
    pipeline: records.ParamPipeline | None = None
    parent: records.ParamSource | None = None
    blind: Blind | None = None


# ---------------------------------------------------------------------------
# the chain


def owner_name(node: records.ResolvedNode) -> str:
    """The name a parameter file has to match.

    A missing `name=` is not a missing node: the runtime name then comes from the
    executable, which is exactly how the reference stack launches controller_server and
    collision_monitor. The name is not always the executable, so the committed table is
    asked first: `nav2_map_server/map_saver_server` runs as `map_saver`, and reading the
    executable there turns the one block that does configure it into a mismatch.
    """
    if any(field in ("name", "namespace") for field, _expr in node.command_blinds):
        raise ContractError("the node name depends on a command substitution")
    stated, _sentinel = identity.stated_name(node)
    known = identity.known_name(node.pkg, node.executable) if not stated else None
    base = stated or known or node.executable or node.plugin_class or ""
    if not base:
        raise ContractError("a resolved node needs a name, an executable or a plugin class")
    namespace, _stated = identity.stated_namespace(node)
    return ids.normalise_ros_name(base, namespace or identity.ROOT)


def owner_node_id(node: records.ResolvedNode) -> str:
    """Node id of the owner, decided by `resolve.identity` and nowhere else.

    Deciding it here as well is what gave one unnamed node two ids, one per collector, and
    left every parameter of that node hanging off an owner that exists in no layer.
    """
    return identity.node_id(node)


def child_nodes(node: records.ResolvedNode) -> list[records.ResolvedNode]:
    """The child node instances this executable creates at runtime, from the table.

    Nothing about any particular package is spelled here. The table row is the evidence, and
    an executable with no row has no children, so a stack that uses none of them is unchanged.

    A child reads the same parameter files as its parent, in the same order. An inline dict
    is the one input that does not always reach it: launch writes an inline dict into a file
    keyed on the parent's fully specified name, and only falls back to the `/**` key when
    launch stated no name, so the dict reaches the child in exactly that case.
    """
    table = identity.known_table()
    accessor = getattr(table, "child_nodes", None) if table is not None else None
    if accessor is None or not node.pkg or not (node.executable or node.plugin_class):
        return []
    try:
        names = accessor(node.pkg, node.executable, node.plugin_class or "")
    except Exception:
        return []
    parent = node.namespace or "/"
    params = tuple(source for source in node.params if _reaches_child(source, node))
    out: list[records.ResolvedNode] = []
    for child in sorted({str(name) for name in names or ()}):
        if not child:
            continue
        out.append(
            records.ResolvedNode(
                pkg=node.pkg,
                executable=node.executable,
                namespace=ids.join_namespace(parent, child),
                form=node.form,
                name=child,
                name_source=model.NameSource.EXECUTABLE_DEFAULT.value,
                params=params,
                gates=node.gates,
                conditions=node.conditions,
                site=node.site,
                attrs={
                    "child_of": owner_node_id(node),
                    "created_by": node.pkg,
                },
            )
        )
    return out


def _reaches_child(source: records.ParamSource, node: records.ResolvedNode) -> bool:
    return source.kind != "inline_dict" or not node.name


def chain_for(
    walk: records.LaunchWalk,
    node: records.ResolvedNode,
    *,
    ws: Workspace,
    group: int = 0,
    pipelines: Sequence[records.ParamPipeline] = (),
    name: str = "",
) -> list[ParamCandidateRow]:
    """The ordered candidate chain of every parameter of one node, sorted by name.

    `name` follows the chain under another node name of the same process, see
    `alternate_owners`.
    """
    try:
        fqn = name or owner_name(node)
    except ContractError:
        if not node.command_blinds:
            raise
        fqn = ""
    inputs: list[_Input] = []
    notes: list[str] = []
    blinds: dict[str, list[Blind]] = {}
    steps = _plan(walk, node.params, pipelines, ws)
    merge_group = _merge_group([step.source for step in steps if step.source is not None])
    for step in steps:
        if step.source is not None:
            found, reason = _inputs_of(step.source, fqn, ws=ws, walk=walk, merge_group=merge_group)
            inputs.extend(_marked(found, step.pipeline, step.blind))
            if reason:
                notes.append(reason)
            continue
        found, blind, reason = _override_inputs(step, fqn, ws=ws, walk=walk)
        inputs.extend(found)
        if blind is not None:
            blinds.setdefault(blind[0], []).append(blind[1])
        if reason:
            notes.append(reason)
    inputs.extend(_displaced_inputs(walk, fqn, inputs=inputs, ws=ws))
    rows = _rows(
        inputs,
        fqn=fqn,
        owner_id=ids.build_node(name) if name else owner_node_id(node),
        group=group,
        blinds=blinds,
    )
    if notes:
        rows = [replace(row, note=", ".join([row.note, *notes]).strip(", ")) for row in rows]
    return rows


ALTERNATE_NOTE = (
    "launch names no node of this process, so only a node its source names so reads this block"
)


def alternate_owners(
    walk: records.LaunchWalk,
    node: records.ResolvedNode,
    *,
    ws: Workspace,
    pipelines: Sequence[records.ParamPipeline] = (),
    taken: frozenset[str] = frozenset(),
) -> list[str]:
    """Other names a process launch leaves unnamed may run nodes under, read off its files.

    Launch names neither the process nor its nodes and no table names the executable, so the
    name its chain is followed under is the executable, which is a guess. One executable can
    run several nodes, every one of them reads every file the launch site hands the process,
    and only the source says what they are called. A block of those files keyed on another
    name in the node's namespace, which no launched node of the profile owns, can be one of
    them, so its chain is followed under that name as well and the join decides.
    """
    decided = identity.identify(node)
    if (
        not ids.is_synthetic(decided.node_id)
        or decided.known_name
        or decided.name_source == model.NameSource.RUNTIME_RANDOMISED.value
        or not decided.namespace
        or not node.executable
        or node.plugin_class
        or node.command_blinds
    ):
        return []
    try:
        guessed = owner_name(node)
    except ContractError:
        return []
    out: set[str] = set()
    for step in _plan(walk, node.params, pipelines, ws):
        source = step.source
        if source is None or source.kind not in _FILE_KINDS:
            continue
        path = source.source_path if source.kind == "rewritten_file" else source.path
        document = _load(path) if path else None
        if document is None or not document.is_parameter_file():
            continue
        reshaped, _root_key = _reshape(source, document)
        for block in yaml_lines.param_blocks(reshaped):
            if _specificity(block.selector) < 2:
                continue
            try:
                fqn = ids.normalise_ros_name(
                    block.selector.strip("/").rpartition("/")[2], decided.namespace
                )
            except ContractError:
                continue
            if fqn not in taken and fqn != guessed and _selector_matches(block.selector, fqn):
                out.add(fqn)
    return sorted(out)


def alternate_chain(
    walk: records.LaunchWalk,
    node: records.ResolvedNode,
    fqn: str,
    *,
    ws: Workspace,
    group: int = 0,
    pipelines: Sequence[records.ParamPipeline] = (),
) -> list[ParamCandidateRow]:
    """The parameters a block keyed on `fqn` sets, chained as a node of that name reads them.

    Only a key some block of that name writes is kept: a key the wildcard blocks alone write
    is already in the chain of the process. No launched node is named so, so nothing wins and
    the state is the one of a block no node owns, with the owner stated, which is what the
    join reads when the source names the node.
    """
    guessed = owner_name(node)
    out: list[ParamCandidateRow] = []
    for row in chain_for(walk, node, ws=ws, group=group, pipelines=pipelines, name=fqn):
        own = [
            candidate.selector
            for candidate in row.candidates
            if _specificity(candidate.selector) > 1
            and not _selector_matches(candidate.selector, guessed)
        ]
        if not own:
            continue
        if row.state == model.ParamState.EFFECTIVE.value:
            row = replace(
                row,
                state=model.ParamState.OWNER_NAME_MISMATCH.value,
                effective=None,
                selector_key=own[0],
                candidates=tuple(
                    replace(candidate, note=_with_note(candidate.note, ALTERNATE_NOTE))
                    for candidate in row.candidates
                ),
            )
        out.append(row)
    return out


_DISPLACED_ORDER = -1
"""Below every real input of its own entry point, so a file this profile never read can
never rank first: the row is kept only for a name that entry point already sets."""


def _displaced_inputs(
    walk: records.LaunchWalk,
    fqn: str,
    *,
    inputs: Sequence[_Input],
    ws: Workspace,
) -> list[_Input]:
    """Every launch argument default this profile displaced, as a candidate that never wins.

    A default the argument overrode is still a write site an engineer finds when they read
    the launch file, and a chain that omits it says the default contributes nothing anywhere
    rather than nothing here. Each row is marked `not_read_in_profile`, and each is kept only
    for a name this profile already sets: a key that only a displaced file spells is not
    configured here at all, and minting it would invent configuration.

    The scopes are the entry point and every file it includes, because an argument the entry
    point passes down displaces a default the included file declares.
    """
    known = {item.name for item in inputs}
    seen: set[tuple[str, str, str]] = set()
    out: list[_Input] = []
    for declarations, supplied in _arg_scopes(walk, ws):
        rows = _displaced_files(declarations, supplied, fqn=fqn, known=known, ws=ws)
        rows.extend(_displaced_values(declarations, supplied, inputs=inputs, ws=ws))
        for row in rows:
            key = (row.site.path, row.site.locator, row.name)
            if key in seen:
                continue
            seen.add(key)
            out.append(row)
    return out


def _displaced_files(
    declarations: Sequence[records.ArgDeclaration],
    supplied: Mapping[str, str],
    *,
    fqn: str,
    known: set[str],
    ws: Workspace,
) -> list[_Input]:
    """The parameters file a displaced default names, one candidate per key it spells.

    Two arguments defaulting to one file are one write site. The yaml line is deletable once,
    so it is one candidate and not one per argument that named the file.
    """
    seen: set[tuple[str, str]] = set()
    out: list[_Input] = []
    for declaration in sorted(declarations, key=lambda item: item.name):
        path = _displaced_default(declaration, supplied)
        if path is None or not _readable(ws, path):
            continue
        document = _load(path)
        if document is None or not document.is_parameter_file():
            continue
        cited = ws.localise(path)
        note = f"the {declaration.name} argument displaced this file in this profile"
        for block in yaml_lines.param_blocks(document):
            if not _selector_matches(block.selector, fqn):
                continue
            for leaf in yaml_lines.leaves(block.values):
                if leaf.name not in known:
                    continue
                site = _cited_site(document, (*block.key_path, *leaf.key_path), cited, "", ws)
                if (site.path, site.locator) in seen:
                    continue
                seen.add((site.path, site.locator))
                out.append(
                    _Input(
                        name=leaf.name,
                        value=leaf.value,
                        site=site,
                        mechanism=model.Mechanism.LAUNCH_PARAMS_FILE.value,
                        order=_DISPLACED_ORDER,
                        selector=block.selector,
                        specificity=_specificity(block.selector),
                        block_order=block.order,
                        key_order=leaf.order,
                        shadow_kind=model.ShadowKind.NOT_READ_IN_PROFILE.value,
                        note=note,
                    )
                )
    return out


def _displaced_values(
    declarations: Sequence[records.ArgDeclaration],
    supplied: Mapping[str, str],
    *,
    inputs: Sequence[_Input],
    ws: Workspace,
) -> list[_Input]:
    """A displaced default that an inline dict of the declaring file carries into a parameter.

    Two things have to hold. The delivering key reads that argument and nothing else, which
    the source spells as `LaunchConfiguration(name)`, and the profile supplied exactly one
    displaced argument of that file with the value the key carries. Only then is the default
    what stands at that same site when nobody supplies the argument. A value two arguments
    share, a key the source does not spell as reading an argument, and a key whose value the
    launch file computed rather than passed through, all state nothing and are dropped rather
    than guessed: the value one of them carries is not the value the default would put there.
    """
    by_value: dict[str, list[records.ArgDeclaration]] = {}
    for declaration in declarations:
        text = supplied.get(declaration.name)
        if text is None or declaration.default is None or text == declaration.default:
            continue
        if declaration.site is None:
            continue
        by_value.setdefault(text, []).append(declaration)
    out: list[_Input] = []
    for item in sorted(inputs, key=lambda row: (row.name, row.order)):
        if item.mechanism != model.Mechanism.LAUNCH_INLINE_DICT.value:
            continue
        if not isinstance(item.value, str) or not item.reads_arg:
            continue
        found = by_value.get(item.value, ())
        if len(found) != 1:
            continue
        declaration = found[0]
        if declaration.name != item.reads_arg:
            continue
        site = declaration.site
        local = item.delivered_at or item.site
        if site is None or ws.localise(site.path) != ws.localise(local.path):
            continue
        out.append(
            _Input(
                name=item.name,
                value=declaration.default,
                site=site,
                mechanism=item.mechanism,
                order=_DISPLACED_ORDER,
                shadow_kind=model.ShadowKind.NOT_READ_IN_PROFILE.value,
                note=(f"the {declaration.name} argument displaced this default in this profile"),
            )
        )
    return out


def _displaced_default(
    declaration: records.ArgDeclaration,
    supplied: Mapping[str, str],
) -> str | None:
    """The default text of a declaration this profile overrode, or nothing."""
    default = declaration.default
    if not default or supplied.get(declaration.name, default) == default:
        return None
    return default


def _arg_scopes(
    walk: records.LaunchWalk,
    ws: Workspace,
) -> list[tuple[list[records.ArgDeclaration], dict[str, str]]]:
    """The argument declarations this profile can displace, with what it supplied to each.

    The entry point answers for itself and the executed walk already performed its defaults,
    so the text reader only fills in the line and a default the walk could not state. An
    included file answers for the arguments the include passed it, and its declarations are
    read from the text, because the walk keeps only the entry point's own.
    """
    out = [(_declarations(walk.entry_path, ws, walk.arg_declarations), dict(walk.args))]
    seen = {ws.localise(walk.entry_path)}
    for include in sorted(walk.includes, key=lambda item: (item.path, item.args)):
        if not include.resolved or not include.path or not _readable(ws, include.path):
            continue
        cited = ws.localise(include.path)
        if cited in seen:
            continue
        seen.add(cited)
        out.append((_declarations(include.path, ws), dict(include.args)))
    return out


def _declarations(
    path: str,
    ws: Workspace,
    executed: Sequence[records.ArgDeclaration] = (),
) -> list[records.ArgDeclaration]:
    """The `DeclareLaunchArgument` rows of one launch file, with a line and a default.

    The syntax reader in `resolve/launch_exec.py` is the one implementation that reads them.
    What it cannot state is a default that names a file through the package share, which is a
    workspace lookup and is finished here.
    """
    read = _read_declarations(path, ws)
    text = _arg_text(path, ws)
    rows = list(executed) if executed else list(read)
    sites = {item.name: item.site for item in read}
    return [_completed(item, text.get(item.name), sites.get(item.name)) for item in rows]


_bases: weakref.WeakKeyDictionary[Workspace, dict[str, frozenset[str]]] = (
    weakref.WeakKeyDictionary()
)


def _exported_bases(ws: Workspace) -> dict[str, frozenset[str]]:
    found = _bases.get(ws)
    if found is None:
        found = _bases[ws] = plugins.exported_bases(ws)
    return found


_declared: weakref.WeakKeyDictionary[Workspace, dict[tuple[str, int, int], tuple]] = (
    weakref.WeakKeyDictionary()
)
"""The syntax reader's declarations, once per file content and workspace, because every
parameter chain of every profile asks again for the same included files."""


def _read_declarations(path: str, ws: Workspace) -> tuple[records.ArgDeclaration, ...]:
    try:
        stat = Path(path).stat()
        key = (path, stat.st_size, stat.st_mtime_ns)
    except OSError:
        key = (path, -1, -1)
    held = _declared.setdefault(ws, {})
    if key not in held:
        try:
            held[key] = tuple(launch_exec.walk_ast(path, workspace=ws).arg_declarations)
        except Exception:
            held[key] = ()
    return held[key]


def _completed(
    declaration: records.ArgDeclaration,
    text: tuple[str | None, int | None] | None,
    site: records.Site | None,
) -> records.ArgDeclaration:
    """One declaration with the default the walk stated and the line the source states.

    The line cited is the line of the default itself, which is the line an engineer edits and
    is not always the line the call opens on.
    """
    default, line = text or (None, None)
    if declaration.default is not None:
        default = declaration.default
    placed = declaration.site or site
    if placed is not None and line is not None:
        placed = replace(placed, line_hint=line)
    return replace(declaration, default=default, site=placed)


# ---------------------------------------------------------------------------
# launch file text: the lines and the package share paths a walk does not state

_SHARE_CALLS = ("FindPackageShare", "get_package_share_directory")

_JOIN_CALLS = ("PathJoinSubstitution", "os.path.join", "path.join")

_ARG_CALL = "DeclareLaunchArgument"

_trees: dict[tuple[str, int, int], ast.Module | None] = {}
"""One parse per file content. The stamp is the size and the modification time, so a file
rewritten under the same path is read again rather than answered from the cache."""


def _tree(path: str) -> ast.Module | None:
    try:
        stat = Path(path).stat()
    except OSError:
        return None
    key = (path, stat.st_size, stat.st_mtime_ns)
    if key not in _trees:
        try:
            source = Path(path).read_text(encoding="utf-8", errors="replace")
            _trees[key] = parse_python(source, filename=path)
        except (OSError, SyntaxError, ValueError):
            _trees[key] = None
    return _trees[key]


def _arg_text(path: str, ws: Workspace) -> dict[str, tuple[str | None, int | None]]:
    """What the source says about each argument default: the path it names, and its line.

    A default that names a file through the package share is the one the syntax walk cannot
    state, because it is a workspace lookup rather than a substitution.
    """
    tree = _tree(path)
    if tree is None:
        return {}
    out: dict[str, tuple[str | None, int | None]] = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or _dotted(node.func) != _ARG_CALL:
            continue
        name = _literal(_call_arg(node, "name", 0))
        default = _call_arg(node, "default_value", 1)
        if name:
            out[name] = (
                _path_text(default, ws),
                None if default is None else default.lineno,
            )
    return out


def _path_text(node: ast.expr | None, ws: Workspace) -> str | None:
    """A path an argument default states, or nothing when any part of it is not literal."""
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    if isinstance(node, (ast.List, ast.Tuple)):
        return _path_join(node.elts, ws)
    if isinstance(node, ast.Call):
        name = _dotted(node.func)
        if name in _SHARE_CALLS:
            package = _literal(_call_arg(node, "package", 0)) or _literal(
                _call_arg(node, "package_name", 0)
            )
            return ws.share(package) if package else None
        if name in _JOIN_CALLS:
            if len(node.args) == 1 and isinstance(node.args[0], (ast.List, ast.Tuple)):
                return _path_join(node.args[0].elts, ws)
            return _path_join(node.args, ws)
        if name == "TextSubstitution":
            return _literal(_call_arg(node, "text", 0))
    return None


def _path_join(parts: Sequence[ast.expr], ws: Workspace) -> str | None:
    resolved: list[str] = []
    for part in parts:
        text = _path_text(part, ws)
        if text is None:
            return None
        resolved.append(text)
    if not resolved:
        return None
    return str(Path(resolved[0]).joinpath(*resolved[1:]))


def _dotted(node: ast.expr) -> str:
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        return f"{_dotted(node.value)}.{node.attr}".lstrip(".")
    return ""


def _literal(node: ast.expr | None) -> str:
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return ""


def _call_arg(call: ast.Call, name: str, index: int | None) -> ast.expr | None:
    for keyword in call.keywords:
        if keyword.arg == name:
            return keyword.value
    if index is not None and len(call.args) > index:
        return call.args[index]
    return None


def _inline_dict(
    site: records.Site, values: Sequence[tuple[str, Any]], ws: Workspace
) -> ast.Dict | None:
    """The dict literal an inline parameters source was written as.

    The launch action states the attribute the dict came from and not a line, so the dict is
    found again by its own key set, which is what tells two dicts of one launch file apart. A
    key set that matches more than one dict states nothing rather than the wrong dict.
    """
    if not site.path.endswith(".py"):
        return None
    cited, _root = ws.cite(site.path, site.root)
    tree = _tree(cited)
    if tree is None:
        return None
    wanted = tuple(sorted(str(name) for name, _value in values))
    found = [node for node in ast.walk(tree) if _dict_keys(node) == wanted]
    if site.line_hint is not None:
        # The site names the line of the action the dict sits in, so a dict inside that call
        # is the one, and two equal dicts of one file are told apart by it.
        inside = {
            id(node): node
            for call in ast.walk(tree)
            if isinstance(call, ast.Call) and call.lineno == site.line_hint
            for node in ast.walk(call)
            if node in found
        }
        found = (
            list(inside.values())
            or [node for node in found if node.lineno == site.line_hint]
            or found
        )
    return found[0] if len(found) == 1 else None


def _inline_lines(
    site: records.Site, values: Sequence[tuple[str, Any]], ws: Workspace
) -> dict[str, int]:
    """Where each key of an inline parameters dict is written in the launch file.

    The chain stopped at the file, or at the line of the action, while each key is written on
    a line an engineer can open.
    """
    found = _inline_dict(site, values, ws)
    if found is None:
        return {}
    lines: dict[str, int] = {}
    _dict_lines(found, (), lines)
    return lines


def _inline_configs(
    site: records.Site, values: Sequence[tuple[str, Any]], ws: Workspace
) -> dict[str, str]:
    """The launch argument each key of an inline dict reads, where it reads one directly.

    A key whose value is anything else reads no argument, so an argument default can never be
    attached to it. Two keys of one dict carrying one value is the common shape, and matching
    them on the value alone puts a default on the key that never had one.
    """
    found = _inline_dict(site, values, ws)
    if found is None:
        return {}
    out: dict[str, str] = {}
    _dict_configs(found, (), out)
    return out


def _dict_keys(node: ast.AST) -> tuple[str, ...] | None:
    if not isinstance(node, ast.Dict) or not node.keys:
        return None
    names = [_literal(key) for key in node.keys if key is not None]
    if len(names) != len(node.keys) or not all(names):
        return None
    return tuple(sorted(names))


def _dict_lines(node: ast.Dict, prefix: tuple[str, ...], out: dict[str, int]) -> None:
    for key, value in zip(node.keys, node.values, strict=False):
        if key is None:
            continue
        name = _literal(key)
        if not name:
            continue
        path = (*prefix, name)
        out[".".join(path)] = key.lineno
        if isinstance(value, ast.Dict):
            _dict_lines(value, path, out)


def _dict_configs(node: ast.Dict, prefix: tuple[str, ...], out: dict[str, str]) -> None:
    for key, value in zip(node.keys, node.values, strict=False):
        if key is None:
            continue
        name = _literal(key)
        if not name:
            continue
        path = (*prefix, name)
        if isinstance(value, ast.Dict):
            _dict_configs(value, path, out)
            continue
        argument = _config_name(value)
        if argument:
            out[".".join(path)] = argument


def _config_name(node: ast.expr) -> str:
    """The argument name a `LaunchConfiguration(...)` value reads."""
    if not isinstance(node, ast.Call):
        return ""
    if _dotted(node.func).rsplit(".", 1)[-1] != "LaunchConfiguration":
        return ""
    return _literal(_call_arg(node, "variable_name", 0))


# ---------------------------------------------------------------------------
# parameters assembled outside launch


def _plan(
    walk: records.LaunchWalk,
    sources: Sequence[records.ParamSource],
    pipelines: Sequence[records.ParamPipeline],
    ws: Workspace | None = None,
) -> list[_Step]:
    """The node's parameter inputs with every pipeline expanded into what it really did.

    The temp file the launcher handed to launch is replaced by the documents it was built
    from, in the order they were applied, so nothing downstream ever sees the temp path.
    """
    steps: list[_Step] = []
    order = 0
    for source in sorted(sources, key=lambda item: item.order):
        found = _pipelines_for(walk, source, pipelines, ws)
        if len(found) != 1:
            steps.append(
                _Step(
                    order=order,
                    source=replace(source, order=order),
                    blind=_ambiguous(found),
                )
            )
            order += 1
            continue
        pipeline = found[0]
        for produced in _pipeline_sources(source, pipeline, ws):
            steps.append(
                _Step(
                    order=order,
                    source=replace(produced, order=order),
                    pipeline=pipeline,
                    parent=source,
                )
            )
            order += 1
        for override in pipeline.overrides:
            steps.append(_Step(order=order, override=override, pipeline=pipeline, parent=source))
            order += 1
    return steps


def _pipelines_for(
    walk: records.LaunchWalk,
    source: records.ParamSource,
    pipelines: Sequence[records.ParamPipeline],
    ws: Workspace | None = None,
) -> list[records.ParamPipeline]:
    """Every pipeline that could have produced this parameter source.

    Two questions, both answered from the records and neither guessed. Which entry point the
    pipeline feeds, which is the binding the entry point scraper wrote on it, so a profile is
    never handed values from a command it does not run. And which of that entry point's
    parameter sources came out of it: the source path is the pipeline's own output token, or
    one of the documents the pipeline assembled, which is what a configuration file has to
    name because the real value is a temp path that exists in no repository.

    More than one answer is not a winner. Two launcher functions can build one argument name
    from one base document for two different entry points, and picking either would be a
    guess.
    """
    path = source.source_path if source.kind == "rewritten_file" else source.path
    if not path or source.kind not in _FILE_KINDS:
        return []
    return [
        pipeline
        for pipeline in sorted(pipelines, key=_pipeline_key)
        if _targets(pipeline, walk) and _feeds(pipeline, path, source.kind, walk, ws)
    ]


def _targets(pipeline: records.ParamPipeline, walk: records.LaunchWalk) -> bool:
    """Whether the entry point the pipeline was bound to is the one this walk ran.

    The binding is the command the launcher builds, so the package and the target decide it.
    Two pipelines of one module differ in exactly that: one runs the upstream
    navigation_launch.py and the other a repository launch file that overlays it, while
    both write the same argument name from the same base document. A pipeline bound to no
    entry point is not filtered here, and an ambiguity that survives is recorded rather than
    resolved.
    """
    package, target = _binding(pipeline)
    if not target:
        return True
    if py_pipeline.launch_argument(pipeline):
        # the invocation is a launch file of this walk, either the entry point or an include
        names = {Path(walk.entry_path).name}
        names.update(Path(item.path).name for item in walk.includes)
        return Path(target).name in names
    if walk.entry_pkg and package and package != walk.entry_pkg:
        return False
    return Path(walk.entry_path).name == Path(target).name


def _binding(pipeline: records.ParamPipeline) -> tuple[str, str]:
    """The package and the launch file or executable the pipeline feeds, empty when unbound.

    The analyser states it as the target of the command it read. A pipeline declared by hand
    or recovered from a scraped invocation carries the command instead, and the same two
    words are read out of it.
    """
    target = py_pipeline.sink_target(pipeline)
    if not target:
        words = entrypoints.entrypoint_of(pipeline).split()
        target = " ".join(words[2:4]) if len(words) >= 4 else ""
    parts = target.split()
    if len(parts) >= 2:
        return parts[0], parts[1]
    return "", parts[0] if parts else ""


def _feeds(
    pipeline: records.ParamPipeline,
    path: str,
    kind: str,
    walk: records.LaunchWalk,
    ws: Workspace | None,
) -> bool:
    """Whether this parameter source is the document the pipeline handed the entry point.

    The argument the pipeline writes into has to carry it, either as the temp path the
    launcher really wrote or as a document of the pipeline, which is what a configuration
    file has to name in its place. A file the entry point reads on its own, with no argument
    naming it, is not fed by the pipeline even when the pipeline reads the same file.

    A launch file that assembled the document for itself hands over a temp path directly, so
    there is no argument value to compare and the temp name is random. The suffix the dump
    site stated is what identifies it, and it is what tells two documents of one launch file
    apart.
    """
    if py_pipeline.launch_argument(pipeline):
        suffix = py_pipeline.temp_suffix(pipeline)
        return bool(suffix) and kind in _PRODUCED_KINDS and path.endswith(suffix)
    named = [value for name, value in walk.args if name == _sink_argument(pipeline)]
    if path in named:
        return True
    tokens = _pipeline_tokens(pipeline, ws)
    if path not in tokens:
        return False
    return path == pipeline.output_token or any(value in tokens for value in named)


def _sink_argument(pipeline: records.ParamPipeline) -> str:
    """The launch argument the produced file is handed to."""
    return py_pipeline.sink_argument(pipeline) or pipeline.output_token


def _pipeline_tokens(pipeline: records.ParamPipeline, ws: Workspace | None) -> set[str]:
    """Every path this pipeline can be named by: its output token and its own documents."""
    out = {pipeline.output_token} if pipeline.output_token else set()
    for raw in (pipeline.base, *pipeline.overlays):
        if raw:
            out.add(_pipeline_path(raw, ws))
    return out


def _ambiguous(found: Sequence[records.ParamPipeline]) -> Blind | None:
    if len(found) < 2:
        return None
    return Blind(
        expr=", ".join(sorted(pipeline.applied_by for pipeline in found)),
        reason="pipeline_ambiguous",
        site=found[0].site,
    )


def _pipeline_key(pipeline: records.ParamPipeline) -> tuple[str, str, str]:
    return (pipeline.output_token, pipeline.base or "", pipeline.applied_by)


def _pipeline_symbol(pipeline: records.ParamPipeline, ws: Workspace | None) -> str | None:
    """The launcher symbol as an id, so `applied_by` is a thing a reader can look up.

    The analyser states the symbol as it is spelled in the module, which is not an id. The
    file it was read from is what turns it into one.
    """
    text = (pipeline.applied_by or "").strip()
    if not text:
        return None
    try:
        ids.parse(text)
        return text
    except ContractError:
        pass
    site = pipeline.site
    if ws is None or site is None or not site.path:
        return None
    try:
        package, relative = ws.rel(site.path)
    except ContractError:
        return None
    parts = text.split(".")
    if parts and parts[0] == Path(site.path).stem:
        parts = parts[1:]
    return ids.build_sym(package or ids.REPO_PKG, relative, ".".join(parts) or text)


def _pipeline_path(path: str, ws: Workspace | None) -> str:
    """A document path the analyser stated relative to the repository root."""
    if not path or ws is None or Path(path).is_absolute():
        return path
    candidate = Path(ws.repo_root) / path
    return str(candidate) if candidate.exists() else path


def _pipeline_sources(
    source: records.ParamSource,
    pipeline: records.ParamPipeline,
    ws: Workspace | None = None,
) -> list[records.ParamSource]:
    """The base document and every overlay, as sources that cite the real files.

    A wrapper the launch file put around the temp path (a `RewrittenYaml` root key and its
    rewrites) applies to the assembled document, so it stays on every one of these.
    """
    symbol = _pipeline_symbol(pipeline, ws)
    if not pipeline.base:
        return [replace(source, produced_by=source.produced_by or symbol)]
    rewritten = source.kind == "rewritten_file"
    out: list[records.ParamSource] = []
    for index, raw in enumerate((pipeline.base, *pipeline.overlays)):
        path = _pipeline_path(raw, ws)
        mechanism = (
            model.Mechanism.YAML_BASE.value
            if index == 0
            else model.Mechanism.YAML_OVERLAY_MERGE.value
        )
        note = _with_note(source.note if rewritten else "", f"mechanism={mechanism}")
        if rewritten:
            out.append(
                replace(
                    source,
                    source_path=path,
                    path=None,
                    produced_by=symbol,
                    note=note,
                )
            )
            continue
        out.append(
            records.ParamSource(
                order=source.order,
                kind="repo_file",
                path=path,
                produced_by=symbol,
                site=pipeline.site,
                note=note,
            )
        )
    return out


def _override_inputs(
    step: _Step,
    fqn: str,
    *,
    ws: Workspace,
    walk: records.LaunchWalk,
) -> tuple[list[_Input], tuple[str, Blind] | None, str]:
    """One python write, as an input of this owner, or nothing when it writes elsewhere."""
    override = step.override
    pipeline = step.pipeline
    if override is None or pipeline is None:
        return [], None, ""
    selector, tail = _override_target(override.key_path)
    if not tail:
        return [], None, f"a python write at {'.'.join(override.key_path)} names no parameter"
    selector = _wrapped_selector(selector, step.parent)
    if not _selector_matches(selector, fqn):
        return [], None, ""
    name = ids.normalise_param_name(list(tail))
    site = override.site or pipeline.site or ws.site(walk.entry_path)
    applied_by = _pipeline_symbol(pipeline, ws)
    blind = None
    if not override.resolved:
        blind = (
            name,
            Blind(
                expr=override.expr or name,
                reason=entrypoints.PARAM_WRITE_UNRESOLVED,
                site=site,
            ),
        )
        if override.value is None:
            return [], blind, ""
    out = [
        _Input(
            name=name,
            value=override.value,
            site=site,
            mechanism=model.Mechanism.PYTHON_OVERRIDE.value,
            order=step.order,
            applied_by=applied_by,
            guard=override.expr if blind is not None else "",
            blind=_pipeline_blind(pipeline),
        )
    ]
    rewrite = _rewrite_after(step.parent, tail[-1], site)
    if rewrite is not None:
        out.append(replace(out[0], **rewrite))
    return out, blind, ""


def _override_target(key_path: Sequence[str]) -> tuple[str, tuple[str, ...]]:
    """The block key a write lands in, and the key path below `ros__parameters`."""
    parts = [str(part) for part in key_path]
    if yaml_lines.PARAM_KEY not in parts:
        return "", ()
    cut = parts.index(yaml_lines.PARAM_KEY)
    return yaml_lines.selector_of(parts[:cut]), tuple(parts[cut + 1 :])


def _wrapped_selector(selector: str, parent: records.ParamSource | None) -> str:
    """The block key as the node sees it, under a RewrittenYaml root key when there is one."""
    if (
        parent is None
        or parent.kind != "rewritten_file"
        or not parent.root_key
        or _mark_inside(parent.root_key)
    ):
        return selector
    return f"{parent.root_key}/{selector}" if selector else parent.root_key


def _rewrite_after(
    parent: records.ParamSource | None,
    key: str,
    fallback: records.Site,
) -> dict[str, Any] | None:
    """What a RewrittenYaml does to a key the launcher had just written.

    The rewrite runs at launch time, after the launcher wrote its file, so it wins. Recording
    it as its own later candidate keeps the python write visible as the shadowed one.
    """
    if parent is None or parent.kind != "rewritten_file":
        return None
    table = dict(parent.rewrites)
    if key not in table:
        return None
    mark = _mark_inside(table[key])
    return {
        "value": None
        if mark
        else rewritten_yaml.convert_value(
            table[key], convert_types="convert_types" in (parent.note or "")
        ),
        "mechanism": model.Mechanism.LAUNCH_REWRITTEN_YAML.value,
        "site": parent.site or fallback,
        "block_order": 1,
        "applied_by": None,
        "note": "a RewrittenYaml rewrite of this key runs after the python write",
        "blind": Blind(expr=mark[0], reason=mark[1], site=parent.site or fallback)
        if mark
        else None,
    }


def _pipeline_blind(pipeline: records.ParamPipeline) -> Blind | None:
    """Nothing when the analyser followed the pipeline to the end, a blind spot otherwise."""
    if pipeline.complete and pipeline.base:
        return None
    expr = ", ".join(pipeline.notes) if pipeline.notes else pipeline.output_token
    reason = "pipeline_incomplete" if pipeline.complete else "pipeline_not_followed"
    if not pipeline.base:
        reason = "pipeline_base_unknown"
    return Blind(expr=expr, reason=reason, site=pipeline.site)


def _marked(
    inputs: Sequence[_Input],
    pipeline: records.ParamPipeline | None,
    blind: Blind | None = None,
) -> list[_Input]:
    blind = blind or (_pipeline_blind(pipeline) if pipeline is not None else None)
    if blind is None:
        return list(inputs)
    return [replace(item, blind=blind) for item in inputs]


def _with_note(note: str, addition: str) -> str:
    return ", ".join(part for part in (note, addition) if part)


def orphan_rows(
    walk: records.LaunchWalk,
    *,
    ws: Workspace,
    group: int = 0,
    pipelines: Sequence[records.ParamPipeline] = (),
    alternates: Sequence[str] = (),
) -> list[ParamCandidateRow]:
    """Parameter blocks in the walk's files that match no node the walk launched.

    A wildcard block is never an orphan: it applies to whatever node the file was handed
    to. A plain node name that nothing launches is the silent case this exists for. A block
    keyed on one of `alternates` is not either, `alternate_chain` follows it.

    A node whose runtime name cannot be formed (an ExecuteProcess whose executable is a
    script file name) makes the question unanswerable rather than answering it no, so the
    blocks of that walk become `unresolved_owner` instead of `owner_name_mismatch`. Letting
    the name rule raise here emptied the whole profile.
    """
    owners = []
    unnamed = False
    for node in walk.nodes:
        for owner in (node, *child_nodes(node)):
            try:
                owners.append(owner_name(owner))
            except ContractError:
                unnamed = True
    owners.extend(alternates)
    state = (
        model.ParamState.UNRESOLVED_OWNER.value
        if unnamed
        else model.ParamState.OWNER_NAME_MISMATCH.value
    )
    out: list[ParamCandidateRow] = []
    for path, source in sorted(_walk_files(walk, pipelines, ws), key=lambda item: item[0]):
        document = _load(path)
        if document is None or not document.is_parameter_file():
            continue
        reshaped, root_key = _reshape(source, document)
        for block in yaml_lines.param_blocks(reshaped):
            if _specificity(block.selector) < 3:
                continue
            if any(_selector_matches(block.selector, owner) for owner in owners):
                continue
            out.extend(
                _orphan_block_rows(
                    document,
                    block,
                    path=path,
                    ws=ws,
                    walk=walk,
                    source=source,
                    group=group,
                    root_key=root_key,
                    state=state,
                )
            )
    return sorted(out, key=lambda row: (row.selector_key, row.name))


def _orphan_block_rows(
    document: yaml_lines.Document,
    block: yaml_lines.ParamBlock,
    *,
    path: str,
    ws: Workspace,
    walk: records.LaunchWalk,
    source: records.ParamSource,
    group: int,
    root_key: str = "",
    state: str = model.ParamState.OWNER_NAME_MISMATCH.value,
) -> list[ParamCandidateRow]:
    fqn = _selector_fqn(block.selector)
    note = (
        "no launched node matches this block key"
        if state == model.ParamState.OWNER_NAME_MISMATCH.value
        else "a node of this walk has no name that a block key can be compared against"
    )
    cited, locator_prefix = _provenance(source, path, ws=ws, walk=walk)
    document = _lines_of(document, cited, path, ws)
    block_site = _cited_site(
        document, _on_disk_path(block.key_path[:-1], root_key), cited, locator_prefix, ws
    )
    rows: list[ParamCandidateRow] = []
    for leaf in yaml_lines.leaves(block.values):
        key_path = _on_disk_path((*block.key_path, *leaf.key_path), root_key)
        site = _cited_site(document, key_path, cited, locator_prefix, ws)
        row = CandidateRow(
            value=leaf.value,
            value_type=yaml_lines.type_name(leaf.value),
            mechanism=_mechanism(source, selector=block.selector, merge_rank=None),
            site=site,
            order=source.order,
            group=group,
            rank=0,
            wins=False,
            selector=block.selector,
            specificity=_specificity(block.selector),
            block_order=block.order,
            key_order=leaf.order,
        )
        rows.append(
            ParamCandidateRow(
                name=leaf.name,
                owner_fqn="",
                param_id=(ids.build_param(ids.build_node(fqn), leaf.name) if fqn else None),
                state=state,
                candidates=(row,),
                effective=None,
                value_type=yaml_lines.type_name(leaf.value),
                selector_key=block.selector,
                block_site=block_site,
                note=note,
            )
        )
    return rows


def _rows(
    inputs: Sequence[_Input],
    *,
    fqn: str,
    owner_id: str,
    group: int,
    blinds: Mapping[str, Sequence[Blind]] | None = None,
) -> list[ParamCandidateRow]:
    grouped: dict[str, list[_Input]] = {}
    for item in inputs:
        grouped.setdefault(item.name, []).append(item)
    for name in blinds or {}:
        grouped.setdefault(name, [])
    blind_rows = {
        name: tuple(sorted(_unique_blinds(name, items, blinds), key=Blind.key))
        for name, items in grouped.items()
    }
    winners: dict[str, Any] = {}
    ranked: dict[str, list[CandidateRow]] = {}
    for name, items in grouped.items():
        rows = _rank([_candidate_row(item, group) for item in items if not item.blind_only])
        ranked[name] = rows
        if rows and not blind_rows[name]:
            winners[name] = rows[0].value
    selected = _selected_instances(winners)
    out: list[ParamCandidateRow] = []
    for name in sorted(ranked):
        rows = ranked[name]
        blind = blind_rows[name]
        block, selector_key = _governing_block(name, winners, selected)
        inert = bool(block) and not blind
        state = _row_state(inert=inert, blind=bool(blind))
        rows = _classify(rows, inert=inert, wins=not inert and not blind)
        winner = rows[0] if rows else None
        effective = winner.value if winner is not None and state == "effective" else None
        out.append(
            ParamCandidateRow(
                name=name,
                owner_fqn=fqn,
                param_id=ids.build_param(owner_id, name),
                state=state,
                candidates=tuple(rows),
                effective=effective,
                value_type=winner.value_type if winner is not None else "",
                block=block,
                plugin=str(winners.get(f"{block}.{PLUGIN_KEY}", "")) if block else "",
                selector_key=selector_key,
                note="the block key is in no selector list" if inert else "",
                unresolved=blind,
            )
        )
    return out


def _unique_blinds(
    name: str,
    items: Sequence[_Input],
    blinds: Mapping[str, Sequence[Blind]] | None,
) -> set[Blind]:
    found = {item.blind for item in items if item.blind is not None}
    found.update((blinds or {}).get(name, ()))
    return found


def _row_state(*, inert: bool, blind: bool) -> str:
    if blind:
        return model.ParamState.OTHER.value
    if inert:
        return model.ParamState.INERT_BY_DESIGN.value
    return model.ParamState.EFFECTIVE.value


def _candidate_row(item: _Input, group: int) -> CandidateRow:
    return CandidateRow(
        value=item.value,
        value_type="unknown"
        if item.blind is not None and item.blind.reason == "command_substitution"
        else yaml_lines.type_name(item.value),
        mechanism=item.mechanism,
        site=item.site,
        order=item.order,
        group=group,
        selector=item.selector,
        specificity=item.specificity,
        block_order=item.block_order,
        key_order=item.key_order,
        raw=item.raw,
        applied_by=item.applied_by,
        guard=item.guard,
        shadow_kind=item.shadow_kind,
        note=item.note,
    )


def _rank(rows: Sequence[CandidateRow]) -> list[CandidateRow]:
    ordered = sorted(rows, key=CandidateRow.order_key)
    return [replace(row, rank=index) for index, row in enumerate(ordered)]


def _classify(
    rows: Sequence[CandidateRow],
    *,
    inert: bool,
    wins: bool | None = None,
) -> list[CandidateRow]:
    """Ranked rows with the winner marked, or with nobody marked.

    `wins` is False and `inert` is False for a chain whose top value is not knowable: every
    write site stays readable and shadowed the way it really is, and no candidate claims to
    be the effective one.
    """
    if not rows:
        return []
    winner = rows[0]
    takes = (not inert) if wins is None else wins
    out = [replace(winner, rank=0, wins=takes, shadow_kind=None, shadowed_by=None)]
    for row in rows[1:]:
        kind = row.shadow_kind or (
            model.ShadowKind.INERT_PLUGIN_BLOCK.value
            if inert
            else _shadow_kind(winner.value, row.value)
        )
        out.append(replace(row, wins=False, shadow_kind=kind, shadowed_by=0))
    return out


def _shadow_kind(winner: Any, loser: Any) -> str:
    if yaml_lines.type_name(winner) != yaml_lines.type_name(loser):
        return model.ShadowKind.TYPE_CHANGED.value
    if winner == loser:
        return model.ShadowKind.RESTATED_IDENTICAL.value
    return model.ShadowKind.VALUE_CHANGED.value


# ---------------------------------------------------------------------------
# one parameter source


def _inputs_of(
    source: records.ParamSource,
    fqn: str,
    *,
    ws: Workspace,
    walk: records.LaunchWalk,
    merge_group: Mapping[int, int],
) -> tuple[list[_Input], str]:
    if source.kind == "inline_dict":
        return _inline_inputs(source, ws=ws, walk=walk), ""
    if source.kind == "ros_args_assign":
        return _assign_inputs(source, ws=ws, walk=walk), ""
    if source.kind in _FILE_KINDS:
        return _file_inputs(source, fqn, ws=ws, walk=walk, merge_group=merge_group)
    raise ContractError(f"unknown parameter source kind {source.kind!r}")


def _inline_inputs(
    source: records.ParamSource,
    *,
    ws: Workspace,
    walk: records.LaunchWalk,
) -> list[_Input]:
    site = source.site or ws.site(walk.entry_path)
    mechanism = _mechanism(source, selector="", merge_rank=None)
    applied_by = _applied_by(source)
    carried, uncarried = _carried(source.values)
    lines = _inline_lines(site, carried, ws)
    configs = _inline_configs(site, carried, ws)
    passed = _passed_in(site, walk, ws)
    out: list[_Input] = []
    for name, expr, reason in uncarried:
        placed = site if site.locator else site.with_locator(name)
        out.append(
            _Input(
                name=ids.normalise_param_name(name),
                value=None,
                site=placed,
                mechanism=mechanism,
                order=source.order,
                applied_by=applied_by,
                note=reason if reason == "command_substitution" else _clean_note(source.note),
                blind=Blind(
                    expr=substitutions.command_text(expr, ws.token)
                    if reason == "command_substitution"
                    else expr,
                    reason=reason,
                    site=placed,
                ),
                blind_only=reason != "command_substitution",
            )
        )
    for leaf in yaml_lines.leaves(dict(carried)):
        placed = site if site.locator else site.with_locator(leaf.name)
        line = lines.get(leaf.name)
        placed = placed if line is None else replace(placed, line_hint=line)
        stated = passed.get(configs.get(leaf.name, ""))
        note = _clean_note(source.note)
        delivered_at = None
        if stated is not None and _same_text(leaf.value, stated[0]):
            placed, delivered_at = stated[1], placed
            note = _delivered_note(note, site, ws)
        out.append(
            _Input(
                name=leaf.name,
                value=leaf.value,
                site=placed,
                mechanism=mechanism,
                order=source.order,
                key_order=leaf.order,
                applied_by=applied_by,
                note=note,
                reads_arg=configs.get(leaf.name, ""),
                delivered_at=delivered_at,
            )
        )
    return out


def _carried(
    values: Sequence[tuple[str, Any]],
) -> tuple[tuple[tuple[str, Any], ...], list[tuple[str, str, str]]]:
    """The inline dict split into the values the sandbox carried and the ones it could not.

    A non finite number has no json spelling, so the launch child marks it instead of
    dying on it. The mark becomes a blind spot on that one key here and never a value.
    A sequence is one parameter value, so a mark anywhere inside it refuses the whole
    value: letting the list through published the mark itself as the effective value of a
    node, which is a value no source states.
    """
    kept: list[tuple[str, Any]] = []
    refused: list[tuple[str, str, str]] = []
    for name, value in values:
        mark = _mark_inside(value)
        if mark is None:
            kept.append((name, value))
            continue
        refused.append((name, _mark_text(value), mark[1]))
    return tuple(kept), refused


def _mark_inside(value: Any) -> tuple[str, str] | None:
    """The first uncarried mark this value holds, at any depth, None when it holds none."""
    mark = uncarried_reason(value)
    if mark is not None:
        return mark
    if isinstance(value, (list, tuple)):
        for item in value:
            found = _mark_inside(item)
            if found is not None:
                return found
    elif isinstance(value, dict):
        for _key, item in sorted(value.items()):
            found = _mark_inside(item)
            if found is not None:
                return found
    return None


def _mark_text(value: Any) -> str:
    """The value as the source spells it, with each refused number back in its place."""
    mark = uncarried_reason(value)
    if mark is not None:
        return mark[0]
    if isinstance(value, (list, tuple)):
        return "[" + ", ".join(_mark_text(item) for item in value) + "]"
    if isinstance(value, dict):
        inner = ", ".join(f"{key!r}: {_mark_text(item)}" for key, item in sorted(value.items()))
        return "{" + inner + "}"
    return repr(value)


def _same_text(value: Any, text: str) -> bool:
    """Whether a delivered value is the text the include wrote, typed or not.

    launch_ros infers a type from an argument that is always a string, so a rate written as
    `30.0` one file up arrives as a float. Comparing the two as text keeps that hop, and a
    spelling the inference changed keeps the local site rather than guessing.
    """
    return value == text if isinstance(value, str) else str(value) == text


def _passed_in(
    site: records.Site,
    walk: records.LaunchWalk,
    ws: Workspace,
) -> dict[str, tuple[str, records.Site]]:
    """The value and the write site of each argument an include passed into this file.

    A key of an inline dict that reads a launch argument carries whatever the including file
    handed down, so the line an engineer edits to change the value is one file up, at the
    `launch_arguments` key. The chain otherwise terminates at the line that reads the
    argument, which is the plumbing and not the value. Two includes of one file state two
    argument sets and neither of them is this one, so nothing is repointed then, and a
    launch_arguments key the text reader cannot place keeps the local site.
    """
    cited = ws.localise(site.path)
    found = [
        include
        for include in walk.includes
        if include.resolved and include.path and ws.localise(include.path) == cited
    ]
    if len(found) != 1 or found[0].site is None:
        return {}
    include = found[0]
    lines = _inline_lines(replace(include.site, line_hint=None), include.args, ws)
    out: dict[str, tuple[str, records.Site]] = {}
    for name, text in include.args:
        line = lines.get(name)
        if line is None:
            continue
        placed = replace(include.site, locator=f"launch_arguments({name})", line_hint=line)
        out[name] = (text, placed)
    return out


def _delivered_note(note: str, site: records.Site, ws: Workspace) -> str:
    """The hop the chain skipped, named by file so the value site stays the cited one."""
    cited, _root = ws.cite(site.path, site.root)
    package, relative = ws.rel(cited)
    where = f"{package}/{relative}" if package else relative
    delivered = f"delivered through the inline dict in {where}"
    return ", ".join(item for item in (note, delivered) if item)


def _assign_inputs(
    source: records.ParamSource,
    *,
    ws: Workspace,
    walk: records.LaunchWalk,
) -> list[_Input]:
    site = source.site or ws.site(walk.entry_path)
    out: list[_Input] = []
    for raw_name, raw_value in source.values:
        name = ids.normalise_param_name(raw_name)
        text = raw_value if isinstance(raw_value, str) else str(raw_value)
        placed = site if site.locator else site.with_locator(name)
        # Three ways one `-p` value names no value. An rcl refusal takes the whole argument
        # list down and the process never starts, so a value recorded for it is a fact about
        # nothing. A rosio refusal starts, and the value it runs under is not a property of
        # the text. `sets_nothing` starts too, and declares the parameter with no value.
        reason = ros_args.refuse_value(text) or (
            ros_args.SETS_NOTHING if ros_args.sets_nothing(text) else ""
        )
        if reason:
            out.append(
                _Input(
                    name=name,
                    value=None,
                    site=placed,
                    mechanism=model.Mechanism.ROS_ARGS_P.value,
                    order=source.order,
                    raw=text,
                    note=_clean_note(source.note),
                    blind=Blind(expr=f"-p {name}:={text}", reason=reason, site=placed),
                    blind_only=True,
                )
            )
            continue
        out.append(
            _Input(
                name=name,
                value=ros_args.parse_value(text),
                site=placed,
                mechanism=model.Mechanism.ROS_ARGS_P.value,
                order=source.order,
                raw=text,
                note=_clean_note(source.note),
            )
        )
    return out


def _file_inputs(
    source: records.ParamSource,
    fqn: str,
    *,
    ws: Workspace,
    walk: records.LaunchWalk,
    merge_group: Mapping[int, int],
) -> tuple[list[_Input], str]:
    path = source.source_path if source.kind == "rewritten_file" else source.path
    if not path:
        return [], f"parameter source {source.kind} has no path"
    document = _load(path)
    if document is None:
        return [], f"{path} could not be read"
    if document.data is None:
        return [], ""
    if not document.is_parameter_file():
        return [], f"{path} is not a parameter file: {yaml_lines.refuse_reason(document.data)}"
    reshaped, root_key = _reshape(source, document)
    applied_by = _applied_by(source)
    cited, locator_prefix = _provenance(source, path, ws=ws, walk=walk)
    lines = _lines_of(document, cited, path, ws)
    note = _clean_note(source.note)
    if lines is not document:
        note = "; ".join(item for item in (note, UNDERLAY_NOTE) if item)
    unresolved_root, unresolved_keys = _unresolved_wrapping(source.note)
    out: list[_Input] = []
    for block in yaml_lines.param_blocks(reshaped):
        possible_root = _mark_inside(source.root_key) and ids.param_key_matches(
            "/**/" + block.selector.strip("/"), fqn
        )
        if not _selector_matches(block.selector, fqn) and not possible_root:
            continue
        specificity = _specificity(block.selector)
        for leaf in yaml_lines.leaves(block.values):
            source_path = _on_disk_path((*block.key_path, *leaf.key_path), root_key)
            site = _cited_site(lines, source_path, cited, locator_prefix, ws)
            table = dict(source.rewrites)
            mark = _mark_inside(source.root_key) or _mark_inside(
                table.get(".".join(source_path), table.get(leaf.name.split(".")[-1]))
            )
            out.append(
                _Input(
                    name=leaf.name,
                    value=None if mark else leaf.value,
                    site=site,
                    mechanism=_mechanism(
                        source,
                        selector=block.selector,
                        merge_rank=merge_group.get(source.order),
                    ),
                    order=source.order,
                    selector=block.selector,
                    specificity=specificity,
                    block_order=block.order,
                    key_order=leaf.order,
                    applied_by=applied_by,
                    note=note,
                    blind=Blind(
                        expr=substitutions.command_text(mark[0], ws.token),
                        reason=mark[1],
                        site=site,
                    )
                    if mark
                    else _wrapping_blind(unresolved_root, unresolved_keys, leaf.name, path, site),
                )
            )
    return out, ""


_UNRESOLVED_ROOT_NOTE = "unresolved root_key"
_UNRESOLVED_REWRITES_NOTE = "unresolved rewrites: "


def _unresolved_wrapping(note: str) -> tuple[bool, frozenset[str]]:
    """What the RewrittenYaml shim could not perform, read back off the source note.

    The record carries one text channel, so both facts travel in it. They are not the same
    blind spot: a rewrite key nobody could perform makes exactly that key unknowable, while
    a root key nobody could perform makes every block key of the file unknowable.
    """
    text = note or ""
    root = _UNRESOLVED_ROOT_NOTE in text
    text = text.replace(_UNRESOLVED_ROOT_NOTE, "")
    _head, _sep, tail = text.partition(_UNRESOLVED_REWRITES_NOTE)
    return root, frozenset(part.strip() for part in tail.split(",") if part.strip())


def _wrapping_blind(
    unresolved_root: bool,
    unresolved_keys: frozenset[str],
    name: str,
    path: str,
    site: records.Site,
) -> Blind | None:
    """The blind spot a RewrittenYaml leaves on one parameter, or nothing.

    Without this the file value is reported as the effective one while launch rewrites it
    to something nobody read, which is a confident wrong answer rather than a missing one.
    """
    if unresolved_root:
        return Blind(expr=f"root_key of {Path(path).name}", reason="root_key_unresolved", site=site)
    if name.split(".")[-1] in unresolved_keys:
        return Blind(expr=name.split(".")[-1], reason="rewrite_unresolved", site=site)
    return None


def _reshape(
    source: records.ParamSource,
    document: yaml_lines.Document,
) -> tuple[yaml_lines.Document, str]:
    """The document as the node receives it, plus the root key a locator has to lose again.

    A RewrittenYaml wraps the whole file under a namespace and rewrites keys at every
    depth, so the blocks a node matches are not the blocks on disk. Every reader of the
    blocks needs the reshaped document, including the orphan pass: comparing an unwrapped
    key against a namespaced node reports every block in the file as an owner mismatch.
    """
    root_key = "" if _mark_inside(source.root_key) else source.root_key or ""
    if source.kind != "rewritten_file" or not (source.rewrites or root_key):
        return document, ""
    data = rewritten_yaml.apply_rewrites(
        dict(document.data),
        root_key,
        [(key, value) for key, value in source.rewrites if not _mark_inside(value)],
        convert_types="convert_types" in (source.note or ""),
    )
    return yaml_lines.Document(data=data, lines={}, path=document.path), root_key


def _on_disk_path(key_path: tuple[str, ...], root_key: str) -> tuple[str, ...]:
    if root_key and key_path[:1] == (root_key,):
        return key_path[1:]
    return key_path


def _cited_site(
    document: yaml_lines.Document,
    key_path: tuple[str, ...],
    cited: str,
    locator_prefix: str,
    ws: Workspace,
) -> records.Site:
    """One yaml key as provenance on the file the evidence can actually cite.

    The line belongs to the document that was read. When the citation moves to another file,
    because the document was assembled outside the tracked trees, the line does not move with
    it: a reader following it lands on an unrelated line, and on a shorter file on no line at
    all.
    """
    return records.Site(
        path=cited,
        locator=locator_prefix + document.locator(key_path),
        line_hint=None if locator_prefix else document.line(key_path),
        root=ws.root_of(cited),
    )


def _selector_matches(selector: str, fqn: str) -> bool:
    """`ids.param_key_matches` is the only wildcard rule.

    A block key that cannot be a ros name matches nothing rather than raising, so one odd
    yaml key cannot empty a whole node's chain or abort the layer.
    """
    try:
        if not fqn and _specificity(selector) != 1:
            return False
        return ids.param_key_matches(selector, fqn or "/")
    except ContractError:
        return False


def _provenance(
    source: records.ParamSource,
    path: str,
    *,
    ws: Workspace,
    walk: records.LaunchWalk,
) -> tuple[str, str]:
    """Which file the evidence cites, and a locator prefix when that is not the read file.

    A parameters file written by a process outside the tracked trees keeps its content and
    loses its path: the temp name exists in no repository, so the citation moves to the
    symbol that wrote it and the locator keeps the key path.

    The kind is not the only way that happens. A launch file that wraps a temp document in a
    RewrittenYaml hands over a `rewritten_file` whose source path is still the temp one, so
    the question asked here is whether the workspace can cite the path at all.
    """
    if _citable(source, path, ws=ws):
        return ws.localise(path), ""
    twin = ws.underlay_twin(path)
    if twin is not None:
        return twin, ""
    writer = _applied_by(source)
    if writer is not None:
        parsed = ids.parse(writer)
        if parsed.kind == "sym":
            return ws.abspath(ids.build_file(parsed.segments[0], parsed.segments[1])), (
                f"{parsed.segments[2]}:"
            )
        if parsed.kind == "file":
            return ws.abspath(writer), ""
    return walk.entry_path, "external params file:"


UNDERLAY_NOTE = (
    "launch read the released copy of this file, and the workspace file cited here takes "
    "effect once the workspace is built and its setup file sourced"
)


def _lines_of(
    document: yaml_lines.Document, cited: str, path: str, ws: Workspace
) -> yaml_lines.Document:
    """The document whose lines a citation states. A released copy cited as the source file
    that stands for it takes the lines of that file, where a key may sit elsewhere."""
    if cited != ws.underlay_twin(path):
        return document
    return _load(cited) or document


def _citable(source: records.ParamSource, path: str, *, ws: Workspace) -> bool:
    """Whether provenance may name the file the values were read from.

    Root membership is the workspace's answer, not the source kind's. A launch file that
    self locates with a real path names workspace content through a spelling the walk could
    not match against the configured roots, so it arrives classified as external while the
    file is right there in the repository. Only a path that names the same file either way
    keeps the classification it came with, which leaves a genuine temp document external.
    """
    if not _readable(ws, path):
        return False
    if source.kind != "external_file":
        return True
    return ws.aliased(path)


def _similar_files(ws: Workspace, path: str) -> list[str]:
    """Files beside a missing one whose names are one slip from its name, as file ids:
    `lidar.yaml` beside a launch default naming `lidar_params.yaml`."""
    local = Path(ws.localise(path))
    if not local.parent.is_dir():
        return []
    names = sorted(item.name for item in local.parent.iterdir() if item.is_file())
    close = difflib.get_close_matches(local.name, names, n=3, cutoff=0.6)
    return sorted(ws.file_id(str(local.parent / name)) for name in close)


def _merge_group(sources: Sequence[records.ParamSource]) -> dict[int, int]:
    """Rank of each file source inside a merge performed outside the launch system.

    The reference launcher deep merges a base and an overlay into one temp file, so the
    first file of that group is the base and the rest are overlays. The rank is what names
    the mechanism, and the merge site is what makes the value citable.
    """
    merged = [
        source
        for source in sorted(sources, key=lambda item: item.order)
        if source.kind in ("repo_file", "external_file") and source.produced_by
    ]
    return {source.order: index for index, source in enumerate(merged)}


def _mechanism(
    source: records.ParamSource,
    *,
    selector: str,
    merge_rank: int | None,
) -> str:
    override = _note_mechanism(source.note)
    if override:
        return override
    if source.kind == "inline_dict":
        if source.produced_by:
            return model.Mechanism.PYTHON_OVERRIDE.value
        if source.note.startswith("SetParameter"):
            return model.Mechanism.LAUNCH_SET_PARAMETER.value
        return model.Mechanism.LAUNCH_INLINE_DICT.value
    if source.kind == "ros_args_assign":
        return model.Mechanism.ROS_ARGS_P.value
    if source.kind == "ros_args_file":
        return model.Mechanism.ROS_ARGS_PARAMS_FILE.value
    if source.kind == "rewritten_file":
        return model.Mechanism.LAUNCH_REWRITTEN_YAML.value
    if _specificity(selector) < 3:
        return model.Mechanism.YAML_WILDCARD.value
    if merge_rank is not None:
        return (
            model.Mechanism.YAML_BASE.value
            if merge_rank == 0
            else model.Mechanism.YAML_OVERLAY_MERGE.value
        )
    return model.Mechanism.LAUNCH_PARAMS_FILE.value


def _note_mechanism(note: str) -> str:
    """`mechanism=<name>` in a source note names a mechanism the record kinds cannot.

    A launch file that loops `ros2 param set` from a file some seconds after start is
    `param_set_runtime`, and without it those parameters read as pure drift.
    """
    for part in (note or "").split(","):
        text = part.strip()
        if text.startswith("mechanism="):
            wanted = text.split("=", 1)[1].strip()
            allowed = {item.value for item in model.Mechanism}
            if wanted not in allowed:
                raise ContractError(f"note asked for unknown mechanism {wanted!r}")
            return wanted
    return ""


def _clean_note(note: str) -> str:
    parts = [
        part.strip()
        for part in (note or "").split(",")
        if part.strip() and not part.strip().startswith("mechanism=")
    ]
    return ", ".join(parts)


def _applied_by(source: records.ParamSource) -> str | None:
    if not source.produced_by:
        return None
    try:
        ids.parse(source.produced_by)
    except ContractError:
        return None
    return source.produced_by


def _walk_files(
    walk: records.LaunchWalk,
    pipelines: Sequence[records.ParamPipeline] = (),
    ws: Workspace | None = None,
) -> list[tuple[str, records.ParamSource]]:
    seen: dict[str, records.ParamSource] = {}
    for path, source, _node in _file_reads(walk, pipelines, ws):
        seen.setdefault(path, source)
    return sorted(seen.items())


def _file_reads(
    walk: records.LaunchWalk,
    pipelines: Sequence[records.ParamPipeline] = (),
    ws: Workspace | None = None,
) -> Iterator[tuple[str, records.ParamSource, records.ResolvedNode]]:
    """Every parameters file a node of the walk reads, with the node that reads it."""
    for node in walk.nodes:
        for step in _plan(walk, node.params, pipelines, ws):
            source = step.source
            if source is None or source.kind not in _FILE_KINDS:
                continue
            path = source.source_path if source.kind == "rewritten_file" else source.path
            if path:
                yield path, source, node


def _load(path: str) -> yaml_lines.Document | None:
    try:
        stat = Path(path).stat()
        return _load_stamped(path, stat.st_size, stat.st_mtime_ns)
    except (OSError, CollectorError):
        return None


@functools.lru_cache(maxsize=256)
def _load_stamped(path: str, _size: int, _mtime: int) -> yaml_lines.Document:
    """One parse per file content: every profile reads the same parameter files again, and
    no reader edits a loaded document, a reshape copies it first."""
    return yaml_lines.load_file(path)


def _walks_for(ctx: CollectContext) -> list[records.LaunchWalk]:
    """The launch walks this pass covers, which is everything it reads."""
    return [
        walk
        for walk in ctx.declared.walks
        if not ctx.profile or not walk.profile or walk.profile == ctx.profile
    ]


_YAML_LINE = re.compile(r"is not readable as yaml: line (\d+), column")


def _refusal(path: str) -> str:
    """Why this path is not a parameters file this collector can read, empty when it is.

    The reason is compared data, so the absolute path the reader raised with is reduced to
    the file name the entity already spells.
    """
    try:
        document = yaml_lines.load_file(path)
    except CollectorError as exc:
        return str(exc).replace(path, Path(path).name)
    if document.data is None:
        # rcl takes an empty parameters file and sets nothing (measured on jazzy)
        return ""
    return yaml_lines.refuse_reason(document.data)


def _specificity(selector: str) -> int:
    cleaned = (selector or "").strip()
    if cleaned in ("/**", "**", "/**/", "/*", ""):
        return 1
    if cleaned.startswith("/**/") or cleaned.startswith("**/"):
        return 2
    return 3


def _selector_fqn(selector: str) -> str:
    try:
        return ids.normalise_ros_name("/" + selector.strip().strip("/"))
    except ContractError:
        return ""


def _selected_instances(winners: Mapping[str, Any]) -> dict[str, str]:
    """Every instance a selector list activates, mapped to the list that named it.

    One node carries several selector lists at one scope, and the base class of an instance
    follows the list that spells it, not the first list the node happens to hold. Keeping
    the pair together is the only way the two can never come apart.
    """
    out: dict[str, str] = {}
    chosen: dict[str, str] = {}
    for name in sorted(winners):
        parts = name.split(".")
        prefix = ".".join(parts[:-1])
        value = winners[name]
        if parts[-1] not in SELECTOR_KEYS:
            # A string naming a sibling block that declares a plugin selects it, the way
            # `motion_model: "diff_drive"` picks the `diff_drive` block of an MPPI controller.
            block = f"{prefix}.{value}" if prefix else str(value)
            if (
                isinstance(value, str)
                and value
                and parts[-1] != PLUGIN_KEY
                and f"{block}.{PLUGIN_KEY}" in winners
            ):
                chosen.setdefault(block, name)
            continue
        items = value if isinstance(value, (list, tuple)) else [value]
        for item in items:
            if not isinstance(item, str) or not item:
                continue
            out.setdefault(f"{prefix}.{item}" if prefix else item, name)
    for block, name in chosen.items():
        out.setdefault(block, name)
    # A plugin block nested in a selected instance is that plugin's to load, by a name its
    # code states: MPPI reads `<name>.TrajectoryValidator.plugin`. The selector is the parent's
    # plugin key, which names no base class, so no plugin id is minted for it.
    suffix = f".{PLUGIN_KEY}"
    for key in sorted((name for name in winners if name.endswith(suffix)), key=_depth):
        block = key[: -len(suffix)]
        parent = block.rpartition(".")[0]
        while parent and block not in out:
            if parent in out and f"{parent}{suffix}" in winners:
                out[block] = f"{parent}{suffix}"
            parent = parent.rpartition(".")[0]
    return out


def _depth(name: str) -> tuple[int, str]:
    return (name.count("."), name)


def _governing_block(
    name: str,
    winners: Mapping[str, Any],
    selected: Mapping[str, str],
) -> tuple[str, str]:
    """The plugin block that gates this parameter, and the selector key that named it.

    A block gates its keys only when it declares a plugin of its own. A plain nested group
    (a goal checker's tolerances, a diagnostics group) has no plugin key and is not a
    plugin block, so it is never reported inert.

    A block no list names carries no selector key. The first selector key at that scope used
    to be recorded instead, which is a guess as soon as a node holds two lists, and a guess
    inside a recorded fact is worse than an absent field: the rule that reads it then tells
    an engineer to add a progress checker to the controller list. The absent field is the
    honest form, and the rule says the block is claimed by no selector.
    """
    parts = name.split(".")
    for depth in range(len(parts) - 1, 0, -1):
        block = ".".join(parts[:depth])
        if f"{block}.{PLUGIN_KEY}" not in winners:
            continue
        if block in selected:
            return "", selected[block]
        return block, ""
    return "", ""


# ---------------------------------------------------------------------------
# the collector


def _entrypoint_order(
    walks: Sequence[records.LaunchWalk],
    ctx: CollectContext,
) -> list[records.LaunchWalk]:
    """The walks of each profile in the order that profile declares its entry points.

    Precedence between two entry points that write one parameter is the order the profile
    runs them in, which is the order the launcher issues the commands in. Sorting by launch
    file path instead lets an entry point declared first override one declared later, which
    is a wrong answer stated with confidence.
    """
    seen = {id(walk): index for index, walk in enumerate(ctx.declared.walks)}
    return sorted(
        walks,
        key=lambda walk: (walk.profile, _entry_index(walk, ctx), seen.get(id(walk), 0)),
    )


def _entry_index(walk: records.LaunchWalk, ctx: CollectContext) -> int:
    """Position of the entry point this walk ran in its profile, last when unmatched."""
    slug = walk.profile or ctx.profile
    try:
        spec = ctx.cfg.profile(slug) if slug else None
    except UsageError:
        return 0
    if spec is None:
        return 0
    name = Path(walk.entry_path).name
    for index, entry in enumerate(spec.entrypoints):
        if Path(entry.target).name == name and tuple(entry.args) == tuple(walk.args):
            return index
    return len(spec.entrypoints)


@dataclass
class _Pending:
    state: str
    rows: list[CandidateRow] = field(default_factory=list)
    row: ParamCandidateRow | None = None
    blinds: list[Blind] = field(default_factory=list)
    block_sites: list[records.Site] = field(default_factory=list)
    """Every block key this parameter is written under, one per file. A key spelled in a
    base file and again in an overlay sits in two blocks, a reader deleting either one
    deletes from its own first line, and one site per parameter names only one of them."""


class ParamsCollector:
    """Declared layer collector for parameter values.

    It consumes `ctx.declared.walks`, so it never reads another collector's atlas entries
    and running the collectors in a different order produces the same atlas.
    """

    name = "params"
    layer = LAYER

    def inputs_present(self, ctx: CollectContext) -> bool | str:
        """Whether any launch walk was handed to this pass.

        The walk list is this collector's whole input, and it is empty for two reasons that
        look identical here: nothing was launched, or the pass that walks launch files died.
        Saying `completed` over an empty list claimed a parameter pass that read everything
        while it had read nothing, so an empty list is stated as an absence and the dead
        launch pass is reported by its own row.
        """
        return True if _walks_for(ctx) else NO_WALK

    def collect(self, ctx: CollectContext) -> None:
        walks = _walks_for(ctx)
        if not walks:
            return  # nothing to read, which `inputs_present` states to the orchestrator
        pipelines = tuple(getattr(ctx.declared, "pipelines", ()) or ())
        by_param: dict[tuple[str, str], _Pending] = {}
        ordered = _entrypoint_order(walks, ctx)
        grouped: dict[str, list[records.LaunchWalk]] = {}
        for walk in ordered:
            slug = walk.profile or ctx.profile
            profile_id = ids.build_profile(slug) if slug else model.ANY_PROFILE
            grouped.setdefault(profile_id, []).append(walk)
        runs = {
            profile_id: self._run(
                ctx, group_walks, group_walks[0].profile or ctx.profile, pipelines
            )
            for profile_id, group_walks in grouped.items()
        }
        memo: dict[str, bool] = {}
        taken = {
            profile_id: frozenset(name for walk in group_walks for name in _owner_names(walk))
            for profile_id, group_walks in grouped.items()
        }
        for group, walk in enumerate(ordered):
            slug = walk.profile or ctx.profile
            profile_id = ids.build_profile(slug) if slug else model.ANY_PROFILE
            alternates: set[str] = set()
            for node in sorted(walk.nodes, key=_node_key):
                if node.attrs.get("unreachable") or any(
                    gate.outcome == "false" for gate in node.gates
                ):
                    # A condition that holds false in this profile: the node never starts,
                    # so nothing it would read is live configuration of anything.
                    continue
                for owner in (node, *child_nodes(node)):
                    try:
                        rows = chain_for(walk, owner, ws=ctx.ws, group=group, pipelines=pipelines)
                    except ContractError:
                        # The blocks of a walk whose node names cannot be formed become
                        # `unresolved_owner` rows in `orphan_rows`, which is where that
                        # refusal is stated.
                        continue
                    self._register_owner(ctx, owner)
                    for row in rows:
                        self._accumulate(by_param, row, profile_id, ctx.ws, memo)
                for name in alternate_owners(
                    walk, node, ws=ctx.ws, pipelines=pipelines, taken=taken[profile_id]
                ):
                    alternates.add(name)
                    for row in alternate_chain(
                        walk, node, name, ws=ctx.ws, group=group, pipelines=pipelines
                    ):
                        self._accumulate(by_param, row, profile_id, ctx.ws, memo)
            for row in orphan_rows(
                walk,
                ws=ctx.ws,
                group=group,
                pipelines=pipelines,
                alternates=sorted(alternates),
            ):
                self._accumulate(by_param, row, profile_id, ctx.ws, memo)
        for profile_id, group_walks in sorted(grouped.items()):
            self._refused_files(ctx, group_walks, pipelines, profile_id)
        self._emit(ctx, by_param, runs)
        self._plugin_pass(ctx, by_param, runs)

    # ------------------------------------------------------------------

    def _refused_files(
        self,
        ctx: CollectContext,
        walks: Sequence[records.LaunchWalk],
        pipelines: Sequence[records.ParamPipeline],
        profile_id: str,
    ) -> None:
        """Every parameters file a profile names and this collector could not read.

        A file that refuses to load left nothing at all behind: no entity, no exception row
        and no blind spot, so an atlas missing a whole file read exactly like an atlas that
        had it. The values are still gone, and now the file says so.
        """
        seen: dict[str, tuple[str, records.Site | None, str]] = {}
        exprs: dict[str, str] = {}
        origins: dict[str, records.Site] = {}
        roots = [spec.path for spec in ctx.ws.roots]
        for walk in walks:
            supplied = {name for name, _value in walk.args}
            for declaration in walk.arg_declarations:
                # A path an argument default spells is written on the declaration's line,
                # and that line is the one to correct when the file is not there.
                if declaration.default and declaration.site and declaration.name not in supplied:
                    origins.setdefault(declaration.default, declaration.site)
            for path, source, node in _file_reads(walk, pipelines, ctx.ws):
                if path not in seen:
                    reader = node.site or source.site
                    seen[path] = (_refusal(path), reader, "parameters_file")
                    if identity.under_temp(path, roots):
                        # The launch file wrote it while it was evaluated, under a random
                        # name, so the reader, its namespace and the item as the launch file
                        # spells it are what tell two apart. A position would move whenever
                        # an enclosing group gained a SetParameter.
                        spelled = (source.text or source.kind).replace(
                            path, identity.EXTERNAL_TOKEN
                        )
                        exprs[path] = (
                            f"{identity.EXTERNAL_TOKEN} read at "
                            f"{reader.locator if reader else 'parameters'} in "
                            f"{node.namespace or identity.ROOT} as {spelled}"
                        )
            for node in walk.nodes:
                # a site whose condition read false never opens the file in this profile
                if node.attrs.get("unreachable") or any(
                    gate.outcome == "false" for gate in node.gates
                ):
                    continue
                for read in node.file_reads:
                    if (
                        read.path in seen
                        or not _readable(ctx.ws, read.path)
                        or Path(ctx.ws.localise(read.path)).exists()
                    ):
                        continue
                    seen[read.path] = (MISSING_FILE, read.site or node.site, read.role)
        for path in sorted(seen):
            reason, reader, role = seen[path]
            if not reason:
                continue
            exists = Path(ctx.ws.localise(path)).exists()
            expr = exprs.get(path)
            # A file that is not there is refused for that, whichever reader reached it, so
            # two profiles naming one missing file state one refusal.
            because = REFUSED_PARAMS_FILE if exists else _missing_reason(ctx.ws, path)
            if not exists and expr is None:
                reason = because
            owner = self._refused_file_entity(ctx, path, reason, because) or profile_id
            evidence = []
            if owner.startswith("file:") and exists:
                line = _YAML_LINE.search(reason)
                site = ctx.ws.site(path, line_hint=int(line.group(1)) if line else None)
                evidence.append(ctx.ws.source(site))
            if not exists and path in origins:
                evidence.append(ctx.ws.source(origins[path]))
            # a reader inside a launch file no root holds (nav2_bringup from the release)
            # has no line here to cite
            held = reader is not None and _readable(ctx.ws, reader.path)
            if not exists and held:
                evidence.append(ctx.ws.source(reader))
            if expr is not None:
                if held and not evidence:
                    evidence.append(ctx.ws.source(reader))
                reason = (
                    reason.replace(Path(path).name, identity.EXTERNAL_TOKEN)
                    if Path(path).exists()
                    else "the launch file wrote it under a temp directory while it was "
                    "evaluated, and it was gone before it could be read"
                )
            ctx.builder.unresolved(
                model.Unresolved(
                    want=model.Want.OTHER.value,
                    owner=owner,
                    expr=expr or Path(path).name,
                    reason=reason,
                    role=role,
                    resolvable_by=["a file a profile names has to be readable"],
                    evidence=evidence,
                )
            )

    def _refused_file_entity(
        self, ctx: CollectContext, path: str, reason: str, because: str
    ) -> str:
        """The file entity of a refused parameters file, empty when no root can place it.

        The refusal is attributes and no fact. A refusal is a statement about the reader and
        not a claim any layer made about the system, and the fact this used to write put
        `declared` in the entity's `layers`, so one consumer filtering by layer met the
        parameters refusals and none of the others.
        """
        ws = ctx.ws
        if not _readable(ws, path):
            return ""
        file_id = ws.file_id(path)
        attrs: dict[str, Any] = {"loadable": False, "refused_because": because}
        if reason != because:
            attrs["refused_detail"] = reason
        if because == BUILT_ONLY:
            attrs["found_elsewhere"] = same_name_in_package(ws, path)
        if because == MISSING_FILE:
            similar = _similar_files(ws, path)
            if similar:
                attrs["similar_files"] = similar
        held = ctx.builder.atlas.entities.get(file_id)
        stated = held.attrs.get("refused_because") if held is not None else None
        if held is not None and stated:
            # One file keeps one refusal: a weaker one another profile stated gives way, in
            # place, so the order the profiles were read in decides nothing.
            if stated in _REFUSAL_ORDER and _REFUSAL_ORDER.index(because) < _REFUSAL_ORDER.index(
                stated
            ):
                held.attrs.pop("refused_detail", None)
                held.attrs.update(attrs)
            return file_id
        ctx.builder.node(file_id, attrs=attrs)
        package, _relpath = ws.rel(path)
        if package and package != ids.REPO_PKG:
            pkg_id = ids.build_pkg(package)
            ctx.builder.node(pkg_id)
            ctx.builder.edge(model.EdgeKind.CONTAINS.value, pkg_id, file_id)
        return file_id

    def _run(
        self,
        ctx: CollectContext,
        walks: Sequence[records.LaunchWalk],
        slug: str,
        pipelines: Sequence[records.ParamPipeline] = (),
    ) -> str:
        """One run per profile, over every entry point that profile is composed of.

        A profile on a real stack is an ordered set of entry points, so a digest over the
        first one alone would leave the rest of the inputs and their filesystem predicates
        out of the record, and two boxes that disagree about a file would look like drift.

        Every entry is framed by `ids.digest_fields`, which states the argument count and
        escapes each member. Space joining the arguments first gave `('a', 'b c=d')` and
        `('a', 'b'), ('c', 'd')` one digest, which is one input for two entry points.
        """
        files: set[str] = set()
        entries: list[str] = []
        predicates: dict[str, bool] = {}
        for walk in walks:
            files.update(
                ctx.ws.file_id(path)
                for path, _source in _walk_files(walk, pipelines, ctx.ws)
                if _readable(ctx.ws, path)
            )
            args: list[str] = []
            for key, value in walk.args:
                args.append(str(key))
                args.append(ctx.ws.token(value))
            entries.append(
                ids.digest_fields({"entry": _entry_token(ctx.ws, walk.entry_path), "args": args})
            )
            for name in sorted(walk.predicates):
                predicates[name] = walk.predicates[name]
        digest = ids.digest_fields(
            {
                "collector": self.name,
                "entries": sorted(entries),
                "files": sorted(files),
            }
        )
        run = ctx.builder.add_run(
            LAYER,
            profile=ids.build_profile(slug) if slug else None,
            input_digest=digest,
            roots=ctx.ws.root_hashes(),
            predicates=dict(sorted(predicates.items())),
        )
        ctx.builder.declare_layer(LAYER, present=True, run=run)
        return run

    def _register_owner(
        self,
        ctx: CollectContext,
        node: records.ResolvedNode,
    ) -> None:
        """The owner entity, with the id and the name source `resolve.identity` decided."""
        decided = identity.identify(node)
        held = ctx.builder.atlas.entities.get(decided.node_id)
        parent = node.attrs.get("child_of")
        # The strongest answer wins whoever wrote first, see `ids.NAME_SOURCE_ORDER`.
        # A child names the node that creates it, so a probe that sees the child running
        # can tell it apart from a node nothing in the repository starts.
        ctx.builder.node(
            decided.node_id,
            name_source=ids.strongest_name_source(
                held.name_source if held is not None else None, decided.name_source
            ),
            attrs={"child_of": {parent: node.attrs.get("created_by", "")}} if parent else None,
        )

    def _accumulate(
        self,
        by_param: dict[tuple[str, str], _Pending],
        row: ParamCandidateRow,
        profile_id: str,
        ws: Workspace,
        memo: dict[str, bool],
    ) -> None:
        if row.param_id is None:
            return
        row = _without_unplaceable(_without_unwritable(row), ws, memo)
        key = (row.param_id, profile_id)
        pending = by_param.get(key)
        if pending is None:
            pending = _Pending(state=row.state, rows=list(row.candidates), row=row)
            by_param[key] = pending
        else:
            pending.state = _stronger(pending.state, row.state)
            pending.rows.extend(row.candidates)
            if pending.row is None or row.state == model.ParamState.EFFECTIVE.value:
                pending.row = row
        if row.block_site is not None and row.block_site not in pending.block_sites:
            pending.block_sites.append(row.block_site)
        for blind in row.unresolved:
            if blind not in pending.blinds:
                pending.blinds.append(blind)
        if pending.blinds:
            # One entry point that could not be followed is not repaired by another that
            # could: the value the node ends up with is still not known.
            pending.state = model.ParamState.OTHER.value

    def _emit(
        self,
        ctx: CollectContext,
        by_param: dict[tuple[str, str], _Pending],
        runs: Mapping[str, str],
    ) -> None:
        per_id: dict[str, list[tuple[str, _Pending]]] = {}
        for (param_id, profile_id), pending in by_param.items():
            per_id.setdefault(param_id, []).append((profile_id, pending))
        for param_id in sorted(per_id):
            entries = sorted(per_id[param_id], key=lambda item: item[0])
            state = _stronger_all(pending.state for _profile, pending in entries)
            if any(pending.blinds for _profile, pending in entries):
                state = model.ParamState.OTHER.value
            candidates: list[model.Candidate] = []
            for profile_id, pending in entries:
                rows = _classified(pending)
                candidates.extend(row.to_candidate(ctx.ws, profile_id) for row in rows)
            for candidate in candidates:
                ctx.builder.node(candidate.source.file)
            ctx.builder.set_param(param_id, state=state, candidates=candidates)
            if state == model.ParamState.EFFECTIVE.value:
                ctx.builder.count_resolved(model.Want.PARAM_VALUE.value)
            for profile_id, pending in entries:
                self._fact(ctx, param_id, profile_id, pending, runs.get(profile_id))
                self._blind(ctx, param_id, pending)

    def _fact(
        self,
        ctx: CollectContext,
        param_id: str,
        profile_id: str,
        pending: _Pending,
        run: str | None,
    ) -> None:
        row = pending.row
        if row is None:
            return
        rows = _classified(pending)
        if not rows:
            return
        winner = rows[0]
        values: dict[str, Any] = {
            "state": pending.state,
            "mechanism": winner.mechanism,
            "value_type": winner.value_type,
        }
        if row.owner_fqn:
            values["owner"] = row.owner_fqn
        if pending.state == model.ParamState.EFFECTIVE.value:
            values["value"] = ctx.ws.stated_value(winner.value)
        if row.block:
            values["block"] = row.block
        if row.selector_key:
            values["selector"] = row.selector_key
        # The chain is the provenance of a parameter and a fact carries no evidence of its
        # own, with one exception: a block no node owns is reported per block, and the row a
        # reader is told to delete is the block key, not the first key inside it. One block
        # is one file, so every file this key is written in contributes its own block key
        # and the rule reading them picks the one that names the block it reports.
        blocks = [cited for site in pending.block_sites if (cited := _cite(ctx.ws, site))]
        for block_at in blocks:
            ctx.builder.node(block_at.file)
        ctx.builder.fact(
            param_id,
            LAYER,
            profile=profile_id,
            run=run,
            values=values,
            evidence=blocks,
            confidence=model.Confidence.RESOLVED.value,
        )
        owner = ids.parse_param(param_id).owner
        if row.owner_fqn and owner in ctx.builder.atlas.entities:
            ctx.builder.edge(
                model.EdgeKind.HAS_PARAM.value,
                owner,
                param_id,
                layer=LAYER,
                profile=profile_id,
                run=run,
            )

    def _blind(self, ctx: CollectContext, param_id: str, pending: _Pending) -> None:
        """One unresolved entry per blind spot on this parameter's path.

        The entry is what stops a hollow chain from reading as a complete one. The chain
        itself stays in the atlas, so the write sites that were found are still readable.
        """
        for blind in sorted(pending.blinds, key=Blind.key):
            cited = _cite(ctx.ws, blind.site)
            ctx.builder.unresolved(
                model.Unresolved(
                    want=model.Want.PARAM_VALUE.value,
                    owner=param_id,
                    expr=ctx.ws.stated_value(blind.expr, ctx.cfg.ros_setup),
                    reason=blind.reason,
                    evidence=[cited] if cited is not None else [],
                )
            )

    def _plugin_pass(
        self,
        ctx: CollectContext,
        by_param: Mapping[tuple[str, str], _Pending],
        runs: Mapping[str, str],
    ) -> None:
        """Plugin instances, once per owner per profile, after the chains are aggregated.

        Two entry points of one profile can hand one node different plugin selectors, and a
        fact slot is written once: writing per walk turns that into a contract error and
        loses the aggregated winner.
        """
        live: dict[tuple[str, str], dict[str, CandidateRow]] = {}
        for (param_id, profile_id), pending in by_param.items():
            if pending.state != model.ParamState.EFFECTIVE.value:
                continue
            parsed = ids.parse_param(param_id)
            live.setdefault((parsed.owner, profile_id), {})[parsed.name] = _classify(
                _rank(pending.rows), inert=False
            )[0]
        for owner, profile_id in sorted(live):
            self._plugins(ctx, owner, live[(owner, profile_id)], profile_id, runs.get(profile_id))

    def _plugins(
        self,
        ctx: CollectContext,
        owner: str,
        live: Mapping[str, CandidateRow],
        profile_id: str,
        run: str | None,
    ) -> None:
        """Plugin instances and plugin scoped parameters of one node.

        Plugin identity gates which sub keys of a block are live, so the selected instance
        is an entity of its own. Without it, swapping one plugin selector reports every
        surviving sub key of the old block as drift.
        """
        winners = {name: row.value for name, row in live.items()}
        selected = _selected_instances(winners)
        for instance in sorted(selected):
            lookup = winners.get(f"{instance}.{PLUGIN_KEY}")
            if not isinstance(lookup, str) or not lookup:
                continue
            selector_key = selected[instance]
            exported = _exported_bases(ctx.ws).get(lookup, frozenset())
            if len(exported) == 1:
                # the manifest that exports the selected class names its base class
                base_class = next(iter(exported))
            elif exported:
                base_class = ""
            else:
                base_class = SELECTOR_BASE_CLASS.get(selector_key.split(".")[-1], "")
            chosen = live.get(f"{instance}.{PLUGIN_KEY}")
            cited = _cite(ctx.ws, chosen.site) if chosen is not None else None
            evidence = [cited] if cited is not None else []
            pinst = ids.build_pinst(owner, instance)
            ctx.builder.fact(
                pinst,
                LAYER,
                profile=profile_id,
                run=run,
                values={"plugin": lookup, "selector": selector_key, "base_class": base_class},
                evidence=evidence,
            )
            if not base_class:
                continue
            plugin = ids.build_plugin(base_class, lookup)
            ctx.builder.fact(
                plugin,
                LAYER,
                profile=profile_id,
                run=run,
                values={"lookup": lookup, "base_class": base_class},
                evidence=evidence,
            )
            ctx.builder.edge(
                model.EdgeKind.INSTANTIATES.value,
                pinst,
                plugin,
                layer=LAYER,
                profile=profile_id,
                run=run,
                evidence=evidence,
            )
            self._pparams(ctx, live, instance, base_class, lookup, profile_id, run)

    def _pparams(
        self,
        ctx: CollectContext,
        live: Mapping[str, CandidateRow],
        instance: str,
        base_class: str,
        lookup: str,
        profile_id: str,
        run: str | None,
    ) -> None:
        """The sub keys the yaml names under one plugin instance.

        A pparam is identity on the plugin side, shared by every instance of that class, so
        the fact says which suffix exists and cites every yaml line that spells it. The
        value belongs to the node scoped parameter: two costmaps in one profile give the
        same class two different inflation radii, and a value on the shared entity would
        make one of them a false claim.
        """
        prefix = f"{instance}."
        for name in sorted(live):
            if not name.startswith(prefix) or name == f"{instance}.{PLUGIN_KEY}":
                continue
            suffix = name[len(prefix) :]
            pparam = ids.build_pparam(base_class, lookup, suffix)
            cited = _cite(ctx.ws, live[name].site)
            ctx.builder.fact(
                pparam,
                LAYER,
                profile=profile_id,
                run=run,
                values={"suffix": suffix, "base_class": base_class, "lookup": lookup},
                evidence=[cited] if cited is not None else [],
            )
            ctx.builder.set_param(pparam, state=model.ParamState.EFFECTIVE.value)


def _classified(pending: _Pending) -> list[CandidateRow]:
    """The aggregated chain, ranked, with a winner only when the state allows one."""
    effective = pending.state == model.ParamState.EFFECTIVE.value
    other = pending.state == model.ParamState.OTHER.value
    rows = _read_somewhere_in_profile(pending.rows)
    return _classify(_rank(rows), inert=not effective and not other, wins=effective)


def _read_somewhere_in_profile(rows: Sequence[CandidateRow]) -> list[CandidateRow]:
    """A displaced row drops when another entry point of the same profile reads that line.

    A profile is an ordered set of entry points and the displaced scan sees one of them, so a
    file the second entry point overrode can be the file the first one read. Keeping the row
    would say `not_read_in_profile` about a line this profile does read, and the real
    candidate on that line already states it.
    """
    displaced = model.ShadowKind.NOT_READ_IN_PROFILE.value
    read = {(row.site.path, row.site.locator) for row in rows if row.shadow_kind != displaced}
    return [
        row
        for row in rows
        if row.shadow_kind != displaced or (row.site.path, row.site.locator) not in read
    ]


def _owner_names(walk: records.LaunchWalk) -> list[str]:
    out: list[str] = []
    for node in walk.nodes:
        for owner in (node, *child_nodes(node)):
            try:
                out.append(owner_name(owner))
            except ContractError:
                continue
    return out


def _node_key(node: records.ResolvedNode) -> tuple[str, str, str]:
    return (node.namespace or "/", node.name or "", node.executable or "")


def _entry_token(ws: Workspace, path: str) -> str:
    """The entry point as compared data: a file id, never a machine path.

    An input digest holding an absolute path makes two boxes with the same repository at
    different mount points look like two different runs.
    """
    try:
        return ws.file_id(path)
    except ContractError:
        return Path(path).name


def _missing_reason(ws: Workspace, path: str) -> str:
    """Why a file a launch file names is not there: never installed, installed only by a
    build that has not happened, or simply absent."""
    if ws.src_twin(path):
        return NOT_INSTALLED
    package = ws.packages.get(ws.package_of(path) or "")
    if package is not None and not package.share_path and same_name_in_package(ws, path):
        return BUILT_ONLY
    return MISSING_FILE


def _readable(ws: Workspace, path: str) -> bool:
    try:
        ws.file_id(path)
    except ContractError:
        return False
    return True


def _stronger(left: str, right: str) -> str:
    return model.stronger_param_state(left, right)


def _stronger_all(states: Any) -> str:
    out = model.ParamState.OTHER.value
    for index, state in enumerate(states):
        out = state if index == 0 else _stronger(out, state)
    return out


register("params", ParamsCollector)


__all__ = [
    "ALTERNATE_NOTE",
    "BUILT_ONLY",
    "LAYER",
    "NOT_INSTALLED",
    "NO_WALK",
    "PLUGIN_KEY",
    "REFUSED_PARAMS_FILE",
    "SELECTOR_BASE_CLASS",
    "SELECTOR_KEYS",
    "Blind",
    "CandidateRow",
    "ParamCandidateRow",
    "ParamsCollector",
    "alternate_chain",
    "alternate_owners",
    "chain_for",
    "child_nodes",
    "orphan_rows",
    "owner_name",
    "owner_node_id",
]
