"""Frame, remap and lifecycle rules.

Every function here is a pure function of the joined atlas. Nothing reads a file, nothing
re extracts and nothing mutates an entity. Each rule binds to its committed id through
`rules.register` and takes its severity from the table in `rosio/drift/rules.py`, so the
gate is graded in one place. `RULES` is the same mapping for a caller that wants this
family alone. Both modules still need naming in `rules.FAMILIES` to load by themselves.

Three scoping decisions decide whether these rules are quiet, and all three are forced by
the reference stack rather than tuned against it.

Profile. A rule compares inside one profile key and never across two. The urdf collector
states its facts for every profile at once, so its profile key is `*`, and `*` is treated
as a scope of its own. Nothing is compared between `*` and a named profile.

Model. A tf_parent fact carries the urdf model that produced the geometry, because one
robot has three models here and each states its own origin for the same joint. A finding
that crosses models keys on the sorted model list, so a difference that is deliberate can
be waived once by id and stays waived when either value changes. Only a model some profile
loads is compared: two models of two different vehicles state different geometry by
construction, and one of the three here is a second vehicle model that no profile in the
bench builds. Which model a profile loads is read off the `robot_description` the profile
gives its state publisher, because that is the only per profile statement of the model in
the atlas, and the model is recognised by the links it declares, which the frame entities
already name.

Coverage. A frame rule that needs to know who publishes a transform stays silent until the
atlas holds that edge kind. Reporting `map` as unpublished because no layer in the atlas
can state a runtime broadcast would be wrong on every ROS 2 stack, not only on this one.
The same reading applies to a driver that broadcasts its own frames: a node configured
with a transform publish flag publishes the frames it is configured with, so a repository
that does not declare them is not missing anything.

Guards. A broadcast written under a runtime guard (`condition` on its facts) counts as a
publisher and as a tie in every rule here. rosio never evaluates the guard, so it cannot
claim that nothing publishes a frame or that nothing ties two frames, and silence claims
nothing. A finding that does fire states the guard of a tie it relies on, with its file.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping, Sequence
from typing import Any

from rosio import ids, model
from rosio.drift import plugins, rules
from rosio.errors import ContractError
from rosio.model import Atlas, Finding, LayerFact, Source

WORLD_FRAMES = frozenset({"earth", "map", "odom", "utm", "world"})
"""REP 105 names earth, map and odom as the frames a running system owns, and utm and
world are the two other conventional world frames. A robot model is not expected to
declare any of them, so a rule that reads a robot model never reports them."""

STATIC_TF_EXECUTABLE = "static_transform_publisher"
ROBOT_STATE_PUBLISHER = "robot_state_publisher"
ROBOT_DESCRIPTION = "robot_description"

TF_PUBLISH_FLAGS = frozenset(
    {
        "broadcast_tf",
        "pub_static_tf",
        "publish_tf",
        "publish_transform",
        "tf",
        "tf_broadcast",
        "tf_send_en",
    }
)
"""Last segments of the boolean a driver carries to say it broadcasts its own transforms.
`tf_send_en` is the `publish.tf_send_en` the ROS 2 Point-LIO ports declare.
Set true, the node publishes the frames its own configuration names, so no robot model is
expected to declare them and no lookup through them fails. The value is read as well as
the name: the same key set false is the opposite statement."""

_PARENT_FLAGS = ("--frame-id", "--frame_id")
_CHILD_FLAGS = ("--child-frame-id", "--child_frame_id")

_CHILD_FIRST = frozenset({"urdf"})
"""tf_parent sources whose edge runs from the child to the parent."""

_PARENT_FIRST = frozenset({"cpp_broadcast", "py_broadcast"})
"""tf_parent sources whose edge runs from the parent to the child. The two collectors
disagree about the direction on this stack, so direction is read from the fact that
states the source and never assumed from the edge."""

_ENDPOINT_KINDS = (
    model.EdgeKind.PUBLISHES.value,
    model.EdgeKind.SUBSCRIBES.value,
    model.EdgeKind.SERVES.value,
    model.EdgeKind.CALLS.value,
)


# ---------------------------------------------------------------------------
# shared reading


def _disc(*parts: object) -> str:
    """A digest discriminator, prefixed so it can never read as an ordinal."""
    return "h" + ids.digest(*parts)


def _frame_text(frame_id: str) -> str:
    return ids.parse_frame(frame_id)


def _frame_name(value: object) -> str:
    """The frame a value names, or an empty string when it does not name one."""
    if not isinstance(value, str):
        return ""
    text = value.strip()
    if not text or any(mark in text for mark in ("$", "{", " ", "\\")):
        return ""
    try:
        return ids.normalise_frame(text)
    except ContractError:  # a value that is not a frame id is not a frame reference
        return ""


class _Tf:
    """One tf_parent statement, with the direction its own source states."""

    __slots__ = ("child", "edge", "fact", "model", "parent", "source")

    def __init__(self, edge: model.Edge, fact: LayerFact) -> None:
        source = str(fact.values.get("strategy") or fact.values.get("source") or "")
        self.edge = edge
        self.fact = fact
        self.source = source
        self.model = str(fact.values.get("model") or "")
        if source in _PARENT_FIRST:
            self.parent, self.child = edge.src, edge.dst
        else:
            self.child, self.parent = edge.src, edge.dst

    @property
    def directed(self) -> bool:
        return self.source in _CHILD_FIRST or self.source in _PARENT_FIRST

    @property
    def profile(self) -> str:
        return self.fact.profile


def _tf_rows(atlas: Atlas) -> list[_Tf]:
    rows = []
    for handle in sorted(atlas.edges):
        edge = atlas.edges[handle]
        if edge.kind != model.EdgeKind.TF_PARENT.value:
            continue
        for fact in edge.facts:
            if rules.usable(fact):
                rows.append(_Tf(edge, fact))
    return rows


class _Static:
    """A static_transform_publisher, read off the argv the launch layer recorded.

    The argv is a fact, so reading it is reading the atlas. Nothing is minted from it: it
    only says that a pair of frames is tied, which is the difference between a true and a
    false connectivity finding on this stack.
    """

    __slots__ = ("child", "node", "parent", "profile")

    def __init__(self, node: str, profile: str, parent: str, child: str) -> None:
        self.node = node
        self.profile = profile
        self.parent = parent
        self.child = child


def _static_pairs(atlas: Atlas) -> list[_Static]:
    out: list[_Static] = []
    for node in atlas.by_kind(model.NodeKind.NODE.value):
        for fact in node.facts:
            if not rules.usable(fact) or fact.values.get("executable") != STATIC_TF_EXECUTABLE:
                continue
            parent, child = _static_frames(fact.values.get("argv") or [])
            if parent and child:
                out.append(_Static(node.id, fact.profile, parent, child))
    return out


def _static_frames(argv: Sequence[Any]) -> tuple[str, str]:
    words = [str(item) for item in argv]
    parent = child = ""
    for index, word in enumerate(words[:-1]):
        if word in _PARENT_FLAGS:
            parent = _frame_name(words[index + 1])
        elif word in _CHILD_FLAGS:
            child = _frame_name(words[index + 1])
    if not parent and not child and len(words) in (8, 9):
        # The positional form ends with the frame pair, tf2_ros before the flag form.
        parent, child = _frame_name(words[-2]), _frame_name(words[-1])
    return parent, child


def _rsp_profiles(atlas: Atlas) -> set[str]:
    """Profiles that run a robot_state_publisher, which is what publishes urdf joints."""
    out: set[str] = set()
    for node in atlas.by_kind(model.NodeKind.NODE.value):
        for fact in node.facts:
            if rules.usable(fact) and fact.values.get("executable") == ROBOT_STATE_PUBLISHER:
                out.add(fact.profile)
    return out


def _urdf_frames(atlas: Atlas) -> dict[str, list[str]]:
    """Frame name to the sorted models that declare it as a link."""
    out: dict[str, list[str]] = {}
    for node in atlas.by_kind(model.NodeKind.FRAME.value):
        if not any(
            rules.usable(fact) and fact.values.get("declared_in_urdf") for fact in node.facts
        ):
            continue
        models = [str(item) for item in node.attrs.get("urdf_models") or []]
        out[ids.parse_frame(node.id)] = sorted(dict.fromkeys(models))
    return out


def _model_links(atlas: Atlas) -> dict[str, frozenset[str]]:
    """Model file id to the links it declares, which is what identifies one model."""
    out: dict[str, set[str]] = {}
    for frame, models in _urdf_frames(atlas).items():
        for model_id in models:
            out.setdefault(model_id, set()).add(frame)
    return {key: frozenset(value) for key, value in out.items()}


def _loaded_models(atlas: Atlas) -> dict[str, list[str]]:
    """Model file id to the sorted profiles that load it.

    The urdf collector attaches no profile to a model and no edge ties a launch entity to
    one, so the only per profile statement of the model a system loads is the
    `robot_description` the profile hands its state publisher, which carries the document
    itself. A document is recognised by the link names in it, and a link set that matches
    two models identifies neither, so nothing is guessed.
    """
    links = _model_links(atlas)
    if not links:
        return {}
    every = sorted(set().union(*links.values()))
    out: dict[str, set[str]] = {}
    for node in atlas.by_kind(model.NodeKind.PARAM.value):
        if ids.parse_param(node.id).name.rsplit(".", 1)[-1] != ROBOT_DESCRIPTION:
            continue
        for profile in sorted(node.effective):
            entry = node.effective[profile].get("value")
            if entry is None or entry.confidence == model.Confidence.UNKNOWN.value:
                continue
            if not isinstance(entry.value, str):
                continue
            stated = frozenset(name for name in every if f'name="{name}"' in entry.value)
            matched = [key for key in sorted(links) if links[key] == stated]
            if len(matched) == 1:
                out.setdefault(matched[0], set()).add(profile)
    return {key: sorted(out[key]) for key in sorted(out)}


def _self_publishers(atlas: Atlas) -> set[tuple[str, str]]:
    """Profile and owner pairs whose node states it broadcasts its own transforms."""
    out: set[tuple[str, str]] = set()
    for node in atlas.by_kind(model.NodeKind.PARAM.value):
        owner, name = ids.parse_param(node.id)
        if name.rsplit(".", 1)[-1] not in TF_PUBLISH_FLAGS:
            continue
        if node.state != model.ParamState.EFFECTIVE.value:
            continue
        for profile in sorted(node.effective):
            entry = node.effective[profile].get("value")
            if entry is None or entry.confidence == model.Confidence.UNKNOWN.value:
                continue
            if entry.value is True:
                out.add((profile, owner))
    return out


def _bound_names(atlas: Atlas) -> set[str]:
    """Interface names that carry an endpoint, so their kind is stated after the join."""
    out: set[str] = set()
    for handle in sorted(atlas.edges):
        edge = atlas.edges[handle]
        if edge.kind in _ENDPOINT_KINDS and any(_stated(fact) for fact in edge.facts):
            out.add(edge.dst)
    return out


def _stated(fact: LayerFact) -> bool:
    """A fact that binds a name, wrapper endpoints included.

    A wrapper endpoint is never reported as an endpoint, and it still states the kind of
    the name it binds. Dropping it here would let a rule say nothing names a topic that
    the wrapper table names.
    """
    return fact.confidence != model.Confidence.UNKNOWN.value


def _selected_plugin(atlas: Atlas, param_id: str) -> bool:
    """False when the parameter is a sub key of a plugin instance that is not selected."""
    owner, name = ids.parse_param(param_id)
    head, sep, _rest = name.partition(".")
    if not sep:
        return True
    prefix = f"{model.NodeKind.PINST.value}{ids.KIND_SEP}"
    known = [
        entity_id
        for entity_id in atlas.entities
        if entity_id.startswith(prefix) and ids.parse_pinst(entity_id).owner == owner
    ]
    if not known:
        return True
    return ids.build_pinst(owner, head) in atlas.entities


class _Ref:
    """One frame name a parameter states, with the profile and the line that states it."""

    __slots__ = ("frame", "owner", "param", "profile", "source")

    def __init__(self, param: str, owner: str, profile: str, frame: str, source: Source) -> None:
        self.param = param
        self.owner = owner
        self.profile = profile
        self.frame = frame
        self.source = source


def _config_frames(atlas: Atlas) -> list[_Ref]:
    """Every frame a parameter value names, per profile, with its winning source line."""
    out: list[_Ref] = []
    for node in atlas.by_kind(model.NodeKind.PARAM.value):
        if node.state != model.ParamState.EFFECTIVE.value:
            continue
        owner, name = ids.parse_param(node.id)
        if not name.split(".")[-1].endswith(("frame", "frame_id")):
            continue
        if not _selected_plugin(atlas, node.id):
            continue
        for profile in sorted(node.effective):
            entry = node.effective[profile].get("value")
            if entry is None or entry.confidence == model.Confidence.UNKNOWN.value:
                continue
            frame = _frame_name(entry.value)
            if not frame or entry.source is None:
                continue
            out.append(_Ref(node.id, owner, profile, frame, entry.source))
    return out


# ---------------------------------------------------------------------------
# frame.parent_differs


@rules.register("frame.parent_differs")
def frame_parent_differs(atlas: Atlas) -> list[Finding]:
    """One child frame placed differently by two robot models a profile loads.

    Three models of one robot live here and the same joint carries a different origin in
    each, which is the case this rule exists for. Only the models some profile loads are
    compared: a model no profile loads is a vehicle nobody is building, and the geometry
    of a second vehicle model is not drift against the geometry of the first. When the
    atlas ties no model to a profile the rule says nothing, because it cannot then tell
    the two apart. The finding keys on the sorted model list, never on the values, so a
    difference that turns out to be deliberate is waived once and stays waived when either
    side is edited.
    """
    loaded = _loaded_models(atlas)
    if not loaded:
        return []
    grouped: dict[tuple[str, str], list[_Tf]] = {}
    for row in _tf_rows(atlas):
        if row.source not in _CHILD_FIRST or row.model not in loaded:
            continue
        grouped.setdefault((row.profile, row.child), []).append(row)
    findings: list[Finding] = []
    for profile, child in sorted(grouped):
        rows = grouped[(profile, child)]
        placements = {_placement(row): row for row in rows}
        models = sorted({row.model for row in rows})
        if len(placements) < 2 or len(models) < 2:
            continue
        if child not in atlas.entities:
            continue
        parents = sorted({row.parent for row in rows})
        joints = sorted({str(row.fact.values.get("joint") or "") for row in rows} - {""})
        findings.append(
            Finding(
                rule="frame.parent_differs",
                severity=rules.severity_of("frame.parent_differs"),
                subject=child,
                title=_differs_title(child, parents, models),
                detail=_differs_detail(child, rows, loaded),
                fix=(
                    "edit the joint in one of the models so both state one geometry, or waive "
                    "this finding if the two profiles really run two vehicles"
                ),
                say=_differs_say(child, rows, parents, joints, loaded),
                discriminator=_disc(profile, *models),
                profiles=sorted({item for name in models for item in loaded[name]}),
                layers=sorted({row.fact.layer for row in rows}),
                where=[source for row in rows for source in row.fact.evidence],
            )
        )
    return findings


def _placement(row: _Tf) -> tuple[str, str, str]:
    return (row.parent, _vec(row.fact.values.get("xyz")), _vec(row.fact.values.get("rpy")))


def _vec(value: object) -> str:
    if not isinstance(value, (list, tuple)):
        return ""
    return " ".join(_number(item) for item in value)


def _number(value: object) -> str:
    if isinstance(value, float):
        text = f"{value:.6f}".rstrip("0").rstrip(".")
        return text or "0"
    return str(value)


def _differs_title(child: str, parents: Sequence[str], models: Sequence[str]) -> str:
    name = _frame_text(child)
    if len(parents) > 1:
        return f"{name} hangs off a different parent in {len(models)} robot models"
    return f"{name} sits at a different origin in {len(models)} robot models"


def _loaded_by(model_id: str, loaded: dict[str, list[str]]) -> str:
    """Which profiles load one model, in prose."""
    slugs = [rules.slug_of(profile) for profile in loaded.get(model_id, [])]
    if not slugs:
        return "loaded by no profile"
    if len(slugs) == 1:
        return f"loaded by profile {slugs[0]}"
    return "loaded by profiles " + ", ".join(slugs[:-1]) + f" and {slugs[-1]}"


def _differs_detail(child: str, rows: Sequence[_Tf], loaded: dict[str, list[str]]) -> str:
    lines = []
    for row in sorted(rows, key=lambda item: (item.model, item.parent)):
        lines.append(
            f"{rules.path_of(row.model)} ({_loaded_by(row.model, loaded)}) puts "
            f"{_frame_text(child)} under {_frame_text(row.parent)} at xyz "
            f"[{_vec(row.fact.values.get('xyz'))}] rpy [{_vec(row.fact.values.get('rpy'))}]"
        )
    return ". ".join(lines) + ". One edit reaches one model, so the others keep the old geometry."


def _differs_pair(rows: Sequence[_Tf], parents: Sequence[str]) -> tuple[_Tf, _Tf]:
    """The two rows the sentence quotes: the first model and the first one that differs.

    Three models state some of these joints and the first and the last of them can agree
    while the middle one differs, so a sentence built from the ends would quote one value
    twice and read as a claim that two equal numbers are not equal.
    """
    ordered = sorted(rows, key=lambda item: (item.model, item.parent))
    first = ordered[0]
    by_parent = len(parents) > 1
    for row in ordered[1:]:
        if by_parent and row.parent != first.parent:
            return first, row
        if not by_parent and _placement(row) != _placement(first):
            return first, row
    return first, ordered[-1]


def _differs_say(
    child: str,
    rows: Sequence[_Tf],
    parents: Sequence[str],
    joints: Sequence[str],
    loaded: dict[str, list[str]],
) -> str:
    first, last = _differs_pair(rows, parents)
    joint = joints[0] if len(joints) == 1 else "the joint"
    name = _frame_text(child)
    here = f"{rules.path_of(first.model)} ({_loaded_by(first.model, loaded)})"
    there = f"{rules.path_of(last.model)} ({_loaded_by(last.model, loaded)})"
    if len(parents) > 1:
        return (
            f"{joint} puts {name} under {_frame_text(first.parent)} in {here} and under "
            f"{_frame_text(last.parent)} in {there}, so those profiles run different robots."
        )
    return (
        f"{joint} puts {name} at xyz [{_vec(first.fact.values.get('xyz'))}] in {here} and at "
        f"xyz [{_vec(last.fact.values.get('xyz'))}] in {there}, so edit one of the two files."
    )


# ---------------------------------------------------------------------------
# frame.orphan_root


@rules.register("frame.orphan_root")
def frame_orphan_root(atlas: Atlas) -> list[Finding]:
    """A group of frames that no transform ties to the rest of the tree.

    Connectivity is undirected on purpose, because a static publisher, a urdf joint and a
    code broadcast are three sources with three conventions, and a lookup fails when the
    pair is missing whichever way round it is stated. Only frames that some transform
    names are considered, so a frame that exists only in a yaml value is the business of
    frame.not_in_urdf and never of this rule.
    """
    links: list[tuple[str, str]] = []
    children: set[str] = set()
    sources: dict[str, set[str]] = {}
    played = _in_play(atlas, _tf_rows(atlas))
    for row in played:
        links.append((row.child, row.parent))
        for frame in (row.child, row.parent):
            sources.setdefault(frame, set()).add(row.model)
        if row.directed:
            children.add(row.child)
    for pair in _static_pairs(atlas):
        child, parent = ids.build_frame(pair.child), ids.build_frame(pair.parent)
        links.append((child, parent))
        for frame in (child, parent):
            sources.setdefault(frame, set()).add(SYSTEM_SOURCE)
        children.add(child)
    if not links:
        return []
    groups = _components(links)
    if len(groups) < 2:
        return []
    findings: list[Finding] = []
    for family in _families(groups, sources):
        home, *others = family
        anchor = _root_of(home, children)
        for group in others:
            found = _orphan_finding(atlas, group, home, anchor, children, _guarded(played, group))
            if found is not None:
                findings.append(found)
    return findings


def _in_play(atlas: Atlas, rows: list[_Tf]) -> list[_Tf]:
    """The transforms a system can actually hold.

    A robot model that only sits in an installed tree is no second tree of anything unless a
    profile loads it: on `/opt/ros/jazzy` the simulation turtlebot and a vendored test model
    were reported as orphans of a profile that loads no model at all. A model in the
    workspace's own sources is always in, because a broken tree there is the user's to fix
    whether a profile loads it yet or not. A transform only a test states is never in.
    """
    loaded = _loaded_models(atlas)
    return [
        row
        for row in rows
        if not _from_tests(row)
        and (
            row.model == SYSTEM_SOURCE
            or row.model in loaded
            or any(source.root != INSTALL_ROOT for source in row.fact.evidence)
        )
    ]


TEST_DIRECTORIES = frozenset({"test", "tests"})


def _from_tests(row: _Tf) -> bool:
    """A transform only a test file states. Test code broadcasts the frames its fixture
    needs, and those are no part of the system the profiles start."""
    files = [source.file for source in row.fact.evidence]
    if not files:
        return False
    for file_id in files:
        try:
            relpath = ids.parse_file(file_id).relpath
        except ContractError:
            return False
        if not TEST_DIRECTORIES & set(relpath.split("/")[:-1]):
            return False
    return True


def _guarded(rows: Sequence[_Tf], group: Sequence[str]) -> list[tuple[str, str, str, Source]]:
    """Ties inside one group that hold only under a runtime guard, with the guard's site.

    The guard is never evaluated. A tie still connects, and the sentence says it holds only
    when the guard does. A pair any unguarded statement also ties is left out.
    """
    members = set(group)
    plain = {(row.parent, row.child) for row in rows if not row.fact.values.get("condition")}
    out: dict[tuple[str, str, str], Source] = {}
    for row in rows:
        guard = str(row.fact.values.get("condition") or "")
        pair = (row.parent, row.child)
        if not guard or row.child not in members or pair in plain or not row.fact.evidence:
            continue
        out.setdefault((*pair, guard), _guard_site(row.fact))
    return [(*key, out[key]) for key in sorted(out)]


def _guard_site(fact: LayerFact) -> Source:
    """The evidence row inside the broadcasting symbol, which is where the guard is written."""
    try:
        qualname = ids.parse_sym(str(fact.values.get("symbol") or "")).qualname
    except ContractError:
        qualname = ""
    return next((row for row in fact.evidence if row.locator == qualname), fact.evidence[0])


def _guard_text(guarded: Sequence[tuple[str, str, str, Source]], site: Callable) -> str:
    return ", and ".join(
        f"{_frame_text(parent)} to {_frame_text(child)} is broadcast only when {guard}, "
        f"in {site(source)}"
        for parent, child, guard, source in guarded
    )


def _orphan_finding(
    atlas: Atlas,
    group: Sequence[str],
    home: Sequence[str],
    anchor: str,
    children: set[str],
    guarded: Sequence[tuple[str, str, str, Source]] = (),
) -> Finding | None:
    subject = _root_of(group, children)
    if subject not in atlas.entities:
        return None
    names = ", ".join(_frame_text(frame) for frame in group)
    # The detail is fingerprinted, so it names the file and leaves the line to the say.
    noted = _guard_text(guarded, lambda source: rules.path_of(source.file))
    spoken = _guard_text(guarded, rules.where)
    return Finding(
        rule="frame.orphan_root",
        severity=rules.severity_of("frame.orphan_root"),
        subject=subject,
        title=f"{_frame_text(subject)} is the root of a second transform tree",
        detail=(
            f"{names} are tied to each other and to nothing else, while "
            f"{_frame_text(anchor)} carries the {len(home)} frame tree the rest of "
            "the system uses. A lookup across the two roots fails at run time."
            + (f" Of those ties, {noted}." if noted else "")
        ),
        fix=(
            f"publish a transform between {_frame_text(subject)} and a frame of the "
            "main tree, or correct the frame name that was meant to join them"
        ),
        say=(
            f"{_frame_text(subject)} is not connected to "
            f"{_frame_text(anchor)} by any transform this atlas holds, so a lookup "
            "between them fails" + (f", and {spoken}." if spoken else ".")
        ),
        discriminator=_disc(*group),
        layers=sorted(_layers_of(atlas, group)),
        where=_frame_sources(atlas, group),
    )


INSTALL_ROOT = "install"

SYSTEM_SOURCE = ""
"""The source of a transform that belongs to the running system rather than to one robot
model: a code broadcast, a launch argument, a static publisher. Its empty model says so."""


def _families(
    groups: Sequence[Sequence[str]], sources: dict[str, set[str]]
) -> list[list[Sequence[str]]]:
    """Connected frame groups, gathered per robot, the largest group of each family first.

    An install tree holds the description of every robot its packages ship, a turtlebot
    beside a test arm, and each is a tree of its own by design: two robots are not one robot
    with a broken tree. Groups that share a robot model are one family, so a model whose own
    joints leave a second root is still reported. What the running system links (a code
    broadcast, a static publisher) belongs to the family of the largest tree, which is the
    tree that system uses.
    """
    ordered = sorted(groups, key=lambda group: (-len(group), group[0]))
    models = [set().union(*(sources.get(frame, set()) for frame in group)) for group in ordered]
    family_of = list(range(len(ordered)))

    def find(index: int) -> int:
        while family_of[index] != index:
            index = family_of[index]
        return index

    for first in range(len(ordered)):
        for second in range(first + 1, len(ordered)):
            shared = models[first] & models[second]
            joins_system = first == 0 and SYSTEM_SOURCE in models[second]
            if shared - {SYSTEM_SOURCE} or joins_system:
                family_of[find(second)] = find(first)
    families: dict[int, list[Sequence[str]]] = {}
    for index, group in enumerate(ordered):
        families.setdefault(find(index), []).append(group)
    return [members for _, members in sorted(families.items())]


def _root_of(group: Sequence[str], children: set[str]) -> str:
    """The frame of a group that is nobody's child, or the first one when that is unclear."""
    roots = [frame for frame in group if frame not in children]
    return roots[0] if len(roots) == 1 else group[0]


def _components(links: Iterable[tuple[str, str]]) -> list[list[str]]:
    parent: dict[str, str] = {}

    def find(item: str) -> str:
        parent.setdefault(item, item)
        while parent[item] != item:
            parent[item] = parent[parent[item]]
            item = parent[item]
        return item

    for left, right in links:
        a, b = find(left), find(right)
        if a != b:
            parent[max(a, b)] = min(a, b)
    groups: dict[str, list[str]] = {}
    for item in sorted(parent):
        groups.setdefault(find(item), []).append(item)
    return [sorted(group) for _root, group in sorted(groups.items())]


def _layers_of(atlas: Atlas, frames: Sequence[str]) -> set[str]:
    out: set[str] = set()
    for frame in frames:
        node = atlas.entities.get(frame)
        if node is not None:
            out.update(node.layers)
    return out


def _frame_sources(atlas: Atlas, frames: Sequence[str]) -> list[Source]:
    out: list[Source] = []
    for frame in frames:
        node = atlas.entities.get(frame)
        if node is None:
            continue
        for fact in node.facts:
            out.extend(fact.evidence)
    return out


# ---------------------------------------------------------------------------
# frame.not_in_urdf


LOOKED_UP = "looked_up"
"""The variant of `frame.not_in_urdf` for a frame a source looks up by its literal name. A
lookup of a frame nothing states fails on every call, which is louder than a configured value
nobody may read, so it is graded one step up."""


def _looked_up_not_in_urdf(
    atlas: Atlas, declared: Mapping[str, Any], tied: set[str]
) -> list[Finding]:
    """Frames the code asks a transform for by name that no model, no transform and no
    configuration states anywhere: almost always a typo of a link the model has."""
    sites: dict[str, list[tuple[str, model.LayerFact]]] = {}
    for handle in sorted(atlas.edges):
        edge = atlas.edges[handle]
        if edge.kind != model.EdgeKind.LISTENS_TF.value:
            continue
        frame = ids.parse_frame(edge.dst)
        if frame in declared or frame in tied or frame in WORLD_FRAMES:
            continue
        for fact in edge.facts:
            role = str(fact.values.get("role") or "")
            if not rules.usable(fact) or fact.values.get(f"{role}_from_parameter"):
                continue
            if fact.evidence:
                sites.setdefault(frame, []).append((edge.src, fact))
    findings: list[Finding] = []
    for frame, rows in sorted(sites.items()):
        nodes = sorted({rules.node_text(node) for node, _fact in rows})
        lead = rows[0][1]
        site = rules.where(lead.evidence[0])
        near = _nearest(frame, declared)
        hint = f", the model has {near}" if near else ""
        findings.append(
            Finding(
                rule="frame.not_in_urdf",
                severity=rules.severity_of("frame.not_in_urdf", LOOKED_UP),
                subject=ids.build_frame(frame),
                title=f"{', '.join(nodes)} looks up {frame}, which no robot model has",
                detail=(
                    f"{', '.join(nodes)} asks for a transform to {frame} by name at {site}. None "
                    f"of the {len(declared)} links the robot models declare carries that name and "
                    "no transform in this atlas names it, so the lookup fails on every call"
                    f"{hint}."
                ),
                fix=f"correct the frame name at {site}{hint}, or declare {frame} in the model",
                say=(
                    f"{site} looks up the frame {frame}, which no robot model declares and nothing "
                    f"publishes, so the lookup never succeeds{hint}."
                ),
                discriminator=_disc(frame, LOOKED_UP),
                profiles=[],
                layers=sorted({fact.layer for _node, fact in rows}),
                where=rules.sources([row for _node, fact in rows for row in fact.evidence]),
            )
        )
    return findings


def _nearest(frame: str, declared: Mapping[str, Any]) -> str:
    """The declared link a mistyped frame most likely meant, or nothing when none is close."""
    import difflib

    match = difflib.get_close_matches(frame, sorted(declared), n=1, cutoff=0.5)
    return match[0] if match else ""


@rules.register("frame.not_in_urdf")
def frame_not_in_urdf(atlas: Atlas) -> list[Finding]:
    """A frame a configuration file names that no robot model declares and nothing publishes.

    Grouped by owner and frame, so four septentrio keys naming one frame are one finding
    with four cited lines rather than four findings. A frame that any transform in the
    atlas names is skipped, and so is every REP 105 world frame, because neither is a link
    the robot model is expected to own.

    Two more silences, both about the consequence the finding asserts rather than about the
    claim. A driver whose configuration says it broadcasts its own transforms publishes the
    frame it is given, so the lookup resolves and there is nothing to declare. And a value
    the atlas can place in no line was delivered by a site rather than written there: on
    this stack that is a launch file loading a vendor defaults document out of the ros
    share tree, and naming the launch file sends an engineer to a file the value is not in.
    A finding that cannot say where the frame id came from is not a finding.
    """
    declared = _urdf_frames(atlas)
    if not declared:
        return []
    publishers = _self_publishers(atlas)
    tied: set[str] = set()
    for row in _tf_rows(atlas):
        tied.update({ids.parse_frame(row.child), ids.parse_frame(row.parent)})
    for pair in _static_pairs(atlas):
        tied.update({pair.parent, pair.child})
    for handle in sorted(atlas.edges):
        edge = atlas.edges[handle]
        if edge.kind == model.EdgeKind.BROADCASTS_TF.value and any(
            rules.usable(fact) for fact in edge.facts
        ):
            tied.add(ids.parse_frame(edge.dst))
    grouped: dict[tuple[str, str], list[_Ref]] = {}
    for ref in _config_frames(atlas):
        if ref.frame in declared or ref.frame in tied or ref.frame in WORLD_FRAMES:
            continue
        if (ref.profile, ref.owner) in publishers or ref.source.line_hint is None:
            continue
        grouped.setdefault((ref.owner, ref.frame), []).append(ref)
    findings: list[Finding] = _looked_up_not_in_urdf(atlas, declared, tied)
    for key in sorted(grouped):
        owner, frame = key
        refs = sorted(grouped[key], key=lambda item: (item.param, item.profile))
        lead = refs[0]
        subject = lead.param
        keys = sorted({ids.parse_param(ref.param).name for ref in refs})
        site = rules.where(lead.source)
        findings.append(
            Finding(
                rule="frame.not_in_urdf",
                severity=rules.severity_of("frame.not_in_urdf"),
                subject=subject,
                title=f"{rules.node_text(owner)} names the frame {frame}, which no robot model has",
                detail=(
                    f"{', '.join(keys)} on {rules.node_text(owner)} "
                    f"{'resolves' if len(keys) == 1 else 'resolve'} to {frame}, written "
                    f"in {rules.path_of(lead.source.file)}. None of the {len(declared)} links "
                    "the robot models declare "
                    "carries that name, no transform in this atlas names it, and no parameter "
                    "of that node says it broadcasts its own transforms, so the only statement "
                    "of that frame is the configuration line."
                ),
                fix=(
                    f"declare {frame} in the robot model, or correct the value to a link the "
                    "model already has"
                ),
                say=(
                    f"{site} sets {ids.parse_param(lead.param).name} to {frame}, no robot model "
                    "declares a link with that name and nothing in this atlas publishes it, so "
                    "a lookup against the robot body cannot be resolved."
                ),
                discriminator=_disc(frame, *sorted({ref.param for ref in refs})),
                profiles=sorted({ref.profile for ref in refs} - {model.ANY_PROFILE}),
                layers=[model.Layer.DECLARED.value],
                where=[ref.source for ref in refs],
            )
        )
    return findings


# ---------------------------------------------------------------------------
# frame.no_publisher


@rules.register("frame.no_publisher")
def frame_no_publisher(atlas: Atlas) -> list[Finding]:
    """A frame something needs and nothing broadcasts, inside one profile.

    Gated on the atlas holding at least one node to frame broadcast edge. Without that
    edge kind the only honest answer about who publishes a transform is that the atlas
    does not know, and a rule that answers anyway reports every runtime frame on every
    stack. A REP 105 world frame is skipped for the same reason: it is published by a
    localisation node whose broadcast a static read cannot see.
    """
    broadcast: dict[str, set[str]] = {}
    broadcasters: dict[str, set[str]] = {}
    for handle in sorted(atlas.edges):
        edge = atlas.edges[handle]
        if edge.kind != model.EdgeKind.BROADCASTS_TF.value:
            continue
        for fact in edge.facts:
            if rules.usable(fact):
                broadcast.setdefault(fact.profile, set()).add(edge.dst)
                broadcasters.setdefault(fact.profile, set()).add(edge.src)
    if not broadcast:
        return []
    published = _published_frames(atlas, broadcast)
    wanted = _wanted_frames(atlas)
    blind = rules.blind_files(atlas, model.Want.FRAME_ID.value)
    # A broadcaster whose code holds a frame name rosio could not read may publish any frame.
    unseen = {
        profile
        for profile, nodes in broadcasters.items()
        if any(rules.coded_sources(atlas, node, profile) & blind for node in nodes)
    }
    findings: list[Finding] = []
    for key in sorted(wanted):
        profile, frame = key
        if profile not in broadcast or profile in unseen:
            continue
        if ids.parse_frame(frame) in WORLD_FRAMES:
            continue
        if frame in published.get(profile, set()) or frame in published.get(
            model.ANY_PROFILE, set()
        ):
            continue
        rows = wanted[key]
        findings.append(
            Finding(
                rule="frame.no_publisher",
                severity=rules.severity_of("frame.no_publisher"),
                subject=frame,
                title=f"nothing publishes {_frame_text(frame)} in this profile",
                detail=(
                    f"{', '.join(sorted({row[0] for row in rows}))} needs "
                    f"{_frame_text(frame)}, and no node in this profile broadcasts it, no "
                    "robot model joint carries it and no static publisher ties it."
                ),
                fix=(
                    f"start the node that broadcasts {_frame_text(frame)} in this profile, or "
                    "point the configuration at the frame that is published"
                ),
                say=(
                    f"{_frame_text(frame)} is needed in this profile and nothing in it "
                    "publishes that transform, so every lookup through it fails."
                ),
                discriminator=_disc(profile),
                profiles=[profile] if profile != model.ANY_PROFILE else [],
                layers=[model.Layer.DECLARED.value],
                where=[row[1] for row in rows if row[1] is not None],
            )
        )
    return findings


def _published_frames(atlas: Atlas, broadcast: dict[str, set[str]]) -> dict[str, set[str]]:
    """Who publishes what, per profile.

    A urdf joint is published by the robot_state_publisher of the profile that loads the
    model, and the urdf collector states its facts for every profile at once. A profile
    whose loaded model is identified gets that model's joints only, so an unused model
    elsewhere in the workspace cannot publish for it; a profile whose model nothing
    identifies gets every model's joints, because which one it loads is unknown there. A
    profile that runs no state publisher gets nothing from the models.
    """
    out: dict[str, set[str]] = {key: set(value) for key, value in broadcast.items()}
    rsp = sorted(_rsp_profiles(atlas))
    loaded = _loaded_models(atlas)
    identified = {profile for profiles in loaded.values() for profile in profiles}
    for row in _tf_rows(atlas):
        if row.source not in _CHILD_FIRST:
            out.setdefault(row.profile, set()).add(row.child)
            continue
        if row.profile == model.ANY_PROFILE:
            targets = [
                profile
                for profile in rsp
                if profile not in identified or profile in loaded.get(row.model, ())
            ]
        else:
            targets = [row.profile] if row.profile in rsp else []
        for profile in targets:
            out.setdefault(profile, set()).add(row.child)
    for pair in _static_pairs(atlas):
        out.setdefault(pair.profile, set()).add(ids.build_frame(pair.child))
    return out


def _wanted_frames(atlas: Atlas) -> dict[tuple[str, str], list[tuple[str, Source | None]]]:
    """Frames a profile needs: a listener, a configuration value or a transform parent."""
    out: dict[tuple[str, str], list[tuple[str, Source | None]]] = {}
    for handle in sorted(atlas.edges):
        edge = atlas.edges[handle]
        if edge.kind != model.EdgeKind.LISTENS_TF.value:
            continue
        for fact in edge.facts:
            if not rules.usable(fact):
                continue
            row = (rules.node_text(edge.src), fact.evidence[0] if fact.evidence else None)
            out.setdefault((fact.profile, edge.dst), []).append(row)
    for ref in _config_frames(atlas):
        frame_id = ids.build_frame(ref.frame)
        if frame_id in atlas.entities:
            out.setdefault((ref.profile, frame_id), []).append(
                (rules.node_text(ref.owner), ref.source)
            )
    return out


# ---------------------------------------------------------------------------
# remap.unresolved


@rules.register("remap.unresolved")
def remap_unresolved(atlas: Atlas) -> list[Finding]:
    """A remap whose destination kind nothing states.

    Reported once per remap written in one file, with the raw name exactly as that file
    writes it: one line read through eleven entry points or two robot namespaces is one
    line to look at. No topic, service or action id is minted here: the whole point of the
    blind spot is that nothing in the atlas says which of the three it is. A destination
    that the join has since bound to an endpoint is silent, and so is a destination a robot
    model the profile loads states, per profile: a name one profile's model publishes is
    still a blind spot in a profile that loads no model naming it. A launch site whose
    condition is false in a profile starts no node there, so its remap binds nothing.
    """
    bound = _bound_names(atlas)
    modelled = _model_names(atlas)
    sites: dict[tuple[str, str], list[tuple[model.Unresolved, list[str]]]] = {}
    for entry in sorted(atlas.unresolved, key=lambda item: item.id):
        if entry.role != "remap" or entry.owner not in atlas.entities:
            continue
        _source, sep, target = entry.expr.partition(":=")
        if not sep or not target:
            continue
        names = [
            f"{kind}{ids.KIND_SEP}{name}"
            for name in _remap_names(atlas, entry.owner, target)
            for kind in ("topic", "service", "action")
        ]
        if any(name in bound for name in names):
            continue
        owner = atlas.entities[entry.owner]
        owned = _owner_profiles(atlas, entry.owner)
        live = [profile for profile in owned if rules.site_live(owner, profile)]
        if owned and not live:
            continue
        stated = set().union(*(modelled.get(name, set()) for name in names))
        profiles = [profile for profile in live if profile not in stated]
        if not profiles and stated:
            continue
        file_id = entry.evidence[0].file if entry.evidence else ""
        subject = file_id if file_id in atlas.entities else entry.owner
        sites.setdefault((subject, entry.expr), []).append((entry, profiles))
    findings: list[Finding] = []
    for (subject, expr), group in sorted(sites.items()):
        source, _sep, target = expr.partition(":=")
        first = group[0][0]
        where = _base_name(subject) or _remap_label(first)
        launched = ", ".join(sorted({ids.label_for(entry.owner) for entry, _ in group}))
        findings.append(
            Finding(
                rule="remap.unresolved",
                severity=rules.severity_of("remap.unresolved"),
                subject=subject,
                title=f"nothing states what kind of endpoint the remap {source} to {target} binds",
                detail=(
                    f"{where} remaps {source} to {target} on {launched}. No source in the read "
                    "set publishes or subscribes either name and no interface definition names "
                    f"it, so nothing states whether {target} is a topic, a service or an "
                    "action, and the endpoint stays unbound."
                ),
                fix=(
                    f"read the node source for the endpoint behind {source}, or add the package "
                    "that owns it to the workspace so the kind is stated"
                ),
                say=(
                    f"{where} remaps {source} to {target} and nothing in the workspace states "
                    f"what kind of endpoint {target} is, so the connection is unverified."
                ),
                discriminator=_disc(expr),
                profiles=[profile for _, profiles in group for profile in profiles],
                layers=[model.Layer.DECLARED.value],
                where=[row for entry, _ in group for row in entry.evidence],
            )
        )
    return findings


def _launch_label(entity_id: str, file_id: str = "") -> str:
    if ids.kind_of(entity_id) != model.NodeKind.LAUNCHENT.value:
        return ids.label_for(entity_id)
    parsed = ids.parse_launchent(entity_id)
    tail = _base_name(file_id) or parsed.relpath.rsplit("/", 1)[-1]
    return f"{tail} {ids.label_for(entity_id)}"


def _base_name(file_id: str) -> str:
    if not file_id:
        return ""
    try:
        return ids.parse_file(file_id).relpath.rsplit("/", 1)[-1]
    except ContractError:
        return ""


def _remap_names(atlas: Atlas, owner: str, target: str) -> list[str]:
    """The absolute names a remap destination can resolve to, one per node it applies to.

    A launch file writes the destination the way ROS resolves it, relative to the namespace
    of the node that carries the remap, so comparing the text as written against a bound
    name misses every relative destination. Nothing is minted from the result: it decides
    whether another statement already names the endpoint.
    """
    out: list[str] = []
    scopes = [_node_scope(node_id) for node_id in _launched_by(atlas, owner)] or [("/", "")]
    for namespace, name in scopes:
        try:
            out.append(ids.normalise_ros_name(target, namespace or "/", name or None))
        except ContractError:
            continue
    return sorted(dict.fromkeys(out))


def _launched_by(atlas: Atlas, owner: str) -> list[str]:
    return sorted(
        {edge.dst for edge in atlas.out_edges(owner) if edge.kind == model.EdgeKind.LAUNCHES.value}
    )


def _node_scope(node_id: str) -> tuple[str, str]:
    parsed = ids.parse_node(node_id)
    if parsed.synthetic:
        return parsed.namespace or "/", ""
    return _namespace_of(node_id), (parsed.fqn or "").rsplit("/", 1)[-1]


def _remap_label(entry: model.Unresolved) -> str:
    """Which file the sentence names, which has to be the file that holds the remap.

    A launch entity id is built from the entry point, and an entry point that includes
    another launch file holds no remap of its own, so naming it sends a reader to a file
    with nothing to edit in it. The evidence row the entry already carries names the file
    the walk read the remap in, and the entry point is named only when there is none.
    """
    row = entry.evidence[0] if entry.evidence else None
    return _launch_label(entry.owner, row.file if row is not None else "")


def _model_names(atlas: Atlas) -> dict[str, set[str]]:
    """Interface names a robot model states, per profile that loads that model.

    A gazebo sensor block inside a urdf remaps its own output onto a name, which states that
    the name is a topic wherever that model runs. It states nothing in a profile that loads
    another model, and a model no profile loads states nothing at all, which is the same
    scoping `frame.parent_differs` uses.
    """
    loaded = _loaded_models(atlas)
    out: dict[str, set[str]] = {}
    for handle in sorted(atlas.edges):
        edge = atlas.edges[handle]
        if edge.kind != model.EdgeKind.REMAPS.value:
            continue
        if ids.kind_of(edge.src) != model.NodeKind.FILE.value:
            continue
        if not any(_stated(fact) for fact in edge.facts):
            continue
        for profile in loaded.get(edge.src, ()):
            out.setdefault(edge.dst, set()).add(profile)
    return out


# ---------------------------------------------------------------------------
# lifecycle.unmanaged


@rules.register("lifecycle.unmanaged")
def lifecycle_unmanaged(atlas: Atlas) -> list[Finding]:
    """Lifecycle wiring that does not close, inside one profile.

    Two shapes, one failure. A lifecycle node that no manager and no launch event handler
    drives never leaves the unconfigured state. A manager whose node_names entry names a
    node the profile never starts blocks in its bond wait and takes the rest of the stack
    with it.
    """
    managed = _managed(atlas)
    gated = _state_gated(atlas)
    findings = _unmanaged_nodes(atlas, managed, gated)
    findings.extend(_absent_managed_names(atlas))
    blind = _lifecycle_blind(atlas)
    return [item for item in findings if not blind.intersection(item.profiles)]


def _lifecycle_blind(atlas: Atlas) -> set[str]:
    """Profiles whose launch walk stopped early.

    Both shapes of the rule are claims about absence, a manager nobody started or a node a
    manager never gets, and the unread rest of the tree can hold either one. In nav2 examples
    whose walk stopped on a simulator file, the managers were read and every server was not.
    """
    return {
        profile
        for profile in rules.profiles_of(atlas)
        if plugins.unread_entry_points(atlas, profile)
    }


def silences(atlas: Atlas) -> list[dict[str, str]]:
    """The rules this module kept quiet over this atlas, and why, as the other families do."""
    return [
        {
            "rule": "lifecycle.unmanaged",
            "profile": profile,
            "reason": (
                "the launch tree of this profile was not read to the end ("
                + ", ".join(
                    plugins.launch_path(item)
                    for item in plugins.unread_entry_points(atlas, profile)
                )
                + "), so a manager or a managed node may be in the part not read"
            ),
        }
        for profile in sorted(_lifecycle_blind(atlas))
    ]


def _managed(atlas: Atlas) -> dict[str, set[str]]:
    out: dict[str, set[str]] = {}
    for handle in sorted(atlas.edges):
        edge = atlas.edges[handle]
        if edge.kind != model.EdgeKind.MANAGES_LIFECYCLE.value:
            continue
        for fact in edge.facts:
            if rules.usable(fact):
                out.setdefault(fact.profile, set()).add(edge.dst)
    return out


def _state_gated(atlas: Atlas) -> dict[str, set[str]]:
    """Nodes a launch file drives with its own transition events."""
    out: dict[str, set[str]] = {}
    for handle in sorted(atlas.edges):
        edge = atlas.edges[handle]
        if edge.kind != model.EdgeKind.GATES.value:
            continue
        for fact in edge.facts:
            if rules.usable(fact) and fact.values.get("mechanism") == "OnStateTransition":
                out.setdefault(fact.profile, set()).add(edge.dst)
    return out


def _unmanaged_nodes(
    atlas: Atlas, managed: dict[str, set[str]], gated: dict[str, set[str]]
) -> list[Finding]:
    findings: list[Finding] = []
    for node in atlas.by_kind(model.NodeKind.NODE.value):
        for fact in sorted(node.facts, key=LayerFact.sort_key):
            if not rules.usable(fact) or fact.values.get("form") != "lifecycle_node":
                continue
            profile = fact.profile
            if node.id in managed.get(profile, set()) or node.id in gated.get(profile, set()):
                continue
            if node.id in managed.get(model.ANY_PROFILE, set()):
                continue
            if _driven_by_a_peer(atlas, node, profile):
                continue
            has_manager = _has_manager(atlas, managed, profile)
            unread = [] if has_manager else _unread_peers(atlas, node.id, profile)
            if unread:
                findings.append(_possibly_driven(node, fact, unread))
                continue
            findings.append(
                Finding(
                    rule="lifecycle.unmanaged",
                    severity=rules.severity_of("lifecycle.unmanaged"),
                    subject=node.id,
                    title=f"{node.label} is a lifecycle node with no manager in this profile",
                    detail=(
                        f"{node.label} is declared as a lifecycle node, and this profile holds "
                        "no lifecycle manager that lists it and no launch event that emits its "
                        "transitions. It starts unconfigured and stays there."
                    ),
                    fix=(
                        f"add {node.label} to the node_names list of the manager that runs in "
                        "this profile, or emit its configure and activate events from launch"
                    ),
                    say=(
                        f"{node.label} is a lifecycle node and nothing in this profile "
                        "configures or activates it, so it never reaches the active state."
                    ),
                    discriminator=_disc(profile, "form"),
                    profiles=[profile] if profile != model.ANY_PROFILE else [],
                    layers=[fact.layer],
                    where=list(fact.evidence),
                )
            )
    return findings


def _driven_by_a_peer(atlas: Atlas, node: model.Node, profile: str) -> bool:
    """Whether another node this profile starts calls this node's `change_state` service.

    That is how a client drives a lifecycle node without a manager, and the coded layer
    states the call when it read the client's source. The caller has to be started by the
    profile, because code that is built and never launched drives nothing.
    """
    fqn = ids.parse_node(node.id).fqn
    if not fqn:
        return False
    service = ids.build_service(f"{fqn}/change_state")
    for edge in atlas.in_edges(service):
        if edge.kind != model.EdgeKind.CALLS.value or edge.src == node.id:
            continue
        caller = atlas.entities.get(edge.src)
        if caller is not None and any(
            rules.usable(fact) and fact.profile == profile for fact in caller.facts
        ):
            return True
    return False


def _has_manager(atlas: Atlas, managed: dict[str, set[str]], profile: str) -> bool:
    """Whether the profile runs a lifecycle manager, listing this node or not."""
    if managed.get(profile):
        return True
    return any(
        ids.parse_param(param.id).name == "node_names" and profile in param.profiles
        for param in atlas.by_kind(model.NodeKind.PARAM.value)
    )


def _unread_peers(atlas: Atlas, node_id: str, profile: str) -> list[str]:
    """Other nodes of the profile whose code rosio never read, by label.

    Any of them can call the lifecycle services of a node, which is how a small client
    drives a lifecycle demo, so a profile that runs one cannot be said to leave a node alone.
    """
    out = []
    for other in atlas.by_kind(model.NodeKind.NODE.value):
        if other.id == node_id or model.Layer.CODED.value in other.layers:
            continue
        if any(rules.usable(fact) and fact.profile == profile for fact in other.facts):
            out.append(other.label)
    return sorted(out)


def _possibly_driven(node: model.Node, fact: LayerFact, unread: list[str]) -> Finding:
    peers = ", ".join(unread[:3]) + (f" and {len(unread) - 3} more" if len(unread) > 3 else "")
    return Finding(
        rule="lifecycle.unmanaged",
        severity=rules.severity_of("lifecycle.unmanaged", "unproven"),
        subject=node.id,
        title=f"{node.label} is a lifecycle node no manager lists",
        detail=(
            f"{node.label} is declared as a lifecycle node, and this profile holds no lifecycle "
            "manager and no launch event that emits its transitions. Processes of this profile "
            "run code rosio did not read and may call its lifecycle services."
        ),
        fix=(
            f"if none of {peers} drives {node.label}, add it to the node_names list of a "
            "lifecycle manager or emit its configure and activate events from launch"
        ),
        say=(
            f"{node.label} is a lifecycle node that no manager or launch event in this profile "
            f"configures; {peers} run code rosio did not read and may drive it."
        ),
        discriminator=_disc(fact.profile, "form"),
        profiles=[fact.profile] if fact.profile != model.ANY_PROFILE else [],
        layers=[fact.layer],
        where=list(fact.evidence),
    )


def _absent_managed_names(atlas: Atlas) -> list[Finding]:
    findings: list[Finding] = []
    for node in atlas.by_kind(model.NodeKind.PARAM.value):
        owner, name = ids.parse_param(node.id)
        if name != "node_names" or node.state != model.ParamState.EFFECTIVE.value:
            continue
        namespace = _namespace_of(owner)
        for profile in sorted(node.effective):
            entry = node.effective[profile].get("value")
            if entry is None or entry.confidence == model.Confidence.UNKNOWN.value:
                continue
            if not isinstance(entry.value, (list, tuple)):
                continue
            missing = sorted(
                raw
                for raw in (str(item) for item in entry.value)
                if _never_started(atlas, raw, namespace, profile)
            )
            if not missing:
                continue
            findings.append(
                Finding(
                    rule="lifecycle.unmanaged",
                    severity=rules.severity_of("lifecycle.unmanaged"),
                    subject=node.id,
                    title=(
                        f"{rules.node_text(owner)} waits for {len(missing)} "
                        f"{'node' if len(missing) == 1 else 'nodes'} it never gets"
                    ),
                    detail=(
                        f"node_names on {rules.node_text(owner)} lists {', '.join(missing)}, and "
                        "this profile starts no node under that fully qualified name. The "
                        "manager blocks in its bond wait, so every node it does manage stays "
                        "unconfigured with it."
                    ),
                    fix=(
                        f"remove {missing[0]} from node_names in "
                        f"{rules.where(entry.source)}, or start it in this profile"
                    ),
                    say=(
                        f"{rules.where(entry.source)} lists {missing[0]} in node_names and "
                        f"{_profile_phrase(profile)} starts no such node, so the manager never "
                        "finishes activating."
                    ),
                    discriminator=_disc(profile, *missing),
                    profiles=[profile] if profile != model.ANY_PROFILE else [],
                    layers=[model.Layer.DECLARED.value],
                    where=[entry.source] if entry.source is not None else [],
                )
            )
    return findings


def _profile_phrase(profile: str) -> str:
    if profile == model.ANY_PROFILE:
        return "the workspace"
    return f"profile {rules.slug_of(profile)}"


def _owner_profiles(atlas: Atlas, entity_id: str) -> list[str]:
    node = atlas.entities.get(entity_id)
    return list(node.profiles) if node is not None else []


def _namespace_of(node_id: str) -> str:
    """The namespace a node_names entry is resolved against, from the id grammar."""
    parsed = ids.parse_node(node_id)
    if parsed.synthetic:
        return parsed.namespace
    return (parsed.fqn or "/").rsplit("/", 1)[0] or "/"


def _never_started(atlas: Atlas, raw: str, namespace: str, profile: str) -> bool:
    """A listed name this profile starts no node under.

    A value the name grammar cannot resolve (a substitution nothing replaced, an empty
    entry) says nothing about what the profile starts, so it is not reported as missing.
    """
    try:
        fqn = ids.normalise_ros_name(raw, namespace)
    except ContractError:
        return False
    return not _launched(atlas, fqn, profile)


def _launched(atlas: Atlas, fqn: str, profile: str) -> bool:
    node = atlas.entities.get(ids.build_node(fqn))
    if node is None:
        return False
    return any(
        rules.usable(fact) and fact.profile in (profile, model.ANY_PROFILE) for fact in node.facts
    )


# ---------------------------------------------------------------------------
# registry


RULES: dict[str, Callable[[Atlas], list[Finding]]] = {
    "frame.no_publisher": frame_no_publisher,
    "frame.not_in_urdf": frame_not_in_urdf,
    "frame.orphan_root": frame_orphan_root,
    "frame.parent_differs": frame_parent_differs,
    "lifecycle.unmanaged": lifecycle_unmanaged,
    "remap.unresolved": remap_unresolved,
}

RULE_IDS = tuple(sorted(RULES))


def collect(atlas: Atlas) -> list[Finding]:
    """Every rule in this module, in rule id order, then in finding sort order."""
    findings: list[Finding] = []
    for rule in RULE_IDS:
        findings.extend(RULES[rule](atlas))
    return sorted(findings, key=Finding.sort_key)


__all__ = [
    "RULES",
    "RULE_IDS",
    "WORLD_FRAMES",
    "collect",
    "frame_no_publisher",
    "frame_not_in_urdf",
    "frame_orphan_root",
    "frame_parent_differs",
    "lifecycle_unmanaged",
    "remap_unresolved",
    "silences",
]
