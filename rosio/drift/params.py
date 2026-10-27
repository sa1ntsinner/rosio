"""The parameter rule family: seven rules over the candidate chain the declared layer built
and the declarations the coded layer read.

Every rule here is scoped to one profile, skips facts rosio was not sure about, and skips
sub keys of a plugin block the yaml did not select, because those are configuration and not
drift. Four judgements decide what stays quiet, and each one was made against the reference
stack rather than against a preference.

  * An overlay file that changes a base file is what an overlay file is for. On the bench
    that is 28 of the 62 shadowed values, one file against its own base, and reporting them
    would bury the six that matter. `param.shadowed` is silent on that pair and ranks the
    rest by where the losing line lives: a value that loses inside a parameter file, the
    document an engineer opens to tune a number, is a warn, and a launch inline default that
    loses to a parameter file is an info. One file that sets one parameter twice with two
    values is an error, because no reading of the file makes both true.

  * A code side default that no file overrides is not drift. 26 parameters on the bench have
    only a `code_default` candidate, and all of them are ordinary nodes configured by their
    own defaults. Reporting them would fire on every well behaved node in every workspace,
    so nothing here does. `param.set_undeclared` runs the other direction, and only for a
    node whose source rosio actually read in that profile: a yaml line for a parameter no
    declaration names is a line that does nothing.

  * `use_sim_time` is declared by rclcpp and rclpy themselves, so a file setting it is never
    an undeclared write. It is the whole population of that case on the bench, 5 of 5. It is
    also the reason `param.restated_identical` is graded rather than flat: 10 of the 32
    restatements on the bench are `use_sim_time`, nobody will ever act on one, and putting
    them on the same line as a parameter document that restates what a launcher writes
    makes the loud case unreadable. `_restated_severity` carries that judgement.

  * `param.declared_unread` needs positive evidence that the value is unreachable.
    `declare_parameter` returns the value in rclcpp, so "no `get_parameter` call" is not
    that evidence: 53 parameters on the bench are read exclusively through the return of
    their own declaration, and firing on the absent read would be 53 false positives. The
    rule therefore asks for a declaration that states it returns nothing usable, and it is
    silent until a collector records that. Silence is the correct output here, not a
    weaker rule.

A wildcard yaml key and an inline `parameters=` dict expand one source line into N
parameters, so all four chain rules group by the site an engineer would edit. One deletable
line is one finding, whatever it expands to.

The two block rules, `param.orphan_block` and `param.owner_name_mismatch`, reason about who
owns a block, and both are gated on whether the owner could have been seen at all. A node
creates child nodes at run time: a costmap is a node inside its parent, and a behaviour tree
action server appends a suffix to its parent's name and starts a second node. The child is
in the parent's source and nowhere else, so on a tree with no sources the declared layer
alone cannot tell a block owned by such a child from a block owned by nothing. Asserting the
second reads as a finding that says delete a working configuration, which is what these two
rules did on stock nav2, four times. `owner_gate` therefore asks first whether a launched
node could own the key at run time, from two structural facts and not from a package name:
the key extends a launched node's name by a suffix, which is what a runtime child is called,
or the key names a node in a namespace no launch file created, which is where a runtime
child lives. When one of those holds and the coded layer never read the sources of the nodes
that would declare that child, the rule states nothing. A key that is a truncation of a
launched name is the opposite case, a misspelling, and stays reported: a child extends its
parent's name, it never shortens it.

`silences(atlas)` reports what those gates suppressed and why, one row per rule and profile,
and `rosio/drift/plugins.py` exposes the same function. A drift report that lists a silent
rule without a reason tells a reader the rule looked and found nothing.
"""

from __future__ import annotations

import re
from collections import defaultdict
from dataclasses import dataclass
from typing import Any

from rosio import ids, join, model
from rosio.drift import plugins, rules
from rosio.errors import ContractError
from rosio.resolve import yaml_lines

DECLARED = model.Layer.DECLARED.value
CODED = model.Layer.CODED.value
PARAM = model.NodeKind.PARAM.value
FILE = model.NodeKind.FILE.value
NODE = model.NodeKind.NODE.value
EFFECTIVE = model.ParamState.EFFECTIVE.value

AUTO_DECLARED = join.AUTO_DECLARED
AUTO_DECLARED_PREFIXES = join.AUTO_DECLARED_PREFIXES

FILE_MECHANISMS = frozenset(
    {
        model.Mechanism.YAML_BASE.value,
        model.Mechanism.YAML_OVERLAY_MERGE.value,
        model.Mechanism.YAML_WILDCARD.value,
        model.Mechanism.LAUNCH_PARAMS_FILE.value,
        model.Mechanism.ROS_ARGS_PARAMS_FILE.value,
    }
)
"""Mechanisms whose site is a parameter document. A value that loses in one of these is a
line that reads as the configuration and is not."""

CODE_TYPES = {
    "bool": "bool",
    "double": "float",
    "float": "float",
    "int": "int",
    "int64_t": "int",
    "long": "int",
    "string": "str",
    "std::string": "str",
}
"""C++ declaration type to the value type a yaml reader spells. A vector of anything maps to
list, handled below, and an unmapped spelling is skipped rather than guessed."""

_LIST_MARKERS = ("vector<", "[]")


@dataclass(frozen=True)
class _Row:
    param: model.Node
    profile: str
    winner: model.Candidate
    loser: model.Candidate

    @property
    def name(self) -> str:
        return ids.parse_param(self.param.id).name

    @property
    def owner(self) -> str:
        return ids.parse_param(self.param.id).owner


# ---------------------------------------------------------------------------
# shared reading


def _params(atlas: model.Atlas) -> list[model.Node]:
    return [
        atlas.entities[key] for key in sorted(atlas.entities) if atlas.entities[key].kind == PARAM
    ]


def _by_profile(node: model.Node) -> dict[str, list[model.Candidate]]:
    out: dict[str, list[model.Candidate]] = defaultdict(list)
    for candidate in node.candidates:
        if candidate.profile != model.ANY_PROFILE:
            out[candidate.profile].append(candidate)
    return {
        profile: sorted(rows, key=lambda row: row.rank) for profile, rows in sorted(out.items())
    }


def _live(node: model.Node, profile: str) -> bool:
    """The parameter is live configuration of this profile, said by this profile's own fact.

    The entity keeps the strongest state of every profile written, so a key that is effective
    in one profile and inert in another reads as effective here. Asking the profile's own fact
    is what keeps a chain rule off a sub key whose plugin block this profile does not select,
    and off a value no layer tracked.
    """
    fact = node.fact(DECLARED, profile)
    if not rules.usable(fact):
        return False
    if str(fact.values.get("state") or node.state) == EFFECTIVE:
        return True
    # A block followed under a name launch never states wins once the join reads that name
    # in the source, and the fact the parameter collector wrote before it knew stays as said.
    return bool(fact.values.get("owner")) and any(
        row.wins and row.profile == profile for row in node.candidates
    )


def _layers(*candidates: model.Candidate) -> list[str]:
    return sorted(
        {
            CODED if candidate.mechanism == model.Mechanism.CODE_DEFAULT.value else DECLARED
            for candidate in candidates
        }
    )


_auto_declared = join.auto_declared


def _restated_severity(row: _Row) -> str:
    """How loud a restatement is, from what the two sites are.

    Two parameter documents stating one value is hygiene: both are the same kind of
    document, an engineer editing either sees the other in the same review, and the
    behaviour is the same whichever line goes. A parameter document restating what a launch
    file or a launcher module writes is a different animal. The two sites are in two
    languages, only one of them is where a reader looks, and the line in the document that
    reads as the configuration already does nothing. The day one of them changes, the
    engineer who changed the document gets no effect and no warning, which is the shape of
    the shadowed findings on this stack. That is worth a line above hygiene, with one
    exception: a name the client library declares for every node, `use_sim_time` and the
    qos overrides, is restated by every launch file in every workspace and acting on it
    changes nothing anywhere, so it stays where it was.
    """
    if _auto_declared(row.name) or row.winner.mechanism in FILE_MECHANISMS:
        return rules.severity_of("param.restated_identical")
    return rules.WARN


def _deliberate(winner: model.Candidate, loser: model.Candidate) -> bool:
    """An overlay document beating the base document it was merged onto."""
    return (
        winner.mechanism == model.Mechanism.YAML_OVERLAY_MERGE.value
        and loser.mechanism in FILE_MECHANISMS
        and loser.source.file != winner.source.file
    )


def _editable(winner: model.Candidate, loser: model.Candidate) -> bool:
    """A losing site an engineer can point at and would read as the configuration.

    A losing inline dict is a default, and a default that a parameter file overrides is
    doing its job. Its site is also unnameable: a `parameters=` list has one locator whether
    it holds one dict or three, and one of the three on this bench is loaded from a third
    party file that is in no repository. A finding that cannot name a line to change is not
    a finding.
    """
    if loser.mechanism not in FILE_MECHANISMS or loser.source.line_hint is None:
        return False
    return (loser.source.file, loser.source.locator) != (winner.source.file, winner.source.locator)


def _shadow_groups(atlas: model.Atlas, kind: str) -> dict[tuple[str, ...], list[_Row]]:
    groups: dict[tuple[str, ...], list[_Row]] = defaultdict(list)
    for node in _params(atlas):
        for profile, chain in _by_profile(node).items():
            if not _live(node, profile):
                continue
            winner = next((row for row in chain if row.wins), None)
            if winner is None:
                continue
            for loser in chain:
                if loser.shadow_kind != kind or not _editable(winner, loser):
                    continue
                if kind == model.ShadowKind.VALUE_CHANGED.value and _deliberate(winner, loser):
                    continue
                key = (profile, loser.source.file, loser.source.locator, winner.source.file)
                groups[key].append(_Row(node, profile, winner, loser))
    return {key: sorted(rows, key=lambda row: row.param.id) for key, rows in sorted(groups.items())}


def _discriminators(groups: dict[tuple[str, ...], list[_Row]]) -> dict[tuple[str, ...], str]:
    """One readable discriminator per group, and a content digest where a parameter carries
    two groups in one profile."""
    seen: dict[tuple[str, str], int] = defaultdict(int)
    for key, group in groups.items():
        if len(group) == 1:
            seen[(group[0].param.id, key[0])] += 1
    out: dict[tuple[str, ...], str] = {}
    for key, group in groups.items():
        slug = rules.slug_of(key[0])
        single = len(group) == 1 and seen[(group[0].param.id, key[0])] == 1
        out[key] = slug if single else f"{slug}.{ids.digest(key[1], key[2], key[3])}"
    return out


def _subject(atlas: model.Atlas, key: tuple[str, ...], group: list[_Row]) -> str:
    if len(group) == 1:
        return group[0].param.id
    return key[1] if key[1] in atlas.entities else group[0].param.id


# ---------------------------------------------------------------------------
# the rules


@rules.register("param.shadowed")
def shadowed(atlas: model.Atlas) -> list[model.Finding]:
    """A value that a reader would take for the configuration and that never takes effect."""
    groups = _shadow_groups(atlas, model.ShadowKind.VALUE_CHANGED.value)
    discs = _discriminators(groups)
    out: list[model.Finding] = []
    for key, group in groups.items():
        head = group[0]
        slug = rules.slug_of(head.profile)
        same_file = head.loser.source.file == head.winner.source.file
        trusted_loser = (
            head.loser.mechanism in FILE_MECHANISMS and head.winner.mechanism not in FILE_MECHANISMS
        )
        if same_file:
            severity = rules.severity_of("param.shadowed", "self_contradiction")
        elif trusted_loser:
            severity = rules.severity_of("param.shadowed")
        else:
            severity = rules.INFO
        lose_at = rules.where(head.loser.source)
        win_at = rules.where(head.winner.source)
        owner = rules.node_text(head.owner)
        if len(group) == 1:
            title = f"{head.name} at {lose_at} is overridden"
            detail = (
                f"{owner} reads {rules.value_text(head.winner.value)} for {head.name} in "
                f"profile {slug}, set by {rules.path_of(head.winner.source.file)} through "
                f"{head.winner.mechanism}. The value {rules.value_text(head.loser.value)} in "
                f"{rules.path_of(head.loser.source.file)} loses."
            )
            say = (
                f"In profile {slug}, {owner} {head.name} is "
                f"{rules.value_text(head.winner.value)} from {win_at}, so "
                f"{rules.value_text(head.loser.value)} at {lose_at} never takes effect. "
                f"Edit {win_at} to change it."
            )
        else:
            names = ", ".join(dict.fromkeys(row.name for row in group[:3]))
            owners = sorted({rules.node_text(row.owner) for row in group})
            whose = owner if len(owners) == 1 else f"{len(owners)} nodes ({', '.join(owners[:3])})"
            title = f"{len(group)} values at {lose_at} are overridden"
            detail = (
                f"{rules.path_of(head.winner.source.file)} sets {len(group)} parameters of "
                f"{whose} that {rules.path_of(head.loser.source.file)} also sets, in profile "
                f"{slug}, and wins all of them. Affected: {names}."
            )
            say = (
                f"In profile {slug}, {rules.path_of(head.winner.source.file)} overrides "
                f"{len(group)} parameters that {lose_at} sets for {whose}, including "
                f"{head.name} ({rules.value_text(head.loser.value)} becomes "
                f"{rules.value_text(head.winner.value)}). Edit "
                f"{rules.path_of(head.winner.source.file)}, the values at {lose_at} are dead."
            )
        fix = (
            f"Change the value at {win_at}, or remove the losing line at {lose_at} so one file "
            "states it."
        )
        later = _block_of(head.winner.source)
        if same_file and later == "/**" and _block_of(head.loser.source) not in ("", "/**"):
            # The more specific key was meant to win and loses only by its place in the file.
            fix = (
                f"Drop {head.name} from the /** block at {win_at}, or move the /** block above "
                f"the {_block_of(head.loser.source)} block, so the value at {lose_at} takes "
                "effect: in one file the later key wins, however much it matches."
            )
        out.append(
            model.Finding(
                rule="param.shadowed",
                severity=severity,
                subject=_subject(atlas, key, group),
                discriminator=discs[key],
                title=title,
                detail=detail,
                fix=fix,
                say=say,
                profiles=[head.profile],
                layers=_layers(head.winner, head.loser),
                where=[head.winner.source, head.loser.source],
            )
        )
    return out


@rules.register("param.restated_identical")
def restated_identical(atlas: model.Atlas) -> list[model.Finding]:
    """A line that repeats, byte for byte, a value another file already sets."""
    groups = _shadow_groups(atlas, model.ShadowKind.RESTATED_IDENTICAL.value)
    # One deletable line is one finding, whatever profiles read it and whatever each restates.
    merged: dict[tuple[Any, ...], list[tuple[tuple[str, ...], list[_Row]]]] = defaultdict(list)
    for key, group in groups.items():
        merged[(key[1], key[2], key[3])].append((key, group))
    out: list[model.Finding] = []
    for line in sorted(merged):
        key = merged[line][0][0]
        rows = {(row.param.id, repr(row.loser.value)): row for _k, g in merged[line] for row in g}
        group = [rows[item] for item in sorted(rows)]
        profiles = sorted({item[0] for item, _rows in merged[line]})
        head = group[0]
        slug = ("profile " if len(profiles) == 1 else "profiles ") + ", ".join(
            rules.slug_of(item) for item in profiles
        )
        lose_at = rules.where(head.loser.source)
        win_at = rules.where(head.winner.source)
        owner = rules.node_text(head.owner)
        if len(group) == 1:
            title = f"{head.name} at {lose_at} restates the value already in force"
            detail = (
                f"{head.name} is {rules.value_text(head.winner.value)} for {owner} in "
                f"{slug} whether or not {rules.path_of(head.loser.source.file)} states it, "
                f"because {rules.path_of(head.winner.source.file)} sets the same value and "
                "wins."
            )
            say = (
                f"{lose_at} sets {owner} {head.name} to "
                f"{rules.value_text(head.loser.value)}, the same value {win_at} already gives "
                f"it in {slug}. Delete the line at {lose_at}."
            )
        else:
            names = ", ".join(row.name for row in group[:3])
            title = f"{len(group)} values at {lose_at} restate what is already in force"
            detail = (
                f"{rules.path_of(head.loser.source.file)} restates {len(group)} parameters of "
                f"{owner} that {rules.path_of(head.winner.source.file)} sets to the same "
                f"values in {slug}. Affected: {names}."
            )
            say = (
                f"{lose_at} restates {len(group)} parameters of {owner} that {win_at} already "
                f"sets to the same values in {slug}, including {head.name} = "
                f"{rules.value_text(head.loser.value)}. Delete them at {lose_at}."
            )
        restated = {
            item[0]: ", ".join(
                f"{row.name} = {rules.value_text(row.loser.value)} for {rules.node_text(row.owner)}"
                for row in found
            )
            for item, found in merged[line]
        }
        if len(set(restated.values())) > 1:
            listed = "; ".join(
                f"{rules.slug_of(profile)}, {text}" for profile, text in sorted(restated.items())
            )
            detail += f" What the line restates differs by profile: {listed}."
        out.append(
            model.Finding(
                rule="param.restated_identical",
                severity=_restated_severity(head),
                subject=_subject(atlas, key, group),
                discriminator=ids.digest(key[1], key[2], key[3]),
                title=title,
                detail=detail,
                fix=f"Delete the restated line at {lose_at}.",
                say=say,
                profiles=profiles,
                layers=_layers(head.winner, head.loser),
                where=[head.winner.source, head.loser.source],
            )
        )
    return out


@rules.register("param.set_undeclared")
def set_undeclared(atlas: model.Atlas) -> list[model.Finding]:
    """A file sets a parameter that the node's own source never declares.

    One line is one finding, with every profile that reads it and every node that ignores it:
    a yaml key nineteen profiles read, or one `SetParameter` loop handing a group its keys, is
    one place to edit. When the node declares a name close to the one the file sets, the
    finding names it: a key one letter off from a declared parameter is a typo far more often
    than a new parameter.

    A line some node reads and declares, in any profile, is silent: a `/**` key or a launch
    `SetParameter` that reaches several nodes exists for the node that reads it, and deleting
    it breaks that node. A `/**` block in a file only this node is handed reaches this node
    alone, so it is reported like any other line.
    """
    blind, declared = join.parameter_declarations(atlas)
    needed: set[tuple[str, str, str]] = set()
    for node in _params(atlas):
        parsed = ids.parse_param(node.id)
        for row in node.candidates:
            if parsed.name in join.declared_in(declared, parsed.owner, row.profile) | declared.get(
                (parsed.owner, join.REACHED), set()
            ):
                needed.add((parsed.name, row.source.file, row.source.locator))
    lines: dict[tuple[str, str, int], list[_Ignored]] = defaultdict(list)
    for node in _params(atlas):
        owner_id = ids.parse_param(node.id).owner
        name = ids.parse_param(node.id).name
        owner = atlas.get(owner_id)
        if owner is None:
            continue
        for profile, chain in _by_profile(node).items():
            if not _live(node, profile) or not join.ignored(atlas, node, profile, blind, declared):
                continue
            # The join took the win off the line the node ignores and said so on it.
            winner = next((row for row in chain if row.wins or row.note == join.IGNORED_NOTE), None)
            if winner is None:
                continue
            if (name, winner.source.file, winner.source.locator) in needed:
                continue
            near = _nearest_name(name, join.declared_in(declared, owner_id, profile))
            source = winner.source
            lines[(source.file, source.locator, source.line_hint or 0)].append(
                _Ignored(profile, node.id, owner_id, name, winner, near, _read_by(owner, profile))
            )
    # The line number only groups. A yaml line is named by its key, and a launch action that
    # hands keys to nodes (a SetParameter loop, an inline dict) by its locator, which names
    # the action and not the nodes, so neither a key added to it nor a profile that starts one
    # more node moves the id.
    merged: dict[str, list[_Ignored]] = defaultdict(list)
    for key in sorted(lines):
        rows = lines[key]
        source = rows[0].winner.source
        names = [] if rows[0].winner.mechanism in _HANDED else sorted({row.name for row in rows})
        merged[ids.digest(source.file, source.locator, *names)].extend(rows)
    return [_undeclared_line(disc, merged[disc]) for disc in sorted(merged)]


_HANDED = frozenset(
    {
        model.Mechanism.LAUNCH_SET_PARAMETER.value,
        model.Mechanism.LAUNCH_INLINE_DICT.value,
        model.Mechanism.ROS_ARGS_P.value,
    }
)


@dataclass(frozen=True)
class _Ignored:
    profile: str
    param: str
    owner: str
    name: str
    winner: model.Candidate
    near: str
    read: tuple[str, ...]


def _read_by(owner: model.Node, profile: str) -> tuple[str, ...]:
    """The files the join read for a node, the ones that state the node itself first.

    A declaration belongs beside the name literal of the node, and a solver or a plugin the
    node loads sorted ahead of that file by name sent a reader to the wrong place.
    """
    coded = owner.fact(CODED, profile)
    if coded is None:
        return ()
    values = coded.values
    home = ids.parse(str(values["exe"])).segments[0] if values.get("exe") else ""
    stating = {source.file for source in coded.evidence}
    return tuple(
        sorted(
            values.get("coded_sources") or [],
            key=lambda item: (item not in stating, ids.parse_file(item).package != home, item),
        )
    )


def _listing(items: list[str], limit: int = 4, noun: str = "") -> str:
    shown = list(items[:limit])
    if len(items) > limit:
        shown.append(f"{len(items) - limit} more{noun}")
    if len(shown) < 2:
        return "".join(shown)
    return f"{', '.join(shown[:-1])} and {shown[-1]}"


def _undeclared_line(disc: str, rows: list[_Ignored]) -> model.Finding:
    rows = sorted(rows, key=lambda row: (row.name, row.owner, row.profile))
    head = rows[0]
    source = head.winner.source
    at = rules.where(source)
    names = sorted({row.name for row in rows})
    owners = sorted({row.owner for row in rows})
    profiles = sorted({row.profile for row in rows})
    owner_text = _listing([rules.node_text(item) for item in owners], noun=" nodes")
    profile_text = ("profile " if len(profiles) == 1 else "profiles ") + _listing(
        [rules.slug_of(item) for item in profiles], limit=6
    )
    name_text = _listing(names, limit=6)
    single = len(names) == 1
    shown = [rules.path_of(item) for item in head.read[:3]]
    if len(head.read) > 3:
        shown.append(f"{len(head.read) - 3} more files")
    read = _listing(shown) if shown else "its source"
    if len(owners) > 1:
        read = "their sources"
    declares = "declares" if len(head.read) <= 1 and len(owners) == 1 else "declare"
    if single and len(owners) == 1:
        title = f"{head.name} is set for {owner_text} and never declared"
    elif single:
        title = f"{head.name} is set for {len(owners)} nodes and none declares it"
    else:
        title = f"{len(names)} parameters at {at} are set and never declared"
    if head.winner.mechanism == model.Mechanism.LAUNCH_SET_PARAMETER.value:
        # A SetParameter hands its key to every node of its group, so the node that reads it
        # may be in a profile this atlas does not run.
        drop = (
            f"stop the SetParameter at {at} handing {name_text} to the group if no node it "
            f"starts reads {'it' if single else 'them'}"
        )
    elif head.winner.mechanism == model.Mechanism.LAUNCH_INLINE_DICT.value:
        # A dict the launch file builds holds every key on the line that hands it over, and
        # that line also carries the values the node does read.
        drop = f"remove {name_text} from the dictionary passed at {at}"
    else:
        drop = f"delete the line at {at}"
    nears = sorted({(row.name, row.near) for row in rows if row.near})
    meant = "".join(
        f" {rules.node_text(next(row.owner for row in rows if row.name == name))} declares "
        f"{near}, which is likely what was meant."
        for name, near in nears
    )
    if nears and len(nears) == len(names):
        fix = " ".join(
            f"Rename {name} to {near} at {at}, or correct the declaration if {near} is the "
            "misspelling."
            for name, near in nears
        )
    else:
        fix = f"Declare {name_text} in the node source, or {drop}."
    value = f" to {rules.value_text(head.winner.value)}" if single and len(rows) == 1 else ""
    subject_text = (
        f"{owner_text} {name_text}{value}" if len(owners) == 1 else f"{name_text} on {owner_text}"
    )
    those = "that name" if single else "those names"
    say = (
        f"In {profile_text}, {at} sets {subject_text} and {read} {declares} no parameter with "
        f"{those}, so {'the value is' if single else 'the values are'} ignored.{meant}"
        + ("" if nears else f" {drop[:1].upper()}{drop[1:]} or declare the parameter.")
    )
    return model.Finding(
        rule="param.set_undeclared",
        severity=rules.severity_of("param.set_undeclared"),
        subject=source.file,
        discriminator=disc,
        title=title,
        detail=(
            f"{owner_text} {'is' if len(owners) == 1 else 'are'} configured with {name_text} "
            f"at {at} in {profile_text}, and no declaration for {those} was found in {read}. "
            "A node ignores a parameter it never declares."
        ),
        fix=fix,
        say=say,
        profiles=profiles,
        layers=[CODED, DECLARED],
        where=sorted({row.winner.source for row in rows}, key=model.Source.sort_key),
    )


def _nearest_name(name: str, declared: set[str]) -> str:
    """The declared name a mistyped key most likely meant, or nothing when none is close.

    Close means one slip of the hand: a letter missing, added, wrong or swapped with its
    neighbour, inside a word of three letters or more. `min_laser_range` beside
    `max_laser_range` differs by a word, and `kd` beside `ki` or `x_min` beside `y_min` by a
    word of one or two letters, so naming one the misspelling of the other sends an engineer
    to rename a parameter that is merely undeclared.
    """
    return next(
        (
            item
            for item in sorted(declared - {name})
            if _one_slip(name, item) or _word_dropped(name, item)
        ),
        "",
    )


def _word_dropped(a: str, b: str) -> bool:
    """One spelling is the other with an inner word left out: `simulation_step` for
    `simulation_time_step`. A first or last word left out names another thing (`range`
    beside `min_range`), so only a word between two kept ones counts."""
    head_a, _, last_a = a.rpartition(".")
    head_b, _, last_b = b.rpartition(".")
    if head_a != head_b:
        return False
    shorter, longer = sorted((last_a.split("_"), last_b.split("_")), key=len)
    if len(longer) != len(shorter) + 1 or len(shorter) < 2:
        return False
    return any(longer[:index] + longer[index + 1 :] == shorter for index in range(1, len(shorter)))


def _one_slip(a: str, b: str) -> bool:
    if abs(len(a) - len(b)) > 1:
        return False
    start = 0
    while start < min(len(a), len(b)) and a[start] == b[start]:
        start += 1
    if min(len(_word_at(a, start)), len(_word_at(b, start))) < 3:
        return False
    a, b = a[start:], b[start:]
    if len(a) == len(b):
        return a[1:] == b[1:] or (a[2:] == b[2:] and a[:2] == b[1::-1])
    shorter, longer = sorted((a, b), key=len)
    return longer[1:] == shorter


def _word_at(name: str, index: int) -> str:
    """The word of a dotted or underscored name that holds `index`, or ends just before it."""
    position = 0
    for word in re.split(r"[._/]", name):
        if position <= index <= position + len(word):
            return word
        position += len(word) + 1
    return ""


@rules.register("param.declared_unread")
def declared_unread(atlas: model.Atlas) -> list[model.Finding]:
    """A declaration whose value no code path can reach.

    The coded fact has to state both that nothing reads the parameter back and that the
    declaration itself returns nothing usable. Without the second key the rule would report
    every `auto value = declare_parameter(...)`, which is the normal rclcpp idiom.
    """
    out: list[model.Finding] = []
    for node in _params(atlas):
        owner_id = ids.parse_param(node.id).owner
        name = ids.parse_param(node.id).name
        for fact in sorted(node.facts, key=model.LayerFact.sort_key):
            if fact.layer != CODED or not rules.usable(fact):
                continue
            values = fact.values
            if not values.get("declared_in_code"):
                continue
            if values.get("read", True) or values.get("declaration_returns_value", True):
                continue
            slug = rules.slug_of(fact.profile) if fact.profile != model.ANY_PROFILE else "*"
            at = rules.where(fact.evidence[0] if fact.evidence else None)
            owner_text = rules.node_text(owner_id)
            out.append(
                model.Finding(
                    rule="param.declared_unread",
                    severity=rules.severity_of("param.declared_unread"),
                    subject=node.id,
                    discriminator=slug,
                    title=f"{name} is declared and never read",
                    detail=(
                        f"{owner_text} declares {name} and no code path reads it back, and the "
                        "declaration returns nothing the node keeps."
                    ),
                    fix=f"Read {name} where it is used, or delete the declaration at {at}.",
                    say=(
                        f"{at} declares {owner_text} {name} and nothing reads it, so every "
                        "value a file gives it is ignored. Read it or delete the declaration."
                    ),
                    profiles=[] if fact.profile == model.ANY_PROFILE else [fact.profile],
                    layers=[CODED],
                    where=list(fact.evidence),
                )
            )
    return out


def _type_of(spelling: str) -> str:
    text = spelling.strip()
    if any(marker in text for marker in _LIST_MARKERS):
        return "list"
    return CODE_TYPES.get(text, "")


@rules.register("param.type_changed")
def type_changed(atlas: model.Atlas) -> list[model.Finding]:
    """A declaration type disagreeing with the value that wins, inside one profile.

    A node declared with one type refuses a value of another at start up, so this is an
    error wherever it is true. A losing candidate never reaches the node, so its type is
    not compared, and a value that is a whole launch substitution has no type until launch
    runs it.
    """
    out: list[model.Finding] = []
    for node in _params(atlas):
        name = ids.parse_param(node.id).name
        owner_text = rules.node_text(ids.parse_param(node.id).owner)
        for profile, chain in _by_profile(node).items():
            if not _live(node, profile):
                continue
            winner = next((row for row in chain if row.wins), None)
            if (
                winner is None
                or not winner.value_type
                or winner.value_type == "unknown"
                or _substitution(winner.value)
            ):
                continue
            coded = node.fact(CODED, profile)
            if not rules.usable(coded):
                continue
            declared_type = _type_of(str(coded.values.get("type") or ""))
            if not declared_type or declared_type == winner.value_type:
                continue
            slug = rules.slug_of(profile)
            win_at = rules.where(winner.source)
            declared_at = rules.where(coded.evidence[0] if coded.evidence else None)
            out.append(
                model.Finding(
                    rule="param.type_changed",
                    severity=rules.severity_of("param.type_changed"),
                    subject=node.id,
                    discriminator=slug,
                    title=f"{name} has two types in profile {slug}",
                    detail=(
                        f"{owner_text} {name} is {winner.value_type} where it wins and is "
                        f"declared {declared_type} in code, inside one profile."
                    ),
                    fix=f"Make the value at {win_at} a {declared_type}, or change the declaration.",
                    say=(
                        f"In profile {slug}, {owner_text} {name} is "
                        f"{rules.value_text(winner.value)} ({winner.value_type}) at {win_at} "
                        f"while {declared_at} declares it {declared_type}. The node refuses "
                        "the value at start up."
                    ),
                    profiles=[profile],
                    layers=sorted({*_layers(winner), CODED}),
                    where=[winner.source, *coded.evidence],
                )
            )
    return out


def _substitution(value: object) -> bool:
    text = value.strip() if isinstance(value, str) else ""
    return text.startswith("$(") and text.endswith(")")


# ---------------------------------------------------------------------------
# blocks with no owner


ORPHAN_STATES = (
    model.ParamState.OWNER_NAME_MISMATCH.value,
    model.ParamState.UNRESOLVED_OWNER.value,
)


def _line(source: model.Source) -> int:
    """Line hints order a block's rows, for an atlas whose rows cite no block key. Advisory
    only: it never enters an id, a discriminator or a detail."""
    return source.line_hint if source.line_hint is not None else 1 << 30


def _keys(count: int) -> str:
    return "1 key" if count == 1 else f"{count} keys"


@dataclass(frozen=True)
class _BlockRow:
    """One key of one block, where the chain writes it and where its block key is."""

    state: str
    name: str
    site: model.Source
    block_at: model.Source | None


@dataclass(frozen=True)
class _Block:
    profile: str
    file: str
    selector: str
    state: str
    keys: tuple[str, ...]
    source: model.Source
    """Where the block key is written, which is the line a reader deleting the block starts
    at. The parameter collector cites one block key per file on the fact, and this block
    takes the one that names its own file. An atlas built before the collector wrote them
    leaves the first key inside the block instead, two lines past the block key, which is the
    closest row such an atlas has."""
    live_in: tuple[str, ...] = ()
    """Other profiles in which a node owns keys of this block, read from the same file."""

    @property
    def proven(self) -> bool:
        return self.state == model.ParamState.OWNER_NAME_MISMATCH.value


def _blocks(atlas: model.Atlas) -> list[_Block]:
    # One block is one file. A key written under the same block name in two files belongs to
    # two blocks, a reader has to delete both, and the count each finding states is the keys
    # of its own block, so every write site of the chain is read and never one site standing
    # for the parameter. The block key comes from the fact evidence row that names the file
    # the block is in, so two blocks of one key cite their own first line and not each
    # other's.
    # The entity state is the strongest over all profiles (AGENTS section 7), so a block that
    # one profile reads would hide the profile that ignores it; the fact state decides.
    rows: dict[tuple[str, str, str], list[_BlockRow]] = defaultdict(list)
    shared: dict[tuple[str, str, str], set[str]] = defaultdict(set)
    for node in _params(atlas):
        name = ids.parse_param(node.id).name
        for fact in node.facts:
            if fact.layer != DECLARED or not rules.usable(fact):
                continue
            state = str(fact.values.get("state") or "")
            selector = str(fact.values.get("selector") or "")
            if state not in ORPHAN_STATES or not selector:
                continue
            owner = str(fact.values.get("owner") or "")
            if owner and _starts(atlas, owner, fact.profile):
                continue
            keys = {row.file: row for row in fact.evidence}
            for site in _write_sites(node, fact.profile):
                if owner and _block_of(site) != selector:
                    continue
                rows[(fact.profile, site.file, selector)].append(
                    _BlockRow(state, name, site, keys.get(site.file))
                )
                shared[(fact.profile, site.file, selector)].update(
                    row.profile
                    for row in node.candidates
                    if row.profile != fact.profile
                    and row.source is not None
                    and row.source.file == site.file
                    and _live(node, row.profile)
                )
    out: list[_Block] = []
    for (profile, file_id, selector), found in sorted(rows.items()):
        state = (
            model.ParamState.OWNER_NAME_MISMATCH.value
            if any(row.state == model.ParamState.OWNER_NAME_MISMATCH.value for row in found)
            else model.ParamState.UNRESOLVED_OWNER.value
        )
        ordered = sorted(found, key=lambda row: (_line(row.site), row.name))
        block_at = next((row.block_at for row in ordered if row.block_at is not None), None)
        out.append(
            _Block(
                profile=profile,
                file=file_id,
                selector=selector,
                state=state,
                keys=tuple(sorted({row.name for row in found})),
                source=block_at or ordered[0].site,
                live_in=tuple(sorted(shared[(profile, file_id, selector)])),
            )
        )
    return out


def _starts(atlas: model.Atlas, fqn: str, profile: str) -> bool:
    """Whether the profile starts a node of this name, which a source can name for a process
    the launch file leaves unnamed. The parameter collector cannot know that, the join can."""
    try:
        node = atlas.get(ids.build_node(fqn))
    except ContractError:
        return False
    return node is not None and node.fact(DECLARED, profile) is not None


def _block_of(site: model.Source) -> str:
    """The block key a yaml write site sits under, read off its key path locator."""
    head, found, _rest = site.locator.partition(".ros__parameters.")
    return yaml_lines.selector_of(head.split(".")) if found else ""


def _write_sites(node: model.Node, profile: str) -> list[model.Source]:
    """Every file and line this profile writes this parameter at, best rank first.

    `rules.chain_site` answers with one site, which is the right answer for a rule that
    reports a parameter. A rule that reports a block reports one per file, so it asks for all
    of them: a key spelled in a base file and again in an overlay is one key of each block.
    """
    seen: dict[tuple[str, str], model.Source] = {}
    for row in sorted(node.candidates, key=model.Candidate.sort_key):
        if row.profile != profile or not row.source:
            continue
        seen.setdefault((row.source.file, row.source.locator), row.source)
    return list(seen.values())


def _launched(atlas: model.Atlas, profile: str) -> list[str]:
    return sorted(
        entity.id
        for entity in atlas.entities.values()
        if entity.kind == model.NodeKind.NODE.value
        and profile in entity.profiles
        and not ids.is_synthetic(entity.id)
    )


def _unnamed(atlas: model.Atlas, profile: str) -> int:
    return sum(
        1
        for entity in atlas.entities.values()
        if entity.kind == model.NodeKind.NODE.value
        and profile in entity.profiles
        and ids.is_synthetic(entity.id)
    )


def _near_names(atlas: model.Atlas, profile: str, selector: str) -> list[str]:
    """Launched nodes whose name is the block key plus or minus a suffix, or, for a key that
    names no namespace, the node of that name in a namespace: rcl reads `planner:` as
    `/planner`, so the block misses `/robot1/planner`."""
    key = selector.strip("/").rsplit("/", 1)[-1]
    if not key:
        return []
    bare = "/" not in selector.strip("/")
    found = []
    for node_id in _launched(atlas, profile):
        fqn = ids.parse_node(node_id).fqn or ""
        base = fqn.strip("/").rsplit("/", 1)[-1]
        if base == key:
            if bare and fqn.count("/") > 1:
                found.append(fqn)
            continue
        if base.startswith(f"{key}_") or key.startswith(f"{base}_"):
            found.append(fqn)
    return sorted(found)


def _coded_packages(atlas: model.Atlas) -> frozenset[str]:
    """Packages whose sources the coded layer read. A package with no file read cannot
    contradict anything the declared layer says about a node it would create."""
    return frozenset(
        ids.parse_file(node.id).package for node in atlas.by_kind(FILE) if CODED in node.layers
    )


def _package_of(atlas: model.Atlas, node_id: str, profile: str) -> str:
    node = atlas.get(node_id)
    if node is None:
        return ""
    for fact in sorted(node.facts, key=model.LayerFact.sort_key):
        if fact.layer != DECLARED or not rules.usable(fact):
            continue
        if fact.profile not in (profile, model.ANY_PROFILE):
            continue
        package = str(fact.values.get("pkg") or "")
        if package:
            return package
    return ""


def _fqn(node_id: str) -> str:
    return ids.parse_node(node_id).fqn or ""


def _namespace_of(selector: str) -> str:
    try:
        fqn = ids.normalise_ros_name("/" + selector.strip().strip("/"))
    except ContractError:
        return ""
    head = fqn.rsplit("/", 1)[0]
    return head or "/"


def _runtime_namespace(atlas: model.Atlas, profile: str, selector: str) -> bool:
    """The block key names a node in a namespace no launch file of this profile created."""
    namespace = _namespace_of(selector)
    if namespace in ("", "/"):
        return False
    prefix = namespace + "/"
    return not any(_fqn(node_id).startswith(prefix) for node_id in _launched(atlas, profile))


def _parent_names(atlas: model.Atlas, profile: str, selector: str) -> list[str]:
    """Launched nodes whose name the block key extends by a suffix.

    That is the shape of a node another node creates at run time, so the key may be owned by
    a child of one of these. The reverse relation, a key that is a truncation of a launched
    name, is a misspelled key and not a child, because a child extends its parent's name.
    """
    key = selector.strip("/").rsplit("/", 1)[-1]
    if not key:
        return []
    found = []
    for node_id in _launched(atlas, profile):
        base = _fqn(node_id).strip("/").rsplit("/", 1)[-1]
        if base and base != key and key.startswith(f"{base}_"):
            found.append(node_id)
    return sorted(found)


def owner_gate(atlas: model.Atlas, profile: str, selector: str) -> str:
    """Empty when a rule may state that no node owns this block, otherwise why it may not.

    Three questions in order: was this profile's launch tree read, could a launched node own
    the key through a child it creates at run time, and were the sources of those nodes read.
    """
    unread = plugins.unread_entry_points(atlas, profile)
    if unread:
        return (
            "the launch tree of this profile was not read to the end ("
            + ", ".join(plugins.launch_path(item) for item in unread)
            + "), so the launched node set is a lower bound"
        )
    parents = _parent_names(atlas, profile, selector)
    reason = "the key extends the name of a launched node, which is what a runtime child is called"
    if _runtime_namespace(atlas, profile, selector):
        parents = _launched(atlas, profile)
        reason = "the key names a node in a namespace no launch file of this profile creates"
    if not parents:
        return ""
    coded = _coded_packages(atlas)
    # A node whose package is unknown has unread sources by the same test.
    owners = {
        _package_of(atlas, node_id, profile) or ids.parse_node(node_id).package
        for node_id in parents
    }
    blind = sorted(owners - coded)
    if not blind:
        return ""
    packages = ", ".join(item for item in blind if item) or "the launched nodes"
    return (
        f"{reason}, and no source of {packages} was read, so a child node that reads the "
        "block cannot be ruled out"
    )


def _gated(atlas: model.Atlas) -> dict[tuple[str, str], str]:
    """The blocks the owner gate holds back, keyed by profile and block key."""
    return {
        (block.profile, block.selector): reason
        for block in _blocks(atlas)
        if (reason := owner_gate(atlas, block.profile, block.selector))
    }


def _reachable(atlas: model.Atlas, owner: str, profile: str) -> bool:
    sites = []
    for edge in atlas.edges.values():
        if edge.kind != "launches" or edge.dst != owner:
            continue
        if not any(
            rules.usable(edge.fact(DECLARED, slot)) for slot in (profile, model.ANY_PROFILE)
        ):
            continue
        sites.append(rules.site_live(atlas.entities[edge.src], profile))
    if sites:
        return any(sites)
    # A node with no launch site and no fact in this profile is not started by it.
    entity = atlas.entities[owner]
    started = profile in entity.profiles or any(
        rules.usable(fact) and fact.profile == model.ANY_PROFILE for fact in entity.facts
    )
    return started and not entity.attrs.get("unreachable", False)


@rules.register("param.sim_time_mixed")
def sim_time_mixed(atlas: model.Atlas) -> list[model.Finding]:
    """Nodes of one profile that disagree about use_sim_time.

    One node on wall time among nodes on simulated time stamps its messages decades away from
    the others, and the symptom is a TF lookup that extrapolates into the past or the future,
    far from the parameter that caused it. Only values the chain states are compared: a node
    whose use_sim_time no file sets is not assumed false, so a profile fires only when two
    stated values disagree.
    """
    sides: dict[str, dict[bool, list[tuple[str, model.Candidate]]]] = defaultdict(
        lambda: defaultdict(list)
    )
    for node in _params(atlas):
        owner, name = ids.parse_param(node.id)
        if name != "use_sim_time":
            continue
        for profile, rows in _by_profile(node).items():
            winner = next((row for row in rows if row.wins), None)
            if (
                winner is None
                or not isinstance(winner.value, bool)
                or not _live(node, profile)
                or not _reachable(atlas, owner, profile)
            ):
                continue
            sides[profile][winner.value].append((owner, winner))
    out: list[model.Finding] = []
    for profile, split in sorted(sides.items()):
        if len(split) < 2:
            continue
        on_sim, on_wall = split[True], split[False]
        fewer, more, fewer_value = (
            (on_wall, on_sim, False) if len(on_wall) <= len(on_sim) else (on_sim, on_wall, True)
        )
        slug = rules.slug_of(profile)
        named = ", ".join(rules.node_text(owner) for owner, _ in sorted(fewer)[:3])
        if len(fewer) > 3:
            named += f" and {len(fewer) - 3} more"
        first_owner, first = sorted(fewer)[0]
        include = _included_at(atlas, first.source, profile, fewer_value)
        at = f", included at {rules.where(include)}" if include else ""
        other = ""
        if len(fewer) == len(more):
            # No side is the odd one out, so the line of each value is named.
            second_owner, second = sorted(more)[0]
            beside = _included_at(atlas, second.source, profile, not fewer_value)
            other = (
                f" {rules.node_text(second_owner)} gets {not fewer_value} from "
                f"{rules.where(second.source)}"
                + (f", included at {rules.where(beside)}" if beside else "")
                + "."
            )
            include_rows = [beside] if beside else []
        else:
            include_rows = []
        out.append(
            model.Finding(
                rule="param.sim_time_mixed",
                severity=rules.SEVERITY["param.sim_time_mixed"],
                subject=ids.build_profile(slug),
                title=(
                    f"use_sim_time is {fewer_value} for {len(fewer)} and "
                    f"{not fewer_value} for {len(more)} nodes in profile {slug}"
                ),
                detail=(
                    f"In profile {slug}, {len(on_sim)} nodes run on simulated time and "
                    f"{len(on_wall)} on wall time. Messages stamped by one clock reach nodes "
                    "reading the other, so transforms and message filters drop or "
                    f"extrapolate. On {fewer_value}: {named}."
                ),
                fix=(
                    f"Give every node of {slug} the same use_sim_time, "
                    + (
                        f"for example by passing use_sim_time at {rules.where(include)}, or "
                        if include
                        else "for example "
                    )
                    + "with one SetParameter action at the top of the launch file that starts "
                    "them."
                ),
                say=(
                    f"{named} run with use_sim_time {fewer_value} while {len(more)} other nodes "
                    f"of profile {slug} use {not fewer_value}; {rules.node_text(first_owner)} "
                    f"gets it from {rules.where(first.source)}{at}.{other} Set one value for "
                    "the profile."
                ),
                profiles=[profile],
                layers=sorted({layer for _, row in fewer + more for layer in _layers(row)}),
                where=rules.sources(
                    [row.source for _, row in sorted(fewer)]
                    + ([include] if include else [])
                    + include_rows
                ),
            )
        )
    return out


def _included_at(
    atlas: model.Atlas, source: model.Source | None, profile: str, value: object = None
) -> model.Source | None:
    """The include line that brought the launch file holding a value into the profile.

    nav2 states rviz's use_sim_time as a default inside rviz_launch.py, and the line to edit
    is the include that does not pass it, in the file that starts the profile. An include
    that passes the value, read from a literal the launch walk found on another line of the
    including file (a table an OpaqueFunction reads), is cited at that literal."""
    if source is None:
        return None
    try:
        parsed = ids.parse_file(source.file)
        target = ids.build_launchfile(parsed.package, parsed.relpath)
    except ContractError:
        return None
    spelled = str(value).lower() if isinstance(value, bool) else None
    for entity in atlas.by_kind(model.NodeKind.LAUNCHENT.value):
        fact = entity.fact(DECLARED, profile)
        if fact is None or fact.values.get("included_file") != target or not fact.evidence:
            continue
        passed = {str(row[0]): str(row[1]).lower() for row in fact.values.get("args") or []}
        if spelled is None or passed.get("use_sim_time") != spelled:
            continue
        for row in fact.values.get("arg_lines") or []:
            if row.get("name") == "use_sim_time" and row.get("line_hint"):
                at = fact.evidence[0]
                return model.Source(file=at.file, root=at.root, line_hint=row["line_hint"])
    for edge in sorted(atlas.edges.values(), key=lambda item: item.id):
        if edge.kind != "includes" or edge.dst != target:
            continue
        for fact in edge.facts:
            if fact.profile == profile and fact.evidence:
                return fact.evidence[0]
    return None


@rules.register("param.owner_name_mismatch")
def owner_name_mismatch(atlas: model.Atlas) -> list[model.Finding]:
    """A block key that misses a launched node by a suffix.

    This is the case that is silently ignored today and is one rename away from working, so
    it is separated from a block that owns nothing at all.
    """
    out: list[model.Finding] = []
    gated = _gated(atlas)
    for block in _blocks(atlas):
        if not block.proven or (block.profile, block.selector) in gated:
            continue
        near = _near_names(atlas, block.profile, block.selector)
        if not near:
            continue
        slug = rules.slug_of(block.profile)
        at = rules.where(block.source)
        listed = ", ".join(near)
        target = near[0] if len(near) == 1 else "the node it configures"
        key = block.selector.strip("/")
        namespaced = all(name.rsplit("/", 1)[-1] == key for name in near)
        if namespaced and len(near) > 1:
            target = f"/**/{key}"
        how = (
            "which run in a namespace, and a key with no namespace names the node in the root "
            "namespace alone"
            if namespaced
            else "whose names start the same way"
        )
        out.append(
            model.Finding(
                rule="param.owner_name_mismatch",
                severity=rules.severity_of("param.owner_name_mismatch"),
                subject=block.file,
                discriminator=f"{slug}.{block.selector}",
                title=f"block {block.selector} misses the node name in {slug}",
                detail=(
                    f"The block {block.selector} carries {_keys(len(block.keys))} and profile "
                    f"{slug} launches no node of that name. It launches {listed}, {how}, so "
                    "nothing in the block is read."
                ),
                fix=f"Rename the block key {block.selector} to {target}.",
                say=(
                    f"{at} configures a node called {block.selector} with "
                    f"{_keys(len(block.keys))}, and profile {slug} launches {listed} instead, "
                    f"so none of the block is read. Rename the block key at {at} to {target}."
                ),
                profiles=[block.profile],
                layers=[DECLARED],
                where=[block.source],
            )
        )
    return out


@rules.register("param.orphan_block")
def orphan_block(atlas: model.Atlas) -> list[model.Finding]:
    """A parameter block no node of a profile can own.

    Reported once per block and file, with the profiles it is dead in, because one deletable
    block is one edit: a shared parameters file read by eleven profiles made one line eleven
    findings. A profile that holds a process whose runtime name rosio could not form cannot
    prove the block owns nothing, so such a finding drops to info and says which half of the
    claim is unproven.

    The instruction is scoped to the profiles the claim is scoped to. A bare delete tells a
    reader to remove a block that another profile of the same workspace does launch.
    """
    gated = _gated(atlas)
    grouped: dict[tuple[str, str], list[_Block]] = defaultdict(list)
    for block in _blocks(atlas):
        if (block.profile, block.selector) in gated:
            continue
        if block.proven and _near_names(atlas, block.profile, block.selector):
            continue
        grouped[(block.file, block.selector)].append(block)
    out: list[model.Finding] = []
    for (file_id, selector), blocks in sorted(grouped.items()):
        first = min(blocks, key=lambda block: (_line(block.source), block.profile))
        dead = sorted({block.profile for block in blocks})
        slugs = _and([rules.slug_of(item) for item in dead])
        named = ("profiles " if len(dead) > 1 else "profile ") + slugs
        at = rules.where(first.source)
        live_in = sorted({item for block in blocks for item in block.live_in} - set(dead))
        unproven = [block for block in blocks if not block.proven]
        variant = "shared" if live_in else ("unproven" if len(unproven) == len(blocks) else "")
        severity = rules.severity_of("param.orphan_block", variant)
        caveat = ""
        if unproven:
            blind = sum(_unnamed(atlas, block.profile) for block in unproven)
            where = _and([rules.slug_of(block.profile) for block in unproven])
            caveat = (
                f" {blind} processes of {'profiles' if len(unproven) > 1 else 'profile'} "
                f"{where} carry no name rosio could form, so the block is dead there unless "
                "one of them answers to it."
            )
        if live_in:
            count = len(live_in)
            others = ", ".join(rules.slug_of(item) for item in live_in[:3])
            if count > 3:
                others += f" and {count - 3} more"
            launch = f"profile {others} launches" if count == 1 else f"profiles {others} launch"
            caveat += (
                f" The file is shared: {launch} a node that reads this block, so it is not "
                "dead configuration."
            )
        keys = _keys(len(set().union(*(block.keys for block in blocks))))
        out.append(
            model.Finding(
                rule="param.orphan_block",
                severity=severity,
                subject=file_id,
                discriminator=selector,
                title=f"block {selector} owns nothing in {slugs}",
                detail=(
                    f"The block {selector} carries {keys} and no node launched by {named} "
                    "answers to that name."
                ),
                fix=(
                    f"Launch the node in {named}, or delete the {selector} block once no "
                    "profile launches it."
                ),
                say=(
                    f"{at} configures a node called {selector} with {keys} and {named} "
                    f"launch{'' if len(dead) > 1 else 'es'} no node by that name or a similar "
                    f"one.{caveat} Launch the node there, or delete the block at {at} once no "
                    "profile launches it."
                ),
                profiles=dead,
                layers=[DECLARED],
                where=rules.sources(
                    [
                        block.source
                        for block in sorted(
                            blocks, key=lambda block: (_line(block.source), block.profile)
                        )
                    ]
                ),
            )
        )
    return out


def _and(names: list[str]) -> str:
    return names[0] if len(names) == 1 else f"{', '.join(names[:-1])} and {names[-1]}"


def silences(atlas: model.Atlas) -> list[dict[str, str]]:
    """The rules this module kept quiet over this atlas, and why.

    One row per rule, profile and reason, sorted, with the block keys the reason held back.
    `rosio/drift/plugins.py` exposes the same function, so a renderer walks
    `rules.FAMILIES` and asks each module that has one.
    """
    grouped: dict[tuple[str, str], list[str]] = defaultdict(list)
    for (profile, selector), reason in sorted(_gated(atlas).items()):
        grouped[(profile, reason)].append(selector)
    out: list[dict[str, str]] = []
    for (profile, reason), blocks in sorted(grouped.items()):
        listed = ", ".join(sorted(blocks))
        for rule in ("param.orphan_block", "param.owner_name_mismatch"):
            out.append(
                {
                    "rule": rule,
                    "profile": profile,
                    "reason": f"{reason}, so nothing is stated about {listed}",
                }
            )
    return sorted(out, key=lambda row: (row["rule"], row["profile"], row["reason"]))


__all__ = [
    "declared_unread",
    "orphan_block",
    "owner_gate",
    "owner_name_mismatch",
    "restated_identical",
    "set_undeclared",
    "shadowed",
    "silences",
    "type_changed",
]
