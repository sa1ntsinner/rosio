"""The plugin rule family: what a yaml selector list loads, what it leaves behind, and what
it names that no manifest exports.

Plugin identity gates which parameter sub keys of a node are live, so this family is what
keeps the parameter family quiet. 55 parameters on the reference stack sit inside blocks
that no selector list names. They are configuration that was deliberately kept and
deliberately switched off, and reporting them one by one would be exactly the noise that
gets a report ignored, so `plugin.inert_by_design` reports the block once, at info, and the
parameter rules skip every key inside it.

`plugin.unused` is scoped to the base classes a yaml selector list chooses, which is data in
`rosio/collect/params.py`, and of those to the ones some read profile has a selector list
for: a profile that runs no controller server says nothing about controllers. An rviz tool is
exported the same way and is chosen inside an rviz config, so a rule that ignored the base
class would report it as unused every run.

`plugin.unresolved` fires only when the lookup name belongs to a package in the workspace,
because a manifest rosio never read is a blind spot and not a missing class.

Two things are gated here, and both gates are named in the report rather than left for a
reader to infer.

  * `plugin.unused` needs the selector lists of a profile, and a selector list arrives
    through that profile's launch walk. Coverage is therefore asked per profile: an entry
    point that did not load, a walk that stopped part way and an include that did not
    resolve each remove the evidence, and each removes it from the profile whose tree it
    belongs to and from no other. Gating over the whole atlas was wrong in both directions.
    One ROS 1 launch file in a package no profile launches silenced the rule everywhere,
    and a profile whose own tree was never read was spoken about anyway because the refusal
    was recorded on another profile's file. The finding now names the profiles it covers and
    the profiles it does not.
  * A launch file that belongs to no profile gates every profile, because a refusal nobody
    can attribute cannot be excluded from anywhere.

No rule in this module reasons about a node owning a parameter block, so none of them takes
the owner test that `rosio/drift/params.py` applies. `plugin.inert_by_design` reads
selector lists and blocks of one parameter document, which the declared layer holds in full,
and its owner is a node the same document names.

`silences(atlas)` reports the rules this module kept quiet and why, one row per rule and
profile, so a reader of a drift report is never left thinking a silent rule found nothing.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass

from rosio import ids, model
from rosio.collect.params import SELECTOR_BASE_CLASS
from rosio.drift import rules

DECLARED = model.Layer.DECLARED.value
PARAM = model.NodeKind.PARAM.value
LAUNCHFILE = model.NodeKind.LAUNCHFILE.value
INERT = model.ParamState.INERT_BY_DESIGN.value


def _one_key_per_base() -> dict[str, str]:
    grouped: dict[str, list[str]] = defaultdict(list)
    for key, base_class in sorted(SELECTOR_BASE_CLASS.items()):
        grouped[base_class].append(key)
    return {base: keys[0] for base, keys in grouped.items() if len(keys) == 1}


SELECTOR_KEY_OF_BASE = _one_key_per_base()
"""The one selector key a base class goes in, for the base classes that have exactly one.
A base class two keys spell is absent, because picking one of two spellings is a guess."""


def _entry_points(atlas: model.Atlas, profile: str) -> list[model.Node]:
    """The launch files whose read state decides what one profile knows.

    A file carrying no profile at all is in every list: a refusal nobody can attribute is a
    refusal that cannot be excluded from anywhere.
    """
    return [
        node
        for node in atlas.by_kind(LAUNCHFILE)
        if not profile or not node.profiles or profile in node.profiles
    ]


def _unread(node: model.Node, profile: str) -> bool:
    """Whether the walk stopped before this file was read to the end, in this profile."""
    if node.attrs.get("loadable") is False:
        return True
    for fact in node.facts:
        if profile and fact.profile not in (profile, model.ANY_PROFILE):
            continue
        values = fact.values
        if values.get("loadable") is False or values.get("partial") or values.get("raised"):
            return True
    return False


def unread_entry_points(atlas: model.Atlas, profile: str = "") -> list[str]:
    """The launch files of one profile that the walk did not read to the end.

    `plugin.unused` says that nothing selects a class. The thing that would have selected it
    is a parameter file, and a parameter file arrives through the launch walk, so an entry
    point that did not load, a walk that stopped part way and an include that did not
    resolve each remove the evidence the rule needs while leaving the rule able to fire. On
    a run where the launch machinery could not be imported at all, eight of twenty three
    findings of this rule were manufactured that way. The connectivity rules are gated on
    the observed layer for the same reason, and this is the same gate one layer down.

    An empty profile asks about the whole atlas, which is what a caller with no profile in
    hand means.
    """
    found = {node.id for node in _entry_points(atlas, profile) if _unread(node, profile)}
    owners = {node.id for node in _entry_points(atlas, profile)}
    found.update(
        entry.owner
        for entry in atlas.unresolved
        if entry.role == "include" and (not profile or entry.owner in owners)
    )
    return sorted(found)


def launch_path(entity_id: str) -> str:
    """A launch file id as an engineer types it. `rules.path_of` takes a file id, and an
    entry point is a launchfile id, which is a different kind."""
    parsed = ids.parse_launchfile(entity_id)
    return f"{parsed.package}/{parsed.relpath}"


def launch_tree_read(atlas: model.Atlas, profile: str = "") -> bool:
    """Whether every launch file one profile names was actually read."""
    return not unread_entry_points(atlas, profile)


def profile_coverage(atlas: model.Atlas) -> tuple[list[str], dict[str, list[str]]]:
    """The profiles whose launch tree was read, and the unread files of the rest."""
    read: list[str] = []
    blind: dict[str, list[str]] = {}
    for profile in rules.profiles_of(atlas):
        unread = unread_entry_points(atlas, profile)
        if unread:
            blind[profile] = unread
        else:
            read.append(profile)
    return read, blind


def _line(source: model.Source) -> int:
    """Line hints order a block's rows so the finding cites where the block opens."""
    return source.line_hint if source.line_hint is not None else 1 << 30


def _keys(count: int) -> str:
    return "1 key" if count == 1 else f"{count} keys"


@dataclass(frozen=True)
class _InertBlock:
    profile: str
    owner: str
    block: str
    selector: str
    file: str
    keys: tuple[str, ...]
    source: model.Source
    plugin: str


def _lookup(node: model.Node, profile: str, values: dict, name: str, block: str) -> str:
    """The class the block would load. The selector key of the block carries it as its own
    value, so a fact that does not name the class is not a reason to leave it out."""
    stated = str(values.get("plugin") or "")
    if stated or name != f"{block}.plugin":
        return stated
    for row in sorted(node.candidates, key=lambda item: item.rank):
        if row.profile == profile and isinstance(row.value, str):
            return row.value
    return ""


def _base_classes(atlas: model.Atlas) -> dict[str, set[str]]:
    """Every base class each lookup name is exported or selected under."""
    out: dict[str, set[str]] = defaultdict(set)
    for plugin in _plugins(atlas):
        parsed = ids.parse_plugin(plugin.id)
        out[parsed.lookup].add(parsed.base_class)
    return out


def _recovered_selector(block: str, plugin: str, bases: dict[str, set[str]]) -> str:
    """The selector key a block belongs in, from the class the block itself names.

    A block claimed by no list carries no selector key, and naming the first list the node
    happens to hold is a guess. The class the block declares is not a guess: it is exported
    under one base class, and a base class that exactly one selector key spells names that
    key. Two base classes for one lookup name, or two spellings of one key, and this returns
    nothing and the finding says the block is claimed by no list.
    """
    found = bases.get(plugin) or set()
    if len(found) != 1:
        return ""
    key = SELECTOR_KEY_OF_BASE.get(next(iter(found)), "")
    if not key:
        return ""
    prefix = block.rsplit(".", 1)[0] if "." in block else ""
    return f"{prefix}.{key}" if prefix else key


def _inert_blocks(atlas: model.Atlas) -> list[_InertBlock]:
    rows: dict[tuple[str, str, str, str], list[tuple[str, model.Source, str]]] = defaultdict(list)
    for key in sorted(atlas.entities):
        node = atlas.entities[key]
        if node.kind != PARAM:
            continue
        for fact in node.facts:
            values = fact.values
            if fact.layer != DECLARED or not rules.usable(fact):
                continue
            if values.get("state") != INERT:
                continue
            site = rules.site(node, fact.profile, fact)
            if site is None:
                continue
            block = str(values.get("block") or "")
            owner = str(values.get("owner") or "")
            selector = str(values.get("selector") or "")
            if not block or not owner:
                continue
            name = str(values.get("name") or ids.parse_param(node.id).name)
            rows[(fact.profile, owner, block, selector)].append(
                (name, site, _lookup(node, fact.profile, values, name, block))
            )
    bases = _base_classes(atlas)
    out: list[_InertBlock] = []
    for (profile, owner, block, selector), found in sorted(rows.items()):
        first = min(found, key=lambda row: (_line(row[1]), row[0]))[1]
        plugin = next((row[2] for row in sorted(found) if row[2]), "")
        out.append(
            _InertBlock(
                profile=profile,
                owner=owner,
                block=block,
                selector=selector or _recovered_selector(block, plugin, bases),
                file=first.file,
                keys=tuple(sorted(row[0] for row in found)),
                source=first,
                plugin=plugin,
            )
        )
    return out


@rules.register("plugin.inert_by_design")
def inert_by_design(atlas: model.Atlas) -> list[model.Finding]:
    """A fully configured plugin block that no selector list names.

    Naming the first list the node happens to hold used to fill the gap, which is how this
    rule came to tell an engineer to add a progress checker to the controller list. The key
    is recovered from the class the block itself declares instead, and only when that class
    determines one key. When it does not, the sentence says the block is claimed by no list
    and names none.
    """
    out: list[model.Finding] = []
    grouped: dict[tuple[str, str, str], list] = {}
    for item in _inert_blocks(atlas):
        grouped.setdefault((item.file, item.owner, item.block), []).append(item)
    # One deletable block is one finding, with the profiles it is inert in.
    for key in sorted(grouped):
        blocks = sorted(grouped[key], key=lambda item: item.profile)
        block = blocks[0]
        profiles = sorted({item.profile for item in blocks})
        slug = ", ".join(rules.slug_of(item) for item in profiles)
        at = rules.where(block.source)
        named = bool(block.selector)
        misses = (
            f"{block.selector} does not name {block.block}"
            if named
            else f"no selector list of {block.owner} names {block.block}"
        )
        target = block.selector if named else f"a selector list of {block.owner}"
        loads = f" It would load {block.plugin}." if block.plugin else ""
        out.append(
            model.Finding(
                rule="plugin.inert_by_design",
                severity=rules.severity_of("plugin.inert_by_design"),
                subject=block.file,
                discriminator=f"{block.owner.strip('/').replace('/', '.')}.{block.block}",
                title=f"block {block.block} is configured and not selected in {slug}",
                detail=(
                    f"{block.owner} carries {_keys(len(block.keys))} under {block.block}, and "
                    f"{misses}, so none of them is loaded in {_profile_word(profiles)} "
                    f"{slug}.{loads}"
                ),
                fix=(
                    f"Add {block.block} to {target} to switch it on, or delete the block to "
                    "stop it reading as configuration."
                ),
                say=(
                    f"{at} configures {block.owner} block {block.block} with "
                    f"{_keys(len(block.keys))}, and {misses}, so none of it is loaded in "
                    f"{_profile_word(profiles)} {slug}. Add {block.block} to {target} at "
                    f"{rules.path_of(block.file)} or delete the block."
                ),
                profiles=profiles,
                layers=[DECLARED],
                where=[block.source],
            )
        )
    return out


def _profile_word(profiles: list[str]) -> str:
    return "profile" if len(profiles) == 1 else "profiles"


def _plugins(atlas: model.Atlas) -> list[model.Node]:
    return [
        atlas.entities[key]
        for key in sorted(atlas.entities)
        if atlas.entities[key].kind == model.NodeKind.PLUGIN.value
    ]


def _exported(atlas: model.Atlas, plugin_id: str) -> str:
    """The package whose manifest exports this class, empty when no manifest names it."""
    for edge in atlas.in_edges(plugin_id):
        if edge.kind == model.EdgeKind.PROVIDES_PLUGIN.value:
            return ids.parse_pkg(edge.src)
    return ""


def _instantiated(atlas: model.Atlas, plugin_id: str) -> list[str]:
    return sorted(
        edge.src
        for edge in atlas.in_edges(plugin_id)
        if edge.kind == model.EdgeKind.INSTANTIATES.value
    )


def _manifest(plugin: model.Node) -> model.Source | None:
    for fact in sorted(plugin.facts, key=model.LayerFact.sort_key):
        if fact.layer == DECLARED and rules.usable(fact) and fact.evidence:
            return fact.evidence[0]
    return None


def _selector_for(base_class: str) -> str:
    """The selector key a class of this base class goes in. Two spellings of one key are
    both named, because which one a document uses is the document's choice and telling an
    engineer to add a line to a key their file does not spell is a wrong instruction."""
    found = sorted(key for key, value in SELECTOR_BASE_CLASS.items() if value == base_class)
    return " or ".join(found)


def _profile_text(profiles: list[str]) -> str:
    """`profile a`, or `profiles a, b and c`. A `say` is quoted verbatim, so a singular noun
    in front of three slugs is read as one profile with a strange name."""
    names = sorted(rules.slug_of(item) for item in profiles)
    if len(names) < 2:
        return f"profile {''.join(names)}"
    return "profiles " + ", ".join(names[:-1]) + " and " + names[-1]


@rules.register("plugin.unused")
def unused(atlas: model.Atlas) -> list[model.Finding]:
    """A plugin this workspace builds and exports that no profile selects.

    A manifest read from an install tree with no source copy is not built here, and a plugin
    an installed release exports is not this workspace's to stop exporting, so it is skipped.

    The claim is made for the profiles whose launch tree was read and for no others. A
    profile that did select the class keeps the rule quiet whether its tree was read or not,
    because a selection is positive evidence and needs no completeness.
    """
    out: list[model.Finding] = []
    read, blind = profile_coverage(atlas)
    if not read:
        return out
    unread = (
        ""
        if not blind
        else (
            f" The launch tree of {_profile_text(sorted(blind))} was not read to the end, so "
            f"nothing here covers {'it' if len(blind) == 1 else 'them'}."
        )
    )
    loaded = _loaded_bases(atlas, read)
    for plugin in _plugins(atlas):
        parsed = ids.parse_plugin(plugin.id)
        if parsed.base_class not in loaded:
            continue
        # Only the profiles that load this base class could have selected it.
        covers = _profile_text(loaded[parsed.base_class])
        package = _exported(atlas, plugin.id)
        if not package or _instantiated(atlas, plugin.id):
            continue
        source = _manifest(plugin)
        if source is not None and source.root == model.Root.INSTALL.value:
            # Read from an install tree with no source beside it: a package this workspace
            # does not build, such as one apt installed, so there is nothing to stop exporting.
            continue
        at = rules.where(source)
        key = _selector_for(parsed.base_class)
        selector = key or "selector"
        add_to = key or "a selector list"
        out.append(
            model.Finding(
                rule="plugin.unused",
                severity=rules.severity_of("plugin.unused"),
                subject=plugin.id,
                title=f"{parsed.lookup} is built and never selected",
                detail=(
                    f"{package} exports {parsed.lookup} for {parsed.base_class}, and no "
                    f"{selector} entry of {covers} names it, so nothing loads the "
                    f"library.{unread}"
                ),
                fix=(
                    f"Add {parsed.lookup} to {add_to} of the profile that should run it. A "
                    "plugin a package offers as one choice among several needs no change."
                ),
                say=(
                    f"{package} exports the plugin {parsed.lookup} at {at} and no {selector} "
                    f"list of {covers} selects it, so nothing loads it there. Add it to "
                    f"{add_to} if a profile should run it."
                ),
                profiles=list(loaded[parsed.base_class]),
                layers=plugin.layers,
                where=[source] if source else [],
            )
        )
    return out


def _loaded_bases(atlas: model.Atlas, profiles: list[str]) -> dict[str, list[str]]:
    """The base classes some node of these profiles loads from a selector list, each with
    the profiles that load it.

    A profile that runs no controller server has no `controller_plugins` list, and a
    controller nothing could load is not unused, it is out of scope: robot_localization
    launched next to an installed nav2 tree would otherwise call every nav2 plugin unused.
    """
    wanted = set(profiles)
    out: dict[str, set[str]] = {}
    for param in atlas.by_kind(PARAM):
        base = SELECTOR_BASE_CLASS.get(ids.parse_param(param.id).name.split(".")[-1])
        if not base:
            continue
        # A shared file names the list in profiles that never start its node.
        owner = atlas.entities.get(ids.parse_param(param.id).owner)
        running = {item for item in (owner.profiles if owner else ()) if item != "*"} or wanted
        found = wanted & set(param.profiles) & running
        if found:
            out.setdefault(base, set()).update(found)
    return {base: sorted(found) for base, found in out.items()}


def _package_of(lookup: str) -> str:
    if "/" in lookup:
        return lookup.split("/", 1)[0]
    if "::" in lookup:
        return lookup.split("::", 1)[0]
    return ""


@rules.register("plugin.unresolved")
def unresolved(atlas: model.Atlas) -> list[model.Finding]:
    """A selected lookup name that the owning package's manifest does not export.

    Only a package this atlas read is judged. A nav2 lookup name whose manifest lives in an
    installed package rosio never opened is a blind spot, and reporting it would fail the
    gate on every well configured stack.
    """
    out: list[model.Finding] = []
    for plugin in _plugins(atlas):
        parsed = ids.parse_plugin(plugin.id)
        package = _package_of(parsed.lookup)
        if not package or ids.build_pkg(package) not in atlas.entities:
            continue
        if _exported(atlas, plugin.id):
            continue
        for instance_id in _instantiated(atlas, plugin.id):
            instance = atlas.get(instance_id)
            if instance is None:
                continue
            owner, name = ids.parse_pinst(instance_id)
            for profile in sorted(instance.profiles):
                fact = instance.fact(DECLARED, profile)
                if not rules.usable(fact):
                    continue
                slug = rules.slug_of(profile)
                at = rules.where(fact.evidence[0] if fact.evidence else None)
                owner_text = rules.node_text(owner)
                out.append(
                    model.Finding(
                        rule="plugin.unresolved",
                        severity=rules.severity_of("plugin.unresolved"),
                        subject=plugin.id,
                        # The owner is part of the site: two servers selecting one
                        # missing class are two lines to edit.
                        discriminator=f"{slug}{ids.label_for(owner)}.{name}",
                        title=f"{parsed.lookup} is selected and not exported",
                        detail=(
                            f"{owner_text} selects {parsed.lookup} as {name} in profile "
                            f"{slug}, and the pluginlib manifest of {package} exports no "
                            "class under that lookup name, so pluginlib cannot load it."
                        ),
                        fix=(
                            f"Fix the lookup name at the selector site, or export "
                            f"{parsed.lookup} from {package}."
                        ),
                        say=(
                            f"Profile {slug} selects the plugin {parsed.lookup} as {name} for "
                            f"{owner_text} at {at}, and {package} exports no class with that "
                            "lookup name, so the node fails when it configures. Fix the name "
                            f"at {at} or export the class from {package}."
                        ),
                        profiles=[profile],
                        layers=[DECLARED],
                        where=list(fact.evidence),
                    )
                )
    return out


def silences(atlas: model.Atlas) -> list[dict[str, str]]:
    """The rules this module kept quiet over this atlas, and why.

    One row per rule and profile, sorted, with `rule`, `profile` (empty when the reason
    holds for the whole atlas) and `reason`. A drift report that lists a silent rule without
    this row tells a reader the rule found nothing, which is a different statement.
    `rosio/drift/params.py` exposes the same function, so a renderer walks
    `rules.FAMILIES` and asks each module that has one.

    One fact is stated once. A profile whose tree was not read has its own row, so the atlas
    wide row is only for an atlas that holds no profile to speak about at all. Emitting both
    made a renderer that prints every row say the same thing twice.
    """
    read, blind = profile_coverage(atlas)
    out = [
        {
            "rule": "plugin.unused",
            "profile": profile,
            "reason": (
                "the launch tree of this profile was not read to the end ("
                + ", ".join(launch_path(item) for item in sorted(files))
                + "), so a selector list that names the class may be missing"
            ),
        }
        for profile, files in sorted(blind.items())
    ]
    if not read and not blind:
        out.append(
            {
                "rule": "plugin.unused",
                "profile": "",
                "reason": (
                    "this atlas holds no profile, so nothing states which classes are selected"
                ),
            }
        )
    return sorted(out, key=lambda row: (row["rule"], row["profile"], row["reason"]))


__all__ = [
    "inert_by_design",
    "launch_path",
    "launch_tree_read",
    "profile_coverage",
    "silences",
    "unread_entry_points",
    "unresolved",
    "unused",
]
