"""Middleware identity and router presence in one observed profile."""

from __future__ import annotations

import shlex

from rosio import ids, model
from rosio.drift import rules
from rosio.drift.nodes import disc_for, evidence_of, observed_profiles, profile_ids, slug, stated
from rosio.resolve import ros_args

OBSERVED = model.Layer.OBSERVED.value
ZENOH = "rmw_zenoh_cpp"
NODE_LIMIT = 8


def _profiles(atlas: model.Atlas) -> list[str]:
    profiles = set(observed_profiles(atlas))
    for run in atlas.by_kind("run"):
        for fact in run.facts:
            if fact.layer != OBSERVED or fact.confidence == "unknown":
                continue
            if fact.values.get("launchers"):
                profiles.add(fact.profile)
    if model.ANY_PROFILE in profiles and profile_ids(atlas):
        profiles.discard(model.ANY_PROFILE)
        profiles.update(profile_ids(atlas))
    return sorted(profiles)


def _runs(atlas: model.Atlas, profile: str) -> list[model.LayerFact]:
    return [
        fact
        for entity in atlas.by_kind("run")
        if (fact := stated(entity, OBSERVED, profile)) is not None
    ]


def _groups(atlas: model.Atlas, profile: str) -> dict[str, list[str]]:
    defaults = {
        fact.values["rmw_default"]
        for fact in _runs(atlas, profile)
        if fact.values.get("rmw_default") not in (None, "", "unknown", "default")
    }
    default = next(iter(defaults)) if len(defaults) == 1 else "default"
    groups: dict[str, list[str]] = {}
    named: set[tuple[str, str]] = set()
    for entity in atlas.by_kind("node"):
        fact = stated(entity, OBSERVED, profile)
        if fact is None or fact.values.get("internal"):
            continue
        rmw = fact.values.get("rmw")
        if not isinstance(rmw, str) or rmw in ("", "unknown"):
            continue
        if rmw == "default":
            rmw = default
        groups.setdefault(rmw, []).append(rules.node_text(entity.id))
        named.add((rmw, entity.id))
    for fact in _runs(atlas, profile):
        for row in fact.values.get("launchers", []):
            rmw = row.get("rmw")
            if not isinstance(rmw, str) or rmw in ("", "unknown"):
                continue
            rmw = default if rmw == "default" else rmw
            command = row["command"]
            tokens = shlex.split(command)
            if tokens[:2] == ["ros2", "run"] and len(tokens) >= 4:
                args = ros_args.parse_ros_args(tokens)
                # Without a name rule the node usually carries the executable's name, so a
                # node of that name on the same middleware is the one this command started.
                name = args.node_name or tokens[3]
                fqn = ids.normalise_ros_name(name, ids.normalise_namespace(args.namespace))
                if (rmw, ids.build_node(fqn)) in named:
                    continue
            groups.setdefault(rmw, []).append(command)
    return {rmw: sorted(set(names)) for rmw, names in groups.items()}


def _names(nodes: list[str]) -> str:
    labels = sorted(nodes)
    text = ", ".join(labels[:NODE_LIMIT])
    if len(labels) > NODE_LIMIT:
        text += f" ({len(labels) - NODE_LIMIT} more)"
    return text


def _subject(atlas: model.Atlas, profile: str) -> str:
    """The profile, or the observed run of it when only a probe wrote this atlas.

    `rosio probe` on a running system with no extraction behind it knows the profile by its
    slug alone, and a finding about an entity the atlas does not hold is refused.
    """
    if profile in atlas.entities:
        return profile
    runs = sorted(entity.id for entity in atlas.by_kind("run") if stated(entity, OBSERVED, profile))
    return runs[0] if runs else profile


def _finding(
    atlas: model.Atlas,
    rule: str,
    profile: str,
    title: str,
    detail: str,
    fix: str,
    facts: list[model.LayerFact],
    variant: str = "",
) -> model.Finding:
    return model.Finding(
        rule=rule,
        severity=rules.severity_of(rule, variant),
        subject=_subject(atlas, profile),
        title=title,
        detail=detail,
        say=f"{detail}; {fix}",
        fix=fix,
        discriminator=disc_for(profile),
        profiles=[profile],
        layers=[OBSERVED],
        where=evidence_of(*facts),
    )


@rules.register("rmw.mixed")
def rmw_mixed(atlas: model.Atlas) -> list[model.Finding]:
    """Two stated middleware implementations among nodes of one profile."""
    out = []
    for profile in _profiles(atlas):
        groups = _groups(atlas, profile)
        if len(groups) < 2:
            continue
        values = "; ".join(f"{rmw}: {_names(groups[rmw])}" for rmw in sorted(groups))
        named = sorted(set(groups) - {"default"})
        # No probe pass established what an unset RMW_IMPLEMENTATION runs, so a single named
        # implementation beside it is mixed only if the default is something else.
        unproven = len(named) == 1
        condition = f", unless the default middleware is {named[0]}" if unproven else ""
        facts = _runs(atlas, profile) + [
            fact
            for node in atlas.by_kind("node")
            if (fact := stated(node, OBSERVED, profile)) is not None
        ]
        out.append(
            _finding(
                atlas,
                "rmw.mixed",
                profile,
                f"{slug(profile)} uses mixed middleware",
                f"{slug(profile)} runs different middleware implementations ({values}){condition}",
                "start every process under one RMW_IMPLEMENTATION in the launch environment "
                "or the shell that started the odd ones out",
                [fact for fact in facts if fact is not None],
                variant="unproven" if unproven else "",
            )
        )
    return out


@rules.register("zenoh.router_unreachable")
def zenoh_router_unreachable(atlas: model.Atlas) -> list[model.Finding]:
    """Zenoh nodes with a process scan that found no router."""
    out = []
    for profile in _profiles(atlas):
        nodes = _groups(atlas, profile).get(ZENOH, [])
        runs = _runs(atlas, profile)
        router = {fact.values["zenoh_router"] for fact in runs if "zenoh_router" in fact.values}
        if not nodes or router != {"absent"}:
            continue
        out.append(
            _finding(
                atlas,
                "zenoh.router_unreachable",
                profile,
                f"{slug(profile)} has no Zenoh router",
                f"{slug(profile)} runs {ZENOH} nodes ({_names(nodes)}) and the process table "
                "records zenoh_router: absent",
                "start ros2 run rmw_zenoh_cpp rmw_zenohd or point the session config at a router",
                runs,
            )
        )
    return out


RULES = (("rmw.mixed", rmw_mixed), ("zenoh.router_unreachable", zenoh_router_unreachable))


def check(atlas: model.Atlas) -> list[model.Finding]:
    return sorted(
        [finding for _name, function in RULES for finding in function(atlas)],
        key=model.Finding.sort_key,
    )
