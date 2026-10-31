"""The ros2_control rule family: what a spawner asks a controller manager to load, and what a
loaded controller asks of the robot model.

Both rules read the profile's own statement of the manager, a `ros2_control_node` the launch
walk started and the `<controller>.type` keys its parameters files hand it, so a manager that
runs inside a simulator plugin, or one whose parameters the walk could not read, states
nothing and neither rule speaks for it.
"""

from __future__ import annotations

import difflib
from collections import defaultdict

from rosio import ids, model
from rosio.drift import build, rules

DECLARED = model.Layer.DECLARED.value

MANAGER = ("controller_manager", "ros2_control_node")
SPAWNER = ("controller_manager", "spawner")

_VALUED_FLAGS = frozenset(
    {
        "-c",
        "--controller-manager",
        "-n",
        "--namespace",
        "--controller-manager-timeout",
        "--switch-timeout",
        "--service-call-timeout",
        "--controller-ros-args",
        "--fallback_controllers",
    }
)
"""Spawner flags that take the next word as their value (jazzy `spawner.py`)."""

_OWN_DEFINITION = frozenset({"-p", "--param-file", "-t", "--controller-type"})
"""Flags with which a spawner hands the controller its own type or parameters file, so the
manager's configuration need not define it."""


def _executable(fact: model.LayerFact) -> tuple[str, str]:
    return str(fact.values.get("pkg") or ""), str(fact.values.get("executable") or "")


def _managers(atlas: model.Atlas) -> dict[str, list[str]]:
    """Profile to the controller managers the profile starts."""
    out: dict[str, list[str]] = defaultdict(list)
    for node in atlas.by_kind(model.NodeKind.NODE.value):
        for fact in node.facts:
            if fact.layer == DECLARED and _executable(fact) == MANAGER:
                out[fact.profile].append(node.id)
    return out


def _defined(atlas: model.Atlas, manager: str, profile: str) -> set[str]:
    """The controllers a `<name>.type` key defines for the manager in this profile."""
    out: set[str] = set()
    for node in atlas.by_kind(model.NodeKind.PARAM.value):
        parsed = ids.parse_param(node.id)
        name, _, last = parsed.name.rpartition(".")
        if parsed.owner != manager or last != "type" or not name or "." in name:
            continue
        if any(row.wins and row.profile == profile for row in node.candidates):
            out.add(name)
    return out


def _spawned(argv: list[str]) -> list[str]:
    """The controller names a spawner command line asks for, or none when it defines one
    itself."""
    names: list[str] = []
    words = iter(argv)
    for word in words:
        if word == "--ros-args":
            break
        if word in _OWN_DEFINITION:
            return []
        if word in _VALUED_FLAGS:
            next(words, None)
        elif not word.startswith("-"):
            names.append(word)
    return names


def _closest(name: str, known: set[str]) -> str:
    match = difflib.get_close_matches(name, sorted(known), n=1, cutoff=0.6)
    return match[0] if match else ""


@rules.register("controller.undefined")
def controller_undefined(atlas: model.Atlas) -> list[model.Finding]:
    """A spawner asks the controller manager for a controller no configuration defines.

    The manager loads a controller by its `<name>.type` parameter, so the spawner fails and the
    controller never runs. Decided only where the profile starts one manager whose
    configuration defines at least one controller, and silent on a spawner that hands the
    controller its own type or parameters file.
    """
    managers = _managers(atlas)
    found: dict[tuple[str, str], list[tuple[str, model.LayerFact, str]]] = defaultdict(list)
    for node in atlas.by_kind(model.NodeKind.NODE.value):
        for fact in node.facts:
            if fact.layer != DECLARED or _executable(fact) != SPAWNER:
                continue
            if not rules.usable(fact) or len(managers.get(fact.profile, [])) != 1:
                continue
            defined = _defined(atlas, managers[fact.profile][0], fact.profile)
            if not defined:
                continue
            for name in _spawned([str(word) for word in fact.values.get("argv") or []]):
                if name not in defined:
                    found[(node.id, name)].append((fact.profile, fact, _closest(name, defined)))
    out: list[model.Finding] = []
    for (spawner, name), rows in sorted(found.items()):
        profiles = sorted({profile for profile, _fact, _near in rows})
        fact = rows[0][1]
        near = rows[0][2]
        at = rules.where(fact.evidence[0] if fact.evidence else None)
        held = ", ".join(rules.slug_of(item) for item in profiles)
        hint = f" The configuration defines {near}, which is likely what was meant." if near else ""
        out.append(
            model.Finding(
                rule="controller.undefined",
                severity=rules.severity_of("controller.undefined"),
                subject=spawner,
                discriminator=name,
                title=f"the spawner at {at} loads {name}, which no configuration defines",
                detail=(
                    f"In {held} the spawner started at {at} asks the controller manager for "
                    f"{name}, and no parameters file the manager is handed sets "
                    f"{name}.type, so the spawner fails and the controller never runs.{hint}"
                ),
                fix=(
                    f"spawn {near} at {at}, or define {name} in the controller configuration"
                    if near
                    else f"define {name} with a type in the controller configuration, or "
                    f"remove it from the spawner at {at}"
                ),
                say=(
                    f"{at} spawns {name}, which no controller configuration defines in {held}, "
                    f"so the controller never loads.{hint}"
                ),
                profiles=profiles,
                layers=[DECLARED],
                where=rules.sources([row for _p, item, _n in rows for row in item.evidence]),
            )
        )
    return out


_JOINT_KEYS = ("joints", "joint")


def _model_joints(atlas: model.Atlas) -> dict[tuple[str, str], set[str]]:
    """(model file, profile or `*`) to the joint names the model states."""
    out: dict[tuple[str, str], set[str]] = defaultdict(set)
    for edge in atlas.edges.values():
        if edge.kind != model.EdgeKind.TF_PARENT.value:
            continue
        for fact in edge.facts:
            joint, owner = fact.values.get("joint"), fact.values.get("model")
            if fact.layer == DECLARED and joint and owner:
                out[(str(owner), fact.profile)].add(str(joint))
    return out


@rules.register("controller.joint_missing")
def controller_joint_missing(atlas: model.Atlas) -> list[model.Finding]:
    """A controller the manager loads lists a joint the robot model the profile loads lacks.

    The controller fails to configure, because the hardware exports no interface for that
    joint. Decided only for a profile that loads exactly one robot model whose joints were
    read, for the `joints` and `joint` keys of a controller the manager defines.
    """
    managers = _managers(atlas)
    loaded = build._loaded_models(atlas)
    joints = _model_joints(atlas)
    out: list[model.Finding] = []
    seen: dict[tuple[str, str], list[tuple[str, model.Candidate, str]]] = defaultdict(list)
    for profile, owners in sorted(managers.items()):
        models = sorted(item for item, profiles in loaded.items() if profile in profiles)
        if len(owners) != 1 or len(models) != 1:
            continue
        stated = joints.get((models[0], profile)) or joints.get((models[0], model.ANY_PROFILE))
        if not stated:
            continue
        namespace = (
            ids.parse_node(owners[0]).fqn.rpartition("/")[0]
            if not ids.is_synthetic(owners[0])
            else ""
        )
        for controller in sorted(_defined(atlas, owners[0], profile)):
            owner = ids.build_node(f"{namespace}/{controller}")
            for key in _JOINT_KEYS:
                entity = atlas.entities.get(ids.build_param(owner, key))
                chain = [
                    row for row in (entity.candidates if entity else []) if row.profile == profile
                ]
                if not chain:
                    continue
                # the manager hands the block to a controller no launch line names, so no
                # candidate wins and the one ranked first is the value it reads
                winner = min(chain, key=lambda row: row.rank)
                values = winner.value if isinstance(winner.value, list) else [winner.value]
                for joint in values:
                    if isinstance(joint, str) and joint and joint not in stated:
                        seen[(entity.id, joint)].append((profile, winner, models[0]))
    for (param_id, joint), rows in sorted(seen.items()):
        profiles = sorted({profile for profile, _w, _m in rows})
        winner, model_id = rows[0][1], rows[0][2]
        controller = rules.node_text(ids.parse_param(param_id).owner)
        at = rules.where(winner.source)
        held = ", ".join(rules.slug_of(item) for item in profiles)
        out.append(
            model.Finding(
                rule="controller.joint_missing",
                severity=rules.severity_of("controller.joint_missing"),
                subject=param_id,
                discriminator=joint,
                title=f"{controller} lists {joint}, which the robot model has no joint for",
                detail=(
                    f"In {held} {at} gives {controller} the joint {joint}, and "
                    f"{rules.path_of(model_id)}, the robot model the profile loads, declares no "
                    "joint of that name, so the hardware exports no interface for it and the "
                    "controller fails to configure."
                ),
                fix=f"remove {joint} from {at}, or add the joint to {rules.path_of(model_id)}",
                say=(
                    f"{at} lists {joint} for {controller}, and {rules.path_of(model_id)} has no "
                    f"such joint, so the controller fails to configure in {held}."
                ),
                profiles=profiles,
                layers=[DECLARED],
                where=rules.sources([row[1].source for row in rows]),
            )
        )
    return out
