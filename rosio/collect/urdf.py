"""urdf and xacro: the frame tree, joint geometry and gazebo plugin remaps.

Three properties of real robot descriptions shape this module.

* A workspace holds several models that share one robot name. Keying a model by that name
  merges different robots, so the model identity here is the file that declared it, and every
  tf_parent edge carries that model in its key and in its values. A single valued origin
  publishes a wrong number as fact for two profiles out of three.
* A frame is only a frame when a link declares it. A joint whose parent link is never
  declared, and the child it leaves stranded, are unresolved entries, never invented frames.
* A gazebo plugin block is a real remap source: `<ros><remapping>` rewires topics with no
  launch file in sight, which is why those remaps are emitted with scope urdf_plugin.

xacro is handled by running the xacro tool in a subprocess, because reimplementing its macro
and property expansion would be a second source of truth. When the tool is missing, or when a
model needs an argument nobody supplied, the file is refused with a note and the blind spot is
recorded. It is never silently skipped.

Every refusal leaves the same shape, a urdf that is not parsable xml included: the file entity
is there and says it is not loadable, it hangs off its package, and one blind spot under the
frame want states the loss. A refused file that left nothing behind took its frames and the
whole frame want out of the resolution counter, so the atlas read healthiest where the loss was
worst.
"""

from __future__ import annotations

import re
import shutil
import subprocess
from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from xml.etree import ElementTree

from rosio import ids, model, workspace
from rosio.collect import CollectContext, register
from rosio.errors import CollectorError, ContractError
from rosio.resolve import records

LAYER = model.Layer.DECLARED.value
NAME = "urdf"

URDF_SUFFIX = ".urdf"
XACRO_SUFFIX = ".xacro"

XACRO_NAMESPACES = ("{http://www.ros.org/wiki/xacro}", "{http://ros.org/wiki/xacro}")
"""Both spellings the xacro tool accepts. jazzy's own nav2 descriptions bind the prefix to the
one without `www`, and reading only the other turned every declared argument into a missing
one."""

XACRO_TIMEOUT = 60.0

UNKNOWN = "unknown"
"""What a numeric attribute reads as when it is written and is not a number."""

REFUSAL_ROLES = {
    "urdf_unreadable": "urdf",
    "xacro_unreadable": "xacro",
    "xacro_argument_missing": "xacro",
    "xacro_tool_missing": "xacro",
    "xacro_failed": "xacro",
}
"""Why a description file gave nothing, and which reader met it."""

REFUSAL_FIXES = {
    "urdf_unreadable": ("write the model as parsable xml with a robot root",),
    "xacro_unreadable": ("write the model as parsable xml",),
    "xacro_argument_missing": ("supply the argument in the profile",),
    "xacro_tool_missing": ("install the xacro tool",),
    "xacro_failed": ("fix what the xacro tool refused",),
}

XACRO_DIRECTIVES = frozenset(
    {
        "arg",
        "attribute",
        "call",
        "element",
        "if",
        "include",
        "insert_block",
        "insert_block_list",
        "macro",
        "property",
        "unless",
    }
)
"""Everything else in the xacro namespace is a macro call."""

_PACKAGE_URI = re.compile(r"^package://([^/]+)/(.+)$")
_FILE_URI = "file://"
_ARG_USE = re.compile(r"\$\(arg\s+([A-Za-z_][A-Za-z0-9_]*)\s*\)")
_FIND_USE = re.compile(r"\$\(find(?:-pkg-share)?\s+([A-Za-z0-9_]+)\s*\)")

PackageResolver = Callable[[str], "str | None"]


# ---------------------------------------------------------------------------
# records


@dataclass(frozen=True)
class Joint:
    name: str
    type: str
    parent: str
    child: str
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    axis: tuple[float, float, float] | None = None
    limit: tuple[tuple[str, float], ...] = ()
    mimic: tuple[tuple[str, Any], ...] = ()
    unparsed: tuple[tuple[str, str], ...] = ()
    """Attributes that are present and are not numbers, with the text as written. An
    absent origin is the urdf identity, an unreadable one is unknown, and the two must
    not report the same zeros."""
    line_hint: int | None = None

    def view(self) -> dict[str, Any]:
        out: dict[str, Any] = {
            "type": self.type,
            "parent": self.parent,
            "child": self.child,
            "xyz": list(self.xyz),
            "rpy": list(self.rpy),
        }
        if self.axis is not None:
            out["axis"] = list(self.axis)
        if self.limit:
            out["limit"] = dict(self.limit)
        if self.mimic:
            out["mimic"] = dict(self.mimic)
        if self.unparsed:
            out["unparsed"] = dict(self.unparsed)
        return out

    def origin_view(self) -> dict[str, Any]:
        """Origin for comparison. An unreadable component says unknown, never a zero."""
        bad = dict(self.unparsed)
        return {
            "rpy": UNKNOWN if "origin.rpy" in bad else list(self.rpy),
            "xyz": UNKNOWN if "origin.xyz" in bad else list(self.xyz),
        }


@dataclass(frozen=True)
class GazeboPlugin:
    name: str
    filename: str
    namespace: str | None = None
    remappings: tuple[str, ...] = ()
    reference: str | None = None
    sensor: str | None = None
    frame_name: str | None = None
    line_hint: int | None = None

    def view(self) -> dict[str, Any]:
        out: dict[str, Any] = {
            "name": self.name,
            "filename": self.filename,
            "namespace": self.namespace,
            "remappings": list(self.remappings),
            "reference": self.reference,
            "frame_name": self.frame_name,
        }
        if self.sensor is not None:
            out["sensor"] = self.sensor
        return out


@dataclass(frozen=True)
class Mesh:
    filename: str
    package: str | None
    path: str | None
    exists: bool

    def view(self) -> dict[str, Any]:
        return {"filename": self.filename, "package": self.package, "exists": self.exists}


@dataclass(frozen=True)
class Frame:
    name: str
    declared: bool
    joint: str | None
    parent: str | None

    def view(self) -> dict[str, Any]:
        return {"declared": self.declared, "joint": self.joint, "parent": self.parent}


@dataclass
class UrdfModel:
    """One model, addressed by the file that declared it and never by its robot name."""

    path: str
    robot_name: str | None
    links: tuple[str, ...] = ()
    joints: tuple[Joint, ...] = ()
    gazebo: tuple[GazeboPlugin, ...] = ()
    meshes: tuple[Mesh, ...] = ()
    strategy: str = "urdf"
    xacro_args: tuple[tuple[str, str], ...] = ()
    notes: tuple[str, ...] = ()
    shapes: tuple[tuple[str, dict[str, Any]], ...] = ()
    """Per declared link, its visual and collision rows as the model states them."""
    link_lines: tuple[tuple[str, int | None], ...] = ()

    def joint_map(self) -> dict[str, Joint]:
        return {joint.name: joint for joint in self.joints}

    def frames(self) -> dict[str, Frame]:
        declared = set(self.links)
        out: dict[str, Frame] = {
            name: Frame(name=name, declared=True, joint=None, parent=None) for name in declared
        }
        for joint in self.joints:
            for side in (joint.parent, joint.child):
                if side and side not in out:
                    out[side] = Frame(name=side, declared=False, joint=None, parent=None)
            if joint.child:
                out[joint.child] = Frame(
                    name=joint.child,
                    declared=joint.child in declared,
                    joint=joint.name,
                    parent=joint.parent,
                )
        return dict(sorted(out.items()))

    def roots(self) -> list[str]:
        return sorted(name for name, frame in self.frames().items() if frame.parent is None)

    def undeclared_parent_links(self) -> list[str]:
        declared = set(self.links)
        return sorted({joint.parent for joint in self.joints if joint.parent not in declared})

    def orphan_children(self) -> list[str]:
        declared = set(self.links)
        return sorted(
            {
                joint.child
                for joint in self.joints
                if joint.parent not in declared and joint.child in declared
            }
        )

    def view(self, relpath: str = "") -> dict[str, Any]:
        return {
            "path": relpath or self.path,
            "robot_name": self.robot_name,
            "links": sorted(self.links),
            "link_count": len(self.links),
            "joints": {joint.name: joint.view() for joint in sorted(self.joints, key=_joint_key)},
            "joint_count": len(self.joints),
            "frames": {name: frame.view() for name, frame in self.frames().items()},
            "roots": self.roots(),
            "undeclared_parent_links": self.undeclared_parent_links(),
            "orphan_children": self.orphan_children(),
            "gazebo_plugins": [plugin.view() for plugin in self.gazebo],
            "meshes": [mesh.view() for mesh in self.meshes],
        }


def _joint_key(joint: Joint) -> str:
    return joint.name


# ---------------------------------------------------------------------------
# urdf parsing


def parse_urdf_text(
    text: str,
    *,
    path: str = "",
    resolve_package: PackageResolver | None = None,
) -> UrdfModel:
    try:
        root = ElementTree.fromstring(text)
    except ElementTree.ParseError as exc:
        raise CollectorError(f"{path or 'urdf'} is not parsable xml: {exc}", LAYER, NAME) from exc
    if root.tag != "robot":
        raise CollectorError(f"{path or 'urdf'} root element is {root.tag}, not robot", LAYER, NAME)
    lines = _LineIndex(text)
    links = tuple(
        sorted(
            (element.get("name") or "").strip()
            for element in root.findall("link")
            if (element.get("name") or "").strip()
        )
    )
    joints = tuple(_joint(element, lines) for element in root.findall("joint"))
    joints = tuple(joint for joint in joints if joint is not None)
    gazebo = tuple(sorted(_gazebo(root, lines), key=lambda item: (item.name, item.filename)))
    materials = {
        name: _material(element, {}, [])
        for element in root.findall("material")
        if (name := (element.get("name") or "").strip())
    }
    shapes: dict[str, dict[str, Any]] = {}
    for element in root.findall("link"):
        name = (element.get("name") or "").strip()
        if name and name not in shapes:
            shapes[name] = {
                "visuals": _shapes(element, "visual", materials),
                "collisions": _shapes(element, "collision", materials),
            }
    return UrdfModel(
        path=path,
        robot_name=(root.get("name") or "").strip() or None,
        links=links,
        joints=joints,
        gazebo=gazebo,
        meshes=tuple(_meshes(root, resolve_package)),
        shapes=tuple(sorted(shapes.items())),
        link_lines=tuple((name, lines.find("link", name)) for name in links),
    )


def parse_urdf_file(path: str, *, resolve_package: PackageResolver | None = None) -> UrdfModel:
    try:
        text = Path(path).read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        raise CollectorError(f"cannot read {path}: {exc}", LAYER, NAME) from exc
    return parse_urdf_text(text, path=path, resolve_package=resolve_package)


class _LineIndex:
    """Advisory line numbers for a named element. Never compared, humans only."""

    def __init__(self, text: str) -> None:
        self._lines = text.splitlines()

    def find(self, tag: str, name: str) -> int | None:
        needle = f'name="{name}"'
        opener = f"<{tag}"
        for offset, line in enumerate(self._lines, start=1):
            if opener in line and needle in line:
                return offset
        return None


def _joint(element: ElementTree.Element, lines: _LineIndex) -> Joint | None:
    name = (element.get("name") or "").strip()
    parent = element.find("parent")
    child = element.find("child")
    if not name or parent is None or child is None:
        return None
    origin = element.find("origin")
    axis = element.find("axis")
    limit = element.find("limit")
    mimic = element.find("mimic")
    unparsed: list[tuple[str, str]] = []
    xyz = _triple(origin.get("xyz") if origin is not None else None, "origin.xyz", unparsed)
    rpy = _triple(origin.get("rpy") if origin is not None else None, "origin.rpy", unparsed)
    axis_values = _triple(axis.get("xyz"), "axis.xyz", unparsed) if axis is not None else None
    limit_values: list[tuple[str, float]] = []
    if limit is not None:
        for key in ("lower", "upper", "effort", "velocity"):
            raw = limit.get(key)
            if raw is None:
                continue
            value = _number(raw)
            if value is None:
                unparsed.append((f"limit.{key}", raw))
                continue
            limit_values.append((key, value))
    mimic_values: list[tuple[str, Any]] = []
    if mimic is not None:
        mimic_values = [("joint", (mimic.get("joint") or "").strip())]
        for key, default in (("multiplier", 1.0), ("offset", 0.0)):
            raw = mimic.get(key)
            value = default if raw is None else _number(raw)
            if value is None:
                unparsed.append((f"mimic.{key}", str(raw)))
                continue
            mimic_values.append((key, value))
    return Joint(
        name=name,
        type=(element.get("type") or "").strip(),
        parent=(parent.get("link") or "").strip(),
        child=(child.get("link") or "").strip(),
        xyz=xyz,
        rpy=rpy,
        axis=axis_values,
        limit=tuple(sorted(limit_values)),
        mimic=tuple(mimic_values),
        unparsed=tuple(sorted(unparsed)),
        line_hint=lines.find("joint", name),
    )


def _gazebo(root: ElementTree.Element, lines: _LineIndex) -> Iterable[GazeboPlugin]:
    for block in root.findall("gazebo"):
        reference = (block.get("reference") or "").strip() or None
        for plugin in block.findall("plugin"):
            found = _plugin(plugin, reference, None, lines)
            if found is not None:
                yield found
        for sensor in block.findall("sensor"):
            sensor_name = (sensor.get("name") or "").strip() or None
            for plugin in sensor.findall("plugin"):
                found = _plugin(plugin, reference, sensor_name, lines)
                if found is not None:
                    yield found


def _plugin(
    element: ElementTree.Element,
    reference: str | None,
    sensor: str | None,
    lines: _LineIndex,
) -> GazeboPlugin | None:
    name = (element.get("name") or "").strip()
    filename = (element.get("filename") or "").strip()
    if not name and not filename:
        return None
    ros = element.find("ros")
    namespace = None
    remappings: list[str] = []
    if ros is not None:
        namespace_node = ros.find("namespace")
        if namespace_node is not None and (namespace_node.text or "").strip():
            namespace = (namespace_node.text or "").strip()
        for remap in ros.findall("remapping"):
            text = (remap.text or "").strip()
            if text:
                remappings.append(text)
    frame_node = element.find("frame_name")
    frame_name = (frame_node.text or "").strip() if frame_node is not None else None
    return GazeboPlugin(
        name=name,
        filename=filename,
        namespace=namespace,
        remappings=tuple(remappings),
        reference=reference,
        sensor=sensor,
        frame_name=frame_name or None,
        line_hint=lines.find("plugin", name),
    )


def _meshes(root: ElementTree.Element, resolve_package: PackageResolver | None) -> list[Mesh]:
    out: dict[str, Mesh] = {}
    for mesh in root.iter("mesh"):
        filename = (mesh.get("filename") or "").strip()
        if not filename or filename in out:
            continue
        match = _PACKAGE_URI.match(filename)
        package = match.group(1) if match else None
        path: str | None = None
        exists = False
        if match and resolve_package is not None:
            base = resolve_package(package or "")
            if base:
                path = str(Path(base) / match.group(2))
                exists = Path(path).is_file()
        elif _local(filename):
            path = _local(filename)
            exists = Path(path).is_file()
        out[filename] = Mesh(filename=filename, package=package, path=path, exists=exists)
    return [out[key] for key in sorted(out)]


SHAPE_NUMBERS = {
    "box": (("size", 3),),
    "cylinder": (("radius", 1), ("length", 1)),
    "capsule": (("radius", 1), ("length", 1)),
    "sphere": (("radius", 1),),
    "mesh": (("scale", 3),),
}
"""The numeric attributes of each urdf geometry, with how many numbers each holds."""


def _shapes(
    link: ElementTree.Element, tag: str, materials: Mapping[str, dict[str, Any]]
) -> list[dict[str, Any]]:
    """Visual or collision rows. Only what is written is recorded: an absent origin is the
    urdf identity and stays absent, a number that does not read is named in `unreadable`."""
    rows = []
    for block in link.findall(tag):
        row: dict[str, Any] = {}
        unreadable: list[str] = []
        name = (block.get("name") or "").strip()
        if name:
            row["name"] = name
        origin = block.find("origin")
        if origin is not None:
            row["origin"] = _stated(origin, (("xyz", 3), ("rpy", 3)), "origin", unreadable)
        geometry = block.find("geometry")
        shape = next(iter(geometry), None) if geometry is not None else None
        if shape is not None and isinstance(shape.tag, str):
            found = {"type": shape.tag}
            found.update(_stated(shape, SHAPE_NUMBERS.get(shape.tag, ()), shape.tag, unreadable))
            filename = (shape.get("filename") or "").strip()
            if shape.tag == "mesh" and filename:
                found["filename"] = filename
            row["geometry"] = found
        material = block.find("material") if tag == "visual" else None
        if material is not None:
            row["material"] = _material(material, materials, unreadable)
        if unreadable:
            row["unreadable"] = sorted(unreadable)
        rows.append(row)
    return sorted(rows, key=lambda row: model.dumps(model.canonical_value(row)))


def _material(
    element: ElementTree.Element,
    defined: Mapping[str, dict[str, Any]],
    unreadable: list[str],
) -> dict[str, Any]:
    """A material as written, with the colour of a named material the model defines when
    the reference states none of its own."""
    out: dict[str, Any] = {}
    name = (element.get("name") or "").strip()
    color = element.find("color")
    if color is not None:
        out.update(_stated(color, (("rgba", 4),), "material", unreadable))
    texture = element.find("texture")
    if texture is not None and (texture.get("filename") or "").strip():
        out["texture"] = (texture.get("filename") or "").strip()
    if name and not out and name in defined:
        out.update(defined[name])
    if name:
        out["name"] = name
    return out


def _stated(
    element: ElementTree.Element,
    wanted: Iterable[tuple[str, int]],
    prefix: str,
    unreadable: list[str],
) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for key, count in wanted:
        raw = element.get(key)
        if raw is None:
            continue
        values = [_number(item) for item in raw.replace(",", " ").split()]
        if len(values) != count or any(value is None for value in values):
            unreadable.append(f"{prefix}.{key}")
        else:
            out[key] = values[0] if count == 1 else values
    return out


def _triple(
    raw: str | None,
    attribute: str,
    unparsed: list[tuple[str, str]],
) -> tuple[float, float, float]:
    """Three numbers. An absent attribute is the urdf identity, a written one that is not
    three numbers is recorded as unknown instead of being padded or truncated into a
    certain looking vector."""
    if not raw:
        return (0.0, 0.0, 0.0)
    parts = [item for item in raw.replace(",", " ").split() if item]
    values = [_number(item) for item in parts]
    if len(values) != 3 or any(value is None for value in values):
        unparsed.append((attribute, raw))
        return (0.0, 0.0, 0.0)
    return (values[0], values[1], values[2])


def _number(raw: str | None) -> float | None:
    """One number, or None when the text is not one the atlas can hold. A written infinity
    leaves by the same door as a written word: the attribute is recorded as unparsed, so one
    origin costs itself and never the model around it."""
    if raw is None:
        return None
    try:
        value = float(str(raw).strip())
    except ValueError:
        return None
    return None if model.unrepresentable(value) else value


def _unparsed_reason(raw: str) -> str:
    """Why a numeric attribute did not become a value, in the words a reader needs."""
    for item in str(raw).replace(",", " ").split():
        try:
            value = float(item)
        except ValueError:
            continue
        refused = model.unrepresentable(value)
        if refused:
            return refused
    return "joint_attribute_unreadable"


# ---------------------------------------------------------------------------
# comparing models


def diff_models(models: Mapping[str, UrdfModel]) -> dict[str, Any]:
    """Exact differences between named variants of one robot.

    Reported per variant, never reduced to one number: the same joint carries three different
    origins across the reference models, and collapsing them would publish one of the three as
    fact for all of them.
    """
    variants = sorted(models)
    links = {name: set(models[name].links) for name in variants}
    joints = {name: models[name].joint_map() for name in variants}
    in_all_links = sorted(set.intersection(*links.values())) if links else []
    joint_names = [set(joints[name]) for name in variants]
    in_all_joints = sorted(set.intersection(*joint_names)) if joint_names else []
    frames = {name: models[name].frames() for name in variants}
    parents: dict[str, dict[str, str | None]] = {}
    for variant in variants:
        for frame_name, frame in frames[variant].items():
            if frame.declared:
                parents.setdefault(frame_name, {})[variant] = frame.parent
    # An origin only means something for a fixed parent and child. When a joint moves to
    # another parent the reparenting is the difference, and its numbers are not comparable.
    reparented = _differs(
        _per_joint(variants, joints, lambda joint: [joint.parent, joint.child]),
    )
    return {
        "variants": variants,
        "robot_name": {name: models[name].robot_name for name in variants},
        "counts": {
            name: {"links": len(models[name].links), "joints": len(models[name].joints)}
            for name in variants
        },
        "links_in_all": in_all_links,
        "links_only_in": _only_in(links),
        "joints_in_all": in_all_joints,
        "joints_only_in": _only_in({name: set(joints[name]) for name in variants}),
        "frame_parent_differs": _differs(parents),
        "joint_type_differs": _differs(
            _per_joint(variants, joints, lambda joint: joint.type),
        ),
        "joint_origin_differs": _differs(
            {
                name: per_variant
                for name, per_variant in _per_joint(
                    variants,
                    joints,
                    lambda joint: joint.origin_view(),
                ).items()
                if name not in reparented
            },
        ),
        "joint_reparented": reparented,
        "joint_limit_differs": _differs(
            _per_joint(variants, joints, lambda joint: dict(joint.limit) or None),
        ),
        "joint_mimic_differs": _differs(
            _per_joint(variants, joints, lambda joint: dict(joint.mimic) or None),
        ),
        "gazebo_plugins": {
            name: [plugin.view() for plugin in models[name].gazebo] for name in variants
        },
    }


def _only_in(sets: Mapping[str, set[str]]) -> dict[str, list[str]]:
    out: dict[str, list[str]] = {}
    for name, members in sets.items():
        others: set[str] = set()
        for other, values in sets.items():
            if other != name:
                others |= values
        out[name] = sorted(members - others)
    return out


def _per_joint(
    variants: Sequence[str],
    joints: Mapping[str, Mapping[str, Joint]],
    pick: Callable[[Joint], Any],
) -> dict[str, dict[str, Any]]:
    out: dict[str, dict[str, Any]] = {}
    for variant in variants:
        for name, joint in joints[variant].items():
            out.setdefault(name, {})[variant] = pick(joint)
    return out


def _differs(per_subject: Mapping[str, Mapping[str, Any]]) -> dict[str, dict[str, Any]]:
    out: dict[str, dict[str, Any]] = {}
    for subject in sorted(per_subject):
        per_variant = per_subject[subject]
        if len(per_variant) < 2:
            continue
        distinct = {model.dumps(model.canonical_value(value)) for value in per_variant.values()}
        if len(distinct) > 1:
            out[subject] = dict(sorted(per_variant.items()))
    return out


# ---------------------------------------------------------------------------
# xacro


@dataclass(frozen=True)
class XacroInclude:
    raw: str
    package: str | None
    resolved: str | None


@dataclass
class XacroFile:
    path: str
    kind: str
    robot_name: str | None
    properties: tuple[tuple[str, str], ...] = ()
    macros: tuple[tuple[str, tuple[str, ...]], ...] = ()
    args: tuple[tuple[str, str | None], ...] = ()
    includes: tuple[XacroInclude, ...] = ()
    macro_calls: tuple[tuple[str, tuple[tuple[str, str], ...]], ...] = ()
    needs_arguments: tuple[str, ...] = ()

    def view(self, relpath: str = "") -> dict[str, Any]:
        out: dict[str, Any] = {
            "path": relpath or self.path,
            "kind": self.kind,
            "robot_name": self.robot_name,
            "properties": [{"name": name, "value": value} for name, value in self.properties],
            "macros": [{"name": name, "params": list(params)} for name, params in self.macros],
            "args": [{"name": name, "default": default} for name, default in self.args],
            "includes": [
                {"raw": item.raw, "package": item.package, "resolved": item.resolved}
                for item in self.includes
            ],
            "macro_calls": [
                {"macro": name, "args": dict(call_args)} for name, call_args in self.macro_calls
            ],
        }
        if self.needs_arguments:
            out["needs_arguments"] = list(self.needs_arguments)
        return out


def _xacro_namespace(tag: object) -> str | None:
    if not isinstance(tag, str):
        return None
    return next((namespace for namespace in XACRO_NAMESPACES if tag.startswith(namespace)), None)


def scan_xacro(path: str, *, resolve_package: PackageResolver | None = None) -> XacroFile:
    """Static read of one xacro file. Expansion is the tool's job, not this function's."""
    try:
        text = Path(path).read_text(encoding="utf-8", errors="replace")
        root = ElementTree.fromstring(text)
    except (OSError, ElementTree.ParseError) as exc:
        raise CollectorError(f"cannot read {path}: {exc}", LAYER, NAME) from exc
    properties: list[tuple[str, str]] = []
    macros: list[tuple[str, tuple[str, ...]]] = []
    args: list[tuple[str, str | None]] = []
    includes: list[XacroInclude] = []
    calls: list[tuple[str, tuple[tuple[str, str], ...]]] = []

    def walk(parent: ElementTree.Element) -> None:
        for element in parent:
            tag = element.tag
            namespace = _xacro_namespace(tag)
            if namespace is None:
                walk(element)
                continue
            local = tag[len(namespace) :]
            if local == "property" and element.get("name"):
                properties.append((element.get("name", ""), element.get("value", "")))
            elif local == "macro" and element.get("name"):
                params = tuple(
                    part.split(":=")[0].lstrip("*")
                    for part in (element.get("params") or "").split()
                    if part
                )
                macros.append((element.get("name", ""), params))
                continue
            elif local == "arg" and element.get("name"):
                args.append((element.get("name", ""), element.get("default")))
            elif local == "include" and element.get("filename"):
                includes.append(_include(element.get("filename", ""), path, resolve_package))
            elif local not in XACRO_DIRECTIVES:
                calls.append((local, tuple(sorted(element.attrib.items()))))
            walk(element)

    walk(root)
    declared = {name for name, _default in args}
    used = sorted(set(_ARG_USE.findall(text)))
    robot_name = (root.get("name") or "").strip() or None
    return XacroFile(
        path=path,
        kind="model" if robot_name else "include_only",
        robot_name=robot_name,
        properties=tuple(properties),
        macros=tuple(macros),
        args=tuple(args),
        includes=tuple(includes),
        macro_calls=tuple(calls),
        needs_arguments=tuple(name for name in used if name not in declared),
    )


def _include(
    raw: str,
    parent: str,
    resolve_package: PackageResolver | None,
) -> XacroInclude:
    package: str | None = None
    tail = raw
    match = _PACKAGE_URI.match(raw)
    if match:
        package, tail = match.group(1), match.group(2)
    else:
        found = _FIND_USE.search(raw)
        if found:
            package = found.group(1)
            tail = raw[found.end() :].lstrip("/")
    if package:
        base = resolve_package(package) if resolve_package else None
        resolved = str(Path(base) / tail) if base else None
    else:
        candidate = Path(parent).parent / tail
        resolved = str(candidate) if candidate.is_file() else None
    return XacroInclude(raw=raw, package=package, resolved=resolved)


def uses_xacro(path: str) -> bool:
    """Whether a file holds xacro markup: an element in the xacro namespace, or a `${...}`
    or `$(...)` expression outside a comment. A namespace declaration alone is not markup,
    and neither is commented out xacro, which nav2's own tb3 model carries."""
    try:
        root = ElementTree.fromstring(Path(path).read_text(encoding="utf-8", errors="replace"))
    except (OSError, ElementTree.ParseError):
        return False
    for element in root.iter():
        if _xacro_namespace(element.tag) is not None:
            return True
        texts = [element.text or "", *element.attrib.values()]
        if any("${" in text or "$(" in text for text in texts):
            return True
    return False


def xacro_available() -> bool:
    return shutil.which("xacro") is not None


def expand_xacro(path: str, args: Sequence[tuple[str, str]] = ()) -> str:
    """Run the xacro tool. A missing tool and a missing argument both raise, with the reason
    in the message, because a silently skipped model loses the whole frame tree."""
    if not xacro_available():
        raise CollectorError(f"{path} needs the xacro tool, which is not on PATH", LAYER, NAME)
    command = ["xacro", path, *[f"{key}:={value}" for key, value in args]]
    try:
        done = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=XACRO_TIMEOUT,
            check=False,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise CollectorError(f"xacro failed on {path}: {exc}", LAYER, NAME) from exc
    if done.returncode != 0:
        message = " ".join((done.stderr or done.stdout or "").split())[:400]
        raise CollectorError(f"xacro refused {path}: {message}", LAYER, NAME)
    return done.stdout


def parse_xacro_file(
    path: str,
    args: Sequence[tuple[str, str]] = (),
    *,
    resolve_package: PackageResolver | None = None,
) -> UrdfModel:
    text = expand_xacro(path, args)
    found = parse_urdf_text(text, path=path, resolve_package=resolve_package)
    found.strategy = "xacro"
    found.xacro_args = tuple(args)
    return found


# ---------------------------------------------------------------------------
# the collector


@dataclass
class UrdfCollector:
    name: str = NAME
    layer: str = LAYER
    xacro_args: tuple[tuple[str, tuple[tuple[str, str], ...]], ...] = ()
    """Arguments per xacro file basename, for the models that declare no default."""

    def collect(self, ctx: CollectContext) -> None:
        ws = ctx.ws
        builder = ctx.builder
        profile = ids.build_profile(ctx.profile) if ctx.profile else model.ANY_PROFILE
        urdfs = [path for path in ws.source_files([URDF_SUFFIX]) if _describes(ws, path)]
        xacros = [path for path in ws.source_files([XACRO_SUFFIX]) if _describes(ws, path)]
        # A model named .urdf that holds xacro markup is a xacro model. Read as urdf, its
        # `${namespace}base_scan` became a frame id no running system ever names.
        marked = [path for path in urdfs if uses_xacro(path)]
        urdfs = [path for path in urdfs if path not in marked]
        xacros = sorted(xacros + marked)
        parsed: list[tuple[str, UrdfModel]] = []
        exceptions: list[dict[str, Any]] = []
        refusals: list[tuple[str, str, str]] = []
        for path in urdfs:
            try:
                parsed.append((path, parse_urdf_file(path, resolve_package=ws.share)))
            except CollectorError as exc:
                # The file still costs its entity and its blind spot. Without them a model
                # that is not parsable xml took its frames, its edges and the whole frame
                # want out of the atlas with nothing but a run exception row to show it.
                exceptions.append(_exception_row(ctx, path, exc))
                refusals.append((path, "urdf_unreadable", str(exc)))
        for path in xacros:
            try:
                scan = scan_xacro(path, resolve_package=ws.share)
            except CollectorError as exc:
                exceptions.append(_exception_row(ctx, path, exc))
                refusals.append((path, "xacro_unreadable", str(exc)))
                continue
            if scan.kind != "model":
                continue
            args = self._args_for(path)
            declared = _declared_in_includes(scan, ws.share)
            missing = [
                name
                for name in scan.needs_arguments
                if name not in dict(args) and name not in declared
            ]
            if missing:
                refusals.append((path, "xacro_argument_missing", ", ".join(sorted(missing))))
                continue
            if not xacro_available():
                refusals.append((path, "xacro_tool_missing", "xacro"))
                continue
            try:
                parsed.append((path, parse_xacro_file(path, args, resolve_package=ws.share)))
            except CollectorError as exc:
                exceptions.append(_exception_row(ctx, path, exc))
                refusals.append((path, "xacro_failed", str(exc)))
        run = builder.add_run(
            LAYER,
            profile=profile if ctx.profile else None,
            input_digest=ids.digest(
                NAME,
                *sorted(ws.file_id(path) for path in urdfs + xacros),
                *[f"{key}={value}" for key, value in sorted(ws.root_hashes().items())],
            ),
            roots=ws.root_hashes(),
            exceptions=[*exceptions, *ws.undecodable()],
        )
        workspace.record_undecodable(builder, ws)
        builder.declare_layer(LAYER, present=True, run=run)
        frame_models: dict[str, list[str]] = {}
        frame_shapes: dict[str, dict[str, Any]] = {}
        for path, found in parsed:
            self._emit_model(ctx, run, profile, path, found, frame_models, frame_shapes)
        _frame_attrs(builder, frame_models, frame_shapes)
        for path, reason, detail in refusals:
            self._refuse(ctx, path, reason, detail)

    # -----------------------------------------------------------------
    def _args_for(self, path: str) -> tuple[tuple[str, str], ...]:
        name = Path(path).name
        for candidate, args in self.xacro_args:
            if candidate in (name, path):
                return args
        return ()

    def _emit_model(
        self,
        ctx: CollectContext,
        run: str,
        profile: str,
        path: str,
        found: UrdfModel,
        frame_models: dict[str, list[str]],
        frame_shapes: dict[str, dict[str, Any]],
    ) -> None:
        ws = ctx.ws
        builder = ctx.builder
        file_id = _file_entity(ctx, path, found)
        declared = set(found.links)
        source = ws.source(ws.site(path))
        # Lines of xacro output are not lines of the source file.
        own_lines = found.strategy == "urdf"
        link_lines = dict(found.link_lines) if own_lines else {}
        meshes = {
            mesh.filename: ws.file_id(mesh.path)
            for mesh in found.meshes
            if mesh.path and mesh.exists and _spelled(ws, mesh.filename)[1]
        }
        outside: set[str] = set()
        for name, shapes in found.shapes:
            frame_shapes.setdefault(name, {})[file_id] = _with_mesh_files(
                ws, shapes, meshes, outside
            )
        for spelled in sorted(outside):
            builder.unresolved(
                model.Unresolved(
                    want=model.Want.OTHER.value,
                    owner=file_id,
                    expr=spelled,
                    reason="file_outside_roots",
                    role="mesh",
                    resolvable_by=["add the tree that holds the file to the roots"],
                    evidence=[source],
                )
            )
        for name, frame in found.frames().items():
            if not frame.declared:
                continue
            frame_id = ids.build_frame(name)
            builder.fact(
                frame_id,
                LAYER,
                profile=profile,
                run=run,
                values={"declared_in_urdf": True},
                confidence=model.Confidence.CERTAIN.value,
                evidence=[ws.source(ws.site(path, f"link[{name}]", link_lines.get(name)))],
            )
            frame_models.setdefault(name, []).append(file_id)
        for joint in found.joints:
            if joint.parent not in declared or joint.child not in declared:
                continue
            child = ids.build_frame(joint.child)
            parent = ids.build_frame(joint.parent)
            unreadable = dict(joint.unparsed)
            values: dict[str, Any] = {
                "model": file_id,
                "robot_name": found.robot_name or "",
                "joint": joint.name,
                "joint_type": joint.type,
                "strategy": found.strategy,
            }
            if found.xacro_args:
                values["xacro_args"] = ws.stated_value([list(pair) for pair in found.xacro_args])
            if "origin.xyz" not in unreadable:
                values["xyz"] = list(joint.xyz)
            if "origin.rpy" not in unreadable:
                values["rpy"] = list(joint.rpy)
            if joint.axis is not None and "axis.xyz" not in unreadable:
                values["axis"] = list(joint.axis)
            if joint.limit:
                values["limit"] = dict(joint.limit)
            if joint.mimic:
                values["mimic"] = dict(joint.mimic)
            if unreadable:
                values["unreadable"] = sorted(unreadable)
                for attribute, raw in sorted(joint.unparsed):
                    builder.unresolved(
                        model.Unresolved(
                            want=model.Want.OTHER.value,
                            owner=file_id,
                            expr=f'joint[{joint.name}] {attribute}="{raw}"',
                            reason=_unparsed_reason(raw),
                            role="tf_parent",
                            resolvable_by=[
                                "write the attribute as numbers",
                                "expand the model with xacro",
                            ],
                            evidence=[source],
                        )
                    )
            builder.edge(
                model.EdgeKind.TF_PARENT.value,
                child,
                parent,
                key=ids.edge_key(model=file_id, joint=joint.name, profile=profile),
                layer=LAYER,
                profile=profile,
                run=run,
                values=values,
                confidence=model.Confidence.CERTAIN.value,
                evidence=[
                    ws.source(
                        ws.site(
                            path, f"joint[{joint.name}]", joint.line_hint if own_lines else None
                        )
                    )
                ],
            )
            builder.count_resolved(model.Want.FRAME_ID.value)
        self._emit_remaps(ctx, run, profile, path, file_id, found)
        for name in found.undeclared_parent_links():
            builder.unresolved(
                model.Unresolved(
                    want=model.Want.FRAME_ID.value,
                    owner=file_id,
                    expr=f'<parent link="{name}"/>',
                    reason="parent_link_never_declared",
                    role="tf_parent",
                    resolvable_by=["declare the link in this model"],
                    evidence=[source],
                )
            )
        for name in found.orphan_children():
            builder.unresolved(
                model.Unresolved(
                    want=model.Want.FRAME_ID.value,
                    owner=file_id,
                    expr=f"orphan frame {name}",
                    reason="parent_frame_unresolved",
                    role="tf_parent",
                    resolvable_by=["declare the parent link", "check the joint parent name"],
                    evidence=[source],
                )
            )

    def _emit_remaps(
        self,
        ctx: CollectContext,
        run: str,
        profile: str,
        path: str,
        file_id: str,
        found: UrdfModel,
    ) -> None:
        ws = ctx.ws
        builder = ctx.builder
        for plugin in found.gazebo:
            declared = plugin.namespace is not None
            namespace = ids.normalise_namespace(plugin.namespace)
            for entry in plugin.remappings:
                frm, _, to = entry.partition(":=")
                frm, to = frm.strip(), to.strip()
                site = ws.site(path, f"gazebo.plugin[{plugin.name}].remapping", plugin.line_hint)
                reason = ""
                if not to or ids.is_private(to) or not ids.is_valid_ros_name(to):
                    reason = "remap_target_unresolved"
                elif ids.is_relative(to) and not declared:
                    # gazebo takes the node namespace from the plugin block. Without one,
                    # assuming the root namespace would mint a topic id nothing declared.
                    reason = "remap_target_namespace_unknown"
                if reason:
                    builder.unresolved(
                        model.Unresolved(
                            want=model.Want.TOPIC_NAME.value,
                            owner=file_id,
                            expr=entry,
                            reason=reason,
                            role="urdf_plugin",
                            evidence=[ws.source(site)],
                        )
                    )
                    continue
                topic = ids.build_topic(to, namespace)
                builder.node(topic)
                builder.edge(
                    model.EdgeKind.REMAPS.value,
                    file_id,
                    topic,
                    key=ids.edge_key(
                        plugin=plugin.name,
                        model=file_id,
                        frm=frm,
                        to=to,
                        profile=profile,
                    ),
                    layer=LAYER,
                    profile=profile,
                    run=run,
                    values={
                        "scope": records.REMAP_SCOPES[2],
                        "from": frm,
                        "to": to,
                        "plugin": plugin.name,
                        "plugin_library": plugin.filename,
                        **({"namespace": namespace} if declared else {}),
                    },
                    confidence=model.Confidence.RESOLVED.value,
                    evidence=[ws.source(site)],
                )
                builder.count_resolved(model.Want.TOPIC_NAME.value)

    def _refuse(self, ctx: CollectContext, path: str, reason: str, detail: str) -> None:
        ws = ctx.ws
        file_id = ws.file_id(path)
        # The detail is compared data and it can quote a tool message, so the machine
        # path in it becomes the file id and every other mount point its label.
        detail = ws.stated_value(detail.replace(path, file_id), ctx.cfg.ros_setup)
        ctx.builder.node(
            file_id,
            attrs={"loadable": False, "refused_because": reason, "refused_detail": detail},
        )
        package = ws.package_of(path)
        if package:
            pkg_id = ids.build_pkg(package)
            ctx.builder.node(pkg_id)
            ctx.builder.edge(model.EdgeKind.CONTAINS.value, pkg_id, file_id)
        ctx.builder.unresolved(
            model.Unresolved(
                want=model.Want.FRAME_ID.value,
                owner=file_id,
                expr=detail,
                reason=reason,
                role=REFUSAL_ROLES[reason],
                resolvable_by=list(REFUSAL_FIXES[reason]),
                evidence=[ws.source(ws.site(path))],
            )
        )


def emit_loaded(
    ctx: CollectContext,
    run: str,
    profile: str,
    path: str,
    args: Sequence[tuple[str, str]],
) -> bool:
    """One model as one profile loads it, expanded with the xacro arguments its launch line
    passes, stated in that profile's slot. Returns False and writes nothing when those
    arguments give the links and joints the defaults give, which the collector above already
    states for every profile.

    A frame both expansions declare keeps the geometry the default expansion wrote, so the
    attribute does not depend on which pass ran first."""
    found = parse_xacro_file(path, args, resolve_package=ctx.ws.share)
    try:
        default: UrdfModel | None = parse_xacro_file(path, (), resolve_package=ctx.ws.share)
    except CollectorError:
        default = None
    if default is not None and (default.links, default.joints) == (found.links, found.joints):
        return False
    frame_models: dict[str, list[str]] = {}
    frame_shapes: dict[str, dict[str, Any]] = {}
    UrdfCollector()._emit_model(ctx, run, profile, path, found, frame_models, frame_shapes)
    shared = frozenset(default.links) if default is not None else frozenset()
    _frame_attrs(ctx.builder, frame_models, frame_shapes, shared)
    return True


def _frame_attrs(
    builder: model.AtlasBuilder,
    frame_models: Mapping[str, Sequence[str]],
    frame_shapes: Mapping[str, Mapping[str, Any]],
    keep: frozenset[str] = frozenset(),
) -> None:
    """The models declaring each frame, unioned with what an earlier pass wrote, and the
    geometry of each model, except for the frames in `keep`, whose geometry for that model
    the default expansion states. The urdf pass and a profile expansion therefore write the
    same attributes in either order."""
    for name in sorted(frame_models):
        node = builder.node(ids.build_frame(name))
        held = set(node.attrs.get("urdf_models") or ())
        node.attrs["urdf_models"] = sorted(held | set(frame_models[name]))
        geometry = node.attrs.setdefault("urdf_geometry", {})
        if name in keep:
            continue
        for file_id, shapes in sorted(frame_shapes.get(name, {}).items()):
            geometry.setdefault(file_id, model.canonical_value(shapes))


def _with_mesh_files(
    ws: Any, shapes: dict[str, Any], files: Mapping[str, str], outside: set[str]
) -> dict[str, Any]:
    """The rows with the file id of each mesh the workspace holds beside its written name.

    An absolute name, which is what `$(find pkg)` expands to, is spelled as `_spelled` states
    it, and a name outside every root is added to `outside` for its blind spot.
    """
    out: dict[str, Any] = {}
    for kind, rows in shapes.items():
        out[kind] = []
        for row in rows:
            geometry = row.get("geometry", {})
            filename = geometry.get("filename")
            if filename:
                spelled, inside = _spelled(ws, filename)
                geometry = {**geometry, "filename": spelled}
                if filename in files:
                    geometry["file"] = files[filename]
                if not inside:
                    outside.add(spelled)
                row = {**row, "geometry": geometry}
            texture = row.get("material", {}).get("texture")
            if texture:
                spelled, inside = _spelled(ws, texture)
                if not inside:
                    outside.add(spelled)
                row = {**row, "material": {**row["material"], "texture": spelled}}
            out[kind].append(row)
    return out


def _local(filename: str) -> str:
    """The absolute path a model file name names, `file://` or bare, or empty."""
    local = filename[len(_FILE_URI) :] if filename.startswith(_FILE_URI) else filename
    return local if local.startswith("/") else ""


def _spelled(ws: Any, filename: str) -> tuple[str, bool]:
    """(the file name as compared data may state it, whether it sits inside a root).

    A name inside a package becomes the `package://` spelling a model would have written,
    any other name inside a root has its mount point stated by the root label, and a name
    outside every root is stated by its name and content, never by where it sits here.
    """
    local = _local(filename)
    if not local:
        return filename, True
    try:
        package, relative = ws.rel(local)
    except ContractError:
        return ws.stated_by_content([local])[0], False
    return (f"package://{package}/{relative}" if package else ws.stated_value(local)), True


def _file_entity(ctx: CollectContext, path: str, found: UrdfModel) -> str:
    ws = ctx.ws
    builder = ctx.builder
    file_id = ws.file_id(path)
    attrs: dict[str, Any] = {
        "strategy": found.strategy,
        "robot_name": found.robot_name or "",
    }
    missing = sorted(
        _spelled(ws, mesh.filename)[0]
        for mesh in found.meshes
        if (mesh.package or mesh.path) and not mesh.exists
    )
    if missing:
        attrs["missing_meshes"] = missing
    # A package mesh looked for and not found, split by whether the package source holds it.
    absent: dict[str, list[str]] = {"meshes_not_installed": [], "meshes_absent": []}
    for mesh in found.meshes:
        if not mesh.path or mesh.exists:
            continue
        spelled = _spelled(ws, mesh.filename)[0]
        if not spelled.startswith("package://"):
            continue
        if ws.src_twin(mesh.path):
            absent["meshes_not_installed"].append(spelled)
            owner = ws.packages.get(spelled[len("package://") :].split("/", 1)[0])
            if owner is not None:
                builder.node(ids.build_pkg(owner.name), attrs={"build_type": owner.build_type})
        else:
            absent["meshes_absent"].append(spelled)
    attrs.update({key: sorted(rows) for key, rows in absent.items() if rows})
    builder.node(file_id, attrs=attrs)
    package = ws.package_of(path)
    if package:
        pkg_id = ids.build_pkg(package)
        builder.node(pkg_id)
        builder.edge(model.EdgeKind.CONTAINS.value, pkg_id, file_id)
    return file_id


INCLUDE_LIMIT = 64
"""How many included files one model's arguments are looked up in. A description tree is a
few dozen files, and a cycle or a runaway include must not stall the pass."""


def _declared_in_includes(scan: XacroFile, resolve_package: PackageResolver | None) -> set[str]:
    """Arguments the files a model includes declare, at any depth.

    A xacro argument is global once declared, so a model may use `$(arg gazebo)` that only a
    file it includes declares, with a default. Such an argument is not missing. An include
    that cannot be read declares nothing here, and the expansion reports it in its own words.
    """
    declared: set[str] = set()
    pending = [include.resolved for include in scan.includes if include.resolved]
    seen: set[str] = set()
    while pending and len(seen) < INCLUDE_LIMIT:
        path = pending.pop()
        if path in seen:
            continue
        seen.add(path)
        try:
            included = scan_xacro(path, resolve_package=resolve_package)
        except CollectorError:
            continue
        declared.update(name for name, _default in included.args)
        pending.extend(include.resolved for include in included.includes if include.resolved)
    return declared


def _describes(ws: Any, path: str) -> bool:
    """Whether a model file can describe a robot this workspace runs.

    An install prefix also holds trees no ament package installed, the vendored test data and
    documentation of a physics engine under `opt/` for one, and a robot model there is never
    loaded by anything a profile starts. A loose file in a source checkout is still read,
    because a description kept outside a package is an ordinary layout.
    """
    return ws.package_of(path) is not None or ws.root_of(path) != "install"


def _exception_row(ctx: CollectContext, path: str, exc: Exception) -> dict[str, Any]:
    ws = ctx.ws
    file_id = ws.file_id(path)
    return {
        "file": file_id,
        "phase": "describe",
        "exc_type": type(exc).__name__,
        # The message is compared data, so the machine path in it becomes the file id and
        # every other mount point its label.
        "message": ws.stated_value(str(exc).replace(path, file_id), ctx.cfg.ros_setup),
    }


register(NAME, UrdfCollector)


__all__ = [
    "Frame",
    "GazeboPlugin",
    "Joint",
    "Mesh",
    "UrdfCollector",
    "UrdfModel",
    "XacroFile",
    "XacroInclude",
    "diff_models",
    "emit_loaded",
    "expand_xacro",
    "parse_urdf_file",
    "parse_urdf_text",
    "parse_xacro_file",
    "scan_xacro",
    "uses_xacro",
    "xacro_available",
]
