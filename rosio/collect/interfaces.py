"""msg, srv and action definitions, read with the rosidl grammar.

Two rules here come straight from the reference stack.

* Discovery keys on the `rosidl_generate_interfaces` argument list, not on a filesystem glob.
  One package ships two files named `BlockHeader.msg`, in `msg/` and in `msg_ros2/msg/`, and a
  glob registers the type twice. The generator list also carries the `<dir>:<relative path>`
  form, where only the second half names the type.
* A field type written without a slash resolves to the declaring package, and that is recorded
  as `written_relative` rather than rewritten in place, so a reader still sees what the file
  says. Arrays keep their kind: unbounded, fixed and bounded are three different types.
"""

from __future__ import annotations

import re
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from rosio import ids, model, workspace
from rosio.collect import CollectContext, register
from rosio.errors import CollectorError

LAYER = model.Layer.DECLARED.value
NAME = "interfaces"

SUFFIXES = {".msg": "msg", ".srv": "srv", ".action": "action"}

SECTIONS = {
    "msg": ("",),
    "srv": ("request", "response"),
    "action": ("goal", "result", "feedback"),
}

PRIMITIVES = frozenset(
    {
        "bool",
        "byte",
        "char",
        "float32",
        "float64",
        "int8",
        "uint8",
        "int16",
        "uint16",
        "int32",
        "uint32",
        "int64",
        "uint64",
        "string",
        "wstring",
    }
)

SEPARATOR = "---"

_GENERATE = re.compile(r"rosidl_generate_interfaces\s*\(", re.IGNORECASE)
_CONSTANT = re.compile(r"^(?P<type>\S+)\s+(?P<name>[A-Z][A-Z0-9_]*)\s*=\s*(?P<value>.*)$")
_FIELD = re.compile(r"^(?P<type>\S+)\s+(?P<name>[A-Za-z][A-Za-z0-9_]*)(?:\s+(?P<default>.*))?$")
_ARRAY = re.compile(r"^(?P<base>.+?)\[(?P<bound>[^\]]*)\]$")
_STRING_BOUND = re.compile(r"^(?P<base>w?string)<=(?P<size>\d+)$")


# ---------------------------------------------------------------------------
# records


@dataclass(frozen=True)
class Field:
    name: str
    type: str
    package: str | None
    primitive: bool
    written_relative: bool
    array: tuple[tuple[str, Any], ...] = ()
    string_upper_bound: int | None = None
    default: Any = None
    default_text: str = ""
    """The default as the file spells it, kept so a value the atlas cannot hold is still
    reported as the text an engineer can edit."""

    def view(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "type": self.type,
            "package": self.package,
            "primitive": self.primitive,
            "written_relative": self.written_relative,
            "array": dict(self.array) if self.array else None,
            "string_upper_bound": self.string_upper_bound,
            "default": self.default,
        }

    @property
    def dependency(self) -> str | None:
        if self.primitive or not self.package:
            return None
        return f"{self.package}/msg/{self.type}"


@dataclass(frozen=True)
class Constant:
    name: str
    type: str
    value: Any
    text: str = ""

    def view(self) -> dict[str, Any]:
        return {"name": self.name, "type": self.type, "value": self.value}


@dataclass(frozen=True)
class Section:
    name: str
    fields: tuple[Field, ...] = ()
    constants: tuple[Constant, ...] = ()

    def view(self) -> dict[str, Any]:
        return {
            "fields": [item.view() for item in self.fields],
            "constants": [item.view() for item in self.constants],
        }


@dataclass(frozen=True)
class Interface:
    pkg: str
    kind: str
    name: str
    path: str
    sections: tuple[Section, ...] = ()
    discovery: str = "generator_list"

    @property
    def key(self) -> str:
        return f"{self.pkg}/{self.kind}/{self.name}"

    @property
    def entity_id(self) -> str:
        return ids.build_type(self.pkg, self.kind, self.name)

    def depends_on(self) -> list[str]:
        found: set[str] = set()
        for section in self.sections:
            for item in section.fields:
                dependency = item.dependency
                if dependency and dependency != f"{self.pkg}/msg/{self.name}":
                    found.add(dependency)
        return sorted(found)

    def view(self, relpath: str = "") -> dict[str, Any]:
        out: dict[str, Any] = {
            "path": relpath or self.path,
            "depends_on": self.depends_on(),
        }
        if self.kind == "msg":
            out.update(self.sections[0].view())
        else:
            for section in self.sections:
                out[section.name] = section.view()
        return out


# ---------------------------------------------------------------------------
# parsing


def parse_interface_text(text: str, *, pkg: str, kind: str, name: str, path: str = "") -> Interface:
    if kind not in SECTIONS:
        raise CollectorError(f"{path or name}: unknown interface kind {kind!r}", LAYER, NAME)
    wanted = SECTIONS[kind]
    blocks = _split(text)
    if len(blocks) != len(wanted):
        # A missing section is a broken definition, and padding it with an empty one would
        # publish a service with no response or an action with no feedback as fact.
        named = ", ".join(item for item in wanted if item)
        raise CollectorError(
            f"{path or name}: {len(blocks)} sections for a {kind} definition, "
            f"{len(wanted)} expected" + (f" ({named})" if named else ""),
            LAYER,
            NAME,
        )
    sections: list[Section] = []
    for section_name, lines in zip(wanted, blocks, strict=True):
        fields, constants = _body(lines, pkg=pkg, path=path)
        sections.append(Section(name=section_name, fields=fields, constants=constants))
    return Interface(pkg=pkg, kind=kind, name=name, path=path, sections=tuple(sections))


def parse_interface_file(path: str, *, pkg: str) -> Interface:
    target = Path(path)
    kind = SUFFIXES.get(target.suffix)
    if kind is None:
        raise CollectorError(f"{path} is not an interface definition", LAYER, NAME)
    try:
        text = target.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        raise CollectorError(f"cannot read {path}: {exc}", LAYER, NAME) from exc
    return parse_interface_text(text, pkg=pkg, kind=kind, name=target.stem, path=path)


def _split(text: str) -> list[list[str]]:
    """Sections in order. A parser that splits without keeping the order assigns the
    constants declared inside the result section of an action to the goal."""
    blocks: list[list[str]] = [[]]
    for line in text.splitlines():
        if line.strip() == SEPARATOR:
            blocks.append([])
            continue
        blocks[-1].append(line)
    return blocks


def _body(
    lines: Sequence[str], *, pkg: str, path: str
) -> tuple[tuple[Field, ...], tuple[Constant, ...]]:
    fields: list[Field] = []
    constants: list[Constant] = []
    for raw in lines:
        line = _strip_comment(raw).strip()
        if not line:
            continue
        constant = _CONSTANT.match(line)
        if constant is not None:
            type_name = constant.group("type")
            text = constant.group("value").strip()
            constants.append(
                Constant(
                    name=constant.group("name"),
                    type=type_name,
                    value=_value(text, type_name),
                    text=text,
                )
            )
            continue
        field = _FIELD.match(line)
        if field is None:
            raise CollectorError(f"{path}: cannot read the field line {line!r}", LAYER, NAME)
        fields.append(_field(field, pkg=pkg))
    return tuple(fields), tuple(constants)


def _field(match: re.Match[str], *, pkg: str) -> Field:
    spec = match.group("type")
    array: tuple[tuple[str, Any], ...] = ()
    found = _ARRAY.match(spec)
    if found is not None:
        spec = found.group("base")
        bound = found.group("bound").strip()
        if not bound:
            array = (("kind", "unbounded"),)
        elif bound.startswith("<="):
            array = (("kind", "bounded"), ("size", int(bound[2:])))
        else:
            array = (("kind", "fixed"), ("size", int(bound)))
    upper: int | None = None
    bounded_string = _STRING_BOUND.match(spec)
    if bounded_string is not None:
        spec = bounded_string.group("base")
        upper = int(bounded_string.group("size"))
    package: str | None = None
    written_relative = False
    if "/" in spec:
        head, _, spec = spec.rpartition("/")
        package = head.split("/")[0]
        primitive = False
    elif spec in PRIMITIVES:
        primitive = True
    else:
        primitive = False
        package = pkg
        written_relative = True
    default_text = (match.group("default") or "").strip()
    return Field(
        name=match.group("name"),
        type=spec,
        package=package,
        primitive=primitive,
        written_relative=written_relative,
        array=array,
        string_upper_bound=upper,
        default=_value(default_text, spec, array=bool(array)) if default_text else None,
        default_text=default_text,
    )


def _strip_comment(line: str) -> str:
    quote = ""
    for index, char in enumerate(line):
        if quote:
            if char == quote:
                quote = ""
            continue
        if char in "'\"":
            quote = char
            continue
        if char == "#":
            return line[:index]
    return line


def _items(text: str) -> list[str]:
    """Array elements. A comma inside a quoted string is part of the string, so splitting
    on every comma turns one label into two."""
    out: list[str] = []
    current: list[str] = []
    quote = ""
    escaped = False
    for char in text:
        if escaped:
            current.append(char)
            escaped = False
            continue
        if quote:
            current.append(char)
            if char == "\\":
                escaped = True
            elif char == quote:
                quote = ""
            continue
        if char in "'\"":
            quote = char
            current.append(char)
            continue
        if char == ",":
            out.append("".join(current))
            current = []
            continue
        current.append(char)
    if quote:
        raise CollectorError(f"unterminated string in the array default {text!r}", LAYER, NAME)
    out.append("".join(current))
    return [item.strip() for item in out if item.strip()]


def _value(text: str, type_name: str, *, array: bool = False) -> Any:
    raw = text.strip()
    if not raw:
        return None
    if array or raw.startswith("["):
        inner = raw[1:-1] if raw.startswith("[") and raw.endswith("]") else raw
        return [_value(part, type_name) for part in _items(inner)]
    if type_name in ("string", "wstring"):
        if len(raw) >= 2 and raw[0] == raw[-1] and raw[0] in "'\"":
            return raw[1:-1]
        return raw
    if type_name == "bool":
        lowered = raw.lower()
        if lowered in ("true", "1"):
            return True
        if lowered in ("false", "0"):
            return False
        return raw
    if type_name in ("float32", "float64"):
        try:
            return float(raw)
        except ValueError:
            return raw
    if type_name in PRIMITIVES:
        try:
            return int(raw)
        except ValueError:
            return raw
    if len(raw) >= 2 and raw[0] == raw[-1] and raw[0] in "'\"":
        return raw[1:-1]
    return raw


# ---------------------------------------------------------------------------
# discovery


def generator_entries(text: str) -> list[str]:
    """The argument list of `rosidl_generate_interfaces`, in the order it is written."""
    out: list[str] = []
    for found in _GENERATE.finditer(text):
        depth = 1
        index = found.end()
        while index < len(text) and depth:
            char = text[index]
            if char == "(":
                depth += 1
            elif char == ")":
                depth -= 1
                if not depth:
                    break
            index += 1
        body = text[found.end() : index]
        for token in re.findall(r'"([^"]+)"', body):
            if token.strip():
                out.append(token.strip())
    return out


def discover(
    ws: Any,
    *,
    pkg: str | None = None,
    problems: list[tuple[str, str]] | None = None,
) -> list[Interface]:
    """Every interface the packages of this workspace generate.

    A definition that cannot be read raises, unless the caller passes `problems`: the
    collector records the file and carries on, because one unreadable definition must not
    empty the whole layer.
    """
    out: list[Interface] = []
    for name in sorted(ws.packages):
        if pkg is not None and name != pkg:
            continue
        package = ws.packages[name]
        base = package.src_path or package.share_path
        if not base:
            continue
        cmake = Path(base) / "CMakeLists.txt"
        entries: list[str] = []
        if cmake.is_file():
            try:
                entries = generator_entries(cmake.read_text(encoding="utf-8", errors="replace"))
            except OSError as exc:
                if problems is None:
                    raise CollectorError(f"cannot read {cmake}: {exc}", LAYER, NAME) from exc
                # The generator list is the discovery key, so a build file this process
                # cannot open is a refusal of its own and never a silent directory scan.
                problems.append((str(cmake), f"cannot read {cmake}: {exc}"))
        discovery = "generator_list"
        paths: list[str] = []
        for entry in entries:
            resolved = _entry_path(entry, base, name)
            if resolved:
                paths.append(resolved)
        if not paths:
            discovery = "directory_scan"
            for suffix in sorted(SUFFIXES):
                for found in sorted(Path(base).glob(f"{SUFFIXES[suffix]}/*{suffix}")):
                    paths.append(str(found))
        for path in paths:
            if not Path(path).is_file():
                continue
            try:
                found_interface = parse_interface_file(path, pkg=name)
            except CollectorError as exc:
                if problems is None:
                    raise
                problems.append((path, str(exc)))
                continue
            out.append(
                Interface(
                    pkg=found_interface.pkg,
                    kind=found_interface.kind,
                    name=found_interface.name,
                    path=found_interface.path,
                    sections=found_interface.sections,
                    discovery=discovery,
                )
            )
    return sorted(out, key=lambda item: item.key)


def _entry_path(entry: str, base: str, pkg: str) -> str | None:
    """One generator argument. The `<dir>:<relative path>` form names the type by its second
    half, which is how one package ships two files of the same name."""
    text = entry.replace("${PROJECT_NAME}", pkg).replace("${CMAKE_CURRENT_SOURCE_DIR}", base)
    directory = ""
    if ":" in text:
        directory, _, text = text.rpartition(":")
    if not any(text.endswith(suffix) for suffix in SUFFIXES):
        return None
    root = Path(directory) if directory else Path(base)
    if not root.is_absolute():
        root = Path(base) / root
    return str(root / text)


def _constant_row(item: Constant, prefix: str, blind: list[tuple[str, str, str]]) -> dict[str, Any]:
    """One constant as compared data. A value the atlas cannot hold costs itself and never
    the definition around it, so the name, the type and the text stay and the value goes."""
    row = item.view()
    refused = model.unrepresentable(item.value)
    if not refused:
        return row
    row["value"] = None
    row["value_state"] = "unresolved"
    row["value_expr"] = item.text
    blind.append((f"{prefix}constants[{item.name}]", f"{item.name}={item.text}", refused))
    return row


def _field_row(item: Field, prefix: str, blind: list[tuple[str, str, str]]) -> dict[str, Any]:
    row = item.view()
    refused = model.unrepresentable(item.default)
    if not refused:
        return row
    row["default"] = None
    row["default_state"] = "unresolved"
    row["default_expr"] = item.default_text
    blind.append((f"{prefix}fields[{item.name}]", f"{item.name} {item.default_text}", refused))
    return row


def package_view(interfaces: Sequence[Interface]) -> dict[str, dict[str, Any]]:
    out: dict[str, dict[str, Any]] = {}
    for item in interfaces:
        row = out.setdefault(item.pkg, {"msg": [], "srv": [], "action": []})
        row[item.kind].append(item.name)
    for row in out.values():
        for kind in ("msg", "srv", "action"):
            row[kind] = sorted(row[kind])
    return dict(sorted(out.items()))


# ---------------------------------------------------------------------------
# the collector


@dataclass
class InterfacesCollector:
    name: str = NAME
    layer: str = LAYER

    def collect(self, ctx: CollectContext) -> None:
        ws = ctx.ws
        builder = ctx.builder
        profile = ids.build_profile(ctx.profile) if ctx.profile else model.ANY_PROFILE
        problems: list[tuple[str, str]] = []
        found = discover(ws, problems=problems)
        run = builder.add_run(
            LAYER,
            profile=profile if ctx.profile else None,
            input_digest=ids.digest(
                NAME,
                *[item.key for item in found],
                *[f"{key}={value}" for key, value in sorted(ws.root_hashes().items())],
            ),
            roots=ws.root_hashes(),
            exceptions=[
                {
                    "file": ws.file_id(path),
                    "phase": "describe",
                    "exc_type": CollectorError.__name__,
                    # The message is compared data, so it carries the file id, not a path.
                    "message": message.replace(path, ws.file_id(path)),
                }
                for path, message in sorted(problems)
            ]
            + ws.undecodable(),
        )
        workspace.record_undecodable(builder, ws)
        builder.declare_layer(LAYER, present=True, run=run)
        for item in found:
            self._emit(ctx, run, profile, item)
        for path, message in sorted(problems):
            self._refuse(ctx, path, message)

    def _emit(self, ctx: CollectContext, run: str, profile: str, item: Interface) -> None:
        ws = ctx.ws
        builder = ctx.builder
        file_id = ws.file_id(item.path)
        builder.node(file_id, attrs={"interface_kind": item.kind})
        pkg_id = ids.build_pkg(item.pkg)
        builder.node(pkg_id)
        builder.edge(model.EdgeKind.CONTAINS.value, pkg_id, file_id)
        type_id = item.entity_id
        values: dict[str, Any] = {
            "interface": item.kind,
            "discovery": item.discovery,
            "depends_on": item.depends_on(),
        }
        blind: list[tuple[str, str, str]] = []
        for section in item.sections:
            prefix = f"{section.name}." if section.name else ""
            values[f"{prefix}fields"] = [_field_row(row, prefix, blind) for row in section.fields]
            values[f"{prefix}constants"] = [
                _constant_row(row, prefix, blind) for row in section.constants
            ]
        builder.fact(
            type_id,
            LAYER,
            profile=profile,
            run=run,
            values=values,
            confidence=model.Confidence.CERTAIN.value,
            evidence=[ws.source(ws.site(item.path))],
        )
        builder.edge(model.EdgeKind.CONTAINS.value, pkg_id, type_id)
        builder.edge(model.EdgeKind.DEFINES.value, file_id, type_id)
        builder.count_resolved(model.Want.TYPE.value)
        for locator, expr, reason in blind:
            builder.unresolved(
                model.Unresolved(
                    want=model.Want.OTHER.value,
                    owner=type_id,
                    expr=expr,
                    reason=reason,
                    role="rosidl",
                    resolvable_by=["write a value json can hold"],
                    evidence=[ws.source(ws.site(item.path, locator))],
                )
            )

    def _refuse(self, ctx: CollectContext, path: str, message: str) -> None:
        """A definition the grammar could not read. The file is named and marked, never
        skipped, so a hollow extraction cannot look complete."""
        ws = ctx.ws
        file_id = ws.file_id(path)
        definition = Path(path).suffix in SUFFIXES
        reason = "interface_unreadable" if definition else "generator_list_unreadable"
        ctx.builder.node(file_id, attrs={"loadable": False, "refused_because": reason})
        package = ws.package_of(path)
        if package:
            pkg_id = ids.build_pkg(package)
            ctx.builder.node(pkg_id)
            ctx.builder.edge(model.EdgeKind.CONTAINS.value, pkg_id, file_id)
        ctx.builder.unresolved(
            model.Unresolved(
                want=model.Want.TYPE.value,
                owner=file_id,
                expr=message.replace(path, file_id),
                reason=reason,
                role="rosidl",
                resolvable_by=(
                    ["fix the definition", "check the rosidl grammar"]
                    if definition
                    else ["make the build file readable"]
                ),
                evidence=[ws.source(ws.site(path))],
            )
        )


register(NAME, InterfacesCollector)


__all__ = [
    "PRIMITIVES",
    "SECTIONS",
    "SUFFIXES",
    "Constant",
    "Field",
    "Interface",
    "InterfacesCollector",
    "Section",
    "discover",
    "generator_entries",
    "package_view",
    "parse_interface_file",
    "parse_interface_text",
]
