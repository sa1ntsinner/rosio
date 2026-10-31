"""pluginlib manifests, and the executable to source mapping the join needs.

Plugin declaration is not one mechanism. A manifest reaches pluginlib through a `package.xml`
export tag, through `pluginlib_export_plugin_description_file` in CMake, or through both, and
neither source alone is complete on the reference stack: one package declares its manifest only
in CMake, so a package.xml only scan loses the plugin entirely. Both are read and unioned, and
each manifest records which mechanisms exported it.

Two refusals are recorded rather than guessed. A manifest a `package.xml` exports and nothing
writes is a missing file, and a manifest naming a library no target in the workspace produces
is a library that will not load. Both are notes plus blind spots, never invented plugins.

The executable side lives here for one reason: a node executable has to be joinable to its
source in M2. `add_executable`, `install(TARGETS)`, the `EXECUTABLE` clause of
`rclcpp_components_register_node` and `setup.py` console scripts are four different ways to
name one, and a scan of files with a `main()` misses the component executable completely.
"""

from __future__ import annotations

import re
from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any
from xml.etree import ElementTree

import yaml

from rosio import ids, model
from rosio.collect import CollectContext, register
from rosio.errors import CollectorError, ContractError

LAYER = model.Layer.DECLARED.value
NAME = "plugins"

_PROBLEM_KEYS = ("package", "phase", "exc_type", "message")

SOURCE_SUFFIXES = (".cpp", ".cc", ".cxx", ".c", ".hpp", ".hh", ".hxx", ".h", ".ipp")

TARGET_KEYWORDS = frozenset(
    {
        "ALIAS",
        "EXCLUDE_FROM_ALL",
        "IMPORTED",
        "INTERFACE",
        "MODULE",
        "OBJECT",
        "SHARED",
        "STATIC",
        "UNKNOWN",
    }
)

INSTALL_KEYWORDS = frozenset(
    {
        "ARCHIVE",
        "BUNDLE",
        "COMPONENT",
        "DESTINATION",
        "DIRECTORY",
        "EXPORT",
        "FILES",
        "FRAMEWORK",
        "INCLUDES",
        "LIBRARY",
        "OPTIONAL",
        "PRIVATE_HEADER",
        "PROGRAMS",
        "PUBLIC_HEADER",
        "RESOURCE",
        "RUNTIME",
        "TARGETS",
    }
)

MECHANISM_ADD_EXECUTABLE = "add_executable"
MECHANISM_COMPONENT_NODE = "rclcpp_components_register_node"
MECHANISM_COMPONENT_NODES = "rclcpp_components_register_nodes"
MECHANISM_CONSOLE_SCRIPTS = "console_scripts"

_TARGET_COMMANDS = frozenset(
    {
        "add_executable",
        "add_library",
        "ament_target_dependencies",
        MECHANISM_COMPONENT_NODE,
        MECHANISM_COMPONENT_NODES,
        "target_link_libraries",
    }
)
"""Commands whose first argument is a build target name."""

AMENT_AUTO = {
    "ament_auto_add_executable": "add_executable",
    "ament_auto_add_library": "add_library",
}
"""ament_cmake_auto spellings of the two target commands, which take the same leading name
and list their sources the same way."""

_COMMAND = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)\s*\(")
_VARIABLE = re.compile(r"\$\{([A-Za-z0-9_]+)\}")
_EXPORT_MACRO = re.compile(
    r"PLUGINLIB_EXPORT_CLASS\s*\(\s*([A-Za-z_][\w:]*)\s*,\s*([A-Za-z_][\w:]*)\s*\)",
    re.MULTILINE,
)
_COMPONENT_MACRO = re.compile(r"RCLCPP_COMPONENTS_REGISTER_NODE\s*\(\s*([A-Za-z_][\w:]*)")
_CONSOLE_ENTRY = re.compile(
    r"['\"]\s*(?P<name>[A-Za-z0-9_.\-]+)\s*=\s*(?P<module>[A-Za-z0-9_.]+)\s*:\s*(?P<entry>[A-Za-z0-9_]+)\s*['\"]"
)
_MAIN = re.compile(r"^\s*(?:int|auto)\s+main\s*\(", re.MULTILINE)
_PY_MAIN = re.compile(r"^def\s+(?P<name>[A-Za-z_][A-Za-z0-9_]*)\s*\(", re.MULTILINE)
"""A console script entry point is a module level name. An indented def is a method, and
matching it would claim a symbol the interpreter cannot import."""


# ---------------------------------------------------------------------------
# CMake reading


@dataclass(frozen=True)
class Command:
    name: str
    args: tuple[str, ...]
    line_hint: int


def commands(text: str) -> list[Command]:
    """Every CMake command call, with its arguments and its line.

    A hand written tokenizer rather than a line regex, because every interesting call on the
    reference stack spans several lines and half of them carry a `${variable}`.
    """
    out: list[Command] = []
    stripped, lines = _strip_comments(text)
    position = 0
    while True:
        match = _COMMAND.search(stripped, position)
        if match is None:
            break
        depth = 1
        index = match.end()
        while index < len(stripped) and depth:
            char = stripped[index]
            if char == "(":
                depth += 1
            elif char == ")":
                depth -= 1
                if not depth:
                    break
            index += 1
        body = stripped[match.end() : index]
        out.append(
            Command(
                name=match.group(1).lower(),
                args=tuple(_arguments(body)),
                line_hint=lines[match.start()],
            )
        )
        position = index + 1
    return out


def _expand_loops(rows: Sequence[Command]) -> list[Command]:
    """Unroll `foreach(<var> <item>...)` over literal items, so `add_executable(${node} ...)`
    in a loop names each target. A loop over a variable stays as it is, because this reader
    does not know the value at this point."""
    out: list[Command] = []
    index = 0
    while index < len(rows):
        row = rows[index]
        index += 1
        if row.name != "foreach" or not row.args:
            out.append(row)
            continue
        depth, body = 1, []
        while index < len(rows) and depth:
            depth += {"foreach": 1, "endforeach": -1}.get(rows[index].name, 0)
            if depth:
                body.append(rows[index])
            index += 1
        name, items = row.args[0], list(row.args[1:])
        if items[:2] == ["IN", "ITEMS"]:
            items = items[2:]
        if not items or any("${" in item or item == "IN" for item in items):
            out.extend([row, *body, replace(row, name="endforeach", args=())])
            continue
        for item in items:
            for child in _expand_loops(body):
                args = tuple(arg.replace("${" + name + "}", item) for arg in child.args)
                out.append(replace(child, args=args))
    return out


def _expand_functions(rows: Sequence[Command]) -> list[Command]:
    """Bind local CMake function arguments without executing the build description."""
    functions: dict[str, tuple[tuple[str, ...], list[Command]]] = {}
    top: list[Command] = []
    definition: Command | None = None
    body: list[Command] = []
    for row in rows:
        if row.name == "function" and row.args:
            definition, body = row, []
        elif row.name == "endfunction" and definition is not None:
            functions[definition.args[0].lower()] = (definition.args[1:], body)
            definition = None
        elif definition is not None:
            body.append(row)
        else:
            top.append(row)

    def expand(row: Command, active: frozenset[str]) -> list[Command]:
        function = functions.get(row.name)
        if function is None or row.name in active:
            return [row]
        parameters, body = function
        if len(row.args) < len(parameters):
            return []
        local = dict(zip(parameters, row.args, strict=False))
        local["ARGN"] = ";".join(row.args[len(parameters) :])
        out: list[Command] = []
        for child in body:
            args: list[str] = []
            for arg in child.args:
                value = _VARIABLE.sub(lambda hit: local.get(hit.group(1), hit.group(0)), arg)
                args.extend(value.split(";") if _VARIABLE.fullmatch(arg) else [value])
            if child.name == "cmake_parse_arguments" and len(args) >= 2:
                prefix = args[0]
                # The tokenizer omits empty strings, while keyword lists keep semicolons.
                signature = child.args[1:-1]
                keywords = {word for group in signature for word in group.split(";") if word}
                current = ""
                for word in local["ARGN"].split(";"):
                    if word in keywords:
                        current = f"{prefix}_{word}"
                        local[current] = ""
                    elif current:
                        local[current] = ";".join(filter(None, (local[current], word)))
                continue
            if child.name == "set" and len(args) >= 2:
                local[args[0]] = ";".join(args[1:])
            out.extend(expand(Command(child.name, tuple(args), row.line_hint), active | {row.name}))
        return out

    return [item for row in top for item in expand(row, frozenset())]


def _strip_comments(text: str) -> tuple[str, list[int]]:
    """Text with comments blanked out, plus a line number per character."""
    out: list[str] = []
    lines: list[int] = []
    line = 1
    quote = ""
    comment = False
    for char in text:
        keep = char
        if char == "\n":
            comment = False
            quote = ""
        elif comment:
            keep = " "
        elif quote:
            if char == quote:
                quote = ""
        elif char in "'\"":
            quote = char
        elif char == "#":
            comment = True
            keep = " "
        out.append(keep)
        lines.append(line)
        if char == "\n":
            line += 1
    return "".join(out), lines


def _arguments(body: str) -> list[str]:
    args: list[str] = []
    current: list[str] = []
    quote = ""
    for char in body:
        if quote:
            if char == quote:
                quote = ""
            else:
                current.append(char)
            continue
        if char in "'\"":
            quote = char
            continue
        if char.isspace():
            if current:
                args.append("".join(current))
                current = []
            continue
        current.append(char)
    if current:
        args.append("".join(current))
    return args


@dataclass(frozen=True)
class Target:
    name: str
    kind: str
    sources: tuple[str, ...]
    line_hint: int
    dependencies: tuple[str, ...] = ()
    linked_targets: tuple[str, ...] = ()
    installed: bool = False
    include_dirs: tuple[str, ...] = ()
    output_name: str = ""


@dataclass(frozen=True)
class Component:
    class_name: str
    target: str
    mechanism: str
    line_hint: int
    executable: str | None = None


@dataclass(frozen=True)
class CmakeExport:
    base_package: str
    file: str
    line_hint: int


@dataclass(frozen=True)
class UnresolvedTarget:
    """A target whose name is a variable this reader cannot expand.

    A foreach that builds one executable per source file names its target with the loop
    variable. There is no honest name for it, and inventing one, empty or otherwise, is
    what took the rest of the workspace down with it.
    """

    command: str
    expression: str
    line_hint: int


@dataclass
class CmakeScan:
    project: str = ""
    targets: dict[str, Target] = field(default_factory=dict)
    components: tuple[Component, ...] = ()
    exports: tuple[CmakeExport, ...] = ()
    installed: tuple[str, ...] = ()
    """Names listed in an `install(TARGETS ...)` call. A generated component executable is
    not one of them, so its installed state comes from the install tree or stays false."""
    unresolved_targets: tuple[UnresolvedTarget, ...] = ()
    include_dirs: tuple[str, ...] = ()
    parameter_libraries: tuple[tuple[str, str], ...] = ()
    """`generate_parameter_library(<target> <definition yaml>)`, the target a node links to
    have its parameters declared from the definition."""


def usable_target(name: str) -> bool:
    """Whether a scanned target name can carry an id. A variable this scan could not expand
    leaves either an empty string or the `${...}` text, and neither names a build target."""
    return bool(name) and "${" not in name


def scan_cmake(text: str, *, pkg: str = "") -> CmakeScan:
    variables: dict[str, list[str]] = {}
    project = pkg
    raw_targets: dict[str, dict[str, Any]] = {}
    components: list[Component] = []
    exports: list[CmakeExport] = []
    installed: set[str] = set()
    unresolved: list[UnresolvedTarget] = []
    include_dirs: list[str] = []
    parameter_libraries: list[tuple[str, str]] = []

    def expand(args: Sequence[str]) -> list[str]:
        out: list[str] = []
        for arg in args:
            text_arg = arg.replace("${PROJECT_NAME}", project)
            found = _VARIABLE.fullmatch(text_arg)
            if found is not None and found.group(1) in variables:
                out.extend(variables[found.group(1)])
                continue
            out.append(
                _VARIABLE.sub(lambda hit: "".join(variables.get(hit.group(1), [])), text_arg)
            )
        return out

    for command in _expand_functions(_expand_loops(commands(text))):
        auto = command.name in AMENT_AUTO
        if auto:
            command = replace(command, name=AMENT_AUTO[command.name])
        args = expand(command.args)
        if auto and args and usable_target(args[0]):
            # ament_auto_package installs every target the auto commands add
            installed.add(args[0])
        if command.name in {"include_directories", "target_include_directories"}:
            stated_dirs: list[str] = []
            interface = False
            paths = (
                command.args[1:] if command.name == "target_include_directories" else command.args
            )
            for path in paths:
                if path in {"INTERFACE", "PUBLIC", "PRIVATE"}:
                    interface = path == "INTERFACE"
                    continue
                if interface:
                    continue
                path = (
                    path.replace("${CMAKE_CURRENT_SOURCE_DIR}", ".")
                    .replace("${PROJECT_SOURCE_DIR}", ".")
                    .replace("${PROJECT_NAME}", project)
                )
                path = _VARIABLE.sub(
                    lambda hit: (
                        ";".join(variables[hit.group(1)])
                        if hit.group(1) in variables
                        else hit.group(0)
                    ),
                    path,
                )
                if path.startswith("$<BUILD_INTERFACE:") and path.endswith(">"):
                    path = path[len("$<BUILD_INTERFACE:") : -1]
                if "$" not in path and path not in {
                    "PUBLIC",
                    "PRIVATE",
                    "INTERFACE",
                    "SYSTEM",
                    "BEFORE",
                    "AFTER",
                }:
                    stated_dirs.extend(path.split(";"))
            if command.name == "include_directories":
                lists = [
                    include_dirs,
                    *[row.setdefault("include_dirs", []) for row in raw_targets.values()],
                ]
            elif args and usable_target(args[0]):
                lists = [
                    raw_targets.setdefault(args[0], {}).setdefault(
                        "include_dirs", list(include_dirs)
                    )
                ]
            else:
                lists = []
            for directories in lists:
                if "BEFORE" in command.args:
                    directories[:0] = stated_dirs
                else:
                    directories.extend(stated_dirs)
        if args and command.name in _TARGET_COMMANDS and not usable_target(args[0]):
            # The name is a variable this reader cannot expand. Recorded and skipped: a
            # target with no name cannot carry an id, and raising here loses every package
            # sorted after this one.
            if command.name in ("add_executable", "add_library"):
                unresolved.append(
                    UnresolvedTarget(
                        command=command.name,
                        expression=command.args[0] if command.args else "",
                        line_hint=command.line_hint,
                    )
                )
            continue
        if command.name == "project" and args:
            project = args[0]
        elif command.name == "set" and args:
            variables[args[0]] = list(args[1:])
        elif command.name in ("add_executable", "add_library") and args:
            kind = "executable" if command.name == "add_executable" else "library"
            sources = [item for item in args[1:] if _is_source(item)]
            if args[1:] and all(item in TARGET_KEYWORDS for item in args[1:]):
                sources = []
            raw_targets.setdefault(args[0], {}).update(
                {
                    "kind": kind,
                    "sources": sources,
                    "line_hint": command.line_hint,
                }
            )
            raw_targets[args[0]].setdefault("include_dirs", list(include_dirs))
        elif command.name == "target_link_libraries" and len(args) > 1:
            row = raw_targets.setdefault(args[0], {})
            linked = [item for item in args[1:] if item not in TARGET_KEYWORDS]
            row.setdefault("linked_targets", []).extend(linked)
        elif command.name == "ament_target_dependencies" and len(args) > 1:
            row = raw_targets.setdefault(args[0], {})
            row.setdefault("dependencies", []).extend(args[1:])
        elif command.name == MECHANISM_COMPONENT_NODES and len(args) > 1:
            for class_name in args[1:]:
                components.append(
                    Component(
                        class_name=class_name,
                        target=args[0],
                        mechanism=MECHANISM_COMPONENT_NODES,
                        line_hint=command.line_hint,
                    )
                )
        elif command.name == MECHANISM_COMPONENT_NODE and args:
            class_name = _keyword_value(args, "PLUGIN")
            executable = _keyword_value(args, "EXECUTABLE")
            if class_name:
                components.append(
                    Component(
                        class_name=class_name,
                        target=args[0],
                        mechanism=MECHANISM_COMPONENT_NODE,
                        line_hint=command.line_hint,
                        executable=executable,
                    )
                )
        elif command.name == "pluginlib_export_plugin_description_file" and len(args) > 1:
            exports.append(
                CmakeExport(
                    base_package=args[0],
                    file=args[1],
                    line_hint=command.line_hint,
                )
            )
        elif command.name == "install" and args and args[0] == "TARGETS":
            for item in args[1:]:
                if item in INSTALL_KEYWORDS:
                    break
                if usable_target(item):
                    installed.add(item)
        elif (
            command.name == "generate_parameter_library"
            and len(args) > 1
            and usable_target(args[0])
        ):
            parameter_libraries.append((args[0], args[1]))
        elif command.name == "set_target_properties" and args and "OUTPUT_NAME" in args:
            # image_rotate builds `image_rotate_bin` and installs it as `image_rotate`
            raw_targets.setdefault(args[0], {})["output_name"] = _keyword_value(args, "OUTPUT_NAME")
    targets = {
        name: Target(
            name=name,
            kind=str(row.get("kind", "library")),
            sources=tuple(dict.fromkeys(row.get("sources", ()))),
            line_hint=int(row.get("line_hint", 0)),
            dependencies=tuple(sorted(set(row.get("dependencies", ())))),
            linked_targets=tuple(dict.fromkeys(row.get("linked_targets", ()))),
            installed=name in installed,
            include_dirs=tuple(dict.fromkeys(row.get("include_dirs", ()))),
            output_name=str(row.get("output_name") or ""),
        )
        for name, row in sorted(raw_targets.items())
        if "kind" in row
    }
    return CmakeScan(
        project=project,
        targets=targets,
        components=tuple(components),
        exports=tuple(exports),
        installed=tuple(sorted(installed)),
        unresolved_targets=tuple(
            sorted(unresolved, key=lambda item: (item.command, item.expression, item.line_hint))
        ),
        include_dirs=tuple(dict.fromkeys(include_dirs)),
        parameter_libraries=tuple(sorted(set(parameter_libraries))),
    )


def parameter_definition(text: str) -> tuple[list[str], list[str]]:
    """The names a generate_parameter_library definition declares, and the expressions of
    the names it builds at run time.

    The top level key is the namespace of the generated struct and names no parameter. A
    mapping holding a `type` is one parameter, and the keys above it fold into its dotted
    name. A `__map_<param>` group repeats its parameters once per value of `<param>`, so a
    name under it is an expression, `"gains." + joints + ".p"`, which states the literal
    pieces the name has to hold.
    """
    loaded = yaml.safe_load(text)
    names: list[str] = []
    patterns: list[str] = []

    def expression(path: list[str | None], mapped: list[str]) -> str:
        pieces: list[str] = []
        literal = ""
        runtime = iter(mapped)
        for position, segment in enumerate(path):
            literal += "." if position else ""
            if segment is None:
                if literal:
                    pieces.append(f'"{literal}"')
                literal = ""
                pieces.append(next(runtime))
            else:
                literal += segment
        if literal:
            pieces.append(f'"{literal}"')
        return " + ".join(pieces)

    def walk(node: dict[Any, Any], path: list[str | None], mapped: list[str]) -> None:
        for key, value in node.items():
            if not isinstance(value, dict):
                continue
            key = str(key)
            if key.startswith("__map_"):
                walk(value, [*path, None], [*mapped, key.removeprefix("__map_")])
            elif isinstance(value.get("type"), str):
                if mapped:
                    patterns.append(expression([*path, key], mapped))
                else:
                    names.append(".".join(str(item) for item in [*path, key]))
            else:
                walk(value, [*path, key], mapped)

    if isinstance(loaded, dict):
        for top in loaded.values():
            if isinstance(top, dict):
                walk(top, [], [])
    return sorted(set(names)), sorted(set(patterns))


def _keyword_value(args: Sequence[str], keyword: str) -> str | None:
    for index, item in enumerate(args):
        if item == keyword and index + 1 < len(args):
            return args[index + 1]
    return None


def _is_source(name: str) -> bool:
    return any(name.endswith(suffix) for suffix in SOURCE_SUFFIXES)


# ---------------------------------------------------------------------------
# setup.py reading


@dataclass(frozen=True)
class ConsoleScript:
    name: str
    module: str
    entry: str
    line_hint: int


def scan_console_scripts(text: str) -> list[ConsoleScript]:
    """`console_scripts` entries. The executable name is the join key and the module is only
    where the code lives: the two disagree on purpose on the reference stack."""
    marker = text.find(MECHANISM_CONSOLE_SCRIPTS)
    if marker < 0:
        return []
    start = text.find("[", marker)
    end = text.find("]", start)
    if start < 0 or end < 0:
        return []
    block = text[start:end]
    offset = text[:start].count("\n") + 1
    out: list[ConsoleScript] = []
    for found in _CONSOLE_ENTRY.finditer(block):
        out.append(
            ConsoleScript(
                name=found.group("name"),
                module=found.group("module"),
                entry=found.group("entry"),
                line_hint=offset + block[: found.start()].count("\n"),
            )
        )
    return sorted(out, key=lambda item: item.name)


# ---------------------------------------------------------------------------
# plugin manifests


@dataclass(frozen=True)
class PluginClass:
    lookup_name: str
    type: str
    base_class: str
    name_attribute_present: bool

    @property
    def entity_id(self) -> str:
        return ids.build_plugin(self.base_class, self.lookup_name)

    def view(self) -> dict[str, Any]:
        return {
            "lookup_name": self.lookup_name,
            "type": self.type,
            "base_class": self.base_class,
            "name_attribute_present": self.name_attribute_present,
            "plugin_id": self.entity_id,
        }


@dataclass(frozen=True)
class PackageXmlExport:
    tag: str
    attribute: str
    value: str

    def view(self) -> dict[str, Any]:
        return {"tag": self.tag, "attribute": self.attribute, "value": self.value}


@dataclass(frozen=True)
class Manifest:
    path: str
    exists: bool
    library: str | None
    exported_by: tuple[str, ...]
    export_tags: tuple[str, ...]
    classes: tuple[PluginClass, ...] = ()
    library_built_here: bool | None = None
    unreadable: str = ""
    """Why the file is on disk and still gave nothing, if that happened."""

    def view(self, rel: Callable[[str], str] | None = None) -> dict[str, Any]:
        out: dict[str, Any] = {
            "path": rel(self.path) if rel else self.path,
            "exists": self.exists,
            "library": self.library,
            "exported_by": list(self.exported_by),
            "export_tags": list(self.export_tags),
            "classes": [item.view() for item in self.classes],
        }
        if self.library_built_here is not None:
            out["library_built_here"] = self.library_built_here
        if self.unreadable:
            out["unreadable"] = self.unreadable
        return out


def parse_manifest_text(text: str, *, path: str = "") -> tuple[str | None, list[PluginClass]]:
    try:
        root = ElementTree.fromstring(text)
    except ElementTree.ParseError as exc:
        raise CollectorError(
            f"{path or 'manifest'} is not parsable xml: {exc}", LAYER, NAME
        ) from exc
    libraries = [root] if root.tag == "library" else list(root.findall("library"))
    library: str | None = None
    classes: list[PluginClass] = []
    for element in libraries:
        path_attr = (element.get("path") or "").strip()
        if path_attr and library is None:
            library = Path(path_attr).name.removeprefix("lib")
        for class_element in element.findall("class"):
            type_name = (class_element.get("type") or "").strip()
            base = (class_element.get("base_class_type") or "").strip()
            name = (class_element.get("name") or "").strip()
            if not type_name or not base:
                continue
            classes.append(
                PluginClass(
                    lookup_name=name or type_name,
                    type=type_name,
                    base_class=base,
                    name_attribute_present=bool(name),
                )
            )
    classes.sort(key=lambda item: (item.base_class, item.lookup_name))
    return library, classes


def package_xml_exports(path: str) -> list[PackageXmlExport]:
    try:
        root = ElementTree.parse(path).getroot()
    except (ElementTree.ParseError, OSError):
        return []
    out: list[PackageXmlExport] = []
    for export in root.findall("export"):
        for child in export:
            value = child.get("plugin")
            if value:
                out.append(
                    PackageXmlExport(tag=str(child.tag), attribute="plugin", value=value.strip())
                )
    return out


@dataclass(frozen=True)
class ExportMacro:
    type: str
    base_class: str
    file: str
    line_hint: int
    wrapped: bool

    def view(self, rel: Callable[[str], str] | None = None) -> dict[str, Any]:
        return {
            "type": self.type,
            "base_class": self.base_class,
            "file": rel(self.file) if rel else self.file,
            "line_hint": self.line_hint,
            "wrapped": self.wrapped,
        }


def scan_export_macros(text: str, *, path: str = "") -> list[ExportMacro]:
    """`PLUGINLIB_EXPORT_CLASS` sites. The macro is wrapped across lines in live code, so a
    single line regex loses half of them."""
    out: list[ExportMacro] = []
    for found in _EXPORT_MACRO.finditer(text):
        head = text[: found.start()]
        body = found.group(0)
        out.append(
            ExportMacro(
                type=found.group(1),
                base_class=found.group(2),
                file=path,
                line_hint=head.count("\n") + 1,
                wrapped="\n" in body,
            )
        )
    return out


# ---------------------------------------------------------------------------
# executables


@dataclass(frozen=True)
class Executable:
    pkg: str
    name: str
    mechanism: str
    declared_in: str
    sources: tuple[str, ...]
    has_main: bool
    installed: bool
    line_hint: int | None = None
    target: str | None = None
    module: str | None = None
    entry: str | None = None
    dependencies: tuple[str, ...] = ()
    linked_targets: tuple[str, ...] = ()
    symbol: str = ""
    symbol_file: str = ""

    @property
    def entity_id(self) -> str:
        return ids.build_exe(self.pkg, self.name)

    def view(self, rel: Callable[[str], str] | None = None) -> dict[str, Any]:
        out: dict[str, Any] = {
            "package": self.pkg,
            "name": self.name,
            "mechanism": self.mechanism,
            "declared_in": rel(self.declared_in) if rel else self.declared_in,
            "sources": [rel(item) if rel else item for item in self.sources],
            "has_main": self.has_main,
            "installed": self.installed,
        }
        if self.mechanism == MECHANISM_CONSOLE_SCRIPTS:
            out["module"] = self.module
            out["entry"] = self.entry
            return out
        out["line_hint"] = self.line_hint
        out["target"] = self.target
        if self.linked_targets:
            out["linked_targets"] = list(self.linked_targets)
        out["dependencies"] = list(self.dependencies)
        return out


@dataclass
class PackageBuild:
    """Everything one package declares about what it builds."""

    pkg: str
    base: str
    cmake_path: str = ""
    setup_path: str = ""
    scan: CmakeScan = field(default_factory=CmakeScan)
    console_scripts: tuple[ConsoleScript, ...] = ()
    executables: tuple[Executable, ...] = ()
    components: tuple[Component, ...] = ()
    manifests: tuple[Manifest, ...] = ()
    xml_exports: tuple[PackageXmlExport, ...] = ()
    export_macros: tuple[ExportMacro, ...] = ()
    python_modules_without_executable: tuple[str, ...] = ()

    installed_names: frozenset[str] = frozenset()

    def library(self, name: str) -> Target | None:
        target = self.scan.targets.get(name)
        return target if target is not None and target.kind == "library" else None


def read_package(ws: Any, pkg: str, *, libraries: Iterable[str] = ()) -> PackageBuild:
    """Read one package: CMake targets, console scripts, manifests and export macros."""
    package = ws.packages[pkg]
    base = package.src_path or package.share_path
    if not base:
        raise CollectorError(f"package {pkg} has no path", LAYER, NAME)
    build = PackageBuild(pkg=pkg, base=base)
    cmake = Path(base) / "CMakeLists.txt"
    if cmake.is_file():
        build.cmake_path = str(cmake)
        build.scan = scan_cmake(_read(ws, cmake), pkg=pkg)
    build.installed_names = frozenset(build.scan.installed) | _installed_in_tree(package)
    setup = Path(base) / "setup.py"
    if setup.is_file():
        build.setup_path = str(setup)
        build.console_scripts = tuple(scan_console_scripts(_read(ws, setup)))
    build.components = build.scan.components
    build.executables = tuple(_executables(build))
    build.xml_exports = tuple(package_xml_exports(str(Path(base) / "package.xml")))
    build.manifests = tuple(_manifests(ws, build, set(libraries)))
    macros: list[ExportMacro] = []
    for path in sorted(Path(base).rglob("*.cpp")):
        macros.extend(scan_export_macros(_read(ws, path), path=str(path)))
    build.export_macros = tuple(sorted(macros, key=lambda item: (item.file, item.type)))
    build.python_modules_without_executable = tuple(_orphan_modules(build))
    return build


def _installed_in_tree(package: Any) -> frozenset[str]:
    """What the install tree really holds. A generated component executable appears in no
    install call, so the tree is the only place that can confirm it."""
    if not package.share_path:
        return frozenset()
    prefix = Path(package.share_path).parent.parent / "lib" / package.name
    if not prefix.is_dir():
        return frozenset()
    return frozenset(item.name for item in prefix.iterdir() if item.is_file())


def _executables(build: PackageBuild) -> list[Executable]:
    out: list[Executable] = []
    for name, target in build.scan.targets.items():
        if target.kind != "executable" or not usable_target(name):
            continue
        sources = [str(Path(build.base) / item) for item in target.sources]
        symbol_file = next((path for path in sources if _has_main(path)), "")
        out.append(
            Executable(
                pkg=build.pkg,
                name=target.output_name or name,
                mechanism=MECHANISM_ADD_EXECUTABLE,
                declared_in=build.cmake_path,
                sources=tuple(sources),
                has_main=bool(symbol_file),
                installed=target.installed,
                line_hint=target.line_hint,
                target=name,
                dependencies=target.dependencies,
                linked_targets=target.linked_targets,
                symbol="main",
                symbol_file=symbol_file,
            )
        )
    for component in build.scan.components:
        if not component.executable:
            continue
        target = build.scan.targets.get(component.target)
        sources = (
            [str(Path(build.base) / item) for item in target.sources] if target is not None else []
        )
        symbol_file = next((path for path in sources if _defines_component(path, component)), "")
        if not symbol_file:
            # The target's sources sit behind a variable or a DIRECTORY glob this reader does
            # not expand, so the package's own sources are asked for the registration.
            symbol_file = next(
                (
                    str(path)
                    for path in sorted(Path(build.base).rglob("*.cpp"))
                    if _defines_component(str(path), component)
                ),
                "",
            )
        out.append(
            Executable(
                pkg=build.pkg,
                name=component.executable,
                mechanism=MECHANISM_COMPONENT_NODE,
                declared_in=build.cmake_path,
                sources=tuple(sources),
                has_main=any(_has_main(path) for path in sources),
                installed=component.executable in build.installed_names,
                line_hint=component.line_hint,
                target=component.target,
                dependencies=target.dependencies if target is not None else (),
                symbol=component.class_name,
                symbol_file=symbol_file,
            )
        )
    for script in build.console_scripts:
        module_path = str(Path(build.base) / (script.module.replace(".", "/") + ".py"))
        exists = Path(module_path).is_file()
        has_entry = exists and _has_python_entry(module_path, script.entry)
        out.append(
            Executable(
                pkg=build.pkg,
                name=script.name,
                mechanism=MECHANISM_CONSOLE_SCRIPTS,
                declared_in=build.setup_path,
                sources=(module_path,) if exists else (),
                has_main=has_entry,
                installed=exists,
                module=script.module,
                entry=script.entry,
                symbol=script.entry,
                symbol_file=module_path if has_entry else "",
            )
        )
    return sorted(out, key=lambda item: item.name)


def _manifests(ws: Any, build: PackageBuild, libraries: set[str]) -> list[Manifest]:
    """One row per manifest, keyed by identity and not by path.

    The two export mechanisms name the same file through two trees: package.xml resolves
    `${prefix}` into the install share while CMake names it relative to the source. Keying by
    the path would split one manifest into two and then report the source half as missing from
    package.xml, which is the exact false positive this collector exists to avoid.
    """
    rows: dict[str, dict[str, Any]] = {}

    def add(path: str, mechanism: str, tag: str) -> None:
        key = _manifest_key(ws, path)
        row = rows.setdefault(key, {"exported_by": set(), "tags": set(), "paths": []})
        row["exported_by"].add(mechanism)
        row["tags"].add(tag)
        row["paths"].append(path)

    for export in build.xml_exports:
        add(_resolve_export(ws, build, export.value), "package.xml", export.tag)
    for export in build.scan.exports:
        add(str(Path(build.base) / export.file), "cmake", export.base_package)
    out: list[Manifest] = []
    for key in sorted(rows):
        row = rows[key]
        path = _preferred_path(ws, row["paths"])
        exists = Path(path).is_file()
        library: str | None = None
        classes: tuple[PluginClass, ...] = ()
        unreadable = ""
        if exists:
            try:
                library, found = parse_manifest_text(_read(ws, path), path=path)
                classes = tuple(found)
            except CollectorError as exc:
                unreadable = str(exc)
        out.append(
            Manifest(
                path=path,
                exists=exists,
                library=library,
                exported_by=tuple(sorted(row["exported_by"])),
                export_tags=tuple(sorted(row["tags"])),
                classes=classes,
                library_built_here=_library_built(ws, build, library, libraries),
                unreadable=unreadable,
            )
        )
    return out


def _held_manifests(build: PackageBuild, missing: str) -> list[str]:
    """The pluginlib manifests a package does hold beside one an export names and it does
    not, which is where that export was most likely meant to point."""
    base = Path(build.base)
    out: list[str] = []
    for path in sorted([*base.glob("*.xml"), *base.glob("*/*.xml")]):
        if path.name == "package.xml" or path.parent.name in ("test", "tests"):
            continue
        if str(path) == str(Path(missing)):
            continue
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        if "<class_libraries" in text or ("<library" in text and "<class " in text):
            out.append(str(path))
    return out


def _manifest_key(ws: Any, path: str) -> str:
    try:
        return ws.file_id(path)
    except ContractError:
        return path


def _preferred_path(ws: Any, paths: Sequence[str]) -> str:
    """The editable copy wins: src first, then whatever exists, then the first seen."""
    existing = [path for path in sorted(set(paths)) if Path(path).is_file()]
    for path in existing:
        if ws.root_of(path) == "src":
            return path
    if existing:
        return existing[0]
    return sorted(set(paths))[0]


def _resolve_export(ws: Any, build: PackageBuild, value: str) -> str:
    text = value.replace("$prefix", "${prefix}")
    if "${prefix}" in text:
        share = ws.share(build.pkg) or build.base
        tail = text.split("${prefix}", 1)[1].lstrip("/")
        candidate = Path(share) / tail
        if not candidate.is_file():
            source = Path(build.base) / tail
            if source.is_file():
                return str(source)
        return str(candidate)
    candidate = Path(text)
    return str(candidate if candidate.is_absolute() else Path(build.base) / text)


def _orphan_modules(build: PackageBuild) -> list[str]:
    if not build.console_scripts:
        return []
    used = {script.module.replace(".", "/") + ".py" for script in build.console_scripts}
    out: list[str] = []
    for path in sorted(Path(build.base).rglob("*.py")):
        relative = str(path.relative_to(build.base))
        if relative in ("setup.py",) or Path(relative).name == "__init__.py":
            continue
        if relative.split("/")[0] in ("test", "tests"):
            continue
        if relative in used:
            continue
        out.append(str(path))
    return out


def _has_main(path: str) -> bool:
    try:
        return bool(_MAIN.search(Path(path).read_text(encoding="utf-8", errors="replace")))
    except OSError:
        return False


def _has_python_entry(path: str, entry: str) -> bool:
    try:
        text = Path(path).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return False
    return any(found.group("name") == entry for found in _PY_MAIN.finditer(text))


def _defines_component(path: str, component: Component) -> bool:
    try:
        text = Path(path).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return False
    return any(found.group(1) == component.class_name for found in _COMPONENT_MACRO.finditer(text))


def _read(ws: Any, path: Path | str) -> str:
    try:
        return Path(path).read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        # The refusal is recorded on a run and is therefore compared data. A machine path in
        # it, and OSError spells one out, makes the same refusal two atlases on two boxes.
        reason = f"[Errno {exc.errno}] {exc.strerror}" if exc.errno else type(exc).__name__
        raise CollectorError(f"cannot read {_named(ws, path)}: {reason}", LAYER, NAME) from exc


def _named(ws: Any, path: Path | str) -> str:
    try:
        return str(ws.file_id(str(path)))
    except (ContractError, KeyError):
        return Path(path).name


# ---------------------------------------------------------------------------
# views and problems


LIBRARY_FILES = ("lib{}.so", "{}.so", "lib{}.dylib", "{}.dll")
"""The file names a pluginlib library path resolves to in an install tree's `lib`."""


def _library_built(
    ws: Any, build: PackageBuild, library: str | None, libraries: set[str]
) -> bool | None:
    """Whether the library a manifest names exists, or None when this reader cannot tell.

    A package read from source builds its library or does not, and the CMake targets say
    which. An installed package has no CMakeLists, so its own `lib` directory is asked
    instead. A library that is not there may be a python plugin path, which is no file at
    all, so an install tree never states that a library is missing.
    """
    if not library:
        return None
    if build.cmake_path:
        return library in libraries
    package = ws.packages.get(build.pkg)
    share = getattr(package, "share_path", None)
    if not share:
        return None
    lib = Path(share).parents[1] / "lib"
    return True if any((lib / name.format(library)).is_file() for name in LIBRARY_FILES) else None


def library_targets(ws: Any) -> set[str]:
    """Every library target the workspace builds, so a manifest can be told that the library
    it names is produced by nothing here."""
    out: set[str] = set()
    for name in sorted(ws.packages):
        package = ws.packages[name]
        base = package.src_path or package.share_path
        if not base:
            continue
        cmake = Path(base) / "CMakeLists.txt"
        if not cmake.is_file():
            continue
        scan = scan_cmake(_read(ws, cmake), pkg=name)
        out.update(target.name for target in scan.targets.values() if target.kind == "library")
    return out


def problems(build: PackageBuild) -> list[dict[str, str]]:
    """The refusals this collector can report, as data rather than prose."""
    out: list[dict[str, str]] = []
    for target in build.scan.unresolved_targets:
        out.append(
            {
                "kind": "target_name_unresolved",
                "subject": f"{target.command}({target.expression})",
                "detail": "the target name is a CMake variable this reader cannot expand",
            }
        )
    for manifest in build.manifests:
        if not manifest.exists:
            out.append(
                {
                    "kind": "manifest_missing_on_disk",
                    "subject": manifest.path,
                    "detail": "a package.xml export names a file that nothing writes",
                }
            )
            continue
        if manifest.unreadable:
            out.append(
                {
                    "kind": "manifest_unreadable",
                    "subject": manifest.path,
                    "detail": manifest.unreadable,
                }
            )
            continue
        if manifest.library and manifest.library_built_here is False:
            out.append(
                {
                    "kind": "library_not_built",
                    "subject": manifest.library,
                    "detail": f"{Path(manifest.path).name} names a library no target here produces",
                }
            )
        if "package.xml" not in manifest.exported_by:
            out.append(
                {
                    "kind": "manifest_not_in_package_xml",
                    "subject": manifest.path,
                    "detail": "exported only by pluginlib_export_plugin_description_file",
                }
            )
        for item in manifest.classes:
            if not item.name_attribute_present:
                out.append(
                    {
                        "kind": "lookup_name_defaulted_to_type",
                        "subject": item.entity_id,
                        "detail": "the class element carries no name attribute",
                    }
                )
    return sorted(out, key=lambda row: (row["kind"], row["subject"]))


def plugin_view(build: PackageBuild, rel: Callable[[str], str] | None = None) -> dict[str, Any]:
    return {
        "package": build.pkg,
        "manifests": [item.view(rel) for item in build.manifests],
        "package_xml_exports": [item.view() for item in build.xml_exports],
        "cmake_exports": [
            {
                "base_package": item.base_package,
                "file": item.file,
                "line_hint": item.line_hint,
            }
            for item in build.scan.exports
        ],
        "source_export_macros": [item.view(rel) for item in build.export_macros],
        "expected_problems": [
            {
                "kind": row["kind"],
                "subject": rel(row["subject"])
                if rel and row["kind"].startswith("manifest")
                else row["subject"],
                "detail": row["detail"],
            }
            for row in problems(build)
        ],
    }


def executable_view(
    builds: Sequence[PackageBuild],
    rel: Callable[[str], str] | None = None,
) -> dict[str, Any]:
    executables: dict[str, Any] = {}
    components: dict[str, Any] = {}
    libraries: dict[str, Any] = {}
    orphans: list[str] = []
    for build in builds:
        for item in build.executables:
            executables[item.entity_id] = item.view(rel)
        for component in build.components:
            target = build.scan.targets.get(component.target)
            sources = (
                [str(Path(build.base) / source) for source in target.sources]
                if target is not None
                else []
            )
            macro_line = next(
                (
                    macro.line_hint
                    for macro in build.export_macros
                    if macro.type == component.class_name
                ),
                None,
            )
            if macro_line is None:
                macro_line = _component_macro_line(sources, component)
            components[component.class_name] = {
                "package": build.pkg,
                "mechanism": component.mechanism,
                "declared_in": rel(build.cmake_path) if rel else build.cmake_path,
                "line_hint": component.line_hint,
                "target": component.target,
                "sources": [rel(item) if rel else item for item in sources],
                "macro_line_hint": macro_line,
                "executable": component.executable,
            }
        for name, target in build.scan.targets.items():
            if target.kind != "library":
                continue
            libraries[name] = {
                "package": build.pkg,
                "declared_in": rel(build.cmake_path) if rel else build.cmake_path,
                "line_hint": target.line_hint,
                "sources": [
                    rel(str(Path(build.base) / source)) if rel else str(Path(build.base) / source)
                    for source in target.sources
                ],
                "installed": target.installed,
            }
        orphans.extend(
            rel(path) if rel else path for path in build.python_modules_without_executable
        )
    return {
        "executables": dict(sorted(executables.items())),
        "components": dict(sorted(components.items())),
        "libraries": dict(sorted(libraries.items())),
        "python_modules_without_executable": sorted(orphans),
    }


def _component_macro_line(sources: Sequence[str], component: Component) -> int | None:
    for path in sources:
        try:
            text = Path(path).read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        for found in _COMPONENT_MACRO.finditer(text):
            if found.group(1) == component.class_name:
                return text[: found.start()].count("\n") + 1
    return None


# ---------------------------------------------------------------------------
# the collector


@dataclass
class PluginsCollector:
    name: str = NAME
    layer: str = LAYER

    def collect(self, ctx: CollectContext) -> None:
        ws = ctx.ws
        builder = ctx.builder
        profile = ids.build_profile(ctx.profile) if ctx.profile else model.ANY_PROFILE
        libraries = library_targets(ws)
        builds: list[PackageBuild] = []
        problems: list[dict[str, Any]] = []
        for pkg in sorted(ws.packages):
            try:
                builds.append(read_package(ws, pkg, libraries=libraries))
            except (CollectorError, ContractError) as exc:
                package = ws.packages[pkg]
                base = package.src_path or package.share_path or ""
                problems.append(
                    {
                        "package": pkg,
                        "phase": "describe",
                        "exc_type": type(exc).__name__,
                        # The message is compared data, so the machine path leaves it.
                        "message": str(exc).replace(base, pkg) if base else str(exc),
                    }
                )
        run = builder.add_run(
            LAYER,
            profile=profile if ctx.profile else None,
            input_digest=ids.digest(
                NAME,
                *sorted(ws.packages),
                *[f"{key}={value}" for key, value in sorted(ws.root_hashes().items())],
            ),
            roots=ws.root_hashes(),
            exceptions=problems,
        )
        builder.declare_layer(LAYER, present=True, run=run)
        late = builder.atlas.entities[run].attrs["exceptions"]
        for build in builds:
            # One package that cannot be emitted must not delete every package sorted after
            # it. Before this, one unexpandable CMake variable dropped 22 packages, 14
            # executables and 28 plugin classes with no line anywhere saying so.
            try:
                self._emit_plugins(ctx, run, profile, build)
                self._emit_executables(ctx, run, profile, build)
                self._emit_libraries(ctx, run, profile, build)
                self._emit_unresolved_targets(ctx, build)
            except (CollectorError, ContractError) as exc:
                late.append(
                    {
                        "package": build.pkg,
                        "phase": "emit",
                        "exc_type": type(exc).__name__,
                        "message": str(exc).replace(build.base, build.pkg),
                    }
                )
        late.sort(key=lambda row: tuple(str(row.get(key, "")) for key in _PROBLEM_KEYS))

    # -----------------------------------------------------------------
    def _emit_plugins(
        self,
        ctx: CollectContext,
        run: str,
        profile: str,
        build: PackageBuild,
    ) -> None:
        ws = ctx.ws
        builder = ctx.builder
        if not build.manifests:
            return
        pkg_id = ids.build_pkg(build.pkg)
        builder.node(pkg_id)
        for manifest in build.manifests:
            file_id = ws.file_id(manifest.path)
            attrs: dict[str, Any] = {
                "exists": manifest.exists,
                "exported_by": list(manifest.exported_by),
                "export_tags": list(manifest.export_tags),
            }
            if manifest.library:
                attrs["library"] = manifest.library
            if manifest.library_built_here is not None:
                attrs["library_built_here"] = manifest.library_built_here
            if not manifest.exists:
                attrs["loadable"] = False
                attrs["refused_because"] = "plugin_manifest_missing"
                attrs["note"] = "exported by package.xml and absent on disk"
                held = _held_manifests(build, manifest.path)
                if held:
                    attrs["manifests_held"] = [ws.file_id(path) for path in held]
            elif manifest.unreadable:
                attrs["loadable"] = False
                attrs["refused_because"] = "plugin_manifest_unreadable"
                attrs["refused_detail"] = manifest.unreadable.replace(
                    manifest.path, ws.file_id(manifest.path)
                )
            builder.node(file_id, attrs=attrs)
            builder.edge(model.EdgeKind.CONTAINS.value, pkg_id, file_id)
            source = ws.source(ws.site(manifest.path))
            if manifest.exists and manifest.unreadable:
                # The blind spot sits on the manifest and not on the package, because the
                # entity a reader opens after seeing `loadable false` is the file.
                builder.unresolved(
                    model.Unresolved(
                        want=model.Want.OTHER.value,
                        owner=file_id,
                        expr=manifest.unreadable.replace(manifest.path, file_id),
                        reason="plugin_manifest_unreadable",
                        role="pluginlib",
                        resolvable_by=["fix the manifest xml"],
                        evidence=[source],
                    )
                )
                continue
            if not manifest.exists:
                builder.unresolved(
                    model.Unresolved(
                        want=model.Want.OTHER.value,
                        owner=file_id,
                        expr=file_id,
                        reason="plugin_manifest_missing",
                        role="pluginlib",
                        resolvable_by=["install the manifest", "drop the package.xml export"],
                        evidence=[source],
                    )
                )
                continue
            if manifest.library and manifest.library_built_here is False:
                builder.unresolved(
                    model.Unresolved(
                        want=model.Want.OTHER.value,
                        owner=pkg_id,
                        expr=manifest.library,
                        reason="plugin_library_not_built",
                        role="pluginlib",
                        resolvable_by=["add the library target", "fix the library path"],
                        evidence=[source],
                    )
                )
            for item in manifest.classes:
                plugin_id = item.entity_id
                values: dict[str, Any] = {
                    "type": item.type,
                    "base_class": item.base_class,
                    "lookup_name": item.lookup_name,
                    "name_attribute_present": item.name_attribute_present,
                    "exported_by": list(manifest.exported_by),
                }
                if manifest.library:
                    values["library"] = manifest.library
                if manifest.library_built_here is not None:
                    # An unknown state is left out: false would read as a library nothing
                    # builds.
                    values["library_built_here"] = manifest.library_built_here
                builder.fact(
                    plugin_id,
                    LAYER,
                    profile=profile,
                    run=run,
                    values=values,
                    confidence=model.Confidence.CERTAIN.value,
                    evidence=[ws.source(ws.site(manifest.path, f"class[{item.lookup_name}]"))],
                )
                builder.edge(model.EdgeKind.PROVIDES_PLUGIN.value, pkg_id, plugin_id)
                for macro in build.export_macros:
                    if macro.type != item.type:
                        continue
                    macro_site = ws.site(macro.file, "PLUGINLIB_EXPORT_CLASS", macro.line_hint)
                    macro_file = ws.file_id(macro.file)
                    builder.node(macro_file)
                    builder.edge(model.EdgeKind.CONTAINS.value, pkg_id, macro_file)
                    builder.edge(
                        model.EdgeKind.DEFINES.value,
                        macro_file,
                        plugin_id,
                        layer=LAYER,
                        profile=profile,
                        run=run,
                        values={"macro": "PLUGINLIB_EXPORT_CLASS", "wrapped": macro.wrapped},
                        confidence=model.Confidence.CERTAIN.value,
                        evidence=[ws.source(macro_site)],
                    )
                builder.count_resolved(model.Want.OTHER.value)

    def _emit_unresolved_targets(self, ctx: CollectContext, build: PackageBuild) -> None:
        """One blind spot per build target this reader could not name."""
        if not build.scan.unresolved_targets:
            return
        pkg_id = ids.build_pkg(build.pkg)
        ctx.builder.node(pkg_id)
        for target in build.scan.unresolved_targets:
            site = ctx.ws.site(
                build.cmake_path,
                f"{target.command}({target.expression})",
                target.line_hint,
            )
            ctx.builder.unresolved(
                model.Unresolved(
                    want=model.Want.OTHER.value,
                    owner=pkg_id,
                    expr=f"{target.command}({target.expression})",
                    reason="cmake_target_name_unresolved",
                    role="build_target",
                    resolvable_by=["name the target with a literal", "read the CMake variable"],
                    evidence=[ctx.ws.source(site)],
                )
            )

    def _emit_libraries(
        self, ctx: CollectContext, run: str, profile: str, build: PackageBuild
    ) -> None:
        """Library sources are build claims, even when no component exports the target."""
        rows = []
        for name, target in sorted(build.scan.targets.items()):
            if target.kind != "library" or not usable_target(name):
                continue
            sources = []
            for path in target.sources:
                try:
                    file_id = ctx.ws.file_id(str(Path(build.base) / path))
                except ContractError:
                    continue
                ctx.builder.node(file_id)
                sources.append(file_id)
            row = {
                "target": name,
                "sources": sorted(sources),
                "linked_targets": sorted(target.linked_targets),
            }
            registered = sorted(
                {
                    item.class_name
                    for item in build.scan.components
                    if item.target == name and item.mechanism == MECHANISM_COMPONENT_NODES
                }
            )
            if registered:
                row["components"] = registered
            rows.append(row)
        for name, path in build.scan.parameter_libraries:
            definition = Path(build.base) / path
            try:
                definition_id = ctx.ws.file_id(str(definition))
            except ContractError:
                continue
            ctx.builder.node(definition_id)
            row = {
                "target": name,
                "sources": [],
                "linked_targets": [],
                "definition": definition_id,
            }
            try:
                names, patterns = parameter_definition(_read(ctx.ws, definition))
                row["parameters"] = names
                if patterns:
                    row["parameter_patterns"] = patterns
            except (CollectorError, yaml.YAMLError):
                row["parameters_unread"] = True
            rows.append(row)
        rows.sort(key=lambda row: row["target"])
        if rows:
            file_id = ctx.ws.file_id(build.cmake_path)
            ctx.builder.fact(
                file_id,
                LAYER,
                profile=profile,
                run=run,
                values={"library_targets": rows},
                confidence=model.Confidence.CERTAIN.value,
                evidence=[ctx.ws.source(ctx.ws.site(build.cmake_path))],
            )

    def _emit_executables(
        self,
        ctx: CollectContext,
        run: str,
        profile: str,
        build: PackageBuild,
    ) -> None:
        ws = ctx.ws
        builder = ctx.builder
        if not build.executables:
            return
        pkg_id = ids.build_pkg(build.pkg)
        builder.node(pkg_id)
        for item in build.executables:
            exe_id = item.entity_id
            source_ids: list[str] = []
            outside = 0
            for path in item.sources:
                try:
                    source_ids.append(ws.file_id(path))
                except ContractError:
                    # A generated source under a build directory is outside every root.
                    outside += 1
            values: dict[str, Any] = {
                "mechanism": item.mechanism,
                "sources": source_ids,
                "has_main": item.has_main,
                "installed": item.installed,
            }
            if outside:
                values["sources_outside_workspace"] = outside
            if item.target:
                values["target"] = item.target
            if item.mechanism == MECHANISM_COMPONENT_NODE:
                values["component_class"] = item.symbol
            if item.module:
                values["module"] = item.module
                values["entry"] = item.entry or ""
            if item.dependencies:
                values["dependencies"] = list(item.dependencies)
            if item.linked_targets:
                values["linked_targets"] = list(item.linked_targets)
            declared_site = ws.site(
                item.declared_in, f"{item.mechanism}({item.name})", item.line_hint
            )
            builder.fact(
                exe_id,
                LAYER,
                profile=profile,
                run=run,
                values=values,
                confidence=model.Confidence.CERTAIN.value,
                evidence=[ws.source(declared_site)],
            )
            builder.edge(model.EdgeKind.CONTAINS.value, pkg_id, exe_id)
            for source_id in source_ids:
                builder.node(source_id)
                builder.edge(model.EdgeKind.CONTAINS.value, pkg_id, source_id)
            if not item.symbol or not item.symbol_file:
                builder.unresolved(
                    model.Unresolved(
                        want=model.Want.OTHER.value,
                        owner=exe_id,
                        expr=item.name,
                        reason="executable_entry_unresolved",
                        role="exe_to_source",
                        resolvable_by=["read the sources in the coded layer"],
                        evidence=[ws.source(declared_site)],
                    )
                )
                continue
            package, relative = ws.rel(item.symbol_file)
            sym_id = ids.build_sym(package or ids.REPO_PKG, relative, item.symbol)
            builder.node(sym_id)
            source_id = ws.file_id(item.symbol_file)
            builder.node(source_id)
            builder.edge(
                model.EdgeKind.DEFINES.value,
                source_id,
                sym_id,
                layer=LAYER,
                profile=profile,
                run=run,
                values={"mechanism": item.mechanism},
                confidence=model.Confidence.RESOLVED.value,
                evidence=[ws.source(ws.site(item.symbol_file, item.symbol))],
            )
            builder.edge(
                model.EdgeKind.RUNS.value,
                exe_id,
                sym_id,
                layer=LAYER,
                profile=profile,
                run=run,
                values={"mechanism": item.mechanism},
                confidence=model.Confidence.RESOLVED.value,
                evidence=[ws.source(declared_site)],
            )
            builder.count_resolved(model.Want.OTHER.value)


register(NAME, PluginsCollector)


__all__ = [
    "CmakeExport",
    "CmakeScan",
    "Command",
    "Component",
    "ConsoleScript",
    "Executable",
    "ExportMacro",
    "Manifest",
    "PackageBuild",
    "PackageXmlExport",
    "PluginClass",
    "PluginsCollector",
    "Target",
    "UnresolvedTarget",
    "commands",
    "executable_view",
    "library_targets",
    "package_xml_exports",
    "parse_manifest_text",
    "plugin_view",
    "problems",
    "read_package",
    "scan_cmake",
    "scan_console_scripts",
    "scan_export_macros",
    "usable_target",
]
