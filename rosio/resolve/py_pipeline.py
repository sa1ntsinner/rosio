"""Parameter documents assembled in python, before launch ever sees one.

A launcher module loads a yaml parameter file, deep merges a second one, writes a few keys
into the loaded structure and dumps the result into a temp file, then hands that temp path
to `ros2 launch`. The running stack is configured by a file that exists in no repository,
so a reader that stops at the repository yaml states the wrong value with full confidence.

This module reads such a module with `ast` and produces `records.ParamPipeline`. It never
executes the module and never guesses: a path or a value that does not fold is recorded with
its raw expression text, and the pipeline is marked incomplete.

Two things are resolved across module boundaries, because the reference launcher keeps its
paths in a sibling module: module level constants, and the recursive merge helper, which is
recognised by its shape rather than its name (the reference repo holds four independent
copies of it under four different names).

A launch file does the same thing to itself. It loads a base parameters yaml, merges an
overlay, writes a few keys, dumps a temp file and hands that path to two includes, all inside
one nested function of one launch file. That is the same pipeline with
three differences the analyser has to cover: the constants are locals of the enclosing
function rather than module level, the merge helper is nested too, and the produced path
reaches a launch argument in a dict instead of a command string. `launch_file=True` names the
file itself as the sink target, because the invocation that receives the document is the
launch file being walked.

Three record conventions a consumer has to know:

* One module builds more than one document, and both are handed to an argument called
  `params_file`. The argument name alone therefore binds nothing. `sink_target` carries the
  package and the launch file or executable of the command string the document feeds, and
  `sink_function` carries the function that built it. Bind on those.
* A guarded write carries `resolved` false, because the write happens only when the
  condition holds and a consumer reading `value` and `resolved` must not read it as live
  configuration. The value it would write is still on the record, the condition is in the
  site locator, and `override_guard` returns it.
* A write whose value does not fold is a blind spot on that one key and not on the document.
  The override carries `resolved` false with the expression text, and `complete` stays true
  unless the key path itself is unknown, because then the write could have landed anywhere.
* A folded path is repository relative. The keys whose value is a path are listed in the
  `path_value=` notes, so a caller that also wants the absolute form the running node sees
  knows which values to rebuild against a repository root.
"""

from __future__ import annotations

import ast
import re
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path, PurePosixPath
from typing import Any

from rosio.errors import CollectorError
from rosio.resolve import parse_python, records

YAML_LOADERS = ("safe_load", "load", "full_load", "unsafe_load")
YAML_DUMPERS = ("dump", "safe_dump")

PATH_IDENTITY = ("resolve", "absolute", "expanduser")
CONTAINER_CALLS = ("setdefault", "get")
SHARE_CALLS = ("get_package_share_directory", "get_package_share_path", "FindPackageShare")
YAML_MODULES = ("yaml", "ruamel")
LAUNCH_PARAM_KEYWORDS = ("parameters", "launch_arguments")

NOTE_AFTER_DUMP = "after_dump="
NOTE_SINK_ARG = "sink_argument="
NOTE_SINK_TARGET = "sink_target="
NOTE_SINK_FUNCTION = "sink_function="
NOTE_OUTPUT_PATH = "output_path="
NOTE_PATH_VALUE = "path_value="
NOTE_TEMP_SUFFIX = "temp_suffix="
NOTE_CONDITIONAL = "conditional="
NOTE_ENV_UNSET = "env_unset="
NOTE_FALLBACK = "fallback="
NOTE_UNRESOLVED = "unresolved="
NOTE_LAUNCH_ARG = "launch_argument="
NOTE_REPO_RELATIVE = "paths are relative to the repository root"

GUARD_SEPARATOR = " if "

INTERPOLATION = "${?}"
"""Stands in for an interpolated piece of a command string, so a pattern cannot read across
one and invent a package or a launch file out of the two literals around it."""

MAX_HELPER_DEPTH = 2
"""A write helper called from the pipeline function is followed, and a helper it calls in
turn is followed once. Deeper than that the trail is recorded, not walked."""


class _Unresolved:
    __slots__ = ()

    def __repr__(self) -> str:
        return "<unresolved>"


UNKNOWN = _Unresolved()


@dataclass(frozen=True)
class RepoPath:
    """A folded filesystem path. `text` is relative to the repository root unless the
    expression started at an absolute literal, because an absolute path is machine
    dependent and two boxes must not differ on it."""

    text: str
    absolute: bool = False

    def joined(self, part: str) -> RepoPath:
        if part.startswith("/"):
            return RepoPath(str(PurePosixPath(part)), absolute=True)
        base = PurePosixPath(self.text) if self.text else PurePosixPath()
        return RepoPath(str(base / part), absolute=self.absolute)

    def up(self, levels: int) -> RepoPath | _Unresolved:
        """`parents[n]` and `.parent` both land here: `levels` components are dropped."""
        parts = [part for part in self.text.split("/") if part]
        if levels > len(parts):
            return UNKNOWN
        kept = parts[: len(parts) - levels]
        text = "/".join(kept)
        return RepoPath("/" + text if self.absolute else text, absolute=self.absolute)


@dataclass(frozen=True)
class ModuleResult:
    """What one python module produced. A parse error is reported, never raised, so one
    unreadable module does not empty the layer."""

    path: str
    pipelines: tuple[records.ParamPipeline, ...] = ()
    parse_error: str = ""
    notes: tuple[str, ...] = ()


# ---------------------------------------------------------------------------
# public entry points


def analyse_file(
    path: str,
    relpath: str = "",
    text: str | None = None,
    *,
    packages: Mapping[str, str] | None = None,
    launch_file: bool = False,
    tree: ast.Module | None = None,
) -> ModuleResult:
    """Read one module and return the parameter pipelines it builds.

    `relpath` is the module path relative to the repository root. Without it a constant
    anchored at `Path(__file__).resolve().parents[n]` cannot fold, which is how the
    reference launcher spells every one of its paths.

    `packages` maps a package name to its repository relative directory, which is what lets
    a path anchored at the package share fold. `launch_file` says the module is itself the
    invocation the produced document is handed to.
    """
    if text is None:
        try:
            text = Path(path).read_text(encoding="utf-8", errors="replace")
        except OSError as exc:
            return ModuleResult(path=str(path), parse_error=str(exc))
    return analyse_source(
        path, text, relpath, packages=packages, launch_file=launch_file, tree=tree
    )


def analyse_source(
    path: str,
    text: str,
    relpath: str = "",
    *,
    packages: Mapping[str, str] | None = None,
    launch_file: bool = False,
    tree: ast.Module | None = None,
) -> ModuleResult:
    """`tree` is the parse of `text` when the caller already holds it."""
    if tree is None:
        try:
            tree = parse_python(text)
        except SyntaxError as exc:
            return ModuleResult(path=str(path), parse_error=f"{type(exc).__name__}: {exc}")
    registry = _Registry(packages)
    module = registry.adopt(str(path), tree, relpath)
    drafts: list[_Draft] = []
    for enclosing, function in _functions(tree):
        analyser = _Analyser(
            module,
            registry,
            outer=_outer_scope(module, enclosing),
            launch_file=launch_file,
        )
        drafts.extend(analyser.run(function))
    notes = registry.notes()
    pipelines = [
        records.ParamPipeline(
            output_token=draft.output_token,
            base=draft.base,
            overlays=tuple(draft.overlays),
            overrides=tuple(draft.overrides),
            applied_by=draft.applied_by,
            site=draft.site,
            complete=draft.complete,
            notes=tuple(sorted(set(draft.notes) | set(notes))),
        )
        for draft in drafts
    ]
    pipelines.sort(key=lambda item: (item.applied_by, item.output_token))
    return ModuleResult(path=str(path), pipelines=tuple(pipelines), notes=tuple(notes))


def fold_constants(path: str, relpath: str = "", text: str | None = None) -> dict[str, Any]:
    """Module level constants that fold, for a caller that wants the table itself."""
    if text is None:
        try:
            text = Path(path).read_text(encoding="utf-8", errors="replace")
        except OSError as exc:
            raise CollectorError(f"cannot read {path}: {exc}") from exc
    try:
        tree = parse_python(text)
    except SyntaxError as exc:
        raise CollectorError(f"cannot parse {path}: {exc}") from exc
    registry = _Registry()
    module = registry.adopt(str(path), tree, relpath)
    out: dict[str, Any] = {}
    for name in sorted(module.assigns):
        value = module.value(name)
        if value is not UNKNOWN:
            out[name] = plain(value)
    return out


def is_merge_helper(function: ast.AST) -> bool:
    """True for a recursive two dict walk, whatever it is called.

    Shape, not name: the reference repo holds four copies of this function under four
    different names, so matching on a name finds one of them at best.
    """
    if not isinstance(function, (ast.FunctionDef, ast.AsyncFunctionDef)):
        return False
    args = list(function.args.posonlyargs) + list(function.args.args)
    if len(args) < 2:
        return False
    first, second = args[0].arg, args[1].arg
    recursive = False
    dict_test = False
    writes = False
    reads_second = False
    for node in ast.walk(function):
        if isinstance(node, ast.Call):
            if isinstance(node.func, ast.Name) and node.func.id == function.name:
                recursive = True
            if isinstance(node.func, ast.Name) and node.func.id == "isinstance":
                dict_test = dict_test or _tests_dict(node)
            if isinstance(node.func, ast.Attribute) and node.func.attr in ("items", "keys"):
                reads_second = reads_second or _is_name(node.func.value, second)
        if isinstance(node, ast.For) and _is_name(node.iter, second):
            reads_second = True
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Subscript) and _is_name(target.value, first):
                    writes = True
    return recursive and dict_test and writes and reads_second


def dotted(key_path: tuple[str, ...]) -> str:
    return ".".join(key_path)


def override_guard(override: records.ParamOverride) -> str:
    """The `if` text of a conditional write, empty when the write is unconditional.

    The guard is read off the locator behind the key path, not behind the first `if` in it,
    because a key can hold that word itself and an unconditional write must never come back
    as a conditional one.
    """
    locator = override.site.locator if override.site else ""
    head, separator, tail = locator.partition(f"{dotted(override.key_path)}{GUARD_SEPARATOR}")
    return tail if separator and head.endswith(":") else ""


def sink_argument(pipeline: records.ParamPipeline) -> str:
    """The launch argument or argv flag the produced file is handed to."""
    return _note_value(pipeline, NOTE_SINK_ARG)


def launch_argument(pipeline: records.ParamPipeline) -> str:
    """The launch argument or keyword the document reaches inside its own launch file.

    Empty for a document handed to a command string, which is the launcher module case.
    """
    return _note_value(pipeline, NOTE_LAUNCH_ARG)


def override_folded(override: records.ParamOverride) -> bool:
    """Whether the analyser folded the value the write puts there.

    A guarded write is not live configuration, so it is never `resolved`, but its value can
    be perfectly well known. The two questions have different answers and different readers:
    liveness gates the candidate chain, folding gates the resolution rate.
    """
    return override.resolved or (bool(override_guard(override)) and override.value is not None)


def sink_target(pipeline: records.ParamPipeline) -> str:
    """The invocation the produced file feeds, as `<package> <launch file or executable>`.

    The argument name alone does not bind a pipeline: one launcher module builds two
    parameter documents and hands both to an argument called `params_file`. The command
    string is what tells them apart, so it travels with the record.
    """
    return _note_value(pipeline, NOTE_SINK_TARGET)


def sink_function(pipeline: records.ParamPipeline) -> str:
    """The function that built the document and the command string."""
    return _note_value(pipeline, NOTE_SINK_FUNCTION) or pipeline.applied_by.rsplit(".", 1)[-1]


def output_path(pipeline: records.ParamPipeline) -> str:
    """The repository relative path the document was dumped to, for a named sink."""
    return _note_value(pipeline, NOTE_OUTPUT_PATH)


def path_valued(pipeline: records.ParamPipeline) -> tuple[str, ...]:
    """The dotted keys whose folded value is a repository relative path."""
    prefix = NOTE_PATH_VALUE
    return tuple(note[len(prefix) :] for note in pipeline.notes if note.startswith(prefix))


def temp_suffix(pipeline: records.ParamPipeline) -> str:
    """The suffix of the temp file, which is how an observed `/tmp` path is recognised."""
    return _note_value(pipeline, NOTE_TEMP_SUFFIX)


def _note_value(pipeline: records.ParamPipeline, prefix: str) -> str:
    for note in pipeline.notes:
        if note.startswith(prefix):
            return note[len(prefix) :]
    return ""


def plain(value: Any) -> Any:
    """A folded value as json data. A path becomes its repository relative text."""
    if isinstance(value, RepoPath):
        return value.text
    if isinstance(value, (list, tuple)):
        return [plain(item) for item in value]
    if isinstance(value, dict):
        return {key: plain(item) for key, item in value.items()}
    return None if value is UNKNOWN else value


# ---------------------------------------------------------------------------
# modules and constant folding


class _Registry:
    """Every module read while folding one file, so a sibling is parsed once."""

    def __init__(self, packages: Mapping[str, str] | None = None) -> None:
        self.modules: dict[str, _Module] = {}
        self.packages = dict(packages or {})

    def adopt(self, path: str, tree: ast.Module, relpath: str) -> _Module:
        module = _Module(path, tree, relpath, self)
        self.modules[str(Path(path))] = module
        return module

    def load(self, path: Path, relpath: str) -> _Module | None:
        key = str(path)
        found = self.modules.get(key)
        if found is not None:
            return found
        try:
            tree = parse_python(path.read_text(encoding="utf-8", errors="replace"))
        except (OSError, SyntaxError, ValueError):
            return None
        return self.adopt(key, tree, relpath)

    def notes(self) -> list[str]:
        out: set[str] = set()
        for module in self.modules.values():
            out.update(module.notes)
        return sorted(out)


class _Module:
    """One parsed module: its constants, its functions and the names it imported."""

    def __init__(self, path: str, tree: ast.Module, relpath: str, registry: _Registry) -> None:
        self.path = str(path)
        self.tree = tree
        self.relpath = relpath.strip("/")
        self.registry = registry
        self.assigns: dict[str, ast.expr] = {}
        self.imports: dict[str, tuple[str, int, str]] = {}
        self.aliases: dict[str, str] = {}
        self.functions: dict[str, ast.FunctionDef | ast.AsyncFunctionDef] = {}
        self.nested: dict[str, ast.FunctionDef | ast.AsyncFunctionDef] = {}
        self.notes: set[str] = set()
        self._folded: dict[str, Any] = {}
        self._active: set[str] = set()
        self._read()

    # ------------------------------------------------------------------

    def _read(self) -> None:
        for node in self.tree.body:
            if isinstance(node, ast.Assign):
                for target in node.targets:
                    if isinstance(target, ast.Name):
                        self.assigns[target.id] = node.value
            elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
                if node.value is not None:
                    self.assigns[node.target.id] = node.value
            elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                self.functions[node.name] = node
            elif isinstance(node, ast.ImportFrom):
                for alias in node.names:
                    local = alias.asname or alias.name
                    self.imports[local] = (node.module or "", node.level, alias.name)
        for node in ast.walk(self.tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    self.aliases[alias.asname or alias.name.split(".")[0]] = alias.name
            elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                self.nested.setdefault(node.name, node)

    @property
    def stem(self) -> str:
        return Path(self.path).stem

    def is_yaml_module(self, expression: str) -> bool:
        """Whether a dotted call expression is rooted at a yaml module, alias included.

        A launch file that spells `import yaml as _yaml` is the ordinary case on this stack,
        and matching the spelling instead of the import reads its merge as no merge at all.
        """
        root = expression.split(".")[0]
        target = self.aliases.get(root, root)
        return target.split(".")[0] in YAML_MODULES

    def package_dir(self, name: str) -> RepoPath | _Unresolved:
        found = self.registry.packages.get(name)
        if not found:
            return UNKNOWN
        return RepoPath(str(PurePosixPath(found)), absolute=found.startswith("/"))

    def self_path(self) -> RepoPath | _Unresolved:
        return RepoPath(self.relpath) if self.relpath else UNKNOWN

    def value(self, name: str) -> Any:
        if name in self._folded:
            return self._folded[name]
        if name in self._active:
            return UNKNOWN
        self._active.add(name)
        try:
            node = self.assigns.get(name)
            folded = self.fold(node) if node is not None else self._imported(name)
        finally:
            self._active.discard(name)
        self._folded[name] = folded
        return folded

    def function(self, name: str) -> tuple[_Module, ast.FunctionDef | ast.AsyncFunctionDef] | None:
        found = self.functions.get(name) or self.nested.get(name)
        if found is not None:
            return self, found
        target = self._module_of(name)
        if target is None:
            return None
        other, original = target
        found = other.functions.get(original)
        return (other, found) if found is not None else None

    def _imported(self, name: str) -> Any:
        target = self._module_of(name)
        if target is None:
            return UNKNOWN
        other, original = target
        return other.value(original)

    def _module_of(self, name: str) -> tuple[_Module, str] | None:
        entry = self.imports.get(name)
        if entry is None:
            return None
        dotted_name, level, original = entry
        other = self._resolve_module(dotted_name, level)
        return None if other is None else (other, original)

    def _resolve_module(self, dotted_name: str, level: int) -> _Module | None:
        parts = [part for part in dotted_name.split(".") if part]
        bases: list[Path] = []
        here = Path(self.path).parent
        if level:
            bases.append(_up_dirs(here, level - 1))
        else:
            bases.extend(_search_roots(here))
        for base in bases:
            for candidate in (
                base.joinpath(*parts).with_suffix(".py"),
                base.joinpath(*parts, "__init__.py"),
            ):
                if candidate.is_file():
                    return self.registry.load(candidate, self._relpath_of(candidate))
        return None

    def _relpath_of(self, target: Path) -> str:
        if not self.relpath:
            return ""
        root = _up_dirs(Path(self.path), len(PurePosixPath(self.relpath).parts))
        try:
            return str(target.relative_to(root)).replace("\\", "/")
        except ValueError:
            return ""

    # ------------------------------------------------------------------
    # the folder

    def fold(self, node: ast.expr | None, local: dict[str, Any] | None = None) -> Any:
        if node is None:
            return UNKNOWN
        handler = getattr(self, f"_fold_{type(node).__name__.lower()}", None)
        if handler is None:
            return UNKNOWN
        return handler(node, local)

    def _fold_constant(self, node: ast.Constant, _local: dict[str, Any] | None) -> Any:
        allowed = (str, int, float, bool, type(None))
        return node.value if isinstance(node.value, allowed) else UNKNOWN

    def _fold_name(self, node: ast.Name, local: dict[str, Any] | None) -> Any:
        if local is not None and node.id in local:
            return local[node.id]
        if node.id == "__file__":
            return self.self_path()
        return self.value(node.id)

    def _fold_joinedstr(self, node: ast.JoinedStr, local: dict[str, Any] | None) -> Any:
        parts: list[str] = []
        for piece in node.values:
            if isinstance(piece, ast.Constant):
                parts.append(str(piece.value))
                continue
            if not isinstance(piece, ast.FormattedValue):
                return UNKNOWN
            folded = self.fold(piece.value, local)
            text = _text(folded)
            if text is None:
                return UNKNOWN
            parts.append(text)
        return "".join(parts)

    def _fold_list(self, node: ast.List, local: dict[str, Any] | None) -> Any:
        return self._fold_sequence(node.elts, local)

    def _fold_tuple(self, node: ast.Tuple, local: dict[str, Any] | None) -> Any:
        return self._fold_sequence(node.elts, local)

    def _fold_sequence(self, elements: list[ast.expr], local: dict[str, Any] | None) -> Any:
        out: list[Any] = []
        for item in elements:
            folded = self.fold(item, local)
            if folded is UNKNOWN:
                return UNKNOWN
            out.append(folded)
        return out

    def _fold_dict(self, node: ast.Dict, local: dict[str, Any] | None) -> Any:
        out: dict[str, Any] = {}
        for key, value in zip(node.keys, node.values, strict=False):
            if not isinstance(key, ast.Constant) or not isinstance(key.value, str):
                return UNKNOWN
            folded = self.fold(value, local)
            if folded is UNKNOWN:
                return UNKNOWN
            out[key.value] = folded
        return out

    def _fold_unaryop(self, node: ast.UnaryOp, local: dict[str, Any] | None) -> Any:
        folded = self.fold(node.operand, local)
        if folded is UNKNOWN:
            return UNKNOWN
        if isinstance(node.op, ast.USub) and isinstance(folded, (int, float)):
            return -folded
        if isinstance(node.op, ast.Not):
            return not folded
        return UNKNOWN

    def _fold_binop(self, node: ast.BinOp, local: dict[str, Any] | None) -> Any:
        left = self.fold(node.left, local)
        right = self.fold(node.right, local)
        if left is UNKNOWN or right is UNKNOWN:
            return UNKNOWN
        if isinstance(node.op, ast.Div):
            part = _text(right)
            return left.joined(part) if isinstance(left, RepoPath) and part else UNKNOWN
        if isinstance(node.op, ast.Add):
            if isinstance(left, list) and isinstance(right, list):
                return left + right
            if isinstance(left, (str, RepoPath)) and isinstance(right, (str, RepoPath)):
                # a path with text glued onto it is text, not a path any more
                return f"{_text(left)}{_text(right)}"
        return UNKNOWN

    def _fold_ifexp(self, node: ast.IfExp, local: dict[str, Any] | None) -> Any:
        test = self.fold(node.test, local)
        if test is UNKNOWN:
            return UNKNOWN
        return self.fold(node.body if test else node.orelse, local)

    def _fold_boolop(self, node: ast.BoolOp, local: dict[str, Any] | None) -> Any:
        folded = UNKNOWN
        for item in node.values:
            folded = self.fold(item, local)
            if folded is UNKNOWN:
                return UNKNOWN
            truthy = bool(folded)
            if isinstance(node.op, ast.Or) and truthy:
                return folded
            if isinstance(node.op, ast.And) and not truthy:
                return folded
        return folded

    def _fold_attribute(self, node: ast.Attribute, local: dict[str, Any] | None) -> Any:
        base = self.fold(node.value, local)
        if isinstance(base, RepoPath):
            if node.attr == "parent":
                return base.up(1)
            if node.attr == "name":
                return PurePosixPath(base.text).name
            if node.attr == "stem":
                return PurePosixPath(base.text).stem
        return UNKNOWN

    def _fold_subscript(self, node: ast.Subscript, local: dict[str, Any] | None) -> Any:
        index = self.fold(node.slice, local)
        if isinstance(node.value, ast.Attribute) and node.value.attr == "parents":
            base = self.fold(node.value.value, local)
            if isinstance(base, RepoPath) and isinstance(index, int):
                return base.up(index + 1)
            return UNKNOWN
        base = self.fold(node.value, local)
        if isinstance(base, (list, str)) and isinstance(index, int):
            try:
                return base[index]
            except IndexError:
                return UNKNOWN
        if isinstance(base, dict) and isinstance(index, str):
            return base.get(index, UNKNOWN)
        return UNKNOWN

    def _fold_call(self, node: ast.Call, local: dict[str, Any] | None) -> Any:
        name = _call_name(node.func)
        if name in ("Path", "PurePath", "PosixPath"):
            return self._path_call(node, local)
        if name in ("str", "fspath"):
            folded = self.fold(node.args[0], local) if node.args else UNKNOWN
            # a path stays a path: the value is written as text, but a caller that wants the
            # absolute form the running node sees has to know this one was a path
            if isinstance(folded, RepoPath):
                return folded
            text = _text(folded)
            return UNKNOWN if text is None else text
        if name in ("int", "float"):
            folded = self.fold(node.args[0], local) if node.args else UNKNOWN
            if isinstance(folded, (int, float, str)) and not isinstance(folded, bool):
                try:
                    return int(folded) if name == "int" else float(folded)
                except ValueError:
                    return UNKNOWN
            return UNKNOWN
        if name == "join":
            return self._path_join(node, local)
        if name in SHARE_CALLS:
            wanted = _text(self.fold(node.args[0], local)) if node.args else None
            return self.package_dir(wanted) if wanted else UNKNOWN
        if name == "dirname" and node.args:
            base = self.fold(node.args[0], local)
            return base.up(1) if isinstance(base, RepoPath) else UNKNOWN
        if name in ("get", "getenv") and _is_environment(node.func):
            return self._environment(node, local)
        if isinstance(node.func, ast.Attribute):
            if node.func.attr in PATH_IDENTITY:
                return self.fold(node.func.value, local)
            if node.func.attr == "joinpath":
                return self._path_join(node, local, base=self.fold(node.func.value, local))
            if node.func.attr == "home":
                # the home directory is a machine fact, not a repository fact
                return UNKNOWN
        return UNKNOWN

    def _path_call(self, node: ast.Call, local: dict[str, Any] | None) -> Any:
        if not node.args:
            return UNKNOWN
        first = self.fold(node.args[0], local)
        if isinstance(first, RepoPath):
            base: RepoPath | _Unresolved = first
        else:
            text = _text(first)
            if text is None:
                return UNKNOWN
            base = RepoPath(str(PurePosixPath(text)), absolute=text.startswith("/"))
        for extra in node.args[1:]:
            part = _text(self.fold(extra, local))
            if part is None or not isinstance(base, RepoPath):
                return UNKNOWN
            base = base.joined(part)
        return base

    def _path_join(
        self,
        node: ast.Call,
        local: dict[str, Any] | None,
        base: Any = None,
    ) -> Any:
        args = list(node.args)
        if base is None:
            if not args:
                return UNKNOWN
            base = self.fold(args[0], local)
            args = args[1:]
            if not isinstance(base, RepoPath):
                text = _text(base)
                if text is None:
                    return UNKNOWN
                base = RepoPath(str(PurePosixPath(text)), absolute=text.startswith("/"))
        for extra in args:
            part = _text(self.fold(extra, local))
            if part is None or not isinstance(base, RepoPath):
                return UNKNOWN
            base = base.joined(part)
        return base

    def _environment(self, node: ast.Call, local: dict[str, Any] | None) -> Any:
        """An environment read folds to its default, with the variable recorded.

        The analyser injects no environment, so the branch it takes is the branch a box
        with that variable unset takes, and the run has to say so.
        """
        key = _text(self.fold(node.args[0], local)) if node.args else None
        if key:
            self.notes.add(f"{NOTE_ENV_UNSET}{key}")
        if len(node.args) > 1:
            return self.fold(node.args[1], local)
        return None


def _up_dirs(path: Path, levels: int) -> Path:
    for _ in range(levels):
        path = path.parent
    return path


def _search_roots(start: Path) -> list[Path]:
    """Directories an absolute import of a sibling package could be resolved against."""
    out: list[Path] = []
    here = start
    while (here / "__init__.py").is_file() and here.parent != here:
        here = here.parent
        out.append(here)
    if not out:
        out.append(start)
    return out


def _call_name(func: ast.expr) -> str:
    if isinstance(func, ast.Name):
        return func.id
    if isinstance(func, ast.Attribute):
        return func.attr
    return ""


def _is_environment(func: ast.expr) -> bool:
    text = _unparse(func)
    return text.startswith(("os.environ.", "environ.", "os.getenv"))


def _text(value: Any) -> str | None:
    if isinstance(value, RepoPath):
        return value.text
    if isinstance(value, str):
        return value
    if isinstance(value, bool) or value is None or value is UNKNOWN:
        return None
    if isinstance(value, (int, float)):
        return str(value)
    return None


def _tests_dict(node: ast.Call) -> bool:
    """`isinstance(x, dict)`, also when the second argument is a tuple of types."""
    if len(node.args) < 2:
        return False
    wanted = node.args[1]
    candidates = wanted.elts if isinstance(wanted, ast.Tuple) else [wanted]
    return any(isinstance(item, ast.Name) and item.id == "dict" for item in candidates)


def _is_name(node: ast.AST, name: str) -> bool:
    return isinstance(node, ast.Name) and node.id == name


def _unparse(node: ast.AST) -> str:
    try:
        return ast.unparse(node)
    except Exception:  # unparse is best effort, the shape still matters
        return type(node).__name__


def _functions(tree: ast.Module) -> list[tuple[tuple[Any, ...], Any]]:
    """Every function with the chain of functions it is nested in, outermost first."""
    out: list[tuple[tuple[Any, ...], Any]] = []

    def visit(node: ast.AST, chain: tuple[Any, ...]) -> None:
        for child in ast.iter_child_nodes(node):
            if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)):
                out.append((chain, child))
                visit(child, (*chain, child))
                continue
            visit(child, chain)

    visit(tree, ())
    return out


def _outer_scope(module: _Module, chain: tuple[Any, ...]) -> dict[str, Any]:
    """The locals of the enclosing functions a nested function reads.

    A launch file keeps its parameter file paths in `generate_launch_description` and does
    the merge in a nested function, so without this the base document never folds.
    """
    scope: dict[str, Any] = {}
    for enclosing in chain:
        for statement in _plain_statements(enclosing.body):
            if not isinstance(statement, ast.Assign) or len(statement.targets) != 1:
                continue
            target = statement.targets[0]
            if not isinstance(target, ast.Name):
                continue
            folded = module.fold(statement.value, scope)
            if folded is not UNKNOWN:
                scope[target.id] = folded
    return scope


def _plain_statements(body: list[ast.stmt]) -> list[ast.stmt]:
    """Statements of a body and of the blocks inside it, without entering a nested def."""
    out: list[ast.stmt] = []
    for statement in body:
        if isinstance(statement, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            continue
        out.append(statement)
        for field_name in ("body", "orelse", "finalbody"):
            inner = getattr(statement, field_name, None)
            if isinstance(inner, list):
                out.extend(_plain_statements(inner))
    return out


# ---------------------------------------------------------------------------
# one function body


@dataclass
class _Draft:
    var: str
    site: records.Site
    applied_by: str
    base: str | None = None
    base_resolved: bool = True
    overlays: list[str] = field(default_factory=list)
    overrides: list[records.ParamOverride] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    complete: bool = True
    temp_var: str = ""
    suffix: str = ""
    sink: str = ""
    output_token: str = ""
    output_path: str = ""
    consumed: bool = False
    dumped: bool = False

    def content(self) -> bool:
        """True once the document is more than a plain read, so it is worth reporting."""
        return bool(self.overrides or self.overlays or self.dumped)

    def unresolved(self, what: str) -> None:
        self.complete = False
        note = f"{NOTE_UNRESOLVED}{what}"
        if note not in self.notes:
            self.notes.append(note)


@dataclass(frozen=True)
class _Sink:
    """Where the produced file goes: the token, the argument it fills, and the command."""

    token: str = ""
    argument: str = ""
    target: str = ""
    launch: bool = False
    """True when the document is consumed by the launch file it was assembled in, rather
    than handed to a command string."""


@dataclass(frozen=True)
class _Alias:
    doc: str
    path: tuple[str, ...]
    resolved: bool = True


class _Analyser:
    """Follows one function body from the yaml load to the command string."""

    def __init__(
        self,
        module: _Module,
        registry: _Registry,
        outer: Mapping[str, Any] | None = None,
        launch_file: bool = False,
    ) -> None:
        self.module = module
        self.registry = registry
        self.launch_file = launch_file
        self.drafts: dict[str, _Draft] = {}
        self.aliases: dict[str, _Alias] = {}
        self.handles: dict[str, Any] = {}
        self.temps: dict[str, str] = {}
        self.paths: dict[str, str] = {}
        self.scope: dict[str, Any] = dict(outer or {})
        self.function = ""
        self.depth = 0
        self.prefix: tuple[str, ...] = ()
        self.prefix_resolved = True
        self.repo_relative = False

    # ------------------------------------------------------------------

    def run(self, function: ast.FunctionDef | ast.AsyncFunctionDef) -> list[_Draft]:
        self.function = function.name
        self._walk(function.body, ())
        self._trace_sink(function)
        out: list[_Draft] = []
        for draft in self.drafts.values():
            if draft.consumed or not draft.content():
                continue
            if self.repo_relative:
                draft.notes.append(NOTE_REPO_RELATIVE)
            draft.notes.append(f"{NOTE_SINK_FUNCTION}{self.function}")
            out.append(draft)
        return out

    # ------------------------------------------------------------------
    # statements

    def _walk(self, body: list[ast.stmt], guards: tuple[str, ...]) -> None:
        for statement in body:
            self._statement(statement, guards)

    def _statement(self, node: ast.stmt, guards: tuple[str, ...]) -> None:
        if isinstance(node, ast.If):
            test = _unparse(node.test)
            self._walk(node.body, (*guards, test))
            self._walk(node.orelse, (*guards, f"not ({test})"))
            return
        if isinstance(node, ast.Try):
            self._walk(node.body, guards)
            self._walk(node.orelse, guards)
            self._walk(node.finalbody, guards)
            self._handlers(node)
            return
        if isinstance(node, ast.With):
            self._with(node, guards)
            return
        if isinstance(node, (ast.For, ast.While)):
            self._walk(node.body, guards)
            return
        if isinstance(node, ast.Assign):
            self._assign(node, guards)
            return
        if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call):
            self._call(node.value, guards)

    def _handlers(self, node: ast.Try) -> None:
        """The except path is not the pipeline, but a fallback it assigns is worth saying."""
        for handler in node.handlers:
            for statement in ast.walk(handler):
                if not isinstance(statement, ast.Assign):
                    continue
                for target in statement.targets:
                    if isinstance(target, ast.Name) and target.id in self.paths:
                        for draft in self.drafts.values():
                            draft.notes.append(
                                f"{NOTE_FALLBACK}{target.id} = {_unparse(statement.value)}"
                            )

    def _with(self, node: ast.With, guards: tuple[str, ...]) -> None:
        for item in node.items:
            target = item.optional_vars
            if not isinstance(target, ast.Name):
                continue
            call = item.context_expr
            if not isinstance(call, ast.Call):
                continue
            opened = self._opened(call)
            if opened is not None:
                self.handles[target.id] = opened
            suffix = self._temp_suffix(call)
            if suffix is not None:
                self.temps[target.id] = suffix
        self._walk(node.body, guards)

    def _assign(self, node: ast.Assign, guards: tuple[str, ...]) -> None:
        value = node.value
        for target in node.targets:
            if isinstance(target, ast.Subscript):
                self._subscript_write(target, value, guards)
                continue
            if not isinstance(target, ast.Name):
                continue
            name = target.id
            if self._load(name, value, guards):
                continue
            suffix = self._temp_suffix(value)
            if suffix is not None:
                self.temps[name] = suffix
                continue
            temp = self._temp_name(value)
            if temp:
                self.paths[name] = temp
                continue
            opened = self._opened(value)
            if opened is not None:
                self.handles[name] = opened
                continue
            merged = self._merge(value, guards)
            if merged is not None:
                self.aliases.pop(name, None)
                continue
            alias = self._alias(value)
            if alias is not None:
                self.aliases[name] = alias
                continue
            folded = self._fold(value)
            if folded is not UNKNOWN:
                self.scope[name] = folded

    def _call(self, node: ast.Call, guards: tuple[str, ...]) -> None:
        if self._merge(node, guards) is not None:
            return
        if self._dump(node):
            return
        if self._update(node, guards):
            return
        self._helper(node, guards, depth=self.depth + 1)

    # ------------------------------------------------------------------
    # recognisers

    def _opened(self, node: ast.expr) -> Any:
        """`open(P)` or `P.read_text()`, folded to the path it reads."""
        if not isinstance(node, ast.Call):
            return None
        if isinstance(node.func, ast.Name) and node.func.id == "open" and node.args:
            return (self._fold(node.args[0]), _unparse(node.args[0]))
        if isinstance(node.func, ast.Attribute) and node.func.attr in ("read_text", "read_bytes"):
            return (self._fold(node.func.value), _unparse(node.func.value))
        return None

    def _temp_suffix(self, node: ast.expr) -> str | None:
        if not isinstance(node, ast.Call):
            return None
        text = _unparse(node.func)
        if "NamedTemporaryFile" not in text and "mkstemp" not in text:
            return None
        for keyword in node.keywords:
            if keyword.arg == "suffix":
                folded = self._fold(keyword.value)
                return _text(folded) or ""
        return ""

    def _temp_name(self, node: ast.expr) -> str:
        if isinstance(node, ast.Attribute) and node.attr == "name":
            base = node.value
            if isinstance(base, ast.Name) and base.id in self.temps:
                return base.id
        return ""

    def _yaml_load(self, node: ast.expr) -> ast.Call | None:
        """The yaml load call an expression is, or None."""
        call = _strip_or(node)
        if not isinstance(call, ast.Call):
            return None
        if _call_name(call.func) not in YAML_LOADERS or not call.args:
            return None
        if isinstance(call.func, ast.Attribute) and not self.module.is_yaml_module(
            _unparse(call.func)
        ):
            return None
        return call

    def _load(self, name: str, value: ast.expr, guards: tuple[str, ...]) -> bool:
        """A yaml load bound to a name starts a document."""
        call = self._yaml_load(value)
        if call is None:
            return False
        source, expr = self._source_of(call.args[0])
        site = records.Site(
            path=self.module.path,
            locator=f"{self.function}:{name}",
            line_hint=getattr(value, "lineno", None),
        )
        draft = _Draft(
            var=name,
            site=site,
            applied_by=f"{self.module.stem}.{self.function}",
        )
        folded = _as_path(source)
        if folded is None:
            draft.base_resolved = False
            draft.unresolved(f"base {expr}")
        else:
            draft.base = folded.text
            self.repo_relative = self.repo_relative or not folded.absolute
        if guards:
            draft.notes.append(f"{NOTE_CONDITIONAL}{name}{GUARD_SEPARATOR}{' and '.join(guards)}")
        self.drafts[name] = draft
        self.aliases.pop(name, None)
        return True

    def _source_of(self, node: ast.expr) -> tuple[Any, str]:
        if isinstance(node, ast.Name) and node.id in self.handles:
            return self.handles[node.id]
        opened = self._opened(node)
        if opened is not None:
            return opened
        return (UNKNOWN, _unparse(node))

    def _merge(self, node: ast.expr, guards: tuple[str, ...]) -> _Draft | None:
        """`merge(base, overlay)` where merge walks two dicts recursively."""
        if not isinstance(node, ast.Call) or len(node.args) < 2:
            return None
        found = self.module.function(_call_name(node.func))
        if found is None or not is_merge_helper(found[1]):
            return None
        target = self._alias(node.args[0])
        if target is None or target.doc not in self.drafts:
            return None
        draft = self.drafts[target.doc]
        overlay = node.args[1]
        loaded = self._yaml_load(overlay)
        if isinstance(overlay, ast.Name) and overlay.id in self.drafts:
            other = self.drafts[overlay.id]
            other.consumed = True
            if other.base is None:
                draft.unresolved(f"overlay {overlay.id}")
            else:
                draft.overlays.append(other.base)
                draft.overrides.extend(other.overrides)
        elif loaded is not None:
            # the overlay is loaded straight into the merge, which is what a launch file does
            source, expr = self._source_of(loaded.args[0])
            folded = _as_path(source)
            if folded is None:
                draft.unresolved(f"overlay {expr}")
            else:
                draft.overlays.append(folded.text)
                self.repo_relative = self.repo_relative or not folded.absolute
        else:
            draft.unresolved(f"overlay {_unparse(overlay)}")
        if guards:
            draft.notes.append(f"{NOTE_CONDITIONAL}overlay{GUARD_SEPARATOR}{' and '.join(guards)}")
        return draft

    def _dump(self, node: ast.Call) -> bool:
        if _call_name(node.func) not in YAML_DUMPERS or len(node.args) < 2:
            return False
        alias = self._alias(node.args[0])
        if alias is None or alias.doc not in self.drafts:
            return False
        sink = node.args[1]
        draft = self.drafts[alias.doc]
        draft.dumped = True
        if isinstance(sink, ast.Name) and sink.id in self.temps:
            draft.temp_var = sink.id
            draft.suffix = self.temps[sink.id]
            if draft.suffix:
                draft.notes.append(f"{NOTE_TEMP_SUFFIX}{draft.suffix}")
        elif isinstance(sink, ast.Name) and sink.id in self.handles:
            written = _as_path(self.handles[sink.id][0])
            draft.temp_var = sink.id
            if written is None:
                draft.unresolved(f"dump target {self.handles[sink.id][1]}")
            else:
                draft.output_path = written.text
                draft.notes.append(f"{NOTE_OUTPUT_PATH}{written.text}")
                self.repo_relative = self.repo_relative or not written.absolute
        else:
            draft.unresolved(f"dump target {_unparse(sink)}")
        return True

    def _update(self, node: ast.Call, guards: tuple[str, ...]) -> bool:
        if not isinstance(node.func, ast.Attribute) or node.func.attr != "update":
            return False
        alias = self._alias(node.func.value)
        if alias is None or alias.doc not in self.drafts:
            return False
        draft = self.drafts[alias.doc]
        if not node.args or not isinstance(node.args[0], ast.Dict):
            draft.unresolved(f"update {_unparse(node)}")
            return True
        payload = node.args[0]
        for key, value in zip(payload.keys, payload.values, strict=False):
            if not isinstance(key, ast.Constant) or not isinstance(key.value, str):
                draft.unresolved(f"update key {_unparse(key) if key else '**'}")
                continue
            self._write(draft, (*alias.path, key.value), alias.resolved, value, guards)
        return True

    def _helper(self, node: ast.Call, guards: tuple[str, ...], depth: int) -> None:
        """A module function handed the document writes into it too."""
        if not node.args:
            return
        alias = self._alias(node.args[0])
        if alias is None or alias.doc not in self.drafts:
            return
        found = self.module.function(_call_name(node.func))
        if found is None:
            return
        other, function = found
        if other is not self.module:
            return
        args = list(function.args.posonlyargs) + list(function.args.args)
        if not args:
            return
        draft = self.drafts[alias.doc]
        if depth > MAX_HELPER_DEPTH:
            draft.unresolved(f"writes of {function.name} below depth {MAX_HELPER_DEPTH}")
            return
        inner = _Analyser(self.module, self.registry, self.scope, self.launch_file)
        inner.function = function.name
        inner.depth = depth
        # the helper may be handed a sub dict, and its writes are under that key path
        inner.prefix = (*self.prefix, *alias.path)
        inner.prefix_resolved = self.prefix_resolved and alias.resolved
        inner.drafts[args[0].arg] = draft
        inner.temps = self.temps
        inner.paths = self.paths
        inner._walk(function.body, guards)
        self.repo_relative = self.repo_relative or inner.repo_relative

    # ------------------------------------------------------------------
    # writes

    def _subscript_write(
        self,
        target: ast.Subscript,
        value: ast.expr,
        guards: tuple[str, ...],
    ) -> None:
        alias = self._alias(target)
        if alias is None or alias.doc not in self.drafts:
            return
        draft = self.drafts[alias.doc]
        self._write(draft, alias.path, alias.resolved, value, guards)

    def _write(
        self,
        draft: _Draft,
        key_path: tuple[str, ...],
        resolved_path: bool,
        value: ast.expr,
        guards: tuple[str, ...],
    ) -> None:
        key_path = (*self.prefix, *key_path)
        expr = _unparse(value)
        if draft.dumped:
            # the file was already written, so this write is not in it
            note = f"{NOTE_AFTER_DUMP}{dotted(key_path)} = {expr}"
            if note not in draft.notes:
                draft.notes.append(note)
            return
        folded = self._fold(value)
        folds = resolved_path and self.prefix_resolved and folded is not UNKNOWN
        if isinstance(folded, RepoPath):
            self.repo_relative = self.repo_relative or not folded.absolute
            if folds:
                draft.notes.append(f"{NOTE_PATH_VALUE}{dotted(key_path)}")
        guard = " and ".join(guards)
        locator = f"{self.function}:{dotted(key_path)}"
        if guard:
            locator = f"{locator}{GUARD_SEPARATOR}{guard}"
            expr = f"{expr}{GUARD_SEPARATOR}{guard}"
        # a guarded write is not live configuration, so `resolved` stays false and the value
        # is only what the write would put there when the guard holds
        draft.overrides.append(
            records.ParamOverride(
                key_path=key_path,
                value=plain(folded) if folds else None,
                resolved=folds and not guard,
                expr=expr,
                site=records.Site(
                    path=self.module.path,
                    locator=locator,
                    line_hint=getattr(value, "lineno", None),
                ),
            )
        )
        if guard:
            draft.notes.append(f"{NOTE_CONDITIONAL}{dotted(key_path)}{GUARD_SEPARATOR}{guard}")
        if not (resolved_path and self.prefix_resolved):
            # the key is unknown, so the write could land anywhere in the document
            draft.unresolved(f"{dotted(key_path)} = {expr}")

    def _alias(self, node: ast.expr) -> _Alias | None:
        """The document and key path an expression points into, or None."""
        if isinstance(node, ast.Name):
            if node.id in self.drafts:
                return _Alias(doc=node.id, path=())
            return self.aliases.get(node.id)
        if isinstance(node, ast.Subscript):
            parent = self._alias(node.value)
            if parent is None:
                return None
            key = self._fold(node.slice)
            if isinstance(key, str):
                return _Alias(parent.doc, (*parent.path, key), parent.resolved)
            return _Alias(parent.doc, (*parent.path, _unparse(node.slice)), False)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            if node.func.attr not in CONTAINER_CALLS or not node.args:
                return None
            parent = self._alias(node.func.value)
            if parent is None:
                return None
            key = self._fold(node.args[0])
            if isinstance(key, str):
                return _Alias(parent.doc, (*parent.path, key), parent.resolved)
            return _Alias(parent.doc, (*parent.path, _unparse(node.args[0])), False)
        return None

    def _fold(self, node: ast.expr | None) -> Any:
        return self.module.fold(node, self.scope)

    # ------------------------------------------------------------------
    # the sink

    def _trace_sink(self, function: ast.FunctionDef | ast.AsyncFunctionDef) -> None:
        """The produced path has to reach a command string or a return value."""
        for draft in self.drafts.values():
            if not draft.content():
                continue
            if not draft.temp_var:
                draft.unresolved("the document is never dumped")
                self._name_the_output(draft)
                continue
            names = sorted(name for name, temp in self.paths.items() if temp == draft.temp_var)
            if not names:
                names = [draft.temp_var]
            found = self._sink_of(function, names)
            if not found.token and self.launch_file:
                found = self._launch_sink(function, names)
            if not found.token:
                # a dump to a named path is its own answer, a temp file that reaches no
                # command string is a blind spot
                if not draft.output_path:
                    draft.unresolved(f"the temp file of {draft.var} reaches no command")
                self._name_the_output(draft)
                continue
            draft.sink = found.token
            draft.output_token = found.token
            if found.argument:
                prefix = NOTE_LAUNCH_ARG if found.launch else NOTE_SINK_ARG
                draft.notes.append(f"{prefix}{found.argument}")
            if found.target:
                draft.notes.append(f"{NOTE_SINK_TARGET}{found.target}")
            else:
                draft.unresolved(f"the command the temp file of {draft.var} feeds")

    def _name_the_output(self, draft: _Draft) -> None:
        """A document that reaches no command still gets a token, never silence.

        A dump to a named path is a real answer: the path is the token, and a launch file
        naming that path joins on it. Anything else keeps the function and the variable, which
        is stable text and not an id.
        """
        draft.output_token = draft.output_path or f"{self.function}:{draft.var}"

    def _sink_of(
        self,
        function: ast.FunctionDef | ast.AsyncFunctionDef,
        names: list[str],
    ) -> _Sink:
        fallback = _Sink()
        for node in ast.walk(function):
            if isinstance(node, ast.JoinedStr):
                found = self._sink_in_string(node, names)
                # a log line also interpolates the path, so the string that names a launch
                # argument or an argv flag wins over the first one seen
                if found.argument:
                    return found
                if found.token and not fallback.token:
                    fallback = found
            if (
                isinstance(node, ast.Return)
                and isinstance(node.value, ast.Name)
                and node.value.id in names
                and not fallback.token
            ):
                fallback = _Sink(token=node.value.id)
        return fallback

    def _launch_sink(
        self,
        function: ast.FunctionDef | ast.AsyncFunctionDef,
        names: list[str],
    ) -> _Sink:
        """The launch argument or node keyword the produced path is handed to.

        A launch file that assembles its own parameters document does not build a command
        string: the path goes into a `launch_arguments` mapping or into a `parameters=` list
        of the same file, so the invocation that receives it is the launch file itself.
        """
        target = Path(self.module.path).name
        for node in ast.walk(function):
            if isinstance(node, ast.Dict):
                for key, value in zip(node.keys, node.values, strict=False):
                    held = _held_name(value, names)
                    if held and isinstance(key, ast.Constant) and isinstance(key.value, str):
                        return _Sink(
                            token=f"{self.function}:{held}",
                            argument=key.value,
                            target=target,
                            launch=True,
                        )
            if isinstance(node, ast.keyword) and node.arg in LAUNCH_PARAM_KEYWORDS:
                items = node.value.elts if isinstance(node.value, ast.List) else []
                for item in items:
                    held = _held_name(item, names)
                    if held:
                        return _Sink(
                            token=f"{self.function}:{held}",
                            argument=node.arg,
                            target=target,
                            launch=True,
                        )
        return _Sink()

    def _sink_in_string(self, node: ast.JoinedStr, names: list[str]) -> _Sink:
        previous = ""
        head: list[str] = []
        for piece in node.values:
            if isinstance(piece, ast.Constant):
                previous = str(piece.value)
                head.append(previous)
                continue
            if isinstance(piece, ast.FormattedValue):
                expression = _unparse(piece.value)
                if expression in names:
                    return _Sink(expression, _argument_of(previous), _command_target("".join(head)))
                head.append(INTERPOLATION)
                previous = ""
        return _Sink()


_ARGUMENT = re.compile(r"([A-Za-z_][A-Za-z0-9_.]*):=\s*$")
_FLAG = re.compile(r"(--?[A-Za-z][A-Za-z0-9_-]*)[= ]\s*$")
_COMMAND = re.compile(r"\bros2\s+(?:launch|run)\s+([A-Za-z0-9_.-]+)\s+([A-Za-z0-9_.-]+)")


def _command_target(head: str) -> str:
    """The package and launch file or executable the command string names.

    `head` is the literal text in front of the interpolation, with every interpolation it
    crossed replaced by `INTERPOLATION`. Two rules follow from that. The last command that
    started before the produced file is the command that receives it, and one string can hold
    two of them. And an interpolated package or target is not a target the analyser knows, so
    the marker keeps the pattern from spanning it and half a name never binds the pipeline to
    the wrong invocation.
    """
    found = None
    for match in _COMMAND.finditer(head):
        found = match
    return f"{found.group(1)} {found.group(2)}" if found else ""


def _argument_of(previous: str) -> str:
    """The launch argument or argv flag the interpolation sits behind."""
    found = _ARGUMENT.search(previous)
    if found:
        return found.group(1)
    found = _FLAG.search(previous)
    return found.group(1) if found else ""


def _held_name(node: ast.expr, names: list[str]) -> str:
    """The tracked name an expression is, whether it was bound first or read off the handle.

    A launch file passes either the variable it stored the temp path in or `handle.name`
    straight into the action, and both are the same document.
    """
    if isinstance(node, ast.Name) and node.id in names:
        return node.id
    if isinstance(node, ast.Attribute) and node.attr == "name":
        base = node.value
        if isinstance(base, ast.Name) and base.id in names:
            return base.id
    return ""


def _as_path(value: Any) -> RepoPath | None:
    """A folded path, whether the constant was a `Path` or a plain string."""
    if isinstance(value, RepoPath):
        return value
    if isinstance(value, str) and value:
        return RepoPath(str(PurePosixPath(value)), absolute=value.startswith("/"))
    return None


def _strip_or(node: ast.expr) -> ast.expr:
    """`yaml.safe_load(f) or {}` is still a yaml load."""
    if isinstance(node, ast.BoolOp) and isinstance(node.op, ast.Or) and node.values:
        return node.values[0]
    return node


__all__ = [
    "INTERPOLATION",
    "MAX_HELPER_DEPTH",
    "NOTE_AFTER_DUMP",
    "NOTE_CONDITIONAL",
    "NOTE_ENV_UNSET",
    "NOTE_FALLBACK",
    "NOTE_LAUNCH_ARG",
    "NOTE_OUTPUT_PATH",
    "NOTE_PATH_VALUE",
    "NOTE_REPO_RELATIVE",
    "NOTE_SINK_ARG",
    "NOTE_SINK_FUNCTION",
    "NOTE_SINK_TARGET",
    "NOTE_TEMP_SUFFIX",
    "NOTE_UNRESOLVED",
    "UNKNOWN",
    "ModuleResult",
    "RepoPath",
    "analyse_file",
    "analyse_source",
    "dotted",
    "fold_constants",
    "is_merge_helper",
    "launch_argument",
    "output_path",
    "override_folded",
    "override_guard",
    "path_valued",
    "plain",
    "sink_argument",
    "sink_function",
    "sink_target",
    "temp_suffix",
]
