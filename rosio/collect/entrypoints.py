"""Profile composers that are not launch files.

No single launch file describes a running system on the reference stack. The desktop
launcher builds the real robot out of about thirty `ros2 launch` and about twenty `ros2 run`
command strings, with the arguments interpolated by f strings, and one of them names a launch
file that no longer exists. A resolver that only reads launch files therefore describes a
system that never runs, which is why this collector exists at all.

Sources it reads: shell scripts, systemd units, Makefiles and python modules that build
command strings. Launch files themselves are left to the launch collector, which executes
them and sees their `ExecuteProcess` argv properly.

Two recovery rules:

* An interpolated value is recovered when the interpolation is a literal, a module level
  literal constant, or a function parameter with a literal default. Anything else keeps its
  place in the command as `${?}` and the invocation is marked so, because a guessed argument
  value silently changes which nodes a profile contains.
* A target that does not exist on disk is recorded with a note, not skipped. The launcher
  referencing a deleted launch file is a real finding about a real robot, and a scraper that
  drops the row hides it.

The same modules also build the parameter file the profile actually runs on. `resolve.py_pipeline`
follows that out of python, and this collector binds each recovered pipeline to the entry point
argument it feeds, so `params_file` for an entry point can be a base document, an overlay and a
list of code writes rather than a repository file. A profile may also declare a pipeline by hand,
which produces the same record shape for the case where the analyser cannot see it.
"""

from __future__ import annotations

import ast
import hashlib
import shlex
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, replace
from pathlib import Path

from rosio import ids, model
from rosio.collect import CollectContext, register
from rosio.errors import UsageError
from rosio.resolve import parse_python, py_pipeline, records

LAYER = "declared"
NAME = "entrypoints"

VERBS = ("launch", "run")

UNRESOLVED = "${?}"
"""Placeholder for an interpolated value that could not be recovered."""

SHELL_SUFFIXES = (".bash", ".sh", ".zsh")
UNIT_SUFFIXES = (".service", ".socket", ".timer", ".path", ".target", ".mount")
MAKE_SUFFIXES = (".mk", ".make")
MAKE_NAMES = ("GNUmakefile", "Makefile", "makefile")
PYTHON_SUFFIXES = (".py",)

LAUNCH_SUFFIXES = (
    ".launch.py",
    "_launch.py",
    "-launch.py",
    ".launch.xml",
    ".launch.yaml",
    ".launch.yml",
    ".launch",
)

MAX_BYTES = 2 * 1024 * 1024

NO_FILE = "the workspace holds no indexed file, so no command string exists to read"
"""Why this collector has nothing to read. It is the absent inputs condition and never a
refusal: a configured root holding no file is a workspace where nothing is wrong, and a pass
recorded as refused there fires `extract.pass_incomplete` at error on it."""

_STOP_TOKENS = frozenset({"&", "&&", ";", ";;", "|", "||", ">", ">>", "<", ")", "fi", "then"})

CONFIDENCE_RESOLVED = "resolved"
CONFIDENCE_INFERRED = "inferred"
CONFIDENCE_UNKNOWN = "unknown"


# ---------------------------------------------------------------------------
# scraping


@dataclass(frozen=True)
class ScrapeResult:
    """Everything one file produced, so a caller can report per file."""

    path: str
    entrypoints: tuple[records.ScrapedEntrypoint, ...] = ()
    parse_error: str = ""


def is_scrapeable(path: str) -> bool:
    """True for a file this collector knows how to read.

    Launch files are excluded: the launch collector executes them, and reading their argv
    twice would state one invocation as two entry points.
    """
    name = Path(path).name
    if any(name.endswith(suffix) for suffix in LAUNCH_SUFFIXES):
        return False
    if name in MAKE_NAMES:
        return True
    suffix = Path(name).suffix
    return suffix in SHELL_SUFFIXES + UNIT_SUFFIXES + MAKE_SUFFIXES + PYTHON_SUFFIXES


def scrape_file(path: str, text: str | None = None, tree: ast.Module | None = None) -> ScrapeResult:
    """Read one file and return the invocations it builds, in file order. `tree` is the
    parse of a python file the caller already holds."""
    target = Path(path)
    if text is None:
        try:
            if target.stat().st_size > MAX_BYTES:
                return ScrapeResult(path=str(path), parse_error=f"larger than {MAX_BYTES} bytes")
            text = target.read_text(encoding="utf-8", errors="replace")
        except OSError as exc:
            return ScrapeResult(path=str(path), parse_error=str(exc))
    if target.suffix in PYTHON_SUFFIXES:
        return _scrape_python(str(path), text, tree)
    return ScrapeResult(path=str(path), entrypoints=tuple(_scrape_text(str(path), text)))


def as_command(entry: records.ScrapedEntrypoint) -> str:
    """The invocation written back out, which is what a config file would hold."""
    words = ["ros2", entry.verb, entry.pkg, entry.target]
    words += [f"{key}:={value}" for key, value in entry.args]
    words += list(entry.argv)
    return " ".join(words)


# ---------------------------------------------------------------------------
# plain text sources


def _scrape_text(path: str, text: str) -> list[records.ScrapedEntrypoint]:
    joined = text.replace("\\\n", " ")
    out: list[records.ScrapedEntrypoint] = []
    for offset, line in enumerate(joined.splitlines(), start=1):
        stripped = line.strip()
        if stripped.startswith("#") or "ros2" not in stripped:
            continue
        for entry in _from_words(_words(stripped)):
            out.append(
                _entrypoint(
                    entry,
                    site=records.Site(path=path, locator=_locator("", entry), line_hint=offset),
                )
            )
    return out


def _words(line: str) -> list[str]:
    out: list[str] = []
    for word in _split(line):
        # `bash -c '...'` hands a whole command line over as one quoted token.
        out.extend(_split(word) if " " in word and "ros2" in word else [word])
    return out


def _split(line: str) -> list[str]:
    try:
        return shlex.split(line, comments=True)
    except ValueError:
        return line.split()


# ---------------------------------------------------------------------------
# python sources


def _scrape_python(path: str, text: str, tree: ast.Module | None = None) -> ScrapeResult:
    if tree is None:
        try:
            tree = parse_python(text)
        except SyntaxError as exc:
            return ScrapeResult(path=path, parse_error=f"{type(exc).__name__}: {exc}")
    constants = _module_constants(tree)
    out: list[records.ScrapedEntrypoint] = []
    for scope, node in _string_nodes(tree):
        rendered, interpolated, recovered = _render(node, constants, scope)
        if "ros2" not in rendered:
            continue
        for entry in _from_words(_words(rendered)):
            out.append(
                _entrypoint(
                    entry,
                    site=records.Site(
                        path=path,
                        locator=_locator(scope.name, entry),
                        line_hint=getattr(node, "lineno", None),
                    ),
                    interpolated=interpolated,
                    recovered=recovered,
                )
            )
    return ScrapeResult(path=path, entrypoints=tuple(out))


@dataclass(frozen=True)
class _Scope:
    """The enclosing function, which supplies both the locator and the defaults."""

    name: str
    defaults: tuple[tuple[str, object], ...] = ()

    def value(self, key: str) -> object | None:
        for name, value in self.defaults:
            if name == key:
                return value
        return None


def _module_constants(tree: ast.Module) -> dict[str, object]:
    """Module level literal assignments, which is where the launcher keeps its constants."""
    out: dict[str, object] = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        value = _literal(node.value)
        if value is None:
            continue
        for target in node.targets:
            if isinstance(target, ast.Name):
                out[target.id] = value
    return out


def _string_nodes(tree: ast.Module) -> list[tuple[_Scope, ast.expr]]:
    """Every string expression, paired with the function it sits in."""
    out: list[tuple[_Scope, ast.expr]] = []
    stack: list[_Scope] = [_Scope(name="")]

    def visit(node: ast.AST) -> None:
        pushed = False
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            stack.append(_Scope(name=node.name, defaults=_defaults(node)))
            pushed = True
        if isinstance(node, ast.Expr) and isinstance(node.value, ast.Constant):
            # A bare string statement is a docstring, and prose about ros2 is not a command.
            if pushed:
                stack.pop()
            return
        if isinstance(node, ast.JoinedStr):
            # Not descending: the literal pieces are rendered with the whole f string.
            out.append((stack[-1], node))
        else:
            if isinstance(node, ast.Constant) and isinstance(node.value, str):
                out.append((stack[-1], node))
            for child in ast.iter_child_nodes(node):
                visit(child)
        if pushed:
            stack.pop()

    visit(tree)
    return out


def _defaults(node: ast.FunctionDef | ast.AsyncFunctionDef) -> tuple[tuple[str, object], ...]:
    args = list(node.args.posonlyargs) + list(node.args.args)
    out: list[tuple[str, object]] = []
    padding = len(args) - len(node.args.defaults)
    for index, arg in enumerate(args):
        if index < padding:
            continue
        value = _literal(node.args.defaults[index - padding])
        if value is not None:
            out.append((arg.arg, value))
    for arg, default in zip(node.args.kwonlyargs, node.args.kw_defaults, strict=False):
        if default is None:
            continue
        value = _literal(default)
        if value is not None:
            out.append((arg.arg, value))
    return tuple(out)


def _literal(node: ast.expr | None) -> object | None:
    if node is None:
        return None
    try:
        return ast.literal_eval(node)
    except (ValueError, TypeError, SyntaxError, MemoryError, RecursionError):
        return None


def _render(
    node: ast.expr,
    constants: dict[str, object],
    scope: _Scope,
) -> tuple[str, tuple[str, ...], tuple[str, ...]]:
    """A string node with its interpolations filled in where they are recoverable.

    Adjacent literals and f strings are already one node after parsing, so the launcher's
    multi line commands arrive whole. Returns the text, the interpolated expressions in
    order, and the subset that was recovered.
    """
    if isinstance(node, ast.Constant):
        return str(node.value), (), ()
    parts: list[str] = []
    seen: list[str] = []
    recovered: list[str] = []
    for piece in node.values:
        if isinstance(piece, ast.Constant):
            parts.append(str(piece.value))
            continue
        if not isinstance(piece, ast.FormattedValue):
            parts.append(UNRESOLVED)
            continue
        expression = _source(piece.value)
        if expression not in seen:
            seen.append(expression)
        value = _interpolated(piece.value, constants, scope)
        if value is None:
            parts.append(UNRESOLVED)
            continue
        parts.append(value)
        if expression not in recovered:
            recovered.append(expression)
    return "".join(parts), tuple(seen), tuple(recovered)


def _interpolated(node: ast.expr, constants: dict[str, object], scope: _Scope) -> str | None:
    literal = _literal(node)
    if literal is not None and not isinstance(literal, (list, dict, tuple, set)):
        return _text(literal)
    if isinstance(node, ast.Name):
        found = scope.value(node.id)
        if found is None:
            found = constants.get(node.id)
        return None if found is None else _text(found)
    return None


def _text(value: object) -> str:
    if isinstance(value, bool):
        return "True" if value else "False"
    return str(value)


def _source(node: ast.expr) -> str:
    try:
        return ast.unparse(node)
    except Exception:
        return type(node).__name__


# ---------------------------------------------------------------------------
# argv shaping


@dataclass(frozen=True)
class _Raw:
    verb: str
    pkg: str
    target: str
    args: tuple[tuple[str, str], ...]
    argv: tuple[str, ...]


def _from_words(words: Sequence[str]) -> list[_Raw]:
    """Every `ros2 launch` or `ros2 run` invocation inside one tokenised command line."""
    out: list[_Raw] = []
    index = 0
    while index < len(words):
        if not _is_ros2(words[index]) or index + 3 >= len(words):
            index += 1
            continue
        if words[index + 1] not in VERBS:
            index += 1
            continue
        verb = words[index + 1]
        pkg, pkg_tail = _split_marker(words[index + 2])
        target, target_tail = _split_marker(words[index + 3])
        if pkg.startswith("-"):
            index += 2
            continue
        args: list[tuple[str, str]] = []
        argv: list[str] = [item for item in (pkg_tail, target_tail) if item]
        cursor = index + 4
        in_argv = False
        while cursor < len(words):
            word = words[cursor]
            if word in _STOP_TOKENS or _is_ros2(word):
                break
            if word.startswith("-"):
                in_argv = True
            if not in_argv and ":=" in word:
                key, _, value = word.partition(":=")
                args.append((key, value))
            else:
                argv.append(word)
            cursor += 1
        out.append(_Raw(verb=verb, pkg=pkg, target=target, args=tuple(args), argv=tuple(argv)))
        index = cursor
    return out


def _is_ros2(word: str) -> bool:
    """True for the ros2 command however a unit file or a wrapper spelled it.

    A systemd unit writes `ExecStart=/opt/ros/humble/bin/ros2`, and a launcher glues an
    unrecovered shell prefix straight onto the command (`f"{gl_prefix}ros2 launch ..."` builds
    all three Gazebo profiles that way), so neither an exact match nor a bare basename check on
    its own finds it.
    """
    tail = word.rsplit("=", 1)[-1]
    while tail.startswith(UNRESOLVED):
        tail = tail[len(UNRESOLVED) :]
    return tail.rsplit("/", 1)[-1] == "ros2"


def _split_marker(word: str) -> tuple[str, str]:
    """Peel an unrecovered interpolation off a package or target token.

    `f"ros2 launch p tb3_simulation_launch.py{map_arg}{rviz_arg}"` glues whatever the two
    interpolations hold onto the file name. Keeping the stem separate is what lets the target
    still be checked against the workspace, while the unknown tail stays visible in argv.
    """
    if UNRESOLVED not in word:
        return word, ""
    head, _, tail = word.partition(UNRESOLVED)
    if not head:
        return word, ""
    return head, UNRESOLVED + tail


def _entrypoint(
    raw: _Raw,
    *,
    site: records.Site,
    interpolated: tuple[str, ...] = (),
    recovered: tuple[str, ...] = (),
) -> records.ScrapedEntrypoint:
    return records.ScrapedEntrypoint(
        verb=raw.verb,
        pkg=raw.pkg,
        target=raw.target,
        args=raw.args,
        argv=raw.argv,
        site=site,
        interpolated=interpolated,
        confidence=_confidence(raw, interpolated, recovered),
    )


def _confidence(
    raw: _Raw,
    interpolated: tuple[str, ...],
    recovered: tuple[str, ...],
) -> str:
    words = [raw.pkg, raw.target, *raw.argv, *(value for _key, value in raw.args)]
    if any(UNRESOLVED in word for word in words):
        return CONFIDENCE_UNKNOWN
    if any("$" in word for word in words):
        # A shell expansion the scraper cannot perform, so the text is not the final value.
        return CONFIDENCE_INFERRED
    if interpolated:
        return CONFIDENCE_INFERRED if recovered else CONFIDENCE_UNKNOWN
    return CONFIDENCE_RESOLVED


def _locator(scope: str, raw: _Raw) -> str:
    """A stable handle: the enclosing function and the invocation, never an ordinal."""
    invocation = f"ros2 {raw.verb} {raw.pkg} {raw.target}"
    return f"{scope}:{invocation}" if scope else invocation


# ---------------------------------------------------------------------------
# target resolution


def target_file(ws, entry: records.ScrapedEntrypoint) -> str | None:
    """The launch file or executable the invocation names, or None when it is absent.

    A `run` target is an executable name and is resolved through the package manifest, not
    through the filesystem, because an ament_python entry point is a console script name that
    exists nowhere as a file in the source tree.
    """
    package = ws.packages.get(entry.pkg)
    if package is None:
        return None
    if entry.verb == "run":
        return entry.target if entry.target in package.executables else None
    for base in (package.share_path, package.src_path):
        if not base:
            continue
        for candidate in (Path(base) / entry.target, Path(base) / "launch" / entry.target):
            if candidate.is_file():
                return str(candidate)
    return None


# ---------------------------------------------------------------------------
# parameter pipelines

NOTE_ENTRYPOINT = "entrypoint="
"""Written on a pipeline once it is bound to the invocation it feeds."""

PARAM_WRITE_UNRESOLVED = "param_write_unresolved"
"""The one reason code for a code write whose value the analyser could not fold. It is used
by this collector and by the parameter chain, because it is one fact about one write and two
codes for it split the same blind spot into two."""

_PIPELINE_KEYS = frozenset(
    {
        "applied_by",
        "argument",
        "base",
        "complete",
        "notes",
        "output_token",
        "overlays",
        "overrides",
        "source",
    }
)

_OVERRIDE_KEYS = frozenset({"expr", "key", "line", "locator", "resolved", "value"})


def pipelines_in(
    path: str,
    relpath: str = "",
    text: str | None = None,
    *,
    packages: Mapping[str, str] | None = None,
    launch_file: bool = False,
    tree: ast.Module | None = None,
) -> tuple[records.ParamPipeline, ...]:
    """The parameter pipelines one python module builds.

    `relpath` is the module path relative to the repository root, which is what lets a
    constant anchored at `Path(__file__).resolve().parents[n]` fold. `packages` does the same
    for a path anchored at a package share, which is how a launch file spells one.
    """
    if Path(path).suffix not in PYTHON_SUFFIXES:
        return ()
    result = py_pipeline.analyse_file(
        path, relpath, text, packages=packages, launch_file=launch_file, tree=tree
    )
    return result.pipelines


def _scan(ws, path: str) -> tuple[ScrapeResult, tuple[records.ParamPipeline, ...]]:
    """The commands one file builds and the parameter pipelines it assembles, from one parse.

    A file that does not parse is handed to each reader as before, so each states its own
    refusal in its own words.
    """
    text = tree = None
    if Path(path).suffix in PYTHON_SUFFIXES:
        try:
            text = Path(path).read_text(encoding="utf-8", errors="replace")
            tree = parse_python(text)
        except (OSError, SyntaxError, ValueError):
            text = tree = None
    relative = _repo_relative(ws, path)
    return scrape_file(path, tree=tree), pipelines_in(path, relative, text, tree=tree)


def package_dirs(ws) -> dict[str, str]:
    """Each package share directory, relative to the repository root.

    The share tree is what `get_package_share_directory` returns, so that is the anchor a
    launch file resolves its config paths against. Keeping the answer repository relative is
    what keeps the folded path out of the machine.
    """
    out: dict[str, str] = {}
    for name in ws.packages:
        share = ws.share(name)
        relative = _repo_relative(ws, share) if share else ""
        if relative:
            out[name] = relative
    return out


def pipelines_for(
    entry: records.ScrapedEntrypoint,
    pipelines: Iterable[records.ParamPipeline],
) -> tuple[records.ParamPipeline, ...]:
    """The pipelines that feed one invocation.

    Four tests, and every one of them is load bearing on the reference launcher. Both were
    built by the same function. The command the pipeline names is this invocation, which is
    the only thing that tells two documents apart when one module builds both and hands both
    to an argument called `params_file`. The command interpolates the expression the pipeline
    produced. And the argument the pipeline writes into is one the scraper could not recover,
    which is exactly what an interpolated temp path looks like from the command side.
    """
    scope = _scope_of(entry)
    out = []
    for pipeline in pipelines:
        if not scope or _scope_of_pipeline(pipeline) != scope:
            continue
        if not _targets(entry, pipeline):
            continue
        if entry.interpolated and pipeline.output_token not in entry.interpolated:
            continue
        if _feeds(entry, py_pipeline.sink_argument(pipeline)):
            out.append(pipeline)
    return tuple(out)


def _targets(entry: records.ScrapedEntrypoint, pipeline: records.ParamPipeline) -> bool:
    """Whether the command the pipeline was built for is this invocation.

    A pipeline that names no command is not refused, because a hand declared one has no
    command string to name. It binds on the argument alone, as it did before.
    """
    target = py_pipeline.sink_target(pipeline)
    return not target or target == f"{entry.pkg} {entry.target}"


def attach(
    entries: Sequence[records.ScrapedEntrypoint],
    pipelines: Sequence[records.ParamPipeline],
) -> tuple[records.ParamPipeline, ...]:
    """Bind each pipeline to the invocation it feeds, recorded on the pipeline notes."""
    out: list[records.ParamPipeline] = []
    for pipeline in pipelines:
        bound = [entry for entry in entries if pipeline in pipelines_for(entry, [pipeline])]
        notes = set(pipeline.notes)
        notes.update(f"{NOTE_ENTRYPOINT}{as_command(entry)}" for entry in bound)
        out.append(replace(pipeline, notes=tuple(sorted(notes))))
    return tuple(out)


def entrypoint_of(pipeline: records.ParamPipeline) -> str:
    """The invocation a pipeline was bound to, empty when it was bound to none."""
    for note in pipeline.notes:
        if note.startswith(NOTE_ENTRYPOINT):
            return note[len(NOTE_ENTRYPOINT) :]
    return ""


def _scope_of(entry: records.ScrapedEntrypoint) -> str:
    locator = entry.site.locator if entry.site else ""
    scope, separator, _rest = locator.partition(":")
    return scope if separator else ""


def _scope_of_pipeline(pipeline: records.ParamPipeline) -> str:
    locator = pipeline.site.locator if pipeline.site else ""
    scope, separator, _rest = locator.partition(":")
    if separator:
        return scope
    return pipeline.applied_by.rsplit(".", 1)[-1]


def _feeds(entry: records.ScrapedEntrypoint, argument: str) -> bool:
    if not argument:
        return False
    for key, value in entry.args:
        if key == argument and UNRESOLVED in value:
            return True
    words = list(entry.argv)
    for index, word in enumerate(words):
        if word == argument and index + 1 < len(words) and UNRESOLVED in words[index + 1]:
            return True
        if word.startswith(f"{argument}=") and UNRESOLVED in word:
            return True
    return False


def pipeline_from_mapping(payload: object, source_root: str = "") -> records.ParamPipeline:
    """The config escape hatch: a pipeline declared by hand in `rosio.yaml`.

    The analyser cannot read every launcher ever written, and a wrong answer is worse than a
    missing one, so a user can state the same thing and get the same record shape.
    """
    if not isinstance(payload, dict):
        raise UsageError("a parameter pipeline must be a mapping")
    unknown = sorted(set(payload) - _PIPELINE_KEYS)
    if unknown:
        raise UsageError(f"unknown pipeline keys: {', '.join(unknown)}")
    argument = str(payload.get("argument", "")).strip()
    token = str(payload.get("output_token", "") or argument).strip()
    if not token:
        raise UsageError("a pipeline needs an argument or an output_token")
    source = str(payload.get("source", "")).strip()
    site = None
    if source:
        site = records.Site(
            path=_under_root(source, source_root), locator=str(payload.get("applied_by", ""))
        )
    notes = [str(note) for note in payload.get("notes") or ()]
    if argument:
        notes.append(f"{py_pipeline.NOTE_SINK_ARG}{argument}")
    overrides = tuple(_override_from_mapping(item, site) for item in payload.get("overrides") or ())
    complete = bool(payload.get("complete", True)) and all(item.resolved for item in overrides)
    return records.ParamPipeline(
        output_token=token,
        base=_optional(payload.get("base"), source_root),
        overlays=tuple(
            _under_root(str(item), source_root) for item in payload.get("overlays") or ()
        ),
        overrides=overrides,
        applied_by=str(payload.get("applied_by", "")),
        site=site,
        complete=complete,
        notes=tuple(sorted(set(notes))),
    )


def declared_pipelines(profile: object, source_root: str = "") -> tuple[records.ParamPipeline, ...]:
    """Pipelines a profile declared by hand. An already built record is passed through."""
    out: list[records.ParamPipeline] = []
    for item in getattr(profile, "pipelines", ()) or ():
        if isinstance(item, records.ParamPipeline):
            out.append(item)
            continue
        out.append(pipeline_from_mapping(item, source_root))
    return tuple(out)


def _override_from_mapping(payload: object, site: records.Site | None) -> records.ParamOverride:
    if not isinstance(payload, dict):
        raise UsageError("a pipeline override must be a mapping")
    unknown = sorted(set(payload) - _OVERRIDE_KEYS)
    if unknown:
        raise UsageError(f"unknown pipeline override keys: {', '.join(unknown)}")
    key = str(payload.get("key", "")).strip()
    if not key:
        raise UsageError("a pipeline override needs a key")
    resolved = bool(payload.get("resolved", "value" in payload))
    locator = str(payload.get("locator", "") or key)
    where = None
    if site is not None:
        line = payload.get("line")
        where = records.Site(
            path=site.path,
            locator=locator,
            line_hint=int(line) if isinstance(line, int) else None,
            root=site.root,
        )
    return records.ParamOverride(
        key_path=tuple(key.split(".")),
        value=payload.get("value") if resolved else None,
        resolved=resolved,
        expr=str(payload.get("expr", "") or repr(payload.get("value"))),
        site=where,
    )


def _optional(value: object, source_root: str) -> str | None:
    return None if value in (None, "") else _under_root(str(value), source_root)


def _under_root(path: str, source_root: str) -> str:
    if not source_root or path.startswith("/"):
        return path
    return str(Path(source_root) / path)


def pipeline_row(pipeline: records.ParamPipeline, repo_root: str = "") -> dict[str, object]:
    """One pipeline as fact values. Provenance terminates at the symbol, never at a temp path.

    Every path is repository relative, because an absolute one is a machine fact and two
    boxes must not differ on it. The launcher writes the absolute form into the document
    though, so the absolute rendering sits beside it under `volatile`, which is dropped from
    the compared bytes but is there for a reader holding an observed value.
    """
    row: dict[str, object] = {
        "output_token": pipeline.output_token,
        "applied_by": pipeline.applied_by,
        "function": py_pipeline.sink_function(pipeline),
        "complete": pipeline.complete,
        "overrides": [_override_row(override) for override in pipeline.overrides],
    }
    if pipeline.base:
        row["base"] = pipeline.base
    if pipeline.overlays:
        row["overlays"] = list(pipeline.overlays)
    sink: dict[str, object] = {}
    for key, value in (
        ("argument", py_pipeline.sink_argument(pipeline)),
        ("target", py_pipeline.sink_target(pipeline)),
        ("output_path", py_pipeline.output_path(pipeline)),
    ):
        if value:
            sink[key] = value
    if sink:
        row["sink"] = sink
    argument = py_pipeline.sink_argument(pipeline)
    if argument:
        row["argument"] = argument
    if pipeline.notes:
        row["notes"] = list(pipeline.notes)
    absolute = _absolute_forms(pipeline, repo_root)
    if absolute:
        row["volatile"] = absolute
    return row


def _override_row(override: records.ParamOverride) -> dict[str, object]:
    """One python write.

    A guarded write is never stated as a value: `resolved` is false and `value` is absent, so
    a consumer that stops after those two cannot read it as live configuration. What it would
    write, and when, are on their own keys.
    """
    guard = py_pipeline.override_guard(override)
    row: dict[str, object] = {
        "key": py_pipeline.dotted(override.key_path),
        "resolved": override.resolved,
        "expr": override.expr,
        "guard": guard,
        "conditional": bool(guard),
    }
    if guard:
        row["value"] = None
        row["value_when_guard_holds"] = override.value
    else:
        row["value"] = override.value
    return row


def _absolute_forms(pipeline: records.ParamPipeline, repo_root: str) -> dict[str, object]:
    """What the running node sees, for the paths that the launcher writes out absolute.

    Repository relative is the form an editor can act on and the form two machines agree
    about. The launcher hands launch an absolute path, so a later comparison against an
    observed value needs both or it reads as drift that is not there.
    """
    if not repo_root:
        return {}
    keys = set(py_pipeline.path_valued(pipeline))
    values = {
        py_pipeline.dotted(override.key_path): _absolute(str(override.value), repo_root)
        for override in pipeline.overrides
        if py_pipeline.dotted(override.key_path) in keys and isinstance(override.value, str)
    }
    out: dict[str, object] = {"note": "absolute forms, machine specific, never compared"}
    if pipeline.base:
        out["base_absolute"] = _absolute(pipeline.base, repo_root)
    if pipeline.overlays:
        out["overlays_absolute"] = [_absolute(item, repo_root) for item in pipeline.overlays]
    if values:
        out["override_values_absolute"] = values
    return out if len(out) > 1 else {}


def _absolute(path: str, repo_root: str) -> str:
    return path if path.startswith("/") else str(Path(repo_root) / path)


# ---------------------------------------------------------------------------
# the collector


class EntrypointsCollector:
    """Declared layer collector for the non launch composers."""

    name = NAME
    layer = LAYER

    def inputs_present(self, ctx: CollectContext) -> bool | str:
        """Whether there is any file here to read a command string out of.

        The orchestrator asks this before it attempts the pass, so a workspace with no file
        is recorded as not attempted with this sentence rather than as a pass that stopped
        with data still to read.
        """
        return True if ctx.ws.files() else NO_FILE

    def collect(self, ctx: CollectContext) -> None:
        ws = ctx.ws
        builder = ctx.builder
        if not ws.files():
            return  # nothing to read, which `inputs_present` states to the orchestrator
        sources = [path for path in ws.files() if is_scrapeable(path)]
        scanned = [_scan(ws, path) for path in sources]
        results = [result for result, _built in scanned]
        inside = self._inside_launch(ws)
        run = builder.add_run(
            LAYER,
            profile=ids.build_profile(ctx.profile) if ctx.profile else None,
            input_digest=_digest(ws, [*results, *(result for result, _built in inside)]),
            roots=ws.root_hashes(),
        )
        builder.declare_layer(LAYER, present=True, run=run)
        found: list[records.ScrapedEntrypoint] = []
        pipelines: list[records.ParamPipeline] = list(_from_config(ctx))
        for result, built in inside:
            pipelines.extend(built)
            self._record(ctx, run, result, (), built)
            self._blind_spots(ctx, result, built)
        for result, read in scanned:
            entries = _dedup(result.entrypoints)
            # a launcher split over two modules keeps the document in one and the command in
            # the other, so a module with no command string still has to be read
            built = attach(entries, read)
            built = tuple(
                pipeline
                for pipeline in built
                if entrypoint_of(pipeline)
                or py_pipeline.sink_argument(pipeline)
                or py_pipeline.output_path(pipeline)
                or any("ros__parameters" in item.key_path for item in pipeline.overrides)
            )
            if not entries and not built and not result.parse_error:
                continue
            found.extend(entries)
            pipelines.extend(built)
            self._record(ctx, run, result, entries, built)
            self._blind_spots(ctx, result, built)
        ctx.declared.scraped.extend(found)
        ctx.declared.pipelines.extend(pipelines)

    def _inside_launch(self, ws) -> list[tuple[ScrapeResult, tuple[records.ParamPipeline, ...]]]:
        """The parameter documents launch files assemble for themselves.

        A launch file that loads a base yaml, merges an overlay, writes keys and dumps a temp
        file is the same pipeline the desktop launcher runs, and until it is decomposed the
        chain names the launch file instead of the overlay line an engineer would edit. No
        invocation is scraped here: the launch collector executes these files and reading
        their argv twice would state one invocation as two entry points.
        """
        packages = package_dirs(ws)
        out: list[tuple[ScrapeResult, tuple[records.ParamPipeline, ...]]] = []
        for path in ws.launch_files():
            if Path(path).suffix not in PYTHON_SUFFIXES:
                continue
            built = pipelines_in(
                path,
                _repo_relative(ws, path),
                packages=packages,
                launch_file=True,
            )
            if built:
                out.append((ScrapeResult(path=path), built))
        return out

    def _record(
        self,
        ctx: CollectContext,
        run: str,
        result: ScrapeResult,
        entries: Sequence[records.ScrapedEntrypoint],
        pipelines: Sequence[records.ParamPipeline] = (),
    ) -> None:
        ws = ctx.ws
        builder = ctx.builder
        package, _relative = ws.rel(result.path)
        file_id = ws.file_id(result.path)
        if package:
            pkg_id = ids.build_pkg(package)
            builder.node(pkg_id, label=package)
            builder.node(file_id)
            builder.edge("contains", pkg_id, file_id, layer=LAYER, run=run)
        notes: list[str] = []
        rows: list[dict[str, object]] = []
        for entry in entries:
            row: dict[str, object] = {
                "verb": entry.verb,
                "pkg": entry.pkg,
                "target": entry.target,
                "args": [list(pair) for pair in entry.args],
                "argv": list(entry.argv),
                "confidence": entry.confidence,
                "interpolated": list(entry.interpolated),
                "command": as_command(entry),
            }
            resolved = target_file(ws, entry)
            if entry.verb == "launch":
                if resolved:
                    row["target_file"] = ws.file_id(resolved)
                elif entry.pkg in ws.packages:
                    row["target_missing"] = True
                    notes.append(
                        f"{as_command(entry)} names a launch file that is not in "
                        f"package {entry.pkg}"
                    )
                else:
                    row["target_unknown_package"] = True
            elif resolved:
                row["executable"] = ids.build_exe(entry.pkg, entry.target)
            elif entry.pkg in ws.packages:
                row["target_missing"] = True
                notes.append(
                    f"{as_command(entry)} names an executable package {entry.pkg} does not export"
                )
            else:
                row["target_unknown_package"] = True
            fed = pipelines_for(entry, pipelines)
            if fed:
                row["pipelines"] = [pipeline_row(item, ws.repo_root) for item in fed]
            rows.append(row)
        values: dict[str, object] = {"entrypoints": rows}
        loose = [pipeline for pipeline in pipelines if not entrypoint_of(pipeline)]
        if loose:
            values["pipelines"] = [pipeline_row(item, ws.repo_root) for item in loose]
        if notes:
            values["notes"] = sorted(set(notes))
        if result.parse_error:
            values["parse_error"] = result.parse_error
        builder.fact(
            file_id,
            LAYER,
            profile=_profile_id(ctx),
            run=run,
            values=values,
            confidence=model.Confidence.RESOLVED.value,
            evidence=[ws.source(records.Site(path=result.path, root=ws.root_of(result.path)))],
        )
        if notes:
            builder.node(file_id, attrs={"notes": sorted(set(notes))})

    def _blind_spots(
        self,
        ctx: CollectContext,
        result: ScrapeResult,
        pipelines: Sequence[records.ParamPipeline],
    ) -> None:
        """What the analyser could not follow, and what it could.

        The resolution rate is the CI gate surface, so a pipeline that stopped short has to
        arrive as an unresolved entry with its raw expression, and the writes that did fold
        have to arrive as a denominator.

        A guarded write whose value folded is resolved. The guard is a runtime dictionary
        lookup, so no amount of static reading will ever settle it, and counting it against
        the rate would put a permanent unclearable debit on the gate and reward guessing the
        branch. The guard stays on the record and on the candidate, where it belongs: the
        write is not live configuration and the chain says so.
        """
        builder = ctx.builder
        file_id = ctx.ws.file_id(result.path)
        source = ctx.ws.source(records.Site(path=result.path, root=ctx.ws.root_of(result.path)))
        for pipeline in pipelines:
            resolved = 0
            for override in pipeline.overrides:
                if py_pipeline.override_folded(override):
                    resolved += 1
                    continue
                key = py_pipeline.dotted(override.key_path)
                builder.unresolved(
                    model.Unresolved(
                        want=model.Want.PARAM_VALUE.value,
                        owner=file_id,
                        expr=f"{pipeline.output_token}: {key} = {override.expr}",
                        reason=PARAM_WRITE_UNRESOLVED,
                        resolvable_by=[pipeline.applied_by] if pipeline.applied_by else [],
                        evidence=[source],
                    )
                )
            if pipeline.base:
                resolved += 1
            resolved += len(pipeline.overlays)
            if resolved:
                builder.count_resolved(model.Want.PARAM_VALUE.value, resolved)
            for note in pipeline.notes:
                if not note.startswith(py_pipeline.NOTE_UNRESOLVED):
                    continue
                builder.unresolved(
                    model.Unresolved(
                        want=model.Want.PARAM_VALUE.value,
                        owner=file_id,
                        expr=f"{pipeline.output_token}: {note[len(py_pipeline.NOTE_UNRESOLVED) :]}",
                        reason="pipeline_incomplete",
                        resolvable_by=[pipeline.applied_by] if pipeline.applied_by else [],
                        evidence=[source],
                    )
                )


def _repo_relative(ws, path: str) -> str:
    """The module path relative to the repository root, which is the anchor a launcher
    constant is written against. Empty when the file sits outside every root."""
    try:
        return str(Path(path).resolve().relative_to(Path(ws.repo_root).resolve()))
    except ValueError:
        return ""


def _from_config(ctx: CollectContext) -> tuple[records.ParamPipeline, ...]:
    """Pipelines the profile in play declared by hand."""
    for profile in ctx.cfg.profiles:
        if profile.slug == ctx.profile:
            return declared_pipelines(profile, ctx.ws.repo_root)
    return ()


def _profile_id(ctx: CollectContext) -> str:
    """A fact slot takes a profile id, never a bare slug, and `*` when nothing is in play."""
    return ids.build_profile(ctx.profile) if ctx.profile else model.ANY_PROFILE


def _dedup(entries: Iterable[records.ScrapedEntrypoint]) -> tuple[records.ScrapedEntrypoint, ...]:
    """One row per distinct invocation and locator. The line hint never distinguishes two."""
    out: list[records.ScrapedEntrypoint] = []
    seen: set[tuple[str, ...]] = set()
    for entry in entries:
        key = (
            entry.verb,
            entry.pkg,
            entry.target,
            repr(entry.args),
            repr(entry.argv),
            entry.site.locator if entry.site else "",
        )
        if key in seen:
            continue
        seen.add(key)
        out.append(entry)
    return tuple(out)


def _digest(ws, results: Sequence[ScrapeResult]) -> str:
    """Over the files read and the commands recovered, never over a clock or a path order."""
    sink = hashlib.sha256()
    rows = []
    for result in results:
        _package, relative = ws.rel(result.path)
        rows.append((relative, tuple(as_command(entry) for entry in result.entrypoints)))
    for relative, commands in sorted(rows):
        sink.update(relative.encode("utf-8"))
        for command in commands:
            sink.update(b"\x00")
            sink.update(command.encode("utf-8"))
        sink.update(b"\n")
    return sink.hexdigest()[: ids.DIGEST_LENGTH]


def _factory() -> EntrypointsCollector:
    return EntrypointsCollector()


register(NAME, _factory)


__all__ = [
    "LAYER",
    "NAME",
    "NOTE_ENTRYPOINT",
    "NO_FILE",
    "PARAM_WRITE_UNRESOLVED",
    "UNRESOLVED",
    "EntrypointsCollector",
    "ScrapeResult",
    "as_command",
    "attach",
    "declared_pipelines",
    "entrypoint_of",
    "is_scrapeable",
    "package_dirs",
    "pipeline_from_mapping",
    "pipeline_row",
    "pipelines_for",
    "pipelines_in",
    "scrape_file",
    "target_file",
]
