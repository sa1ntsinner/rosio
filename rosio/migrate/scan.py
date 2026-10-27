"""Find what a move between two releases changes in a workspace's own packages.

Each matcher reads one kind of file the way that file is written: package.xml as XML tags,
CMake as command invocations, C and C++ with comments and string literals blanked out,
python through its own parser, parameter yaml through its marked key paths.
A regular expression over raw text would match a commented
out `ament_target_dependencies` or a header named in a log message, and an edit there is
worse than no edit.

A change is either a rewrite, with the exact span and the new text, or an instruction. A
rewrite is only produced where the new form is known to build: the header it writes is
measured to exist in the target, the targets it links are exported by workspace packages.
"""

from __future__ import annotations

import ast
import json
import re
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field
from functools import cache
from itertools import pairwise
from pathlib import Path
from xml.parsers import expat

from rosio import ids
from rosio.errors import CollectorError, ContractError
from rosio.migrate import cmake, distros
from rosio.migrate.rules import Catalog, Rule, catalog
from rosio.model import dumps, dumps_line
from rosio.resolve import parse_python, yaml_lines

CPP_SUFFIXES = (".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx", ".ipp", ".tpp")
DEPEND_TAGS = (
    "depend",
    "build_depend",
    "build_export_depend",
    "buildtool_depend",
    "buildtool_export_depend",
    "exec_depend",
    "test_depend",
    "doc_depend",
)
_XML_OPEN = re.compile(r"""<(?:[^>"']|"[^"]*"|'[^']*')*>""")
_INCLUDE = re.compile(r'^[ \t]*#[ \t]*include[ \t]*[<"](?P<path>[^>"\n]+)[>"]', re.M)
TEMPLATE_VERSION = "3.8"
"""What `ros2 pkg create` writes into cmake_minimum_required on every supported release."""


@dataclass(frozen=True)
class SourcePackage:
    name: str
    path: Path
    files: tuple[Path, ...]
    depends: frozenset[str] = frozenset()
    exports_targets: bool = False
    """Whether the package's CMake exports `<name>_TARGETS` for its dependents."""


@dataclass(frozen=True)
class Change:
    rule: str
    package: str
    file: str
    """Relative to the workspace root, with forward slashes."""
    line: int
    column: int
    found: str
    title: str
    """What changed, in a few words, for a list. `say` is the sentence to act on."""
    effect: str
    """removed, deprecated or behaviour (an unset parameter default changes)."""
    release: str
    say: str
    source: str
    replacement: str | None = None
    """The text that replaces `found` when the change is a rewrite."""
    works_from: str | None = None
    span: tuple[int, int] | None = field(default=None, compare=False)
    """Offsets of `found` in the file, for the editor that applies the rewrite."""

    @property
    def automatic(self) -> bool:
        return self.replacement is not None

    def to_json(self) -> dict:
        out = {
            "rule": self.rule,
            "package": self.package,
            "file": self.file,
            "line": self.line,
            "column": self.column,
            "found": self.found,
            "title": self.title,
            "effect": self.effect,
            "release": self.release,
            "say": self.say,
            "source": self.source,
        }
        if self.replacement is not None:
            out["replacement"] = self.replacement
        if self.works_from:
            out["works_from"] = self.works_from
        return out


@dataclass
class _Context:
    start: str
    target: str
    root: Path
    table: Catalog
    workspace: dict[str, SourcePackage]


def scan(packages: Sequence[SourcePackage], start: str, target: str, root: Path) -> list[Change]:
    """Every change the move asks of these packages, in file order."""
    if distros.index(start) >= distros.index(target):
        from rosio.errors import UsageError

        raise UsageError(f"{start} to {target} is not an upgrade")
    context = _Context(start, target, root, catalog(), {p.name: p for p in packages})
    changes: list[Change] = []
    for package in packages:
        for path in package.files:
            changes.extend(_file(context, package, path))
    return sorted(changes, key=lambda c: (c.package, c.file, c.line, c.column, c.rule))


def _file(context: _Context, package: SourcePackage, path: Path) -> Iterable[Change]:
    name = path.name
    if not (
        name == "CMakeLists.txt"
        or name.endswith((".cmake", ".py", *CPP_SUFFIXES))
        or path.suffix.lower() in {".xml", ".yaml", ".yml"}
    ):
        return ()
    if contained_path(context.root, path) is None:
        return (_refused(context, package, path, "path resolves outside the workspace"),)
    try:
        # Newlines as stored: offsets must hold for the bytes `apply` rewrites.
        with path.open(encoding="utf-8", newline="") as handle:
            text = handle.read()
    except UnicodeDecodeError:
        return (_refused(context, package, path, "file is not UTF-8"),)
    except OSError:
        return (_refused(context, package, path, "file could not be read"),)
    if name == "package.xml":
        return _manifest(context, package, path, text)
    if name == "CMakeLists.txt" or name.endswith(".cmake"):
        return _cmake(context, package, path, text)
    if name.endswith(CPP_SUFFIXES):
        return _cpp(context, package, path, text)
    if name.endswith(".py"):
        return _python(context, package, path, text)
    if path.suffix.lower() in (".yaml", ".yml"):
        return _params_yaml(context, package, path, text)
    if path.suffix.lower() == ".xml":
        return _bt_xml(context, package, path, text)
    return ()


def contained_path(root: Path, path: Path) -> Path | None:
    try:
        resolved = path.resolve()
        return resolved if resolved.is_relative_to(root.resolve()) else None
    except (OSError, RuntimeError):
        return None


def _refused(context, package, path, reason) -> Change:
    return Change(
        rule="input.refused",
        package=package.name,
        file=path.relative_to(context.root).as_posix(),
        line=1,
        column=1,
        found="",
        title="Input not scanned",
        effect="refused",
        release=context.target,
        say=f"{reason}; inspect this input before migrating",
        source="https://docs.python.org/3/library/pathlib.html#pathlib.Path.open",
    )


def _bt_xml(context, package, path, text) -> Iterable[Change]:
    rules = [
        rule
        for rule in context.table.rules
        if rule.matcher == "bt_xml"
        and set(rule.find["package_depends"]) & package.depends
        and rule.effect(context.start, context.target)
    ]
    if not rules:
        return
    parser = expat.ParserCreate()
    raw = text.encode("utf-8")
    sites = []
    seen = set()

    def site(name, attrs=None):
        offset = parser.CurrentByteIndex
        closing = attrs is None
        prefix = "</" if closing else "<"
        token = prefix.encode() + re.escape(name.encode("utf-8"))
        token += rb"\s*>" if closing else rb"(?=[\s/>])"
        if not re.match(token, raw[offset:]):
            return
        start = len(raw[:offset].decode("utf-8")) + len(prefix)
        span = (start, start + len(name))
        if span not in seen:
            seen.add(span)
            sites.append((name, attrs, span))

    parser.StartElementHandler = site
    parser.EndElementHandler = site
    try:
        parser.Parse(raw, True)
    except expat.ExpatError:
        return
    if not sites or sites[0][0] != "root":
        return
    root_attrs = sites[0][1]
    if not {"BTCPP_format", "main_tree_to_execute"} & root_attrs.keys():
        return
    for name, attrs, span in sites:
        for rule in rules:
            wanted = rule.find["bt_xml"]
            if wanted == "format":
                if name != "root" or attrs is None or attrs.get("BTCPP_format") == "4":
                    continue
            elif name != wanted or (attrs is None and not rule.fix):
                continue
            fix = rule.fix.get("rename") if isinstance(rule.fix, dict) else None
            yield _rule_change(
                context,
                package,
                path,
                text,
                span,
                rule,
                rule.effect(context.start, context.target),
                replacement=fix,
            )


@cache
def _defaults() -> dict:
    path = Path(__file__).with_name("data") / "defaults.json"
    return json.loads(path.read_text(encoding="utf-8"))


def _params_yaml(context, package, path, text) -> Iterable[Change]:
    """Parameter files against the measured node defaults of `defaults.json`.

    Only a block whose selector ends in the node name is read: a `/**` block says nothing
    about which nodes a workspace runs. A configured key the target no longer declares is
    ignored by the node, and an unset key whose default changed changes what the node does.
    """
    facts = _defaults()
    checked = facts["checked"]
    if context.start not in checked or context.target not in checked:
        return
    try:
        doc = yaml_lines.load(text)
    except CollectorError:
        return
    blocks = []
    for block in yaml_lines.param_blocks(doc):
        try:
            blocks.append((block, {leaf.name: leaf for leaf in yaml_lines.leaves(block.values)}))
        except ContractError:
            continue
    rules = {r.find["params_yaml"]: r for r in context.table.rules if r.matcher == "params_yaml"}
    lines = text.splitlines(keepends=True)
    for block, leaves in blocks:
        node = block.selector.rstrip("/").rsplit("/", 1)[-1]
        for entry in facts["parameters"]:
            if entry["node"] != node:
                continue
            name, releases = entry["name"], entry["releases"]
            replacement = cited = None
            if name in leaves:
                if not releases.get(context.start, {}).get("declared"):
                    continue
                since = None
                crossed = [r for r in checked if distros.crossed(r, context.start, context.target)]
                for release in reversed(crossed):
                    if releases.get(release, {}).get("declared", True):
                        break
                    since = release
                if since is None:
                    continue
                kind, key_path = "not_declared", (*block.key_path, *leaves[name].key_path)
                said = releases[since]
                say = f"{node} stops declaring {name} in {since}, so this value is ignored"
                successor = context.table.parameters.get(f"{node}.{name}")
                if successor and context.target in successor.checked:
                    replacement, cited = successor.use, successor.source
                    instead = f"reads {successor.use} instead"
                    say = (
                        f"{node} stops declaring {name} in {since} and {instead}"
                        if successor.use
                        else f"{say}; {successor.say}"
                    )
            else:
                if "*" in block.selector:
                    continue
                if any(
                    name in other
                    and ids.param_key_matches(
                        other_block.selector, "/" + block.selector.lstrip("/")
                    )
                    for other_block, other in blocks
                ):
                    continue
                changed = [
                    (after, releases[before], releases[after])
                    for before, after in pairwise(checked)
                    if distros.crossed(after, context.start, context.target)
                    and _known_default(releases.get(before))
                    and _known_default(releases.get(after))
                    and dumps(releases[before]["default"]) != dumps(releases[after]["default"])
                ]
                if not changed:
                    continue
                since, old, said = changed[0]
                kind, key_path = "default_changed", block.key_path[:-1]
                was, now = dumps_line(old["default"]), dumps_line(said["default"])
                say = (
                    f"{node}.{name} defaults to {now} instead of {was} from {since}; "
                    f"set {name}: {was} to keep today's behaviour"
                )
            rule = rules.get(kind)
            line = doc.line(key_path)
            if rule is None or line is None:
                continue
            key = str(key_path[-1])
            start = sum(len(row) for row in lines[: line - 1]) + lines[line - 1].find(key)
            where = said["source"]
            yield _change(
                context,
                package,
                path,
                text,
                (start, start + len(key)),
                rule=rule.id,
                title=f"{node}.{name}",
                effect="behaviour",
                release=since,
                say=say,
                source=cited
                or f"{facts['repository']}/blob/{where['branch']}/{where['path']}#L{where['line']}",
                replacement=replacement,
            )


def _known_default(value) -> bool:
    return bool(value and value["declared"] and value.get("default_state") == "known")


def _change(
    context: _Context,
    package: SourcePackage,
    path: Path,
    text: str,
    span: tuple[int, int],
    **fields,
) -> Change:
    start = span[0]
    line = text.count("\n", 0, start) + 1
    column = start - (text.rfind("\n", 0, start) + 1) + 1
    return Change(
        package=package.name,
        file=path.relative_to(context.root).as_posix(),
        line=line,
        column=column,
        found=text[span[0] : span[1]],
        span=span,
        **fields,
    )


def _rule_change(context, package, path, text, span, rule: Rule, effect, **fields) -> Change:
    kind, release = effect
    return _change(
        context,
        package,
        path,
        text,
        span,
        rule=rule.id,
        effect=kind,
        release=release,
        title=fields.pop("title", rule.title),
        say=fields.pop("say", rule.say),
        source=rule.source,
        works_from=fields.pop("works_from", rule.works_from),
        **fields,
    )


# ---------------------------------------------------------------------------
# dependencies


def _unreleased(context: _Context, dependency: str) -> str | None:
    """The release from which a dependency the start ships stays unreleased up to the target.

    Only the target decides. A package skipped by a short lived release in between (many
    were never released for iron) is back in the next one, and the move does not care.
    """
    if dependency in context.workspace or not distros.released(dependency, context.start):
        return None
    return _gone_since(context, lambda release: distros.released(dependency, release) is False)


def _gone_since(context: _Context, missing) -> str | None:
    """The first release of the trailing run of releases where `missing` holds, ending at
    the target. None when the target itself does not miss it."""
    gone = None
    for release in reversed(distros.between(context.start, context.target)):
        if not missing(release):
            break
        gone = release
    return gone


def _dependency_change(context, package, path, text, span, dependency: str, release: str):
    successor = context.table.packages.get(dependency)
    say = f"{dependency} is not released for {release}"
    if successor:
        say = f"{say}; {successor.say}"
    return _change(
        context,
        package,
        path,
        text,
        span,
        rule="dependency.unreleased",
        title=dependency,
        effect="removed",
        release=release,
        say=say,
        source=successor.source if successor else f"https://index.ros.org/p/{dependency}/",
    )


def _manifest(context, package, path, text) -> Iterable[Change]:
    for name, span in _dependency_sites(text):
        release = _unreleased(context, name)
        if release:
            yield _dependency_change(context, package, path, text, span, name, release)


def package_depends(text: str) -> frozenset[str]:
    return frozenset(name for name, _span in _dependency_sites(text))


def _dependency_sites(text: str) -> list[tuple[str, tuple[int, int]]]:
    parser = expat.ParserCreate()
    raw = text.encode("utf-8")
    stack = []
    sites = []

    def start(name, _attrs):
        body_start = None
        if len(stack) == 1 and stack[0][0] == "package" and name in DEPEND_TAGS:
            offset = len(raw[: parser.CurrentByteIndex].decode("utf-8"))
            match = _XML_OPEN.match(text, offset)
            if match:
                body_start = match.end()
        stack.append((name, body_start, []))

    def characters(value):
        if len(stack) == 2 and stack[-1][1] is not None:
            stack[-1][2].append(value)

    def end(_name):
        _tag, body_start, chunks = stack.pop()
        if len(stack) != 1 or body_start is None:
            return
        name = "".join(chunks).strip()
        if not re.fullmatch(r"[A-Za-z0-9_.-]+", name):
            return
        close = len(raw[: parser.CurrentByteIndex].decode("utf-8"))
        body = text[body_start:close]
        start = body_start + len(body) - len(body.lstrip())
        sites.append((name, (start, body_start + len(body.rstrip()))))

    parser.StartElementHandler = start
    parser.CharacterDataHandler = characters
    parser.EndElementHandler = end
    try:
        parser.Parse(raw, True)
    except expat.ExpatError:
        return []
    return sites


# ---------------------------------------------------------------------------
# cmake


def _cmake(context, package, path, text) -> Iterable[Change]:
    found = cmake.commands(text)
    lists = _inherited_lists(package, path)
    lists.update(cmake.literal_lists(found))
    for command in found:
        if command.lower == "find_package" and command.args:
            name = command.args[0].text
            release = _unreleased(context, name) if distros.is_ros_package(name) else None
            if release:
                arg = command.args[0]
                yield _dependency_change(
                    context, package, path, text, (arg.start, arg.end), name, release
                )
        for rule in context.table.rules:
            if rule.matcher != "cmake_call" or rule.find["cmake_call"] != command.lower:
                continue
            effect = rule.effect(context.start, context.target)
            if effect is None:
                continue
            if rule.fix == "target_link_libraries":
                yield _target_link_libraries(
                    context, package, path, text, command, lists, rule, effect
                )
            elif rule.fix == "cmake_minimum_version":
                change = _minimum_version(context, package, path, text, command, rule, effect)
                if change:
                    yield change


def _inherited_lists(package: SourcePackage, path: Path) -> dict[str, tuple[str, ...] | None]:
    """The plain lists a CMakeLists in a subdirectory sees from the ones above it in the
    package, the package's own first. add_subdirectory hands a child every variable its
    parent has set, so `test/CMakeLists.txt` reading `${dependencies}` reads the top list."""
    lists: dict[str, tuple[str, ...] | None] = {}
    if path.name != "CMakeLists.txt":
        return lists
    parents = [p for p in reversed(path.parents) if p == package.path or package.path in p.parents]
    for directory in parents:
        above = directory / "CMakeLists.txt"
        if above == path or not above.is_file():
            continue
        with above.open(encoding="utf-8", errors="replace", newline="") as handle:
            lists.update(cmake.literal_lists(cmake.commands(handle.read())))
    return lists


def _minimum_version(context, package, path, text, command, rule, effect) -> Change | None:
    args = list(command.args)
    if len(args) < 2 or args[0].text.upper() != "VERSION":
        return None
    version = args[1]
    low, separator, high = version.text.partition("...")
    if _version(high if separator else low) >= _version(rule.find["below"]):
        return None
    return _rule_change(
        context,
        package,
        path,
        text,
        (version.start, version.end),
        rule,
        effect,
        replacement=f"{low}...{TEMPLATE_VERSION}" if separator else TEMPLATE_VERSION,
    )


def _version(text: str) -> tuple[int, ...]:
    parts = []
    for part in text.split("."):
        digits = re.match(r"\d+", part)
        parts.append(int(digits.group()) if digits else 0)
    return tuple(parts)


def _target_link_libraries(context, package, path, text, command, lists, rule, effect) -> Change:
    """`ament_target_dependencies(t [SYSTEM] [INTERFACE|PUBLIC] pkg...)` as the
    `target_link_libraries` call it performed for packages that export targets.

    The keyword is carried over as written. Without one, ament_target_dependencies linked
    with the plain signature, and CMake refuses a target that mixes plain and keyword
    calls, so adding PUBLIC where there was none would break a file that builds today.
    """
    span = (command.start, command.end)
    args = list(command.args)
    target, rest = args[0].text, args[1:]
    keyword = ""
    if rest and rest[0].text == "SYSTEM":
        return _rule_change(
            context,
            package,
            path,
            text,
            span,
            rule,
            effect,
            say=f"{rule.say}; SYSTEM has no target_link_libraries form, link by hand",
        )
    if rest and rest[0].text in {"INTERFACE", "PUBLIC"}:
        keyword, rest = rest[0].text, rest[1:]
    names: list[str] = []
    for arg in rest:
        variable = cmake.VARIABLE.match(arg.text)
        if variable:
            value = lists.get(variable.group(1))
            if not value:
                return _rule_change(
                    context,
                    package,
                    path,
                    text,
                    span,
                    rule,
                    effect,
                    say=f"{rule.say}; the packages come from {arg.text}, which is not a plain "
                    "list in this file, so link them by hand",
                )
            names.extend(value)
        else:
            names.append(arg.text)
    plain = [name for name in dict.fromkeys(names) if not _exports_targets(context, name)]
    if plain:
        own = [name for name in plain if name in context.workspace]
        other = [name for name in plain if name not in context.workspace]
        reasons = []
        if own:
            verb = "exports" if len(own) == 1 else "export"
            reasons.append(
                f"{', '.join(own)} in this workspace {verb} no targets yet, add "
                "ament_export_targets there first"
            )
        if other:
            reasons.append(
                f"exports for {', '.join(other)} are not measured per release; "
                "check the package's exported targets and link by hand"
            )
        return _rule_change(
            context,
            package,
            path,
            text,
            span,
            rule,
            effect,
            say=f"{rule.say}; {'; '.join(reasons)}",
        )
    links = [f"${{{name}_TARGETS}}" for name in dict.fromkeys(names)]
    head = f"target_link_libraries({target}" + (f" {keyword}" if keyword else "")
    one_line = f"{head} {' '.join(links)})"
    if "\n" not in text[command.start : command.end] and command.column - 1 + len(one_line) <= 100:
        replacement = one_line
    else:
        indent = " " * (command.column - 1 + 2)
        body = "\n".join(indent + link for link in links)
        replacement = f"{head}\n{body}\n{' ' * (command.column - 1)})"
    return _rule_change(
        context,
        package,
        path,
        text,
        span,
        rule,
        effect,
        replacement=replacement,
        works_from=context.start,
    )


def _exports_targets(context: _Context, name: str) -> bool:
    own = context.workspace.get(name)
    if own is not None:
        return own.exports_targets
    return False


def exports_targets(cmake_text: str) -> bool:
    """A package of this workspace defines `<name>_TARGETS` for find_package callers when it
    exports targets or generates interfaces, which export them for it."""
    names = {command.lower for command in cmake.commands(cmake_text)}
    return bool(names & {"ament_export_targets", "rosidl_generate_interfaces"})


# ---------------------------------------------------------------------------
# c and c++


def _cpp(context, package, path, text) -> Iterable[Change]:
    for match in _INCLUDE.finditer(blank_comments_and_strings(text, strings=False)):
        change = _include(context, package, path, text, match)
        if change:
            yield change
    code = None
    for rule in context.table.rules:
        if rule.matcher not in {"cpp_identifier", "cpp_regex"}:
            continue
        effect = rule.effect(context.start, context.target)
        if effect is None:
            continue
        needs = rule.find.get("package_depends")
        if needs and not set(needs) & package.depends:
            continue
        if code is None:
            code = blank_comments_and_strings(text)
        fix = rule.fix.get("rename") if isinstance(rule.fix, dict) else None
        for start, end in _cpp_spans(rule, code):
            yield _rule_change(
                context, package, path, text, (start, end), rule, effect, replacement=fix
            )


def _cpp_spans(rule: Rule, code: str) -> Iterable[tuple[int, int]]:
    """Where a C++ rule matches, in code whose comments and literals are blanked. An
    identifier rule spans the name alone, so a rename replaces exactly it."""
    if rule.matcher == "cpp_regex":
        for match in re.finditer(rule.find["cpp_regex"], code):
            yield match.span()
        return
    names = rule.find["cpp_identifier"]
    names = [names] if isinstance(names, str) else names
    after = rule.find.get("after", "")
    for name in names:
        pattern = re.escape(after).replace(r"::", r"\s*::\s*") + r"\b" + re.escape(name) + r"\b"
        for match in re.finditer(pattern, code):
            yield match.end() - len(name), match.end()


def _include(context, package, path, text, match) -> Change | None:
    include = match.group("path")
    facts = distros.header(include)
    if facts is None:
        return None
    start, target = context.start, context.target
    if not (facts.known(start) and facts.has(start)):
        return None
    if not facts.known(target):
        return None
    gone = _gone_since(context, lambda r: not facts.known(r) or not facts.has(r))
    warned = None
    if gone is None and start not in facts.warns:
        warned = _gone_since(context, lambda r: r in facts.warns)
    if gone is None and warned is None:
        return None
    if facts.package == "libpcl-dev" and _pcl_guarded(text, match.start()):
        return None
    successor = context.table.headers.get(include)
    replacement = successor.use if successor else None
    if successor is None:
        for prefix, entry in context.table.header_prefixes.items():
            if include.startswith(prefix):
                successor = entry
                replacement = entry.use + include[len(prefix) :]
                break
    if replacement is None:
        replacement = _sibling(include)
    works_from = None
    if replacement:
        measured = distros.header(replacement)
        if measured is None or not measured.clean(target):
            replacement = None
        else:
            works_from = next(r for r in distros.DISTROS if measured.has(r))
    if gone:
        say = f"<{include}> is not installed by {facts.package} in {gone}"
    else:
        say = f"<{include}> warns that it is deprecated from {warned}"
    if successor and successor.use is None:
        say = f"{say}; {successor.say}"
    elif replacement:
        say = f"{say}; include <{replacement}>"
    return _change(
        context,
        package,
        path,
        text,
        match.span("path"),
        rule="include.removed" if gone else "include.deprecated",
        title=f"<{include}>",
        effect="removed" if gone else "deprecated",
        release=gone or warned,
        say=say,
        source=successor.source if successor else f"https://index.ros.org/p/{facts.package}/",
        replacement=replacement,
        works_from=works_from,
    )


def _sibling(include: str) -> str | None:
    stem, dot, ext = include.rpartition(".")
    if not dot or ext not in {"h", "hpp"}:
        return None
    return f"{stem}.{'hpp' if ext == 'h' else 'h'}"


def _pcl_guarded(text: str, offset: int) -> bool:
    # PCL version guards stay with the compiler until library versions are measured.
    stack = []
    code = blank_comments_and_strings(text[:offset])
    for match in re.finditer(r"^[ \t]*#[ \t]*(if|ifdef|ifndef|elif|endif)\b([^\n]*)", code, re.M):
        directive, expression = match.groups()
        guarded = bool(re.search(r"\bPCL_VERSION_COMPARE\s*\(", expression))
        if directive in {"if", "ifdef", "ifndef"}:
            stack.append(guarded)
        elif directive == "endif" and stack:
            stack.pop()
        elif directive == "elif" and stack:
            stack[-1] |= guarded
    return any(stack)


def blank_comments_and_strings(text: str, *, strings: bool = True) -> str:
    """Blank comments and literal bodies while keeping offsets and newlines.

    `strings=False` preserves quoted include paths, which directive matching must read.
    """
    out = list(text)
    i, n = 0, len(text)
    while i < n:
        if text.startswith("//", i):
            end = text.find("\n", i)
            end = n if end < 0 else end
            out[i:end] = " " * (end - i)
            i = end
        elif text.startswith("/*", i):
            end = text.find("*/", i + 2)
            end = n if end < 0 else end + 2
            out[i:end] = [c if c == "\n" else " " for c in text[i:end]]
            i = end
        elif text[i] in "\"'":
            end = _literal_end(text, i)
            line_start = text.rfind("\n", 0, i) + 1
            include_path = text[i] == '"' and re.fullmatch(
                r"[ \t]*#[ \t]*include[ \t]*", "".join(out[line_start:i])
            )
            if strings or not include_path:
                out[i + 1 : end - 1] = [c if c == "\n" else " " for c in text[i + 1 : end - 1]]
            i = end
        else:
            i += 1
    return "".join(out)


def _literal_end(text: str, i: int) -> int:
    """The offset after the string or character literal opening at `i`. A literal never
    runs past its line, so a digit separator (`1'000`) read as a quote costs one line."""
    n = len(text)
    if text[i] == '"' and i > 0 and text[i - 1] == "R":
        close_paren = text.find("(", i)
        delim = text[i + 1 : close_paren] if close_paren >= 0 else ""
        end = text.find(")" + delim + '"', max(close_paren, i))
        return n if end < 0 else end + len(delim) + 2
    quote, j = text[i], i + 1
    while j < n and text[j] != quote and text[j] != "\n":
        j += 2 if text[j] == "\\" else 1
    return min(j + 1, n)


# ---------------------------------------------------------------------------
# python


def _python(context, package, path, text) -> Iterable[Change]:
    try:
        tree = parse_python(text)
    except SyntaxError:
        return
    offsets = _Offsets(text)
    yield from _imports(context, package, path, text, tree, offsets)
    yield from _launch_nodes(context, package, path, text, tree, offsets)


def _launch_nodes(context, package, path, text, tree, offsets) -> Iterable[Change]:
    """`Node(executable="x", keyword=...)` in a python launch file. The value is read as the
    file states it: a node whose executable is a substitution is not this node, as far as a
    reader of the text can tell, and is left alone."""
    for rule in context.table.rules:
        if rule.matcher != "launch_node":
            continue
        effect = rule.effect(context.start, context.target)
        if effect is None:
            continue
        wanted = rule.find["launch_node"]
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            name = node.func.attr if isinstance(node.func, ast.Attribute) else ""
            name = node.func.id if isinstance(node.func, ast.Name) else name
            if name != "Node":
                continue
            keywords = {k.arg: k for k in node.keywords if k.arg}
            executable = keywords.get("executable")
            if not (
                executable is not None
                and isinstance(executable.value, ast.Constant)
                and executable.value.value == wanted["executable"]
                and wanted["keyword"] in keywords
            ):
                continue
            target = keywords[wanted["keyword"]].value
            if not isinstance(target, (ast.List, ast.Tuple)):
                continue
            for pair in target.elts:
                if not isinstance(pair, (ast.Tuple, ast.List)) or len(pair.elts) != 2:
                    continue
                source = pair.elts[0]
                literal = source.value if isinstance(source, ast.Constant) else None
                if isinstance(literal, str) and literal in wanted.get("exclude_sources", ()):
                    continue
                if "source" in wanted and literal != wanted["source"]:
                    continue
                start = offsets.at(pair.lineno, pair.col_offset)
                end = offsets.at(pair.end_lineno, pair.end_col_offset)
                yield _rule_change(context, package, path, text, (start, end), rule, effect)


def _imports(context, package, path, text, tree, offsets) -> Iterable[Change]:
    rule = next((r for r in context.table.rules if r.matcher == "python_import"), None)
    if rule is None:
        return
    old = distros.version(context.start, "python")
    removed: dict[str, tuple[int, ...]] = {}
    for version, names in context.table.removed_modules.items():
        if old < version <= distros.version(context.target, "python"):
            removed.update(dict.fromkeys(names, version))
    if not removed:
        return
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            modules = [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
            modules = [node.module]
        else:
            continue
        for module in modules:
            top = module.split(".")[0]
            if top not in removed:
                continue
            release = next(
                r
                for r in distros.between(context.start, context.target)
                if distros.version(r, "python") >= removed[top]
            )
            start = offsets.at(node.lineno, node.col_offset)
            end = offsets.at(node.end_lineno, node.end_col_offset)
            python = ".".join(str(part) for part in removed[top])
            yield _change(
                context,
                package,
                path,
                text,
                (start, end),
                rule=rule.id,
                title=f"import {top}",
                effect="removed",
                release=release,
                say=f"{top} was removed in python {python}, which {release} ships",
                source=f"https://docs.python.org/3/whatsnew/{python}.html",
            )


class _Offsets:
    """Text offsets for `ast` positions, whose columns count UTF-8 bytes of their line."""

    def __init__(self, text: str) -> None:
        self.lines = text.splitlines(keepends=True) or [""]
        self.starts = [0]
        for line in self.lines:
            self.starts.append(self.starts[-1] + len(line))

    def at(self, lineno: int, col: int) -> int:
        line = self.lines[lineno - 1]
        return self.starts[lineno - 1] + len(line.encode("utf-8")[:col].decode("utf-8", "ignore"))
