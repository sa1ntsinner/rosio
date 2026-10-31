"""`rosio init`: the configuration rosio would use anyway, written out for a person to edit.

Without a `rosio.yaml` extraction already falls back to one profile per launch file found in
the workspace (`config.default_config`). That is enough to extract, and it is not a claim about
any system that runs, because a running robot is usually several entry points started
together. Writing the fallback out turns it into the place where that knowledge goes: the
person merges the entry points that start together and states their arguments.
"""

from __future__ import annotations

import ast
import re
import xml.etree.ElementTree as ET
from collections.abc import Mapping, Sequence
from dataclasses import replace
from fnmatch import fnmatch
from pathlib import Path

import yaml

from rosio.config import (
    CONFIG_NAME,
    DEFAULT_LAUNCH_TIMEOUT,
    TEST_DIRS,
    TEST_IGNORE,
    TEST_NAME,
    Config,
    Entrypoint,
    ProfileSpec,
    _default_roots,
    _slug_for,
    default_config,
)
from rosio.errors import UsageError
from rosio.resolve import parse_python
from rosio.workspace import Workspace

HEADER = """# rosio configuration, written by `rosio init`.
#
# roots      the trees rosio reads: src for sources, install for a built or installed tree
# ignore     paths under a root that rosio skips, matched against the path inside the root.
#            Left out, the test and tests directories are skipped.
# ros_setup  the setup files a launch file needs sourced before rosio can walk it
# profiles   one per system you actually run. Below is one per launch file rosio found.
#            Merge the entry points that start together into one profile, state the
#            arguments they are started with, and delete the rest. A finding is always
#            about one profile, so a profile nobody runs only produces noise.
#            A launch file that reads an environment variable with no default gets a
#            commented env block under its profile: uncomment it and fill in the values.
#
# Every key is described in docs/configuration.md.
"""

SETUP_VARIABLES = ("AMENT_", "COLCON_", "ROS_")
"""Prefixes of the variables a sourced ROS setup file exports. A launch file that reads
ROS_DISTRO is not asking the person who writes the profile."""


def offered(cfg: Config) -> Config:
    """The fallback configuration less the launch files init does not offer as profiles."""
    included = _included(cfg.profiles)
    kept = [
        profile
        for profile in cfg.profiles
        if not any(
            _not_offered(item, cfg.ros_setup) or _include_only(item, included)
            for item in profile.entrypoints
        )
    ]
    return replace(cfg, profiles=tuple(_one_slug_rule(kept)))


MANY_PROFILES = 20
"""Past this many, init names the files a robot most likely starts from."""


def run_entrypoints(cfg: Config) -> list[str]:
    """`ros2 run` lines for the executables the roots build, for a workspace with no launch
    file to offer as a profile."""
    if not cfg.roots:
        return []
    ws = Workspace.discover(cfg.roots, cfg.ignore)
    return [
        f"ros2 run {name} {exe}"
        for name, package in ws.packages.items()
        if package.src_path
        for exe in package.executables
    ]


def top_level(cfg: Config) -> list[str]:
    """The launch files of the offered profiles that no other launch file includes."""
    included = _included(cfg.profiles)
    return sorted(
        f"{item.pkg}/{Path(item.target).name}"
        for profile in cfg.profiles
        for item in profile.entrypoints
        if item.verb == "launch" and _launch_key(item) not in included
    )


def _launch_key(entry: Entrypoint) -> tuple[str, Path]:
    return entry.pkg, Path(entry.origin.path if entry.origin else entry.target).resolve()


def _included(
    profiles: Sequence[ProfileSpec], *, started_only: bool = False
) -> dict[tuple[str, Path], list[Entrypoint]]:
    """The files one launch file of the tree includes from another, by package and path,
    read from the include itself: `PythonLaunchDescriptionSource([..., "rrbot_base.launch.py"])`,
    the variable such a source is handed, or the `file` of an xml or yaml include. A path the
    reader cannot follow includes nothing. With `started_only`, an include under a condition
    or one that passes arguments is left out, because it does not start the file as it is.
    Each key maps to the entry points that include it."""
    out: dict[tuple[str, Path], list[Entrypoint]] = {}
    for profile in profiles:
        for item in profile.entrypoints:
            if item.verb != "launch" or item.origin is None:
                continue
            path = Path(item.origin.path)
            try:
                text = path.read_text(encoding="utf-8", errors="replace")
            except OSError:
                continue
            spelled = (
                _python_includes(text, started_only)
                if path.suffix == ".py"
                else _frontend_includes(text, path.suffix, started_only)
            )
            for value in sorted(set(spelled)):
                share = re.fullmatch(r"\$\(find-pkg-share ([\w]+)\)/(.*)", value)
                if share:
                    package, relative = share.groups()
                    root = next((p for p in path.parents if (p / "package.xml").is_file()), None)
                    if package != item.pkg or root is None:
                        continue
                    target = root / relative
                else:
                    if "$" in value or Path(value).is_absolute():
                        continue
                    target = path.parent / value
                key = item.pkg, target.resolve()
                if key != _launch_key(item):
                    out.setdefault(key, []).append(item)
    return out


INCLUDE_CALLS = frozenset(
    {
        "IncludeLaunchDescription",
        "PythonLaunchDescriptionSource",
        "AnyLaunchDescriptionSource",
        "FrontendLaunchDescriptionSource",
        "XMLLaunchDescriptionSource",
        "YAMLLaunchDescriptionSource",
    }
)


def _python_includes(text: str, started_only: bool = False) -> list[str]:
    try:
        tree = parse_python(text)
    except (SyntaxError, ValueError):
        return []
    parents = {child: node for node in ast.walk(tree) for child in ast.iter_child_nodes(node)}

    def scope(node: ast.AST) -> ast.AST:
        while not isinstance(node, (ast.Module, ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)):
            node = parents[node]
        return node

    assigned: dict[tuple[ast.AST, str], list[ast.AST]] = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name):
                    assigned.setdefault((scope(node), target.id), []).append(node.value)

    def values(node: ast.Name) -> list[ast.AST]:
        owner = scope(node)
        while True:
            if (owner, node.id) in assigned:
                return assigned[owner, node.id]
            if isinstance(owner, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)) and any(
                isinstance(arg, ast.arg) and arg.arg == node.id for arg in ast.walk(owner.args)
            ):
                return []
            if owner not in parents:
                return []
            owner = scope(parents[owner])

    def called(node: ast.AST) -> str:
        if not isinstance(node, ast.Call):
            return ""
        return ast.unparse(node.func)

    def resolve(node: ast.AST, seen: frozenset[str] = frozenset()) -> str | None:
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            return node.value
        if isinstance(node, ast.Name) and node.id not in seen:
            found = values(node)
            return resolve(found[0], seen | {node.id}) if len(found) == 1 else None
        if isinstance(node, (ast.List, ast.Tuple)):
            parts = [resolve(part, seen) for part in node.elts]
            return "".join(parts) if all(part is not None for part in parts) else None
        if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
            left, right = resolve(node.left, seen), resolve(node.right, seen)
            return left + right if left is not None and right is not None else None
        if isinstance(node, ast.JoinedStr):
            parts = [
                resolve(part.value if isinstance(part, ast.FormattedValue) else part, seen)
                for part in node.values
            ]
            return "".join(parts) if all(part is not None for part in parts) else None
        name = called(node)
        if name == "ThisLaunchFileDir" and not node.args:
            return "."
        if not isinstance(node, ast.Call) or node.keywords:
            return None
        if (
            name == "os.path.dirname"
            and len(node.args) == 1
            and isinstance(node.args[0], ast.Name)
            and node.args[0].id == "__file__"
        ):
            return "."
        if name in {"get_package_share_directory", "FindPackageShare"} and len(node.args) == 1:
            package = node.args[0]
            if isinstance(package, ast.Constant) and isinstance(package.value, str):
                return f"$(find-pkg-share {package.value})"
        if name in INCLUDE_CALLS - {"IncludeLaunchDescription"} and len(node.args) == 1:
            return resolve(node.args[0], seen)
        if name == "os.path.join" or name == "PathJoinSubstitution":
            args = node.args
            if name == "PathJoinSubstitution":
                if len(args) != 1 or not isinstance(args[0], (ast.List, ast.Tuple)):
                    return None
                args = args[0].elts
            parts = [resolve(part, seen) for part in args]
            if parts and all(part is not None for part in parts):
                return Path(parts[0]).joinpath(*parts[1:]).as_posix()
        return None

    def empty_arguments(node: ast.AST, seen: frozenset[str] = frozenset()) -> bool:
        if isinstance(node, ast.Name) and node.id not in seen:
            found = values(node)
            return len(found) == 1 and empty_arguments(found[0], seen | {node.id})
        if isinstance(node, (ast.List, ast.Tuple)):
            return not node.elts
        if isinstance(node, ast.Dict):
            return not node.keys
        if isinstance(node, ast.Call) and called(node).endswith(".items"):
            return not node.args and not node.keywords and empty_arguments(node.func.value, seen)
        return isinstance(node, ast.Constant) and node.value is None

    blocked: set[ast.AST] = set()
    if started_only:
        pending = [
            node
            for node in ast.walk(tree)
            if called(node).split(".")[-1] == "GroupAction"
            and any(key.arg in {"condition", None} for key in node.keywords)
        ]
        while pending:
            node = pending.pop()
            if node in blocked:
                continue
            blocked.add(node)
            pending.extend(ast.iter_child_nodes(node))
            if isinstance(node, ast.Name):
                pending.extend(values(node))
    out: list[str] = []
    for node in ast.walk(tree):
        if called(node).split(".")[-1] != "IncludeLaunchDescription" or node in blocked:
            continue
        keywords = {key.arg: key.value for key in node.keywords}
        if None in keywords or (started_only and "condition" in keywords):
            continue
        if (
            started_only
            and "launch_arguments" in keywords
            and not empty_arguments(keywords["launch_arguments"])
        ):
            continue
        if len(node.args) > 1:
            continue
        source = node.args[0] if node.args else keywords.get("launch_description_source")
        value = resolve(source) if source is not None else None
        if value:
            out.append(value)
    return out


def _frontend_includes(text: str, suffix: str, started_only: bool = False) -> list[str]:
    out: list[str] = []

    def xml(node: ET.Element, blocked: bool = False) -> None:
        blocked |= node.tag in {"include", "group"} and bool({"if", "unless"} & node.attrib.keys())
        if node.tag == "include" and not (started_only and (blocked or len(node))):
            out.append(node.get("file", ""))
        for child in node:
            xml(child, blocked)

    def markup(node: object, blocked: bool = False) -> None:
        if isinstance(node, list):
            for child in node:
                markup(child, blocked)
        elif isinstance(node, dict):
            for kind, body in node.items():
                if kind in {"include", "group"} and isinstance(body, dict):
                    gated = blocked or bool({"if", "unless"} & body.keys())
                    if kind == "include" and not (
                        started_only and (gated or body.get("arg") or body.get("args"))
                    ):
                        value = body.get("file")
                        if isinstance(value, str):
                            out.append(value)
                    markup(body, gated)
                else:
                    markup(body, blocked)

    try:
        if suffix in {".yaml", ".yml"}:
            markup(yaml.safe_load(text))
        else:
            xml(ET.fromstring(text))
    except (ET.ParseError, yaml.YAMLError):
        return []
    return [value for value in out if value]


def _include_only(
    entry: Entrypoint, included: Mapping[tuple[str, Path], Sequence[Entrypoint]]
) -> bool:
    """A launch file that cannot start without arguments, included by a file that does not
    ask for them itself: that includer is how it runs, so as a profile it only raises.

    An entry point nothing includes that requires one of the same arguments hands the person
    the same question, so it is no better a place to start: autoware.launch.xml needs
    map_path, and so does every simulator file that includes it, so it stays offered. The
    components it includes are left out, map_path included, because autoware.launch.xml is
    itself included and is not where a person starts."""
    if entry.verb != "launch" or entry.origin is None:
        return False
    required = set(_required_arguments(entry))
    if not required:
        return False
    return any(
        _launch_key(includer) in included or not required & set(_required_arguments(includer))
        for includer in included.get(_launch_key(entry), ())
        if includer.origin is not None
    )


def _required_arguments(entry: Entrypoint) -> tuple[str, ...]:
    from rosio.resolve.launch_exec import required_arguments, required_frontend_arguments

    if entry.verb != "launch" or entry.origin is None:
        return ()
    path = Path(entry.origin.path)
    try:
        source = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ()
    if path.suffix == ".py":
        required = required_arguments(source)
    else:
        required = required_frontend_arguments(source, path.suffix)
    stated = {key for key, _value in entry.args}
    return tuple(name for name in required if name not in stated)


def _one_slug_rule(profiles: Sequence[ProfileSpec]) -> list[ProfileSpec]:
    """Every slug a file stem, or every slug `package.stem` once one stem names two files in
    the tree. The fallback prefixes only the files that collide, which wrote
    `nav2_bringup.localization` next to `navigation` and `slam`. Decided over the whole
    tree, as the fallback is, so a package filter never renames a profile."""
    if all(profile.slug == _stem(profile) for profile in profiles):
        return list(profiles)
    names = [_package_stem(profile) for profile in profiles]
    if not all(names) or len(set(names)) != len(names):
        return list(profiles)
    renamed = [replace(profile, slug=name) for profile, name in zip(profiles, names, strict=True)]
    return sorted(renamed, key=lambda profile: profile.slug)


def _stem(profile: ProfileSpec) -> str:
    if len(profile.entrypoints) != 1 or profile.entrypoints[0].verb != "launch":
        return ""
    return _slug_for(profile.entrypoints[0].target)


def _package_stem(profile: ProfileSpec) -> str:
    stem = _stem(profile)
    return f"{profile.entrypoints[0].pkg}.{stem}" if stem else ""


def render(cfg: Config, base: Path) -> str:
    """The configuration as yaml, paths relative to the workspace when they sit inside it."""
    cfg = offered(cfg)
    payload: dict = {
        "roots": [{"kind": spec.kind, "path": _relative(spec.path, base)} for spec in cfg.roots],
        "ignore": list(cfg.ignore),
        "ros_setup": list(cfg.ros_setup),
        "launch_timeout": int(DEFAULT_LAUNCH_TIMEOUT),
    }
    defaults = ""
    if _defaults_hold(cfg, base):
        # Left out, each extract takes the defaults again, so a later colcon build is read.
        shown = {key: payload.pop(key) for key in ("roots", "ros_setup") if payload[key]}
        payload.pop("ros_setup", None)
        defaults = DEFAULTS_NOTE + "".join(
            f"#   {line}\n" for line in yaml.safe_dump(shown, sort_keys=False).splitlines()
        )
    if payload["ignore"] == list(TEST_IGNORE):
        del payload["ignore"]
    for key in ("ignore", "ros_setup"):
        if key in payload and not payload[key]:
            del payload[key]
    body = yaml.safe_dump(payload, sort_keys=False, default_flow_style=False)
    inside = started_inside(cfg.profiles)
    profiles = "".join(
        _profile(profile) if profile.slug not in inside else _commented(profile, inside)
        for profile in cfg.profiles
    )
    empty = all(profile.slug in inside for profile in cfg.profiles)
    return f"{HEADER}\n{defaults}{body}profiles:" + (" []\n" if empty else "\n") + profiles


def started_inside(profiles: Sequence[ProfileSpec]) -> dict[str, str]:
    """Slug to the offered file of the same package that includes this profile's launch file
    with no condition and no argument, and starts with no argument itself. Read as a profile of
    its own, such a file restates every finding of the file that starts it, so it is written
    commented out. An include under a condition or with arguments starts something else."""
    included = _included(profiles, started_only=True)
    runs = {
        id(item): f"{item.pkg}/{Path(item.target).name}"
        for profile in profiles
        for item in profile.entrypoints
        if item.verb == "launch" and not _required_arguments(item)
    }
    out = {}
    for profile in profiles:
        if len(profile.entrypoints) != 1:
            continue
        (entry,) = profile.entrypoints
        if entry.verb != "launch" or entry.origin is None:
            continue
        starters = sorted(
            runs[id(includer)]
            for includer in included.get(_launch_key(entry), ())
            if id(includer) in runs and includer.pkg == entry.pkg
        )
        if starters:
            out[profile.slug] = starters[0]
    return out


def _commented(profile: ProfileSpec, inside: Mapping[str, str]) -> str:
    note = f"{inside[profile.slug]} includes and starts this one; uncomment it to read it alone"
    return f"# {note}\n" + "".join(f"# {line}\n" for line in _profile(profile).splitlines())


DEFAULTS_NOTE = """\
# roots and ros_setup are left out, so every extract takes the defaults: src, the install
# tree beside it once colcon build has made one, and the setup files of the sourced ROS
# release and of that install tree. Here and now they are:
"""


def _defaults_hold(cfg: Config, base: Path) -> bool:
    """Whether an extract run where this file sits finds the same roots on its own."""
    try:
        found = _default_roots(str(base))
    except UsageError:
        return False
    return [(spec.kind, spec.path) for spec in found] == [
        (spec.kind, spec.path) for spec in cfg.roots
    ]


def _profile(profile: ProfileSpec) -> str:
    entry = {
        "slug": profile.slug,
        "label": profile.label,
        "entrypoints": [{item.verb: f"{item.pkg} {item.target}"} for item in profile.entrypoints],
    }
    text = yaml.safe_dump([entry], sort_keys=False, default_flow_style=False)
    for item in profile.entrypoints:
        required = _required_arguments(item)
        if required:
            lines = [
                f"{Path(item.target).name} declares these with no default, "
                "state them under its entry point:",
                "  args:",
                *(f"    {name}: ''" for name in required),
            ]
            text += "".join(f"  # {line}\n" for line in lines)
    for item in profile.entrypoints:
        names = _unset_environment(item)
        if not names:
            continue
        lines = [
            f"{Path(item.target).name} reads these with no default, "
            "set them to what the robot starts with:",
            "env:",
            *(f"  {name}: ''" for name in names),
        ]
        text += "".join(f"  # {line}\n" for line in lines)
    return text


TEST_MARKERS = ("import launch_testing", "from launch_testing", "generate_test_description")
"""What a launch_testing file holds. It describes a test run with its own assertions, which
is not a system anybody starts, and as a profile it only reported its test dependencies."""


FRONTENDS = {".xml": ("launch_xml",), ".yaml": ("launch_yaml",), ".yml": ("launch_yaml",)}


def _not_offered(entry: Entrypoint, ros_setup: Sequence[str] = ()) -> bool:
    """A launch file init leaves out: a launch_testing file, a ROS 1 launch file, or a markup
    file no installed frontend parses, which extraction could only refuse. The frontend is
    looked for where extraction walks, in the sourced environment, and a file is offered when
    that could not be asked."""
    from rosio.resolve.launch_exec import importable, is_ros1_launch

    if entry.verb != "launch" or entry.origin is None:
        return False
    path = Path(entry.origin.path)
    if TEST_DIRS & {path.parent.name, path.parent.parent.name} or fnmatch(path.name, TEST_NAME):
        return True
    if path.suffix != ".py":
        if is_ros1_launch(str(path)):
            return True
        wanted = FRONTENDS.get(path.suffix, ("launch_xml", "launch_yaml"))
        found = importable(wanted, ros_setup)
        return found is not None and not found
    try:
        source = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return False
    return any(marker in source for marker in TEST_MARKERS)


def _unset_environment(entry: Entrypoint) -> tuple[str, ...]:
    """What the launch file of one entry point reads from the environment with no default,
    less what a sourced setup file exports."""
    from rosio.resolve.launch_exec import environment_reads

    if entry.verb != "launch" or entry.origin is None:
        return ()
    try:
        source = Path(entry.origin.path).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ()
    return tuple(name for name in environment_reads(source) if not name.startswith(SETUP_VARIABLES))


def write(
    workspace: Path,
    *,
    force: bool = False,
    packages: Sequence[str] = (),
    target: Path | None = None,
) -> tuple[Path, Config]:
    """Write `rosio.yaml`, into the workspace unless `target` names the file, refusing to
    replace one unless asked. Paths are written relative to the file when they sit below it,
    so a tree nobody may write into, such as `/opt/ros/<distro>`, is configured from outside.

    When the workspace is such a tree and no file was named, the file goes into the working
    directory instead, and the returned path says so. A named file that cannot be written is
    refused with the two ways out.
    """
    cfg = offered(default_config(str(workspace), packages))
    wanted = target or workspace / CONFIG_NAME
    inside = started_inside(cfg.profiles)
    shown = replace(cfg, profiles=tuple(item for item in cfg.profiles if item.slug not in inside))
    try:
        return _write(wanted, cfg, force), shown
    except OSError as exc:
        here = Path.cwd() / CONFIG_NAME
        if target is not None or here.absolute() == wanted.absolute():
            raise UsageError(
                f"cannot write {wanted}: {exc.strerror or exc}. "
                "name a file you may write with --config FILE, or print it with --print"
            ) from exc
    try:
        return _write(here, cfg, force), shown
    except OSError as exc:
        raise UsageError(
            f"cannot write {wanted} or {here}: {exc.strerror or exc}. "
            "name a file you may write with --config FILE, or print it with --print"
        ) from exc


def _write(target: Path, cfg: Config, force: bool) -> Path:
    if target.exists() and not force:
        raise UsageError(f"{target} already exists, pass --force to replace it")
    target.write_text(render(cfg, target.parent), encoding="utf-8", newline="\n")
    return target


def _relative(path: str, base: Path) -> str:
    resolved, root = Path(path).resolve(), base.resolve()
    if resolved == root:
        return "."
    try:
        return resolved.relative_to(root).as_posix()
    except ValueError:
        return path
