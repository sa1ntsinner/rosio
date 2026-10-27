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
        if item.verb == "launch" and Path(item.target).name not in included
    )


def _included(profiles: Sequence[ProfileSpec]) -> dict[str, list[Entrypoint]]:
    """The file names one launch file of the tree includes from another, read from the
    include itself: `PythonLaunchDescriptionSource([..., "rrbot_base.launch.py"])`, the
    variable such a source is handed, or the `file` of an xml or yaml include. A name a
    comment mentions, or one that ends another name, includes nothing. Each name maps to the
    entry points that include it."""
    out: dict[str, list[Entrypoint]] = {}
    for profile in profiles:
        for item in profile.entrypoints:
            if item.verb != "launch" or item.origin is None:
                continue
            path = Path(item.origin.path)
            try:
                text = path.read_text(encoding="utf-8", errors="replace")
            except OSError:
                continue
            spelled = _python_includes(text) if path.suffix == ".py" else _frontend_includes(text)
            names = {re.split(r"[/\\]", value.strip())[-1] for value in spelled} - {path.name}
            for name in sorted(names):
                out.setdefault(name, []).append(item)
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


def _python_includes(text: str) -> list[str]:
    try:
        tree = parse_python(text)
    except (SyntaxError, ValueError):
        return []
    assigned: dict[str, list[ast.AST]] = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name):
                    assigned.setdefault(target.id, []).append(node.value)
    out: list[str] = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        called = node.func.attr if isinstance(node.func, ast.Attribute) else ""
        called = node.func.id if isinstance(node.func, ast.Name) else called
        if called not in INCLUDE_CALLS:
            continue
        pending: list[ast.AST] = [*node.args, *(item.value for item in node.keywords)]
        seen: set[str] = set()
        while pending:
            part = pending.pop()
            if isinstance(part, ast.Name) and part.id not in seen:
                seen.add(part.id)
                pending.extend(assigned.get(part.id, []))
                continue
            for inner in ast.walk(part):
                if isinstance(inner, ast.Constant) and isinstance(inner.value, str):
                    out.append(inner.value)
                elif isinstance(inner, ast.Name) and inner.id not in seen:
                    pending.append(inner)
    return out


def _frontend_includes(text: str) -> list[str]:
    return re.findall(r"<include\b[^>]*?\bfile\s*=\s*[\"']([^\"']+)", text) + re.findall(
        r"\binclude:\s*\n\s*file:\s*[\"']?([^\"'\n]+)", text
    )


def _include_only(entry: Entrypoint, included: Mapping[str, Sequence[Entrypoint]]) -> bool:
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
        Path(includer.origin.path).name in included
        or not required & set(_required_arguments(includer))
        for includer in included.get(Path(entry.origin.path).name, ())
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
    profiles = "".join(_profile(profile) for profile in cfg.profiles)
    return f"{HEADER}\n{defaults}{body}profiles:" + (f"\n{profiles}" if profiles else " []\n")


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
    try:
        return _write(wanted, cfg, force), cfg
    except OSError as exc:
        here = Path.cwd() / CONFIG_NAME
        if target is not None or here.absolute() == wanted.absolute():
            raise UsageError(
                f"cannot write {wanted}: {exc.strerror or exc}. "
                "name a file you may write with --config FILE, or print it with --print"
            ) from exc
    try:
        return _write(here, cfg, force), cfg
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
