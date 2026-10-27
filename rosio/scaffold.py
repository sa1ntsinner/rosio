"""`rosio init`: the configuration rosio would use anyway, written out for a person to edit.

Without a `rosio.yaml` extraction already falls back to one profile per launch file found in
the workspace (`config.default_config`). That is enough to extract, and it is not a claim about
any system that runs, because a running robot is usually several entry points started
together. Writing the fallback out turns it into the place where that knowledge goes: the
person merges the entry points that start together and states their arguments.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import replace
from pathlib import Path

import yaml

from rosio.config import (
    CONFIG_NAME,
    DEFAULT_LAUNCH_TIMEOUT,
    Config,
    Entrypoint,
    ProfileSpec,
    _slug_for,
    default_config,
)
from rosio.errors import UsageError

HEADER = """# rosio configuration, written by `rosio init`.
#
# roots      the trees rosio reads: src for sources, install for a built or installed tree
# ignore     paths under a root that rosio skips, matched against the path inside the root
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
    kept = [
        profile
        for profile in cfg.profiles
        if not any(_not_offered(item, cfg.ros_setup) for item in profile.entrypoints)
    ]
    return replace(cfg, profiles=tuple(_one_slug_rule(kept)))


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
    for key in ("ignore", "ros_setup"):
        if not payload[key]:
            del payload[key]
    body = yaml.safe_dump(payload, sort_keys=False, default_flow_style=False)
    profiles = "".join(_profile(profile) for profile in cfg.profiles)
    return f"{HEADER}\n{body}profiles:" + (f"\n{profiles}" if profiles else " []\n")


def _profile(profile: ProfileSpec) -> str:
    entry = {
        "slug": profile.slug,
        "label": profile.label,
        "entrypoints": [{item.verb: f"{item.pkg} {item.target}"} for item in profile.entrypoints],
    }
    text = yaml.safe_dump([entry], sort_keys=False, default_flow_style=False)
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
    """A launch file init leaves out: a launch_testing file, or a markup file no installed
    frontend parses, which extraction could only refuse. The frontend is looked for where
    extraction walks, in the sourced environment, and a file is offered when that could not
    be asked."""
    from rosio.resolve.launch_exec import importable

    if entry.verb != "launch" or entry.origin is None:
        return False
    path = Path(entry.origin.path)
    if path.suffix != ".py":
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
