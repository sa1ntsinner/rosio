"""`rosio init`: the configuration rosio would use anyway, written out for a person to edit.

Without a `rosio.yaml` extraction already falls back to one profile per launch file found in
the workspace (`config.default_config`). That is enough to extract, and it is not a claim about
any system that runs, because a running robot is usually several entry points started
together. Writing the fallback out turns it into the place where that knowledge goes: the
person merges the entry points that start together and states their arguments.
"""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

import yaml

from rosio.config import (
    CONFIG_NAME,
    DEFAULT_LAUNCH_TIMEOUT,
    Config,
    Entrypoint,
    ProfileSpec,
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


def render(cfg: Config, base: Path) -> str:
    """The configuration as yaml, paths relative to the workspace when they sit inside it."""
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
    cfg = default_config(str(workspace), packages)
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
