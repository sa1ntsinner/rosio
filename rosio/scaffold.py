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

from rosio.config import CONFIG_NAME, DEFAULT_LAUNCH_TIMEOUT, Config, default_config
from rosio.errors import UsageError

HEADER = """\
# rosio configuration, written by `rosio init`.
#
# roots      the trees rosio reads: src for sources, install for a built or installed tree
# ros_setup  the setup files a launch file needs sourced before rosio can walk it
# profiles   one per system you actually run. Below is one per launch file rosio found.
#            Merge the entry points that start together into one profile, state the
#            arguments they are started with, and delete the rest. A finding is always
#            about one profile, so a profile nobody runs only produces noise.
"""


def render(cfg: Config, base: Path) -> str:
    """The configuration as yaml, paths relative to the workspace when they sit inside it."""
    payload = {
        "roots": [{"kind": spec.kind, "path": _relative(spec.path, base)} for spec in cfg.roots],
        "ros_setup": list(cfg.ros_setup),
        "launch_timeout": int(DEFAULT_LAUNCH_TIMEOUT),
        "profiles": [
            {
                "slug": profile.slug,
                "label": profile.label,
                "entrypoints": [
                    {entry.verb: f"{entry.pkg} {entry.target}"} for entry in profile.entrypoints
                ],
            }
            for profile in cfg.profiles
        ],
    }
    if not payload["ros_setup"]:
        del payload["ros_setup"]
    return HEADER + "\n" + yaml.safe_dump(payload, sort_keys=False, default_flow_style=False)


def write(
    workspace: Path,
    *,
    force: bool = False,
    packages: Sequence[str] = (),
    target: Path | None = None,
) -> tuple[Path, Config]:
    """Write `rosio.yaml`, into the workspace unless `target` names the file, refusing to
    replace one unless asked. Paths are written relative to the file when they sit below it,
    so a tree nobody may write into, such as `/opt/ros/<distro>`, is configured from outside."""
    target = target or workspace / CONFIG_NAME
    if target.exists() and not force:
        raise UsageError(f"{target} already exists, pass --force to replace it")
    cfg = default_config(str(workspace), packages)
    target.write_text(render(cfg, target.parent), encoding="utf-8", newline="\n")
    return target, cfg


def _relative(path: str, base: Path) -> str:
    resolved, root = Path(path).resolve(), base.resolve()
    if resolved == root:
        return "."
    try:
        return resolved.relative_to(root).as_posix()
    except ValueError:
        return path
