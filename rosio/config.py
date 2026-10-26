"""Run configuration: which trees to read, and which compositions actually run.

A profile is the unit that matters. No single launch file describes a running system on the
reference stack: the desktop launcher builds about twenty five `ros2 launch` and `ros2 run`
commands, and two of the interesting profiles differ only by one launch argument. A profile
is therefore an ordered list of entry points with their arguments and environment, and every
declared fact carries the profile it came from.

`rosio.yaml` is the config file name and an absent file is not an error: `default_config`
falls back to one profile per discovered launch file, which is enough to extract but is not
a claim about any system that runs.

Waivers are configuration too, and they are the one part of it that never reaches an atlas.
`waivers` names the files, `load_waivers` reads them, and what they hold decides how a gate
grades a finding, never whether the finding is in the file. So two machines with different
waiver files still produce the same atlas byte for byte, and `Config.fingerprint` leaves the
key out for the same reason it leaves the output directory out.
"""

from __future__ import annotations

import os
import re
import shlex
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from rosio.errors import UsageError
from rosio.resolve import records
from rosio.workspace import LAUNCH_SUFFIXES, SRC_SKIP_DIRS, Workspace

CONFIG_NAME = "rosio.yaml"
CONFIG_ENV = "ROSIO_CONFIG"
WAIVERS_NAME = "rosio-waivers.yaml"
DEFAULT_OUT_DIR = "rosio-out"
DEFAULT_LAUNCH_TIMEOUT = 60.0

VERBS = ("launch", "run")

INSTALL = records.ROOTS[1]
"""The built tree. `--install-tree` names one, and `with_install_tree` puts it in the config."""

AMENT_INDEX = Path("share", "ament_index", "resource_index", "packages")

_TOP_KEYS = frozenset(
    {
        "deterministic",
        "ignore",
        "launch_timeout",
        "launch_sandbox",
        "out",
        "out_dir",
        "profiles",
        "roots",
        "ros_setup",
        "waivers",
    }
)

_ROOT_KEYS = frozenset({"kind", "path"})

_PROFILE_KEYS = frozenset({"entrypoints", "env", "label", "slug"})

_ENTRY_KEYS = frozenset({"args", "argv", "launch", "pkg", "package", "run", "target", "verb"})

_WAIVER_KEYS = frozenset({"key", "reason"})

WAIVER_PREFIXES = ("find:", "sha256:")
"""What a waiver key looks like: a finding id, or a finding fingerprint. A rule id is
neither, and accepting one would silence a whole rule under a spelling that reads like one
finding."""


@dataclass(frozen=True)
class RootSpec:
    kind: str
    path: str

    def __post_init__(self) -> None:
        if self.kind not in records.ROOTS:
            raise UsageError(f"root kind {self.kind!r} is not one of {records.ROOTS}")
        if not self.path:
            raise UsageError("a root needs a path")


@dataclass(frozen=True)
class Entrypoint:
    """One `ros2 launch` or `ros2 run` invocation. `target` is the launch file relative to
    the package launch directory, or the executable name for a run."""

    verb: str
    pkg: str
    target: str
    args: tuple[tuple[str, str], ...] = ()
    argv: tuple[str, ...] = ()
    origin: records.Site | None = None

    def __post_init__(self) -> None:
        if self.verb not in VERBS:
            raise UsageError(f"entry point verb {self.verb!r} is not one of {VERBS}")
        if not self.pkg or not self.target:
            raise UsageError("an entry point needs a package and a target")

    @property
    def label(self) -> str:
        return f"ros2 {self.verb} {self.pkg} {self.target}"


@dataclass(frozen=True)
class ProfileSpec:
    slug: str
    label: str = ""
    entrypoints: tuple[Entrypoint, ...] = ()
    env: tuple[tuple[str, str], ...] = ()

    def __post_init__(self) -> None:
        if not self.slug:
            raise UsageError("a profile needs a slug")


@dataclass(frozen=True)
class Waiver:
    """One finding a repository has decided to live with, and why.

    The reason is mandatory. A waiver with no reason is how a gate rots: nobody can tell a
    year later whether the finding was understood or only silenced, so nobody deletes it.
    """

    key: str
    reason: str

    def __post_init__(self) -> None:
        if not self.key.startswith(WAIVER_PREFIXES):
            raise UsageError(
                f"waiver key {self.key!r} is neither a finding id nor a fingerprint. "
                f"a key starts with {' or '.join(WAIVER_PREFIXES)}, and "
                "rosio drift --json prints both for every finding"
            )
        if not self.reason.strip():
            raise UsageError(f"waiver {self.key} needs a reason")


@dataclass(frozen=True)
class Config:
    roots: tuple[RootSpec, ...]
    profiles: tuple[ProfileSpec, ...]
    out_dir: str = DEFAULT_OUT_DIR
    deterministic: bool = False
    launch_timeout: float = DEFAULT_LAUNCH_TIMEOUT
    ros_setup: tuple[str, ...] = ()
    ignore: tuple[str, ...] = ()
    waivers: tuple[str, ...] = ()
    launch_sandbox: str = "auto"

    def __post_init__(self) -> None:
        if self.launch_sandbox not in ("auto", "required", "off"):
            raise UsageError("launch_sandbox must be auto, required or off")
        if not self.roots:
            raise UsageError("a config needs at least one root")
        slugs = [profile.slug for profile in self.profiles]
        duplicate = sorted({slug for slug in slugs if slugs.count(slug) > 1})
        if duplicate:
            raise UsageError(f"duplicate profile slugs: {', '.join(duplicate)}")

    def profile(self, slug: str) -> ProfileSpec:
        for profile in self.profiles:
            if profile.slug == slug:
                return profile
        raise UsageError(f"unknown profile: {slug}")

    def root(self, kind: str) -> str | None:
        for spec in self.roots:
            if spec.kind == kind:
                return spec.path
        return None

    def fingerprint(self) -> str:
        """A digest of everything in this configuration that decides what an atlas holds.

        The output directory is not in it: where the file is written is not part of what is
        in it. The waiver files are not in it either, for the same reason and a sharper one:
        a waiver grades a finding that is already in the atlas, so a machine that carries one
        and a machine that carries none must still produce the same bytes. Sandbox selection
        belongs to run predicates rather than identity. A profile, a root, an ignore rule,
        a setup file or a timeout that moved is a different question asked of the same tree.
        """
        from rosio import ids

        return ids.digest_fields(
            {
                "deterministic": str(bool(self.deterministic)),
                "ignore": list(self.ignore),
                "launch_timeout": f"{float(self.launch_timeout):.6f}",
                "profiles": [
                    "|".join(
                        [
                            spec.slug,
                            spec.label,
                            *(f"env {key}={value}" for key, value in spec.env),
                            *(
                                f"{entry.verb} {entry.pkg} {entry.target} "
                                + " ".join(
                                    [*(f"{key}:={value}" for key, value in entry.args), *entry.argv]
                                )
                                for entry in spec.entrypoints
                            ),
                        ]
                    )
                    for spec in self.profiles
                ],
                "roots": [f"{spec.kind}={spec.path}" for spec in self.roots],
                "ros_setup": list(self.ros_setup),
            },
            length=32,
        )


# ---------------------------------------------------------------------------
# loading


def load_config(path: str | None, *, workspace: Any) -> Config:
    """Read `rosio.yaml`, or fall back to `default_config` when there is none.

    `workspace` is a directory or an already discovered Workspace. An explicit path that
    does not exist is a usage error, an implicit one that does not exist is not.
    """
    explicit = path or os.environ.get(CONFIG_ENV) or None
    if explicit:
        target = Path(explicit)
        if not target.is_file():
            raise UsageError(f"config file not found: {explicit}")
    else:
        target = _find_config(_workspace_dir(workspace))
        if target is None:
            return default_config(workspace)
    payload = _read_yaml(target)
    return _from_payload(payload, target, workspace)


def default_config(workspace: Any, packages: Sequence[str] = ()) -> Config:
    """One profile per discovered launch file, slug taken from the file.

    This is a starting point, not a description of a running system: a workspace wide union
    of launch files describes a system that never runs, which is why every fact still names
    the entry point it came from. `packages` keeps the profiles of those packages only, and
    the slugs stay the ones the whole workspace gives, so a profile keeps its name whichever
    packages were asked for.
    """
    ws = _as_workspace(workspace)
    roots = tuple(RootSpec(kind=spec.kind, path=spec.path) for spec in ws.roots)
    rows = _launch_profiles(ws)
    if packages:
        wanted = set(packages)
        missing = sorted(wanted - {entry.pkg for _slug, _label, entry in rows})
        if missing:
            raise UsageError(f"no launch file found in package: {', '.join(missing)}")
        rows = [row for row in rows if row[2].pkg in wanted]
    profiles = tuple(
        ProfileSpec(slug=slug, label=label, entrypoints=(entry,)) for slug, label, entry in rows
    )
    return Config(
        roots=roots,
        profiles=profiles,
        out_dir=DEFAULT_OUT_DIR,
        ros_setup=_default_ros_setup(ws),
    )


def with_install_tree(cfg: Config, path: str) -> Config:
    """The install tree named on the command line, in place of any the config declares.

    The command line wins over the config file, so the named tree replaces the install root
    rather than adding a second one, and the overlay setup file the launch sandbox sources
    moves with it: an install tree is where the ament index answers from, and sourcing the
    setup file of a tree that is no longer a root would answer for the wrong one. A setup
    file the config wrote by hand is left alone, because that one was a decision.
    """
    import dataclasses

    target = Path(os.path.normpath(Path(path).expanduser().absolute()))
    if not target.is_dir():
        raise UsageError(f"install tree does not exist: {target}")
    replaced = [spec.path for spec in cfg.roots if spec.kind == INSTALL]
    roots = tuple(spec for spec in cfg.roots if spec.kind != INSTALL)
    setup = str(target / "setup.bash")
    dropped = {str(Path(old) / "setup.bash") for old in replaced}
    kept = [entry for entry in cfg.ros_setup if entry not in dropped and entry != setup]
    if Path(setup).is_file():
        kept.append(setup)
    return dataclasses.replace(
        cfg,
        roots=(*roots, RootSpec(kind=INSTALL, path=str(target))),
        ros_setup=tuple(kept),
    )


def _from_payload(payload: dict[str, Any], target: Path, workspace: Any) -> Config:
    unknown = sorted(set(payload) - _TOP_KEYS)
    if unknown:
        raise UsageError(f"{target}: unknown config keys: {', '.join(unknown)}")
    base = target.parent
    roots = _roots(payload.get("roots"), base, workspace)
    ignore = _strings(payload.get("ignore"), "ignore")
    profiles = _profiles(payload.get("profiles"), base)
    if not profiles:
        ws = _as_workspace(workspace, roots=roots, ignore=ignore)
        profiles = tuple(
            ProfileSpec(slug=slug, label=label, entrypoints=(entry,))
            for slug, label, entry in _launch_profiles(ws)
        )
    # A config that never mentions setup files gets the ones the workspace implies, the distro
    # and the overlay of an install root, exactly as having no config at all does. A config that
    # states them, even as an empty list, made a decision and is taken as written.
    if "ros_setup" in payload:
        ros_setup = _strings(payload.get("ros_setup"), "ros_setup", base=base)
    else:
        ros_setup = _default_ros_setup(_as_workspace(workspace, roots=roots, ignore=ignore))
    return Config(
        roots=roots,
        profiles=profiles,
        out_dir=str(payload.get("out") or payload.get("out_dir") or DEFAULT_OUT_DIR),
        deterministic=bool(payload.get("deterministic", False)),
        launch_timeout=float(payload.get("launch_timeout", DEFAULT_LAUNCH_TIMEOUT)),
        launch_sandbox=(
            "off"
            if payload.get("launch_sandbox") is False
            else payload.get("launch_sandbox", "auto")
        ),
        ros_setup=ros_setup,
        ignore=ignore,
        waivers=_strings(payload.get("waivers"), "waivers", base=base, always=True),
    )


def _roots(raw: Any, base: Path, workspace: Any) -> tuple[RootSpec, ...]:
    if raw is None:
        return tuple(RootSpec(kind=spec.kind, path=spec.path) for spec in _default_roots(workspace))
    if not isinstance(raw, list):
        raise UsageError("roots must be a list")
    out: list[RootSpec] = []
    for item in raw:
        if isinstance(item, str):
            out.append(RootSpec(kind=records.ROOTS[0], path=_resolve(item, base)))
            continue
        if not isinstance(item, dict):
            raise UsageError("a root must be a string or a mapping")
        unknown = sorted(set(item) - _ROOT_KEYS)
        if unknown:
            raise UsageError(f"unknown root keys: {', '.join(unknown)}")
        out.append(
            RootSpec(
                kind=str(item.get("kind", records.ROOTS[0])),
                path=_resolve(str(item.get("path", "")), base),
            )
        )
    if not out:
        raise UsageError("roots is empty")
    return tuple(out)


def _profiles(raw: Any, base: Path) -> tuple[ProfileSpec, ...]:
    if raw is None:
        return ()
    if not isinstance(raw, list):
        raise UsageError("profiles must be a list")
    out: list[ProfileSpec] = []
    for item in raw:
        if not isinstance(item, dict):
            raise UsageError("a profile must be a mapping")
        unknown = sorted(set(item) - _PROFILE_KEYS)
        if unknown:
            raise UsageError(f"unknown profile keys: {', '.join(unknown)}")
        slug = str(item.get("slug", "")).strip()
        entrypoints = tuple(_entrypoint(entry, base) for entry in item.get("entrypoints") or ())
        out.append(
            ProfileSpec(
                slug=slug,
                label=str(item.get("label", "")),
                entrypoints=entrypoints,
                env=_pairs(item.get("env"), "env"),
            )
        )
    return tuple(out)


def _entrypoint(raw: Any, base: Path) -> Entrypoint:
    if isinstance(raw, str):
        return _entrypoint_from_command(raw)
    if not isinstance(raw, dict):
        raise UsageError("an entry point must be a string or a mapping")
    unknown = sorted(set(raw) - _ENTRY_KEYS)
    if unknown:
        raise UsageError(f"unknown entry point keys: {', '.join(unknown)}")
    verb = str(raw.get("verb", "")).strip()
    pkg = str(raw.get("pkg") or raw.get("package") or "").strip()
    target = str(raw.get("target", "")).strip()
    for candidate in VERBS:
        shorthand = raw.get(candidate)
        if shorthand is None:
            continue
        verb = candidate
        words = shlex.split(str(shorthand))
        if len(words) == 2:
            pkg, target = words
        elif len(words) == 1 and pkg:
            target = words[0]
        else:
            raise UsageError(f"{candidate} shorthand must be '<package> <target>'")
    if not verb:
        verb = VERBS[0]
    args = _pairs(raw.get("args"), "args")
    argv = _strings(raw.get("argv"), "argv", base=base, resolve=False)
    return Entrypoint(verb=verb, pkg=pkg, target=target, args=args, argv=argv)


def _entrypoint_from_command(command: str) -> Entrypoint:
    """`ros2 launch pkg file.launch.py k:=v` as written in a script or a unit file.

    Split the way a shell splits it: a quoted value holds one argument, and `str.split` cuts
    `path:="/a b"` into a truncated value and a stray word.
    """
    try:
        words = shlex.split(command)
    except ValueError as exc:
        raise UsageError(f"cannot read an entry point from: {command!r} ({exc})") from exc
    if len(words) >= 4 and words[0] == "ros2" and words[1] in VERBS:
        verb, pkg, target = words[1], words[2], words[3]
        tail = words[4:]
    elif len(words) >= 3 and words[0] in VERBS:
        verb, pkg, target = words[0], words[1], words[2]
        tail = words[3:]
    else:
        raise UsageError(f"cannot read an entry point from: {command!r}")
    args: list[tuple[str, str]] = []
    argv: list[str] = []
    for word in tail:
        if ":=" in word and not word.startswith("-"):
            key, _, value = word.partition(":=")
            args.append((key, value))
        else:
            argv.append(word)
    return Entrypoint(
        verb=verb,
        pkg=pkg,
        target=target,
        args=tuple(args),
        argv=tuple(argv),
    )


def _pairs(raw: Any, what: str) -> tuple[tuple[str, str], ...]:
    if raw is None:
        return ()
    if isinstance(raw, dict):
        items = [(str(key), _scalar(value)) for key, value in raw.items()]
        return tuple(sorted(items))
    if isinstance(raw, list):
        out: list[tuple[str, str]] = []
        for item in raw:
            if isinstance(item, dict) and len(item) == 1:
                key, value = next(iter(item.items()))
                out.append((str(key), _scalar(value)))
            elif isinstance(item, str) and ":=" in item:
                key, _, value = item.partition(":=")
                out.append((key, value))
            elif isinstance(item, str) and "=" in item:
                key, _, value = item.partition("=")
                out.append((key, value))
            else:
                raise UsageError(f"cannot read a {what} entry from {item!r}")
        return tuple(out)
    raise UsageError(f"{what} must be a mapping or a list")


def _strings(
    raw: Any,
    what: str,
    base: Path | None = None,
    resolve: bool = True,
    always: bool = False,
) -> tuple[str, ...]:
    if raw is None:
        return ()
    if isinstance(raw, str):
        raw = [raw]
    if not isinstance(raw, list):
        raise UsageError(f"{what} must be a string or a list of strings")
    out: list[str] = []
    for item in raw:
        text = str(item)
        if base is not None and resolve and (always or "/" in text or text.startswith(".")):
            text = _resolve(text, base)
        out.append(text)
    return tuple(out)


def _scalar(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return "" if value is None else str(value)


def _resolve(path: str, base: Path) -> str:
    if not path:
        raise UsageError("a path in the config is empty")
    candidate = Path(path)
    if not candidate.is_absolute():
        candidate = base / candidate
    return os.path.normpath(str(candidate))


def _read_yaml(target: Path) -> dict[str, Any]:
    try:
        payload = yaml.safe_load(target.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise UsageError(f"{target}: cannot read the config: {exc}") from exc
    if payload is None:
        return {}
    if not isinstance(payload, dict):
        raise UsageError(f"{target}: the config root must be a mapping")
    return payload


def _find_config(start: Path | None) -> Path | None:
    if start is None:
        return None
    for directory in (start, start.parent):
        candidate = directory / CONFIG_NAME
        if candidate.is_file():
            return candidate
    return None


# ---------------------------------------------------------------------------
# waivers


def waiver_files(config_path: str | None, start: Path | None = None) -> tuple[str, ...]:
    """The waiver files a configuration names, read without building a run configuration.

    A gate verb reads an atlas off disk and has no workspace to discover, so it must not pay
    for package discovery to find out which findings this repository has accepted.
    """
    explicit = config_path or os.environ.get(CONFIG_ENV) or None
    if explicit:
        target = Path(explicit)
        if not target.is_file():
            raise UsageError(f"config file not found: {explicit}")
    else:
        target = _find_config(start or Path.cwd())
        if target is None:
            return ()
    payload = _read_yaml(target)
    unknown = sorted(set(payload) - _TOP_KEYS)
    if unknown:
        raise UsageError(f"{target}: unknown config keys: {', '.join(unknown)}")
    return _strings(payload.get("waivers"), "waivers", base=target.parent, always=True)


def load_waivers(paths: Sequence[str]) -> tuple[Waiver, ...]:
    """Every waiver the named files hold, sorted by key.

    One key is stated once. Two files that accept the same finding for two reasons are a
    question about which reason is true, and picking one of them by file order would answer
    it silently.
    """
    seen: dict[str, tuple[str, str]] = {}
    for name in paths:
        target = Path(name).expanduser()
        if not target.is_file():
            raise UsageError(f"waiver file not found: {name}")
        for waiver in _waivers_of(_read_yaml(target), target):
            first = seen.get(waiver.key)
            if first is not None and first[1] != waiver.reason:
                raise UsageError(
                    f"{target}: waiver {waiver.key} is already stated in {first[0]} "
                    "with another reason, so state it once"
                )
            seen[waiver.key] = (str(target), waiver.reason)
    return tuple(Waiver(key=key, reason=seen[key][1]) for key in sorted(seen))


def _waivers_of(payload: dict[str, Any], target: Path) -> list[Waiver]:
    unknown = sorted(set(payload) - {"waivers"})
    if unknown:
        raise UsageError(f"{target}: unknown waiver file keys: {', '.join(unknown)}")
    raw = payload.get("waivers")
    if raw is None:
        return []
    if not isinstance(raw, list):
        raise UsageError(f"{target}: waivers must be a list")
    out: list[Waiver] = []
    for item in raw:
        if not isinstance(item, dict):
            raise UsageError(f"{target}: a waiver must be a mapping with a key and a reason")
        extra = sorted(set(item) - _WAIVER_KEYS)
        if extra:
            raise UsageError(f"{target}: unknown waiver keys: {', '.join(extra)}")
        out.append(Waiver(key=str(item.get("key", "")), reason=str(item.get("reason", ""))))
    return out


# ---------------------------------------------------------------------------
# workspace plumbing


def _workspace_dir(workspace: Any) -> Path | None:
    if isinstance(workspace, Workspace):
        return Path(workspace.roots[0].path) if workspace.roots else None
    if workspace is None:
        return None
    return Path(str(workspace))


def _as_workspace(
    workspace: Any,
    roots: tuple[RootSpec, ...] = (),
    ignore: tuple[str, ...] = (),
) -> Workspace:
    if isinstance(workspace, Workspace):
        return workspace
    return Workspace.discover(roots or _default_roots(workspace), ignore)


def _default_roots(workspace: Any) -> tuple[RootSpec, ...]:
    """src plus the install tree next to it, which is what a colcon workspace looks like.

    A directory that is itself an ament prefix (`/opt/ros/<distro>`, a colcon `install/`,
    merged or isolated) is one install root, and so is a workspace holding only `install/`.
    Packages beside that `install/` make the directory the src root as well.
    The merged index is asked first, because `/opt/ros/<distro>` also holds a `src`.
    """
    if isinstance(workspace, Workspace):
        return tuple(RootSpec(kind=spec.kind, path=spec.path) for spec in workspace.roots)
    if workspace is None:
        raise UsageError("a workspace path is needed to build a default config")
    directory = Path(str(workspace)).absolute()
    if not directory.is_dir():
        raise UsageError(f"workspace directory does not exist: {directory}")
    if (directory / AMENT_INDEX).is_dir():
        return (RootSpec(kind=INSTALL, path=str(directory)),)
    out: list[RootSpec] = []
    src = directory / "src"
    install = directory / "install"
    if src.is_dir():
        out.append(RootSpec(kind="src", path=str(src)))
    elif not _is_install_tree(install):
        if _is_install_tree(directory):
            return (RootSpec(kind=INSTALL, path=str(directory)),)
        out.append(RootSpec(kind="src", path=str(directory)))
        install = directory.parent / "install"
    elif _holds_package(directory):
        out.append(RootSpec(kind="src", path=str(directory)))
    if install.is_dir():
        out.append(RootSpec(kind="install", path=str(install)))
    return tuple(out)


def _holds_package(directory: Path) -> bool:
    """A package manifest anywhere a source walk would reach, so not under install or build."""
    for _, dirs, files in os.walk(directory):
        if "package.xml" in files:
            return True
        dirs[:] = [name for name in dirs if name not in SRC_SKIP_DIRS]
    return False


def _is_install_tree(path: Path) -> bool:
    """An ament prefix: merged holds one index, an isolated colcon install one per package."""
    if (path / AMENT_INDEX).is_dir():
        return True
    return path.is_dir() and any((child / AMENT_INDEX).is_dir() for child in path.iterdir())


def _default_ros_setup(ws: Workspace) -> tuple[str, ...]:
    """The shell files a child process has to source. Machine dependent by nature, so the
    run record carries them and two boxes that differ show an input difference."""
    out: list[str] = []
    distro = os.environ.get("ROS_DISTRO", "")
    if distro:
        candidate = Path("/opt/ros") / distro / "setup.bash"
        if candidate.is_file():
            out.append(str(candidate))
    install = next((spec.path for spec in ws.roots if spec.kind == "install"), "")
    if install:
        overlay = Path(install) / "setup.bash"
        if overlay.is_file() and str(overlay) not in out:
            out.append(str(overlay))
    return tuple(out)


def _launch_profiles(ws: Workspace) -> list[tuple[str, str, Entrypoint]]:
    """One entry per launch file, with a slug that stays stable when a file is added.

    The bare file stem is used when it is unique in the workspace, and the package is
    prefixed when it is not, so two `mapping.launch.py` files never collide and neither slug
    carries an ordinal.
    """
    rows: list[tuple[str, str, str]] = []
    for path in ws.launch_files():
        package, relative = ws.rel(path)
        if package is None and ws.root_of(path) == INSTALL:
            # Outside every package share, so no `ros2 launch` reaches it.
            continue
        rows.append((package or "repo", relative, path))
    out: list[tuple[str, str, Entrypoint]] = []
    for package, relative, path in sorted(rows):
        target = relative.split("/", 1)[1] if relative.startswith("launch/") else relative
        entry = Entrypoint(
            verb="launch",
            pkg=package,
            target=target,
            origin=ws.site(path),
        )
        out.append((_unique_slug(package, relative, rows), f"{package} {target}", entry))
    return sorted(out, key=lambda row: row[0])


def _slug_candidates(package: str, relative: str) -> tuple[str, ...]:
    stem = _slug_for(relative)
    flat = re.sub(r"[^A-Za-z0-9]+", "_", relative.removeprefix("launch/")).strip("_")
    return (stem, f"{package}.{stem}", f"{package}.{flat}")


def _unique_slug(package: str, relative: str, rows: Sequence[tuple[str, str, str]]) -> str:
    """The shortest candidate that names exactly one launch file in this workspace."""
    everyone = [_slug_candidates(pkg, rel) for pkg, rel, _path in rows]
    mine = _slug_candidates(package, relative)
    for index, candidate in enumerate(mine):
        if sum(1 for level in everyone if level[index] == candidate) == 1:
            return candidate
    return mine[-1]


def _slug_for(relative: str) -> str:
    name = relative.rsplit("/", 1)[-1]
    for suffix in sorted(LAUNCH_SUFFIXES, key=len, reverse=True):
        if name.endswith(suffix):
            return name[: -len(suffix)]
    return Path(name).stem


__all__ = [
    "CONFIG_ENV",
    "CONFIG_NAME",
    "DEFAULT_LAUNCH_TIMEOUT",
    "DEFAULT_OUT_DIR",
    "VERBS",
    "WAIVERS_NAME",
    "WAIVER_PREFIXES",
    "Config",
    "Entrypoint",
    "ProfileSpec",
    "RootSpec",
    "Waiver",
    "default_config",
    "load_config",
    "load_waivers",
    "waiver_files",
    "with_install_tree",
]
