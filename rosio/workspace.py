"""Package discovery over the configured roots.

One workspace object answers every "where does this file live" question for the whole run:
which package owns a path, which tree it came from, what a stable id for it is, and which
files are worth reading at all. Every layer uses these answers, so a path is classified
once and the same way everywhere.

Three properties of the reference stack shape this module:

* The install tree can be a copy of src or a tree of symlinks into it, and it can be older
  than the sources. A file in the install share and its source are therefore the same file
  id with a different root, so a stale build shows up as one staleness finding instead of
  dozens of content drifts, and a selection prefers src because that is the editable tree.
* `get_package_share_directory` reads the install tree, so `share` prefers install and
  falls back to src, and records which one it used. `cite` is the other half of that: a
  launch file that resolved a config path through the package share read the install copy,
  and the file a reader can edit is the src copy at the same package relative path. Equal
  bytes are cited as src, differing bytes stay cited as install and are recorded.
* Not every yaml is a parameter file and not every flat yaml is not one, so parameter file
  detection is shape based: a `ros__parameters` key somewhere under a mapping root.

Root membership is decided by real path. A launch file that self locates with
`os.path.realpath(__file__)` names the canonical path of a root that was configured through a
symlink, and a textual comparison then puts its own config directory outside the workspace,
which writes a machine path into compared data and drops the provenance of everything under
it. `localise` is the one place that translation happens.
"""

from __future__ import annotations

import hashlib
import os
import re
import tempfile
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from fnmatch import fnmatch
from pathlib import Path
from typing import TYPE_CHECKING, Any
from xml.etree import ElementTree

import yaml

from rosio import ids, model
from rosio.errors import CollectorError, ContractError, UsageError
from rosio.resolve import records

if TYPE_CHECKING:
    from rosio.config import RootSpec

SRC_SKIP_DIRS = frozenset(
    {
        ".git",
        ".hg",
        ".idea",
        ".mypy_cache",
        ".pytest_cache",
        ".ruff_cache",
        ".svn",
        ".tox",
        ".venv",
        "__pycache__",
        "build",
        "install",
        "log",
        "node_modules",
        "venv",
    }
)
"""Never walked as source. `install` is walked only when it is itself an install root."""

ANY_SKIP_DIRS = frozenset({".git", "__pycache__"})

IGNORE_MARKERS = ("COLCON_IGNORE", "AMENT_IGNORE", "CATKIN_IGNORE")

LAUNCH_SUFFIXES = (
    ".launch.py",
    "_launch.py",
    ".launch.xml",
    ".launch.yaml",
    ".launch.yml",
    ".launch",
)
"""Both spellings are in live use, and the bare `.launch` files are ROS 1 XML that a
resolver has to recognise rather than fail on."""

PARAM_YAML_SUFFIXES = (".yaml", ".yml")

PARAM_KEY = "ros__parameters"

MAX_YAML_BYTES = 4 * 1024 * 1024

CONTENT_SUFFIXES = frozenset(
    {
        ".action",
        ".bash",
        ".c",
        ".cc",
        ".cfg",
        ".cmake",
        ".cpp",
        ".cxx",
        ".dsv",
        ".egg-link",
        ".h",
        ".hpp",
        ".ini",
        ".json",
        ".launch",
        ".md",
        ".msg",
        ".py",
        ".rviz",
        ".sdf",
        ".sh",
        ".srv",
        ".toml",
        ".txt",
        ".urdf",
        ".xacro",
        ".xml",
        ".yaml",
        ".yml",
        ".zsh",
    }
)
"""Hashed by content for the root hash, with this machine's own root prefixes normalised out
first, see `Workspace.content_marks`. Anything else contributes its path and size, so a
recorded bag or a vendored tarball does not have to be read to fingerprint a root. A file
whose whole content is an absolute path has to be in here, because its size then carries the
length of that path: `colcon build --symlink-install` writes one `.egg-link` per python
package holding the build directory and nothing else."""

CONTENT_NAMES = frozenset({"CMakeLists.txt", "package.xml", "setup.cfg", "setup.py"})

OUTSIDE = "@outside"
"""Pseudo root of a file that sits outside every configured root and outside the checkout,
the way `ids.REPO_PKG` is the pseudo package of a file that belongs to no ament package. See
`Workspace.stated_by_content`."""


@dataclass(frozen=True)
class Package:
    name: str
    src_path: str | None
    share_path: str | None
    build_type: str
    executables: tuple[str, ...]
    plugin_manifests: tuple[str, ...]

    @property
    def anchor(self) -> str:
        """The tree a reader should edit: src when it exists, else the install share."""
        path = self.src_path or self.share_path
        if path is None:
            raise ContractError(f"package {self.name} has no path")
        return path


@dataclass(frozen=True)
class ShareRead:
    """A file reached through the package share that also exists under src at the same
    package relative path. `identical` false is the build staleness case: the launch system
    read bytes that no longer match the editable file."""

    file_id: str
    src_path: str
    install_path: str
    identical: bool


class Workspace:
    """The discovered roots, packages and file index. Read only after construction, apart
    from the caches and the share read log, which are functions of the files on disk."""

    def __init__(
        self,
        roots: Sequence[RootSpec],
        packages: dict[str, Package],
        files: Sequence[str],
        repo_root: str,
        ignore: Sequence[str] = (),
        undecodable: Sequence[str] = (),
    ) -> None:
        self.roots = tuple(roots)
        self.packages = dict(sorted(packages.items()))
        self.repo_root = repo_root
        self.ignore = tuple(ignore)
        self._files = tuple(sorted(files))
        self._undecodable = tuple(sorted(undecodable))
        self._roots_by_depth = sorted(
            ((str(root.path), str(root.kind)) for root in self.roots),
            key=lambda item: (-len(item[0]), item[0]),
        )
        self._anchors = self._build_anchors()
        self._root_hashes: dict[str, str] | None = None
        self._param_files: tuple[str, ...] | None = None
        self._selections: dict[Any, tuple[str, ...]] = {}
        self._twins: dict[str, str | None] = {}
        self._share_reads: dict[str, ShareRead] = {}
        self._real_roots: list[tuple[str, str]] | None = None
        self._localised: dict[str, str] = {}
        self._rels: dict[str, tuple[str | None, str]] = {}
        self._marks: list[tuple[bytes, bytes]] | None = None
        self._by_name: dict[str, tuple[str, ...]] | None = None

    # ------------------------------------------------------------------
    # discovery

    @classmethod
    def discover(cls, roots: Sequence[RootSpec], ignore: Sequence[str] = ()) -> Workspace:
        """Index every root, then join what src and install say about each package."""
        if not roots:
            raise UsageError("workspace discovery needs at least one root")
        normalised = [_normalise_root(root) for root in roots]
        files: list[str] = []
        undecodable: list[str] = []
        manifests: list[tuple[int, str, str]] = []
        for order, (kind, path) in enumerate(normalised):
            if not Path(path).is_dir():
                raise UsageError(f"{kind} root does not exist: {path}")
            for found in _walk(path, kind, ignore):
                if not _nameable(found):
                    undecodable.append(found)
                    continue
                files.append(found)
                if Path(found).name == "package.xml":
                    manifests.append((order, kind, found))
        repo_root = _repo_root([path for _kind, path in normalised])
        packages = _packages(manifests, files)
        specs = [_RootSpec(kind=kind, path=path) for kind, path in normalised]
        return cls(specs, packages, files, repo_root, ignore, undecodable)

    # ------------------------------------------------------------------
    # paths and identity

    def files(self) -> tuple[str, ...]:
        """Every indexed file, sorted, absolute."""
        return self._files

    def named(self, name: str) -> tuple[str, ...]:
        """Every indexed file called `name`, sorted, from one index built on first use, so the
        ignore rules hold and a hundred lookups cost one pass over the index."""
        if self._by_name is None:
            index: dict[str, list[str]] = {}
            for path in self._files:
                index.setdefault(Path(path).name, []).append(path)
            self._by_name = {key: tuple(value) for key, value in index.items()}
        return self._by_name.get(name, ())

    def undecodable(self) -> list[dict[str, str]]:
        """One row per file the walk found and could not name, ready for a run record.

        A byte sequence on a filesystem is not a promise of valid text, and every id, every
        digest and every evidence row in this tool is text. Such a file is therefore out of
        the index, and it is named here with its bytes percent escaped so it stays greppable
        and so no collector that reads a root can present the index as complete.
        """
        rows = [
            {
                "path": self._name_of(path),
                "phase": "walk",
                "exc_type": "UnicodeDecodeError",
                "message": "the file name is not valid utf-8, so the file is not in the index",
            }
            for path in self._undecodable
        ]
        return sorted(rows, key=lambda row: row["path"])

    def undecodable_owners(self) -> list[tuple[str, str]]:
        """(owning package id, escaped name) per file the walk could not name."""
        out = [
            (ids.build_pkg(self.package_of(path) or ids.REPO_PKG), self._name_of(path))
            for path in self._undecodable
        ]
        return sorted(set(out))

    def _name_of(self, path: str) -> str:
        """The package relative spelling of a path, bytes escaped, never a machine path."""
        try:
            package, relative = self.rel(path)
        except ContractError:
            package, relative = None, Path(path).name
        return _spell(f"{package or ids.REPO_PKG}/{relative}")

    def package_of(self, path: str) -> str | None:
        anchor = self._anchor_for(path)
        return None if anchor is None else anchor[1]

    def localise(self, path: str) -> str:
        """The configured spelling of a path that names workspace content.

        A path that is already under a configured root, or under the repository root, is
        returned unchanged. Anything else is compared again by real path, so content reached
        through a symlinked or otherwise aliased root resolves inside the workspace and its
        provenance survives. A path that is genuinely outside comes back as it went in.
        """
        found = self._localised.get(path)
        if found is not None:
            return found
        absolute = _abs(path)
        translated = absolute if self._textually_inside(absolute) else self._translate(absolute)
        self._localised[path] = translated
        return translated

    def inside(self, path: str) -> bool:
        """Whether a path names content of a configured root or of the repository."""
        return self._textually_inside(self.localise(path))

    def in_repo(self, path: str) -> str:
        """Where a path sits in the checkout, or an empty string when it is outside it.

        The repository root itself is `.`, and never its directory name: a clone is free to
        sit in a directory of any name, so the name is a property of one machine.

        A checkout is asked for by name here rather than taken from `repo_root`, which falls
        back to the parent of the roots when nothing under them holds a `.git`. That fallback
        is right for a `file:@repo` id, which only has to be stable inside one atlas, and
        wrong for this answer: a root at `/opt/ros/humble` with no repository anywhere would
        be stated as sitting at `humble` in a checkout that does not exist.
        """
        if not self.checkout:
            return ""
        absolute = self.localise(path)
        if not _under(absolute, self.repo_root):
            return ""
        return "." if absolute == self.repo_root else _relative(absolute, self.repo_root)

    @property
    def checkout(self) -> str:
        """The repository root when one is really there, empty when nothing is checked out."""
        return self.repo_root if self.repo_root and Path(self.repo_root, ".git").exists() else ""

    def stated(self, paths: Iterable[str]) -> list[str]:
        """Paths as an atlas may state them, in the order they were given.

        One inside the checkout becomes the place it holds there, one outside it keeps its
        path, which is a difference between two machines and not a property of the checkout.
        Anything a compared value or a digest is built from goes through here, or one
        workspace at two paths is two atlases and every clone rewrites the committed file.
        """
        return [self.in_repo(path) or path for path in paths]

    def stated_by_content(self, paths: Iterable[str]) -> list[str]:
        """Paths as compared data may state them, including the ones outside the checkout.

        `stated` answers for a path inside the checkout and hands back a machine path for
        anything else, which is right where the difference between two machines is the
        answer, and wrong for the setup files of the ROS installation. Those name the same
        distribution on two boxes and the boxes mount it at two prefixes, so an input digest
        taken over the raw path gave identical sources two run ids and two atlases, which is
        the same failure `stated` closed one level in.

        A file outside every root is therefore stated by its name and the hash of its
        content, with the prefix it sits under normalised out first, the way
        `content_marks` already does it for a root: two machines match, and a setup file
        that gained a package, an overlay or a hand written line still moves the mark, so
        the gate the digest is still notices a real change. A path that names nothing
        readable says so rather than borrowing an identity.
        """
        return [self.in_repo(path) or self._by_content(path) for path in paths]

    def stated_value(self, value: Any, ros_setup: Sequence[str] = ()) -> Any:
        """A value as compared data may carry it, with every place this machine holds a root
        named by that root's label, `@install/share/bot/urdf/bot.xacro` for one.

        A value is what a node receives or what a tool said, so the text around the prefix is
        kept and only the mount point changes, by the same table `content_marks` hashes with.
        The directory of a setup file in `ros_setup` becomes `@outside`, as `stated_by_content`
        hashes it, unless it is a root. Any other path outside every root keeps its spelling,
        as `stated` keeps it.
        """
        marks = dict(row for setup in ros_setup for row in _prefix_marks(str(Path(setup).parent)))
        marks.update(self.content_marks())
        rows = [
            (path.decode("utf-8", "surrogateescape"), mark.strip(b"\x00").decode())
            for path, mark in sorted(marks.items(), key=lambda item: (-len(item[0]), item[0]))
        ]
        return _replaced(value, rows)

    def stated_path(self, path: str, ros_setup: Sequence[str] = ()) -> str:
        """A path a message or a blind spot names, as compared data may carry it.

        Inside a root it is `<package>/<relative path>`. Under the directory of a setup file
        in `ros_setup` it is the rest of the path below `@outside`, which names the same file
        on every box that holds the distribution. Anywhere else it is `@outside/<name>`,
        because the directories around it are a property of one machine.
        """
        try:
            package, relative = self.rel(path)
        except ContractError:
            stated = self.stated_value(path, ros_setup)
            return stated if stated != path else _outside(path)
        return f"{package}/{relative}" if package else relative

    def stated_message(self, text: str, ros_setup: Sequence[str] = ()) -> str:
        """Text a tool said, with every absolute path in it stated by `stated_path`.

        A ROS name starts with a slash too, so `_names_a_file` decides which tokens are paths.
        `/cmd_vel` stays, `/somewhere/else/x.yaml` becomes `@outside/x.yaml`. A path inside a
        root is named by its package first, `bot/config/x.yaml`, because the root label of
        its mount point (`@src/ws/bot/config/x.yaml`) reads as a file nobody can open.
        """

        def packaged(match: re.Match[str]) -> str:
            token = match.group(0).rstrip(".")
            try:
                package, relative = self.rel(token)
            except ContractError:
                return match.group(0)
            if not package:
                return match.group(0)
            return f"{package}/{relative}" + match.group(0)[len(token) :]

        text = self.stated_value(_ABSOLUTE.sub(packaged, text), ros_setup)

        def stated(match: re.Match[str]) -> str:
            token = match.group(0).rstrip(".")
            quoted = _PYTHON_QUOTED.search(text[max(0, match.start() - 3) : match.start()])
            if not _names_a_file(token, quoted is not None):
                return match.group(0)
            return self.stated_path(token, ros_setup) + match.group(0)[len(token) :]

        return _ABSOLUTE.sub(stated, text)

    def _by_content(self, path: str) -> str:
        absolute = self.localise(path)
        target = Path(absolute)
        marks = [
            *self.content_marks(),
            *_prefix_marks(str(target.parent)),
        ]
        fingerprint = _file_fingerprint(absolute, marks)
        if fingerprint == b"missing":
            return f"{OUTSIDE}/{_spell(target.name)}#absent"
        return f"{OUTSIDE}/{_spell(target.name)}#{fingerprint.hex()[: ids.DIGEST_LENGTH]}"

    def aliased(self, path: str) -> bool:
        """Whether a path reaches workspace content through a spelling no root matches."""
        return self.localise(path) != _abs(path)

    def _textually_inside(self, absolute: str) -> bool:
        if any(_under(absolute, root_path) for root_path, _kind in self._roots_by_depth):
            return True
        return bool(self.repo_root) and _under(absolute, self.repo_root)

    def _translate(self, absolute: str) -> str:
        real = os.path.realpath(absolute)
        for base, real_base in self._real_bases():
            if _under(real, real_base):
                if real == real_base:
                    return base
                return os.path.normpath(str(Path(base) / _relative(real, real_base)))
        return absolute

    def _real_bases(self) -> list[tuple[str, str]]:
        if self._real_roots is None:
            bases = [root_path for root_path, _kind in self._roots_by_depth]
            if self.repo_root:
                bases.append(self.repo_root)
            self._real_roots = sorted(
                ((base, os.path.realpath(base)) for base in bases),
                key=lambda item: (-len(item[1]), item[1]),
            )
        return self._real_roots

    def rel(self, path: str) -> tuple[str | None, str]:
        """(package name or None, path relative to the package or to the repository)."""
        found = self._rels.get(path)
        if found is None:
            found = self._rels[path] = self._rel(path)
        return found

    def _rel(self, path: str) -> tuple[str | None, str]:
        absolute = self.localise(path)
        anchor = self._anchor_for(absolute)
        if anchor is not None:
            base, name = anchor
            return name, _relative(absolute, base)
        if _under(absolute, self.repo_root):
            return None, _relative(absolute, self.repo_root)
        for root_path, _kind in self._roots_by_depth:
            if _under(absolute, root_path):
                return None, _relative(absolute, root_path)
        raise ContractError(f"path is outside every configured root: {_outside(path)}")

    def file_id(self, path: str) -> str:
        package, relative = self.rel(path)
        return ids.build_file(package or ids.REPO_PKG, relative)

    def _bundled_files(self) -> str:
        """The `files/` root of an opened bundle, or nothing outside one.

        A bundle stores repository files under `files/` beside its `manifest.json`, while the
        ids in its atlas are relative to the sender's checkout, so the root stands in for it.
        """
        for root in self.roots:
            candidate = Path(root.path)
            if candidate.name == "files" and (candidate.parent / "manifest.json").is_file():
                return str(candidate)
        return ""

    def abspath(self, file_id: str) -> str:
        """Inverse of file_id, for a collector that has provenance and wants the bytes."""
        parsed = ids.parse_file(file_id)
        if parsed.package == ids.REPO_PKG:
            return str(Path(self._bundled_files() or self.repo_root) / parsed.relpath)
        package = self.packages.get(parsed.package)
        if package is None:
            raise ContractError(f"unknown package in file id: {file_id}")
        for base in (package.src_path, package.share_path):
            if base is None:
                continue
            candidate = Path(base) / parsed.relpath
            if candidate.exists():
                return str(candidate)
        return str(Path(package.anchor) / parsed.relpath)

    def root_of(self, path: str) -> str:
        """Which tree a path came from. The install tree here is a copy of src, so a fact
        that does not say which tree it read cannot be trusted about content."""
        absolute = self.localise(path)
        for root_path, kind in self._roots_by_depth:
            if _under(absolute, root_path):
                return kind
        return records.ROOTS[0]

    def site(self, path: str, locator: str = "", line_hint: int | None = None) -> records.Site:
        return records.Site(
            path=self.localise(path),
            locator=locator,
            line_hint=line_hint,
            root=self.root_of(path),
        )

    def source(self, site: records.Site) -> model.Source:
        """Provenance row for a site. The atlas cites file ids, never machine paths."""
        cited, root = self.cite(site.path, site.root)
        return model.Source(
            file=self.file_id(cited),
            locator=site.locator,
            line_hint=site.line_hint,
            root=root,
        )

    def token(self, value: str) -> str:
        """One argument or path as compared data, never a machine path.

        A value that names workspace content becomes its file id, a value that names a
        directory inside the workspace becomes its repository relative path, and anything
        else is passed through. An input digest taken over the raw value gives the same
        repository two run ids when it is mounted somewhere else.
        """
        text = str(value)
        if not text.startswith("/"):
            return text
        local = self.localise(text)
        if not self._textually_inside(local):
            return text
        try:
            if Path(local).is_dir():
                package, relative = self.rel(local)
                return f"{package}/{relative}" if package else relative
            return self.file_id(local)
        except ContractError:
            return text

    def src_twin(self, path: str) -> str | None:
        """The src file at the same package relative path as an install path, or None.

        Same path is evidence and not proof that it is the file the installed copy was built
        from. A package that installs a message out of a second directory keeps a different
        file of that name under src, so equal bytes are what makes the two interchangeable
        and differing bytes are reported as a measurement, not as a cause.
        """
        absolute = self.localise(path)
        if absolute in self._twins:
            return self._twins[absolute]
        twin: str | None = None
        if self.root_of(absolute) == records.ROOTS[1]:
            name, relative = self.rel(absolute)
            package = self.packages.get(name or "")
            if package is not None and package.src_path:
                candidate = Path(package.src_path) / relative
                if candidate.is_file():
                    twin = str(candidate)
        self._twins[absolute] = twin
        return twin

    def cite(self, path: str, root: str = "") -> tuple[str, str]:
        """Which path an evidence row should name, and which tree that content came from.

        A launch file resolves a config path through the package share, so the read happened
        in the install tree while the file a reader can edit is the src copy. Equal bytes
        make the two interchangeable, so the citation moves to src. Differing bytes do not:
        the citation stays on the tree that was actually read, and the pair is logged so the
        staleness is reported once instead of surfacing as content drift on every value.

        Between src and install the path decides, not the caller: a site built with the
        default root and an install path is claiming a tree it did not read. A runtime or bag
        root names something that is in no tree at all, so it is passed through.
        """
        absolute = self.localise(path)
        if root in records.ROOTS[2:]:
            return absolute, root
        twin = self.src_twin(absolute)
        if twin is None:
            return absolute, self.root_of(absolute)
        file_id = self.file_id(twin)
        read = self._share_reads.get(file_id)
        if read is None:
            read = ShareRead(
                file_id=file_id,
                src_path=twin,
                install_path=absolute,
                identical=_same_bytes(twin, absolute),
            )
            self._share_reads[file_id] = read
        identical = read.identical
        if identical:
            return twin, records.ROOTS[0]
        return absolute, records.ROOTS[1]

    def share_reads(self) -> list[ShareRead]:
        """Every file cited so far that was reached through the package share, sorted."""
        return [self._share_reads[key] for key in sorted(self._share_reads)]

    def install_unnamed(self) -> bool:
        """Whether a colcon install tree sits beside a src root and no root names it, so a
        build the person ran is not what this run reads."""
        return bool(self._unnamed_installs())

    def _unnamed_installs(self) -> list[Path]:
        named = {Path(spec.path).resolve() for spec in self.roots}
        index = Path("share", "ament_index", "resource_index", "packages")
        out: list[Path] = []
        for spec in self.roots:
            if spec.kind != records.ROOTS[0]:
                continue
            install = Path(spec.path).resolve().parent / "install"
            if install in named or not install.is_dir():
                continue
            if (install / index).is_dir() or any(
                (child / index).is_dir() for child in install.iterdir() if child.is_dir()
            ):
                out.append(install)
        return out

    def unnamed_build(self, share: str) -> str:
        """How the install tree beside a src root that no root names holds a share directory:
        `symlinked` when a `--symlink-install` build links files of it back to a src root,
        `copied` when it holds them as copies, and empty when the share is not in such a tree.
        """
        directory = Path(share).resolve()
        if not any(directory.is_relative_to(install) for install in self._unnamed_installs()):
            return ""
        sources = [
            Path(spec.path).resolve() for spec in self.roots if spec.kind == records.ROOTS[0]
        ]
        for parent, _dirs, files in os.walk(directory):
            for name in files:
                path = Path(parent, name)
                if path.is_symlink() and any(
                    path.resolve().is_relative_to(source) for source in sources
                ):
                    return "symlinked"
        return "copied"

    def underlay_twin(self, path: str) -> str | None:
        """The workspace file a share file outside every root stands for: the same package
        relative path in the share the install root holds, else in the package source.
        `ros2 launch` reads the released copy until the workspace is built and sourced, and
        the workspace file is the one to edit."""
        if self.inside(path):
            return None
        parts = Path(path).parts
        for index in range(len(parts) - 2, 0, -1):
            if parts[index] != "share":
                continue
            package = self.packages.get(parts[index + 1])
            base = package and (package.share_path or package.src_path)
            if not base:
                return None
            twin = Path(base, *parts[index + 2 :])
            return str(twin) if twin.is_file() else None
        return None

    def share(self, pkg: str) -> str | None:
        """Share directory, install first then src, because get_package_share_directory
        reads the install tree and that is what a launch file will have opened."""
        package = self.packages.get(pkg)
        if package is None:
            return None
        return package.share_path or package.src_path

    def share_root(self, pkg: str) -> str | None:
        """Which root `share` returned, so a stale copy stays attributable."""
        package = self.packages.get(pkg)
        if package is None:
            return None
        if package.share_path:
            return self.root_of(package.share_path)
        if package.src_path:
            return self.root_of(package.src_path)
        return None

    # ------------------------------------------------------------------
    # file selections

    def launch_files(self) -> list[str]:
        return self._select("launch", lambda path: _is_launch(Path(path).name))

    def param_files(self) -> list[str]:
        """Shape based: a mapping root with a `ros__parameters` key somewhere below it.

        Extension based detection is wrong in both directions on the reference stack. A map
        descriptor, a doc builder list and a ros1 to ros2 name table are yaml and are not
        parameter files, while three flat driver configs read like parameter files and carry
        no node key at all.
        """
        if self._param_files is None:
            self._param_files = tuple(
                path
                for path in self._files
                if Path(path).suffix in PARAM_YAML_SUFFIXES and _is_param_file(path)
            )
        shaped = frozenset(self._param_files)
        return self._select("param", lambda path: path in shaped)

    def source_files(self, exts: Sequence[str]) -> list[str]:
        wanted = {ext if ext.startswith(".") else f".{ext}" for ext in exts}
        return self._select(("source", *sorted(wanted)), lambda path: Path(path).suffix in wanted)

    def content_marks(self) -> list[tuple[bytes, bytes]]:
        """Where this machine holds the workspace, and what a hash calls those places.

        A build tool writes the absolute prefix of the machine it ran on into its own setup
        files, and those files sit inside a root, so two clones of one checkout that both
        built hash their install root differently and produce two atlases. The prefix is the
        one part of such a file that is a property of the machine rather than of the tree, so
        it is replaced by the root label before the bytes are hashed, which is the same
        translation `stated` and `token` already apply to every path that reaches compared
        data. Longest first, so a root nested inside the checkout wins over the checkout, and
        both the configured spelling and its real path are covered, because a root reached
        through a symlink is named both ways by the files under it.
        """
        if self._marks is None:
            labels = _root_labels(self._roots_by_depth)
            named: dict[str, bytes] = {
                root_path: f"\x00@{labels[root_path]}\x00".encode()
                for root_path, _kind in self._roots_by_depth
            }
            if self.repo_root:
                named.setdefault(self.repo_root, b"\x00@checkout\x00")
            rows: set[tuple[bytes, bytes]] = set()
            for path, mark in named.items():
                rows.add((path.encode("utf-8", "surrogateescape"), mark))
                rows.add((os.path.realpath(path).encode("utf-8", "surrogateescape"), mark))
            self._marks = sorted(rows, key=lambda item: (-len(item[0]), item[0]))
        return list(self._marks)

    def root_hashes(self) -> dict[str, str]:
        """One hash per root, so a run record pins the inputs it actually read.

        Every name under the root is in it, and the content of every file worth reading, with
        this machine's own prefixes taken out by `content_marks`. A file that changes for a
        reason still moves the hash: only the substring that says where the tree is mounted
        is normalised, and a setup file that gains a package, an overlay outside the
        workspace or a line somebody wrote moves it exactly as before.
        """
        if self._root_hashes is not None:
            return dict(self._root_hashes)
        labels = _root_labels(self._roots_by_depth)
        marks = self.content_marks()
        out: dict[str, str] = {}
        for root_path, _kind in self._roots_by_depth:
            digest = hashlib.sha256()
            # A name the index refused still counts as an input: dropping it here would let
            # a root change without the run saying so.
            for path in sorted([*self._files, *self._undecodable]):
                if not _under(path, root_path):
                    continue
                digest.update(_relative(path, root_path).encode("utf-8", "surrogateescape"))
                digest.update(b"\x00")
                digest.update(_file_fingerprint(path, marks))
                digest.update(b"\x00")
            out[labels[root_path]] = digest.hexdigest()[: ids.DIGEST_LENGTH]
        self._root_hashes = dict(sorted(out.items()))
        return dict(self._root_hashes)

    def root_records(self) -> list[dict[str, str]]:
        """One row per root, stating what a second machine can match it by.

        The kind, the label the run records key on, the hash of the content, and where the
        root sits in the checkout when it sits in it at all. Where it sits on this machine is
        not in it: an atlas is committed, and a file that names the absolute path of the
        workspace it was built in is rewritten by every clone on its first rebuild.
        """
        labels = _root_labels(self._roots_by_depth)
        hashes = self.root_hashes()
        rows: list[dict[str, str]] = []
        for root in self.roots:
            label = labels[str(root.path)]
            row = {"kind": str(root.kind), "label": label, "hash": hashes.get(label, "")}
            where = self.in_repo(str(root.path))
            if where:
                row["path"] = where
            rows.append(row)
        return sorted(rows, key=lambda item: (item["kind"], item["label"]))

    def package_records(self) -> dict[str, dict[str, str]]:
        """Where each package sits, per root label, relative to that root.

        A `file` id names a package and a package relative path, so a reader that wants the
        bytes, an editor opening the cited line, needs this one step more. Relative to the
        root and never absolute, for the reason `root_records` states.
        """
        labels = _root_labels(self._roots_by_depth)
        out: dict[str, dict[str, str]] = {}
        for package in self.packages.values():
            places: dict[str, str] = {}
            for base in (package.src_path, package.share_path):
                if not base:
                    continue
                for root_path, _kind in self._roots_by_depth:
                    if _under(_abs(base), root_path):
                        inside = os.path.relpath(_abs(base), root_path).replace(os.sep, "/")
                        places[labels[root_path]] = inside
                        break
            if places:
                out[package.name] = dict(sorted(places.items()))
        return dict(sorted(out.items()))

    # ------------------------------------------------------------------
    # internals

    def _build_anchors(self) -> dict[str, str]:
        anchors: list[tuple[str, str]] = []
        for package in self.packages.values():
            for base in (package.src_path, package.share_path):
                if base:
                    anchors.append((base, package.name))
            if package.share_path:
                prefix = str(Path(package.share_path).parent.parent)
                if Path(prefix).name == package.name:
                    anchors.append((prefix, package.name))
        out: dict[str, str] = {}
        for base, name in sorted(set(anchors)):
            out.setdefault(base.rstrip("/") or "/", name)
        return out

    def _anchor_for(self, path: str) -> tuple[str, str] | None:
        """The deepest package directory holding the path, found by walking up from it."""
        probe = self.localise(path)
        while True:
            name = self._anchors.get(probe)
            if name is not None:
                return probe, name
            parent = os.path.split(probe)[0]
            if parent == probe:
                return None
            probe = parent

    def _select(self, key: Any, keep) -> list[str]:
        """Sorted, one entry per (package, relative path), src winning over a stale copy.

        Kept per selection, because every profile asks again and the files never change.
        """
        found = self._selections.get(key)
        if found is None:
            found = self._selections[key] = tuple(self._choose(keep))
        return list(found)

    def _choose(self, keep) -> list[str]:
        chosen: dict[tuple[str, str], tuple[int, str]] = {}
        for path in self._files:
            if not keep(path):
                continue
            package, relative = self.rel(path)
            key = (package or ids.REPO_PKG, relative)
            rank = 0 if self.root_of(path) == records.ROOTS[0] else 1
            current = chosen.get(key)
            if current is None or rank < current[0]:
                chosen[key] = (rank, path)
        return [chosen[key][1] for key in sorted(chosen)]


# ---------------------------------------------------------------------------
# discovery helpers


@dataclass(frozen=True)
class _RootSpec:
    kind: str
    path: str


def _normalise_root(root: Any) -> tuple[str, str]:
    kind = str(getattr(root, "kind", ""))
    path = str(getattr(root, "path", ""))
    if kind not in records.ROOTS:
        raise UsageError(f"root kind {kind!r} is not one of {records.ROOTS}")
    if not path:
        raise UsageError("a root needs a path")
    return kind, _abs(path)


def _walk(root: str, kind: str, ignore: Sequence[str]) -> Iterable[str]:
    skip = SRC_SKIP_DIRS if kind == records.ROOTS[0] else ANY_SKIP_DIRS
    for current, dirnames, filenames in os.walk(root):
        if current != root and any(marker in filenames for marker in IGNORE_MARKERS):
            dirnames[:] = []
            continue
        dirnames[:] = sorted(
            name
            for name in dirnames
            if name not in skip and not _ignored(_relative(str(Path(current) / name), root), ignore)
        )
        for name in sorted(filenames):
            path = str(Path(current) / name)
            if _ignored(_relative(path, root), ignore):
                continue
            if Path(path).is_symlink() and not Path(path).exists():
                continue
            if Path(path).is_file():
                yield path


def _ignored(relative: str, patterns: Sequence[str]) -> bool:
    base = relative.rsplit("/", 1)[-1]
    for raw in patterns:
        pattern = raw.strip().rstrip("/")
        if not pattern:
            continue
        if "/" in pattern:
            if fnmatch(relative, pattern.lstrip("/")) or fnmatch(relative, pattern):
                return True
        elif fnmatch(base, pattern):
            return True
    return False


def _repo_root(paths: Sequence[str]) -> str:
    """Shallowest ancestor holding a .git, so a file outside every package still gets a
    stable repository relative id.

    The walk stops at the temp directory, the home directory and the filesystem root: a
    repository root is never one of those, and a stray .git in a shared directory would
    otherwise reparent every id on one machine and not on another.
    """
    stop = {Path("/"), Path(tempfile.gettempdir()).absolute(), Path.home()}
    found: list[str] = []
    for path in paths:
        current = Path(path)
        for candidate in (current, *current.parents):
            if candidate in stop:
                break
            if (candidate / ".git").exists():
                found.append(str(candidate))
                break
    if found:
        return min(found, key=lambda item: (len(item), item))
    if len(paths) == 1:
        return str(Path(paths[0]).parent)
    return os.path.commonpath(list(paths))


def _packages(
    manifests: Sequence[tuple[int, str, str]], files: Sequence[str]
) -> dict[str, Package]:
    """Join what every root says about each package.

    One package name can appear in more than one root, which is what an overlay is for. The
    copy that wins is the one from the earliest configured root, never the one whose absolute
    path sorts first: a workspace overlaid on a base would otherwise take its facts from the
    base and report them as the configured system. Within one root the path decides, so the
    result stays stable.
    """
    by_dir = {Path(path).parent: set() for path in files}
    for path in files:
        by_dir[Path(path).parent].add(Path(path).name)
    src_path: dict[str, str] = {}
    share_path: dict[str, str] = {}
    build_type: dict[str, str] = {}
    executables: dict[str, set[str]] = {}
    exports: dict[str, set[str]] = {}
    for _order, kind, manifest in sorted(manifests):
        parsed = _read_manifest(manifest)
        if parsed is None:
            continue
        name, declared, plugin_exports = parsed
        directory = Path(manifest).parent
        installed = (
            kind == "install" and directory.name == name and directory.parent.name == "share"
        )
        if kind == "install" and not installed:
            # A manifest elsewhere in an install tree is vendored data, a sample model under
            # `opt/<vendor>/share/doc` for one, and no package of this system. Taking it for a
            # source package cited the install tree as src.
            continue
        if installed:
            share_path.setdefault(name, str(directory))
            prefix = directory.parent.parent
            executables.setdefault(name, set()).update(
                sorted(by_dir.get(prefix / "lib" / name, set()))
            )
        else:
            src_path.setdefault(name, str(directory))
            executables.setdefault(name, set()).update(_src_executables(directory))
        if declared:
            build_type.setdefault(name, declared)
        elif name not in build_type:
            build_type[name] = _infer_build_type(by_dir.get(directory, set()))
        exports.setdefault(name, set()).update(plugin_exports)
    names = sorted(set(src_path) | set(share_path))
    out: dict[str, Package] = {}
    for name in names:
        # ${prefix} is expanded once the package is complete, so a package read from two
        # roots states one manifest path and not one per root.
        base = share_path.get(name) or src_path.get(name) or ""
        manifests_for = {
            plugin.replace("${prefix}", base).replace("$prefix", base)
            for plugin in exports.get(name, set())
        }
        out[name] = Package(
            name=name,
            src_path=src_path.get(name),
            share_path=share_path.get(name),
            build_type=build_type.get(name, "unknown"),
            executables=tuple(sorted(executables.get(name, set()))),
            plugin_manifests=tuple(sorted(manifests_for)),
        )
    return out


def _read_manifest(path: str) -> tuple[str, str, tuple[str, ...]] | None:
    try:
        root = ElementTree.parse(path).getroot()
    except (ElementTree.ParseError, OSError):
        return None
    name_node = root.find("name")
    if name_node is None or not (name_node.text or "").strip():
        return None
    name = (name_node.text or "").strip()
    build_type = ""
    plugins: list[str] = []
    for export in root.findall("export"):
        for child in export:
            if child.tag == "build_type" and (child.text or "").strip():
                build_type = (child.text or "").strip()
            plugin = child.get("plugin")
            if plugin:
                plugins.append(plugin.strip())
    return name, build_type, tuple(plugins)


def _infer_build_type(names: Iterable[str]) -> str:
    entries = set(names)
    if "setup.py" in entries:
        return "ament_python"
    if "CMakeLists.txt" in entries:
        return "ament_cmake"
    return "unknown"


def _src_executables(directory: Path) -> set[str]:
    """Executable names as launch `executable=` will spell them: console_scripts entries,
    add_executable targets and the EXECUTABLE clause of a component registration."""
    out: set[str] = set()
    for name, scrape in (("setup.py", _console_scripts), ("CMakeLists.txt", _cmake_executables)):
        target = directory / name
        if not target.is_file():
            continue
        try:
            text = _read_text(target)
        except CollectorError:
            # Discovery is one level under every collector, so a file it cannot open must
            # leave the index standing: the collector that reads that file is the one that
            # records the refusal on it, and it never runs if this raises.
            continue
        out.update(scrape(text))
    return out


def _console_scripts(text: str) -> set[str]:
    out: set[str] = set()
    marker = "console_scripts"
    index = text.find(marker)
    if index < 0:
        return out
    start = text.find("[", index)
    end = text.find("]", start)
    if start < 0 or end < 0:
        return out
    for raw in text[start + 1 : end].split(","):
        entry = raw.strip().strip("'\"")
        if "=" in entry:
            out.add(entry.split("=", 1)[0].strip())
    return out


_CMAKE_SET = re.compile(r"\bset\(\s*(\w+)\s+\"?([\w.-]+)\"?\s*\)", re.IGNORECASE)
_CMAKE_PROJECT = re.compile(r"\bproject\(\s*([\w.-]+)", re.IGNORECASE)
_CMAKE_VARIABLE = re.compile(r"\$\{(\w+)\}")


def _cmake_executables(text: str) -> set[str]:
    """Target names, with a variable the same file sets to one word expanded:
    `set(EXECUTABLE_NAME "turtlebot3_ros")` before `add_executable(${EXECUTABLE_NAME} ...)`."""
    variables = dict(_CMAKE_SET.findall(text))
    project = _CMAKE_PROJECT.search(text)
    if project is not None:
        variables.setdefault("PROJECT_NAME", project.group(1))
    out = {
        _CMAKE_VARIABLE.sub(lambda hit: variables.get(hit.group(1), hit.group(0)), name)
        for name in _cmake_names(text)
    }
    return {name for name in out if name and "$" not in name}


_CMAKE_TARGET = re.compile(r"\badd_executable\(\s*([^\s()]+)|\bEXECUTABLE\s+([^\s()]+)")


def _cmake_names(text: str) -> set[str]:
    """`add_executable` targets and the `EXECUTABLE` clause of a component registration, as
    written. `EXECUTABLE_NAME` is a variable and not the clause."""
    return {first or second for first, second in _CMAKE_TARGET.findall(text)}


# ---------------------------------------------------------------------------
# file predicates


def _is_launch(name: str) -> bool:
    return any(name.endswith(suffix) for suffix in LAUNCH_SUFFIXES)


def _is_param_file(path: str) -> bool:
    try:
        loaded = _load_yaml(path)
    except CollectorError:
        return False
    return _has_param_key(loaded, 0)


def _load_yaml(path: str) -> Any:
    """Load one yaml document, with every escape route closed.

    Hostile yaml does not only raise YAMLError: deep nesting comes out as RecursionError and
    an unhashable mapping key as TypeError, both straight out of the parser. A discovery pass
    that lets either through takes the whole run with it, so every failure leaves here as a
    CollectorError and the caller decides whether to carry on.
    """
    target = Path(path)
    try:
        if target.stat().st_size > MAX_YAML_BYTES:
            raise CollectorError(f"{path} is larger than the {MAX_YAML_BYTES} byte yaml limit")
        return yaml.safe_load(_read_text(target))
    except CollectorError:
        raise
    except RecursionError as exc:
        raise CollectorError(f"{path} nests deeper than the parser can follow") from exc
    except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
        raise CollectorError(f"cannot read {path} as yaml: {exc}") from exc


def _has_param_key(node: Any, depth: int) -> bool:
    if depth > 8 or not isinstance(node, dict):
        return False
    if any(str(key) == PARAM_KEY for key in node):
        return True
    return any(_has_param_key(value, depth + 1) for value in node.values())


def _read_text(path: Path) -> str:
    """One file as text, with every open failure leaving here as a CollectorError.

    A build file this process cannot open used to raise `PermissionError` straight out of
    discovery, which ended the whole extraction with a traceback and no atlas, before any
    collector ran and before any collector could record the refusal on the file itself.
    """
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        raise CollectorError(f"cannot read {path}: {exc}") from exc


def _file_fingerprint(path: str, marks: Sequence[tuple[bytes, bytes]] = ()) -> bytes:
    target = Path(path)
    if target.name in CONTENT_NAMES or target.suffix in CONTENT_SUFFIXES:
        try:
            raw = target.read_bytes()
        except OSError:
            return b"missing"
        for base, mark in marks:
            raw = raw.replace(base, mark)
        return hashlib.sha256(raw).digest()
    try:
        return str(target.stat().st_size).encode("ascii")
    except OSError:
        return b"missing"


def _prefix_marks(prefix: str) -> list[tuple[bytes, bytes]]:
    """The mount point of a file outside the workspace, as `content_marks` states a root.

    A setup file holds the prefix of the tree it was generated for, so hashing its bytes
    raw separates two boxes that hold the same distribution in two places.
    """
    if not prefix or prefix == os.sep:
        return []
    mark = f"\x00@{OUTSIDE.lstrip('@')}\x00".encode()
    rows = {prefix, os.path.realpath(prefix)}
    return sorted(
        ((row.encode("utf-8", "surrogateescape"), mark) for row in rows),
        key=lambda item: (-len(item[0]), item[0]),
    )


# A root ends where a name ends, so `/ws/src` never rewrites `/ws/src_backup`, and starts where
# a path starts, so `/opt/ros/jazzy` never rewrites the middle of `/data/opt/ros/jazzy`.
_NAME_ENDS = r"(?![\w.+~@-])"
_PATH_STARTS = r"(?<![\w.+~@%-])"


def _replaced(value: Any, rows: Sequence[tuple[str, str]]) -> Any:
    if isinstance(value, str):
        for path, mark in rows:
            value = re.sub(
                _PATH_STARTS + re.escape(path) + _NAME_ENDS, lambda _, mark=mark: mark, value
            )
        return value
    if isinstance(value, list):
        return [_replaced(item, rows) for item in value]
    if type(value) is tuple:
        return tuple(_replaced(item, rows) for item in value)
    if isinstance(value, dict):
        return {key: _replaced(item, rows) for key, item in value.items()}
    return value


def _same_bytes(left: str, right: str) -> bool:
    try:
        return Path(left).read_bytes() == Path(right).read_bytes()
    except OSError:
        return False


def record_undecodable(builder: model.AtlasBuilder, ws: Workspace) -> None:
    """Mirror every unnameable file onto the package that holds it.

    The row is identical from every collector that reads a root, so the four passes state
    one blind spot and not four, and the atlas says the same thing whichever of them ran.
    """
    for pkg_id, name in ws.undecodable_owners():
        builder.node(pkg_id)
        builder.unresolved(
            model.Unresolved(
                want=model.Want.OTHER.value,
                owner=pkg_id,
                expr=name,
                reason="the file name is not valid utf-8",
                role="workspace_walk",
                resolvable_by=["give the file a name that is valid utf-8"],
            )
        )


def _nameable(path: str) -> bool:
    """Whether a path the walk produced is text. `os.walk` decodes a name with the
    surrogateescape handler, so bytes no decoder accepts arrive as lone surrogates and
    every later encode of them raises."""
    try:
        path.encode("utf-8")
    except UnicodeEncodeError:
        return False
    return True


# A slash after a name, a mark, a scheme or another slash is inside something else
# (`a/b`, `@src/x`, `package://x`), except the third slash of `file:///`.
_ABSOLUTE = re.compile(r"(?:(?<=file://)|(?<![\w@.~%:/-]))/[^\s'\"`,;:()\[\]{}<>|=]+")
_SYSTEM_TOPS = frozenset({"etc", "home", "media", "mnt", "opt", "root", "srv", "tmp", "usr", "var"})
# Python quotes a path it raises on after a colon, and every item of a list it prints.
_PYTHON_QUOTED = re.compile(r"(?:\[|: |, )['\"]$")
_NOT_IN_A_ROS_NAME = re.compile(r"[^A-Za-z0-9_]")


def _names_a_file(token: str, quoted: bool = False) -> bool:
    """Whether an absolute token in a message is a path rather than a ROS name.

    Nothing on disk decides it, so two machines state one message one way. A token is a path
    when a segment holds a character no ROS name can (`x.yaml`, `my-ws`), when it sits under a
    top every Linux box has, or when it has two segments or more and Python quoted it the way
    it quotes a path (`No such file or directory: '/data/maps/office'`, the search list of a
    missing package naming an overlay at `/overlay_ws`). A ROS name keeps its spelling outside
    those quotes, `/robot/cmd_vel` in prose or `topic '/robot/odom'`, and a quoted name of one
    segment is never taken for a path. What stays is a directory of ROS legal names under any
    other top, written bare in prose.
    """
    parts = token.strip("/").split("/")
    if any(_NOT_IN_A_ROS_NAME.search(part) for part in parts):
        return True
    return len(parts) > 1 and (quoted or parts[0] in _SYSTEM_TOPS)


def _outside(path: str) -> str:
    return f"{OUTSIDE}/{_spell(Path(path).name)}"


def _spell(text: str) -> str:
    """One name as printable ASCII, every other byte percent escaped, and `%` with it so a
    file literally named `bad%FF.py` and one holding the byte are two spellings."""
    out: list[str] = []
    for byte in text.encode("utf-8", "surrogateescape"):
        out.append(chr(byte) if 32 <= byte < 127 and byte != 0x25 else f"%{byte:02X}")
    return "".join(out)


def _abs(path: str) -> str:
    target = Path(path)
    if not target.is_absolute():
        target = Path.cwd() / target
    return os.path.normpath(str(target))


def _under(path: str, base: str) -> bool:
    return path == base or path.startswith(base.rstrip("/") + "/")


def _relative(path: str, base: str) -> str:
    if path == base:
        return Path(path).name
    return os.path.relpath(path, base).replace(os.sep, "/")


def _root_labels(roots: Sequence[tuple[str, str]]) -> dict[str, str]:
    """One label per root, unique within a kind.

    Two roots of one kind commonly end in the same directory name (`/opt/a/src` and
    `/srv/b/src`), so a label built from the basename alone collides and one root hash
    overwrites the other. The label grows by trailing components until it separates them,
    which keeps it short and independent of where the trees are mounted.
    """
    counts: dict[str, int] = {}
    for _path, kind in roots:
        counts[kind] = counts.get(kind, 0) + 1
    labels: dict[str, str] = {}
    for path, kind in roots:
        if counts[kind] == 1:
            labels[path] = kind
            continue
        others = [other for other, other_kind in roots if other_kind == kind and other != path]
        labels[path] = f"{kind}:{_root_suffix(path, others)}"
    return labels


def _root_suffix(path: str, others: Sequence[str]) -> str:
    parts = [part for part in Path(path).parts if part not in ("/", "")]
    for depth in range(1, len(parts) + 1):
        tail = parts[-depth:]
        if not any(_ends_with(other, tail) for other in others):
            return "/".join(tail)
    return "/".join(parts)


def _ends_with(path: str, tail: Sequence[str]) -> bool:
    parts = [part for part in Path(path).parts if part not in ("/", "")]
    return len(parts) >= len(tail) and parts[-len(tail) :] == list(tail)


__all__ = [
    "CONTENT_NAMES",
    "CONTENT_SUFFIXES",
    "LAUNCH_SUFFIXES",
    "PARAM_KEY",
    "SRC_SKIP_DIRS",
    "Package",
    "ShareRead",
    "Workspace",
]
