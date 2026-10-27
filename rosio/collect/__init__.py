"""Collector registry, the context every collector is handed, and the one handoff between
collectors.

A collector is one source of claims for one layer. It never reads another layer's output and
it never reads another collector's atlas entries. The declared collectors do hand each other
intermediate records, through the bundle on `ctx.declared` and through nothing else, and that
handoff is the one ordering obligation in the extraction. AGENTS section 2.1 states it, names
both halves and is the authority.

The rule is enforced here rather than trusted. The registry binds a producer and a consumer
to their pass: a producer opens the bundle when its pass is reached, whichever way that pass
goes, and a consumer whose producer has not run raises `ContractError` and is recorded as a
refused pass. It used to be handed an empty bundle instead, which reads exactly like a
workspace that launches nothing, so a parameter pass that had read nothing was
indistinguishable from one that had read everything.

Registration is by name with a factory, not with an instance, so importing the registry never
builds anything and an optional dependency stays unimported until the collector runs.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from typing import Any, Protocol, runtime_checkable

from rosio import model
from rosio.config import Config
from rosio.errors import ContractError, UsageError
from rosio.resolve import records
from rosio.workspace import Workspace

BUNDLE_WRITERS = ("entrypoints", "launch_declared")
"""The collectors that produce the shared bundle. A bundle with none of them behind it is
closed, whatever it holds."""

BUNDLE_READERS = ("params",)
"""The collectors that consume it. Named here so the refusal can say who was waiting for
whom, and so a reader of this file meets both halves of the obligation in one place."""


class CollectContext:
    """What a collector may read and the one builder it writes through.

    `profile` is the profile slug in play. An empty slug means profile independent, which is
    the honest answer for a fact that holds whatever runs (an interface definition, a plugin
    manifest) and never for a node set.

    `run` is the orchestration run, shared by independent collectors extending one fact
    slot. A collector called directly may register its own run instead.

    `declared` is the shared bundle. A caller that hands one in has stated its content and
    owns it, which is how a consumer is tested against records a test wrote. A context that
    builds its own hands it out only after a producer reached its pass.
    """

    def __init__(
        self,
        *,
        ws: Workspace,
        cfg: Config,
        builder: model.AtlasBuilder,
        declared: records.DeclaredBundle | None = None,
        profile: str = "",
        run: str | None = None,
    ) -> None:
        self.ws = ws
        self.cfg = cfg
        self.builder = builder
        self.profile = profile
        self.run = run
        self.declared = declared if declared is not None else records.DeclaredBundle()
        self.produced: list[str] = [] if declared is None else ["the caller"]
        self.launch_sandbox: dict[str, str] = {}
        self.launch_walks: dict[Any, Any] = {}

    def open_bundle(self, producer: str) -> None:
        """One producer reached its pass, so the bundle is now what that pass left behind.

        Recorded whichever way the pass went. A producer whose inputs were absent produced an
        empty bundle, which is an answer, and a consumer that finds nothing in it after that
        is honestly empty rather than early.
        """
        if producer not in self.produced:
            self.produced.append(producer)

    def require_bundle(self, reader: str) -> None:
        """Refuse a consumer whose producer has not run. AGENTS section 2a."""
        if self.produced:
            return
        raise ContractError(
            f"{reader} reads the declared bundle and no producer has run in this selection. "
            f"{' and '.join(BUNDLE_WRITERS)} write the records it consumes, so this pass would "
            "report a workspace that launches nothing rather than one it never read. select a "
            "producer with it, or hand the records in"
        )

    def close(self) -> None:
        """Stop the launch walks started ahead of passes that will now never read them."""
        started = self.launch_walks.get("started")
        if started is not None:
            started.stop()

    def profile_id(self) -> str:
        """The profile id a fact should carry, `*` when the fact is profile independent."""
        return self.profile or model.ANY_PROFILE

    def for_profile(self, profile: str) -> CollectContext:
        scoped = CollectContext(
            ws=self.ws,
            cfg=self.cfg,
            builder=self.builder,
            declared=self.declared,
            profile=profile,
        )
        scoped.produced = self.produced
        scoped.launch_sandbox = self.launch_sandbox
        scoped.launch_walks = self.launch_walks
        return scoped


@runtime_checkable
class Collector(Protocol):
    name: str
    layer: str

    def collect(self, ctx: CollectContext) -> None: ...


class _Pass:
    """One collector from the registry, with its half of the bundle obligation recorded
    around its pass.

    The pass boundary is the only place that knows which collector is running: a producer
    cannot state it from inside, because its own first act is to read the bundle it is about
    to append to. A collector a caller built directly is not bound, and that caller owns the
    bundle it hands in.
    """

    def __init__(self, name: str, inner: Any) -> None:
        self._name = name
        self._inner = inner

    @property
    def name(self) -> str:
        return self._inner.name

    @property
    def layer(self) -> str:
        return self._inner.layer

    def __getattr__(self, item: str) -> Any:
        return getattr(self._inner, item)

    def _boundary(self, ctx: CollectContext) -> None:
        if self._name in BUNDLE_WRITERS:
            ctx.open_bundle(self._name)
        else:
            ctx.require_bundle(self._name)

    def inputs_present(self, ctx: CollectContext) -> Any:
        self._boundary(ctx)
        probe = getattr(self._inner, "inputs_present", None)
        return True if probe is None else probe(ctx)

    def collect(self, ctx: CollectContext) -> None:
        self._boundary(ctx)
        self._inner.collect(ctx)


BUILT_ONLY = "file_after_build"
"""A file a launch file or a robot model reads from the share of a package that is not built,
which the package source holds under another directory. The install rule puts it where the
reader looks, so it exists only after a build and nothing in the reader is wrong."""

ELSEWHERE_LIMIT = 3
"""How many same named files one missing file names. A package holding more than that under
one name gives a reader a list to search rather than a path to use."""


def same_name_in_package(ws: Workspace, path: str) -> list[str]:
    """The package relative paths of other files that carry the name of `path` inside the
    package directory it points into, sorted. A launch file or a robot model naming a file
    the package holds under another directory is an install rule or a path that moved, and
    the place it does exist is the useful half of the answer."""
    name = ws.package_of(path)
    package = ws.packages.get(name or "")
    if package is None:
        return []
    target = Path(ws.localise(path))
    for base in (package.share_path, package.src_path):
        if base and target.is_relative_to(base):
            found = sorted(
                item.relative_to(base).as_posix()
                for item in Path(base).rglob(target.name)
                if item.is_file() and item != target
            )
            return found[:ELSEWHERE_LIMIT]
    return []


REGISTRY: dict[str, Callable[[], Collector]] = {}


def register(name: str, factory: Callable[[], Collector]) -> None:
    if not name:
        raise ContractError("a collector needs a name")
    existing = REGISTRY.get(name)
    if existing is not None and existing is not factory:
        raise ContractError(f"collector {name} is already registered")
    REGISTRY[name] = factory


def get(name: str) -> Collector:
    factory = REGISTRY.get(name)
    if factory is None:
        known = ", ".join(sorted(REGISTRY)) or "none"
        raise UsageError(f"unknown collector {name!r}, registered: {known}")
    built = factory()
    return _Pass(name, built) if name in BUNDLE_WRITERS + BUNDLE_READERS else built


def names(layer: str = "") -> list[str]:
    """Registered collector names, optionally restricted to one layer."""
    if not layer:
        return sorted(REGISTRY)
    return sorted(name for name in REGISTRY if get(name).layer == layer)


__all__ = [
    "BUILT_ONLY",
    "BUNDLE_READERS",
    "BUNDLE_WRITERS",
    "REGISTRY",
    "CollectContext",
    "Collector",
    "get",
    "names",
    "register",
    "same_name_in_package",
]
