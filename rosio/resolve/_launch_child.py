"""The sandbox child that builds and walks one launch description.

Run as a script with one json request on stdin and one json response on stdout, behind a
marker line so a login shell banner or a chatty launch file cannot corrupt the payload. The
parent never imports launch, so a description that raises, hangs or leaves the interpreter
dirty costs one walk.

The walk never starts a process. Every action that would spawn something is read instead of
executed: `Node` and `LifecycleNode` have their substitutions performed and their expanded
attributes read, `ExecuteProcess` has its command performed and its argv parsed, an event
handler has its payload pulled out by reflection rather than waited for, and a `TimerAction`
is recorded as a gate. Everything that only mutates the context is visited, so launch itself
keeps owning namespace scoping, group scoping and argument defaults.

While the description is built and walked, the filesystem and environment probes it makes are
recorded. On the reference stack one `os.path.exists` call decides which slam executable
runs, so two machines resolve the same file differently. Recording the probe turns that into
a visible input difference instead of drift.
"""

from __future__ import annotations

import contextlib
import errno
import itertools
import json
import os
import re
import shlex
import sys
from collections.abc import Sequence
from dataclasses import replace
from pathlib import Path
from typing import Any

_REPO = str(Path(__file__).resolve().parents[2])
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

from rosio import model  # noqa: E402
from rosio.errors import CollectorError  # noqa: E402
from rosio.resolve import identity, launch_exec, records, ros_args, substitutions  # noqa: E402

MAX_DEPTH = 60

ROOT = identity.ROOT
"""What launch_ros means by its unspecified namespace sentinel: the node runs in the root
namespace. The sentinel text itself never leaves this module."""

_LIBRARY_MARKERS = ("/site-packages/", "/dist-packages/", "/lib/python3")
"""Where python libraries live, the launch and launch_ros packages, xacro and the standard
library among them. Where a distribution is mounted says nothing about what a file is: every
launch file a binary install ships sits under `/opt/ros/<distro>/share/<pkg>/launch/`."""

_LAUNCH_DIR = re.compile(r"/share/[^/]+/launch/")

TEMP_ENV = frozenset({"TMPDIR", "TEMP", "TMP"})
"""Where a process keeps its temp files is a property of the machine and never a branch of a
launch file. Recording it put a machine path into the run digest, so two machines extracting
one tree wrote two atlases."""

PROBE_VALUE_CAP = 64
"""How much of an environment value a predicate key carries. The value is what decided the
branch, so it has to be recorded, and an unbounded one would put a whole file into a run."""

OUTPUT_FLAGS = ("-o", "--output")
OUTPUT_TOKEN = "@output"
"""A temp file a process writes is spelled by the command that writes it, with this token
where the command names the file, so the random name never leaves the child."""


class _MissingArgumentError(Exception):
    """A declared launch argument with no default and no supplied value."""

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.arg_name = name


class _Probes:
    """Records the filesystem and environment questions the launch code asks.

    A key is compared data, so it carries no host layout: a probed path is stated relative to
    the root that holds it, and the absolute path is recoverable from the roots the run
    record already lists. An environment key carries the value that decided the branch,
    because two delays that differ recorded the same `true` before.
    """

    def __init__(self, roots: Sequence[str] = (), sandbox: bool = False) -> None:
        self.seen: dict[str, bool] = {}
        self.sandbox = sandbox
        self.roots = [str(root) for root in roots]
        self._saved: list[tuple[Any, str, Any]] = []

    def install(self) -> None:
        for name in ("exists", "isfile", "isdir"):
            original = getattr(os.path, name)
            self._saved.append((os.path, name, original))
            setattr(os.path, name, self._wrap_path(name, original))
        original_get = os.environ.get
        self._saved.append((os.environ, "get", original_get))
        os.environ.get = self._wrap_env(original_get)  # type: ignore[method-assign]

    def restore(self) -> None:
        for owner, name, original in reversed(self._saved):
            with contextlib.suppress(AttributeError, TypeError):
                setattr(owner, name, original)
        self._saved.clear()

    def _wrap_path(self, label: str, original):
        def probe(path, *rest, **kwargs):
            outcome = original(path, *rest, **kwargs)
            if _from_launch_code():
                self.seen[f"{label}:{self._relative(str(path))}"] = bool(outcome)
            return outcome

        return probe

    def _wrap_env(self, original):
        def probe(key, *rest, **kwargs):
            outcome = original(key, *rest, **kwargs)
            if (
                key not in TEMP_ENV
                and not (self.sandbox and key in ("ROS_HOME", "ROS_LOG_DIR"))
                and _from_launch_code()
            ):
                self.seen[f"env:{key}={_capped(outcome)}"] = bool(outcome)
            return outcome

        return probe

    def _relative(self, path: str) -> str:
        return identity.relative_path(path, self.roots)


def _capped(value: Any) -> str:
    """The value an environment probe read, short enough to sit in a key."""
    if value is None:
        return ""
    text = str(value).strip().replace("\n", " ")
    return text if len(text) <= PROBE_VALUE_CAP else text[:PROBE_VALUE_CAP] + "..."


class _NoProcesses:
    """Blocks a process start for the duration of the walk.

    Replaying an event handler payload runs launch file code that was written for a live
    system, and one shutdown handler on the reference stack detaches a helper process. The
    walk resolves a description, so anything that would spawn is refused here rather than
    trusted not to happen.
    """

    def __init__(self, sandbox: bool = False) -> None:
        self.sandbox = sandbox
        self.blocked: list[str] = []
        self._saved: list[tuple[Any, str, Any]] = []

    def install(self) -> None:
        import subprocess

        global _ACTIVE
        _ACTIVE = self
        if not _HOOKED:
            _hook()

        for owner, name in (
            (subprocess, "Popen"),
            (subprocess, "run"),
            (subprocess, "call"),
            (subprocess, "check_call"),
            (subprocess, "check_output"),
            (os, "system"),
            (os, "posix_spawn"),
            (os, "fork"),
        ):
            original = getattr(owner, name, None)
            if original is None:
                continue
            self._saved.append((owner, name, original))
            setattr(owner, name, self._refuse(name))

    def restore(self) -> None:
        global _ACTIVE
        _ACTIVE = None
        for owner, name, original in reversed(self._saved):
            with contextlib.suppress(AttributeError, TypeError):
                setattr(owner, name, original)
        self._saved.clear()

    def _refuse(self, name: str):
        def refuse(*_args, **_kwargs):
            self.blocked.append(name)
            raise RuntimeError(f"the launch walk refused to start a process through {name}")

        return refuse


PROCESS_EVENTS = frozenset(
    {
        "os.exec",
        "os.fork",
        "os.forkpty",
        "os.posix_spawn",
        "os.spawn",
        "os.startfile",
        "os.system",
        "subprocess.Popen",
    }
)
"""Audit events CPython raises on every way it can start a process. The patched names above
say which call was refused; the hook is what refuses the ones nobody thought to patch."""

_ACTIVE: _NoProcesses | None = None
_HOOKED = False


def _hook() -> None:
    """Audit hooks cannot be removed, so one is added per child and consults the active guard."""
    global _HOOKED

    def audit(event: str, _args: tuple) -> None:
        guard = _ACTIVE
        if guard is not None and guard.sandbox and event == "socket.__new__":
            raise CollectorError(launch_exec.SANDBOX_REFUSED)
        if guard is not None and event in PROCESS_EVENTS:
            guard.blocked.append(event)
            raise RuntimeError(f"the launch walk refused to start a process through {event}")

    sys.addaudithook(audit)
    _HOOKED = True


class _Replay:
    """A stand in for the event an event handler was written for.

    Every attribute answers, and a process exit reports success, because the payload of the
    success branch is the node set the launch file really has.
    """

    returncode = 0
    execute_process_returncode = 0

    def __getattr__(self, name: str):
        del name
        return


_OWN_PACKAGE = str(Path(__file__).resolve().parents[1])
"""This package's own directory. Its code is never launch file code, wherever it is installed:
a checkout unpacked under a temp directory sits outside every library marker."""


def _from_launch_code() -> bool:
    """True when the caller is launch file code rather than a library or this walker."""
    frame = sys._getframe(2)
    name = frame.f_code.co_filename
    if name == __file__ or name.endswith("_launch_child.py"):
        return False
    if name.startswith(_OWN_PACKAGE):
        return False
    if _LAUNCH_DIR.search(name):
        return True
    return not any(marker in name for marker in _LIBRARY_MARKERS)


class _Frame:
    """Where the walk currently is: which file, behind which gates, at which action path."""

    __slots__ = ("file", "gates", "path")

    def __init__(self, file: str, gates: tuple = (), path: tuple = ()) -> None:
        self.file = file
        self.gates = gates
        self.path = path

    def step(self, segment: str, gate: records.Gate | None = None) -> _Frame:
        gates = (*self.gates, gate) if gate is not None else self.gates
        return _Frame(self.file, gates, (*self.path, segment))

    def within(self, file: str) -> _Frame:
        return _Frame(file, self.gates, self.path)


class _ConstructionSites:
    def __init__(self) -> None:
        self.originals = []

    def install(self) -> None:
        from launch.action import Action
        from launch_ros.descriptions import ComposableNode

        for cls in (Action, ComposableNode):
            original = cls.__init__
            self.originals.append((cls, original))
            cls.__init__ = self._capture(original)

    @staticmethod
    def _capture(original):
        def construct(action, *args, **kwargs):
            lines = {}
            frame = sys._getframe(1)
            while frame is not None:
                lines.setdefault(frame.f_code.co_filename, frame.f_lineno)
                frame = frame.f_back
            original(action, *args, **kwargs)
            action._rosio_construction_lines = lines

        return construct

    def restore(self) -> None:
        for cls, original in self.originals:
            cls.__init__ = original


_COMMAND = re.compile(r"__rosio_command_([0-9a-f]*)__")


def _command_text(value: str) -> str | None:
    if not _COMMAND.search(value):
        return None
    try:
        return _COMMAND.sub(lambda match: bytes.fromhex(match[1]).decode("utf-8"), value)
    except (ValueError, UnicodeDecodeError):
        return None


class _Commands:
    def __init__(self) -> None:
        self.partial: set[str] = set()
        """Command texts holding a part the walk could not perform, spelled as described."""

    def install(self) -> None:
        from launch import LaunchContext
        from launch.substitutions import Command
        from launch_ros.parameter_descriptions import ParameterValue

        self.originals = (
            Command.perform,
            ParameterValue.evaluate,
            LaunchContext.perform_substitution,
        )
        original_evaluate = ParameterValue.evaluate
        original_perform = LaunchContext.perform_substitution
        self.performed = []

        def perform(command, context):
            parts = []
            described = False
            for part in command.command:
                try:
                    parts.append(substitutions.perform(context, part))
                except BaseException:
                    parts.append(substitutions.describe_value(part))
                    described = True
            text = "".join(parts)
            text = _command_text(text) or text
            self.performed.append(text)
            if described:
                self.partial.add(text)
            return "__rosio_command_" + text.encode("utf-8").hex() + "__"

        def evaluate(value, context):
            from launch import Substitution

            raw = value.value
            if isinstance(raw, (list, tuple)) and any(isinstance(p, Substitution) for p in raw):
                text = substitutions.perform(context, raw)
                if _command_text(text) is not None:
                    return text
            return original_evaluate(value, context)

        def perform_substitution(context, substitution):
            value = original_perform(context, substitution)
            if isinstance(value, str) and (text := _command_text(value)) is not None:
                self.performed.append(text)
            return value

        Command.perform = perform
        ParameterValue.evaluate = evaluate
        LaunchContext.perform_substitution = perform_substitution

    def restore(self) -> None:
        from launch import LaunchContext
        from launch.substitutions import Command
        from launch_ros.parameter_descriptions import ParameterValue

        Command.perform, ParameterValue.evaluate, LaunchContext.perform_substitution = (
            self.originals
        )


class _ModelTexts:
    """The text of every file launch code read whole, and of every document xacro processed.

    A robot model reaches a node as text, so the path is gone by the time the parameter is
    read. A parameter value equal to the text of exactly one file read here is that file; a
    value read and then changed equals none, and nothing is guessed from a file name.
    """

    def __init__(self) -> None:
        self.opened: set[tuple[str, Any, Any, Any]] = set()
        self.produced: dict[str, set[str]] = {}
        self._saved: list[tuple[Any, str, Any]] = []
        self._open = open
        self._read: dict[tuple[str, Any, Any, Any], str] = {}

    def install(self) -> None:
        import builtins

        self._patch(builtins, "open", self._wrap_open)
        self._patch(Path, "read_text", self._wrap_read_text)
        try:
            import xacro
        except ImportError:
            return
        if callable(getattr(xacro, "process_file", None)):
            self._patch(xacro, "process_file", self._wrap_xacro)

    def restore(self) -> None:
        for owner, name, original in reversed(self._saved):
            setattr(owner, name, original)
        self._saved.clear()

    def texts(self) -> dict[str, set[str]]:
        out = {text: set(paths) for text, paths in self.produced.items()}
        for key in self.opened - self._read.keys():
            path, encoding, errors, newline = key
            try:
                with self._open(path, encoding=encoding, errors=errors, newline=newline) as f:
                    self._read[key] = f.read()
            except (OSError, UnicodeError, ValueError, TypeError, LookupError):
                self._read[key] = ""
        for (path, *_), text in self._read.items():
            out.setdefault(text, set()).add(path)
        out.pop("", None)
        return out

    def _patch(self, owner, name: str, wrap) -> None:
        original = getattr(owner, name)
        self._saved.append((owner, name, original))
        setattr(owner, name, wrap(original))

    def _wrap_open(self, original):
        def opened(
            file, mode="r", buffering=-1, encoding=None, errors=None, newline=None, *rest, **kw
        ):
            handle = original(file, mode, buffering, encoding, errors, newline, *rest, **kw)
            if (
                isinstance(file, (str, os.PathLike))
                and isinstance(mode, str)
                and set(mode) <= set("rt")
                and _from_launch_code()
            ):
                self.opened.add((str(Path(file).absolute()), encoding, errors, newline))
            return handle

        return opened

    def _wrap_read_text(self, original):
        def read_text(path, *args, **kwargs):
            text = original(path, *args, **kwargs)
            if _from_launch_code():
                self.produced.setdefault(text, set()).add(str(Path(path).absolute()))
            return text

        return read_text

    def _wrap_xacro(self, original):
        def process_file(input_file_name, *args, **kwargs):
            document = original(input_file_name, *args, **kwargs)
            path = str(Path(input_file_name).absolute())
            for name in ("toxml", "toprettyxml"):
                method = getattr(document, name, None)
                if method is not None:
                    with contextlib.suppress(AttributeError, TypeError):
                        setattr(document, name, self._serialised(method, path))
            return document

        return process_file

    def _serialised(self, method, path: str):
        def serialise(*args, **kwargs):
            text = method(*args, **kwargs)
            if isinstance(text, str):
                self.produced.setdefault(text, set()).add(path)
            return text

        return serialise


def _action_site(action, file: str, locator: str) -> records.Site:
    lines = getattr(action, "_rosio_construction_lines", {})
    return records.Site(file, locator, lines.get(file))


class _PushedPair(tuple):
    site: records.Site | None = None


class _PushedPath(str):
    site: records.Site | None = None


def _pushed(value, site: records.Site):
    """One group scoped entry, carrying the action that pushed it through to every node."""
    if isinstance(value, tuple):
        marked = _PushedPair(value)
    elif isinstance(value, str):
        marked = _PushedPath(value)
    else:
        return value
    marked.site = site
    return marked


# launch_ros keeps group scoped rules in launch configurations, so a node reads them with no
# trace of the action, or the file, that pushed them.
_SCOPED_LISTS = {
    "SetRemap": "ros_remaps",
    "SetParameter": "global_params",
    "SetParametersFromFile": "global_params",
}


class Walker:
    """One entry point, one argument set, one result."""

    def __init__(self, request: dict[str, Any]) -> None:
        self.entry = str(request.get("entry_path", ""))
        self.args = [(str(k), str(v)) for k, v in request.get("args", ())]
        self.env = [(str(k), str(v)) for k, v in request.get("env", ())]
        self.roots = [str(path) for path in request.get("roots", ())]
        self.probes = _Probes(self.roots, sandbox=bool(request.get("sandbox")))
        self.guard = _NoProcesses(sandbox=bool(request.get("sandbox")))
        self.sites = _ConstructionSites()
        self.commands = _Commands()
        self.models = _ModelTexts()
        self._described_cache: dict[int, Any] = {}
        self._lifecycle_nodes: list[tuple[Any, records.ResolvedNode]] = []
        self._lifecycle_events: list[Any] = []
        self.produced: dict[str, str] = {}
        """Temp files a process of this walk writes when it runs, with the command that does."""
        self._touched: list[tuple[int, str, str]] = []
        self.result = records.LaunchWalk(
            entry_path=self.entry,
            args=tuple(self.args),
            env=tuple(self.env),
        )
        self.context: Any = None

    # ------------------------------------------------------------------

    def run(self) -> records.LaunchWalk:
        substitutions.require_launch()
        from launch import LaunchContext

        for key, value in self.env:
            os.environ[key] = value
        if self.guard.sandbox:
            os.environ.update(
                TMPDIR="/tmp", ROS_HOME="/tmp/rosio-ros", ROS_LOG_DIR="/tmp/rosio-ros/log"
            )
        self.context = LaunchContext(argv=[])
        for key, value in self.args:
            self.context.launch_configurations[key] = value
        # ThisLaunchFileDir reads these locals, and only a launch service sets them.
        self.context.extend_locals(
            {
                "current_launch_file_path": self.entry,
                "current_launch_file_directory": str(Path(self.entry).parent),
            }
        )
        module = None
        self.probes.install()
        self.guard.install()
        self.sites.install()
        self.commands.install()
        self.models.install()
        try:
            module = self._load()
            if module is not None:
                description = self._describe(module)
                if description is not None:
                    self._walk(description)
                    self._bind_lifecycle_events()
        finally:
            self.models.restore()
            self.commands.restore()
            self.sites.restore()
            self.guard.restore()
            self.probes.restore()
            self.result.predicates = dict(sorted(self.probes.seen.items()))
            if self.guard.blocked:
                blocked = ", ".join(sorted(set(self.guard.blocked)))
                self.result.notes.append(f"a process start was refused during the walk: {blocked}")
        self._unrun_outputs()
        self.result.nodes.sort(key=_order)
        self.result.includes.sort(key=lambda item: (item.path, item.args))
        self.result.arg_declarations.sort(key=lambda item: item.name)
        return self.result

    def _load(self):
        from launch.launch_description_sources.python_launch_file_utilities import (
            load_python_launch_file_as_module,
        )

        try:
            return load_python_launch_file_as_module(self.entry)
        except BaseException as exc:
            self._raised("import", exc, self.entry)
            # A module that reads an unset variable at import is a launch file the profile
            # has not configured yet, which launch.raised says, and not a file of another kind.
            self.result.loadable = bool(self.result.exceptions[-1].env)
            return None

    def _describe(self, module):
        function = getattr(module, "generate_launch_description", None)
        if function is None:
            self.result.loadable = False
            self.result.notes.append("no generate_launch_description in this file")
            return None
        try:
            return function()
        except BaseException as exc:
            self._raised("describe", exc, self.entry)
            return None

    def _walk(self, description) -> None:
        try:
            self._entities(_sub_entities(description), _Frame(self.entry), 0)
        except _MissingArgumentError as missing:
            self.result.exceptions.append(
                records.WalkException(
                    path=self.entry,
                    phase="argument",
                    exc_type="RuntimeError",
                    message=(f"required launch argument {missing.arg_name!r} was not provided"),
                )
            )
            self.result.notes.append(f"the walk stopped at the declaration of {missing.arg_name!r}")

    # ------------------------------------------------------------------
    # dispatch

    def _entities(self, entities, frame: _Frame, depth: int) -> None:
        if depth > MAX_DEPTH:
            self.result.notes.append("the action tree is deeper than the walk follows")
            return
        for entity in entities or ():
            self._entity(entity, frame, depth)

    def _entity(self, entity, frame: _Frame, depth: int) -> None:
        import launch
        import launch_ros

        if isinstance(entity, launch.LaunchDescription):
            self._entities(_sub_entities(entity), frame, depth + 1)
            return
        if isinstance(entity, launch.actions.DeclareLaunchArgument):
            self._declaration(entity, frame)
            self._visit(entity, frame, depth)
            return
        if isinstance(entity, launch_ros.actions.LoadComposableNodes):
            self._load_composable(entity, frame)
            return
        if isinstance(entity, launch_ros.actions.ComposableNodeContainer):
            self._container(entity, frame, depth)
            return
        if isinstance(entity, launch_ros.actions.Node):
            self._node(entity, frame)
            return
        if isinstance(entity, launch.actions.EmitEvent):
            self._emit_event(entity, frame)
            return
        if isinstance(entity, launch.actions.RegisterEventHandler):
            self._event_handler(entity, frame, depth)
            return
        if isinstance(entity, launch.actions.TimerAction):
            self._timer(entity, frame, depth)
            return
        if isinstance(entity, launch.actions.IncludeLaunchDescription):
            self._include(entity, frame, depth)
            return
        if isinstance(entity, launch.actions.OpaqueFunction):
            self._opaque(entity, frame, depth)
            return
        if isinstance(entity, launch.actions.ExecuteProcess):
            self._process(entity, frame)
            return
        key = _SCOPED_LISTS.get(type(entity).__name__)
        if key is None:
            self._visit(entity, frame, depth)
            return
        configurations = self.context.launch_configurations
        start = len(configurations.get(key) or ())
        self._visit(entity, frame, depth)
        entries = configurations.get(key) or []
        site = _action_site(entity, frame.file, type(entity).__name__)
        for index in range(start, len(entries)):
            entries[index] = _pushed(entries[index], site)

    def _visit(self, entity, frame: _Frame, depth: int) -> None:
        """Let launch execute an action that only mutates the context, then walk the result.

        Group scoping works because the pop actions come back in the returned list, so
        nothing here reimplements a scope.
        """
        try:
            produced, frame = self._perform_action(entity, frame)
        except _MissingArgumentError:
            raise
        except BaseException as exc:
            self._raised("visit", exc, frame.file, _label(entity))
            return
        if produced:
            self._entities(produced, frame, depth + 1)

    def _perform_action(self, action, frame: _Frame):
        gates, _, _ = self._condition_gate(getattr(action, "condition", None), frame)
        if gates != frame.gates and gates[-1].commands:
            return action.execute(self.context), _Frame(frame.file, gates, frame.path)
        return action.visit(self.context), frame

    # ------------------------------------------------------------------
    # actions

    def _declaration(self, action, frame: _Frame) -> None:
        default = getattr(action, "default_value", None)
        text = None
        if default is not None:
            try:
                text = substitutions.perform(self.context, default)
            except BaseException as exc:
                self._raised("argument", exc, frame.file, f"DeclareLaunchArgument({action.name})")
        description = str(getattr(action, "description", "") or "")
        if description == "no description given":
            description = ""
        if frame.file == self.entry:
            # An argument declared by an included file belongs to that file, not to this
            # entry point: a profile can only supply what the entry point itself declares.
            self.result.arg_declarations.append(
                records.ArgDeclaration(
                    name=str(action.name),
                    default=text,
                    description=description,
                    site=records.Site(frame.file, f"DeclareLaunchArgument({action.name})"),
                )
            )
        supplied = str(action.name) in self.context.launch_configurations
        conditional = bool(getattr(action, "_conditionally_included", False))
        if default is None and not supplied and not conditional:
            raise _MissingArgumentError(str(action.name))

    def _timer(self, action, frame: _Frame, depth: int) -> None:
        gates, _, _ = self._condition_gate(getattr(action, "condition", None), frame)
        frame = _Frame(frame.file, gates, frame.path)
        start = len(self.commands.performed)
        period = substitutions.perform_period(self.context, getattr(action, "period", None))
        gate = records.Gate(
            mechanism="TimerAction",
            period=period,
            commands=tuple(sorted(set(self.commands.performed[start:]))),
            command_role="timer_period",
            site=records.Site(frame.file, "TimerAction"),
        )
        disc = "unknown" if period is None else f"{float(period):g}s"
        self._entities(
            getattr(action, "actions", ()) or (),
            frame.step(f"TimerAction({disc})", gate),
            depth + 1,
        )

    def _opaque(self, action, frame: _Frame, depth: int) -> None:
        name = substitutions.function_name(
            getattr(action, "_OpaqueFunction__function", None), self.roots
        )
        try:
            produced, frame = self._perform_action(action, frame)
        except _MissingArgumentError:
            raise
        except BaseException as exc:
            self._raised("visit", exc, frame.file, f"OpaqueFunction({name})")
            return
        self._entities(produced, frame.step(f"OpaqueFunction({name})"), depth + 1)

    def _include(self, action, frame: _Frame, depth: int) -> None:
        source = getattr(action, "launch_description_source", None)
        try:
            produced, frame = self._perform_action(action, frame)
        except _MissingArgumentError:
            raise
        except BaseException as exc:
            location = _location(source)[0]
            command = _command_text(location)
            if command is None:
                self._raised("include", exc, frame.file, "IncludeLaunchDescription")
            self.result.includes.append(
                records.LaunchInclude(
                    path=command if command is not None else location,
                    site=_action_site(action, frame.file, "IncludeLaunchDescription"),
                    resolved=False,
                    reason="command_substitution" if command is not None else "",
                )
            )
            return
        if produced is None:
            # The condition excluded it, so launch itself did not include the file and its
            # path was never expanded. Nothing was missed here, and recording it as a blind
            # spot claims a hole that the profile does not have.
            condition = substitutions.describe_condition(getattr(action, "condition", None))
            self.result.notes.append(f"an include is switched off by {condition}")
            return
        location, resolved = _location(source)
        arguments = substitutions.perform_pairs(
            self.context, getattr(action, "launch_arguments", ()) or ()
        )
        self.result.includes.append(
            records.LaunchInclude(
                path=location,
                args=arguments,
                entity_path=(*frame.path, f"Include({Path(location).name or 'unknown'})"),
                site=_action_site(action, frame.file, "IncludeLaunchDescription"),
                resolved=resolved,
            )
        )
        child = frame.within(location if resolved else frame.file)
        disc = (Path(location).name if resolved else "") or "unknown"
        self._entities(produced, child.step(f"Include({disc})"), depth + 1)

    def _event_handler(self, action, frame: _Frame, depth: int) -> None:
        handler = getattr(action, "event_handler", None)
        mechanism = type(handler).__name__
        if mechanism not in records.GATE_MECHANISMS:
            self._visit(action, frame, depth)
            return
        gates, _, _ = self._condition_gate(getattr(action, "condition", None), frame)
        frame = _Frame(frame.file, gates, frame.path)
        target = self._target_name(handler)
        gate = records.Gate(
            mechanism=mechanism,
            target=target,
            site=records.Site(frame.file, mechanism),
        )
        payload = self._payload(handler, frame, mechanism)
        disc = target or substitutions.function_name(handler.__class__, self.roots)
        self._entities(payload, frame.step(f"{mechanism}({disc})", gate), depth + 1)

    def _emit_event(self, action, frame: _Frame) -> None:
        from launch_ros.events.lifecycle import ChangeState

        event = action.event
        if not isinstance(event, ChangeState):
            return
        gates, _, _ = self._condition_gate(getattr(action, "condition", None), frame)
        if any(gate.outcome in ("false", "unknown") for gate in gates):
            return
        self._lifecycle_events.append(event)

    def _bind_lifecycle_events(self) -> None:
        for event in self._lifecycle_events:
            transition = (
                event.valid_transitions[event.transition_id].removeprefix("TRANSITION_").lower()
            )
            for action, node in self._lifecycle_nodes:
                if node.attrs.get("unreachable"):
                    continue
                try:
                    matches = event.lifecycle_node_matcher(action)
                except BaseException as exc:
                    self._raised("visit", exc, self.entry, "ChangeState")
                    continue
                if matches:
                    previous = node.attrs.get("lifecycle_transitions", [])
                    node.attrs["lifecycle_transitions"] = sorted({*previous, transition})

    def _payload(self, handler, frame: _Frame, mechanism: str) -> list:
        actions = getattr(handler, "_OnActionEventBase__actions_on_event", None)
        if actions:
            return list(actions)
        entities = getattr(handler, "entities", None)
        if entities:
            return list(entities) if isinstance(entities, (list, tuple)) else [entities]
        for attr in (
            "_OnActionEventBase__on_event",
            "_OnShutdown__on_shutdown",
            "_OnProcessIO__on_stdin",
        ):
            callback = getattr(handler, attr, None)
            if callback is None or not callable(callback):
                continue
            produced = self._call_callback(callback, frame, mechanism)
            if produced is not None:
                return produced
        return []

    def _call_callback(self, callback, frame: _Frame, mechanism: str) -> list | None:
        for arguments in ((_Replay(), self.context), (None, self.context), (self.context,), ()):
            try:
                produced = callback(*arguments)
            except _MissingArgumentError:
                raise
            except BaseException:
                continue
            if produced is None:
                return []
            if isinstance(produced, (list, tuple)):
                return list(produced)
            return [produced]
        self.result.notes.append(f"the {mechanism} payload callback could not be replayed")
        self._raised(
            "visit",
            RuntimeError("the event handler payload callback raised"),
            frame.file,
            mechanism,
        )
        return None

    def _target_name(self, handler) -> str:
        for attr in (
            "_OnActionEventBase__action_matcher",
            "_OnStateTransition__target_lifecycle_node",
        ):
            target = getattr(handler, attr, None)
            if target is None or (callable(target) and not hasattr(target, "name")):
                continue
            return self._action_name(target)
        return ""

    def _action_name(self, action) -> str:
        import launch_ros

        if isinstance(action, launch_ros.actions.Node):
            raw = getattr(action, "_Node__node_name", None)
            if raw is not None:
                return substitutions.perform(self.context, raw)
            return substitutions.perform(self.context, getattr(action, "node_executable", ""))
        name = getattr(action, "name", None)
        if name is None:
            return ""
        try:
            return substitutions.perform(self.context, name)
        except BaseException:
            return ""

    # ------------------------------------------------------------------
    # nodes

    def _container(self, action, frame: _Frame, depth: int) -> None:
        container = self._node(action, frame)
        descriptions = getattr(action, "_ComposableNodeContainer__composable_node_descriptions", ())
        fqn = _fqn(container) if container is not None else None
        for description in descriptions or ():
            self._composable(description, frame, fqn)
        del depth

    def _load_composable(self, action, frame: _Frame) -> None:
        gates, conditions, reachable = self._condition_gate(
            getattr(action, "condition", None), frame
        )
        frame = _Frame(frame.file, gates, frame.path)
        target = getattr(action, "_LoadComposableNodes__target_container", None)
        fqn = None
        if target is not None:
            name = self._action_name(target)
            namespace = _container_namespace(target)
            # Without a namespace the container name stays relative. Joining an empty one
            # normalises to the root and names a container that may sit somewhere else.
            fqn = _join(namespace, name) if (name and namespace) else (name or None)
        descriptions = getattr(action, "_LoadComposableNodes__composable_node_descriptions", ())
        for description in descriptions or ():
            self._composable(description, frame, fqn, conditions, reachable)

    def _composable(
        self,
        description,
        frame: _Frame,
        container: str | None,
        conditions: tuple = (),
        reachable: bool = True,
    ) -> None:
        from launch_ros.utilities import make_namespace_absolute, prefix_namespace

        condition = getattr(description, "condition", None)
        condition = condition() if callable(condition) else condition
        gates, own_conditions, own_reachable = self._condition_gate(condition, frame)
        conditions = (*conditions, *own_conditions)
        reachable = reachable and own_reachable
        name = substitutions.perform_optional(
            self.context, getattr(description, "_ComposableNode__node_name", None)
        )
        stated = getattr(description, "_ComposableNode__node_namespace", None)
        namespace = None
        if stated is not None:
            namespace = substitutions.perform(self.context, stated)
        base = self.context.launch_configurations.get("ros_namespace", None)
        composed = make_namespace_absolute(prefix_namespace(base, namespace))
        # None here means neither the description nor an enclosing group stated a namespace,
        # which is the root namespace and not an unknown one.
        composed = ROOT if composed is None else composed
        plugin = substitutions.perform(
            self.context, getattr(description, "_ComposableNode__node_plugin", "")
        )
        package = substitutions.perform(
            self.context, getattr(description, "_ComposableNode__package", "")
        )
        # launch_ros hands a component the group scoped `SetParameter` values and
        # `SetRemap` rules before its own, exactly as it does for a process, in
        # `get_composable_node_load_request`. Without them every component of a composed
        # bringup lost the `use_sim_time` its launch file sets for the whole group.
        global_params = list(self.context.launch_configurations.get("global_params", []) or [])
        global_remaps = list(self.context.launch_configurations.get("ros_remaps", []) or [])
        raw = getattr(description, "_ComposableNode__parameters", ()) or ()
        disc = name or plugin
        params = self._params(raw, global_params, frame, kept=f"ComposableNode({disc})")
        own_remaps = substitutions.perform_pairs(
            self.context, getattr(description, "_ComposableNode__remappings", ()) or ()
        )
        remaps = self._remaps([*global_remaps, *own_remaps], len(global_remaps), frame)
        node = records.ResolvedNode(
            pkg=package,
            executable="",
            namespace=substitutions.namespace_of(composed),
            form="composable_node",
            name=name,
            name_source="literal" if name else "executable_default",
            container=container,
            plugin_class=plugin,
            remaps=remaps,
            params=params,
            entity_path=(*frame.path, f"ComposableNode({disc})"),
            gates=gates,
            conditions=conditions,
            role_content=_role_content((), remaps, params, self.roots),
            site=_action_site(description, frame.file, f"ComposableNode({disc})"),
            attrs={} if reachable else {"unreachable": True},
        )
        node = _finish_node(node, self.roots, self.models.texts(), self.commands.partial)
        self.result.nodes.append(node)

    def _node(self, action, frame: _Frame) -> records.ResolvedNode | None:
        import launch_ros

        gates, conditions, reachable = self._condition_gate(
            getattr(action, "condition", None), frame
        )
        raw_params = tuple(getattr(action, "_Node__parameters", ()) or ())
        # Described before anything is performed: a RewrittenYaml materialises a temp file at
        # evaluation time and a ParameterFile then reports that temp path as its own, so the
        # provenance is gone the moment the substitutions run.
        provenance = self._provenance(raw_params)
        global_params = list(self.context.launch_configurations.get("global_params", []) or [])
        global_remaps = list(self.context.launch_configurations.get("ros_remaps", []) or [])
        stated_name = getattr(action, "_Node__node_name", None)
        performed = True
        failed: BaseException | None = None
        try:
            action._perform_substitutions(self.context)
        except BaseException as exc:
            performed = False
            failed = exc
        package = self._safe(getattr(action, "node_package", None))
        executable = self._safe(getattr(action, "node_executable", None))
        raw_name = getattr(action, "_Node__expanded_node_name", None)
        raw_namespace = getattr(action, "_Node__expanded_node_namespace", None)
        name = substitutions.name_of(raw_name)
        disc = name or executable or "unknown"
        kept = f"{type(action).__name__}({disc})"
        namespace = _namespace_of(raw_namespace, performed)
        seen = len(self.result.exceptions)
        if performed:
            expanded_remaps = list(getattr(action, "_Node__expanded_remappings", None) or [])
        else:
            # launch_ros expands the remappings last, so a raise before them leaves them unset.
            try:
                own = getattr(action, "_Node__remappings", ()) or ()
                expanded_remaps = [*global_remaps, *substitutions.perform_pairs(self.context, own)]
            except BaseException as exc:
                expanded_remaps = list(global_remaps)
                self._raised("visit", exc, frame.file, _label(action), kept)
        remaps = self._remaps(expanded_remaps, len(global_remaps), frame)
        params = self._params(raw_params, global_params, frame, provenance, kept)
        # launch_ros evaluates the parameters inside `_perform_substitutions` too, so one
        # failing item raises twice. The parameters row is the one that names it.
        if failed is not None and not _restated(self.result.exceptions[seen:], failed):
            self._raised("visit", failed, frame.file, _label(action), kept)
        try:
            argv = tuple(
                substitutions.perform_argv(
                    self.context, getattr(action, "_Node__arguments", ()) or ()
                )
            )
        except BaseException as exc:
            argv = ()
            performed = False
            self._raised("visit", exc, frame.file, _label(action), kept)
        form = "lifecycle_node" if isinstance(action, launch_ros.actions.LifecycleNode) else "node"
        attrs: dict[str, Any] = {}
        if not reachable:
            attrs["unreachable"] = True
        if not performed:
            attrs["expansion_failed"] = True
        if raw_name == substitutions.UNSPECIFIED_NAME:
            attrs["name_sentinel"] = substitutions.UNSPECIFIED_NAME
        if raw_namespace == substitutions.UNSPECIFIED_NAMESPACE:
            attrs["namespace_sentinel"] = substitutions.UNSPECIFIED_NAMESPACE
        node = records.ResolvedNode(
            pkg=package,
            executable=executable,
            namespace=namespace,
            form=form,
            name=name,
            name_source=_name_source(stated_name, name),
            remaps=remaps,
            params=params,
            argv=argv,
            entity_path=(*frame.path, f"{type(action).__name__}({disc})"),
            gates=gates,
            conditions=conditions,
            role_content=_role_content(argv, remaps, params, self.roots),
            site=_action_site(action, frame.file, f"{type(action).__name__}({disc})"),
            attrs=attrs,
        )
        node = _finish_node(node, self.roots, self.models.texts(), self.commands.partial)
        self.result.nodes.append(node)
        if form == "lifecycle_node":
            self._lifecycle_nodes.append((action, node))
        return node

    def _process(self, action, frame: _Frame) -> None:
        gates, conditions, reachable = self._condition_gate(
            getattr(action, "condition", None), frame
        )
        try:
            argv = substitutions.perform_argv(self.context, getattr(action, "cmd", ()) or ())
        except BaseException as exc:
            self._raised("visit", exc, frame.file, _label(action))
            return
        self._outputs(argv)
        if not _is_ros_argv(argv):
            return
        parsed = ros_args.parse_ros_args(argv)
        package, executable = _executable_of(argv)
        disc = parsed.node_name or executable or "process"
        # launch computes no namespace for a process and pushes none into it, so only argv
        # can state one. Reading root here would claim a namespace nothing tracked, and a
        # ROS_NAMESPACE in the launch environment would make the claim wrong.
        base = records.ResolvedNode(
            pkg=package,
            executable=executable,
            namespace="",
            form="process",
            name_source="executable_default",
            argv=tuple(argv),
            entity_path=(*frame.path, f"ExecuteProcess({disc})"),
            gates=gates,
            conditions=conditions,
            site=_action_site(action, frame.file, f"ExecuteProcess({disc})"),
            attrs={} if reachable else {"unreachable": True},
        )
        node = ros_args.apply_ros_args(base, parsed)
        node = replace(
            node,
            role_content=_role_content(node.argv, node.remaps, node.params, self.roots),
        )
        node = _finish_node(node, self.roots, self.models.texts(), self.commands.partial)
        self.result.nodes.append(node)

    # ------------------------------------------------------------------
    # pieces of a node

    def _condition_gate(self, condition, frame: _Frame) -> tuple[tuple, tuple, bool]:
        """Gate trail and reachability for one action's own condition.

        A node whose condition is false stays in the record, with the gate that killed it.
        Dropping it loses the fact that the launch file can start it at all. A condition the
        walk could not evaluate is not a false one: the gate says unknown and the node stays
        reachable, because claiming it never starts is a value the walk does not have.
        """
        if condition is None:
            return frame.gates, (), True
        start = len(self.commands.performed)
        outcome = substitutions.evaluate_condition(condition, self.context)
        commands = list(self.commands.performed[start:])
        for key, name in vars(condition).items():
            if key.endswith("__launch_configuration_name"):
                value = self.context.launch_configurations.get(name, "")
                if isinstance(value, str) and (command := _command_text(value)) is not None:
                    commands.append(command)
        commands = tuple(sorted(set(commands)))
        if commands:
            outcome = None
        text = substitutions.describe_condition(condition)
        gate = records.Gate(
            mechanism=substitutions.condition_mechanism(condition),
            expression=text,
            outcome="unknown" if outcome is None else str(bool(outcome)).lower(),
            commands=commands,
            site=records.Site(frame.file, "condition"),
        )
        return (*frame.gates, gate), (text,), outcome is None or bool(outcome)

    def _remaps(self, pairs, globals_count: int, frame: _Frame) -> tuple:
        out = []
        for index, pair in enumerate(pairs):
            left, right = pair
            out.append(
                records.Remap(
                    frm=str(left),
                    to=str(right),
                    scope="global" if index < globals_count else "node",
                    site=getattr(pair, "site", None) or records.Site(frame.file, "remappings"),
                )
            )
        return tuple(out)

    def _provenance(self, raw) -> list:
        """Describe every raw parameter item, before any of them is evaluated.

        Memoised by object identity because one `ParameterFile` is routinely shared by every
        node in a launch file, and it caches its evaluated temp path on first use.
        """
        out = []
        for item in raw or ():
            key = id(item)
            if key not in self._described_cache:
                self._described_cache[key] = self._described(item, 0)
            out.append(self._described_cache[key])
        return out

    def _params(self, raw, global_params, frame: _Frame, provenance=None, kept="") -> tuple:
        from launch_ros.utilities import evaluate_parameters

        if provenance is None:
            provenance = self._provenance(raw)
        out: list[records.ParamSource] = []
        order = 0
        for item in global_params:
            source = self._global_param(item, order, frame)
            if source is not None:
                out.append(source)
                order += 1
        evaluated: tuple = ()
        if raw:
            try:
                evaluated = evaluate_parameters(self.context, raw)
            except BaseException as exc:
                self._raised("visit", exc, frame.file, "parameters", kept)
                evaluated = ()
        for index, item in enumerate(raw):
            value = evaluated[index] if index < len(evaluated) else None
            described = provenance[index] if index < len(provenance) else None
            source = self._param(item, value, order, frame, described)
            if source is not None:
                out.append(source)
                order += 1
        return tuple(out)

    def _global_param(self, item, order: int, frame: _Frame) -> records.ParamSource | None:
        if isinstance(item, tuple) and len(item) == 2:
            return records.ParamSource(
                order=order,
                kind="inline_dict",
                values=((str(item[0]), _plain(item[1])),),
                note="SetParameter, group scoped",
                site=getattr(item, "site", None) or records.Site(frame.file, "SetParameter"),
            )
        path = str(item)
        return records.ParamSource(
            order=order,
            kind=self._file_kind(path),
            path=path,
            note="group scoped parameters file",
            site=getattr(item, "site", None) or records.Site(frame.file, "SetParametersFromFile"),
        )

    def _param(
        self, item, value, order: int, frame: _Frame, described=None
    ) -> records.ParamSource | None:
        if described is None:
            described = self._described(item, order)
        text = _item_text(item)
        described = None if described is None else replace(described, order=order, text=text)
        if described is not None:
            if described.kind == "rewritten_file":
                # An absent root key and an empty one are different statements about the
                # launch file, so whichever the shim read is passed on unchanged.
                return replace(described, site=records.Site(frame.file, "parameters"))
            path = described.path or (str(value) if value is not None else "")
            return replace(
                described,
                kind=self._file_kind(path),
                path=path,
                site=records.Site(frame.file, "parameters"),
            )
        if isinstance(value, dict):
            return records.ParamSource(
                order=order,
                kind="inline_dict",
                values=tuple(sorted((str(key), _plain(inner)) for key, inner in value.items())),
                site=records.Site(frame.file, "parameters"),
            )
        if value is not None and hasattr(value, "evaluate") and not isinstance(value, Path):
            try:
                key, evaluated = value.evaluate(self.context)
            except BaseException:
                return None
            return records.ParamSource(
                order=order,
                kind="inline_dict",
                values=((str(key), _plain(evaluated)),),
                note="a single Parameter description",
                site=records.Site(frame.file, "parameters"),
            )
        if value is None:
            return None
        path = str(value)
        return records.ParamSource(
            order=order,
            kind=self._file_kind(path),
            path=path,
            site=records.Site(frame.file, "parameters"),
            text=text,
        )

    def _described(self, item, order: int) -> records.ParamSource | None:
        """Provenance for one `parameters=` item, file or rewritten file.

        The shared shim is the only place the duck typing lives. It takes a resolver, so the
        context this walk holds is what turns a rewrite value that is a substitution into
        text.
        """
        from rosio.resolve import rewritten_yaml

        try:
            source = rewritten_yaml.describe(item, resolve=self._resolver())
        except BaseException:
            return None
        if source is None:
            return None
        if source.kind == "rewritten_file" and not substitutions.usable_rewrites(source):
            return None
        return replace(source, order=order)

    def _resolver(self):
        def resolve(value):
            if value is None:
                return None
            try:
                return substitutions.perform(self.context, value)
            except BaseException:
                return None

        return resolve

    def _file_kind(self, path: str) -> str:
        """A parameters file inside a tracked root is provenance, one outside it is data.

        A launcher that merges yaml into a temp file hands the node a path that exists in no
        repository, and calling that a repository file would point every reader at nothing.
        The temp roots are the union every writer on this box could have used, not this
        process's own `TMPDIR`, so a file another process wrote under a different temp root
        is external here too and the fallback below cannot claim it.
        """
        if identity.under_temp(path, self.roots):
            return "external_file"
        if not self.roots:
            return "repo_file"
        for root in self.roots:
            if _under(path, root):
                return "repo_file"
        return "external_file"

    def _safe(self, value) -> str:
        try:
            return substitutions.perform(self.context, value)
        except BaseException:
            return ""

    def _outputs(self, argv) -> None:
        """Record the temp files this process names as its output, by the command it runs.

        nav2's simulation launch files hand `xacro -o` a `tempfile.mktemp` name and read or
        remove that file later. The walk starts no process, so the file never exists and its
        name is random per run.
        """
        words = [str(word) for word in argv]
        for flag, path in itertools.pairwise(words):
            if flag in OUTPUT_FLAGS and identity.under_temp(path, self.roots):
                self.produced[path] = " ".join(
                    OUTPUT_TOKEN
                    if word == path
                    else identity.relative_path(word, self.roots)
                    if word.startswith("/")
                    else word
                    for word in words
                )

    def _unrun_outputs(self) -> None:
        """Restate a failure on a file an unrun process writes as what it is."""
        for index, filename, locator in self._touched:
            command = self.produced.get(filename)
            if command is None:
                continue
            failure = self.result.exceptions[index]
            head = f"{locator}: " if locator else ""
            self.result.exceptions[index] = replace(
                failure,
                exc_type=launch_exec.UNRUN_OUTPUT,
                message=(
                    f"{head}{failure.exc_type} on a file the process `{command}` writes when "
                    "it runs, and the walk starts no process"
                ),
            )

    def _raised(
        self, phase: str, exc: BaseException, path: str, locator: str = "", kept: str = ""
    ) -> None:
        message = str(exc) or type(exc).__name__
        if (
            self.guard.sandbox
            and isinstance(exc, OSError)
            and exc.errno
            in (
                errno.EROFS,
                errno.EACCES,
                errno.EPERM,
                errno.ENOENT,
                errno.ENETUNREACH,
                errno.ECONNREFUSED,
            )
        ):
            message = f"{exc.strerror or type(exc).__name__}. {launch_exec.SANDBOX_REFUSED}"
        if isinstance(exc, OSError) and exc.filename:
            self._touched.append((len(self.result.exceptions), str(exc.filename), locator))
        if locator:
            message = f"{locator}: {message}"
        self.result.exceptions.append(
            records.WalkException(
                path=path,
                phase=phase,
                exc_type=type(exc).__name__,
                message=message,
                kept=kept,
                env=_unset_environment(exc),
            )
        )


def _unset_environment(exc: BaseException) -> str:
    """The variable an `os.environ[...]` lookup raised on, or an empty string.

    Only a KeyError raised inside `os.environ` itself counts: a dictionary lookup in the launch
    file raises the same type for a different reason, and naming it as the environment would
    send the user to set a variable nothing reads."""
    if not isinstance(exc, KeyError) or len(exc.args) != 1 or not isinstance(exc.args[0], str):
        return ""
    trace = exc.__traceback__
    while trace is not None and trace.tb_next is not None:
        trace = trace.tb_next
    if trace is None or trace.tb_frame.f_code.co_name != "__getitem__":
        return ""
    return exc.args[0] if trace.tb_frame.f_locals.get("self") is os.environ else ""


# ---------------------------------------------------------------------------
# argv


def _is_ros_argv(argv) -> bool:
    """Whether this process argv is a ros node at all.

    An unparsed argv turns a real node into one that looks observed only, which is why the
    parse is not optional. A process with no ros arguments is not a node and gets no entity.
    """
    if not argv:
        return False
    items = [str(item) for item in argv]
    if ros_args.ROS_ARGS in items:
        return True
    return any(item.startswith(("__node:=", "__name:=", "__ns:=")) for item in items)


def _executable_of(argv) -> tuple[str, str]:
    """Package and executable a process argv really runs.

    Three argv shapes on the reference stack: an interpreter with a script, `ros2 run` with a
    package and an executable, and a bare executable.
    """
    items = [str(item) for item in argv]
    first = Path(items[0]).name
    if first.startswith("python") and len(items) > 1:
        for item in items[1:]:
            if item == "-c":
                return "", "python3 -c"
            if not item.startswith("-"):
                return "", Path(item).name
        return "", first
    if first == "ros2" and len(items) > 3 and items[1] == "run":
        return items[2], items[3]
    return "", first


# ---------------------------------------------------------------------------
# small helpers


def _sub_entities(description) -> list:
    entities = getattr(description, "entities", None)
    if entities is None:
        return []
    return list(entities)


def _label(entity) -> str:
    return type(entity).__name__


def _item_text(item) -> str:
    """A parameters item as its launch file spells it, never as it evaluated."""
    fields = substitutions.rewritten_fields(item)
    if fields is None:
        return substitutions.describe_value(substitutions.unwrap_param_file(item))
    rewrites = ", ".join(
        f"{key}={substitutions.describe_value(value)}"
        for key, value in sorted(dict(fields["param_rewrites"]).items(), key=lambda kv: str(kv[0]))
    )
    return (
        f"RewrittenYaml({substitutions.describe_value(fields['source_file'])}, "
        f"root_key={substitutions.describe_value(fields.get('root_key'))}, {{{rewrites}}})"
    )


def _restated(rows, exc: BaseException) -> bool:
    text = str(exc) or type(exc).__name__
    return any(row.exc_type == type(exc).__name__ and row.message.endswith(text) for row in rows)


def _namespace_of(raw: str | None, performed: bool = True) -> str:
    """The namespace of a node, with the launch_ros sentinel read for what it means.

    `Node.UNSPECIFIED_NODE_NAMESPACE` is the value launch_ros leaves in the expanded field
    when neither the action nor an enclosing group stated a namespace, and such a node runs
    in the root namespace. Passing the sentinel text on mints a junk name, and blanking it
    loses the fact that the fully qualified name is known. When the expansion failed the
    field says nothing at all, so the namespace stays unknown.
    """
    if raw == substitutions.UNSPECIFIED_NAMESPACE:
        return ROOT if performed else ""
    return substitutions.namespace_of(raw)


def _container_namespace(target) -> str:
    """Namespace of a `LoadComposableNodes` target container.

    The expanded field carries the sentinel from construction until the container action
    runs, so reading it early cannot tell a container that stated no namespace from one whose
    namespace has not been computed yet. `__final_node_name` is the flag launch_ros itself
    tests before it trusts the container name, so it decides here too.
    """
    if getattr(target, "_Node__final_node_name", None) is None:
        return ""
    return _namespace_of(getattr(target, "expanded_node_namespace", None))


def _name_source(stated, name: str | None) -> str:
    """Why the node carries the name it carries.

    A stated name that did not survive its substitutions leaves the walk with no name at all,
    and calling that literal would claim a name nobody read.
    """
    if stated is None:
        return "executable_default"
    return "literal" if name else "unknown"


def _plain(value):
    """JSON safe copy of an evaluated parameter value, with the type preserved.

    A value the process boundary cannot carry is marked rather than dropped and rather
    than allowed to end the walk. `collect/params.py` reads the mark and records the one
    value as a blind spot, so the file keeps every node and every other parameter it has.
    """
    if isinstance(value, str):
        command = _command_text(value)
        return uncarried(command, "command_substitution") if command is not None else value
    if isinstance(value, (bool, int)) or value is None:
        return value
    if isinstance(value, float):
        reason = model.unrepresentable(value)
        return uncarried(repr(value), reason) if reason else value
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    if isinstance(value, dict):
        for key in value:
            if isinstance(key, str) and (command := _command_text(key)) is not None:
                return uncarried(command, "command_substitution")
        return {str(key): _plain(item) for key, item in value.items()}
    return str(value)


def _finish_node(
    node: records.ResolvedNode,
    roots,
    texts: dict[str, set[str]] | None = None,
    partial: set[str] | frozenset[str] = frozenset(),
) -> records.ResolvedNode:
    blinds = [(gate.command_role, command) for gate in node.gates for command in gate.commands]

    def known(value, role):
        if not isinstance(value, str):
            return value
        command = _command_text(value)
        if command is None:
            return value
        for match in _COMMAND.finditer(value):
            text = _command_text(match[0])
            if text is not None:
                blinds.append((role, text))
        return ""

    fields = {
        key: known(getattr(node, key), key)
        for key in ("name", "namespace", "pkg", "executable", "plugin_class", "container")
    }
    argv = tuple(known(raw, "arguments") for raw in node.argv)
    remaps = []
    for remap in node.remaps:
        frm, to = known(remap.frm, "remappings"), known(remap.to, "remappings")
        if frm and to:
            remaps.append(replace(remap, frm=frm, to=to))
    params = []
    for source in node.params:
        path = known(source.path, "parameters")
        source_path = known(source.source_path, "parameters")
        if (source.path and not path) or (source.source_path and not source_path):
            continue
        values = tuple(
            (name, value) for raw, value in source.values if (name := known(raw, "parameter_name"))
        )
        site = source.site
        if site and node.site and site.path == node.site.path and site.line_hint is None:
            site = replace(site, line_hint=node.site.line_hint)
        rewrites = tuple(
            (key, _plain(value))
            for raw, value in source.rewrites
            if (key := known(raw, "parameter_name"))
        )
        params.append(
            replace(
                source,
                path=path,
                source_path=source_path,
                values=values,
                root_key=_plain(source.root_key),
                rewrites=rewrites,
                site=site,
            )
        )
    node = replace(
        node,
        **fields,
        argv=argv,
        remaps=tuple(remaps),
        params=tuple(params),
        command_blinds=tuple(sorted(set(blinds))),
    )
    if blinds:
        disc = node.name or node.executable or node.plugin_class or "unknown"
        action = node.entity_path[-1].partition("(")[0]
        node = replace(node, entity_path=(*node.entity_path[:-1], f"{action}({disc})"))
        if node.site:
            node = replace(node, site=node.site.with_locator(f"{action}({disc})"))
    role = _role_content(node.argv, node.remaps, node.params, roots)
    for key, expr in node.command_blinds:
        text = substitutions.command_text(expr, lambda path: identity.relative_path(path, roots))
        role += (f"command:{key}:{text}",)
    return replace(node, file_reads=_file_reads(node, texts, partial), role_content=role)


def _file_reads(
    node: records.ResolvedNode,
    texts: dict[str, set[str]] | None = None,
    partial: set[str] | frozenset[str] = frozenset(),
) -> tuple[records.FileRead, ...]:
    reads: set[tuple[str, str]] = set()
    models: set[tuple[str, tuple[tuple[str, str], ...], str]] = set()

    def commands(value):
        marked = uncarried_reason(value)
        if marked is not None and marked[1] == "command_substitution":
            try:
                words = shlex.split(marked[0])
            except ValueError:
                return
            if len(words) > 1 and Path(words[0]).name == "xacro" and Path(words[1]).is_absolute():
                blind = marked[0] if any(text in marked[0] for text in partial) else ""
                # xacro folds its mappings into a dict, so a repeated name takes its last value
                args = tuple(
                    sorted(
                        {
                            key: stated
                            for word in words[2:]
                            if not word.startswith("-")
                            for key, sep, stated in [word.partition(":=")]
                            if sep and key
                        }.items()
                    )
                )
                models.add((words[1], () if blind else args, blind))
        elif isinstance(value, (list, tuple)):
            for item in value:
                commands(item)
        elif isinstance(value, dict):
            for item in value.values():
                commands(item)

    for source in node.params:
        path = source.source_path if source.kind == "rewritten_file" else source.path
        if path:
            reads.add((path, "parameters_file"))
        for name, value in source.values:
            commands(value)
            if name.rsplit(".", 1)[-1] == "robot_description" and isinstance(value, str):
                paths = (texts or {}).get(value, ())
                if len(paths) == 1:
                    reads.add((next(iter(paths)), "robot_model"))
    for index, word in enumerate(node.argv):
        if Path(word).is_absolute():
            flag = node.argv[index - 1] if index else ""
            role = "rviz_configuration" if flag == "-d" else "node_argument"
            if flag == "--params-file":
                role = "parameters_file"
            reads.add((word, role))
    out = [records.FileRead(path, role, node.site) for path, role in reads]
    out.extend(
        records.FileRead(path, "robot_model", node.site, xacro_args=args, blind=blind)
        for path, args, blind in models
    )
    return tuple(sorted(out, key=lambda item: (item.path, item.role, item.xacro_args, item.blind)))


UNCARRIED = "~uncarried"
"""Marks a value the walk read and the process boundary cannot carry, with the text the
source states and the reason. Both ends of this channel are the sandbox child and
`collect/params.py`, and the mark never reaches the atlas: the parameter collector turns
it into an unresolved entry, which is what a value nobody can write has to become."""


def uncarried(expr: str, reason: str) -> dict[str, dict[str, str]]:
    return {UNCARRIED: {"expr": expr, "reason": reason}}


def uncarried_reason(value) -> tuple[str, str] | None:
    """(expression, reason) when a value carries the mark, None otherwise."""
    if not isinstance(value, dict) or list(value) != [UNCARRIED]:
        return None
    inner = value[UNCARRIED]
    if not isinstance(inner, dict):
        return None
    return str(inner.get("expr", "")), str(inner.get("reason", ""))


def _scrub(payload):
    """Last guard before the boundary: mark every value json refuses, anywhere.

    `_plain` covers the parameter values this has been measured on. The walk carries
    timer periods and argument defaults too, and one of those ending the walk would cost
    the whole file for a reason that belongs to one number.
    """
    if isinstance(payload, float):
        reason = model.unrepresentable(payload)
        return (uncarried(repr(payload), reason), [reason]) if reason else (payload, [])
    marked = uncarried_reason(payload)
    if marked is not None:
        return payload, [marked[1]]
    if isinstance(payload, list):
        found: list[str] = []
        out = []
        for item in payload:
            value, reasons = _scrub(item)
            out.append(value)
            found.extend(reasons)
        return out, found
    if isinstance(payload, dict):
        found = []
        out = {}
        for key, item in payload.items():
            value, reasons = _scrub(item)
            out[key] = value
            found.extend(reasons)
        return out, found
    return payload, []


def _without_outputs(payload, produced: dict[str, str]):
    """Every spelling of a file an unrun process writes, replaced by the command writing it."""
    pairs = []
    for path, command in sorted(produced.items()):
        token = f"{OUTPUT_TOKEN}({command})"
        pairs += [(path, token), (f"{identity.TEMP_TOKEN}/{Path(path).name}", token)]

    def clean(value):
        if isinstance(value, str):
            for old, new in pairs:
                value = value.replace(old, new)
            return value
        if isinstance(value, list):
            return [clean(item) for item in value]
        if isinstance(value, dict):
            return {clean(key): clean(item) for key, item in value.items()}
        return value

    return clean(payload) if pairs else payload


def _role_content(argv, remaps, params, roots):
    def stated(value):
        mark = uncarried_reason(value)
        if mark is not None and mark[1] == "command_substitution":
            return uncarried(
                substitutions.command_text(
                    mark[0], lambda path: identity.relative_path(path, roots)
                ),
                mark[1],
            )
        if isinstance(value, dict):
            return {key: stated(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [stated(item) for item in value]
        return value

    params = [replace(p, values=tuple((key, stated(v)) for key, v in p.values)) for p in params]
    return identity.role_content(argv, remaps, params, roots)


EXTERNAL_TOKEN = identity.EXTERNAL_TOKEN


def _under(path: str, base: str) -> bool:
    return path == base or path.startswith(base.rstrip("/") + "/")


_REPR_MARK = " object at 0x"


def _location(source) -> tuple[str, bool]:
    """The included file, or a stable description of the expression that did not fold.

    A python repr carries a memory address, and this text reaches the digest of a blind
    spot, so two runs over one unchanged file would key it under two ids.
    """
    raw = getattr(source, "location", "") or ""
    text = str(raw)
    if text and _REPR_MARK not in text:
        return text, True
    if _REPR_MARK in text:
        head, _sep, _tail = text.partition(_REPR_MARK)
        return f"{head}>", False
    return substitutions.describe_value(raw), False


def _join(namespace: str, name: str) -> str:
    from rosio import ids

    return ids.join_namespace(namespace, name)


def _fqn(node: records.ResolvedNode) -> str | None:
    if node.name is None or not node.namespace:
        return None
    return _join(node.namespace, node.name)


def _order(node: records.ResolvedNode):
    return (node.namespace, node.name or "", node.pkg, node.executable, node.plugin_class or "")


def main() -> int:
    request = json.loads(sys.stdin.read() or "{}")
    walker = Walker(request)
    try:
        result = walker.run()
    except BaseException as exc:
        result = walker.result
        result.loadable = False
        result.strategy = "refused"
        result.exceptions.append(
            records.WalkException(
                path=walker.entry,
                phase="visit",
                exc_type=type(exc).__name__,
                message=str(exc) or type(exc).__name__,
            )
        )
    payload = _without_outputs(launch_exec.encode_walk(result), walker.produced)
    scrubbed, refused = _scrub(payload)
    if refused:
        notes = list(scrubbed.get("notes", ()))
        for reason in sorted(set(refused)):
            note = f"a value of this file did not cross the sandbox boundary: {reason}"
            if note not in notes:
                notes.append(note)
        scrubbed["notes"] = notes
        payload = scrubbed
    sys.stdout.write("\n" + launch_exec.MARKER + "\n")
    sys.stdout.write(model.dumps(payload))
    sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
