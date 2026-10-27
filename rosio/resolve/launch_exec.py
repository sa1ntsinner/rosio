"""Resolve a launch entry point into the node set it really describes.

Two strategies, one record shape.

`walk` executes the launch description in a child process. That is the only way to get a
right answer on a real stack: on the reference workspace four launch files put every node
inside an `OpaqueFunction` and two hide their whole payload behind chained `OnProcessExit`
callbacks, so a reader of the syntax sees an empty file. The child never starts a process,
it builds the description, performs substitutions and walks the entities.

`walk_ast` reads the source text instead and reports only what it can see literally. It
exists for the case with no ROS on the path, and it is deliberately poorer: a value it
cannot see is a value it refuses to guess.

The parent process never imports `launch`. It talks to the child over one json object in
each direction, so a launch file that raises, hangs or leaves the interpreter in a bad state
costs one walk and not the run.
"""

from __future__ import annotations

import ast
import json
import os
import shlex
import shutil
import subprocess
import threading
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

from rosio import ids, model
from rosio.errors import CollectorError
from rosio.resolve import parse_python, records, substitutions

if TYPE_CHECKING:
    from rosio.workspace import Workspace

CHILD = str(Path(__file__).with_name("_launch_child.py"))

MARKER = "__rosio_launch_walk__"
"""Sentinel before the json object. A login shell can print a banner, and a launch file can
print anything at all, so the payload is found rather than assumed to be all of stdout."""

DEFAULT_TIMEOUT = 60.0

SANDBOX_REQUIRED = (
    "launch_sandbox required needs a working bwrap (apt install bubblewrap). "
    "The tool is missing or the operating system refused its namespaces"
)
SANDBOX_REFUSED = "The launch sandbox refused this access. --launch-sandbox off disables it."

UNRUN_OUTPUT = "UnrunProcessOutput"
"""The exception type of a walk step that touched a file a process of the same launch file
writes when it runs. The walk starts no process, so the file never exists, in the sandbox or
out of it, and that is a blind spot of the walk rather than a launch file that raised."""


def _sandbox_command(command: str) -> list[str]:
    return [
        command,
        "--ro-bind",
        "/",
        "/",
        "--tmpfs",
        "/tmp",
        "--dev",
        "/dev",
        "--proc",
        "/proc",
        "--unshare-net",
        "--unshare-pid",
        "--die-with-parent",
        "--new-session",
    ]


def probe_sandbox(ros_setup: Sequence[str] = ()) -> str:
    """The bwrap command, only after the required namespaces and mounts worked."""
    env = sourced_environment(ros_setup)
    command = shutil.which("bwrap", path=(env or os.environ).get("PATH"))
    if command is None:
        return ""
    try:
        done = subprocess.run(
            [*_sandbox_command(command), "--", "/bin/true"],
            env=env,
            capture_output=True,
            timeout=10,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return ""
    return command if done.returncode == 0 else ""


ROS1_MARKERS = ("<rosparam", "clear_params", "launch-prefix", "$(find ", "$(arg ")

NODE_CALLS = ("Node", "LifecycleNode")

NOTE_ROS1 = "ROS 1 XML launch syntax, refused by shape and never parsed"
NOTE_MARKUP = (
    "an XML or YAML ROS 2 launch frontend, which this walk does not execute: only a python "
    "launch description is executed"
)
NOTE_UNAVAILABLE = "the launch machinery could not be imported"
"""Three refusal notes with a stable head, so a collector can tell them apart without
guessing from prose. A markup frontend is a supported ROS 2 launch format that this walk
declines to run, which is a different sentence from a ROS 1 file that cannot run at all."""

SETUP_FIX = (
    "source a ROS 2 setup file before running rosio, or name it under ros_setup in the "
    "configuration"
)
VENV_FIX = (
    "the interpreter that walks launch files is a virtual environment that hides the system "
    "packages, and ROS 2 installs part of its own python stack as operating system packages. "
    "Recreate it with python3 -m venv --system-site-packages <dir> and install rosio "
    "into it again"
)
GENERIC_FIX = "make python3 able to import launch and launch_ros in a login shell"

PROBE_SCRIPT = "\n".join(
    (
        "import json, os, sys",
        "out = {",
        "    'venv': sys.prefix != sys.base_prefix,",
        "    'system_site': any(",
        "        str(item).startswith(sys.base_prefix)",
        "        and str(item).endswith(('dist-packages', 'site-packages'))",
        "        for item in sys.path",
        "    ),",
        "    'ros': bool(os.environ.get('AMENT_PREFIX_PATH')),",
        "}",
        "try:",
        "    import launch",
        "    import launch_ros",
        "except BaseException as exc:",
        "    out['ok'] = False",
        "    out['module'] = getattr(exc, 'name', '') or ''",
        "    out['exc_type'] = type(exc).__name__",
        "    out['message'] = str(exc)",
        "else:",
        "    out['ok'] = True",
        f"print({MARKER!r})",
        "print(json.dumps(out))",
    )
)
"""Asked once per setup file set. No path from the probed box is reported back, because the
answer is written into the atlas and a prefix is machine dependent."""

_probes: dict[tuple[str, ...], LaunchImport] = {}

_environments: dict[tuple[str, ...], dict[str, str] | None] = {}
"""The environment a login shell has after sourcing one setup file set, read once per run.

Sourcing a ROS setup takes most of half a second, and a workspace with twenty entry points
used to pay it twenty times. Every child gets the same environment it would have built itself.
"""

SHELL_STATE = frozenset({"_", "SHLVL", "PWD", "OLDPWD"})
"""Variables a shell keeps about itself rather than about the setup it sourced."""

ENV_SCRIPT = f"import json, os; print({MARKER!r} + json.dumps(dict(os.environ)))"

IMPORTABLE_SCRIPT = (
    "import importlib.util, json, sys; "
    f"print({MARKER!r} + json.dumps("
    "{'found': [n for n in sys.argv[1:] if importlib.util.find_spec(n)]}))"
)

_importable: dict[tuple[tuple[str, ...], tuple[str, ...]], frozenset[str] | None] = {}


@dataclass(frozen=True)
class LaunchImport:
    """Whether a child of this box can import the launch packages, and what to do when not.

    A boolean was not enough: the common failure is neither a missing ROS nor a broken launch
    file, it is an interpreter that hides the operating system packages launch depends on,
    and a run that cannot name that produces a thin atlas that reads as a complete one.
    """

    ok: bool
    module: str = ""
    exc_type: str = ""
    message: str = ""
    fix: str = ""

    @property
    def note(self) -> str:
        if self.ok:
            return ""
        detail = ": ".join(part for part in (self.exc_type, self.message) if part)
        return f"{NOTE_UNAVAILABLE}: {detail}" if detail else NOTE_UNAVAILABLE

    @property
    def summary(self) -> str:
        return f"{self.note}. {self.fix}" if self.fix else self.note


# ---------------------------------------------------------------------------
# the executed walk


def sourced_environment(ros_setup: Sequence[str] = ()) -> dict[str, str] | None:
    """What a login shell exports after sourcing these setup files, or None when it failed.

    None sends every child back to sourcing for itself, which is slower and says why it failed.
    """
    key = tuple(ros_setup)
    if key in _environments:
        return _environments[key]
    script = f"exec python3 -c {shlex.quote(ENV_SCRIPT)}"
    try:
        done = subprocess.run(
            ["bash", "-lc", _sourced(ros_setup, script)],
            capture_output=True,
            text=True,
            timeout=120,
            check=False,
        )
        payload = _payload(done.stdout) if done.returncode == 0 else None
    except (OSError, subprocess.SubprocessError):
        payload = None
    env = {str(k): str(v) for k, v in payload.items() if k not in SHELL_STATE} if payload else None
    _environments[key] = env
    return env


def _python(ros_setup: Sequence[str], script: str) -> tuple[list[str], dict[str, str] | None]:
    """The command and environment that run a python3 command line in the sourced setup.

    `script` is the argument list after `python3`, already quoted for a shell.
    """
    env = sourced_environment(ros_setup)
    if env is None:
        return ["bash", "-lc", _sourced(ros_setup, f"exec python3 {script}")], None
    return ["bash", "-c", f"exec python3 {script}"], env


def importable(names: Sequence[str], ros_setup: Sequence[str] = ()) -> frozenset[str] | None:
    """The modules of `names` the interpreter of the launch walk finds, after sourcing these
    setup files, or None when it could not be asked. rosio's own interpreter can sit in a
    virtual environment that sees none of the ROS packages the walk imports."""
    key = (tuple(ros_setup), tuple(sorted(names)))
    if key in _importable:
        return _importable[key]
    arguments = " ".join(shlex.quote(name) for name in key[1])
    command, child_env = _python(ros_setup, f"-c {shlex.quote(IMPORTABLE_SCRIPT)} {arguments}")
    try:
        done = subprocess.run(
            command, env=child_env, capture_output=True, text=True, timeout=120, check=False
        )
        payload = _payload(done.stdout) if done.returncode == 0 else None
    except (OSError, subprocess.SubprocessError):
        payload = None
    found = frozenset(str(name) for name in payload["found"]) if payload else None
    _importable[key] = found
    return found


def probe(ros_setup: Sequence[str] = ()) -> LaunchImport:
    """Ask once whether the executed walk can be used at all, and why when it cannot."""
    key = tuple(ros_setup)
    cached = _probes.get(key)
    if cached is not None:
        return cached
    command, child_env = _python(ros_setup, f"-c {shlex.quote(PROBE_SCRIPT)}")
    try:
        done = subprocess.run(
            command,
            env=child_env,
            capture_output=True,
            text=True,
            timeout=120,
            check=False,
        )
        tail = (done.stderr or "").strip().splitlines()[-1:]
        report = import_report(
            _payload(done.stdout) or {},
            fallback=" ".join(tail) or f"the probe exited with {done.returncode}",
        )
    except (OSError, subprocess.SubprocessError) as exc:
        report = import_report({}, fallback=f"{type(exc).__name__}: {exc}")
    _probes[key] = report
    return report


def import_report(payload: Mapping[str, Any], fallback: str = "") -> LaunchImport:
    """The probe payload as a decision, with the fix that matches what the child reported."""
    if payload.get("ok"):
        return LaunchImport(ok=True)
    module = str(payload.get("module") or "")
    if not payload.get("ros"):
        fix = SETUP_FIX
    elif payload.get("venv") and not payload.get("system_site"):
        fix = VENV_FIX
    elif module:
        fix = f"install {module} into the interpreter that runs the launch walk"
    else:
        fix = GENERIC_FIX
    return LaunchImport(
        ok=False,
        module=module,
        exc_type=str(payload.get("exc_type") or "ChildFailed"),
        message=str(payload.get("message") or fallback),
        fix=fix,
    )


def available(ros_setup: Sequence[str] = ()) -> bool:
    """Whether a child sourcing these setup files can import launch and launch_ros."""
    return probe(ros_setup).ok


class Children:
    """The launch children of one run, so a run that stops takes its children with it.

    A walk asked for after `stop` raises rather than starting a process.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._live: set[subprocess.Popen[str]] = set()
        self._stopped = False

    def run(
        self, command: list[str], env: Mapping[str, str] | None, text: str, timeout: float
    ) -> subprocess.CompletedProcess[str]:
        with self._lock:
            if self._stopped:
                raise CollectorError("the run stopped before this launch walk started")
            process = subprocess.Popen(
                command,
                env=env,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            self._live.add(process)
        try:
            with process:
                try:
                    stdout, stderr = process.communicate(text, timeout=timeout)
                except BaseException:
                    process.kill()
                    raise
        finally:
            with self._lock:
                self._live.discard(process)
        return subprocess.CompletedProcess(command, process.returncode, stdout, stderr)

    def stop(self) -> None:
        with self._lock:
            self._stopped = True
            for process in self._live:
                process.kill()


NOTE_SOURCE_SHARE = "the share directory of {} was read from its source tree"

MISSING_SHARE = "/@package_not_found"
"""The directory a package lookup is answered with while a launch file builds its
description and no tree holds the package. Nothing exists under it, so whatever reads from it
fails on its own, and the walk keeps every action that does not."""

PACKAGE_NOT_FOUND = "package_not_found"
"""The blind spot reason of an include that reads from a package nothing holds."""


def source_shares(workspace: Workspace | None) -> dict[str, str]:
    """The packages a source root holds and no install root does, by name.

    The child answers a share lookup the ament index cannot with the package source
    directory, which is the layout `install(DIRECTORY ...)` and `data_files` copy in the
    common case. Only the directory is assumed: a file the source does not hold at the same
    package relative path stays missing, so a flattening or generating install rule reads
    as the refusal it would have been.
    """
    return {
        name: package.src_path
        for name, package in getattr(workspace, "packages", {}).items()
        if package.src_path and not package.share_path
    }


def installed_shares(workspace: Workspace | None) -> dict[str, str]:
    """The share directory of every package an install root holds, by name. A lookup the
    ament index answers from anywhere else read a copy the configured roots do not hold."""
    return {
        name: package.share_path
        for name, package in getattr(workspace, "packages", {}).items()
        if package.share_path
    }


def walk(
    entry_path: str,
    *,
    args: Sequence[tuple[str, str]] = (),
    env: Sequence[tuple[str, str]] = (),
    ros_setup: Sequence[str] = (),
    timeout: float = DEFAULT_TIMEOUT,
    workspace: Workspace | None = None,
    launch_sandbox: str = "auto",
    sandbox_command: str | None = None,
    children: Children | None = None,
) -> records.LaunchWalk:
    """Execute one entry point with one argument set and report what it launches."""
    entry = _abs(entry_path)
    refusal = _refuse(entry, args, env, workspace)
    if refusal is not None:
        return refusal
    sandbox = ""
    if launch_sandbox != "off":
        sandbox = probe_sandbox(ros_setup) if sandbox_command is None else sandbox_command
        if not sandbox and launch_sandbox == "required":
            raise CollectorError(SANDBOX_REQUIRED)
    request = {
        "entry_path": entry,
        "args": [list(pair) for pair in args],
        "env": [list(pair) for pair in env],
        "roots": [root.path for root in getattr(workspace, "roots", ())],
        "source_shares": source_shares(workspace),
        "installed_shares": installed_shares(workspace),
        "timeout": float(timeout),
        "sandbox": bool(sandbox),
    }
    # -B: importing a launch file must not leave __pycache__ in the user's source tree.
    command, child_env = _python(ros_setup, f"-B {shlex.quote(CHILD)}")
    if sandbox:
        prefix = _sandbox_command(sandbox)
        roots = [Path(entry).parent, Path(CHILD).resolve().parents[2]]
        for root in getattr(workspace, "roots", ()):
            # A symlink install points into the build tree beside it.
            path = Path(root.path)
            roots.append(path.parent if root.kind == "install" else path)
        # Inputs stored under /tmp stay readable, while the rest of host /tmp stays hidden.
        for root in sorted(set(roots)):
            if root.is_relative_to("/tmp") and root != Path("/tmp"):
                prefix.extend(["--ro-bind", str(root), str(root)])
        prefix.extend(
            [
                "--setenv",
                "TMPDIR",
                "/tmp",
                "--setenv",
                "ROS_HOME",
                "/tmp/rosio-ros",
                "--setenv",
                "ROS_LOG_DIR",
                "/tmp/rosio-ros/log",
            ]
        )
        command = [*prefix, "--", *command]
    try:
        done = (children or Children()).run(
            command, child_env, model.dumps(request), max(1.0, float(timeout))
        )
    except subprocess.TimeoutExpired:
        return _refused(
            entry,
            args,
            env,
            workspace,
            phase="visit",
            exc_type="TimeoutExpired",
            message=f"the launch child did not finish within {timeout:g} seconds",
            sandboxed=bool(sandbox),
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return _refused(
            entry,
            args,
            env,
            workspace,
            phase="import",
            exc_type=type(exc).__name__,
            message=str(exc),
        )
    payload = _payload(done.stdout)
    if payload is None:
        tail = (done.stderr or done.stdout or "").strip().splitlines()[-3:]
        return _refused(
            entry,
            args,
            env,
            workspace,
            phase="import" if done.returncode else "visit",
            exc_type="ChildFailed",
            message=" ".join(tail) or f"the launch child exited with {done.returncode}",
            sandboxed=bool(sandbox),
        )
    result = decode_walk(payload, workspace=workspace)
    result.sandboxed = bool(sandbox)
    result.entry_path = entry
    result.args = tuple((str(k), str(v)) for k, v in args)
    result.env = tuple((str(k), str(v)) for k, v in env)
    if workspace is not None:
        result.entry_pkg = workspace.package_of(entry)
    result.nodes.sort(key=_node_order)
    result.includes.sort(key=lambda item: (item.path, item.args))
    result.arg_declarations.sort(key=lambda item: item.name)
    return result


# ---------------------------------------------------------------------------
# the text only walk


def walk_ast(
    entry_path: str,
    *,
    args: Sequence[tuple[str, str]] = (),
    workspace: Workspace | None = None,
) -> records.LaunchWalk:
    """Read the source and report only the nodes it states literally.

    Everything here is a refusal to guess. A node whose package or executable is computed at
    run time is counted in a note and left out, because a node with an unknown executable is
    not a node, it is a rumour.
    """
    entry = _abs(entry_path)
    result = _empty(entry, args, (), workspace, strategy="ast")
    refusal = _refuse(entry, args, (), workspace, strategy="ast")
    if refusal is not None:
        return refusal
    try:
        tree = parse_python(
            Path(entry).read_text(encoding="utf-8", errors="replace"), filename=entry
        )
    except (OSError, SyntaxError) as exc:
        result.exceptions.append(
            records.WalkException(
                path=entry, phase="import", exc_type=type(exc).__name__, message=str(exc)
            )
        )
        result.loadable = False
        return result
    values = dict(args)
    declared: list[records.ArgDeclaration] = []
    for call in _calls(tree, ("DeclareLaunchArgument",)):
        name = _ast_text(_arg(call, "name", 0), values)
        if name is None:
            continue
        default = _ast_text(_arg(call, "default_value", 1), values)
        declared.append(
            records.ArgDeclaration(
                name=name,
                default=default,
                description=_ast_text(_arg(call, "description", None), values) or "",
                site=_site(workspace, entry, f"DeclareLaunchArgument({name})", call.lineno),
            )
        )
        if default is not None:
            values.setdefault(name, default)
    result.arg_declarations = sorted(declared, key=lambda item: item.name)
    skipped = 0
    for call in _calls(tree, NODE_CALLS):
        node = _ast_node(call, values, entry, workspace)
        if node is None:
            skipped += 1
            continue
        result.nodes.append(node)
    for call in _calls(tree, ("IncludeLaunchDescription",)):
        folded = _include_text(call, values)
        result.includes.append(
            records.LaunchInclude(
                path=folded if folded is not None else _include_source(call),
                site=_site(workspace, entry, "IncludeLaunchDescription", call.lineno),
                resolved=folded is not None,
            )
        )
    if skipped:
        result.notes.append(f"{skipped} node calls were not literal enough to resolve")
    for name in ("OpaqueFunction", "RegisterEventHandler"):
        if any(True for _ in _calls(tree, (name,))):
            result.notes.append(f"{name} is used here, so the executed walk sees more")
    result.nodes.sort(key=_node_order)
    result.includes.sort(key=lambda item: (item.path, item.args))
    return result


# ---------------------------------------------------------------------------
# json codec, shared with the child


def encode_walk(walk_result: records.LaunchWalk) -> dict[str, Any]:
    return {
        "entry_path": walk_result.entry_path,
        "profile": walk_result.profile,
        "entry_pkg": walk_result.entry_pkg,
        "args": [list(pair) for pair in walk_result.args],
        "env": [list(pair) for pair in walk_result.env],
        "nodes": [_encode_node(node) for node in walk_result.nodes],
        "includes": [
            {
                "path": include.path,
                "entity_path": list(include.entity_path),
                "args": [list(pair) for pair in include.args],
                "site": _encode_site(include.site),
                "resolved": include.resolved,
                "reason": include.reason,
            }
            for include in walk_result.includes
        ],
        "arg_declarations": [
            {
                "name": item.name,
                "default": item.default,
                "description": item.description,
                "site": _encode_site(item.site),
            }
            for item in walk_result.arg_declarations
        ],
        "exceptions": [
            {
                "path": item.path,
                "phase": item.phase,
                "exc_type": item.exc_type,
                "message": item.message,
                **({"kept": item.kept} if item.kept else {}),
                **({"env": item.env} if item.env else {}),
            }
            for item in walk_result.exceptions
        ],
        "predicates": dict(sorted(walk_result.predicates.items())),
        "loadable": walk_result.loadable,
        "strategy": walk_result.strategy,
        "notes": list(walk_result.notes),
        "underlay": dict(sorted(walk_result.underlay.items())),
    }


def decode_walk(
    payload: Mapping[str, Any], *, workspace: Workspace | None = None
) -> records.LaunchWalk:
    return records.LaunchWalk(
        entry_path=str(payload.get("entry_path", "")),
        profile=str(payload.get("profile", "")),
        entry_pkg=payload.get("entry_pkg"),
        args=tuple((str(k), str(v)) for k, v in payload.get("args", ())),
        env=tuple((str(k), str(v)) for k, v in payload.get("env", ())),
        nodes=[_decode_node(item, workspace) for item in payload.get("nodes", ())],
        includes=[
            records.LaunchInclude(
                path=str(item.get("path", "")),
                entity_path=tuple(item.get("entity_path", ())),
                args=tuple((str(k), str(v)) for k, v in item.get("args", ())),
                site=_decode_site(item.get("site"), workspace),
                resolved=bool(item.get("resolved", True)),
                reason=str(item.get("reason", "")),
            )
            for item in payload.get("includes", ())
        ],
        arg_declarations=[
            records.ArgDeclaration(
                name=str(item.get("name", "")),
                default=item.get("default"),
                description=str(item.get("description", "")),
                site=_decode_site(item.get("site"), workspace),
            )
            for item in payload.get("arg_declarations", ())
        ],
        exceptions=[
            records.WalkException(
                path=str(item.get("path", "")),
                phase=str(item.get("phase", "visit")),
                exc_type=str(item.get("exc_type", "Exception")),
                message=str(item.get("message", "")),
                kept=str(item.get("kept", "")),
                env=str(item.get("env", "")),
            )
            for item in payload.get("exceptions", ())
        ],
        predicates={str(k): bool(v) for k, v in dict(payload.get("predicates", {})).items()},
        loadable=bool(payload.get("loadable", True)),
        strategy=str(payload.get("strategy", "executed")),
        notes=[str(note) for note in payload.get("notes", ())],
        underlay={str(k): str(v) for k, v in dict(payload.get("underlay", {})).items()},
    )


def _encode_node(node: records.ResolvedNode) -> dict[str, Any]:
    return {
        "pkg": node.pkg,
        "executable": node.executable,
        "namespace": node.namespace,
        "form": node.form,
        "name": node.name,
        "name_source": node.name_source,
        "container": node.container,
        "plugin_class": node.plugin_class,
        "remaps": [
            {
                "frm": remap.frm,
                "to": remap.to,
                "scope": remap.scope,
                "site": _encode_site(remap.site),
            }
            for remap in node.remaps
        ],
        "params": [
            {
                "order": item.order,
                "kind": item.kind,
                "path": item.path,
                "source_path": item.source_path,
                "root_key": item.root_key,
                "rewrites": [list(pair) for pair in item.rewrites],
                "values": [list(pair) for pair in item.values],
                "produced_by": item.produced_by,
                "site": _encode_site(item.site),
                "note": item.note,
                "text": item.text,
            }
            for item in node.params
        ],
        "argv": list(node.argv),
        "file_reads": [
            {
                "path": item.path,
                "role": item.role,
                "site": _encode_site(item.site),
                **(
                    {"xacro_args": [list(pair) for pair in item.xacro_args]}
                    if item.xacro_args
                    else {}
                ),
                **({"blind": item.blind} if item.blind else {}),
            }
            for item in node.file_reads
        ],
        "command_blinds": [list(pair) for pair in node.command_blinds],
        "entity_path": list(node.entity_path),
        "gates": [
            {
                "mechanism": gate.mechanism,
                "expression": gate.expression,
                "period": gate.period,
                "target": gate.target,
                "outcome": gate.outcome,
                "site": _encode_site(gate.site),
            }
            for gate in node.gates
        ],
        "conditions": list(node.conditions),
        "role_content": list(node.role_content),
        "site": _encode_site(node.site),
        "attrs": dict(sorted(node.attrs.items())),
    }


def _decode_node(payload: Mapping[str, Any], workspace: Workspace | None) -> records.ResolvedNode:
    return records.ResolvedNode(
        pkg=str(payload.get("pkg", "")),
        executable=str(payload.get("executable", "")),
        namespace=str(payload.get("namespace", "")),
        form=str(payload.get("form", "node")),
        name=payload.get("name"),
        name_source=str(payload.get("name_source", "unknown")),
        container=payload.get("container"),
        plugin_class=payload.get("plugin_class"),
        remaps=tuple(
            records.Remap(
                frm=str(item.get("frm", "")),
                to=str(item.get("to", "")),
                scope=str(item.get("scope", "node")),
                site=_decode_site(item.get("site"), workspace),
            )
            for item in payload.get("remaps", ())
        ),
        params=tuple(
            records.ParamSource(
                order=int(item.get("order", 0)),
                kind=str(item.get("kind", "inline_dict")),
                path=item.get("path"),
                source_path=item.get("source_path"),
                root_key=item.get("root_key"),
                rewrites=tuple((str(k), v) for k, v in item.get("rewrites", ())),
                values=tuple((str(k), v) for k, v in item.get("values", ())),
                produced_by=item.get("produced_by"),
                site=_decode_site(item.get("site"), workspace),
                note=str(item.get("note", "")),
                text=str(item.get("text", "")),
            )
            for item in payload.get("params", ())
        ),
        argv=tuple(str(item) for item in payload.get("argv", ())),
        command_blinds=tuple((str(k), str(v)) for k, v in payload.get("command_blinds", ())),
        file_reads=tuple(
            records.FileRead(
                path=str(item["path"]),
                role=str(item["role"]),
                site=_decode_site(item.get("site"), workspace),
                xacro_args=tuple((str(k), str(v)) for k, v in item.get("xacro_args", ())),
                blind=str(item.get("blind", "")),
            )
            for item in payload.get("file_reads", ())
        ),
        entity_path=tuple(str(item) for item in payload.get("entity_path", ())),
        gates=tuple(
            records.Gate(
                mechanism=str(item.get("mechanism", "IfCondition")),
                expression=str(item.get("expression", "")),
                period=item.get("period"),
                target=str(item.get("target", "")),
                outcome=str(item.get("outcome", "")),
                site=_decode_site(item.get("site"), workspace),
            )
            for item in payload.get("gates", ())
        ),
        conditions=tuple(str(item) for item in payload.get("conditions", ())),
        role_content=tuple(str(item) for item in payload.get("role_content", ())),
        site=_decode_site(payload.get("site"), workspace),
        attrs=dict(payload.get("attrs", {})),
    )


def _encode_site(site: records.Site | None) -> dict[str, Any] | None:
    if site is None:
        return None
    return {"path": site.path, "locator": site.locator, "line_hint": site.line_hint}


def _decode_site(
    payload: Mapping[str, Any] | None, workspace: Workspace | None
) -> records.Site | None:
    if not payload:
        return None
    path = str(payload.get("path", ""))
    locator = str(payload.get("locator", ""))
    hint = payload.get("line_hint")
    if workspace is not None:
        try:
            return workspace.site(path, locator, hint)
        except Exception:
            pass
    return records.Site(path=path, locator=locator, line_hint=hint)


# ---------------------------------------------------------------------------
# refusals and shells


def _refuse(
    entry: str,
    args: Sequence[tuple[str, str]],
    env: Sequence[tuple[str, str]],
    workspace: Workspace | None,
    strategy: str = "executed",
) -> records.LaunchWalk | None:
    """Reasons not to run a file at all, decided by shape and not by extension."""
    if not Path(entry).is_file():
        return _refused(
            entry,
            args,
            env,
            workspace,
            phase="import",
            exc_type="FileNotFoundError",
            message="the launch file does not exist",
        )
    text = _head(entry)
    if _is_ros1_xml(text):
        result = _empty(entry, args, env, workspace, strategy="refused")
        result.loadable = False
        result.notes.append(NOTE_ROS1)
        return result
    if strategy == "ast" and not entry.endswith(".py"):
        result = _empty(entry, args, env, workspace, strategy="refused")
        result.loadable = False
        result.notes.append("only a python launch file can be read as source")
        return result
    if entry.endswith(".py") and text.lstrip().startswith("<"):
        result = _empty(entry, args, env, workspace, strategy="refused")
        result.loadable = False
        result.notes.append(NOTE_MARKUP)
        return result
    return None


def _refused(
    entry: str,
    args: Sequence[tuple[str, str]],
    env: Sequence[tuple[str, str]],
    workspace: Workspace | None,
    *,
    phase: str,
    exc_type: str,
    message: str,
    sandboxed: bool = False,
) -> records.LaunchWalk:
    result = _empty(entry, args, env, workspace, strategy="refused")
    result.sandboxed = sandboxed
    result.loadable = False
    result.exceptions.append(
        records.WalkException(path=entry, phase=phase, exc_type=exc_type, message=message)
    )
    return result


def _empty(
    entry: str,
    args: Sequence[tuple[str, str]],
    env: Sequence[tuple[str, str]],
    workspace: Workspace | None,
    strategy: str,
) -> records.LaunchWalk:
    package = None
    if workspace is not None:
        try:
            package = workspace.package_of(entry)
        except Exception:
            package = None
    return records.LaunchWalk(
        entry_path=entry,
        entry_pkg=package,
        args=tuple((str(k), str(v)) for k, v in args),
        env=tuple((str(k), str(v)) for k, v in env),
        strategy=strategy,
    )


def _sourced(ros_setup: Sequence[str], script: str) -> str:
    parts = [f". {shlex.quote(str(path))}" for path in ros_setup]
    parts.append(script)
    return " && ".join(parts)


def _payload(stdout: str) -> dict[str, Any] | None:
    if MARKER not in stdout:
        return None
    tail = stdout.rsplit(MARKER, 1)[1]
    try:
        loaded = json.loads(tail)
    except json.JSONDecodeError:
        return None
    return loaded if isinstance(loaded, dict) else None


def _is_ros1_xml(text: str) -> bool:
    """ROS 1 XML by shape: a launch root with markers ROS 2 launch has no meaning for."""
    stripped = text.lstrip()
    if not stripped.startswith("<"):
        return False
    if "<launch" not in stripped:
        return False
    if any(marker in text for marker in ROS1_MARKERS):
        return True
    return "<node" in text and "type=" in text and "exec=" not in text


def _head(path: str, limit: int = 65536) -> str:
    try:
        with Path(path).open("r", encoding="utf-8", errors="replace") as handle:
            return handle.read(limit)
    except OSError:
        return ""


def _abs(path: str) -> str:
    target = Path(path)
    if not target.is_absolute():
        target = Path.cwd() / target
    return os.path.normpath(str(target))


def _node_order(node: records.ResolvedNode) -> tuple[str, str, str, str, str]:
    return (
        node.namespace,
        node.name or "",
        node.pkg,
        node.executable,
        node.plugin_class or "",
    )


def _site(workspace: Workspace | None, path: str, locator: str, line: int | None) -> records.Site:
    if workspace is not None:
        try:
            return workspace.site(path, locator, line)
        except Exception:
            pass
    return records.Site(path=path, locator=locator, line_hint=line)


# ---------------------------------------------------------------------------
# ast helpers


def _scope(tree: ast.AST) -> tuple[ast.AST, bool]:
    """The body of `generate_launch_description`, which is the whole visible description.

    A node built inside another function is reachable only if something calls that function,
    and no reader of the syntax can know whether anything does. Reporting it anyway is how a
    static walker claims a node set that never runs, so the search stops at the function
    boundary and the executed walk is left to find the rest.
    """
    for node in getattr(tree, "body", ()):
        if isinstance(node, ast.FunctionDef) and node.name == "generate_launch_description":
            return node, True
    return tree, False


def environment_reads(source: str) -> tuple[str, ...]:
    """The environment variables a launch file needs, in source order.

    `os.environ[...]` read, and `EnvironmentVariable` without `default_value`, which both
    raise when the variable is unset. `os.environ.get` and `os.getenv` answer None instead, so
    a file reading one that way decides for itself what an unset variable means. A walk stops
    at the first one that is unset, so this is how every other one the same file needs is
    named before a second extract.
    """
    try:
        tree = parse_python(source)
    except SyntaxError:
        return ()
    found: list[tuple[int, int, str]] = []
    for node in ast.walk(tree):
        name = None
        if (
            isinstance(node, ast.Subscript)
            and isinstance(node.ctx, ast.Load)
            and _environ(node.value)
        ):
            name = node.slice
        if (
            isinstance(node, ast.Call)
            and _call_name(node.func) == "EnvironmentVariable"
            and _arg(node, "default_value", 1) is None
        ):
            name = _arg(node, "name", 0)
        if isinstance(name, ast.Constant) and isinstance(name.value, str):
            found.append((node.lineno, node.col_offset, name.value))
    return tuple(dict.fromkeys(text for _line, _col, text in sorted(found)))


def required_arguments(source: str) -> tuple[str, ...]:
    """The launch arguments a python launch file declares with no default, in source order.

    `ros2 launch` refuses the file until each one is given, so a file that has them is either
    started with arguments or only ever included by another launch file that passes them.
    """
    try:
        tree = parse_python(source)
    except SyntaxError:
        return ()
    found: list[tuple[int, int, str]] = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or _call_name(node.func) != "DeclareLaunchArgument":
            continue
        name = _arg(node, "name", 0)
        if (
            _arg(node, "default_value", 1) is None
            and isinstance(name, ast.Constant)
            and isinstance(name.value, str)
        ):
            found.append((node.lineno, node.col_offset, name.value))
    return tuple(dict.fromkeys(text for _line, _col, text in sorted(found)))


def _environ(node: ast.AST) -> bool:
    """`os.environ`, or `environ` imported from os."""
    if isinstance(node, ast.Attribute):
        return node.attr == "environ"
    return isinstance(node, ast.Name) and node.id == "environ"


def _calls(tree: ast.AST, names: Sequence[str]):
    wanted = set(names)
    root, _found = _scope(tree)
    for node in _within(root):
        if isinstance(node, ast.Call) and _call_name(node.func) in wanted:
            yield node


def _within(root: ast.AST):
    """Every syntax node under root, without descending into a nested function or lambda."""
    yield root
    for child in ast.iter_child_nodes(root):
        if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)):
            continue
        yield from _within(child)


def _call_name(func: ast.expr) -> str:
    if isinstance(func, ast.Name):
        return func.id
    if isinstance(func, ast.Attribute):
        return func.attr
    return ""


def _arg(call: ast.Call, name: str, position: int | None) -> ast.expr | None:
    for keyword in call.keywords:
        if keyword.arg == name:
            return keyword.value
    if position is not None and len(call.args) > position:
        return call.args[position]
    return None


def _ast_expr(node: ast.expr | None) -> substitutions.Expr | None:
    """The text side expression for a syntax node, or None when nothing is visible."""
    if node is None:
        return None
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return substitutions.literal(node.value)
    if isinstance(node, ast.Call):
        name = _call_name(node.func)
        if name == "LaunchConfiguration":
            inner = _ast_expr(_arg(node, "variable_name", 0))
            rendered = substitutions.render(inner, {})
            return None if rendered is None else substitutions.config(rendered)
        if name in ("TextSubstitution",):
            return _ast_expr(_arg(node, "text", 0))
    if isinstance(node, ast.JoinedStr):
        parts = [_ast_expr(value) for value in node.values]
        if any(part is None for part in parts):
            return None
        return substitutions.join([part for part in parts if part is not None])
    if isinstance(node, (ast.List, ast.Tuple)):
        parts = [_ast_expr(value) for value in node.elts]
        if any(part is None for part in parts):
            return None
        return substitutions.join([part for part in parts if part is not None])
    return None


def _ast_text(node: ast.expr | None, values: Mapping[str, str]) -> str | None:
    return substitutions.render(_ast_expr(node), values)


def _ast_node(
    call: ast.Call,
    values: Mapping[str, str],
    entry: str,
    workspace: Workspace | None,
) -> records.ResolvedNode | None:
    package = _ast_text(_arg(call, "package", None), values)
    executable = _ast_text(_arg(call, "executable", None), values)
    if not package or not executable:
        return None
    name = _ast_text(_arg(call, "name", None), values)
    namespace_node = _arg(call, "namespace", None)
    namespace = _ast_text(namespace_node, values)
    if namespace_node is not None and namespace is None:
        return None
    absolute = "" if namespace_node is None else _absolute_namespace(namespace or "")
    remaps: list[records.Remap] = []
    for pair in _pairs(_arg(call, "remappings", None)):
        left = _ast_text(pair[0], values)
        right = _ast_text(pair[1], values)
        if left is None or right is None:
            continue
        remaps.append(
            records.Remap(
                frm=left,
                to=right,
                site=_site(workspace, entry, f"{_call_name(call.func)}.remappings", call.lineno),
            )
        )
    gates: list[records.Gate] = []
    conditions: list[str] = []
    condition = _arg(call, "condition", None)
    if condition is not None:
        mechanism = _call_name(condition.func) if isinstance(condition, ast.Call) else "IfCondition"
        if mechanism not in records.GATE_MECHANISMS:
            mechanism = "IfCondition"
        text = ast.unparse(condition)
        conditions.append(text)
        gates.append(records.Gate(mechanism=mechanism, expression=text, outcome="unknown"))
    kind = _call_name(call.func)
    disc = name or executable
    return records.ResolvedNode(
        pkg=package,
        executable=executable,
        namespace=absolute,
        form="lifecycle_node" if kind == "LifecycleNode" else "node",
        name=name,
        name_source="literal" if name else "executable_default",
        remaps=tuple(remaps),
        entity_path=(f"{kind}({disc})",),
        gates=tuple(gates),
        conditions=tuple(conditions),
        role_content=tuple(f"remap:{item.frm}={item.to}" for item in remaps),
        site=_site(workspace, entry, f"{kind}({disc})", call.lineno),
        attrs={"strategy": "ast"},
    )


def _pairs(node: ast.expr | None) -> list[tuple[ast.expr, ast.expr]]:
    if not isinstance(node, (ast.List, ast.Tuple)):
        return []
    out: list[tuple[ast.expr, ast.expr]] = []
    for item in node.elts:
        if isinstance(item, (ast.List, ast.Tuple)) and len(item.elts) == 2:
            out.append((item.elts[0], item.elts[1]))
    return out


def _include_text(call: ast.Call, values: Mapping[str, str]) -> str | None:
    source = _arg(call, "launch_description_source", 0)
    if isinstance(source, ast.Call):
        return _ast_text(_arg(source, "launch_file_path", 0), values)
    return _ast_text(source, values)


def _include_source(call: ast.Call) -> str:
    """The raw text of an include whose path did not fold.

    Three includes of one file that all collapsed are three blind spots, and an entry that
    carries no expression at all makes them one row keyed on the same digest.
    """
    source = _arg(call, "launch_description_source", 0)
    return ast.unparse(source) if source is not None else "IncludeLaunchDescription"


def _absolute_namespace(namespace: str) -> str:
    return ids.normalise_namespace(namespace)


__all__ = [
    "CHILD",
    "MARKER",
    "NOTE_MARKUP",
    "NOTE_ROS1",
    "NOTE_UNAVAILABLE",
    "LaunchImport",
    "available",
    "decode_walk",
    "encode_walk",
    "environment_reads",
    "import_report",
    "importable",
    "probe",
    "walk",
    "walk_ast",
]
