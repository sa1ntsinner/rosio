"""Build, launch, robot model mesh, budget and extraction completeness rules.

Every function here is a pure function of the joined atlas. Nothing reads a file, nothing
re extracts and nothing mutates an entity. Each rule binds to its committed id through
`rules.register` and takes its severity from the table in `rosio/drift/rules.py`, so the
gate is graded in one place. `RULES` is the same mapping for a caller that wants this
family alone. Both modules still need naming in `rules.FAMILIES` to load by themselves.

Three of these rules read what a collector recorded rather than what a layer claimed, and
that is deliberate. A launch file that refused to load and a file whose install copy no
longer matches src are facts about the extraction, and until M3 they were emitted from
inside the collector and from `extract`. The subject, the discriminator and the text are
reproduced here, so the finding id and therefore every waiver stay where they were. The
collector still writes `launch.unloadable` itself and the note into the run record, so the
detail this rule builds is the detail the collector built. `launch.raised` has one producer,
the rule below, reading the run rows.

Why a file was not loaded is read from the row and not guessed from its message. A ROS 2
XML launch file, a ROS 1 launch file and an environment that cannot import launch at all
are three different repairs, and the three sentences come from `collect/launch_declared`
so that the collector and this rule can never word one case two ways.
"""

from __future__ import annotations

import re
from collections.abc import Callable, Mapping, Sequence
from typing import Any

from rosio import ids, model
from rosio.collect import cpp_code, launch_declared
from rosio.collect import params as params_collector
from rosio.collect.bag import BAG_PKG
from rosio.drift import rules
from rosio.errors import ContractError
from rosio.model import Atlas, Finding, Source
from rosio.render import plural
from rosio.resolve import launch_exec

RULE_UNLOADABLE = "launch.unloadable"
RULE_RAISED = "launch.raised"

DEFAULT_RESOLUTION_BUDGET = 0.95
"""The overall resolution rate a run must keep. The reference stack measures 0.956 with
the coded layer honest about what a C++ source does not state, so the budget sits just
below it and a real drop fails the build before any other finding does."""

BUDGET_KEY = "resolution_budget"
"""Key under `atlas.x['drift']`, written there by extract from the `resolution_budget` key of
rosio.yaml, as a number for the overall rate or as a mapping of want to number. The argument
wins over the atlas when both are given."""

BUDGET_FIX = (
    "run rosio unresolved and resolve the sites it names, or set resolution_budget in "
    "rosio.yaml to the rate this workspace can hold and record why the rest cannot be followed"
)


# ---------------------------------------------------------------------------
# shared reading


def _disc(*parts: object) -> str:
    """A digest discriminator, prefixed so it can never read as an ordinal."""
    return "h" + ids.digest(*parts)


def _launchfile_index(atlas: Atlas) -> dict[str, str]:
    """`<pkg>/<relpath>` to launchfile id, which is the form an exception row carries."""
    out: dict[str, str] = {}
    for node in atlas.by_kind(model.NodeKind.LAUNCHFILE.value):
        parsed = ids.parse_launchfile(node.id)
        if parsed.package == ids.REPO_PKG:
            out[parsed.relpath] = node.id
        else:
            out[f"{parsed.package}/{parsed.relpath}"] = node.id
    return out


def _file_for(launchfile_id: str) -> str:
    parsed = ids.parse_launchfile(launchfile_id)
    return ids.build_file(parsed.package, parsed.relpath)


class _Row:
    """One recorded refusal, from the exceptions list of one run."""

    __slots__ = (
        "because",
        "env",
        "exc_type",
        "kept",
        "label",
        "message",
        "missing",
        "phase",
        "profile",
        "subject",
    )

    def __init__(self, subject: str, label: str, profile: str, row: Mapping[str, Any]) -> None:
        self.subject = subject
        self.label = label
        self.profile = profile
        self.phase = str(row.get("phase", ""))
        self.exc_type = str(row.get("exc_type", ""))
        self.message = str(row.get("message", ""))
        self.because = str(row.get("refused_because", ""))
        self.kept = str(row.get("kept", ""))
        self.env = str(row.get("env", ""))
        self.missing = str(row.get("missing", ""))

    def key(self, rule: str) -> tuple[str, str]:
        if rule == RULE_UNLOADABLE:
            return (self.subject, "")
        return (
            self.subject,
            _raised_disc(self.phase, self.exc_type, self.message, self.kept),
        )


def _refusals(atlas: Atlas, rule: str) -> dict[tuple[str, str], list[_Row]]:
    index = _launchfile_index(atlas)
    grouped: dict[tuple[str, str], list[_Row]] = {}
    for run in atlas.by_kind(model.NodeKind.RUN.value):
        profile = str(run.attrs.get("profile") or "")
        for row in run.attrs.get("exceptions") or []:
            if str(row.get("note", "")) != rule:
                continue
            subject = index.get(str(row.get("path", "")))
            if subject is None:
                continue
            entry = _Row(subject, str(row.get("path", "")), profile, row)
            grouped.setdefault(entry.key(rule), []).append(entry)
    return grouped


def _profile_ids(rows: list[_Row]) -> list[str]:
    return sorted({_profile_id(row.profile) for row in rows if row.profile})


def _profile_id(value: str) -> str:
    """A run records its profile as an id, older ones as a slug. Both arrive here."""
    prefix = f"{model.NodeKind.PROFILE.value}{ids.KIND_SEP}"
    return value if value.startswith(prefix) else ids.build_profile(value)


# ---------------------------------------------------------------------------
# launch.unloadable


@rules.register("launch.unloadable")
def launch_unloadable(atlas: Atlas) -> list[Finding]:
    """A file a profile names as a launch file that is not one.

    Two ROS 1 XML launch files on the reference stack are refused by shape before anything
    executes. The file is recorded with `loadable` false and contributes no node, which is
    exactly the case a profile that still names it needs told about.
    """
    findings = _missing_entrypoints(atlas)
    grouped = _refusals(atlas, RULE_UNLOADABLE)
    for key in sorted(grouped):
        rows = sorted(grouped[key], key=lambda item: (item.profile, item.phase))
        first = rows[0]
        findings.append(
            Finding(
                rule=RULE_UNLOADABLE,
                severity=rules.severity_of(RULE_UNLOADABLE),
                subject=first.subject,
                title=launch_declared.unloadable_title(first.label, first.because),
                detail=(
                    f"{first.message}. The file was recorded and never parsed, so it "
                    "contributes no node to any profile."
                ),
                fix=launch_declared.unloadable_fix(first.because),
                say=launch_declared.unloadable_say(first.label, first.because),
                discriminator=key[1],
                profiles=_profile_ids(rows),
                layers=[model.Layer.DECLARED.value],
                where=[_where(row) for row in rows],
            )
        )
    return findings


def _missing_entrypoints(atlas: Atlas) -> list[Finding]:
    """A profile entry point naming a launch file no package in the roots holds.

    There is no launch file entity to hang it on, so the profile is the subject, and one
    profile naming two missing files is two findings. A typo in rosio.yaml is the likeliest
    mistake a configuration holds, and it used to pass the gate with a perfect rate. One
    file named twice by one profile, a launch file started once per robot, is one finding.
    """
    out: list[Finding] = []
    seen: set[tuple[str, str]] = set()
    for run in atlas.by_kind(model.NodeKind.RUN.value):
        profile = str(run.attrs.get("profile") or "")
        for row in run.attrs.get("exceptions") or []:
            if str(row.get("refused_because", "")) != launch_declared.REFUSED_MISSING:
                continue
            if not profile:
                continue
            subject = _profile_id(profile)
            if subject not in atlas.entities:
                continue
            label = str(row.get("path", ""))
            if (subject, label) in seen:
                continue
            seen.add((subject, label))
            because = launch_declared.REFUSED_MISSING
            out.append(
                Finding(
                    rule=RULE_UNLOADABLE,
                    severity=rules.severity_of(RULE_UNLOADABLE),
                    subject=subject,
                    title=launch_declared.unloadable_title(label, because),
                    detail=(
                        f"Profile {rules.slug_of(subject)} names {label} as an entry point and "
                        "no package in the roots holds a launch file by that name, so nothing "
                        "was read for it."
                    ),
                    fix=launch_declared.unloadable_fix(because),
                    say=launch_declared.unloadable_say(label, because),
                    discriminator=_disc(label),
                    profiles=[subject],
                    layers=[model.Layer.DECLARED.value],
                )
            )
    return out


# ---------------------------------------------------------------------------
# launch.raised


@rules.register("launch.raised")
def launch_raised(atlas: Atlas) -> list[Finding]:
    """A launch file that raised while building its description.

    The node set of that entry point is a lower bound after this, unless the row names the
    node the walk `kept` without the part that failed. Every rule downstream
    reads a graph with a hole in it and no count shows the hole, which is why the
    vocabulary grades this one at the top of the gate.
    """
    findings: list[Finding] = []
    grouped = _refusals(atlas, RULE_RAISED)
    for key in sorted(grouped):
        rows = sorted(grouped[key], key=lambda item: (item.profile, item.phase))
        first = rows[0]
        missing = atlas.entities.get(first.missing)
        if missing is not None and missing.attrs.get(REFUSED_KEY) == params_collector.BUILT_ONLY:
            # A share file only the install rules put in place: build.unbuilt names it.
            continue
        sandboxed = launch_exec.SANDBOX_REFUSED in first.message
        fix = (
            "extract again with --launch-sandbox off: the sandbox refused a file access, and "
            "only a walk without it tells a missing file from a hidden one"
            if sandboxed
            else (
                "supply the argument or the environment the file needs, or make the "
                "file build its description without raising"
            )
        )
        say = _raised_say(first.label, first.exc_type, first.phase, first.message, first.kept)
        slugs = ", ".join(ids.parse_profile(item) for item in _profile_ids(rows))
        package = launch_declared.missing_package(first.message)
        module = (
            launch_declared.missing_module(first.message)
            if first.exc_type == "ModuleNotFoundError"
            else None
        )
        argument = _REQUIRED_ARGUMENT.search(first.message)
        tag = _UNKNOWN_TAG.search(first.message)
        if tag and not sandboxed:
            fix = (
                f"install the package whose launch frontend extension defines <{tag.group(1)}> "
                "where the walk runs, or write the tag as a node the stock frontend knows"
            )
            say = (
                f"{first.label} uses the launch tag <{tag.group(1)}>, which no installed launch "
                "frontend extension defines, so the nodes it starts are missing from this "
                f"atlas: {fix}."
            )
        elif argument and slugs:
            name = argument.group(1)
            fix = (
                f"state {name} under args of the entry point of profile {slugs} in rosio.yaml "
                f"(args: {{{name}: <value>}}), or delete that profile when another launch file "
                f"includes {first.label} and passes it"
            )
            say = (
                f"{first.label} needs the launch argument {name}, which profile {slugs} does "
                f"not state, so the nodes it starts are missing from this atlas: {fix}."
            )
        elif first.env and not first.kept and slugs:
            names = _and(first.env.split())
            variables = "variables" if " " in first.env else "variable"
            fix = (
                f"set {names} under env of profile {slugs} in rosio.yaml, to the value "
                "the robot is started with"
            )
            say = (
                f"{first.label} reads the environment {variables} {names}, which profile "
                f"{slugs} does not set, so the nodes it starts are missing from this atlas: "
                f"set {names} under env of that profile in rosio.yaml."
            )
        elif package and not sandboxed:
            fix = _package_fix(*package)
            if not first.kept:
                say = (
                    f"{first.label} needs the package {package[0]}, which no root holds and "
                    "no sourced setup file provides, so the nodes it starts are missing from "
                    f"this atlas: {fix}."
                )
        elif module and not sandboxed:
            fix = _module_fix(*module)
            say = (
                f"{first.label} imports the python module {module[0]}, which this machine "
                f"does not have, so the nodes it starts are missing from this atlas: {fix}."
            )
        findings.append(
            Finding(
                rule=RULE_RAISED,
                severity=rules.severity_of(RULE_RAISED),
                subject=first.subject,
                title=(
                    f"{first.label} raised {first.exc_type} while building its launch description"
                ),
                detail=_raised_detail(first.phase, first.message, first.kept),
                fix=fix,
                say=say,
                discriminator=key[1],
                profiles=_profile_ids(rows),
                layers=[model.Layer.DECLARED.value],
                where=[_where(row) for row in rows],
            )
        )
    return findings


_REQUIRED_ARGUMENT = re.compile(r"required launch argument '([^']+)' was not provided")
_UNKNOWN_TAG = re.compile(r"Unrecognized entity of the type: ([\w:.-]+)")


def _package_fix(package: str, distro: str) -> str:
    """Install the package, by its apt name when the release is known, or read its tree."""
    apt = f"ros-{distro or '$ROS_DISTRO'}-{package.replace('_', '-')}"
    return (
        f"install {package} (sudo apt install {apt} when it is a released package), "
        "or add the tree that holds it as a root in rosio.yaml"
    )


def _module_fix(module: str, distro: str) -> str:
    """The apt package when a ROS package of that name ships the module, else pip."""
    from rosio.migrate import distros

    if distros.is_ros_package(module):
        return _package_fix(module, distro)
    return (
        f"install the python module {module} for the interpreter that walks launch files, "
        "or add the tree that holds it as a root in rosio.yaml"
    )


def _and(names: list[str]) -> str:
    return names[0] if len(names) == 1 else f"{', '.join(names[:-1])} and {names[-1]}"


def _raised_disc(phase: str, exc_type: str, message: str, kept: str = "") -> str:
    """A raise that dropped nothing keeps the discriminator it always had."""
    return _disc(phase, exc_type, message, kept) if kept else _disc(phase, exc_type, message)


def _raised_detail(phase: str, message: str, kept: str = "") -> str:
    if kept:
        return (
            f"{kept} raised in the {phase} phase: {message.rstrip('.')}. The walk recorded "
            f"{kept} without the part that failed to expand and carried on."
        )
    return (
        f"The walk stopped in the {phase} phase: {message.rstrip('.')}. "
        "The node set of this entry point is incomplete."
    )


def _raised_say(label: str, exc_type: str, phase: str, message: str, kept: str = "") -> str:
    where = " inside the launch sandbox" if launch_exec.SANDBOX_REFUSED in message else ""
    if kept:
        return (
            f"{label} raised {exc_type} in the {phase} phase{where} while expanding {kept} "
            f"({_clip(message).rstrip('.')}), so this atlas holds {kept} without the values that "
            "failed to expand."
        )
    return (
        f"{label} raised {exc_type} in the {phase} phase{where}, so the nodes it starts are "
        "missing from this atlas."
    )


def _where(row: _Row) -> Source:
    # The phase is in the detail. As a locator it printed as a word after the path.
    return Source(file=_file_for(row.subject), root=model.Root.SRC.value)


# ---------------------------------------------------------------------------
# launch.file_not_installed


RULE_NOT_INSTALLED = "launch.file_not_installed"


def _install_fix(atlas: Atlas, package: str, relpath: str) -> str:
    """The install line a package's build file lacks for one package relative path."""
    pkg = atlas.entities.get(ids.build_pkg(package))
    top, _, rest = relpath.partition("/")
    if pkg is not None and pkg.attrs.get("build_type") == "ament_python":
        line = (
            f"(os.path.join('share', package_name, '{top}'), glob('{top}/*'))"
            if rest
            else f"('share/' + package_name, ['{top}'])"
        )
        return f"add {line} to data_files in {package}/setup.py, then build again"
    line = (
        f"install(DIRECTORY {top} DESTINATION share/${{PROJECT_NAME}})"
        if rest
        else f"install(FILES {top} DESTINATION share/${{PROJECT_NAME}})"
    )
    return f"add {line} to {package}/CMakeLists.txt, then build again"


@rules.register(RULE_NOT_INSTALLED)
def launch_file_not_installed(atlas: Atlas) -> list[Finding]:
    """A file a launch file reads from the package share that the build never installed.

    The package source holds the file at the same package relative path, so the launch file
    is right and the build file is missing one install line. At run time the launch fails
    far from the cause, or a node runs on its defaults.
    """
    findings: list[Finding] = []
    for node in sorted(atlas.entities.values(), key=lambda item: item.id):
        if node.attrs.get(REFUSED_KEY) != params_collector.NOT_INSTALLED:
            continue
        parsed = ids.parse_file(node.id)
        label = rules.path_of(node.id)
        fix = _install_fix(atlas, parsed.package, parsed.relpath)
        readers = sorted(
            {
                row
                for entry in atlas.unresolved
                if entry.owner == node.id
                for row in entry.evidence
                if row.file != node.id
            },
            key=lambda row: (row.file, row.locator, row.line_hint or 0, row.root),
        )
        reader = rules.path_of(readers[0].file) if readers else "the launch file"
        roles = {entry.role for entry in atlas.unresolved if entry.owner == node.id}
        role = next(
            (
                text
                for key, text in (
                    ("robot_model", "a robot model"),
                    ("rviz_configuration", "an rviz configuration"),
                    ("parameters_file", "a parameters file"),
                )
                if key in roles
            ),
            "a node argument",
        )
        findings.append(
            Finding(
                rule=RULE_NOT_INSTALLED,
                severity=rules.severity_of(RULE_NOT_INSTALLED),
                subject=node.id,
                title=f"{label} is in the source but not installed",
                detail=(
                    f"{reader} reads {label} as {role} from the share directory of "
                    f"{parsed.package}. "
                    f"The package source holds the file and the install tree does not, so the "
                    f"build file never installs it."
                ),
                fix=fix,
                say=f"{label} is never installed, so {reader} reads {role} that is missing; {fix}.",
                layers=[model.Layer.DECLARED.value],
                profiles=sorted(node.profiles),
                where=[Source(file=node.id, root=model.Root.SRC.value), *readers],
            )
        )
    return findings


# ---------------------------------------------------------------------------
# model.mesh_missing


RULE_MESH_MISSING = "model.mesh_missing"

_PACKAGE_URI = "package://"


def _loaded_models(atlas: Atlas) -> dict[str, dict[str, list[Source]]]:
    """Model file id to profile to the launch sites that read it as the robot model."""
    out: dict[str, dict[str, list[Source]]] = {}
    for site in atlas.by_kind(model.NodeKind.LAUNCHENT.value):
        for fact in site.facts:
            if fact.layer != model.Layer.DECLARED.value or not rules.usable(fact):
                continue
            if not rules.site_live(site, fact.profile):
                continue
            for read in fact.values.get("file_reads") or ():
                if read.get("role") == "robot_model":
                    rows = out.setdefault(read["file"], {}).setdefault(fact.profile, [])
                    rows.extend(fact.evidence[:1])
    return out


def _mesh_links(atlas: Atlas, model_id: str) -> dict[str, list[str]]:
    """Mesh name as written to the links of one model whose geometry names it."""
    out: dict[str, set[str]] = {}
    for frame in atlas.by_kind(model.NodeKind.FRAME.value):
        shapes = (frame.attrs.get("urdf_geometry") or {}).get(model_id) or {}
        for rows in shapes.values():
            for row in rows:
                name = (row.get("geometry") or {}).get("filename")
                if name:
                    out.setdefault(name, set()).add(ids.parse_frame(frame.id))
    return {key: sorted(value) for key, value in out.items()}


@rules.register(RULE_MESH_MISSING)
def model_mesh_missing(atlas: Atlas) -> list[Finding]:
    """A robot model some profile loads names a package mesh that is not there.

    rviz and gazebo draw nothing for the links that use it and say so only in a log line.
    When the package source holds the mesh the build file is missing one install line and
    the finding is an error, read the way `launch.file_not_installed` reads it. When no tree
    holds it the path is wrong or the file was never committed, and rosio cannot tell which,
    so it is a warning. One finding per model and
    mesh, never per link. A mesh that is not a package name stays the blind spot the urdf
    collector left.
    """
    loaded = _loaded_models(atlas)
    findings: list[Finding] = []
    for model_id in sorted(loaded):
        node = atlas.entities.get(model_id)
        if node is None or node.kind != model.NodeKind.FILE.value:
            continue
        cases = [
            (name, uninstalled)
            for key, uninstalled in (("meshes_not_installed", True), ("meshes_absent", False))
            for name in node.attrs.get(key) or ()
            if name.startswith(_PACKAGE_URI)
        ]
        if not cases:
            continue
        label = rules.path_of(model_id)
        profiles = sorted(loaded[model_id])
        readers = sorted(
            {row for rows in loaded[model_id].values() for row in rows},
            key=lambda row: (row.file, row.locator, row.line_hint or 0, row.root),
        )
        named = ("profiles " if len(profiles) > 1 else "profile ") + ", ".join(
            rules.slug_of(item) for item in profiles
        )
        links = _mesh_links(atlas, model_id)
        elsewhere = node.attrs.get("meshes_elsewhere") or {}
        for name, uninstalled in sorted(cases):
            package, _, relpath = name[len(_PACKAGE_URI) :].partition("/")
            used_by = links.get(name) or []
            drawn = (
                f"links {', '.join(used_by)} draw"
                if len(used_by) > 1
                else f"link {used_by[0]} draws"
                if used_by
                else "the links that use it draw"
            )
            if uninstalled:
                fix = _install_fix(atlas, package, relpath)
                title = f"{label} names {name}, which the source holds and the build never installs"
                why = (
                    f"the package source of {package} holds {relpath} and its install tree "
                    "does not, so the build file never installs it"
                )
                say = (
                    f"{label} names the mesh {name}, which {package} never installs, so the "
                    f"{drawn} nothing in {named}; {fix}."
                )
                variant = ""
            else:
                moved = list(elsewhere.get(name) or ())
                title = f"{label} names {name}, which no tree holds"
                why = f"neither the source nor the install tree of {package} holds {relpath}"
                if moved:
                    held = " or ".join(moved)
                    fix = f"change the mesh filename {name} in {label} to {held}"
                    why += f", while the package holds a file of that name at {held}"
                    say = (
                        f"{label} names the mesh {name}, which no tree holds, so the {drawn} "
                        f"nothing in {named}; {package} holds the file at {held}, so change "
                        f"the filename in {label} to that."
                    )
                else:
                    fix = (
                        f"correct the mesh filename {name} in {label} to a file {package} "
                        f"holds, or commit the mesh at {package}/{relpath}"
                    )
                    say = (
                        f"{label} names the mesh {name}, which no tree holds, so the {drawn} "
                        f"nothing in {named}; correct the filename in {label} or commit "
                        "the mesh."
                    )
                variant = "absent"
            findings.append(
                Finding(
                    rule=RULE_MESH_MISSING,
                    severity=rules.severity_of(RULE_MESH_MISSING, variant),
                    subject=model_id,
                    title=title,
                    detail=(
                        f"{named} loads {label} as the robot model, and {why}. rviz and "
                        f"gazebo then show nothing for the {drawn.rsplit(' ', 1)[0]}."
                    ),
                    fix=fix,
                    say=say,
                    discriminator=_disc(name),
                    layers=[model.Layer.DECLARED.value],
                    profiles=profiles,
                    where=[Source(file=model_id, root=model.Root.SRC.value), *readers],
                )
            )
    return findings


# ---------------------------------------------------------------------------
# build.stale


@rules.register("build.stale")
def build_stale(atlas: Atlas) -> list[Finding]:
    """A file whose src and install copies hold different bytes.

    One finding per file, never one per value read from it. Launch files and the files they
    reach through the package share are read from the install tree, which is what
    `ros2 launch` runs, so the atlas states what runs and src holds the next build. A python
    module is read from src, so the atlas states the next build and the installed copy is
    what runs. Either way the user meets the difference, and the text says which tree the
    atlas shows.

    What is measured is a byte difference at one package relative path. That is evidence
    and not proof of a stale build: a package that keeps two spellings of one interface and
    installs the second one leaves a different file under src at the same relative path,
    with nothing stale about it. The fix leaves that case to the reader.
    """
    findings: list[Finding] = []
    for node in atlas.by_kind(model.NodeKind.FILE.value):
        if not node.attrs.get("install_copy_differs"):
            continue
        parsed = ids.parse_file(node.id)
        relpath, package = parsed.relpath, parsed.package
        if node.attrs.get("via_package_share"):
            shown, root = "the installed copy, which is what runs now", model.Root.INSTALL.value
            say = (
                f"{package} {relpath} differs between src and install; rosio shows the "
                "installed copy, which is what runs, and the src edits run after colcon build."
            )
        else:
            shown, root = "the src copy, which runs only after a build", model.Root.SRC.value
            say = (
                f"{package} {relpath} differs between src and install; rosio shows the src "
                "copy, and the installed one is what runs until colcon build."
            )
        findings.append(
            Finding(
                rule="build.stale",
                severity=rules.severity_of("build.stale"),
                subject=node.id,
                title=f"src and install hold different bytes at {package}/{relpath}",
                detail=(
                    f"{relpath} of {package} under src holds different bytes from its installed "
                    f"copy, and the values in this atlas come from {shown}. A stale build is "
                    "the usual cause."
                ),
                fix=(
                    f"colcon build --packages-select {package} and extract again; when the two "
                    f"still differ, read the install rule for {relpath} to find the file it is "
                    "built from"
                ),
                say=say,
                layers=[model.Layer.DECLARED.value],
                where=[Source(file=node.id, root=root)],
            )
        )
    return findings


# ---------------------------------------------------------------------------
# build.unbuilt


RULE_UNBUILT = "build.unbuilt"


@rules.register(RULE_UNBUILT)
def build_unbuilt(atlas: Atlas) -> list[Finding]:
    """Packages with no install tree whose launch files and models read files only a build
    puts in place.

    A source workspace is read as it stands, and a launch file looks in the package share,
    which for an unbuilt package is its source tree. An install rule that copies
    `bringup/config` to `config` makes every such read miss, and one refusal per file buried
    the one action that repairs them all, so this is one finding for the run. It names each
    package, each file and where the source holds it, and its subject is the first package by
    name.
    """
    grouped: dict[str, list[model.Node]] = {}
    for node in sorted(atlas.entities.values(), key=lambda item: item.id):
        if node.attrs.get(REFUSED_KEY) == params_collector.BUILT_ONLY:
            grouped.setdefault(ids.parse_file(node.id).package, []).append(node)
    packages = [name for name in sorted(grouped) if ids.build_pkg(name) in atlas.entities]
    if not packages:
        return []
    nodes = [node for name in packages for node in grouped[name]]
    placed = "; ".join(
        f"{name}: " + ", ".join(map(_built_only, grouped[name])) for name in packages
    )
    files = plural(len(nodes), "file")
    exist = "exists" if len(nodes) == 1 else "exist"
    named = packages[0] if len(packages) == 1 else plural(len(packages), "package")
    are = "is" if len(packages) == 1 else "are"
    readers = sorted(
        {
            row
            for node in nodes
            for entry in atlas.unresolved
            if entry.owner == node.id
            for row in entry.evidence
            if row.file != node.id
        },
        key=lambda row: (row.file, row.locator, row.line_hint or 0, row.root),
    )
    fix = _build_fix(atlas, packages[0] if len(packages) == 1 else "")
    return [
        Finding(
            rule=RULE_UNBUILT,
            severity=rules.severity_of(RULE_UNBUILT),
            subject=ids.build_pkg(packages[0]),
            title=(
                f"{named} {are} not built, and {files} read from a package share "
                f"{exist} only after a build"
            ),
            detail=(
                "Launch files and robot models read these from package share directories no "
                "install tree holds, so the source tree stood in and the install rules were "
                f"never applied. {placed}. What they state is missing from this atlas."
            ),
            fix=fix,
            say=(
                f"{named} {are} not built, so {files} that only the install rules put in place "
                f"{'is' if len(nodes) == 1 else 'are'} missing from this atlas: {fix}."
            ),
            layers=[model.Layer.DECLARED.value],
            where=readers,
        )
    ]


def _predicate_profiles(atlas: Atlas, key: str) -> list[str]:
    """The profiles whose run recorded one predicate as true."""
    return sorted(
        _profile_id(str(run.attrs.get("profile")))
        for run in atlas.by_kind(model.NodeKind.RUN.value)
        if run.attrs.get("profile") and (run.attrs.get("predicates") or {}).get(key)
    )


def _build_fix(atlas: Atlas, package: str = "") -> str:
    """How the build a reader ran becomes what rosio reads."""
    if _predicate_profiles(atlas, launch_declared.INSTALL_UNNAMED):
        return (
            "an install tree already sits beside src and rosio.yaml names no install root: "
            "add it under roots (kind: install, path: install) or delete roots and ros_setup "
            "from rosio.yaml so the defaults read it, then extract again"
        )
    target = f" --packages-up-to {package}" if package else ""
    return (
        f"build the workspace (colcon build{target}), source its install tree and extract "
        "again: rosio reads the install tree beside src by default, and a rosio.yaml that "
        "lists roots has to list it too"
    )


RULE_UNDERLAY = "build.underlay"


@rules.register(RULE_UNDERLAY)
def build_underlay(atlas: Atlas) -> list[Finding]:
    """A package the roots hold that the launch walk read from another tree.

    A share lookup goes to the ament index, which answers from the sourced trees. For a
    package only a src root holds, that is the released copy before the source tree is
    asked, which is what `ros2 launch` runs before a build, so the walk keeps reading it. For
    a package an install root holds whose setup file nobody sourced, it is the release again.
    Either way every value of that package comes from a copy nobody in this workspace edits,
    and an edit that changes nothing reads as a tool that is wrong, so one finding per
    package says so. A share in the install tree beside src that no root names is the build of
    the src package, sourced but not read by rosio, and the finding says so rather than
    calling the package unbuilt.
    """
    findings: list[Finding] = []
    for node in atlas.by_kind(model.NodeKind.PKG.value):
        tree = str(node.attrs.get("share_read_from") or "")
        if not tree:
            continue
        package = ids.parse(node.id).segments[0]
        if node.attrs.get("install_not_sourced"):
            held = "an install root holds it and its setup file is not sourced"
            fix = (
                f"source the setup.bash of the install tree that holds {package}: list it "
                "under ros_setup in rosio.yaml, or delete ros_setup so the default sources "
                "it, then extract again"
            )
            say = (
                f"{package} is built but its install tree is not sourced, so its launch files "
                f"read the copy in {tree}: source that install tree's setup.bash."
            )
        elif node.attrs.get("share_build"):
            held = (
                "the install tree beside the source root holds a build of it and no root names "
                "that tree, so rosio reads src while the launch walk reads the build"
            )
            fix = _build_fix(atlas, package)
            took = (
                "a file the build links back to src takes effect as soon as it is saved, and "
                "one it copied only after colcon build."
                if node.attrs["share_build"] == "symlinked"
                else "edits in src take effect only after colcon build."
            )
            say = f"{package} is read from {tree}, the build of its src package: {took}"
        else:
            held = "only a source root holds it, which is what ros2 launch reads before a build"
            fix = _build_fix(atlas, package)
            say = (
                f"{package} in src is not built, so its launch files read the copy in {tree}: "
                "edits in src take effect only after colcon build and sourcing the overlay."
            )
        findings.append(
            Finding(
                rule=RULE_UNDERLAY,
                severity=rules.severity_of(RULE_UNDERLAY),
                subject=node.id,
                title=f"{package} is read from {tree}, not from the workspace",
                detail=(
                    f"The launch files looked up the share directory of {package} and the "
                    f"ament index answered with the copy in {tree}, because {held}. The files "
                    f"and values of {package} in this atlas come from that copy."
                ),
                fix=fix,
                say=say,
                profiles=_predicate_profiles(
                    atlas, f"{launch_declared.UNDERLAY_PREDICATE}{package}"
                ),
                layers=[model.Layer.DECLARED.value],
            )
        )
    return findings


def _built_only(node: model.Node) -> str:
    relpath = ids.parse_file(node.id).relpath
    held = " or ".join(node.attrs.get("found_elsewhere") or ["another directory"])
    needs = node.attrs.get("needs") or []
    if needs:
        included = ids.parse_file(needs[0]).relpath
        return f"{relpath} includes {included}, which the source holds at {held}"
    return f"{relpath} is in the source at {held}"


# ---------------------------------------------------------------------------
# resolution.below_budget


@rules.register("resolution.below_budget")
def resolution_below_budget(
    atlas: Atlas, budget: float | Mapping[str, float] | None = None
) -> list[Finding]:
    """The CI gate surface: the extraction resolved less than the budget allows.

    One finding for the run, because the rate is a property of the whole run: one per
    profile repeated the same rate fifty times over and blamed a profile holding one blind
    spot as loudly as the one holding forty. The profiles that hold the most blind spots are
    named, worst first, in the text. The subject is the same for the whole run whichever
    profile is worst, so repairing one profile never moves the id a waiver keys on.
    """
    overall, per_want = _budget(atlas, budget)
    rates = dict(atlas.counters.get("resolution_rate") or {})
    resolved = dict(atlas.counters.get("resolved") or {})
    unresolved = dict(atlas.counters.get("unresolved") or {})
    if not rates:
        return []
    total = sum(resolved.values()) + sum(unresolved.values())
    rate = round(sum(resolved.values()) / total, 6) if total else 1.0
    fallen = sorted(
        want
        for want, value in rates.items()
        if want in per_want and round(float(value), 6) < per_want[want]
    )
    if rate >= overall and not fallen:
        return []
    owners = _unresolved_owners(atlas)
    counts = sorted(
        (
            (len(owners[node.id]), node.id)
            for node in atlas.by_kind(model.NodeKind.PROFILE.value)
            if owners.get(node.id)
        ),
        key=lambda item: (-item[0], item[1]),
    )
    subject = _whole_run_subject(atlas)
    if not subject:
        return []
    worst = ", ".join(f"{ids.parse_profile(profile)} ({count})" for count, profile in counts[:3])
    if len(counts) > 3:
        worst += f" and {len(counts) - 3} more profiles"
    held = f"; most blind spots in {worst}" if counts else ""
    unreached = sum(
        1
        for entry in atlas.unresolved
        if not set(getattr(atlas.entities.get(entry.owner), "profiles", ())) - {model.ANY_PROFILE}
    )
    if unreached:
        held += (
            f"; {unreached} of {len(atlas.unresolved)} sit in code no profile starts, "
            "which no profile change lifts"
        )
    return [
        Finding(
            rule="resolution.below_budget",
            # Under the default budget nothing in the user's code can lift the rate, so a
            # first run is not red under --fail-on warn. A budget in rosio.yaml makes it warn.
            severity=rules.severity_of(
                "resolution.below_budget", "" if _budget_stated(atlas, budget) else "unbudgeted"
            ),
            subject=subject,
            title=f"the resolution rate {rate:.3f} is under the budget {overall:.3f}{held}",
            detail=_budget_detail(
                rate, overall, len(atlas.unresolved), fallen, rates, resolved, unresolved, worst
            ),
            fix=BUDGET_FIX,
            say=(
                f"the extraction resolved {rate:.3f} of what it read against a budget of "
                f"{overall:.3f}{held}; rosio unresolved lists each one with the line that "
                "asked for it."
            ),
            discriminator=_disc(f"{overall:.6f}", *fallen),
            profiles=[profile for _count, profile in counts],
            layers=sorted(atlas.counters.get("layers_present") or []),
        )
    ]


def _whole_run_subject(atlas: Atlas) -> str:
    """The workspace, which extraction always writes as `pkg:@repo`, so adding a profile or a
    package moves no id. An atlas without it falls back to the first profile, else the first
    package, else the first run or owner of a blind spot.

    A run id carries the digest of its inputs and changes with every edit, so it is the last
    choice: a subject that moves when a blind spot is repaired un waives the finding.
    """
    workspace = ids.build_pkg(ids.REPO_PKG)
    if workspace in atlas.entities:
        return workspace
    for kind in (model.NodeKind.PROFILE.value, model.NodeKind.PKG.value):
        found = sorted(node.id for node in atlas.by_kind(kind))
        if found:
            return found[0]
    runs = sorted(node.id for node in atlas.by_kind(model.NodeKind.RUN.value))
    owners = sorted(entry.owner for entry in atlas.unresolved if entry.owner in atlas.entities)
    return (runs or owners or [""])[0]


def _budget_stated(atlas: Atlas, budget: object) -> bool:
    return budget is not None or (atlas.x.get("drift") or {}).get(BUDGET_KEY) is not None


def _budget(
    atlas: Atlas, budget: float | Mapping[str, float] | None
) -> tuple[float, dict[str, float]]:
    stated = budget
    if stated is None:
        stated = (atlas.x.get("drift") or {}).get(BUDGET_KEY)
    if stated is None:
        return DEFAULT_RESOLUTION_BUDGET, {}
    if isinstance(stated, Mapping):
        per_want = {str(key): _fraction(value) for key, value in stated.items()}
        return per_want.pop("overall", DEFAULT_RESOLUTION_BUDGET), per_want
    return _fraction(stated), {}


def _fraction(value: object) -> float:
    try:
        number = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError) as exc:
        raise ContractError(f"a resolution budget must be a number, got {value!r}") from exc
    if not 0.0 <= number <= 1.0:
        raise ContractError(f"a resolution budget must be between 0 and 1, got {number}")
    return number


def _unresolved_owners(atlas: Atlas) -> dict[str, list[str]]:
    """Blind spot ids grouped by the profile of the entity that owns the site."""
    out: dict[str, list[str]] = {}
    for entry in sorted(atlas.unresolved, key=lambda item: item.id):
        owner = atlas.entities.get(entry.owner)
        if owner is None:
            continue
        for profile in owner.profiles:
            out.setdefault(profile, []).append(entry.id)
    return out


def _budget_detail(
    rate: float,
    overall: float,
    count: int,
    fallen: list[str],
    rates: Mapping[str, float],
    resolved: Mapping[str, int],
    unresolved: Mapping[str, int],
    held_by: str = "",
) -> str:
    parts = [
        f"the run resolved {rate:.6f} of the names, values and frames it read, against a "
        f"budget of {overall:.6f}, with {plural(count, 'unresolved site')}"
        + (f", most of them owned by profiles {held_by}" if held_by else "")
    ]
    worst = sorted(rates, key=lambda want: (rates[want], want))[:3]
    for want in worst:
        parts.append(
            f"{want} {float(rates[want]):.6f} over {int(resolved.get(want, 0))} resolved and "
            f"{int(unresolved.get(want, 0))} unresolved"
        )
    if fallen:
        parts.append("under its own budget: " + ", ".join(fallen))
    return ". ".join(parts) + "."


# ---------------------------------------------------------------------------
# extract.pass_incomplete


RULE_INCOMPLETE = "extract.pass_incomplete"

REASON_LIMIT = 160
"""How much of a recorded reason reaches the sentence. Long enough for the exception type
and what it named, short enough that the line stays quotable."""


@rules.register(RULE_INCOMPLETE)
def extract_pass_incomplete(atlas: Atlas) -> list[Finding]:
    """A collector pass that did not finish, which no rate can report.

    `counters.resolution_rate` is resolved over resolved plus unresolved per want, so a pass
    that died contributes neither: its want keeps a perfect rate over a smaller denominator
    or leaves the counter entirely, and the atlas reads as complete while data was dropped.
    This reads the pass record instead, and grades it at the top of the gate for the same
    reason `launch.raised` sits there.

    A pass whose inputs were absent is not in here. A collector with nothing to read is
    complete, and firing on it would make this rule noise on every small repository.
    """
    findings: list[Finding] = []
    for run_id, row in model.incomplete_passes(atlas):
        run = atlas.entities[run_id]
        layer = str(run.attrs.get("layer") or "")
        profile = str(run.attrs.get("profile") or "")
        collector = str(row.get("collector") or "")
        reason = _clip(str(row.get("reason") or "no reason recorded"))
        missing = str(row.get("missing") or f"whatever the {collector} collector reads")
        where = f"the {layer} layer" if layer else "its layer"
        scope = f" of {rules.slug_of(profile)}" if profile else ""
        fix = (
            "repair the input the collector named and extract again, or take that "
            "input out of the roots so the atlas stops claiming to cover it"
        )
        if cpp_code.INSTALL_HINT in str(row.get("reason") or ""):
            fix = (
                "install the C++ extra into the environment rosio runs from "
                f"({cpp_code.INSTALL_HINT}), then extract again"
            )
        findings.append(
            Finding(
                rule=RULE_INCOMPLETE,
                severity=rules.severity_of(RULE_INCOMPLETE),
                subject=run_id,
                title=f"the {collector} collector did not finish{scope}",
                detail=(
                    f"The pass over {where}{scope} stopped with {reason}, so {missing} never "
                    "reached this atlas. Every count below is a lower bound, and the "
                    "resolution rate cannot show it: a pass that stopped takes its own "
                    "denominator with it, so the rate stays where it was or better."
                ),
                fix=fix,
                say=(
                    f"the {collector} collector stopped in {where}{scope} with {reason}, so "
                    f"{missing} are missing from this atlas."
                ),
                discriminator=_disc(collector, layer, profile, reason),
                profiles=[profile] if profile else [],
                layers=[layer] if layer else [],
            )
        )
    return findings


def _clip(text: str) -> str:
    """One line of a recorded reason, with no synthetic marker left in it.

    A long reason is cut at its first sentence, and at the last word before the limit when
    the first sentence is longer than that. Cutting mid word leaves a line that reads as a
    truncated identifier, which is how a reader ends up repairing the wrong thing.
    """
    one = " ".join(text.split())
    if len(one) > REASON_LIMIT:
        first = one.split(". ")[0]
        head = (
            first if len(first) <= REASON_LIMIT - 3 else one[: REASON_LIMIT - 3].rsplit(" ", 1)[0]
        )
        one = head.rstrip(" ,;:") + "..."
    return one.replace(ids.KIND_SEP + ids.SYNTHETIC, ids.KIND_SEP + " " + ids.SYNTHETIC)


# ---------------------------------------------------------------------------
# extract.input_refused


RULE_REFUSED = "extract.input_refused"

REFUSED_KEY = "refused_because"
"""The one shape a refused entity takes, AGENTS section 4. A collector that states its
refusal any other way is invisible to this rule, which is why the shape is in the contract
and not left to each collector."""

REFUSAL_COSTS = {
    "bag_storage_absent": "the per topic counts and time spans this storage file would carry",
    "bag_storage_unreadable": "the per topic counts and time spans this storage file carries",
    "bag_unreadable": "every topic, type and count this recording holds",
    "generator_list_unreadable": "the interface list this build file names",
    "interface_unreadable": "the fields of the type this file defines",
    "file_missing": "whatever this file states",
    "params_file_unreadable": "every parameter value this file sets",
    "plugin_manifest_missing": "the plugin classes this manifest exports",
    "plugin_manifest_unreadable": "the plugin classes this manifest exports",
    "py_parse_error": "the endpoints, parameters and node names this source states",
    "urdf_unreadable": "the frames and the links this model declares",
    "xacro_argument_missing": "the frames and the links this model declares",
    "xacro_failed": "the frames and the links this model declares",
    "xacro_tool_missing": "the frames and the links this model declares",
    "xacro_unreadable": "the frames and the links this model declares",
}
"""What the atlas does not hold, per recorded reason. Read from the reason and never
guessed from the message, the same rule the launch refusals above follow."""

UNKNOWN_COST = "whatever this file states"


@rules.register(RULE_REFUSED)
def extract_input_refused(atlas: Atlas) -> list[Finding]:
    """One input a collector could not read, inside a pass that finished.

    `extract.pass_incomplete` is the wrong rule for this and always was: the pass did reach
    the end of its inputs, so it is `completed` and that rule is silent, while the frames,
    the parameters or the topics of the refused file are gone all the same. What a reader
    met until now was a blind spot and a fallen rate, neither of which names the file.

    It fires on the recorded refusal and never on a message, so a file that was read cannot
    reach it: only a collector that refused an input writes the attribute this reads.
    """
    findings: list[Finding] = []
    models = [
        row
        for site in atlas.by_kind(model.NodeKind.LAUNCHENT.value)
        for fact in site.facts
        for row in fact.values.get("file_reads") or []
        if row.get("role") == "robot_model"
    ]
    expanded = {str(row.get("file") or "") for row in models if row.get("xacro_args")}
    loaded = {str(row.get("file") or "") for row in models}
    for node in sorted(atlas.entities.values(), key=lambda item: item.id):
        because = str(node.attrs.get(REFUSED_KEY) or "")
        if not because or because in (params_collector.NOT_INSTALLED, params_collector.BUILT_ONLY):
            continue
        detail = str(node.attrs.get("refused_detail") or "")
        if because in XACRO_DEFAULTS and (
            node.id in expanded or (node.id not in loaded and _EMPTY_ARGUMENT.search(detail))
        ):
            # The profile runs the model with the arguments its launch line passes, and that
            # expansion is what the atlas states. The defaults alone are nobody's model, and
            # a template that builds a package name from an empty default and that no profile
            # loads is a model nothing in the workspace runs.
            continue
        label = _refused_label(node)
        cost = REFUSAL_COSTS.get(because, UNKNOWN_COST)
        where = _refused_where(atlas, node)
        readers = _refused_readers(atlas, node)
        line = where[0].line_hint if where else None
        said = detail.partition("is not readable as yaml: ")[2]
        title = f"{label} was refused: {because}"
        say = f"{label} was refused with {because}, so this atlas does not hold {cost}."
        fix = _refused_fix(node, because, label, detail, readers)
        if said:
            title = f"{label}:{line} is not valid yaml" if line else f"{label} is not valid yaml"
            say = f"{label} is not valid yaml, {said}, so this atlas does not hold {cost}."
            position = said.partition(": ")[0]
            fix = f"correct the yaml at {position} of {label}, then extract again"
        findings.append(
            Finding(
                rule=RULE_REFUSED,
                severity=rules.severity_of(RULE_REFUSED),
                subject=node.id,
                title=title,
                detail=(
                    f"The collector that reads {label} could not read it ({because})"
                    + (f": {_clip(detail)}" if detail else "")
                    + f". {cost[0].upper()}{cost[1:]} never reached this atlas."
                ),
                fix=fix,
                say=say,
                layers=sorted(node.layers),
                profiles=sorted(node.profiles) or _reading_profiles(atlas, node.id),
                where=[*where, *readers],
            )
        )
    return findings


XACRO_DEFAULTS = frozenset({"xacro_failed", "xacro_argument_missing"})
"""Refusals of a model expanded with its own defaults, which a launch line passing arguments
makes moot."""

_EMPTY_ARGUMENT = re.compile(r"'_\w*' is not a valid package name")
_MISSING_PACKAGE = re.compile(r"package '([^']+)' (?:is )?not found")


def _refused_readers(atlas: Atlas, node: model.Node) -> list[Source]:
    """The launch lines that named a refused file, from the blind spot the refusal left."""
    return sorted(
        {
            row
            for entry in atlas.unresolved
            if entry.owner == node.id
            for row in entry.evidence
            if row.file != node.id
        },
        key=Source.sort_key,
    )


def _refused_fix(
    node: model.Node,
    because: str,
    label: str,
    detail: str = "",
    readers: Sequence[Source] = (),
) -> str:
    """What repairs one refusal, in the words its own cause deserves."""
    if because == "plugin_manifest_missing" and node.kind == model.NodeKind.FILE.value:
        package = ids.parse_file(node.id).package
        return (
            f"{package}/package.xml exports {label}, which the package does not install: "
            f"correct the export or the install rule of {package}, or report it upstream "
            "when it is a released package, which nothing in this workspace can change"
        )
    if because == params_collector.MISSING_FILE:
        at = f" at {rules.where(readers[0])}" if readers else ""
        return f"correct the path{at} that names {label} or add the file, then extract again"
    package = _MISSING_PACKAGE.search(detail)
    if because == "xacro_failed" and package:
        return (
            f"{label} includes from package {package.group(1)}, which no root and no sourced "
            "setup file holds: install it or put its source under a root, then extract again"
        )
    if because in XACRO_DEFAULTS:
        return (
            f"fix what xacro refused in {label}, or state the arguments it needs under the "
            "entry point that loads it, then extract again"
        )
    return f"make {label} readable and extract again, or take it out of the roots"


def _reading_profiles(atlas: Atlas, file_id: str) -> list[str]:
    """The profiles whose launch sites read a file. A refused file states no fact, so its
    own profiles are empty, and an empty list reads as every profile."""
    return sorted(
        {
            fact.profile
            for site in atlas.by_kind(model.NodeKind.LAUNCHENT.value)
            for fact in site.facts
            if fact.profile != model.ANY_PROFILE
            and any(row.get("file") == file_id for row in fact.values.get("file_reads") or [])
        }
    )


def _refused_label(node: model.Node) -> str:
    if node.kind == model.NodeKind.BAG.value:
        return ids.parse_bag(node.id)
    if node.kind == model.NodeKind.FILE.value:
        return rules.path_of(node.id)
    return node.label or node.id


def _refused_where(atlas: Atlas, node: model.Node) -> list[Source]:
    """The refused file, in the tree it was read from.

    A refusal carries no fact, so the root comes from the blind spot the refusal left
    (AGENTS 13 point 3), which every collector cites with its root. A file in an install
    tree cited as `src` sent an editor looking for it in a checkout that does not hold it.
    """
    if node.kind != model.NodeKind.FILE.value:
        return []
    cited = sorted(
        {
            row.root
            for entry in atlas.unresolved
            if entry.owner == node.id
            for row in entry.evidence
            if row.file == node.id
        }
    )
    lines = sorted(
        {
            row.line_hint
            for entry in atlas.unresolved
            if entry.owner == node.id
            for row in entry.evidence
            if row.file == node.id and row.line_hint
        }
    )
    if cited:
        root = cited[0]
    elif ids.parse_file(node.id).package == BAG_PKG:
        root = model.Root.BAG.value
    else:
        root = model.Root.SRC.value
    return [Source(file=node.id, root=root, line_hint=lines[0] if lines else None)]


# ---------------------------------------------------------------------------
# registry


RULES: dict[str, Callable[[Atlas], list[Finding]]] = {
    "build.stale": build_stale,
    "build.unbuilt": build_unbuilt,
    "build.underlay": build_underlay,
    "extract.input_refused": extract_input_refused,
    "extract.pass_incomplete": extract_pass_incomplete,
    "launch.file_not_installed": launch_file_not_installed,
    "launch.raised": launch_raised,
    "launch.unloadable": launch_unloadable,
    "model.mesh_missing": model_mesh_missing,
    "resolution.below_budget": resolution_below_budget,
}

RULE_IDS = tuple(sorted(RULES))


def collect(atlas: Atlas) -> list[Finding]:
    """Every rule in this module, in rule id order, then in finding sort order."""
    findings: list[Finding] = []
    for rule in RULE_IDS:
        findings.extend(RULES[rule](atlas))
    return sorted(findings, key=Finding.sort_key)


__all__ = [
    "BUDGET_KEY",
    "DEFAULT_RESOLUTION_BUDGET",
    "REASON_LIMIT",
    "REFUSAL_COSTS",
    "REFUSED_KEY",
    "RULES",
    "RULE_IDS",
    "RULE_INCOMPLETE",
    "RULE_MESH_MISSING",
    "RULE_NOT_INSTALLED",
    "RULE_REFUSED",
    "RULE_UNBUILT",
    "UNKNOWN_COST",
    "build_stale",
    "build_unbuilt",
    "build_underlay",
    "collect",
    "extract_input_refused",
    "extract_pass_incomplete",
    "launch_file_not_installed",
    "launch_raised",
    "launch_unloadable",
    "model_mesh_missing",
    "resolution_below_budget",
]
