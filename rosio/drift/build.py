"""Build, launch, budget and extraction completeness rules.

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

from collections.abc import Callable, Mapping
from typing import Any

from rosio import ids, model
from rosio.collect import launch_declared
from rosio.collect import params as params_collector
from rosio.collect.bag import BAG_PKG
from rosio.drift import rules
from rosio.errors import ContractError
from rosio.model import Atlas, Finding, Source
from rosio.resolve import launch_exec

RULE_UNLOADABLE = "launch.unloadable"
RULE_RAISED = "launch.raised"

DEFAULT_RESOLUTION_BUDGET = 0.95
"""The overall resolution rate a run must keep. The reference stack measures 0.956 with
the coded layer honest about what a C++ source does not state, so the budget sits just
below it and a real drop fails the build before any other finding does."""

BUDGET_KEY = "resolution_budget"
"""Key under `atlas.x['drift']` a configuration may set, as a number for the overall rate
or as a mapping of want to number. The argument wins over the atlas when both are given."""


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
        "exc_type",
        "kept",
        "label",
        "message",
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
    findings: list[Finding] = []
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
                say=_raised_say(
                    first.label, first.exc_type, first.phase, first.message, first.kept
                ),
                discriminator=key[1],
                profiles=_profile_ids(rows),
                layers=[model.Layer.DECLARED.value],
                where=[_where(row) for row in rows],
            )
        )
    return findings


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
            f"{label} raised {exc_type} in the {phase} phase{where} while expanding {kept}, "
            f"so this atlas holds {kept} without the values that failed to expand."
        )
    return (
        f"{label} raised {exc_type} in the {phase} phase{where}, so the nodes it starts are "
        "missing from this atlas."
    )


def _where(row: _Row) -> Source:
    return Source(file=_file_for(row.subject), locator=row.phase, root=model.Root.SRC.value)


# ---------------------------------------------------------------------------
# launch.file_not_installed


RULE_NOT_INSTALLED = "launch.file_not_installed"


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
        package = atlas.entities.get(ids.build_pkg(parsed.package))
        python = package is not None and package.attrs.get("build_type") == "ament_python"
        top, _, rest = parsed.relpath.partition("/")
        if python:
            build_file = f"{parsed.package}/setup.py"
            line = (
                f"(os.path.join('share', package_name, '{top}'), glob('{top}/*'))"
                if rest
                else f"('share/' + package_name, ['{top}'])"
            )
            fix = f"add {line} to data_files in {build_file}, then build again"
        else:
            build_file = f"{parsed.package}/CMakeLists.txt"
            line = (
                f"install(DIRECTORY {top} DESTINATION share/${{PROJECT_NAME}})"
                if rest
                else f"install(FILES {top} DESTINATION share/${{PROJECT_NAME}})"
            )
            fix = f"add {line} to {build_file}, then build again"
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
# build.stale


@rules.register("build.stale")
def build_stale(atlas: Atlas) -> list[Finding]:
    """A file read through the package share whose two copies hold different bytes.

    One finding per file, never one per value read from it. The install tree here holds
    copies rather than symlinks and can be older than src, so without this rule the same
    build state arrives as a content difference on every parameter that file carries.

    What is measured is a byte difference at one package relative path. That is evidence
    and not proof of a stale build: a package that keeps two spellings of one interface and
    installs the second one leaves a different file under src at the same relative path,
    with nothing stale about it. The text says what was measured and leaves the cause to the
    reader.
    """
    findings: list[Finding] = []
    for node in atlas.by_kind(model.NodeKind.FILE.value):
        if not node.attrs.get("install_copy_differs"):
            continue
        parsed = ids.parse_file(node.id)
        relpath, package = parsed.relpath, parsed.package
        findings.append(
            Finding(
                rule="build.stale",
                severity=rules.severity_of("build.stale"),
                subject=node.id,
                title=f"the two trees hold different bytes at {relpath}",
                detail=(
                    f"a launch file resolved {relpath} through the package share of "
                    f"{package}, so the values come from the install tree, and the file at "
                    "the same package relative path under src holds different bytes. A stale "
                    "build is the usual cause. The other one is an installed copy generated "
                    "from a different source file, in which case src is not the file to edit."
                ),
                fix=(
                    f"rebuild {package} and compare again, and when the two still differ read "
                    f"the install rule for {relpath} to find the source it is built from"
                ),
                say=(
                    f"{package} {relpath} was read from the install tree and the src file at "
                    "that path holds different bytes, so rebuild the package and compare "
                    "again before trusting these values."
                ),
                layers=[model.Layer.DECLARED.value],
                where=[Source(file=node.id, root=model.Root.INSTALL.value)],
            )
        )
    return findings


# ---------------------------------------------------------------------------
# resolution.below_budget


@rules.register("resolution.below_budget")
def resolution_below_budget(
    atlas: Atlas, budget: float | Mapping[str, float] | None = None
) -> list[Finding]:
    """The CI gate surface: the extraction resolved less than the budget allows.

    Reported per profile, with the rate, the budget and the wants that fell, because the
    unit a user extracts and gates on is a profile. The overall rate is a property of the
    whole run and is stated as such, and each profile carries the count of blind spots its
    own entities own, so the profile that has to be repaired is named rather than guessed.
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
    findings: list[Finding] = []
    for node in atlas.by_kind(model.NodeKind.PROFILE.value):
        count = len(owners.get(node.id, ()))
        if not count:
            continue
        findings.append(
            Finding(
                rule="resolution.below_budget",
                severity=rules.severity_of("resolution.below_budget"),
                subject=node.id,
                title=f"the resolution rate {rate:.3f} is under the budget {overall:.3f}",
                detail=_budget_detail(rate, overall, count, fallen, rates, resolved, unresolved),
                fix=(
                    "resolve the sites the unresolved list names, or state the budget this "
                    "workspace can hold and record why the rest cannot be followed"
                ),
                say=(
                    f"the extraction resolved {rate:.3f} of what it read against a budget of "
                    f"{overall:.3f}, and {count} of the unresolved sites belong to this "
                    "profile, so the atlas is missing answers a rule needs."
                ),
                discriminator=_disc(f"{overall:.6f}", *fallen),
                profiles=[node.id],
                layers=sorted(atlas.counters.get("layers_present") or []),
            )
        )
    subject = _whole_run_subject(atlas)
    if findings or not subject:
        return findings
    # No profile owns a blind spot (or the workspace declares none), and a failed gate
    # still has to say so once.
    return [
        Finding(
            rule="resolution.below_budget",
            severity=rules.severity_of("resolution.below_budget"),
            subject=subject,
            title=f"the resolution rate {rate:.3f} is under the budget {overall:.3f}",
            detail=_budget_detail(
                rate, overall, len(atlas.unresolved), fallen, rates, resolved, unresolved
            ),
            fix=(
                "resolve the sites the unresolved list names, or state the budget this "
                "workspace can hold and record why the rest cannot be followed"
            ),
            say=(
                f"the extraction resolved {rate:.3f} of what it read against a budget of "
                f"{overall:.3f}, so the atlas is missing answers a rule needs."
            ),
            discriminator=_disc(f"{overall:.6f}", *fallen),
            layers=sorted(atlas.counters.get("layers_present") or []),
        )
    ]


def _whole_run_subject(atlas: Atlas) -> str:
    """A run when there is one, else the first owner of a blind spot."""
    runs = sorted(node.id for node in atlas.by_kind(model.NodeKind.RUN.value))
    owners = sorted(entry.owner for entry in atlas.unresolved if entry.owner in atlas.entities)
    return (runs or owners or [""])[0]


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
) -> str:
    parts = [
        f"the run resolved {rate:.6f} of the names, values and frames it read, against a "
        f"budget of {overall:.6f}, and {count} unresolved sites are owned by entities of "
        "this profile"
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
                fix=(
                    "repair the input the collector named and extract again, or take that "
                    "input out of the roots so the atlas stops claiming to cover it"
                ),
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
    for node in sorted(atlas.entities.values(), key=lambda item: item.id):
        because = str(node.attrs.get(REFUSED_KEY) or "")
        if not because or because == params_collector.NOT_INSTALLED:
            continue
        label = _refused_label(node)
        detail = str(node.attrs.get("refused_detail") or "")
        cost = REFUSAL_COSTS.get(because, UNKNOWN_COST)
        findings.append(
            Finding(
                rule=RULE_REFUSED,
                severity=rules.severity_of(RULE_REFUSED),
                subject=node.id,
                title=f"{label} was refused: {because}",
                detail=(
                    f"The collector that reads {label} recorded {because}"
                    + (f", {_clip(detail)}" if detail else "")
                    + f". This is one input inside a pass, so no pass row states it, and {cost} "
                    "never reached this atlas. Until this rule existed, the blind spot it "
                    "owns and the rate that fell with it were all a reader met."
                ),
                fix=f"make {label} readable and extract again, or take it out of the roots",
                say=f"{label} was refused with {because}, so this atlas does not hold {cost}.",
                layers=sorted(node.layers),
                profiles=sorted(node.profiles),
                where=_refused_where(atlas, node),
            )
        )
    return findings


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
    if cited:
        root = cited[0]
    elif ids.parse_file(node.id).package == BAG_PKG:
        root = model.Root.BAG.value
    else:
        root = model.Root.SRC.value
    return [Source(file=node.id, root=root)]


# ---------------------------------------------------------------------------
# registry


RULES: dict[str, Callable[[Atlas], list[Finding]]] = {
    "build.stale": build_stale,
    "extract.input_refused": extract_input_refused,
    "extract.pass_incomplete": extract_pass_incomplete,
    "launch.file_not_installed": launch_file_not_installed,
    "launch.raised": launch_raised,
    "launch.unloadable": launch_unloadable,
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
    "RULE_NOT_INSTALLED",
    "RULE_REFUSED",
    "UNKNOWN_COST",
    "build_stale",
    "collect",
    "extract_input_refused",
    "extract_pass_incomplete",
    "launch_file_not_installed",
    "launch_raised",
    "launch_unloadable",
    "resolution_below_budget",
]
