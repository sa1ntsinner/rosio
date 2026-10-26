"""The atlas data model: entities, edges, per (layer, profile) facts, findings.

Three shape rules decide everything else.

1. One entity, one id, N views. A layer never creates a parallel entity. It adds a
   LayerFact keyed by (layer, profile). Disagreement is data, never a lost merge, and
   it is never resolved by overwriting.

2. Every slot is keyed by (layer, profile), not by layer. Profiles are plural on this
   kind of stack: three mutually exclusive ExecuteProcess variants can all resolve to
   one node name with different argv, and one launch file can put different parameters
   on the same nodes under different arguments. A single declared slot would drop two
   truths out of three while claiming all three.

3. Answers are precomputed on the entity. `effective`, `conflicts`, `agreement`,
   `state` and `candidates` live inline so that reading one entity answers the question
   with no traversal, which is what a small model with no client library needs.

Determinism: one encoder, sorted keys, sorted collections, no clock inside compared
data. `canonical_bytes()` drops the generation block, every `volatile` sub object and
every `line_hint`, so two runs over unchanged inputs are byte identical.
"""

from __future__ import annotations

import json
import math
import re
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from enum import Enum
from typing import Any

from rosio import ATLAS_SCHEMA_VERSION, __version__, ids
from rosio.errors import ContractError

SCHEMA_MINOR = "15"
"""What has been added since `<major>.0`, all of it additive and none of it moving a
stored id: the `model.mesh_missing` rule and the model file attributes
`meshes_not_installed` and `meshes_absent`; frame `urdf_geometry`, the visual and collision
rows each model states;
endpoint and tf broadcast record `guard_parameters`;
the launch exception row `kept`; the unresolved entry `construction`;
`class_templates` definition counts and specialization presence; construction
`definition_file`;
launch entity `file_reads`;
the `launch.file_not_installed` rule and its
`file_not_installed` refusal reason; the `param.sim_time_mixed` rule;
observed topic `volatile.stamps` checks and
run `volatile.stamp_verdict`; the `qos.incompatible` rule; observed `rmw`,
`zenoh_environment` presence and run `zenoh_router` facts, optional run `rmw_default`, and
`rmw.mixed` and `zenoh.router_unreachable` rules; recorded header stamp checks, stamp
verdicts and seven `time.*` rules;
the `passes` list on a run record and the `complete` object under
`counters.passes`, the rule ids `extract.pass_incomplete` and `extract.input_refused`, and
the `refused_because` and `refused_detail` attributes a refused entity carries, whose
value `manifest_unreadable` was renamed to `plugin_manifest_unreadable`. A reader on
`<major>.0` still loads a file stating this minor: `Atlas.from_json` compares the major
alone, and unknown keys and unknown enum members are preserved rather than refused."""

SCHEMA_VERSION = f"{ATLAS_SCHEMA_VERSION}.{SCHEMA_MINOR}"

ANY_PROFILE = "*"
"""Profile key for a fact that holds regardless of profile."""


class Layer(str, Enum):
    DECLARED = "declared"
    CODED = "coded"
    OBSERVED = "observed"
    RECORDED = "recorded"


LAYER_ORDER = tuple(item.value for item in Layer)

EFFECTIVE_PRECEDENCE = ("observed", "recorded", "declared", "coded")
"""A running system outbids a written intent. Losers are always kept and readable.
Parameter values are the documented exception: their authority is the candidate chain,
because the observed value can come from a temp file that exists in no repository."""


class NodeKind(str, Enum):
    PKG = "pkg"
    FILE = "file"
    SYM = "sym"
    EXE = "exe"
    LAUNCHFILE = "launchfile"
    LAUNCHENT = "launchent"
    PROFILE = "profile"
    NODE = "node"
    TOPIC = "topic"
    SERVICE = "service"
    ACTION = "action"
    PARAM = "param"
    PPARAM = "pparam"
    FRAME = "frame"
    TYPE = "type"
    PLUGIN = "plugin"
    PINST = "pinst"
    RUN = "run"
    BAG = "bag"
    BAGTOPIC = "bagtopic"


class EdgeKind(str, Enum):
    CONTAINS = "contains"
    DEFINES = "defines"
    RUNS = "runs"
    LAUNCHES = "launches"
    INCLUDES = "includes"
    GATES = "gates"
    PUBLISHES = "publishes"
    SUBSCRIBES = "subscribes"
    SERVES = "serves"
    CALLS = "calls"
    EXPANDS_TO = "expands_to"
    USES_TYPE = "uses_type"
    HAS_PARAM = "has_param"
    DECLARES_PARAM = "declares_param"
    READS_PARAM = "reads_param"
    SETS_PARAM = "sets_param"
    REMAPS = "remaps"
    BROADCASTS_TF = "broadcasts_tf"
    LISTENS_TF = "listens_tf"
    TF_PARENT = "tf_parent"
    PROVIDES_PLUGIN = "provides_plugin"
    LOADS_PLUGIN = "loads_plugin"
    INSTANTIATES = "instantiates"
    MANAGES_LIFECYCLE = "manages_lifecycle"
    SAMPLES = "samples"


class Confidence(str, Enum):
    CERTAIN = "certain"
    RESOLVED = "resolved"
    INFERRED = "inferred"
    UNKNOWN = "unknown"


class Severity(str, Enum):
    ERROR = "error"
    WARN = "warn"
    INFO = "info"


SEVERITY_ORDER = {Severity.ERROR.value: 0, Severity.WARN.value: 1, Severity.INFO.value: 2}


class Agreement(str, Enum):
    AGREE = "agree"
    DIFFER = "differ"
    SINGLE_LAYER = "single_layer"
    UNKNOWN = "unknown"


class ParamState(str, Enum):
    """Lifecycle of a parameter. Disjoint from Agreement on purpose: one enum space
    per field name, so a reader never has to guess which vocabulary applies."""

    EFFECTIVE = "effective"
    INERT_BY_DESIGN = "inert_by_design"
    UNDECLARED = "undeclared"
    DECLARED_UNREAD = "declared_unread"
    OWNER_NAME_MISMATCH = "owner_name_mismatch"
    UNRESOLVED_OWNER = "unresolved_owner"
    OTHER = "other"


PARAM_STATE_PRIORITY = (
    ParamState.EFFECTIVE.value,
    ParamState.INERT_BY_DESIGN.value,
    ParamState.UNDECLARED.value,
    ParamState.DECLARED_UNREAD.value,
    ParamState.OWNER_NAME_MISMATCH.value,
    ParamState.UNRESOLVED_OWNER.value,
    ParamState.OTHER.value,
)
"""Strongest first, and every member of the vocabulary is ranked. One parameter entity
carries one state while its chain covers several profiles, so a profile that resolved a
value keeps it visible when another profile could not follow the same parameter. The four
states that named an owner outrank the two that could not reach one. The blind profile
stays readable: its candidates never win, so it gets no effective entry, and its unresolved
entry is mirrored on the entity.

The order has to be total. Two states outside it compared equal, so which of them a
parameter ended up with was decided by the order the two profiles were written in."""


def stronger_param_state(left: str, right: str) -> str:
    order = {state: index for index, state in enumerate(PARAM_STATE_PRIORITY)}
    # The name breaks the tie between two states a newer minor added, so an unknown member
    # is not decided by which profile was written first either.
    return min((left, right), key=lambda state: (order.get(state, len(order)), state))


class Mechanism(str, Enum):
    YAML_BASE = "yaml_base"
    YAML_OVERLAY_MERGE = "yaml_overlay_merge"
    YAML_WILDCARD = "yaml_wildcard"
    LAUNCH_PARAMS_FILE = "launch_params_file"
    LAUNCH_INLINE_DICT = "launch_inline_dict"
    LAUNCH_REWRITTEN_YAML = "launch_rewritten_yaml"
    ROS_ARGS_P = "ros_args_p"
    ROS_ARGS_PARAMS_FILE = "ros_args_params_file"
    PYTHON_OVERRIDE = "python_override"
    PARAM_SET_RUNTIME = "param_set_runtime"
    CODE_DEFAULT = "code_default"
    OBSERVED_RUNTIME = "observed_runtime"
    COMMENTED_OUT = "commented_out"
    OTHER = "other"


class ShadowKind(str, Enum):
    VALUE_CHANGED = "value_changed"
    RESTATED_IDENTICAL = "restated_identical"
    TYPE_CHANGED = "type_changed"
    NOT_READ_IN_PROFILE = "not_read_in_profile"
    INERT_PLUGIN_BLOCK = "inert_plugin_block"
    OTHER = "other"


class NameSource(str, Enum):
    LITERAL = "literal"
    EXECUTABLE_DEFAULT = "executable_default"
    ARGV = "argv"
    OBSERVED = "observed"
    RUNTIME_RANDOMISED = "runtime_randomised"
    UNKNOWN = "unknown"


class Root(str, Enum):
    """Which tree a fact was read from. The install tree here holds copies, not
    symlinks, and can be older than src, so a mixed root on one entity is a build
    staleness finding rather than a content finding."""

    SRC = "src"
    INSTALL = "install"
    RUNTIME = "runtime"
    BAG = "bag"


class PassState(str, Enum):
    """Whether one collector of one run finished.

    A collector with nothing to read is `completed`: it read everything there was. Absent
    inputs are `not_attempted`, which is legitimate and says so. Only `refused` means the
    pass stopped with data still to read, which is the state no rate can see, because a
    ratio loses its own denominator when a pass dies.
    """

    COMPLETED = "completed"
    REFUSED = "refused"
    NOT_ATTEMPTED = "not_attempted"


PASSES_KEY = "passes"
"""Where a run records one row per collector it was given."""


class Want(str, Enum):
    """What an unresolved site failed to produce."""

    TOPIC_NAME = "topic_name"
    SERVICE_NAME = "service_name"
    ACTION_NAME = "action_name"
    NODE_NAME = "node_name"
    PARAM_NAME = "param_name"
    PARAM_OWNER = "param_owner"
    PARAM_DEFAULT = "param_default"
    PARAM_VALUE = "param_value"
    FRAME_ID = "frame_id"
    QOS = "qos"
    TYPE = "type"
    RATE = "rate"
    OTHER = "other"


# Allowed (source kind, destination kind) pairs. An edge outside the table is a
# contract violation, not a warning: it means a collector invented a relation.
_N = NodeKind
EDGE_DOMAINS: dict[str, tuple[frozenset[str], frozenset[str]]] = {
    EdgeKind.CONTAINS.value: (
        frozenset({_N.PKG.value, _N.LAUNCHFILE.value, _N.BAG.value}),
        frozenset(
            {
                _N.FILE.value,
                _N.EXE.value,
                _N.LAUNCHFILE.value,
                _N.LAUNCHENT.value,
                _N.BAGTOPIC.value,
                _N.TYPE.value,
            }
        ),
    ),
    EdgeKind.DEFINES.value: (
        frozenset({_N.FILE.value, _N.SYM.value}),
        frozenset({_N.SYM.value, _N.NODE.value, _N.TYPE.value, _N.PLUGIN.value}),
    ),
    EdgeKind.RUNS.value: (frozenset({_N.EXE.value}), frozenset({_N.SYM.value})),
    EdgeKind.LAUNCHES.value: (
        frozenset({_N.LAUNCHENT.value, _N.PROFILE.value}),
        frozenset({_N.NODE.value, _N.LAUNCHENT.value}),
    ),
    EdgeKind.INCLUDES.value: (
        frozenset({_N.LAUNCHFILE.value, _N.LAUNCHENT.value}),
        frozenset({_N.LAUNCHFILE.value}),
    ),
    EdgeKind.GATES.value: (
        frozenset({_N.LAUNCHENT.value, _N.PROFILE.value}),
        frozenset({_N.LAUNCHENT.value, _N.NODE.value}),
    ),
    EdgeKind.PUBLISHES.value: (frozenset({_N.NODE.value}), frozenset({_N.TOPIC.value})),
    EdgeKind.SUBSCRIBES.value: (frozenset({_N.NODE.value}), frozenset({_N.TOPIC.value})),
    EdgeKind.SERVES.value: (
        frozenset({_N.NODE.value}),
        frozenset({_N.SERVICE.value, _N.ACTION.value}),
    ),
    EdgeKind.CALLS.value: (
        frozenset({_N.NODE.value}),
        frozenset({_N.SERVICE.value, _N.ACTION.value}),
    ),
    EdgeKind.EXPANDS_TO.value: (
        frozenset({_N.ACTION.value}),
        frozenset({_N.TOPIC.value, _N.SERVICE.value}),
    ),
    EdgeKind.USES_TYPE.value: (
        frozenset({_N.TOPIC.value, _N.SERVICE.value, _N.ACTION.value, _N.BAGTOPIC.value}),
        frozenset({_N.TYPE.value}),
    ),
    EdgeKind.HAS_PARAM.value: (frozenset({_N.NODE.value}), frozenset({_N.PARAM.value})),
    EdgeKind.DECLARES_PARAM.value: (
        frozenset({_N.SYM.value, _N.NODE.value, _N.PLUGIN.value}),
        frozenset({_N.PARAM.value, _N.PPARAM.value}),
    ),
    EdgeKind.READS_PARAM.value: (
        frozenset({_N.SYM.value, _N.NODE.value}),
        frozenset({_N.PARAM.value, _N.PPARAM.value}),
    ),
    EdgeKind.SETS_PARAM.value: (
        frozenset({_N.FILE.value, _N.SYM.value, _N.LAUNCHENT.value, _N.PROFILE.value}),
        frozenset({_N.PARAM.value}),
    ),
    EdgeKind.REMAPS.value: (
        frozenset({_N.LAUNCHENT.value, _N.FILE.value}),
        frozenset({_N.TOPIC.value, _N.SERVICE.value, _N.ACTION.value}),
    ),
    EdgeKind.BROADCASTS_TF.value: (frozenset({_N.NODE.value}), frozenset({_N.FRAME.value})),
    EdgeKind.LISTENS_TF.value: (frozenset({_N.NODE.value}), frozenset({_N.FRAME.value})),
    EdgeKind.TF_PARENT.value: (frozenset({_N.FRAME.value}), frozenset({_N.FRAME.value})),
    EdgeKind.PROVIDES_PLUGIN.value: (frozenset({_N.PKG.value}), frozenset({_N.PLUGIN.value})),
    EdgeKind.LOADS_PLUGIN.value: (
        frozenset({_N.NODE.value, _N.PINST.value}),
        frozenset({_N.PLUGIN.value}),
    ),
    EdgeKind.INSTANTIATES.value: (frozenset({_N.PINST.value}), frozenset({_N.PLUGIN.value})),
    EdgeKind.MANAGES_LIFECYCLE.value: (
        frozenset({_N.NODE.value, _N.LAUNCHENT.value, _N.PROFILE.value}),
        frozenset({_N.NODE.value}),
    ),
    EdgeKind.SAMPLES.value: (frozenset({_N.BAGTOPIC.value}), frozenset({_N.TOPIC.value})),
}

KEYED_EDGE_KINDS = frozenset(
    {
        EdgeKind.PUBLISHES.value,
        EdgeKind.SUBSCRIBES.value,
        EdgeKind.SERVES.value,
        EdgeKind.CALLS.value,
        EdgeKind.SETS_PARAM.value,
        EdgeKind.REMAPS.value,
        EdgeKind.BROADCASTS_TF.value,
        EdgeKind.LISTENS_TF.value,
        EdgeKind.TF_PARENT.value,
        EdgeKind.LAUNCHES.value,
        EdgeKind.GATES.value,
        EdgeKind.MANAGES_LIFECYCLE.value,
        EdgeKind.DECLARES_PARAM.value,
        EdgeKind.READS_PARAM.value,
    }
)
"""Edge kinds that legitimately repeat between the same pair and therefore carry a
mandatory disambiguator over the producing site (launch entity, profile, source
symbol, urdf model, frame pair). Without it two real sites silently become one."""

VOLATILE_KEYS = frozenset({"volatile", "line_hint"})
"""Keys excluded from canonical_bytes at every depth. Rates, message counts, wall
clocks and advisory line numbers must never enter a commit to commit diff."""

NON_FINITE = "json has no infinity and no not a number"
"""The representation decision for `inf`, `-inf` and `nan`, which a `-p .inf` and a yaml
`.inf` both produce.

json has no spelling for them. `Infinity` and `NaN` are a javascript extension that
`allow_nan=False` refuses and that no strict reader accepts, a string `"inf"` turns a float
into a string and manufactures the type change this tool exists to catch, and a tagged
object turns one value into a mapping every consumer would have to know about. So rosio
represents them the only honest way left: not as a value at all. The value is refused here,
and `unrepresentable` lets a collector ask first and record the raw text as an unresolved
entry, which keeps the fact and its site without inventing a number."""


def _enum_value(value: Any, enum_cls: type[Enum], field_name: str, strict: bool = True) -> str:
    if isinstance(value, enum_cls):
        return str(value.value)
    text = str(value)
    allowed = {str(item.value) for item in enum_cls}
    if strict and text not in allowed:
        raise ContractError(f"{field_name} must be one of {sorted(allowed)}, got {text!r}")
    return text


def canonical_value(value: Any) -> Any:
    """The one value encoder. Types are preserved: an int stays an int, a float stays
    a float. A collector must never turn 20 into 20.0, that alone is a phantom drift."""
    if isinstance(value, bool) or value is None:
        return value
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ContractError(f"{NON_FINITE}: {value!r}")
        return value
    if isinstance(value, str):
        return value
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, Mapping):
        return {str(key): canonical_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set, frozenset)):
        items = [canonical_value(item) for item in value]
        if isinstance(value, (set, frozenset)):
            items.sort(key=_sort_key)
        return items
    raise ContractError(f"value of type {type(value).__name__} is not serialisable: {value!r}")


def _refuse_substitution_markers(value: Any) -> None:
    if isinstance(value, str):
        for match in re.finditer(r"__rosio_command_([0-9a-f]*)__", value):
            try:
                bytes.fromhex(match[1]).decode("utf-8")
            except (ValueError, UnicodeDecodeError):
                continue
            raise ContractError("internal substitution marker in fact values")
    elif isinstance(value, Mapping):
        if "~uncarried" in value:
            raise ContractError("internal substitution marker in fact values")
        for key, item in value.items():
            _refuse_substitution_markers(key)
            _refuse_substitution_markers(item)
    elif isinstance(value, list):
        for item in value:
            _refuse_substitution_markers(item)


def unrepresentable(value: Any) -> str | None:
    """Why `canonical_value` would refuse this value, None when it accepts it.

    A collector asks before it emits, so one value that cannot be written costs its own
    entry and never the whole pass.
    """
    try:
        canonical_value(value)
    except ContractError as exc:
        return str(exc)
    return None


def _sort_key(value: Any) -> tuple[int, str]:
    return (0, json.dumps(canonical_value(value), sort_keys=True))


def dumps(payload: Any) -> str:
    """The pinned serialisation. Every writer in rosio goes through this function."""
    try:
        text = json.dumps(
            payload,
            sort_keys=True,
            indent=2,
            ensure_ascii=True,
            allow_nan=False,
            separators=(",", ": "),
        )
    except ValueError as exc:
        raise ContractError(f"atlas is not serialisable: {exc}") from exc
    return text + "\n"


def dumps_line(payload: Any) -> str:
    """The pinned serialisation of one record of a line oriented file, newline excluded.

    Same encoder, same sorting, same refusal of a non finite float, on one line, so a log
    or an index written a row at a time still goes through this module.
    """
    try:
        return json.dumps(
            payload,
            sort_keys=True,
            ensure_ascii=True,
            allow_nan=False,
            separators=(", ", ": "),
        )
    except ValueError as exc:
        raise ContractError(f"record is not serialisable: {exc}") from exc


def _load_bool(value: Any, where: str) -> bool:
    """A stored boolean, refused when it is anything else.

    A hand edited or truncated file writes the string "false" where a boolean belongs, and
    a plain bool() call reads that as True, which states a layer is present when the file
    says it is not. An unknown key or an unknown enum member is tolerated on load, a wrong
    type on a closed field is not.
    """
    if isinstance(value, bool):
        return value
    raise ContractError(f"{where}: {value!r} is not a boolean, so this atlas is not readable")


def strip_volatile(payload: Any) -> Any:
    """Drop every volatile key at every depth."""
    if isinstance(payload, dict):
        return {
            key: strip_volatile(value) for key, value in payload.items() if key not in VOLATILE_KEYS
        }
    if isinstance(payload, list):
        return [strip_volatile(item) for item in payload]
    return payload


# ---------------------------------------------------------------------------
# provenance


@dataclass(frozen=True)
class Source:
    """One inline provenance row. Never an index into a shared table: a slice of the
    atlas has to be self sufficient, and an index shifts when a file gains a line."""

    file: str
    locator: str = ""
    line_hint: int | None = None
    root: str = Root.SRC.value

    def __post_init__(self) -> None:
        ids.parse(self.file)
        object.__setattr__(self, "root", _enum_value(self.root, Root, "source.root"))

    def to_json(self) -> dict[str, Any]:
        payload: dict[str, Any] = {"file": self.file, "root": self.root}
        if self.locator:
            payload["locator"] = self.locator
        if self.line_hint is not None:
            payload["line_hint"] = int(self.line_hint)
        return payload

    @classmethod
    def from_json(cls, payload: Mapping[str, Any]) -> Source:
        return cls(
            file=str(payload["file"]),
            locator=str(payload.get("locator", "")),
            line_hint=payload.get("line_hint"),
            root=str(payload.get("root", Root.SRC.value)),
        )

    def sort_key(self) -> tuple[str, str, int, str]:
        return (
            self.file,
            self.locator,
            -1 if self.line_hint is None else int(self.line_hint),
            self.root,
        )


def _sorted_sources(sources: Iterable[Source]) -> list[Source]:
    return sorted(dict.fromkeys(sources), key=lambda item: item.sort_key())


def _first_stated(held: str, other: str) -> str:
    """The smaller of two codes, counting an empty one as nothing said."""
    stated = sorted(text for text in (held, other) if text)
    return stated[0] if stated else ""


# ---------------------------------------------------------------------------
# facts


@dataclass
class LayerFact:
    """What one layer, in one profile, said about one entity or one edge."""

    layer: str
    profile: str = ANY_PROFILE
    run: str | None = None
    values: dict[str, Any] = field(default_factory=dict)
    confidence: str = Confidence.RESOLVED.value
    evidence: list[Source] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.layer = _enum_value(self.layer, Layer, "fact.layer")
        self.confidence = _enum_value(self.confidence, Confidence, "fact.confidence")
        self.profile = self.profile or ANY_PROFILE
        if self.profile != ANY_PROFILE:
            ids.parse_profile(self.profile)
        if self.run is not None:
            ids.parse_run(self.run)
        self.values = canonical_value(dict(self.values))
        _refuse_substitution_markers(self.values)
        self.evidence = _sorted_sources(self.evidence)

    @property
    def slot(self) -> tuple[str, str]:
        return (self.layer, self.profile)

    def sort_key(self) -> tuple[int, str, str]:
        order = LAYER_ORDER.index(self.layer) if self.layer in LAYER_ORDER else len(LAYER_ORDER)
        return (order, self.layer, self.profile)

    def to_json(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "layer": self.layer,
            "profile": self.profile,
            "confidence": self.confidence,
            "values": canonical_value(self.values),
            "evidence": [source.to_json() for source in self.evidence],
        }
        if self.run:
            payload["run"] = self.run
        return payload

    @classmethod
    def from_json(cls, payload: Mapping[str, Any]) -> LayerFact:
        fact = cls.__new__(cls)
        fact.layer = _enum_value(payload["layer"], Layer, "fact.layer", strict=False)
        fact.profile = str(payload.get("profile") or ANY_PROFILE)
        fact.run = payload.get("run")
        fact.values = dict(payload.get("values") or {})
        fact.confidence = _enum_value(
            payload.get("confidence", Confidence.RESOLVED.value),
            Confidence,
            "fact.confidence",
            strict=False,
        )
        fact.evidence = _sorted_sources(
            Source.from_json(row) for row in payload.get("evidence") or []
        )
        return fact


@dataclass
class EffectiveValue:
    """The precomputed answer for one key, on one entity, in one profile."""

    value: Any
    layer: str
    confidence: str = Confidence.RESOLVED.value
    value_type: str | None = None
    from_candidate: int | None = None
    source: Source | None = None

    def to_json(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "value": canonical_value(self.value),
            "layer": self.layer,
            "confidence": self.confidence,
        }
        if self.value_type is not None:
            payload["value_type"] = self.value_type
        if self.from_candidate is not None:
            payload["from_candidate"] = int(self.from_candidate)
        if self.source is not None:
            payload["source"] = self.source.to_json()
        return payload

    @classmethod
    def from_json(cls, payload: Mapping[str, Any]) -> EffectiveValue:
        source = payload.get("source")
        return cls(
            value=payload.get("value"),
            layer=str(payload.get("layer", "")),
            confidence=str(payload.get("confidence", Confidence.RESOLVED.value)),
            value_type=payload.get("value_type"),
            from_candidate=payload.get("from_candidate"),
            source=Source.from_json(source) if source else None,
        )


@dataclass
class Candidate:
    """One write site for one parameter, with its own provenance.

    Provenance always terminates at a source file or at the symbol that performed a
    merge. It never points at a temp path, because the merged params file the running
    stack was handed is a random name under /tmp that exists in no repository.
    """

    value: Any
    value_type: str
    mechanism: str
    source: Source
    rank: int
    wins: bool = False
    profile: str = ANY_PROFILE
    shadow_kind: str | None = None
    shadowed_by: int | None = None
    applied_by: str | None = None
    guard: str = ""
    note: str = ""

    def __post_init__(self) -> None:
        self.mechanism = _enum_value(self.mechanism, Mechanism, "candidate.mechanism")
        if self.shadow_kind is not None:
            self.shadow_kind = _enum_value(self.shadow_kind, ShadowKind, "candidate.shadow_kind")
        self.profile = self.profile or ANY_PROFILE
        if self.profile != ANY_PROFILE:
            ids.parse_profile(self.profile)
        if self.applied_by is not None:
            ids.parse(self.applied_by)
        self.value = canonical_value(self.value)

    def sort_key(self) -> tuple[str, int, str, str]:
        return (self.profile, int(self.rank), self.source.file, self.source.locator)

    def to_json(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "rank": int(self.rank),
            "wins": bool(self.wins),
            "value": canonical_value(self.value),
            "value_type": self.value_type,
            "mechanism": self.mechanism,
            "profile": self.profile,
            "source": self.source.to_json(),
        }
        if self.shadow_kind is not None:
            payload["shadow_kind"] = self.shadow_kind
        if self.shadowed_by is not None:
            payload["shadowed_by"] = int(self.shadowed_by)
        if self.applied_by:
            payload["applied_by"] = self.applied_by
        if self.guard:
            payload["guard"] = self.guard
        if self.note:
            payload["note"] = self.note
        return payload

    @classmethod
    def from_json(cls, payload: Mapping[str, Any]) -> Candidate:
        candidate = cls.__new__(cls)
        candidate.value = payload.get("value")
        candidate.value_type = str(payload.get("value_type", ""))
        candidate.mechanism = _enum_value(
            payload.get("mechanism", Mechanism.OTHER.value), Mechanism, "mechanism", strict=False
        )
        candidate.source = Source.from_json(payload["source"])
        candidate.rank = int(payload.get("rank", 0))
        candidate.wins = bool(payload.get("wins", False))
        candidate.profile = str(payload.get("profile") or ANY_PROFILE)
        shadow = payload.get("shadow_kind")
        candidate.shadow_kind = (
            _enum_value(shadow, ShadowKind, "shadow_kind", strict=False) if shadow else None
        )
        candidate.shadowed_by = payload.get("shadowed_by")
        candidate.applied_by = payload.get("applied_by")
        candidate.guard = str(payload.get("guard", ""))
        candidate.note = str(payload.get("note", ""))
        return candidate


@dataclass
class Unresolved:
    """One blind spot. Never promoted to an entity, never a placeholder id.

    Mirrored onto the owning entity by recompute, so a reader of one entity slice
    cannot mistake a hollow extraction for a complete one.
    """

    want: str
    owner: str
    expr: str
    reason: str
    role: str = ""
    resolvable_by: list[str] = field(default_factory=list)
    evidence: list[Source] = field(default_factory=list)
    construction: str = ""

    def __post_init__(self) -> None:
        self.want = _enum_value(self.want, Want, "unresolved.want")
        ids.parse(self.owner)
        self.resolvable_by = sorted(dict.fromkeys(str(item) for item in self.resolvable_by))
        self.evidence = _sorted_sources(self.evidence)

    @property
    def id(self) -> str:
        return ids.build_unresolved(
            self.owner, self.expr, self.want, construction=self.construction
        )

    def to_json(self) -> dict[str, Any]:
        payload = {
            "id": self.id,
            "want": self.want,
            "owner": self.owner,
            "expr": self.expr,
            "reason": self.reason,
            "resolvable_by": list(self.resolvable_by),
            "evidence": [source.to_json() for source in self.evidence],
        }
        if self.role:
            payload["role"] = self.role
        if self.construction:
            payload["construction"] = self.construction
        return payload

    @classmethod
    def from_json(cls, payload: Mapping[str, Any]) -> Unresolved:
        entry = cls.__new__(cls)
        entry.want = _enum_value(payload.get("want", Want.OTHER.value), Want, "want", strict=False)
        entry.owner = str(payload["owner"])
        entry.expr = str(payload.get("expr", ""))
        entry.reason = str(payload.get("reason", ""))
        entry.role = str(payload.get("role", ""))
        entry.construction = str(payload.get("construction", ""))
        entry.resolvable_by = sorted(str(item) for item in payload.get("resolvable_by") or [])
        entry.evidence = _sorted_sources(
            Source.from_json(row) for row in payload.get("evidence") or []
        )
        return entry


# ---------------------------------------------------------------------------
# entities and edges


@dataclass
class Node:
    """One entity. Kind is always recoverable from the id prefix."""

    id: str
    kind: str
    label: str = ""
    facts: list[LayerFact] = field(default_factory=list)
    attrs: dict[str, Any] = field(default_factory=dict)
    aka: list[str] = field(default_factory=list)
    conflicts: list[str] = field(default_factory=list)
    agreement: str = Agreement.UNKNOWN.value
    effective: dict[str, dict[str, EffectiveValue]] = field(default_factory=dict)
    state: str | None = None
    candidates: list[Candidate] = field(default_factory=list)
    name_source: str | None = None
    unresolved: list[str] = field(default_factory=list)
    extra: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.label = self.label or ids.label_for(self.id)

    @property
    def layers(self) -> list[str]:
        return sorted(
            {fact.layer for fact in self.facts},
            key=lambda name: (
                LAYER_ORDER.index(name) if name in LAYER_ORDER else len(LAYER_ORDER),
                name,
            ),
        )

    @property
    def profiles(self) -> list[str]:
        seen = {fact.profile for fact in self.facts}
        seen.update(candidate.profile for candidate in self.candidates)
        return sorted(seen - {ANY_PROFILE})

    def fact(self, layer: str, profile: str = ANY_PROFILE) -> LayerFact | None:
        for item in self.facts:
            if item.slot == (str(layer), profile or ANY_PROFILE):
                return item
        return None

    def to_json(self) -> dict[str, Any]:
        payload: dict[str, Any] = dict(self.extra)
        payload.update(
            {
                "id": self.id,
                "kind": self.kind,
                "label": self.label,
                "layers": self.layers,
                "profiles": self.profiles,
                "agreement": self.agreement,
                "conflicts": list(self.conflicts),
                "attrs": canonical_value(self.attrs),
                "facts": [fact.to_json() for fact in sorted(self.facts, key=LayerFact.sort_key)],
            }
        )
        if self.aka:
            payload["aka"] = sorted(self.aka)
        if self.effective:
            payload["effective"] = {
                profile: {key: value.to_json() for key, value in per_key.items()}
                for profile, per_key in self.effective.items()
            }
        if self.state is not None:
            payload["state"] = self.state
        if self.candidates:
            payload["candidates"] = [
                candidate.to_json() for candidate in sorted(self.candidates, key=Candidate.sort_key)
            ]
        if self.name_source is not None:
            payload["name_source"] = self.name_source
        if self.unresolved:
            payload["unresolved"] = sorted(self.unresolved)
        return payload

    @classmethod
    def from_json(cls, payload: Mapping[str, Any]) -> Node:
        known = {
            "id",
            "kind",
            "label",
            "layers",
            "profiles",
            "agreement",
            "conflicts",
            "attrs",
            "facts",
            "aka",
            "effective",
            "state",
            "candidates",
            "name_source",
            "unresolved",
        }
        node = cls(
            id=str(payload["id"]),
            kind=str(payload["kind"]),
            label=str(payload.get("label", "")),
            facts=[LayerFact.from_json(row) for row in payload.get("facts") or []],
            attrs=dict(payload.get("attrs") or {}),
            aka=sorted(str(item) for item in payload.get("aka") or []),
            conflicts=list(payload.get("conflicts") or []),
            agreement=str(payload.get("agreement", Agreement.UNKNOWN.value)),
            candidates=[Candidate.from_json(row) for row in payload.get("candidates") or []],
            state=payload.get("state"),
            name_source=payload.get("name_source"),
            unresolved=sorted(str(item) for item in payload.get("unresolved") or []),
            extra={key: value for key, value in payload.items() if key not in known},
        )
        node.effective = {
            str(profile): {
                str(key): EffectiveValue.from_json(value) for key, value in per_key.items()
            }
            for profile, per_key in (payload.get("effective") or {}).items()
        }
        return node


@dataclass
class Edge:
    """One relation. The key is mandatory for the kinds that repeat, and a duplicate
    (kind, src, dst, key) is refused at build time rather than silently merged."""

    kind: str
    src: str
    dst: str
    key: str = ""
    facts: list[LayerFact] = field(default_factory=list)
    attrs: dict[str, Any] = field(default_factory=dict)
    conflicts: list[str] = field(default_factory=list)
    agreement: str = Agreement.UNKNOWN.value
    effective: dict[str, dict[str, EffectiveValue]] = field(default_factory=dict)
    extra: dict[str, Any] = field(default_factory=dict)

    @property
    def id(self) -> str:
        return ids.build_edge(self.kind, self.src, self.dst, self.key)

    @property
    def layers(self) -> list[str]:
        return sorted(
            {fact.layer for fact in self.facts},
            key=lambda name: (
                LAYER_ORDER.index(name) if name in LAYER_ORDER else len(LAYER_ORDER),
                name,
            ),
        )

    @property
    def profiles(self) -> list[str]:
        return sorted({fact.profile for fact in self.facts} - {ANY_PROFILE})

    def fact(self, layer: str, profile: str = ANY_PROFILE) -> LayerFact | None:
        for item in self.facts:
            if item.slot == (str(layer), profile or ANY_PROFILE):
                return item
        return None

    def to_json(self) -> dict[str, Any]:
        payload: dict[str, Any] = dict(self.extra)
        payload.update(
            {
                "id": self.id,
                "kind": self.kind,
                "src": self.src,
                "dst": self.dst,
                "key": self.key,
                "layers": self.layers,
                "profiles": self.profiles,
                "agreement": self.agreement,
                "conflicts": list(self.conflicts),
                "attrs": canonical_value(self.attrs),
                "facts": [fact.to_json() for fact in sorted(self.facts, key=LayerFact.sort_key)],
            }
        )
        if self.effective:
            payload["effective"] = {
                profile: {key: value.to_json() for key, value in per_key.items()}
                for profile, per_key in self.effective.items()
            }
        return payload

    @classmethod
    def from_json(cls, payload: Mapping[str, Any]) -> Edge:
        known = {
            "id",
            "kind",
            "src",
            "dst",
            "key",
            "layers",
            "profiles",
            "agreement",
            "conflicts",
            "attrs",
            "facts",
            "effective",
        }
        edge = cls(
            kind=str(payload["kind"]),
            src=str(payload["src"]),
            dst=str(payload["dst"]),
            key=str(payload.get("key", "")),
            facts=[LayerFact.from_json(row) for row in payload.get("facts") or []],
            attrs=dict(payload.get("attrs") or {}),
            conflicts=list(payload.get("conflicts") or []),
            agreement=str(payload.get("agreement", Agreement.UNKNOWN.value)),
            extra={key: value for key, value in payload.items() if key not in known},
        )
        edge.effective = {
            str(profile): {
                str(key): EffectiveValue.from_json(value) for key, value in per_key.items()
            }
            for profile, per_key in (payload.get("effective") or {}).items()
        }
        return edge


@dataclass(frozen=True)
class EdgeRef:
    """Structured reference to an edge. Findings use this instead of embedding an edge
    triple in a finding id."""

    kind: str
    src: str
    dst: str
    key: str = ""

    @property
    def id(self) -> str:
        return ids.build_edge(self.kind, self.src, self.dst, self.key)

    def to_json(self) -> dict[str, Any]:
        return {"kind": self.kind, "src": self.src, "dst": self.dst, "key": self.key}

    @classmethod
    def from_json(cls, payload: Mapping[str, Any]) -> EdgeRef:
        return cls(
            kind=str(payload["kind"]),
            src=str(payload["src"]),
            dst=str(payload["dst"]),
            key=str(payload.get("key", "")),
        )


@dataclass
class Finding:
    """A drift or hygiene result, in the shape of a CI failure line.

    `say` is the one sentence a small model may quote verbatim. It must name the
    concrete values and the file to edit, and it must never contain a synthetic id, a
    finding id or an edge triple.
    """

    rule: str
    severity: str
    subject: str | EdgeRef
    title: str
    detail: str = ""
    fix: str = ""
    say: str = ""
    discriminator: str = ""
    profiles: list[str] = field(default_factory=list)
    layers: list[str] = field(default_factory=list)
    where: list[Source] = field(default_factory=list)
    extra: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.severity = _enum_value(self.severity, Severity, "finding.severity")
        self.profiles = sorted(dict.fromkeys(self.profiles))
        self.layers = sorted(dict.fromkeys(self.layers))
        self.where = _sorted_sources(self.where)
        if isinstance(self.subject, EdgeRef):
            ids.parse(self.subject.src)
            ids.parse(self.subject.dst)
        else:
            ids.parse(self.subject)

    @property
    def subject_key(self) -> str:
        return self.subject.id if isinstance(self.subject, EdgeRef) else self.subject

    @property
    def measured(self) -> str:
        """What a live measurement read this time, kept out of every compared field."""
        return str((self.extra.get("volatile") or {}).get("measured", ""))

    @property
    def spoken(self) -> str:
        """`say` with this run's measurement, for a person reading one report."""
        said = self.say or self.title
        return f"{said} Measured: {self.measured}." if self.measured else said

    @property
    def id(self) -> str:
        return ids.build_finding(self.rule, self.subject_key, self.discriminator)

    @property
    def fingerprint(self) -> str:
        return "sha256:" + ids.digest(
            self.rule,
            self.subject_key,
            self.discriminator,
            " ".join(self.detail.split()),
            length=16,
        )

    def sort_key(self) -> tuple[int, str, str, str]:
        return (
            SEVERITY_ORDER.get(self.severity, len(SEVERITY_ORDER)),
            self.rule,
            self.subject_key,
            self.discriminator,
        )

    def to_json(self) -> dict[str, Any]:
        payload: dict[str, Any] = dict(self.extra)
        payload.update(
            {
                "id": self.id,
                "rule": self.rule,
                "severity": self.severity,
                "title": self.title,
                "detail": self.detail,
                "fix": self.fix,
                "say": self.say,
                "fingerprint": self.fingerprint,
                "profiles": list(self.profiles),
                "layers": list(self.layers),
                "where": [source.to_json() for source in self.where],
            }
        )
        if isinstance(self.subject, EdgeRef):
            payload["about"] = {"edge": self.subject.to_json()}
        else:
            payload["about"] = {"entity": self.subject}
        if self.discriminator:
            payload["discriminator"] = self.discriminator
        return payload

    @classmethod
    def from_json(cls, payload: Mapping[str, Any]) -> Finding:
        about = payload.get("about") or {}
        subject: str | EdgeRef
        if "edge" in about:
            subject = EdgeRef.from_json(about["edge"])
        else:
            subject = str(about.get("entity", ""))
        known = {
            "id",
            "rule",
            "severity",
            "title",
            "detail",
            "fix",
            "say",
            "fingerprint",
            "profiles",
            "layers",
            "where",
            "about",
            "discriminator",
        }
        finding = cls.__new__(cls)
        finding.rule = str(payload["rule"])
        finding.severity = _enum_value(
            payload.get("severity", Severity.INFO.value), Severity, "severity", strict=False
        )
        finding.subject = subject
        finding.title = str(payload.get("title", ""))
        finding.detail = str(payload.get("detail", ""))
        finding.fix = str(payload.get("fix", ""))
        finding.say = str(payload.get("say", ""))
        finding.discriminator = str(payload.get("discriminator", ""))
        finding.profiles = sorted(str(item) for item in payload.get("profiles") or [])
        finding.layers = sorted(str(item) for item in payload.get("layers") or [])
        finding.where = _sorted_sources(Source.from_json(row) for row in payload.get("where") or [])
        finding.extra = {key: value for key, value in payload.items() if key not in known}
        return finding


@dataclass
class LayerStatus:
    present: bool = False
    runs: list[str] = field(default_factory=list)
    reason: str = ""
    root: str = Root.SRC.value

    def to_json(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "present": bool(self.present),
            "runs": sorted(self.runs),
            "root": self.root,
        }
        if self.reason:
            payload["reason"] = self.reason
        return payload

    @classmethod
    def from_json(cls, payload: Mapping[str, Any], name: str = "") -> LayerStatus:
        return cls(
            present=_load_bool(payload.get("present", False), f"layers.{name or '?'}.present"),
            runs=sorted(str(item) for item in payload.get("runs") or []),
            reason=str(payload.get("reason", "")),
            root=str(payload.get("root", Root.SRC.value)),
        )


# ---------------------------------------------------------------------------
# the atlas


@dataclass
class Atlas:
    schema_version: str = SCHEMA_VERSION
    generator: dict[str, Any] = field(default_factory=dict)
    capabilities: list[str] = field(default_factory=list)
    generated: dict[str, Any] = field(default_factory=dict)
    layers: dict[str, LayerStatus] = field(default_factory=dict)
    entities: dict[str, Node] = field(default_factory=dict)
    edges: dict[str, Edge] = field(default_factory=dict)
    findings: list[Finding] = field(default_factory=list)
    unresolved: list[Unresolved] = field(default_factory=list)
    counters: dict[str, Any] = field(default_factory=dict)
    indexes: dict[str, Any] = field(default_factory=dict)
    x: dict[str, Any] = field(default_factory=dict)
    extra: dict[str, Any] = field(default_factory=dict)

    # lookups -------------------------------------------------------------
    def get(self, entity_id: str) -> Node | None:
        return self.entities.get(entity_id)

    def out_edges(self, entity_id: str) -> list[Edge]:
        found = [key for key, edge in self.edges.items() if edge.src == entity_id]
        return [self.edges[key] for key in sorted(found)]

    def in_edges(self, entity_id: str) -> list[Edge]:
        found = [key for key, edge in self.edges.items() if edge.dst == entity_id]
        return [self.edges[key] for key in sorted(found)]

    def by_kind(self, kind: str) -> list[Node]:
        wanted = str(kind)
        found = [key for key, node in self.entities.items() if node.kind == wanted]
        return [self.entities[key] for key in sorted(found)]

    # serialisation -------------------------------------------------------
    def to_json(self) -> dict[str, Any]:
        payload: dict[str, Any] = dict(self.extra)
        payload.update(
            {
                "schema_version": self.schema_version,
                "generator": canonical_value(self.generator),
                "capabilities": sorted(self.capabilities),
                "generated": canonical_value(self.generated),
                "layers": {name: self.layers[name].to_json() for name in sorted(self.layers)},
                "counters": canonical_value(self.counters),
                "indexes": canonical_value(self.indexes),
                "entities": {key: self.entities[key].to_json() for key in sorted(self.entities)},
                "edges": {key: self.edges[key].to_json() for key in sorted(self.edges)},
                "findings": [
                    finding.to_json() for finding in sorted(self.findings, key=Finding.sort_key)
                ],
                "unresolved": [
                    entry.to_json() for entry in sorted(self.unresolved, key=lambda item: item.id)
                ],
                "x": canonical_value(self.x),
            }
        )
        return payload

    def to_bytes(self) -> bytes:
        return dumps(self.to_json()).encode("utf-8")

    def canonical_bytes(self) -> bytes:
        """Bytes of everything that is compared. The generation block and every
        volatile key are excluded, so two runs over unchanged inputs are equal."""
        payload = self.to_json()
        payload.pop("generated", None)
        return dumps(strip_volatile(payload)).encode("utf-8")

    def content_hash(self) -> str:
        return "sha256:" + ids.digest(self.canonical_bytes().decode("utf-8"), length=16)

    @classmethod
    def from_json(cls, payload: Mapping[str, Any]) -> Atlas:
        """Load an atlas. A newer minor is accepted, a newer major is refused."""
        version = str(payload.get("schema_version", SCHEMA_VERSION))
        major = version.split(".", 1)[0]
        want_major = SCHEMA_VERSION.split(".", 1)[0]
        if major != want_major:
            raise ContractError(
                f"atlas schema_version {version} is not readable by this build "
                f"(expected major {want_major})"
            )
        known = {
            "schema_version",
            "generator",
            "capabilities",
            "generated",
            "layers",
            "entities",
            "edges",
            "findings",
            "unresolved",
            "counters",
            "indexes",
            "x",
        }
        return cls(
            schema_version=version,
            generator=dict(payload.get("generator") or {}),
            capabilities=sorted(str(item) for item in payload.get("capabilities") or []),
            generated=dict(payload.get("generated") or {}),
            layers={
                str(name): LayerStatus.from_json(row, str(name))
                for name, row in (payload.get("layers") or {}).items()
            },
            entities={
                str(key): Node.from_json(row)
                for key, row in (payload.get("entities") or {}).items()
            },
            edges={
                str(key): Edge.from_json(row) for key, row in (payload.get("edges") or {}).items()
            },
            findings=[Finding.from_json(row) for row in payload.get("findings") or []],
            unresolved=[Unresolved.from_json(row) for row in payload.get("unresolved") or []],
            counters=dict(payload.get("counters") or {}),
            indexes=dict(payload.get("indexes") or {}),
            x=dict(payload.get("x") or {}),
            extra={key: value for key, value in payload.items() if key not in known},
        )

    @classmethod
    def loads(cls, text: str | bytes) -> Atlas:
        if isinstance(text, bytes):
            text = text.decode("utf-8")
        return cls.from_json(json.loads(text))


# ---------------------------------------------------------------------------
# builder


class AtlasBuilder:
    """The only sanctioned way to grow an atlas.

    Contract enforced here, so no collector can break it by accident:
      * an entity id must parse and its kind must match its prefix
      * a fact for a missing entity creates the entity with the right kind
      * asking for an existing entity with a different kind raises ContractError
      * an edge must reference entities that already exist, and its (source kind,
        destination kind) pair must be in EDGE_DOMAINS
      * repeatable edge kinds must carry a non empty key, and a duplicate
        (kind, src, dst, key) raises ContractError instead of merging two sites
      * a second fact in the same (layer, profile) slot may add keys but may never
        change one, because a slot is written by exactly one run
    """

    def __init__(
        self,
        *,
        tool_version: str = __version__,
        capabilities: Sequence[str] = (),
        generated: Mapping[str, Any] | None = None,
    ) -> None:
        self.atlas = Atlas(
            schema_version=SCHEMA_VERSION,
            generator={"name": "rosio", "version": tool_version, "determinism": "sorted"},
            capabilities=sorted(capabilities),
            generated=dict(generated or {}),
            layers={layer: LayerStatus() for layer in LAYER_ORDER},
        )
        self._resolved_counts: dict[str, int] = {}
        self._row_ids: dict[str, tuple[list[Any], int, dict[str, Any]]] = {}

    # entities ------------------------------------------------------------
    def node(
        self,
        entity_id: str,
        *,
        kind: str | None = None,
        label: str | None = None,
        attrs: Mapping[str, Any] | None = None,
        name_source: str | None = None,
    ) -> Node:
        parsed = ids.parse(entity_id)
        if parsed.kind not in ids.ENTITY_KINDS:
            raise ContractError(f"{entity_id!r} is not an entity id")
        want = _enum_value(kind, NodeKind, "node.kind") if kind is not None else parsed.kind
        if want != parsed.kind:
            raise ContractError(
                f"entity {entity_id!r} has kind {parsed.kind} from its id, caller asked for {want}"
            )
        existing = self.atlas.entities.get(entity_id)
        if existing is None:
            existing = Node(id=entity_id, kind=parsed.kind, label=label or "")
            self.atlas.entities[entity_id] = existing
        elif existing.kind != parsed.kind:
            raise ContractError(f"entity {entity_id!r} already exists with kind {existing.kind}")
        if label:
            existing.label = label
        if attrs:
            _merge_values(existing.attrs, canonical_value(dict(attrs)), f"{entity_id} attrs")
        if name_source is not None:
            offered = _enum_value(name_source, NameSource, "name_source")
            # Every writer folds, so the field is what the strongest source said and not what
            # the last pass said. Writing it flat made the atlas move when the collectors were
            # reordered, which AGENTS section 9 point 7 refuses. `ids.NAME_SOURCE_ORDER` states
            # the precedence and the reason.
            existing.name_source = ids.strongest_name_source(existing.name_source, offered)
        return existing

    def fact(
        self,
        entity_id: str,
        layer: str,
        *,
        profile: str = ANY_PROFILE,
        run: str | None = None,
        values: Mapping[str, Any] | None = None,
        confidence: str = Confidence.RESOLVED.value,
        evidence: Iterable[Source] = (),
        attrs: Mapping[str, Any] | None = None,
    ) -> LayerFact:
        node = self.node(entity_id, attrs=attrs)
        return _add_fact(
            node.facts,
            entity_id,
            LayerFact(
                layer=layer,
                profile=profile,
                run=run,
                values=dict(values or {}),
                confidence=confidence,
                evidence=list(evidence),
            ),
        )

    # edges ---------------------------------------------------------------
    def edge(
        self,
        kind: str,
        src: str,
        dst: str,
        *,
        key: str = "",
        layer: str | None = None,
        profile: str = ANY_PROFILE,
        run: str | None = None,
        values: Mapping[str, Any] | None = None,
        confidence: str = Confidence.RESOLVED.value,
        evidence: Iterable[Source] = (),
        attrs: Mapping[str, Any] | None = None,
    ) -> Edge:
        edge_kind = _enum_value(kind, EdgeKind, "edge.kind")
        for endpoint in (src, dst):
            if endpoint not in self.atlas.entities:
                raise ContractError(
                    f"edge {edge_kind} references {endpoint!r}, which is not in the atlas"
                )
        src_kinds, dst_kinds = EDGE_DOMAINS[edge_kind]
        src_kind = self.atlas.entities[src].kind
        dst_kind = self.atlas.entities[dst].kind
        if src_kind not in src_kinds or dst_kind not in dst_kinds:
            raise ContractError(f"edge {edge_kind} is not allowed from {src_kind} to {dst_kind}")
        if edge_kind in KEYED_EDGE_KINDS and not key:
            raise ContractError(
                f"edge {edge_kind} needs a key over its producing site "
                "(launch entity, profile, symbol, urdf model or frame pair)"
            )
        handle = ids.build_edge(edge_kind, src, dst, key)
        existing = self.atlas.edges.get(handle)
        if existing is None:
            existing = Edge(kind=edge_kind, src=src, dst=dst, key=key)
            self.atlas.edges[handle] = existing
        elif edge_kind in KEYED_EDGE_KINDS:
            added = _second_site(
                existing,
                layer,
                profile,
                dict(values or {}),
                confidence,
                list(evidence),
                canonical_value(dict(attrs)) if attrs else {},
            )
            if added:
                raise ContractError(
                    f"edge {edge_kind} from {src} to {dst} already carries key {key!r} and "
                    f"this call states {added}, so two producing sites share one key. "
                    "give the second site a key of its own with ids.edge_key over its "
                    "launch entity, profile, symbol, urdf model or frame pair"
                )
        if attrs:
            _merge_values(existing.attrs, canonical_value(dict(attrs)), f"{handle} attrs")
        if layer is not None:
            _add_fact(
                existing.facts,
                handle,
                LayerFact(
                    layer=layer,
                    profile=profile,
                    run=run,
                    values=dict(values or {}),
                    confidence=confidence,
                    evidence=list(evidence),
                ),
            )
        elif values:
            raise ContractError(f"edge {handle} was given values with no layer")
        return existing

    # parameters ----------------------------------------------------------
    def set_param(
        self,
        param_id: str,
        *,
        state: str,
        candidates: Iterable[Candidate] = (),
        note: str = "",
    ) -> Node:
        """Write the chain of the profiles this call names, keeping every other profile.

        A parameter entity is shared by every profile that sets it, while a run covers one
        profile, so assigning the whole list would leave only the profile written last with
        a chain. The rows of the profiles named here replace those profiles, the rows of the
        profiles not named stay, and the invariants are checked over the result: ranks are
        unique per profile and at most one candidate wins per profile.
        """
        node = self.node(param_id)
        if node.kind not in (NodeKind.PARAM.value, NodeKind.PPARAM.value):
            raise ContractError(f"{param_id!r} is not a parameter id")
        wanted = _enum_value(state, ParamState, "param.state")
        written = list(candidates)
        touched = {candidate.profile for candidate in written}
        kept = [candidate for candidate in node.candidates if candidate.profile not in touched]
        rows = kept + written
        seen: set[tuple[str, int]] = set()
        winners: dict[str, int] = {}
        for candidate in rows:
            slot = (candidate.profile, int(candidate.rank))
            if slot in seen:
                raise ContractError(
                    f"parameter {param_id} has two candidates at rank {candidate.rank} "
                    f"in profile {candidate.profile}"
                )
            seen.add(slot)
            if candidate.wins:
                winners[candidate.profile] = winners.get(candidate.profile, 0) + 1
        for profile, count in winners.items():
            if count > 1:
                raise ContractError(
                    f"parameter {param_id} has {count} winning candidates in profile {profile}"
                )
        node.state = stronger_param_state(node.state, wanted) if kept and node.state else wanted
        node.candidates = rows
        if note:
            node.attrs.setdefault("notes", [])
            notes = node.attrs["notes"]
            if note not in notes:
                notes.append(note)
                notes.sort()
        return node

    # findings, blind spots, provenance -----------------------------------
    def finding(self, finding: Finding) -> Finding:
        subject = finding.subject_key
        if isinstance(finding.subject, str) and subject not in self.atlas.entities:
            raise ContractError(f"finding {finding.rule} is about unknown entity {subject!r}")
        sigil = ids.KIND_SEP + ids.SYNTHETIC
        if sigil in finding.say or sigil in finding.title:
            raise ContractError(
                "a finding say or title must not contain a synthetic id, use the label"
            )
        key = finding.id
        if key in self._ids_of("findings"):
            raise ContractError(f"duplicate finding id {key}")
        self._append("findings", key, finding)
        return finding

    def unresolved(self, entry: Unresolved) -> Unresolved:
        key = entry.id
        existing = self._ids_of("unresolved").get(key)
        if existing is not None:
            return existing
        self._append("unresolved", key, entry)
        return entry

    def _ids_of(self, name: str) -> dict[str, Any]:
        """The first row per id of one atlas list, rebuilt when the list was replaced.

        A join or a supersession assigns a new list rather than editing rows in place, so
        the list object and its length are enough to tell a stale index from a live one.
        """
        rows = getattr(self.atlas, name)
        held = self._row_ids.get(name)
        if held is None or held[0] is not rows or held[1] != len(rows):
            index: dict[str, Any] = {}
            for row in rows:
                index.setdefault(row.id, row)
            held = self._row_ids[name] = (rows, len(rows), index)
        return held[2]

    def _append(self, name: str, key: str, row: Any) -> None:
        index = self._ids_of(name)
        rows = getattr(self.atlas, name)
        rows.append(row)
        index.setdefault(key, row)
        self._row_ids[name] = (rows, len(rows), index)

    def count_resolved(self, want: str, count: int = 1) -> None:
        """Record a site that WAS resolved, so resolution_rate has a denominator."""
        name = _enum_value(want, Want, "want")
        self._resolved_counts[name] = self._resolved_counts.get(name, 0) + int(count)

    def declare_layer(
        self,
        layer: str,
        *,
        present: bool,
        run: str | None = None,
        reason: str = "",
        root: str = Root.SRC.value,
    ) -> LayerStatus:
        name = _enum_value(layer, Layer, "layer")
        status = self.atlas.layers.setdefault(name, LayerStatus())
        status.present = present
        status.reason = reason
        status.root = _enum_value(root, Root, "layer.root")
        if run:
            ids.parse_run(run)
            if run not in status.runs:
                status.runs.append(run)
                status.runs.sort()
        return status

    def add_run(
        self,
        layer: str,
        *,
        profile: str | None = None,
        input_digest: str,
        tool_version: str = __version__,
        roots: Mapping[str, str] | None = None,
        env: Mapping[str, str] | None = None,
        predicates: Mapping[str, bool] | None = None,
        exceptions: Sequence[Mapping[str, Any]] = (),
        volatile: Mapping[str, Any] | None = None,
    ) -> str:
        """Register one extraction pass.

        `predicates` holds the machine dependent inputs (which paths existed, which
        launch arguments were injected). Without them, two boxes that disagree only
        about a file on disk look like drift instead of a different input.
        """
        name = _enum_value(layer, Layer, "layer")
        slug = ids.parse_profile(profile) if profile else ""
        run_id = ids.build_run(name, slug, input_digest)
        self.node(
            run_id,
            attrs={
                "layer": name,
                "profile": profile or "",
                "tool_version": tool_version,
                "input_digest": input_digest,
                "roots": dict(roots or {}),
                "env": dict(env or {}),
                "predicates": dict(predicates or {}),
                "exceptions": [dict(item) for item in exceptions],
                "volatile": dict(volatile or {}),
            },
        )
        return run_id

    def record_pass(
        self,
        run_id: str,
        collector: str,
        *,
        state: str,
        reason: str = "",
        missing: str = "",
    ) -> dict[str, Any]:
        """State whether one collector of this run finished, and what it cost when it did not.

        A run says which collectors it intended to run. Until this row exists nothing says
        whether each of them got to the end, so an atlas whose C++ pass died reads exactly
        like one that had no C++ to read. `missing` is what the atlas therefore does not
        hold, in the words of the orchestrator that knows the collector set.
        """
        node = self.atlas.entities.get(run_id)
        if node is None or node.kind != NodeKind.RUN.value:
            raise ContractError(f"cannot record a pass on {run_id!r}, which is not a run")
        if not collector:
            raise ContractError(f"a pass on {run_id} needs the name of the collector that ran")
        wanted = _enum_value(state, PassState, "pass.state")
        if wanted != PassState.COMPLETED.value and not reason:
            raise ContractError(f"pass {collector} on {run_id} is {wanted} and states no reason")
        rows = node.attrs.setdefault(PASSES_KEY, [])
        row: dict[str, Any] = {"collector": collector, "state": wanted}
        if reason:
            row["reason"] = reason
        if missing:
            row["missing"] = missing
        for held in rows:
            if str(held.get("collector")) != collector:
                continue
            # One pass states one outcome. The same outcome twice is one statement, and a
            # different one means two passes wrote one run, which the row cannot express.
            if held == row:
                return held
            raise ContractError(f"run {run_id} already records the {collector} pass differently")
        rows.append(row)
        rows.sort(key=lambda item: str(item.get("collector", "")))
        return row

    def add_profile(
        self,
        slug: str,
        *,
        label: str = "",
        entry: Sequence[str] = (),
        entry_kind: str = "",
        inputs: Mapping[str, Any] | None = None,
    ) -> str:
        profile_id = ids.build_profile(slug)
        for item in entry:
            ids.parse(item)
        self.node(
            profile_id,
            label=label or slug,
            attrs={
                "entry": sorted(entry),
                "entry_kind": entry_kind,
                "inputs": dict(inputs or {}),
            },
        )
        return profile_id

    # identity ------------------------------------------------------------
    def supersede(self, old_id: str, new_id: str) -> Node:
        """Resolve a synthetic identity to a real one, in one pass, with one survivor.

        Facts and attrs merge, the old id is kept in `aka`, every edge endpoint is
        rewritten, every derived id (parameters, plugin instances) is superseded in
        turn, and the old entity is deleted. Ambiguous matches must not call this:
        the collector keeps both entities and reports one identity finding instead.
        """
        if old_id == new_id:
            return self.node(old_id)
        old = self.atlas.entities.get(old_id)
        if old is None:
            raise ContractError(f"cannot supersede unknown entity {old_id!r}")
        if ids.kind_of(old_id) != ids.kind_of(new_id):
            raise ContractError(f"cannot supersede {old_id!r} by {new_id!r}, the kinds differ")
        new = self.node(new_id)
        for fact in old.facts:
            _add_fact(new.facts, new_id, fact)
        _merge_values(new.attrs, old.attrs, f"{new_id} attrs")
        for alias in [old_id, *old.aka]:
            if alias not in new.aka:
                new.aka.append(alias)
        new.aka.sort()
        if new.name_source is None:
            new.name_source = old.name_source
        if old.state is not None:
            new.state = (
                old.state if new.state is None else stronger_param_state(new.state, old.state)
            )
        new.candidates = _merge_candidates(new_id, new.candidates, old.candidates)
        self._rewrite_unresolved_owner(old_id, new_id)
        for finding in self.atlas.findings:
            if finding.subject == old_id:
                finding.subject = new_id
            elif isinstance(finding.subject, EdgeRef):
                finding.subject = EdgeRef(
                    kind=finding.subject.kind,
                    src=new_id if finding.subject.src == old_id else finding.subject.src,
                    dst=new_id if finding.subject.dst == old_id else finding.subject.dst,
                    key=finding.subject.key,
                )
        del self.atlas.entities[old_id]
        self._rewrite_edges(old_id, new_id)
        for derived_old, derived_new in self._derived_pairs(old_id, new_id):
            self.supersede(derived_old, derived_new)
        return new

    def _rewrite_unresolved_owner(self, old_id: str, new_id: str) -> None:
        """Move every blind spot of the old identity onto the new one, one entry per spot.

        An unresolved id is a digest over the owner, the expression and what was wanted, so
        rewriting the owner is what decides the id. Three synthetic identities of one process
        resolving to one name therefore land on one id, and leaving the rows side by side
        gives the atlas two entries with one id, which is a schema violation and a reader
        that cannot tell one blind spot from two. Rows that land on one id are one blind
        spot and are merged into it here.

        The merge is a union, so it holds whatever order the identities resolve in: every
        evidence row and every repair hint of both rows survives, and a stated reason or
        role beats an unstated one with the smaller of the two stated codes winning. That is
        richer than `unresolved`, which keeps the row it already holds and returns it, so a
        run that knew the name first states one site of a blind spot and this states all of
        them.
        """
        if not any(entry.owner == old_id for entry in self.atlas.unresolved):
            return
        merged: dict[str, Unresolved] = {}
        for entry in self.atlas.unresolved:
            if entry.owner == old_id:
                entry.owner = new_id
            held = merged.get(entry.id)
            if held is None:
                merged[entry.id] = entry
                continue
            held.reason = _first_stated(held.reason, entry.reason)
            held.role = _first_stated(held.role, entry.role)
            held.resolvable_by = sorted(dict.fromkeys([*held.resolvable_by, *entry.resolvable_by]))
            held.evidence = _sorted_sources([*held.evidence, *entry.evidence])
        self.atlas.unresolved = [merged[key] for key in sorted(merged)]

    def _rewrite_edges(self, old_id: str, new_id: str) -> None:
        for handle in sorted(self.atlas.edges):
            edge = self.atlas.edges[handle]
            if old_id not in (edge.src, edge.dst):
                continue
            del self.atlas.edges[handle]
            edge.src = new_id if edge.src == old_id else edge.src
            edge.dst = new_id if edge.dst == old_id else edge.dst
            target = edge.id
            merged = self.atlas.edges.get(target)
            if merged is None:
                self.atlas.edges[target] = edge
                continue
            for fact in edge.facts:
                _add_fact(merged.facts, target, fact)
            _merge_values(merged.attrs, edge.attrs, f"{target} attrs")

    def _derived_pairs(self, old_id: str, new_id: str) -> list[tuple[str, str]]:
        if ids.kind_of(old_id) != NodeKind.NODE.value:
            return []
        pairs: list[tuple[str, str]] = []
        for entity_id in sorted(self.atlas.entities):
            kind = self.atlas.entities[entity_id].kind
            if kind == NodeKind.PARAM.value:
                owner, name = ids.parse_param(entity_id)
                if owner == old_id:
                    pairs.append((entity_id, ids.build_param(new_id, name)))
            elif kind == NodeKind.PINST.value:
                owner, instance = ids.parse_pinst(entity_id)
                if owner == old_id:
                    pairs.append((entity_id, ids.build_pinst(new_id, instance)))
        return pairs

    # derived data --------------------------------------------------------
    def recompute(self) -> Atlas:
        """Recompute everything derived. Pure function of entities, edges, findings and
        unresolved entries, and a fixpoint: running it twice changes nothing."""
        atlas = self.atlas
        for node in atlas.entities.values():
            node.conflicts, node.agreement = _agreement(node.facts)
            if node.candidates:
                node.effective = _effective_from_candidates(node)
                if not node.facts:
                    # A candidate chain is a declared layer statement on its own.
                    node.agreement = Agreement.SINGLE_LAYER.value
            elif node.state in (None, ParamState.EFFECTIVE.value):
                node.effective = _effective_from_facts(node.facts)
            else:
                # A parameter that is not effective never carries an effective value, and
                # a fact from a later layer must not create one.
                node.effective = {}
            node.unresolved = []
        for edge in atlas.edges.values():
            edge.conflicts, edge.agreement = _agreement(edge.facts)
            edge.effective = _effective_from_facts(edge.facts)
        for entry in atlas.unresolved:
            owner = atlas.entities.get(entry.owner)
            if owner is not None and entry.id not in owner.unresolved:
                owner.unresolved.append(entry.id)
        for node in atlas.entities.values():
            node.unresolved.sort()
        atlas.findings.sort(key=Finding.sort_key)
        atlas.unresolved.sort(key=lambda item: item.id)
        atlas.counters = self._counters()
        atlas.indexes = self._indexes()
        return atlas

    def build(self) -> Atlas:
        self.recompute()
        _validate_param_states(self.atlas)
        return self.atlas

    def _counters(self) -> dict[str, Any]:
        atlas = self.atlas
        by_kind: dict[str, int] = {}
        for node in atlas.entities.values():
            by_kind[node.kind] = by_kind.get(node.kind, 0) + 1
        edges_by_kind: dict[str, int] = {}
        for edge in atlas.edges.values():
            edges_by_kind[edge.kind] = edges_by_kind.get(edge.kind, 0) + 1
        severities = dict.fromkeys(SEVERITY_ORDER, 0)
        for finding in atlas.findings:
            severities[finding.severity] = severities.get(finding.severity, 0) + 1
        unresolved: dict[str, int] = {}
        for entry in atlas.unresolved:
            unresolved[entry.want] = unresolved.get(entry.want, 0) + 1
        rates: dict[str, float] = {}
        for want in sorted(set(unresolved) | set(self._resolved_counts)):
            resolved = self._resolved_counts.get(want, 0)
            total = resolved + unresolved.get(want, 0)
            rates[want] = round(resolved / total, 6) if total else 1.0
        return {
            "entities": len(atlas.entities),
            "entities_by_kind": by_kind,
            "edges": len(atlas.edges),
            "edges_by_kind": edges_by_kind,
            "findings": severities,
            "unresolved": unresolved,
            "resolved": dict(self._resolved_counts),
            "resolution_rate": rates,
            "passes": completeness(atlas),
            "layers_present": sorted(
                name for name, status in atlas.layers.items() if status.present
            ),
        }

    def _indexes(self) -> dict[str, Any]:
        atlas = self.atlas
        by_kind: dict[str, list[str]] = {}
        for entity_id in sorted(atlas.entities):
            by_kind.setdefault(atlas.entities[entity_id].kind, []).append(entity_id)
        out: dict[str, list[str]] = {}
        into: dict[str, list[str]] = {}
        for handle in sorted(atlas.edges):
            edge = atlas.edges[handle]
            out.setdefault(edge.src, []).append(handle)
            into.setdefault(edge.dst, []).append(handle)
        differ = sorted(
            [
                entity_id
                for entity_id, node in atlas.entities.items()
                if node.agreement == Agreement.DIFFER.value
            ]
            + [
                handle
                for handle, edge in atlas.edges.items()
                if edge.agreement == Agreement.DIFFER.value
            ]
        )
        layer_only: dict[str, list[str]] = {}
        for entity_id in sorted(atlas.entities):
            seen = atlas.entities[entity_id].layers
            if len(seen) == 1:
                layer_only.setdefault(seen[0], []).append(entity_id)
        by_profile: dict[str, list[str]] = {}
        for entity_id in sorted(atlas.entities):
            for profile in atlas.entities[entity_id].profiles:
                by_profile.setdefault(profile, []).append(entity_id)
        findings_by_rule: dict[str, list[str]] = {}
        for finding in sorted(atlas.findings, key=Finding.sort_key):
            findings_by_rule.setdefault(finding.rule, []).append(finding.id)
        return {
            "by_kind": by_kind,
            "out": out,
            "in": into,
            "differ": differ,
            "layer_only": layer_only,
            "by_profile": by_profile,
            "findings_by_rule": findings_by_rule,
        }


# ---------------------------------------------------------------------------
# pass completeness, which the resolution rate cannot express


def passes_of(atlas: Atlas) -> list[tuple[str, dict[str, Any]]]:
    """Every recorded pass in the atlas, run id first, sorted by run then collector."""
    out: list[tuple[str, dict[str, Any]]] = []
    for run_id in sorted(atlas.entities):
        node = atlas.entities[run_id]
        if node.kind != NodeKind.RUN.value:
            continue
        for row in node.attrs.get(PASSES_KEY) or []:
            out.append((run_id, dict(row)))
    return sorted(out, key=lambda item: (item[0], str(item[1].get("collector", ""))))


def incomplete_passes(atlas: Atlas) -> list[tuple[str, dict[str, Any]]]:
    """The passes that stopped with data still to read. Absent inputs are not in here."""
    return [row for row in passes_of(atlas) if row[1].get("state") == PassState.REFUSED.value]


def completeness(atlas: Atlas) -> dict[str, Any]:
    """Whether the atlas is whole, in one object beside the resolution rate.

    The rate is a ratio per want, so a pass that died contributes neither a resolved nor an
    unresolved count and the number moves the wrong way: the more a collector loses, the
    healthier it reads. This counts passes instead, so a reader and a gate can both ask one
    question, and an atlas written before passes were recorded answers `complete` with an
    attempted count of zero rather than a false claim.
    """
    rows = [row for _, row in passes_of(atlas)]
    completed = sum(1 for row in rows if row.get("state") == PassState.COMPLETED.value)
    refused = sum(1 for row in rows if row.get("state") == PassState.REFUSED.value)
    skipped = sum(1 for row in rows if row.get("state") == PassState.NOT_ATTEMPTED.value)
    return {
        "attempted": len(rows) - skipped,
        "completed": completed,
        "refused": refused,
        "not_attempted": skipped,
        "refused_collectors": sorted(
            {
                str(row.get("collector", ""))
                for row in rows
                if row.get("state") == PassState.REFUSED.value
            }
        ),
        "complete": refused == 0,
    }


# ---------------------------------------------------------------------------
# helpers shared by the builder and the derived passes


def _second_site(
    edge: Edge,
    layer: str | None,
    profile: str,
    values: Mapping[str, Any],
    confidence: str,
    evidence: Sequence[Source],
    attrs: Mapping[str, Any],
) -> str:
    """What a repeat write of one keyed edge states that the edge does not already hold.

    The key is the producing site, so one (layer, profile) slot of a keyed edge is one
    site. Restating it is how a rerun and the join stay fixpoints, and it is allowed. A
    repeat that carries anything new is a second site wearing the key of the first, and
    merging the two would report one publication where a workspace has two.

    A layer that never wrote this edge before is a new view of the same site, not a second
    site, so it opens its own slot.
    """
    for key, value in attrs.items():
        if key not in edge.attrs or edge.attrs[key] != value:
            return f"attrs.{key}"
    if layer is None:
        return ""
    slot = next((row for row in edge.facts if row.slot == (layer, profile)), None)
    if slot is None:
        return ""
    for key, value in values.items():
        if key not in slot.values or slot.values[key] != value:
            return f"{layer} values.{key}"
    if confidence != slot.confidence:
        return f"{layer} confidence {confidence}"
    known = set(slot.evidence)
    for source in evidence:
        if source not in known:
            return f"{layer} evidence at {source.locator or source.file}"
    return ""


def _add_fact(target: list[LayerFact], owner: str, fact: LayerFact) -> LayerFact:
    for existing in target:
        if existing.slot != fact.slot:
            continue
        _merge_values(existing.values, fact.values, f"{owner} {existing.layer} values")
        existing.evidence = _sorted_sources([*existing.evidence, *fact.evidence])
        if existing.run is None:
            existing.run = fact.run
        elif fact.run is not None and existing.run != fact.run:
            raise ContractError(
                f"{owner} slot ({existing.layer}, {existing.profile}) was written by two runs"
            )
        if existing.confidence != fact.confidence:
            existing.confidence = _weakest(existing.confidence, fact.confidence)
        return existing
    target.append(fact)
    target.sort(key=LayerFact.sort_key)
    return fact


_CONFIDENCE_RANK = {
    Confidence.CERTAIN.value: 0,
    Confidence.RESOLVED.value: 1,
    Confidence.INFERRED.value: 2,
    Confidence.UNKNOWN.value: 3,
}


def _weakest(left: str, right: str) -> str:
    return max((left, right), key=lambda name: _CONFIDENCE_RANK.get(name, 3))


def _merge_values(target: dict[str, Any], incoming: Mapping[str, Any], where: str) -> None:
    for key, value in incoming.items():
        if isinstance(target.get(key), dict) and isinstance(value, dict):
            _merge_values(target[key], value, f"{where}.{key}")
            continue
        if key in target and target[key] != value:
            raise ContractError(
                f"{where}: key {key!r} already holds {target[key]!r}, refusing to overwrite "
                f"with {value!r}"
            )
        target[key] = value


def _merge_candidates(
    param_id: str, target: Sequence[Candidate], incoming: Sequence[Candidate]
) -> list[Candidate]:
    """Union two candidate chains by profile, one survivor per profile.

    A chain is keyed by profile and every profile is a truth of its own, so a supersession
    that kept only one side dropped whatever the other side had followed. A profile stated
    on both sides is either the same chain twice, which is one truth, or two chains for one
    slot, which is a collision and cannot be merged by picking one.
    """
    if not incoming:
        return list(target)
    if not target:
        return list(incoming)
    held = _chains_by_profile(target)
    added = _chains_by_profile(incoming)
    for profile in sorted(set(held) & set(added)):
        left = [row.to_json() for row in held[profile]]
        right = [row.to_json() for row in added[profile]]
        if left != right:
            raise ContractError(
                f"parameter {param_id} is superseded onto a chain that already states "
                f"profile {profile}, with different rows on each side"
            )
    rows = list(target) + [
        row for profile, chain in added.items() if profile not in held for row in chain
    ]
    return sorted(rows, key=Candidate.sort_key)


def _chains_by_profile(rows: Sequence[Candidate]) -> dict[str, list[Candidate]]:
    out: dict[str, list[Candidate]] = {}
    for row in sorted(rows, key=Candidate.sort_key):
        out.setdefault(row.profile, []).append(row)
    return out


def _flatten(values: Mapping[str, Any], prefix: str = "") -> dict[str, Any]:
    """Dotted view of one values dict. Volatile keys are dropped here, so a rate or a
    message count can never enter a conflict, an effective value or a diff."""
    flat: dict[str, Any] = {}
    for key in sorted(values):
        if key in VOLATILE_KEYS:
            continue
        value = values[key]
        dotted = f"{prefix}{key}"
        if isinstance(value, Mapping):
            flat.update(_flatten(value, prefix=f"{dotted}."))
        else:
            flat[dotted] = value
    return flat


def _profile_groups(facts: Sequence[LayerFact]) -> dict[str, list[LayerFact]]:
    """The facts each profile sees, its own before the `*` ones.

    Readers take the first value of a field per layer, so a profile's own statement wins and a
    shared one only fills the fields it leaves open.
    """
    profiles = sorted({fact.profile for fact in facts} - {ANY_PROFILE})
    shared = [fact for fact in facts if fact.profile == ANY_PROFILE]
    if not profiles:
        return {ANY_PROFILE: list(shared)} if shared else {}
    return {
        profile: [fact for fact in facts if fact.profile == profile] + shared
        for profile in profiles
    }


def _agreement(facts: Sequence[LayerFact]) -> tuple[list[str], str]:
    """Precomputed conflicts and agreement, computed per profile and then reduced.

    Two profiles are allowed to disagree, that is configuration. Two layers inside one
    profile disagreeing is drift, and the dotted key names say exactly which field.

    The reduction is per profile on purpose. Unioning the layers seen across profiles would
    call an entity `agree` when its only declared fact sits in one profile and its only coded
    fact in another, so no two layers were ever compared anywhere.
    """
    if not facts:
        return [], Agreement.UNKNOWN.value
    conflicts: set[str] = set()
    usable = 0
    compared = False
    for group in _profile_groups(facts).values():
        per_key: dict[str, dict[str, Any]] = {}
        layers_here: set[str] = set()
        for fact in group:
            if fact.confidence == Confidence.UNKNOWN.value:
                continue
            usable += 1
            layers_here.add(fact.layer)
            for dotted, value in _flatten(fact.values).items():
                if value is None:
                    continue
                per_key.setdefault(dotted, {}).setdefault(fact.layer, value)
        if len(layers_here) > 1:
            compared = True
        for dotted, per_layer in per_key.items():
            if len(per_layer) < 2:
                continue
            distinct = {
                json.dumps(canonical_value(item), sort_keys=True) for item in per_layer.values()
            }
            if len(distinct) > 1:
                conflicts.add(dotted)
    if conflicts:
        return sorted(conflicts), Agreement.DIFFER.value
    if usable == 0:
        return [], Agreement.UNKNOWN.value
    if not compared:
        return [], Agreement.SINGLE_LAYER.value
    return [], Agreement.AGREE.value


def _effective_from_facts(
    facts: Sequence[LayerFact],
) -> dict[str, dict[str, EffectiveValue]]:
    out: dict[str, dict[str, EffectiveValue]] = {}
    for profile, group in _profile_groups(facts).items():
        per_key: dict[str, EffectiveValue] = {}
        for layer in EFFECTIVE_PRECEDENCE:
            for fact in group:
                if fact.layer != layer or fact.confidence == Confidence.UNKNOWN.value:
                    continue
                for dotted, value in _flatten(fact.values).items():
                    if value is None or dotted in per_key:
                        continue
                    per_key[dotted] = EffectiveValue(
                        value=value, layer=layer, confidence=fact.confidence
                    )
        if per_key:
            out[profile] = dict(sorted(per_key.items()))
    return out


def _effective_from_candidates(node: Node) -> dict[str, dict[str, EffectiveValue]]:
    """A parameter's authority is its candidate chain, not the observed layer.

    The observed value of a parameter can come from a merged temp file that exists in
    no repository, so an agent told to read `effective` would otherwise be handed a
    number it cannot edit anywhere. The observed value stays visible as a fact.
    """
    if node.state != ParamState.EFFECTIVE.value:
        return {}
    out: dict[str, dict[str, EffectiveValue]] = {}
    for candidate in sorted(node.candidates, key=Candidate.sort_key):
        if not candidate.wins:
            continue
        out[candidate.profile] = {
            "value": EffectiveValue(
                value=candidate.value,
                layer=Layer.DECLARED.value,
                confidence=Confidence.RESOLVED.value,
                value_type=candidate.value_type,
                from_candidate=int(candidate.rank),
                source=candidate.source,
            )
        }
    return out


def _validate_param_states(atlas: Atlas) -> None:
    for node in atlas.entities.values():
        if node.kind not in (NodeKind.PARAM.value, NodeKind.PPARAM.value):
            continue
        if node.state is None:
            raise ContractError(f"parameter {node.id} has no state")
        if node.state != ParamState.EFFECTIVE.value and node.effective:
            raise ContractError(
                f"parameter {node.id} has state {node.state} and an effective value, "
                "effective must be empty unless the state is effective"
            )


__all__ = [
    "ANY_PROFILE",
    "EDGE_DOMAINS",
    "EFFECTIVE_PRECEDENCE",
    "KEYED_EDGE_KINDS",
    "LAYER_ORDER",
    "NON_FINITE",
    "PARAM_STATE_PRIORITY",
    "PASSES_KEY",
    "SCHEMA_MINOR",
    "SCHEMA_VERSION",
    "SEVERITY_ORDER",
    "VOLATILE_KEYS",
    "Agreement",
    "Atlas",
    "AtlasBuilder",
    "Candidate",
    "Confidence",
    "Edge",
    "EdgeKind",
    "EdgeRef",
    "EffectiveValue",
    "Finding",
    "Layer",
    "LayerFact",
    "LayerStatus",
    "Mechanism",
    "NameSource",
    "Node",
    "NodeKind",
    "ParamState",
    "PassState",
    "Root",
    "Severity",
    "ShadowKind",
    "Source",
    "Unresolved",
    "Want",
    "canonical_value",
    "completeness",
    "dumps",
    "dumps_line",
    "incomplete_passes",
    "passes_of",
    "strip_volatile",
    "stronger_param_state",
    "unrepresentable",
]
