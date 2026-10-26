"""Canonical rosio ids: build, parse, normalise. Pure functions, no state.

One grammar, no exceptions:

    id       = kind ":" body
    kind     = [a-z][a-z0-9_]*        recoverable from the text before the first colon
    body     = segment ("|" segment)* ordered, kind specific, count checked
    segment  = printable ASCII, with "%", "|" and "~" always percent escaped

"|" is the single reserved separator inside a body. It is escaped as %7C wherever
it occurs in a name, so splitting a body on a raw "|" is always correct. "%" is
escaped as %25, and "~" as %7E so that only the synthetic marker below can ever be
a raw "~". Anything outside printable ASCII is percent escaped byte by byte, so an
id is always typeable and greppable.

A body whose first character is "~" carries a synthetic identity: the runtime name
of the entity is either not known yet or known to be randomised at runtime (every
static_transform_publisher, an unnamed rviz2). Synthetic ids have their own segment
count and their own accessors. They never appear in human facing prose.

Two rules the id forms exist to enforce:

  * no id embeds another id's kind prefix, except an edge endpoint and a finding
    subject, which are complete entity references by definition. A parameter is
    param:<owner node body>|<dotted name>, never param:node:...
  * no id contains an ordinal, a line number, an absolute path, a temp path or a
    clock. Repeated things are separated by a digest over their own distinguishing
    content, so inserting a sibling adds one id and changes none.
"""

from __future__ import annotations

import hashlib
import re
from typing import NamedTuple

from rosio.errors import ContractError

KIND_SEP = ":"
SEP = "|"
SYNTHETIC = "~"
REPO_PKG = "@repo"
"""Pseudo package for files that belong to no ament package."""

DIGEST_LENGTH = 8

_KIND_RE = re.compile(r"^[a-z][a-z0-9_]*$")
_TOKEN_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_STEP_RE = re.compile(r"^([A-Za-z_][A-Za-z0-9_]*)\((.*)\)$")
_ORDINAL_RE = re.compile(r"^\d+$")

_SAFE = frozenset(chr(code) for code in range(0x21, 0x7F)) - {"%", SEP, SYNTHETIC}

# Segment counts per kind, (minimum, maximum), maximum None means open ended.
SEGMENTS: dict[str, tuple[int, int | None]] = {
    "pkg": (1, 1),
    "file": (2, 2),
    "sym": (3, 3),
    "exe": (2, 2),
    "launchfile": (2, 2),
    "launchent": (3, None),
    "profile": (1, 1),
    "node": (1, 1),
    "topic": (1, 1),
    "service": (1, 1),
    "action": (1, 1),
    "param": (2, 2),
    "pparam": (3, 3),
    "frame": (1, 1),
    "type": (3, 3),
    "plugin": (2, 2),
    "pinst": (2, 2),
    "run": (3, 3),
    "bag": (1, 1),
    "bagtopic": (2, 2),
    "edge": (4, 4),
    "find": (3, 3),
    "unres": (1, 1),
}

# Segment counts for the synthetic form of the kinds that have one.
SYNTHETIC_SEGMENTS: dict[str, tuple[int, int | None]] = {
    "node": (4, 4),
    "param": (5, 5),
    "pinst": (5, 5),
}

ENTITY_KINDS = frozenset(SEGMENTS) - {"edge", "find", "unres"}
"""Kinds that live in atlas.entities."""

KINDS = frozenset(SEGMENTS)


# ---------------------------------------------------------------------------
# escaping


def escape_segment(text: str) -> str:
    """Percent escape one segment. Everything that is not printable ASCII, plus % and |."""
    out: list[str] = []
    for char in text:
        if char in _SAFE:
            out.append(char)
        else:
            out.extend(f"%{byte:02X}" for byte in char.encode("utf-8"))
    return "".join(out)


def unescape_segment(text: str) -> str:
    """Inverse of escape_segment."""
    raw = bytearray()
    index = 0
    length = len(text)
    while index < length:
        char = text[index]
        if char == "%":
            hexits = text[index + 1 : index + 3]
            if len(hexits) != 2:
                raise ContractError(f"truncated percent escape in id segment: {text!r}")
            try:
                raw.append(int(hexits, 16))
            except ValueError as exc:
                raise ContractError(f"bad percent escape in id segment: {text!r}") from exc
            index += 3
        else:
            raw.extend(char.encode("utf-8"))
            index += 1
    return raw.decode("utf-8")


# ---------------------------------------------------------------------------
# digests, the only allowed disambiguator


def digest(*parts: object, length: int = DIGEST_LENGTH) -> str:
    """Stable short digest over an ordered list of parts, every part framed by its length.

    A separator alone is not a frame: joining with `\\x00` made digest('a', 'b') and
    digest('a\\x00b') one value, so a single part carrying the separator answered for two
    parts. A command line value can hold a `\\x00` through an escape and reach here, so the
    boundary has to survive a member that spells it. Length framing is unambiguous for every
    input, which no reserved character is.
    """
    text = ["" if part is None else str(part) for part in parts]
    payload = "".join(f"{len(item)}\x00{item}" for item in text)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()[:length]


def digest_fields(fields: dict[str, object], length: int = DIGEST_LENGTH) -> str:
    """Stable short digest over a mapping, key sorted so caller order cannot matter.

    A sequence states its length and every member is escaped the way the id grammar escapes
    its own separator, so the member boundaries survive a member that contains the
    separator. Joining members with a bare comma made ('sensor,--mode', 'fast') and
    ('sensor', '--mode,fast') one digest, which is one identity for two nodes.
    """
    ordered: list[object] = []
    for key in sorted(fields):
        ordered.append(key)
        value = fields[key]
        if isinstance(value, (list, tuple)):
            ordered.append(f"seq{len(value)}")
            ordered.extend(escape_segment(str(item)) for item in value)
        else:
            ordered.append(value)
    return digest(*ordered, length=length)


def role_digest(**fields: object) -> str:
    """Identity digest for a node whose name cannot be identity (argv, remaps, frames)."""
    return digest_fields(fields)


def edge_key(**fields: object) -> str:
    """Mandatory disambiguator for a repeatable edge, over the producing site."""
    return digest_fields(fields)


# ---------------------------------------------------------------------------
# generic build and parse


class ParsedId(NamedTuple):
    kind: str
    segments: tuple[str, ...]
    synthetic: bool


def build(kind: str, *segments: str, synthetic: bool = False) -> str:
    """Build any id from its kind and its ordered segments."""
    if not _KIND_RE.match(kind):
        raise ContractError(f"id kind must match [a-z][a-z0-9_]*, got {kind!r}")
    if not segments:
        raise ContractError(f"id of kind {kind} needs at least one segment")
    parts = [escape_segment(segment) for segment in segments]
    for index, segment in enumerate(segments):
        if segment == "":
            raise ContractError(f"empty segment {index} in {kind} id")
    _check_segments(kind, len(parts), synthetic)
    body = SEP.join(parts)
    if synthetic:
        body = SYNTHETIC + body
    return f"{kind}{KIND_SEP}{body}"


def parse(entity_id: str) -> ParsedId:
    """Parse any id. Unknown kinds are accepted so a newer minor schema still loads."""
    if not isinstance(entity_id, str) or KIND_SEP not in entity_id:
        raise ContractError(f"not an id: {entity_id!r}")
    kind, body = entity_id.split(KIND_SEP, 1)
    if not _KIND_RE.match(kind):
        raise ContractError(f"id kind must match [a-z][a-z0-9_]*, got {kind!r}")
    if body == "":
        raise ContractError(f"id {entity_id!r} has an empty body")
    synthetic = body.startswith(SYNTHETIC)
    if synthetic:
        body = body[1:]
    segments = tuple(unescape_segment(part) for part in body.split(SEP))
    if kind in SEGMENTS:
        _check_segments(kind, len(segments), synthetic)
    return ParsedId(kind=kind, segments=segments, synthetic=synthetic)


def _check_segments(kind: str, count: int, synthetic: bool) -> None:
    table = SYNTHETIC_SEGMENTS if synthetic else SEGMENTS
    if kind not in table:
        if synthetic and kind in SEGMENTS:
            raise ContractError(f"kind {kind} has no synthetic form")
        return
    low, high = table[kind]
    if count < low or (high is not None and count > high):
        want = f"{low}" if low == high else f"{low}..{high or 'n'}"
        raise ContractError(f"{kind} id needs {want} segments, got {count}")


def kind_of(entity_id: str) -> str:
    """Kind of an id, from the text before the first colon."""
    return parse(entity_id).kind


def is_synthetic(entity_id: str) -> bool:
    return parse(entity_id).synthetic


def is_entity(entity_id: str) -> bool:
    return kind_of(entity_id) in ENTITY_KINDS


def label_for(entity_id: str) -> str:
    """Short human label for an id. Never used as identity."""
    parsed = parse(entity_id)
    if parsed.kind == "node" and parsed.synthetic:
        package, executable, namespace, _role = parsed.segments
        return f"{package}/{executable}@{namespace}"
    if parsed.kind == "launchent":
        return parsed.segments[-1]
    return parsed.segments[-1]


# ---------------------------------------------------------------------------
# per kind builders and parsers


def build_pkg(name: str) -> str:
    return build("pkg", name)


def parse_pkg(entity_id: str) -> str:
    return _require(entity_id, "pkg").segments[0]


class FileId(NamedTuple):
    package: str
    relpath: str


def build_file(package: str, relpath: str) -> str:
    """A file, addressed package relative. Use REPO_PKG for files in no package."""
    return build("file", package, _norm_relpath(relpath))


def parse_file(entity_id: str) -> FileId:
    package, relpath = _require(entity_id, "file").segments
    return FileId(package=package, relpath=relpath)


class SymId(NamedTuple):
    package: str
    relpath: str
    qualname: str


def build_sym(package: str, relpath: str, qualname: str) -> str:
    """A function, method or class. The file stays in the id: C++ helpers in an
    anonymous namespace have no unique language qualified name across a package."""
    return build("sym", package, _norm_relpath(relpath), qualname)


def parse_sym(entity_id: str) -> SymId:
    package, relpath, qualname = _require(entity_id, "sym").segments
    return SymId(package=package, relpath=relpath, qualname=qualname)


class ExeId(NamedTuple):
    package: str
    name: str


def build_exe(package: str, name: str) -> str:
    """The launch executable= join key, from console_scripts, add_executable or the
    EXECUTABLE clause of rclcpp_components_register_node."""
    return build("exe", package, name)


def parse_exe(entity_id: str) -> ExeId:
    package, name = _require(entity_id, "exe").segments
    return ExeId(package=package, name=name)


def build_launchfile(package: str, relpath: str) -> str:
    return build("launchfile", package, _norm_relpath(relpath))


def parse_launchfile(entity_id: str) -> FileId:
    package, relpath = _require(entity_id, "launchfile").segments
    return FileId(package=package, relpath=relpath)


class LaunchStep(NamedTuple):
    action: str
    disc: str


class LaunchEntId(NamedTuple):
    package: str
    relpath: str
    steps: tuple[LaunchStep, ...]


def build_launchent(package: str, relpath: str, steps: tuple[LaunchStep, ...]) -> str:
    """One resolved launch action occurrence.

    Each step is (action kind, discriminator). The discriminator is the declared
    name when there is one, else the executable, else a digest over the action's own
    content. A bare ordinal is refused: it renumbers when a sibling is inserted,
    which is exactly the diff churn the id form exists to prevent. Python local
    variable names are also refused as a matter of contract, they do not survive
    generate_launch_description.
    """
    if not steps:
        raise ContractError("launch entity id needs at least one step")
    encoded: list[str] = []
    for step in steps:
        action, disc = LaunchStep(*step)
        if not _TOKEN_RE.match(action):
            raise ContractError(f"launch action kind must be an identifier, got {action!r}")
        if disc == "":
            raise ContractError(f"launch step {action} needs a discriminator")
        if _ORDINAL_RE.match(disc):
            raise ContractError(
                f"launch step {action} discriminator {disc!r} is an ordinal, "
                "use a digest over the action's own content"
            )
        encoded.append(f"{action}({disc})")
    return build("launchent", package, _norm_relpath(relpath), *encoded)


def parse_launchent(entity_id: str) -> LaunchEntId:
    segments = _require(entity_id, "launchent").segments
    package, relpath = segments[0], segments[1]
    steps: list[LaunchStep] = []
    for raw in segments[2:]:
        match = _STEP_RE.match(raw)
        if match is None:
            raise ContractError(f"bad launch step in id: {raw!r}")
        steps.append(LaunchStep(action=match.group(1), disc=match.group(2)))
    return LaunchEntId(package=package, relpath=relpath, steps=tuple(steps))


def build_profile(slug: str) -> str:
    """A named composition that actually runs: a launcher command, a sim profile,
    or a launch file plus its argument vector."""
    return build("profile", slug)


def parse_profile(entity_id: str) -> str:
    return _require(entity_id, "profile").segments[0]


class NodeId(NamedTuple):
    fqn: str | None
    package: str = ""
    executable: str = ""
    namespace: str = ""
    role: str = ""
    synthetic: bool = False


def build_node(fqn: str) -> str:
    """A node whose name is identity: a static literal in the source or a launch name=."""
    return build("node", normalise_ros_name(fqn))


def build_node_synthetic(package: str, executable: str, namespace: str, role: str) -> str:
    """A node whose name cannot be identity.

    Used for a launch Node with no name=, and for the randomised name families
    (static_transform_publisher appends a runtime suffix, an unnamed rviz2
    randomises), which must join on the role digest and never on the name.
    """
    if not role:
        raise ContractError("synthetic node id needs a role digest")
    return build(
        "node",
        package,
        executable,
        normalise_namespace(namespace),
        role,
        synthetic=True,
    )


def parse_node(entity_id: str) -> NodeId:
    parsed = _require(entity_id, "node")
    if parsed.synthetic:
        package, executable, namespace, role = parsed.segments
        return NodeId(
            fqn=None,
            package=package,
            executable=executable,
            namespace=namespace,
            role=role,
            synthetic=True,
        )
    return NodeId(fqn=parsed.segments[0])


def build_topic(name: str, namespace: str = "/", node_name: str | None = None) -> str:
    return build("topic", normalise_ros_name(name, namespace, node_name))


def parse_topic(entity_id: str) -> str:
    return _require(entity_id, "topic").segments[0]


def build_service(name: str, namespace: str = "/", node_name: str | None = None) -> str:
    return build("service", normalise_ros_name(name, namespace, node_name))


def parse_service(entity_id: str) -> str:
    return _require(entity_id, "service").segments[0]


def build_action(name: str, namespace: str = "/", node_name: str | None = None) -> str:
    return build("action", normalise_ros_name(name, namespace, node_name))


def parse_action(entity_id: str) -> str:
    return _require(entity_id, "action").segments[0]


class ParamId(NamedTuple):
    owner: str
    name: str


def build_param(owner_node_id: str, name: str) -> str:
    """A (node, dotted parameter name) pair.

    The owner contributes its body only, never its kind prefix, so the id stays one
    flat namespace. A synthetic owner is legal: the flagship parameter on this bench
    (FollowPath.plugin on controller_server) belongs to a node that no launch file
    names and whose source is not in the workspace.
    """
    parsed = _require(owner_node_id, "node")
    return build(
        "param",
        *parsed.segments,
        normalise_param_name(name),
        synthetic=parsed.synthetic,
    )


def parse_param(entity_id: str) -> ParamId:
    parsed = _require(entity_id, "param")
    owner_segments, name = parsed.segments[:-1], parsed.segments[-1]
    owner = build("node", *owner_segments, synthetic=parsed.synthetic)
    return ParamId(owner=owner, name=name)


class PluginParamId(NamedTuple):
    base_class: str
    lookup: str
    suffix: str


def build_pparam(base_class: str, lookup: str, suffix: str) -> str:
    """A parameter the code layer can only name as a suffix, because the prefix is a
    plugin instance name that lives in a yaml selector list."""
    return build("pparam", base_class, lookup, normalise_param_name(suffix))


def parse_pparam(entity_id: str) -> PluginParamId:
    base_class, lookup, suffix = _require(entity_id, "pparam").segments
    return PluginParamId(base_class=base_class, lookup=lookup, suffix=suffix)


def build_frame(frame_id: str) -> str:
    return build("frame", normalise_frame(frame_id))


def parse_frame(entity_id: str) -> str:
    return _require(entity_id, "frame").segments[0]


class TypeId(NamedTuple):
    package: str
    interface: str
    name: str


INTERFACE_KINDS = ("msg", "srv", "action")


def build_type(package: str, interface: str, name: str) -> str:
    if interface not in INTERFACE_KINDS:
        raise ContractError(f"interface kind must be one of {INTERFACE_KINDS}, got {interface!r}")
    return build("type", package, interface, name)


def parse_type(entity_id: str) -> TypeId:
    package, interface, name = _require(entity_id, "type").segments
    return TypeId(package=package, interface=interface, name=name)


class PluginId(NamedTuple):
    base_class: str
    lookup: str


def build_plugin(base_class: str, lookup: str) -> str:
    """A pluginlib or BehaviorTree registration. The lookup name falls back to the C++
    type when the plugin xml <class> has no name attribute."""
    return build("plugin", base_class, lookup)


def parse_plugin(entity_id: str) -> PluginId:
    base_class, lookup = _require(entity_id, "plugin").segments
    return PluginId(base_class=base_class, lookup=lookup)


class PluginInstanceId(NamedTuple):
    owner: str
    instance: str


def build_pinst(owner_node_id: str, instance: str) -> str:
    """The yaml slot that activates a plugin. Plugin identity gates which parameter
    sub keys of a block are live, which is what keeps phantom drift out of the report."""
    parsed = _require(owner_node_id, "node")
    return build("pinst", *parsed.segments, instance, synthetic=parsed.synthetic)


def parse_pinst(entity_id: str) -> PluginInstanceId:
    parsed = _require(entity_id, "pinst")
    owner = build("node", *parsed.segments[:-1], synthetic=parsed.synthetic)
    return PluginInstanceId(owner=owner, instance=parsed.segments[-1])


class RunId(NamedTuple):
    layer: str
    profile: str
    input_digest: str


def build_run(layer: str, profile_slug: str, input_digest: str) -> str:
    """One extraction pass. The input digest is part of the id so two machines that
    disagree only about which files exist cannot produce the same run id."""
    return build("run", layer, profile_slug or "-", input_digest)


def parse_run(entity_id: str) -> RunId:
    layer, profile, input_digest = _require(entity_id, "run").segments
    return RunId(layer=layer, profile=profile, input_digest=input_digest)


def build_bag(name: str) -> str:
    return build("bag", name)


def parse_bag(entity_id: str) -> str:
    return _require(entity_id, "bag").segments[0]


class BagTopicId(NamedTuple):
    bag: str
    topic: str


def build_bagtopic(bag_name: str, topic: str) -> str:
    return build("bagtopic", bag_name, normalise_ros_name(topic))


def parse_bagtopic(entity_id: str) -> BagTopicId:
    bag_name, topic = _require(entity_id, "bagtopic").segments
    return BagTopicId(bag=bag_name, topic=topic)


class EdgeId(NamedTuple):
    kind: str
    src: str
    dst: str
    key: str


def build_edge(kind: str, src: str, dst: str, key: str = "") -> str:
    """An edge handle.

    src and dst are complete entity ids, so their own separators are escaped. Read
    the src, kind and dst fields of the edge object rather than parsing the handle.
    """
    if not _TOKEN_RE.match(kind):
        raise ContractError(f"edge kind must be an identifier, got {kind!r}")
    return build("edge", kind, src, dst, key or "-")


def parse_edge(entity_id: str) -> EdgeId:
    kind, src, dst, key = _require(entity_id, "edge").segments
    return EdgeId(kind=kind, src=src, dst=dst, key="" if key == "-" else key)


class FindingId(NamedTuple):
    rule: str
    subject: str
    disc: str


def build_finding(rule: str, subject: str, disc: str = "") -> str:
    """A finding handle: rule plus subject plus a content derived discriminator.

    Message text never enters the id, so rewording a finding does not invalidate a
    waiver. The discriminator must come from sorted content, never from iteration
    order, or waivers break on every run.
    """
    if not rule:
        raise ContractError("finding id needs a rule")
    if _ORDINAL_RE.match(disc or "-"):
        raise ContractError(f"finding discriminator {disc!r} is an ordinal")
    return build("find", rule, subject, disc or "-")


def parse_finding(entity_id: str) -> FindingId:
    rule, subject, disc = _require(entity_id, "find").segments
    return FindingId(rule=rule, subject=subject, disc="" if disc == "-" else disc)


def unresolved_site(file: str = "", locator: str = "") -> str:
    """The producing site of a blind spot: a file id and a locator inside it.

    The line hint is deliberately absent. It is advisory, so a file gaining a line above the
    site would otherwise move the handle and split one blind spot into two.
    """
    return f"{file}#{locator}"


def build_unresolved(
    owner: str, expr: str, want: str, site: str = "", construction: str = ""
) -> str:
    """One unresolved expression site. The digest covers the site as well as the owner, the
    expression and what was wanted.

    Without the site, one plugin list writing the same unreadable expression at two entries
    collapses into a single handle and every evidence row but the first is dropped, which
    reads as one blind spot where there are two. Build the site with `unresolved_site`, so
    the owner, the expression and the site are spelled one way everywhere.

    An entry with no site keeps the handle it had before the site existed, so adding a site
    where one is known does not renumber the blind spots that have none. The construction
    context of a source blind spot follows the same rule: one expression read in two
    contexts is two blind spots, and an entry with no context keeps its handle.
    """
    parts: tuple[str, ...] = (owner, expr, want)
    if site or construction:
        parts += (site,)
    if construction:
        parts += (construction,)
    return build("unres", digest(*parts))


def parse_unresolved(entity_id: str) -> str:
    return _require(entity_id, "unres").segments[0]


def _require(entity_id: str, kind: str) -> ParsedId:
    parsed = parse(entity_id)
    if parsed.kind != kind:
        raise ContractError(f"expected a {kind} id, got {entity_id!r}")
    return parsed


def _norm_relpath(relpath: str) -> str:
    """Package relative path. Absolute paths and parent traversals are refused: an id
    that carries a machine path cannot survive a checkout move."""
    cleaned = relpath.strip().replace("\\", "/")
    while "//" in cleaned:
        cleaned = cleaned.replace("//", "/")
    while cleaned.startswith("./"):
        cleaned = cleaned[2:]
    if cleaned.startswith("/"):
        raise ContractError(f"file id path must be package relative, got {relpath!r}")
    if ".." in cleaned.split("/"):
        raise ContractError(f"file id path must not traverse upwards, got {relpath!r}")
    if not cleaned:
        raise ContractError("file id path is empty")
    return cleaned


# ---------------------------------------------------------------------------
# ROS name normalisation, one engine shared by every layer


def normalise_namespace(namespace: str | None) -> str:
    """Absolute namespace, always a leading slash, never a trailing one."""
    if namespace is None or namespace.strip() in ("", "/"):
        return "/"
    cleaned = namespace.strip()
    if not cleaned.startswith("/"):
        cleaned = "/" + cleaned
    while "//" in cleaned:
        cleaned = cleaned.replace("//", "/")
    return cleaned.rstrip("/") or "/"


def join_namespace(namespace: str, name: str) -> str:
    base = normalise_namespace(namespace)
    tail = name.strip().lstrip("/")
    if base == "/":
        return "/" + tail
    return f"{base}/{tail}"


def is_relative(name: str) -> bool:
    """True for a bare name like input or cmd_vel, which a remap or a namespace resolves."""
    cleaned = name.strip()
    return bool(cleaned) and not cleaned.startswith("/") and not cleaned.startswith("~")


def is_private(name: str) -> bool:
    return name.strip().startswith("~")


def is_valid_ros_name(name: str) -> bool:
    """True when normalise_ros_name will accept the name without a namespace or node."""
    try:
        normalise_ros_name(name, "/", "probe")
    except ContractError:
        return False
    return True


def normalise_ros_name(name: str, namespace: str = "/", node_name: str | None = None) -> str:
    """Fully qualified ROS name.

    The same engine resolves topic and service names from source literals, remap
    targets, and relative names that appear in parameter VALUES (node_names entries
    like ouster/os_driver, sensors.*.topic, cmd_vel_in_topic). Using one rule
    everywhere is what keeps node:ouster/os_driver and node:/ouster/os_driver from
    being two ids for one node.
    """
    if not isinstance(name, str):
        raise ContractError(f"ros name must be a string, got {name!r}")
    cleaned = name.strip()
    if not cleaned:
        raise ContractError("ros name is empty")
    if cleaned.startswith("~"):
        if not node_name:
            raise ContractError(f"private name {name!r} needs the owning node name")
        private_root = join_namespace(namespace, node_name)
        tail = cleaned[1:].lstrip("/")
        absolute = join_namespace(private_root, tail) if tail else private_root
    elif cleaned.startswith("/"):
        absolute = cleaned
    else:
        absolute = join_namespace(namespace, cleaned)
    while "//" in absolute:
        absolute = absolute.replace("//", "/")
    absolute = absolute.rstrip("/") or "/"
    for token in absolute.split("/")[1:]:
        if not _TOKEN_RE.match(token):
            raise ContractError(f"ros name {name!r} has an unusable token {token!r}")
    return absolute


def split_ros_name(fqn: str) -> tuple[str, str]:
    """(namespace, base name) for a fully qualified name."""
    absolute = normalise_ros_name(fqn)
    head, _, base = absolute.rpartition("/")
    return normalise_namespace(head), base


def normalise_frame(frame_id: str) -> str:
    """TF frame id, tf2 convention, no leading slash."""
    cleaned = frame_id.strip().lstrip("/")
    if not cleaned:
        raise ContractError("frame id is empty")
    if "/" in cleaned and not all(_TOKEN_RE.match(part) for part in cleaned.split("/")):
        raise ContractError(f"frame id {frame_id!r} has an unusable token")
    return cleaned


def normalise_param_name(name: object) -> str:
    """Dotted parameter name.

    A nested yaml dict and a dotted yaml key mean the same parameter, so both
    normalise to the same string before any id is formed or any comparison is made.

    An empty component is refused and never dropped. Dropping it folded `a..b`, `.a.b` and
    `a.b.` onto the one id `a.b`, so three spellings no source agrees on shared one
    parameter and whichever was read last owned its value. A name with a gap in it is not a
    name.
    """
    if isinstance(name, str):
        parts = name.split(".")
    elif isinstance(name, (list, tuple)):
        parts = []
        for item in name:
            parts.extend(str(item).split("."))
    else:
        raise ContractError(f"parameter name must be a string or a sequence, got {name!r}")
    if not parts or parts == [""]:
        raise ContractError(f"parameter name is empty: {name!r}")
    if "" in parts:
        raise ContractError(f"parameter name has an empty component: {name!r}")
    return ".".join(parts)


# ---------------------------------------------------------------------------
# node name sources, one precedence shared by every writer

NAME_SOURCE_ORDER = (
    "literal",
    "runtime_randomised",
    "argv",
    "executable_default",
    "observed",
    "unknown",
)
"""Why a node carries the name it carries, strongest first.

Two collectors reach the same node and each has an answer, so without a precedence the
field held whatever the last pass wrote and the atlas moved when the passes were reordered.
This ranks the answers to "why does this node carry this name", for one node id every
answer already agrees on. It is not the runtime rule for which name a process runs under:
measured on humble, rcl keeps the first `__node` rule it is handed, and `launch_ros`
appends its own after the arguments a launch file passed, so an argv assignment is the name
that runs. `resolve/ros_args` folds argv before an id is built, so by the time a name source
is ranked here the name is settled and this only decides which source is cited for it.

So: what a source states wins, because it is where the name is edited and where a reader
goes to change it. `runtime_randomised` is a statement of the same kind, a source refusing
to trust the name at all, and one reading of one run must never soften it. Then the readings
that can say where the name came from, argv and the committed executable table, then a live
system reporting the name it happens to run under, then no answer.

The members are the members of `model.NameSource`, spelled here because `ids` imports
nothing, and a test holds the two lists equal.
"""

_NAME_SOURCE_RANK = {source: rank for rank, source in enumerate(NAME_SOURCE_ORDER)}


def name_source_rank(source: str | None) -> int:
    """Where a name source sits in `NAME_SOURCE_ORDER`, 0 being the strongest.

    An unknown member ranks below every known one rather than raising, because AGENTS
    section 9a says a consumer tolerates an enum member it has not heard of.
    """
    if not source:
        return len(NAME_SOURCE_ORDER)
    return _NAME_SOURCE_RANK.get(str(source), len(NAME_SOURCE_ORDER))


def strongest_name_source(*sources: str | None) -> str | None:
    """The strongest of the name sources offered, or None when none was.

    Order independent by construction: two collectors writing in either order land on the
    same answer.
    """
    known = [source for source in sources if source]
    if not known:
        return None
    return min(known, key=lambda source: (name_source_rank(source), source))


def param_key_matches(key: str, node_fqn: str) -> bool:
    """Does a parameter file key select this node.

    Shapes in live use: an exact node name, /**, and /**/<node>. The wildcard means
    zero or more namespace levels and must also match the root namespace, so
    /**/ekf_local selects /ekf_local as well as /robot1/ekf_local.
    """
    fqn = normalise_ros_name(node_fqn)
    cleaned = key.strip()
    if cleaned in ("/**", "**", "/**/", "/*"):
        return True
    if cleaned.startswith("/**/"):
        suffix = cleaned[4:].strip("/")
        if not suffix:
            return True
        return fqn == "/" + suffix or fqn.endswith("/" + suffix)
    if not cleaned.startswith("/"):
        cleaned = "/" + cleaned
    return normalise_ros_name(cleaned) == fqn


__all__ = [
    "DIGEST_LENGTH",
    "ENTITY_KINDS",
    "INTERFACE_KINDS",
    "KINDS",
    "KIND_SEP",
    "NAME_SOURCE_ORDER",
    "REPO_PKG",
    "SEGMENTS",
    "SEP",
    "SYNTHETIC",
    "SYNTHETIC_SEGMENTS",
    "BagTopicId",
    "EdgeId",
    "ExeId",
    "FileId",
    "FindingId",
    "LaunchEntId",
    "LaunchStep",
    "NodeId",
    "ParamId",
    "ParsedId",
    "PluginId",
    "PluginInstanceId",
    "PluginParamId",
    "RunId",
    "SymId",
    "TypeId",
    "build",
    "build_action",
    "build_bag",
    "build_bagtopic",
    "build_edge",
    "build_exe",
    "build_file",
    "build_finding",
    "build_frame",
    "build_launchent",
    "build_launchfile",
    "build_node",
    "build_node_synthetic",
    "build_param",
    "build_pinst",
    "build_pkg",
    "build_plugin",
    "build_pparam",
    "build_profile",
    "build_run",
    "build_service",
    "build_sym",
    "build_topic",
    "build_type",
    "build_unresolved",
    "digest",
    "digest_fields",
    "edge_key",
    "escape_segment",
    "is_entity",
    "is_private",
    "is_relative",
    "is_synthetic",
    "is_valid_ros_name",
    "join_namespace",
    "kind_of",
    "label_for",
    "name_source_rank",
    "normalise_frame",
    "normalise_namespace",
    "normalise_param_name",
    "normalise_ros_name",
    "param_key_matches",
    "parse",
    "parse_action",
    "parse_bag",
    "parse_bagtopic",
    "parse_edge",
    "parse_exe",
    "parse_file",
    "parse_finding",
    "parse_frame",
    "parse_launchent",
    "parse_launchfile",
    "parse_node",
    "parse_param",
    "parse_pinst",
    "parse_pkg",
    "parse_plugin",
    "parse_pparam",
    "parse_profile",
    "parse_run",
    "parse_service",
    "parse_sym",
    "parse_topic",
    "parse_type",
    "parse_unresolved",
    "role_digest",
    "split_ros_name",
    "strongest_name_source",
    "unescape_segment",
    "unresolved_site",
]
