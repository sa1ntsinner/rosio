"""Node identity: one role content builder, one role digest, one node id.

Every collector that meets a `records.ResolvedNode` asks the same two questions. What
content distinguishes a node whose name cannot be its identity, and which id does that node
carry. Answering them twice is what split one unnamed node into two entities with two ids
and two halves of its edge set, so both answers live here and nowhere else.

Three rules the digest obeys, all of them from AGENTS.md section 3:

* Nothing machine specific ever reaches the digest. A path under a tracked root contributes
  its basename, a path under the temp directory contributes a fixed marker, and a ROS name
  is left alone.
* Nothing ordinal reaches it either. The content is the node's own attributes, so inserting
  a sibling adds one id and changes none.
* A record that already carries `role_content` keeps it. The launch walk builds that tuple
  where the configured roots are known, and recomputing it later with fewer roots would give
  one node two digests again.
"""

from __future__ import annotations

import tempfile
from collections.abc import Iterable, Sequence
from dataclasses import dataclass

from rosio import ids, model
from rosio.errors import ContractError
from rosio.resolve import records

UNSPECIFIED_NAME = "<node_name_unspecified>"
UNSPECIFIED_NAMESPACE = "<node_namespace_unspecified>"
"""`launch_ros.actions.Node.UNSPECIFIED_NODE_NAME` and `UNSPECIFIED_NODE_NAMESPACE`. They
appear verbatim in the expanded fields when the launch file stated neither, and `node_name`
returns the two concatenated. They are statements, not values: the namespace is the root,
the name comes from the executable, and neither string may ever reach an id."""

EXTERNAL_TOKEN = "external_file"
"""Stands in for a file no tracked root holds. The merged parameters file a real launcher
hands to launch is a random name, so hashing it makes one node two nodes on the second run."""

EXTERNAL_KINDS = frozenset({"external_file"})
"""Parameter source kinds the walk that met the file already called external. The walk knows
which files it wrote and which root holds them, so its statement decides, and no later reader
re derives the answer from the path."""

TEMP_ROOTS = ("/tmp", "/usr/tmp", "/var/tmp")
"""The temp directories every process on a POSIX box shares, whatever `TMPDIR` any one of
them was started with. The parent and the launch child have to agree about one path: a
launch file that hardcodes `dir='/tmp'` writes there while the orchestrator's `TMPDIR` points
somewhere else, and a decision taken from the asking process alone put that random name into
a launch entity id, a label and an unresolved digest."""

UNKNOWN_SEGMENT = "@unknown"
"""Stand in for an id segment the walk could not fill, used only inside a synthetic id."""

ROOT = "/"

RANDOMISED_EXECUTABLES = frozenset(
    {
        ("tf2_ros", "static_transform_publisher"),
        ("rviz2", "rviz2"),
    }
)
"""Identity guard for the families that append or randomise a runtime suffix. These are
exactly the nodes that own the frame layer, so joining them on a name fabricates TF edges.
`known_names` may carry the same statement, and either source is enough."""

USABLE_NAME_SOURCES = (
    model.NameSource.LITERAL.value,
    model.NameSource.ARGV.value,
    model.NameSource.EXECUTABLE_DEFAULT.value,
)
"""Name sources a name may be joined on. `runtime_randomised` and `unknown` are absent on
purpose: those names are not identity."""

FRAME_FLAGS = ("--frame-id", "--child-frame-id")

TEMP_TOKEN = "@temp"
OUTSIDE_TOKEN = "@outside"
"""Stand ins for a path that lies under no configured root. The basename is kept and the
host layout is not, because a path that reaches an id or a compared key must not depend on
where the workspace is mounted."""


@dataclass(frozen=True)
class Identity:
    """The identity decision for one resolved node, with the reason it was made."""

    node_id: str
    name_source: str
    fqn: str | None
    name: str | None
    role: str
    known_name: str | None = None
    namespace: str = ""
    namespace_stated: bool = False


# ---------------------------------------------------------------------------
# role content


def temp_roots() -> tuple[str, ...]:
    """Every root a writer on this box could have put a temp file under, sorted.

    A union, so it is a superset in every run and two runs cannot disagree about a path one
    of them wrote.
    """
    found = {tempfile.gettempdir().rstrip("/"), *TEMP_ROOTS}
    return tuple(sorted(root for root in found if root))


def under_temp(path: str, roots: Sequence[str] = ()) -> bool:
    """Whether a path is a file some writer dropped in a temp directory.

    A path inside a configured root is the workspace's own file wherever that root is
    mounted: a workspace checked out under `/tmp` is still a workspace, and its yaml is
    still provenance.
    """
    value = str(path)
    if any(under(value, str(root)) for root in roots):
        return False
    return any(under(value, root) for root in temp_roots())


def under(path: str, base: str) -> bool:
    return path == base or path.startswith(str(base).rstrip("/") + "/")


def file_token(path: str, roots: Sequence[str] = ()) -> str:
    """A parameters file as identity content, or the external marker.

    Relative to the root holding it, so `left/config.yaml` and `right/config.yaml` stay two
    nodes. A path under no known root falls back to its basename, which is what a mount
    point cannot change.
    """
    value = str(path)
    if under_temp(value, roots):
        return EXTERNAL_TOKEN
    for root in sorted((str(root) for root in roots), key=len, reverse=True):
        if under(value, root) and value != root:
            return value[len(root.rstrip("/")) + 1 :]
    return value.rsplit("/", 1)[-1]


def relative_path(path: str, roots: Sequence[str] = ()) -> str:
    """A path as compared data may carry it: relative to the root that holds it.

    The absolute path stays recoverable from the roots the run record lists, and nothing
    that depends on where the workspace is mounted enters an id or a predicate key.
    """
    value = str(path)
    if under_temp(value, roots):
        return f"{TEMP_TOKEN}/{value.rsplit('/', 1)[-1]}"
    for root in sorted((str(root) for root in roots), key=len, reverse=True):
        if under(value, root):
            return value[len(root.rstrip("/")) + 1 :] or value.rsplit("/", 1)[-1]
    return f"{OUTSIDE_TOKEN}/{value.rsplit('/', 1)[-1]}"


def role_token(text: str, roots: Sequence[str] = ()) -> str:
    """One argv token, with a filesystem path reduced to something an id can carry.

    A ROS name also starts with a slash, so only a path under a tracked root or a last
    segment carrying a suffix is treated as a file.
    """
    value = str(text)
    if not value.startswith("/"):
        return value
    if under_temp(value, roots):
        return EXTERNAL_TOKEN
    if any(under(value, str(root)) for root in roots):
        return file_token(value, roots)
    name = value.rsplit("/", 1)[-1]
    return name if "." in name else value


def param_token(item: records.ParamSource, roots: Sequence[str] = ()) -> str:
    """One parameter input as identity content, on the walk's own statement about it.

    `kind` was decided where the file was met, against the roots that walk was given, so it
    is read here instead of asking a path prefix a second time in another process.
    """
    if item.kind in EXTERNAL_KINDS:
        return f"file:{EXTERNAL_TOKEN}"
    if item.path:
        return f"file:{file_token(item.path, roots)}"
    if item.source_path:
        return f"source:{file_token(item.source_path, roots)}"
    return ""


def role_content(
    argv: Iterable[str],
    remaps: Iterable[records.Remap],
    params: Iterable[records.ParamSource],
    roots: Sequence[str] = (),
) -> tuple[str, ...]:
    """Distinguishing content of a node whose name cannot be its identity.

    An inline value goes through the same path rule as an argv token, because a launch file
    that hands a node the merged temp file by name would otherwise put a random path back
    into the digest.
    """
    out = [f"argv:{role_token(item, roots)}" for item in argv]
    out.extend(f"remap:{item.frm}={item.to}" for item in remaps)
    for item in params:
        token = param_token(item, roots)
        if token:
            out.append(token)
            continue
        out.extend(f"set:{key}={_value_token(value, roots)}" for key, value in item.values)
    return tuple(out)


def _value_token(value: object, roots: Sequence[str]) -> str:
    """An inline value with its type, so `1`, `1.0`, `True` and `'1'` are four settings.

    A string carries no prefix, which keeps every id minted before types were told apart.
    """
    if isinstance(value, str):
        return role_token(value, roots)
    return f"{type(value).__name__}:{role_token(str(value), roots)}"


def content_of(node: records.ResolvedNode, roots: Sequence[str] = ()) -> tuple[str, ...]:
    """The content a node's digest is taken over.

    The record wins when it carries one: the walk built it where the roots were known.
    """
    if node.role_content:
        return tuple(node.role_content)
    return role_content(node.argv, node.remaps, node.params, roots)


def frame_pair(argv: Sequence[str]) -> tuple[str, str]:
    flags = dict.fromkeys(FRAME_FLAGS, "")
    for index, word in enumerate(argv):
        if word in flags and index + 1 < len(argv):
            flags[word] = argv[index + 1]
    return flags[FRAME_FLAGS[0]], flags[FRAME_FLAGS[1]]


def role_digest(node: records.ResolvedNode, roots: Sequence[str] = ()) -> str:
    """Role digest over the node's own distinguishing content.

    A randomised name family joins on this and never on the name, so everything that makes
    one static publisher different from its sibling is in here: the argv, the frame pair
    inside it, the remaps, the ordered parameter inputs and the namespace launch stated.
    """
    parent, child = frame_pair(tuple(node.argv))
    return ids.role_digest(
        child_frame=child,
        container=node.container or "",
        content=list(content_of(node, roots)),
        executable=node.executable,
        form=node.form,
        name=stated_name(node)[0] or "",
        namespace=role_namespace(node),
        parent_frame=parent,
        pkg=node.pkg,
        plugin_class=node.plugin_class or "",
    )


def role_namespace(node: records.ResolvedNode) -> str:
    """The namespace as the digest sees it. The sentinel is not a namespace."""
    text = node.namespace.strip()
    return "" if text == UNSPECIFIED_NAMESPACE else text


# ---------------------------------------------------------------------------
# names


def stated_name(node: records.ResolvedNode) -> tuple[str | None, bool]:
    """The name launch stated, with the sentinels stripped.

    `node_name` on a launch_ros Node returns the namespace sentinel joined to the name, so a
    record built from that property carries `<node_namespace_unspecified>/slam_toolbox`. The
    tail is a real stated name and the head is a statement that the namespace is the root,
    and neither the whole string nor the sentinel alone may become an id.
    """
    text = (node.name or "").strip()
    if not text or text == UNSPECIFIED_NAME:
        return None, False
    head, sep, tail = text.partition("/")
    if head == UNSPECIFIED_NAMESPACE and sep:
        return (tail.strip() or None), True
    if UNSPECIFIED_NAME in text or UNSPECIFIED_NAMESPACE in text:
        return None, False
    return text, False


def stated_namespace(node: records.ResolvedNode, executed: bool = True) -> tuple[str, bool]:
    """(namespace, whether launch stated one).

    An unstated namespace is not unknown on an executed walk: launch_ros wrote its
    UNSPECIFIED_NODE_NAMESPACE sentinel into the expanded field, so the node runs at the
    root. A walk that was not executed says nothing, so the namespace stays unknown there
    and the id stays synthetic.
    """
    text = node.namespace.strip()
    if node.attrs.get("namespace_sentinel"):
        return (ids.normalise_namespace(text) if text else ROOT), False
    if text and text != UNSPECIFIED_NAMESPACE:
        try:
            return ids.normalise_namespace(text), True
        except ContractError:
            return (ROOT if executed else ""), False
    return (ROOT if executed else ""), False


def known_table():
    """The committed third party name table, or None when it is not installed.

    An absent module is not an error, it only means fewer names resolve and more nodes carry
    a synthetic id.
    """
    try:
        from rosio.resolve import known_names
    except ImportError:
        return None
    return known_names


def known_name(pkg: str, executable: str) -> str | None:
    """The name the table is willing to join on, which is never a randomised or an
    unverified row."""
    table = known_table()
    if table is None or not pkg or not executable:
        return None
    return table.name_for(pkg, executable)


def randomised(node: records.ResolvedNode, name: str | None = None) -> bool:
    """A randomised family only randomises when launch left the name to it. A stated name is
    the runtime name, so a named static transform publisher joins like anything else.

    `name` is the name with the launch_ros sentinels stripped, which is the only name that
    counts: the sentinel says launch stated nothing.
    """
    stated = name if name is not None else stated_name(node)[0]
    if stated and node.name_source != model.NameSource.EXECUTABLE_DEFAULT.value:
        return node.name_source == model.NameSource.RUNTIME_RANDOMISED.value
    if (node.pkg, node.executable) in RANDOMISED_EXECUTABLES:
        return True
    if node.name_source == model.NameSource.RUNTIME_RANDOMISED.value:
        return True
    table = known_table()
    if table is None:
        return False
    return bool(table.is_randomised(node.pkg, node.executable))


# ---------------------------------------------------------------------------
# the decision


def synthetic_id(node: records.ResolvedNode, role: str, namespace: str = "") -> str:
    executable = node.executable or node.plugin_class or UNKNOWN_SEGMENT
    return ids.build_node_synthetic(
        node.pkg or UNKNOWN_SEGMENT,
        executable,
        namespace or ROOT,
        role,
    )


def identify(
    node: records.ResolvedNode,
    executed: bool = True,
    roots: Sequence[str] = (),
) -> Identity:
    """The one identity decision, for every collector that meets a resolved node."""
    role = role_digest(node, roots)
    name, from_sentinel = stated_name(node)
    name_source = node.name_source or model.NameSource.UNKNOWN.value
    if name is None and node.name:
        name_source = model.NameSource.UNKNOWN.value
    namespace, stated = stated_namespace(node, executed or from_sentinel)
    known = None
    unknown = {field for field, _expr in node.command_blinds}
    if "name" in unknown:
        name_source = model.NameSource.UNKNOWN.value
    if "namespace" in unknown:
        namespace, stated = "", False
    if not name and "name" not in unknown:
        known = known_name(node.pkg, node.executable)
        if known:
            name = known
            name_source = model.NameSource.EXECUTABLE_DEFAULT.value
    if randomised(node, name):
        return Identity(
            node_id=synthetic_id(node, role, namespace),
            name_source=model.NameSource.RUNTIME_RANDOMISED.value,
            fqn=None,
            name=name,
            role=role,
            known_name=known,
            namespace=namespace,
            namespace_stated=stated,
        )
    if name and namespace and name_source in USABLE_NAME_SOURCES:
        try:
            fqn = ids.normalise_ros_name(name, namespace)
        except ContractError:
            fqn = None
        if fqn:
            return Identity(
                node_id=ids.build_node(fqn),
                name_source=name_source,
                fqn=fqn,
                name=ids.split_ros_name(fqn)[1],
                role=role,
                known_name=known,
                namespace=namespace,
                namespace_stated=stated,
            )
    return Identity(
        node_id=synthetic_id(node, role, namespace),
        name_source=name_source,
        fqn=None,
        name=name,
        role=role,
        known_name=known,
        namespace=namespace,
        namespace_stated=stated,
    )


def node_id(node: records.ResolvedNode, executed: bool = True) -> str:
    """The id of one resolved node. The whole decision is `identify`."""
    return identify(node, executed).node_id


__all__ = [
    "EXTERNAL_KINDS",
    "EXTERNAL_TOKEN",
    "RANDOMISED_EXECUTABLES",
    "ROOT",
    "TEMP_ROOTS",
    "UNKNOWN_SEGMENT",
    "UNSPECIFIED_NAME",
    "UNSPECIFIED_NAMESPACE",
    "USABLE_NAME_SOURCES",
    "Identity",
    "content_of",
    "file_token",
    "frame_pair",
    "identify",
    "known_name",
    "known_table",
    "node_id",
    "param_token",
    "randomised",
    "relative_path",
    "role_content",
    "role_digest",
    "role_namespace",
    "role_token",
    "stated_name",
    "stated_namespace",
    "synthetic_id",
    "temp_roots",
    "under",
    "under_temp",
]
