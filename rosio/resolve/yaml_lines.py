"""A yaml loader that keeps a line number for every key path.

Provenance on a parameter has to cite the line a reader can delete, so the loader keeps the
compose tree marks next to the constructed values. Locators are key paths and never
ordinals: a sequence item has no stable handle, and a wildcard key owns one parameter per
node, so the key itself is the citable thing.

The same module decides whether a document is a ros parameter file at all. Detection is
shape based, a `ros__parameters` key somewhere below a mapping root, because on the
reference stack three flat driver configs read like parameter files and carry no node key,
while a map descriptor, a doc builder list and a name mapping table are yaml and are not
parameter files.
"""

from __future__ import annotations

import re
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

import yaml

from rosio import ids
from rosio.errors import CollectorError

PARAM_KEY = "ros__parameters"


class _Loader(yaml.SafeLoader):
    """SafeLoader typing the scalars a parameters file has, the way rcl types them.

    rcl does not read yaml 1.1, it reads its own scalar set, and three differences reach a
    real node. Each was measured on humble with one process per value, a parameters file
    handed to `--ros-args --params-file` and the type read back off `Node.get_parameter`.

    * `5e3` with no dot is a double at the node and a string in pyyaml. The reference stack
      has an incident comment about exactly that value, so the string would be a phantom
      type drift on a real tuning number.
    * `y`, `Y`, `n` and `N` are booleans at the node and strings in pyyaml, singly and
      inside a sequence, where `[y, N]` is a bool array.
    * `null`, `Null`, `NULL` and `~` are the strings `null`, `Null`, `NULL` and `~` at the
      node, because rcl has no null spelling. An empty value is the one null left: rcl
      refuses to start under it, so there is no node value to agree with.

    A quoted scalar is unaffected in both, which the same measurement confirmed.
    """


_DOTLESS_EXPONENT = re.compile(r"^[-+]?[0-9]+[eE][-+]?[0-9]+$")
_LETTER_BOOL = re.compile(r"^[yYnN]$")
_EMPTY = re.compile(r"^$")

_Loader.add_implicit_resolver("tag:yaml.org,2002:float", _DOTLESS_EXPONENT, list("-+0123456789"))
_Loader.add_implicit_resolver("tag:yaml.org,2002:bool", _LETTER_BOOL, list("yYnN"))
_Loader.bool_values = dict(yaml.SafeLoader.bool_values, y=True, n=False)

# An added resolver only ever loses to an inherited one, so the null spellings rcl does not
# read are dropped before the empty scalar is put back.
for _rules in _Loader.yaml_implicit_resolvers.values():
    _rules[:] = [rule for rule in _rules if rule[0] != "tag:yaml.org,2002:null"]
_Loader.add_implicit_resolver("tag:yaml.org,2002:null", _EMPTY, [""])

MAX_BYTES = 4 * 1024 * 1024

REFUSE_NOT_YAML = "not readable as yaml"
REFUSE_EMPTY = "empty document"
REFUSE_ROOT_LIST = "the document root is a list"
REFUSE_ROOT_SCALAR = "the document root is a scalar"
REFUSE_NO_PARAM_KEY = "no ros__parameters key anywhere below the mapping root"


@dataclass(frozen=True)
class Document:
    """One loaded yaml file: the values, plus the line of every key path."""

    data: Any
    lines: dict[tuple[str, ...], int] = field(default_factory=dict)
    path: str = ""

    def line(self, key_path: Sequence[str]) -> int | None:
        return self.lines.get(tuple(str(part) for part in key_path))

    def locator(self, key_path: Sequence[str]) -> str:
        return ".".join(str(part) for part in key_path)

    def is_parameter_file(self) -> bool:
        return not refuse_reason(self.data)


@dataclass(frozen=True)
class ParamBlock:
    """One `ros__parameters` mapping and the node selector that owns it.

    `selector` is the yaml key path above `ros__parameters` joined with a slash, so the
    three live shapes (a node name, a `/**` wildcard, a two level costmap key) all become
    one string that `ids.param_key_matches` can answer.
    """

    selector: str
    key_path: tuple[str, ...]
    values: Mapping[str, Any]
    order: int = 0
    """Position in the document. Inside one file the last matching key wins, verified by
    running a node against a file that carries both a `/**` key and an exact node key, so
    document order is precedence and specificity is not."""


@dataclass(frozen=True)
class Leaf:
    """One parameter inside a block: the dotted name, the value and the raw key path.

    `order` is the position of the key in the block, in document order. A dotted key and a
    nested mapping fold to one parameter name, so without it the two spellings of one
    parameter cannot be told apart and the later one stops winning.
    """

    name: str
    value: Any
    key_path: tuple[str, ...]
    order: int = 0


def _said(exc: BaseException) -> str:
    """What the parser said, at the line and column a reader opens, without its own name
    for the input."""
    mark = getattr(exc, "problem_mark", None)
    problem = getattr(exc, "problem", None)
    if mark is None or not problem:
        return str(exc)
    return f"line {mark.line + 1}, column {mark.column + 1}: {problem}"


def load(text: str, *, path: str = "") -> Document:
    """Load one yaml document, recording a line for every key path.

    A yaml file in a workspace is hostile input. A recursive anchor raises something other
    than a YAMLError and a mapping key that is itself a list is refused here, and every
    refusal leaves as a `CollectorError`, because anything else empties the whole layer
    instead of one file.
    """
    loader = _Loader(text)
    try:
        node = loader.get_single_node()
        lines: dict[tuple[str, ...], int] = {}
        data = None if node is None else _build(node, loader, (), lines)
    except (CollectorError, yaml.YAMLError, RecursionError, TypeError, ValueError) as exc:
        raise CollectorError(f"{path or 'yaml'} is not readable as yaml: {_said(exc)}") from exc
    finally:
        loader.dispose()
    return Document(data=data, lines=lines, path=path)


def load_file(path: str) -> Document:
    try:
        with Path(path).open(encoding="utf-8") as handle:
            text = handle.read(MAX_BYTES + 1)
    except (OSError, UnicodeDecodeError) as exc:
        # A byte that is not utf-8 used to leave here as a UnicodeDecodeError, which no
        # caller catches, and one such byte ended the whole parameter pass for the profile.
        raise CollectorError(f"cannot read {path}: {exc}") from exc
    if len(text) > MAX_BYTES:
        raise CollectorError(f"{path} is larger than {MAX_BYTES} bytes")
    return load(text, path=path)


def refuse_reason(data: Any) -> str:
    """Empty when the document is a ros parameter file, else why it was refused."""
    if data is None:
        return REFUSE_EMPTY
    if isinstance(data, (list, tuple)):
        return REFUSE_ROOT_LIST
    if not isinstance(data, Mapping):
        return REFUSE_ROOT_SCALAR
    if not _has_param_key(data):
        return REFUSE_NO_PARAM_KEY
    return ""


def is_parameter_document(data: Any) -> bool:
    return not refuse_reason(data)


def param_blocks(doc: Document) -> list[ParamBlock]:
    """Every `ros__parameters` mapping in the document, in document order.

    A `ros__parameters` mapping is never walked for further blocks. A nested one, as the
    one profile writes under `smoother_server`, is what rcl sees: dotted parameters
    named `costmap.ros__parameters.*`, not a second node.
    """
    blocks: list[ParamBlock] = []
    if not isinstance(doc.data, Mapping):
        return blocks
    _collect_blocks(doc.data, (), blocks)
    return [replace(block, order=index) for index, block in enumerate(blocks)]


def leaves(values: Mapping[str, Any]) -> list[Leaf]:
    """Flatten one block into dotted parameters.

    A nested mapping and a dotted key mean the same parameter, so both fold through
    `ids.normalise_param_name` before anything compares them. Which of the two spellings a
    reader is holding is then invisible, so each leaf keeps its position in the document and
    the caller ranks on it.
    """
    out: list[Leaf] = []
    _collect_leaves(values, (), (), out)
    ordered = [replace(leaf, order=index) for index, leaf in enumerate(out)]
    return sorted(ordered, key=lambda leaf: (leaf.name, leaf.order))


def selector_of(key_path: Sequence[str]) -> str:
    """Node selector for the key path above a `ros__parameters` key."""
    parts = [str(part) for part in key_path if str(part).strip("/")]
    if not parts:
        return ""
    head, *rest = parts
    out = head.rstrip("/")
    for part in rest:
        out = out.rstrip("/") + "/" + part.strip("/")
    return out


def type_name(value: Any) -> str:
    """Value type as the report spells it. A bool is never an int here."""
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int):
        return "int"
    if isinstance(value, float):
        return "float"
    if isinstance(value, str):
        return "str"
    if isinstance(value, Mapping):
        return "dict"
    if isinstance(value, (list, tuple)):
        return "list"
    return type(value).__name__


# ---------------------------------------------------------------------------
# internals


def _build(
    node: yaml.Node,
    loader: _Loader,
    prefix: tuple[str, ...],
    lines: dict[tuple[str, ...], int],
) -> Any:
    if isinstance(node, yaml.MappingNode):
        out: dict[Any, Any] = {}
        for key_node, value_node in node.value:
            text = _key_node_text(key_node)
            path = (*prefix, text)
            # A key spelled twice keeps its last value, so the last line is the citable one.
            # Two mappings under one key are one mapping to rcl, the later key winning inside
            # it: measured on jazzy with two `/**` keys in one parameters file.
            lines[path] = key_node.start_mark.line + 1
            out[text] = _merged(out.get(text), _build(value_node, loader, path, lines))
        return out
    return loader.construct_object(node, deep=True)


def _merged(held: Any, value: Any) -> Any:
    if not isinstance(held, dict) or not isinstance(value, dict):
        return value
    out = dict(held)
    for key, item in value.items():
        out[key] = _merged(out.get(key), item)
    return out


def _key_node_text(node: yaml.Node) -> str:
    """A mapping key is a name, so it is read as text and never typed.

    rcl reads a key as the name of a node or of a parameter, and the reference stack spells
    an initial pose `x:` and `y:`. Typing that key would rename the parameter to `true`. A
    key that is a list or a mapping is not a name at all, and refusing the file is the same
    answer the unhashable key used to produce.
    """
    if isinstance(node, yaml.ScalarNode):
        return node.value
    raise CollectorError(f"a mapping key is a {node.id}, not a name")


def _key_text(key: Any) -> str:
    if isinstance(key, str):
        return key
    if isinstance(key, bool):
        return "true" if key else "false"
    if key is None:
        return "null"
    return str(key)


def _has_param_key(data: Mapping[str, Any]) -> bool:
    for key, value in data.items():
        if _key_text(key) == PARAM_KEY and isinstance(value, Mapping):
            return True
        if isinstance(value, Mapping) and _has_param_key(value):
            return True
    return False


def _collect_blocks(
    data: Mapping[str, Any],
    prefix: tuple[str, ...],
    blocks: list[ParamBlock],
) -> None:
    for key, value in data.items():
        text = _key_text(key)
        path = (*prefix, text)
        if not isinstance(value, Mapping):
            continue
        if text == PARAM_KEY:
            if prefix:
                blocks.append(ParamBlock(selector=selector_of(prefix), key_path=path, values=value))
            continue
        _collect_blocks(value, path, blocks)


def _collect_leaves(
    values: Mapping[str, Any],
    name_parts: tuple[str, ...],
    key_path: tuple[str, ...],
    out: list[Leaf],
) -> None:
    for key, value in values.items():
        text = _key_text(key)
        parts = (*name_parts, text)
        path = (*key_path, text)
        if isinstance(value, Mapping) and value:
            _collect_leaves(value, parts, path, out)
            continue
        out.append(Leaf(name=ids.normalise_param_name(parts), value=value, key_path=path))


__all__ = [
    "MAX_BYTES",
    "PARAM_KEY",
    "REFUSE_EMPTY",
    "REFUSE_NOT_YAML",
    "REFUSE_NO_PARAM_KEY",
    "REFUSE_ROOT_LIST",
    "REFUSE_ROOT_SCALAR",
    "Document",
    "Leaf",
    "ParamBlock",
    "is_parameter_document",
    "leaves",
    "load",
    "load_file",
    "param_blocks",
    "refuse_reason",
    "selector_of",
    "type_name",
]
