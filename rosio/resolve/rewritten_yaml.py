"""Recognition of the nav2_common parameter file wrappers, by duck typing only.

`nav2_common.launch.RewrittenYaml` materialises a NEW yaml file under /tmp at execute time,
usually wrapped in `launch_ros.descriptions.ParameterFile`. The path the node is handed
therefore exists in no repository, and provenance that points at it is worthless. This
module unwraps the pair and returns a `ParamSource` that terminates at the real source file
with its root key and its rewrite list, so the citable thing is the line a reader can edit.

Nothing here imports nav2_common or launch_ros. Both wrappers are recognised by their
attributes, in the name mangled spelling first and in the public spelling second: the public
`ParameterFile.param_file` returns the temp path once the object has been evaluated, and
`RewrittenYaml.perform` writes a new temp yaml, so describing has to stay off both.

Three shapes cost the whole provenance if they are not handled here, and all three appear on
the reference stack:

  * `normalize_parameters` hands a one element list of substitutions where the item is, so
    the descent has to go through it before anything is recognised
  * `source_file`, `root_key` and every rewrite value are lists of substitutions, so they are
    text only through the resolver callback the caller passes in
  * `root_key` is None when it was never given and a substitution that performs to an empty
    string when it was given and is empty, and those two are not the same file shape

Three behaviours of the real class are reproduced in `apply_rewrites`, all three verified on
the reference stack:

  1. only keys that already exist are rewritten, so a rewrite of a key the file never had
     is a silent no op
  2. a key is rewritten at every depth it occurs, matched on the bare name
  3. `root_key` wraps the whole document, so a non empty namespace reshapes every node key
"""

from __future__ import annotations

import copy
import os
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from typing import Any

from rosio.resolve import records

_SOURCE_ATTRS = ("_RewrittenYaml__source_file", "source_file")
_REWRITE_ATTRS = ("_RewrittenYaml__param_rewrites", "param_rewrites")
_ROOT_ATTRS = ("_RewrittenYaml__root_key", "root_key")
_CONVERT_ATTRS = ("_RewrittenYaml__convert_types", "convert_types")
_WRAPPED_ATTRS = ("_ParameterFile__param_file", "param_file")
_ALLOW_SUBSTS_ATTRS = ("_ParameterFile__allow_substs", "allow_substs")

Resolver = Callable[[Any], str | None]

REWRITTEN = "rewritten_file"
REPO_FILE = "repo_file"


@dataclass(frozen=True)
class Wrapper:
    """What one item of a `parameters=` list turned out to be.

    `root_key` is None when the object was built without one and a string, possibly empty,
    when it was given one. An empty root key wraps nothing, exactly like an absent one, but
    the two are different statements about the launch file and only one of them is a place a
    reader can edit.
    """

    kind: str
    source: Any = None
    rewrites: tuple[tuple[str, str], ...] = ()
    root_key: str | None = None
    convert_types: bool = False
    unresolved_rewrites: tuple[str, ...] = ()
    unresolved_root_key: bool = False
    wrapped_in_parameter_file: bool = False
    allow_substs: bool = False


def is_parameter_file(item: Any) -> bool:
    """A ParameterFile style wrapper, which carries the real item inside it."""
    return _first(_descend(item), _WRAPPED_ATTRS) is not None


def is_rewritten_yaml(item: Any) -> bool:
    """A RewrittenYaml style substitution, recognised by its own attributes."""
    target = _descend(item)
    return _first(target, _SOURCE_ATTRS) is not None and _first(target, _REWRITE_ATTRS) is not None


def unwrap(item: Any, *, resolve: Resolver | None = None) -> Wrapper:
    """Peel ParameterFile wrappers until the inner item is reached.

    A one element substitution list is the item, not a value of its own: launch_ros
    normalises everything that is not a dict into one, so stopping at the list is what makes
    a real `ParameterFile(RewrittenYaml(...))` unrecognisable and sends the caller on to
    perform it, which writes a temp yaml and loses the source file.
    """
    wrapped = False
    allow_substs = False
    current = _descend(item)
    seen: list[int] = []
    while is_parameter_file(current) and id(current) not in seen:
        seen.append(id(current))
        wrapped = True
        allow_substs = _flag(_first(current, _ALLOW_SUBSTS_ATTRS), resolve) or allow_substs
        current = _descend(_first(current, _WRAPPED_ATTRS))
    if is_rewritten_yaml(current):
        rewrites, unresolved = _rewrite_pairs(_first(current, _REWRITE_ATTRS), resolve)
        root_key, unresolved_root = _root_key(_first(current, _ROOT_ATTRS), resolve)
        return Wrapper(
            kind=REWRITTEN,
            source=_first(current, _SOURCE_ATTRS),
            rewrites=rewrites,
            root_key=root_key,
            convert_types=bool(_first(current, _CONVERT_ATTRS)),
            unresolved_rewrites=unresolved,
            unresolved_root_key=unresolved_root,
            wrapped_in_parameter_file=wrapped,
            allow_substs=allow_substs,
        )
    return Wrapper(
        kind="plain",
        source=current,
        wrapped_in_parameter_file=wrapped,
        allow_substs=allow_substs,
    )


def describe(param_item: Any, *, resolve: Resolver | None = None) -> records.ParamSource | None:
    """One `parameters=` item as a ParamSource, or None when nothing static is left.

    A RewrittenYaml becomes a `rewritten_file` source whose `source_path` is the real file
    and whose `rewrites` carry what was substituted. A plain path becomes a `repo_file`
    source: whether that path is inside a tracked tree is the caller's question, and the
    caller reclassifies it as `external_file` when it is not.
    """
    wrapper = unwrap(param_item, resolve=resolve)
    path = _resolve_path(wrapper.source, resolve)
    if path is None:
        return None
    notes = []
    if wrapper.convert_types:
        notes.append("convert_types")
    if wrapper.wrapped_in_parameter_file:
        notes.append("parameter_file")
    if wrapper.allow_substs:
        notes.append("allow_substs")
    if wrapper.unresolved_rewrites:
        notes.append("unresolved rewrites: " + ", ".join(wrapper.unresolved_rewrites))
    if wrapper.unresolved_root_key:
        notes.append("unresolved root_key")
    if wrapper.kind == REWRITTEN:
        return records.ParamSource(
            order=0,
            kind=REWRITTEN,
            source_path=path,
            root_key=wrapper.root_key,
            rewrites=wrapper.rewrites,
            note=", ".join(notes),
        )
    return records.ParamSource(
        order=0,
        kind=REPO_FILE,
        path=path,
        note=", ".join(notes),
    )


def apply_rewrites(
    doc: dict,
    root_key: str | None,
    rewrites: Sequence[tuple[str, str]],
    *,
    convert_types: bool = True,
) -> dict:
    """The reshaping RewrittenYaml performs, on a copy of the document."""
    data = copy.deepcopy(doc)
    table = dict(rewrites)
    if table:
        _substitute(data, table, convert_types)
    if root_key:
        data = {root_key: data}
    return data


def convert_value(text: Any, *, convert_types: bool = True) -> Any:
    """The type coercion RewrittenYaml applies, by string inspection.

    A rewritten value containing a dot becomes a float, which is how a path or a version
    like string silently changes type. The number step is the one `convert_types` gates:
    true and false become booleans either way, so a rewrite of autostart is a bool even
    when the launch file leaves convert_types at its default.
    """
    if not isinstance(text, str):
        return text
    if convert_types:
        try:
            return float(text) if "." in text else int(text)
        except ValueError:
            pass
    if text.lower() in ("true", "false"):
        return text.lower() == "true"
    return text


def rewritten_keys(doc: Mapping[str, Any], rewrites: Sequence[tuple[str, str]]) -> list[str]:
    """Which rewrite keys the document actually contains, sorted. The rest are no ops."""
    wanted = {key for key, _value in rewrites}
    found: set[str] = set()
    _present(doc, wanted, found)
    return sorted(found)


# ---------------------------------------------------------------------------
# internals


def _substitute(node: Any, table: Mapping[str, Any], convert_types: bool) -> None:
    if not isinstance(node, dict):
        return
    for key in list(node):
        value = node[key]
        if isinstance(value, dict):
            _substitute(value, table, convert_types)
        elif str(key) in table:
            node[key] = convert_value(table[str(key)], convert_types=convert_types)


def _present(node: Any, wanted: set[str], found: set[str]) -> None:
    if not isinstance(node, Mapping):
        return
    for key, value in node.items():
        if isinstance(value, Mapping):
            _present(value, wanted, found)
        elif str(key) in wanted:
            found.add(str(key))


def _descend(item: Any) -> Any:
    """A one element substitution list is the item it holds."""
    current = item
    while isinstance(current, (list, tuple)) and len(current) == 1:
        current = current[0]
    return current


def _first(item: Any, names: Sequence[str]) -> Any:
    for name in names:
        value = getattr(item, name, None)
        if value is not None:
            return value
    return None


def _rewrite_pairs(
    raw: Any,
    resolve: Resolver | None,
) -> tuple[tuple[tuple[str, str], ...], tuple[str, ...]]:
    if not isinstance(raw, Mapping):
        return (), ()
    pairs: list[tuple[str, str]] = []
    unresolved: list[str] = []
    for key in sorted(raw, key=str):
        text = _text(raw[key], resolve)
        if text is None:
            unresolved.append(str(key))
        else:
            pairs.append((str(key), text))
    return tuple(pairs), tuple(unresolved)


def _root_key(raw: Any, resolve: Resolver | None) -> tuple[str | None, bool]:
    """The root key, whether it is absent, empty or a substitution."""
    if raw is None:
        return None, False
    text = _text(raw, resolve)
    if text is None:
        return None, True
    return text, False


def _text(value: Any, resolve: Resolver | None = None) -> str | None:
    value = _descend(value)
    if value is None:
        return None
    if isinstance(value, str):
        return value
    if isinstance(value, (bool, int, float)):
        return "true" if value is True else "false" if value is False else str(value)
    if resolve is None:
        return None
    resolved = resolve(value)
    return resolved if isinstance(resolved, str) else None


def _flag(value: Any, resolve: Resolver | None) -> bool:
    text = _text(value, resolve)
    if text is None:
        return bool(value)
    return text.strip().lower() in ("true", "1")


def _resolve_path(source: Any, resolve: Resolver | None) -> str | None:
    """The path a source file item names, without performing anything that writes a file."""
    inner = _descend(source)
    if isinstance(inner, str):
        return inner
    if isinstance(inner, os.PathLike):
        return os.fspath(inner)
    if is_rewritten_yaml(inner) or is_parameter_file(inner):
        # Performing either of these materialises a temp yaml and returns its path.
        return None
    if resolve is not None:
        for candidate in (source, inner) if source is not inner else (source,):
            resolved = resolve(candidate)
            if isinstance(resolved, str) and resolved:
                return resolved
    if isinstance(inner, (list, tuple)):
        parts = [_resolve_path(item, resolve) for item in inner]
        if all(isinstance(part, str) for part in parts):
            return "".join(str(part) for part in parts)
    return None


__all__ = [
    "Resolver",
    "Wrapper",
    "apply_rewrites",
    "convert_value",
    "describe",
    "is_parameter_file",
    "is_rewritten_yaml",
    "rewritten_keys",
    "unwrap",
]
