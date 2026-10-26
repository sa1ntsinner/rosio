"""Substitution helpers for the launch resolver.

Two audiences, one module, because both answer the same question and must answer it the
same way: what text does this launch value have.

* The executed side runs inside the sandbox child. It needs `launch` and therefore every
  helper imports it inside the function, never at module import time.
* The text side is a tiny expression model (`literal`, `config`, `join`) that the AST walk
  fills in from source text with no `launch` anywhere. It reports None the moment it cannot
  see a value, because an invented default here becomes an invented node later.

Nothing in here touches the filesystem and nothing starts a process.
"""

from __future__ import annotations

import shlex
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

from rosio.errors import CollectorError
from rosio.resolve import records

UNSPECIFIED_NAME = "<node_name_unspecified>"
UNSPECIFIED_NAMESPACE = "<node_namespace_unspecified>"
"""launch_ros puts these sentinels in the expanded fields when launch stated neither a name
nor a namespace. Treating one as a real name mints a junk id, so they are matched by value
and never passed on."""

_REPR_MARK = " object at 0x"


def require_launch() -> tuple[Any, Any]:
    """Import launch and launch_ros, or refuse with the collector error."""
    try:
        import launch
        import launch_ros
    except ImportError as exc:  # pragma: no cover, exercised only without ROS
        raise CollectorError(
            "the executed launch walk needs launch and launch_ros on the path",
            layer="declared",
            collector="launch_exec",
        ) from exc
    return launch, launch_ros


# ---------------------------------------------------------------------------
# performing, inside the sandbox child


def perform(context: Any, value: Any) -> str:
    """Text of one substitution, a list of substitutions or a plain string."""
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

    if value is None:
        return ""
    if isinstance(value, str):
        return value
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def perform_optional(context: Any, value: Any) -> str | None:
    """Text of a value launch may not have stated at all. None stays None."""
    return None if value is None else perform(context, value)


def perform_pairs(context: Any, pairs: Any) -> tuple[tuple[str, str], ...]:
    if not pairs:
        return ()
    return tuple((perform(context, left), perform(context, right)) for left, right in pairs)


def perform_argv(context: Any, argv: Any) -> tuple[str, ...]:
    if not argv:
        return ()
    return tuple(perform(context, item) for item in argv)


def command_text(expr: str, token) -> str:
    """Keep command text while stating each machine path through the caller's path rule."""
    from rosio.resolve import identity

    try:
        words = shlex.split(expr)
    except ValueError:
        words = expr.split()
    for word in sorted(words, key=len, reverse=True):
        path = word.partition("=")[2] if "=" in word else word
        if not Path(path).is_absolute():
            continue
        stated = token(path)
        if Path(stated).is_absolute():
            stated = identity.relative_path(path)
        expr = expr.replace(path, stated)
    return expr


def perform_period(context: Any, period: Any) -> float | None:
    """A TimerAction period, which is often a substitution rather than a number."""
    from launch.utilities import type_utils

    if period is None:
        return None
    if isinstance(period, (int, float)) and not isinstance(period, bool):
        return float(period)
    try:
        return float(type_utils.perform_typed_substitution(context, period, float))
    except Exception:
        try:
            return float(perform(context, period))
        except (TypeError, ValueError):
            return None


def namespace_of(expanded: str | None) -> str:
    """Namespace as the declared layer records it.

    An unstated namespace stays the empty string. It is not a stand in for the root: the
    runtime name comes from the executable and inventing `/` here fabricates drift.
    """
    if not expanded or expanded == UNSPECIFIED_NAMESPACE:
        return ""
    return expanded


def name_of(expanded: str | None) -> str | None:
    if not expanded or expanded == UNSPECIFIED_NAME:
        return None
    return expanded


# ---------------------------------------------------------------------------
# describing, without a context


def describe_value(value: Any) -> str:
    """Readable text for a substitution the walk did not perform.

    Used for gate expressions and notes only. `LaunchConfiguration.describe` spells itself
    `LaunchConfig`, which no reader of a launch file recognises, so the class name is put
    back.
    """
    if value is None:
        return ""
    if isinstance(value, str):
        return repr(value)
    if isinstance(value, (list, tuple)):
        return " + ".join(describe_value(item) for item in value)
    described = ""
    describe = getattr(value, "describe", None)
    if callable(describe):
        try:
            described = str(describe())
        except Exception:
            described = ""
    if not described or _REPR_MARK in described:
        described = f"{type(value).__name__}(...)"
    if described.startswith("LaunchConfig("):
        described = "LaunchConfiguration(" + described[len("LaunchConfig(") :]
    return described


def condition_mechanism(condition: Any) -> str:
    """Gate mechanism for a condition object, from its own class name."""
    name = type(condition).__name__
    return name if name in records.GATE_MECHANISMS else "IfCondition"


def describe_condition(condition: Any) -> str:
    inner = None
    for attr in (
        "_IfCondition__predicate_expression",
        "_UnlessCondition__predicate_expression",
        "predicate_expression",
    ):
        if hasattr(condition, attr):
            inner = getattr(condition, attr)
            break
    name = type(condition).__name__
    if inner is None:
        return f"{name}(...)"
    return f"{name}({describe_value(inner)})"


def evaluate_condition(condition: Any, context: Any) -> bool | None:
    """Outcome of a condition, or None when it could not be evaluated."""
    try:
        return bool(condition.evaluate(context))
    except Exception:
        return None


def function_name(function: Any, roots: Sequence[str] = ()) -> str:
    """Discriminator for an OpaqueFunction, from the function itself.

    A qualified name is an attribute of the callable, not a local variable name, so it
    survives `generate_launch_description`. A lambda has no usable name, so the module that
    defines it stands in, stated relative to the workspace, together with the lambda's own
    content: the file it was compiled from and the line it starts on are a machine path and
    an ordinal, and neither may sit inside a launch entity id, while two lambdas in one file
    still have to stay two discriminators.
    """
    name = getattr(function, "__qualname__", "") or getattr(function, "__name__", "")
    name = name.replace("<locals>.", "").strip(".")
    if not name or "<lambda>" in name:
        from rosio import ids
        from rosio.resolve import identity

        code = getattr(function, "__code__", None)
        module = identity.relative_path(str(getattr(code, "co_filename", "")), roots)
        return f"fn{ids.digest(module, name, *code_content(code))}"
    return name


def code_content(code: Any) -> tuple[str, ...]:
    """What one anonymous callback has that its sibling in the same file does not.

    Argument names, the names it reads and its literal constants all come from the source,
    so they separate two lambdas without carrying the file or the line they were compiled
    from. A nested code object is skipped because its repr carries both.
    """
    plain = (str, int, float, bool, type(None))
    out = [str(getattr(code, "co_argcount", 0))]
    out.extend(str(item) for item in getattr(code, "co_varnames", ()) or ())
    out.extend(str(item) for item in getattr(code, "co_names", ()) or ())
    out.extend(
        repr(item) for item in getattr(code, "co_consts", ()) or () if isinstance(item, plain)
    )
    return tuple(out)


# ---------------------------------------------------------------------------
# parameter items, duck typed


def unwrap_param_file(item: Any) -> Any:
    """Follow ParameterFile style wrappers down to the innermost described object.

    `normalize_parameters` wraps everything that is not a dict in a `ParameterFile`, and a
    nav2 `RewrittenYaml` arrives wrapped twice, so the chain has to be followed rather than
    peeled once.
    """
    seen: list[int] = []
    current = item
    while current is not None and id(current) not in seen:
        seen.append(id(current))
        # The name mangled attribute first, on purpose: the public property of an already
        # evaluated ParameterFile returns the temp path it produced, and reading that instead
        # loses the provenance for every node after the first one that shares the object.
        inner = getattr(current, "_ParameterFile__param_file", None)
        if inner is None:
            inner = getattr(current, "param_file", None)
        if inner is None:
            break
        if isinstance(inner, (list, tuple)):
            if len(inner) != 1:
                break
            inner = inner[0]
        current = inner
    return current


def rewritten_fields(item: Any) -> dict[str, Any] | None:
    """Fields of a nav2_common RewrittenYaml, by duck typing only.

    nav2_common is not a dependency and lives outside the workspace, so the object is
    recognised by the attributes it carries under either spelling, public or name mangled.
    """
    target = unwrap_param_file(item)
    if target is None:
        return None
    found: dict[str, Any] = {}
    for field, aliases in (
        ("source_file", ("source_file", "_RewrittenYaml__source_file")),
        ("param_rewrites", ("param_rewrites", "_RewrittenYaml__param_rewrites")),
        ("root_key", ("root_key", "_RewrittenYaml__root_key")),
        ("convert_types", ("convert_types", "_RewrittenYaml__convert_types")),
    ):
        for alias in aliases:
            if hasattr(target, alias):
                found[field] = getattr(target, alias)
                break
    if "source_file" not in found or "param_rewrites" not in found:
        return None
    return found


def describe_param_item(item: Any, order: int, context: Any = None) -> records.ParamSource | None:
    """Fallback for `rewritten_yaml.describe`, with a context so values are real text.

    The shared shim takes no context, so a rewrite value that is a substitution can only be
    rendered there. Here the context is in hand, which is what a rewrite of
    `{'use_sim_time': LaunchConfiguration('use_sim_time')}` needs.
    """
    fields = rewritten_fields(item)
    if fields is None:
        return None
    source = fields["source_file"]
    source_path = perform(context, source) if context is not None else describe_value(source)
    root_key = fields.get("root_key")
    root_text = "" if root_key is None else _text(root_key, context)
    rewrites = tuple(
        sorted(
            (str(key), _text(value, context))
            for key, value in dict(fields["param_rewrites"]).items()
        )
    )
    return records.ParamSource(
        order=order,
        kind="rewritten_file",
        path=None,
        source_path=source_path,
        root_key=root_text,
        rewrites=rewrites,
    )


def usable_rewrites(source: records.ParamSource) -> bool:
    """Whether a described rewrite list carries text rather than object reprs."""
    if _REPR_MARK in (source.source_path or ""):
        return False
    return all(_REPR_MARK not in value for _key, value in source.rewrites)


def _text(value: Any, context: Any) -> str:
    if isinstance(value, str):
        return value
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float)):
        return str(value)
    if context is not None:
        try:
            return perform(context, value)
        except Exception:
            return describe_value(value)
    return describe_value(value)


# ---------------------------------------------------------------------------
# the text only expression model, for the walk with no ROS


LITERAL = "literal"
CONFIG = "config"
JOIN = "join"
OPAQUE = "opaque"

Expr = tuple


def literal(text: str) -> Expr:
    return (LITERAL, text)


def config(name: str, default: Expr | None = None) -> Expr:
    return (CONFIG, name, default)


def join(parts: Sequence[Expr], separator: str = "") -> Expr:
    return (JOIN, tuple(parts), separator)


def opaque(text: str = "") -> Expr:
    return (OPAQUE, text)


def render(expr: Expr | None, values: Mapping[str, str]) -> str | None:
    """Text of a text side expression, or None when any part of it is unknown."""
    if expr is None:
        return None
    kind = expr[0]
    if kind == LITERAL:
        return str(expr[1])
    if kind == CONFIG:
        name = str(expr[1])
        if name in values:
            return values[name]
        return render(expr[2], values)
    if kind == JOIN:
        parts = []
        for part in expr[1]:
            rendered = render(part, values)
            if rendered is None:
                return None
            parts.append(rendered)
        return str(expr[2]).join(parts)
    return None


__all__ = [
    "CONFIG",
    "JOIN",
    "LITERAL",
    "OPAQUE",
    "UNSPECIFIED_NAME",
    "UNSPECIFIED_NAMESPACE",
    "code_content",
    "condition_mechanism",
    "config",
    "describe_condition",
    "describe_param_item",
    "describe_value",
    "evaluate_condition",
    "function_name",
    "join",
    "literal",
    "name_of",
    "namespace_of",
    "opaque",
    "perform",
    "perform_argv",
    "perform_optional",
    "perform_pairs",
    "perform_period",
    "render",
    "require_launch",
    "rewritten_fields",
    "unwrap_param_file",
    "usable_rewrites",
]
