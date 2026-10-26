"""Just enough of the CMake language to find command invocations and their arguments.

A migration rewrites calls such as `ament_target_dependencies(...)` in place, so it needs the
exact span of each call and of each argument, with comments, quoted and bracket arguments
and nested parentheses taken into account. Nothing is evaluated: a variable reference stays
the text `${name}`, and `set()` is only read back by `literal_lists` for the common case of a
list defined once with plain words.
"""

from __future__ import annotations

import re
from dataclasses import dataclass

_IDENT = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
_BRACKET_OPEN = re.compile(r"\[(=*)\[")


@dataclass(frozen=True)
class Argument:
    text: str
    start: int
    end: int
    quoted: bool = False


@dataclass(frozen=True)
class Command:
    name: str
    """As written. CMake command names are case insensitive, compare with `lower`."""
    start: int
    end: int
    """Offsets of the whole invocation, name to closing parenthesis inclusive."""
    line: int
    column: int
    args: tuple[Argument, ...]

    @property
    def lower(self) -> str:
        return self.name.lower()


def commands(text: str) -> list[Command]:
    """Every command invocation in a CMake file, in order."""
    found: list[Command] = []
    i = 0
    n = len(text)
    at_line_start = True
    while i < n:
        c = text[i]
        if c == "#":
            i = _skip_comment(text, i)
            continue
        if c in "\"'":
            # A stray quote outside a command is not CMake, skip to its line end.
            i = text.find("\n", i)
            i = n if i < 0 else i
            continue
        if c == "\n":
            at_line_start = True
            i += 1
            continue
        if c.isspace():
            i += 1
            continue
        match = _IDENT.match(text, i)
        if match and at_line_start:
            j = match.end()
            while j < n and text[j] in " \t":
                j += 1
            if j < n and text[j] == "(":
                args, close = _arguments(text, j + 1)
                line = text.count("\n", 0, i) + 1
                column = i - (text.rfind("\n", 0, i) + 1) + 1
                found.append(Command(match.group(), i, close + 1, line, column, tuple(args)))
                i = close + 1
                at_line_start = False
                continue
        at_line_start = False
        i = match.end() if match else i + 1
    return found


def _skip_comment(text: str, i: int) -> int:
    bracket = _BRACKET_OPEN.match(text, i + 1)
    if bracket:
        end = text.find("]" + bracket.group(1) + "]", bracket.end())
        return len(text) if end < 0 else end + len(bracket.group(1)) + 2
    end = text.find("\n", i)
    return len(text) if end < 0 else end


def _arguments(text: str, i: int) -> tuple[list[Argument], int]:
    """The arguments from just after `(` to the matching `)`, and the offset of that `)`."""
    args: list[Argument] = []
    depth = 0
    n = len(text)
    while i < n:
        c = text[i]
        if c == "#":
            i = _skip_comment(text, i)
            continue
        if c.isspace():
            i += 1
            continue
        if c == "(":
            depth += 1
            i += 1
            continue
        if c == ")":
            if depth == 0:
                return args, i
            depth -= 1
            i += 1
            continue
        if c == '"':
            j = i + 1
            while j < n and text[j] != '"':
                j += 2 if text[j] == "\\" else 1
            args.append(Argument(text[i + 1 : j], i, j + 1, quoted=True))
            i = j + 1
            continue
        bracket = _BRACKET_OPEN.match(text, i)
        if bracket:
            close = "]" + bracket.group(1) + "]"
            end = text.find(close, bracket.end())
            end = n if end < 0 else end
            args.append(Argument(text[bracket.end() : end], i, end + len(close), quoted=True))
            i = end + len(close)
            continue
        j = i
        while j < n and not text[j].isspace() and text[j] not in '()#"':
            if text[j] == "$" and j + 1 < n and text[j + 1] == "{":
                close = _variable_end(text, j)
                j = close
                continue
            j += 1
        args.append(Argument(text[i:j], i, j))
        i = j
    return args, n


def _variable_end(text: str, i: int) -> int:
    """The offset after a `${...}` reference starting at `i`, nested references included."""
    depth = 0
    j = i
    while j < len(text):
        if text.startswith("${", j):
            depth += 1
            j += 2
            continue
        if text[j] == "}":
            depth -= 1
            j += 1
            if depth == 0:
                return j
            continue
        j += 1
    return j


VARIABLE = re.compile(r"^\$\{([A-Za-z0-9_.+-]+)\}$")


def literal_lists(found: list[Command]) -> dict[str, tuple[str, ...] | None]:
    """Variables set to plain words with `set(NAME word...)`. A variable set more than once,
    or to anything that is not a plain word, maps to None: its value depends on evaluation."""
    values: dict[str, tuple[str, ...] | None] = {}
    for command in found:
        if command.lower not in {"set", "list"} or not command.args:
            continue
        name = command.args[0].text
        if command.lower == "list":
            if len(command.args) > 1:
                values[command.args[1].text] = None
            continue
        words = command.args[1:]
        plain = all(not w.quoted and "$" not in w.text and w.text.isidentifier() for w in words)
        if name in values or not plain or not words:
            values[name] = None
        else:
            values[name] = tuple(w.text for w in words)
    return values
