"""Parser for a ros 2 argv tail.

An `ExecuteProcess` that is really a ROS node carries its whole identity in argv: the
name, the namespace, the remaps, the parameter assignments and the parameters files. On the
reference stack seven such processes exist, so an argv that is not parsed becomes seven
nodes that look observed only and every value in them reads as drift.

Two properties matter more than coverage:

* Malformed argv never raises. Whatever could be read is returned, and every token that was
  not understood stays in `leftover`, so nothing is silently dropped.
* Types are preserved exactly. `rcl` parses a `-p` value as a yaml scalar, so the text `20`
  is an int and the text `20.0` is a float. Turning one into the other is a phantom drift on
  its own, which is why the raw text is kept in the record and `parse_value` is the only
  place that types it.
"""

from __future__ import annotations

import math
import re
import sys
from collections.abc import Sequence
from dataclasses import replace
from typing import Any

from rosio import ids, model
from rosio.errors import ContractError
from rosio.resolve import records

ROS_ARGS = "--ros-args"
SEPARATOR = "--"
ASSIGN = ":="

PARAM_FLAGS = ("-p", "--param")
REMAP_FLAGS = ("-r", "--remap")
PARAMS_FILE_FLAGS = ("--params-file",)
LOG_LEVEL_FLAGS = ("--log-level",)
ENCLAVE_FLAGS = ("-e", "--enclave")
OTHER_VALUE_FLAGS = ("--log-config-file",)

VALUE_FLAGS = (
    PARAM_FLAGS
    + REMAP_FLAGS
    + PARAMS_FILE_FLAGS
    + LOG_LEVEL_FLAGS
    + ENCLAVE_FLAGS
    + OTHER_VALUE_FLAGS
)

BARE_FLAGS = (
    "--disable-external-lib-logs",
    "--disable-rosout-logs",
    "--disable-stdout-logs",
    "--enable-external-lib-logs",
    "--enable-rosout-logs",
    "--enable-stdout-logs",
)

NODE_KEYS = ("__node", "__name")
NAMESPACE_KEYS = ("__ns",)
PARAMS_FILE_KEYS = ("__params",)

INPUT_ASSIGN = "assign"
INPUT_FILE = "params_file"
"""The two members of `records.RosArgs.inputs`. A file entry carries the path, an assign
entry carries the `(name, raw text)` pair, and their order in the tuple is the precedence."""

SEQUENCE_MIXED_TYPES = "sequence_mixed_types"
VALUE_EMPTY = "value_empty"
VALUE_UNPARSABLE = "value_unparsable"
VALUE_UNSUPPORTED = "value_unsupported"
"""The four ways rcl throws out a `-p` value, one per message it prints: "Sequence should be
of same type", "No value at line N", "Error parsing an event" and "Unknown YAML event". Each
of them refuses the whole argument list, so the process never starts and nothing about it is
a fact."""

SEQUENCE_NESTED = "sequence_nested"
"""The one shape rosio refuses while rcl takes it, see `refuse_value`. The process starts,
and rcl leaves more than one slot of its variant filled, so the value the node holds is the
first slot its client library looks at rather than anything the text states."""

VALUE_UNMODELLED = "value_unmodelled"
"""A construct this parser has no model of, see `refuse_value`. It states nothing about
whether the process started, because the same construct does both: `&x 1` runs as the
integer 1 and `&x` alone takes `rcl_init` down."""

RCL_REFUSALS = (SEQUENCE_MIXED_TYPES, VALUE_EMPTY, VALUE_UNPARSABLE, VALUE_UNSUPPORTED)
ROSIO_REFUSALS = (SEQUENCE_NESTED,)
UNMODELLED_REFUSALS = (VALUE_UNMODELLED,)
"""An rcl refusal means the process never started. A rosio refusal means it did, and that
the value it runs under is not a property of the text. An unmodelled refusal claims neither.
The three carry different sentences, so they are three tuples and not one."""

SETS_NOTHING = "sets_nothing"
"""Not a refusal: the process starts, the parameter is declared, and it has no value and no
type at all. See `sets_nothing`."""

_TRUE = ("y", "Y", "yes", "Yes", "YES", "true", "True", "TRUE", "on", "On", "ON")
_FALSE = ("n", "N", "no", "No", "NO", "false", "False", "FALSE", "off", "Off", "OFF")
"""The eleven spellings per side that rcl accepts, an exact case list and not a case
insensitive rule: `true`, `True` and `TRUE` are booleans while `TrUe` and `yEs` are
strings."""

_INT_DEC = re.compile(r"^[+-]?(0|[1-9][0-9]*)$")
_INT_OCT = re.compile(r"^[+-]?0[0-7]+$")
_INT_HEX = re.compile(r"^[+-]?0[xX][0-9a-fA-F]+$")
"""rcl reads an integer with `strtol` in base 0, so `0x10` is 16 and `017` is 15, while
`0o17` and `0b101` are strings. Verified on humble."""

_INT_MIN = -(2**63)
_INT_MAX = 2**63 - 1
_DOUBLE_MIN = sys.float_info.min

_FLOAT_DEC = re.compile(r"^[+-]?([0-9]+\.[0-9]*|\.[0-9]+|[0-9]+)([eE][+-]?[0-9]+)?$")
_FLOAT_HEX = re.compile(
    r"^[+-]?0[xX]([0-9a-fA-F]+\.?[0-9a-fA-F]*|\.[0-9a-fA-F]+)([pP][+-]?[0-9]+)?$"
)
_FLOAT_WORD = re.compile(r"^[+-]?(inf|infinity|nan)$", re.IGNORECASE)
_YAML_INF = re.compile(r"^[+-]?\.(inf|Inf|INF)$")
_YAML_NAN = re.compile(r"^\.(nan|NaN|NAN)$")
"""The float side is `strtod`, which takes `inf`, `infinity` and `nan` in any case and hex
floats, plus the two yaml 1.1 spellings libyaml resolves before rcl sees them. `.iNf` and
`-.NAN` are outside both, and are strings."""

_ESCAPES = {
    "0": "\x00",
    "a": "\a",
    "b": "\b",
    "t": "\t",
    "\t": "\t",
    "n": "\n",
    "v": "\v",
    "f": "\f",
    "r": "\r",
    "e": "\x1b",
    " ": " ",
    '"': '"',
    "/": "/",
    "\\": "\\",
    "N": "\x85",
    "_": "\xa0",
    "L": "\u2028",
    "P": "\u2029",
}
_HEX_ESCAPES = {"x": 2, "u": 4, "U": 8}


# ---------------------------------------------------------------------------
# parsing


def parse_ros_args(argv: Sequence[str]) -> records.RosArgs:
    """Read every `--ros-args` region of an argv tail.

    Tokens before the first `--ros-args` belong to the program, a bare `--` closes the
    region, and a second `--ros-args` opens a new one. Everything not understood, including
    the tokens of a flag that lost its value, is kept in `leftover` in argv order.
    """
    state = _State()
    _walk(state, argv)
    return state.result()


def parse_value(text: str) -> Any:
    """Type a `-p` value the way rcl reads it.

    The rules are neither pyyaml's nor C's, so both were measured against a node on humble
    rather than assumed. libyaml hands rcl the scalar and its quoting style, and rcl types
    the plain ones itself with `strtol` in base 0 and then `strtod`:

    * a quoted scalar is a string whatever it spells, with the escapes of its own style, so
      `"a\\nb"` is two lines and `'a\\nb'` keeps the backslash
    * `0x10` is 16 and `017` is 15, `0o17` and `0b101` are strings
    * `.inf`, `-.Inf`, `.nan`, `inf` and `infinity` are floats, `.iNf` and `-.NAN` are not
    * a value out of the range of its type stays a string, `1e400` and `1e-320` included
    * the boolean spellings are an exact case list, `_TRUE` and `_FALSE`
    * a comment is gone before anything is typed, so `1 #c` is the integer 1 while `a#c` is
      the three character string. `_scan` holds the four positions one opens at, measured
    * one process per value, and the refusal a comment causes when a value follows it
    * a scalar spanning more than one line is folded before it is typed, `_fold`
    * a block sequence is an array, `_block_sequence`

    pyyaml is not used here: it reads `null` and `~` as None, which rcl never produces.

    How the boolean list was measured: each spelling was passed one per process as
    `-p k0:=<text>` to an `rclpy` node with
    `automatically_declare_parameters_from_overrides`, and the parameter's `type_` and
    `value` were read back. `y`, `Y`, `yes`, `Yes`, `YES`, `true`, `True`, `TRUE`, `on`,
    `On` and `ON` came back `Type.BOOL True`, and `n`, `N`, `no`, `No`, `NO`, `false`,
    `False`, `FALSE`, `off`, `Off` and `OFF` came back `Type.BOOL False`. `TrUe`, `tRue`,
    `yEs`, `yES`, `nO`, `oFF`, `oN`, `ye` and `Ye` came back `Type.STRING` with the text
    unchanged, which is why this is a list and not a lowercase comparison. `[true, no]`,
    `[yes, off]` and `[y, N]` came back `Type.BOOL_ARRAY [True, False]`, so a list of two
    boolean spellings is one boolean array and not a mixed one.

    A value `refuse_value` refuses and a value `sets_nothing` answers for both come back as
    the raw text, because the text is then the only thing about them that is true: either
    nothing typed it, or it typed to no value at all. Ask both before recording a value
    anywhere, and record the blind spot instead. The three functions walk the same text once
    in `_evaluate`, so they cannot disagree: this one returns the raw text for exactly the
    values the other two speak for.
    """
    raw = str(text)
    reason, value = _evaluate(raw)
    return raw if reason else value


def refuse_value(text: str) -> str:
    """Why rcl would throw out this `-p` value, or an empty string when it takes it.

    Ask this before recording a value. A value rcl refuses takes the whole argument list
    down with it, `rcl_init` fails and the node never starts, so a value recorded for it is
    a fact about a process that does not exist. Measured on humble, one value per process,
    against an rclpy node with `automatically_declare_parameters_from_overrides`:

    * `sequence_mixed_types`: the elements of a sequence do not all type the same. Integer
      and double are two types here and neither promotes, so `[1, 2.0]` and `[1e5, 2]` are
      refused exactly like `[1, x]` and `[true, 1]`. `[1, 2]`, `[a, b]`, `[true, no]`,
      `[0x10, 2]` and `[.nan, 1.0]` are taken, so this is the typing `parse_value` already
      performs and not a second rule. A block sequence types the same way, `- 1` and a break
      and `- 2.0` is refused and `- 1` and a break and `- 0x10` is `[1, 16]`.
    * `value_empty`: a node with nothing in it, which rcl reports as "No value at line N".
      Every one of these is a block sequence with an empty item: `-`, `- `, `-` and a break
      and `- a`, `- a` and a break and `-`, and `- -`.
    * `value_unparsable`: libyaml refuses the text, "Error parsing an event". The text opens
      a flow collection and does not close one, with an empty element (`[,]`, `[, 1]`,
      `[1, , 2]`), an unclosed or unbalanced bracket (`[`, `[a`, `[[1, 2]`, `[1, 2]]`,
      `[a]x`) and a leading `]` all this. A comma after the last element is not: `[1,]` is
      taken and is `[1]`. A quoted scalar that never closes is this (`'a`, `"a`, `'`), and so
      is anything but blanks and comments after one that did close (`'a' b`, `'a'b`,
      `'a' 'b'`, `'a b'` and a break and `c`). A comment reaches this rule twice: it swallows
      the bracket that would have closed the collection (`[a #c]` and `[1 #c]` start no
      process), and a comment that ends one scalar with more of that scalar after it leaves
      two document nodes, which `_scan` measures and answers. A node opening with `,`, `@` or
      a backtick is this in every position, and inside a flow collection so are `:`, `- `,
      `|` and `>`.
    * `value_unsupported`: an event rcl has no case for, "Unknown YAML event". A mapping,
      `{a: 1}`, `[{a: 1}]`, `a: 1`, `a:`, `'a': 1` and `- a: 1`, a complex mapping key, `?`
      and `? a` and `[?a]`, and an alias, `*x` and `[*x]` and `- *x`. A key on a later line
      of a scalar that already started is not this and is `value_unparsable`, measured: `a: 1`
      refuses with "Unknown YAML event" and `a` and a break and `b: 1` with "Error parsing an
      event".

    A plain scalar carrying `[`, `{`, `]` or `}` anywhere but at the front is a string and
    starts fine, `a[b]`, `/topic[0]` and `foo{bar}` included, and so is any quoted scalar.
    Two refusals open no flow collection: `-p k0:=`, which ends `rcl_init` with "Couldn't
    parse parameter override rule: '-p k0:='", and the whitespace `_refused_whitespace`
    names. A value that is spaces, line feeds or a comment is a third answer again and is
    not a refusal, see `sets_nothing`.

    * `sequence_nested`: a sequence holding a sequence. This is the one refusal rcl does not
      make, and rosio makes it because the value the process runs under is not a property
      of the text. Measured twice on humble: against a node, `[[1, 2], [3]]` comes back
      `[3]`, `[[1, 2], [a]]` comes back `[1, 2]`, `[[a], b]` comes back the string `b` and
      `[x, [1, 2]]` comes back `[1, 2]`. Against `rcl_parse_yaml_value` directly, the same
      values leave more than one slot of `rcl_variant_t` filled at once (`[[1, 2], [a]]`
      leaves the integer array `[1, 2]` and the string array `[a]`), against that header's
      own "Only one pointer in this struct will store the value". Which of the filled slots
      becomes the parameter is the order the client library reads them in, and only rclpy
      was measured, so naming one of them as the value would state for a C++ node a number
      that was never measured. The whole text stays as the blind spot instead. A block
      sequence nests the same way and reads the same, `- [1,2]` and a break and `- [3]` comes
      back `[3]` and `- a` and a break and `- - b` comes back `[b]`. An empty sequence is not
      this: `[[]]` and `- []` set nothing and `sets_nothing` answers them.

    * `value_unmodelled`: the value opens a construct this parser has no model of, and
      unlike every code above it claims nothing about whether the process started, because
      the same construct does both. Five of them, measured in all three positions a node
      appears in (a whole value, an element of a flow collection, an item of a block
      sequence): an anchor (`&x 1` runs as the integer 1, `&x a` as the string `a`, `&x`
      alone takes `rcl_init` down with "No value", `& x` and `&x[1, 2]` with "Error parsing
      an event"), a tag (`!!str 1` runs as the integer 1, so rcl drops the tag rather than
      reading it, `!x a` and `! a` and `!<tag:x> a` run as the string `a`, `!t` alone and
      `!!str` alone take it down), a block scalar header (`|` and a break and ` a` runs as
      the string `a`, `|-` and `|1` and `>` the same, `|` alone takes it down, `| a` takes it
      down), a directive (`%YAML 1.1` and a break and `---` and a break and `a` runs as the
      string `a`, `%YAML 1.1` alone takes it down) and a document marker at column 0 (`--- a`
      runs as the string `a`, `a` and a break and `---` and a break and `b` runs as the
      string `b` because the last document wins, `---` alone takes it down). Modelling any of
      them means modelling chomping, tag resolution and multi document precedence, and each
      of the five is absent from every stack this was measured against, so the shape is
      refused and no value is named. A marker only counts at column 0: `[---]`, `- ---`,
      `...a` and `x---` are plain scalars and are taken.

    A refusal in `RCL_REFUSALS` means the process never started. The one in
    `ROSIO_REFUSALS` means it did. The one in `UNMODELLED_REFUSALS` says neither.
    """
    reason = _evaluate(str(text))[0]
    return "" if reason == SETS_NOTHING else reason


def sets_nothing(text: str) -> bool:
    """Whether rcl takes this `-p` value and the parameter comes out with no value at all.

    Ask this before recording a value, next to `refuse_value`. The process starts, the
    parameter is declared, and its type is `NOT_SET`: naming the text as the value states a
    value no node holds.

    Measured on humble, one value per process, against an rclpy node with
    `automatically_declare_parameters_from_overrides`, reading `type_` and `value` back. The
    rule is that the yaml document the text spells is empty:

    * spaces, line feeds and carriage returns only: `' '`, `'   '`, `'\\n'`, `' \\n '` and
      `'\\r'` all come back `Type.NOT_SET`
    * a comment only: `'#'`, `'#c'`, `' #c'`, `' # '` and `'#[1]'`, same answer
    * an empty sequence at any depth, flow or block: `'[]'`, `'[ ]'`, `' [] '`, `'[] #c'`,
      `'[[]]'`, `'[[], []]'`, `'- []'` and `'- []'` and a break and `'- []'`, same answer
    * `''` is not this and is refused, see `refuse_value`, and neither is a leading tab,
      which refuses the argument list rather than emptying the document. A tab outside the
      leading run pads like a space, so `'[]\\t'`, `'[] \\t'` and `'[]\\t#c'` are all
      `Type.NOT_SET` too
    * a quoted scalar always sets a value: `"' '"` is the one character string `' '` and
      `"''"` is the empty string

    A value with something in it beside a comment sets that something, so this answers no for
    `'a #c'` and for `'[[],1]'`.
    """
    return _evaluate(str(text))[0] == SETS_NOTHING


def typed_assignments(parsed: records.RosArgs) -> tuple[tuple[str, Any], ...]:
    """The `-p` assignments with their values typed, in argv order."""
    return tuple((name, parse_value(text)) for name, text in parsed.assignments)


def param_inputs(argv: Sequence[str]) -> tuple[tuple[str, Any], ...]:
    """The parameter inputs of an argv tail, in argv order."""
    return parse_ros_args(argv).inputs


def apply_ros_args(node: records.ResolvedNode, parsed: records.RosArgs) -> records.ResolvedNode:
    """Fold an argv parse into a resolved node.

    Precedence is read off `inputs`, which keeps the files and the assignments interleaved as
    argv wrote them: a `--params-file` after a `-p` wins the keys they share, exactly as a
    later item of a `parameters=` list does.
    """
    return _apply(node, parsed, parsed.inputs)


def apply_argv(node: records.ResolvedNode, argv: Sequence[str]) -> records.ResolvedNode:
    """Parse and fold in one step."""
    return apply_ros_args(node, parse_ros_args(argv))


STATIC_TF_PROGRAM = ("tf2_ros", "static_transform_publisher")
_STATIC_TF_FLAGS = {
    "--x": "translation.x",
    "--y": "translation.y",
    "--z": "translation.z",
    "--qx": "rotation.x",
    "--qy": "rotation.y",
    "--qz": "rotation.z",
    "--qw": "rotation.w",
    "--roll": "roll",
    "--pitch": "pitch",
    "--yaw": "yaw",
    "--frame-id": "frame_id",
    "--frame_id": "frame_id",
    "--child-frame-id": "child_frame_id",
    "--child_frame_id": "child_frame_id",
}


ANGLE_UNREADABLE = "angle_unreadable"
"""A roll, pitch or yaw that is not a number, so no rotation of that transform is known."""
_ROTATION = ("rotation.x", "rotation.y", "rotation.z", "rotation.w")


def program_params(node: records.ResolvedNode) -> records.ResolvedNode:
    """The parameters a known program sets from its own arguments.

    `tf2_ros/static_transform_publisher` reads its flags, or the positional form ending in the
    frame pair, and hands them to its node as parameter overrides
    (`static_transform_broadcaster_program.cpp`), so the node's `frame_id` default is never
    what runs once the arguments name a frame.
    """
    found, unreadable = _program(node.pkg, node.executable, node.argv)
    if not found and not unreadable:
        return node
    order = max((item.order for item in node.params), default=-1) + 1
    base = node.site.locator if node.site else ""
    note = "an argument static_transform_publisher hands its node as a parameter"
    added = [
        records.ParamSource(
            order=order + index,
            kind="ros_args_assign",
            values=((name, found[name][0]),),
            note=note,
            site=_site(node, base, found[name][1]),
        )
        for index, name in enumerate(sorted(found))
    ]
    if unreadable:
        from rosio.resolve._launch_child import uncarried

        expr = " ".join(f"{flag} {text}" for text, flag in unreadable)
        what = " ".join(dict.fromkeys(flag for _text, flag in unreadable))
        added.append(
            records.ParamSource(
                order=order + len(added),
                kind="inline_dict",
                values=tuple((name, uncarried(expr, ANGLE_UNREADABLE)) for name in _ROTATION),
                note=f"mechanism={model.Mechanism.ROS_ARGS_P.value}, {note}",
                site=_site(node, base, what),
            )
        )
    return replace(node, params=(*node.params, *added))


def program_assignments(
    pkg: str, executable: str, argv: Sequence[Any]
) -> dict[str, tuple[str, str]]:
    """Parameter name to (text, the argument that states it) for a known program."""
    return _program(pkg, executable, argv)[0]


def _program(
    pkg: str, executable: str, argv: Sequence[Any]
) -> tuple[dict[str, tuple[str, str]], list[tuple[str, str]]]:
    """The assignments, and the (text, argument) of each angle that is not a number, which
    leaves the rotation unknown rather than the identity."""
    if (pkg, executable) != STATIC_TF_PROGRAM:
        return {}, []
    words = [str(item) for item in argv]
    if ROS_ARGS in words:
        words = words[: words.index(ROS_ARGS)]
    found: dict[str, tuple[str, str]] = {}
    if words and words[0] in _STATIC_TF_FLAGS:
        for flag, value in zip(words[::2], words[1::2], strict=False):
            if flag in _STATIC_TF_FLAGS:
                found[_STATIC_TF_FLAGS[flag]] = (value, flag)
    elif len(words) in (8, 9):
        names = ["translation.x", "translation.y", "translation.z"]
        names += (
            ["yaw", "pitch", "roll"]
            if len(words) == 8
            else ["rotation.x", "rotation.y", "rotation.z", "rotation.w"]
        )
        names += ["frame_id", "child_frame_id"]
        found = {name: (value, "positional") for name, value in zip(names, words, strict=True)}
    stated = [found.pop(name, ("0", "")) for name in ("roll", "pitch", "yaw")]
    angles = [_number(text) for text, _flag in stated]
    unreadable = [row for row, angle in zip(stated, angles, strict=True) if angle is None]
    if "rotation.w" in found:
        unreadable = []
    elif not unreadable and any(angles):
        flags = " ".join(dict.fromkeys(flag for _text, flag in stated if flag))
        quaternion = _quaternion(*angles)  # type: ignore[arg-type]
        for index, name in enumerate(_ROTATION):
            found[name] = (repr(quaternion[index]), flags)
    for name, (value, flag) in list(found.items()):
        number = _number(value)
        if name.startswith(("translation.", "rotation.")) and number is not None:
            # the program parses every number as a double, whatever the text spells
            found[name] = (repr(number), flag)
    return found, unreadable


def _number(text: str) -> float | None:
    try:
        return float(text)
    except ValueError:
        return None


def _quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """`tf2::Quaternion::setRPY`."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def available(*_args: object, **_kwargs: object) -> bool:
    """Argv parsing needs nothing optional. Present so callers can probe uniformly."""
    return True


# ---------------------------------------------------------------------------
# internals


class _State:
    def __init__(self) -> None:
        self.node_name: str | None = None
        self.namespace: str | None = None
        self.params_files: list[str] = []
        self.assignments: list[tuple[str, str]] = []
        self.remaps: list[tuple[str, str]] = []
        self.log_level: str | None = None
        self.enclave: str | None = None
        self.leftover: list[str] = []
        self.inputs: list[tuple[str, Any]] = []

    def add_file(self, path: str) -> None:
        self.params_files.append(path)
        self.inputs.append((INPUT_FILE, path))

    def add_assignment(self, name: str, text: str) -> None:
        self.assignments.append((name, text))
        self.inputs.append((INPUT_ASSIGN, (name, text)))

    def result(self) -> records.RosArgs:
        return records.RosArgs(
            node_name=self.node_name,
            namespace=self.namespace,
            params_files=tuple(self.params_files),
            assignments=tuple(self.assignments),
            remaps=tuple(self.remaps),
            log_level=self.log_level,
            enclave=self.enclave,
            leftover=tuple(self.leftover),
            inputs=tuple(self.inputs),
        )


def _walk(state: _State, argv: Sequence[str]) -> None:
    tokens = [str(item) for item in argv]
    inside = False
    index = 0
    while index < len(tokens):
        token = tokens[index]
        index += 1
        if token == ROS_ARGS:
            inside = True
            continue
        if not inside:
            state.leftover.append(token)
            continue
        if token == SEPARATOR:
            inside = False
            continue
        flag, inline = _split_flag(token)
        if flag in VALUE_FLAGS:
            value = inline
            if value is None:
                if index >= len(tokens) or tokens[index] == ROS_ARGS:
                    state.leftover.append(token)
                    continue
                value = tokens[index]
                index += 1
            _apply_flag(state, flag, value)
            continue
        if token in BARE_FLAGS:
            state.leftover.append(token)
            continue
        if ASSIGN in token and not token.startswith("-"):
            _apply_rule(state, token)
            continue
        state.leftover.append(token)


def _split_flag(token: str) -> tuple[str, str | None]:
    """`--params-file=/x` is one token in hand written argv, two in what rcl emits."""
    if token.startswith("--") and "=" in token:
        head, _, tail = token.partition("=")
        if head in VALUE_FLAGS:
            return head, tail
    return token, None


def _apply_flag(state: _State, flag: str, value: str) -> None:
    if flag in PARAM_FLAGS:
        if ASSIGN not in value:
            state.leftover.extend([flag, value])
            return
        name, _, text = value.partition(ASSIGN)
        try:
            name = ids.normalise_param_name(name.strip())
        except ContractError:
            state.leftover.extend([flag, value])
            return
        state.add_assignment(name, text)
        return
    if flag in REMAP_FLAGS:
        _apply_rule(state, value, flag=flag)
        return
    if flag in PARAMS_FILE_FLAGS:
        if value:
            state.add_file(value)
        else:
            state.leftover.append(flag)
        return
    if flag in LOG_LEVEL_FLAGS:
        state.log_level = value
        return
    if flag in ENCLAVE_FLAGS:
        state.enclave = value
        return
    state.leftover.extend([flag, value])


def _apply_rule(state: _State, rule: str, flag: str = "") -> None:
    """One remap rule, either after `-r` or bare in the legacy spelling.

    A rule with a node prefix (`cropper:input:=/points`) is kept whole in the remap list:
    the parser does not know which node this argv belongs to, and applying another node's
    identity rule would rename the wrong node.

    The first `__node` rule and the first `__ns` rule of an argument list win, which is the
    opposite of a repeated `-p` and is what rcl does. Measured on humble, one process per
    case, `ROS_DOMAIN_ID=77` and `ROS_LOCALHOST_ONLY=1`, each process reporting its own name
    and namespace: `-r __node:=first -r __node:=second` runs as `first`, the same pair split
    across two `--ros-args` groups also runs as `first`, and `-r __ns:=/a -r __ns:=/b` lands
    in `/a`. AGENTS section 9 point 5.
    """
    left, sep, right = rule.partition(ASSIGN)
    left = left.strip()
    right = right.strip()
    if not sep or not left or not right:
        state.leftover.extend([flag, rule] if flag else [rule])
        return
    prefix, _, key = left.rpartition(":")
    if prefix:
        state.remaps.append((left, right))
        return
    if key in NODE_KEYS:
        if state.node_name is None:
            state.node_name = right
        return
    if key in NAMESPACE_KEYS:
        if state.namespace is None:
            state.namespace = right
        return
    if key in PARAMS_FILE_KEYS:
        state.add_file(right)
        return
    state.remaps.append((left, right))


def _apply(
    node: records.ResolvedNode,
    parsed: records.RosArgs,
    inputs: Sequence[tuple[str, Any]],
) -> records.ResolvedNode:
    base = node.site.locator if node.site else ""
    params = list(node.params)
    order = max((item.order for item in params), default=-1) + 1
    for kind, payload in inputs:
        if kind == INPUT_FILE:
            params.append(
                records.ParamSource(
                    order=order,
                    kind="ros_args_file",
                    path=str(payload),
                    site=_site(node, base, "--params-file"),
                )
            )
        else:
            name, text = payload
            params.append(
                records.ParamSource(
                    order=order,
                    kind="ros_args_assign",
                    values=((name, text),),
                    note="raw argv text, the type is the consumer's business",
                    site=_site(node, base, f"-p {name}"),
                )
            )
        order += 1
    remaps = list(node.remaps)
    for frm, to in parsed.remaps:
        remaps.append(
            records.Remap(frm=frm, to=to, scope="node", site=_site(node, base, f"-r {frm}"))
        )
    attrs = dict(node.attrs)
    if parsed.log_level:
        attrs["log_level"] = parsed.log_level
    if parsed.enclave:
        attrs["enclave"] = parsed.enclave
    if parsed.leftover:
        attrs["ros_args_leftover"] = list(parsed.leftover)
    name = node.name
    name_source = node.name_source
    # Measured on humble, one process per case: rcl keeps the first `__node` rule it is
    # handed, and launch_ros appends its own after the arguments a launch file passed, so
    # an argv assignment is the name that runs whatever the launch file stated.
    if parsed.node_name:
        name = parsed.node_name
        name_source = "argv"
    namespace = node.namespace
    if parsed.namespace:
        namespace = ids.normalise_namespace(parsed.namespace)
    return replace(
        node,
        name=name,
        name_source=name_source,
        namespace=namespace,
        params=tuple(params),
        remaps=tuple(remaps),
        attrs=attrs,
    )


def _site(node: records.ResolvedNode, base: str, what: str) -> records.Site | None:
    if node.site is None:
        return None
    locator = f"{base}.cmd[{what}]" if base else f"cmd[{what}]"
    return node.site.with_locator(locator)


def _integer(text: str) -> int | None:
    """`strtol` in base 0 over the whole token, refusing what would overflow an int64."""
    body = text[1:] if text[0] in "+-" else text
    if _INT_DEC.match(text):
        value = int(body, 10)
    elif _INT_OCT.match(text):
        value = int(body, 8)
    elif _INT_HEX.match(text):
        value = int(body, 16)
    else:
        return None
    if text[0] == "-":
        value = -value
    return value if _INT_MIN <= value <= _INT_MAX else None


def _double(text: str) -> float | None:
    """`strtod` over the whole token, plus the two yaml spellings of infinity and nan.

    A token whose value does not fit a double is a string, because rcl checks `errno` and
    falls back. That covers overflow (`1e400`) and underflow (`1e-320`) alike.
    """
    if _YAML_INF.match(text):
        return float("-inf") if text[0] == "-" else float("inf")
    if _YAML_NAN.match(text):
        return float("nan")
    if _FLOAT_WORD.match(text):
        return float(text)
    if _FLOAT_HEX.match(text):
        try:
            value = float.fromhex(text)
        except (ValueError, OverflowError):
            return None
    elif _FLOAT_DEC.match(text):
        value = float(text)
    else:
        return None
    return None if _out_of_range(text, value) else value


def _out_of_range(text: str, value: float) -> bool:
    """Whether strtod would have set ERANGE: a significand that is not zero, and a result
    that is infinite, zero or subnormal."""
    if -_DOUBLE_MIN < value < _DOUBLE_MIN and any(digit in text for digit in "123456789"):
        return True
    return abs(value) == float("inf")


def _unquote(body: str, style: str) -> str:
    """The content of a quoted scalar, with the escapes its own quoting style gives it."""
    if style == "'":
        return body.replace("''", "'")
    out: list[str] = []
    index = 0
    while index < len(body):
        char = body[index]
        if char != "\\" or index + 1 >= len(body):
            out.append(char)
            index += 1
            continue
        marker = body[index + 1]
        width = _HEX_ESCAPES.get(marker)
        if width is not None:
            hexits = body[index + 2 : index + 2 + width]
            if len(hexits) == width:
                try:
                    out.append(chr(int(hexits, 16)))
                    index += 2 + width
                    continue
                except ValueError:
                    pass
        if marker in _ESCAPES:
            out.append(_ESCAPES[marker])
            index += 2
            continue
        out.extend((char, marker))
        index += 2
    return "".join(out)


_TOP = "top"
_FLOW = "flow"
_ITEM = "item"
"""The three positions a node appears in: a whole `-p` value, an element of a flow
collection, an item of a block sequence. A few answers differ by position and each one was
measured in all three, see `_opener`."""

_BREAK = re.compile(r"\r\n|\r|\n")


def _evaluate(raw: str) -> tuple[str, Any]:
    """One walk behind all three public answers: the reason nothing can be named, or the
    value. The reason is `SETS_NOTHING`, a refusal code, or empty when the value stands."""
    if raw == "":
        return VALUE_UNPARSABLE, None
    refusal = _refused_whitespace(raw)
    if refusal:
        return refusal, None
    if _document_marker(raw):
        return VALUE_UNMODELLED, None
    return _node(raw, _TOP)


def _node(text: str, position: str) -> tuple[str, Any]:
    """One yaml node, in one of the three positions `_TOP`, `_FLOW` and `_ITEM`."""
    body, split = _scan(text)
    stripped = body.strip(_YAML_BLANK)
    if not stripped:
        if position == _TOP:
            return SETS_NOTHING, None
        return (VALUE_EMPTY if position == _ITEM else VALUE_UNPARSABLE), None
    lines = _lines(text)
    if position != _FLOW and _opens_sequence(lines):
        return _block_sequence(lines)
    if split:
        return VALUE_UNPARSABLE, None
    head = stripped[0]
    if head in "[]{}":
        return _flow_node(stripped)
    if head in ("'", '"'):
        return _quoted_node(stripped)
    refusal = _opener(text, stripped, position)
    if refusal:
        return refusal, None
    refusal = _mapping(lines, position)
    if refusal:
        return refusal, None
    if position == _FLOW and ("[" in stripped or "{" in stripped):
        return VALUE_UNPARSABLE, None
    return "", _typed(_fold(body, False).strip(_YAML_BLANK))


def _sequence(texts: Sequence[str], position: str) -> tuple[str, Any]:
    """The typed elements of a sequence, and why rcl would refuse it.

    Flow and block share this: `- a` and a break and `- b` reads exactly as `[a, b]`, so
    every rule below is the one flow already had. Measured one value per process, humble:
    `- 1` and a break and `- 2` is `[1, 2]`, `- true` and a break and `- false` is
    `[True, False]`, `- a` and a break and `- 1` is refused for mixed types, `- []` sets
    nothing and `- [1,2]` and a break and `- [3]` is the nested blind spot.

    Two elements that both have something wrong with them are answered by the worse one, and
    an rcl refusal is worse than a blind spot: a sequence whose first element nests and whose
    third element is a mapping never starts, so the mapping is what it is refused for. Mixed
    types are read off the elements before the first nested one, because rcl checks the type
    of each element against the one before it and a nested element ends that run: `[a, no, []]`,
    `[a, 1, []]`, `[true, a, [1]]` and `[1, a, [2]]` all take `rcl_init` down over the two
    plain elements that disagree, while `[[], a, no]`, `[[1], a, no]`, `[1, [], a]` and
    `[1, 2, [a]]` all start and are the nested blind spot, whatever the elements after the
    nesting spell. The two disagreeing elements are what `[.inf, 'q', [[2]]]` is refused for,
    measured, even though its third element nests twice.
    """
    answers = [_node(text, position) for text in texts]
    refusals = [refusal for refusal, _ in answers if refusal and refusal != SETS_NOTHING]
    for group in (RCL_REFUSALS, UNMODELLED_REFUSALS):
        for refusal in refusals:
            if refusal in group:
                return refusal, None
    if all(refusal == SETS_NOTHING for refusal, _ in answers):
        return SETS_NOTHING, None
    nests = [_is_sequence(text, position) for text in texts]
    typed = []
    for (_, value), nested in zip(answers, nests, strict=True):
        if nested:
            break
        typed.append(value)
    if _mixed(typed):
        return SEQUENCE_MIXED_TYPES, None
    if any(nests) or refusals:
        return SEQUENCE_NESTED, None
    return "", [value for _, value in answers]


def _is_sequence(text: str, position: str) -> bool:
    stripped = _scan(text)[0].strip(_YAML_BLANK)
    if stripped[:1] == "[":
        return True
    return position != _FLOW and _opens_sequence(_lines(text))


def _flow_node(stripped: str) -> tuple[str, Any]:
    if stripped[0] in "]}":
        return VALUE_UNPARSABLE, None
    if stripped[0] == "{":
        return VALUE_UNSUPPORTED, None
    if not stripped.endswith("]"):
        return VALUE_UNPARSABLE, None
    items = _elements(stripped[1:-1])
    if items is None:
        return VALUE_UNPARSABLE, None
    return _sequence(items, _FLOW)


def _quoted_node(stripped: str) -> tuple[str, Any]:
    """A node opening with a quote. It has to close, and nothing but blanks and comments may
    follow it: measured one value per process, `'a' b`, `'a'b`, `'a' 'b'`, `'a b'` and a
    break and `c`, and `'a'` and a comment and a break and `b` all take `rcl_init` down,
    while `'a'#c`, `'a'  ` and `'a'` and two breaks are the scalar."""
    style = stripped[0]
    end = _quote_end(stripped)
    if end < 0:
        return VALUE_UNPARSABLE, None
    rest = stripped[end + 1 :].strip(_YAML_BLANK)
    if rest:
        if rest[0] == ":" and rest[1:2] in ("", " ", "\t"):
            return VALUE_UNSUPPORTED, None
        return VALUE_UNPARSABLE, None
    return "", _unquote(_fold(stripped[1:end], style == '"'), style)


def _quote_end(stripped: str) -> int:
    """The index of the quote that closes the one the text opens with, or -1."""
    style = stripped[0]
    index = 1
    size = len(stripped)
    while index < size:
        char = stripped[index]
        if style == '"' and char == "\\":
            index += 2
            continue
        if char == style:
            if style == "'" and stripped[index + 1 : index + 2] == "'":
                index += 2
                continue
            return index
        index += 1
    return -1


def _opener(raw: str, stripped: str, position: str) -> str:
    """The refusal the first character of a node carries, or an empty string.

    One value per process, humble, every row measured in all three positions as `<text>`,
    `[<text>]` and `- <text>`:

    * `,`, `@` and a backtick: "Error parsing an event" everywhere, `,a` and `[@a]` and
      `- ,a` alike. A comma anywhere but the front is a plain scalar, `a,` and `a, b` run.
    * `*`: "Unknown YAML event" everywhere, `*x`, `[*x]`, `- *x`, and `[&x a, *x]` too, so an
      alias never reaches a parameter whether or not its anchor exists.
    A plain scalar is also refused inside a flow collection when it carries `[` or `{` after
    its first character, where the same text is a string as a whole value and as an item:
    `[a[1]]`, `[a{1}]`, `[1 [2]]` and `[a` and a break and `[1]]` take `rcl_init` down, while
    `a[b]` and `- a [1]` run. Quotes and dashes inside one are content in every position,
    `[a "d"]`, `[a 'd']` and `[a - b]` all run.

    * `?` and `:`: a mapping event everywhere they open a node with a blank after them. Two
      spellings differ by position and are measured: `?a` is the plain scalar `?a` as a whole
      value and as an item but `[?a]` refuses, and `:a` is the plain scalar `:a` as a whole
      value and as an item while `[:a]` refuses with "Error parsing an event".
    * `- ` inside a flow collection: "Error parsing an event", `[- a]`, `[- ]`, `[-\t]`,
      `[ - ]`, `[a, - ]` and `[- , a]`, while `[-]` and `[a, -]` are the one character
      string `-`. The blank is read off the element before its trailing blanks are dropped,
      because a dash that is the whole of a stripped element still opened a block sequence.
    * `|` and `>` inside a flow collection: "Error parsing an event", `[|]`, `[| a]`, `[>]`.
    * `%` anywhere but column 0: "Error parsing an event", `[%a]`, `- %a` and `' %a'`, while
      `%YAML 1.1` at column 0 is a directive and is `value_unmodelled`.
    """
    head = stripped[0]
    blank = stripped[1:2] in ("", " ", "\t")
    if head in ",@`":
        return VALUE_UNPARSABLE
    if head == "*":
        return VALUE_UNSUPPORTED
    if head == "?":
        return VALUE_UNSUPPORTED if blank or position == _FLOW else ""
    if head == ":":
        if position == _FLOW:
            return VALUE_UNPARSABLE
        return VALUE_UNSUPPORTED if blank else ""
    if head == "-" and position == _FLOW:
        tail = _scan(raw)[0].lstrip(_YAML_BLANK)[1:2]
        return VALUE_UNPARSABLE if tail in (" ", "\t") else ""
    if head == "%":
        return VALUE_UNMODELLED if position == _TOP and raw[:1] == "%" else VALUE_UNPARSABLE
    if head in "&!":
        return VALUE_UNMODELLED
    if head in "|>":
        return VALUE_UNPARSABLE if position == _FLOW else VALUE_UNMODELLED
    return ""


def _mapping(lines: Sequence[str], position: str) -> str:
    """The refusal a `key:` carries. On the first line of the node rcl reaches the mapping
    event and prints "Unknown YAML event", on a later line the scalar has already started and
    libyaml prints "Error parsing an event": `a: 1` is the first, `a` and a break and `b: 1`
    is the second. A colon with no blank after it is content, `a:1`, `:a` and `- a :b`."""
    first = True
    for line in lines:
        if _blank_line(line):
            continue
        if _has_key(_scan(line)[0]):
            return VALUE_UNSUPPORTED if first or position == _FLOW else VALUE_UNPARSABLE
        first = False
    return ""


def _has_key(text: str) -> bool:
    for index, char in enumerate(text):
        if char == ":" and text[index + 1 : index + 2] in ("", " ", "\t", "\r", "\n"):
            return True
    return False


def _document_marker(raw: str) -> bool:
    """Whether a line of the value is `---` or `...` at column 0. Measured one value per
    process: `--- a` is the string `a`, `a` and a break and `---` and a break and `b` is the
    string `b`, `- a` and a break and `---` and a break and `- b` is `[b]`, and `---` alone
    takes `rcl_init` down. Indented or inside a collection the same three characters are a
    plain scalar and are taken, `[---]`, `- ---`, `x---` and `...a`."""
    for line in _BREAK.split(raw):
        if line[:3] in ("---", "...") and line[3:4] in ("", " ", "\t"):
            return True
    return False


def _lines(text: str) -> list[str]:
    """The logical lines of a node. A break inside a quoted scalar or an open flow collection
    continues the line rather than ending it, measured one value per process: `- 'a` and a
    break and `b'` and a break and `- c` is the two element array `['a b', 'c']`, and
    `- [1,` and a break and `2]` and a break and `- [3]` is one nested sequence."""
    out: list[str] = []
    current: list[str] = []
    depth = 0
    quote = ""
    comment = False
    content = False
    opener = True
    index = 0
    size = len(text)
    while index < size:
        char = text[index]
        index += 1
        if char in "\r\n":
            if char == "\r" and text[index : index + 1] == "\n":
                index += 1
            comment = False
            opener = True
            if quote or depth:
                current.append("\n")
                continue
            out.append("".join(current))
            current = []
            continue
        current.append(char)
        if comment:
            continue
        if quote:
            if char == "\\" and quote == '"' and index < size:
                current.append(text[index])
                index += 1
            elif char == quote:
                if quote == "'" and text[index : index + 1] == "'":
                    current.append("'")
                    index += 1
                else:
                    quote = ""
                    content = True
            continue
        if char == "#":
            comment = True
        elif char in ("'", '"') and not content:
            quote = char
        elif char in "[{":
            depth += 1
            content = False
        elif char in "]}" and depth:
            depth -= 1
            content = True
        elif char == ",":
            content = False
        elif char not in _YAML_BLANK:
            item = opener and not depth and char == "-" and _blank_next(text, index)
            content = content or not item
        if char not in _YAML_BLANK:
            opener = False
    out.append("".join(current))
    return out


def _blank_next(text: str, index: int) -> bool:
    return text[index : index + 1] in ("", " ", "\t", "\r", "\n")


def _blank_line(line: str) -> bool:
    return not _scan(line)[0].strip(_YAML_BLANK)


def _opens_sequence(lines: Sequence[str]) -> bool:
    for line in lines:
        if _blank_line(line):
            continue
        body = line.lstrip(" ")
        return body[:1] == "-" and body[1:2] in ("", " ", "\t")
    return False


def _block_sequence(lines: Sequence[str]) -> tuple[str, Any]:
    """A block sequence, read as the array rcl builds from it.

    The indent of the first item is the sequence, every later line decides by its own indent,
    and each was measured one value per process on humble: a line at that indent opening with
    a dash and a blank is the next item (`- a` and a break and `- b` is `['a', 'b']`), a more
    indented line is more of the item it follows and folds into it (`- a` and a break and
    `  b` is `['a b']`, and so is `- a` and a break and `  - b` with the dash inside the
    scalar, `['a - b']`), a blank or comment only line belongs to no item and is skipped
    (`- a` and a break and `#c` and a break and `- b` is `['a', 'b']`), and any other line
    takes `rcl_init` down (`- a` and a break and `- b` and a break and `c`, and `- a` and a
    break and `-b`, and ` - a` and a break and `- b`). A tab indents nothing: `- a` and a
    break and a tab and `- b`, `-` and a tab and `a`, and `- ` and a tab and `a` are all
    refused, while a tab inside an item is content, `- a` and a tab and `b` is `['a\\tb']`.
    An item with nothing in it is `value_empty`. The item keeps the column its content
    started at, because the indent inside it is measured against that column and not against
    the left margin: `- - z` and a break and `  []` takes `rcl_init` down, where the `[]` sits
    at the indent of the inner sequence and opens no item, while `- a` and a break and `  []`
    is the one element array `['a []']`, where the same line is more of a plain scalar.
    """
    items: list[list[str]] = []
    indent = -1
    for line in lines:
        if _blank_line(line):
            if items:
                items[-1].append(line)
            continue
        lead = line[: len(line) - len(line.lstrip(" \t"))]
        if "\t" in lead:
            return VALUE_UNPARSABLE, None
        column = len(lead)
        rest = line[column:]
        if indent < 0:
            indent = column
        if column == indent and rest[:1] == "-" and rest[1:2] in ("", " ", "\t"):
            head = " " * (column + 1) + rest[1:]
            if "\t" in head[: len(head) - len(head.lstrip(" \t"))]:
                return VALUE_UNPARSABLE, None
            items.append([head])
            continue
        if column > indent and items:
            items[-1].append(line)
            continue
        return VALUE_UNPARSABLE, None
    return _sequence(["\n".join(part) for part in items], _ITEM)


def _fold(text: str, escapes: bool) -> str:
    """A scalar spanning more than one line, folded the way libyaml folds it.

    The blanks next to a break are dropped, one break between two lines becomes one
    space, and a run of N breaks becomes N-1 line feeds, so a blank line is a line feed. A
    line holding nothing but blanks is a blank line and the breaks at the two ends of the
    scalar are dropped. `\\r\\n` and a bare `\\r` are one break like `\\n`. Measured one value per
    process on humble: `a` and a break and `b` is `a b`, `a` and three breaks and `b` is `a`
    and two line feeds and `b`, `a  ` and a break and `  b` is `a b`, `a` and a
    break and ` b` and a blank line and ` c` is `a b` and a line feed and `c`, and `1` and a
    break and `2` is the string `1 2` and not an integer, because the fold runs first.

    The same fold runs inside a quoted scalar (`'a` and a break and `b'` is `a b`) and inside
    a flow collection (`[a` and a break and `b, c]` is `['a b', 'c']`). `escapes` is the one
    thing only a double quoted scalar has: a backslash at the end of a line eats that break,
    measured, `"a\\` and a break and ` b"` is `ab` and `"a\\` and two breaks and `b"` is `a`
    and a line feed and `b`, while an even run of backslashes is content and folds normally,
    `"a\\\\` and a break and `b"` is `a\\ b`.
    """
    lines = _BREAK.split(text)
    if len(lines) == 1:
        return text
    last = len(lines) - 1
    trimmed = []
    for index, line in enumerate(lines):
        head = line.lstrip(" \t") if index else line
        trimmed.append(head if index == last else head.rstrip(" \t"))
    lines = trimmed
    while lines and not lines[0]:
        lines.pop(0)
    while lines and not lines[-1]:
        lines.pop()
    if not lines:
        return ""
    out = lines[0]
    breaks = 0
    for line in lines[1:]:
        if not line:
            breaks += 1
            continue
        eaten = escapes and _escaped_break(out)
        if eaten:
            out = out[:-1]
        if breaks:
            out += "\n" * breaks
        elif not eaten:
            out += " "
        out += line
        breaks = 0
    return out


def _escaped_break(text: str) -> bool:
    return (len(text) - len(text.rstrip("\\"))) % 2 == 1


def _typed(text: str) -> Any:
    """A folded plain scalar, typed. The padding around it is gone before this is reached,
    because that is the scalar libyaml hands rcl: measured on humble, one value per process,
    `-p k0:=' a '`, `-p k0:='a '` and `-p k0:='a\\t'` all come back the one character string
    `a`."""
    if text in _TRUE:
        return True
    if text in _FALSE:
        return False
    number = _integer(text)
    if number is None:
        number = _double(text)
    return text if number is None else number


def _elements(body: str) -> list[str] | None:
    """The elements of a bracketed value, or None when yaml refuses the flow sequence.

    A comma after the last element is allowed and adds no element. An empty element
    anywhere else, and a bracket that does not close, are parse errors, which is what rcl
    does. An element keeps the blanks it was written with, because the blank after a `-`
    decides whether the element opened a block sequence and `[- ]` refuses while `[-]` runs.
    """
    if not _balanced(body):
        return None
    parts = _split_list(body)
    if len(parts) > 1 and not parts[-1].strip(_YAML_BLANK):
        parts = parts[:-1]
    return None if any(not part.strip(_YAML_BLANK) for part in parts) else parts


def _balanced(body: str) -> bool:
    depth = 0
    quote = ""
    content = False
    for char in body:
        if quote:
            quote = "" if char == quote else quote
        elif char in ("'", '"') and not content:
            quote = char
        elif char in "[{":
            depth += 1
            content = False
        elif char in "]}":
            depth -= 1
            content = True
            if depth < 0:
                return False
        elif char == ",":
            content = False
        elif char not in _YAML_BLANK:
            content = True
    return depth == 0 and not quote


_YAML_BLANK = " \t\n\r"
"""What libyaml lets a document be padded with. A tab pads and does not indent, measured one
value per process: `'a\\t'` comes back the one character string `a` and `'[]\\t'` comes back
`NOT_SET`, while a tab in the leading run refuses the argument list, which
`_refused_whitespace` answers before this is reached."""


def _scan(text: str) -> tuple[str, bool]:
    """Walk the text once, removing comments and answering whether one split the value.

    A comment opens at a `#` the scanner meets between tokens and runs to the end of that
    line. Between tokens means four places, all measured on humble one value per process
    against a node with `automatically_declare_parameters_from_overrides`: the start of a
    line (`'#c'` and a newline and `'1'` is 1), after a blank (`'a #c'` and `'a\t#c'` are
    both `a`), after a flow collection closed (`'[1]#c'` is the array `[1]`, `'[]#c'` sets
    nothing, `'[[1],[2]]#c'` is the nested case) and after a quoted scalar closed
    (`"'a'#c"`, `'"a"#c'` and `"'a b'#c"` are all the scalar without the comment). A comma
    inside a flow collection is the same position, `'[1,#c'` and a newline and `'2]'` is
    `[1, 2]`.

    Everywhere else the marker is one more character of a plain scalar: `'a#c'`, `'a]#c'`,
    `'1#c'`, `'true#c'`, `'-#c'`, `'-1#c'`, `':#c'`, `'a:#c'` and `'1;#c'` all come back
    with the marker in the string, and inside a quoted scalar it is content, so `"'a #c'"`
    is `a #c` and `"[ 'a #c' ]"` is a one element array.

    The second answer is the refusal the first one causes. A comment terminates the plain
    scalar it follows, so content after it at the top level is a second document node and
    `rcl_init` fails. Measured, ten values, process never started: `'a #c'` and a newline
    and `'b'`, the same with `'\r'`, with a leading blank on the second line, with a
    second comment between them, with two more lines, and with `'[1]'`, `'[1,2]'` or `'1'`
    in front of the comment. The same shapes without the marker start and fold into one
    scalar (`'a'` newline `'b'` is the string `a b`), so the marker is the whole cause. A
    comment with nothing but blanks or further comments after it is not this: `'a #c'` and
    a newline and `'#d'` is `a`, and a comment before the only value is not this either.
    """
    out: list[str] = []
    size = len(text)
    depth = 0
    quote = ""
    plain = False
    before = False
    cut = False
    split = False
    index = 0
    while index < size:
        char = text[index]
        index += 1
        if quote:
            out.append(char)
            if char == "\\" and quote == '"' and index < size:
                out.append(text[index])
                index += 1
            elif char == quote:
                if quote == "'" and text[index : index + 1] == "'":
                    out.append("'")
                    index += 1
                else:
                    quote = ""
            continue
        if char == "#" and not plain:
            while index < size and text[index] not in "\r\n":
                index += 1
            cut = cut or before
            continue
        out.append(char)
        if char in " \t\r\n":
            plain = False
            continue
        opening = char in "[{" and (depth or not plain)
        closing = char in "]}" and depth
        boundary = char == "," and depth
        content = before
        if opening:
            split = split or cut
            before = False
            cut = False
        elif depth and boundary:
            before = False
            cut = False
        elif closing:
            before = True
            cut = False
        else:
            split = split or cut
            before = True
        if char in ("'", '"') and not content:
            quote = char
        elif opening:
            depth += 1
            plain = False
        elif closing:
            depth -= 1
            plain = False
        elif boundary:
            plain = False
        else:
            plain = True
    return "".join(out), split


def _refused_whitespace(raw: str) -> str:
    """Why libyaml refuses a value over whitespace alone, or an empty string.

    Measured on humble, one value per process: a tab in the leading run refuses the argument
    list ("Error parsing an event"), because a tab cannot indent, while `a\\tb`, `a\\t` and
    `[1,\\t2]` all start and keep their value. A vertical tab or a form feed refuses
    anywhere, including in the middle of a word.
    """
    lead = raw[: len(raw) - len(raw.lstrip(" \t\n\r"))]
    if "\t" in lead or "\v" in raw or "\f" in raw:
        return VALUE_UNPARSABLE
    return ""


def _mixed(values: Sequence[Any]) -> bool:
    """Whether rcl would call this sequence more than one type.

    `bool` is checked before `int` because it is a subclass of it, and rcl keeps the two
    apart: `[true, 1]` is refused. A nested element never reaches here, `_flow` answers
    `sequence_nested` first.
    """
    return len({_type_of(value) for value in values}) > 1


def _type_of(value: Any) -> str:
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int):
        return "integer"
    if isinstance(value, float):
        return "double"
    if isinstance(value, str):
        return "string"
    return type(value).__name__


def _split_list(body: str) -> list[str]:
    """Top level commas of a bracketed value, quotes respected, nesting respected. Each
    element is returned as written, blanks included, and `_elements` decides emptiness."""
    out: list[str] = []
    current: list[str] = []
    depth = 0
    quote = ""
    content = False
    for char in body:
        if quote:
            current.append(char)
            if char == quote:
                quote = ""
            continue
        if char in ("'", '"') and not content:
            quote = char
            content = True
            current.append(char)
            continue
        if char in "[{":
            depth += 1
            content = False
        elif char in "]}":
            depth -= 1
            content = True
        elif char == ",":
            content = False
        elif char not in _YAML_BLANK:
            content = True
        if char == "," and depth == 0:
            out.append("".join(current))
            current = []
            continue
        current.append(char)
    tail = "".join(current)
    if tail.strip(_YAML_BLANK) or out:
        out.append(tail)
    return out


__all__ = [
    "ASSIGN",
    "BARE_FLAGS",
    "INPUT_ASSIGN",
    "INPUT_FILE",
    "RCL_REFUSALS",
    "ROSIO_REFUSALS",
    "ROS_ARGS",
    "SEPARATOR",
    "SEQUENCE_MIXED_TYPES",
    "SEQUENCE_NESTED",
    "SETS_NOTHING",
    "UNMODELLED_REFUSALS",
    "VALUE_EMPTY",
    "VALUE_FLAGS",
    "VALUE_UNMODELLED",
    "VALUE_UNPARSABLE",
    "VALUE_UNSUPPORTED",
    "apply_argv",
    "apply_ros_args",
    "available",
    "param_inputs",
    "parse_ros_args",
    "parse_value",
    "refuse_value",
    "sets_nothing",
    "typed_assignments",
]
