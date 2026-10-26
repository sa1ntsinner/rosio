"""Argv parsing, against the fixture argv and the shapes that break naive parsers."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from rosio.resolve import records, ros_args

FIXTURE_ARGV = [
    "python3",
    "/ws/src/lx_bringup/scripts/crop_cloud.py",
    "--ros-args",
    "-p",
    "rate_hz:=20.0",
    "-p",
    "decimate_rings:=2",
    "-r",
    "input:=/points_raw",
    "__node:=cropper",
    "__ns:=/prep",
    "--params-file",
    "/ws/src/lx_bringup/config/crop.yaml",
]


def test_fixture_argv_is_fully_parsed():
    parsed = ros_args.parse_ros_args(FIXTURE_ARGV)
    assert parsed.node_name == "cropper"
    assert parsed.namespace == "/prep"
    assert parsed.remaps == (("input", "/points_raw"),)
    assert parsed.assignments == (("rate_hz", "20.0"), ("decimate_rings", "2"))
    assert parsed.params_files == ("/ws/src/lx_bringup/config/crop.yaml",)
    assert parsed.leftover == ("python3", "/ws/src/lx_bringup/scripts/crop_cloud.py")


def test_fixture_argv_matches_the_expected_chain(fixtures_dir: Path):
    """The expected chain says the file listed after the two -p flags wins rate_hz."""
    expected = json.loads(
        (fixtures_dir / "expected" / "params" / "argv_chains.json").read_text(encoding="utf-8")
    )
    owner = expected["owner"]
    parsed = ros_args.parse_ros_args(FIXTURE_ARGV)
    assert owner == f"{parsed.namespace}/{parsed.node_name}"
    typed = dict(ros_args.typed_assignments(parsed))
    for row in expected["parameters"]:
        assign = [
            candidate
            for candidate in row["candidates"]
            if candidate["mechanism"] == "ros_args_assign"
        ]
        if not assign:
            continue
        assert typed[row["name"]] == assign[0]["value_as_yaml"]
    inputs = parsed.inputs
    assert inputs[-1][0] == ros_args.INPUT_FILE, "the file is the latest input, so it wins"


def test_inputs_keep_the_true_order():
    argv = ["--ros-args", "--params-file", "/a.yaml", "-p", "rate:=5", "--params-file", "/b.yaml"]
    wanted = (
        (ros_args.INPUT_FILE, "/a.yaml"),
        (ros_args.INPUT_ASSIGN, ("rate", "5")),
        (ros_args.INPUT_FILE, "/b.yaml"),
    )
    assert ros_args.parse_ros_args(argv).inputs == wanted
    assert ros_args.param_inputs(argv) == wanted


def test_inputs_carry_every_flat_entry_and_nothing_else():
    """The two flat tuples and the ordered one are one parse, never two."""
    parsed = ros_args.parse_ros_args(FIXTURE_ARGV)
    files = [payload for kind, payload in parsed.inputs if kind == ros_args.INPUT_FILE]
    assigns = [payload for kind, payload in parsed.inputs if kind == ros_args.INPUT_ASSIGN]
    assert tuple(files) == parsed.params_files
    assert tuple(assigns) == parsed.assignments
    assert len(parsed.inputs) == len(files) + len(assigns)


def test_legacy_params_assignment_is_an_input():
    parsed = ros_args.parse_ros_args(["--ros-args", "__params:=/legacy.yaml"])
    assert parsed.inputs == ((ros_args.INPUT_FILE, "/legacy.yaml"),)


@pytest.mark.parametrize(
    ("text", "value"),
    [
        ("20", 20),
        ("20.0", 20.0),
        ("-3", -3),
        ("1e3", 1000.0),
        ("true", True),
        ("False", False),
        ("'20'", "20"),
        ('"true"', "true"),
        ("[1, 2, 3]", [1, 2, 3]),
        ("[a, b]", ["a", "b"]),
        ("[]", "[]"),
        ("", ""),
        ("/ouster/points", "/ouster/points"),
        ("None", "None"),
        ("yes", True),
        ("off", False),
        ("y", True),
        ("N", False),
        ("ON", True),
        ("TrUe", "TrUe"),
        ("yEs", "yEs"),
        ("[true, no]", [True, False]),
    ],
)
def test_value_types_are_exact(text, value):
    parsed = ros_args.parse_value(text)
    assert parsed == value
    assert type(parsed) is type(value)


def test_the_boolean_spellings_are_an_exact_case_list():
    """Measured one value per process against an rclpy node on humble.

    The eleven per side come back `Type.BOOL`, and any other casing comes back
    `Type.STRING`, so a lowercase comparison would type `TrUe` as a boolean the node never
    sees.
    """
    for text in ("y", "Y", "yes", "Yes", "YES", "true", "True", "TRUE", "on", "On", "ON"):
        assert ros_args.parse_value(text) is True
    for text in ("n", "N", "no", "No", "NO", "false", "False", "FALSE", "off", "Off", "OFF"):
        assert ros_args.parse_value(text) is False
    for text in ("TrUe", "tRue", "yEs", "yES", "nO", "oFF", "oN", "ye", "Ye", "yeS"):
        assert ros_args.parse_value(text) == text


def test_a_mixed_boolean_list_is_one_boolean_array():
    assert ros_args.parse_value("[true, no]") == [True, False]
    assert [type(item) for item in ros_args.parse_value("[y, N]")] == [bool, bool]


def test_int_never_becomes_float():
    assert repr(ros_args.parse_value("20")) == "20"
    assert repr(ros_args.parse_value("20.0")) == "20.0"


def test_long_flags_accept_the_joined_spelling():
    parsed = ros_args.parse_ros_args(
        ["--ros-args", "--params-file=/a.yaml", "--param", "gain:=2", "--remap", "in:=/out"]
    )
    assert parsed.params_files == ("/a.yaml",)
    assert parsed.assignments == (("gain", "2"),)
    assert parsed.remaps == (("in", "/out"),)


def test_legacy_assignments_are_identity_not_parameters():
    parsed = ros_args.parse_ros_args(
        ["--ros-args", "__name:=probe", "__ns:=/lab", "__params:=/legacy.yaml"]
    )
    assert parsed.node_name == "probe"
    assert parsed.namespace == "/lab"
    assert parsed.params_files == ("/legacy.yaml",)
    assert parsed.assignments == ()
    assert parsed.remaps == ()


def test_node_scoped_rule_is_not_applied_as_identity():
    parsed = ros_args.parse_ros_args(["--ros-args", "-r", "other:__node:=stranger"])
    assert parsed.node_name is None
    assert parsed.remaps == (("other:__node", "stranger"),)


def test_private_and_absolute_remaps_survive():
    parsed = ros_args.parse_ros_args(
        ["--ros-args", "-r", "~/status:=/health", "-r", "/tf:=tf", "-r", "cmd_vel:=cmd_vel_nav"]
    )
    assert parsed.remaps == (
        ("~/status", "/health"),
        ("/tf", "tf"),
        ("cmd_vel", "cmd_vel_nav"),
    )


def test_log_level_and_enclave():
    parsed = ros_args.parse_ros_args(
        ["--ros-args", "--log-level", "debug", "--enclave", "/robot", "--disable-rosout-logs"]
    )
    assert parsed.log_level == "debug"
    assert parsed.enclave == "/robot"
    assert parsed.leftover == ("--disable-rosout-logs",)


def test_double_dash_closes_the_region():
    parsed = ros_args.parse_ros_args(
        ["node", "--ros-args", "-p", "a:=1", "--", "positional", "-p", "b:=2"]
    )
    assert parsed.assignments == (("a", "1"),)
    assert parsed.leftover == ("node", "positional", "-p", "b:=2")


def test_two_ros_args_regions_accumulate():
    parsed = ros_args.parse_ros_args(
        [
            "python3",
            "script.py",
            "--ros-args",
            "-p",
            "a:=1",
            "--",
            "user",
            "--ros-args",
            "-p",
            "b:=2",
            "-r",
            "__ns:=/second",
        ]
    )
    assert parsed.assignments == (("a", "1"), ("b", "2"))
    assert parsed.namespace == "/second"
    assert parsed.leftover == ("python3", "script.py", "user")


def test_malformed_argv_never_raises_and_keeps_the_leftover():
    parsed = ros_args.parse_ros_args(
        ["--ros-args", "-p", "--ros-args", "-p", "nocolon", "-r", "onlyleft:=", "--params-file"]
    )
    assert parsed.assignments == ()
    assert parsed.remaps == ()
    assert parsed.params_files == ()
    assert parsed.leftover == ("-p", "-p", "nocolon", "-r", "onlyleft:=", "--params-file")


def test_a_nameless_assignment_never_raises():
    """`-p :=5` has no parameter name at all, and the shared normaliser refuses one."""
    for text in (":=5", ".:=5", "..:=5", " :=5"):
        parsed = ros_args.parse_ros_args(["--ros-args", "-p", text])
        assert parsed.assignments == ()
        assert parsed.leftover == ("-p", text)


def test_empty_argv():
    parsed = ros_args.parse_ros_args([])
    assert parsed == records.RosArgs()


def test_assignments_outside_a_region_stay_leftover():
    parsed = ros_args.parse_ros_args(["__ns:=/nope", "-p", "a:=1"])
    assert parsed.namespace is None
    assert parsed.assignments == ()
    assert parsed.leftover == ("__ns:=/nope", "-p", "a:=1")


def test_dotted_parameter_name_is_normalised():
    parsed = ros_args.parse_ros_args(["--ros-args", "-p", "FootprintApproach.footprint_topic:=/fp"])
    assert parsed.assignments == (("FootprintApproach.footprint_topic", "/fp"),)


# ---------------------------------------------------------------------------
# folding into a node


def _node() -> records.ResolvedNode:
    return records.ResolvedNode(
        pkg="lx_bringup",
        executable="crop_cloud.py",
        namespace="",
        form="process",
        site=records.Site(
            path="/ws/src/lx_bringup/launch/exec_process.launch.py",
            locator="ExecuteProcess(cropper_proc)",
        ),
    )


def test_apply_argv_builds_the_chain_in_argv_order():
    node = ros_args.apply_argv(_node(), FIXTURE_ARGV)
    assert node.name == "cropper"
    assert node.name_source == "argv"
    assert node.namespace == "/prep"
    assert [item.kind for item in node.params] == [
        "ros_args_assign",
        "ros_args_assign",
        "ros_args_file",
    ]
    assert [item.order for item in node.params] == [0, 1, 2]
    assert node.params[0].values == (("rate_hz", "20.0"),)
    assert node.params[0].site is not None
    assert node.params[0].site.locator == "ExecuteProcess(cropper_proc).cmd[-p rate_hz]"
    assert node.params[2].path == "/ws/src/lx_bringup/config/crop.yaml"
    assert [(item.frm, item.to, item.scope) for item in node.remaps] == [
        ("input", "/points_raw", "node")
    ]


def test_apply_ros_args_reads_the_order_off_the_parse():
    """The parse carries the order, so folding a parse and folding the argv agree."""
    argv = ["--ros-args", "--params-file", "/a.yaml", "-p", "gain:=2"]
    node = ros_args.apply_ros_args(_node(), ros_args.parse_ros_args(argv))
    assert [(item.order, item.kind) for item in node.params] == [
        (0, "ros_args_file"),
        (1, "ros_args_assign"),
    ]
    assert node.params == ros_args.apply_argv(_node(), argv).params


def test_apply_keeps_existing_inputs_and_their_order():
    base = records.ResolvedNode(
        pkg="lx_bringup",
        executable="probe",
        namespace="/",
        params=(records.ParamSource(order=0, kind="repo_file", path="/ws/config/base.yaml"),),
    )
    node = ros_args.apply_argv(base, ["--ros-args", "-p", "gain:=2"])
    assert [(item.order, item.kind) for item in node.params] == [
        (0, "repo_file"),
        (1, "ros_args_assign"),
    ]


def test_apply_records_log_level_and_leftover_on_attrs():
    node = ros_args.apply_argv(
        _node(), ["python3", "--ros-args", "--log-level", "warn", "--enclave", "/e"]
    )
    assert node.attrs["log_level"] == "warn"
    assert node.attrs["enclave"] == "/e"
    assert node.attrs["ros_args_leftover"] == ["python3"]


def test_apply_leaves_the_name_alone_when_argv_states_none():
    node = ros_args.apply_argv(_node(), ["--ros-args", "-p", "a:=1"])
    assert node.name is None
    assert node.name_source == "unknown"


def test_parsing_is_repeatable():
    first = ros_args.parse_ros_args(FIXTURE_ARGV)
    second = ros_args.parse_ros_args(list(FIXTURE_ARGV))
    assert first == second


# ---------------------------------------------------------------------------
# the scalar syntax rcl accepts, measured against a node on humble


MEASURED_SCALARS = [
    ("0x10", 16),
    ("0X1A", 26),
    ("-0x10", -16),
    ("+0x10", 16),
    ("017", 15),
    ("007", 7),
    ("0", 0),
    ("09", 9.0),
    ("0b101", "0b101"),
    ("0o17", "0o17"),
    ("1_000", "1_000"),
    ("9223372036854775807", 9223372036854775807),
    ("9223372036854775808", 9.223372036854776e18),
    (".inf", float("inf")),
    ("-.Inf", float("-inf")),
    (".INF", float("inf")),
    ("+.inf", float("inf")),
    (".iNf", ".iNf"),
    ("-.NAN", "-.NAN"),
    ("inf", float("inf")),
    ("infinity", float("inf")),
    ("-inf", float("-inf")),
    ("0x1p3", 8.0),
    ("1e400", "1e400"),
    ("1e-320", "1e-320"),
    ("1.2.3", "1.2.3"),
    (".5", 0.5),
    ("5.", 5.0),
    (".", "."),
    ("0.0", 0.0),
    ('"a\\nb"', "a\nb"),
    ("'a\\nb'", "a\\nb"),
    ('"\\u0041"', "A"),
    ('"\\t"', "\t"),
    ('"a\\"b"', 'a"b'),
    ("'it''s'", "it's"),
    ("'0x10'", "0x10"),
    ('"true"', "true"),
    ("null", "null"),
    ("~", "~"),
]
"""Every row was read back off a node started with `-p k:=<text>` on humble. The type is
part of the row: an int that becomes a float is a phantom drift on its own."""


@pytest.mark.parametrize("text,value", MEASURED_SCALARS)
def test_a_scalar_is_typed_the_way_rcl_types_it(text, value):
    parsed = ros_args.parse_value(text)
    assert parsed == value
    assert type(parsed) is type(value)


def test_the_two_yaml_spellings_of_nan_are_floats():
    for text in (".nan", ".NaN", ".NAN", "nan", "NAN", "-nan"):
        parsed = ros_args.parse_value(text)
        assert isinstance(parsed, float) and parsed != parsed, text


def test_a_hex_value_inside_a_list_is_typed_too():
    assert ros_args.parse_value("[0x10, 017]") == [16, 15]


MEASURED_SEQUENCES = [
    ("[1, 2]", [1, 2]),
    ("[0x10, 2]", [16, 2]),
    ("[1, 017]", [1, 15]),
    ("[-1, +2]", [-1, 2]),
    ("[a, b]", ["a", "b"]),
    ("[a, 'b']", ["a", "b"]),
    ('["a", "b"]', ["a", "b"]),
    ("[~, a]", ["~", "a"]),
    ("[true, no]", [True, False]),
    ("[y, N]", [True, False]),
    ("[true, TRUE]", [True, True]),
    ("[1e5, 2.0]", [100000.0, 2.0]),
    ("[inf, 1.5]", [float("inf"), 1.5]),
    ("[1,]", [1]),
]
"""Bracketed values an rclpy node on humble started with, one per process. `[1,]` is the
trailing comma yaml allows and it is one element, not two. `[]` and `[ ]` are not in here:
the node starts and the parameter is not set at all, which no value in this table can say."""

MEASURED_REFUSALS = [
    ("[1, x]", ros_args.SEQUENCE_MIXED_TYPES),
    ("[true, 1]", ros_args.SEQUENCE_MIXED_TYPES),
    ("[1, 2.0]", ros_args.SEQUENCE_MIXED_TYPES),
    ("[2.0, 1]", ros_args.SEQUENCE_MIXED_TYPES),
    ("[1e5, 2]", ros_args.SEQUENCE_MIXED_TYPES),
    ("[.inf, 1]", ros_args.SEQUENCE_MIXED_TYPES),
    ("[1, .inf]", ros_args.SEQUENCE_MIXED_TYPES),
    ("[a, 1.5]", ros_args.SEQUENCE_MIXED_TYPES),
    ("[x, 1]", ros_args.SEQUENCE_MIXED_TYPES),
    ("[1, '2']", ros_args.SEQUENCE_MIXED_TYPES),
    ("['a', 1]", ros_args.SEQUENCE_MIXED_TYPES),
    ('[1, "x"]', ros_args.SEQUENCE_MIXED_TYPES),
    ("[1, 2, x]", ros_args.SEQUENCE_MIXED_TYPES),
    ("[true, a]", ros_args.SEQUENCE_MIXED_TYPES),
    ("[1, TrUe]", ros_args.SEQUENCE_MIXED_TYPES),
    ("[TrUe, 1]", ros_args.SEQUENCE_MIXED_TYPES),
    ("[null, 1]", ros_args.SEQUENCE_MIXED_TYPES),
    ("[1e400, 2]", ros_args.SEQUENCE_MIXED_TYPES),
    ("[,]", ros_args.VALUE_UNPARSABLE),
    ("[, 1]", ros_args.VALUE_UNPARSABLE),
    ("[1, , 2]", ros_args.VALUE_UNPARSABLE),
    ("[ , ]", ros_args.VALUE_UNPARSABLE),
    ("[", ros_args.VALUE_UNPARSABLE),
    ("[a", ros_args.VALUE_UNPARSABLE),
    ("[1, 2", ros_args.VALUE_UNPARSABLE),
    ("[1, 2}", ros_args.VALUE_UNPARSABLE),
    ("[[1, 2]", ros_args.VALUE_UNPARSABLE),
    ("[1, 2]]", ros_args.VALUE_UNPARSABLE),
    ("[]]", ros_args.VALUE_UNPARSABLE),
    ("[a]x", ros_args.VALUE_UNPARSABLE),
    ("]", ros_args.VALUE_UNPARSABLE),
    ("]a", ros_args.VALUE_UNPARSABLE),
    ("}", ros_args.VALUE_UNPARSABLE),
    ("}a", ros_args.VALUE_UNPARSABLE),
    ("{a: 1}", ros_args.VALUE_UNSUPPORTED),
    ("{a: 1", ros_args.VALUE_UNSUPPORTED),
    ("[{a: 1}]", ros_args.VALUE_UNSUPPORTED),
]
"""Every one of these took `rcl_init` down with it, so the node never started. The three
groups are the three messages rcl prints: "Sequence should be of same type", "Error parsing
an event" and "Unknown YAML event". Measured one value per process, because the failure is
the whole argument list."""


@pytest.mark.parametrize("text,value", MEASURED_SEQUENCES)
def test_a_sequence_rcl_takes_is_typed_the_way_rcl_types_it(text, value):
    assert ros_args.refuse_value(text) == ""
    parsed = ros_args.parse_value(text)
    assert parsed == value
    assert [type(item) for item in parsed] == [type(item) for item in value]


@pytest.mark.parametrize("text,reason", MEASURED_REFUSALS)
def test_a_sequence_rcl_refuses_is_refused_here_too(text, reason):
    """Recording a value for a process that cannot start is a wrong fact, so the value is
    never typed and the raw text is what comes back for a caller to quote."""
    assert ros_args.refuse_value(text) == reason
    assert ros_args.parse_value(text) == text


def test_a_bracket_that_is_not_the_first_character_is_text():
    """rcl falls back to string for a plain scalar whatever it spells, so a bracket that is
    not the first character is text. All of these started a node on humble."""
    for text in ("1, 2]", "a[b]", "x[a]", "a]", "a}", "a{b", "/topic[0]", "foo{bar}", "1,2"):
        assert ros_args.refuse_value(text) == "", text
        assert ros_args.parse_value(text) == text, text
    for text in ("TrUe", "1e400", "os_sensor", "0o17", '"[1, 2"', "'[1, 2'"):
        assert ros_args.refuse_value(text) == "", text


def test_whitespace_alone_refuses_the_argument_list_or_empties_the_document():
    """Whitespace is two answers and neither of them is a value, measured one value per
    process on humble against an rclpy node.

    A leading tab refuses the argument list, "Error parsing an event", because a tab cannot
    indent a document, and a vertical tab or a form feed refuses anywhere. A tab that is not
    in the leading run is text: `a\\tb` starts and keeps both characters. Spaces, line feeds
    and carriage returns alone start the node and leave the parameter `Type.NOT_SET`.

    Outside the leading run a tab also pads, which is why it is in `_YAML_BLANK`: `[]` with
    a trailing tab comes back `Type.NOT_SET` and used to be recorded as the empty list.
    """
    for text in ("\t", " \t ", "\ta", "a\vb", "\f", "a\x0cb"):
        assert ros_args.refuse_value(text) == ros_args.VALUE_UNPARSABLE, repr(text)
        assert not ros_args.sets_nothing(text), repr(text)
    for text in ("a\tb", "a\t", "[1,\t2]", "[1]\t"):
        assert ros_args.refuse_value(text) == "", repr(text)
        assert not ros_args.sets_nothing(text), repr(text)
    for text in (" ", "   ", "\n", " \n ", "\r"):
        assert ros_args.refuse_value(text) == "", repr(text)
        assert ros_args.sets_nothing(text), repr(text)
    for text in ("[]\t", "[] \t", "[]\t#c", "[] #c\t"):
        assert ros_args.refuse_value(text) == "", repr(text)
        assert ros_args.sets_nothing(text), repr(text)


def test_a_value_that_sets_nothing_names_no_value():
    """An input that starts the node and leaves the parameter with no value at all.

    Measured on humble, one value per process, reading `type_` back from an rclpy node with
    `automatically_declare_parameters_from_overrides`. Every text on the first list came back
    `Type.NOT_SET` with value `None`: the parameter is declared and never initialised, so
    naming the text as its value states a value no node holds. A quoted scalar always sets
    something, and text beside a comment sets that text.
    """
    for text in (" ", "   ", "\n", "\r", "#", "#c", " #c", " # ", "#[1]"):
        assert ros_args.sets_nothing(text), repr(text)
    for text in ("[]", "[ ]", " [] ", "[]  ", "[] #c", "[[]]", "[[], []]"):
        assert ros_args.sets_nothing(text), repr(text)
    for text in ("' '", "''", '""', "a #c", "[[],1]", "0", "[0]", "a#c"):
        assert not ros_args.sets_nothing(text), repr(text)
    assert not ros_args.sets_nothing("")
    assert ros_args.refuse_value("") == ros_args.VALUE_UNPARSABLE
    for text in (" ", "#c", "[]", "[[]]"):
        assert ros_args.refuse_value(text) == "", repr(text)
        assert ros_args.parse_value(text) == text, repr(text)


def test_a_value_with_no_text_at_all_is_refused():
    """`-p k0:=` opens no flow collection and rcl still refuses it: measured on humble,
    `rcl_init` ends with "Couldn't parse parameter override rule: '-p k0:='" and the process
    never starts. `-p k0:=' '` is the neighbour that does start, with the parameter left
    unset, so the two are not one answer."""
    assert ros_args.refuse_value("") == ros_args.VALUE_UNPARSABLE
    assert ros_args.refuse_value(" ") == ""
    assert ros_args.refuse_value("  ") == ""


NESTED_RUNTIME = (
    ("[[1, 2], [3]]", "INTEGER_ARRAY", [3]),
    ("[[1, 2], [3, 4]]", "INTEGER_ARRAY", [3, 4]),
    ("[[1, 2], [a]]", "INTEGER_ARRAY", [1, 2]),
    ("[[1], [a]]", "INTEGER_ARRAY", [1]),
    ("[a, [b]]", "STRING_ARRAY", ["b"]),
    ("[[a], b]", "STRING", "b"),
    ("[[1], 2]", "INTEGER", 2),
    ("[1, [2]]", "INTEGER_ARRAY", [2]),
    ("[x, [1, 2]]", "INTEGER_ARRAY", [1, 2]),
    ("[[1, 2], 3.0]", "DOUBLE", 3.0),
    ("[[1], [2.0]]", "INTEGER_ARRAY", [1]),
    ("[[a], [1]]", "INTEGER_ARRAY", [1]),
    ("[[1], [true]]", "BOOL_ARRAY", [True]),
    ("[[1], a, 2]", "INTEGER", 2),
    ("[[1], 2, a]", "INTEGER", 2),
    ("[1, [2], 3]", "INTEGER", 3),
    ("[[1, 2], [3], [4, 5, 6]]", "INTEGER_ARRAY", [4, 5, 6]),
    ("[[], 1]", "INTEGER", 1),
)
"""What a node on humble held after one `-p k0:=<text>`, one value per process, read back as
`type_` and `value`. Kept here because the rule is not the one the text states and not the
one the shape suggests: it is not the last inner sequence, `[[1, 2], [a]]` is `[1, 2]`."""


def test_a_nested_sequence_is_refused_because_the_value_is_not_in_the_text():
    """rcl takes a nested sequence and rosio refuses it, measured twice.

    Against a node, the values in `NESTED_RUNTIME`. Against `rcl_parse_yaml_value` directly,
    the same texts leave more than one slot of `rcl_variant_t` filled at once, against that
    header's own "Only one pointer in this struct will store the value": `[[1, 2], [a]]`
    leaves the integer array `[1, 2]` and the string array `[a]`, `[[a], b]` leaves the
    string `b` and the string array `[a]`. Which filled slot becomes the parameter is the
    order the client reads the slots in, and only rclpy was measured, so no value here is a
    property of the command line. The process does start, which is why this refusal is not
    one of the rcl three.
    """
    for text, _type, _value in NESTED_RUNTIME:
        assert ros_args.refuse_value(text) == ros_args.SEQUENCE_NESTED, text
        assert ros_args.parse_value(text) == text, text
    assert ros_args.SEQUENCE_NESTED in ros_args.ROSIO_REFUSALS
    assert ros_args.SEQUENCE_NESTED not in ros_args.RCL_REFUSALS
    # An inner refusal is the stronger fact: that process never starts at all.
    assert ros_args.refuse_value("[[1, a]]") == ros_args.SEQUENCE_MIXED_TYPES
    assert ros_args.refuse_value("[[1, 2], [3, a]]") == ros_args.SEQUENCE_MIXED_TYPES
    assert ros_args.refuse_value("[[{a: 1}]]") == ros_args.VALUE_UNSUPPORTED


def test_the_text_comes_back_exactly_when_no_value_is_named():
    """`parse_value` hands back the text for every text the other two speak for, so a caller
    that asks them first can never record a typed value for one of those. One walk answers
    all three."""
    texts = [
        "",
        " ",
        "#c",
        "[]",
        "[[]]",
        "\t",
        "20",
        "20.0",
        "true",
        "'20'",
        "os_sensor",
        "[1, 2]",
        "[a, b]",
        "[1, x]",
        "[, 1]",
        "{a: 1}",
        "[[1, 2], [3]]",
        "[[], 1]",
        "a[b]",
        "[1,]",
    ]
    for text in texts:
        spoken_for = bool(ros_args.refuse_value(text)) or ros_args.sets_nothing(text)
        if spoken_for:
            assert ros_args.parse_value(text) == text, text
    assert ros_args.parse_value("20") == 20
    assert ros_args.parse_value("[1, 2]") == [1, 2]
    assert ros_args.parse_value("'20'") == "20"


def test_an_argv_name_is_the_name_that_runs_whatever_the_source_stated():
    """Measured on humble, one process per case. rcl keeps the first `__node` rule it is
    handed, and launch_ros appends its own after the arguments a launch file passed, so a
    launch file that states `name="stated"` and passes `-r __node:=from_argv` in its own
    arguments starts a node called `from_argv`. The command line built for that case is
    `[--ros-args, -r, __node:=from_argv, --ros-args, -r, __node:=stated]` and the node
    reported `from_argv`."""
    stated = records.ResolvedNode(
        pkg="lx_bringup",
        executable="plain_node",
        namespace="/",
        form="node",
        name="stated",
        name_source="literal",
    )
    folded = ros_args.apply_argv(stated, ["--ros-args", "-r", "__node:=from_argv"])
    assert (folded.name, folded.name_source) == ("from_argv", "argv")

    bare = records.ResolvedNode(
        pkg="lx_bringup",
        executable="plain_node",
        namespace="/",
        form="process",
        name=None,
        name_source="unknown",
    )
    folded = ros_args.apply_argv(bare, ["--ros-args", "-r", "__node:=from_argv"])
    assert (folded.name, folded.name_source) == ("from_argv", "argv")


def test_the_first_name_rule_of_an_argument_list_is_the_one_that_runs():
    """AGENTS section 9 point 5, measured on humble one process per case.

    `-r __node:=first -r __node:=second` runs as `first`, the same pair split across two
    `--ros-args` groups also runs as `first`, `-r __ns:=/a -r __ns:=/b` lands in `/a`, and
    `-r chatter:=/first -r chatter:=/second` publishes on `/first`. A repeated `-p` is the
    other rule and the last one wins: `-p k:=1 -p k:=2` reads 2.
    """
    first = ros_args.parse_ros_args(["--ros-args", "-r", "__node:=first", "-r", "__node:=second"])
    assert first.node_name == "first"

    split = ros_args.parse_ros_args(
        ["--ros-args", "-r", "__node:=first", "--ros-args", "-r", "__node:=second"]
    )
    assert split.node_name == "first"

    spaces = ros_args.parse_ros_args(["--ros-args", "-r", "__ns:=/a", "-r", "__ns:=/b"])
    assert spaces.namespace == "/a"

    both = ros_args.parse_ros_args(
        ["--ros-args", "-r", "__node:=one", "-r", "__ns:=/x", "-r", "__node:=two", "-r", "__ns:=/y"]
    )
    assert (both.node_name, both.namespace) == ("one", "/x")

    values = ros_args.parse_ros_args(["--ros-args", "-p", "k:=1", "-p", "k:=2"])
    assert values.assignments == (("k", "1"), ("k", "2"))
    assert ros_args.typed_assignments(values)[-1] == ("k", 2)


def test_a_plain_scalar_is_recorded_without_the_padding_around_it():
    """Measured on humble, one value per process, read back off the node.

    `-p k0:=' a '`, `-p k0:='a '`, `-p k0:=' a'` and `-p k0:='a\t'` all come back the one
    character string `a`, so recording the padding states a value no node holds. A tab inside
    the scalar is part of it, `a\tb` comes back whole, and a quoted scalar keeps whatever it
    spells.
    """
    for text in (" a ", "a ", " a", "a  ", "a\t"):
        assert ros_args.parse_value(text) == "a", text
    assert ros_args.parse_value("a\tb") == "a\tb"
    assert ros_args.parse_value("' '") == " "
    assert ros_args.parse_value(" 1 ") == 1


def test_a_comment_ends_at_its_own_line_and_not_at_the_end_of_the_value():
    """`sets_nothing` reads the yaml document the text spells, and a comment is one line of
    it. Measured on humble, one value per process: `-p k0:='#c'` leaves the parameter
    `Type.NOT_SET`, and `-p k0:=` with `#c`, a newline and `1` comes back `INTEGER 1`."""
    for text in ("#", "#c", " #c", " # ", "#[1]", "\n#c\n", "[] #c"):
        assert ros_args.sets_nothing(text), text
    for text in ("#c\n1", "#c\n[1]", "#c\r1", "a #c"):
        assert not ros_args.sets_nothing(text), text


def test_a_comment_opens_at_the_four_positions_the_runtime_opens_one_at():
    """Every row here was measured on humble, one value per process, against an rclpy node
    with `automatically_declare_parameters_from_overrides`.

    libyaml opens a comment at a marker it meets between tokens, which is the start of a
    line, after a blank, after a flow collection closed, after a quoted scalar closed and
    after a comma inside a flow collection. Reading only the first two kept the comment as
    part of the value, which was twelve wrong values, eight false refusals and two missed
    refusals in a table of 190.
    """
    taken = {
        "a #c": "a",
        "1 #c": 1,
        "a\t#c": "a",
        "a #c d": "a",
        "a #": "a",
        "1 # ": 1,
        "#c\n1": 1,
        "#c\r1": 1,
        "1\n#c": 1,
        "a\n#c": "a",
        "1 #c\n": 1,
        "a #c\n#d": "a",
        "[1] #c": [1],
        "[1,2] # x": [1, 2],
        "[1]#c": [1],
        "[1]\t#c": [1],
        "[1]\n\n#c": [1],
        "[a,b]#c": ["a", "b"],
        "[1.5]#c": [1.5],
        "[true]#c": [True],
        "'a'#c": "a",
        '"a"#c': "a",
        "'a b'#c": "a b",
        "''#c": "",
        "'a' #c": "a",
        "[1, #c\n2]": [1, 2],
        "[1,#c\n2]": [1, 2],
        "[a,#c\nb]": ["a", "b"],
        "[ #c\n1]": [1],
        "[#c\n1,2]": [1, 2],
        "[1,2,#c\n3]": [1, 2, 3],
        "[1,2 #c\n]": [1, 2],
        "[1] #c]": [1],
    }
    for text, value in taken.items():
        assert ros_args.refuse_value(text) == "", repr(text)
        assert not ros_args.sets_nothing(text), repr(text)
        assert ros_args.parse_value(text) == value, repr(text)
    for text in ("[]#c", "[ ]#c", "[[]]#c", "[[], []]#c", "[] #c"):
        assert ros_args.sets_nothing(text), repr(text)
    for text in ("[a #c]", "[1 #c]", "[1,2 #c]", "['a' #c]"):
        assert ros_args.refuse_value(text) == ros_args.VALUE_UNPARSABLE, repr(text)


def test_a_marker_inside_a_scalar_is_one_more_character_of_it():
    """The other side of the same rule, measured the same way.

    A marker the scanner meets in the middle of a plain scalar is text, whatever character
    sits in front of it, and a marker inside a quoted scalar is content. `']'` at the top
    level closes no flow collection, so `'a]#c'` is a plain scalar and keeps the marker while
    `'[1]#c'` does not.
    """
    kept = {
        "a#c": "a#c",
        "a]#c": "a]#c",
        "1#c": "1#c",
        "true#c": "true#c",
        "-#c": "-#c",
        "-1#c": "-1#c",
        ":#c": ":#c",
        "a:#c": "a:#c",
        "1;#c": "1;#c",
        "a# c": "a# c",
        "'a#c'": "a#c",
        "'a #c'": "a #c",
        '"a #c"': "a #c",
        "'#c'": "#c",
        "[ 'a #c' ]": ["a #c"],
        "['a#c',b]": ["a#c", "b"],
    }
    for text, value in kept.items():
        assert ros_args.refuse_value(text) == "", repr(text)
        assert ros_args.parse_value(text) == value, repr(text)


def test_a_comment_that_splits_a_top_level_value_refuses_the_argument_list():
    """A comment terminates the scalar it follows, so a second value after it is a second
    document node and `rcl_init` fails.

    Measured on humble, one value per process, ten shapes, none of which started. The same
    shapes without the marker do start and fold into one scalar, `'a'` and a newline and
    `'b'` coming back as the string `a b`, so the marker is the whole cause. A comment with
    nothing but blanks or further comments after it is not this, and neither is a comment in
    front of the only value or one inside a flow collection.
    """
    for text in (
        "a #c\nb",
        "1 #c\n2",
        "a\n#c\nb",
        "a #\n1",
        "a #c\r2",
        "a #c\n b",
        "a #c\n2 #d",
        "a #c\n[1]",
        "1 #c\n#d\n2",
        "1 #c\n2\n3",
        "[1] #c\n2",
        "[1] #c\n[2]",
        "[1] #c\n\n2",
        "[1,2] #c\n3",
    ):
        assert ros_args.refuse_value(text) == ros_args.VALUE_UNPARSABLE, repr(text)
        assert ros_args.parse_value(text) == text, repr(text)
        assert not ros_args.sets_nothing(text), repr(text)
    for text in ("a #c\n#d", "1 #c\n", "[1] #\n", "[1]\n\n#c", "[1, #c\n2]"):
        assert ros_args.refuse_value(text) == "", repr(text)


def test_the_process_table_fact_keeps_the_order_argv_states(tmp_path):
    """Argv order is the precedence rcl applied, so it is data and not presentation.
    Sorting the parameters files, the assignments and the remaps made the fact contradict
    the argv preserved beside it: `z.yaml` before `a.yaml` came back the other way round,
    and so did `gain:=2` before `gain:=1`."""
    from rosio import ids, model
    from rosio.collect import CollectContext, proc
    from rosio.config import Config, ProfileSpec, RootSpec
    from rosio.workspace import Workspace

    (tmp_path / "src" / "fx_pkg").mkdir(parents=True)
    (tmp_path / "src" / "fx_pkg" / "package.xml").write_text(
        "<package><name>fx_pkg</name></package>", encoding="utf-8"
    )
    ws = Workspace.discover([RootSpec("src", str(tmp_path / "src"))])
    cfg = Config(
        roots=tuple(RootSpec(kind=spec.kind, path=spec.path) for spec in ws.roots),
        profiles=(ProfileSpec(slug="live"),),
    )
    argv = (
        "/opt/ros/humble/lib/demo_nodes_cpp/talker",
        "--ros-args",
        "-r",
        "__node:=talker",
        "--params-file",
        "/ws/z.yaml",
        "--params-file",
        "/ws/a.yaml",
        "-p",
        "gain:=2",
        "-p",
        "gain_other:=1",
        "-r",
        "z_in:=/z",
        "-r",
        "a_in:=/a",
    )
    builder = model.AtlasBuilder()
    ctx = CollectContext(ws=ws, cfg=cfg, builder=builder, profile="")
    proc.ProcCollector(processes=[proc.parse(argv, pid=7)]).collect(ctx)
    values = builder.atlas.entities[ids.build_node("/talker")].facts[0].values
    assert [item.rsplit("/", 1)[-1] for item in values["params_files"]] == ["z.yaml", "a.yaml"]
    assert [pair[0] for pair in values["assignments"]] == ["gain", "gain_other"]
    assert [pair[0] for pair in values["remaps"]] == ["z_in", "a_in"]


def test_a_block_sequence_is_the_array_the_runtime_builds():
    """Every row measured on humble, one value per process, against an rclpy node with
    `automatically_declare_parameters_from_overrides`, reading `type_` and `value` back.

    A dash and a blank open a block sequence and it reads exactly as the flow sequence of the
    same items, typing included. The indent of the first item is the sequence: a line at that
    indent opening with a dash is the next item, a more indented line is more of the item it
    follows and folds into it, a blank or comment only line belongs to no item, and anything
    else takes `rcl_init` down. The item keeps the column it started at, so the indent inside
    it is measured against that column: `- - z` and a break and `  []` refuses while `- a` and
    a break and `  []` is `['a []']`.
    """
    arrays = {
        "- a": ["a"],
        "- 1": [1],
        "- 1.5": [1.5],
        "- true": [True],
        "- no": [False],
        "- 0x10": [16],
        "- .nan\n- 1.0": None,
        "- a\n- b": ["a", "b"],
        "- 1\n- 2\n- 3": [1, 2, 3],
        "- 1\n- 0x10": [1, 16],
        "- a\n- b\n- c": ["a", "b", "c"],
        "-  a": ["a"],
        "- 'a b'": ["a b"],
        '- "a"': ["a"],
        "- ''": [""],
        "- ' a '": [" a "],
        "- 'a\nb'\n- c": ["a b", "c"],
        "- a\n  b": ["a b"],
        "- a\n - b": ["a - b"],
        "- a\n    b\n  c": ["a b c"],
        "- a\n\n- b": ["a", "b"],
        "- a\n \n- b": ["a", "b"],
        "- a\n- b\n": ["a", "b"],
        "\n- a": ["a"],
        "  - a": ["a"],
        " - a\n - b": ["a", "b"],
        "#c\n- a": ["a"],
        "- a\n#c\n- b": ["a", "b"],
        "- a #c": ["a"],
        "- a #c\n- b": ["a", "b"],
        "- 1 #c\n- 2": [1, 2],
        "- a,b": ["a,b"],
        "- a b": ["a b"],
        "- a\tb": ["a\tb"],
        "- a]": ["a]"],
        "- -1": [-1],
        "- -a": ["-a"],
        "- ---": ["---"],
        "- ...": ["..."],
        "- null": ["null"],
        "- ~": ["~"],
        "- a :b": ["a :b"],
        "- a:1": ["a:1"],
        "- :a": [":a"],
        "- a\n  []": ["a []"],
        "- a\n\n  []": ["a\n[]"],
    }
    for text, value in arrays.items():
        if value is None:
            continue
        assert ros_args.refuse_value(text) == "", repr(text)
        assert not ros_args.sets_nothing(text), repr(text)
        assert ros_args.parse_value(text) == value, repr(text)
    assert ros_args.parse_value("- .nan\n- 1.0")[1] == 1.0
    assert ros_args.parse_value("- 0x10")[0] == 16

    for text in ("- []", "- [ ]", "- []\n- []"):
        assert ros_args.sets_nothing(text), repr(text)
        assert ros_args.refuse_value(text) == "", repr(text)

    for text in ("- - a", "- [1, 2]", "- a\n- - b", "- a\n- [1]", "- []\n- a"):
        assert ros_args.refuse_value(text) == ros_args.SEQUENCE_NESTED, repr(text)
        assert ros_args.parse_value(text) == text, repr(text)

    assert ros_args.refuse_value("- a\n- 1") == ros_args.SEQUENCE_MIXED_TYPES
    assert ros_args.refuse_value("- 1\n- 2.0") == ros_args.SEQUENCE_MIXED_TYPES
    assert ros_args.refuse_value("- true\n- 1") == ros_args.SEQUENCE_MIXED_TYPES


def test_a_block_sequence_refuses_the_shapes_that_take_rcl_init_down():
    """The other half of the same measurement, one value per process, process never started.

    An item with nothing in it is the "No value at line N" message, a tab where an indent
    belongs and a line that is neither an item nor more of one are "Error parsing an event",
    and an item that is a mapping is "Unknown YAML event".
    """
    for text in ("-", "- ", "-  ", "-\n", "-\n- a", "- \n- a", "- a\n-", "- a\n- b\n- ", "- -"):
        assert ros_args.refuse_value(text) == ros_args.VALUE_EMPTY, repr(text)
        assert ros_args.parse_value(text) == text, repr(text)
        assert not ros_args.sets_nothing(text), repr(text)
    for text in (
        "-\ta",
        "- \ta",
        "-\t",
        "- a\n\t- b",
        "- a\n-b",
        "- a\n- b\nc",
        "- a\n- b\n\nc",
        " - a\n- b",
        "  - a\n - b",
        "- a #c\n  b",
        "- a\n  #c\n  b",
        "- a\n- b #c\nc",
        "- 'a' b",
        "- 'a\nb",
        "- [1]\n  x",
        "- - z\n  []",
        "- - z\n\n  x\n\n- [1]",
    ):
        assert ros_args.refuse_value(text) == ros_args.VALUE_UNPARSABLE, repr(text)
        assert ros_args.parse_value(text) == text, repr(text)
    for text in ("- a: 1", "- {a: 1}", "- : a", "- ?", "- *x", "- - : v"):
        assert ros_args.refuse_value(text) in (
            ros_args.VALUE_UNSUPPORTED,
            ros_args.VALUE_UNMODELLED,
        ), repr(text)
    assert ros_args.refuse_value("- a: 1") == ros_args.VALUE_UNSUPPORTED
    assert ros_args.refuse_value("- *x") == ros_args.VALUE_UNSUPPORTED


def test_a_scalar_over_more_than_one_line_folds_before_it_is_typed():
    """libyaml folds a plain or quoted scalar that spans lines, and rcl types what it is
    handed, so the fold happens before the type and `1` and a break and `2` is the string
    `1 2` and not an integer. Measured on humble, one value per process.

    The blanks next to a break are dropped, one break becomes one space, and a run of N
    breaks becomes N-1 line feeds, so a blank line is a line feed. A line of blanks is a
    blank line, `\\r\\n` and a bare `\\r` are one break, and the breaks at the two ends of the
    scalar are dropped.
    """
    folded = {
        "a\nb": "a b",
        "a\nb\nc": "a b c",
        "a\nb\nc\nd": "a b c d",
        "1\n2": "1 2",
        "1\n2\n3\n4": "1 2 3 4",
        "1\n2.0": "1 2.0",
        "0x10\n1": "0x10 1",
        ".nan\n1": ".nan 1",
        "true\nfalse": "true false",
        "a \nb": "a b",
        "a\n b": "a b",
        "a  \n  b": "a b",
        "a\nb ": "a b",
        "a\tb\nc": "a\tb c",
        "a\r\nb": "a b",
        "a\rb": "a b",
        "a\rb\rc": "a b c",
        "a\n\tb": "a b",
        "a\n\nb": "a\nb",
        "a\n\n\nb": "a\n\nb",
        "a\n\n\n\nb": "a\n\n\nb",
        "a\n \nb": "a\nb",
        "a\n\t\nb": "a\nb",
        "a\r\n\r\nb": "a\nb",
        "a\r\rb": "a\nb",
        "a\n\nb\nc": "a\nb c",
        "a\n b\n\n c": "a b\nc",
        "a\n\nb\n\nc": "a\nb\nc",
        "a\n": "a",
        "a\n\n": "a",
        "\na": "a",
        "\n\na": "a",
        "a\nb\n": "a b",
        "a\n[1]": "a [1]",
        "a\n{1}": "a {1}",
        "a\n,": "a ,",
        "a\n@b": "a @b",
        "a\n'b'": "a 'b'",
        "a\n- b": "a - b",
        "a\nb #c": "a b",
        "a\nb\n#c": "a b",
        "a#c\nb": "a#c b",
        "a\nb#c": "a b#c",
        "'a\nb'": "a b",
        "'a\nb\nc'": "a b c",
        "'a \n b'": "a b",
        "'a\n\nb'": "a\nb",
        '"a\nb"': "a b",
        '"a\n\nb"': "a\nb",
        '"a\\\nb"': "ab",
        '"a\\\n b"': "ab",
        '"a\\\n\nb"': "a\nb",
        '"a\\\\\nb"': "a\\ b",
        "[a\nb]": ["a b"],
        "[a\nb\nc]": ["a b c"],
        "[a\nb, c]": ["a b", "c"],
        "[a\n\nb]": ["a\nb"],
        "[1\n2]": ["1 2"],
        "['a\nb']": ["a b"],
        "[1,\n2]": [1, 2],
    }
    for text, value in folded.items():
        assert ros_args.refuse_value(text) == "", repr(text)
        assert not ros_args.sets_nothing(text), repr(text)
        assert ros_args.parse_value(text) == value, repr(text)
    assert isinstance(ros_args.parse_value("1\n2"), str)
    assert ros_args.parse_value("' a '") == " a "
    assert ros_args.parse_value("' '") == " "


def test_a_scalar_that_ended_and_a_value_that_follows_it_take_rcl_init_down():
    """A node that closed and more content after it are two document nodes, and `rcl_init`
    fails. Measured one value per process: a quoted scalar with anything but blanks and
    comments after it, a flow collection with content after it, and a `key:` on a later line
    of a plain scalar that already started.
    """
    for text in (
        "'a' b",
        "'a'b",
        "'a' 'b'",
        "'a b'\nc",
        "'a'\nb",
        "'a' #c\nb",
        "'a''b'\nc",
        "'a",
        '"a',
        "'",
        '"',
        "'a\nb",
        "[1]\n2",
        "[1]\n[2]",
        "[1] #c\nb",
        "a\nb: 1",
        "a\n:",
        "a #c\nb",
    ):
        assert ros_args.refuse_value(text) == ros_args.VALUE_UNPARSABLE, repr(text)
        assert ros_args.parse_value(text) == text, repr(text)
    for text in ("'a'#c", "'a'  ", "'a'\n", "'a'\n\n", "\n'a'", "[1, 2]\n", "\n[1, 2]"):
        assert ros_args.refuse_value(text) == "", repr(text)


def test_a_mapping_refuses_with_the_message_rcl_prints_for_it():
    """A mapping reaches rcl as an event it has no case for, "Unknown YAML event", the same
    message `{a: 1}` already had. A key on a later line of a scalar that already started is
    libyaml's own "Error parsing an event" instead, so the two carry different codes.
    Measured one value per process, every row here, process never started.
    """
    for text in (
        "a: 1",
        "a:",
        "a: a",
        "a: [1, 2]",
        "a: 1\nb: 2",
        "a: 1 #c",
        "a: 1\n#c",
        "a:\n b: 1",
        "'a': 1",
        '"a": 1',
        "a : 1",
        "1: 2",
        ": 1",
        ":",
        "::",
        "a: b: c",
        " a: 1",
        "? a",
        "?",
        "[a: 1]",
        "[? a]",
        "[?a]",
    ):
        assert ros_args.refuse_value(text) == ros_args.VALUE_UNSUPPORTED, repr(text)
        assert ros_args.parse_value(text) == text, repr(text)
    for text in ("a\nb: 1", "a\n:", "[: a]", "[:a]"):
        assert ros_args.refuse_value(text) == ros_args.VALUE_UNPARSABLE, repr(text)
    for text in ("a:1", ":a", "?a", "a,", "a, b", "a ,b", "/topic:=x"):
        assert ros_args.refuse_value(text) == "", repr(text)
        assert ros_args.parse_value(text) == text, repr(text)


def test_an_opener_this_parser_does_not_model_names_no_value_and_claims_nothing():
    """Five constructs are refused rather than modelled, and their code says nothing about
    whether the process started, because the same construct does both.

    Measured on humble, one value per process, in all three positions a node appears in: an
    anchor (`&x 1` runs as the integer 1, `&x` alone takes `rcl_init` down), a tag (`!!str 1`
    runs as the integer 1, so rcl drops the tag, `!t` alone takes it down), a block scalar
    header (`|` and a break and ` a` runs as the string `a`, `|` alone takes it down), a
    directive (`%YAML 1.1` and a break and `---` and a break and `a` runs as the string `a`,
    `%YAML 1.1` alone takes it down) and a document marker at column 0 (`--- a` runs as the
    string `a`, `a` and a break and `---` and a break and `b` runs as `b` because the last
    document wins, `---` alone takes it down). Naming a value for any of them means modelling
    chomping, tag resolution and multi document precedence, so none is named.
    """
    for text in (
        "&x 1",
        "&x a",
        "&x",
        "&x [1, 2]",
        "& x",
        "&x[1, 2]",
        "!!str 1",
        "!!str a",
        "!x a",
        "! a",
        "!<tag:x> a",
        "!t",
        "!!str",
        "!",
        "|",
        "|\n a",
        "|-\n a",
        "|1\n a",
        "| a",
        ">",
        ">\n a",
        "%YAML 1.1",
        "%YAML 1.1\n---\na",
        "%a",
        "---",
        "--- a",
        "--- ",
        "...",
        "...\na",
        "a\n---\nb",
        "a\n...",
        "- a\n---\n- b",
        "- &x 1",
        "- !!str 1",
        "- |\n a",
        "[&x 1]",
        "[!!str 1]",
    ):
        assert ros_args.refuse_value(text) == ros_args.VALUE_UNMODELLED, repr(text)
        assert ros_args.parse_value(text) == text, repr(text)
        assert not ros_args.sets_nothing(text), repr(text)
    assert ros_args.VALUE_UNMODELLED in ros_args.UNMODELLED_REFUSALS
    assert ros_args.VALUE_UNMODELLED not in ros_args.RCL_REFUSALS
    assert ros_args.VALUE_UNMODELLED not in ros_args.ROSIO_REFUSALS
    assert ros_args.VALUE_EMPTY in ros_args.RCL_REFUSALS
    for text in ("[---]", "[--- a]", "[...]", "- ---", "...a", "---a", "x---", "a\n%b"):
        assert ros_args.refuse_value(text) == "", repr(text)


def test_a_reserved_opener_refuses_wherever_a_node_starts():
    """`,`, `@` and a backtick open no node anywhere, `*` is an alias event rcl has no case
    for, and inside a flow collection `- `, `|` and `>` are errors too. Measured one value per
    process in all three positions, process never started.
    """
    for text in (",", ",a", ",a,", "@a", "@", "`a", "`", "[,a]", "[@a]", "[`a]", "- ,a", "- @a"):
        assert ros_args.refuse_value(text) == ros_args.VALUE_UNPARSABLE, repr(text)
    for text in ("*x", "*", "[*x]", "- *x", "[&x a, *x]"):
        assert ros_args.refuse_value(text) == ros_args.VALUE_UNSUPPORTED, repr(text)
    for text in ("[- a]", "[|]", "[| a]", "[>]", "[%a]"):
        assert ros_args.refuse_value(text) == ros_args.VALUE_UNPARSABLE, repr(text)
    for text in ("[-]", "a,", "a@b", "[a - b]", "[a 'd']", '[a "d"]'):
        assert ros_args.refuse_value(text) == "", repr(text)
    assert ros_args.parse_value("[-]") == ["-"]
    assert ros_args.parse_value('[a "d"]') == ['a "d"']
    for text in ("[a[1]]", "[a{1}]", "[1 [2]]", "[a\n[1]]", "[a #c [1]]"):
        assert ros_args.refuse_value(text) == ros_args.VALUE_UNPARSABLE, repr(text)
    assert ros_args.parse_value("a[b]") == "a[b]"
    assert ros_args.parse_value("- a [1]") == ["a [1]"]


def test_a_dash_in_a_flow_element_is_read_before_the_element_loses_its_blanks():
    """What decides is the blank after the dash, and an element that is nothing but a dash
    and blanks kept none of it once the split trimmed it. Measured one value per process,
    every row on the left never started and every row on the right did.
    """
    for text in (
        "[- ]",
        "[ - ]",
        "[-  ]",
        "[- \t]",
        "[-\t]",
        "[- \n]",
        "[- ,]",
        "[- ,a]",
        "[- , a]",
        "[a, - ]",
        "[a , - ]",
        "[-, - ]",
        "[a, b, - ]",
        "[\t- ]",
        "[- #c]",
        "[[- ]]",
        "[[- ], 1]",
        "- [- ]",
    ):
        assert ros_args.refuse_value(text) == ros_args.VALUE_UNPARSABLE, repr(text)
    assert ros_args.parse_value("[-]") == ["-"]
    assert ros_args.parse_value("[a, -]") == ["a", "-"]
    assert ros_args.parse_value("[--]") == ["--"]
    assert ros_args.parse_value("[-#c]") == ["-#c"]
    assert ros_args.parse_value("[1,]") == [1]
    assert ros_args.sets_nothing("[ ]")


def test_a_sequence_types_off_the_elements_before_the_first_nested_one():
    """rcl checks each element against the one before it and a nested element ends that run,
    so a sequence that both nests and mixes is refused only when the two elements that
    disagree are both in front of the nesting. Measured one value per process: `[a, no, []]`,
    `[a, 1, []]`, `[true, a, [1]]`, `[1, a, [2]]` and `[.inf, 'q', [[2]]]` never start, while
    `[[], a, no]`, `[[1], a, no]`, `[1, [], a]` and `[1, 2, [a]]` all start and hold a value
    that is not in the text.
    """
    mixed = ("[a, no, []]", "[a, 1, []]", "[true, a, [1]]", "[1, a, [2]]", "[.inf, 'q', [[2]]]")
    for text in mixed:
        assert ros_args.refuse_value(text) == ros_args.SEQUENCE_MIXED_TYPES, repr(text)
    for text in (
        "[[], a, no]",
        "[[1], a, no]",
        "[1, [], a]",
        "[1, 2, [a]]",
        "[[], no, 1]",
        "- []\n- x\n- no",
        "- [1]\n- x\n- no",
        "- true\n- false\n- - 1",
        "- 1\n- 2\n- - a",
    ):
        assert ros_args.refuse_value(text) == ros_args.SEQUENCE_NESTED, repr(text)
        assert ros_args.parse_value(text) == text, repr(text)
    assert ros_args.refuse_value("- true\n- a:1\n- - 1e400") == ros_args.SEQUENCE_MIXED_TYPES
