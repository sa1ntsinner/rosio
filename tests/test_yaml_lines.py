"""The parameters file door, against the scalar set rcl reads.

Every expectation here was measured on humble one value per process: a parameters file
handed to `rclpy.init(args=['prog', '--ros-args', '--params-file', <file>])` on a node with
`automatically_declare_parameters_from_overrides=True`, then `Node.get_parameter('k0').type_`
and `.value` read back. Forty three values, forty two of which start a node.
"""

from __future__ import annotations

import pytest

from rosio.errors import CollectorError
from rosio.resolve import yaml_lines


def value_of(text: str):
    doc = yaml_lines.load(f"probe_node:\n  ros__parameters:\n    k0: {text}\n")
    block = yaml_lines.param_blocks(doc)[0]
    return yaml_lines.leaves(block.values)[0].value


# text, what the node reported, one row per measured value
MEASURED_BOOL = [
    ("y", True),
    ("Y", True),
    ("n", False),
    ("N", False),
    ("yes", True),
    ("Yes", True),
    ("YES", True),
    ("no", False),
    ("No", False),
    ("NO", False),
    ("true", True),
    ("True", True),
    ("TRUE", True),
    ("false", False),
    ("False", False),
    ("FALSE", False),
    ("on", True),
    ("On", True),
    ("ON", True),
    ("off", False),
    ("Off", False),
    ("OFF", False),
]

MEASURED_STRING = [
    ("null", "null"),
    ("Null", "Null"),
    ("NULL", "NULL"),
    ("~", "~"),
    ("TrUe", "TrUe"),
    ("ye", "ye"),
    ("Ye", "Ye"),
    ("nul", "nul"),
    ("None", "None"),
    ("yES", "yES"),
    ("oN", "oN"),
    ("'y'", "y"),
    ('"null"', "null"),
    ("'~'", "~"),
]


@pytest.mark.parametrize(("text", "expected"), MEASURED_BOOL)
def test_a_scalar_the_node_reads_as_a_bool_is_a_bool(text, expected):
    value = value_of(text)
    assert value is expected
    assert yaml_lines.type_name(value) == "bool"


@pytest.mark.parametrize(("text", "expected"), MEASURED_STRING)
def test_a_scalar_the_node_reads_as_a_string_is_a_string(text, expected):
    """rcl has no null spelling, so `null` and `~` arrive as their own text."""
    value = value_of(text)
    assert value == expected
    assert yaml_lines.type_name(value) == "str"


def test_a_number_keeps_its_type():
    assert value_of("7") == 7
    assert yaml_lines.type_name(value_of("7")) == "int"
    assert value_of("1.5") == 1.5
    assert value_of("5e3") == 5000.0
    assert yaml_lines.type_name(value_of("5e3")) == "float"


def test_a_sequence_resolves_its_members_the_same_way():
    """`[y, N]` is a bool array at the node and `[null, x]` is a string array."""
    assert value_of("[y, N]") == [True, False]
    assert value_of("[null, x]") == ["null", "x"]


def test_an_empty_value_is_the_one_null_left():
    """rcl refuses to start under an empty value, `No value at line 3`, so there is no node
    value to agree with and the yaml 1.1 null stays."""
    assert value_of("") is None
    assert yaml_lines.type_name(None) == "null"


def test_a_key_is_a_name_and_never_a_typed_value():
    """The reference stack spells an initial pose `x:` and `y:`, and rcl reads both as
    names. Typing the key would rename that parameter to `true`."""
    doc = yaml_lines.load("probe_node:\n  ros__parameters:\n    y: 1\n    ~: 2\n    on: 3\n")
    block = yaml_lines.param_blocks(doc)[0]
    assert sorted(leaf.name for leaf in yaml_lines.leaves(block.values)) == ["on", "y", "~"]


def test_a_key_that_is_not_a_name_refuses_the_file():
    with pytest.raises(CollectorError):
        yaml_lines.load("? [1, 2]\n: v\n", path="hostile.yaml")


def test_lines_still_land_on_the_key():
    doc = yaml_lines.load("probe_node:\n  ros__parameters:\n    a: y\n    b: null\n")
    assert doc.line(("probe_node", "ros__parameters", "a")) == 3
    assert doc.line(("probe_node", "ros__parameters", "b")) == 4


def test_a_hostile_document_raises_a_collector_error():
    with pytest.raises(CollectorError):
        yaml_lines.load("a: [unclosed\n", path="bad.yaml")


def test_a_document_without_the_param_key_is_refused():
    doc = yaml_lines.load("a:\n  b: y\n")
    assert not doc.is_parameter_file()
    assert yaml_lines.refuse_reason(doc.data) == yaml_lines.REFUSE_NO_PARAM_KEY


def test_a_byte_that_is_not_utf8_leaves_as_a_collector_error(tmp_path):
    """It used to leave as a UnicodeDecodeError, which no caller catches, and one such byte
    ended the whole parameter pass for the profile."""
    path = tmp_path / "bad.yaml"
    path.write_bytes(b"node:\n  ros__parameters:\n    k: \xff\n")
    with pytest.raises(CollectorError):
        yaml_lines.load_file(str(path))
