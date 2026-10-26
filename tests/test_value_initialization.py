import pytest

from rosio.collect import cpp_code as cpp

pytestmark = pytest.mark.cpp


@pytest.mark.parametrize(
    ("declaration", "expected"),
    [("std::vector<double> value;", []), ("std::string value;", "")],
)
def test_default_constructed_local_has_its_language_defined_value(declaration, expected):
    unit = cpp.parse_text(
        "void Node::configure() {" + declaration + 'declare_parameter("setting", value);}',
        path="value.cpp",
    )
    cpp.analyse([unit])
    report = cpp.file_report(unit)
    row = report["parameters"][0]
    assert row["has_default"]
    assert row["default"] == expected
    assert report["unresolved"] == []


def test_an_uninitialized_primitive_is_still_unknown():
    unit = cpp.parse_text(
        'void Node::configure() { double value; declare_parameter("setting", value);}',
        path="primitive.cpp",
    )
    cpp.analyse([unit])
    report = cpp.file_report(unit)
    assert report["parameters"][0]["default"] is None
    assert report["parameters"][0]["default_state"] == "unresolved"
