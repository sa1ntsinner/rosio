from pathlib import Path

import pytest

from rosio import ids, model, schema
from rosio.collect import CollectContext
from rosio.collect import cpp_code as cpp
from rosio.config import Config, RootSpec
from rosio.workspace import Workspace

pytestmark = pytest.mark.cpp

HEADER = """
namespace demo {
static std::map<std::string, int> policies = {{"zeta", 1}, {"alpha", 2}};
}
"""
SOURCE = """
#include "policies.hpp"
void demo::Node::configure() {
  declare_parameter("policy", policies.begin()->first);
}
"""


def read_default(header=HEADER, source=SOURCE, *, header_package="demo"):
    definitions = cpp.parse_text(
        header, path="policies.hpp", pkg=header_package, relpath="src/policies.hpp"
    )
    consumer = cpp.parse_text(source, path="main.cpp", pkg="demo", relpath="src/main.cpp")
    cpp.analyse([definitions, consumer])
    return cpp.file_report(consumer)["parameters"][0]


def test_included_map_default_uses_key_order_instead_of_initializer_order():
    row = read_default()
    assert row["default"] == "alpha"
    assert row["type"] == "string"


@pytest.mark.parametrize(
    "mutation",
    [
        'policies.erase("alpha");',
        'policies["aardvark"] = 3;',
        "change(policies);",
        "auto &alias = policies;",
        "auto *alias = &policies;",
        "auto alias = &policies;",
        "Holder holder(policies);",
        "sink << policies;",
    ],
)
def test_a_map_that_can_change_or_escape_does_not_supply_a_default(mutation):
    row = read_default(source=SOURCE + "void demo::change_policy() {" + mutation + "}")
    assert row["default"] is None
    assert row["default_state"] == "unresolved"


def test_a_same_package_header_that_was_not_included_is_not_visible():
    row = read_default(source=SOURCE.replace("policies.hpp", "unrelated.hpp"))
    assert row["default"] is None


def test_another_package_map_is_not_visible():
    row = read_default(header_package="other")
    assert row["default"] is None


def test_a_returned_map_reference_does_not_supply_a_default():
    row = read_default(source=SOURCE + "namespace demo { auto &expose() { return policies; } }")
    assert row["default"] is None


def test_a_custom_comparator_does_not_assume_lexical_order():
    row = read_default(
        header=HEADER.replace(
            "std::map<std::string, int>",
            "std::map<std::string, int, std::greater<std::string>>",
        )
    )
    assert row["default"] is None


def test_an_unknown_key_prevents_an_ordered_default():
    row = read_default(header=HEADER.replace('"zeta"', "runtime_key()"))
    assert row["default"] is None


def test_an_escaped_key_does_not_use_partial_string_decoding():
    row = read_default(header=HEADER.replace('"zeta"', r'"\x61"'))
    assert row["default"] is None


def test_a_mutable_map_with_external_linkage_does_not_supply_a_default():
    row = read_default(header=HEADER.replace("static ", ""))
    assert row["default"] is None


def test_included_map_default_cites_the_header_and_is_deterministic(tmp_path: Path):
    package = tmp_path / "src/demo"
    (package / "src").mkdir(parents=True)
    (package / "package.xml").write_text(
        '<package format="3"><name>demo</name><version>0.0.0</version>'
        '<description>Fixture</description><maintainer email="dev@example.com">Dev</maintainer>'
        "<license>Apache-2.0</license></package>"
    )
    (package / "src/policies.hpp").write_text(HEADER)
    (package / "src/main.cpp").write_text(SOURCE)
    roots = (RootSpec("src", str(tmp_path / "src")),)
    ws = Workspace.discover(roots)

    def collect():
        builder = model.AtlasBuilder()
        ctx = CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder)
        cpp.CppCodeCollector().collect(ctx)
        return builder.build()

    atlas = collect()
    symbol = atlas.entities[ids.build_sym("demo", "src/main.cpp", "demo::Node::configure")]
    assert any(
        source.file == ids.build_file("demo", "src/policies.hpp")
        and source.locator == "demo::policies"
        and source.line_hint == 3
        for fact in symbol.facts
        for source in fact.evidence
    )
    assert schema.validate(atlas.to_json()) == []
    assert atlas.canonical_bytes() == collect().canonical_bytes()
