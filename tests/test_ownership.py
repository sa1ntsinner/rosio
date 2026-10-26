import pytest

from rosio import ids, join, model
from rosio.collect import cpp_code as cpp

pytestmark = pytest.mark.cpp


def test_class_inventory_preserves_declared_bases():
    unit = cpp.parse_text("namespace p { class Derived : public Base, private other::Mixin {}; }")
    assert getattr(unit, "class_bases", {}) == {"p::Derived": ["Base", "other::Mixin"]}


def test_constructed_class_binds_inherited_methods_from_transitive_library():
    builder = model.AtlasBuilder()
    main_file = ids.build_file("p", "main.cpp")
    derived_file = ids.build_file("p", "derived.cpp")
    base_file = ids.build_file("p", "base.cpp")
    other_file = ids.build_file("p", "other.cpp")
    exe = ids.build_exe("p", "node")
    builder.fact(
        exe,
        "declared",
        values={"sources": [main_file], "linked_targets": ["derived"]},
    )
    builder.fact(
        ids.build_file("p", "CMakeLists.txt"),
        "declared",
        values={
            "library_targets": [
                {"target": "derived", "sources": [derived_file], "linked_targets": ["base"]},
                {
                    "target": "base",
                    "sources": [base_file, other_file],
                    "linked_targets": ["derived"],
                },
            ]
        },
    )
    inventory = join.Carrier(
        owner="header",
        file=ids.build_file("p", "nodes.hpp"),
        symbol="",
        language="cpp",
        classes=(
            {"name": "p::Derived", "bases": ["Base"]},
            {"name": "p::Base", "bases": ["rclcpp::Node"]},
            {"name": "p::Other"},
        ),
    )
    main = join.Carrier(
        owner=ids.build_sym("p", "main.cpp", "main"),
        file=main_file,
        symbol=ids.build_sym("p", "main.cpp", "main"),
        language="cpp",
        constructions=({"class": "p::Derived"},),
    )
    methods = [
        join.Carrier(
            owner=ids.build_sym("p", path, f"p::{name}::configure"),
            file=file,
            symbol=ids.build_sym("p", path, f"p::{name}::configure"),
            language="cpp",
            parameters=({"name": name.lower(), "owner_symbol": f"p::{name}::configure"},),
        )
        for name, path, file in [
            ("Derived", "derived.cpp", derived_file),
            ("Base", "base.cpp", base_file),
            ("Other", "other.cpp", other_file),
        ]
    ]
    by_exe = {}
    join._constructed_carriers(builder, [inventory, main, *methods], by_exe, set())
    assert {carrier.owner for carrier in by_exe[exe]} == {methods[0].owner, methods[1].owner}
