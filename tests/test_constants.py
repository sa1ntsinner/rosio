import pytest

from rosio import ids, join, model
from rosio.collect import cpp_code as cpp
from rosio.drift import params

pytestmark = pytest.mark.cpp


def test_unknown_interface_constant_keeps_the_parameter_declaration():
    unit = cpp.parse_text(
        "void Host::configure() { declare_parameter(dep::msg::Constants::FLAG, true); }",
        path="constant.cpp",
    )
    report = cpp.file_report(unit)
    assert len(report["parameters"]) == 1
    row = report["parameters"][0]
    assert row["name"] is None
    assert row["name_expr"] == "dep::msg::Constants::FLAG"
    assert row["default"] is True


def test_interface_constant_resolution_preserves_truly_undeclared_findings():
    builder = model.AtlasBuilder()
    profile = builder.add_profile("configured")
    file = ids.build_file("p", "host.cpp")
    symbol = ids.build_sym("p", "host.cpp", "p::Host::configure")
    node = ids.build_node("/host")
    executable = ids.build_exe("p", "host")
    expr = "dep::msg::Constants::FLAG"
    builder.node(file, attrs={"language": "cpp"})
    builder.fact(
        symbol,
        "coded",
        values={
            "parameters": [
                {
                    "name": None,
                    "name_expr": expr,
                    "name_state": "unresolved",
                    "default": True,
                    "has_default": True,
                    "owner_symbol": "p::Host::configure",
                }
            ]
        },
    )
    builder.node(executable)
    builder.edge("runs", executable, symbol, layer="declared")
    builder.node(node, name_source="literal")
    builder.fact(
        node,
        "declared",
        profile=profile,
        values={"pkg": "p", "executable": "host", "name": "host", "namespace": "/"},
    )
    builder.unresolved(
        model.Unresolved(
            "param_name", symbol, expr, "not_stated_in_source", evidence=[model.Source(file)]
        )
    )
    builder.fact(
        ids.build_type("dep", "msg", "Constants"),
        "declared",
        values={"constants": [{"name": "FLAG", "type": "string", "value": "/known_flag"}]},
    )
    builder.set_param(
        ids.build_param(node, "truly_absent"),
        state="effective",
        candidates=[
            model.Candidate(
                0.0, "float", "launch_inline_dict", model.Source(file), 0, True, profile
            )
        ],
    )
    builder.fact(
        ids.build_param(node, "truly_absent"),
        "declared",
        profile=profile,
        values={"state": "effective"},
    )
    join.join(builder)
    atlas = builder.atlas
    assert atlas.unresolved == []
    assert (
        atlas.entities[ids.build_param(node, "/known_flag")]
        .fact("coded", profile)
        .values["declared_in_code"]
    )
    findings = params.set_undeclared(atlas)
    assert [finding.subject for finding in findings] == [ids.build_param(node, "truly_absent")]
    before = atlas.canonical_bytes()
    join.join(builder)
    assert atlas.canonical_bytes() == before


def test_a_bond_constant_no_workspace_defines_is_read_from_the_committed_table():
    """slam_toolbox declares bond::msg::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM, and that one
    blind declaration hid every undeclared key the node was configured with."""
    builder = model.AtlasBuilder()
    profile = builder.add_profile("configured")
    file = ids.build_file("p", "host.cpp")
    symbol = ids.build_sym("p", "host.cpp", "p::Host::Host")
    node = ids.build_node("/host")
    executable = ids.build_exe("p", "host")
    expr = "bond::msg::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM"
    builder.node(file, attrs={"language": "cpp"})
    builder.fact(
        symbol,
        "coded",
        values={
            "parameters": [
                {
                    "name": None,
                    "name_expr": expr,
                    "name_state": "unresolved",
                    "default": True,
                    "has_default": True,
                    "owner_symbol": "p::Host::Host",
                }
            ]
        },
    )
    builder.node(executable)
    builder.edge("runs", executable, symbol, layer="declared")
    builder.node(node, name_source="literal")
    builder.fact(
        node,
        "declared",
        profile=profile,
        values={"pkg": "p", "executable": "host", "name": "host", "namespace": "/"},
    )
    builder.unresolved(
        model.Unresolved(
            "param_name", symbol, expr, "not_stated_in_source", evidence=[model.Source(file)]
        )
    )
    join.join(builder)
    assert builder.atlas.unresolved == []
    declared = ids.build_param(node, "/bond_disable_heartbeat_timeout")
    assert builder.atlas.entities[declared].fact("coded", profile).values["declared_in_code"]
