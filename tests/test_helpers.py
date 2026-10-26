import pytest

from rosio import ids, join, model
from rosio.collect import cpp_code as cpp

pytestmark = pytest.mark.cpp


def test_node_helpers_follow_only_explicit_same_node_arguments():
    header = cpp.parse_text(
        "namespace p { class Host : public rclcpp::Node {}; "
        "class Helper {}; class Configurer {}; class Other {}; }",
        pkg="p",
        relpath="nodes.hpp",
    )
    source = cpp.parse_text(
        """
        namespace p {
        void Host::start() {
          auto direct = std::make_unique<Helper>(shared_from_this());
          configurer_ = std::make_unique<Configurer>();
          auto unrelated = std::make_unique<Other>();
        }
        void Host::configure() { configurer_->configure(shared_from_this()); }
        }
        """,
        pkg="p",
        relpath="host.cpp",
    )
    cpp.analyse([header, source])
    assert getattr(header, "class_helpers", {}).get("p::Host") == {"Helper", "Configurer"}


def test_node_plugin_selector_follows_parameter_read_and_same_node_argument():
    header = cpp.parse_text(
        "namespace p { class Host : public rclcpp::Node {}; }", pkg="p", relpath="nodes.hpp"
    )
    source = cpp.parse_text(
        """
        namespace p {
        void Host::configure() {
          std::string selected = get_parameter("solver_plugin").as_string();
          solver_ = loader_.createSharedInstance(selected);
          solver_->Configure(shared_from_this());
          std::string other = get_parameter("other_plugin").as_string();
          other_ = loader_.createSharedInstance(other);
          other_->Configure(other_node);
        }
        }
        """,
        pkg="p",
        relpath="host.cpp",
    )
    cpp.analyse([header, source])
    assert getattr(header, "class_node_plugins", {}).get("p::Host") == {"solver_plugin"}


@pytest.mark.parametrize("selected", ["p::Chosen", "p::Other", None, "unknown"])
def test_delegated_plugins_bind_the_profile_selected_lookup_only(selected):
    builder = model.AtlasBuilder()
    profile = builder.add_profile("configured")
    node = ids.build_node("/host")
    source = ids.build_file("p", "host.cpp")
    symbol = ids.build_sym("p", "host.cpp", "p::Host::configure")
    builder.node(node, name_source="literal")
    builder.node(source, attrs={"language": "cpp"})
    builder.fact(
        source,
        "coded",
        values={"classes": [{"name": "p::Host", "node_plugin_parameters": ["solver_plugin"]}]},
    )
    found = (
        join.Carrier(
            owner=symbol,
            file=source,
            symbol=symbol,
            language="cpp",
            parameters=({"name": "solver_plugin", "default": "p::Chosen", "has_default": True},)
            if selected == "unknown"
            else (),
        ),
    )
    for lookup in ("p::Chosen", "p::Other"):
        path = lookup.rsplit("::", 1)[1].lower() + ".cpp"
        plugin_file = ids.build_file("p", path)
        plugin = ids.build_plugin("p::Solver", lookup)
        builder.node(plugin_file, attrs={"language": "cpp"})
        builder.node(plugin)
        builder.fact(plugin, "coded", values={"type": lookup})
        builder.edge("defines", plugin_file, plugin, layer="coded")
        builder.fact(
            plugin_file,
            "coded",
            values={
                "parameters": [
                    {
                        "name": lookup.rsplit("::", 1)[1].lower(),
                        "owner_symbol": lookup + "::Configure",
                    },
                    {"name": "unrelated", "owner_symbol": "p::Unrelated::Configure"},
                ]
            },
        )
    if selected == "unknown":
        builder.set_param(
            ids.build_param(node, "solver_plugin"),
            state="other",
            candidates=[
                model.Candidate(
                    None, "unknown", "launch_inline_dict", model.Source(source), 0, False, profile
                )
            ],
        )
    elif selected is not None:
        builder.set_param(
            ids.build_param(node, "solver_plugin"),
            state="effective",
            candidates=[
                model.Candidate(
                    selected, "str", "launch_inline_dict", model.Source(source), 0, True, profile
                )
            ],
        )
    instance = join.Instance(node, profile, "p", "host", "/", "host", "literal", "", ())
    actual = getattr(join, "_with_node_plugins", lambda atlas, instance, found: found)(
        builder.atlas, instance, found
    )
    parameters = [
        row["name"]
        for carrier in actual
        for row in carrier.parameters
        if row["name"] != "solver_plugin"
    ]
    assert parameters == (
        [selected.rsplit("::", 1)[1].lower()] if selected not in (None, "unknown") else []
    )


def test_unknown_helper_reassignment_does_not_bind_the_previous_class():
    unit = cpp.parse_text(
        """
        namespace p {
        class Host : public rclcpp::Node {
          void configure() {
            helper_ = std::make_unique<Helper>();
            helper_ = factory();
            helper_->configure(shared_from_this());
          }
        };
        }
        """,
        pkg="p",
    )
    cpp.analyse([unit])
    assert getattr(unit, "class_helpers", {}).get("p::Host", set()) == set()


def test_passing_this_from_a_non_node_does_not_claim_node_ownership():
    unit = cpp.parse_text(
        "namespace p { class Helper { "
        "void start() { auto other = std::make_unique<Other>(this); } }; }",
        pkg="p",
    )
    cpp.analyse([unit])
    assert getattr(unit, "class_helpers", {}).get("p::Helper", set()) == set()
