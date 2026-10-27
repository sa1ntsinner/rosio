from pathlib import Path

import pytest

from rosio import extract, ids, join, model, schema
from rosio.collect import cpp_code as cpp
from rosio.config import Config, Entrypoint, ProfileSpec, RootSpec
from rosio.workspace import Workspace

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


HOST = """
namespace p { class Host : public rclcpp::Node {}; class Imu {}; class Listener {};
class Stray {}; }
"""


def test_a_shared_pointer_over_this_is_the_node_a_helper_receives():
    header = cpp.parse_text(HOST, pkg="p", relpath="nodes.hpp")
    source = cpp.parse_text(
        """
        using p::Host;
        Host::Host() : Node("host") {
          node_handle_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
          listener_ = std::make_unique<Listener>(node_handle_);
          stray_ = std::make_unique<Stray>(other_);
        }
        void Host::add_sensors() { sensors_.push_back(new Imu(node_handle_, "imu")); }
        """,
        pkg="p",
        relpath="host.cpp",
    )
    cpp.analyse([header, source])
    assert getattr(header, "class_helpers", {}).get("p::Host") == {"Imu", "Listener"}


def test_a_handle_also_assigned_something_else_is_not_the_node():
    header = cpp.parse_text(HOST, pkg="p", relpath="nodes.hpp")
    source = cpp.parse_text(
        """
        namespace p {
        Host::Host(rclcpp::Node::SharedPtr other) : Node("host") {
          node_handle_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
          if (other) { node_handle_ = other; }
          listener_ = std::make_unique<Listener>(node_handle_);
        }
        }
        """,
        pkg="p",
        relpath="host.cpp",
    )
    cpp.analyse([header, source])
    assert getattr(header, "class_helpers", {}).get("p::Host", set()) == set()


IMU = """
namespace p { class Imu {
 public:
  Imu(rclcpp::Node::SharedPtr node, const std::string & topic) {
    pub_ = node->create_publisher<sensor_msgs::msg::Imu>(topic, 10);
  }
  void publish() { auto message = std::make_unique<sensor_msgs::msg::Imu>(); }
}; }
"""


@pytest.mark.parametrize(
    ("sites", "name"),
    [
        ('new Imu(node_handle_, "imu");', "imu"),
        ('new Imu(node_handle_, "imu"); new Imu(node_handle_, "imu2");', None),
    ],
)
def test_a_helper_constructed_at_one_site_takes_its_literal_arguments(sites, name):
    header = cpp.parse_text(IMU, pkg="p", relpath="imu.hpp")
    source = cpp.parse_text(
        "namespace p { void Host::start() { " + sites + " } }", pkg="p", relpath="host.cpp"
    )
    cpp.analyse([header, source])
    assert [row["name"] for row in cpp.file_report(header)["endpoints"]] == [name]


FIXTURE = Path(__file__).parent / "fixtures" / "ws_helpers"


def _extract(tmp_path: Path) -> model.Atlas:
    roots = (RootSpec(kind="src", path=str(FIXTURE / "src")),)
    cfg = Config(
        roots=roots,
        profiles=(
            ProfileSpec(
                slug="pair",
                entrypoints=(Entrypoint(verb="launch", pkg="helped", target="pair.launch.py"),),
            ),
        ),
        out_dir=str(tmp_path / "out"),
        deterministic=True,
    )
    return extract.run(cfg, Workspace.discover(roots))


def _out(atlas: model.Atlas, node: str, kind: str) -> set[str]:
    return {edge.dst for edge in atlas.out_edges(node) if edge.kind == kind}


@pytest.mark.ros
def test_helper_records_bind_to_the_node_that_hands_itself_to_the_helper(tmp_path):
    atlas = _extract(tmp_path)
    assert _out(atlas, "node:/host", "publishes") == {"topic:/imu"}
    assert _out(atlas, "node:/host", "subscribes") == {"topic:/cmd_vel"}
    assert _out(atlas, "node:/driver", "publishes") == {"topic:/odom"}
    assert _out(atlas, "node:/driver", "subscribes") == set()
    # a base only the driver inherits declares its parameter on the driver alone
    assert _out(atlas, "node:/host", "has_param") == {"param:/host|imu.frame_id"}
    assert _out(atlas, "node:/driver", "has_param") == {
        "param:/driver|gain",
        "param:/driver|odometry.frame_id",
    }
    # a helper handed a node this reader cannot trace is a blind spot, not nothing
    assert [(row.owner, row.reason) for row in atlas.unresolved] == [
        ("sym:helped|src/stray.cpp|Stray::Stray", "node_handle_untraced")
    ]
    assert "topic:/stray" not in atlas.entities
    assert schema.validate(atlas.to_json()) == []


@pytest.mark.ros
def test_the_helper_fixture_extracts_twice_into_the_same_atlas(tmp_path):
    first = _extract(tmp_path / "first")
    second = _extract(tmp_path / "second")
    assert first.canonical_bytes() == second.canonical_bytes()
