from pathlib import Path

import pytest

from rosio import extract, ids, join, model, schema
from rosio.collect import cpp_code as cpp
from rosio.collect import plugins
from rosio.config import Config, Entrypoint, ProfileSpec, RootSpec
from rosio.resolve import wrappers
from rosio.workspace import Workspace

pytestmark = pytest.mark.cpp

FIXTURE = Path(__file__).parent / "fixtures" / "ws_declares"


def _declared(unit: cpp.Unit) -> set[str]:
    return {item.name.value for item in unit.params if item.name.status == cpp.LITERAL}


def test_a_dependent_template_call_declares_its_name():
    unit = cpp.parse_text(
        'void A::A(rclcpp::Node * node) { node->template declare_parameter<double>("x.y"); }'
    )
    assert _declared(unit) == {"x.y"}


def test_get_or_declare_parameter_declares_the_name_the_caller_passes():
    unit = cpp.parse_text(
        "void A::A(rclcpp::Node & node) {"
        ' autoware_utils::get_or_declare_parameter<bool>(node, "enabled");'
        ' get_or_declare_parameter<double>(*this, "ratio"); }'
    )
    assert _declared(unit) == {"enabled", "ratio"}


def test_a_lambda_that_declares_its_argument_is_a_wrapper():
    unit = cpp.parse_text(
        """
        void A::A(rclcpp::Node & node) {
          const auto dp = [&](const std::string & s) { return node.declare_parameter<int>(s); };
          const auto dd = [&](const auto & p) { return node.declare_parameter<double>(p); };
          dp("count");
          dd("limit");
        }
        """
    )
    assert _declared(unit) == {"count", "limit"}
    assert [item for item in unit.params if item.name.status != cpp.LITERAL] == []


def test_a_member_initializer_declares_its_name():
    unit = cpp.parse_text(
        'A::A() : rclcpp::Node("a"), gain_(declare_parameter<double>("gain")),'
        ' mode_(static_cast<int>(declare_parameter<int>("mode"))) {}'
    )
    assert _declared(unit) == {"gain", "mode"}


def test_a_wrapper_another_file_of_the_package_defines_declares_the_call_name():
    header = cpp.parse_text(
        "namespace p::utils { template <typename T> T declare(rclcpp::Node & node,"
        " const std::string & name) { return node.declare_parameter<T>(name); } }",
        pkg="p",
        relpath="include/p/utils.hpp",
    )
    source = cpp.parse_text(
        'void A::A() { p::utils::declare<double>(*this, "ratio"); }', pkg="p", relpath="a.cpp"
    )
    other = cpp.parse_text(
        'void B::B() { declare<double>(*this, "ratio"); }', pkg="q", relpath="b.cpp"
    )
    assert _declared(source) == set()
    shared = cpp.share_wrappers([header, source, other], cpp.load_parser())
    assert _declared(shared[1]) == {"ratio"}
    assert shared[2] is other


def test_a_class_loader_names_the_base_class_it_loads():
    unit = cpp.parse_text(
        'class M { pluginlib::ClassLoader<PluginModuleInterface> loader_{"p", "p::I"}; };'
    )
    assert unit.plugin_loaders == ["PluginModuleInterface"]


def test_a_parameter_definition_folds_nested_keys_and_maps_repeated_groups():
    names, patterns = plugins.parameter_definition(
        """
        node:
          rate: {type: double}
          limits:
            upper: {type: double, default_value: 1.0}
          joints: {type: string_array}
          gains:
            __map_joints:
              p: {type: double}
        """
    )
    assert names == ["joints", "limits.upper", "rate"]
    assert patterns == ['"gains." + joints + ".p"']
    assert join.could_name(patterns[0], "gains.wheel.p")
    assert not join.could_name(patterns[0], "gains.wheel.i")


def test_generate_parameter_library_is_a_scanned_target():
    scan = plugins.scan_cmake(
        "project(p)\ngenerate_parameter_library(${PROJECT_NAME}_parameters\n  param/p.yaml\n)\n"
    )
    assert scan.parameter_libraries == (("p_parameters", "param/p.yaml"),)


def _extract(tmp_path: Path) -> model.Atlas:
    roots = (RootSpec(kind="src", path=str(FIXTURE / "src")),)
    cfg = Config(
        roots=roots,
        profiles=(
            ProfileSpec(
                slug="deck",
                entrypoints=(Entrypoint(verb="launch", pkg="deck", target="deck.launch.py"),),
            ),
        ),
        out_dir=str(tmp_path / "out"),
        deterministic=True,
    )
    return extract.run(cfg, Workspace.discover(roots))


@pytest.fixture(scope="module")
def atlas(tmp_path_factory) -> model.Atlas:
    return _extract(tmp_path_factory.mktemp("declares"))


def _undeclared(atlas: model.Atlas) -> set[str]:
    return {
        finding.title.split(" ", 1)[0]
        for finding in atlas.findings
        if finding.rule == "param.set_undeclared"
    }


@pytest.mark.ros
@pytest.mark.parametrize(
    "name",
    [
        "gain",  # a member initializer
        "depth",  # node->template declare_parameter
        "enabled",  # get_or_declare_parameter
        "ratio",  # a template wrapper in a header of the package
        "limit",  # a lambda wrapper
        "kp",  # a class of another package the node hands itself to
        "opt.weight",  # a struct a helper of the node builds from the node it was handed
        "stop.margin",  # a plugin class of a base the package loads
        "rate",  # generate_parameter_library
        "limits.upper",  # generate_parameter_library, nested
        "fit.target",  # a member the node initializes with itself, of another package
        "diagnostic_updater.period",  # a known wrapper re-exported in another namespace
        "diagnostic_updater.use_fqn",
        "speed_cap",  # a template method an .ipp file defines
    ],
)
def test_a_name_declared_where_the_node_runs_it_is_effective(atlas, name):
    param = atlas.entities[ids.build_param("node:/deck", name)]
    assert param.state == model.ParamState.EFFECTIVE.value
    assert name not in _undeclared(atlas)


@pytest.mark.ros
@pytest.mark.parametrize("name", ["slow.hold", "wheel.p"])
def test_a_name_only_a_run_time_declaration_could_build_is_other(atlas, name):
    param = atlas.entities[ids.build_param("node:/deck", name)]
    assert param.state == model.ParamState.OTHER.value
    assert all(row.note == join.BLIND_NOTE for row in param.candidates)
    assert name not in _undeclared(atlas)


@pytest.mark.ros
def test_run_time_names_of_reached_and_generated_code_are_the_node_blind_spots(atlas):
    blind, _declared = join.parameter_declarations(atlas)
    assert {'ns + ".hold"', 'joints + ".p"'} <= set(blind["node:/deck"])


@pytest.mark.ros
def test_a_key_no_code_declares_is_still_reported(atlas):
    assert _undeclared(atlas) == {"unused_key"}


@pytest.mark.ros
def test_the_declares_fixture_validates_and_extracts_twice_into_the_same_atlas(atlas, tmp_path):
    assert schema.validate(atlas.to_json()) == []
    assert _extract(tmp_path).canonical_bytes() == atlas.canonical_bytes()


@pytest.mark.ros
def test_a_name_another_executable_of_the_package_declares_is_still_undeclared(tmp_path):
    """Only `a` runs. The `speed` that `b` declares says nothing about `a`, so the line that
    sets it for `a` is ignored and reported."""
    pkg = tmp_path / "src" / "p"
    for directory in ("src", "launch", "config"):
        (pkg / directory).mkdir(parents=True)
    (pkg / "package.xml").write_text(
        '<?xml version="1.0"?><package format="3"><name>p</name><version>0.0.1</version>'
        '<description>d</description><maintainer email="m@example.com">m</maintainer>'
        "<license>Apache-2.0</license><buildtool_depend>ament_cmake</buildtool_depend></package>"
    )
    (pkg / "CMakeLists.txt").write_text(
        "project(p)\nadd_executable(a src/a.cpp)\nadd_executable(b src/b.cpp)\n"
    )
    for name, key in (("a", "rate"), ("b", "speed")):
        (pkg / "src" / f"{name}.cpp").write_text(
            f"class {name.upper()} : public rclcpp::Node {{ public: {name.upper()}()"
            f' : rclcpp::Node("{name}") {{ declare_parameter<int>("{key}"); }} }};\n'
            f"int main() {{ rclcpp::spin(std::make_shared<{name.upper()}>()); }}\n"
        )
    (pkg / "config" / "a.yaml").write_text("a:\n  ros__parameters:\n    rate: 2\n    speed: 3\n")
    (pkg / "launch" / "a.launch.py").write_text(
        "import os\nfrom launch import LaunchDescription\nfrom launch_ros.actions import Node\n\n"
        "def generate_launch_description():\n"
        "    here = os.path.dirname(os.path.realpath(__file__))\n"
        "    return LaunchDescription([Node(package='p', executable='a', name='a',"
        " parameters=[os.path.join(here, '..', 'config', 'a.yaml')])])\n"
    )
    roots = (RootSpec(kind="src", path=str(tmp_path / "src")),)
    cfg = Config(
        roots=roots,
        profiles=(
            ProfileSpec(
                slug="a", entrypoints=(Entrypoint(verb="launch", pkg="p", target="a.launch.py"),)
            ),
        ),
        out_dir=str(tmp_path / "out"),
        deterministic=True,
    )
    atlas = extract.run(cfg, Workspace.discover(roots))
    assert atlas.entities["param:/a|rate"].state == model.ParamState.EFFECTIVE.value
    assert atlas.entities["param:/a|speed"].state == model.ParamState.UNDECLARED.value
    assert _undeclared(atlas) == {"speed"}


def test_a_lambda_argument_the_declare_call_does_not_take_is_no_default():
    unit = cpp.parse_text(
        """
        void A::load() {
          auto load = [this](const std::string & parameter, Eigen::MatrixXd & covariance) {
            declare_parameter(parameter, rclcpp::PARAMETER_DOUBLE_ARRAY);
            return true;
          };
          load("process_noise_covariance", process_noise_covariance_);
        }
        """
    )
    [param] = unit.params
    assert param.name.value == "process_noise_covariance"
    assert param.default_expr == ""


def test_a_temporary_a_node_class_constructs_with_itself_is_its_helper():
    header = cpp.parse_text(
        "namespace p { class Host : public rclcpp::Node {}; }", pkg="p", relpath="host.hpp"
    )
    source = cpp.parse_text(
        "namespace p { void Host::start() {"
        " info_ = vehicle::VehicleInfoUtils(*this).getVehicleInfo(); } }",
        pkg="p",
        relpath="host.cpp",
    )
    cpp.analyse([header, source])
    assert header.class_helpers.get("p::Host") == {"vehicle::VehicleInfoUtils"}


def test_a_string_extended_in_place_is_not_the_piece_it_appends():
    unit = cpp.parse_text(
        """
        void M::M(rclcpp::Node & node) {
          std::string ns(getModuleName());
          ns += ".speed";
          get_or_declare_parameter<double>(node, ns + ".max");
          std::string fixed = "slow";
          fixed += ".speed";
          get_or_declare_parameter<double>(node, fixed + ".min");
        }
        """
    )
    assert _declared(unit) == {"slow.speed.min"}


def test_a_class_records_what_it_builds_from_the_node_it_was_handed():
    header = cpp.parse_text(
        "namespace p { class Optimizer { struct Param {}; }; }", pkg="p", relpath="o.hpp"
    )
    source = cpp.parse_text(
        "namespace p { Optimizer::Optimizer(rclcpp::Node * node) { param_ = Param(node);"
        " declare_all(node); } }",
        pkg="p",
        relpath="o.cpp",
    )
    cpp.analyse([header, source])
    assert header.node_handoffs == {"p::Optimizer": {"Param", "declare_all()"}}


def test_a_static_method_a_node_class_hands_itself_to_reaches_its_class():
    header = cpp.parse_text(
        "namespace p { class Host : public rclcpp::Node {}; struct NodeParam {}; }",
        pkg="p",
        relpath="host.hpp",
    )
    source = cpp.parse_text(
        'namespace p { Host::Host() : Node("host") { param_ = NodeParam::init(*this); } }',
        pkg="p",
        relpath="host.cpp",
    )
    cpp.analyse([header, source])
    assert "NodeParam" in header.node_handoffs["p::Host"]


def test_a_member_method_a_node_class_hands_itself_to_reaches_the_field_class():
    header = cpp.parse_text(
        "namespace p { class Smoother {}; class Host : public rclcpp::Node {"
        " Smoother smoother_; }; }",
        pkg="p",
        relpath="host.hpp",
    )
    source = cpp.parse_text(
        'namespace p { Host::Host() : Node("host") { smoother_.initialize(this, params_); } }',
        pkg="p",
        relpath="host.cpp",
    )
    cpp.analyse([header, source])
    assert "Smoother" in header.node_handoffs["p::Host"]


def test_a_free_function_hands_the_node_on_to_the_classes_it_builds():
    source = cpp.parse_text(
        "namespace p { class Speed {};\n"
        "inline std::vector<std::unique_ptr<Base>> build(rclcpp::Node * node_ptr) {\n"
        "  std::vector<std::unique_ptr<Base>> out;\n"
        '  out.push_back(std::make_unique<Speed>(node_ptr, "speed")); return out; }\n'
        "Chain::Chain(rclcpp::Node * node_ptr) { items_ = build(node_ptr); } }",
        pkg="p",
        relpath="chain.cpp",
    )
    cpp.analyse([source])
    assert source.node_handoffs["p::build()"] == {"Speed"}
    assert "build()" in source.node_handoffs["p::Chain"]


def test_a_node_cast_to_its_own_type_is_still_the_node():
    source = cpp.parse_text(
        "namespace p { void Host::start() { strategy_ = std::make_unique<Naive<A>>(\n"
        "  std::dynamic_pointer_cast<Host>(this->shared_from_this()), offsets_); } }",
        pkg="p",
        relpath="host.cpp",
    )
    cpp.analyse([source])
    assert source.node_handoffs["p::Host"] == {"Naive<A>"}


def test_a_member_a_node_class_initializes_with_itself_is_its_helper():
    header = cpp.parse_text(
        "namespace p { class Host : public rclcpp::Node { fit::Fitter fitter_;"
        " std::unique_ptr<fit::Other> other_; }; }",
        pkg="p",
        relpath="host.hpp",
    )
    source = cpp.parse_text(
        'namespace p { Host::Host() : rclcpp::Node("host"), fitter_(this), other_(nullptr) {} }',
        pkg="p",
        relpath="host.cpp",
    )
    cpp.analyse([header, source])
    assert header.class_helpers.get("p::Host") == {"fit::Fitter"}
    assert header.node_handoffs.get("p::Host") == {"fit::Fitter"}


def test_a_known_wrapper_nested_in_another_namespace_is_that_wrapper():
    row = wrappers.lookup("vendor::wrapper::diagnostic_updater::Updater", "cpp")
    assert row is not None and row.symbol == "diagnostic_updater::Updater"
    assert wrappers.parameters_for(row.symbol) == (
        "diagnostic_updater.period",
        "diagnostic_updater.use_fqn",
    )
    assert wrappers.lookup("my_pkg.diagnostic_updater.Updater", "python") is None


def test_a_wrapper_member_a_header_declares_is_constructed_where_the_source_initializes_it():
    header = cpp.parse_text(
        "namespace p { class Host : public rclcpp::Node {"
        " q::diagnostic_updater::Updater diagnostics_; }; }",
        pkg="p",
        relpath="include/p/host.hpp",
    )
    source = cpp.parse_text(
        'namespace p { Host::Host() : rclcpp::Node("host"), diagnostics_(this) {} }',
        pkg="p",
        relpath="src/host.cpp",
    )
    assert source.endpoints == []
    shared = cpp.share_wrappers([header, source], cpp.load_parser())
    assert [(item.name.value, item.via_known_wrapper) for item in shared[1].endpoints] == [
        ("/diagnostics", "diagnostic_updater::Updater")
    ]


def test_an_ipp_file_is_a_cpp_source():
    assert ".ipp" in cpp.SUFFIXES


@pytest.mark.parametrize("head", ["class", "struct"])
def test_an_export_macro_in_a_class_head_is_not_the_class_name(head):
    unit = cpp.parse_text(
        f"namespace a {{ {head} FOLLOWER_PUBLIC Controller : public rclcpp::Node"
        ' { Controller() : rclcpp::Node("controller") {} }; }'
    )
    assert unit.parsed
    assert unit.class_names == ["a::Controller"]
    assert {item.qualname for item in unit.classes} == {"a::Controller"}


def test_a_brace_initialized_variable_of_an_all_caps_type_is_not_an_export_macro():
    assert cpp._EXPORT_MACRO.sub(b"", b"struct POINT origin{0, 0};") == (
        b"struct POINT origin{0, 0};"
    )
    head = b"class FOO_PUBLIC Controller\n{\n};\n"
    assert cpp._EXPORT_MACRO.sub(lambda m: m.group(1), head) == b"class  Controller\n{\n};\n"


def test_an_all_caps_class_name_is_kept():
    unit = cpp.parse_text("class ABC final : public rclcpp::Node {}; struct XYZ : Base {};")
    assert unit.class_names == ["ABC", "XYZ"]


def test_an_unqualified_class_is_looked_up_in_the_enclosing_namespaces_first():
    def carrier(file_id: str, *rows: dict) -> join.Carrier:
        return join.Carrier(owner=file_id, file=file_id, symbol="", classes=rows)

    found = [
        carrier("file:a|n.hpp", {"name": "ns::Node", "node_handoffs": ["Data"]}),
        carrier("file:b|d.hpp", {"name": "ns::Data"}),
        carrier("file:c|d.hpp", {"name": "other::Data"}),
    ]
    reach = join._Reach(model.Atlas(), found)
    assert reach.of({("a", "Node")}) == {("a", "Node"), ("b", "Data")}
