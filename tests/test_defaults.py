import pytest

from rosio.collect import cpp_code as cpp

pytestmark = pytest.mark.cpp


def parameters(source):
    unit = cpp.parse_text(source, path="defaults.cpp")
    cpp.analyse([unit])
    return {row["name"]: row for row in cpp.file_report(unit)["parameters"]}


@pytest.mark.parametrize(
    ("declaration", "expected", "value_type"),
    [
        ("double value = 0.05;", 0.05, "double"),
        ("double value = 25;", 25.0, "double"),
        ("bool value = 0;", False, "bool"),
        ("int value = 2.5;", 2, "int"),
        ("int value = 20;", 20, "int"),
        ("bool value = false;", False, "bool"),
        ('std::string value = std::string("map");', "map", "string"),
    ],
)
def test_guarded_declarations_keep_the_stated_default(declaration, expected, value_type):
    row = parameters(
        "void Node::configure() {"
        + declaration
        + 'if (!has_parameter("setting")) { declare_parameter("setting", value); }}'
    )["setting"]
    assert row["default"] == expected
    assert type(row["default"]) is type(expected)
    assert row["type"] == value_type


def test_member_assignment_and_parameter_value_wrapper_keep_default():
    row = parameters("""
        void Node::configure() {
          this->resolution_ = 0.05;
          if (!node->has_parameter("resolution")) {
            node->declare_parameter("resolution", rclcpp::ParameterValue(this->resolution_));
          }
        }
    """)["resolution"]
    assert row["default"] == 0.05


def test_constructor_member_initializers_supply_defaults_only_in_the_constructor():
    rows = parameters("""
        Node::Node() : rclcpp::Node("node"), count_(20), enabled_(false) {
          declare_parameter("count", count_);
          declare_parameter("enabled", enabled_);
        }
        void Node::later() { declare_parameter("later", count_); }
    """)
    assert rows["count"]["default"] == 20
    assert rows["enabled"]["default"] is False
    assert rows["later"]["default"] is None


def test_assignment_reads_its_default_before_overwriting_the_local():
    row = parameters("""
        void Node::configure() {
          double frequency = 10.0;
          frequency = declare_parameter("frequency", frequency);
        }
    """)["frequency"]
    assert row["default"] == 10.0


@pytest.mark.parametrize(
    "mutation",
    [
        'get_parameter("timeout", value);',
        'value = get_parameter("timeout").as_double();',
        "if (runtime_flag) { value = 2.0; }",
        "value += 1.0;",
        "++value;",
        "mutate(&value);",
    ],
)
def test_runtime_writes_do_not_reuse_an_earlier_literal(mutation):
    row = parameters(
        "void Node::configure() { double value = 0.5;"
        + mutation
        + 'declare_parameter("interval", value); }'
    )["interval"]
    assert row["default"] is None
    assert row["default_state"] == "unresolved"


def test_a_call_initializer_can_mutate_a_default():
    row = parameters("""
        void Node::configure() {
          double value = 0.5;
          auto result = mutate(value);
          declare_parameter("interval", value);
        }
    """)["interval"]
    assert row["default"] is None


def test_runtime_string_writes_do_not_reuse_an_earlier_literal():
    row = parameters("""
        void Node::configure() {
          std::string value = "map";
          get_parameter("frame", value);
          declare_parameter("fallback", value);
        }
    """)["fallback"]
    assert row["default"] is None


@pytest.mark.parametrize("mutation", ['get_parameter("frame", frame_);', "mutate(frame_);"])
def test_runtime_member_writes_are_not_revived_by_another_file(mutation):
    initializer = cpp.parse_text(
        'void Node::initialize() { frame_ = "map"; }', path="initialize.cpp"
    )
    consumer = cpp.parse_text(
        "void Node::configure() {" + mutation + 'declare_parameter("later", frame_);}',
        path="consumer.cpp",
    )
    cpp.analyse([initializer, consumer])
    row = cpp.file_report(consumer)["parameters"][0]
    assert row["default"] is None
    assert row["default_state"] == "unresolved"


def test_inner_shadow_does_not_change_the_outer_default():
    row = parameters("""
        void Node::configure() {
          int value = 1;
          { int value = 2; }
          declare_parameter("count", value);
        }
    """)["count"]
    assert row["default"] == 1


def test_another_function_local_is_not_a_default():
    row = parameters("""
        void first() { double value = 0.5; }
        void second() { declare_parameter("interval", value); }
    """)["interval"]
    assert row["default"] is None


def test_an_unrepresentable_local_default_is_refused_without_casting():
    unit = cpp.parse_text(
        """
        void Node::configure() {
          int value = 1e400;
          declare_parameter("count", value);
        }
    """,
        path="overflow.cpp",
    )
    cpp.analyse([unit])
    report = cpp.file_report(unit)
    assert report["parameters"][0]["default"] is None
    assert any("json has no infinity" in row["reason"] for row in report["unresolved"])


def test_runtime_constructor_initializer_cannot_reuse_another_file_value():
    initializer = cpp.parse_text('void Node::reset() { frame_ = "map"; }', path="reset.cpp")
    constructor = cpp.parse_text(
        """
        Node::Node() : rclcpp::Node("node"), frame_(runtime_frame()) {
          declare_parameter("frame", frame_);
        }
    """,
        path="constructor.cpp",
    )
    cpp.analyse([initializer, constructor])
    row = cpp.file_report(constructor)["parameters"][0]
    assert row["default"] is None


@pytest.mark.parametrize("binding", ["int &alias = value;", "int *alias = &value;"])
def test_reference_escape_invalidates_a_default(binding):
    row = parameters(
        "void Node::configure() { int value = 1;"
        + binding
        + 'mutate(alias); declare_parameter("count", value); }'
    )["count"]
    assert row["default"] is None


def test_assignment_default_keeps_its_declared_double_type():
    row = parameters("""
        void Node::configure() {
          double value = 0;
          value = 1;
          declare_parameter("interval", value);
        }
    """)["interval"]
    assert row["default"] == 1.0
    assert type(row["default"]) is float
    assert row["type"] == "double"
