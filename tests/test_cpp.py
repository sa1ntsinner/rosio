import pytest

from rosio.collect import cpp_code as cpp

pytestmark = pytest.mark.cpp


def test_type_only_declarations_have_no_default_value():
    unit = cpp.parse_text(
        """
        void Node::configure() {
          declare_parameter("frames", rclcpp::PARAMETER_STRING_ARRAY);
          declare_parameter("enabled", rclcpp::ParameterType::PARAMETER_BOOL);
        }
    """,
        path="typed.cpp",
    )
    cpp.analyse([unit])
    report = cpp.file_report(unit)
    rows = {row["name"]: row for row in report["parameters"]}
    assert rows["frames"]["type"] == "string_array"
    assert rows["enabled"]["type"] == "bool"
    assert all(not row["has_default"] for row in rows.values())
    assert report["unresolved"] == []


@pytest.mark.parametrize(
    "expression", ["std::vector<double>()", "std::vector<bool>{}", "std::vector<uint8_t>{}"]
)
def test_empty_vector_defaults_are_known(expression):
    unit = cpp.parse_text(
        f'void Node::configure() {{ declare_parameter("values", {expression}); }}',
        path="vector.cpp",
    )
    cpp.analyse([unit])
    report = cpp.file_report(unit)
    row = report["parameters"][0]
    assert row["default"] == []
    assert row["has_default"]
    assert report["unresolved"] == []


@pytest.mark.parametrize("interfaces", [False, True])
def test_action_client_overloads_keep_action_name(interfaces):
    prefix = (
        "get_node_base_interface(), get_node_graph_interface(), "
        "get_node_logging_interface(), get_node_waitables_interface()"
        if interfaces
        else "this"
    )
    unit = cpp.parse_text(
        "void Client::configure() { "
        f'rclcpp_action::create_client<demo::action::Move>({prefix}, "move"); }}',
        path="client.cpp",
    )
    report = cpp.file_report(unit)
    row = report["endpoints"][0]
    assert row["kind"] == "action_client"
    assert row["name"] == "move"
    assert report["unresolved"] == []


@pytest.mark.parametrize("interfaces", [False, True])
def test_action_server_overloads_keep_action_name(interfaces):
    prefix = (
        "get_node_base_interface(), get_node_clock_interface(), "
        "get_node_logging_interface(), get_node_waitables_interface()"
        if interfaces
        else "this"
    )
    unit = cpp.parse_text(
        "void Server::configure() { "
        f"rclcpp_action::create_server<demo::action::Move>({prefix}, "
        '"move", on_goal, on_cancel, on_accept); }',
        path="server.cpp",
    )
    row = cpp.file_report(unit)["endpoints"][0]
    assert row["kind"] == "action_server"
    assert row["name"] == "move"


TF_GUARDS = """
class Filter : public rclcpp::Node {
public:
  Filter() : Node("filter") {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  }
  void publish(const Odometry & odom, bool corrected) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom_frame_;
    if (publish_tf_) {
      broadcaster_->sendTransform(tf);
    } else {
      broadcaster_->sendTransform(tf);
    }
    if (odom.header.frame_id == odom_frame_) {
      return;
    } else if (odom.header.frame_id == map_frame_) {
      if (can_transform) {
        broadcaster_->sendTransform(tf);
      }
    }
  }
};
"""


def _tf_rows() -> list[dict]:
    unit = cpp.parse_text(TF_GUARDS, path="filter.cpp")
    cpp.analyse([unit])
    return cpp.file_report(unit)["tf"]["broadcasts"]


def test_a_broadcast_under_a_runtime_guard_states_the_guard():
    rows = _tf_rows()
    assert [(row["parent"], row["child"]) for row in rows] == [("map", "odom")] * 3
    assert rows[2]["condition"] == (
        "!(odom.header.frame_id == odom_frame_) && "
        "(odom.header.frame_id == map_frame_) && can_transform"
    )


def test_a_parameter_guard_is_named_by_the_parameter_and_negated_on_the_else_arm():
    rows = _tf_rows()
    assert [row["condition"] for row in rows[:2]] == ["publish_tf", "!publish_tf"]


def test_guarded_broadcasts_read_the_same_twice():
    assert _tf_rows() == _tf_rows()


ENDPOINT_GUARDS = """
class Talker : public rclcpp::Node {
public:
  Talker() : Node("talker") {
    chatter_ = create_publisher<std_msgs::msg::String>("chatter", 10);
    if (debug) {
      debug_ = create_publisher<std_msgs::msg::String>("debug", 10);
      if (verbose) {
        trace_ = create_subscription<std_msgs::msg::String>("trace", 10, cb);
      }
    } else {
      quiet_ = create_service<std_srvs::srv::Trigger>("quiet", cb);
    }
    if (fast) {
      timer_ = create_wall_timer(100ms, cb);
    }
  }
};
"""


def _endpoint_rows() -> dict:
    unit = cpp.parse_text(ENDPOINT_GUARDS, path="talker.cpp")
    cpp.analyse([unit])
    rows = cpp.file_report(unit)["endpoints"]
    return {row["name"] or row["kind"]: row for row in rows}


def test_a_guarded_endpoint_states_its_guard():
    rows = _endpoint_rows()
    assert rows["debug"]["condition"] == "debug"
    assert rows["trace"]["condition"] == "debug && verbose"
    assert rows["quiet"]["condition"] == "!(debug)"
    assert rows["timer"]["condition"] == "fast"


def test_an_unguarded_endpoint_carries_no_condition():
    assert "condition" not in _endpoint_rows()["chatter"]


def test_guarded_endpoints_read_the_same_twice():
    assert _endpoint_rows() == _endpoint_rows()


MEMBER_GUARDS = """
class Pose : public rclcpp::Node {
public:
  Pose() : Node("pose") {
    if (publish_tf) {
      broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    }
    if (mirror) {
      mirror_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    } else {
      mirror_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    }
  }
  void tick(bool moving) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    broadcaster_->sendTransform(tf);
    if (moving) {
      broadcaster_->sendTransform(tf);
      mirror_->sendTransform(tf);
    }
    if (publish_tf) {
      broadcaster_->sendTransform(tf);
    }
  }
};
"""


def test_a_construction_guard_is_joined_outside_the_send_guard():
    unit = cpp.parse_text(MEMBER_GUARDS, path="pose.cpp")
    cpp.analyse([unit])
    rows = cpp.file_report(unit)["tf"]["broadcasts"]
    assert [row["condition"] for row in rows] == [
        "publish_tf",
        "publish_tf && moving",
        "moving",
        "publish_tf",
    ]


def test_the_else_arm_of_a_parameter_guarded_endpoint_is_negated():
    unit = cpp.parse_text(
        """
        void Filter::configure() {
          bool stamped = declare_parameter<bool>("stamped_control", false);
          if (stamped) {
            sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("cmd_vel", 10, cb);
          } else {
            sub_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, cb);
          }
        }
    """,
        path="filter.cpp",
    )
    cpp.analyse([unit])
    rows = cpp.file_report(unit)["endpoints"]
    assert [(row["type"], row["condition"]) for row in rows] == [
        ("geometry_msgs/msg/TwistStamped", "stamped_control"),
        ("geometry_msgs/msg/Twist", "!stamped_control"),
    ]


def test_a_parameter_guard_states_the_parameter_and_the_value_each_arm_needs():
    rows = _tf_rows()
    assert [row.get("guard_parameters") for row in rows] == [
        [{"name": "publish_tf", "value": True}],
        [{"name": "publish_tf", "value": False}],
        None,
    ]


def test_a_guard_with_a_raw_term_states_no_guard_parameters():
    """`debug` reads like a parameter name but nothing declares it, so it is source text."""
    assert all("guard_parameters" not in row for row in _endpoint_rows().values())


def test_a_construction_guard_on_a_parameter_carries_into_the_send_site():
    unit = cpp.parse_text(
        """
        void Pose::configure() {
          bool publish = declare_parameter<bool>("publish_tf", true);
          if (publish) {
            broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
          }
          geometry_msgs::msg::TransformStamped tf;
          tf.header.frame_id = "odom";
          tf.child_frame_id = "base_link";
          broadcaster_->sendTransform(tf);
        }
    """,
        path="pose.cpp",
    )
    cpp.analyse([unit])
    rows = cpp.file_report(unit)["tf"]["broadcasts"]
    assert [row.get("guard_parameters") for row in rows] == [
        [{"name": "publish_tf", "value": True}]
    ]


def _guard_of(body: str) -> str:
    unit = cpp.parse_text(f"void Talker::configure() {{ {body} }}", path="talker.cpp")
    cpp.analyse([unit])
    return cpp.file_report(unit)["endpoints"][0]["condition"]


PUB = 'pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);'


def test_a_joined_guard_keeps_the_precedence_of_each_term():
    assert _guard_of(f"if (a||b) {{ if (c) {{ {PUB} }} }}") == "(a || b) && c"
    assert _guard_of(f"if (a) {{ if (b ? c : d) {{ {PUB} }} }}") == "a && (b ? c : d)"


def test_a_single_operand_is_joined_without_parentheses():
    assert (
        _guard_of(f"if (node->ready()) {{ if (!done) {{ if (cfg.on) {{ {PUB} }} }} }}")
        == "node->ready() && !done && cfg.on"
    )


@pytest.mark.parametrize("spelling", ["a||b", "a || b", "a  ||\n   b", "a|| b"])
def test_a_raw_guard_reads_the_same_whatever_its_whitespace(spelling):
    assert _guard_of(f"if ({spelling}) {{ {PUB} }}") == "a || b"


def test_a_raw_guard_spaces_only_binary_operators_and_commas():
    assert _guard_of(f"if (f(x,y)&&!g( z )) {{ {PUB} }}") == "f(x, y) && !g(z)"
    assert _guard_of(f"if (count_>=limit) {{ {PUB} }}") == "count_ >= limit"
    assert _guard_of(f"if (period_!=0ms) {{ {PUB} }}") == "period_ != 0ms"


def test_a_sub_node_prefixes_its_namespace_onto_relative_names_only():
    unit = cpp.parse_text(
        """
class Sensors : public rclcpp::Node {
 public:
  Sensors() : Node("sensors") {
    lidar_ = this->create_sub_node("lidar");
    auto front = lidar_->create_sub_node("front");
    scan_ = front->create_publisher<std_msgs::msg::String>("scan", 10);
    status_ = lidar_->create_publisher<std_msgs::msg::String>("/status", 10);
    own_ = lidar_->create_publisher<std_msgs::msg::String>("~/own", 10);
    plain_ = create_publisher<std_msgs::msg::String>("plain", 10);
  }
  void later() { info_ = lidar_->create_service<std_srvs::srv::Trigger>("info", cb); }
};
""",
        path="sensors.cpp",
    )
    cpp.analyse([unit])
    names = sorted(row["name"] for row in cpp.file_report(unit)["endpoints"])
    assert names == ["/status", "lidar/front/scan", "lidar/info", "plain", "~/own"]
