#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
namespace argument_nodes {
class Talker : public rclcpp::Node {
public:
  explicit Talker(const rclcpp::NodeOptions & options) : Node("component_talker", options) {}
};
class Other : public rclcpp::Node {
public:
  explicit Other(const rclcpp::NodeOptions & options) : Node("other_component", options) {}
};
}
RCLCPP_COMPONENTS_REGISTER_NODE(argument_nodes::Talker)
