#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace fx_cpp_nodes
{
class BetaNode : public rclcpp::Node
{
public:
  explicit BetaNode(const rclcpp::NodeOptions & options)
  : Node("fx_beta", options) {}
};
}  // namespace fx_cpp_nodes

RCLCPP_COMPONENTS_REGISTER_NODE(fx_cpp_nodes::BetaNode)
