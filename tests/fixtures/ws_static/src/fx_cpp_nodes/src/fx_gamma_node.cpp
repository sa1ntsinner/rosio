#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace fx_cpp_nodes
{
class GammaNode : public rclcpp::Node
{
public:
  explicit GammaNode(const rclcpp::NodeOptions & options)
  : Node("fx_gamma", options) {}
};
}  // namespace fx_cpp_nodes

// No main() anywhere: the executable name exists only in CMakeLists.
RCLCPP_COMPONENTS_REGISTER_NODE(fx_cpp_nodes::GammaNode)
