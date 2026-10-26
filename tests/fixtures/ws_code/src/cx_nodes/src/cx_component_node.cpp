// A component: no main(), the executable name lives only in CMakeLists.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace cx_nodes
{

class MarkerComponent : public rclcpp::Node
{
public:
  explicit MarkerComponent(const rclcpp::NodeOptions & options)
  : Node("cx_marker", options)
  {
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "markers", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
};

}  // namespace cx_nodes

RCLCPP_COMPONENTS_REGISTER_NODE(cx_nodes::MarkerComponent)
