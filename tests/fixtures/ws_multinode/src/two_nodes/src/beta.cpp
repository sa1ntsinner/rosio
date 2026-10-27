#include "two_nodes/nodes.hpp"

namespace pair
{
Beta::Beta()
: Node("beta")
{
  declare_parameter<double>("gain", 1.0);
  subscription_ = create_subscription<std_msgs::msg::String>(
    "chatter", 10, [](std_msgs::msg::String::ConstSharedPtr) {});
}
}  // namespace pair
