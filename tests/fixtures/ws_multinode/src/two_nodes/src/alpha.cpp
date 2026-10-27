#include "two_nodes/nodes.hpp"

namespace pair
{
Alpha::Alpha()
: Node("alpha")
{
  declare_parameter<int>("rate", 10);
  publisher_ = create_publisher<std_msgs::msg::String>("chatter", 10);
}
}  // namespace pair
