#include "nodes.hpp"
namespace linked_nodes {
void Navigation::initialize() {
  auto pub = create_publisher<std_msgs::msg::String>("position", 10);
}
}
