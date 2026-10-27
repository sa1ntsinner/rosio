#include "helped/nodes.hpp"

using helped::Configurable;

void Configurable::configure(rclcpp::Node * node)
{
  node->declare_parameter<double>("gain", 1.0);
}
