#include "helped/nodes.hpp"

using helped::Stray;

Stray::Stray(std::shared_ptr<rclcpp::Node> node)
{
  pub_ = node->create_publisher<nav_msgs::msg::Odometry>("stray", 10);
}
