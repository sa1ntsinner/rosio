#pragma once

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace deck::utils
{
template <typename T>
T declare(rclcpp::Node & node, const std::string & name)
{
  return node.declare_parameter<T>(name);
}
}  // namespace deck::utils
