#pragma once

#include <rclcpp/rclcpp.hpp>

namespace deck_ctrl
{
class Controller
{
public:
  explicit Controller(rclcpp::Node & node);
};
}  // namespace deck_ctrl
