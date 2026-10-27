#pragma once

#include <rclcpp/rclcpp.hpp>

namespace deck_fit
{
class Fitter
{
public:
  explicit Fitter(rclcpp::Node * node);
};
}  // namespace deck_fit
