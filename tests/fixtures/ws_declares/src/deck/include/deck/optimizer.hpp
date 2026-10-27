#pragma once

#include <rclcpp/rclcpp.hpp>

namespace deck
{
class Optimizer
{
public:
  struct Param
  {
    Param() = default;
    explicit Param(rclcpp::Node * node);
    double weight{};
  };

  explicit Optimizer(rclcpp::Node * node);

private:
  Param param_;
};
}  // namespace deck
