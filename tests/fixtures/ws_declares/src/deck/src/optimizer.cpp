#include "deck/optimizer.hpp"

namespace deck
{
Optimizer::Param::Param(rclcpp::Node * node)
{
  weight = node->declare_parameter<double>("opt.weight");
}

Optimizer::Optimizer(rclcpp::Node * node)
{
  param_ = Param(node);
}
}  // namespace deck
