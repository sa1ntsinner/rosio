#include "deck_ctrl/controller.hpp"

namespace deck_ctrl
{
Controller::Controller(rclcpp::Node & node)
{
  node.declare_parameter<double>("kp");
}
}  // namespace deck_ctrl
