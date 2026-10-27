#include "deck_fit/fitter.hpp"

namespace deck_fit
{
Fitter::Fitter(rclcpp::Node * node)
{
  node->declare_parameter<std::string>("fit.target");
}
}  // namespace deck_fit
