#include <deck/deck.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace deck_modules
{
class Stop : public deck::Module
{
public:
  void init(rclcpp::Node & node) override { node.declare_parameter<double>("stop.margin"); }
};
}  // namespace deck_modules

PLUGINLIB_EXPORT_CLASS(deck_modules::Stop, deck::Module)
