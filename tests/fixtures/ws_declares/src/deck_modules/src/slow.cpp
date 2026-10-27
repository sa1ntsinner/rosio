#include <autoware_utils_rclcpp/parameter.hpp>
#include <deck/deck.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <string>

namespace deck_modules
{
class Slow : public deck::Module
{
public:
  void init(rclcpp::Node & node) override
  {
    const std::string ns(module_name());
    autoware_utils_rclcpp::get_or_declare_parameter<double>(node, ns + ".hold");
  }

private:
  const char * module_name() const;
};
}  // namespace deck_modules

PLUGINLIB_EXPORT_CLASS(deck_modules::Slow, deck::Module)
