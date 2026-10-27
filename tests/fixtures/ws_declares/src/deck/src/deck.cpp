#include "deck/deck.hpp"

#include "deck/declare.hpp"

#include <autoware_utils_rclcpp/parameter.hpp>

namespace deck
{
Deck::Deck() : rclcpp::Node("deck"), gain_(declare_parameter<double>("gain"))
{
  const auto depth = this->template declare_parameter<int>("depth");
  const auto enabled = autoware_utils_rclcpp::get_or_declare_parameter<bool>(*this, "enabled");
  const auto ratio = utils::declare<double>(*this, "ratio");
  const auto dp = [&](const std::string & s) { return declare_parameter<double>(s); };
  const auto limit = dp("limit");
  controller_ = std::make_unique<deck_ctrl::Controller>(*this);
  optimizer_ = std::make_shared<Optimizer>(this);
  for (const auto & name : declare_parameter<std::vector<std::string>>("modules")) {
    const auto module = loader_.createSharedInstance(name);
    module->init(*this);
    modules_.push_back(module);
  }
}
}  // namespace deck
