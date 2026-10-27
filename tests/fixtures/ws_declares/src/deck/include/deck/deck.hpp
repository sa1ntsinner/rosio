#pragma once

#include <deck_ctrl/controller.hpp>
#include <deck_ctrl/diagnostic_updater.hpp>
#include <deck_fit/fitter.hpp>
#include "deck/optimizer.hpp"
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <vector>

namespace deck
{
class Module
{
public:
  virtual ~Module() = default;
  virtual void init(rclcpp::Node & node) = 0;
};

class DECK_PUBLIC Deck : public rclcpp::Node
{
public:
  Deck();
  template <typename T>
  T speed_cap();

private:
  double gain_;
  pluginlib::ClassLoader<Module> loader_{"deck", "deck::Module"};
  std::vector<std::shared_ptr<Module>> modules_;
  std::unique_ptr<deck_ctrl::Controller> controller_;
  std::shared_ptr<Optimizer> optimizer_;
  deck_fit::Fitter fitter_;
  deck_ctrl::diagnostic_updater::Updater diagnostics_;
};
}  // namespace deck

#include "deck/limits.ipp"
