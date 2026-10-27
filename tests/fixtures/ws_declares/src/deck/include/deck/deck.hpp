#pragma once

#include <deck_ctrl/controller.hpp>
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

class Deck : public rclcpp::Node
{
public:
  Deck();

private:
  double gain_;
  pluginlib::ClassLoader<Module> loader_{"deck", "deck::Module"};
  std::vector<std::shared_ptr<Module>> modules_;
  std::unique_ptr<deck_ctrl::Controller> controller_;
  std::shared_ptr<Optimizer> optimizer_;
};
}  // namespace deck
