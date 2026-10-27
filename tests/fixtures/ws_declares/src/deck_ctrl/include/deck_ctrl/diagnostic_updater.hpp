#pragma once

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

namespace deck_ctrl::diagnostic_updater
{
class Updater : public ::diagnostic_updater::Updater
{
public:
  explicit Updater(rclcpp::Node * node) : ::diagnostic_updater::Updater(node) {}
};
}  // namespace deck_ctrl::diagnostic_updater
