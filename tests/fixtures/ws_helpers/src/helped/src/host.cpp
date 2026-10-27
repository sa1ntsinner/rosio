#include "helped/nodes.hpp"

using helped::Host;

Host::Host(std::shared_ptr<rclcpp::Node> other)
: Node("host"), other_(other)
{
  node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});
  add_sensors();
  listener_ = std::make_unique<Listener>(node_handle_);
  stray_ = std::make_unique<Stray>(other_);
}

void Host::add_sensors()
{
  sensors_.push_back(new sensors::Imu(node_handle_, "imu"));
}
