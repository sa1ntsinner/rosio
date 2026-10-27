#include "helped/nodes.hpp"

using helped::Driver;

Driver::Driver()
: Node("driver")
{
  nh_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
  odometry_ = std::make_unique<Odometry>(nh_);
  configure(this);
}
