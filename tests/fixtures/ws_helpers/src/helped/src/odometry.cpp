#include "helped/nodes.hpp"

using helped::Odometry;

Odometry::Odometry(std::shared_ptr<rclcpp::Node> & nh)
: nh_(nh)
{
  nh_->declare_parameter<std::string>("odometry.frame_id", "odom");
  pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
}
