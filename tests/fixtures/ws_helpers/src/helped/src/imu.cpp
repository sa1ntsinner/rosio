#include "helped/nodes.hpp"

using helped::sensors::Imu;

Imu::Imu(std::shared_ptr<rclcpp::Node> & nh, const std::string & topic)
: Sensor(nh)
{
  nh_->declare_parameter<std::string>("imu.frame_id", "imu_link");
  pub_ = nh_->create_publisher<sensor_msgs::msg::Imu>(topic, 10);
}

void Imu::publish()
{
  auto message = std::make_unique<sensor_msgs::msg::Imu>();
  pub_->publish(std::move(message));
}
