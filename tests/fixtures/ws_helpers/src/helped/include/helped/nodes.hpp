#ifndef HELPED__NODES_HPP_
#define HELPED__NODES_HPP_

#include <list>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace helped
{
namespace sensors
{
class Sensor
{
public:
  explicit Sensor(std::shared_ptr<rclcpp::Node> & nh)
  : nh_(nh) {}
  virtual ~Sensor() = default;

protected:
  std::shared_ptr<rclcpp::Node> nh_;
};

class Imu : public Sensor
{
public:
  Imu(std::shared_ptr<rclcpp::Node> & nh, const std::string & topic);
  void publish();

private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
};
}  // namespace sensors

class Listener
{
public:
  explicit Listener(const rclcpp::Node::SharedPtr & node)
  {
    sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [](geometry_msgs::msg::Twist::ConstSharedPtr) {});
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

class Odometry
{
public:
  explicit Odometry(std::shared_ptr<rclcpp::Node> & nh);

private:
  std::shared_ptr<rclcpp::Node> nh_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};

class Stray
{
public:
  explicit Stray(std::shared_ptr<rclcpp::Node> node);

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};

class Configurable
{
protected:
  void configure(rclcpp::Node * node);
};

class Host : public rclcpp::Node
{
public:
  explicit Host(std::shared_ptr<rclcpp::Node> other);

private:
  void add_sensors();
  rclcpp::Node::SharedPtr node_handle_;
  rclcpp::Node::SharedPtr other_;
  std::list<sensors::Sensor *> sensors_;
  std::unique_ptr<Listener> listener_;
  std::unique_ptr<Stray> stray_;
};

class Driver : public rclcpp::Node, public Configurable
{
public:
  Driver();

private:
  std::shared_ptr<rclcpp::Node> nh_;
  std::unique_ptr<Odometry> odometry_;
};
}  // namespace helped

#endif  // HELPED__NODES_HPP_
