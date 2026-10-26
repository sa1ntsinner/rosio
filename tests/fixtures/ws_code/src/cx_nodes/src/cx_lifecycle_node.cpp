// The constructor body is empty. Endpoints appear in on_configure and the timer in on_activate.

#include "cx_nodes/cx_lifecycle_node.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace cx_nodes
{
namespace
{
template<typename T>
T declareOrGetParameter(rclcpp::Node & node, const std::string & name, const T & fallback)
{
  if (!node.has_parameter(name)) {
    return node.declare_parameter<T>(name, fallback);
  }
  return node.get_parameter(name).get_value<T>();
}
}  // namespace

BridgeLifecycle::BridgeLifecycle()
: rclcpp_lifecycle::LifecycleNode("cx_bridge")
{
}

CallbackReturn BridgeLifecycle::on_configure(const rclcpp_lifecycle::State &)
{
  cmd_vel_topic_ = declareOrGetParameter<std::string>(*this, "cmd_vel_topic", "/cx/cmd_vel");

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/cx/bridge_odom", 10);
  state_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/cx/bridge_state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  rclcpp::SubscriptionOptions cmd_options;
  cmd_options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, rclcpp::QoS(1),
    [this](geometry_msgs::msg::Twist::SharedPtr msg) {(void)msg;},
    cmd_options);

  return CallbackReturn::SUCCESS;
}

CallbackReturn BridgeLifecycle::on_activate(const rclcpp_lifecycle::State &)
{
  odom_pub_->on_activate();
  state_pub_->on_activate();
  timer_ = create_wall_timer(std::chrono::milliseconds(50), []() {});
  return CallbackReturn::SUCCESS;
}

CallbackReturn BridgeLifecycle::on_deactivate(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  odom_pub_->on_deactivate();
  state_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn BridgeLifecycle::on_cleanup(const rclcpp_lifecycle::State &)
{
  cmd_sub_.reset();
  odom_pub_.reset();
  state_pub_.reset();
  return CallbackReturn::SUCCESS;
}

}  // namespace cx_nodes

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cx_nodes::BridgeLifecycle>()->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
