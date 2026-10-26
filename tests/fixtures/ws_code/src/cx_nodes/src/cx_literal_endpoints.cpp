// Every endpoint here has a string literal topic name. The variation is the QoS form.

#include <chrono>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace cx_nodes
{

class LiteralEndpoints : public rclcpp::Node
{
public:
  LiteralEndpoints()
  : Node("cx_literal")
  {
    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cx/points", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {onPoints(msg);});

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/cx/odom", rclcpp::QoS(rclcpp::KeepLast(5)));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    ready_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/cx/ready", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    auto scan_qos = rclcpp::SensorDataQoS();
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/cx/scan", scan_qos,
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {(void)msg;});

    rclcpp::QoS latched_qos(1);
    latched_qos.transient_local();
    rclcpp::SubscriptionOptions estop_options;
    estop_options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/cx/estop", latched_qos,
      [this](std_msgs::msg::Bool::SharedPtr msg) {(void)msg;},
      estop_options);

    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "/cx/reset",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {(void)req; (void)res;});

    enable_client_ = create_client<std_srvs::srv::SetBool>("/cx/enable_motion");

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), [this]() {publish();});
  }

private:
  void onPoints(const sensor_msgs::msg::PointCloud2::SharedPtr) {}
  void publish() {}

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace cx_nodes

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cx_nodes::LiteralEndpoints>());
  rclcpp::shutdown();
  return 0;
}
