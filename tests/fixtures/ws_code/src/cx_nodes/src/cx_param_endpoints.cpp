// Topic names that are not literals, and the parameter API in every shape this stack uses.

#include <chrono>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cx_core/tuning.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

#define CX_STATUS_TOPIC "/cx/status"

namespace cx_nodes
{
namespace
{
constexpr char kGroundTopic[] = "/cx/ground";
}  // namespace

class ParamEndpoints : public rclcpp::Node
{
public:
  ParamEndpoints()
  : Node("cx_param")
  {
    const std::string input_topic = declare_parameter<std::string>("input_topic", "/cx/points_raw");
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {(void)msg;});

    cmd_topic_ = declare_parameter<std::string>("cmd_topic", "/cx/cmd_vel_safe");
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);

    const std::string prefix = declare_parameter<std::string>("topic_prefix", "/cx");
    health_pub_ = create_publisher<std_msgs::msg::Float32>(prefix + "/health", 10);

    status_pub_ = create_publisher<std_msgs::msg::Bool>(CX_STATUS_TOPIC, 1);

    ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      kGroundTopic, rclcpp::QoS(rclcpp::KeepLast(1)));

    declare_parameter<double>("publish_rate_hz", 20.0);
    declare_parameter<int>("queue_depth", 5);
    declare_parameter<bool>("publish_diagnostics", true);
    declare_parameter<double>("watchdog_timeout_s");
    declare_parameter<double>("gain", tuning_.gain);

    declare_parameters<double>(
      "limits", std::map<std::string, double>{{"max_speed", 3.0}, {"max_accel", 1.0}});

    const std::string sensor_ns = "sensors.front";
    declare_parameter<std::string>(sensor_ns + ".frame_id", "cx_front");

    get_parameter("publish_rate_hz", publish_rate_hz_);
    queue_depth_ = get_parameter("queue_depth").as_int();

    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {return onSetParameters(params);});

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_), [this]() {publish();});

    for (const auto & name : loadDescriptorTopics()) {
      extra_subs_.push_back(
        create_subscription<sensor_msgs::msg::PointCloud2>(
          name, rclcpp::SensorDataQoS(),
          [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {(void)msg;}));
    }
  }

private:
  std::vector<std::string> loadDescriptorTopics();
  void publish() {}

  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> &)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  cx_core::Tuning tuning_;
  std::string cmd_topic_;
  double publish_rate_hz_{0.0};
  int queue_depth_{0};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> extra_subs_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr health_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// The names come out of a model description on disk, so the source cannot state them.
std::vector<std::string> ParamEndpoints::loadDescriptorTopics()
{
  std::vector<std::string> names;
  std::ifstream descriptor(declare_parameter<std::string>("descriptor_file", ""));
  for (std::string line; std::getline(descriptor, line); ) {
    if (!line.empty()) {
      names.push_back(line);
    }
  }
  return names;
}

}  // namespace cx_nodes

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cx_nodes::ParamEndpoints>());
  rclcpp::shutdown();
  return 0;
}
