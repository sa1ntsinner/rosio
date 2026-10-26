// A costmap style plugin. The declare side auto prefixes with the instance name, the read
// side spells the prefix by hand, and the plugin instance name is only in the yaml layer.

#include <memory>
#include <string>

#include "cx_core/layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace cx_nodes
{

class DecayLayer : public cx_core::Layer
{
public:
  void onInitialize() override
  {
    auto node = node_.lock();

    declareParameter("topic", rclcpp::ParameterValue(std::string("obstacle_cloud")));
    declareParameter("decay_s", rclcpp::ParameterValue(3.0));
    declareParameter("enabled", rclcpp::ParameterValue(true));

    node->get_parameter(name_ + "." + "topic", topic_);
    node->get_parameter(name_ + "." + "decay_s", decay_s_);

    cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {(void)msg;});
  }

private:
  std::string topic_;
  double decay_s_{0.0};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

}  // namespace cx_nodes

PLUGINLIB_EXPORT_CLASS(
  cx_nodes::DecayLayer,
  cx_core::Layer)
