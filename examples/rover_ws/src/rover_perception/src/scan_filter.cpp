// Drops lidar returns outside a range window and republishes the scan.

#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class ScanFilter : public rclcpp::Node
{
public:
  ScanFilter()
  : Node("scan_filter"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    min_range_ = declare_parameter("min_range", 0.15);
    max_range_ = declare_parameter("max_range", 12.0);
    frame_id_ = declare_parameter("frame_id", std::string("laser_frame"));

    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "scan_filtered", rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::UniquePtr scan) {on_scan(std::move(scan));});
  }

private:
  void on_scan(sensor_msgs::msg::LaserScan::UniquePtr scan)
  {
    if (!tf_buffer_.canTransform("base_link", "laser_frame", tf2::TimePointZero)) {
      return;
    }
    for (auto & range : scan->ranges) {
      if (range < min_range_ || range > max_range_) {
        range = std::numeric_limits<float>::infinity();
      }
    }
    scan->header.frame_id = frame_id_;
    publisher_->publish(std::move(scan));
  }

  double min_range_;
  double max_range_;
  std::string frame_id_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanFilter>());
  rclcpp::shutdown();
  return 0;
}
