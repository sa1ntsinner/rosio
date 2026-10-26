// TF in both directions, and a node whose name is a constructor argument.

#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace cx_nodes
{

class TfNode : public rclcpp::Node
{
public:
  explicit TfNode(const std::string & node_name)
  : Node(node_name)
  {
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    sensor_frame_ = declare_parameter<std::string>("sensor_frame", "cx_lidar");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    if (publish_tf_) {
      broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cx/points", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {onCloud(msg);});

    sendMount();
  }

  void sendOdom()
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    broadcaster_->sendTransform(tf);
  }

  void sendMount()
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "cx_lidar";
    static_broadcaster_->sendTransform(tf);
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    buffer_->lookupTransform(base_frame_, sensor_frame_, tf2::TimePointZero);
    buffer_->lookupTransform(base_frame_, msg->header.frame_id, tf2::TimePointZero);
    if (buffer_->canTransform("map", odom_frame_, tf2::TimePointZero)) {
      return;
    }
  }

private:
  bool publish_tf_{false};
  std::string odom_frame_;
  std::string base_frame_;
  std::string sensor_frame_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

}  // namespace cx_nodes

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const std::string name = argc > 1 ? argv[1] : std::string("cx_tf");
  rclcpp::spin(std::make_shared<cx_nodes::TfNode>(name));
  rclcpp::shutdown();
  return 0;
}
