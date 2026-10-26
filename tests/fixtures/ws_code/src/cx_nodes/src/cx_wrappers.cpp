// The two known wrapper cases: a diagnostic updater publishes /diagnostics with no
// create_publisher call, and message_filters creates real subscriptions of its own.

#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace cx_nodes
{

class WrapperNode : public rclcpp::Node
{
public:
  WrapperNode()
  : Node("cx_wrappers"),
    updater_(this)
  {
    updater_.setHardwareID("cx-fixture");
    updater_.add("cloud pair", this, &WrapperNode::report);

    left_sub_.subscribe(this, "/cx/left/points");
    right_sub_.subscribe(this, "/cx/right/points");
    sync_ = std::make_shared<Sync>(Policy(10), left_sub_, right_sub_);
    sync_->registerCallback(
      [this](
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr & left,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr & right) {(void)left; (void)right;});
  }

private:
  using Policy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;
  using Sync = message_filters::Synchronizer<Policy>;

  void report(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "paired");
  }

  diagnostic_updater::Updater updater_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> left_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> right_sub_;
  std::shared_ptr<Sync> sync_;
};

}  // namespace cx_nodes

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cx_nodes::WrapperNode>());
  rclcpp::shutdown();
  return 0;
}
