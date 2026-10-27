#include <memory>

#include "two_nodes/nodes.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto alpha = std::make_shared<pair::Alpha>();
  auto beta = std::make_shared<pair::Beta>();
  executor.add_node(alpha);
  executor.add_node(beta);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
