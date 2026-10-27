#include <memory>

#include "helped/nodes.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto driver = std::make_shared<helped::Driver>();
  auto host = std::make_shared<helped::Host>(driver);
  executor.add_node(driver);
  executor.add_node(host);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
