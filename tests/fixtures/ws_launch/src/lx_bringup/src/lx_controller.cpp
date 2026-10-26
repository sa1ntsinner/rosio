// Minimal node so the fixture package has one real executable target.
#include <memory>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("controller_server");
  node->declare_parameter<double>("controller_frequency", 20.0);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
