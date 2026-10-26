#include <memory>

#include "split_nodes/nodes.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto talker = std::make_shared<Talker>();
  talker->initialize();
  rclcpp::spin(talker);
  rclcpp::shutdown();
  return 0;
}
