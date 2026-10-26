#include <memory>
#include <string>

#include "split_nodes/nodes.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::string topic("argv_chatter");
  if (argc > 1) {
    topic = argv[1];
  }
  auto talker = std::make_shared<Talker>(topic);
  talker->initialize();
  rclcpp::spin(talker);
  rclcpp::shutdown();
  return 0;
}
