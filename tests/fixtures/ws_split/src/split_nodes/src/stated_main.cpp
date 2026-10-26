#include <memory>
#include <string>

#include "split_nodes/nodes.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::string topic("stated_chatter");
  if (argc > 0) {
    auto talker = std::make_shared<Talker>(topic);
    talker->initialize();
    rclcpp::spin(talker);
  }
  rclcpp::shutdown();
  return 0;
}
