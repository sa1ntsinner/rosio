#include <memory>

#include "split_nodes/nodes.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto filter = std::make_shared<Filter>();
  filter->load_parameters();
  rclcpp::spin(filter);
  rclcpp::shutdown();
  return 0;
}
