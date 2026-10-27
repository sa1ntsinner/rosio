#ifndef TWO_NODES__NODES_HPP_
#define TWO_NODES__NODES_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace pair
{
class Alpha : public rclcpp::Node
{
public:
  Alpha();

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

class Beta : public rclcpp::Node
{
public:
  Beta();

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
}  // namespace pair

#endif  // TWO_NODES__NODES_HPP_
