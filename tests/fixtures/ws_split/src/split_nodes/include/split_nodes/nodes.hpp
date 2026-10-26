#ifndef SPLIT_NODES__NODES_HPP_
#define SPLIT_NODES__NODES_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace names
{
constexpr char DEFAULT_TOPIC[] = "default_chatter";
constexpr char OTHER_TOPIC[] = "other_chatter";
}  // namespace names

class Talker : public rclcpp::Node
{
public:
  explicit Talker(const std::string & topic = names::DEFAULT_TOPIC);
  void initialize();

private:
  const std::string topic_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

class Filter : public rclcpp::Node
{
public:
  Filter();
  void load_parameters();
  void reload();

private:
  double gain_;
  double offset_;
  double scale_ = 2.0;
  int window_;
};

#endif  // SPLIT_NODES__NODES_HPP_
