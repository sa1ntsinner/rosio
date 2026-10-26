#include <rclcpp/rclcpp.hpp>
namespace linked_nodes {
template<typename T>
class Filter : public rclcpp::Node {
public:
  Filter() : rclcpp::Node("filter") {}
  void initialize();
};
class Navigation : public rclcpp::Node {
public:
  Navigation() : rclcpp::Node("navigation") {}
  void initialize();
};
using SelectedFilter = Filter<int>;
}
