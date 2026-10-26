#include "nodes.hpp"
namespace linked_nodes {
template<typename T>
void Filter<T>::initialize() {
  auto pub = create_publisher<std_msgs::msg::String>("filtered", 10);
  declare_parameter<double>("frequency", 30.0);
}
}
template class linked_nodes::Filter<int>;
