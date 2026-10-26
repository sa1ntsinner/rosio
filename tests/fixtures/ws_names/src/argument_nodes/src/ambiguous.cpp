#include <rclcpp/rclcpp.hpp>
class Ambiguous : public rclcpp::Node {
public:
  explicit Ambiguous(const std::string & node_name) : Node(node_name) {}
};
int main() {
  auto first = std::make_shared<Ambiguous>("first");
  auto second = std::make_shared<Ambiguous>("second");
}
