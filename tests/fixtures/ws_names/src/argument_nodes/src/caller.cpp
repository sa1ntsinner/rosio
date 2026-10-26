#include <rclcpp/rclcpp.hpp>
#define LISTENER_NAME "argument_listener"
class Listener : public rclcpp::Node {
public:
  explicit Listener(int value, const std::string & node_name) : Node(node_name) {}
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto listener = std::make_shared<Listener>(1, LISTENER_NAME);
  rclcpp::spin(listener);
}
