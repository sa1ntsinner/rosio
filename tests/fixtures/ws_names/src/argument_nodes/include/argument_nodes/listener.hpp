class HeaderListener : public rclcpp::Node {
public:
  explicit HeaderListener(const std::string & name = "header_default") : Node(name) {
    create_publisher<std_msgs::msg::String>("header_output", 10);
  }
};
