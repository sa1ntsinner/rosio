#include "deck/deck.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<deck::Deck>());
  rclcpp::shutdown();
  return 0;
}
