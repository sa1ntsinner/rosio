#include "split_nodes/nodes.hpp"

Talker::Talker(const std::string & topic)
: Node("talker"),
  topic_(topic) {}

void Talker::initialize()
{
  publisher_ = create_publisher<std_msgs::msg::String>(topic_, 10);
}

Filter::Filter()
: Node("filter"),
  gain_(0.5),
  offset_(1.0),
  window_(4) {}

void Filter::load_parameters()
{
  gain_ = this->declare_parameter("gain", gain_);
  offset_ = this->declare_parameter("offset", offset_);
  scale_ = this->declare_parameter("scale", scale_);
  window_ = this->declare_parameter("window", window_);
}

void Filter::reload()
{
  offset_ = 3.0;
}
