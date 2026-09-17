#include "hardware/low_command_publisher.hpp"

LowCommandPublisher::LowCommandPublisher(
  rclcpp::Node::SharedPtr node,
  const std::string& low_cmd_topic_name)
: node_(node)
{
  publisher_ = node_->create_publisher<interaction_msgs::msg::LowCommand>(
    low_cmd_topic_name, 10);
}

void LowCommandPublisher::publish(const interaction_msgs::msg::LowCommand& cmd)
{
  publisher_->publish(cmd);
}
