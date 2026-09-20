#ifndef LOW_COMMAND_PUBLISHER_HPP
#define LOW_COMMAND_PUBLISHER_HPP

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "interaction_msgs/msg/low_command.hpp"

class LowCommandPublisher {
   public:
    explicit LowCommandPublisher(rclcpp::Node::SharedPtr node, const std::string& low_cmd_topic_name);

    void publish(const interaction_msgs::msg::LowCommand& cmd);

   private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<interaction_msgs::msg::LowCommand>::SharedPtr publisher_;
};

#endif  // LOW_COMMAND_PUBLISHER_HPP
