#ifndef LOW_STATE_SUBSCRIBER_HPP
#define LOW_STATE_SUBSCRIBER_HPP

#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "interaction_msgs/msg/low_state.hpp"

class LowStateSubscriber
{
public:
  explicit LowStateSubscriber(
    rclcpp::Node::SharedPtr node,
    const std::string& low_state_topic_name,
    const std::string& imu_topic_name);
  ~LowStateSubscriber();

  void start();
  void stop();
  interaction_msgs::msg::LowState get_latest_msg() const;
  sensor_msgs::msg::Imu get_latest_imu_msg() const;
  bool is_low_state_init();
  bool is_imu_init();

private:
  void low_state_callback(const interaction_msgs::msg::LowState::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void print_low_state_msg(const interaction_msgs::msg::LowState& msg);
  void print_imu_message(const sensor_msgs::msg::Imu::SharedPtr msg);

  bool is_low_state_init_{false};
  bool is_imu_init_{false};
  mutable std::mutex data_mutex_;
  interaction_msgs::msg::LowState latest_msg_;
  sensor_msgs::msg::Imu latest_imu_msg_;
  rclcpp::Subscription<interaction_msgs::msg::LowState>::SharedPtr low_state_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
};

#endif // LOW_STATE_SUBSCRIBER_HPP
