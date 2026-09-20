#ifndef JOY_SUBSCRIBER_HPP
#define JOY_SUBSCRIBER_HPP

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

enum ButtonState {
    PRESS = 1,
    RELEASE = 0,
};

struct JoystickData {
    std::vector<float> axes;
    std::vector<int> buttons;

    ButtonState LT{RELEASE};
    ButtonState LB{RELEASE};
    ButtonState RT{RELEASE};
    ButtonState RB{RELEASE};

    ButtonState UP{RELEASE};
    ButtonState DOWN{RELEASE};
    ButtonState LEFT{RELEASE};
    ButtonState RIGHT{RELEASE};

    ButtonState A{RELEASE};
    ButtonState B{RELEASE};
    ButtonState X{RELEASE};
    ButtonState Y{RELEASE};

    ButtonState BACK{RELEASE};
    ButtonState START{RELEASE};

    ButtonState F1{RELEASE};
    ButtonState F2{RELEASE};

    float lx{0.0F};
    float ly{0.0F};
    float rx{0.0F};
    float ry{0.0F};
};

class JoySubscriber {
   public:
    explicit JoySubscriber(rclcpp::Node::SharedPtr node, const std::string& topic_name);
    ~JoySubscriber();
    void start();
    void stop();
    std::shared_ptr<JoystickData> get_latest_data();
    bool is_joy_init();
    bool is_joy_online();
    void set_joy_type(const std::string& joy_type);

   private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    bool warp_robot_remote_control_to_data(const sensor_msgs::msg::Joy::SharedPtr msg);
    bool warp_game_controller_to_data(const sensor_msgs::msg::Joy::SharedPtr msg);
    ButtonState parse_button(float in);
    ButtonState parse_axis(float in);
    ButtonState parse_key_map_button(int key_map, std::uint32_t mask);
    void print_joy_msg(const sensor_msgs::msg::Joy::SharedPtr msg);
    void print_joy_data();

    std::atomic_bool is_joy_init_{false};
    std::atomic_bool is_joy_online_{false};
    rclcpp::Node::SharedPtr node_;
    std::string joy_type_;
    mutable std::mutex data_mutex_;
    std::shared_ptr<JoystickData> joystick_data_ptr_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

#endif  // JOY_SUBSCRIBER_HPP
