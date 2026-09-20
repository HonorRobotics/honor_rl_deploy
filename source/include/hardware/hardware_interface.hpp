#ifndef HARDWARE_INTERFACE_HPP
#define HARDWARE_INTERFACE_HPP

#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "state_machine/common_definition.hpp"
#include "state_machine/fsm_data.hpp"
#include "hardware/joy_subscriber.hpp"
#include "hardware/keyboard_interface.hpp"
#include "hardware/low_command_publisher.hpp"
#include "hardware/low_state_subscriber.hpp"

class HardwareInterface {
   public:
    HardwareInterface(const std::string& config_path, std::shared_ptr<FSMData> fsm_data_ptr);
    ~HardwareInterface();
    void set_command(const Eigen::VectorXd& pos, const Eigen::VectorXd& vel, const Eigen::VectorXd& kp,
                     const Eigen::VectorXd& kd, const Eigen::VectorXd& tau);
    std::shared_ptr<RobotState<double>> get_state();
    std::shared_ptr<JoystickData> get_joy_state();
    bool get_joy_online();
    void read_hw_yaml(const std::string& config_path);
    // Handles both simulation and real-robot IMU, they share one code path.
    void update_imu_state(const sensor_msgs::msg::Imu& imu);
    bool is_init();
    bool is_imu_init();
    bool is_low_state_init();
    bool is_joy_init();

   protected:
   private:
    bool validate_motor_command_dimensions(const Eigen::VectorXd& pos, const Eigen::VectorXd& vel,
                                           const Eigen::VectorXd& kp, const Eigen::VectorXd& kd,
                                           const Eigen::VectorXd& tau) const;

    int hw_motor_dim_{0};
    std::shared_ptr<FSMData> fsm_data_ptr_;
    std::shared_ptr<RobotState<double>> robot_state_ptr_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread spin_thread_;
    std::shared_ptr<LowStateSubscriber> low_state_subscriber_ptr_;
    std::shared_ptr<LowCommandPublisher> low_command_publisher_ptr_;
    std::shared_ptr<JoySubscriber> joy_subscriber_ptr_;
    std::shared_ptr<KeyboardInterface> keyboard_interface_ptr_;

    Eigen::Vector3d imu_zyx_bias_;

    std::string low_cmd_topic_;
    std::string low_state_topic_;
    std::string imu_topic_;
    std::string joy_topic_;
    std::string joy_type_;
};

#endif  // HARDWARE_INTERFACE_HPP
