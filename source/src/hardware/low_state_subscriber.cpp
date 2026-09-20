#include "hardware/low_state_subscriber.hpp"

#include <functional>
#include <iomanip>

#include "common/glog_sink.hpp"

LowStateSubscriber::LowStateSubscriber(rclcpp::Node::SharedPtr node, const std::string& low_state_topic_name,
                                       const std::string& imu_topic_name) {
    low_state_subscription_ = node->create_subscription<interaction_msgs::msg::LowState>(
        low_state_topic_name, 10, std::bind(&LowStateSubscriber::low_state_callback, this, std::placeholders::_1));

    imu_subscription_ = node->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_name, 10, std::bind(&LowStateSubscriber::imu_callback, this, std::placeholders::_1));
}

LowStateSubscriber::~LowStateSubscriber() {
    stop();
}

void LowStateSubscriber::start() {}

void LowStateSubscriber::stop() {}

interaction_msgs::msg::LowState LowStateSubscriber::get_latest_msg() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_msg_;
}

sensor_msgs::msg::Imu LowStateSubscriber::get_latest_imu_msg() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_imu_msg_;
}

void LowStateSubscriber::low_state_callback(const interaction_msgs::msg::LowState::SharedPtr msg) {
    if (!is_low_state_init_) {
        is_low_state_init_ = true;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_msg_ = *msg;
}

void LowStateSubscriber::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (!is_imu_init_) {
        is_imu_init_ = true;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_imu_msg_ = *msg;
}

bool LowStateSubscriber::is_low_state_init() {
    return is_low_state_init_;
}

bool LowStateSubscriber::is_imu_init() {
    return is_imu_init_;
}

void LowStateSubscriber::print_imu_message(const sensor_msgs::msg::Imu::SharedPtr msg) {
    LOG(INFO) << "┌──────────────────────────────";
    LOG(INFO) << "│ IMU Message";
    LOG(INFO) << "├─ Header";
    LOG(INFO) << "│   Frame ID  : " << msg->header.frame_id;
    LOG(INFO) << "│   Timestamp : " << msg->header.stamp.sec << '.'
              << std::setfill('0') << std::setw(9) << msg->header.stamp.nanosec;

    LOG(INFO) << "├─ Orientation (x, y, z, w)";
    LOG(INFO) << "│   " << std::fixed << std::setprecision(6)
              << std::setw(12) << msg->orientation.x << ' '
              << std::setw(12) << msg->orientation.y << ' '
              << std::setw(12) << msg->orientation.z << ' '
              << std::setw(12) << msg->orientation.w;

    LOG(INFO) << "├─ Angular Velocity (x, y, z) [rad/s]";
    LOG(INFO) << "│   " << std::fixed << std::setprecision(6)
              << std::setw(12) << msg->angular_velocity.x << ' '
              << std::setw(12) << msg->angular_velocity.y << ' '
              << std::setw(12) << msg->angular_velocity.z;

    LOG(INFO) << "├─ Linear Acceleration (x, y, z) [m/s^2]";
    LOG(INFO) << "│   " << std::fixed << std::setprecision(6)
              << std::setw(12) << msg->linear_acceleration.x << ' '
              << std::setw(12) << msg->linear_acceleration.y << ' '
              << std::setw(12) << msg->linear_acceleration.z;
    LOG(INFO) << "└──────────────────────────────";
}

void LowStateSubscriber::print_low_state_msg(const interaction_msgs::msg::LowState& msg) {
    LOG(INFO) << "┌──────────────────────────────";
    LOG(INFO) << "│ LowState Message";
    LOG(INFO) << "├─ Version: " << static_cast<unsigned int>(msg.version);
    LOG(INFO) << "├─ Tick: " << msg.tick;
    LOG(INFO) << "├─ Mode: "
              << ((msg.mode_pr == msg.MODE_PR) ? "PR" : "RAW")
              << " ["
              << ((msg.mode_pr == msg.MODE_PR) ? "Position/Torque" : "Raw Motor")
              << ']';

    LOG(INFO) << "├─ Motor States:";
    LOG(INFO) << "│ " << std::left
              << std::setw(5) << "ID" << ' '
              << std::setw(8) << "Pos(rad)" << ' '
              << std::setw(10) << "Vel(rad/s)" << ' '
              << std::setw(10) << "Tau(Nm)";

    for (const auto& motor : msg.motor_state) {
        LOG(INFO) << "│ " << std::left
                          << std::setw(5) << static_cast<unsigned int>(motor.joint_id) << ' '
                          << std::fixed << std::setprecision(3)
                          << std::setw(8) << motor.pos_fb << ' '
                          << std::setw(10) << motor.vel_fb << ' '
                          << std::setw(10) << motor.tau_fb;
    }

    LOG(INFO) << "└──────────────────────────────";
}
