#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "hardware/hardware_interface.hpp"
#include "common/helpers.hpp"
#include "common/glog_sink.hpp"

HardwareInterface::HardwareInterface(const std::string &config_path, std::shared_ptr<FSMData> fsm_data_ptr)
{
    fsm_data_ptr_ = fsm_data_ptr;
    read_hw_yaml(config_path);

    // init ROS2 Node
    rclcpp::NodeOptions options;
    options.start_parameter_event_publisher(false);
    options.start_parameter_services(false);
    node_ = std::make_shared<rclcpp::Node>("motion_intelligence", options);

    low_state_subscriber_ptr_ = std::make_shared<LowStateSubscriber>(node_, low_state_topic_, imu_topic_);
    low_command_publisher_ptr_ = std::make_shared<LowCommandPublisher>(node_, low_cmd_topic_);
    robot_state_ptr_ = std::make_shared<RobotState<double>>();

    low_state_subscriber_ptr_->start();

    if (joy_type_ == "keyboard") {
        keyboard_interface_ptr_ = std::make_shared<KeyboardInterface>();
        keyboard_interface_ptr_->start();
    } else {
        joy_subscriber_ptr_ = std::make_shared<JoySubscriber>(node_, joy_topic_);
        joy_subscriber_ptr_->start();
        joy_subscriber_ptr_->set_joy_type(joy_type_);
    }

    // ROS2 single thread
    executor_.add_node(node_);
    spin_thread_ = std::thread([this]() {
        try {
            executor_.spin();
        } catch (const std::exception& e) {
            LOG(ERROR) << "[HardwareInterface] ROS executor stopped with exception: "
                       << e.what();
        }
    });
}

void HardwareInterface::set_command(const Eigen::VectorXd& pos,
                                    const Eigen::VectorXd& vel,
                                    const Eigen::VectorXd& kp,
                                    const Eigen::VectorXd& kd,
                                    const Eigen::VectorXd& tau)
{
    // check motor dim
    if (!validate_motor_command_dimensions(pos, vel, kp, kd, tau)) {
        return;
    }

    auto low_command = interaction_msgs::msg::LowCommand();
    low_command.header.stamp = node_->get_clock()->now();
    low_command.mode_pr = 0;    // MODE_PR = 0; MODE_RAW = 1;

    for (int i = 0; i < hw_motor_dim_; ++i) {
        interaction_msgs::msg::MotorCommand md;
        md.joint_id = robot_state_ptr_->motor_state.joint_id[i];
        md.cmd_mode = 0x19;
        md.pos = pos[i];
        md.vel = vel[i];
        md.kp = kp[i];
        md.kd = kd[i];
        md.tau = tau[i];
        low_command.motor_cmd.push_back(md);
    }
    low_command_publisher_ptr_->publish(low_command);
}

bool HardwareInterface::validate_motor_command_dimensions(
    const Eigen::VectorXd& pos,
    const Eigen::VectorXd& vel,
    const Eigen::VectorXd& kp,
    const Eigen::VectorXd& kd,
    const Eigen::VectorXd& tau) const
{
    if (!low_state_subscriber_ptr_->is_low_state_init()) {
        LOG_EVERY_N(WARNING, 500)
            << "[HardwareInterface] Skip motor command: LowState is not ready";
        return false;
    }

    if (hw_motor_dim_ <= 0 || hw_motor_dim_ > NUM_MOTORS) {
        LOG_EVERY_N(ERROR, 500)
            << "[HardwareInterface] Skip motor command: invalid motor count "
            << hw_motor_dim_ << ", expected in [1, " << NUM_MOTORS << "]";
        return false;
    }

    const Eigen::Index required = static_cast<Eigen::Index>(hw_motor_dim_);
    if (pos.size() < required ||
        vel.size() < required ||
        kp.size() < required ||
        kd.size() < required ||
        tau.size() < required) {
        LOG_EVERY_N(ERROR, 500)
            << "[HardwareInterface] Skip motor command: required=" << required
            << ", pos=" << pos.size()
            << ", vel=" << vel.size()
            << ", kp=" << kp.size()
            << ", kd=" << kd.size()
            << ", tau=" << tau.size();
        return false;
    }

    if (!pos.head(required).allFinite() ||
        !vel.head(required).allFinite() ||
        !kp.head(required).allFinite() ||
        !kd.head(required).allFinite() ||
        !tau.head(required).allFinite()) {
        LOG_EVERY_N(ERROR, 500)
            << "[HardwareInterface] Skip motor command: command contains NaN or Inf";
        return false;
    }

    return true;
}

std::shared_ptr<RobotState<double>> HardwareInterface::get_state()
{
    auto low_state = low_state_subscriber_ptr_->get_latest_msg();
    auto imu = low_state_subscriber_ptr_->get_latest_imu_msg();

    fsm_data_ptr_->mode_machine = low_state.mode_machine;
    if (!low_state.motor_state.empty() &&
        low_state.motor_state.size() <= NUM_MOTORS) {
        hw_motor_dim_ = static_cast<int>(low_state.motor_state.size());
        for(int i = 0; i < low_state.motor_state.size(); ++i) {
            robot_state_ptr_->motor_state.q[i] = low_state.motor_state[i].pos_fb;
            robot_state_ptr_->motor_state.dq[i] = low_state.motor_state[i].vel_fb;
            robot_state_ptr_->motor_state.tau_est[i] = low_state.motor_state[i].tau_fb;
            robot_state_ptr_->motor_state.joint_id[i] = low_state.motor_state[i].joint_id;
            robot_state_ptr_->motor_state.mode[i] = low_state.motor_state[i].mode;
            robot_state_ptr_->motor_state.T_MOS[i] = low_state.motor_state[i].temp_fb[0];
            robot_state_ptr_->motor_state.T_Rotor[i] = low_state.motor_state[i].temp_fb[1];
            robot_state_ptr_->motor_state.error_code[i] = low_state.motor_state[i].error_code;
        }

    } else {
        hw_motor_dim_ = 0;
        LOG_EVERY_N(ERROR, 500)
            << "[HardwareInterface] Invalid motor state count: "
            << low_state.motor_state.size()
            << ", expected in [1, " << NUM_MOTORS << "]";
    }

    if (fsm_data_ptr_->is_sim &&
        static_cast<std::int32_t>(low_state.mode_machine) >= 0) {
        LOG(ERROR) << "[HardwareInterface] current mode_machine is "
                   << low_state.mode_machine
                   << ", real robot is online, please set is_sim to false!";
    }

    update_imu_state(imu);

    return robot_state_ptr_;
}

std::shared_ptr<JoystickData> HardwareInterface::get_joy_state()
{
    if (keyboard_interface_ptr_) {
        return keyboard_interface_ptr_->get_latest_data();
    }
    return joy_subscriber_ptr_->get_latest_data();
}

bool HardwareInterface::get_joy_online()
{
    if (keyboard_interface_ptr_) {
        return keyboard_interface_ptr_->is_joy_online();
    }
    return joy_subscriber_ptr_->is_joy_online();
}

bool HardwareInterface::is_init()
{
    return low_state_subscriber_ptr_->is_imu_init() &&
            low_state_subscriber_ptr_->is_low_state_init() &&
            is_joy_init();
}

bool HardwareInterface::is_imu_init()
{
    return low_state_subscriber_ptr_->is_imu_init();
}

bool HardwareInterface::is_low_state_init()
{
    return low_state_subscriber_ptr_->is_low_state_init();
}

bool HardwareInterface::is_joy_init()
{
    if (keyboard_interface_ptr_) {
        return keyboard_interface_ptr_->is_joy_init();
    }
    return joy_subscriber_ptr_->is_joy_init();
}

void HardwareInterface::update_imu_state(const sensor_msgs::msg::Imu& imu)
{
    Eigen::Quaterniond quaternion(
        imu.orientation.w,
        imu.orientation.x,
        imu.orientation.y,
        imu.orientation.z);

    constexpr double kMinQuaternionNorm = 1e-6;
    const double quaternion_norm = quaternion.norm();
    if (imu.orientation_covariance[0] < 0.0 ||
        !quaternion.coeffs().allFinite() ||
        !std::isfinite(quaternion_norm) ||
        quaternion_norm < kMinQuaternionNorm) {
        LOG_EVERY_N(ERROR, 500)
            << "[HardwareInterface] Invalid IMU orientation; keeping previous IMU state";
        return;
    }
    quaternion.normalize();

    const Eigen::Quaterniond new_quaternion = add_roll_pitch_bias_to_quaternion(
        quaternion, imu_zyx_bias_[2], imu_zyx_bias_[1]);

    robot_state_ptr_->imu.quaternion = new_quaternion;
    robot_state_ptr_->imu.euler_zyx = quaternion_to_euler_zyx(new_quaternion);
    robot_state_ptr_->imu.gyroscope <<
        imu.angular_velocity.x,
        imu.angular_velocity.y,
        imu.angular_velocity.z;
    robot_state_ptr_->imu.accelerometer <<
        imu.linear_acceleration.x,
        imu.linear_acceleration.y,
        imu.linear_acceleration.z;
}

void HardwareInterface::read_hw_yaml(const std::string &config_path)
{
    if (!fsm_data_ptr_) {
        throw std::invalid_argument("FSM data pointer cannot be null");
    }

    try {
        const YAML::Node root = YAML::LoadFile(config_path);
        const std::string& robot_name = fsm_data_ptr_->robot_name;
        const YAML::Node config = root[robot_name];
        if (!config || !config.IsMap()) {
            throw std::runtime_error(
                "Robot configuration '" + robot_name + "' is missing in '" +
                config_path + "'");
        }

        const std::string low_cmd_topic = config["lowcmd_topic"].as<std::string>();
        const std::string low_state_topic = config["lowstate_topic"].as<std::string>();
        const std::string imu_topic = config["imu_topic"].as<std::string>();
        const std::string joy_topic = config["joy_topic"].as<std::string>();
        const std::string joy_type = config["joy_type"].as<std::string>();
        const double imu_yaw_bias = config["imu_yaw_bias"].as<double>();
        const double imu_pitch_bias = config["imu_pitch_bias"].as<double>();
        const double imu_roll_bias = config["imu_roll_bias"].as<double>();

        low_cmd_topic_ = low_cmd_topic;
        low_state_topic_ = low_state_topic;
        imu_topic_ = imu_topic;
        joy_topic_ = joy_topic;
        joy_type_ = joy_type;
        imu_zyx_bias_ << imu_yaw_bias, imu_pitch_bias, imu_roll_bias;
    } catch (const YAML::Exception& error) {
        throw std::runtime_error(
            "Failed to read hardware configuration '" + config_path +
            "': " + error.what());
    }
}

HardwareInterface::~HardwareInterface() {
    executor_.cancel();
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
    if (node_) {
        executor_.remove_node(node_);
    }

    low_state_subscriber_ptr_->stop();
    if (joy_subscriber_ptr_) {
        joy_subscriber_ptr_->stop();
    }
    if (keyboard_interface_ptr_) {
        keyboard_interface_ptr_->stop();
    }
}
