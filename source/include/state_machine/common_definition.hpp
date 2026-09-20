#ifndef COMMON_DEFINITION_HPP
#define COMMON_DEFINITION_HPP

#include <Eigen/Dense>
#include <mutex>

typedef enum StateID {
  INVALID           = -1,
  PASSIVE           = 10001,
  DAMPER            = 10002,
  RECOVERY_STAND    = 10003,
  RL_LOCOMOTION     = 11001,
  RL_TRACKING       = 12100,
} StateID;

static constexpr int NUM_MOTORS = 35;  // motor num

template <typename T>
struct RobotCommand {
    struct MotorCommand {
        Eigen::Matrix<T, NUM_MOTORS, 1> mode = Eigen::Matrix<T, NUM_MOTORS, 1>::Zero();
        Eigen::Matrix<T, NUM_MOTORS, 1> q = Eigen::Matrix<T, NUM_MOTORS, 1>::Zero();
        Eigen::Matrix<T, NUM_MOTORS, 1> dq = Eigen::Matrix<T, NUM_MOTORS, 1>::Zero();
        Eigen::Matrix<T, NUM_MOTORS, 1> tau = Eigen::Matrix<T, NUM_MOTORS, 1>::Zero();
        Eigen::Matrix<T, NUM_MOTORS, 1> kp = Eigen::Matrix<T, NUM_MOTORS, 1>::Zero();
        Eigen::Matrix<T, NUM_MOTORS, 1> kd = Eigen::Matrix<T, NUM_MOTORS, 1>::Zero();
    } motor_command;
};

template <typename T>
struct RobotState {
    struct IMU {
        // quat (w,x,y,z)
        Eigen::Quaternion<T> quaternion = Eigen::Quaternion<T>(1.0, 0.0, 0.0, 0.0);

        // gyro acc (3D)
        Eigen::Matrix<T, 3, 1> gyroscope = Eigen::Matrix<T, 3, 1>::Zero();
        Eigen::Matrix<T, 3, 1> accelerometer = Eigen::Matrix<T, 3, 1>::Zero();
        Eigen::Matrix<T, 3, 1> euler_zyx = Eigen::Matrix<T, 3, 1>::Zero();
    } imu;

    struct MotorState {
        // motor state
        Eigen::Matrix<T, NUM_MOTORS, 1> q = Eigen::Matrix<T, NUM_MOTORS, 1>::Zero();
        Eigen::Matrix<T, NUM_MOTORS, 1> dq = Eigen::Matrix<T, NUM_MOTORS, 1>::Zero();
        Eigen::Matrix<T, NUM_MOTORS, 1> ddq = Eigen::Matrix<T, NUM_MOTORS, 1>::Zero();
        Eigen::Matrix<T, NUM_MOTORS, 1> tau_est = Eigen::Matrix<T, NUM_MOTORS, 1>::Zero();
        Eigen::Matrix<T, NUM_MOTORS, 1> cur = Eigen::Matrix<T, NUM_MOTORS, 1>::Zero();
        Eigen::Matrix<T, NUM_MOTORS, 1> joint_id = Eigen::Matrix<T, NUM_MOTORS, 1>::Zero();
        Eigen::Matrix<T, NUM_MOTORS, 1> T_MOS = Eigen::Matrix<T, NUM_MOTORS, 1>::Zero();
        Eigen::Matrix<T, NUM_MOTORS, 1> T_Rotor = Eigen::Matrix<T, NUM_MOTORS, 1>::Zero();
        Eigen::Matrix<T, NUM_MOTORS, 1> error_code = Eigen::Matrix<T, NUM_MOTORS, 1>::Zero();
        Eigen::Matrix<T, NUM_MOTORS, 1> mode = Eigen::Matrix<T, NUM_MOTORS, 1>::Zero();
    } motor_state;
};

template <typename T>
struct DesiredCommand {
    StateID state_id;
    size_t motion_id;
    Eigen::Matrix<T, 3, 1> base_pos = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> base_rpy = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> base_lin_vel = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> base_ang_vel = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> head_rpy = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> head_duration = Eigen::Matrix<T, 3, 1>::Ones();
    Eigen::Matrix<T, 3, 1> raw_base_pos = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> raw_base_rpy = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> raw_base_lin_vel = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> raw_base_ang_vel = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> est_base_lin_vel = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> est_base_ang_vel = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> base_pos_min = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> base_pos_max = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> base_lin_vel_min = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> base_lin_vel_max = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> base_ang_vel_min = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> base_ang_vel_max = Eigen::Matrix<T, 3, 1>::Zero();
    double turn_degree = 0.0;
};

#endif  // COMMON_DEFINITION_HPP
