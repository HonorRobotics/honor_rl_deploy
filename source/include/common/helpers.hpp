#ifndef HELPERS_HPP
#define HELPERS_HPP

#include <chrono>
#include <string>
#include <cassert>
#include <thread>
#include <pthread.h>
#include <numeric>

#include <iostream>
#include <vector>
#include <iomanip>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include "common/glog_sink.hpp"

namespace constants {
constexpr double PI = 3.14159265358979323846;
}

/**
 * Helper function to execute a given function and sleep for the remainder of the specified timing.
 * Will not interrupt the function if it is too slow.
 *
 * @tparam Functor : function type
 * @param f : callable object to execute
 * @param frequency : the frequency the function should run at.
 */
template <typename Functor>
void execute_and_sleep(Functor f, double frequency) {
    using clock = std::chrono::high_resolution_clock;
    const auto start = clock::now();

    // Execute wrapped function
    f();

    // Compute desired duration rounded to clock decimation
    const std::chrono::duration<double> desiredDuration(1.0 / frequency);
    const auto dt = std::chrono::duration_cast<clock::duration>(desiredDuration);

    // Sleep
    const auto sleepTill = start + dt;
    std::this_thread::sleep_until(sleepTill);
}

inline double clamp(double val, double lim1, double lim2) {
    auto min = std::min(lim1, lim2);
    auto max = std::max(lim1, lim2);
    return std::max(min, std::min(val, max));
}

template <typename T>
inline T clamp(T val, T low, T high) {
    if (low > high)
        std::swap(low, high);
    return (val < low) ? low : (val > high) ? high : val;
}

inline double first_order_filter(double current, double target, double alpha) {
    return alpha * target + (1 - alpha) * current;
}

// Convert a quaternion to a rotation matrix, normalizing first.
inline Eigen::Matrix3d quaternion_to_matrix(const Eigen::Quaterniond& q_in) {
    Eigen::Quaterniond q = q_in;
    if (q.w() < 0.0)
        q.coeffs() *= -1.0;

    q.normalize();

    return q.toRotationMatrix();
}

// Convert a quaternion to ZYX Euler angles [yaw, pitch, roll].
inline Eigen::Vector3d quaternion_to_euler_zyx(const Eigen::Quaterniond& q_in) {
    Eigen::Quaterniond q = q_in.normalized();

    double x = q.x();
    double y = q.y();
    double z = q.z();
    double w = q.w();

    double as_ = -2.0 * (x * z - w * y);
    as_ = std::clamp(as_, -0.99999, 0.99999);

    Eigen::Vector3d zyx;

    zyx[0] = std::atan2(2.0 * (x * y + w * z), w * w + x * x - y * y - z * z);
    zyx[1] = std::asin(as_);
    zyx[2] = std::atan2(2.0 * (y * z + w * x), w * w - x * x - y * y + z * z);

    return zyx;
}

// Apply a roll/pitch bias rotation on top of a quaternion.
inline Eigen::Quaterniond add_roll_pitch_bias_to_quaternion(const Eigen::Quaterniond& q_in, double roll_bias,
                                                            double pitch_bias) {
    Eigen::AngleAxisd rollAngle(roll_bias, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch_bias, Eigen::Vector3d::UnitY());
    Eigen::Quaterniond q_bias = pitchAngle * rollAngle;

    Eigen::Quaterniond q_new = q_in * q_bias;

    return q_new.normalized();
}

// Rotate a vector by the inverse of a quaternion.
inline Eigen::Vector3d quat_rotate_inverse(Eigen::Quaternion<double>& quat, Eigen::Vector3d& vec) {
    Eigen::Vector3d res;
    double s = quat.w();
    Eigen::Vector3d v(quat.x(), quat.y(), quat.z());
    Eigen::Vector3d vxp = v.cross(vec);
    double vdp = v.dot(vec);
    res = 2 * vdp * v + (2 * s * s - 1) * vec - 2 * s * vxp;
    return res;
}

// Quaternion inverse, falling back to identity for a near-zero quaternion.
inline Eigen::Quaterniond quat_inverse(const Eigen::Quaterniond& q, double eps = 1e-9) {
    double norm_sq = q.squaredNorm();
    if (norm_sq < eps) {
        return Eigen::Quaterniond::Identity();
    }
    return Eigen::Quaterniond(q.conjugate().coeffs() / norm_sq);
}

// Percentage of current/total, clamped to [0, 100].
template <typename T, typename U>
int get_percentage(T current, U total) {
    static_assert(std::is_arithmetic<T>::value && std::is_arithmetic<U>::value,
                  "get_percentage requires arithmetic types");

    if (total <= 0)
        return 0;
    if (current <= 0)
        return 0;
    if (current >= total)
        return 100;

    double percent = static_cast<double>(current) / static_cast<double>(total) * 100;
    return static_cast<int>(std::round(percent));
}

// Shortest signed angular distance from `from` to `to`, in [-π, π].
template <typename T>
inline typename std::enable_if<std::is_floating_point<T>::value, T>::type shortest_angular_distance(T from, T to) {
    static_assert(std::is_floating_point<T>::value, "shortest_angular_distance requires floating-point types");

    T diff = to - from;
    diff = std::fmod(diff, 2.0 * M_PI);

    if (diff > M_PI) {
        diff -= 2.0 * M_PI;
    } else if (diff < -M_PI) {
        diff += 2.0 * M_PI;
    }

    return diff;
}

// Square a value.
template <typename T>
T square(T a) {
    return a * a;
}

// Convert a quaternion to ZYX Euler angles [yaw, pitch, roll].
template <typename T>
Eigen::Matrix<T, 3, 1> quat_to_euler_zyx(const Eigen::Quaternion<T>& q) {
    Eigen::Matrix<T, 3, 1> zyx;

    T as = std::min(std::max(-1.0, -2. * (q.x() * q.z() - q.w() * q.y())), 1.0);
    zyx(0) =
        std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
    zyx(1) = std::asin(as);
    zyx(2) =
        std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
    return zyx;
}

// Quaternion multiplication (optimized form).
template <typename T>
Eigen::Quaternion<T> quaternion_multiplication_optimized(const Eigen::Quaternion<T>& q1,
                                                         const Eigen::Quaternion<T>& q2) {
    T w1 = q1.w();
    T x1 = q1.x();
    T y1 = q1.y();
    T z1 = q1.z();

    T w2 = q2.w();
    T x2 = q2.x();
    T y2 = q2.y();
    T z2 = q2.z();

    T ww = (z1 + x1) * (x2 + y2);
    T yy = (w1 - y1) * (w2 + z2);
    T zz = (w1 + y1) * (w2 - z2);
    T xx = ww + yy + zz;
    T qq = static_cast<T>(0.5) * (xx + (z1 - x1) * (x2 - y2));

    T w = qq - ww + (z1 - y1) * (y2 - z2);
    T x = qq - xx + (x1 + w1) * (x2 + w2);
    T y = qq - yy + (w1 - x1) * (y2 + z2);
    T z = qq - zz + (z1 + y1) * (w2 - x2);

    return Eigen::Quaternion<T>(w, x, y, z).normalized();
}

// Bind the calling thread to the given set of CPU cores.
inline bool bind_current_thread_to_cpu_ids(const std::vector<int>& core_ids) {
    if (core_ids.empty()) {
        LOG(ERROR) << "[ERROR]: Empty core IDs list";
        return false;
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);

    for (int core_id : core_ids) {
        CPU_SET(core_id, &cpuset);
    }

    pthread_t current_thread = pthread_self();
    if (pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset) != 0) {
        LOG(ERROR) << "[ERROR]: Failed to bind thread to CPUs";
        return false;
    }

    std::stringstream ss;
    for (size_t i = 0; i < core_ids.size(); ++i) {
        if (i != 0)
            ss << ", ";
        ss << core_ids[i];
    }

    LOG(INFO) << "[INFO]: Bind thread to CPUs: " << ss.str();
    return true;
}

// Cosine-smooth from pos_init to des_pos over transition_time; counter*dt is the elapsed time.
inline Eigen::VectorXd forder_cos_smooth(const Eigen::VectorXd des_pos, const Eigen::VectorXd pos_init, double dt,
                                         int counter, double transition_time) {
    Eigen::VectorXd pos_cmd = Eigen::VectorXd::Zero(des_pos.size());
    double current_time = counter * dt;
    double phase = std::max(std::min(current_time / transition_time, 1.0), 0.0);
    for (int i = 0; i < des_pos.size(); ++i) {
        pos_cmd[i] = 0.5 * (pos_init[i] - des_pos[i]) * std::cos(M_PI * phase) + 0.5 * (pos_init[i] + des_pos[i]);
    }
    return pos_cmd;
}

#endif  // HELPERS_HPP
