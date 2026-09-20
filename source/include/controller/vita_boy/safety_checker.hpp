#ifndef SAFETY_CHECKER_HPP
#define SAFETY_CHECKER_HPP

#include <cmath>
#include <Eigen/Dense>
#include "state_machine/common_definition.hpp"
#include "state_machine/fsm_data.hpp"
#include "common/helpers.hpp"

class SafetyChecker {
   public:
    SafetyChecker(std::shared_ptr<FSMData> fsm_data_ptr) {
        fsm_data_ptr_ = fsm_data_ptr;
    }

    bool pre_check() {
        bool orientation_safety = check_orientation(fsm_data_ptr_->robot_state_ptr->imu.quaternion);
        bool safety = orientation_safety;
        return safety;
    }

    void post_check() {}

    bool check_imu_quaternion_norm() {
        double norm = fsm_data_ptr_->robot_state_ptr->imu.quaternion.norm();
        if (norm > 1.2 || norm < 0.8) {
            const auto& coeffs = fsm_data_ptr_->robot_state_ptr->imu.quaternion.coeffs();
            LOG(ERROR)
                << "[SAFETY CHECKER] failed! imu quaternion norm is larger or smaller than 1.0, the current norm is "
                << norm;
            LOG(ERROR) << "[SAFETY CHECKER] Current quaternion wxyz is: " 
                       << coeffs[3] << ", " // w
                       << coeffs[0] << ", " // x
                       << coeffs[1] << ", " // y
                       << coeffs[2];        // z
            return false;
        }
        return true;
    }

    bool check_motor_error_code() {
        const auto& error_code = fsm_data_ptr_->robot_state_ptr->motor_state.error_code;
        bool safety = true;

        if (has_printed_error_.size() != error_code.size()) {
            has_printed_error_.resize(error_code.size(), false);
        }

        for (int i = 0; i < error_code.size(); i++) {
            if (error_code[i] > 0) {
                safety = false;
                if (!has_printed_error_[i]) {
                    if (print_check_info_) {
                        LOG(ERROR) << "[SAFETY CHECKER] failed! joint " << i << " error code is " << error_code[i];
                    }
                    has_printed_error_[i] = true;
                }
            }
        }
        return safety;
    }

    bool check_motor_mos_temperature() {
        const auto& T_MOS = fsm_data_ptr_->robot_state_ptr->motor_state.T_MOS;
        bool safety = true;

        if (has_printed_t_mos_.size() != T_MOS.size()) {
            has_printed_t_mos_.resize(T_MOS.size(), false);
        }

        for (int i = 0; i < T_MOS.size(); i++) {
            if (T_MOS[i] > 100) {
                safety = false;
                if (!has_printed_t_mos_[i]) {
                    if (print_check_info_) {
                        LOG(ERROR) << "[SAFETY CHECKER] failed! joint " << i << " mos temperature is " << T_MOS[i];
                    }
                    has_printed_t_mos_[i] = true;
                }
            }
        }
        return safety;
    }

    bool check_motor_rotor_temperature() {
        const auto& T_rotor = fsm_data_ptr_->robot_state_ptr->motor_state.T_Rotor;
        bool safety = true;

        if (has_printed_t_rotor_.size() != T_rotor.size()) {
            has_printed_t_rotor_.resize(T_rotor.size(), false);
        }

        for (int i = 0; i < T_rotor.size(); i++) {
            if (T_rotor[i] > 100) {
                safety = false;
                if (!has_printed_t_rotor_[i]) {
                    if (print_check_info_) {
                        LOG(ERROR) << "[SAFETY CHECKER] failed! joint " << i << " rotor temperature is " << T_rotor[i];
                    }
                    has_printed_t_rotor_[i] = true;
                }
            }
        }
        return safety;
    }

    bool check_orientation(const Eigen::Quaterniond& quaternion) {
        bool safety = true;
        auto euler_zyx = quaternion_to_euler_zyx(quaternion);

        if (euler_zyx(1) > 80.0 / 180.0 * M_PI || euler_zyx(1) < -80.0 / 180.0 * M_PI) {
            if (print_check_info_) {
                LOG(ERROR) << "[SAFETY CHECKER] Orientation safety check failed!";
                const auto& coeffs = quaternion.coeffs();
                LOG(ERROR) << "[SAFETY CHECKER] Current quaternion wxyz is: "
                           << coeffs[3] << ", " // w
                           << coeffs[0] << ", " // x
                           << coeffs[1] << ", " // y
                           << coeffs[2];        // z
                LOG(ERROR) << "[SAFETY CHECKER] Pitch limit (-PI*80/180, PI*80/180), actual orientation Pitch angle: "
                           << euler_zyx(1);
            }
            safety = false;
        } else if (euler_zyx(2) > 50.0 / 180.0 * M_PI || euler_zyx(2) < -50.0 / 180.0 * M_PI) {
            if (print_check_info_) {
                LOG(ERROR) << "[SAFETY CHECKER] Orientation safety check failed!";
                const auto& coeffs = quaternion.coeffs();
                LOG(ERROR) << "[SAFETY CHECKER] Current quaternion wxyz is: " 
                           << coeffs[3] << ", " // w
                           << coeffs[0] << ", " // x
                           << coeffs[1] << ", " // y
                           << coeffs[2];        // z
                LOG(ERROR) << "[SAFETY CHECKER] Roll limit (-PI*50/180, PI*50/180), actual orientation Roll angle: "
                           << euler_zyx(2);
            }
            safety = false;
        }
        return safety;
    }

   private:
    bool print_check_info_{true};
    std::vector<bool> has_printed_error_;
    std::vector<bool> has_printed_t_mos_;
    std::vector<bool> has_printed_t_rotor_;

    std::shared_ptr<FSMData> fsm_data_ptr_;
};

#endif  // SAFETY_CHECKER_HPP
