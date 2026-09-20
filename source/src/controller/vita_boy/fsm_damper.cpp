#include "controller/vita_boy/fsm_damper.hpp"

#include <stdexcept>

FSMStateDamper::FSMStateDamper(std::shared_ptr<FSMData> fsm_data_ptr)
    : FSMState(fsm_data_ptr, StateID::DAMPER, "FSMStateDamper") {
    kp_ = Eigen::VectorXd::Zero(fsm_data_ptr_->num_dofs);
    kd_ = Eigen::VectorXd::Zero(fsm_data_ptr_->num_dofs);

    std::string config_path =
        "../config/" + fsm_data_ptr_->robot_name + "/" + fsm_data_ptr_->robot_version + "/damper/damper.yaml";
    read_yaml(config_path);
}

void FSMStateDamper::onEnter() {
    LOG(INFO) << "FSMStateDamper::onEnter()";
    counter_ = 0;
}

void FSMStateDamper::run() {
    counter_++;
    for (int i = 0; i < fsm_data_ptr_->num_dofs; ++i) {
        fsm_data_ptr_->robot_command_ptr->motor_command.q[i] = 0.0f;
        fsm_data_ptr_->robot_command_ptr->motor_command.dq[i] = 0.0f;
        fsm_data_ptr_->robot_command_ptr->motor_command.kp[i] = 0.0f;
        fsm_data_ptr_->robot_command_ptr->motor_command.kd[i] = kd_[i];
        fsm_data_ptr_->robot_command_ptr->motor_command.tau[i] = 0.0f;
    }

    for (int i = 0; i < fsm_data_ptr_->head_num_dofs; ++i) {
        fsm_data_ptr_->robot_command_ptr->motor_command.q[i + fsm_data_ptr_->num_dofs] = 0.0f;
        fsm_data_ptr_->robot_command_ptr->motor_command.dq[i + fsm_data_ptr_->num_dofs] = 0.0f;
        fsm_data_ptr_->robot_command_ptr->motor_command.kp[i + fsm_data_ptr_->num_dofs] = 0.0f;
        fsm_data_ptr_->robot_command_ptr->motor_command.kd[i + fsm_data_ptr_->num_dofs] = fsm_data_ptr_->head_kd[i];
        fsm_data_ptr_->robot_command_ptr->motor_command.tau[i + fsm_data_ptr_->num_dofs] = 0.0f;
    }
}

void FSMStateDamper::read_yaml(const std::string& config_path) {
    try {
        const YAML::Node config = YAML::LoadFile(config_path)[fsm_data_ptr_->robot_name];
        read_vector_from_yaml(config["kp"], kp_);
        read_vector_from_yaml(config["kd"], kd_);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Failed to load damper config '" + config_path + "': " + e.what());
    }

    const Eigen::Index expected_size = fsm_data_ptr_->num_dofs;
    if (kp_.size() != expected_size || kd_.size() != expected_size) {
        throw std::runtime_error("Invalid vector size in damper config '" + config_path +
                                 "': kp and kd must each contain " + std::to_string(expected_size) + " values");
    }

    if (!kp_.allFinite() || !kd_.allFinite()) {
        throw std::runtime_error("Invalid gain in damper config '" + config_path +
                                 "': kp and kd must contain only finite values");
    }

    if ((kd_.array() < 0.0).any()) {
        throw std::runtime_error("Invalid damping gain in damper config '" + config_path +
                                 "': kd values must be non-negative");
    }
}

void FSMStateDamper::onExit() {
    LOG(INFO) << "FSMStateDamper::onExit()";
}

StateID FSMStateDamper::check_transition() {
    next_state_id_ = state_id_;

    // Switch FSM control mode
    StateID state_id = fsm_data_ptr_->desired_command_ptr->state_id;
    switch (state_id) {
        case StateID::DAMPER:
            break;

        case StateID::PASSIVE:
        case StateID::RECOVERY_STAND:
            next_state_id_ = state_id;
            break;

        default:
            LOG(INFO) << "[CONTROL FSM] Bad Request: Cannot transition from " << StateID::DAMPER << " to " << state_id;
    }

    return next_state_id_;
}
