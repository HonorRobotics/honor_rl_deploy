#include "controller/vita_boy/fsm_passive.hpp"

FSMStatePassive::FSMStatePassive(std::shared_ptr<FSMData> fsm_data_ptr)
    : FSMState(fsm_data_ptr, StateID::PASSIVE, "FSMStatePassive") {}

void FSMStatePassive::onEnter() {
    LOG(INFO) << "FSMStatePassive::onEnter()";
    counter_ = 0;
}

void FSMStatePassive::run() {
    counter_++;

    fsm_data_ptr_->robot_command_ptr->motor_command.q.setZero();
    fsm_data_ptr_->robot_command_ptr->motor_command.dq.setZero();
    fsm_data_ptr_->robot_command_ptr->motor_command.kp.setZero();
    fsm_data_ptr_->robot_command_ptr->motor_command.kd.setZero();
    fsm_data_ptr_->robot_command_ptr->motor_command.tau.setZero();
}

void FSMStatePassive::onExit() {
    LOG(INFO) << "FSMStatePassive::onExit()";
}

StateID FSMStatePassive::check_transition() {
    next_state_id_ = state_id_;

    // Switch FSM control mode
    StateID state_id = fsm_data_ptr_->desired_command_ptr->state_id;
    switch (state_id) {
        case StateID::PASSIVE:
            break;

        case StateID::DAMPER:
        case StateID::RECOVERY_STAND:
            next_state_id_ = state_id;
            break;

        default:
            LOG(ERROR) << "[CONTROL FSM] Bad Request: Cannot transition from " << StateID::PASSIVE << " to "
                       << state_id;
    }

    // Get the next state
    return next_state_id_;
}
