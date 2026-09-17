#include "controller/vita_boy/fsm_recovery_stand.hpp"

#include <cmath>
#include <stdexcept>

FSMStateRecoveryStand::FSMStateRecoveryStand(std::shared_ptr<FSMData> fsm_data_ptr)
    : FSMState(fsm_data_ptr, StateID::RECOVERY_STAND, "FSMStateRecoveryStand") {

  default_dof_pos_ = Eigen::VectorXd::Zero(fsm_data_ptr_->num_dofs);
  kp_ = Eigen::VectorXd::Zero(fsm_data_ptr_->num_dofs);
  kd_ = Eigen::VectorXd::Zero(fsm_data_ptr_->num_dofs);

  std::string config_path = "../config/" + fsm_data_ptr_->robot_name + "/" + fsm_data_ptr_->robot_version + "/recovery_stand/recovery_stand.yaml";
  read_yaml(config_path);
}

void FSMStateRecoveryStand::onEnter() {
  LOG(INFO) << "FSMStateRecoveryStand::onEnter()";
  counter_ = 0;

  // get initial joint state
  q_init_ = Eigen::VectorXd::Zero(fsm_data_ptr_->num_dofs);
  for (size_t i = 0; i < fsm_data_ptr_->num_dofs; ++i) {
    q_init_[i] = fsm_data_ptr_->robot_state_ptr->motor_state.q[i];
  }
  // get head initial head joint state
  head_q_init_ = Eigen::VectorXd::Zero(fsm_data_ptr_->head_num_dofs);
  for (size_t i = 0; i < fsm_data_ptr_->head_num_dofs; ++i) {
    head_q_init_[i] = fsm_data_ptr_->robot_state_ptr->motor_state.q[i+fsm_data_ptr_->num_dofs];
  }
}

void FSMStateRecoveryStand::run() {
  counter_++;

  double recovery_stand_time = counter_ * fsm_data_ptr_->control_dt;
  double phase = std::max(std::min( recovery_stand_time / transition_time_, 1.0), 0.0);

  Eigen::VectorXd q_des = Eigen::VectorXd::Zero(fsm_data_ptr_->num_dofs);
  Eigen::VectorXd dq_des = Eigen::VectorXd::Zero(fsm_data_ptr_->num_dofs);

  for (size_t i = 0; i < fsm_data_ptr_->num_dofs; ++i) {
    q_des[i] = 0.5 * (q_init_[i] - default_dof_pos_[i]) * std::cos(M_PI * phase) + 0.5 * (q_init_[i] + default_dof_pos_[i]);
    dq_des[i] = 0.5 * (q_init_[i] - default_dof_pos_[i]) * (-M_PI * std::sin(M_PI * phase) / transition_time_);
  }

  Eigen::VectorXd head_q_des = Eigen::VectorXd::Zero(fsm_data_ptr_->head_num_dofs);
  Eigen::VectorXd head_dq_des = Eigen::VectorXd::Zero(fsm_data_ptr_->head_num_dofs);
  for (size_t i = 0; i < fsm_data_ptr_->head_num_dofs; ++i) {
    head_q_des[i] = 0.5 * (head_q_init_[i] - 0.0) * std::cos(M_PI * phase) + 0.5 * (head_q_init_[i] + 0.0);
    head_dq_des[i] = 0.5 * (head_q_init_[i] - 0.0) * (-M_PI * std::sin(M_PI * phase) / transition_time_);
  }

  for(int i = 0; i < fsm_data_ptr_->num_dofs; ++i) {
      fsm_data_ptr_->robot_command_ptr->motor_command.q[i] = q_des[i];
      fsm_data_ptr_->robot_command_ptr->motor_command.dq[i] = 0.0f;
      fsm_data_ptr_->robot_command_ptr->motor_command.kp[i] = kp_[i];
      fsm_data_ptr_->robot_command_ptr->motor_command.kd[i] = kd_[i];
      fsm_data_ptr_->robot_command_ptr->motor_command.tau[i] = 0.0f;
  }

  for(int i = 0; i < fsm_data_ptr_->head_num_dofs; ++i) {
      fsm_data_ptr_->robot_command_ptr->motor_command.q[i+fsm_data_ptr_->num_dofs] = head_q_des[i];
      fsm_data_ptr_->robot_command_ptr->motor_command.dq[i+fsm_data_ptr_->num_dofs] = 0.0f;
      fsm_data_ptr_->robot_command_ptr->motor_command.kp[i+fsm_data_ptr_->num_dofs] = fsm_data_ptr_->head_kp[i];
      fsm_data_ptr_->robot_command_ptr->motor_command.kd[i+fsm_data_ptr_->num_dofs] = fsm_data_ptr_->head_kd[i];
      fsm_data_ptr_->robot_command_ptr->motor_command.tau[i+fsm_data_ptr_->num_dofs] = 0.0f;
  }
}

void FSMStateRecoveryStand::read_yaml(const std::string &config_path)
{
  try {
    const YAML::Node config = YAML::LoadFile(config_path)[fsm_data_ptr_->robot_name];
    transition_time_ = config["transition_time"].as<double>();
    read_vector_from_yaml(config["default_dof_pos"], default_dof_pos_);
    read_vector_from_yaml(config["kp"], kp_);
    read_vector_from_yaml(config["kd"], kd_);
  } catch (const YAML::Exception &e) {
    throw std::runtime_error(
        "Failed to load recovery stand config '" + config_path + "': " + e.what());
  }

  if (!std::isfinite(transition_time_) || transition_time_ <= 0.0) {
    throw std::runtime_error(
        "Invalid transition_time in recovery stand config '" + config_path +
        "': expected a finite value greater than zero");
  }

  const Eigen::Index expected_size = fsm_data_ptr_->num_dofs;
  if (default_dof_pos_.size() != expected_size ||
      kp_.size() != expected_size ||
      kd_.size() != expected_size) {
    throw std::runtime_error(
        "Invalid vector size in recovery stand config '" + config_path +
        "': default_dof_pos, kp and kd must each contain " +
        std::to_string(expected_size) + " values");
  }
}

void FSMStateRecoveryStand::onExit() {
  LOG(INFO) << "FSMStateRecoveryStand::onExit()";
}

StateID FSMStateRecoveryStand::check_transition() {
  next_state_id_ = state_id_;

  // Switch FSM control mode
  StateID state_id = fsm_data_ptr_->desired_command_ptr->state_id;
  switch (state_id) {
    case StateID::RECOVERY_STAND:
      break;

    case StateID::PASSIVE:
    case StateID::DAMPER:
    case StateID::RL_LOCOMOTION:
    case StateID::RL_TRACKING:
      next_state_id_ = state_id;
      break;

    default:
      LOG(ERROR) << "[CONTROL FSM] Bad Request: Cannot transition from " << StateID::RECOVERY_STAND << " to "
                << state_id;
  }

  return next_state_id_;
}

