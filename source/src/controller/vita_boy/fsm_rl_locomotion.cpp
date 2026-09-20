#include <array>
#include <chrono>
#include <ctime>
#include <iomanip>
#include "controller/vita_boy/fsm_rl_locomotion.hpp"
#include "common/helpers.hpp"

FSMStateRLLocomotion::FSMStateRLLocomotion(std::shared_ptr<FSMData> fsm_data_ptr)
    : FSMRLBase(fsm_data_ptr, StateID::RL_LOCOMOTION, "FSMStateRLLocomotion") {
    std::string config_path = "../config/" + fsm_data_ptr_->robot_name + "/" + fsm_data_ptr_->robot_version +
                              "/rl_locomotion/rl_locomotion.yaml";
    init_rl_base(config_path);

    std::string policy_path = "../config/" + fsm_data_ptr_->robot_name + "/" + fsm_data_ptr_->robot_version +
                              "/rl_locomotion/" + rl_params_.policy_name;
    init_onnx_model(policy_path);
    init_rl_model_inference_thread();

    locomotion_command_ = Eigen::Vector3d::Zero();

    obs_vec_.resize(rl_params_.num_observations);
    obs_history_vec_.resize(rl_params_.num_observations * rl_params_.observations_history);
}

void FSMStateRLLocomotion::onEnter() {
    LOG(INFO) << "FSMStateRLLocomotion::onEnter()";
    counter_ = 0;

    // get initail joint state
    q_init_ = Eigen::VectorXd::Zero(fsm_data_ptr_->num_dofs);
    for (size_t i = 0; i < fsm_data_ptr_->num_dofs; ++i) {
        q_init_[i] = fsm_data_ptr_->robot_state_ptr->motor_state.q[i];
    }

    // init des_dof_pos_
    for (size_t i = 0; i < fsm_data_ptr_->num_dofs; ++i) {
        des_dof_pos_[i] = q_init_[i];
    }

    // init q_cmd_init_
    q_cmd_init_ = Eigen::VectorXd::Zero(fsm_data_ptr_->num_dofs);
    for (size_t i = 0; i < fsm_data_ptr_->num_dofs; ++i) {
        q_cmd_init_[i] = fsm_data_ptr_->robot_command_ptr->motor_command.q[i];
    }

    reset_input();

    init_head_control();

    last_actions_ = covert_dof_to_action(q_init_);

    update_observation();

    // start inference
    rl_model_inference_running_ = true;
    rl_counter_ = 0;
    history_initialized_ = false;  // re-fill on first call

    // history init
    for (int i = 0; i < rl_params_.observations_history; ++i) {
        std::copy(obs_vec_.begin(), obs_vec_.end(), obs_history_vec_.begin() + i * rl_params_.num_observations);
    }
}

void FSMStateRLLocomotion::run() {
    counter_++;

    Eigen::VectorXd q_cmd = Eigen::VectorXd::Zero(fsm_data_ptr_->num_dofs);
    q_cmd = forder_cos_smooth(des_dof_pos_, q_cmd_init_, fsm_data_ptr_->control_dt, counter_, transition_time_);

    for (int i = 0; i < fsm_data_ptr_->num_dofs; ++i) {
        fsm_data_ptr_->robot_command_ptr->motor_command.q[i] = q_cmd[i];
        fsm_data_ptr_->robot_command_ptr->motor_command.dq[i] = 0.0f;
        fsm_data_ptr_->robot_command_ptr->motor_command.kp[i] = rl_params_.kp[i];
        fsm_data_ptr_->robot_command_ptr->motor_command.kd[i] = rl_params_.kd[i];
        fsm_data_ptr_->robot_command_ptr->motor_command.tau[i] = 0.0f;
    }
    head_control();
}

void FSMStateRLLocomotion::run_model() {
    rl_timer_.start_timer();
    update_observation();
    update_history();
    update_action();
    rl_timer_.end_timer();
}

void FSMStateRLLocomotion::update_measured() {
    obs_.ang_vel = fsm_data_ptr_->robot_state_ptr->imu.gyroscope;
    obs_.base_quat = fsm_data_ptr_->robot_state_ptr->imu.quaternion;
    obs_.gravity_vec = quat_rotate_inverse(obs_.base_quat, gravity_vec_);
    obs_.commands << locomotion_command_[0], locomotion_command_[1], locomotion_command_[2];

    // normal order
    obs_.dof_pos << fsm_data_ptr_->robot_state_ptr->motor_state.q.head(rl_params_.num_actions);
    obs_.dof_vel << fsm_data_ptr_->robot_state_ptr->motor_state.dq.head(rl_params_.num_actions);

    obs_.actions = last_actions_;
}

void FSMStateRLLocomotion::update_command() {
    locomotion_command_[0] = fsm_data_ptr_->desired_command_ptr->base_lin_vel[0];
    locomotion_command_[1] = fsm_data_ptr_->desired_command_ptr->base_lin_vel[1];
    locomotion_command_[2] = fsm_data_ptr_->desired_command_ptr->base_ang_vel[2];
}

void FSMStateRLLocomotion::update_observation() {
    update_measured();
    update_command();

    // normal order -> gym/lab order
    Eigen::VectorXd dof_pos_disorder = obs_.dof_pos;
    Eigen::VectorXd dof_vel_disorder = obs_.dof_vel;
    Eigen::VectorXd default_dof_pos_disorder = rl_params_.default_dof_pos;
    for (size_t i = 0; i < obs_.dof_pos.size(); i++) {
        dof_pos_disorder[i] = obs_.dof_pos[rl_params_.joint_mapping[i]];
        dof_vel_disorder[i] = obs_.dof_vel[rl_params_.joint_mapping[i]];
        default_dof_pos_disorder[i] = rl_params_.default_dof_pos[rl_params_.joint_mapping[i]];
    }

    Eigen::VectorXd obs = Eigen::VectorXd::Zero(rl_params_.num_observations);
    int obs_ind = 0;

    // angluar velocity
    for (int i = 0; i < 3; i++) {
        obs[obs_ind] = obs_.ang_vel[i] * rl_params_.ang_vel_scale;
        obs_ind += 1;
    }
    // gravity vector
    for (int i = 0; i < 3; i++) {
        obs[obs_ind] = obs_.gravity_vec[i];
        obs_ind += 1;
    }
    // command
    for (int i = 0; i < 3; i++) {
        obs[obs_ind] = obs_.commands[i] * rl_params_.commands_scale[i];
        obs_ind += 1;
    }
    // q
    for (int i = 0; i < rl_params_.num_actions; i++) {
        obs[obs_ind] = (dof_pos_disorder[i] - default_dof_pos_disorder[i]) * rl_params_.dof_pos_scale;
        obs_ind += 1;
    }
    // dq
    for (int i = 0; i < rl_params_.num_actions; i++) {
        obs[obs_ind] = dof_vel_disorder[i] * rl_params_.dof_vel_scale;
        obs_ind += 1;
    }
    // last action
    for (int i = 0; i < rl_params_.num_actions; i++) {
        obs[obs_ind] = last_actions_[i];
        obs_ind += 1;
    }

    if (obs_ind != rl_params_.num_observations) {
        LOG(ERROR) << "[FSMStateRLLocomotion::update_observation]: ERROR! observation dimension not match, required "
                   << rl_params_.num_observations << " but given " << obs_ind;
    }

    // clamp obs
    for (int i = 0; i < rl_params_.num_observations; i++) {
        obs_vec_[i] = clamp(obs[i], -rl_params_.clip_obs, rl_params_.clip_obs);
    }
}

void FSMStateRLLocomotion::update_history() {
    // Isaac Lab flattens history
    const int num_hist = rl_params_.observations_history;
    const int num_actions = rl_params_.num_actions;
    const std::array<int, 6> term_offsets{0, 3, 6, 9, 9 + num_actions, 9 + 2 * num_actions};
    const std::array<int, 6> term_sizes{3, 3, 3, num_actions, num_actions, num_actions};
    int history_offset = 0;

    for (size_t term = 0; term < term_sizes.size(); ++term) {
        const int term_size = term_sizes[term];
        double* history_begin = obs_history_vec_.data() + history_offset;
        const double* observation_begin = obs_vec_.data() + term_offsets[term];

        if (!history_initialized_) {
            for (int frame = 0; frame < num_hist; ++frame) {
                std::copy(observation_begin, observation_begin + term_size, history_begin + frame * term_size);
            }
        } else {
            if (num_hist > 1) {
                std::memmove(history_begin, history_begin + term_size, (num_hist - 1) * term_size * sizeof(double));
            }
            std::copy(observation_begin, observation_begin + term_size, history_begin + (num_hist - 1) * term_size);
        }
        history_offset += num_hist * term_size;
    }
    history_initialized_ = true;
}

void FSMStateRLLocomotion::update_action() {
    // update policy input
    for (size_t i = 0; i < obs_history_vec_.size(); i++) {
        input_[0][i] = obs_history_vec_[i];
    }
    // inference model
    ort_.infer(input_, output_);
    // get onnx output
    float* temp_output_action = output_[0];
    if (output_.size() > 1) {
        float* temp_output_h_out = output_[1];
        float* temp_output_c_out = output_[2];
        // update hidden cell
        std::memcpy(input_[1], temp_output_h_out,
                    sizeof(float) * std::accumulate(input_dims_[1].begin(), input_dims_[1].end(), 1, std::multiplies<int64_t>()));
        std::memcpy(input_[2], temp_output_c_out,
                    sizeof(float) * std::accumulate(input_dims_[2].begin(), input_dims_[2].end(), 1, std::multiplies<int64_t>()));
    }

    for (size_t i = 0; i < rl_params_.num_actions; i++) {
        // TODO
        actions_[i] = clamp(temp_output_action[i], rl_params_.clip_actions_lower[i], rl_params_.clip_actions_upper[i]);
    }

    // gym/lab order -> normal order
    auto actions_order = actions_;
    for (size_t i = 0; i < rl_params_.num_actions; i++) {
        actions_order[rl_params_.joint_mapping[i]] = actions_[i];
    }

    for (int i = 0; i < rl_params_.num_actions; i++) {
        des_dof_pos_[i] = rl_params_.default_dof_pos[i] + rl_params_.action_scale[i] * actions_order[i];
    }

    last_actions_ = actions_;
}

void FSMStateRLLocomotion::onExit() {
    rl_model_inference_running_ = false;
    reset_input();
    LOG(INFO) << "FSMStateRLLocomotion::onExit()";
}

void FSMStateRLLocomotion::reset_input() {
    locomotion_command_ = Eigen::Vector3d::Zero();
}

StateID FSMStateRLLocomotion::check_transition() {
    next_state_id_ = state_id_;

    // Switch FSM control mode
    StateID state_id = fsm_data_ptr_->desired_command_ptr->state_id;
    switch (state_id) {
        case StateID::RL_LOCOMOTION:
            break;

        case StateID::PASSIVE:
        case StateID::DAMPER:
        case StateID::RECOVERY_STAND:
        case StateID::RL_TRACKING:
            next_state_id_ = state_id;
            break;

        default:
            LOG(ERROR) << "[CONTROL FSM] Bad Request: Cannot transition from " << StateID::RL_LOCOMOTION << " to "
                       << state_id;
    }

    return next_state_id_;
}

FSMStateRLLocomotion::~FSMStateRLLocomotion() {
    controller_running_ = false;
    if (rl_model_inference_thread_.joinable()) {
        rl_model_inference_thread_.join();
    }
    std::stringstream ss;
    ss << "\n[FSMStateRLLocomotion] RL Locomotion thread cleaned";
    ss << "\n########################################################################";
    ss << "\n### RL Locomotion Benchmarking";
    ss << "\n###   Maximum : " << rl_timer_.get_max_interval_in_milliseconds() << "[ms].";
    ss << "\n###   Average : " << rl_timer_.get_average_in_milliseconds() << "[ms].";
    ss << "\n########################################################################\n";
    LOG(INFO) << ss.str();
}
