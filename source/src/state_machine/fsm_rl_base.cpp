#include "state_machine/fsm_rl_base.hpp"
FSMRLBase::FSMRLBase(std::shared_ptr<FSMData> fsm_data_ptr, StateID id, std::string name)
    : FSMState(fsm_data_ptr, id, name) {

}

void FSMRLBase::onEnter() {
  LOG(INFO) << "FSMRLBase::onEnter()";
  counter_ = 0;
}

void FSMRLBase::run() {
  counter_++;

  for(int i = 0; i < fsm_data_ptr_->num_dofs; ++i) {
      fsm_data_ptr_->robot_command_ptr->motor_command.q[i] = 0.0f;
      fsm_data_ptr_->robot_command_ptr->motor_command.dq[i] = 0.0f;
      fsm_data_ptr_->robot_command_ptr->motor_command.kp[i] = 0.0f;
      fsm_data_ptr_->robot_command_ptr->motor_command.kd[i] = 0.0f;
      fsm_data_ptr_->robot_command_ptr->motor_command.tau[i] = 0.0f;
  }
}

void FSMRLBase::init_rl_base(const std::string config_path)
{
  read_rl_yaml(config_path);
  init_obs();
  init_outputs();
  gravity_vec_ = Eigen::Vector3d::Zero();
  gravity_vec_ << 0.0, 0.0, -1.0;

  default_dof_pos_ = Eigen::VectorXd::Zero(fsm_data_ptr_->num_dofs);
  kp_ = Eigen::VectorXd::Zero(fsm_data_ptr_->num_dofs);
  kd_ = Eigen::VectorXd::Zero(fsm_data_ptr_->num_dofs);
}

void FSMRLBase::init_obs()
{
  obs_.lin_vel = Eigen::VectorXd::Zero(3);
  obs_.ang_vel = Eigen::VectorXd::Zero(3);
  obs_.gravity_vec = Eigen::VectorXd::Zero(3);
  obs_.gravity_vec << 0, 0, -1;
  obs_.commands = Eigen::VectorXd::Zero(3);
  obs_.base_quat = Eigen::Quaternion<double>(1.0, 0.0, 0.0, 0.0);
  obs_.dof_pos = Eigen::VectorXd::Zero(rl_params_.num_actions); //TODO
  obs_.dof_vel = Eigen::VectorXd::Zero(rl_params_.num_actions); //TODO
  obs_.actions = Eigen::VectorXd::Zero(rl_params_.num_actions);
}

void FSMRLBase::read_rl_yaml(const std::string config_path)
{
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_path)[fsm_data_ptr_->robot_name];
    }
    catch (YAML::BadFile &e)
    {
        LOG(ERROR) << LOGGER::ERROR << "The file '" << config_path << "' does not exist";
        return;
    }

    read_vector_from_yaml(config["default_dof_pos"], rl_params_.default_dof_pos);
    read_vector_from_yaml(config["kp"], rl_params_.kp);
    read_vector_from_yaml(config["kd"], rl_params_.kd);

    rl_params_.num_observations = config["num_observations"].as<int>();
    rl_params_.num_actions = config["num_actions"].as<int>();
    rl_params_.policy_name = config["policy_name"].as<std::string>();
    rl_params_.control_decimation = config["control_decimation"].as<int>();
    rl_params_.observations_history = config["observations_history"].as<int>();

    rl_params_.intra_op_num_threads = config["intra_op_num_threads"].as<int>();
    rl_params_.inter_op_num_threads = config["inter_op_num_threads"].as<int>();
    rl_params_.bind_inference_thread_to_core = config["bind_inference_thread_to_core"].as<bool>();
    read_std_vector_int_from_yaml(config["assigned_inference_cores"], rl_params_.assigned_inference_cores);

    rl_params_.dof_pos_scale = config["dof_pos_scale"].as<double>();
    rl_params_.dof_vel_scale = config["dof_vel_scale"].as<double>();
    rl_params_.lin_vel_scale = config["lin_vel_scale"].as<double>();
    rl_params_.ang_vel_scale = config["ang_vel_scale"].as<double>();

    rl_params_.clip_obs = config["clip_obs"].as<double>();
    read_vector_from_yaml(config["clip_actions_upper"], rl_params_.clip_actions_upper);
    read_vector_from_yaml(config["clip_actions_lower"], rl_params_.clip_actions_lower);

    read_vector_from_yaml(config["action_scale"], rl_params_.action_scale);
    read_vector_from_yaml(config["commands_scale"], rl_params_.commands_scale);

    read_vector_from_yaml(config["joint_mapping"], rl_params_.joint_mapping);

    transition_time_ = config["transition_time"].as<double>();

    yaw_to_scale_ = config["yaw_to_scale"].as<bool>(false);
    yaw_to_scale_vel_threshold_ = config["yaw_to_scale_vel_threshold"].as<double>(4.0);

    // print parameters
    print_rl_params(rl_params_);
}

void FSMRLBase::init_onnx_model(const std::string &policy_path) {
  ort_.init_onnx_runtime(policy_path, rl_params_.intra_op_num_threads, rl_params_.inter_op_num_threads);
  input_dims_ = ort_.get_input_dims();
  output_dims_ = ort_.get_output_dims();
  for (auto input_dim : input_dims_) {
      auto size = std::accumulate(input_dim.begin(), input_dim.end(), 1, std::multiplies<int64_t>());
      input_.push_back(new float[size]);
      memset(input_.back(), 0, size * sizeof(float));
  }

  for (auto output_dim: output_dims_) {
      auto size = std::accumulate(output_dim.begin(), output_dim.end(), 1, std::multiplies<int64_t>());
      output_.push_back(new float[size]);
      memset(output_.back(), 0, size * sizeof(float));
  }
}

void FSMRLBase::init_rl_model_inference_thread() {
  rl_model_inference_running_ = false;
  controller_running_ = true;

  double rl_dt = rl_params_.control_decimation * fsm_data_ptr_->control_dt;
  rl_desired_time_step_ = rl_dt;
  rl_desired_frequency_ = 1.0 / rl_dt;

  rl_model_inference_thread_ = std::thread([&]() {
    // Bind to the assigned core(s) as soon as the thread starts
    if (rl_params_.bind_inference_thread_to_core) {
      bind_current_thread_to_cpus(rl_params_.assigned_inference_cores);
    } else {
      LOG(INFO) << "[INFO]: Binding inference thread to core(s) is disabled.";
    }

    while (controller_running_) {
      try {
        execute_and_sleep(
            [&]() {
              if (rl_model_inference_running_) {
                run_model();
              }
            },
            rl_desired_frequency_);
      } catch (const std::exception& e) {
        controller_running_ = false;
        LOG(ERROR) << "[FSMStateRLLocomotion::init_rl_model_inference_thread]: RL model inference thread error : " << e.what();
      }
    }
  });
}

Eigen::VectorXd FSMRLBase::covert_dof_to_action(const Eigen::VectorXd& dof_pos_order) {
  Eigen::VectorXd actions_order =  Eigen::VectorXd::Zero(rl_params_.num_actions);
  Eigen::VectorXd actions = Eigen::VectorXd::Zero(rl_params_.num_actions);
  for (size_t i = 0; i < rl_params_.num_actions; i++) {
    actions_order[i] = dof_pos_order[i] - rl_params_.default_dof_pos[i];
    actions_order[i] = actions_order[i] / rl_params_.action_scale[i];
  }
  for (size_t i = 0; i < rl_params_.num_actions; i++) {
    actions[i] = actions_order[rl_params_.joint_mapping[i]];
  }
  return actions;
}

void FSMRLBase::print_vector(const Eigen::Ref<const Eigen::VectorXd>& vec, const std::string& name) {
    std::stringstream ss;
    ss << name << ": [";
    for (int i = 0; i < vec.size(); ++i) {
        ss << vec[i];
        if (i < vec.size() - 1) {
            ss << ", ";
        }
    }
    ss << "]";
    LOG(INFO) << ss.str();
}

void FSMRLBase::print_vector(const Eigen::Ref<const Eigen::VectorXi>& vec, const std::string& name) {
    std::stringstream ss;
    ss << name << ": [";
    for (int i = 0; i < vec.size(); ++i) {
        ss << vec[i];
        if (i < vec.size() - 1) {
            ss << ", ";
        }
    }
    ss << "]";
    LOG(INFO) << ss.str();
}

void FSMRLBase::print_rl_params(const RLParams& rl_params) {
    print_vector(rl_params.default_dof_pos, "default_dof_pos");
    print_vector(rl_params.kp, "kp");
    print_vector(rl_params.kd, "kd");
    LOG(INFO) << "num_observations: " << rl_params.num_observations;
    LOG(INFO) << "num_actions: " << rl_params.num_actions;
    LOG(INFO) << "policy_name: " << rl_params.policy_name;
    LOG(INFO) << "control_decimation: " << rl_params.control_decimation;
    LOG(INFO) << "observations_history: " << rl_params.observations_history;
    LOG(INFO) << "dof_pos_scale: " << rl_params.dof_pos_scale;
    LOG(INFO) << "dof_vel_scale: " << rl_params.dof_vel_scale;
    LOG(INFO) << "lin_vel_scale: " << rl_params.lin_vel_scale;
    LOG(INFO) << "ang_vel_scale: " << rl_params.ang_vel_scale;
    LOG(INFO) << "clip_obs: " << rl_params.clip_obs;
    print_vector(rl_params.clip_actions_upper, "clip_actions_upper");
    print_vector(rl_params.clip_actions_lower, "clip_actions_lower");
    print_vector(rl_params.action_scale, "action_scale");
    print_vector(rl_params.commands_scale, "commands_scale");
    print_vector(rl_params.joint_mapping, "joint_mapping");

    std::stringstream ss;
    ss << "observations: [";
    for (size_t i = 0; i < rl_params.observations.size(); ++i) {
        ss << rl_params.observations[i];
        if (i < rl_params.observations.size() - 1) {
            ss << ", ";
        }
    }
    ss << "]";
    LOG(INFO) << ss.str();
    LOG(INFO) << "intra_op_num_threads: " << rl_params.intra_op_num_threads;
    LOG(INFO) << "inter_op_num_threads: " << rl_params.inter_op_num_threads;
    LOG(INFO) << "bind_inference_thread_to_core? " << rl_params.bind_inference_thread_to_core;
    std::stringstream ss1;
    ss1 << "assigned_inference_cores: [";
    for (size_t i = 0; i < rl_params.assigned_inference_cores.size(); ++i) {
        ss1 << rl_params.assigned_inference_cores[i];
        if (i < rl_params.assigned_inference_cores.size() - 1) {
            ss1 << ", ";
        }
    }
    ss1 << "]";
    LOG(INFO) << ss1.str();
}

void FSMRLBase::init_outputs() {
  actions_ = Eigen::VectorXd::Zero(rl_params_.num_actions);
  last_actions_ = Eigen::VectorXd::Zero(rl_params_.num_actions);
  des_dof_pos_ = Eigen::VectorXd::Zero(rl_params_.num_actions);
}

void FSMRLBase::onExit() {
  LOG(INFO) << "FSMRLBase::onExit()";
}

StateID FSMRLBase::check_transition() {
  next_state_id_ = state_id_;
  return next_state_id_;
}

bool FSMRLBase::bind_current_thread_to_cpus(const std::vector<int>& core_ids) {
  if (core_ids.empty()) {
    LOG(ERROR) << "[ERROR]: Empty core IDs list";
    return false;
  }

  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);

  // Add all specified cores to the CPU set
  for (int core_id : core_ids) {
    CPU_SET(core_id, &cpuset);
  }

  // Bind the current thread to multiple cores using pthread_setaffinity_np
  pthread_t current_thread = pthread_self();
  if (pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset) != 0) {
    LOG(ERROR) << "[ERROR]: Failed to bind thread to CPUs";
    return false;
  }

  // Build a string of the core ID list for logging
  std::stringstream ss;
  for (size_t i = 0; i < core_ids.size(); ++i) {
    if (i != 0) ss << ", ";
    ss << core_ids[i];
  }

  LOG(INFO) << "[INFO]: Bind thread to CPUs: " << ss.str();
  return true;
}

// Bind the current thread to the specified core
bool FSMRLBase::bind_current_thread_to_cpu(int core_id) {
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(core_id, &cpuset);

  // Get the current thread ID
  pthread_t current_thread = pthread_self();

  // Bind the current thread using pthread_setaffinity_np
  if (pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset) != 0) {
    LOG(ERROR) << "[ERROR]: Failed to bind thread to CPU " << core_id;
    return false;
  }
  LOG(INFO) << "[INFO]: Bind thread to CPU " << core_id;

  return true;
}

// Bind the specified thread to the specified core
bool FSMRLBase::bind_thread_to_cpu(std::thread& thread, int core_id) {
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(core_id, &cpuset);

  int result = pthread_setaffinity_np(thread.native_handle(),
                                    sizeof(cpu_set_t), &cpuset);
  if (result != 0) {
    LOG(ERROR) << "[ERROR]: Failed to bind thread to CPU " << core_id;
    return false;
  }
  LOG(INFO) << "[INFO]: Bind thread to CPU " << core_id;

  return true;
}

void FSMRLBase::init_head_control() {
  int head_joint_index = fsm_data_ptr_->num_dofs;
  for (size_t i = 0; i < fsm_data_ptr_->head_num_dofs; ++i) {
      head_q_start_time_[i] = counter_ * fsm_data_ptr_->control_dt;
      head_q_init_[i] = fsm_data_ptr_->robot_state_ptr->motor_state.q[head_joint_index+i];
  }
  last_head_q_cmd_[0] = fsm_data_ptr_->desired_command_ptr->head_rpy[2];
  last_head_q_cmd_[1] = fsm_data_ptr_->desired_command_ptr->head_rpy[1];
  last_head_q_cmd_[2] = fsm_data_ptr_->desired_command_ptr->head_rpy[0];
}

void FSMRLBase::head_control() {
  int head_joint_index = fsm_data_ptr_->num_dofs;
  auto head_duration = fsm_data_ptr_->desired_command_ptr->head_duration;
  auto head_rpy_des = fsm_data_ptr_->desired_command_ptr->head_rpy;

  Eigen::Matrix<double, 3, 1> head_q_target = Eigen::Matrix<double, 3, 1>::Zero();
  Eigen::Matrix<double, 3, 1> head_q_duration = Eigen::Matrix<double, 3, 1>::Zero();
  head_q_target[0] = head_rpy_des[2]; // yaw joint
  head_q_target[1] = head_rpy_des[1]; // pitch joint
  head_q_target[2] = head_rpy_des[0]; // roll joint
  head_q_duration[0] = head_duration[2]; // yaw joint
  head_q_duration[1] = head_duration[1]; // pitch joint
  head_q_duration[2] = head_duration[0]; // roll joint

  Eigen::Matrix<double, 3, 1> phase = Eigen::Matrix<double, 3, 1>::Zero();

  // if head command is changed
  for (size_t i = 0; i < 3; ++i) {
    if (head_q_target[i] != last_head_q_cmd_[i]) {
      head_q_start_time_[i] = counter_ * fsm_data_ptr_->control_dt;
      head_q_init_[i] = fsm_data_ptr_->robot_state_ptr->motor_state.q[head_joint_index+i];
      // check if duration is too small
      if (head_q_duration[i] <= 0.1) {
        LOG(ERROR) << "[ERROR]: Head " << i <<" duration is too small: " << head_q_duration[i]
                  << "s, duration is reset to 1s";
        head_q_duration[i] = 1.0;
      }

      LOG(ERROR) << "Head desired q angle changed: the " << i << " joint new command is " <<
      head_q_target[i] << " rad, old command is " <<
      last_head_q_cmd_[i] << " rad, the duration is " <<
      head_q_duration[i] << " s";
    }
    double current_time = counter_ * fsm_data_ptr_->control_dt;
    phase[i] = std::max(std::min((current_time - head_q_start_time_[i]) / head_q_duration[i], 1.0), 0.0);
  }

  Eigen::Matrix<double, 3, 1> head_q_des = Eigen::Matrix<double, 3, 1>::Zero();
  Eigen::Matrix<double, 3, 1> head_dq_des = Eigen::Matrix<double, 3, 1>::Zero();
  for (size_t i = 0; i < fsm_data_ptr_->head_num_dofs; ++i) {
    head_q_des[i] = 0.5 * (head_q_init_[i] - head_q_target[i]) * std::cos(M_PI * phase[i]) + 0.5 * (head_q_init_[i] + head_q_target[i]);
    head_dq_des[i] = 0.5 * (head_q_init_[i] - head_q_target[i]) * (-M_PI * std::sin(M_PI * phase[i]) / head_q_duration[i]);
  }

  for(int i = 0; i < fsm_data_ptr_->head_num_dofs; ++i) {
      fsm_data_ptr_->robot_command_ptr->motor_command.q[i+fsm_data_ptr_->num_dofs] = head_q_des[i];
      fsm_data_ptr_->robot_command_ptr->motor_command.dq[i+fsm_data_ptr_->num_dofs] = head_dq_des[i];
      fsm_data_ptr_->robot_command_ptr->motor_command.kp[i+fsm_data_ptr_->num_dofs] = fsm_data_ptr_->head_kp[i];
      fsm_data_ptr_->robot_command_ptr->motor_command.kd[i+fsm_data_ptr_->num_dofs] = fsm_data_ptr_->head_kd[i];
      fsm_data_ptr_->robot_command_ptr->motor_command.tau[i+fsm_data_ptr_->num_dofs] = 0.0f;
  }

  last_head_q_cmd_ = head_q_target;
}

