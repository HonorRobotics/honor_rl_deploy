#include "controller/vita_boy/fsm_rl_tracking.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

#include "cnpy.h"
#include "common/helpers.hpp"

namespace {
using RowMajorMatrixXf = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

std::string shape_to_string(const std::vector<size_t>& shape) {
    std::ostringstream stream;
    stream << '[';
    for (size_t i = 0; i < shape.size(); ++i) {
        if (i != 0) {
            stream << ", ";
        }
        stream << shape[i];
    }
    stream << ']';
    return stream.str();
}

void require_shape(const cnpy::NpyArray& array, const std::string& name, const std::vector<size_t>& expected) {
    if (array.shape != expected) {
        throw std::runtime_error("NPZ array '" + name + "' has shape " + shape_to_string(array.shape) + ", expected " +
                                 shape_to_string(expected));
    }
}

RowMajorMatrixXf map_float_matrix(const cnpy::NpyArray& array, const std::string& name, size_t rows, size_t columns) {
    require_shape(array, name, {rows, columns});
    if (array.word_size != sizeof(float)) {
        throw std::runtime_error("NPZ array '" + name + "' must use float32");
    }
    if (array.fortran_order) {
        throw std::runtime_error("NPZ array '" + name + "' must be C-contiguous");
    }

    const Eigen::Map<const RowMajorMatrixXf> mapped(array.data<float>(), static_cast<Eigen::Index>(rows),
                                                    static_cast<Eigen::Index>(columns));
    return mapped;
}

size_t tensor_size(const std::vector<int64_t>& dimensions) {
    size_t size = 1;
    for (const int64_t dimension : dimensions) {
        if (dimension <= 0) {
            throw std::runtime_error("ONNX tensor has a non-positive dimension");
        }
        size *= static_cast<size_t>(dimension);
    }
    return size;
}

void require_vector_size(const Eigen::VectorXd& vector, int expected, const std::string& name) {
    if (vector.size() != expected) {
        throw std::runtime_error("Config field '" + name + "' has " + std::to_string(vector.size()) +
                                 " values, expected " + std::to_string(expected));
    }
}
}  // namespace

FSMStateRLTracking::FSMStateRLTracking(std::shared_ptr<FSMData> fsm_data_ptr, StateID state_id, std::string state_name)
    : FSMRLBase(std::move(fsm_data_ptr), state_id, std::move(state_name)) {
    config_directory_ = "../config/" + fsm_data_ptr_->robot_name + "/" + fsm_data_ptr_->robot_version + "/rl_tracking";
    initialize(config_directory_ + "/rl_tracking.yaml");
    init_rl_model_inference_thread();
}

void FSMStateRLTracking::initialize(const std::string& config_path) {
    read_config(config_path);
    validate_config();
    load_motions();
    initialize_policies();

    // start from the default motion
    active_motion_id_ = default_motion_id_;
    const auto motion_it = motions_by_id_.find(active_motion_id_);
    active_policy_index_ = policy_index_by_id_.at(motion_it->second.config.model_id);

    initialize_observations();
    FSMRLBase::init_outputs();

    LOG(INFO) << '[' << state_name_ << "] initialized " << motions_by_id_.size() << " NPZ motions and "
              << policies_.size() << " policies";
}

void FSMStateRLTracking::read_config(const std::string& config_path) {
    const YAML::Node root = YAML::LoadFile(config_path);
    const YAML::Node config = root[fsm_data_ptr_->robot_name];
    if (!config) {
        throw std::runtime_error("Missing robot config section '" + fsm_data_ptr_->robot_name + "' in " + config_path);
    }

    read_vector_from_yaml(config["default_dof_pos"], rl_params_.default_dof_pos);
    read_vector_from_yaml(config["kp"], rl_params_.kp);
    read_vector_from_yaml(config["kd"], rl_params_.kd);
    read_vector_from_yaml(config["action_scale"], rl_params_.action_scale);
    read_vector_from_yaml(config["joint_mapping"], rl_params_.joint_mapping);

    rl_params_.num_actions = config["num_actions"].as<int>();
    rl_params_.control_decimation = config["control_decimation"].as<int>();
    rl_params_.dof_pos_scale = config["dof_pos_scale"].as<double>();
    rl_params_.dof_vel_scale = config["dof_vel_scale"].as<double>();
    rl_params_.ang_vel_scale = config["ang_vel_scale"].as<double>();
    rl_params_.clip_obs = config["clip_obs"].as<double>();
    rl_params_.clip_action = config["clip_action"].as<double>();
    // onnx runtime options
    rl_params_.intra_op_num_threads = config["intra_op_num_threads"].as<int>();
    rl_params_.inter_op_num_threads = config["inter_op_num_threads"].as<int>();
    rl_params_.bind_inference_thread_to_core = config["bind_inference_thread_to_core"].as<bool>();
    read_std_vector_int_from_yaml(config["assigned_inference_cores"], rl_params_.assigned_inference_cores);

    transition_time_ = config["transition_time"].as<double>();
    motion_prepare_time_ = config["motion_prepare_time"].as<double>();
    is_single_thread_ = config["is_single_thread"].as<bool>();
    default_motion_id_ = config["default_motion_id"].as<size_t>();

    // models: one onnx per entry
    const YAML::Node models = root["models"];
    if (!models || !models.IsSequence() || models.size() == 0) {
        throw std::runtime_error("Config must contain a non-empty 'models' list");
    }
    model_configs_.clear();
    model_configs_.reserve(models.size());
    for (const YAML::Node& node : models) {
        ModelConfig model;
        model.model_id = node["model_id"].as<size_t>();
        model.model_file = node["model_file"].as<std::string>();
        model.prop_hist = node["prop_hist"].as<int>(1);
        model.demo_hist = node["demo_hist"].as<int>(1);
        model_configs_.push_back(std::move(model));
    }

    // motions: one npz per entry
    const YAML::Node motions = root["motions"];
    if (!motions || !motions.IsSequence() || motions.size() == 0) {
        throw std::runtime_error("Config must contain a non-empty 'motions' list");
    }
    motion_configs_.clear();
    motion_configs_.reserve(motions.size());
    for (const YAML::Node& node : motions) {
        MotionConfig motion;
        motion.motion_id = node["motion_id"].as<size_t>();
        motion.model_id = node["model_id"].as<size_t>();
        motion.motion_file = node["motion_file"].as<std::string>();
        motion.start_frame = node["start_frame"].as<size_t>(0);
        motion.end_frame = node["end_frame"].as<size_t>(0);
        motion_configs_.push_back(std::move(motion));
    }
}

void FSMStateRLTracking::validate_config() const {
    if (rl_params_.num_actions != kNumActions || fsm_data_ptr_->num_dofs != kNumActions) {
        throw std::runtime_error("RL tracking requires exactly 29 body joints");
    }
    if (rl_params_.control_decimation <= 0 || fsm_data_ptr_->control_dt <= 0.0) {
        throw std::runtime_error("Control timestep and decimation must be positive");
    }
    if (motion_prepare_time_ < 0.0 || transition_time_ <= 0.0) {
        throw std::runtime_error(
            "Motion preparation time cannot be negative and transition time "
            "must be positive");
    }
    if (rl_params_.clip_obs <= 0.0 || rl_params_.clip_action <= 0.0) {
        throw std::runtime_error("Observation and action clipping limits must be positive");
    }
    if (rl_params_.intra_op_num_threads <= 0 || rl_params_.inter_op_num_threads <= 0) {
        throw std::runtime_error("ONNX thread counts must be positive");
    }
    if (rl_params_.bind_inference_thread_to_core && rl_params_.assigned_inference_cores.empty()) {
        throw std::runtime_error("At least one inference core is required when CPU binding is enabled");
    }
    if (std::any_of(rl_params_.assigned_inference_cores.begin(), rl_params_.assigned_inference_cores.end(),
                    [](int core) { return core < 0; })) {
        throw std::runtime_error("Inference core IDs cannot be negative");
    }

    require_vector_size(rl_params_.default_dof_pos, kNumActions, "default_dof_pos");
    require_vector_size(rl_params_.kp, kNumActions, "kp");
    require_vector_size(rl_params_.kd, kNumActions, "kd");
    require_vector_size(rl_params_.action_scale, kNumActions, "action_scale");
    require_vector_size(rl_params_.joint_mapping, kNumActions, "joint_mapping");

    // must be a permutation of [0, 28]
    std::vector<bool> mapped_joint(kNumActions, false);
    for (int i = 0; i < kNumActions; ++i) {
        if (std::abs(rl_params_.action_scale[i]) < 1e-12) {
            throw std::runtime_error("action_scale cannot contain zero");
        }
        const double mapping_value = rl_params_.joint_mapping[i];
        const int joint_index = static_cast<int>(std::lround(mapping_value));
        if (std::abs(mapping_value - joint_index) > 1e-9 || joint_index < 0 || joint_index >= kNumActions ||
            mapped_joint[joint_index]) {
            throw std::runtime_error("joint_mapping must be a permutation of [0, 28]");
        }
        mapped_joint[joint_index] = true;
    }

    std::unordered_set<size_t> model_ids;
    for (const ModelConfig& model : model_configs_) {
        if (model.model_file.size() < 6 || model.model_file.substr(model.model_file.size() - 5) != ".onnx") {
            throw std::runtime_error("Model ID " + std::to_string(model.model_id) + " must reference an ONNX file");
        }
        if (model.prop_hist <= 0 || model.demo_hist <= 0) {
            throw std::runtime_error("Model history lengths must be positive");
        }
        if (!model_ids.insert(model.model_id).second) {
            throw std::runtime_error("Duplicate model_id " + std::to_string(model.model_id));
        }
    }

    std::unordered_set<size_t> motion_ids;
    bool has_default_motion = false;
    for (const MotionConfig& motion : motion_configs_) {
        if (!motion_ids.insert(motion.motion_id).second) {
            throw std::runtime_error("Duplicate motion_id " + std::to_string(motion.motion_id));
        }
        if (model_ids.count(motion.model_id) == 0) {
            throw std::runtime_error("Motion ID " + std::to_string(motion.motion_id) + " ('" + motion.motion_file +
                                     "') references an unknown model_id");
        }
        if (motion.motion_file.size() < 5 || motion.motion_file.substr(motion.motion_file.size() - 4) != ".npz") {
            throw std::runtime_error("Motion ID " + std::to_string(motion.motion_id) + " must reference an NPZ file");
        }
        has_default_motion = has_default_motion || motion.motion_id == default_motion_id_;
    }
    if (!has_default_motion) {
        throw std::runtime_error("default_motion_id is not present in motions");
    }
}

FSMStateRLTracking::MotionData FSMStateRLTracking::load_motion(const MotionConfig& config) const {
    const std::string path = config_directory_ + "/" + config.motion_file;
    try {
        // fps: legacy motions use int64, Isaac Lab exports float32
        const cnpy::NpyArray fps_array = cnpy::npz_load(path, "fps");
        require_shape(fps_array, "fps", {1});
        if (fps_array.fortran_order) {
            throw std::runtime_error("NPZ array 'fps' must be C-contiguous");
        }

        double fps_value = 0.0;
        if (fps_array.word_size == sizeof(float)) {
            fps_value = fps_array.data<float>()[0];
        } else if (fps_array.word_size == sizeof(int64_t)) {
            fps_value = static_cast<double>(fps_array.data<int64_t>()[0]);
        } else {
            throw std::runtime_error("NPZ array 'fps' must use float32 or int64");
        }
        const double rounded_fps = std::round(fps_value);
        if (!std::isfinite(fps_value) || fps_value <= 0.0 || fps_value > std::numeric_limits<int>::max() ||
            std::abs(fps_value - rounded_fps) > 1e-6) {
            throw std::runtime_error("NPZ fps must be a positive integer value");
        }

        // joint pos / vel, [T, 29]
        const cnpy::NpyArray joint_pos_array = cnpy::npz_load(path, "joint_pos");
        if (joint_pos_array.shape.size() != 2 || joint_pos_array.shape[1] != kNumActions) {
            throw std::runtime_error("NPZ joint_pos must have shape [T, 29]");
        }
        const size_t total_frames = joint_pos_array.shape[0];
        const RowMajorMatrixXf joint_pos = map_float_matrix(joint_pos_array, "joint_pos", total_frames, kNumActions);

        const cnpy::NpyArray joint_vel_array = cnpy::npz_load(path, "joint_vel");
        const RowMajorMatrixXf joint_vel = map_float_matrix(joint_vel_array, "joint_vel", total_frames, kNumActions);

        // body quat, [T, B, 4] wxyz
        const cnpy::NpyArray body_quat_array = cnpy::npz_load(path, "body_quat_w");
        if (body_quat_array.shape.size() != 3 || body_quat_array.shape[0] != total_frames ||
            body_quat_array.shape[1] <= kAnchorBodyIndex || body_quat_array.shape[2] != 4 ||
            body_quat_array.word_size != sizeof(float) || body_quat_array.fortran_order) {
            throw std::runtime_error(
                "NPZ body_quat_w must be a C-order float32 array [T, B, 4] "
                "with body index 9");
        }
        const size_t body_count = body_quat_array.shape[1];
        const Eigen::Map<const RowMajorMatrixXf> body_quat(body_quat_array.data<float>(),
                                                           static_cast<Eigen::Index>(total_frames),
                                                           static_cast<Eigen::Index>(body_count * 4));

        // trim to [start_frame, end_frame)
        const size_t end_frame = config.end_frame == 0 ? total_frames : config.end_frame;
        if (config.start_frame >= end_frame || end_frame > total_frames) {
            throw std::runtime_error("Invalid frame range [" + std::to_string(config.start_frame) + ", " +
                                     std::to_string(end_frame) + ") for " + path);
        }
        const Eigen::Index start = static_cast<Eigen::Index>(config.start_frame);
        const Eigen::Index frame_count = static_cast<Eigen::Index>(end_frame - config.start_frame);

        MotionData motion;
        motion.config = config;
        motion.fps = static_cast<int>(rounded_fps);
        motion.joint_pos = joint_pos.middleRows(start, frame_count);
        motion.joint_vel = joint_vel.middleRows(start, frame_count);
        motion.anchor_quat_w = body_quat.block(start, kAnchorBodyIndex * 4, frame_count, 4);

        if (!motion.joint_pos.allFinite() || !motion.joint_vel.allFinite() || !motion.anchor_quat_w.allFinite()) {
            throw std::runtime_error("NPZ motion contains NaN or infinity: " + path);
        }
        // normalize anchor quat
        for (Eigen::Index frame = 0; frame < motion.anchor_quat_w.rows(); ++frame) {
            const float norm = motion.anchor_quat_w.row(frame).norm();
            if (norm < 1e-6F) {
                throw std::runtime_error("NPZ contains an invalid anchor quaternion: " + path);
            }
            motion.anchor_quat_w.row(frame) /= norm;
        }

        // fps must match the policy rate
        const double expected_fps = 1.0 / (fsm_data_ptr_->control_dt * rl_params_.control_decimation);
        if (std::abs(expected_fps - motion.fps) > 1e-6) {
            throw std::runtime_error("Motion fps " + std::to_string(motion.fps) + " does not match policy frequency " +
                                     std::to_string(expected_fps));
        }

        LOG(INFO) << '[' << state_name_ << "] loaded motion id=" << config.motion_id << ", file='" << config.motion_file
                  << "' (frames=" << motion.frame_count() << ", fps=" << motion.fps << ") from " << path;
        return motion;
    } catch (const std::exception& error) {
        throw std::runtime_error("Failed to load motion ID " + std::to_string(config.motion_id) + " ('" +
                                 config.motion_file + "') from " + path + ": " + error.what());
    }
}

void FSMStateRLTracking::load_motions() {
    motions_by_id_.clear();
    for (const MotionConfig& config : motion_configs_) {
        motions_by_id_.emplace(config.motion_id, load_motion(config));
    }
}

void FSMStateRLTracking::initialize_policies() {
    policies_.clear();
    policy_index_by_id_.clear();
    policies_.reserve(model_configs_.size());

    for (const ModelConfig& config : model_configs_) {
        PolicyRuntime policy;
        policy.config = config;
        policy.path = config_directory_ + "/" + config.model_file;
        policy.session = std::make_unique<XbotOnnxRuntime>();
        if (!policy.session->init_onnx_runtime(policy.path, rl_params_.intra_op_num_threads,
                                               rl_params_.inter_op_num_threads)) {
            throw std::runtime_error("Failed to initialize ONNX model " + policy.path);
        }

        policy.input_dims = policy.session->get_input_dims();
        policy.output_dims = policy.session->get_output_dims();
        const std::vector<std::string> input_names = policy.session->get_input_names();
        const std::vector<std::string> output_names = policy.session->get_output_names();
        // onnx interface: obs -> actions
        if (policy.input_dims.size() != 1 || policy.output_dims.size() != 1) {
            throw std::runtime_error("Tracking policies must expose one input and one output: " + policy.path);
        }
        if (input_names != std::vector<std::string>{"obs"} || output_names != std::vector<std::string>{"actions"}) {
            throw std::runtime_error("Tracking policy interface must be [obs] -> [actions]: " + policy.path);
        }
        const std::vector<int64_t> expected_input_dims{1, static_cast<int64_t>(config.observation_buffer_size())};
        const std::vector<int64_t> expected_output_dims{1, kNumActions};
        if (policy.input_dims[0] != expected_input_dims || policy.output_dims[0] != expected_output_dims) {
            throw std::runtime_error("Unexpected ONNX input/output dimensions: " + policy.path);
        }

        // allocate io buffers
        policy.input_buffers.reserve(policy.input_dims.size());
        for (const auto& dimensions : policy.input_dims) {
            policy.input_buffers.emplace_back(tensor_size(dimensions), 0.0F);
        }
        policy.output_buffers.reserve(policy.output_dims.size());
        for (const auto& dimensions : policy.output_dims) {
            policy.output_buffers.emplace_back(tensor_size(dimensions), 0.0F);
        }
        policy.inputs.reserve(policy.input_buffers.size());
        for (auto& buffer : policy.input_buffers) {
            policy.inputs.push_back(buffer.data());
        }
        policy.outputs.reserve(policy.output_buffers.size());
        for (auto& buffer : policy.output_buffers) {
            policy.outputs.push_back(buffer.data());
        }

        policy_index_by_id_.emplace(config.model_id, policies_.size());
        policies_.push_back(std::move(policy));
        LOG(INFO) << '[' << state_name_ << "] loaded policy id=" << config.model_id << " from "
                  << policies_.back().path;
    }
}

void FSMStateRLTracking::initialize_observations() {
    FSMRLBase::init_obs();
    initialize_history();
}

void FSMStateRLTracking::initialize_history() {
    // sizes depend on the active policy
    const ModelConfig& config = active_policy().config;
    obs_prop_vec_.assign(kNumProprioceptiveObservations, 0.0);
    obs_demo_vec_.assign(kNumDemoObservations, 0.0);
    obs_buffer_vec_.assign(config.observation_buffer_size(), 0.0);
    obs_hist_prop_vec2d_.assign(config.prop_hist, std::vector<double>(kNumProprioceptiveObservations, 0.0));
    obs_hist_demo_vec2d_.assign(config.demo_hist, std::vector<double>(kNumDemoObservations, 0.0));
    first_update_hist_ = true;
}

void FSMStateRLTracking::onEnter() {
    LOG(INFO) << state_name_ << "::onEnter()";
    // wait for the in-flight inference
    rl_model_inference_running_ = false;
    std::lock_guard<std::mutex> inference_lock(inference_mutex_);

    update_active_motion();
    initialize_history();

    counter_ = 0;
    rl_counter_ = 0;
    rl_control_count_sim_single_thread_ = 0;
    last_actions_.setZero();
    actions_.setZero();

    // smoothing start
    q_cmd_init_ = fsm_data_ptr_->robot_command_ptr->motor_command.q.head(kNumActions);
    {
        std::lock_guard<std::mutex> command_lock(command_mutex_);
        des_dof_pos_ = q_cmd_init_;
    }
    last_fsm_kp_ = fsm_data_ptr_->robot_command_ptr->motor_command.kp;
    last_fsm_kd_ = fsm_data_ptr_->robot_command_ptr->motor_command.kd;

    // rewind the reference motion
    {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        reset_motion_progress_locked();
    }

    // prefill history
    update_observation();
    update_history();

    // start inference
    if (!(fsm_data_ptr_->is_sim && is_single_thread_)) {
        rl_model_inference_running_ = true;
    }
}

void FSMStateRLTracking::run() {
    ++counter_;

    // sim single thread: inline inference
    if (fsm_data_ptr_->is_sim && is_single_thread_) {
        ++rl_control_count_sim_single_thread_;
        if (rl_control_count_sim_single_thread_ >= static_cast<size_t>(rl_params_.control_decimation)) {
            run_model();
            rl_control_count_sim_single_thread_ = 0;
        }
    }

    // snapshot the target
    Eigen::VectorXd desired_dof_pos;
    {
        std::lock_guard<std::mutex> command_lock(command_mutex_);
        desired_dof_pos = des_dof_pos_;
    }
    // smooth to the policy target
    const Eigen::VectorXd q_cmd =
        forder_cos_smooth(desired_dof_pos, q_cmd_init_, fsm_data_ptr_->control_dt, counter_, transition_time_);
    const Eigen::VectorXd kp =
        forder_cos_smooth(rl_params_.kp, last_fsm_kp_, fsm_data_ptr_->control_dt, counter_, transition_time_);
    const Eigen::VectorXd kd =
        forder_cos_smooth(rl_params_.kd, last_fsm_kd_, fsm_data_ptr_->control_dt, counter_, transition_time_);

    {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        auto& interface = *fsm_data_ptr_->interface_parameter_ptr_;
        // acknowledged, replay from start
        if (interface.result == 100 && send_result_) {
            reset_motion_progress_locked();
            counter_ = 0;
        }

        // report progress
        const MotionData& motion = active_motion_locked();
        const double duration = static_cast<double>(motion.frame_count()) / motion.fps;
        interface.schedule = get_percentage(motion_cmd_time_, duration);
        if (interface.schedule >= 100 && !send_result_) {
            send_result_ = true;
            interface.result = 1;
        }
    }

    std::lock_guard<std::mutex> lock(obs_mutex_);
    auto& command = fsm_data_ptr_->robot_command_ptr->motor_command;
    for (int i = 0; i < kNumActions; ++i) {
        command.q[i] = q_cmd[i];
        command.dq[i] = 0.0F;
        command.kp[i] = kp[i];
        command.kd[i] = kd[i];
        command.tau[i] = 0.0F;
    }
    // head not tracked, hold zero
    for (int i = 0; i < fsm_data_ptr_->head_num_dofs; ++i) {
        const int index = kNumActions + i;
        command.q[index] = 0.0F;
        command.dq[index] = 0.0F;
        command.kp[index] = fsm_data_ptr_->head_kp[i];
        command.kd[index] = fsm_data_ptr_->head_kd[i];
        command.tau[index] = 0.0F;
    }
}

void FSMStateRLTracking::run_model() {
    std::lock_guard<std::mutex> inference_lock(inference_mutex_);
    if (!(fsm_data_ptr_->is_sim && is_single_thread_) && !rl_model_inference_running_) {
        return;
    }

    // apply pending motion switch
    update_active_motion();

    rl_timer_.start_timer();
    ++rl_counter_;
    update_observation();
    update_history();
    update_action();
    rl_timer_.end_timer();
}

void FSMStateRLTracking::update_active_motion() {
    size_t requested_motion_id = fsm_data_ptr_->desired_command_ptr->motion_id;
    // unknown id -> default, warn once
    if (motions_by_id_.count(requested_motion_id) == 0) {
        if (last_rejected_motion_id_ != requested_motion_id) {
            LOG(WARNING) << '[' << state_name_ << "] unsupported motion_id " << requested_motion_id
                         << ", using default motion_id " << default_motion_id_;
            last_rejected_motion_id_ = requested_motion_id;
        }
        requested_motion_id = default_motion_id_;
    } else {
        last_rejected_motion_id_ = kInvalidId;
    }
    activate_motion(requested_motion_id);
}

void FSMStateRLTracking::activate_motion(size_t motion_id) {
    const auto motion_it = motions_by_id_.find(motion_id);
    if (motion_it == motions_by_id_.end()) {
        throw std::runtime_error("Cannot activate unknown motion_id " + std::to_string(motion_id));
    }
    const auto policy_it = policy_index_by_id_.find(motion_it->second.config.model_id);
    if (policy_it == policy_index_by_id_.end()) {
        throw std::runtime_error("Motion references an unloaded policy");
    }

    {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        if (active_motion_id_ == motion_id) {
            return;
        }
        active_motion_id_ = motion_id;
        active_policy_index_ = policy_it->second;
        reset_motion_progress_locked();
    }
    initialize_history();

    LOG(INFO) << '[' << state_name_ << "] activated motion id=" << motion_id << ", file='"
              << motion_it->second.config.motion_file << "', model_id=" << motion_it->second.config.model_id;
}

void FSMStateRLTracking::reset_motion_progress_locked() {
    motion_cmd_time_ = -motion_prepare_time_;
    reset_track_motion_cmd_time_ = true;
    init_demo_yaw_flag_ = false;
    quat_yaw_from_demo_to_curr_ = Eigen::Quaterniond::Identity();
    send_result_ = false;
    fsm_data_ptr_->interface_parameter_ptr_->schedule = 0;
    fsm_data_ptr_->interface_parameter_ptr_->result = 100;
}

void FSMStateRLTracking::update_measured() {
    std::lock_guard<std::mutex> lock(obs_mutex_);
    obs_.ang_vel = fsm_data_ptr_->robot_state_ptr->imu.gyroscope;
    obs_.base_quat = fsm_data_ptr_->robot_state_ptr->imu.quaternion;
    obs_.dof_pos = fsm_data_ptr_->robot_state_ptr->motor_state.q.head(kNumActions);
    obs_.dof_vel = fsm_data_ptr_->robot_state_ptr->motor_state.dq.head(kNumActions);
}

void FSMStateRLTracking::update_motion_reference() {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    // advance playback
    if (reset_track_motion_cmd_time_) {
        motion_cmd_time_ = -motion_prepare_time_;
        reset_track_motion_cmd_time_ = false;
    } else {
        motion_cmd_time_ += rl_desired_time_step_;
    }

    const MotionData& motion = active_motion_locked();
    const int last_frame = static_cast<int>(motion.frame_count()) - 1;
    const int frame = std::clamp(static_cast<int>(std::lround(motion_cmd_time_ * motion.fps)), 0, last_frame);

    // demo obs: [joint_pos(29), joint_vel(29)]
    const Eigen::Index frame_index = static_cast<Eigen::Index>(frame);
    for (int i = 0; i < kNumActions; ++i) {
        obs_demo_vec_[i] = motion.joint_pos(frame_index, i);
        obs_demo_vec_[kNumActions + i] = motion.joint_vel(frame_index, i);
    }

    const auto quat = motion.anchor_quat_w.row(frame_index);
    ref_anchor_quat_global_ = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]);

    // align demo yaw, once per motion
    if (!init_demo_yaw_flag_) {
        const double current_yaw = quat_to_euler_zyx(obs_.base_quat)[0];
        const double demo_yaw = quat_to_euler_zyx(ref_anchor_quat_global_)[0];
        const double yaw_offset = shortest_angular_distance(demo_yaw, current_yaw);
        quat_yaw_from_demo_to_curr_ = Eigen::Quaterniond(Eigen::AngleAxisd(yaw_offset, Eigen::Vector3d::UnitZ()));
        quat_yaw_from_demo_to_curr_.normalize();
        init_demo_yaw_flag_ = true;
    }
}

void FSMStateRLTracking::update_observation() {
    update_measured();

    // robot order -> policy order
    Eigen::VectorXd dof_pos_policy(kNumActions);
    Eigen::VectorXd dof_vel_policy(kNumActions);
    Eigen::VectorXd default_dof_pos_policy(kNumActions);
    for (int i = 0; i < kNumActions; ++i) {
        const int mapped_index = static_cast<int>(rl_params_.joint_mapping[i]);
        dof_pos_policy[i] = obs_.dof_pos[mapped_index];
        dof_vel_policy[i] = obs_.dof_vel[mapped_index];
        default_dof_pos_policy[i] = rl_params_.default_dof_pos[mapped_index];
    }

    update_motion_reference();

    // measured torso -> reference torso
    const Eigen::Quaterniond desired_anchor_quat =
        quaternion_multiplication_optimized(quat_yaw_from_demo_to_curr_, ref_anchor_quat_global_);
    const Eigen::Quaterniond measured_anchor_quat = compute_torso_quat(obs_.base_quat, obs_.dof_pos.segment<3>(12));
    const Eigen::Quaterniond relative_quat =
        quaternion_multiplication_optimized(quat_inverse(measured_anchor_quat), desired_anchor_quat);
    const Eigen::Matrix3d relative_rotation = quaternion_to_matrix(relative_quat);

    Eigen::VectorXd observation(kNumProprioceptiveObservations);
    int index = 0;
    // anchor rotation 6d
    observation.segment<6>(index) <<
        relative_rotation(0, 0), relative_rotation(0, 1),
        relative_rotation(1, 0), relative_rotation(1, 1),
        relative_rotation(2, 0), relative_rotation(2, 1);
    index += 6;
    // angular velocity
    observation.segment<3>(index) = obs_.ang_vel * rl_params_.ang_vel_scale;
    index += 3;
    // q
    observation.segment(index, kNumActions) = (dof_pos_policy - default_dof_pos_policy) * rl_params_.dof_pos_scale;
    index += kNumActions;
    // dq
    observation.segment(index, kNumActions) = dof_vel_policy * rl_params_.dof_vel_scale;
    index += kNumActions;
    // last action
    observation.segment(index, kNumActions) = last_actions_;
    index += kNumActions;

    if (index != kNumProprioceptiveObservations) {
        throw std::logic_error("Proprioceptive observation size mismatch");
    }
    // clamp obs
    for (int i = 0; i < observation.size(); ++i) {
        obs_prop_vec_[i] = clamp(observation[i], -rl_params_.clip_obs, rl_params_.clip_obs);
    }
}

void FSMStateRLTracking::update_history() {
    update_observation_history();
    obs_buffer_vec_ = build_observation_buffer();
}

void FSMStateRLTracking::update_observation_history() {
    // after reset: fill every slot
    if (first_update_hist_) {
        obs_hist_prop_vec2d_.assign(active_policy().config.prop_hist, obs_prop_vec_);
        obs_hist_demo_vec2d_.assign(active_policy().config.demo_hist, obs_demo_vec_);
        first_update_hist_ = false;
        return;
    }

    // drop oldest, append newest
    obs_hist_prop_vec2d_.erase(obs_hist_prop_vec2d_.begin());
    obs_hist_prop_vec2d_.push_back(obs_prop_vec_);
    obs_hist_demo_vec2d_.erase(obs_hist_demo_vec2d_.begin());
    obs_hist_demo_vec2d_.push_back(obs_demo_vec_);
}

std::vector<double> FSMStateRLTracking::build_observation_buffer() const {
    // demo history first, then prop
    std::vector<double> buffer;
    buffer.reserve(active_policy().config.observation_buffer_size());
    for (const auto& observation : obs_hist_demo_vec2d_) {
        buffer.insert(buffer.end(), observation.begin(), observation.end());
    }
    for (const auto& observation : obs_hist_prop_vec2d_) {
        buffer.insert(buffer.end(), observation.begin(), observation.end());
    }
    return buffer;
}

void FSMStateRLTracking::update_action() {
    PolicyRuntime& policy = active_policy();
    if (policy.input_buffers.size() != 1 || policy.input_buffers[0].size() != obs_buffer_vec_.size()) {
        throw std::logic_error("Policy input buffer size mismatch");
    }

    // update policy input
    for (size_t i = 0; i < obs_buffer_vec_.size(); ++i) {
        policy.input_buffers[0][i] = static_cast<float>(obs_buffer_vec_[i]);
    }

    // inference model
    rl_action_inference_timer_.start_timer();
    const bool inference_ok = policy.session->infer(policy.inputs, policy.outputs);
    rl_action_inference_timer_.end_timer();
    if (!inference_ok) {
        throw std::runtime_error("ONNX inference failed for " + policy.path);
    }

    // get onnx output
    for (int i = 0; i < kNumActions; ++i) {
        actions_[i] = clamp(policy.output_buffers[0][i], -rl_params_.clip_action, rl_params_.clip_action);
    }

    // policy order -> robot order
    Eigen::VectorXd actions_robot_order(kNumActions);
    for (int i = 0; i < kNumActions; ++i) {
        const int mapped_index = static_cast<int>(rl_params_.joint_mapping[i]);
        actions_robot_order[mapped_index] = actions_[i];
    }
    // scale around the default pose
    Eigen::VectorXd desired_dof_pos(kNumActions);
    for (int i = 0; i < kNumActions; ++i) {
        desired_dof_pos[i] = rl_params_.default_dof_pos[i] + rl_params_.action_scale[i] * actions_robot_order[i];
    }
    {
        std::lock_guard<std::mutex> command_lock(command_mutex_);
        des_dof_pos_ = std::move(desired_dof_pos);
    }
    last_actions_ = actions_;
}

const FSMStateRLTracking::MotionData& FSMStateRLTracking::active_motion_locked() const {
    return motions_by_id_.at(active_motion_id_);
}

FSMStateRLTracking::PolicyRuntime& FSMStateRLTracking::active_policy() {
    return policies_.at(active_policy_index_);
}

const FSMStateRLTracking::PolicyRuntime& FSMStateRLTracking::active_policy() const {
    return policies_.at(active_policy_index_);
}

Eigen::Quaterniond FSMStateRLTracking::compute_torso_quat(const Eigen::Quaterniond& pelvis_quat,
                                                          const Eigen::Vector3d& waist_position) const {
    // pelvis * waist (yaw, roll, pitch)
    Eigen::Quaterniond torso_quat = pelvis_quat.normalized() *
                                    Eigen::AngleAxisd(waist_position[0], Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxisd(waist_position[1], Eigen::Vector3d::UnitX()) *
                                    Eigen::AngleAxisd(waist_position[2], Eigen::Vector3d::UnitY());
    torso_quat.normalize();
    return torso_quat;
}

void FSMStateRLTracking::onExit() {
    // wait for the in-flight inference
    rl_model_inference_running_ = false;
    std::lock_guard<std::mutex> inference_lock(inference_mutex_);
    LOG(INFO) << state_name_ << "::onExit()";
}

StateID FSMStateRLTracking::check_transition() {
    const StateID requested_state = fsm_data_ptr_->desired_command_ptr->state_id;
    switch (requested_state) {
        case StateID::RL_TRACKING:
            return state_id_;
        case StateID::PASSIVE:
        case StateID::DAMPER:
        case StateID::RECOVERY_STAND:
        case StateID::RL_LOCOMOTION:
            return requested_state;
        default:
            LOG(WARNING) << "Bad Request: Cannot transition from " << StateID::RL_TRACKING << " to " << requested_state;
            return state_id_;
    }
}

FSMStateRLTracking::~FSMStateRLTracking() {
    rl_model_inference_running_ = false;
    controller_running_ = false;
    if (rl_model_inference_thread_.joinable()) {
        rl_model_inference_thread_.join();
    }
    LOG(INFO) << '[' << state_name_
              << "] inference thread stopped; total max/avg=" << rl_timer_.get_max_interval_in_milliseconds() << "/"
              << rl_timer_.get_average_in_milliseconds()
              << " ms, ONNX max/avg=" << rl_action_inference_timer_.get_max_interval_in_milliseconds() << "/"
              << rl_action_inference_timer_.get_average_in_milliseconds() << " ms";
}
