#ifndef VITA_BOY_FSM_RL_TRACKING_HPP
#define VITA_BOY_FSM_RL_TRACKING_HPP

#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "state_machine/fsm_rl_base.hpp"

class FSMStateRLTracking : public FSMRLBase
{
public:
    // Constructor
    FSMStateRLTracking(std::shared_ptr<FSMData> fsm_data_ptr, StateID state_id,
                       std::string state_name);

    // Destructor
    ~FSMStateRLTracking() override;

    // onEnter
    void onEnter() override;

    // run
    void run() override;

    // onExit
    void onExit() override;

    // Resolve the next state
    StateID check_transition() override;

private:
    static constexpr int kNumActions = 29;
    static constexpr int kNumProprioceptiveObservations = 96;
    static constexpr int kNumDemoObservations = 58;
    static constexpr int kAnchorBodyIndex = 9;
    static constexpr size_t kInvalidId = std::numeric_limits<size_t>::max();

    using RowMajorMatrixXf =
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

    // ONNX policy config
    struct ModelConfig
    {
        size_t model_id{0};
        std::string model_file;
        int prop_hist{1};
        int demo_hist{1};

        // Observation buffer size
        int observation_buffer_size() const
        {
            return prop_hist * kNumProprioceptiveObservations +
                   demo_hist * kNumDemoObservations;
        }
    };

    // Motion config
    struct MotionConfig
    {
        size_t motion_id{0};
        size_t model_id{0};
        std::string motion_file;
        size_t start_frame{0};
        size_t end_frame{0};  // 0 = to end of file
    };

    // Loaded motion data
    struct MotionData
    {
        MotionConfig config;
        int fps{0};
        RowMajorMatrixXf joint_pos;
        RowMajorMatrixXf joint_vel;
        RowMajorMatrixXf anchor_quat_w;

        // Frame count
        size_t frame_count() const
        {
            return static_cast<size_t>(joint_pos.rows());
        }
    };

    // Loaded ONNX policy
    struct PolicyRuntime
    {
        ModelConfig config;
        std::string path;
        std::unique_ptr<XbotOnnxRuntime> session;
        std::vector<std::vector<int64_t>> input_dims;
        std::vector<std::vector<int64_t>> output_dims;
        std::vector<std::vector<float>> input_buffers;
        std::vector<std::vector<float>> output_buffers;
        std::vector<float*> inputs;
        std::vector<float*> outputs;
    };

    // Initialize
    void initialize(const std::string& config_path);

    // Read config file.
    void read_config(const std::string& config_path);

    // Validate config
    void validate_config() const;

    // Load motions
    void load_motions();

    // Load motion
    MotionData load_motion(const MotionConfig& config) const;

    // Initialize policies
    void initialize_policies();

    // Initialize observations
    void initialize_observations();

    // Initialize history
    void initialize_history();

    // Run inference
    void run_model() override;

    // Update active motion
    void update_active_motion();

    // Activate motion
    void activate_motion(size_t motion_id);

    // Reset motion progress, caller must hold motion_mutex_
    void reset_motion_progress_locked();

    // Update measured state
    void update_measured();

    // Update motion reference
    void update_motion_reference();

    // Update observation
    void update_observation();

    // Update history
    void update_history();

    // Update observation history
    void update_observation_history();

    // Update action
    void update_action();

    // Build observation buffer
    std::vector<double> build_observation_buffer() const;

    // Active motion, caller must hold motion_mutex_
    const MotionData& active_motion_locked() const;

    // Active policy
    PolicyRuntime& active_policy();
    const PolicyRuntime& active_policy() const;

    // Compute torso orientation
    Eigen::Quaterniond compute_torso_quat(
        const Eigen::Quaterniond& pelvis_quat,
        const Eigen::Vector3d& waist_position) const;

    std::string config_directory_;
    std::vector<ModelConfig> model_configs_;
    std::vector<MotionConfig> motion_configs_;
    std::vector<PolicyRuntime> policies_;
    std::unordered_map<size_t, size_t> policy_index_by_id_;
    std::unordered_map<size_t, MotionData> motions_by_id_;

    size_t default_motion_id_{0};
    size_t active_motion_id_{kInvalidId};    // guarded by motion_mutex_
    size_t active_policy_index_{0};          // guarded by motion_mutex_
    size_t last_rejected_motion_id_{kInvalidId};

    std::vector<double> obs_prop_vec_;
    std::vector<double> obs_demo_vec_;
    std::vector<double> obs_buffer_vec_;
    std::vector<std::vector<double>> obs_hist_prop_vec2d_;
    std::vector<std::vector<double>> obs_hist_demo_vec2d_;

    Eigen::VectorXd last_fsm_kp_;
    Eigen::VectorXd last_fsm_kd_;
    Eigen::Quaterniond ref_anchor_quat_global_{Eigen::Quaterniond::Identity()};
    Eigen::Quaterniond quat_yaw_from_demo_to_curr_{Eigen::Quaterniond::Identity()};

    bool is_single_thread_{false};
    bool first_update_hist_{true};
    bool init_demo_yaw_flag_{false};
    bool reset_track_motion_cmd_time_{false};
    bool send_result_{false};
    double motion_cmd_time_{0.0};
    double motion_prepare_time_{0.0};

    std::mutex inference_mutex_;       // serializes inference with state lifecycle
    std::mutex command_mutex_;         // protects des_dof_pos_
    mutable std::mutex motion_mutex_;  // protects active motion and progress
    RepeatedTimer rl_action_inference_timer_;
};

#endif  // VITA_BOY_FSM_RL_TRACKING_HPP
