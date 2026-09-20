#ifndef VITA_BOY_FSM_RL_BASE_HPP
#define VITA_BOY_FSM_RL_BASE_HPP

#include <atomic>
#include <mutex>
#include <thread>
#include <typeinfo>
#include <sched.h>
#include <pthread.h>

#include "state_machine/fsm_state.hpp"
#include "common/onnx_helpers.hpp"
#include "common/helpers.hpp"
#include "common/benchmark.hpp"

struct RLParams {
    Eigen::VectorXd default_dof_pos;
    Eigen::VectorXd kp;
    Eigen::VectorXd kd;
    int num_observations;
    int num_actions;
    std::string policy_name;
    int control_decimation{10};
    int observations_history;
    double dof_pos_scale;
    double dof_vel_scale;
    double lin_vel_scale;
    double ang_vel_scale;
    double clip_obs;
    double clip_action;
    Eigen::VectorXd clip_actions_upper;
    Eigen::VectorXd clip_actions_lower;
    Eigen::VectorXd action_scale;
    Eigen::VectorXd commands_scale;
    Eigen::VectorXd joint_mapping;
    std::vector<std::string> observations;

    int intra_op_num_threads{1};
    int inter_op_num_threads{1};
    bool bind_inference_thread_to_core{false};
    std::vector<int> assigned_inference_cores;
};

struct Observations {
    Eigen::VectorXd lin_vel;
    Eigen::VectorXd ang_vel;
    Eigen::VectorXd gravity_vec;  // 0, 0, -1
    Eigen::VectorXd commands;
    Eigen::Quaternion<double> base_quat;  // w, x, y, z
    Eigen::VectorXd dof_pos;
    Eigen::VectorXd dof_vel;
    Eigen::VectorXd actions;
};

class FSMRLBase : public FSMState {
   public:
    FSMRLBase(std::shared_ptr<FSMData> fsm_data_ptr, StateID id, std::string name);

    void onEnter() override;
    void run() override;
    void onExit() override;
    StateID check_transition() override;

    void init_rl_base(const std::string config_path);
    void init_obs();
    void init_outputs();
    void read_rl_yaml(const std::string config_path);
    void init_onnx_model(const std::string& policy_path);
    void init_rl_model_inference_thread();
    virtual void run_model() = 0;
    Eigen::VectorXd covert_dof_to_action(const Eigen::VectorXd& dof_pos);

    void print_vector(const Eigen::Ref<const Eigen::VectorXd>& vec, const std::string& name);
    void print_vector(const Eigen::Ref<const Eigen::VectorXi>& vec, const std::string& name);
    void print_rl_params(const RLParams& rl_params);
    void init_head_control();
    void head_control();

    RLParams rl_params_;
    std::mutex obs_mutex_;
    Observations obs_;
    Eigen::Vector3d gravity_vec_;
    XbotOnnxRuntime ort_;
    std::vector<std::vector<int64_t>> input_dims_;
    std::vector<std::vector<int64_t>> output_dims_;
    std::vector<float*> input_;
    std::vector<float*> output_;
    Eigen::VectorXd actions_;
    Eigen::VectorXd last_actions_;
    bool controller_running_{false};
    std::atomic<bool> rl_model_inference_running_{false};
    std::thread rl_model_inference_thread_;
    size_t rl_thread_priority_{50};
    size_t rl_control_count_sim_single_thread_{0};
    int rl_counter_{0};
    double rl_desired_frequency_;
    double rl_desired_time_step_;
    double transition_time_;

    bool yaw_to_scale_;
    double yaw_to_scale_vel_threshold_;

    Eigen::VectorXd des_dof_pos_;
    Eigen::VectorXd default_dof_pos_;
    Eigen::VectorXd kp_;
    Eigen::VectorXd kd_;
    Eigen::VectorXd q_init_;
    Eigen::VectorXd q_cmd_init_;

    RepeatedTimer rl_timer_;

    // head control
    Eigen::Matrix<double, 3, 1> head_q_init_;
    Eigen::Matrix<double, 3, 1> head_q_start_time_;
    Eigen::Matrix<double, 3, 1> last_head_q_cmd_;

   protected:
    static bool bind_current_thread_to_cpus(const std::vector<int>& core_ids);
    static bool bind_current_thread_to_cpu(int core_id);
    static bool bind_thread_to_cpu(std::thread& thread, int core_id);

   private:
};

#endif  // VITA_BOY_FSM_RL_BASE_HPP
