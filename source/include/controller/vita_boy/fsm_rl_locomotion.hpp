#ifndef VITA_BOY_FSM_RL_LOCOMOTION_HPP
#define VITA_BOY_FSM_RL_LOCOMOTION_HPP

#include "state_machine/fsm_rl_base.hpp"

class FSMStateRLLocomotion : public FSMRLBase
{
public:
    // Constructor
    FSMStateRLLocomotion(std::shared_ptr<FSMData> fsm_data_ptr);

    // Destructor
    ~FSMStateRLLocomotion();

    // onEnter
    void onEnter() override;

    // run
    void run() override;

    // onExit
    void onExit() override;

    // Resolve the next state
    StateID check_transition() override;

    // Run inference
    void run_model() override;

    // Update measured state
    void update_measured();

    // Update command
    void update_command();

    // Update observation
    void update_observation();

    // Update history
    void update_history();

    // Update action
    void update_action();

private:
    Eigen::Vector3d locomotion_command_;
    std::vector<double> obs_vec_;
    std::vector<double> obs_history_vec_;
    bool history_initialized_{false};

    static constexpr double pi = 3.1415926;

    // Reset input
    void reset_input();
};

#endif // VITA_BOY_FSM_RL_LOCOMOTION_HPP
