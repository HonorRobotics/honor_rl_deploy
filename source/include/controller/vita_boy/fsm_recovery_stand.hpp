#ifndef VITA_BOY_FSM_RECOVERY_STAND_HPP
#define VITA_BOY_FSM_RECOVERY_STAND_HPP

#include "state_machine/fsm_state.hpp"

class FSMStateRecoveryStand : public FSMState
{
public:
    // Constructor
    FSMStateRecoveryStand(std::shared_ptr<FSMData> fsm_data_ptr);

    // onEnter
    void onEnter() override;

    // run
    void run() override;

    // onExit
    void onExit() override;

    // Resolve the next state
    StateID check_transition() override;

private:
    // Read config file.
    void read_yaml(const std::string &config_path);

    Eigen::VectorXd default_dof_pos_;
    Eigen::VectorXd kp_;
    Eigen::VectorXd kd_;

    Eigen::VectorXd q_init_;
    Eigen::VectorXd head_q_init_;
    double transition_time_{1.0};
};

#endif  // VITA_BOY_FSM_RECOVERY_STAND_HPP
