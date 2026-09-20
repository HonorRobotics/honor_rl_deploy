#ifndef VITA_BOY_FSM_DAMPER_HPP
#define VITA_BOY_FSM_DAMPER_HPP

#include "state_machine/fsm_state.hpp"

class FSMStateDamper : public FSMState {
   public:
    // Constructor
    FSMStateDamper(std::shared_ptr<FSMData> fsm_data_ptr);

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
    void read_yaml(const std::string& config_path);

    Eigen::VectorXd kp_;
    Eigen::VectorXd kd_;
};

#endif  // VITA_BOY_FSM_DAMPER_HPP
