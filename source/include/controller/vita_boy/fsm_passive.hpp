#ifndef VITA_BOY_FSM_PASSIVE_HPP
#define VITA_BOY_FSM_PASSIVE_HPP

#include "state_machine/fsm_state.hpp"

class FSMStatePassive : public FSMState
{
public:
    // Constructor
    FSMStatePassive(std::shared_ptr<FSMData> fsm_data_ptr);

    // onEnter
    void onEnter() override;

    // run
    void run() override;

    // onExit
    void onExit() override;

    // Resolve the next state
    StateID check_transition() override;
};

#endif  // VITA_BOY_FSM_PASSIVE_HPP
