#ifndef VITA_BOY_FSM_HPP
#define VITA_BOY_FSM_HPP

#include "state_machine/control_fsm.hpp"
#include "state_machine/fsm_data.hpp"
#include "controller/vita_boy/fsm_passive.hpp"
#include "controller/vita_boy/fsm_damper.hpp"
#include "controller/vita_boy/fsm_recovery_stand.hpp"
#include "controller/vita_boy/fsm_rl_locomotion.hpp"
#include "controller/vita_boy/fsm_rl_tracking.hpp"


class VitaBoyFSMFactory : public FSMFactory
{
public:
    VitaBoyFSMFactory(const StateID& initial) : initial_state_(initial) {}
    std::shared_ptr<FSMState> create_state(void *context, std::shared_ptr<FSMData> fsm_data_ptr, const std::string &state_name) override
    {
        if (state_name == "FSMStatePassive")
            return std::make_shared<FSMStatePassive>(fsm_data_ptr);
        else if (state_name == "FSMStateDamper")
            return std::make_shared<FSMStateDamper>(fsm_data_ptr);
        else if (state_name == "FSMStateRecoveryStand")
            return std::make_shared<FSMStateRecoveryStand>(fsm_data_ptr);
        else if (state_name == "FSMStateRLLocomotion")
            return std::make_shared<FSMStateRLLocomotion>(fsm_data_ptr);
        else if (state_name == "FSMStateRLTracking")
            return std::make_shared<FSMStateRLTracking>(fsm_data_ptr, StateID::RL_TRACKING ,state_name);
        return nullptr;
    }
    std::string get_type() const override { return "vita_boy"; }

    std::vector<std::string> get_supported_states(std::shared_ptr<FSMData> fsm_data_ptr) const override
    {
        return fsm_data_ptr->supported_states;
    }

    StateID get_initial_state() const override { return initial_state_; }
private:
    StateID initial_state_;
};

REGISTER_FSM_FACTORY(VitaBoyFSMFactory, StateID::PASSIVE)

#endif  // VITA_BOY_FSM_HPP
