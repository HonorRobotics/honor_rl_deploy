#ifndef FSM_STATE_HPP
#define FSM_STATE_HPP

#include <iostream>
#include <Eigen/Dense>
#include "state_machine/fsm_data.hpp"
#include "common/yaml_helper.hpp"
#include "common/glog_sink.hpp"

namespace LOGGER
{
    const char *const INFO    = "\033[0;37m[INFO]\033[0m ";
    const char *const WARNING = "\033[0;33m[WARNING]\033[0m ";
    const char *const ERROR   = "\033[0;31m[ERROR]\033[0m ";
    const char *const DEBUG   = "\033[0;32m[DEBUG]\033[0m ";
}

class FSMState
{
public:
    FSMState(std::shared_ptr<FSMData> fsm_data_ptr, StateID id, std::string name) : fsm_data_ptr_(fsm_data_ptr), state_id_(id), state_name_(std::move(name)) {}
    virtual ~FSMState() = default;
    virtual void onEnter() = 0;
    virtual void run() = 0;
    virtual void onExit() = 0;
    virtual StateID check_transition() { return state_id_; }
    const std::string &get_state_name() const { return state_name_; }
    const StateID &get_state_id() const { return state_id_; }

    void print_data() {
        for(int i = 0; i < fsm_data_ptr_->robot_state_ptr->motor_state.q.size(); ++i) {
            LOG(INFO) << "motor pos: " << fsm_data_ptr_->robot_state_ptr->motor_state.q[i];
        }
    }

protected:
    size_t counter_{0};
    StateID state_id_;
    StateID next_state_id_;
    std::string state_name_;
    std::string next_state_name_;
    std::shared_ptr<FSMData> fsm_data_ptr_;
};

#endif // FSM_STATE_HPP