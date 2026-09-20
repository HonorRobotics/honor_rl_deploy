#include "estimator/state_estimation.hpp"

StateEstimation::StateEstimation(std::shared_ptr<HardwareInterface> hw_interface_ptr,
                                 std::shared_ptr<FSMData> fsm_data_ptr)
    : hw_interface_ptr_(hw_interface_ptr), fsm_data_ptr_(fsm_data_ptr) {
    if (!hw_interface_ptr_) {
        throw std::invalid_argument("HardwareInterface pointer cannot be null");
    }
}

void StateEstimation::update() {
    auto robot_state_ptr = hw_interface_ptr_->get_state();
    fsm_data_ptr_->robot_state_ptr = robot_state_ptr;
}

StateEstimation::~StateEstimation() {}