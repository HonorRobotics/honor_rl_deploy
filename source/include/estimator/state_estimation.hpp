#ifndef STATE_ESTIMATION_HPP
#define STATE_ESTIMATION_HPP

#include <memory>
#include <stdexcept>
#include "hardware/hardware_interface.hpp"
#include "state_machine/fsm_data.hpp"

class StateEstimation
{
public:
    StateEstimation(std::shared_ptr<HardwareInterface> hw_interface_ptr, std::shared_ptr<FSMData> fsm_data_ptr);
    ~StateEstimation();
    void update();

private:
    std::shared_ptr<HardwareInterface> hw_interface_ptr_;
    std::shared_ptr<FSMData> fsm_data_ptr_;
};

#endif // STATE_ESTIMATION_HPP

