#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <atomic>
#include <string>

#include "state_machine/control_fsm.hpp"
#include "hardware/hardware_interface.hpp"
#include "estimator/state_estimation.hpp"
#include "command/command_interface.hpp"
#include "common/benchmark.hpp"
#include "controller/vita_boy/safety_checker.hpp"
#include "common/helpers.hpp"

class Controller {
   public:
    Controller(const std::string& robot_name, const std::string& robot_version);
    ~Controller();

    void run();
    void pre_update_interface();
    void post_update_interface();

    void read_base_yaml(const std::string& config_path, const std::string& robot_name);
    void print_fsm_data();
    bool check_safety();
    bool is_sim() {
        return fsm_data_ptr_->is_sim;
    };

   private:
    size_t counter_{0};
    double control_dt_{0.002};
    double cycle_time_{0.0};
    uint64_t scheduling_sample_count_{0};
    uint64_t missed_deadline_count_{0};
    uint64_t scheduler_resync_count_{0};
    double actual_period_sum_{0.0};
    double actual_period_max_{0.0};
    double wakeup_lateness_sum_{0.0};
    double wakeup_lateness_max_{0.0};
    double deadline_lateness_max_{0.0};
    std::atomic<bool> should_exit_{false};
    RepeatedTimer fsm_timer_;
    std::string robot_name;
    bool safety_{true};
    bool fsm_running_{false};

    std::shared_ptr<FSM> fsm_ptr_;
    std::shared_ptr<FSMData> fsm_data_ptr_;
    std::shared_ptr<HardwareInterface> hw_interface_ptr_;
    std::shared_ptr<StateEstimation> est_interface_ptr_;
    std::shared_ptr<CommandInterface> cmd_interface_ptr_;
    std::shared_ptr<SafetyChecker> safety_checker_ptr_;
};

#endif  // CONTROLLER_HPP
