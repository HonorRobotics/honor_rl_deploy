
#include <iostream>
#include <algorithm>
#include <chrono>
#include <thread>
#include <atomic>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <getopt.h>
#include "state_machine/control_fsm.hpp"
#include "hardware/hardware_interface.hpp"
#include "estimator/state_estimation.hpp"
#include "command/command_interface.hpp"
#include "controller/vita_boy/fsm.hpp"
#include "controller.hpp"
#include "common/glog_sink.hpp"

#include "argparse/argparse.hpp"

Controller::Controller(const std::string &robot_name, const std::string &robot_version)
{
    const std::time_t now = std::time(nullptr);
    std::tm local_time{};
    localtime_r(&now, &local_time);
    LOG(INFO) << "\n========================================================================"
              << "\n[Controller] Start time: " << std::put_time(&local_time, "%F %T")
              << "\n========================================================================";

    std::string robot_config_path = "../config/" + robot_name + "/" + robot_version;
    std::string base_config_path = robot_config_path + "/base.yaml";
    std::string command_config_path = robot_config_path + "/command.yaml";
    fsm_data_ptr_ = std::make_shared<FSMData>();

    // read basic info
    read_base_yaml(base_config_path, robot_name);
    control_dt_ = fsm_data_ptr_->control_dt;
    print_fsm_data();

    hw_interface_ptr_ = std::make_shared<HardwareInterface>(base_config_path, fsm_data_ptr_);
    est_interface_ptr_ = std::make_shared<StateEstimation>(hw_interface_ptr_, fsm_data_ptr_);
    cmd_interface_ptr_ = std::make_shared<CommandInterface>(command_config_path, hw_interface_ptr_, fsm_data_ptr_);
    safety_checker_ptr_ = std::make_shared<SafetyChecker>(fsm_data_ptr_);

    // auto load FSM by robot_name
    if (!FSMManager::get_instance().is_type_supported(robot_name))
    {
        throw std::runtime_error("No FSM registered for robot: " + robot_name);
    }

    fsm_ptr_ = FSMManager::get_instance().create_FSM(
        robot_name, robot_version, fsm_data_ptr_, this);
    if (!fsm_ptr_)
    {
        throw std::runtime_error(
            "Failed to create FSM for robot: " + robot_name +
            ", version: " + robot_version);
    }
}

void Controller::read_base_yaml(
    const std::string& config_path, const std::string& robot_name) {
    try
    {
        const YAML::Node config = YAML::LoadFile(config_path)[robot_name];
        if (!config || !config.IsMap()) {
            throw std::runtime_error(
                "Missing or invalid '" + robot_name + "' configuration");
        }

        fsm_data_ptr_->is_sim = config["is_sim"].as<bool>();
        fsm_data_ptr_->robot_name = config["robot_name"].as<std::string>();
        fsm_data_ptr_->robot_version = config["robot_version"].as<std::string>();
        fsm_data_ptr_->control_dt = config["control_dt"].as<double>();
        fsm_data_ptr_->num_dofs = config["num_dofs"].as<int>();
        fsm_data_ptr_->head_num_dofs = config["head_num_dofs"].as<int>();
        read_vector_from_yaml(config["head_kp"], fsm_data_ptr_->head_kp);
        read_vector_from_yaml(config["head_kd"], fsm_data_ptr_->head_kd);
        for (const auto& state : config["supported_states"]) {
            fsm_data_ptr_->supported_states.push_back(state.as<std::string>());
        }
    }
    catch (const std::exception &e)
    {
        throw std::runtime_error(
            "Failed to load base configuration '" + config_path + "': " + e.what());
    }
}

void Controller::pre_update_interface()
{
    // update estimation
    est_interface_ptr_->update();

    // update command
    cmd_interface_ptr_->update();

    // check safety
    safety_ = check_safety();

    // if not safe, change to damper
    if (!safety_) {
        cmd_interface_ptr_->modify_cmd_for_safety();
    }
}

bool Controller::check_safety()
{
    auto state_id = fsm_data_ptr_->current_state_id;
    // for some state, safety is always true
    if (state_id == StateID::PASSIVE ||
        state_id == StateID::DAMPER ||
        state_id == StateID::RECOVERY_STAND
    ) {
        // check imu norm
        safety_ = safety_checker_ptr_->check_imu_quaternion_norm();
    } else {
        // check joint pos, vel and tau limit; base orientation limit
        safety_ = safety_checker_ptr_->pre_check();

        // check imu norm
        if (hw_interface_ptr_->is_imu_init()) {
            safety_ = safety_ && safety_checker_ptr_->check_imu_quaternion_norm();
        }
    }
    // check motor error code and temperature
    const bool motor_error_safe = safety_checker_ptr_->check_motor_error_code();
    const bool motor_mos_temperature_safe = safety_checker_ptr_->check_motor_mos_temperature();
    const bool motor_rotor_temperature_safe = safety_checker_ptr_->check_motor_rotor_temperature();
    safety_ = safety_ && motor_error_safe;
    safety_ = safety_ && motor_mos_temperature_safe;
    safety_ = safety_ && motor_rotor_temperature_safe;

    return safety_;
}

void Controller::print_fsm_data()
{
    std::ostringstream oss;
    const auto append_vector = [&oss](const Eigen::VectorXd& values) {
        oss << '[';
        for (Eigen::Index i = 0; i < values.size(); ++i) {
            if (i > 0) {
                oss << ", ";
            }
            oss << values[i];
        }
        oss << ']';
    };

    oss << "\n┌────────────────────────────────────────"
        << "\n│ Controller Configuration"
        << "\n├─ Robot"
        << "\n│   Name         : " << fsm_data_ptr_->robot_name
        << "\n│   Version      : " << fsm_data_ptr_->robot_version
        << "\n│   Simulation   : " << std::boolalpha << fsm_data_ptr_->is_sim
        << "\n├─ Control"
        << "\n│   Mode Machine : " << fsm_data_ptr_->mode_machine
        << "\n│   Period       : " << fsm_data_ptr_->control_dt << " s"
        << "\n│   Body DoFs    : " << fsm_data_ptr_->num_dofs
        << "\n│   Head DoFs    : " << fsm_data_ptr_->head_num_dofs
        << "\n├─ Head Gains"
        << "\n│   Kp           : ";
    append_vector(fsm_data_ptr_->head_kp);
    oss << "\n│   Kd           : ";
    append_vector(fsm_data_ptr_->head_kd);
    oss << "\n└────────────────────────────────────────";

    LOG(INFO) << oss.str();
}

void Controller::post_update_interface()
{
    fsm_data_ptr_->current_state_id = fsm_ptr_->get_current_state_id();

    hw_interface_ptr_->set_command(fsm_data_ptr_->robot_command_ptr->motor_command.q,
                                   fsm_data_ptr_->robot_command_ptr->motor_command.dq,
                                   fsm_data_ptr_->robot_command_ptr->motor_command.kp,
                                   fsm_data_ptr_->robot_command_ptr->motor_command.kd,
                                   fsm_data_ptr_->robot_command_ptr->motor_command.tau);
}

void Controller::run()
{
    try {
        using Clock = std::chrono::steady_clock;
        const auto control_period = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(control_dt_));
        auto next_tick = Clock::now();
        auto previous_cycle_start = next_tick;
        bool has_previous_cycle = false;

        while (rclcpp::ok() && !should_exit_) {
            const auto scheduled_start = next_tick;
            next_tick += control_period;
            counter_++;
            const auto cycle_start = Clock::now();

            if (has_previous_cycle) {
                const double actual_period = std::chrono::duration<double>(
                    cycle_start - previous_cycle_start).count();
                actual_period_sum_ += actual_period;
                actual_period_max_ = std::max(actual_period_max_, actual_period);
                scheduling_sample_count_++;
            }
            if (has_previous_cycle && cycle_start > scheduled_start) {
                const double wakeup_lateness = std::chrono::duration<double>(
                    cycle_start - scheduled_start).count();
                wakeup_lateness_sum_ += wakeup_lateness;
                wakeup_lateness_max_ = std::max(
                    wakeup_lateness_max_, wakeup_lateness);
            }
            previous_cycle_start = cycle_start;
            has_previous_cycle = true;

            fsm_timer_.start_timer();
            pre_update_interface();    //previous: retrieve status and commands from hardware interfaces
            if (hw_interface_ptr_->is_init()) {
                if (!fsm_running_) {
                    LOG(INFO) << "[Controller::run()] Info: FSM is running...";
                    fsm_running_ = true;
                }
                fsm_ptr_->run();       //step: run control state machines (such as RL controllers)
            } else {
                if (!hw_interface_ptr_->is_imu_init()) {
                    LOG_EVERY_N(ERROR, 500) << "[Controller::run()] Error: IMU is not ready, please check hardware!!!";
                } else if (!hw_interface_ptr_->is_low_state_init()) {
                    LOG_EVERY_N(ERROR, 500) << "[Controller::run()] Error: Motor is not ready, please check hardware!!!";
                } else if (!hw_interface_ptr_->is_joy_init()) {
                    LOG_EVERY_N(ERROR, 500) << "[Controller::run()] Error: Joy stick is not ready, please check hardware!!!";
                }
            }
            post_update_interface();  // post: set commands to hardware interface
            fsm_timer_.end_timer();

            const auto cycle_end = Clock::now();
            const auto cycle_time = cycle_end - cycle_start;
            cycle_time_ = std::chrono::duration<double>(cycle_time).count();

            if (cycle_end < next_tick) {
                std::this_thread::sleep_until(next_tick);
            } else {
                const auto lateness = cycle_end - next_tick;
                const double deadline_lateness =
                    std::chrono::duration<double>(lateness).count();
                missed_deadline_count_++;
                deadline_lateness_max_ = std::max(
                    deadline_lateness_max_, deadline_lateness);
                LOG_EVERY_N(WARNING, 500)
                    << "[Controller::run()] Control deadline missed by: "
                    << deadline_lateness << " s"
                    << " cycle_time: " << cycle_time_ << " s"
                    << " scheduled time: " << counter_ * control_dt_ << " s";

                // Avoid a burst of back-to-back cycles after a long stall.
                if (lateness > control_period * 5) {
                    next_tick = cycle_end;
                    scheduler_resync_count_++;
                }
            }
        }
    } catch (const std::exception& e) {
        should_exit_ = true;
        LOG(ERROR) << "Exception in run loop: " << e.what();
        throw;
    }
}

Controller::~Controller()
{
    fsm_ptr_->Stop();
    std::stringstream ss;
    ss << "\n########################################################################";
    ss << "\n### Controller Benchmarking";
    ss << "\n###   Maximum : " << fsm_timer_.get_max_interval_in_milliseconds() << "[ms].";
    ss << "\n###   Average : " << fsm_timer_.get_average_in_milliseconds() << "[ms].";
    const double average_actual_period = scheduling_sample_count_ > 0
        ? actual_period_sum_ / static_cast<double>(scheduling_sample_count_)
        : 0.0;
    const double average_wakeup_lateness = scheduling_sample_count_ > 0
        ? wakeup_lateness_sum_ / static_cast<double>(scheduling_sample_count_)
        : 0.0;
    ss << "\n### Scheduling Metrics";
    ss << "\n###   Target Period          : " << control_dt_ * 1000.0 << "[ms].";
    ss << "\n###   Average Actual Period  : " << average_actual_period * 1000.0 << "[ms].";
    ss << "\n###   Maximum Actual Period  : " << actual_period_max_ * 1000.0 << "[ms].";
    ss << "\n###   Average Wakeup Lateness: " << average_wakeup_lateness * 1000.0 << "[ms].";
    ss << "\n###   Maximum Wakeup Lateness: " << wakeup_lateness_max_ * 1000.0 << "[ms].";
    ss << "\n###   Missed Deadlines       : " << missed_deadline_count_;
    ss << "\n###   Maximum Deadline Miss  : " << deadline_lateness_max_ * 1000.0 << "[ms].";
    ss << "\n###   Scheduler Resyncs      : " << scheduler_resync_count_;
    ss << "\n###   Scheduling Samples     : " << scheduling_sample_count_;
    ss << "\n########################################################################";
    ss << "\n### Controller Terminated";
    ss << "\n########################################################################";
    ss << "\n";
    LOG(INFO) << ss.str();
}

void get_bind_info_from_yaml(
    const std::string& config_path,
    const std::string& robot_name,
    bool& bind_main_thread_to_core,
    std::vector<int>& assigned_main_cores) {
    try
    {
        const YAML::Node config = YAML::LoadFile(config_path)[robot_name];
        if (!config || !config.IsMap()) {
            throw std::runtime_error(
                "Missing or invalid '" + robot_name + "' configuration");
        }

        const bool parsed_bind = config["bind_main_thread_to_core"].as<bool>();
        std::vector<int> parsed_cores;
        read_std_vector_int_from_yaml(config["assigned_main_cores"], parsed_cores);

        bind_main_thread_to_core = parsed_bind;
        assigned_main_cores = std::move(parsed_cores);
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error(
            "Failed to load CPU binding configuration '" + config_path + "': " + e.what());
    }
}

int main(int argc, char** argv) {

    // process input parameters
    const std::string robot_name{"vita_boy"};
    argparse::ArgumentParser program("motion_intelligence");
    program.add_argument("-v", "--robot-version")
           .default_value(std::string{"V1"})
           .help("Robot version (default: V1)");
    try {
        program.parse_args(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n' << program;
        return 1;
    }

    // get bind config
    const std::string robot_version = program.get<std::string>("--robot-version");
    const std::string robot_config_path = "../config/" + robot_name + "/" + robot_version;
    const std::string base_config_path = robot_config_path + "/base.yaml";

    bool bind_main_thread_to_core{false};
    std::vector<int> assigned_main_cores;
    try {
        get_bind_info_from_yaml(
            base_config_path, robot_name, bind_main_thread_to_core, assigned_main_cores);
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }

    // init ros2
    rclcpp::init(argc, argv);

    // init logger
    FLAGS_alsologtostderr = 1; // info to the terminal
    const std::string log_file = "deploy.log";
    LogManager::GetInstance().Initialize(log_file, argv[0]);

    // bind main thread to cpus
    if (bind_main_thread_to_core) {
        bind_current_thread_to_cpu_ids(assigned_main_cores);
    } else {
        LOG(INFO) << "Main thread CPU binding is disabled";
    }

    // init and run controller
    int exit_code = 0;
    try {
        Controller controller(robot_name, robot_version);
        LOG(INFO) << "Controller started";
        controller.run();
    } catch (const std::exception& e) {
        LOG(ERROR) << "Controller failed: " << e.what();
        exit_code = 1;
    }

    rclcpp::shutdown();
    return exit_code;
}
