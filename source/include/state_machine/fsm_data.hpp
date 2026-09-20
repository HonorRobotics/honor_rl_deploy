#ifndef FSM_DATA_HPP
#define FSM_DATA_HPP

#include <memory>
#include "common_definition.hpp"
#include "interface/interface_parameter.hpp"

struct FSMData {
    bool is_sim;
    std::string robot_name;
    std::string robot_version;
    size_t mode_machine;
    int num_dofs;
    int head_num_dofs;
    double control_dt;
    Eigen::VectorXd head_kp;
    Eigen::VectorXd head_kd;
    StateID current_state_id;
    std::vector<std::string> supported_states;

    std::shared_ptr<interface::InterfaceParameter> interface_parameter_ptr_ =
        std::make_shared<interface::InterfaceParameter>();

    // robot state
    std::shared_ptr<RobotState<double>> robot_state_ptr = std::make_shared<RobotState<double>>();

    // robot cmd
    std::shared_ptr<RobotCommand<double>> robot_command_ptr = std::make_shared<RobotCommand<double>>();

    // desired cmd
    std::shared_ptr<DesiredCommand<double>> desired_command_ptr = std::make_shared<DesiredCommand<double>>();
};

#endif  // FSM_DATA_HPP
