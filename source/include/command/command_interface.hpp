#ifndef COMMAND_INTERFACE_HPP
#define COMMAND_INTERFACE_HPP

#include <memory>
#include <string>

#include "state_machine/common_definition.hpp"

class HardwareInterface;
struct FSMData;

class CommandInterface
{
public:
    CommandInterface(const std::string &config_path, std::shared_ptr<HardwareInterface> hw_interface_ptr, std::shared_ptr<FSMData> fsm_data_ptr);
    void update();
    void modify_cmd_for_safety();

private:
    void print_command();
    void reset_desired_command();
    void read_cmd_config(const std::string &config_path);
    void map_joystick_to_raw_cmd();
    void map_topic_to_raw_cmd();
    void filter_command();
    void clamp_command();
    void apply_velocity_dead_zone();

    std::shared_ptr<HardwareInterface> hw_interface_ptr_;
    std::shared_ptr<FSMData> fsm_data_ptr_;

    Eigen::VectorXd pos_des_min_;
    Eigen::VectorXd pos_des_max_;
    Eigen::VectorXd vel_des_min_;
    Eigen::VectorXd vel_des_max_;

    Eigen::VectorXd ori_des_min_;
    Eigen::VectorXd ori_des_max_;
    Eigen::VectorXd ome_des_min_;
    Eigen::VectorXd ome_des_max_;

    Eigen::VectorXd loco_pos_filter_para_;
    Eigen::VectorXd loco_vel_filter_para_;
    Eigen::VectorXd loco_ori_filter_para_;
    Eigen::VectorXd loco_ome_filter_para_;

    double vel_x_dead_zone_threshold_;
    double vel_y_dead_zone_threshold_;
    double vel_yaw_dead_zone_threshold_;

    double joystick_vel_x_max_;
    double joystick_vel_y_max_;
    double joystick_vel_yaw_max_;

    DesiredCommand<double> des_raw_command_;
};

#endif // COMMAND_INTERFACE_HPP
