#include "command/command_interface.hpp"

#include <cmath>
#include <stdexcept>

#include "common/glog_sink.hpp"
#include "common/helpers.hpp"
#include "common/yaml_helper.hpp"
#include "hardware/hardware_interface.hpp"
#include "state_machine/fsm_data.hpp"

CommandInterface::CommandInterface(const std::string &config_path, std::shared_ptr<HardwareInterface> hw_interface_ptr, std::shared_ptr<FSMData> fsm_data_ptr)
    : hw_interface_ptr_(hw_interface_ptr), fsm_data_ptr_(fsm_data_ptr)
{
    if (!hw_interface_ptr_)
    {
        throw std::invalid_argument("HardwareInterface pointer cannot be null");
    }
    fsm_data_ptr_->desired_command_ptr->state_id = StateID::PASSIVE;
    read_cmd_config(config_path);
}

void CommandInterface::update()
{
    // set state id
    auto joystick_data_ptr = hw_interface_ptr_->get_joy_state();
    auto joy_online = hw_interface_ptr_->get_joy_online();
    auto selected_state = fsm_data_ptr_->desired_command_ptr->state_id;
    auto selected_motion = fsm_data_ptr_->desired_command_ptr->motion_id;
    auto cur_state = fsm_data_ptr_->desired_command_ptr->state_id;
    if (joystick_data_ptr->LB == 1)
    {
        if (joystick_data_ptr->A == 1)
        {
            selected_state = StateID::PASSIVE;
        }
        else if (joystick_data_ptr->B == 1)
        {
            selected_state = StateID::DAMPER;
        }
        else if (joystick_data_ptr->X == 1)
        {
            selected_state = StateID::RECOVERY_STAND;
        }
        else if (joystick_data_ptr->Y == 1)
        {
            selected_state = StateID::RL_LOCOMOTION;
        }
    }

    if (cur_state == StateID::RL_LOCOMOTION && joystick_data_ptr->RB == 1 && joystick_data_ptr->X == 1)
    {
        selected_motion = 0;
        selected_state = StateID::RL_TRACKING;
    }

    // One-button emergency stop --> damper
    if (joystick_data_ptr->LB == 1 && joystick_data_ptr->RB == 1)
    {
        selected_state = StateID::DAMPER;
    }

    // if desired mode is changed and desired mode is locomotion
    // then set vel cmd to zero
    const bool state_changed = selected_state != fsm_data_ptr_->desired_command_ptr->state_id;
    if (state_changed && (selected_state == StateID::RL_LOCOMOTION)) {
        reset_desired_command();
    }

    fsm_data_ptr_->desired_command_ptr->state_id = selected_state;
    fsm_data_ptr_->desired_command_ptr->motion_id = selected_motion;
    if (state_changed) {
        print_command();
    }

    // update cmd from joysticks
    if(fsm_data_ptr_->interface_parameter_ptr_->joystick_enable)
    {
        if (joy_online) {
            map_joystick_to_raw_cmd();
            clamp_command();
            filter_command();
        } else {
            map_topic_to_raw_cmd();
            clamp_command();
            filter_command();
        }
    }
}

void CommandInterface::print_command()
{
    LOG(INFO) << "Desired Raw Command:";
    LOG(INFO) << "State ID: " << fsm_data_ptr_->desired_command_ptr->state_id;
    LOG(INFO) << "Motion ID: " << fsm_data_ptr_->desired_command_ptr->motion_id;
}

void CommandInterface::reset_desired_command()
{
    fsm_data_ptr_->desired_command_ptr->base_pos.setZero();
    fsm_data_ptr_->desired_command_ptr->base_rpy.setZero();
    fsm_data_ptr_->desired_command_ptr->base_lin_vel.setZero();
    fsm_data_ptr_->desired_command_ptr->base_ang_vel.setZero();
    fsm_data_ptr_->desired_command_ptr->head_rpy.setZero();
    fsm_data_ptr_->desired_command_ptr->head_duration.setOnes();
}

void CommandInterface::modify_cmd_for_safety()
{
    const auto previous_state = fsm_data_ptr_->desired_command_ptr->state_id;
    if (previous_state == StateID::DAMPER)
    {
        return;
    }

    LOG(INFO) << "Modifying command for safety...";
    LOG(INFO) << "Previous state ID: " << previous_state;

    fsm_data_ptr_->desired_command_ptr->state_id = StateID::DAMPER;

    LOG(INFO) << "New state ID: " << fsm_data_ptr_->desired_command_ptr->state_id;
    LOG(INFO) << "Command modification completed.";
}

void CommandInterface::read_cmd_config(const std::string &config_path)
{
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_path)[fsm_data_ptr_->robot_name];
    }
    catch (YAML::BadFile &e)
    {
        LOG(ERROR) << "The file '" << config_path << "' does not exist";
        return;
    }
    read_vector_from_yaml(config["pos_des_min"], pos_des_min_);
    read_vector_from_yaml(config["pos_des_max"], pos_des_max_);

    read_vector_from_yaml(config["vel_des_min"], vel_des_min_);
    read_vector_from_yaml(config["vel_des_max"], vel_des_max_);

    read_vector_from_yaml(config["ori_des_min"], ori_des_min_);
    read_vector_from_yaml(config["ori_des_max"], ori_des_max_);

    read_vector_from_yaml(config["ome_des_min"], ome_des_min_);
    read_vector_from_yaml(config["ome_des_max"], ome_des_max_);

    read_vector_from_yaml(config["loco_pos_filter_para"], loco_pos_filter_para_);
    read_vector_from_yaml(config["loco_vel_filter_para"], loco_vel_filter_para_);
    read_vector_from_yaml(config["loco_ori_filter_para"], loco_ori_filter_para_);
    read_vector_from_yaml(config["loco_ome_filter_para"], loco_ome_filter_para_);

    auto dead_zone_config = config["command_dead_zone"];
    vel_x_dead_zone_threshold_ = dead_zone_config["vel_x_dead_zone_threshold"].as<double>();
    vel_y_dead_zone_threshold_ = dead_zone_config["vel_y_dead_zone_threshold"].as<double>();
    vel_yaw_dead_zone_threshold_ = dead_zone_config["vel_yaw_dead_zone_threshold"].as<double>();

    auto command_max_config = config["joystick_command_max"];
    joystick_vel_x_max_ = command_max_config["joystick_vel_x_max"].as<double>();
    joystick_vel_y_max_ = command_max_config["joystick_vel_y_max"].as<double>();
    joystick_vel_yaw_max_ = command_max_config["joystick_vel_yaw_max"].as<double>();

}

void CommandInterface::map_joystick_to_raw_cmd()
{
    auto joystick_data_ptr = hw_interface_ptr_->get_joy_state();
    if (fsm_data_ptr_->desired_command_ptr->state_id == StateID::RL_LOCOMOTION)
    {
        des_raw_command_.base_pos[0] = 0.0;
        des_raw_command_.base_pos[1] = 0.0;
        des_raw_command_.base_pos[2] = 0.0;

        des_raw_command_.base_lin_vel[0] = joystick_data_ptr->ly * joystick_vel_x_max_; // vx
        des_raw_command_.base_lin_vel[1] = joystick_data_ptr->lx * joystick_vel_y_max_; // vy
        des_raw_command_.base_lin_vel[2] = 0.0;

        des_raw_command_.base_rpy[0] = 0.0; // roll
        des_raw_command_.base_rpy[1] = 0.0; // pitch
        des_raw_command_.base_rpy[2] = 0.0; // yaw

        des_raw_command_.base_ang_vel[0] = 0.0; // vroll
        des_raw_command_.base_ang_vel[1] = 0.0; // vpitch
        des_raw_command_.base_ang_vel[2] = joystick_data_ptr->rx * joystick_vel_yaw_max_; // vyaw
    }
}

void CommandInterface::map_topic_to_raw_cmd()
{
    for (size_t i = 0; i < 3; i++) {
        des_raw_command_.base_pos[i] = fsm_data_ptr_->desired_command_ptr->raw_base_pos[i];
        des_raw_command_.base_lin_vel[i] = fsm_data_ptr_->desired_command_ptr->raw_base_lin_vel[i];
        des_raw_command_.base_rpy[i] = fsm_data_ptr_->desired_command_ptr->raw_base_rpy[i];
        des_raw_command_.base_ang_vel[i] = fsm_data_ptr_->desired_command_ptr->raw_base_ang_vel[i];
    }
}

void CommandInterface::filter_command()
{
    apply_velocity_dead_zone();

    for (size_t i = 0; i < 3; i++) {
        fsm_data_ptr_->desired_command_ptr->base_pos[i] = first_order_filter(fsm_data_ptr_->desired_command_ptr->base_pos[i],
                                            des_raw_command_.base_pos[i],
                                            loco_pos_filter_para_[i]);
        fsm_data_ptr_->desired_command_ptr->base_lin_vel[i] = first_order_filter(fsm_data_ptr_->desired_command_ptr->base_lin_vel[i],
                                            des_raw_command_.base_lin_vel[i],
                                            loco_vel_filter_para_[i]);
        fsm_data_ptr_->desired_command_ptr->base_rpy[i] = first_order_filter(fsm_data_ptr_->desired_command_ptr->base_rpy[i],
                                            des_raw_command_.base_rpy[i],
                                            loco_ori_filter_para_[i]);
        fsm_data_ptr_->desired_command_ptr->base_ang_vel[i] = first_order_filter(fsm_data_ptr_->desired_command_ptr->base_ang_vel[i],
                                            des_raw_command_.base_ang_vel[i],
                                            loco_ome_filter_para_[i]);
    }

    auto joy_online = hw_interface_ptr_->get_joy_online();
    if (joy_online){
        fsm_data_ptr_->desired_command_ptr->raw_base_lin_vel[0] = des_raw_command_.base_lin_vel[0];
        fsm_data_ptr_->desired_command_ptr->raw_base_lin_vel[1] = des_raw_command_.base_lin_vel[1];
        fsm_data_ptr_->desired_command_ptr->raw_base_lin_vel[2] = 0.0;
        fsm_data_ptr_->desired_command_ptr->raw_base_ang_vel[0] = 0.0;
        fsm_data_ptr_->desired_command_ptr->raw_base_ang_vel[1] = 0.0;
        fsm_data_ptr_->desired_command_ptr->raw_base_ang_vel[2] = des_raw_command_.base_ang_vel[2];
    }
}

void CommandInterface::clamp_command()
{
    for (size_t i = 0; i < 3; i++) {
        des_raw_command_.base_pos[i] = clamp(des_raw_command_.base_pos[i],
                                            pos_des_min_[i],
                                            pos_des_max_[i]);
        des_raw_command_.base_lin_vel[i] = clamp(des_raw_command_.base_lin_vel[i],
                                            vel_des_min_[i],
                                            vel_des_max_[i]);
        des_raw_command_.base_rpy[i] = clamp(des_raw_command_.base_rpy[i],
                                            ori_des_min_[i],
                                            ori_des_max_[i]);
        des_raw_command_.base_ang_vel[i] = clamp(des_raw_command_.base_ang_vel[i],
                                            ome_des_min_[i],
                                            ome_des_max_[i]);
    }
}

void CommandInterface::apply_velocity_dead_zone() {
    if (std::abs(des_raw_command_.base_lin_vel[0]) < vel_x_dead_zone_threshold_)
    {
        des_raw_command_.base_lin_vel[0] = 0.0;
    }
    if (std::abs(des_raw_command_.base_lin_vel[1]) < vel_y_dead_zone_threshold_)
    {
        des_raw_command_.base_lin_vel[1] = 0.0;
    }
    if (std::abs(des_raw_command_.base_ang_vel[2]) < vel_yaw_dead_zone_threshold_)
    {
        des_raw_command_.base_ang_vel[2] = 0.0;
    }
}
