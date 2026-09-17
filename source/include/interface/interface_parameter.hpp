#ifndef INTERFACE_PARAMETER_HPP
#define INTERFACE_PARAMETER_HPP

#include <mutex>
#include <cstdint>

namespace interface {

struct InterfaceParameter {
    uint32_t action_id;
    uint32_t schedule = 0;  // 0, 1, ..., 100
    uint32_t result = 100;  // 100: init, 1: success, 0: fail
    int state;
    float vx;
    float vy;
    float vyaw;
    int speed = -1;
    int last_speed = -1;
    std::mutex mtx;
    bool joystick_enable = true;

    float loco_pos_filter_para[3] = {0.0f, 0.0f, 0.0f};
    float loco_vel_filter_para[3] = {0.0f, 0.0f, 0.0f};
    float loco_ori_filter_para[3] = {0.0f, 0.0f, 0.0f};
    float loco_ome_filter_para[3] = {0.0f, 0.0f, 0.0f};
    float loco_pos_filter_para_default[3] = {0.0f, 0.0f, 0.0f};
    float loco_vel_filter_para_default[3] = {0.0f, 0.0f, 0.0f};
    float loco_ori_filter_para_default[3] = {0.0f, 0.0f, 0.0f};
    float loco_ome_filter_para_default[3] = {0.0f, 0.0f, 0.0f};
    bool loco_filter_dirty = false;

    float vel_x_dead_zone_threshold = 0.0f;
    float vel_y_dead_zone_threshold = 0.0f;
    float vel_yaw_dead_zone_threshold = 0.0f;
    float vel_x_dead_zone_threshold_default = 0.0f;
    float vel_y_dead_zone_threshold_default = 0.0f;
    float vel_yaw_dead_zone_threshold_default = 0.0f;
    bool dead_zone_dirty = false;

    float deceleration_rate = 0.0f;
    float deceleration_rate_default = 0.0f;
    bool deceleration_rate_dirty = false;

    void clear() {
        action_id = 0;
        schedule = 0;
        result = 0;
        state = 0;
    }
};
}

#endif // INTERFACE_PARAMETER_HPP
