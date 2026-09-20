#include "hardware/joy_subscriber.hpp"

#include <cstdint>
#include <functional>
#include <iomanip>
#include <sstream>

#include "common/glog_sink.hpp"

JoySubscriber::JoySubscriber(rclcpp::Node::SharedPtr node, const std::string& topic_name) : node_(node) {
    subscription_ = node_->create_subscription<sensor_msgs::msg::Joy>(
        topic_name, 10, std::bind(&JoySubscriber::joy_callback, this, std::placeholders::_1));

    joystick_data_ptr_ = std::make_shared<JoystickData>();
}

JoySubscriber::~JoySubscriber() {
    stop();
}

void JoySubscriber::start() {}

void JoySubscriber::stop() {}

std::shared_ptr<JoystickData> JoySubscriber::get_latest_data() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return std::make_shared<JoystickData>(*joystick_data_ptr_);
}

void JoySubscriber::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    bool parsed_successfully = false;

    if (joy_type_ == "robot_remote_control") {
        parsed_successfully = warp_robot_remote_control_to_data(msg);
    } else if (joy_type_ == "game_controller") {
        parsed_successfully = warp_game_controller_to_data(msg);
    } else {
        LOG(ERROR) << "unsupported remote control type: " << joy_type_;
    }

    if (!parsed_successfully) {
        return;
    }

    joystick_data_ptr_->axes = msg->axes;
    joystick_data_ptr_->buttons = msg->buttons;
    is_joy_init_.store(true);
    is_joy_online_.store(true);
    // print_joy_msg(msg);
    // print_joy_data();
}

bool JoySubscriber::is_joy_init() {
    return is_joy_init_.load();
}

bool JoySubscriber::is_joy_online() {
    return is_joy_online_.load();
}

void JoySubscriber::set_joy_type(const std::string& joy_type) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    joy_type_ = joy_type;
}

bool JoySubscriber::warp_robot_remote_control_to_data(const sensor_msgs::msg::Joy::SharedPtr msg) {
    //   axes[0..4] = lx, ly, rx, ry, wave  (range [-1, 1]);
    //                only lx / ly / ry carry data, rx and wave are always 0.
    //   buttons[0] = key_map bitmask (int32).
    if (msg->axes.size() < 4) {
        LOG(ERROR) << "**[Warning] axes dimension is insufficient, current size = " << msg->axes.size()
                   << ", expected at least 4!**";
        return false;
    }
    if (msg->buttons.empty()) {
        LOG(ERROR) << "**[Warning] buttons dimension is insufficient, current size = " << msg->buttons.size()
                   << ", expected at least 1!**";
        return false;
    }

    const int key_map = msg->buttons[0];

    joystick_data_ptr_->lx = msg->axes[1];
    joystick_data_ptr_->ly = msg->axes[0];
    joystick_data_ptr_->ry = msg->axes[2];  // ry (axes[2]) is always 0.
    joystick_data_ptr_->rx = msg->axes[3];

    joystick_data_ptr_->LB = parse_key_map_button(key_map, 1U << 9);                // S1
    joystick_data_ptr_->RB = parse_key_map_button(key_map, 1U << 2);                // M2
    joystick_data_ptr_->A = parse_key_map_button(key_map, (1U << 6) | (1U << 13));  // R1
    joystick_data_ptr_->B = parse_key_map_button(key_map, (1U << 5) | (1U << 10));  // M6
    joystick_data_ptr_->X = parse_key_map_button(key_map, (1U << 7) | (1U << 11));  // R2
    joystick_data_ptr_->Y = parse_key_map_button(key_map, (1U << 4) | (1U << 12));  // R3

    // L1, L2, S2, S3, S4, M1, M3 are not part of any FSM combo on this remote.
    joystick_data_ptr_->LT = RELEASE;
    joystick_data_ptr_->RT = RELEASE;
    joystick_data_ptr_->BACK = RELEASE;
    joystick_data_ptr_->START = RELEASE;
    joystick_data_ptr_->F1 = RELEASE;
    joystick_data_ptr_->F2 = RELEASE;
    joystick_data_ptr_->UP = RELEASE;
    joystick_data_ptr_->DOWN = RELEASE;
    joystick_data_ptr_->LEFT = RELEASE;
    joystick_data_ptr_->RIGHT = RELEASE;
    return true;
}

bool JoySubscriber::warp_game_controller_to_data(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->axes.size() < 6) {
        LOG(ERROR) << "**[Warning] axes dimension is insufficient, current size = " << msg->axes.size()
                   << ", expected at least 6!**";
        return false;
    }
    if (msg->buttons.size() < 21) {
        LOG(ERROR) << "**[Warning] buttons dimension is insufficient, current size = " << msg->buttons.size()
                   << ", expected at least 21!**";
        return false;
    }
    joystick_data_ptr_->lx = msg->axes[0];
    joystick_data_ptr_->ly = msg->axes[1];
    joystick_data_ptr_->rx = msg->axes[2];
    joystick_data_ptr_->ry = msg->axes[3];

    joystick_data_ptr_->LT = parse_axis(msg->axes[4]);  // 0: released, -1: pressed
    joystick_data_ptr_->RT = parse_axis(msg->axes[5]);  // 0: released, -1: pressed

    joystick_data_ptr_->A = parse_button(msg->buttons[0]);
    joystick_data_ptr_->B = parse_button(msg->buttons[1]);
    joystick_data_ptr_->X = parse_button(msg->buttons[2]);
    joystick_data_ptr_->Y = parse_button(msg->buttons[3]);

    joystick_data_ptr_->UP = parse_button(msg->buttons[11]);
    joystick_data_ptr_->DOWN = parse_button(msg->buttons[12]);
    joystick_data_ptr_->LEFT = parse_button(msg->buttons[13]);
    joystick_data_ptr_->RIGHT = parse_button(msg->buttons[14]);

    joystick_data_ptr_->BACK = parse_button(msg->buttons[4]);
    joystick_data_ptr_->START = parse_button(msg->buttons[6]);

    joystick_data_ptr_->LB = parse_button(msg->buttons[9]);
    joystick_data_ptr_->RB = parse_button(msg->buttons[10]);
    return true;
}

ButtonState JoySubscriber::parse_button(float in) {
    switch ((int)in) {
        case 1:
            return ButtonState::PRESS;
        case 0:
            return ButtonState::RELEASE;
        default:
            printf("[Joy] Press switch returned bad value %f\n", in);
            return ButtonState::RELEASE;
    }
}

ButtonState JoySubscriber::parse_axis(float in) {
    if (in < -0.9) {
        return ButtonState::PRESS;
    } else {
        return ButtonState::RELEASE;
    }
}

ButtonState JoySubscriber::parse_key_map_button(int key_map, std::uint32_t mask) {
    const std::uint32_t bits = static_cast<std::uint32_t>(key_map);
    return ((bits & mask) == mask) ? ButtonState::PRESS : ButtonState::RELEASE;
}

void JoySubscriber::print_joy_msg(const sensor_msgs::msg::Joy::SharedPtr msg) {
    std::ostringstream axes_stream;
    axes_stream << std::fixed << std::setprecision(3);
    for (std::size_t i = 0; i < msg->axes.size(); ++i) {
        if (i > 0) {
            axes_stream << ", ";
        }
        axes_stream << '[' << i << "]=" << msg->axes[i];
    }

    std::ostringstream buttons_stream;
    for (std::size_t i = 0; i < msg->buttons.size(); ++i) {
        if (i > 0) {
            buttons_stream << ", ";
        }
        buttons_stream << '[' << i << "]=" << msg->buttons[i];
    }

    LOG(INFO) << "┌──────────────────────────────";
    LOG(INFO) << "│ Joy Message";
    LOG(INFO) << "├─ Axes (count: " << msg->axes.size() << ')';
    LOG(INFO) << "│   " << axes_stream.str();
    LOG(INFO) << "├─ Buttons (count: " << msg->buttons.size() << ')';
    LOG(INFO) << "│   " << buttons_stream.str();
    LOG(INFO) << "└──────────────────────────────";
}

void JoySubscriber::print_joy_data() {
    const auto state_name = [](ButtonState state) { return state == ButtonState::PRESS ? "PRESS" : "RELEASE"; };

    LOG(INFO) << "┌──────────────────────────────";
    LOG(INFO) << "│ Joystick State";
    LOG(INFO) << "├─ Analog Sticks (lx, ly, rx, ry)";
    LOG(INFO) << "│   " << std::fixed << std::setprecision(3)
              << std::setw(8) << joystick_data_ptr_->lx << ' '
              << std::setw(8) << joystick_data_ptr_->ly << ' '
              << std::setw(8) << joystick_data_ptr_->rx << ' '
              << std::setw(8) << joystick_data_ptr_->ry;

    LOG(INFO) << "├─ Shoulder Buttons";
    LOG(INFO) << "│   LT: " << std::setw(7) << std::left << state_name(joystick_data_ptr_->LT)
              << " LB: " << std::setw(7) << state_name(joystick_data_ptr_->LB)
              << " RT: " << std::setw(7) << state_name(joystick_data_ptr_->RT)
              << " RB: " << state_name(joystick_data_ptr_->RB);

    LOG(INFO) << "├─ Direction Pad";
    LOG(INFO) << "│   UP: " << std::setw(7) << std::left << state_name(joystick_data_ptr_->UP)
              << " DOWN: " << std::setw(7) << state_name(joystick_data_ptr_->DOWN)
              << " LEFT: " << std::setw(7) << state_name(joystick_data_ptr_->LEFT)
              << " RIGHT: " << state_name(joystick_data_ptr_->RIGHT);

    LOG(INFO) << "├─ Action Buttons";
    LOG(INFO) << "│   A: " << std::setw(7) << std::left << state_name(joystick_data_ptr_->A)
              << " B: " << std::setw(7) << state_name(joystick_data_ptr_->B)
              << " X: " << std::setw(7) << state_name(joystick_data_ptr_->X)
              << " Y: " << state_name(joystick_data_ptr_->Y);

    LOG(INFO) << "├─ Function Buttons";
    LOG(INFO) << "│   BACK: " << std::setw(7) << std::left << state_name(joystick_data_ptr_->BACK)
              << " START: " << std::setw(7) << state_name(joystick_data_ptr_->START)
              << " F1: " << std::setw(7) << state_name(joystick_data_ptr_->F1)
              << " F2: " << state_name(joystick_data_ptr_->F2);
    LOG(INFO) << "└──────────────────────────────";
}
