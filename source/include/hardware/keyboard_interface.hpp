#ifndef KEYBOARD_INTERFACE_HPP
#define KEYBOARD_INTERFACE_HPP

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

#include <termios.h>

#include "hardware/joy_subscriber.hpp"
#include "state_machine/common_definition.hpp"

// Movement :
//   w / s : forward / backward   (ly = +1 / -1)
//   a / d : strafe left / right  (lx = +1 / -1)
//   q / e : turn  left / right   (rx = +1 / -1)
// State switch :
//   p       : Passive          (LB + A)
//   space   : Damper           (LB + B)
//   r       : Recovery         (LB + X)
//   l       : Locomotion       (LB + Y)
//   m       : Motion tracking  (RB + X, only from Locomotion)
class KeyboardInterface {
   public:
    KeyboardInterface() = default;
    ~KeyboardInterface();

    void start();
    void stop();

    std::shared_ptr<JoystickData> get_latest_data();
    bool is_joy_online();
    bool is_joy_init();

   private:
    using clock = std::chrono::steady_clock;

    void read_loop();
    void handle_key(char key);

    enum class EscState { kNone, kEsc, kCsi };
    EscState esc_state_{EscState::kNone};

    std::thread read_thread_;
    std::atomic_bool running_{false};
    std::atomic_bool is_init_{false};
    bool tty_ok_{false};
    termios saved_termios_{};

    std::mutex data_mutex_;
    float target_lx_{0.0F};
    float target_ly_{0.0F};
    float target_rx_{0.0F};
    clock::time_point ts_lx_{};
    clock::time_point ts_ly_{};
    clock::time_point ts_rx_{};

    StateID pending_state_{PASSIVE};
    bool has_pending_state_{false};
    clock::time_point pending_since_{};
};

#endif  // KEYBOARD_INTERFACE_HPP
