#include "hardware/keyboard_interface.hpp"

#include <cctype>
#include <ios>
#include <unistd.h>

#include "common/glog_sink.hpp"

namespace {
constexpr auto MOVE_HOLD = std::chrono::milliseconds(400);
constexpr auto STATE_PULSE = std::chrono::milliseconds(300);
}  // namespace

KeyboardInterface::~KeyboardInterface() {
    stop();
}

void KeyboardInterface::start() {
    is_init_.store(true);  // never block HardwareInterface::is_init()

    // key board set
    if (!isatty(STDIN_FILENO)) {
        LOG(WARNING) << "[Keyboard] stdin is not a TTY - keyboard control disabled";
        return;
    }
    tcgetattr(STDIN_FILENO, &saved_termios_);
    termios raw = saved_termios_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 1;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);

    tty_ok_ = true;
    running_.store(true);

    // read key board
    read_thread_ = std::thread(&KeyboardInterface::read_loop, this);

    LOG(INFO) << "[Keyboard] w/s/a/d move, q/e turn, "
                 "p=passive space=damper r=recovery l=locomotion  m=tracking";
}

void KeyboardInterface::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
    if (tty_ok_) {
        tcsetattr(STDIN_FILENO, TCSANOW, &saved_termios_);
    }
}

void KeyboardInterface::read_loop() {
    char buf[32];
    while (running_.load()) {
        const ssize_t n = ::read(STDIN_FILENO, buf, sizeof(buf));
        for (ssize_t i = 0; i < n; ++i) {
            const unsigned char c = static_cast<unsigned char>(buf[i]);
            // Prevent undefined key presses.
            switch (esc_state_) {
                case EscState::kNone:
                    if (c == 0x1B) {
                        esc_state_ = EscState::kEsc;
                    } else {
                        handle_key(static_cast<char>(std::tolower(c)));
                    }
                    break;
                case EscState::kEsc:
                    esc_state_ = (c == '[') ? EscState::kCsi : EscState::kNone;
                    break;
                case EscState::kCsi:
                    if (c >= 0x40 && c <= 0x7E) {
                        esc_state_ = EscState::kNone;
                    }
                    break;
            }
        }
    }
}

void KeyboardInterface::handle_key(char key) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const auto now = clock::now();
    switch (key) {
        case 'w': target_ly_ = 1.0F;  ts_ly_ = now; break;
        case 's': target_ly_ = -1.0F; ts_ly_ = now; break;
        case 'a': target_lx_ = 1.0F;  ts_lx_ = now; break;
        case 'd': target_lx_ = -1.0F; ts_lx_ = now; break;
        case 'q': target_rx_ = 1.0F;  ts_rx_ = now; break;
        case 'e': target_rx_ = -1.0F; ts_rx_ = now; break;

        case 'p': pending_state_ = PASSIVE;        has_pending_state_ = true; pending_since_ = now; break;
        case ' ': pending_state_ = DAMPER;         has_pending_state_ = true; pending_since_ = now; break;
        case 'r': pending_state_ = RECOVERY_STAND; has_pending_state_ = true; pending_since_ = now; break;
        case 'l': pending_state_ = RL_LOCOMOTION;  has_pending_state_ = true; pending_since_ = now; break;
        case 'm': pending_state_ = RL_TRACKING;    has_pending_state_ = true; pending_since_ = now; break;

        default:
            if (std::isprint(static_cast<unsigned char>(key))) {
                LOG(INFO) << "[Keyboard] ignored unrecognized key: '" << key << "'";
            } else {
                LOG(INFO) << "[Keyboard] ignored unrecognized key: 0x" << std::hex
                          << static_cast<int>(static_cast<unsigned char>(key));
            }
            break;
    }
}

std::shared_ptr<JoystickData> KeyboardInterface::get_latest_data() {
    auto data = std::make_shared<JoystickData>();

    std::lock_guard<std::mutex> lock(data_mutex_);
    const auto now = clock::now();

    if (now - ts_lx_ < MOVE_HOLD) {
        data->lx = target_lx_;
    }
    if (now - ts_ly_ < MOVE_HOLD) {
        data->ly = target_ly_;
    }
    if (now - ts_rx_ < MOVE_HOLD) {
        data->rx = target_rx_;
    }

    if (has_pending_state_ && now - pending_since_ < STATE_PULSE) {
        switch (pending_state_) {
            case PASSIVE:        data->LB = PRESS; data->A = PRESS; break;
            case DAMPER:         data->LB = PRESS; data->B = PRESS; break;
            case RECOVERY_STAND: data->LB = PRESS; data->X = PRESS; break;
            case RL_LOCOMOTION:  data->LB = PRESS; data->Y = PRESS; break;
            case RL_TRACKING:    data->RB = PRESS; data->X = PRESS; break;
            default: break;
        }
    } else {
        has_pending_state_ = false;
    }

    return data;
}

bool KeyboardInterface::is_joy_online() {
    return true;  // keyboard is the selected input; always "connected"
}

bool KeyboardInterface::is_joy_init() {
    return is_init_.load();
}
