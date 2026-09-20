#ifndef CONTROL_FSM_HPP
#define CONTROL_FSM_HPP

#include <iostream>
#include <string>
#include <unordered_map>
#include <memory>
#include <vector>
#include "fsm_state.hpp"

class FSM {
   public:
    FSM() : current_state_(nullptr), next_state_(nullptr), mode_(Mode::NORMAL) {}

    void add_state(std::shared_ptr<FSMState> state) {
        states_[state->get_state_id()] = state;
    }

    void set_initial_state(const StateID& id) {
        current_state_ = states_.at(id);
        current_state_->onEnter();
        next_state_ = current_state_;
    }

    void run() {
        if (!current_state_) {
            return;
        }

        if (mode_ == Mode::NORMAL) {
            current_state_->run();
            next_state_id_ = current_state_->check_transition();
            // if next state is supported
            if (is_state_supported(next_state_id_)) {
                if (next_state_id_ != current_state_->get_state_id()) {
                    mode_ = Mode::CHANGE;
                    next_state_ = states_.at(next_state_id_);
                    // Print transition initialized info
                    print_info(1);
                }
            } else {
                if (next_state_id_ != last_next_state_id_) {
                    LOG(ERROR) << "[FSM] State: " << next_state_id_ << " is not supported";
                }
            }
            last_next_state_id_ = next_state_id_;
        } else if (mode_ == Mode::CHANGE) {
            current_state_->onExit();
            current_state_ = next_state_;
            current_state_->onEnter();
            current_state_->run();
            mode_ = Mode::NORMAL;
            // Print finalizing transition info
            print_info(2);
        }
        // Print the current state of the FSM
        print_info(0);

        // Increase the iteration counter
        iter_++;
    }

    void Stop() {
        current_state_->onExit();
    }

    void print_info(int opt) {
        switch (opt) {
            case 0:  // Normal printing case at regular intervals
                // Increment printing iteration
                print_iter_++;

                // Print at commanded frequency
                if (print_iter_ == print_num_) {
                    LOG(INFO) << "[FSM] Printing FSM Info...\n";
                    LOG(INFO) << "---------------------------------------------------------\n";
                    LOG(INFO) << "[FSM] Iteration: " << iter_ << "\n";
                    if (mode_ == Mode::NORMAL) {
                        LOG(INFO) << "[FSM] Mode: NORMAL in " << current_state_->get_state_name() << "\n";
                    } else if (mode_ == Mode::CHANGE) {
                        LOG(INFO) << "[FSM] Mode: TRANSITIONING from " << current_state_->get_state_name() << " to "
                                  << next_state_->get_state_name() << "\n";
                    }
                    // Reset iteration counter
                    print_iter_ = 0;
                }
                break;

            case 1:  // Initializing FSM State transition
                LOG(INFO) << "[FSM] Transition initialized from " << current_state_->get_state_name() << " to "
                          << next_state_->get_state_name() << "\n";
                break;

            case 2:  // Finalizing FSM State transition
                LOG(INFO) << "[FSM] Transition finalizing from " << current_state_->get_state_name() << " to "
                          << next_state_->get_state_name() << "\n";
                break;
        }
    }

    StateID get_current_state_id() {
        return current_state_->get_state_id();
    }

    bool is_state_supported(const StateID& id) {
        return states_.find(id) != states_.end();
    }

    enum class Mode { NORMAL, CHANGE };

    std::unordered_map<StateID, std::shared_ptr<FSMState>> states_;
    std::shared_ptr<FSMState> current_state_;
    std::shared_ptr<FSMState> next_state_;
    StateID next_state_id_;
    StateID last_next_state_id_;
    Mode mode_;
    // Choose how often to print info, every N iterations
    int print_num_ = 10000;  // N*(0.002s) in simulation time
    // Track the number of iterations since last info print
    int print_iter_ = 0;  // make larger than printNum to not print
    int iter_ = 0;
};

class FSMFactory {
   public:
    virtual ~FSMFactory() = default;
    virtual std::shared_ptr<FSMState> create_state(void* context, std::shared_ptr<FSMData> fsm_data_ptr,
                                                   const std::string& state_name) = 0;
    virtual std::string get_type() const = 0;
    virtual std::vector<std::string> get_supported_states(std::shared_ptr<FSMData> fsm_data_ptr) const = 0;
    virtual StateID get_initial_state() const = 0;
};

class FSMManager {
   public:
    static FSMManager& get_instance() {
        static FSMManager instance;
        return instance;
    }

    void register_factory(std::shared_ptr<FSMFactory> factory) {
        if (factory) {
            std::string type = factory->get_type();
            factories_[type] = factory;
            LOG(INFO) << "[FSMManager] Registered type: " << type;
        }
    }

    std::shared_ptr<FSM> create_FSM(const std::string& type, const std::string& version,
                                    std::shared_ptr<FSMData> fsm_data_ptr, void* context) {
        auto it = factories_.find(type);
        if (it == factories_.end()) {
            LOG(INFO) << "[FSMManager] Error: Unsupported type: " << type;
            return nullptr;
        }
        auto factory = it->second;
        auto state_names = factory->get_supported_states(fsm_data_ptr);
        if (state_names.empty()) {
            LOG(INFO) << "[FSMManager] Error: No states registered for type: " << type;
            return nullptr;
        }
        auto fsm = std::make_shared<FSM>();
        for (const auto& state_name : state_names) {
            LOG(INFO) << "[FSMManager] Supported state names: " << state_name;
            auto state = factory->create_state(context, fsm_data_ptr, state_name);
            if (state)
                fsm->add_state(state);
        }
        fsm->set_initial_state(factory->get_initial_state());
        LOG(INFO) << "[FSMManager] FSM created for type: " << type;
        return fsm;
    }

    bool is_type_supported(const std::string& type) const {
        return factories_.find(type) != factories_.end();
    }

    std::vector<std::string> get_supported_types() const {
        std::vector<std::string> types;
        for (const auto& pair : factories_)
            types.push_back(pair.first);
        return types;
    }

   private:
    FSMManager() = default;
    std::unordered_map<std::string, std::shared_ptr<FSMFactory>> factories_;
};

#define CONCATENATE_DETAIL(x, y) x##y
#define CONCATENATE(x, y) CONCATENATE_DETAIL(x, y)
#define REGISTER_FSM_FACTORY(FactoryClass, initialStateName)                                           \
    namespace {                                                                                        \
    const bool CONCATENATE(registered_fsm_factory_, __COUNTER__) = []() {                              \
        FSMManager::get_instance().register_factory(std::make_shared<FactoryClass>(initialStateName)); \
        return true;                                                                                   \
    }();                                                                                               \
    }

#endif  // CONTROL_FSM_HPP