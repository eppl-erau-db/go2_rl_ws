/**
 * @file mode_state_machine.hpp
 * @brief Stateful FSM for robot mode control with latching behavior
 * @author Gabriel Rodriguez
 */

#pragma once
#include "rl_deploy/types.hpp"

namespace rl_deploy {

class ModeStateMachine {
    public:
        Mode update(const ButtonState& buttons, const StatusFlags& status);
    private:
        Mode current_mode_{Mode::Idle};  // Current latched mode
};

} // namespace rl_deploy
