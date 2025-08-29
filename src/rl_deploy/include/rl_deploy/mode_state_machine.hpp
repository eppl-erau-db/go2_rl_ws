/**
 * @file mode_state_machine.hpp
 * @brief 
 * @author Gabriel Rodriguez
 */

#pragma once
#include "rl_deploy/types.hpp"

namespace rl_deploy {

class ModeStateMachine {
    public:
        Mode update(const ButtonState& buttons, const StatusFlags& status);
    private:
        Mode last_{Mode::Idle}; // optional book-keeping
};

} // namespace rl_deploy
