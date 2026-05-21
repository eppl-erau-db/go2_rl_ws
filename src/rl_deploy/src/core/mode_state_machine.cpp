#include "rl_deploy/mode_state_machine.hpp"

namespace rl_deploy {

Mode ModeStateMachine::update(const ButtonState& b, const StatusFlags& st) {
  // Priority overrides - always take effect immediately
  if (b.kill) {
    current_mode_ = Mode::Killed;
    return current_mode_;
  }
  if (b.soft_abort) {
    current_mode_ = Mode::Damping;
    return current_mode_;
  }
  if (b.emergency_sit) {
    current_mode_ = Mode::EmergencySitting;
    return current_mode_;
  }

  // Mode transition requests (on button press)
  // These latch the mode until another mode is requested
  if (b.stand) {
    current_mode_ = Mode::Standing;
  } else if (b.sit) {
    current_mode_ = Mode::Sitting;
  } else if (
      b.pedipulate
      && (current_mode_ == Mode::Standing || current_mode_ == Mode::Walking
          || current_mode_ == Mode::Pedipulation)) {
    current_mode_ = Mode::Pedipulation;
  } else if (b.start && current_mode_ == Mode::Pedipulation) {
    current_mode_ = Mode::Walking;
  } else if (b.start && st.good_stand) {
    current_mode_ = Mode::Walking;
  }

  // Mode completion transitions
  // Sitting transitions to Idle when robot has fully sat down
  if (current_mode_ == Mode::Sitting && st.good_sit) {
    current_mode_ = Mode::Idle;
  }
  
  // EmergencySitting transitions to Idle when done
  if (current_mode_ == Mode::EmergencySitting && st.good_sit) {
    current_mode_ = Mode::Idle;
  }
  
  // Note: Standing and Pedipulation stay latched until another mode is requested

  return current_mode_;
}

} // namespace rl_deploy
