#include "rl_deploy/mode_state_machine.hpp"

namespace rl_deploy {
Mode ModeStateMachine::update(const ButtonState& b, const StatusFlags& st) {
  // Minimal stub policy: killed > damping > walking > standing > sitting > idle
  if (b.kill) return Mode::Killed;
  if (b.soft_abort) return Mode::Damping;
  if (b.start && st.good_stand) return Mode::Walking;
  if (b.stand) return Mode::Standing;
  if (b.sit) return Mode::Sitting;
  return Mode::Idle;
}
} // namespace rl_deploy
