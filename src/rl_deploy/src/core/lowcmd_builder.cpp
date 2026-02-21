/**
 * @file lowcmd_builder.cpp
 * @brief Implementation of motor command building for different control modes.
 *
 * Optimized: uses a single parameterized builder instead of per-mode functions,
 * avoiding redundant init_cmd() calls and reducing code duplication.
 *
 * @author Gabriel Rodriguez
 */

#include "rl_deploy/lowcmd_builder.hpp"
#include "rl_deploy/constants.hpp"
#include "motor_crc.h"

namespace rl_deploy {

// Initialize a LowCmd message with safe defaults (protocol header + all motors disabled).
static unitree_go::msg::LowCmd init_cmd() {
  unitree_go::msg::LowCmd cmd{};

  // Set head bytes for Go2 protocol.
  cmd.head[0] = 0xFE;
  cmd.head[1] = 0xEF;

  // Initialize all 20 motor commands (12 leg + 8 unused) to disabled.
  for (size_t i = 0; i < 20; ++i) {
    cmd.motor_cmd[i].mode = 0x00;
    cmd.motor_cmd[i].q    = 0.0f;
    cmd.motor_cmd[i].dq   = 0.0f;
    cmd.motor_cmd[i].kp   = 0.0f;
    cmd.motor_cmd[i].kd   = 0.0f;
    cmd.motor_cmd[i].tau  = 0.0f;
  }

  return cmd;
}

// Fill the 12 leg motor commands with uniform gains and per-joint targets.
static void fill_leg_motors(
    unitree_go::msg::LowCmd& cmd,
    const float* targets,
    float kp, float kd)
{
  for (size_t i = 0; i < 12; ++i) {
    cmd.motor_cmd[i].mode = 0x01;
    cmd.motor_cmd[i].q    = targets[i];
    cmd.motor_cmd[i].dq   = 0.0f;
    cmd.motor_cmd[i].kp   = kp;
    cmd.motor_cmd[i].kd   = kd;
    cmd.motor_cmd[i].tau  = 0.0f;
  }
}

// Public API.

unitree_go::msg::LowCmd make_init_cmd() {
  auto cmd = init_cmd();
  get_crc(cmd);
  return cmd;
}

unitree_go::msg::LowCmd make_cmd_for_mode(
    Mode mode,
    const unitree_go::msg::LowState& latest_state,
    const std::vector<float>& actions)
{
  auto cmd = init_cmd();

  switch (mode) {
    case Mode::Idle:
    case Mode::Killed:
      // All motors stay at mode 0x00 (disabled) from init_cmd().
      break;

    case Mode::Standing: {
      float targets[12];
      for (size_t i = 0; i < 12; ++i)
        targets[i] = static_cast<float>(StandPos[i]);
      fill_leg_motors(cmd, targets,
                      static_cast<float>(kp_stand),
                      static_cast<float>(kd_stand));
      break;
    }

    case Mode::Sitting: {
      float targets[12];
      for (size_t i = 0; i < 12; ++i)
        targets[i] = static_cast<float>(SitPos[i]);
      fill_leg_motors(cmd, targets,
                      static_cast<float>(kp_sit),
                      static_cast<float>(kd_sit));
      break;
    }

    case Mode::EmergencySitting: {
      float targets[12];
      for (size_t i = 0; i < 12; ++i)
        targets[i] = static_cast<float>(SitPos[i]);
      fill_leg_motors(cmd, targets,
                      static_cast<float>(kp_emergency_sit),
                      static_cast<float>(kd_emergency_sit));
      break;
    }

    case Mode::Walking: {
      float targets[12];
      for (size_t i = 0; i < 12; ++i) {
        targets[i] = (i < actions.size())
            ? actions[i]
            : latest_state.motor_state[i].q;
      }
      fill_leg_motors(cmd, targets,
                      static_cast<float>(kp_loco),
                      static_cast<float>(kd_loco));
      break;
    }

    case Mode::Damping: {
      float targets[12];
      for (size_t i = 0; i < 12; ++i)
        targets[i] = latest_state.motor_state[i].q;
      fill_leg_motors(cmd, targets,
                      static_cast<float>(kp_damping),
                      static_cast<float>(kd_damping));
      break;
    }

    default:
      // Unknown mode — stay disabled (safe).
      break;
  }

  get_crc(cmd);
  return cmd;
}

} // namespace rl_deploy
