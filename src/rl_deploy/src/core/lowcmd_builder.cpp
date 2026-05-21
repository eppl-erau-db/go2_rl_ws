#include "rl_deploy/lowcmd_builder.hpp"
#include "rl_deploy/constants.hpp"
#include "motor_crc.h"
#include <algorithm>

namespace rl_deploy {

// Initialize a LowCmd message with safe defaults
static unitree_go::msg::LowCmd init_cmd() {
  unitree_go::msg::LowCmd cmd{};
  
  // Set head bytes for Go2 protocol
  cmd.head[0] = 0xFE;
  cmd.head[1] = 0xEF;
  
  // Initialize all 20 motor commands (12 leg + others)
  for (size_t i = 0; i < 20; ++i) {
    cmd.motor_cmd[i].mode = 0x00;  // Disabled by default
    cmd.motor_cmd[i].q = 0.0f;
    cmd.motor_cmd[i].dq = 0.0f;
    cmd.motor_cmd[i].kp = 0.0f;
    cmd.motor_cmd[i].kd = 0.0f;
    cmd.motor_cmd[i].tau = 0.0f;
  }
  
  return cmd;
}

// Build command for Idle mode - motors disabled, no torque
static unitree_go::msg::LowCmd make_idle_cmd() {
  auto cmd = init_cmd();
  // All motors stay at mode 0x00 (disabled)
  get_crc(cmd);
  return cmd;
}

// Build command for Standing mode - move to stand position
static unitree_go::msg::LowCmd make_stand_cmd() {
  auto cmd = init_cmd();
  
  for (size_t i = 0; i < 12; ++i) {
    cmd.motor_cmd[i].mode = 0x01;  // Position control mode
    cmd.motor_cmd[i].q = static_cast<float>(StandPos[i]);
    cmd.motor_cmd[i].dq = 0.0f;
    cmd.motor_cmd[i].kp = static_cast<float>(kp_stand);
    cmd.motor_cmd[i].kd = static_cast<float>(kd_stand);
    cmd.motor_cmd[i].tau = 0.0f;
  }
  
  get_crc(cmd);
  return cmd;
}

// Build command for Sitting mode - move to sit position
static unitree_go::msg::LowCmd make_sit_cmd() {
  auto cmd = init_cmd();
  
  for (size_t i = 0; i < 12; ++i) {
    cmd.motor_cmd[i].mode = 0x01;  // Position control mode
    cmd.motor_cmd[i].q = static_cast<float>(SitPos[i]);
    cmd.motor_cmd[i].dq = 0.0f;
    cmd.motor_cmd[i].kp = static_cast<float>(kp_sit);
    cmd.motor_cmd[i].kd = static_cast<float>(kd_sit);
    cmd.motor_cmd[i].tau = 0.0f;
  }
  
  get_crc(cmd);
  return cmd;
}

// Build command for Emergency Sitting mode - slower, more damped descent for safety recovery
static unitree_go::msg::LowCmd make_emergency_sit_cmd() {
  auto cmd = init_cmd();
  
  for (size_t i = 0; i < 12; ++i) {
    cmd.motor_cmd[i].mode = 0x01;  // Position control mode
    cmd.motor_cmd[i].q = static_cast<float>(SitPos[i]);
    cmd.motor_cmd[i].dq = 0.0f;
    cmd.motor_cmd[i].kp = static_cast<float>(kp_emergency_sit);  // Lower stiffness = slower
    cmd.motor_cmd[i].kd = static_cast<float>(kd_emergency_sit);  // Higher damping = smoother
    cmd.motor_cmd[i].tau = 0.0f;
  }
  
  get_crc(cmd);
  return cmd;
}

// Build command for policy-driven modes - use RL joint position targets.
static unitree_go::msg::LowCmd make_action_cmd(
    const unitree_go::msg::LowState& state,
    const std::vector<float>& actions) {
  auto cmd = init_cmd();
  
  // Actions should be 12 joint positions from RL policy
  // Already in Unitree joint order from rl_actions.py
  for (size_t i = 0; i < 12; ++i) {
    cmd.motor_cmd[i].mode = 0x01;  // Position control mode
    
    // Get target position from actions (or current position if actions empty)
    double target_q = (i < actions.size()) 
        ? static_cast<double>(actions[i]) 
        : state.motor_state[i].q;
    
    cmd.motor_cmd[i].q = static_cast<float>(target_q);
    cmd.motor_cmd[i].dq = 0.0f;
    cmd.motor_cmd[i].kp = static_cast<float>(kp_loco);
    cmd.motor_cmd[i].kd = static_cast<float>(kd_loco);
    cmd.motor_cmd[i].tau = 0.0f;
  }
  
  get_crc(cmd);
  return cmd;
}

// Build command for Damping mode - soft abort with damping
static unitree_go::msg::LowCmd make_damping_cmd(const unitree_go::msg::LowState& state) {
  auto cmd = init_cmd();
  
  // Damping mode: hold current position with low gains
  // This allows the robot to be manually moved while providing some resistance
  for (size_t i = 0; i < 12; ++i) {
    cmd.motor_cmd[i].mode = 0x01;  // Position control mode
    cmd.motor_cmd[i].q = state.motor_state[i].q;  // Hold current position
    cmd.motor_cmd[i].dq = 0.0f;
    cmd.motor_cmd[i].kp = static_cast<float>(kp_damping);  // Low stiffness
    cmd.motor_cmd[i].kd = static_cast<float>(kd_damping);  // Gentler descent
    cmd.motor_cmd[i].tau = 0.0f;
  }
  
  get_crc(cmd);
  return cmd;
}

// Build command for Killed mode - emergency stop, disable all motors
static unitree_go::msg::LowCmd make_killed_cmd() {
  auto cmd = init_cmd();
  
  // All motors disabled - robot will collapse
  // This is an emergency stop
  for (size_t i = 0; i < 12; ++i) {
    cmd.motor_cmd[i].mode = 0x00;  // Disabled
    cmd.motor_cmd[i].q = 0.0f;
    cmd.motor_cmd[i].dq = 0.0f;
    cmd.motor_cmd[i].kp = 0.0f;
    cmd.motor_cmd[i].kd = 0.0f;
    cmd.motor_cmd[i].tau = 0.0f;
  }
  
  get_crc(cmd);
  return cmd;
}

// Public API implementations

unitree_go::msg::LowCmd make_init_cmd() {
  return make_idle_cmd();
}

unitree_go::msg::LowCmd make_cmd_for_mode(
    Mode mode,
    const unitree_go::msg::LowState& latest_state,
    const std::vector<float>& actions) {
  
  switch (mode) {
    case Mode::Idle:
      return make_idle_cmd();
    
    case Mode::Standing:
      return make_stand_cmd();
    
    case Mode::Sitting:
      return make_sit_cmd();
    
    case Mode::EmergencySitting:
      return make_emergency_sit_cmd();
    
    case Mode::Walking:
    case Mode::Pedipulation:
      return make_action_cmd(latest_state, actions);
    
    case Mode::Damping:
      return make_damping_cmd(latest_state);
    
    case Mode::Killed:
      return make_killed_cmd();
    
    default:
      // Unknown mode - return idle for safety
      return make_idle_cmd();
  }
}

unitree_go::msg::LowCmd make_stand_transition_cmd(
    const std::array<double, 12>& start_positions,
    double alpha) {
  auto cmd = init_cmd();
  alpha = std::clamp(alpha, 0.0, 1.0);

  for (size_t i = 0; i < 12; ++i) {
    const double target =
      start_positions[i] * (1.0 - alpha) + StandPos[i] * alpha;

    cmd.motor_cmd[i].mode = 0x01;
    cmd.motor_cmd[i].q = static_cast<float>(target);
    cmd.motor_cmd[i].dq = 0.0f;
    cmd.motor_cmd[i].kp = static_cast<float>(kp_stand);
    cmd.motor_cmd[i].kd = static_cast<float>(kd_stand);
    cmd.motor_cmd[i].tau = 0.0f;
  }

  get_crc(cmd);
  return cmd;
}

} // namespace rl_deploy
