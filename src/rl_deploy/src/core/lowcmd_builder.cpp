/**
 * @file lowcmd_builder.cpp
 * @brief Implementation of motor command building for different control modes.
 * @author Gabriel Rodriguez
 */

#include "rl_deploy/lowcmd_builder.hpp"
#include "rl_deploy/constants.hpp"
#include "motor_crc.h"
#include <algorithm>
#include <cmath>

namespace rl_deploy {

// Joint position limits for Go2 (radians)
// These are conservative software limits to prevent damage
namespace limits {
  // Hip (abduction/adduction) limits
  constexpr double hip_min = -0.8;
  constexpr double hip_max = 0.8;
  
  // Thigh (flexion/extension) limits
  constexpr double thigh_min = -0.5;
  constexpr double thigh_max = 2.5;
  
  // Calf (knee) limits
  constexpr double calf_min = -2.7;
  constexpr double calf_max = -0.5;
}

// Clamp joint position to safe limits
static double clamp_joint(size_t joint_idx, double value) {
  // Joint indices: FR(0,1,2), FL(3,4,5), RR(6,7,8), RL(9,10,11)
  // Pattern: hip, thigh, calf for each leg
  size_t joint_type = joint_idx % 3;
  
  switch (joint_type) {
    case 0: // Hip
      return std::clamp(value, limits::hip_min, limits::hip_max);
    case 1: // Thigh
      return std::clamp(value, limits::thigh_min, limits::thigh_max);
    case 2: // Calf
      return std::clamp(value, limits::calf_min, limits::calf_max);
    default:
      return value;
  }
}

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
static unitree_go::msg::LowCmd make_stand_cmd(const unitree_go::msg::LowState& state) {
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
static unitree_go::msg::LowCmd make_sit_cmd(const unitree_go::msg::LowState& state) {
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

// Build command for Walking mode - use RL policy actions
static unitree_go::msg::LowCmd make_walk_cmd(
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
    
    // Apply safety limits
    target_q = clamp_joint(i, target_q);
    
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
      return make_stand_cmd(latest_state);
    
    case Mode::Sitting:
      return make_sit_cmd(latest_state);
    
    case Mode::Walking:
      return make_walk_cmd(latest_state, actions);
    
    case Mode::Damping:
      return make_damping_cmd(latest_state);
    
    case Mode::Killed:
      return make_killed_cmd();
    
    default:
      // Unknown mode - return idle for safety
      return make_idle_cmd();
  }
}

} // namespace rl_deploy
