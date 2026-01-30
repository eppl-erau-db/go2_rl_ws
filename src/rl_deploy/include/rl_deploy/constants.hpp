/**
 * @file constants.hpp
 * @brief Contains constants needed for low-level deployment.
 * @author Gabriel Rodriguez
 */

#pragma once
#include <array>

namespace rl_deploy {

// Joint targets and tolerance
// Order: FR(hip,thigh,calf), FL, RR, RL
// Values measured from actual robot via lowstate_monitor.py
constexpr std::array<double, 12> StandPos = {
    0.0, 0.8, -1.5,   // FR - measured: thigh≈0.79-0.81, calf≈-1.52
    0.0, 0.8, -1.5,   // FL
    0.0, 0.8, -1.5,   // RR
    0.0, 0.8, -1.5,   // RL
};

// Exact per-leg values measured from robot laying naturally
// Note: Rear hips are splayed outward (RR negative, RL positive)
constexpr std::array<double, 12> SitPos = {
   -0.046, 1.262, -2.784,   // FR
    0.048, 1.257, -2.794,   // FL
   -0.341, 1.278, -2.810,   // RR - rear hip splayed outward
    0.317, 1.265, -2.788,   // RL - rear hip splayed outward
};

constexpr double position_tolerance = 0.3;

// Low-level control gains
constexpr double kp_stand = 50.0;   // Original value
constexpr double kd_stand = 5.0;    // Original value (provides damping to prevent overshoot)
constexpr double kp_sit   = 30.0;
constexpr double kd_sit   = 10.0;
constexpr double kp_loco  = 20.0;
constexpr double kd_loco  = 0.5;
constexpr double kp_damping = 5.0;   // Low stiffness for manual manipulation
constexpr double kd_damping = 6.0;   // Higher damping for gentler descent

} // namespace rl_deploy
