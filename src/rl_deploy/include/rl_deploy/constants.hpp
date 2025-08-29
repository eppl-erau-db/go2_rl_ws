/**
 * @file constants.hpp
 * @brief Contains constants needed for low-level deployment.
 * @author Gabriel Rodriguez
 */

#pragma once
#include <array>

namespace rl_deploy {

// Joint targets and tolerance
constexpr std::array<double, 12> StandPos = {
    0.0, 1.1, -1.8,
    0.0, 1.1, -1.8,
    0.0, 1.1, -1.8,
    0.0, 1.1, -1.8,
};

constexpr std::array<double, 12> SitPos = {
    -0.1, 1.1, -2.0,
    -0.1, 1.1, -2.0,
    -0.1, 1.1, -2.6,
    -0.1, 1.1, -2.6,
};

constexpr double position_tolerance = 0.3;

// Low-level control gains
constexpr double kp_stand = 50.0;
constexpr double kd_stand = 5.0;
constexpr double kp_sit   = 30.0;
constexpr double kd_sit   = 10.0;
constexpr double kp_loco  = 20.0;
constexpr double kd_loco  = 0.5;

} // namespace rl_deploy
