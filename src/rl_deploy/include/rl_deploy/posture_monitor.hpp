/**
 * @file posture_monitor.hpp
 * @brief Mode-aware posture evaluation for Go2 robot.
 * @author Gabriel Rodriguez
 */

#pragma once
#include "unitree_go/msg/low_state.hpp"

namespace rl_deploy {

struct PoseQuality {
    bool good_stand{false};
    bool good_sit{false};
};

/**
 * Evaluate robot posture against reference positions.
 *
 * @param state      Current LowState from the robot.
 * @param check_stand  Whether to evaluate standing posture (skip if false).
 * @param check_sit    Whether to evaluate sitting posture (skip if false).
 * @return PoseQuality with the requested checks; unchecked fields remain false.
 */
PoseQuality evaluate_pose(
    const unitree_go::msg::LowState& state,
    bool check_stand = true,
    bool check_sit = true);

} // namespace rl_deploy
