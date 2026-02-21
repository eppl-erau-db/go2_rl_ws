/**
 * @file posture_monitor.cpp
 * @brief Implementation of mode-aware posture evaluation for Go2 robot.
 * @author Gabriel Rodriguez
 */

#include "rl_deploy/posture_monitor.hpp"
#include "rl_deploy/constants.hpp"
#include <cmath>

namespace rl_deploy {

PoseQuality evaluate_pose(
    const unitree_go::msg::LowState& state,
    bool check_stand,
    bool check_sit)
{
    PoseQuality result{};

    bool all_stand_ok = true;
    bool all_sit_ok = true;

    for (size_t i = 0; i < 12; ++i) {
        double current_pos = state.motor_state[i].q;

        if (check_stand && all_stand_ok) {
            if (std::abs(current_pos - StandPos[i]) > position_tolerance) {
                all_stand_ok = false;
            }
        }

        if (check_sit && all_sit_ok) {
            if (std::abs(current_pos - SitPos[i]) > position_tolerance) {
                all_sit_ok = false;
            }
        }

        // Early exit: both checks failed, no need to continue.
        if (!all_stand_ok && !all_sit_ok) break;
    }

    if (check_stand) result.good_stand = all_stand_ok;
    if (check_sit)   result.good_sit   = all_sit_ok;

    return result;
}

} // namespace rl_deploy
