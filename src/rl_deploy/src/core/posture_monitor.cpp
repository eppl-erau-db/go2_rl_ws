/**
 * @file posture_monitor.cpp
 * @brief Implementation of posture evaluation for Go2 robot.
 * @author Gabriel Rodriguez
 */

#include "rl_deploy/posture_monitor.hpp"
#include "rl_deploy/constants.hpp"
#include <cmath>

namespace rl_deploy {

PoseQuality evaluate_pose(const unitree_go::msg::LowState& state) {
    PoseQuality result{};
    
    // Check if all joints are within tolerance of standing position
    bool all_stand_ok = true;
    bool all_sit_ok = true;
    
    for (size_t i = 0; i < 12; ++i) {
        double current_pos = state.motor_state[i].q;
        
        // Check standing position
        if (std::abs(current_pos - StandPos[i]) > position_tolerance) {
            all_stand_ok = false;
        }
        
        // Check sitting position
        if (std::abs(current_pos - SitPos[i]) > position_tolerance) {
            all_sit_ok = false;
        }
    }
    
    result.good_stand = all_stand_ok;
    result.good_sit = all_sit_ok;
    
    return result;
}

} // namespace rl_deploy
