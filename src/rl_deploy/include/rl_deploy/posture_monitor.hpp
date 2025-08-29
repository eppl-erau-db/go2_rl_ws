/**
 * @file posture_monitor.hpp
 * @brief 
 * @author Gabriel Rodriguez
 */

#pragma once
#include "unitree_go/msg/low_state.hpp"

namespace rl_deploy {

struct PoseQuality { 
    bool good_stand{false}; 
    bool good_sit{false}; 
};

PoseQuality evaluate_pose(const unitree_go::msg::LowState& state);
} // namespace rl_deploy
