#include "rl_deploy/posture_monitor.hpp"
#include "rl_deploy/constants.hpp"
#include <cmath>


namespace rl_deploy {

PostureMonitor::PostureMonitor(const rclcpp::Logger& logger)
: logger_(logger)
{
}

PoseQuality PostureMonitor::evaluate_pose(const unitree_go::msg::LowState& state) const
{
    PoseQuality result{};
    
    bool all_stand_ok = true;
    bool all_sit_ok = true;

    for (size_t joint = 0; joint < 12; ++joint) {
        double q = state.motor_state[joint].q;
        
        // Check standing position
        if (std::abs(q - StandPos[joint]) > position_tolerance) {
            all_stand_ok = false;
        }
        
        // Check sitting position
        if (std::abs(q - SitPos[joint]) > position_tolerance) {
            all_sit_ok = false;
        }
    }
    
    result.good_stand = all_stand_ok;
    result.good_sit = all_sit_ok;
    return result;
}

JointLimitReport PostureMonitor::check_joint_limits(const unitree_go::msg::LowState& state) const
{
    JointLimitReport result{};
    result.hard_violation = false;
    result.all_in_soft_zone = true;

    for (size_t joint = 0; joint < rl_deploy::num_joints; ++joint) {
        double q = state.motor_state[joint].q;
        double dq = state.motor_state[joint].dq;
        const double min_limit = JointMin[joint];
        const double max_limit = JointMax[joint];
        const double soft_min = min_limit + soft_margin;
        const double soft_max = max_limit - soft_margin;
        
        // Check if inside soft zone 
        if (q < soft_min || q > soft_max) {
            result.all_in_soft_zone = false;
        }
        
        // Hard limit check & error
        if (q < min_limit || q > max_limit) {
            RCLCPP_ERROR(
                logger_, 
                "HARD LIMIT: Joint %zu = %.3f (limits: [%.3f, %.3f])",
                joint, q, min_limit, max_limit
            );
            result.hard_violation = true;
        }

        // Soft limit check & warning
        else if ((q < soft_min && dq < 0) || (q > soft_max && dq > 0)) {
            RCLCPP_WARN(
                logger_,
                "Soft limit: Joint %zu = %.3f, dq = %.2f", 
                joint, q, dq
            );
        }
    }
    return result;
}

}
