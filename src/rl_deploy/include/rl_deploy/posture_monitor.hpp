#pragma once
#include <rclcpp/rclcpp.hpp>
#include "unitree_go/msg/low_state.hpp"
#include "rl_deploy/types.hpp"


namespace rl_deploy {

struct PoseQuality {
  bool good_stand{false};
  bool good_sit{false};
};

struct JointLimitReport {
  bool hard_violation{false};
  bool all_in_soft_zone{true};
};

class PostureMonitor {
public:
  explicit PostureMonitor(const rclcpp::Logger& logger);

  PoseQuality evaluate_pose(const unitree_go::msg::LowState& state) const;
  JointLimitReport check_joint_limits(const unitree_go::msg::LowState& state) const;

private:
  rclcpp::Logger logger_;
};

}