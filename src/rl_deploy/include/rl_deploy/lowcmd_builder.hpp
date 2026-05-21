#pragma once
#include <array>
#include <vector>
#include "rl_deploy/types.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

namespace rl_deploy {

unitree_go::msg::LowCmd make_init_cmd();

unitree_go::msg::LowCmd make_cmd_for_mode(
  Mode mode, 
  const unitree_go::msg::LowState& latest_state, 
  const std::vector<float>& actions
);

unitree_go::msg::LowCmd make_stand_transition_cmd(
  const std::array<double, 12>& start_positions,
  double alpha
);

}
