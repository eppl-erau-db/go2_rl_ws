#include "rl_deploy/lowcmd_builder.hpp"
#include "rl_deploy/constants.hpp"
#include "motor_crc.h" // vendor CRC func

namespace rl_deploy {

unitree_go::msg::LowCmd make_init_cmd() {
  unitree_go::msg::LowCmd cmd{};
  // minimal safe init; you’ll fill this later
  get_crc(cmd);
  return cmd;
}

unitree_go::msg::LowCmd make_cmd_for_mode(
  Mode, const unitree_go::msg::LowState&, const std::vector<float>&)
{
  unitree_go::msg::LowCmd cmd{};
  // minimal placeholder; we’ll implement per-mode later
  get_crc(cmd);
  return cmd;
}

} // namespace rl_deploy
