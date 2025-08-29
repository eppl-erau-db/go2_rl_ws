#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

#include "rl_deploy/types.hpp"
#include "rl_deploy/posture_monitor.hpp"
#include "rl_deploy/mode_state_machine.hpp"
#include "rl_deploy/lowcmd_builder.hpp"

using std::placeholders::_1;

namespace {
rl_deploy::ButtonState buttons_from_msg(const std_msgs::msg::Float32MultiArray& m){
  rl_deploy::ButtonState b{};
  if (m.data.size()>0) b.stand        = m.data[0]==1.f;
  if (m.data.size()>1) b.sit          = m.data[1]==1.f;
  if (m.data.size()>2) b.start        = m.data[2]==1.f;
  if (m.data.size()>3) b.stop_walking = m.data[3]==1.f;
  if (m.data.size()>4) b.soft_abort   = m.data[4]==1.f;
  if (m.data.size()>5) b.kill         = m.data[5]==1.f;
  return b;
}
}

class Go2ControllerNode : public rclcpp::Node {
public:
  Go2ControllerNode() : Node("go2_controller_node") {
    pub_ = create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

    sub_actions_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "actions", 10, [this](std_msgs::msg::Float32MultiArray::SharedPtr m){
        actions_ = m->data;
      });

    sub_buttons_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "buttons", 10, [this](std_msgs::msg::Float32MultiArray::SharedPtr m){
        buttons_ = buttons_from_msg(*m);
      });

    sub_lowstate_ = create_subscription<unitree_go::msg::LowState>(
      "/lowstate", 10, [this](unitree_go::msg::LowState::SharedPtr m){
        lowstate_ = *m;
      });

    timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]{ tick(); });

    RCLCPP_INFO(get_logger(), "Go2ControllerNode started");
  }

private:
  void tick(){
    // derive status from lowstate
    auto pq = rl_deploy::evaluate_pose(lowstate_);
    status_.good_stand = pq.good_stand;
    status_.good_sit   = pq.good_sit;
    status_.is_walking = (mode_ == rl_deploy::Mode::Walking);

    // choose mode
    mode_ = fsm_.update(buttons_, status_);

    // build and publish
    auto cmd = rl_deploy::make_cmd_for_mode(mode_, lowstate_, actions_);
    pub_->publish(cmd);
  }

  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_actions_, sub_buttons_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr sub_lowstate_;
  rclcpp::TimerBase::SharedPtr timer_;

  // data/state
  std::vector<float> actions_;
  unitree_go::msg::LowState lowstate_;
  rl_deploy::ButtonState buttons_;
  rl_deploy::StatusFlags status_;
  rl_deploy::Mode mode_{rl_deploy::Mode::Idle};
  rl_deploy::ModeStateMachine fsm_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
