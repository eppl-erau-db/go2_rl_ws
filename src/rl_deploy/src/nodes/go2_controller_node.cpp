#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "blind_locomotion/msg/button.hpp"

#include <algorithm>

#include "rl_deploy/types.hpp"
#include "rl_deploy/constants.hpp"
#include "rl_deploy/mode_state_machine.hpp"
#include "rl_deploy/lowcmd_builder.hpp"
#include "rl_deploy/posture_monitor.hpp"

namespace {

// Convert Mode enum to string for logging
const char* mode_to_string(rl_deploy::Mode mode) {
  switch (mode) {
    case rl_deploy::Mode::Idle:             return "IDLE";
    case rl_deploy::Mode::Standing:         return "STANDING";
    case rl_deploy::Mode::Sitting:          return "SITTING";
    case rl_deploy::Mode::EmergencySitting: return "EMERGENCY_SITTING";
    case rl_deploy::Mode::Walking:          return "WALKING";
    case rl_deploy::Mode::Damping:          return "DAMPING";
    case rl_deploy::Mode::Killed:           return "KILLED";
    default:                                return "UNKNOWN";
  }
}

// Convert Button message to ButtonState struct
// Button fields: up, down, start, select, a, b, emergency_sit
// ButtonState: stand, sit, emergency_sit, start, stop_walking, soft_abort, kill
rl_deploy::ButtonState buttons_from_msg(const blind_locomotion::msg::Button& m){
  rl_deploy::ButtonState b{};
  b.stand         = m.up;            // up button -> stand
  b.sit           = m.down;          // down button -> sit
  b.emergency_sit = m.emergency_sit; // emergency sit button
  b.start         = m.start;         // start button -> start walking
  b.stop_walking  = m.select;        // select button -> stop walking
  b.soft_abort    = m.a;             // A button -> soft abort (damping)
  b.kill          = m.b;             // B button -> kill (emergency stop)
  return b;
}
}

class Go2ControllerNode : public rclcpp::Node {
public:
  Go2ControllerNode() : Node("go2_controller_node") {
    // Declare parameters
    this->declare_parameter<bool>("enable_joint_limit_monitor", true);

    // Build safety config from parameters
    safety_.enable_joint_limit_monitor = this->get_parameter("enable_joint_limit_monitor").as_bool();
    
    // Publishers
    pub_ = create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

    // Subscribers
    sub_actions_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "actions", 10, [this](std_msgs::msg::Float32MultiArray::SharedPtr m){
        actions_ = m->data;
        last_actions_time_ = this->now();
      });

    sub_buttons_ = create_subscription<blind_locomotion::msg::Button>(
      "buttons", 10, [this](blind_locomotion::msg::Button::SharedPtr m){
        buttons_ = buttons_from_msg(*m);
      });

    sub_lowstate_ = create_subscription<unitree_go::msg::LowState>(
      "/lowstate", 10, [this](unitree_go::msg::LowState::SharedPtr m){
        lowstate_ = *m;
        last_lowstate_time_ = this->now();
        lowstate_received_ = true;
      });

    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](geometry_msgs::msg::Twist::SharedPtr m){
        latest_cmd_vel_ = *m;
        last_cmd_vel_time_ = this->now();
        cmd_vel_received_ = true;
      });

    // Posture monitor
    posture_monitor_ = std::make_unique<rl_deploy::PostureMonitor>(this->get_logger());

    // Control timer at 200Hz
    timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]{ tick(); });  

    // Initialize timestamps
    last_lowstate_time_ = this->now();
    last_actions_time_ = this->now();
    last_cmd_vel_time_ = this->now();

    RCLCPP_INFO(get_logger(), "Go2ControllerNode started");
    RCLCPP_INFO(get_logger(), "Safety config: joint_limit_monitor=%s",
                safety_.enable_joint_limit_monitor ? "ON" : "OFF");
    RCLCPP_INFO(get_logger(), "Waiting for /lowstate...");
  }

private:
  void tick(){
    auto now = this->now();

    // lowstate not recieved check
    if (!lowstate_received_) {
      return;
    }

    // joint limit safety check
    if (safety_.enable_joint_limit_monitor) {
      if (mode_ == rl_deploy::Mode::Walking || joint_limit_triggered_) {
        auto jlr = posture_monitor_->check_joint_limits(lowstate_);

        if (jlr.hard_violation) {
          if (!joint_limit_triggered_) {
            RCLCPP_ERROR(get_logger(),
              "Joint limit violation! Transitioning to EMERGENCY_SITTING.");
            joint_limit_triggered_ = true;
          }
          mode_ = rl_deploy::Mode::EmergencySitting;
          pub_->publish(rl_deploy::make_cmd_for_mode(mode_, lowstate_, {}));
          return;
        }

        // reset logic stays system-level (mode is owned by node)
        if (joint_limit_triggered_ && jlr.all_in_soft_zone && mode_ == rl_deploy::Mode::Idle) {
          RCLCPP_INFO(get_logger(), "All joints in safe zone - limit guard reset");
          joint_limit_triggered_ = false;
        }
      }
    }

    // Derive status from lowstate
    auto pq = posture_monitor_->evaluate_pose(lowstate_);
    status_.good_stand = pq.good_stand;
    status_.good_sit   = pq.good_sit;
    status_.is_walking = (mode_ == rl_deploy::Mode::Walking);

    // Choose mode based on FSM
    auto old_mode = mode_;
    mode_ = fsm_.update(buttons_, status_);
    
    // Log mode transitions
    if (mode_ != old_mode) {
      RCLCPP_INFO(get_logger(), "Mode transition: %s -> %s",
                  mode_to_string(old_mode), mode_to_string(mode_));
    }

    // Validate actions if in walking mode
    std::vector<float> safe_actions = actions_;
    if (mode_ == rl_deploy::Mode::Walking) {
      if (actions_.size() != 12) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Invalid actions size: %zu (expected 12). Using StandPos fallback.", actions_.size());
        safe_actions = std::vector<float>(
            std::begin(rl_deploy::StandPos), std::end(rl_deploy::StandPos));
      }
      
      // Check for stale actions (100ms timeout)
      if ((now - last_actions_time_).seconds() > 0.1) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Actions timeout! age=%.1f ms. Continuing to use last received actions.",
          (now - last_actions_time_).seconds() * 1000.0);
      }
    }

    // Build and publish command
    auto cmd = rl_deploy::make_cmd_for_mode(mode_, lowstate_, safe_actions);
    pub_->publish(cmd);
  }
  
  // Publishers & Subscribers
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_actions_;
  rclcpp::Subscription<blind_locomotion::msg::Button>::SharedPtr sub_buttons_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr sub_lowstate_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Data/state
  std::vector<float> actions_;
  unitree_go::msg::LowState lowstate_;
  geometry_msgs::msg::Twist latest_cmd_vel_{};
  rl_deploy::ButtonState buttons_;
  rl_deploy::StatusFlags status_;
  rl_deploy::SafetyConfig safety_;
  rl_deploy::Mode mode_{rl_deploy::Mode::Idle};
  rl_deploy::ModeStateMachine fsm_;
  std::unique_ptr<rl_deploy::PostureMonitor> posture_monitor_;
  
  // Safety timestamps
  rclcpp::Time last_lowstate_time_;
  rclcpp::Time last_actions_time_;
  rclcpp::Time last_cmd_vel_time_;
  bool lowstate_received_{false};
  bool cmd_vel_received_{false};
  bool joint_limit_triggered_{false};  // True when joint limit violation detected
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
