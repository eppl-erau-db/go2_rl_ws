#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "blind_locomotion/msg/button.hpp"

#include "rl_deploy/types.hpp"
#include "rl_deploy/mode_state_machine.hpp"
#include "rl_deploy/lowcmd_builder.hpp"
#include "rl_deploy/posture_monitor.hpp"

using std::placeholders::_1;

namespace {

// Convert Mode enum to string for logging
const char* mode_to_string(rl_deploy::Mode mode) {
  switch (mode) {
    case rl_deploy::Mode::Idle:     return "IDLE";
    case rl_deploy::Mode::Standing: return "STANDING";
    case rl_deploy::Mode::Sitting:  return "SITTING";
    case rl_deploy::Mode::Walking:  return "WALKING";
    case rl_deploy::Mode::Damping:  return "DAMPING";
    case rl_deploy::Mode::Killed:   return "KILLED";
    default:                        return "UNKNOWN";
  }
}

// Convert Button message to ButtonState struct
// Button fields: up, down, start, select, a, b
// ButtonState: stand, sit, start, stop_walking, soft_abort, kill
rl_deploy::ButtonState buttons_from_msg(const blind_locomotion::msg::Button& m){
  rl_deploy::ButtonState b{};
  b.stand        = m.up;        // up button -> stand
  b.sit          = m.down;      // down button -> sit
  b.start        = m.start;     // start button -> start walking
  b.stop_walking = m.select;    // select button -> stop walking
  b.soft_abort   = m.a;         // A button -> soft abort (damping)
  b.kill         = m.b;         // B button -> kill (emergency stop)
  return b;
}
}

class Go2ControllerNode : public rclcpp::Node {
public:
  Go2ControllerNode() : Node("go2_controller_node") {
    // Declare parameters
    this->declare_parameter<bool>("verbose", true);
    this->declare_parameter<double>("verbose_rate", 2.0);  // Hz for verbose output
    
    verbose_ = this->get_parameter("verbose").as_bool();
    verbose_interval_ = 1.0 / this->get_parameter("verbose_rate").as_double();
    
    // Publishers
    pub_ = create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

    // Subscribers
    sub_actions_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "actions", 10, [this](std_msgs::msg::Float32MultiArray::SharedPtr m){
        actions_ = m->data;
        last_actions_time_ = this->now();
        if (verbose_) {
          RCLCPP_DEBUG(get_logger(), "Received actions: size=%zu", m->data.size());
        }
      });

    sub_buttons_ = create_subscription<blind_locomotion::msg::Button>(
      "buttons", 10, [this](blind_locomotion::msg::Button::SharedPtr m){
        auto old_buttons = buttons_;
        buttons_ = buttons_from_msg(*m);
        
        // Log button presses (only when verbose and button state changes)
        if (verbose_) {
          if (buttons_.stand && !old_buttons.stand) 
            RCLCPP_INFO(get_logger(), "Button: STAND (up) pressed");
          if (buttons_.sit && !old_buttons.sit) 
            RCLCPP_INFO(get_logger(), "Button: SIT (down) pressed");
          if (buttons_.start && !old_buttons.start) 
            RCLCPP_INFO(get_logger(), "Button: START pressed");
          if (buttons_.stop_walking && !old_buttons.stop_walking) 
            RCLCPP_INFO(get_logger(), "Button: STOP WALKING (select) pressed");
          if (buttons_.soft_abort && !old_buttons.soft_abort) 
            RCLCPP_INFO(get_logger(), "Button: SOFT ABORT (A) pressed");
          if (buttons_.kill && !old_buttons.kill) 
            RCLCPP_INFO(get_logger(), "Button: KILL (B) pressed");
        }
      });

    sub_lowstate_ = create_subscription<unitree_go::msg::LowState>(
      "/lowstate", 10, [this](unitree_go::msg::LowState::SharedPtr m){
        lowstate_ = *m;
        last_lowstate_time_ = this->now();
        lowstate_received_ = true;
      });

    // Control timer at 200Hz
    timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]{ tick(); });

    // Initialize timestamps
    last_lowstate_time_ = this->now();
    last_actions_time_ = this->now();
    last_verbose_time_ = this->now();

    RCLCPP_INFO(get_logger(), "Go2ControllerNode started");
    RCLCPP_INFO(get_logger(), "Verbose logging: %s (rate: %.1f Hz)", 
                verbose_ ? "ENABLED" : "DISABLED",
                1.0 / verbose_interval_);
    RCLCPP_INFO(get_logger(), "Waiting for /lowstate...");
  }

private:
  void tick(){
    // Safety check: don't send commands until we've received lowstate
    if (!lowstate_received_) {
      return;
    }

    // Safety check: timeout on lowstate (500ms)
    auto now = this->now();
    if ((now - last_lowstate_time_).seconds() > 0.5) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
        "LowState timeout! Entering damping mode.");
      mode_ = rl_deploy::Mode::Damping;
      auto cmd = rl_deploy::make_cmd_for_mode(mode_, lowstate_, {});
      pub_->publish(cmd);
      return;
    }

    // Derive status from lowstate
    auto pq = rl_deploy::evaluate_pose(lowstate_);
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
          "Invalid actions size: %zu (expected 12). Using zeros.", actions_.size());
        safe_actions = std::vector<float>(12, 0.0f);
      }
      
      // Check for stale actions (100ms timeout)
      if ((now - last_actions_time_).seconds() > 0.1) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Actions timeout! Robot may not respond correctly.");
      }
    }

    // Build and publish command
    auto cmd = rl_deploy::make_cmd_for_mode(mode_, lowstate_, safe_actions);
    pub_->publish(cmd);
    
    // Verbose status output (rate limited)
    if (verbose_ && (now - last_verbose_time_).seconds() >= verbose_interval_) {
      last_verbose_time_ = now;
      log_status();
    }
  }
  
  void log_status() {
    RCLCPP_INFO(get_logger(), 
      "[Status] Mode: %s | good_stand: %s | good_sit: %s | actions_size: %zu",
      mode_to_string(mode_),
      status_.good_stand ? "YES" : "NO",
      status_.good_sit ? "YES" : "NO",
      actions_.size());
      
    // Log first 3 joint positions for quick reference
    if (lowstate_received_) {
      RCLCPP_INFO(get_logger(),
        "[Joints] FR: hip=%.2f thigh=%.2f calf=%.2f",
        lowstate_.motor_state[0].q,
        lowstate_.motor_state[1].q,
        lowstate_.motor_state[2].q);
    }
  }

  // Publishers & Subscribers
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_actions_;
  rclcpp::Subscription<blind_locomotion::msg::Button>::SharedPtr sub_buttons_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr sub_lowstate_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Data/state
  std::vector<float> actions_;
  unitree_go::msg::LowState lowstate_;
  rl_deploy::ButtonState buttons_;
  rl_deploy::StatusFlags status_;
  rl_deploy::Mode mode_{rl_deploy::Mode::Idle};
  rl_deploy::ModeStateMachine fsm_;
  
  // Safety timestamps
  rclcpp::Time last_lowstate_time_;
  rclcpp::Time last_actions_time_;
  bool lowstate_received_{false};
  
  // Verbose logging
  bool verbose_{true};
  double verbose_interval_{0.5};
  rclcpp::Time last_verbose_time_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
