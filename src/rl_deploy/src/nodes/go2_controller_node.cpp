#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "blind_locomotion/msg/button.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>

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
    this->declare_parameter<bool>("verbose", true);
    this->declare_parameter<double>("verbose_rate", 2.0);  // Hz for verbose output
    this->declare_parameter<bool>("enable_joint_limit_monitor", true);
    this->declare_parameter<bool>("debug_enabled", true);
    this->declare_parameter<double>("debug_rate_hz", 5.0);
    
    verbose_ = this->get_parameter("verbose").as_bool();
    const double verbose_rate_hz = std::max(0.1, this->get_parameter("verbose_rate").as_double());
    verbose_interval_ = 1.0 / verbose_rate_hz;
    debug_enabled_ = this->get_parameter("debug_enabled").as_bool();
    const double debug_rate_hz = std::max(0.1, this->get_parameter("debug_rate_hz").as_double());
    debug_interval_ = 1.0 / debug_rate_hz;
    
    // Build safety config from parameters
    safety_.enable_joint_limit_monitor = this->get_parameter("enable_joint_limit_monitor").as_bool();
    
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
          if (buttons_.emergency_sit && !old_buttons.emergency_sit) 
            RCLCPP_INFO(get_logger(), "Button: EMERGENCY SIT (e) pressed");
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
    last_verbose_time_ = this->now();
    last_cmd_vel_time_ = this->now();
    last_debug_time_ = this->now();

    RCLCPP_INFO(get_logger(), "Go2ControllerNode started");
    RCLCPP_INFO(get_logger(), "Verbose logging: %s (rate: %.1f Hz)", 
                verbose_ ? "ENABLED" : "DISABLED",
                1.0 / verbose_interval_);
    RCLCPP_INFO(get_logger(), "Debug telemetry: %s (rate: %.1f Hz)",
                debug_enabled_ ? "ENABLED" : "DISABLED",
                1.0 / debug_interval_);
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
      if (mode_ == rl_deploy::Mode::Walking) {
        log_walking_entry_snapshot(now);
      }
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
    
    // Verbose status output (rate limited)
    if (debug_enabled_ && (now - last_debug_time_).seconds() >= debug_interval_) {
      last_debug_time_ = now;
      log_status(now);
    } else if (verbose_ && (now - last_verbose_time_).seconds() >= verbose_interval_) {
      last_verbose_time_ = now;
      log_status(now);
    }
  }
  
  void log_status(const rclcpp::Time& now) {
    double action_age_ms = (now - last_actions_time_).seconds() * 1000.0;
    double action_min = 0.0;
    double action_max = 0.0;
    bool action_stats_valid = !actions_.empty();
    if (action_stats_valid) {
      auto action_minmax = std::minmax_element(actions_.begin(), actions_.end());
      action_min = static_cast<double>(*action_minmax.first);
      action_max = static_cast<double>(*action_minmax.second);
    }

    const bool cmd_vel_valid = cmd_vel_received_;
    double cmd_age_ms = cmd_vel_valid ? (now - last_cmd_vel_time_).seconds() * 1000.0 : -1.0;

    std::ostringstream status_ss;
    status_ss << std::fixed << std::setprecision(1);
    status_ss
      << "[Status] Mode: " << mode_to_string(mode_)
      << " | good_stand: " << (status_.good_stand ? "YES" : "NO")
      << " | good_sit: " << (status_.good_sit ? "YES" : "NO")
      << " | actions_size: " << actions_.size()
      << " | action_age_ms: " << action_age_ms;
    if (action_stats_valid) {
      status_ss << " | action_range: [" << std::setprecision(3) << action_min << ", " << action_max << "]";
      status_ss << std::setprecision(1);
    } else {
      status_ss << " | action_range: [n/a]";
    }
    status_ss
      << " | cmd_vel: ["
      << std::setprecision(3)
      << latest_cmd_vel_.linear.x << ", "
      << latest_cmd_vel_.linear.y << ", "
      << latest_cmd_vel_.angular.z
      << std::setprecision(1)
      << "] | cmd_age_ms: ";
    if (cmd_vel_valid) {
      status_ss << cmd_age_ms;
    } else {
      status_ss << "n/a";
    }
    RCLCPP_INFO(get_logger(), "%s", status_ss.str().c_str());
      
    // Log first 3 joint positions for quick reference
    if (lowstate_received_) {
      RCLCPP_INFO(get_logger(),
        "[Joints] FR: hip=%.2f thigh=%.2f calf=%.2f",
        lowstate_.motor_state[0].q,
        lowstate_.motor_state[1].q,
        lowstate_.motor_state[2].q);
    }
  }

  void log_walking_entry_snapshot(const rclcpp::Time& now) {
    double action_age_ms = (now - last_actions_time_).seconds() * 1000.0;
    double action_min = 0.0;
    double action_max = 0.0;
    if (!actions_.empty()) {
      auto action_minmax = std::minmax_element(actions_.begin(), actions_.end());
      action_min = static_cast<double>(*action_minmax.first);
      action_max = static_cast<double>(*action_minmax.second);
    }
    double cmd_age_ms = cmd_vel_received_ ? (now - last_cmd_vel_time_).seconds() * 1000.0 : -1.0;

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(1);
    ss << "WALKING entry snapshot: actions_size=" << actions_.size()
       << " action_age_ms=" << action_age_ms;
    if (!actions_.empty()) {
      ss << " action_range=[" << std::setprecision(3) << action_min << ", " << action_max << "]";
      ss << std::setprecision(1);
    } else {
      ss << " action_range=[n/a]";
    }
    ss << " cmd_vel=["
       << std::setprecision(3)
       << latest_cmd_vel_.linear.x << ", "
       << latest_cmd_vel_.linear.y << ", "
       << latest_cmd_vel_.angular.z
       << std::setprecision(1)
       << "] cmd_age_ms=";
    if (cmd_vel_received_) {
      ss << cmd_age_ms;
    } else {
      ss << "n/a";
    }
    RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
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
  
  // Verbose logging
  bool verbose_{true};
  double verbose_interval_{0.5};
  rclcpp::Time last_verbose_time_;
  bool debug_enabled_{true};
  double debug_interval_{0.2};
  rclcpp::Time last_debug_time_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
