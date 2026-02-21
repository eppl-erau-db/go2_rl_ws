#include <rclcpp/rclcpp.hpp>
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "blind_locomotion/msg/button.hpp"
#include "blind_locomotion/msg/joint_position_command.hpp"

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
rl_deploy::ButtonState buttons_from_msg(const blind_locomotion::msg::Button& m){
  rl_deploy::ButtonState b{};
  b.stand         = m.up;
  b.sit           = m.down;
  b.emergency_sit = m.emergency_sit;
  b.start         = m.start;
  b.stop_walking  = m.select;
  b.soft_abort    = m.a;
  b.kill          = m.b;
  return b;
}
}

class Go2ControllerNode : public rclcpp::Node {
public:
  Go2ControllerNode() : Node("go2_controller_node") {
    // Declare parameters
    this->declare_parameter<bool>("verbose", true);
    this->declare_parameter<double>("verbose_rate", 2.0);
    this->declare_parameter<bool>("enable_joint_limit_monitor", true);
    this->declare_parameter<bool>("debug_enabled", true);
    this->declare_parameter<double>("debug_rate_hz", 5.0);
    
    verbose_ = this->get_parameter("verbose").as_bool();
    const double verbose_rate_hz = std::max(0.1, this->get_parameter("verbose_rate").as_double());
    verbose_interval_ = 1.0 / verbose_rate_hz;
    debug_enabled_ = this->get_parameter("debug_enabled").as_bool();
    const double debug_rate_hz = std::max(0.1, this->get_parameter("debug_rate_hz").as_double());
    debug_interval_ = 1.0 / debug_rate_hz;
    
    safety_.enable_joint_limit_monitor = this->get_parameter("enable_joint_limit_monitor").as_bool();
    
    // Publisher
    pub_ = create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

    // Subscribers
    sub_actions_ = create_subscription<blind_locomotion::msg::JointPositionCommand>(
      "actions", 10, [this](blind_locomotion::msg::JointPositionCommand::SharedPtr m){
        actions_.assign(m->positions.begin(), m->positions.end());
        last_actions_time_ = this->now();
        if (verbose_) {
          RCLCPP_DEBUG(get_logger(), "Received actions: size=%zu", actions_.size());
        }
      });

    sub_buttons_ = create_subscription<blind_locomotion::msg::Button>(
      "buttons", 10, [this](blind_locomotion::msg::Button::SharedPtr m){
        auto old_buttons = buttons_;
        buttons_ = buttons_from_msg(*m);
        
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

    // Control timer at 200Hz
    timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]{ tick(); });

    // Initialize timestamps
    last_lowstate_time_ = this->now();
    last_actions_time_ = this->now();
    last_verbose_time_ = this->now();
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
    if (!lowstate_received_) {
      return;
    }

    auto now = this->now();

    // Safety: timeout on lowstate (500ms)
    if ((now - last_lowstate_time_).seconds() > 0.5) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
        "LowState timeout! Entering damping mode.");
      mode_ = rl_deploy::Mode::Damping;
      auto cmd = rl_deploy::make_cmd_for_mode(mode_, lowstate_, {});
      pub_->publish(cmd);
      return;
    }

    // Joint limit safety check (during Walking or recovering from violation)
    if (safety_.enable_joint_limit_monitor) {
      if (mode_ == rl_deploy::Mode::Walking || joint_limit_triggered_) {
        if (check_joint_limits()) {
          if (!joint_limit_triggered_) {
            RCLCPP_ERROR(get_logger(), 
              "Joint limit violation! Transitioning to EMERGENCY_SITTING.");
            joint_limit_triggered_ = true;
          }
          mode_ = rl_deploy::Mode::EmergencySitting;
          auto cmd = rl_deploy::make_cmd_for_mode(mode_, lowstate_, {});
          pub_->publish(cmd);
          return;
        }
      }
    }

    // Mode-aware posture evaluation: only check what the current mode needs.
    bool need_stand_check = (mode_ == rl_deploy::Mode::Standing
                          || mode_ == rl_deploy::Mode::Walking
                          || mode_ == rl_deploy::Mode::Idle);
    bool need_sit_check   = (mode_ == rl_deploy::Mode::Sitting
                          || mode_ == rl_deploy::Mode::EmergencySitting);
    auto pq = rl_deploy::evaluate_pose(lowstate_, need_stand_check, need_sit_check);
    status_.good_stand = pq.good_stand;
    status_.good_sit   = pq.good_sit;
    status_.is_walking = (mode_ == rl_deploy::Mode::Walking);

    // FSM update
    auto old_mode = mode_;
    mode_ = fsm_.update(buttons_, status_);
    
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
      
      if ((now - last_actions_time_).seconds() > 0.1) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Actions timeout! age=%.1f ms. Continuing to use last received actions.",
          (now - last_actions_time_).seconds() * 1000.0);
      }
    }

    auto cmd = rl_deploy::make_cmd_for_mode(mode_, lowstate_, safe_actions);
    pub_->publish(cmd);
    
    // Rate-limited status logging
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
    RCLCPP_INFO(get_logger(), "%s", status_ss.str().c_str());
      
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
    RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
  }

  // Check joint limits against LowState - returns true if hard violation detected
  bool check_joint_limits() {
    bool hard_violation = false;
    bool all_in_soft_zone = true;
    
    for (size_t i = 0; i < 12; ++i) {
      double q = lowstate_.motor_state[i].q;
      double dq = lowstate_.motor_state[i].dq;
      double min_limit = rl_deploy::JointMin[i];
      double max_limit = rl_deploy::JointMax[i];
      double soft_min = min_limit + rl_deploy::soft_margin;
      double soft_max = max_limit - rl_deploy::soft_margin;
      
      if (q < soft_min || q > soft_max) {
        all_in_soft_zone = false;
      }
      
      if (q < min_limit || q > max_limit) {
        RCLCPP_ERROR(get_logger(), 
          "HARD LIMIT: Joint %zu = %.3f (limits: [%.3f, %.3f])",
          i, q, min_limit, max_limit);
        hard_violation = true;
      }
      else if ((q < soft_min && dq < 0) || (q > soft_max && dq > 0)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
          "Soft limit: Joint %zu = %.3f, dq = %.2f", i, q, dq);
      }
    }
    
    if (joint_limit_triggered_ && all_in_soft_zone && mode_ == rl_deploy::Mode::Idle) {
      RCLCPP_INFO(get_logger(), "All joints in safe zone - limit guard reset");
      joint_limit_triggered_ = false;
    }
    
    return hard_violation;
  }

  // Publishers & Subscribers
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr pub_;
  rclcpp::Subscription<blind_locomotion::msg::JointPositionCommand>::SharedPtr sub_actions_;
  rclcpp::Subscription<blind_locomotion::msg::Button>::SharedPtr sub_buttons_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr sub_lowstate_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Data/state
  std::vector<float> actions_;
  unitree_go::msg::LowState lowstate_;
  rl_deploy::ButtonState buttons_;
  rl_deploy::StatusFlags status_;
  rl_deploy::SafetyConfig safety_;
  rl_deploy::Mode mode_{rl_deploy::Mode::Idle};
  rl_deploy::ModeStateMachine fsm_;
  
  // Safety timestamps
  rclcpp::Time last_lowstate_time_;
  rclcpp::Time last_actions_time_;
  bool lowstate_received_{false};
  bool joint_limit_triggered_{false};
  
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
