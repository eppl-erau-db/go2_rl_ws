#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "blind_locomotion/msg/button.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <exception>
#include <string>

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
    case rl_deploy::Mode::Pedipulation:     return "PEDIPULATION";
    case rl_deploy::Mode::Damping:          return "DAMPING";
    case rl_deploy::Mode::Killed:           return "KILLED";
    default:                                return "UNKNOWN";
  }
}

// Convert Button message to ButtonState struct
// Button fields: up, down, start, select, a, b, emergency_sit, f1
// ButtonState: stand, sit, emergency_sit, start, pedipulate, f1, soft_abort, kill
rl_deploy::ButtonState buttons_from_msg(const blind_locomotion::msg::Button& m){
  rl_deploy::ButtonState b{};
  b.stand         = m.up;            // up button -> stand
  b.sit           = m.down;          // down button -> sit
  b.emergency_sit = m.emergency_sit; // emergency sit button
  b.start         = m.start;         // start button -> start walking
  b.pedipulate    = m.select;        // select button -> pedipulation mode
  b.f1            = m.f1;            // F1 button -> pedipulation push trigger
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
    this->declare_parameter<std::string>("walking_actions_topic", "actions");
    this->declare_parameter<std::string>("pedipulation_actions_topic", "actions");
    this->declare_parameter<std::string>(
      "pedipulation_execute_service", "pedipulation/execute_push");
    this->declare_parameter<std::string>(
      "pedipulation_cancel_service", "pedipulation/cancel_push");
    this->declare_parameter<bool>("stand_on_start", false);
    this->declare_parameter<bool>("ignore_pedipulate_until_select_released", false);
    this->declare_parameter<bool>("require_fresh_pedipulation_actions_for_entry", false);
    this->declare_parameter<bool>("disable_start_walking", false);
    this->declare_parameter<bool>("enable_ai_sport_return", false);
    this->declare_parameter<double>("ai_sport_return_sit_timeout_sec", 6.0);
    this->declare_parameter<bool>("wait_for_handoff_enable", false);

    // Build safety config from parameters
    safety_.enable_joint_limit_monitor = this->get_parameter("enable_joint_limit_monitor").as_bool();
    walking_actions_topic_ = this->get_parameter("walking_actions_topic").as_string();
    pedipulation_actions_topic_ = this->get_parameter("pedipulation_actions_topic").as_string();
    pedipulation_execute_service_ =
      this->get_parameter("pedipulation_execute_service").as_string();
    pedipulation_cancel_service_ =
      this->get_parameter("pedipulation_cancel_service").as_string();
    stand_on_start_ = this->get_parameter("stand_on_start").as_bool();
    ignore_pedipulate_until_select_released_ =
      this->get_parameter("ignore_pedipulate_until_select_released").as_bool();
    require_fresh_pedipulation_actions_for_entry_ =
      this->get_parameter("require_fresh_pedipulation_actions_for_entry").as_bool();
    disable_start_walking_ =
      this->get_parameter("disable_start_walking").as_bool();
    enable_ai_sport_return_ =
      this->get_parameter("enable_ai_sport_return").as_bool();
    ai_sport_return_sit_timeout_sec_ = std::max(
      0.0,
      this->get_parameter("ai_sport_return_sit_timeout_sec").as_double());
    wait_for_handoff_enable_ =
      this->get_parameter("wait_for_handoff_enable").as_bool();
    handoff_enable_received_ = !wait_for_handoff_enable_;
    
    // Publishers
    pub_ = create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);
    // Controller status for launch-time sequencing (e.g. start leg odometry
    // only once the robot is actually standing).
    pub_mode_ = create_publisher<std_msgs::msg::String>("controller_mode", 10);
    pub_stand_ready_ = create_publisher<std_msgs::msg::Bool>("stand_ready", 10);
    execute_push_client_ = create_client<std_srvs::srv::Trigger>(pedipulation_execute_service_);
    cancel_push_client_ = create_client<std_srvs::srv::Trigger>(pedipulation_cancel_service_);

    // Subscribers
    sub_walking_actions_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      walking_actions_topic_, 10, [this](std_msgs::msg::Float32MultiArray::SharedPtr m){
        walking_actions_ = m->data;
        last_walking_actions_time_ = this->now();
      });

    sub_pedipulation_actions_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      pedipulation_actions_topic_, 10, [this](std_msgs::msg::Float32MultiArray::SharedPtr m){
        pedipulation_actions_ = m->data;
        last_pedipulation_actions_time_ = this->now();
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

    if (wait_for_handoff_enable_) {
      auto handoff_qos = rclcpp::QoS(rclcpp::KeepLast(1));
      handoff_qos.reliable();
      handoff_qos.transient_local();
      sub_handoff_enable_ = create_subscription<std_msgs::msg::Bool>(
        "/low_level_handoff/enable",
        handoff_qos,
        [this](std_msgs::msg::Bool::SharedPtr m){
          if (!m->data || handoff_enable_received_) {
            return;
          }
          handoff_enable_received_ = true;
          handoff_ramp_started_ = false;
          handoff_ramp_complete_ = false;
          RCLCPP_INFO(
            get_logger(),
            "Received low-level handoff enable; starting standing takeover ramp");
        });
    }

    // Posture monitor
    posture_monitor_ = std::make_unique<rl_deploy::PostureMonitor>(this->get_logger());

    // Control timer at 200Hz
    timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]{ tick(); });  

    // Initialize timestamps
    last_lowstate_time_ = this->now();
    last_walking_actions_time_ = this->now();
    last_pedipulation_actions_time_ = this->now();
    last_cmd_vel_time_ = this->now();
    ai_sport_return_request_time_ = this->now();

    RCLCPP_INFO(get_logger(), "Go2ControllerNode started");
    RCLCPP_INFO(get_logger(), "Safety config: joint_limit_monitor=%s",
                safety_.enable_joint_limit_monitor ? "ON" : "OFF");
    RCLCPP_INFO(get_logger(), "Action sources: walking=%s pedipulation=%s",
                walking_actions_topic_.c_str(), pedipulation_actions_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Pedipulation services: execute=%s cancel=%s",
                pedipulation_execute_service_.c_str(), pedipulation_cancel_service_.c_str());
    if (stand_on_start_) {
      RCLCPP_INFO(get_logger(), "Initial low-level command: STANDING");
    }
    if (ignore_pedipulate_until_select_released_) {
      RCLCPP_INFO(
        get_logger(),
        "Ignoring SELECT pedipulation requests until SELECT is released once");
    }
    if (require_fresh_pedipulation_actions_for_entry_) {
      RCLCPP_INFO(
        get_logger(),
        "Pedipulation entry requires fresh pedipulation actions");
    }
    if (disable_start_walking_) {
      RCLCPP_INFO(
        get_logger(),
        "START walking requests disabled; use UP to return to STANDING");
    }
    if (enable_ai_sport_return_) {
      RCLCPP_INFO(
        get_logger(),
        "ai_sport return enabled: press DOWN to sit/idle, then SELECT to stop low-level control");
    }
    if (wait_for_handoff_enable_) {
      RCLCPP_WARN(
        get_logger(),
        "Waiting for /low_level_handoff/enable before publishing any /lowcmd");
    }
    RCLCPP_INFO(get_logger(), "Waiting for /lowstate...");
  }

private:
  static bool is_action_mode(rl_deploy::Mode mode) {
    return mode == rl_deploy::Mode::Walking || mode == rl_deploy::Mode::Pedipulation;
  }

  const std::vector<float>& actions_for_mode(rl_deploy::Mode mode) const {
    return mode == rl_deploy::Mode::Pedipulation ? pedipulation_actions_ : walking_actions_;
  }

  const rclcpp::Time& action_time_for_mode(rl_deploy::Mode mode) const {
    return mode == rl_deploy::Mode::Pedipulation
      ? last_pedipulation_actions_time_
      : last_walking_actions_time_;
  }

  const char* action_source_name(rl_deploy::Mode mode) const {
    return mode == rl_deploy::Mode::Pedipulation ? "pedipulation" : "walking";
  }

  bool pedipulation_actions_ready(const rclcpp::Time& now) const {
    if (pedipulation_actions_.size() != 12) {
      return false;
    }
    return (now - last_pedipulation_actions_time_).seconds() <= 0.1;
  }

  void capture_handoff_start_positions() {
    for (size_t i = 0; i < handoff_start_positions_.size(); ++i) {
      handoff_start_positions_[i] = lowstate_.motor_state[i].q;
    }
  }

  bool publish_handoff_ramp(const rclcpp::Time& now) {
    if (!wait_for_handoff_enable_ || handoff_ramp_complete_) {
      return false;
    }

    if (!handoff_enable_received_) {
      return true;
    }

    if (!handoff_ramp_started_) {
      capture_handoff_start_positions();
      handoff_ramp_start_time_ = now;
      handoff_ramp_started_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Captured current joint positions; holding before StandPos ramp");
    }

    const double elapsed_sec = (now - handoff_ramp_start_time_).seconds();
    const double ramp_elapsed_sec =
      std::max(0.0, elapsed_sec - handoff_hold_sec_);
    const double alpha =
      handoff_ramp_duration_sec_ > 0.0
        ? std::clamp(ramp_elapsed_sec / handoff_ramp_duration_sec_, 0.0, 1.0)
        : 1.0;

    pub_->publish(
      rl_deploy::make_stand_transition_cmd(handoff_start_positions_, alpha));

    if (alpha >= 1.0) {
      handoff_ramp_complete_ = true;
      RCLCPP_INFO(get_logger(), "Standing takeover ramp complete");
    }
    return true;
  }

  void send_trigger_request(
      const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& client,
      const std::string& service_name,
      const char* action_label) {
    if (!client->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Pedipulation %s service unavailable: %s",
        action_label,
        service_name.c_str());
      return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    client->async_send_request(
      request,
      [this, action_label](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        try {
          const auto response = future.get();
          if (response->success) {
            RCLCPP_INFO(get_logger(), "Pedipulation %s accepted: %s",
                        action_label, response->message.c_str());
          } else {
            RCLCPP_WARN(get_logger(), "Pedipulation %s rejected: %s",
                        action_label, response->message.c_str());
          }
        } catch (const std::exception& e) {
          RCLCPP_ERROR(get_logger(), "Pedipulation %s call failed: %s",
                       action_label, e.what());
        }
      });
  }

  void request_execute_push() {
    send_trigger_request(execute_push_client_, pedipulation_execute_service_, "execute");
  }

  void request_cancel_push() {
    send_trigger_request(cancel_push_client_, pedipulation_cancel_service_, "cancel");
  }

  void tick(){
    auto now = this->now();

    // lowstate not recieved check
    if (!lowstate_received_) {
      return;
    }

    if (wait_for_handoff_enable_ && !handoff_enable_received_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Low-level handoff gate is closed; not publishing /lowcmd yet");
      return;
    }

    if (publish_handoff_ramp(now)) {
      return;
    }

    // joint limit safety check
    if (safety_.enable_joint_limit_monitor) {
      if (is_action_mode(mode_) || joint_limit_triggered_) {
        auto jlr = posture_monitor_->check_joint_limits(lowstate_);

        if (jlr.hard_violation) {
          if (!joint_limit_triggered_) {
            RCLCPP_ERROR(get_logger(),
              "Joint limit violation! Transitioning to EMERGENCY_SITTING.");
            joint_limit_triggered_ = true;
          }
          if (mode_ == rl_deploy::Mode::Pedipulation) {
            request_cancel_push();
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

    auto requested_buttons = buttons_;
    if (ignore_pedipulate_until_select_released_) {
      if (requested_buttons.pedipulate) {
        requested_buttons.pedipulate = false;
      } else {
        ignore_pedipulate_until_select_released_ = false;
        RCLCPP_INFO(get_logger(), "SELECT released; pedipulation entry is enabled");
      }
    }

    if (stand_on_start_ && !initial_stand_request_sent_) {
      requested_buttons.stand = true;
      initial_stand_request_sent_ = true;
    }

    if (disable_start_walking_ && requested_buttons.start) {
      requested_buttons.start = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "START ignored: walking is disabled in this low-level phase; press UP to return to STANDING");
    }

    const bool select_rising =
      requested_buttons.pedipulate && !previous_buttons_.pedipulate;
    if (
        enable_ai_sport_return_
        && !ai_sport_return_requested_
        && select_rising
        && (mode_ == rl_deploy::Mode::Idle || mode_ == rl_deploy::Mode::Sitting)) {
      ai_sport_return_requested_ = true;
      ai_sport_return_request_time_ = now;
      requested_buttons = rl_deploy::ButtonState{};
      requested_buttons.sit = true;
      RCLCPP_INFO(
        get_logger(),
        "SELECT pressed from %s: sitting before exiting for ai_sport return",
        mode_to_string(mode_));
    }

    if (ai_sport_return_requested_) {
      requested_buttons = rl_deploy::ButtonState{};
      requested_buttons.sit = true;
    }

    const bool pedipulation_entry_requested =
      requested_buttons.pedipulate
      && mode_ != rl_deploy::Mode::Pedipulation
      && (mode_ == rl_deploy::Mode::Standing || mode_ == rl_deploy::Mode::Walking);
    if (
        require_fresh_pedipulation_actions_for_entry_
        && pedipulation_entry_requested
        && !pedipulation_actions_ready(now)) {
      requested_buttons.pedipulate = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "SELECT ignored: waiting for fresh pedipulation actions before entering PEDIPULATION");
    }

    // Choose mode based on FSM
    auto old_mode = mode_;
    mode_ = fsm_.update(requested_buttons, status_);
    
    // Log mode transitions
    if (mode_ != old_mode) {
      RCLCPP_INFO(get_logger(), "Mode transition: %s -> %s",
                  mode_to_string(old_mode), mode_to_string(mode_));
    }

    const bool f1_rising = requested_buttons.f1 && !previous_buttons_.f1;
    if (old_mode == rl_deploy::Mode::Pedipulation && mode_ != rl_deploy::Mode::Pedipulation) {
      request_cancel_push();
    } else if (
        f1_rising
        && old_mode == rl_deploy::Mode::Pedipulation
        && mode_ == rl_deploy::Mode::Pedipulation) {
      request_execute_push();
    }

    // Validate actions for policy-driven modes.
    std::vector<float> safe_actions;
    if (is_action_mode(mode_)) {
      safe_actions = actions_for_mode(mode_);
      if (safe_actions.size() != 12) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Invalid %s actions size: %zu (expected 12). Using StandPos fallback.",
          action_source_name(mode_), safe_actions.size());
        safe_actions = std::vector<float>(
            std::begin(rl_deploy::StandPos), std::end(rl_deploy::StandPos));
      }
      
      // Check for stale actions (100ms timeout)
      const auto action_age_ms = (now - action_time_for_mode(mode_)).seconds() * 1000.0;
      if (action_age_ms > 100.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "%s actions timeout! age=%.1f ms. Continuing to use last received actions.",
          action_source_name(mode_),
          action_age_ms);
      }
    }

    // Build and publish command
    auto cmd = rl_deploy::make_cmd_for_mode(mode_, lowstate_, safe_actions);
    pub_->publish(cmd);

    if (ai_sport_return_requested_) {
      const auto return_elapsed_sec =
        (now - ai_sport_return_request_time_).seconds();
      if (status_.good_sit || return_elapsed_sec >= ai_sport_return_sit_timeout_sec_) {
        if (!status_.good_sit) {
          RCLCPP_WARN(
            get_logger(),
            "ai_sport return sit timeout reached after %.2fs; exiting low-level control anyway",
            return_elapsed_sec);
        } else {
          RCLCPP_INFO(get_logger(), "Sitting confirmed; exiting low-level control for ai_sport return");
        }
        rclcpp::shutdown();
        return;
      }
    }

    previous_buttons_ = requested_buttons;
    publish_status();
  }

  // Publish controller mode and stand-ready flag at a reduced rate (every
  // status_publish_period_ticks_ control ticks).
  void publish_status() {
    if (++status_tick_counter_ < status_publish_period_ticks_) {
      return;
    }
    status_tick_counter_ = 0;

    std_msgs::msg::String mode_msg;
    mode_msg.data = mode_to_string(mode_);
    pub_mode_->publish(mode_msg);

    std_msgs::msg::Bool stand_ready_msg;
    stand_ready_msg.data =
      (mode_ == rl_deploy::Mode::Standing) && status_.good_stand;
    pub_stand_ready_->publish(stand_ready_msg);
  }
  
  // Publishers & Subscribers
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_mode_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_stand_ready_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_walking_actions_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_pedipulation_actions_;
  rclcpp::Subscription<blind_locomotion::msg::Button>::SharedPtr sub_buttons_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr sub_lowstate_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_handoff_enable_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr execute_push_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cancel_push_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Data/state
  std::string walking_actions_topic_;
  std::string pedipulation_actions_topic_;
  std::string pedipulation_execute_service_;
  std::string pedipulation_cancel_service_;
  std::vector<float> walking_actions_;
  std::vector<float> pedipulation_actions_;
  unitree_go::msg::LowState lowstate_;
  geometry_msgs::msg::Twist latest_cmd_vel_{};
  rl_deploy::ButtonState buttons_;
  rl_deploy::ButtonState previous_buttons_;
  rl_deploy::StatusFlags status_;
  rl_deploy::SafetyConfig safety_;
  rl_deploy::Mode mode_{rl_deploy::Mode::Idle};
  rl_deploy::ModeStateMachine fsm_;
  std::unique_ptr<rl_deploy::PostureMonitor> posture_monitor_;
  
  // Safety timestamps
  rclcpp::Time last_lowstate_time_;
  rclcpp::Time last_walking_actions_time_;
  rclcpp::Time last_pedipulation_actions_time_;
  rclcpp::Time last_cmd_vel_time_;
  rclcpp::Time ai_sport_return_request_time_;
  rclcpp::Time handoff_ramp_start_time_;
  std::array<double, 12> handoff_start_positions_{};
  bool lowstate_received_{false};
  bool cmd_vel_received_{false};
  bool joint_limit_triggered_{false};  // True when joint limit violation detected
  bool stand_on_start_{false};
  bool initial_stand_request_sent_{false};
  bool ignore_pedipulate_until_select_released_{false};
  bool require_fresh_pedipulation_actions_for_entry_{false};
  bool disable_start_walking_{false};
  bool enable_ai_sport_return_{false};
  bool ai_sport_return_requested_{false};
  bool wait_for_handoff_enable_{false};
  bool handoff_enable_received_{true};
  bool handoff_ramp_started_{false};
  bool handoff_ramp_complete_{false};
  double ai_sport_return_sit_timeout_sec_{6.0};
  int status_tick_counter_{0};
  const int status_publish_period_ticks_{10};  // 200Hz tick -> 20Hz status
  const double handoff_hold_sec_{0.25};
  const double handoff_ramp_duration_sec_{1.50};
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
