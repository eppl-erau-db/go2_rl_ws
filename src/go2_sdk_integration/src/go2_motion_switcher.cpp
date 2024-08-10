#include "rclcpp/rclcpp.hpp"
#include "unitree/robot/go2/motion_switcher/motion_switcher_client.hpp"

class Go2MotionSwitcher : public rclcpp::Node
{
public:
    Go2MotionSwitcher() : Node("go2_motion_switcher")
    {
        // Create MotionSwitcherClient instance
        motion_switcher_client_ = std::make_shared<unitree_sdk2::MotionSwitcherClient>();

        // Check current mode
        std::string current_form;
        std::string current_mode;
        int32_t check_result = motion_switcher_client_->CheckMode(current_form, current_mode);
        if (check_result == 0) {
            RCLCPP_INFO(this->get_logger(), "Current mode: %s, Form: %s", current_mode.c_str(), current_form.c_str());
            
            // Set motion mode to "ai" if not already set
            if (current_mode != "ai") {
                int32_t select_result = motion_switcher_client_->SelectMode("ai");
                if (select_result == 0) {
                    RCLCPP_INFO(this->get_logger(), "Mode switched to ai successfully.");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to switch mode. Error code: %d", select_result);
                }
            } else {
                RCLCPP_INFO(this->get_logger(), "Mode is already set to ai.");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to check mode. Error code: %d", check_result);
        }
    }

private:
    std::shared_ptr<unitree_sdk2::MotionSwitcherClient> motion_switcher_client_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Go2MotionSwitcher>());
    rclcpp::shutdown();
    return 0;
}