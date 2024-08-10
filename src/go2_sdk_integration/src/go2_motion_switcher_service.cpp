#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"
#include <array>
#include <memory>
#include <stdexcept>
#include <cstdio>

constexpr double sitting_limit = 0.10; // (meters)

class MotionSwitcherServiceNode : public rclcpp::Node
{
public:
    MotionSwitcherServiceNode() 
    : Node("motion_switcher_node"),
    is_sitting_(false),
    ai_req_result_(false),
    sport_req_result_(false)
    {
        // Create the service server
        service_ = this->create_service<std_srvs::srv::SetBool>(
            "switch_modes", std::bind(&MotionSwitcherServiceNode::handle_service, this, std::placeholders::_1, std::placeholders::_2));

        // Subscribe to the "sportmodestate" topic
        state_suber_ = this->create_subscription<unitree_go::msg::SportModeState>(
            "lf/sportmodestate", 10, std::bind(&MotionSwitcherServiceNode::state_callback, this, std::placeholders::_1));

        // Initialize the sport mode request publisher
        req_puber_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

        // Network name parameter
        this->declare_parameter<std::string>("network_interface", "enp114s0");
        network_interface_ = this->get_parameter("network_interface").as_string();
    }

private:
    void handle_service(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        // true is sport/normal mode and false is ai
        if (request->data) 
        {
            mode_request_ = "normal";
        }
        else
        {
            mode_request_ = "ai";
        }

        // Gathering the current mode
        check_mode();

        // Check if we are already in the mode they want:
        if (current_mode_ == mode_request_)
        {
            RCLCPP_INFO(this->get_logger(), "The dog is already in the desired mode.");
            response->success = true;
            response->message = "Already in the desired mode";
            return;
        }

        // Handling pose first
        if (!is_sitting_)
        {
            // Telling the dog to sit, and waiting until it does.
            sport_req_.StandDown(req_); 
            req_puber_->publish(req_);
            RCLCPP_INFO(this->get_logger(), "Sitting the dog down.");

            // Waiting for the dog to sit down
            rclcpp::Rate rate(10);
            while (!is_sitting_)
            {
                RCLCPP_INFO(this->get_logger(), "Waiting until the dog is sat down.");
                rate.sleep();
            }
            RCLCPP_INFO(this->get_logger(), "The dog has been sat down.");
        }    

        // Switch to desired mode
        if (mode_request_ == "ai")
        {
            // Switch to ai mode
            switch_to_ai();
            RCLCPP_INFO(this->get_logger(), "AI switcher function called from service request.");

            // Handle ai request result
            response->success = ai_req_result_;
            response->message = ai_req_result_ ? "Successfully switched to ai mode." : "Failed to switch to ai mode.";
        }
        else if (mode_request_ == "sport" || mode_request_ == "normal")
        {
            // Switch to sport mode
            switch_to_sport();
            RCLCPP_INFO(this->get_logger(), "Sport switcher function called from service request.");

            // Handle sport result
            response->success = sport_req_result_;
            response->message = sport_req_result_ ? "Successfully switched to sport mode." : "Failed to switch to sport mode.";
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Provide a valid mode request: ai, sport, or normal");
        } 
    }

    void check_mode() 
    {
        // Creating, logging, executing, and returning string output of command
        std::string command = "install/go2_sdk_integration/lib/go2_sdk_integration/check_mode " + network_interface_;
        RCLCPP_INFO(this->get_logger(), "Executing: %s", command.c_str());
        int mode = std::system(command.c_str());
        if (mode == 0)
        {
            current_mode_ = "normal";
        }
        else if (mode == 1)
        {
            current_mode_ = "ai";
        }
        else
        {
            current_mode_ = "error";
        }
    }

    void switch_to_ai()
    {
        // Creating, logging and executing command
        std::string command = "install/go2_sdk_integration/lib/go2_sdk_integration/ai_switcher " + network_interface_;
        RCLCPP_INFO(this->get_logger(), "Executing: %s", command.c_str());
        ai_req_result_ = std::system(command.c_str()) == 0;
        RCLCPP_INFO(this->get_logger(), "The dog is in ai mode.");
    }
    
    void switch_to_sport()
    {
        // creating, logging and executing command
        std::string command = "install/go2_sdk_integration/lib/go2_sdk_integration/sport_switcher " + network_interface_;
        RCLCPP_INFO(this->get_logger(), "Executing: %s", command.c_str());
        sport_req_result_ = std::system(command.c_str()) == 0;
        RCLCPP_INFO(this->get_logger(), "The dog is in sport mode.");
    }

    void state_callback(const unitree_go::msg::SportModeState::SharedPtr data)
    {
        if (data->position[2] < sitting_limit)
        {
            is_sitting_ = true;
            RCLCPP_INFO(this->get_logger(), "The dog is sat down.");
        }
    }

    // Service server
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

    // Publishers and Subscriptions 
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_suber_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber_;

    // Unitree Go ROS2 request message and sport client:
    unitree_api::msg::Request req_; 
    SportClient sport_req_;

    // Booleans for Mode and Position tracking
    bool is_sitting_;
    bool ai_req_result_;
    bool sport_req_result_;

    // Strings
    std::string network_interface_;
    std::string current_mode_;
    std::string mode_request_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionSwitcherServiceNode>());
    rclcpp::shutdown();
    return 0;
}
