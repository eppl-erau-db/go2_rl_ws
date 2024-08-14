#include "rclcpp/rclcpp.hpp"
#include <cstdlib>
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

constexpr double sitting_limit = 0.10; // (meters)

class SportSwitcherNode : public rclcpp::Node
{
public:
    SportSwitcherNode() 
    : Node("sport_switcher_node"),
    is_sitting_(false),
    called_switcher_(false)
    {
        // subscribing to the "sportmodestate" topic
        state_suber = this->create_subscription<unitree_go::msg::SportModeState>(
            "lf/sportmodestate", 10, std::bind(&SportSwitcherNode::state_callback, this, std::placeholders::_1));

        // sport mode request publisher and timer
        req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

        // network name parameter
        this->declare_parameter<std::string>("network_interface", "enp114s0");
        network_interface_ = this->get_parameter("network_interface").as_string();
    }

private:

    void switch_to_sport()
    {
        // creating, logging and executing command
        std::string command = "unitree_sdk2/build/bin/sport_switcher " + network_interface_;
        RCLCPP_INFO(this->get_logger(), "Executing: %s", command.c_str());
        int result = std::system(command.c_str());

        // handling and reporting error message
        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to execute sport_switcher with error code: %d", result);
        }
    }
    void state_callback(unitree_go::msg::SportModeState::SharedPtr data)
    {
        if (data->position[2] < sitting_limit)
        {
            is_sitting_ = true;
            RCLCPP_INFO(this->get_logger(), "The dog is sat down!");
        }
        else if (!called_switcher_)
        {
            sport_req.StandDown(req); // Call StandDown without assignment
            req_puber->publish(req);
            RCLCPP_INFO(this->get_logger(), "Sending StandDown request...");
            return;
        }

        if (is_sitting_ && !called_switcher_)
        {
            switch_to_sport();
            RCLCPP_INFO(this->get_logger(), "Calling sport switcher function!");
            called_switcher_ = true;
        }
    }

    // Publishers and Subscriptions 
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_suber;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;

    // Unitree Go ROS2 request message and sport client:
    unitree_api::msg::Request req; 
    SportClient sport_req;

    // Position tracking
    bool is_sitting_;
    bool called_switcher_;

    std::string network_interface_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SportSwitcherNode>());
    rclcpp::shutdown();
    return 0;
}
