#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include "unitree_api/msg/request.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "common/ros2_sport_client.h"

using std::placeholders::_1;

class Go2_Nav_Deploy_Node : public rclcpp::Node
{
public:
    Go2_Nav_Deploy_Node() : Node("go2_nav_deploy")
    {
        // Publisher for sending commands to the robot
        command_publisher_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        
        // Subscriber for receiving navigation actions
        actions_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "nav_actions", 10, std::bind(&Go2_Nav_Deploy_Node::actions_callback, this, _1));

        // Sport client used for sending movement commands
        sport_client_ = std::make_shared<SportClient>();
    }

private:
    void actions_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // Extract linear and angular velocities
        float vx = msg->data[0];
        float vy = msg->data[1];
        float vyaw = msg->data[3];

        // Log received command
        RCLCPP_INFO(this->get_logger(), "Received velocity command: vx = %.2f, vy = %.2f, vyaw = %.2f", vx, vy, vyaw);

        // Prepare the request message
        unitree_api::msg::Request request_msg;
        sport_client_->Move(request_msg, vx, vy, vyaw);

        // Publish the request
        command_publisher_->publish(request_msg);
        RCLCPP_INFO(this->get_logger(), "Published movement command to Go2");
    }

    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr command_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr actions_subscriber_;
    std::shared_ptr<SportClient> sport_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Go2_Nav_Deploy_Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}