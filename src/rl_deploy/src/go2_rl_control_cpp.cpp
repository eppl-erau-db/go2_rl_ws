#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "common/motor_crc.h"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/motor_cmd.hpp"
#include <vector>

// Define stop positions and velocities
constexpr double StandPos[] = {0.0, 1.1, -1.8, 0.0, 1.1, -1.8, 0.0, 1.1, -1.8, 0.0, 1.1, -1.8};
constexpr double StandVel[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
constexpr double position_tolerance = 0.2;

class Go2_RL_Control : public rclcpp::Node
{
public:
    Go2_RL_Control() : Node("low_cmd_publisher"), first_run_(true), stand_complete_(false)
    {
        // Publisher for LowCmd messages
        publisher_ = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

        // Subscription to actions topic
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "actions", 10, std::bind(&Go2_RL_Control::actions_callback, this, std::placeholders::_1));

        // Subscription to lowstate topic
        lowstate_subscription_ = this->create_subscription<unitree_go::msg::LowState>(
            "/lowstate", 10, std::bind(&Go2_RL_Control::lowstate_callback, this, std::placeholders::_1));

        // Timer to regularly publish LowCmd messages
        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&Go2_RL_Control::publish_lowcmd, this));

        // Initialize LowCmd message
        init_cmd();

        RCLCPP_INFO(this->get_logger(), "Go2_RL_Control node has been started.");
    }

private:
    // Callback function to handle incoming action messages
    void actions_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        action_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received new actions.");
        if (stand_complete_)
        {
            publish_lowcmd();
        }
    }

    // Callback function to handle lowstate messages
    void lowstate_callback(const unitree_go::msg::LowState::SharedPtr msg)
    {
        // Check if the stand command is complete once
        if (!stand_complete_)
        {
            stand_complete_ = true;
            for (int i = 0; i < 12; ++i)
            {
                if (std::abs(msg->motor_state[i].q - StandPos[i]) > position_tolerance ||
                    std::abs(msg->motor_state[i].dq - StandVel[i]) > position_tolerance)
                {
                    stand_complete_ = false;
                    break;
                }
            }
        }
    }

    // Function to initialize LowCmd message with default values
    void init_cmd()
    {
        for (int i = 0; i < 20; ++i)
        {
            cmd_msg_.motor_cmd[i].mode = 0x01; // Set torque mode, 0x00 is passive mode
            cmd_msg_.motor_cmd[i].q = PosStopF;
            cmd_msg_.motor_cmd[i].kp = 0;
            cmd_msg_.motor_cmd[i].dq = VelStopF;
            cmd_msg_.motor_cmd[i].kd = 0;
            cmd_msg_.motor_cmd[i].tau = 0;
        }
    }

    // Function to handle the standing behavior
    void stand()
    {
        for (int i = 0; i < 12; ++i)
        {
            cmd_msg_.motor_cmd[i].q = StandPos[i];
            cmd_msg_.motor_cmd[i].mode = 0x01;
            cmd_msg_.motor_cmd[i].dq = StandVel[i];
            cmd_msg_.motor_cmd[i].tau = 0.0;
            cmd_msg_.motor_cmd[i].kp = 40.0;
            cmd_msg_.motor_cmd[i].kd = 5.0;
        }

        // Check motor cmd CRC
        get_crc(cmd_msg_);
        RCLCPP_INFO(this->get_logger(), "Publishing stand command.");
        publisher_->publish(cmd_msg_);
    }

    // Function to publish LowCmd message
    void publish_lowcmd()
    {
        if (first_run_)
        {
            stand();
            first_run_ = false;
            return;
        }

        int num_motors = 12;

        // Motor limits corresponding to Unitree convention
        std::vector<std::pair<float, float>> motor_limits = {
            {-0.8377, 0.8377}, // Hip
            {-1.5708, 3.490},  // Thigh - Flipped
            {-2.723, -0.8377}, // Calf
            {-0.8377, 0.8377}, // Hip
            {-1.5708, 3.490},  // Thigh
            {-2.723, -0.8377}, // Calf
            {-0.8377, 0.8377}, // Hip
            {-0.5235, 3.490},  // Thigh
            {-2.723, -0.8377}, // Calf
            {-0.8377, 0.8377}, // Hip
            {-0.5235, 3.490},  // Thigh
            {-2.723, -0.8377}  // Calf
        };

        // Map actions to motor commands and clamp to 75% of the limits
        for (int i = 0; i < num_motors; ++i)
        {
            float action_value = action_.empty() ? 0.0f : action_[i];

            // Calculate 75% of the motor limit range
            float lower_limit = motor_limits[i].first;
            float upper_limit = motor_limits[i].second;

            float clamped_value = std::max(lower_limit, std::min(action_value, upper_limit));

            cmd_msg_.motor_cmd[i].q = clamped_value; // Use clamped value
            cmd_msg_.motor_cmd[i].mode = 0x01;
            cmd_msg_.motor_cmd[i].dq = 0.0;
            cmd_msg_.motor_cmd[i].tau = 0.0;
            cmd_msg_.motor_cmd[i].kp = 65.0;
            cmd_msg_.motor_cmd[i].kd = 5.0;
            cmd_msg_.motor_cmd[i].reserve = {0, 0, 0};
        }

        // Check motor cmd CRC
        get_crc(cmd_msg_);

        RCLCPP_INFO(this->get_logger(), "Publishing lowcmd.");
        publisher_->publish(cmd_msg_);
    }

    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr publisher_; // Publisher for LowCmd messages
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_; // Subscription for action messages
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_subscription_; // Subscription for lowstate messages
    rclcpp::TimerBase::SharedPtr timer_; // Timer for regular publishing
    unitree_go::msg::LowCmd cmd_msg_; // Unitree Go2 LowCmd message
    std::vector<float> action_; // Store the latest actions received
    bool first_run_;
    bool stand_complete_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // Initialize ROS2
    auto node = std::make_shared<Go2_RL_Control>(); // Create an instance of the Go2_RL_Control node
    rclcpp::spin(node); // Spin the node to process callbacks
    rclcpp::shutdown(); // Shutdown ROS2
    return 0;
}
