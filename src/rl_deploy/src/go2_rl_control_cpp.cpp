#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "common/motor_crc.h"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/motor_cmd.hpp"
#include <vector>

// Define stop positions and velocities
constexpr double StandPos[] = {0.0, 1.1, -1.8, 0.0, 1.1, -1.8, 0.0, 1.1, -1.8, 0.0, 1.1, -1.8};
constexpr double SitPos[] = {-0.1, 1.1, -2.5, -0.1, 1.1, -2.5, -0.1, 1.1, -2.5, -0.1, 1.1, -2.5};
constexpr double StandVel[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
constexpr double position_tolerance = 0.2;

class Go2_RL_Control : public rclcpp::Node
{
public:
    Go2_RL_Control() : Node("low_cmd_publisher"), stand_command_(false), sit_command_(false), start_command_(false), stop_command_(false), good_stand_(false), good_sit_(false)
    {
        // Publisher for LowCmd messages
        publisher_ = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

        // Subscription to actions topic
        actions_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "actions", 10, std::bind(&Go2_RL_Control::actions_callback, this, std::placeholders::_1));

        // Subscription to lowstate topic
        lowstate_subscription_ = this->create_subscription<unitree_go::msg::LowState>(
            "/lowstate", 10, std::bind(&Go2_RL_Control::lowstate_callback, this, std::placeholders::_1));

        // Subscription to the wireless controller buttons topic 
        buttons_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "buttons", 10, std::bind(&Go2_RL_Control::buttons_callback, this, std::placeholders::_1));

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
        publish_lowcmd();
    }

    // Callback function to handle incoming wireless controller messages
    void buttons_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // Stand and Sit Commands
        stand_command_ = msg->data[0] == 1;
        sit_command_ = msg->data[1] == 1;
        if (sit_command_ || stand_command_)
        {
            start_command_ = false;
        }

        // Check if the start button has been hit (reset)
        if (!start_command_)
        {
            start_command_ = true;
            if (msg->data[2] == 0)
            {
                start_command_ = false;
            }
        }

        // Check if the stop button has been hit with
        if (!stop_command_)
        {
            stop_command_ = true;
            if (msg->data[3] == 0)
            {
                stop_command_ = false;
                start_command_ = false;
            }
            
        }

        // Logger
        RCLCPP_INFO(this->get_logger(), "Received wireless message.");
    }

    // Callback function to handle lowstate messages
    void lowstate_callback(const unitree_go::msg::LowState::SharedPtr msg)
    {
        // Store the latest motor state information
        motor_state_ = *msg;

        // Check if the stand command is complete once
        if (!good_stand_)
        {
            good_stand_ = true;
            for (int i = 0; i < 12; ++i)
            {
                if (std::abs(msg->motor_state[i].q - StandPos[i]) > position_tolerance ||
                    std::abs(msg->motor_state[i].dq - StandVel[i]) > position_tolerance)
                {
                    good_stand_ = false;
                    break;
                }
            }
        }

        // Check if the sit command is complete once
        if (!good_sit_)
        {
            good_sit_ = true;
            for (int i = 0; i < 12; ++i)
            {
                if (std::abs(msg->motor_state[i].q - SitPos[i]) > position_tolerance ||
                    std::abs(msg->motor_state[i].dq - StandVel[i]) > position_tolerance)
                {
                    good_sit_ = false;
                    break;
                }
            }
        }

        if (good_stand_)
        {
            RCLCPP_INFO(this->get_logger(), "GOOD STAND.");
        }

        if (good_sit_)
        {
            RCLCPP_INFO(this->get_logger(), "GOOD SIT.");
        }
    }

    // Function to initialize LowCmd message with default values
    void init_cmd()
    {
        for (int i = 0; i < 20; ++i)
        {
            cmd_msg_.motor_cmd[i].mode = 0x01; // 0x00 is passive mode
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
            cmd_msg_.motor_cmd[i].kp = 50.0;
            cmd_msg_.motor_cmd[i].kd = 5.0;
        }

        // Check motor cmd CRC
        get_crc(cmd_msg_);
        RCLCPP_INFO(this->get_logger(), "Publishing stand command.");
        publisher_->publish(cmd_msg_);
    }

    // Function to handle the sit behavior 
    void sit()
    {
        for (int i = 0; i < 12; ++i)
        {
            cmd_msg_.motor_cmd[i].q = SitPos[i];
            cmd_msg_.motor_cmd[i].mode = 0x01;
            cmd_msg_.motor_cmd[i].dq = StandVel[i];
            cmd_msg_.motor_cmd[i].tau = 0.0;
            cmd_msg_.motor_cmd[i].kp = 40.0;
            cmd_msg_.motor_cmd[i].kd = 5.0;    
        }

        // Check motor cmd CRC
        get_crc(cmd_msg_);
        RCLCPP_INFO(this->get_logger(), "Publishing sit command.");
        publisher_->publish(cmd_msg_);
    }

    // Function to handle the holding behavior (maintaining current positions)
    void hold_position()
    {
        for (int i = 0; i < 12; ++i)
        {
            cmd_msg_.motor_cmd[i].q = motor_state_.motor_state[i].q; // Maintain the current position
            cmd_msg_.motor_cmd[i].mode = 0x01;
            cmd_msg_.motor_cmd[i].dq = StandVel[i];
            cmd_msg_.motor_cmd[i].tau = 0.0;
            cmd_msg_.motor_cmd[i].kp = 40.0;
            cmd_msg_.motor_cmd[i].kd = 5.0;
        }

    // Check motor cmd CRC
    get_crc(cmd_msg_);
    RCLCPP_INFO(this->get_logger(), "Publishing hold position command.");
    publisher_->publish(cmd_msg_);
    }

    // Function to publish LowCmd message
    void publish_lowcmd()
    {
        if (sit_command_ && !good_sit_)
        {
            sit();
            start_command_ = false;
            return;
        }

        if (start_command_)
        {
            // Publish actions if start command is pressed
            int num_motors = 12;

            // Motor limits corresponding to Unitree convention
            std::vector<std::pair<float, float>> motor_limits = {
                {-0.8377, 0.8377}, // Hip
                {-1.5708, 3.490},  // Thigh - Flipped
                {-2.723, -0.8377}, // Calf
                {-0.8377, 0.8377}, // Hip
                {-1.5708, 3.490},  // Thigh - Flipped
                {-2.723, -0.8377}, // Calf
                {-0.8377, 0.8377}, // Hip
                {-0.5235, 3.490},  // Thigh - Flipped
                {-2.723, -0.8377}, // Calf
                {-0.8377, 0.8377}, // Hip
                {-0.5235, 3.490},  // Thigh -  Flipped
                {-2.723, -0.8377}  // Calf
            };

            // Map actions to motor commands and clamp to 75% of the limits
            for (int i = 0; i < num_motors; ++i)
            {
                float action_value = action_.empty() ? 0.0f : action_[i];

                // Calculate 75% of the motor limit range
                float lower_limit = motor_limits[i].first * 0.95f;
                float upper_limit = motor_limits[i].second * 0.95f;

                float clamped_value = std::max(lower_limit, std::min(action_value, upper_limit));

                cmd_msg_.motor_cmd[i].q = clamped_value; // Use clamped value
                cmd_msg_.motor_cmd[i].mode = 0x01;
                cmd_msg_.motor_cmd[i].dq = 0.0;
                cmd_msg_.motor_cmd[i].tau = 0.0;
                cmd_msg_.motor_cmd[i].kp = 60.0;
                cmd_msg_.motor_cmd[i].kd = 5.0;
                cmd_msg_.motor_cmd[i].reserve[0] = 0;
                cmd_msg_.motor_cmd[i].reserve[1] = 0;
                cmd_msg_.motor_cmd[i].reserve[2] = 0;
            }

            // Check motor cmd CRC
            get_crc(cmd_msg_);
            RCLCPP_INFO(this->get_logger(), "Publishing actions.");
            publisher_->publish(cmd_msg_);
        }
        else 
        {
            stand();
        }
    }

    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr actions_subscription_;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr buttons_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    unitree_go::msg::LowCmd cmd_msg_;
    unitree_go::msg::LowState motor_state_;
    std::vector<float> action_;
    bool stand_command_;
    bool sit_command_;
    bool start_command_;
    bool stop_command_;
    bool good_sit_;
    bool good_stand_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Go2_RL_Control>());
    rclcpp::shutdown();
    return 0;
}