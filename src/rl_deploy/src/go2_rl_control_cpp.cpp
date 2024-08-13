#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "common/motor_crc.h"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/motor_cmd.hpp"
#include <vector>

// Define stop positions and tolerance
constexpr double StandPos[] = {0.0, 1.1, -1.8, 0.0, 1.1, -1.8, 0.0, 1.1, -1.8, 0.0, 1.1, -1.8};
constexpr double SitPos[] = {-0.1, 1.1, -2.0, -0.1, 1.1, -2.0, -0.1, 1.1, -2.6, -0.1, 1.1, -2.6};
constexpr double position_tolerance = 0.3;

class Go2_RL_Control : public rclcpp::Node
{
public:
    Go2_RL_Control() 
        : Node("low_cmd_publisher"), 
        stand_command_(false), 
        sit_command_(false), 
        start_command_(false), 
        stop_walking_command_(false), 
        soft_abort_command_(false), 
        kill_command_(false), 
        good_sit_(false), 
        good_stand_(false), 
        is_walking_(false)
    {
        // Publisher
        publisher_ = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

        // Subscriptions
        actions_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "actions", 10, std::bind(&Go2_RL_Control::actions_callback, this, std::placeholders::_1));
        lowstate_subscription_ = this->create_subscription<unitree_go::msg::LowState>(
            "/lowstate", 10, std::bind(&Go2_RL_Control::lowstate_callback, this, std::placeholders::_1));
        buttons_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "buttons", 10, std::bind(&Go2_RL_Control::buttons_callback, this, std::placeholders::_1));

        // Publisher Timer (20ms)
        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&Go2_RL_Control::publish_lowcmd, this));

        // Initialize LowCmd message and Log
        init_cmd();
        RCLCPP_INFO(this->get_logger(), "Go2_RL_Control node has been started.");
    }

private:
    // Callback function to handle incoming action messages
    void actions_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        action_ = msg->data;
        publish_lowcmd();
    }

    // Callback function to handle incoming wireless controller messages
    void buttons_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (!stand_command_) // Up Button --> Stand Command, set to true until reset!
        {
            stand_command_ = true;
            if (msg->data[0] == 0)
            {
                stand_command_ = false;
            }
        }
        
        if (!sit_command_) // Down Button --> Sit Command, set to true until reset!
        {
            sit_command_ = true;
            if (msg->data[1] == 0)
            {
                sit_command_ = false;
            }
        }   
        
        if (!start_command_) // Start Button --> Start Command, only true if dog has stood, set to true until reset!
        {
            start_command_ = true;
            if (msg->data[2] == 0 || !good_stand_)
            {
                start_command_ = false;
            }
        }
        
        if (!stop_walking_command_) // Select Button --> Stop Walking Command (To Stand), only true if dog is walking, set to true until reset!
        {
            stop_walking_command_ = true;
            if (msg->data[3] == 0 || !is_walking_)
            {
                stop_walking_command_ = false;
            }
        }
        
        if (!soft_abort_command_) // A Button --> Soft Abort Command (Damping Mode), set to true until reset!
        {
            soft_abort_command_ = true;
            if(msg->data[4] == 0)
            {
                soft_abort_command_ = false;
            }
        }

        // B Button --> Kill and Rest Command, set to true until reset!
        kill_command_ = msg->data[5] == 1;


        // Loggers for debugging:
        if (stand_command_)
        {
            RCLCPP_INFO(this->get_logger(), "Stand command is true.");
        }

        if (sit_command_)
        {
            RCLCPP_INFO(this->get_logger(), "Sit command is true.");
        }

        if (start_command_)
        {
            RCLCPP_INFO(this->get_logger(), "Start command is true.");
        }

        if (stop_walking_command_)
        {
            RCLCPP_INFO(this->get_logger(), "Stop walking command is true.");
        }

        if (soft_abort_command_)
        {
            RCLCPP_INFO(this->get_logger(), "Soft abort command is true.");
        }

        if (kill_command_)
        {
            RCLCPP_INFO(this->get_logger(), "Kill command is true.");
        }
    }

    // Callback function to handle lowstate messages
    void lowstate_callback(const unitree_go::msg::LowState::SharedPtr msg)
    {
        // Store the latest motor state information
        motor_state_ = *msg;

        // Variables to track if all motors are in the desired position for standing and sitting
        bool all_standing = true;
        bool all_sitting = true;

        // Compare all motors against the desired positions
        for (int i = 0; i < 12; ++i)
        {
            // Compare each motor's current position and velocity with the desired standing position
            if (!(std::abs(msg->motor_state[i].q - StandPos[i]) < position_tolerance &&
                std::abs(msg->motor_state[i].dq - 0.0) < position_tolerance))
            {
                all_standing = false; // If any motor is not in the standing position, set to false
            }

            // Compare each motor's current position and velocity with the desired sitting position
            if (!(std::abs(msg->motor_state[i].q - SitPos[i]) < position_tolerance &&
                std::abs(msg->motor_state[i].dq - 0.0) < position_tolerance))
            {
                all_sitting = false; // If any motor is not in the sitting position, set to false
            }
        }

        // Update the status of the dog
        good_stand_ = all_standing;
        good_sit_ = all_sitting;

        // Good sit and good stand loggers
        if (good_stand_)
        {
            RCLCPP_INFO(this->get_logger(), "GOOD STAND.");
        }
        if (good_sit_)
        {
            sit_command_ = false;
            RCLCPP_INFO(this->get_logger(), "GOOD SIT.");
        }
    }

    // Function to initialize LowCmd message with default values
    void init_cmd()
    {
        for (int i = 0; i < 20; ++i)
        {
            cmd_msg_.motor_cmd[i].mode = 0x01;
            cmd_msg_.motor_cmd[i].q = PosStopF; 
            cmd_msg_.motor_cmd[i].kp = 0.0;
            cmd_msg_.motor_cmd[i].dq = VelStopF;
            cmd_msg_.motor_cmd[i].kd = 0.0;
            cmd_msg_.motor_cmd[i].tau = 0.0;
        }
    }

    // Function to handle the standing behavior
    void stand()
    {
        for (int i = 0; i < 12; ++i)
        {
            cmd_msg_.motor_cmd[i].q = StandPos[i]; // Map Stand positions
            cmd_msg_.motor_cmd[i].mode = 0x01;
            cmd_msg_.motor_cmd[i].dq = 0.0;
            cmd_msg_.motor_cmd[i].tau = 0.0;
            cmd_msg_.motor_cmd[i].kp = 50.0;
            cmd_msg_.motor_cmd[i].kd = 5.0;
        }

        // Check motor cmd CRC
        get_crc(cmd_msg_);
        RCLCPP_INFO(this->get_logger(), "Publishing stand command.");
        publisher_->publish(cmd_msg_);
    }

    // Function to handle the sit behavior -- TODO: ADD MORE DAMPING
    void sit()
    {
        for (int i = 0; i < 12; ++i)
        {
            cmd_msg_.motor_cmd[i].q = SitPos[i]; // Map sit positions
            cmd_msg_.motor_cmd[i].mode = 0x01; // working mode
            cmd_msg_.motor_cmd[i].dq = 0.0;
            cmd_msg_.motor_cmd[i].tau = 0.0;
            cmd_msg_.motor_cmd[i].kp = 30.0; // Lower to be able to slowly sit
            cmd_msg_.motor_cmd[i].kd = 10.0; // Higher to be able to slowly sit
        }

        // Check motor cmd CRC
        get_crc(cmd_msg_);
        RCLCPP_INFO(this->get_logger(), "Publishing sit command.");
        publisher_->publish(cmd_msg_);
    }

    // Function to handle damping/kill behaviour
    void kill()
    {
        for (int i = 0; i < 12; ++i)
        {
            cmd_msg_.motor_cmd[i].q = SitPos[i]; // Map sit positions
            cmd_msg_.motor_cmd[i].mode = 0x00; // standby mode
            cmd_msg_.motor_cmd[i].dq = 0.0;
            cmd_msg_.motor_cmd[i].tau = 0.0;
            cmd_msg_.motor_cmd[i].kp = 0.0;
            cmd_msg_.motor_cmd[i].kd = 0.0;
        }

        // Check motor cmd CRC
        get_crc(cmd_msg_);
        RCLCPP_INFO(this->get_logger(), "Publishing kill command.");
        publisher_->publish(cmd_msg_);
    }

    // Function to handle RL actions deployment
    void walk()
    {
        // Map actions to motor commands and clamp to 75% of the limits
        for (int i = 0; i < 12; ++i)
        {
            float action_value = action_.empty() ? 0.0f : action_[i];
            cmd_msg_.motor_cmd[i].q = action_value;
            cmd_msg_.motor_cmd[i].mode = 0x01;
            cmd_msg_.motor_cmd[i].dq = 0.0;
            cmd_msg_.motor_cmd[i].tau = 0.0;
            cmd_msg_.motor_cmd[i].kp = 20.0; // From sim --> 25.0 for rough
            cmd_msg_.motor_cmd[i].kd = 0.5; // From sim
            cmd_msg_.motor_cmd[i].reserve[0] = 0;
            cmd_msg_.motor_cmd[i].reserve[1] = 0;
            cmd_msg_.motor_cmd[i].reserve[2] = 0;
        }

        // Check motor cmd CRC
        get_crc(cmd_msg_);
        RCLCPP_INFO(this->get_logger(), "Publishing actions.");
        publisher_->publish(cmd_msg_);
    }

    // Function to handle the damping behavior
    void damping_position()
    {
        for (int i = 0; i < 12; ++i)
        {
            cmd_msg_.motor_cmd[i].q = motor_state_.motor_state[i].q;
            cmd_msg_.motor_cmd[i].mode = 0x01;
            cmd_msg_.motor_cmd[i].dq = 0.0;
            cmd_msg_.motor_cmd[i].tau = 0.0;
            cmd_msg_.motor_cmd[i].kp = 40.0;
            cmd_msg_.motor_cmd[i].kd = 5.0;
        }

        // Check motor cmd CRC
        get_crc(cmd_msg_);
        RCLCPP_INFO(this->get_logger(), "Publishing slow position command.");
        publisher_->publish(cmd_msg_);
    }

    // Function to publish LowCmd message
    void publish_lowcmd()
    {
        if (kill_command_) // Kill and Reset Command
        {
            // Status Tracking
            is_walking_ = false;

            // Commands tracking (reset all after emergency)
            stand_command_ = false;
            sit_command_ = false;
            start_command_ = false;
            stop_walking_command_ = false;
            soft_abort_command_ = false;
            kill_command_ = false;

            // Publishing command
            kill();
            return;
        }
        else if (stand_command_ && !soft_abort_command_ && !start_command_ && !sit_command_) // Standing Command
        {
            // Status Tracking 
            is_walking_ = false;

            // Commands tracking
            sit_command_ = false;
            stop_walking_command_ = false;
            start_command_ = false;
            kill_command_ = false; 
            
            // Publishing command_tester
            kill_command_ = false;

            // Publishing command
            stand();
            return;
        }
        else if (sit_command_ && !soft_abort_command_) // Sitting Command
        {
            // Status Tracking 
            is_walking_ = false;

            // Commands tracking
            stand_command_ = false;
            stop_walking_command_ = false;
            start_command_ = false;
            kill_command_ = false; 
            
            // Publishing command_tester
            kill_command_ = false;

            // Publishing command
            sit();
            return;
        }
        else if (start_command_ && !soft_abort_command_ && !stop_walking_command_) // Start Publishing Actions Command
        {
            // Status tracking 
            is_walking_ = true;

            // Commands tracking 
            stand_command_ = false;
            sit_command_ = false; 
            kill_command_ = false;
            
            // Publishing command
            walk();
            return;            
        }
        else if (soft_abort_command_) // Slowly Abort/Damping Command 
        {
            // Status tracking
            is_walking_ = false;

            // Commands tracking _tester
            start_command_ = false;
            soft_abort_command_ = false;
            stop_walking_command_ = false;
            sit_command_ = false;
            stand_command_ = false;

            // Publishing command
            damping_position();
            return;
        }
        else if (stop_walking_command_) // Stop Walking Command (To Stand Positions)
        {
            // Status tracking
            is_walking_ = false;

            // Commands tracking
            stand_command_ = true; // so it keeps standing!
            soft_abort_command_ = false;
            sit_command_ = false;
            start_command_ = false;
            kill_command_ = false;

            // Publishing command
            stand();
            return;
        }
        else 
        {
            damping_position();
            return;
        }
    }

    // Publishers and Subscriptions
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr actions_subscription_;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr buttons_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    unitree_go::msg::LowCmd cmd_msg_;
    unitree_go::msg::LowState motor_state_;
    std::vector<float> action_;

    // Commands
    bool stand_command_;
    bool sit_command_;
    bool start_command_;
    bool stop_walking_command_;
    bool soft_abort_command_;
    bool kill_command_;

    // Position and Status Tracking
    bool good_sit_;
    bool good_stand_;
    bool is_walking_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Go2_RL_Control>());
    rclcpp::shutdown();
    return 0;
}