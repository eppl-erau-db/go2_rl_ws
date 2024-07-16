#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/motor_cmd.hpp"
#include "unitree_go/msg/bms_cmd.hpp"
#include "common/motor_crc.h" // Ensure this header is in your include path

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
        
        // Timer to regularly publish LowCmd messages
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Go2_RL_Control::publish_lowcmd, this));
        
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
        float target_positions[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65, 0.0, 1.36, -2.65, 0.0, 1.36, -2.65};
        for (int i = 0; i < 12; ++i)
        {
            cmd_msg_.motor_cmd[i].q = target_positions[i];
            cmd_msg_.motor_cmd[i].mode = 0x01;
            cmd_msg_.motor_cmd[i].dq = 0.0;
            cmd_msg_.motor_cmd[i].tau = 0.0;
            cmd_msg_.motor_cmd[i].kp = 5.0;
            cmd_msg_.motor_cmd[i].kd = 2.0;
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
            // Delay for 5 seconds before starting the walking commands
            timer_->cancel();
            rclcpp::sleep_for(std::chrono::seconds(5));
            stand_complete_ = true;
            timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Go2_RL_Control::publish_lowcmd, this));
            return;
        }

        if (!stand_complete_) return;

        int num_motors = 12;

        // Motor indices and their corresponding limits
        std::vector<int> motor_indices = {3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8};
        std::vector<std::pair<float, float>> motor_limits = {
            {-0.8377, 0.8377}, // Hip
            {-0.8377, 0.8377},
            {-0.8377, 0.8377},
            {-0.8377, 0.8377},
            {-3.490, 1.5708}, // Thigh
            {-3.490, 1.5708},
            {-3.490, 1.5708},
            {-3.490, 1.5708},
            {-2.723, -0.8377}, // Calf
            {-2.723, -0.8377},
            {-2.723, -0.8377},
            {-2.723, -0.8377}
        };

        // Map actions to motor commands
        for (int i = 0; i < num_motors; ++i)
        {
            int motor_index = motor_indices[i];
            float action_value = action_.empty() ? 0.0f : action_[i];

            float clamped_value = std::max(motor_limits[i].first, std::min(action_value, motor_limits[i].second));

            cmd_msg_.motor_cmd[motor_index].q = clamped_value; // Use clamped value
            cmd_msg_.motor_cmd[motor_index].mode = 0x01;
            cmd_msg_.motor_cmd[motor_index].dq = 0.0;
            cmd_msg_.motor_cmd[motor_index].tau = 0.0;
            cmd_msg_.motor_cmd[motor_index].kp = 1.0;
            cmd_msg_.motor_cmd[motor_index].kd = 0.0;
            cmd_msg_.motor_cmd[motor_index].reserve = {0, 0, 0};
        }

        // Check motor cmd CRC
        get_crc(cmd_msg_);

        RCLCPP_INFO(this->get_logger(), "Publishing lowcmd.");
        publisher_->publish(cmd_msg_);
    }

    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr publisher_; // Publisher for LowCmd messages
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_; // Subscription for action messages
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
