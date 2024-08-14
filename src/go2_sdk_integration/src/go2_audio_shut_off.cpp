#include "rclcpp/rclcpp.hpp"
#include <cstdlib>

class AudioShutOffNode : public rclcpp::Node
{
public:
    AudioShutOffNode() 
    : Node("audio_shutoff_node")
    {
        // network name parameter
        this->declare_parameter<std::string>("network_interface", "enp114s0");
        network_interface_ = this->get_parameter("network_interface").as_string();

        // calling shut off audio
        shut_off_audio();
    }

private:

    void shut_off_audio()
    {
        // creating, logging, and executing command
        std::string command = "unitree_sdk2/build/bin/shutdown_audio_client " + network_interface_;
        RCLCPP_INFO(this->get_logger(), "Executing: %s", command.c_str());
        int result = std::system(command.c_str());

        // handling and reporting error message
        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to execute shutdown_audio_client with error code: %d", result);
        }
        else 
        {
            RCLCPP_INFO(this->get_logger(), "The audio has been shut off.");
        }
    }
    
    std::string network_interface_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AudioShutOffNode>());
    rclcpp::shutdown();
    return 0;
}
