#include "rclcpp/rclcpp.hpp"
#include <cstdlib>

class MotionSwitcherNode : public rclcpp::Node
{
public:
    MotionSwitcherNode() : Node("motion_switcher_node")
    {
        this->declare_parameter<std::string>("network_interface", "enp114s0");

        std::string network_interface;
        this->get_parameter("network_interface", network_interface);

        std::string command = "lib/go2_sdk_integration/motion_switcher " + network_interface;

        RCLCPP_INFO(this->get_logger(), "Executing: %s", command.c_str());
        int result = std::system(command.c_str());

        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to execute motion_switcher with error code: %d", result);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionSwitcherNode>());
    rclcpp::shutdown();
    return 0;
}
