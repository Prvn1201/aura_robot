#include "ackermann_control/ackermann_controller_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ackermann_control::AckermannControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
