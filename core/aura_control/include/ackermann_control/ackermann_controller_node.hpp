#ifndef ACKERMANN_CONTROL__ACKERMANN_CONTROLLER_NODE_HPP_
#define ACKERMANN_CONTROL__ACKERMANN_CONTROLLER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>

namespace ackermann_control
{

class AckermannControllerNode : public rclcpp::Node
{
public:
    AckermannControllerNode();

private:
    // Callbacks & Functions
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void control_loop();

    // Parameters
    double wheelbase_;
    double track_width_;
    double wheel_radius_;
    double steering_limit_;
    double timeout_;

    // States
    double current_v_;
    double current_w_;

    // ROS 2 Interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steer_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_cmd_time_;
};

}  // namespace ackermann_control

#endif  // ACKERMANN_CONTROL__ACKERMANN_CONTROLLER_NODE_HPP_
