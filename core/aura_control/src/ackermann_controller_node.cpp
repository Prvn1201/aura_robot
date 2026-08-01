#include "ackermann_control/ackermann_controller_node.hpp"
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

namespace ackermann_control
{

AckermannControllerNode::AckermannControllerNode()
: Node("ackermann_controller_node"), current_v_(0.0), current_w_(0.0)
{
    // 1. Declare & Load Parameters
    this->declare_parameter("wheelbase", 0.220);
    this->declare_parameter("track_width", 0.150);
    this->declare_parameter("wheel_radius", 0.0325);
    this->declare_parameter("steering_limit", 0.35);
    this->declare_parameter("timeout", 0.50);

    wheelbase_ = this->get_parameter("wheelbase").as_double();
    track_width_ = this->get_parameter("track_width").as_double();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    steering_limit_ = this->get_parameter("steering_limit").as_double();
    timeout_ = this->get_parameter("timeout").as_double();

    RCLCPP_INFO(this->get_logger(), 
        "Ackermann Controller Node started (wheelbase=%.3fm track_width=%.3fm wheel_radius=%.4fm steering_limit=%.2frad timeout=%.2fs)",
        wheelbase_, track_width_, wheel_radius_, steering_limit_, timeout_);

    // 2. Setup ROS 2 Interfaces
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&AckermannControllerNode::cmd_vel_callback, this, std::placeholders::_1));

    steer_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/front_steering_position_controller/commands", 10);
    wheel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/rear_wheel_velocity_controller/commands", 10);

    // 3. Setup Watchdog & Control Loop
    last_cmd_time_ = this->now();
    timer_ = this->create_wall_timer(
        20ms, std::bind(&AckermannControllerNode::control_loop, this));
}

void AckermannControllerNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    current_v_ = msg->linear.x;
    current_w_ = msg->angular.z;
    last_cmd_time_ = this->now(); // Reset watchdog
}

void AckermannControllerNode::control_loop()
{
    // 1. WATCHDOG: Stop if no recent command
    auto now = this->now();
    if ((now - last_cmd_time_).seconds() > timeout_) {
        current_v_ = 0.0;
        current_w_ = 0.0;
    }

    // 2. REAR WHEEL KINEMATICS
    double target_wheel_vel = current_v_ / wheel_radius_;

    // 3. TRUE ACKERMANN STEERING
    double steer_left = 0.0;
    double steer_right = 0.0;

    if (std::abs(current_v_) > 0.001 && std::abs(current_w_) > 0.001) {
        double radius = current_v_ / current_w_;
        steer_left = std::atan(wheelbase_ / (radius - (track_width_ / 2.0)));
        steer_right = std::atan(wheelbase_ / (radius + (track_width_ / 2.0)));
    } else if (std::abs(current_w_) > 0.001) {
        double max_steer = (current_w_ > 0) ? steering_limit_ : -steering_limit_;
        steer_left = max_steer;
        steer_right = max_steer;
    }

    // 4. CLAMP STEERING LIMITS
    steer_left = std::clamp(steer_left, -steering_limit_, steering_limit_);
    steer_right = std::clamp(steer_right, -steering_limit_, steering_limit_);

    // 5. PUBLISH COMMANDS
    std_msgs::msg::Float64MultiArray steer_msg;
    std_msgs::msg::Float64MultiArray wheel_msg;

    steer_msg.data = {steer_left, steer_right};
    steer_pub_->publish(steer_msg);

    wheel_msg.data = {target_wheel_vel, target_wheel_vel};
    wheel_pub_->publish(wheel_msg);
}

}  // namespace ackermann_control
