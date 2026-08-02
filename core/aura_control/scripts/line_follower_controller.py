#!/usr/bin/env python3
"""
Line Follower PID Controller
Subscribes to:
  - /lane_error (std_msgs/Float32)
  - /lane_lost  (std_msgs/Bool)
Publishes to:
  - /cmd_vel_auto (geometry_msgs/Twist)  <-- Multiplexed via twist_mux
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist

LANE_ERROR_TOPIC = '/lane_error'
LANE_LOST_TOPIC = '/lane_lost'


class LineFollowerController(Node):
    def __init__(self):
        super().__init__('line_follower_controller')

        # Control Parameters (Tune these!)
        self.declare_parameter('kp', 1.2)
        self.declare_parameter('kd', 0.3)
        self.declare_parameter('base_speed', 0.4)  # m/s

        self.kp = float(self.get_parameter('kp').value)
        self.kd = float(self.get_parameter('kd').value)
        self.base_speed = float(self.get_parameter('base_speed').value)

        self.last_error = 0.0
        self.lane_lost = False

        # Subscriptions
        self.create_subscription(Float32, LANE_ERROR_TOPIC, self.error_callback, 10)
        self.create_subscription(Bool, LANE_LOST_TOPIC, self.lane_lost_callback, 10)

        # Output to twist_mux autonomous channel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_auto', 10)

        self.get_logger().info('Line Follower Controller Node started (publishing to /cmd_vel_auto).')

    def lane_lost_callback(self, msg: Bool):
        self.lane_lost = msg.data

    def error_callback(self, msg: Float32):
        twist = Twist()

        if self.lane_lost:
            self.get_logger().warn('Lane lost! Stopping vehicle.', throttle_duration_sec=2.0)
            self.cmd_pub.publish(twist)  # 0 velocity
            return

        error = float(msg.data)
        d_error = error - self.last_error
        self.last_error = error

        # PD steering calculation
        steering_angular = -(self.kp * error + self.kd * d_error)

        twist.linear.x = self.base_speed
        twist.angular.z = steering_angular

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
