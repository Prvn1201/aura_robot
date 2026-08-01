#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import sys

class AckermannTeleop(Node):
    def __init__(self):
        super().__init__('ackermann_teleop')
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.timer_callback) # 20Hz

        # --- ACKERMANN LIMITS ---
        self.max_speed = 1.0       # Maximum forward/reverse speed in m/s
        self.max_steer = 0.5       # Maximum steering angle in radians

        # --- SMOOTH ACCELERATION & DECAY RATES ---
        # How much the value changes every 0.05 seconds
        self.speed_accel = 0.05    # Takes ~1.0 second to reach max speed
        self.speed_decay = 0.10    # Slows down faster than it accelerates when key is released
        
        self.steer_accel = 0.04    # Takes ~0.6 seconds to hit full lock steering
        self.steer_decay = 0.08    # Auto-centers the steering wheel quickly when released

        # --- CURRENT STATE ---
        self.current_speed = 0.0
        self.current_steer = 0.0

        # Initialize the Pygame window
        pygame.init()
        pygame.display.set_caption('Smooth Teleop (W/A/S/D)')
        self.screen = pygame.display.set_mode((400, 200))
        
        self.get_logger().info('Smooth Ackermann Teleop ready!')
        self.get_logger().info('Hold keys to gradually increase speed/steering (like a joystick).')

    def timer_callback(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rclpy.shutdown()
                sys.exit(0)

        keys = pygame.key.get_pressed()

        # ----------------------------------------------------
        # 1. SPEED LOGIC (W / S)
        # ----------------------------------------------------
        if keys[pygame.K_w]:
            self.current_speed += self.speed_accel
        elif keys[pygame.K_s]:
            self.current_speed -= self.speed_accel
        else:
            # Friction / braking when no key is pressed
            if self.current_speed > 0:
                self.current_speed = max(0.0, self.current_speed - self.speed_decay)
            elif self.current_speed < 0:
                self.current_speed = min(0.0, self.current_speed + self.speed_decay)

        # Clamp speed so it doesn't exceed max limits
        self.current_speed = max(-self.max_speed, min(self.max_speed, self.current_speed))

        # ----------------------------------------------------
        # 2. STEERING LOGIC (A / D)
        # ----------------------------------------------------
        if keys[pygame.K_a]:
            self.current_steer += self.steer_accel   # Turn Left
        elif keys[pygame.K_d]:
            self.current_steer -= self.steer_accel   # Turn Right
        else:
            # Auto-center steering wheel when no key is pressed
            if self.current_steer > 0:
                self.current_steer = max(0.0, self.current_steer - self.steer_decay)
            elif self.current_steer < 0:
                self.current_steer = min(0.0, self.current_steer + self.steer_decay)

        # Clamp steering so it doesn't exceed max angle limits
        self.current_steer = max(-self.max_steer, min(self.max_steer, self.current_steer))

        # ----------------------------------------------------
        # 3. PUBLISH
        # ----------------------------------------------------
        twist = Twist()
        twist.linear.x = float(self.current_speed)
        twist.angular.z = float(self.current_steer)
        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        pygame.quit()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
