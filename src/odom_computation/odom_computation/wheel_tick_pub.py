#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import math

class WheelTickPublisher(Node):
    def __init__(self):
        super().__init__('wheel_tick_publisher')

        # Parameters
        self.wheel_radius = 0.065  # meters
        self.wheel_base = 0.21     # meters
        self.ticks_per_rev = 512   # ticks per wheel revolution

        # Motion parameters
        self.linear_velocity = 0.2   # m/s
        self.angular_velocity = 0.0  # rad/s

        # Encoder state
        self.left_ticks = 0
        self.right_ticks = 0

        # Publishers
        self.left_pub = self.create_publisher(Int32, '/left_wheel_ticks', 10)
        self.right_pub = self.create_publisher(Int32, '/right_wheel_ticks', 10)

        # Timer (10 Hz)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        dt = 0.1  # 10 Hz

        # Compute individual wheel velocities
        v_left = self.linear_velocity - (self.angular_velocity * self.wheel_base / 2.0)
        v_right = self.linear_velocity + (self.angular_velocity * self.wheel_base / 2.0)

        left_dist = v_left * dt
        right_dist = v_right * dt

        ticks_per_meter = self.ticks_per_rev / (2 * math.pi * self.wheel_radius)

        self.left_ticks += int(left_dist * ticks_per_meter)
        self.right_ticks += int(right_dist * ticks_per_meter)

        # Publish messages
        self.left_pub.publish(Int32(data=self.left_ticks))
        self.right_pub.publish(Int32(data=self.right_ticks))

def main(args=None):
    rclpy.init(args=args)
    node = WheelTickPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
