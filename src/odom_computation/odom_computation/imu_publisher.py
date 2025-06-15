#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
import math

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        self.publisher = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(0.1, self.publish_imu)  # 10 Hz

        # Simulated data
        self.sim_angular_velocity = 0.0      # rad/s
        self.sim_linear_acceleration = 0.0   # m/s^2
        self.sim_orientation_angle = 0.0     # radians

    def publish_imu(self):
        imu_msg = Imu()

        # Time stamp
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'base_link'

        # Simulate rotation in yaw (theta)
        self.sim_orientation_angle += 0.01  # slowly turning

        q = quaternion_from_euler(0.0, 0.0, self.sim_orientation_angle)
        imu_msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # Simulate angular velocity around Z (yaw)
        imu_msg.angular_velocity.z = 0.1  # rad/s

        # Simulate forward acceleration
        imu_msg.linear_acceleration.x = 0.2  # m/s²

        self.publisher.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
