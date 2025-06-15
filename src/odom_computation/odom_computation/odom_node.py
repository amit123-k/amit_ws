import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
import math

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        # Robot parameters
        self.wheel_radius = 0.065  # meters
        self.wheel_base = 0.21     # meters
        self.ticks_per_rev = 512
        self.ticks_to_rad = (2 * math.pi) / self.ticks_per_rev

        # Robot state
        self.left_ticks_prev = None
        self.right_ticks_prev = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.left_ticks = 0
        self.right_ticks = 0
        self.received_ticks = False

        # Subscribers
        self.create_subscription(Int32, '/left_wheel_ticks', self.left_callback, 10)
        self.create_subscription(Int32, '/right_wheel_ticks', self.right_callback, 10)

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.update_odometry)

    def left_callback(self, msg):
        self.left_ticks = msg.data
        self.received_ticks = True

    def right_callback(self, msg):
        self.right_ticks = msg.data
        self.received_ticks = True

    def update_odometry(self):
        if not self.received_ticks:
            self.get_logger().warn("Waiting for encoder ticks...")
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if self.left_ticks_prev is None or self.right_ticks_prev is None:
            self.left_ticks_prev = self.left_ticks
            self.right_ticks_prev = self.right_ticks
            self.last_time = current_time
            return

        delta_left = self.left_ticks - self.left_ticks_prev
        delta_right = self.right_ticks - self.right_ticks_prev

        self.left_ticks_prev = self.left_ticks
        self.right_ticks_prev = self.right_ticks
        self.last_time = current_time

        # Convert ticks to distance
        d_left = delta_left * self.ticks_to_rad * self.wheel_radius
        d_right = delta_right * self.ticks_to_rad * self.wheel_radius

        # Odometry logic
        d_center = (d_left + d_right) / 2.0
        delta_theta = (d_right - d_left) / self.wheel_base

        self.x += d_center * math.cos(self.theta + delta_theta / 2.0)
        self.y += d_center * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation (theta to quaternion)
        q = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # Velocity
        odom_msg.twist.twist.linear.x = d_center / dt
        odom_msg.twist.twist.angular.z = delta_theta / dt

        self.odom_pub.publish(odom_msg)

        # Debug info
        self.get_logger().info(f"x: {self.x:.3f}, y: {self.y:.3f}, θ: {math.degrees(self.theta):.2f}°")

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
