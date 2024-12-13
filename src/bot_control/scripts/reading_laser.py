#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class ReadingLaser(Node):
    def __init__(self):
        super().__init__('reading_laser')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(LaserScan, '/filtered_scan', 10)

    def listener_callback(self, msg):
        # for i, range in enumerate(msg.ranges):
            #self.get_logger().info('Range at index %d: %f' % (i, range))


        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = -math.pi/3  
        filtered_scan.angle_max = math.pi/3
        
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        start_index = int((filtered_scan.angle_min - msg.angle_min) / msg.angle_increment)  # start from angle -60 degree
        end_index = int((filtered_scan.angle_max - msg.angle_min) / msg.angle_increment)
        filtered_scan.ranges = msg.ranges[start_index:end_index]
        filtered_scan.intensities = msg.intensities[start_index:end_index]
        self.publisher.publish(filtered_scan)

def main(args=None):
    rclpy.init(args=args)
    reading_laser = ReadingLaser()
    rclpy.spin(reading_laser)
    reading_laser.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()