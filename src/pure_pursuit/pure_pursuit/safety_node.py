#!/usr/bin/python3
import rclpy
import numpy as np
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.speed = 0.0
        self.scan_subscription = self.create_subscription( # Subscribe to the scan topic
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.aeb_publisher = self.create_publisher(Bool, '/emergency_braking', 100)

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # Calculate TTC
        emergency_braking = False
        threshold = 0.50  # should be tuned for certain needs
        a = int((math.radians(-10) - scan_msg.angle_min) / scan_msg.angle_increment)
        b = int((math.radians(10) - scan_msg.angle_min) / scan_msg.angle_increment)
        
        ranges = scan_msg.ranges
        for idx, r in enumerate(ranges):
            if idx < a or idx > b:
                continue
            # self.get_logger().info(f"idx: {idx}, distance: {r}")
            if (np.isnan(r)or r > scan_msg.range_max or r < scan_msg.range_min): continue
            min_TTC = r / max(self.speed * np.cos(scan_msg.angle_min + idx * scan_msg.angle_increment), 0.2)
            if min_TTC < threshold: 
                self.get_logger().warn(f'min_TTC is {min_TTC}, threshold is {threshold}')
                emergency_braking = True
                break

        emergency_msg = Bool()
        emergency_msg.data = emergency_braking
        self.aeb_publisher.publish(emergency_msg)
        

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()