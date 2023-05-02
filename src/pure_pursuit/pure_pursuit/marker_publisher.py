#!/usr/bin/python3
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
import sys
import rclpy
import numpy as np
import os
import math
from rclpy.node import Node

waypoints_file = open(
    ('/home/labs/rcws/logs/waypoints_file.csv'), 'r')
waypoints = []
contents = waypoints_file.read()
lines = contents.splitlines()
for i in lines:  # iterate over every logged waypoint
    splitted = i.split(",")
    x = float(splitted[0])
    y = float(splitted[1])
    waypoints.append([y, x])
waypoints_file.close()

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        global waypoints
        self.visualize_pub_markerarray = self.create_publisher(
            MarkerArray, '/static_env', 10)
        while rclpy.ok():
            self.marker_array = MarkerArray()
            ct = 0  # counter for marker ids
            for i in waypoints:
                marker = Marker()
                marker.id = ct
                ct += 1
                marker.header.frame_id = "map"
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = i[1]
                marker.pose.position.y = i[0]
                marker.pose.position.z = 0.0
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                self.marker_array.markers.append(marker)
            self.visualize_pub_markerarray.publish(self.marker_array)


def main(args=None):
    rclpy.init(args=args)
    mp = MarkerPublisher()
    rclpy.spin(mp)
    mp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
