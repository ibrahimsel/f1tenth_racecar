#!/usr/bin/python3
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

import sys
import rclpy
import numpy as np
import os
import math
from rclpy.node import Node

LOOKAHEAD_DISTANCE = 1.2
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


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        global waypoints
        self.waypoints = waypoints
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.steering_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)
        self.visualize_pub_marker = self.create_publisher(
            Marker, '/dynamic_env', 10)

    def distance_to_start(self, car_point):
        return math.dist(car_point, self.waypoints[0])

    def get_goal_waypoint(self, car_point):
        goal_waypoint = self.waypoints[1]
        distance_to_start = self.distance_to_start(car_point)
        if distance_to_start < 1:
            return self.waypoints[2]
        for i in self.waypoints:
            distance = math.dist(car_point, i)
            # below, we choose the furthest waypoint inside our lookahead distance
            # it chooses the furthest because it chooses the last waypoint inside our lookahead distance
            if (distance <= LOOKAHEAD_DISTANCE):
                # prev_waypoint = goal_waypoint
                goal_waypoint = i

        marker = Marker()
        marker.id = -2
        marker.header.frame_id = "map"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = goal_waypoint[1]
        marker.pose.position.y = goal_waypoint[0]
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        self.visualize_pub_marker.publish(marker)
        return goal_waypoint

    def pose_callback(self, pose_msg):
        car_x = pose_msg.pose.pose.position.x
        car_y = pose_msg.pose.pose.position.y
        goal_waypoint = self.get_goal_waypoint([car_y, car_x])

        quaternion = np.array([pose_msg.pose.pose.orientation.x,
                               pose_msg.pose.pose.orientation.y,
                               pose_msg.pose.pose.orientation.z,
                               pose_msg.pose.pose.orientation.w])

        siny_cosp = 2.0 * \
            (quaternion[3] * quaternion[2] + quaternion[0] * quaternion[1])
        cosy_cosp = 1.0 - 2.0 * \
            (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2])
        heading_current = np.arctan2(siny_cosp, cosy_cosp)
        # print(f"siny_cosp: {siny_cosp} / cosy_cosp: {cosy_cosp} / heading_current: {heading_current}")
        euclidean_dist = math.dist(goal_waypoint, [car_y, car_x])
        lookahead_angle = np.arctan2(
            goal_waypoint[0] - car_y, goal_waypoint[1] - car_x)
        delta_y = euclidean_dist * np.sin(lookahead_angle - heading_current)
        angle = self.calculate_steering_angle(
            LOOKAHEAD_DISTANCE, delta_y)
        velocity = self.calculate_velocity(angle)
        
        self.publish_steering(velocity, angle)

    # calculates the curvature of the arc
    def calculate_steering_angle(self, L, y):
        return (2*y) / (math.pow(L, 2))

    def calculate_velocity(self, angle):
        angle = abs(math.degrees(angle))

        # if angle > 20:
            # return 4.0
        # elif angle > 10:
            # return 5.0
        # else:
            # return 6.0

    def publish_steering(self, speed, angle):
        msg = AckermannDriveStamped()
        msg.drive.speed = 1.0
        msg.drive.acceleration = 1.0
        msg.drive.jerk = 1.0
        msg.drive.steering_angle = angle
        msg.drive.steering_angle_velocity = 1.0
        self.steering_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    pp = PurePursuit()
    rclpy.spin(pp)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
