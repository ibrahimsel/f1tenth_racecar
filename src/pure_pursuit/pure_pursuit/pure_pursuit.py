#!/usr/bin/python3
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rclpy.node import Node
import rclpy
import numpy as np
import math


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        self.declare_parameter('L', 1.0)
        self.declare_parameter('min_velocity', 1.0)
        self.declare_parameter('max_velocity', 1.6)
        self.declare_parameter(
            'waypoints_file', '/home/labs/rcws/logs/waypoints_file.csv')
        self.waypoints_file = self.get_parameter(
            'waypoints_file').get_parameter_value().string_value
        self.L = self.get_parameter('L').get_parameter_value().double_value
        self.min_velocity = self.get_parameter('min_velocity').get_parameter_value().double_value
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)
        self.brake_sub = self.create_subscription(Bool, '/emergency_braking', self.brake_callback, 100)
        self.visualize_pub_marker = self.create_publisher(
            Marker, '/dynamic_env', 10)
        self.emergency_braking = False
        self.rate = self.create_rate(5)
        self.velocity = 0

        with open((self.waypoints_file), 'r') as wp_file:
            self.waypoints = []
            contents = wp_file.read()
            lines = contents.splitlines()
            for i in lines:  # iterate over every logged waypoint
                splitted = i.split(",")
                x = float(splitted[0])
                y = float(splitted[1])
                self.waypoints.append([y, x])

    def brake_callback(self, data):
        self.emergency_braking = data.data


    def distance_to_start(self, car_point):
        return math.dist(car_point, self.waypoints[0])
    
    def get_goal_waypoint(self, car_point):
        goal_waypoint = self.waypoints[1]
        distance_to_start = self.distance_to_start(car_point)
        if distance_to_start < 1:
            return self.waypoints[2]
        for i in self.waypoints:
            distance = math.dist(car_point, i)
            if (distance <= self.L):
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

    def odom_callback(self, odom_msg):
        self.speed_x = odom_msg.twist.twist.linear.x

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
        euclidean_dist = math.dist(goal_waypoint, [car_y, car_x])
        lookahead_angle = np.arctan2(
            goal_waypoint[0] - car_y, goal_waypoint[1] - car_x)
        delta_y = euclidean_dist * np.sin(lookahead_angle - heading_current)

        steering_angle = self.calculate_steering_angle(
            self.L, delta_y)
        self.velocity = self.calculate_velocity(lookahead_angle)
        self.get_logger().warn(f'velocity is: {self.velocity:.2f} | curv_angle is {lookahead_angle:.2f}')
        self.publish_steering(self.velocity, steering_angle)


    # calculates the curvature of the arc
    def calculate_steering_angle(self, L, y):
        return (2*y) / (math.pow(L, 2))

    def calculate_velocity(self, lookahead_angle):
        angle_min = -math.pi
        angle_max = math.pi
        self.velocity = self.value_mapper(lookahead_angle, angle_min, angle_max, self.min_velocity, self.max_velocity)
        if self.emergency_braking:
            return 0.0  # stop the car
        else:
            return self.velocity 

    def calculate_acceleration(self, final_velocity, initial_velocity, time):
        if self.emergency_braking:
            return (final_velocity - initial_velocity) / time # deceleration formula
        else:
            return 1.0  # standard acceleration
        
    def value_mapper(self, value, istart, istop, ostart, ostop):
        value = ostart + (ostop - ostart) * \
            ((value - istart) / (istop - istart))
        return round(value, 1)

    def publish_steering(self, speed, angle):
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.acceleration = 1.0
        msg.drive.steering_angle = angle
        msg.drive.jerk = 1.0
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    pp = PurePursuit()
    rclpy.spin(pp)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
