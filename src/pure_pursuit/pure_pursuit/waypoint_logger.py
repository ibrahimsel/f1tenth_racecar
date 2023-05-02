#!/usr/bin/python3
import rclpy
import numpy as np
import atexit
import time
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from nav_msgs.msg import Odometry
import math
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped

home = '/home/labs'
file = open((home + '/rcws/logs/waypoints_file') + '.csv', 'w')

prev_waypoint = {'x': 0, 'y': 0}

def save_waypoint(data):
    dist = 0.0
    dx = data.pose.pose.position.x - prev_waypoint['x']
    dy = data.pose.pose.position.y - prev_waypoint['y']
    dist = (math.pow(dx, 2) + math.pow(dy, 2))
    if dist > 0.1:
        file.write(f'{data.pose.pose.position.x}, {data.pose.pose.position.y}\n')
        prev_waypoint['x'] = data.pose.pose.position.x
        prev_waypoint['y'] = data.pose.pose.position.y


def shutdown():
    file.close()
    print('Goodbye')
 
def listener():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('waypoint_logger')
    node.get_logger().info('Created waypoint_logger node')

    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', save_waypoint, 100)
    rclpy.spin(node)

if __name__ == '__main__':
    atexit.register(shutdown)
    print('Saving waypoints...')
    listener()
    # try:
    # except BaseException:
        # print("program closed unexpectedly")