from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource

def generate_launch_description():
    wp_file = "/home/labs/rcws/logs/waypoints_file.csv"


    node_pure_pursuit = Node(
        package="pure_pursuit",
        executable="pure_pursuit",
        name="pure_pursuit",
        output="screen",
        parameters=[
            {"L": 1.2},
            {"speed": 1.0},
            {"wp_file": wp_file},
        ]
    )

    # Create LaunchDescription object and add nodes to launch
    ld = LaunchDescription()
    ld.add_action(node_pure_pursuit)
    return ld