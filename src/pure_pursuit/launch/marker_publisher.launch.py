from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource

def generate_launch_description():
    wp_file = "/home/labs/rcws/logs/waypoints_file.csv"

    node_marker_publisher = Node(
        package="pure_pursuit",
        executable="marker_publisher",
        name="marker_publisher",
        output="screen",
        parameters=[
            {"wp_file": wp_file},
        ]
    )

    ld = LaunchDescription()
    ld.add_action(node_marker_publisher)
    return ld