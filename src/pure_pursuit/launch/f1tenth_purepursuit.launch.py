import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    f1tenth = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("f1tenth_stack"),
                "launch",
                "bringup_launch.py"
            )
        ])
    )

    pure_pursuit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("pure_pursuit"),
                "launch",
                "pure_pursuit.launch.py"
            )
        ])
    )

    marker_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("pure_pursuit"),
                "launch",
                "marker_publisher.launch.py"
            )
        ])
    )

    ld = LaunchDescription()
    ld.add_action(pure_pursuit)
    ld.add_action(marker_publisher)
    ld.add_action(f1tenth)
    return ld