import os
import sys
from shutil import move

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchDescriptionSource
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = "vr_ur_t42"
    pkg_share = get_package_share_directory(pkg_name)

    vr_publish = Node(package='vr_publisher', 
                executable='vr_publish', 
                output='screen')

    visual = Node(package='vr_controller', 
                executable='rviz_vr_visual', 
                output='screen')

    servo_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'vr_controller.launch.py'),
        ),)

    return LaunchDescription([
        vr_publish,
        visual,
        TimerAction(
            period=5.,
            actions=[
                servo_controller
            ]
        )
    ])
