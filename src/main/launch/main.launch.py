#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Define configuration arguments
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')

    # Include Gazebo and MoveIt! setups from another package
    xarm6_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('xarm6_description'), 'launch', 'xarm6_rviz_display.launch.py'
            ])
        ),
        launch_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns
        }.items(),
    )

    # Combine all parts into a single LaunchDescription
    return LaunchDescription([
        xarm6_rviz_launch,
    ])

