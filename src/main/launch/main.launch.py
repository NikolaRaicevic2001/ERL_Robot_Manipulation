#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc>

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Define configuration arguments
    config_args = {
        'controllers_config': PathJoinSubstitution([
            FindPackageShare('main'), 'config', 'controllers.yaml'
        ])
    }

    # Declare arguments for easier modification
    declare_prefix_arg = DeclareLaunchArgument('prefix', default_value='')
    declare_hw_ns_arg = DeclareLaunchArgument('hw_ns', default_value='xarm')
    
    # Include Gazebo and MoveIt! setups from another package
    robot_moveit_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('xarm_gazebo'), 'launch', 'xarm6_beside_table_gazebo.launch.py'
            ])
        ]),
        launch_arguments={'prefix': LaunchConfiguration('prefix'),
                          'hw_ns': LaunchConfiguration('hw_ns')}.items(),
    )

    # Node to load controller configurations from the YAML file
    load_controllers = Node(
        package='controller_manager',
        executable='spawner',
        name='controller_spawner',
        namespace=LaunchConfiguration('hw_ns'),
        output='screen',
        parameters=[config_args['controllers_config']],
        arguments=['--ros-args', '--params-file', config_args['controllers_config']]
    )

    # Logging for debugging
    log_info = LogInfo(
        msg=["Using controllers configuration from: ", config_args['controllers_config']]
    )

    # Combine all parts into a single LaunchDescription
    return LaunchDescription([
        declare_prefix_arg,
        declare_hw_ns_arg,
        log_info,
        robot_moveit_gazebo_launch,
        load_controllers
    ])

