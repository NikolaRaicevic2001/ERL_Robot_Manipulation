import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Set the path to this package.
    pkg_share = FindPackageShare(package='xarm6').find('xarm6')

    # Set the path to the Xacro file
    xacro_file_path = os.path.join(pkg_share, 'urdf', 'xarm6.urdf.xacro')
    print(f"Xacro file path: {xacro_file_path}")

    # Set the path to the RViz configuration settings
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'display.rviz')
    print(f"RViz config path: {default_rviz_config_path}")

    # Specify the actions

    # Convert Xacro file to URDF
    xacro_to_urdf_cmd = ExecuteProcess(
        cmd=['xacro', xacro_file_path],
        output='screen',
        shell=True,
        name='xacro_to_urdf'
    )

    # Launch RViz with the generated URDF file
    start_rviz_cmd = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path, '-f', LaunchConfiguration('rviz_output_frame')],
        parameters=[{'robot_description': Command(['cat', LaunchConfiguration('urdf_file')])}]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ'
    ))
    ld.add_action(DeclareLaunchArgument(
        name='rviz_output_frame',
        default_value='base_link',
        description='The TF frame that RViz should use as the fixed frame'
    ))

    # Add any actions
    ld.add_action(xacro_to_urdf_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
