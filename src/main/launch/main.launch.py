from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the xarm_gazebo launch file
    xarm_gazebo_launch_file = os.path.join(
        get_package_share_directory('xarm_gazebo'),'launch','xarm6_beside_table_gazebo.launch.py')

    # Include the specified launch file
    xarm_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(xarm_gazebo_launch_file),launch_arguments={'some_argument': 'some_value'}.items())

    # Return the LaunchDescription object
    return LaunchDescription([
        xarm_gazebo_launch
    ])
