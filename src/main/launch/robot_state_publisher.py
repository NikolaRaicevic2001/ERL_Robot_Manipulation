from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    # Declare a launch argument to store the URDF file path
    robot_description_path = DeclareLaunchArgument(
        'robot_description_path',
        default_value=LaunchConfiguration('robot_description_path', default=''),
        description='Path to the URDF file'
    )

    # Load the URDF file using the command substitution
    load_robot_description = Node(
        package='rosparam',
        executable='rosparam',
        output='screen',
        arguments=['load', LaunchConfiguration('robot_description_path')]
    )

    # Start the robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher'
    )

    # Create the launch description
    return LaunchDescription([
        robot_description_path,
        load_robot_description,
        robot_state_publisher
    ])

# ros2 launch main robot_state_publisher.py robot_description_path:=/home/erl-tianyu/ERL_Robot_Manipulation/ERL_Robot_Manipulation/src/robot_descriptions/xarm6_description/urdf/xarm6_robot.urdf

