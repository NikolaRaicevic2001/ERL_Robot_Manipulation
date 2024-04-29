#!/usr/bin/env python3

import os
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import load_python_launch_file_as_module
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.actions import OpaqueFunction
from launch.events import Shutdown
    
def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=True)
    velocity_control = LaunchConfiguration('velocity_control', default=True)
    add_gripper = LaunchConfiguration('add_gripper', default=True)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    dof = LaunchConfiguration('dof', default=6)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='gazebo_ros2_control/GazeboSystem')
    
    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
    add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
    model1300 = LaunchConfiguration('model1300', default=False)
    robot_sn = LaunchConfiguration('robot_sn', default='')
    attach_to = LaunchConfiguration('attach_to', default='world')                        
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')

    add_other_geometry = LaunchConfiguration('add_other_geometry', default=True)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')
    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')
    
    load_controller = LaunchConfiguration('load_controller', default=True)
    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)

    # ros2 control params
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm6_description'), 'launch', 'lib', 'robot_controller_lib.py'))
    generate_ros2_control_params_temp_file = getattr(mod, 'generate_ros2_control_params_temp_file')
    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm6_description'), 'controller', 'xarm6_controllers.yaml'),
        prefix=prefix.perform(context), 
        add_gripper=add_gripper.perform(context) in ('True', 'true'),
        add_bio_gripper=add_bio_gripper.perform(context) in ('True', 'true'),
        ros_namespace=LaunchConfiguration('ros_namespace', default='').perform(context),
        update_rate=1000,
    )

    # robot_description
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm6_description'), 'launch', 'lib', 'robot_description_lib.py'))
    get_xacro_file_content = getattr(mod, 'get_xacro_file_content')
    robot_description = {
        'robot_description': get_xacro_file_content(
            xacro_file=PathJoinSubstitution([FindPackageShare('xarm6_description'), 'urdf', 'xarm_device.urdf.xacro']), 
            arguments={
                'prefix': prefix,
                'dof': dof,
                'robot_type': robot_type,
                'add_gripper': add_gripper,
                'add_vacuum_gripper': add_vacuum_gripper,
                'add_bio_gripper': add_bio_gripper,
                'hw_ns': hw_ns.perform(context).strip('/'),
                'limited': limited,
                'effort_control': effort_control,
                'velocity_control': velocity_control,
                'ros2_control_plugin': ros2_control_plugin,
                'ros2_control_params': ros2_control_params,
                'add_realsense_d435i': add_realsense_d435i,
                'add_d435i_links': add_d435i_links,
                'model1300': model1300,
                'robot_sn': robot_sn,
                'attach_to': attach_to,
                'attach_xyz': attach_xyz,
                'attach_rpy': attach_rpy,
                'add_other_geometry': add_other_geometry,
                'geometry_type': geometry_type,
                'geometry_mass': geometry_mass,
                'geometry_height': geometry_height,
                'geometry_radius': geometry_radius,
                'geometry_length': geometry_length,
                'geometry_width': geometry_width,
                'geometry_mesh_filename': geometry_mesh_filename,
                'geometry_mesh_origin_xyz': geometry_mesh_origin_xyz,
                'geometry_mesh_origin_rpy': geometry_mesh_origin_rpy,
                'geometry_mesh_tcp_xyz': geometry_mesh_tcp_xyz,
                'geometry_mesh_tcp_rpy': geometry_mesh_tcp_rpy,
                'kinematics_suffix': kinematics_suffix,
            }
        ),
    }

    # robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}, robot_description],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # gazebo launch
    # gazebo_ros/launch/gazebo.launch.py
    xarm_gazebo_world = PathJoinSubstitution([FindPackageShare('xarm6_description'), 'worlds', 'table.world'])
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])),
        launch_arguments={
            'world': xarm_gazebo_world,
            'server_required': 'true',
            'gui_required': 'true',
        }.items(),
    )

    # gazebo spawn entity node
    gazebo_spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            # '-entity', '{}{}'.format(robot_type.perform(context), dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else ''),
            '-entity', 'UF_ROBOT',
            '-x', '-0.2',
            '-y', '-0.57',
            '-z', '1.021',
            '-Y', '1.571',
        ],
        parameters=[{'use_sim_time': True}],
    )

    gazebo_spawn_entity_node_box = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output='screen',
        arguments=[
            '-entity', 'box',  # Assuming 'box' as the entity name
            '-file', PathJoinSubstitution([FindPackageShare('xarm6_description'), 'models', 'box.sdf']),
            '-x', '0.2',
            '-y', '1.0',
            '-z', '0.0',
            '-Y', '0.0'
        ],
        parameters=[{'use_sim_time': True}],
    )

    # gazebo_spawn_entity_node_camera = Node(
    #     package="gazebo_ros",
    #     executable="spawn_entity.py",
    #     output='screen',
    #     arguments=[
    #         '-entity', 'camera',  # Assuming 'box' as the entity name
    #         '-file', PathJoinSubstitution([FindPackageShare('xarm6_description'), 'urdf','camera','_d435.gazebo.xacro']),
    #         '-x', '2.2',
    #         '-y', '1.0',
    #         '-z', '0.0',
    #         '-Y', '0.0'
    #     ],
    #     parameters=[{'use_sim_time': True}],
    #     # remappings=[('/tf', 'tf'),('/tf_static', 'tf_static'),]
    # )

    # rviz2 node
    rviz2_params = PathJoinSubstitution([FindPackageShare('xarm6_description'), 'rviz', 'display.rviz'])
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz2_params],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # Load controllers
    controllers = [
        'joint_state_broadcaster',
        # 'xarm6_traj_controller',
        'xarm6_velo_traj_controller',
        # 'xarm_gripper_traj_controller',
    ]
    load_controllers = []
    if load_controller.perform(context) in ('True', 'true'):
        for controller in controllers:
            load_controllers.append(Node(
                package='controller_manager',
                executable='spawner',
                output='screen',
                arguments=[
                    controller,
                    '--controller-manager', '{}/controller_manager'.format(ros_namespace)
                ],
                parameters=[{'use_sim_time': True}],
            ))

    if len(load_controllers) > 0:
        return [
            RegisterEventHandler(event_handler=OnProcessExit(target_action=gazebo_spawn_entity_node,on_exit=load_controllers,)),
            RegisterEventHandler(event_handler=OnProcessExit(target_action=rviz2_node, on_exit=[EmitEvent(event=Shutdown())])),
            gazebo_launch,
            robot_state_publisher_node,
            gazebo_spawn_entity_node,
            gazebo_spawn_entity_node_box,
            # gazebo_spawn_entity_node_camera,
            rviz2_node,
        ]
    else:
        return [
            RegisterEventHandler(event_handler=OnProcessExit(target_action=rviz2_node, on_exit=[EmitEvent(event=Shutdown())])),
            gazebo_launch,
            robot_state_publisher_node,
            gazebo_spawn_entity_node,
            gazebo_spawn_entity_node_box,
            # gazebo_spawn_entity_node_camera,
            rviz2_node,
        ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])

# ros2 topic pub /xarm6_traj_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
#   header: {
#     stamp: {sec: 0, nanosec: 0},
#     frame_id: ''
#   },
#   joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
#   points: [
#     {
#       positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
#       velocities: [0.5, -0.5, 0.3, -0.3, 0.2, -0.2],
#       accelerations: [],
#       effort: [],
#       time_from_start: {sec: 1, nanosec: 0}
#     }
#   ]
# }"

# ros2 topic pub /xarm6_velo_traj_controller/commands std_msgs/msg/Float64MultiArray "{layout: {dim: [{label: 'joints', size: 6, stride: 6}], data_offset: 0}, data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]}"