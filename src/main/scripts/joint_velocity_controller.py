#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/xarm6_traj_controller/joint_trajectory', 10)
        timer_period = 5  # seconds, adjust as needed for your use case
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']  # Adjust names as needed
        velocities = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]  # Example velocities for each joint

        msg = JointTrajectory()
        msg.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.velocities = velocities
        point.time_from_start = Duration(sec=2)  # Adjust time as needed
        msg.points.append(point)

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing joint trajectory')

def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_publisher = JointTrajectoryPublisher()
    rclpy.spin(joint_trajectory_publisher)
    joint_trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
