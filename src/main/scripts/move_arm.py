#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ArmTrajectoryClient(Node):
    def __init__(self):
        super().__init__('arm_trajectory_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/xarm6_traj_controller/follow_joint_trajectory')

    def send_goal(self, positions, time_to_reach):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = time_to_reach
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory

        self._action_client.wait_for_server()
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if not result:
            self.get_logger().info('Goal execution failed')
        else:
            # Assuming result contains an attribute to check for success. 
            # You need to check what attributes are available in the result object.
            self.get_logger().info('Result received')
            # If there's a specific attribute or detail you want to log, check the actual structure of `result`
            # and log that. For example, if `result` has an attribute `error_code`, you might log that:
            if hasattr(result, 'error_code'):
                self.get_logger().info(f'Error code: {result.error_code}')
            else:
                self.get_logger().info('No error code attribute in result.')

        self.get_logger().info(f'Result object: {result}')



def main(args=None):
    rclpy.init(args=args)
    arm_trajectory_client = ArmTrajectoryClient()
    positions = [1.5, -0.5, 0.1, 0.2, 0.3, -0.1]  # Adjust as needed
    time_to_reach = 5  # 5 seconds
    arm_trajectory_client.send_goal(positions, time_to_reach)
    rclpy.spin(arm_trajectory_client)

if __name__ == '__main__':
    main()

