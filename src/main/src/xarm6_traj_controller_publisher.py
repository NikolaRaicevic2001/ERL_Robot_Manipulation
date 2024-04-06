import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import time

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/xarm6_traj_controller/controller_state', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_trajectory)
        self.traj_count = 0

    def publish_trajectory(self):
        # Create a JointTrajectory message
        msg = JointTrajectory()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # Create a trajectory point
        point = JointTrajectoryPoint()
        point.positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]  # Joint positions
        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Joint velocities
        point.time_from_start.sec = 1  # Time from start in seconds
        point.time_from_start.nanosec = 0
        msg.points = [point]
        
        # Publish the trajectory message
        self.publisher_.publish(msg)
        self.traj_count += 1
        if self.traj_count >= 5:  # Publish 5 times
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher = TrajectoryPublisher()
    rclpy.spin(trajectory_publisher)
    trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
