#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import argparse
import time
from sensor_msgs.msg import JointState

class MinimalSubscriber(Node):
    def __init__(self, topic_name):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(JointState,topic_name,self.listener_callback,10)
        self.subscription  # prevent unused variable warning
        self.last_time = time.time()
        self.interval = 5  # Interval in seconds

    def listener_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_time >= self.interval:
            self.last_time = current_time
            self.get_logger().info(f'I heard: "{msg}"')  # Adjust this line to print what you need from the message

def main(args=None):
    parser = argparse.ArgumentParser(description='Subscribe to a ROS 2 topic at specific time intervals.')
    parser.add_argument('topic', type=str, help='The topic to subscribe to.')
    args = parser.parse_args()

    rclpy.init(args=[args.topic])  # Initialize rclpy with extracted arguments
    
    minimal_subscriber = MinimalSubscriber(args.topic)
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# ./timed_subscriber.py /joint_states

