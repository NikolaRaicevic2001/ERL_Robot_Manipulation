#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import numpy as np
import open3d as o3d

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('point_cloud_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'point_cloud', 10)
        self.timer_period = 0.1  # seconds (adjust based on your needs)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.frame_num = 1  # Initialize frame counter

    def timer_callback(self):
        # Frame number formatting to match "frame_0000"
        frame_filename = f"frame_{self.frame_num:04}.ply"
        file_path = f"/home/nikolaraicevic/Nikola_Robot_Manipulation/Data/240313_db_get/data/2024-03-13_17-32-13/data2_point_cloud/{frame_filename}"

        # Attempt to load the point cloud data
        try:
            pcd = o3d.io.read_point_cloud(file_path)
            points = np.asarray(pcd.points)

            # Create a PointCloud2 message
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "camera_front_camera_depth_optical_frame"

            # Create and publish the PointCloud2 message
            ros_cloud = pc2.create_cloud_xyz32(header, points)
            self.publisher_.publish(ros_cloud)
            self.get_logger().info(f'Publishing {frame_filename}')

            # Increment frame number for next callback
            self.frame_num += 1
        except Exception as e:
            self.get_logger().error(f'Failed to load {frame_filename}: {str(e)}')
            # Optionally reset frame_num or handle error differently
            # self.frame_num = 0

def main(args=None):
    rclpy.init(args=args)
    point_cloud_publisher = PointCloudPublisher()
    rclpy.spin(point_cloud_publisher)
    point_cloud_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
