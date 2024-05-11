#!/usr/bin/env python3

import open3d as o3d
import cv2
import numpy as np
import os

# Setup the video capture
video_path = '/home/nikolaraicevic/Nikola_Robot_Manipulation/Data/240313_db_get/data/2024-03-13_17-32-13/data2.mkv'
cap = cv2.VideoCapture(video_path)

# Verify video is opened
if not cap.isOpened():
    print("Error: Could not open video.")
    exit(1)

# Camera Intrinsics (adjust based on selected camera)
fx, fy, cx, cy = 461.98, 462.079, 326.45, 184.237  # Camera 1 Intrinsics
# fx, fy, cx, cy = 321.651, 321.651, 321.608, 181.172  # Uncomment for Camera 2 Intrinsics

frame_num = 0
output_dir = '/home/nikolaraicevic/Nikola_Robot_Manipulation/Data/240313_db_get/data/2024-03-13_17-32-13/data2_point_cloud'

# Ensure the output directory exists
os.makedirs(output_dir, exist_ok=True)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame to grayscale if it's not already
    if frame.ndim == 3:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Convert frame to a depth map (assuming depth in grayscale)
    # Adjust this scaling if the depth is not in millimeters
    depth = np.array(frame, dtype=np.float32) # * (10.0 / 255)  # Convert to meters (adjust based on actual scale)

    # Create an Open3D depth image
    depth_image = o3d.geometry.Image(depth)
    intrinsics = o3d.camera.PinholeCameraIntrinsic(frame.shape[1], frame.shape[0], fx, fy, cx, cy)

    try:
        # Generate a point cloud from the depth image
        point_cloud = o3d.geometry.PointCloud.create_from_depth_image(depth_image, intrinsics, depth_scale=1.0, depth_trunc=3.0, stride=1)

        # Save the point cloud
        frame_num += 1
        output_filename = os.path.join(output_dir, f"frame_{frame_num:04d}.ply")
        o3d.io.write_point_cloud(output_filename, point_cloud)
        print(f"Uploaded: {output_filename}")
    except Exception as e:
        print(f"Error generating point cloud for frame {frame_num}: {e}")

cap.release()
print("Processing completed.")
