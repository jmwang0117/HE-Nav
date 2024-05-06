#!/usr/bin/env python3
import rospy
import struct
import numpy as np
import os
import argparse
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from point2vox import PointCloudVoxelization
import threading
import time
scan_count = 0

class PointCloudListener:
    def __init__(self):

        self.input_folder = "/root/HE-Nav/src/perception/raw_data/velodyne"
        self.output_folder = "/root/HE-Nav/src/perception/raw_data/voxels"
        
        self.grid_size = (256, 256, 32)
        self.voxelizer = PointCloudVoxelization(self.input_folder, self.output_folder, self.grid_size)
        self.lock = threading.Lock()

    def callback(self, data):
        global scan_count
        
        points = pc2.read_points(data, field_names=("x", "y", "z", "intensity"), skip_nans=True)

        # Save point cloud as .bin file in Velodyne format
        self.save_point_cloud_as_bin(points, scan_count)

        # Voxelization in a separate thread for faster processing
        t = threading.Thread(target=self.process_voxelization)
        t.start()

        scan_count += 1

    def save_point_cloud_as_bin(self, points, scan_count):
        file_name = f'{scan_count:06}.bin'
        file_path = os.path.join(self.input_folder, file_name)

        point_cloud = []

        for point in points:
            x, y, z = point
            remission = np.sqrt(x**2 + y**2 + z**2)  # Calculate the distance from the origin
            remission = remission / 10
            #print(f"x: {x}, y: {y}, z: {z}, remission: {remission}")            
            point_cloud.append([x, y, z, remission])
        
        point_cloud = np.array(point_cloud, dtype=np.float32)
        point_cloud_bytes = point_cloud.tobytes()

        with open(file_path, 'wb') as f:
            f.write(point_cloud_bytes)
    
    def process_voxelization(self):
        self.lock.acquire()
        self.voxelizer.voxelization()
        self.lock.release()

    def listener(self):
        rospy.init_node('pointcloud_listener', anonymous=True)

        # Choose the desired topic 
        topic = "/grid_map/occupancy_inflate"  # Replace with the desired topic name

        rospy.Subscriber(topic, PointCloud2, self.callback)
        time.sleep(1)
        rospy.spin()

if __name__ == '__main__':
    pcl = PointCloudListener()
    pcl.listener()