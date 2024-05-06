#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def getGridCoords(worldCoords, resolution, origin):
    gridCoords = np.array(
        [
            int(round((worldCoords[0] - origin[0]) / resolution)),
            int(round((worldCoords[1] - origin[1]) / resolution)),
            int(round((worldCoords[2] - origin[2]) / resolution)),
        ],
        dtype=np.int64
    )
    return gridCoords

def posToIndex(world_coords, resolution_inv, origin):
    grid_coords = np.zeros(3, dtype=np.int64)
    for i in range(3):
        grid_coords[i] = int(np.floor((world_coords[i] - origin[i]) * resolution_inv))
    return grid_coords



        
        
def occupancy_inflate_callback(data):
    resolution = 0.1
    resolution_inv = 1.0 / resolution
    origin = np.array([-10, -10, -0.1])

    # Extract points from sensor_msgs/PointCloud2 without using pcl library
    point_list = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)

    # Iterate through all the points and convert the coordinates accordingly
    for point in point_list:
        world_coords = np.array([point[0], point[1], point[2]])

        # Convert world coordinates to grid coordinates (Eigen::Vector3i equivalent)
        grid_coords = posToIndex(world_coords, resolution_inv, origin)

        # Process the grid_coords
        print(f"Grid Point: x={grid_coords[0]}, y={grid_coords[1]}, z={grid_coords[2]}")
        print(f"World Point: x={world_coords[0]}, y={world_coords[1]}, z={world_coords[2]}")

rospy.init_node('occupancy_inflate_subscriber')
rospy.Subscriber("/sdf_map/occupancy_inflate", PointCloud2, occupancy_inflate_callback)
rospy.spin()