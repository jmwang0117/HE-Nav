#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray

def voxel_index_callback(msg):
    data = np.reshape(np.array(msg.data, dtype=np.int64), (-1, 3))
    for index_coords in data:
        # Convert index_coords to a tuple and print
        index_coords_str = '[' + ' '.join(map(str, index_coords)) + ']'
        print("Coordinate :", index_coords_str)

if __name__ == '__main__':
    rospy.init_node('index2pos')

    # Subscribe to the topic
    index_subscriber = rospy.Subscriber("/non_intersection_coordinates", Float64MultiArray, voxel_index_callback)

    rospy.spin()