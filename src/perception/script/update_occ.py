#!/usr/bin/python3
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

rospy.init_node('occupancy_publisher')

# 创建PointCloud2消息
header = rospy.Header()
header.stamp = rospy.Time.now()
header.frame_id = 'world'
fields = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1),
    PointField('intensity', 12, PointField.FLOAT32, 1),
]
points = []
occupancy_value = 111.0  # 占据信息为1
step = 0.1  # 控制点云的密度，即点之间的间隔
for x in range(30):
    for y in range(30):
        for z in range(30):
            points.append([x * step, y * step, z * step, occupancy_value])
cloud_msg = pc2.create_cloud(header, fields, points)

# 创建Publisher并发布PointCloud2消息
pub = rospy.Publisher('/sdf_map/obstacle_cloud', PointCloud2, queue_size=10)
rate = rospy.Rate(10)  # 发布频率为10Hz
while not rospy.is_shutdown():
    pub.publish(cloud_msg)
    rate.sleep()


