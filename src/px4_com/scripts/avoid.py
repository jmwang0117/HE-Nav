#!/usr/bin/env python

import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Float64, String
import time
from pyquaternion import Quaternion
import math
import threading

class AvoidPath:
    def __init__(self):
        rospy.init_node("avoid_path")
        self.goalListX = rospy.get_param('~goalListX', '0.0, 0.0')
        self.goalListY = rospy.get_param('~goalListY', '0.0, 0.0')
        self.goalListZ = rospy.get_param('~goalListZ', '0.0, 0.0')
        self.goalListYaw = rospy.get_param('~goalListYaw', '0.0, 0.0')
        self.goals = [[float(x), float(y), float(z), float(yaw)] for (x, y, z, yaw) in zip(self.goalListX.split(","),self.goalListY.split(","),self.goalListZ.split(","),self.goalListYaw.split(","))]
        
        self.dir_front = None
        self.local_pose = None
        self.target_pose = PoseStamped()

        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped,self.local_pose_callback)
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.custom_activity_pub = rospy.Publisher('/set_activity/type', String, queue_size=10)
        self.position_target_pub = rospy.Publisher('/set_pose/position', PoseStamped, queue_size=10)
        self.target_point = 0

        time.sleep(1)
        self.custom_activity_pub.publish(String("HOVER"))
        time.sleep(2)       
        check_str = raw_input("Press 1 to continue: ")
        print "input str is: ", check_str
        if check_str != str(1):
            return
        self.move(self.goals[0][0], self.goals[0][1], self.goals[0][2])
        self.update_target_pose(self.target_point)
        rospy.loginfo("moving to target 0")
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if(self.closed_enough(self.local_pose, self.target_pose) | self.stop_front()):
                if(self.target_point>3):
                    self.custom_activity_pub.publish(String("HOVER"))
                else:
                    self.target_point += 1
                    self.move(self.goals[self.target_point][0], self.goals[self.target_point][1], self.goals[self.target_point][2])             
                    self.update_target_pose(self.target_point)
                    rospy.loginfo("moving to target_point: " + str(self.target_point))
            r.sleep()

    def local_pose_callback(self, msg):
        self.local_pose = msg
    
    def laser_callback(self, msg):
        newdata = msg
        newdata.ranges = list(msg.ranges)
        newdata.intensities = list(data.intensities)
        front_dis_sum = 0
        for x in range(355, 365):
            front_dis_sum += newdata.ranges[x]
        self.dir_front = front_dis_sum/10


    def closed_enough(self, cur_p, target_p, threshold=1.5):
        delta_x = math.fabs(cur_p.pose.position.x - target_p.pose.position.x)
        delta_y = math.fabs(cur_p.pose.position.y - target_p.pose.position.y)
        delta_z = math.fabs(cur_p.pose.position.z - target_p.pose.position.z)
        if (delta_x + delta_y + delta_z < threshold):
            return True
        else:
            return False
    
    def stop_front(self):
        if self.dir_front <= 1:
            return True
        else:
            return False
    
    def set_pose(self, x=0, y=0, z=2, BODY_OFFSET_ENU = True):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        if BODY_OFFSET_ENU:
            pose.header.frame_id = 'base_link'
        else:
            pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        return pose
    
    def move(self, x, y, z, BODY_OFFSET_ENU=True):
        self.position_target_pub.publish(self.set_pose(x, y, z, BODY_OFFSET_ENU))
    
    def update_target_pose(self, target_point):
        pose = PoseStamped()
        pose.pose.position.x = self.target_pose.pose.position.x + self.goals[target_point][0]
        pose.pose.position.y = self.target_pose.pose.position.y + self.goals[target_point][1]
        pose.pose.position.z = self.target_pose.pose.position.z + self.goals[target_point][2]
        self.target_pose = pose


if __name__ == "__main__":
    con = AvoidPath()


