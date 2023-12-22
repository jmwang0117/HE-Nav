#!/usr/bin/env python

import rospy
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String
from pyquaternion import Quaternion
from sensor_msgs.msg import Joy
import time
import math

class Commander:
    def __init__(self):
        rospy.init_node("commander_node")
        rate = rospy.Rate(20)
        self.position_target_pub = rospy.Publisher('/set_pose/position', PoseStamped, queue_size=10)
        self.yaw_target_pub = rospy.Publisher('/set_pose/orientation', Float32, queue_size=10)
        self.custom_activity_pub = rospy.Publisher('/set_activity/type', String, queue_size=10)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joycallback)
        rospy.spin()
            
    
    def joycallback(self, msg):
        joymsg = msg
        if joymsg.buttons[4] == 1:
            self.custom_activity_pub.publish(String("ARM"))
            rospy.logwarn("Arming")
        if joymsg.buttons[5] == 1:
            self.custom_activity_pub.publish(String("DISARM"))
            rospy.logwarn("Disarming")

    def move(self, x, y, z, BODY_OFFSET_ENU=True):
        self.position_target_pub.publish(self.set_pose(x, y, z, BODY_OFFSET_ENU))
    
    def turn(self, yaw_degree):
        self.yaw_target_pub.publish(yaw_degree)
    
    def land(self):
        self.custom_activity_pub.publish(String("LAND"))
    
    def hover(self):
        self.custom_activity_pub.publish(String("HOVER"))
    
    def return_home(self, height):
        self.position_target_pub.publish(self.set_pose(0, 0, height, False))
    
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

if __name__ == "__main__":
    con = Commander()

