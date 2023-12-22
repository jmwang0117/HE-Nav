#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from __future__ import print_function
import os
import rospy
import roslib
import sys
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import Twist
import numpy as np

import tf
import math
#import airsim


class CMD:

    def __init__(self):
        rospy.init_node('pub_twist',anonymous=True)
        self.pub_cmd = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        self.sub_cmd = rospy.Subscriber("/planning/pos_cmd",PositionCommand,self.cmd_cb)
        self.odom_cmd = rospy.Subscriber("/odom", Odometry, self.odom_cb)

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0



        rospy.spin()
    
    def cmd_cb(self,msg):
        self.target_x = msg.position.x
        self.target_y = msg.position.y
        self.target_yaw = msg.yaw


        Kp = 0.95
        Kd = 0.007


        dy = self.target_y - self.current_y
        dx = self.target_x - self.current_x
        dL = math.sqrt(dx*dx + dy*dy)
        #dyaw = 0.2*(self.target_yaw - self.current_yaw)
        dyaw =math.atan2(dy,dx)- self.current_yaw
        if(dyaw > 1.5*math.pi):
           dyaw = dyaw - 2*math.pi
        if(dyaw < -1.5*math.pi):
           dyaw = dyaw + 2*math.pi

        print("dyaw: "+ str(dyaw*180/math.pi)+"  self yaw: "+str(self.current_yaw*180/math.pi) + " target: "+str(math.atan2(dy,dx)*180/math.pi))
        
        cmd_command = Twist()
        cmd_command.linear.x = 0.5*dL
        cmd_command.angular.z = Kp*dyaw
        if(dL < 0.08):
           cmd_command.linear.x =0.0

        #if(dyaw < 0.04):
        #   cmd_command.angular.z = 0.0

        if(cmd_command.linear.x>0.75):
           cmd_command.linear.x =0.75
        if(cmd_command.linear.x<-0.75):
           cmd_command.linear.x = -0.75
        #if(cmd_command.angular.z>1.5*math.pi):
        #   cmd_command.angular.z =1.57
        #if(cmd_command.angular.z< -1.57):
        #   cmd_command.angular.z = -1.57
        self.pub_cmd.publish(cmd_command)
        
        #print(" dL: " + str(dL)+" dyaw: " +str(dyaw) +" current_yaw: " + str(self.current_yaw))
    
    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w
        q= airsim.Quaternionr(q_x,q_y,q_z,q_w)
        pitch,roll,self.current_yaw = airsim.to_eularian_angles(q)
        #print ("current_x: " + str(self.current_x) + " current_y: " + str(self.current_y)+" current_yaw: " + str(self.current_yaw))   

if __name__ == '__main__':
   try:
      cmd = CMD()
   except rospy.ROSInterruptException:
      pass
