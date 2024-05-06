#!/usr/bin/env python3

import rospy
from quadrotor_msgs.msg import PositionCommand

def main():
    rospy.init_node('manual_control')

    # 从给定的C++代码中获取相关主题名称
    pos_cmd_topic = "/position_cmd"

    # 创建发布者
    pos_cmd_pub = rospy.Publisher(pos_cmd_topic, PositionCommand, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # 示例：发布位置指令
        pos_cmd_msg = PositionCommand()
        pos_cmd_msg.header.stamp = rospy.Time.now()
        pos_cmd_msg.header.frame_id = "world"
        pos_cmd_msg.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        pos_cmd_msg.trajectory_id = 1  # 可以设置为任意整数，表示轨迹ID

        pos_cmd_msg.position.x = 2.0  # 设置目标x坐标
        pos_cmd_msg.position.y = 1.0  # 设置目标y坐标
        pos_cmd_msg.position.z = 0.0  # 设置目标z坐标

        pos_cmd_msg.velocity.x = 0.5  # 设置目标x方向速度
        pos_cmd_msg.velocity.y = 0.0  # 设置目标y方向速度
        pos_cmd_msg.velocity.z = 0.0  # 设置目标z方向速度

        pos_cmd_msg.acceleration.x = 0.0  # 设置目标x方向加速度
        pos_cmd_msg.acceleration.y = 0.0  # 设置目标y方向加速度
        pos_cmd_msg.acceleration.z = 0.0  # 设置目标z方向加速度

        pos_cmd_msg.yaw = 0.0  # 设置目标偏航角
        pos_cmd_msg.yaw_dot = 0.0  # 设置目标偏航角速度

        pos_cmd_pub.publish(pos_cmd_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass