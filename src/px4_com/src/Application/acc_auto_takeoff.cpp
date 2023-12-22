#include <ros/ros.h>

#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <px4_command/command_acc.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
enum Command
{
    Move_ENU,
    Move_Body,
    Hold,
    Takeoff,
    Land,
    Arm,
    Disarm,
    Failsafe_land,
    Idle
};
px4_command::command_acc Command_now;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "acc_auto_takeoff");
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0);
    // 【发布】发送给position_control.cpp的命令
    ros::Publisher command_pub = nh.advertise<px4_command::command_acc>("/px4/command", 10);

    //check paramater
    int check_flag;
    //输入1,继续，其他，退出程序
    cout << "Please check the parameter and setting，1 for go on， else for quit: "<<endl;
    cin >> check_flag;
    if(check_flag != 1) return -1;

    //check arm
    int Arm_flag;
    cout<<"Whether choose to Arm? 1 for Arm, 0 for quit"<<endl;
    cin >> Arm_flag;
    if(Arm_flag == 1)
    {
        Command_now.command = Arm;
        command_pub.publish(Command_now);
    }
    else return -1;

    //check takeoff
    int Take_off_flag;
    cout<<"Whether choose to Takeoff? 1 for Takeoff, 0 for quit"<<endl;
    cin >> Take_off_flag;
    if(Take_off_flag == 1)
    {
        Command_now.command = Takeoff;
        command_pub.publish(Command_now);
    }
    else return -1;

    //check land
    int Land_flag;
    cout<<"Whether choose to Land? 1 for Land, 0 for quit"<<endl;
    cin >> Land_flag;
    if(Land_flag == 1)
    {
        Command_now.command = Land;
        while (ros::ok())
        {
           command_pub.publish(Command_now);
           rate.sleep();
           cout << "Land"<<endl;
        }
    }
    else return -1;

    cout<<"Program finish, exiting.....";
    return 0;
}












