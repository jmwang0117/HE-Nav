#include <ros/ros.h>

#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <px4_command/command.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
enum Command
{
    Move_ENU,
    Move_Body,
    Hold,
    Land,
    Disarm,
    Arm,
    Failsafe_land,
    Idle,
    Takeoff
};
px4_command::command Command_now;
//---------------------------------------正方形参数---------------------------------------------
float size_square; //正方形边长
float height_square;                //飞行高度
float sleep_time;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "square");
    ros::NodeHandle nh("~");
    ros::Rate rate(1.0);
    ros::Publisher move_pub = nh.advertise<px4_command::command>("/px4/command", 10);
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    nh.param<float>("size_square", size_square, 1.0);
    nh.param<float>("height_square", height_square, 0.5);
    nh.param<float>("sleep_time", sleep_time, 10.0);

    // 这一步是为了程序运行前检查一下参数是否正确
    // 输入1,继续，其他，退出程序
    int check_flag;
    cout << "size_square: "<<size_square<<"[m]"<<endl;
    cout << "height_square: "<<height_square<<"[m]"<<endl;
    cout << "Please check the parameter and setting，1 for go on， else for quit: "<<endl;
    cin >> check_flag;
    if(check_flag != 1)
    {
        return -1;
    }

    //check arm
    int Arm_flag;
    cout<<"Whether choose to Arm? 1 for Arm, 0 for quit"<<endl;
    cin >> Arm_flag;
    if(Arm_flag == 1)
    {
        Command_now.command = Arm;
        move_pub.publish(Command_now);
    }
    else return -1;

    int takeoff_flag;
    cout << "Whether choose to Takeoff? 1 for Takeoff, 0 for quit "<<endl;
    cin >> takeoff_flag;
    if(takeoff_flag == 1)
    {
        Command_now.command = Takeoff;
        move_pub.publish(Command_now);
    }
    else return -1;

    int i=0;
    int comid = 0;
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主程序<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    //takeoff
    /*i = 0;
    while (i < sleep_time)
    {
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = 0;
        Command_now.pos_sp[1] = 0;
        Command_now.pos_sp[2] = height_square;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        cout << "Point 0----->takeoff"<<endl;
        i++;
    }*/

    //point left bottom
    int begin_flag;
    cout << "Whether choose to begin? 1 for begin, 0 for quit "<<endl;
    cin >> begin_flag;
    if(begin_flag != 1) return -1;

    i = 0;
    while(i < sleep_time)
    {
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = -size_square/2.0;
        Command_now.pos_sp[1] = -size_square/2.0;
        Command_now.pos_sp[2] = height_square;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        cout << "Point 1----->right-bottom"<<endl;
        i++;
    }

    //point left top
    i = 0;
    while(i < sleep_time)
    {
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = size_square/2.0;
        Command_now.pos_sp[1] = -size_square/2.0;
        Command_now.pos_sp[2] = height_square;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        cout << "Point 2----->right-top"<<endl;
        i++;
    }

    //point right top
    i = 0;
    while(i < sleep_time)
    {
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = size_square/2.0;
        Command_now.pos_sp[1] = size_square/2.0;
        Command_now.pos_sp[2] = height_square;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        cout << "Point 3----->left-top"<<endl;
        i++;
    }

    //point right bottom
    i = 0;
    while(i < sleep_time)
    {
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = -size_square/2.0;
        Command_now.pos_sp[1] = size_square/2.0;
        Command_now.pos_sp[2] = height_square;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        cout << "Point 4----->left-bottom"<<endl;
        i++;
    }

    //point return
    i = 0;
    while(i < sleep_time)
    {
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = 0;
        Command_now.pos_sp[1] = 0;
        Command_now.pos_sp[2] = height_square;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        cout << "Point 5----->return"<<endl;
        i++;
    }

    //降落
    Command_now.command = Land;
    while (ros::ok())
    {
      move_pub.publish(Command_now);
      rate.sleep();
      cout << "Land"<<endl;
    }
    rate.sleep();
    cout << "Mission complete, exiting...."<<endl;
    return 0;
}











