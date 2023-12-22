/***************************************************************************************************************************
 * collision_avoidance_streo.cpp
 *
 * Author: Qyp
 *
 * Update Time: 2019.4.25
 *
 * 说明: 避障程序 for streo
 *
***************************************************************************************************************************/
//ROS 头文件
#include <ros/ros.h>

//topic 头文件
#include <iostream>
#include <px4_command/command.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

using namespace std;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
enum Command
{
    Move_ENU,
    Move_Body,
    Hold,
    Land,
    Disarm,
    Failsafe_land,
    Idle,
    Takeoff
};
//--------------------------------------------输入--------------------------------------------------
geometry_msgs::Point Streo_distance;
geometry_msgs::Pose pos_drone;                                  //无人机当前位置
float target_x;                                                 //期望位置_x
float target_y;                                                 //期望位置_y
float p_xy;                                                     //追踪部分位置环P
float vel_track[2];                                             //追踪部分速度
float vel_track_max;                                            //追踪部分速度限幅
int flag_land;                                                  //降落标志位
float vel_sp[2];                                           //总速度
px4_command::command Command_now;                               //发送给position_control.cpp的命令
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float satfunc(float data, float Max);
void printf();                                                                       //打印函数
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//接收雷达的数据，并做相应处理,然后计算前后左右四向最小距离
void streo_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    Streo_distance = *msg;
}

void pos_cb(const geometry_msgs::Pose::ConstPtr& msg)
{
    pos_drone = *msg;

}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_avoidance_streo");
    ros::NodeHandle nh("~");

    // 频率 [30Hz]
    ros::Rate rate(30.0);

    // streo
    ros::Subscriber streo_sub = nh.subscribe<geometry_msgs::Point>("/streo_distance", 100, streo_cb);

    //【订阅】无人机当前位置 坐标系 NED系
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::Pose>("/drone/pos", 100, pos_cb);

    // 【发布】发送给position_control.cpp的命令
    ros::Publisher command_pub = nh.advertise<px4_command::command>("/px4/command", 10);

    //读取参数表中的参数
    nh.param<float>("target_x", target_x, 7.0);
    nh.param<float>("target_y", target_y, 0.0);
    nh.param<float>("p_xy", p_xy, 1.0);
    nh.param<float>("vel_track_max", vel_track_max, 1.0);

    int check_flag;
    //输入1,继续，其他，退出程序
    cout << "Please check the parameter and setting，1 for go on， else for quit: "<<endl;
    cin >> check_flag;

    if(check_flag != 1)
    {
        return -1;
    }

    //初值
    vel_track[0]= 0;
    vel_track[1]= 0;

    //四向最小距离 初值
    flag_land = 0;

    //输出指令初始化
    int comid = 1;

    float distance_to_target;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        //回调一次 更新传感器状态
        //1. 更新
        ros::spinOnce();


        distance_to_target = sqrt((pos_drone.position.x - target_x) * (pos_drone.position.x - target_x) + (pos_drone.position.y - target_y) * (pos_drone.position.y - target_y));

        if(distance_to_target < 0.3 || flag_land == 1)
        {
            Command_now.command = Land;     //Land
            flag_land = 1;
        }

        //3. 计算追踪速度
        vel_track[0] = p_xy * (target_x - pos_drone.position.x);
        vel_track[1] = p_xy * (target_y - pos_drone.position.y);

        if (pos_drone.position.x < 5.5)
        {
            vel_track[1] = 0.0;
        }

        //速度限幅
        for (int i = 0; i < 2; i++)
        {
            vel_track[i] = satfunc(vel_track[i],vel_track_max);
        }

        vel_sp[0] = 0.0;
        vel_sp[1] = 0.0;

        //4. 避障策略
        if(Streo_distance.y < 5.0)
        {
            vel_sp[0] = Streo_distance.y / 10.0 * vel_track[0];

            if(Streo_distance.y < 2)
            {
                vel_sp[0] = 0.0;
            }
        }

        if(vel_sp[0] == 0.0)
        {
            if(Streo_distance.z > 2)
            {
                vel_sp[1] = -0.4;
            }
            else if(Streo_distance.x > 2)
            {
                vel_sp[1] = 0.4;
            }else
            {
                if(Streo_distance.x < Streo_distance.z)
                {
                    vel_sp[1] = -0.2;
                }else
                {
                    vel_sp[1] = 0.2;
                }

            }

        }

        if(Streo_distance.x < 1.0 && Streo_distance.z < 1.0)
        {
            vel_sp[1] = 0.0;
        }

        //5. 发布Command指令给position_controller.cpp
        Command_now.command = Move_Body;     //机体系下移动
        Command_now.comid = comid;
        comid++;
        Command_now.sub_mode = 2; // xy 速度控制模式 z 位置控制模式
        Command_now.vel_sp[0] =  vel_sp[0];
        Command_now.vel_sp[1] =  vel_sp[1];
        Command_now.pos_sp[2] =  0;
        Command_now.yaw_sp = 0;

        command_pub.publish(Command_now);

        //打印
        printf();

        rate.sleep();

    }

    return 0;

}

//饱和函数
float satfunc(float data, float Max)
{
    if(abs(data)>Max)
    {
        return ( data > 0 ) ? Max : -Max;
    }
    else
    {
        return data;
    }
}


void printf()
{

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>collision_avoidance_streo<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout << "Streo_distance [m] : "<<endl;
    cout << "Left : " << Streo_distance.x << " [m] "<<endl;
    cout << "Mid  : " << Streo_distance.y << " [m] "<<endl;
    cout << "Right: " << Streo_distance.z << " [m] "<<endl;

    cout << "vel_sp_x : " << vel_sp[0] << " [m/s] "<<endl;
    cout << "vel_sp_y : " << vel_sp[1] << " [m/s] "<<endl;

}


