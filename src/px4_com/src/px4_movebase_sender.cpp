/***************************************************************************************************************************
* px4_sender.cpp
*
* Author: Qyp
*
* Update Time: 2019.3.16
*
* Introduction:  PX4 command sender using px4 default command
*         1. Subscribe command.msg from upper nodes
*         2. Send command using command_to_mavros.h
*         3. Command includes:  (1)xyz+yaw
*                               (2)takeoff
*                               (3)land
*                               (4)idle
*                               (5)loiter
*                               (6)xyz+yaw(body frame)
***************************************************************************************************************************/

#include <ros/ros.h>

#include <command_to_mavros.h>
#include <pos_controller_PID.h>
#include <px4_command/command_acc.h>


#include <Eigen/Eigen>

using namespace std;
using namespace namespace_PID;
using namespace namespace_command_to_mavros;

//自定义的Command变量
//相应的命令分别为 移动(惯性系ENU)，移动(机体系)，悬停，降落，上锁，紧急降落，待机
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

//Command Now [from upper node]
px4_command::command_acc Command_Now;                      //无人机当前执行命令

//Command Last [from upper node]
px4_command::command_acc Command_Last;                     //无人机上一条执行命令

Eigen::Vector3d pos_sp(0,0,0);
Eigen::Vector3d vel_sp(0,0,0);
Eigen::Vector3d acc_sp(0,0,0);

double yaw_sp = 0;
double yaw_rate_sp = 0;
Eigen::Vector3d accel_sp(0,0,0);

void rotation_yaw(float yaw_angle, float input[2], float output[2])
{
    output[0] = input[0] * cos(yaw_angle) - input[1] * sin(yaw_angle);
    output[1] = input[0] * sin(yaw_angle) + input[1] * cos(yaw_angle);
}

float get_ros_time(ros::Time begin);
void prinft_command_state();

void Command_cb(const px4_command::command_acc::ConstPtr& msg)
{
    Command_Now = *msg;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_movebase_sender");
    ros::NodeHandle nh("~");

    ros::Subscriber Command_sub = nh.subscribe<px4_command::command_acc>("/px4/command", 10, Command_cb);

    // 频率 [50Hz]
    ros::Rate rate(50.0);

    command_to_mavros pos_sender;
    pos_controller_PID pos_sender_pid;

    pos_sender.printf_param();

    pos_sender.show_geo_fence();

    int check_flag;
    // 这一步是为了程序运行前检查一下参数是否正确
    // 输入1,继续，其他，退出程序
    //cout << "Please check the parameter and setting，1 for go on， else for quit: "<<endl;
    //cin >> check_flag;
/*
    if(check_flag != 1)
    {
        return -1;
    }
*/
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0.5;  //dyx

    // 等待和飞控的连接
    while(ros::ok() && pos_sender.current_state.connected)
    {
        local_pos_pub.publish(pose);//dyx
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Not Connected");
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // 连接成功
    ROS_INFO("Connected!!");

    // 先读取一些飞控的数据
    int i =0;
    for(i=0;i<50;i++)
    {
        //local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();

    }


    pos_sender.set_takeoff_position();

    Command_Now.comid = 0;
    Command_Now.command = Idle;

    // 记录启控时间
    ros::Time begin_time = ros::Time::now();
    float last_time = get_ros_time(begin_time);
    float dt = 0;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        ros::spinOnce();


        float cur_time = get_ros_time(begin_time);
        dt = cur_time  - last_time;

        dt = constrain_function2(dt, 0.01, 0.03);

        last_time = cur_time;

        //pos_sender.prinft_drone_state2(cur_time);
        //prinft_command_state();
        pos_sender.check_failsafe();

        //无人机一旦接受到Land指令，则会屏蔽其他指令
        if(Command_Last.command == Land)
        {
            Command_Now.command = Land;
        }

        switch (Command_Now.command)
        {
        case Move_ENU:

            pos_sp = Eigen::Vector3d(Command_Now.pos_sp[0],Command_Now.pos_sp[1],Command_Now.pos_sp[2]);
            //vel_sp = Eigen::Vector3d(Command_Now.vel_sp[0],Command_Now.vel_sp[1],Command_Now.vel_sp[2]);
            //acc_sp = Eigen::Vector3d(Command_Now.acc_sp[0],Command_Now.acc_sp[1],Command_Now.acc_sp[2]);
            yaw_sp = Command_Now.yaw_sp;
            //yaw_rate_sp = Command_Now.yaw_rate_sp;

            //pos_sender.send_pos_vel_acc_setpoint(pos_sp,vel_sp,acc_sp,yaw_sp,yaw_rate_sp);
            pos_sender.send_pos_setpoint(pos_sp,yaw_sp);
            //pos_sender.send_vel_setpoint(vel_sp,yaw_sp);
            //pos_sender.send_accel_setpoint(acc_sp,yaw_sp);


            break;

        case Move_Body:

            //vel_sp = Eigen::Vector3d(Command_Now.vel_sp[0],Command_Now.vel_sp[1],Command_Now.vel_sp[2]);
            float d_vel_body[2];
            d_vel_body[0] = Command_Now.vel_sp[0];
            d_vel_body[1] = Command_Now.vel_sp[1];
            float d_vel_enu[2];

            //rotation_yaw(pos_sender.Euler_fcu[2], d_vel_body, d_vel_enu);
            vel_sp[0] = d_vel_enu[0];
            vel_sp[1] = d_vel_enu[1];

            pos_sp[2] = pos_sender.Takeoff_height;

            vel_sp[2] = pos_sender_pid.MPC_Z_P* (pos_sp[2] - pos_sender.pos_drone_fcu[2]) ;

            yaw_rate_sp = Command_Now.yaw_rate_sp;

            pos_sender.send_vel_yawrate_setpoint(vel_sp,yaw_rate_sp);

            break;

        case Hold:

            //pos_sender.loiter();
            cout<<"Hold received！"<<endl;
            if (Command_Last.command != Hold)
            {
                pos_sender.Hold_position = Eigen::Vector3d(pos_sender.pos_drone_fcu[0],pos_sender.pos_drone_fcu[1],pos_sender.pos_drone_fcu[2]);
                pos_sender.Hold_yaw =pos_sender.Euler_fcu[2]* 180/M_PI;
                pos_sp = pos_sender.Hold_position;
                vel_sp = Eigen::Vector3d(0,0,0);
                yaw_sp = pos_sender.Hold_yaw;
            }

            //pos_sender.send_vel_xy_pos_z_setpoint(pos_sp,vel_sp,yaw_sp);
            pos_sender.send_pos_setpoint(pos_sp,yaw_sp);
            break;


        case Land:

            pos_sender.land();

            break;

        case Disarm:

            if(pos_sender.current_state.mode == "OFFBOARD")
            {
                pos_sender.mode_cmd.request.custom_mode = "MANUAL";
                pos_sender.set_mode_client.call(pos_sender.mode_cmd);
            }

            if(pos_sender.current_state.armed)
            {
                pos_sender.arm_cmd.request.value = false;
                pos_sender.arming_client.call(pos_sender.arm_cmd);

            }

            if (pos_sender.arm_cmd.response.success)
            {
                cout<<"Disarm successfully!"<<endl;
            }

            break;

        case Failsafe_land:

            break;

        // 【】
        case Idle:
            pos_sender.idle();
            break;

        case Takeoff:
            pos_sender.mode_cmd.request.custom_mode = "OFFBOARD";
            pos_sender.set_mode_client.call(pos_sender.mode_cmd);
            cout<<"Takeoff receieved!"<<endl;

            pos_sp = Eigen::Vector3d(pos_sender.Takeoff_position[0],pos_sender.Takeoff_position[1],pos_sender.Takeoff_position[2]+pos_sender.Takeoff_height);
            vel_sp = Eigen::Vector3d(0,0,0);
            yaw_sp = pos_sender.Takeoff_yaw* 180/M_PI;
            pos_sender.send_pos_setpoint(pos_sp, yaw_sp);
            //pos_sender.send_vel_xy_pos_z_setpoint(pos_sp,vel_sp,yaw_sp);
            //accel_sp = pos_sender_pid.pos_controller(pos_sender.pos_drone_fcu, pos_sender.vel_drone_fcu, pos_sp, vel_sp, 2, dt);
            //pos_sender.send_accel_setpoint(accel_sp, yaw_sp);
            break;
        case Arm:
            //cout<<"Arm receieved!"<<endl;
            //if(pos_sender.current_state.mode != "OFFBOARD") break;
            if(!pos_sender.current_state.armed)
            {
                 pos_sender.arm_cmd.request.value = true;
                 pos_sender.arming_client.call(pos_sender.arm_cmd);
            }
            if (pos_sender.arm_cmd.response.success)
            {
                cout<<"Arm successfully!"<<endl;
            }
            break;
        }


        Command_Last = Command_Now;

        rate.sleep();
    }

    return 0;

}

// 【获取当前时间函数】 单位：秒
float get_ros_time(ros::Time begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}
// 【打印控制指令函数】
void prinft_command_state()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Command State<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    switch(Command_Now.command)
    {
    case Move_ENU:
        cout << "Command: [ Move_ENU ] " <<endl;
        break;
    case Move_Body:
        cout << "Command: [ Move_Body ] " <<endl;
        break;
    case Hold:
        cout << "Command: [ Hold ] " <<endl;
        break;
    case Land:
        cout << "Command: [ Land ] " <<endl;
        break;
    case Disarm:
        cout << "Command: [ Disarm ] " <<endl;
        break;
    case Failsafe_land:
        cout << "Command: [ Failsafe_land ] " <<endl;
        break;
    case Idle:
        cout << "Command: [ Idle ] " <<endl;
        break;
    case Takeoff:
        cout << "Command: [ Takeoff ] " <<endl;
        break;

    }

    int sub_mode;
    sub_mode = Command_Now.sub_mode;

    if((sub_mode & 0b10) == 0) //xy channel
    {
        cout << "Submode: xy position control "<<endl;
        cout << "X_setpoint   : " << Command_Now.pos_sp[0] << " [ m ]"  << "  Y_setpoint : "<< Command_Now.pos_sp[1] << " [ m ]"<<endl;
    }
    else{
        cout << "Submode: xy velocity control "<<endl;
        cout << "X_setpoint   : " << Command_Now.vel_sp[0] << " [m/s]" << "  Y_setpoint : "<< Command_Now.vel_sp[1] << " [m/s]" <<endl;
    }

    if((sub_mode & 0b01) == 0) //z channel
    {
        cout << "Submode:  z position control "<<endl;
        cout << "Z_setpoint   : "<< Command_Now.pos_sp[2] << " [ m ]" << endl;
    }
    else
    {
        cout << "Submode:  z velocity control "<<endl;
        cout << "Z_setpoint   : "<< Command_Now.vel_sp[2] << " [m/s]" <<endl;
    }

    cout << "Yaw_setpoint : "  << Command_Now.yaw_sp << " [deg] " <<endl;
}
