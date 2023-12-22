
//头文件
#include <ros/ros.h>
#include <fstream>
#include <math.h>
#include <string>
#include <time.h>
#include <queue>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <iomanip>

//msg 头文件
#include <px4_command/ude_log.h>

using namespace std;

px4_command::ude_log ude_log;

void save_flight_data(std::ofstream& out_file, float timenow);                       //储存数据函数
float get_dt(ros::Time last);
void ude_log_cb(const px4_command::ude_log::ConstPtr& msg)
{
    ude_log = *msg;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Data_log");
    ros::NodeHandle nh("~");

    //【订阅】cartographer估计位置
    ros::Subscriber ude_log_sub = nh.subscribe<px4_command::ude_log>("/px4_command/ude_log", 1000, ude_log_cb);

    // 频率
    ros::Rate rate(50.0);


    int check_flag;
    // 这一步是为了程序运行前检查一下参数是否正确
    // 输入1,继续，其他，退出程序
    cout << "1 for data log， else for quit: "<<endl;
    cin >> check_flag;

    if(check_flag != 1)
    {
        return -1;
    }

    //储存数据

    time_t tt = time(NULL);
    tm* t = localtime(&tt);
    char iden_path[256];
    sprintf(iden_path, "/home/nvidia/log/Data-log-%d-%02d-%02d_%02d-%02d.txt", t->tm_year+1900, t->tm_mon+1, t->tm_mday, t->tm_hour, t->tm_min);
    std::ofstream out_data_file(iden_path);

    if (!out_data_file)
    {
        std::cout << "Error: Could not write data!" << std::endl;
        return 0;
    }
    else
    {
        std::cout << "save data!!"<<std::endl;
        out_data_file <<" Time "<< " pos_drone.x " << " pos_drone.y " << " pos_drone.z " \
                                << " vel_drone.x " << " vel_drone.y " << " vel_drone.z " \
                                << " error_pos.x " << " error_pos.y " << " error_pos.z " \
                                << " error_vel.x " << " error_vel.y " << " error_vel.z " \
                                << " u_l.x " << " u_l.y " << " u_l.z " \
                                << " u_d.x " << " u_d.y " << " u_d.z " \
                                << " u_total.x " << " u_total.y " << " u_total.z " \
                                << " thrust_sp.x " << " thrust_sp.y " << " thrust_sp.z " \
                                <<std::endl;
    }

    // 记录启控时间
    ros::Time begin_time = ros::Time::now();

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        // 当前时间
        float cur_time = get_dt(begin_time);

        //回调一次 更新传感器状态
        ros::spinOnce();

        save_flight_data(out_data_file, cur_time);
        std::cout << "Saving Data!!" << std::endl;


        rate.sleep();
    }

    return 0;

}

void save_flight_data(std::ofstream& out_file, float timenow)
{
    out_file << timenow <<"  "<< ude_log.pos[0] <<"  "<< ude_log.pos[1] <<"  "<< ude_log.pos[2] <<"  "\
                              << ude_log.vel[0] <<"  "<< ude_log.vel[1] <<"  "<< ude_log.vel[2] <<"  "\
                              << ude_log.error_pos[0] <<"  "<< ude_log.error_pos[1] <<"  "<< ude_log.error_pos[2] <<"  "\
                              << ude_log.error_vel[0] <<"  "<< ude_log.error_vel[1] <<"  "<< ude_log.error_vel[2] <<"  "\
                              << ude_log.u_l[0] <<"  "<< ude_log.u_l[1] <<"  "<< ude_log.u_l[2] <<"  "\
                              << ude_log.u_d[0] <<"  "<< ude_log.u_d[1] <<"  "<< ude_log.u_d[2] <<"  "\
                              << ude_log.u_total[0] <<"  "<< ude_log.u_total[1] <<"  "<< ude_log.u_total[2] <<"  "\
                              << ude_log.thrust_sp[0] <<"  "<< ude_log.thrust_sp[1] <<"  "<< ude_log.thrust_sp[2] <<"  "\
                              << std::endl;
}

//获取当前时间 单位：秒
float get_dt(ros::Time last)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-last.sec;
    float currTimenSec = time_now.nsec / 1e9 - last.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}
