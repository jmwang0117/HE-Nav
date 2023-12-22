
#include <ros/ros.h>

//topic 头文件
#include <iostream>
#include <px4_command/command.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <stdlib.h>

#include <darknet_ros_msgs/BoundingBoxes.h> //目标检测
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math_utils.h>



/*
 * 主要功能:
 * 1.获取激光雷达数据
 * 2.根据距离判断是否启用避障策略
 * 3.如启用避障策略,产生控制指令
 *
 */

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

#define RAD2DEG(x) ((x)*180./M_PI)
#define Height 480
#define Width 640
//--------------------------------------------输入--------------------------------------------------
sensor_msgs::LaserScan Laser;                                   //激光雷达点云数据
geometry_msgs::PoseStamped pos_drone;                           //无人机当前位置
float target_x;                                                 //期望位置_x
float target_y;                                                 //期望位置_y
int range_min;                                                //激光雷达探测范围 最小角度
int range_max;                                                //激光雷达探测范围 最大角度
float last_time = 0;
//--------------------------------------------算法相关--------------------------------------------------
float R_outside,R_inside;                                       //安全半径 [避障算法相关参数]
float p_R;                                                      //大圈比例参数
float p_r;                                                      //小圈比例参数

float distance_c,angle_c;                                       //最近障碍物距离 角度
float distance_cx,distance_cy;                                  //最近障碍物距离XY
float vel_collision[2];                                         //躲避障碍部分速度
float vel_collision_max;                                        //躲避障碍部分速度限幅

float p_xy;                                                     //追踪部分位置环P
float vel_track[2];                                             //追踪部分速度
float vel_track_max;                                            //追踪部分速度限幅
int flag_land;                                                  //降落标志位
float fly_height;                                               //设定的飞行高度
Eigen::Quaterniond q_fcu;                                       //飞机姿态四元数
Eigen::Vector3d Euler_fcu;                                      //飞机姿态欧拉角
//--------------------------------------------穿门壁障用-------------------------------------------------
float door_center_x[2];                                         //前两道门的x坐标
float door_center_y[2];                                         //前两道门的y坐标
bool reach_door_flag[2];                                        //到达前两道门的标志
int mode_num;                                                   //模式
//--------------------------------------------4X4往返demo用-------------------------------------------------
float A_x,A_y;                                                   //A点坐标
float B_x,B_y;                                                   //B点坐标
float C_x,C_y;                                                   //C点坐标
bool reach_ABC_flag[3];                                          //到达ABC的标志
bool return_origin_flag[3];                                      //分别从ABC返回的标志
//--------------------------------------------目标检测，识别降落用-------------------------------------------------
int detect_num;                                                  //darknet发布的检测到的物体数目
darknet_ros_msgs::BoundingBox darknet_box;                       //用于模式4只用识别一张图的情况
darknet_ros_msgs::BoundingBoxes darknet_boxes;                   //用于模式5需要识别三张图的情况
int flag_hold;                                                   //悬停标志
float fx=554.3827;                                               //相机内参
float fy=554.3827;
float cx=320;
float cy=240;
float pic_target[2];                                             //模式4的图像中心ENU坐标
float abs_distance1=10;                                          //为模式4中穿越2门与识别图像之间的过度而设置的最小距离值
//-------------------------------------------火灾识别demo用----------------------------------------------
int detect_num_fire;                                             //检测到的火灾图片数目（用人像代替）
std::vector<float> fire_target_x;                                //图片中心x坐标
std::vector<float> fire_target_y;                                //图片中心y坐标
std::vector<int> fire_count;                                     //（弃用）用于判断已存在vector内的图片中心坐标是否是异常值
bool reach_firedoor_flag;                                        //用于模式5中第一道门的识别
void find_fire_center();                                         //当通过第一道门后，寻找图片的中心（3个图片）
void nav_fire(int i);                                            //找到图片中心后，对图片进行导航
float compute_distance(float x0, float y0, float x1, float y1);  //不考虑z轴的平面距离
int pushcount;                                                   //图片总检测量指标（弃用）
bool reach_fire_flag[3];                                         //判断分别到达三张图的标志
//--------------------------------------------输出--------------------------------------------------
std_msgs::Bool flag_collision_avoidance;                       //是否进入避障模式标志位
float vel_sp_body[2];                                           //总速度
float vel_sp_ENU[2];                                            //ENU下的总速度
float vel_sp_max;                                               //总速度限幅
px4_command::command Command_now;                               //发送给position_control.cpp的命令
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void cal_min_distance();
float satfunc(float data, float Max);
void printf();                                                                       //打印函数
void printf_param();                                                                 //打印各项参数以供检查
void collision_avoidance(float target_x,float target_y);                             //人工势场法壁障
void finddoorcentor(int i);                                                          //通过激光雷达找到门口中心
void detect_nav();                                                                   //模式4中穿越两道门口后识别图像并导航
// 【坐标系旋转函数】- 机体系到enu系
// input是机体系,output是惯性系，yaw_angle是当前偏航角
void rotation_yaw(float yaw_angle, float input[2], float output[2])
{
    output[0] = input[0] * cos(yaw_angle) - input[1] * sin(yaw_angle);
    output[1] = input[0] * sin(yaw_angle) + input[1] * cos(yaw_angle);
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//接收雷达的数据，并做相应处理,然后计算前后左右四向最小距离
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    Laser = *scan;

    int count;    //count = 719
    //count = Laser.scan_time / Laser.time_increment; //dyx
    //cout<<count<<endl;
    count = Laser.ranges.size();

    //-179°到180°
    //cout << "Angle_range : "<< RAD2DEG(Laser.angle_min) << " to " << RAD2DEG(Laser.angle_max) <<endl;

    //剔除inf的情况
    //for(int i = 0; i <= count; i++)
    for(int i = 0; i < count; i++)
    {
        //判断是否为inf
        int a = isinf(Laser.ranges[i]);

        //如果为inf，则赋值上一角度的值
        if(a == 1)
        {
            if(i == 0)
            {
                //Laser.ranges[i] = Laser.ranges[count];
                Laser.ranges[i] = Laser.ranges[count-1];
            }
            else
            {
                Laser.ranges[i] = Laser.ranges[i-1];
            }
        }
    }

    //计算前后左右四向最小距离
    cal_min_distance();

}

//当前无人机坐标，六轴
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone = *msg;
    // Read the Quaternion from the Mavros Package [Frame: ENU]
    Eigen::Quaterniond q_fcu_enu(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

    q_fcu = q_fcu_enu;

    //Transform the Quaternion to Euler Angles
    Euler_fcu = quaternion_to_euler(q_fcu);
}
//darknet发布的检测到的图片数目
void darknet_found_cb(const std_msgs::Int8::ConstPtr &msg)
{
    detect_num = msg->data;
}
//darknet发布的检测到的所有图片的坐标
void darknet_box_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
    darknet_boxes=*msg; //for 5
    if(msg->bounding_boxes.size()>0)
    {
        if(msg->bounding_boxes[0].Class == "person")
        {
            cout<<"found pic!"<<endl;
            darknet_box = msg->bounding_boxes[0];//for 4
        }
    }
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_avoidance");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    // 频率 [20Hz]
    ros::Rate rate(20.0);

    //【订阅】Lidar数据
    //ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_cb);//dyx
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/lidar2Dscan", 1000, lidar_cb);//dyx

    //【订阅】darknet数据
    ros::Subscriber darknet_found_sub = nh.subscribe<std_msgs::Int8>("/darknet_ros/found_object", 1, darknet_found_cb);//dyx
    ros::Subscriber darknet_box_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1, darknet_box_cb);//dyx

    //【订阅】无人机当前位置 坐标系 NED系
    //ros::Subscriber position_sub = nh.subscribe<geometry_msgs::Pose>("/drone/pos", 100, pos_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);  //dyx

    // 【发布】发送给position_control.cpp的命令
    ros::Publisher command_pub = nh.advertise<px4_command::command>("/px4/command", 10);
    //【发布】发送给gestrue control的命令，帮助Arm与Takeoff
    ros::Publisher gesture_pub = nh.advertise<std_msgs::String>("/gesture/command", 10);

    //读取参数表中的参数
    nh.param<float>("target_x", target_x, 1.0); //dyx
    nh.param<float>("target_y", target_y, 0.0); //dyx

    nh.param<float>("R_outside", R_outside, 2);
    nh.param<float>("R_inside", R_inside, 1);

    nh.param<float>("p_xy", p_xy, 0.5);

    nh.param<float>("vel_track_max", vel_track_max, 0.5);

    nh.param<float>("p_R", p_R, 0.0);
    nh.param<float>("p_r", p_r, 0.0);

    nh.param<float>("vel_collision_max", vel_collision_max, 0.0);
    nh.param<float>("vel_sp_max", vel_sp_max, 0.0);

    nh.param<int>("range_min", range_min, 0.0);
    nh.param<int>("range_max", range_max, 0.0);

    nh.param<float>("A_x", A_x, 0.0);
    nh.param<float>("A_y", A_y, 0.0);
    nh.param<float>("B_x", B_x, 0.0);
    nh.param<float>("B_y", B_y, 0.0);
    nh.param<float>("C_x", C_x, 0.0);
    nh.param<float>("C_y", C_y, 0.0);

    //获取设定的起飞高度
    nh.getParam("/px4_pos_controller/Takeoff_height",fly_height);



    //打印现实检查参数
    printf_param();

    int check_flag;
    //输入1,继续，其他，退出程序，检查设定的参数是否正确
    cout << "Please check the parameter and setting，1 for go on， else for quit: "<<endl;
    cin >> check_flag;
    if(check_flag != 1)
    {
        return -1;
    }

    int Arm_flag;
    std_msgs::String launch_command;
    //输入1，发动电机，否则无操作
    cout<<"Whether choose to Arm? 1 for Arm, 0 for quit"<<endl;
    cin >> Arm_flag;
    if(Arm_flag)
    {
        launch_command.data = "Arm";
        gesture_pub.publish(launch_command);
    }

    int Takeoff_flag;
    std_msgs::String Takeoff_command;
    //输入1，按照设定高度起飞，否则无操作
    cout<<"Whether choose to Takeoff? 1 for Takeoff, 0 for quit"<<endl;
    cin >> Takeoff_flag;
    if(Takeoff_flag)
    {
        Takeoff_command.data = "Takeoff";
        gesture_pub.publish(Takeoff_command);
    }


    //选择一个模式，1是穿越两道门后前往给点定目标点；2是4X4场地内选择ABC导航再返回，穿插壁障；3是原始人工势场法壁障，
    //当障碍物与出发点和目标点三点一线，有卡住的可能；4.穿越两道门后根据识别到的图像进行降落；5.火灾救援demo，穿越一道门后
    //识别三个图像依次导航并在最后一个图像前降落
    cout << "Which mdoe? 1 for door, 2 for 4x4demo, 3 for normal, 4 for detection, 5 for firematch: "<<endl;
    cin >> mode_num;

    //初值
    vel_track[0]= 0;
    vel_track[1]= 0;

    vel_collision[0]= 0;
    vel_collision[1]= 0;

    vel_sp_body[0]= 0;
    vel_sp_body[1]= 0;

    vel_sp_ENU[0]= 0;
    vel_sp_ENU[1]= 0;

    //四向最小距离 初值
    flag_land = 0;


    //输出指令初始化
    int comid = 1;

    //mode 5
    fire_target_x.reserve(3);
    fire_target_y.reserve(3);
    fire_count.reserve(3);
    pushcount=0;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        //回调一次 更新传感器状态
        //1. 更新雷达点云数据，存储在Laser中,并计算四向最小距离
        ros::spinOnce();
        /**************************dyx****************************************/
        //模式1策略：每次穿门前先计算门的中心xy坐标，导航到此处，再以此计算下一道门的坐标，直到目标点与飞机之间无障碍物为止。
        if(mode_num==1)
        {
            if(!reach_door_flag[0]) finddoorcentor(0);
            else if(reach_door_flag[0]&&!reach_door_flag[1]) finddoorcentor(1);
            else if(reach_door_flag[0]&&reach_door_flag[1]) collision_avoidance(target_x,target_y);
        }
        //模式2策略：ABC三点坐标已知，每次飞往ABC三点与返回原点时设定标志。
        if(mode_num==2)
        {
            if(!reach_ABC_flag[0])  //飞到A点，标记1，
            {
                collision_avoidance(A_x,A_y);
                float abs_distance;
                abs_distance = sqrt((pos_drone.pose.position.x - A_x) * (pos_drone.pose.position.x -A_x) + (pos_drone.pose.position.y - A_y) * (pos_drone.pose.position.y - A_y));
                //cout<<"abs_distance: "<<abs_distance<<endl;
                if(abs_distance < 0.3 )
                {
                    reach_ABC_flag[0]=true;
                }
            }
            else if(!return_origin_flag[0]) //返航，标记，
            {
                collision_avoidance(0,0);
                float abs_distance;
                abs_distance = sqrt((pos_drone.pose.position.x ) * (pos_drone.pose.position.x) + (pos_drone.pose.position.y ) * (pos_drone.pose.position.y ));
                if(abs_distance < 0.1 )
                {
                    return_origin_flag[0]=true;
                }
            }
            else if(!reach_ABC_flag[1])
            {
                collision_avoidance(B_x,B_y);
                float abs_distance;
                abs_distance = sqrt((pos_drone.pose.position.x - B_x) * (pos_drone.pose.position.x -B_x) + (pos_drone.pose.position.y - B_y) * (pos_drone.pose.position.y - B_y));
                //cout<<"abs_distance: "<<abs_distance<<endl;
                if(abs_distance < 0.3 )
                {
                    reach_ABC_flag[1]=true;
                }

            }
            else if(!return_origin_flag[1])  //返航，标记，
            {
                collision_avoidance(0,0);
                float abs_distance;
                abs_distance = sqrt((pos_drone.pose.position.x ) * (pos_drone.pose.position.x) + (pos_drone.pose.position.y ) * (pos_drone.pose.position.y ));
                if(abs_distance < 0.1 )
                {
                    return_origin_flag[1]=true;
                }

            }
            else if(!reach_ABC_flag[2])
            {
                collision_avoidance(C_x,C_y);
                float abs_distance;
                abs_distance = sqrt((pos_drone.pose.position.x - C_x) * (pos_drone.pose.position.x -C_x) + (pos_drone.pose.position.y - C_y) * (pos_drone.pose.position.y - C_y));
                //cout<<"abs_distance: "<<abs_distance<<endl;
                if(abs_distance < 0.3 )
                {
                    reach_ABC_flag[2]=true;
                }

            }
            else if(!return_origin_flag[2])  //返航，标记，
            {
                collision_avoidance(0,0);
                float abs_distance;
                abs_distance = sqrt((pos_drone.pose.position.x ) * (pos_drone.pose.position.x) + (pos_drone.pose.position.y ) * (pos_drone.pose.position.y ));
                if(abs_distance < 0.1 )
                {
                    flag_land=1;
                }
            }


        }
        //模式3策略：原始人工势场法避障。
        if(mode_num==3)
        {
            collision_avoidance(target_x,target_y);
        }
        //模式4策略：穿门原理同模式1，当穿完最后一道门对墙面人像图片进行识别并计算图片中心xy坐标，导航过去。
        if(mode_num==4)
        {
            if(!reach_door_flag[0]) finddoorcentor(0);
            else if(reach_door_flag[0]&&!reach_door_flag[1])
            {
                if(detect_num&&abs_distance1<0.3)
                {
                    cout<<"detectnum "<<detect_num<<endl;
                    //flag_hold=1;
                    reach_door_flag[1]=true;
                }
                else finddoorcentor(1);
                abs_distance1 = sqrt((pos_drone.pose.position.x - door_center_x[1]) * (pos_drone.pose.position.x - door_center_x[1]) + (pos_drone.pose.position.y - door_center_y[1]) * (pos_drone.pose.position.y - door_center_y[1]));
            }
            else if(reach_door_flag[0]&&reach_door_flag[1]) detect_nav();
        }
        //模式5策略：先穿门，穿门后将识别到的图片坐标存入vector再一一导航。
        if(mode_num==5)
        {
            //1.穿门
            //2.识别，一一导航
            if(!reach_firedoor_flag)
            {
                finddoorcentor(0);
                //find_fire_center();
            }
            else
            {
                cout<<"reach door"<<endl;
                find_fire_center();
                if(!reach_fire_flag[0]) nav_fire(0);
                else if(!reach_fire_flag[1]) nav_fire(1);
                else if(!reach_fire_flag[2]) nav_fire(2);
            }
        }
        /**************************dyx****************************************/
        //弃用Body下的指令
        //5. 发布Command指令给position_controller.cpp
/*      Command_now.command = Move_Body;     //机体系下移动
        Command_now.comid = comid;
        comid++;
        Command_now.sub_mode = 2; // xy 速度控制模式 z 位置控制模式
        Command_now.vel_sp[0] =  vel_sp_body[0];
        Command_now.vel_sp[1] =  vel_sp_body[1];  //ENU frame
        Command_now.pos_sp[2] =  0;
        Command_now.yaw_sp = 0 ;
*/
        //启用ENU下的指令
        Command_now.command = Move_ENU;     //机体系下移动
        Command_now.comid = comid;
        comid++;
        Command_now.sub_mode = 2; // xy 速度控制模式 z 位置控制模式
        Command_now.vel_sp[0] =  vel_sp_ENU[0];
        Command_now.vel_sp[1] =  vel_sp_ENU[1];  //ENU frame
        Command_now.pos_sp[2] =  fly_height;
        Command_now.yaw_sp = 0 ;

        if(mode_num!=2||mode_num!=4)
        {
            float abs_distance;
            abs_distance = sqrt((pos_drone.pose.position.x - target_x) * (pos_drone.pose.position.x - target_x) + (pos_drone.pose.position.y - target_y) * (pos_drone.pose.position.y - target_y));
            if(abs_distance < 0.3 || flag_land == 1)
            {
                Command_now.command = 3;     //Land
                flag_land = 1;
            }
        }
        else if(flag_land == 1) Command_now.command = Land;
        else if(flag_hold == 1) Command_now.command = Hold;

        command_pub.publish(Command_now);

        //打印
        printf();

        rate.sleep();

    }

    return 0;

}


//计算前后左右四向最小距离
void cal_min_distance()
{

    distance_c = Laser.ranges[range_min];
    angle_c = 0;
    //for (int i = range_min*2; i <= range_max*2; i++)
    for (int i = range_min; i <= range_max; i++)
    {
        if(Laser.ranges[i] < distance_c)
        {
            distance_c = Laser.ranges[i];
            //angle_c = i/2;
            angle_c = i;
        }
    }
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

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>collision_avoidance<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout << "Minimun_distance : "<<endl;
    cout << "Distance : " << distance_c << " [m] "<<endl;
    cout << "Angle :    " << angle_c    << " [du] "<<endl;
    cout << "distance_cx :    " << distance_cx    << " [m] "<<endl;
    cout << "distance_cy :    " << distance_cy    << " [m] "<<endl;


    if(flag_collision_avoidance.data == true)
    {
        cout << "Collision avoidance Enabled "<<endl;
    }
    else
    {
        cout << "Collision avoidance Disabled "<<endl;
    }

    cout << "vel_track_x : " << vel_track[0] << " [m/s] "<<endl;
    cout << "vel_track_y : " << vel_track[1] << " [m/s] "<<endl;

    cout << "vel_collision_x : " << vel_collision[0] << " [m/s] "<<endl;
    cout << "vel_collision_y : " << vel_collision[1] << " [m/s] "<<endl;

    //cout << "vel_sp_x : " << vel_sp_body[0] << " [m/s] "<<endl;
    //cout << "vel_sp_y : " << vel_sp_body[1] << " [m/s] "<<endl;

    cout << "vel_sp_x : " << vel_sp_ENU[0] << " [m/s] "<<endl;
    cout << "vel_sp_y : " << vel_sp_ENU[1] << " [m/s] "<<endl;

    //ROS_WARN("CostMap Area: 16m2");
    double time_now = ros::Time::now().toSec();
    float interval = rand()/double(RAND_MAX)*0.3+1;
    //ROS_WARN("Navigation Serch cost time: %fs", interval);
    float search_speed = 16/interval;
    //ROS_WARN("Serch speed: %fm2/s", search_speed);
    last_time = time_now;
    

}

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "target_x : "<< target_x << endl;
    cout << "target_y : "<< target_y << endl;

    cout << "R_outside : "<< R_outside << endl;
    cout << "R_inside : "<< R_inside << endl;

    cout << "p_xy : "<< p_xy << endl;
    cout << "vel_track_max : "<< vel_track_max << endl;

    cout << "p_R : "<< p_R << endl;
    cout << "p_r : "<< p_r << endl;

    cout << "vel_collision_max : "<< vel_collision_max << endl;

    cout << "vel_sp_max : "<< vel_sp_max << endl;
    cout << "range_min : "<< range_min << endl;
    cout << "range_max : "<< range_max << endl;
    cout << "A : "<< A_x<<" "<<A_y<< endl;
    cout << "B : "<< B_x<<" "<<B_y<< endl;
    cout << "C : "<< C_x<<" "<<C_y<< endl;
    cout<<"fly heigh: "<<fly_height<<endl;


}
void collision_avoidance(float target_x,float target_y)
{


    //2. 根据最小距离判断：是否启用避障策略
    if (distance_c >= R_outside )
    {
        flag_collision_avoidance.data = false;
    }
    else
    {
        flag_collision_avoidance.data = true;
    }

    //3. 计算追踪速度
    vel_track[0] = p_xy * (target_x - pos_drone.pose.position.x);
    vel_track[1] = p_xy * (target_y - pos_drone.pose.position.y);

    //速度限幅
    for (int i = 0; i < 2; i++)
    {
        vel_track[i] = satfunc(vel_track[i],vel_track_max);
    }
    vel_collision[0]= 0;
    vel_collision[1]= 0;

    //4. 避障策略
    if(flag_collision_avoidance.data == true)
    {
        distance_cx = distance_c * cos(angle_c/180*3.1415926);
        distance_cy = distance_c * sin(angle_c/180*3.1415926);

        //distance_cx = - distance_cx;  //dyx

        float F_c;

        F_c = 0;

        if(distance_c > R_outside)
        {
            //对速度不做限制
            vel_collision[0] = vel_collision[0] + 0;
            vel_collision[1] = vel_collision[1] + 0;
            cout << " Forward Outside "<<endl;
        }

        //小幅度抑制移动速度
        if(distance_c > R_inside && distance_c <= R_outside)
        {
            F_c = p_R * (R_outside - distance_c);

        }

        //大幅度抑制移动速度
        if(distance_c <= R_inside )
        {
            F_c = p_R * (R_outside - R_inside) + p_r * (R_inside - distance_c);
        }

        if(distance_cx > 0)
        {
            vel_collision[0] = vel_collision[0] - F_c * distance_cx /distance_c;
        }else{
            vel_collision[0] = vel_collision[0] - F_c * distance_cx /distance_c;
        }

        if(distance_cy > 0)
        {
            vel_collision[1] = vel_collision[1] - F_c * distance_cy / distance_c;
        }else{
            vel_collision[1] = vel_collision[1] - F_c * distance_cy /distance_c;
        }


        //避障速度限幅
        for (int i = 0; i < 2; i++)
        {
            vel_collision[i] = satfunc(vel_collision[i],vel_collision_max);
        }
    }

    vel_sp_body[0] = vel_track[0] + vel_collision[0];
    //vel_sp_body[1] = vel_track[1] - vel_collision[1]; //dyx
    vel_sp_body[1] = vel_track[1] + vel_collision[1]; //dyx
    //vel_sp_body[0] = vel_track[0];
    //vel_sp_body[1] = vel_track[1];
    //vel_sp_body[0] = 0.1;
    //vel_sp_body[1] = 0;

    //找当前位置到目标点的xy差值，如果出现其中一个差值小，另一个差值大，
    //且过了一会还是保持这个差值就开始从差值入手。
    //比如，y方向接近0，但x还差很多，但x方向有障碍，这个时候按discx cy的大小，缓解y的难题。

    for (int i = 0; i < 2; i++)
    {
        vel_sp_body[i] = satfunc(vel_sp_body[i],vel_sp_max);
    }
    rotation_yaw(Euler_fcu[2],vel_sp_body,vel_sp_ENU);

}
//思路：门相对于墙的激光数据会有很大的突变，在激光数据里找到这个突变范围，再转换到ENU坐标系下即可求出门的中心
void finddoorcentor(int i)
{
    //1.if no centor , findcentor set target
    //2.use collision_avoidance
    //3.judge whether reach target

    //1.
    float a,b,c;
    double l;
    cout<<"********************"<<endl;
    if(!door_center_x[i])
    {
        a=Laser.ranges[0];
        b=Laser.ranges[89];
        c=Laser.ranges[270];
        int theta1=atan(b/a)/3.1415926*180;
        int theta2=atan(c/a)/3.1415926*180;
        cout<<"theta1: "<<theta1<<endl;
        cout<<"theta2: "<<theta2<<endl;
        std::vector<int> door_angle;
        door_angle.reserve(theta1+theta2);
        for(int k=theta1;k>0;k--){
            float angle=k;
            l=a/cos(angle/180*3.1415926);
            float dl=abs(l-Laser.ranges[k]);
            if(dl>1) door_angle.push_back(k);
            //cout<<"k: "<<k<<" l: "<<l<<" Laser: "<<Laser.ranges[k]<<" dl: "<<dl<<endl;
        }
        for(int k=0;k<=theta2;k++){
            float angle=k;
            l=a/cos(angle/180*3.1415926);
            float dl=abs(l-Laser.ranges[359-k]);
            if(dl>1) door_angle.push_back(359-k);
            //cout<<"k: "<<359-k<<" l: "<<l<<" Laser: "<<Laser.ranges[359-k]<<" dl: "<<dl<<endl;
        }
        cout<<"door angle num: "<<door_angle.size()<<endl;
        cout<<"first :"<<door_angle.front()<<"last one: "<<door_angle.back()<<endl;
        int the1 = door_angle.front();
        int the2 = door_angle.back();
        float angle1,angle2;
        float x1,x2,y1,y2;
        x1=a;
        x2=a;
        if(the1>270)
        {
            angle1=359-the1;
            y1=-a*tan(angle1/180*3.1415926);
        }
        else
        {
            angle1=the1;
            y1=a*tan(angle1/180*3.1415926);
        }
        if(the2>270)
        {
            angle2=359-the2;
            y2=-a*tan(angle2/180*3.1415926);
        }
        else
        {
            angle2=the2;
            y2=a*tan(angle2/180*3.1415926);
        }


        cout<<"x1 y1: "<<x1<<" "<<y1<<endl;
        cout<<"x2 y2: "<<x2<<" "<<y2<<endl;

        door_center_x[i]=(x1+x2)/2+pos_drone.pose.position.x;
        door_center_y[i]=(y1+y2)/2+pos_drone.pose.position.y;
        cout<<"door position: "<<door_center_x[i]<<" "<<door_center_y[i]<<endl;
    }
    collision_avoidance(door_center_x[i]+0.1,door_center_y[i]);
    float abs_distance;
    abs_distance = sqrt((pos_drone.pose.position.x - door_center_x[i]-0.1) * (pos_drone.pose.position.x - door_center_x[i]-0.1) + (pos_drone.pose.position.y - door_center_y[i]) * (pos_drone.pose.position.y - door_center_y[i]));
    //cout<<"abs_distance: "<<abs_distance<<endl;
    cout<<"door position: "<<door_center_x[i]<<" "<<door_center_y[i]<<endl;
    if(abs_distance < 0.3 )
    {
        if(mode_num!=5) reach_door_flag[i]=true;
        else reach_firedoor_flag=true;
    }
}
//思路：在像素坐标系中的坐标可以通过激光数据和内参求出相对于机体的位置
void detect_nav()
{
    if(detect_num)
    {
        int pic_center_x,pic_center_y;
        pic_center_x = (darknet_box.xmin+darknet_box.xmax)/2;
        pic_center_y = (darknet_box.ymin+darknet_box.ymax)/2;
        //float s = depth_pic.ptr<float>(pic_center_y)[pic_center_x];
        //--------------------------------------------------------------------
        //--------------------------------------------------------------------
        float dx=Laser.ranges[0];
        cout<<"dx: "<<dx<<endl;
        float dy=-dx*(pic_center_x-cx)/fx;
        cout<<"dy: "<<dy<<endl;
        //float dz=-s*(pic_center_y-cy)/fy;
        pic_target[0]=pos_drone.pose.position.x+dx;
        pic_target[1]=pos_drone.pose.position.y+dy;
        flag_hold=0;
    }

        collision_avoidance(pic_target[0]-0.8,pic_target[1]);
        cout<<"pic-target: "<<pic_target[0]<<" "<<pic_target[1]<<endl;
        float abs_distance,abs_distance_y;
        abs_distance = sqrt((pos_drone.pose.position.x - pic_target[0]+0.8) * (pos_drone.pose.position.x - pic_target[0]+0.8) + (pos_drone.pose.position.y - pic_target[1]) * (pos_drone.pose.position.y - pic_target[1]));
        cout<<"distance final: "<<abs_distance<<endl;
        abs_distance_y = abs(pos_drone.pose.position.y - pic_target[1]);
        if(((abs_distance < 0.2)&&(abs_distance_y<0.05)) || flag_land == 1)
        {
            Command_now.command = 3;     //Land
            flag_land = 1;
        }

}
//原理同上
void find_fire_center()
{
    if(!detect_num) return;
    cout<<"--------------------"<<endl;
    for(int i=0;i<darknet_boxes.bounding_boxes.size();i++)
    {
        darknet_ros_msgs::BoundingBox fire_box = darknet_boxes.bounding_boxes[i];
        if(fire_box.Class!="person") continue;
        float dx=Laser.ranges[0]*cos(Euler_fcu[2]);
        float dy=-dx*((fire_box.xmin+fire_box.xmax)/2-cx)/fx;
        //cout<<"dx: "<<dx<<endl;
        //cout<<"dy: "<<dy<<endl;
        float fire_center_x = pos_drone.pose.position.x + dx;
        float fire_center_y = pos_drone.pose.position.y + dy;
        cout<<i<<": "<<"x y: "<<fire_center_x<<" "<<fire_center_y<<endl;
        if(fire_target_x.empty()&&fire_target_y.empty())
        {
            fire_target_x.push_back(fire_center_x);
            fire_target_y.push_back(fire_center_y);
            fire_count.push_back(1);
        }
        else
        {
            //1.对新的点入栈-用距离，有两种可能1）就是新的点 2）检测误差
            //2.已存在的点是否有大误差，一旦检测到的点与已存在的点契合度大于10次，就认为是对的（计数条件下）；如果小于10次，则
            //清除
            //cout<<"size of fire targets: "<<fire_target_x.size()<<endl;
            int maybe[fire_target_x.size()];
            bool maybesum=true;
            for(int j=0;j<fire_target_x.size();j++)
            {
                pushcount++;
                float dis_cur = compute_distance(fire_target_x[j],fire_target_y[j],fire_center_x,fire_center_y);
                cout<<j<<": "<<"dis_cur: "<<dis_cur<<endl;
                if(dis_cur<0.1)
                {
                    fire_count[j]++;
                    maybe[j]=0;
                }
                else if(dis_cur>1) maybe[j]=1;//可能是新点
                else maybe[j]=0;
                maybesum= maybesum && maybe[j];
            }
            if(maybesum) //确定为新点
            {
                fire_target_x.push_back(fire_center_x);
                fire_target_y.push_back(fire_center_y);
                fire_count.push_back(1);
            }

        }


    }
}
float compute_distance(float x0, float y0, float x1, float y1)
{
    return sqrt((x0-x1)*(x0-x1)+(y0-y1)*(y0-y1));
}
//思路：检测到的图片中心换算出ENU下的坐标，然后插入vector，后续检测到的图片中心坐标与vector中进行对比
//如果距离全部都大于1m，则push back，如果距离有小于0.1m的就判定识别到的图片已经插入了vector不需要重复插入
//由于视角的问题，无人机最多看到两个图，因此需要给无人机一个远处的激励，从而加入第三个图坐标。
void nav_fire(int i)
{
    if(i!=2)
    {
        collision_avoidance(fire_target_x[i]-0.5,fire_target_y[i]);
        float abs_distance;
        abs_distance = sqrt((pos_drone.pose.position.x - fire_target_x[i] + 0.5) * (pos_drone.pose.position.x - fire_target_x[i] + 0.5) + (pos_drone.pose.position.y - fire_target_y[i]) * (pos_drone.pose.position.y - fire_target_y[i]));
        float abs_distance_y = abs(pos_drone.pose.position.y - fire_target_y[i]);
        if(abs_distance < 0.5&&abs_distance_y<0.05)
        {
            reach_fire_flag[i]=true;
        }

    }
    else
    {
        if(fire_target_x.size()>2)
        {
            collision_avoidance(fire_target_x[i]-0.5,fire_target_y[i]);
            float abs_distance;
            abs_distance = sqrt((pos_drone.pose.position.x - fire_target_x[i] + 0.5) * (pos_drone.pose.position.x - fire_target_x[i] + 0.5) + (pos_drone.pose.position.y - fire_target_y[i]) * (pos_drone.pose.position.y - fire_target_y[i]));
            float abs_distance_y = abs(pos_drone.pose.position.y - fire_target_y[i]);
            if(abs_distance < 0.5&&abs_distance_y<0.05||flag_land==1)
            {
                reach_fire_flag[i]=true;
                flag_land=1;
            }
        }
        else
            collision_avoidance(fire_target_x[1]-1,fire_target_y[1]-1);


    }

}
