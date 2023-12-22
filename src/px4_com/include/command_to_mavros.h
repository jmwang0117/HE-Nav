/***************************************************************************************************************************
* command_to_mavros.h
*
* Author: Qyp
*
* Update Time: 2019.5.1
*
* Introduction:  Drone control command send class using Mavros package
*         1. Ref to the Mavros plugins (setpoint_raw, loca_position, imu and etc..)
*         2. Ref to the Offboard Flight task in PX4 code: https://github.com/PX4/Firmware/blob/master/src/lib/FlightTasks/tasks/Offboard/FlightTaskOffboard.cpp
*         3. Ref to the Mavlink module in PX4 code: https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.cpp
*         4. Ref to the position control module in PX4: https://github.com/PX4/Firmware/blob/master/src/modules/mc_pos_control
*         5. Ref to the attitude control module in PX4: https://github.com/PX4/Firmware/blob/master/src/modules/mc_att_control
*         6. 还需要考虑复合形式的输出情况
***************************************************************************************************************************/
#ifndef COMMAND_TO_MAVROS_H
#define COMMAND_TO_MAVROS_H

#include <ros/ros.h>
#include <math_utils.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/ActuatorControl.h>
#include <sensor_msgs/Imu.h>
#include <bitset>

using namespace std;

namespace namespace_command_to_mavros {

class command_to_mavros
{
    public:
    //constructed function
    command_to_mavros(void):
        command_nh("~")
    {
        command_nh.param<float>("Takeoff_height", Takeoff_height, 1.0);

        command_nh.param("geo_fence/x_min", geo_fence_x[0], -100.0);
        command_nh.param("geo_fence/x_max", geo_fence_x[1], 100.0);
        command_nh.param("geo_fence/y_min", geo_fence_y[0], -100.0);
        command_nh.param("geo_fence/y_max", geo_fence_y[1], 100.0);
        command_nh.param("geo_fence/z_min", geo_fence_z[0], -100.0);
        command_nh.param("geo_fence/z_max", geo_fence_z[1], 100.0);

        pos_drone_fcu           = Eigen::Vector3d(0.0,0.0,0.0);
        vel_drone_fcu           = Eigen::Vector3d(0.0,0.0,0.0);
        q_fcu                   = Eigen::Quaterniond(0.0,0.0,0.0,0.0);
        Euler_fcu               = Eigen::Vector3d(0.0,0.0,0.0);
        rates_fcu               = Eigen::Vector3d(0.0,0.0,0.0);
        pos_drone_fcu_target    = Eigen::Vector3d(0.0,0.0,0.0);
        vel_drone_fcu_target    = Eigen::Vector3d(0.0,0.0,0.0);
        pos_drone_fcu_target    = Eigen::Vector3d(0.0,0.0,0.0);
        accel_drone_fcu_target  = Eigen::Vector3d(0.0,0.0,0.0);
        q_fcu_target            = Eigen::Quaterniond(0.0,0.0,0.0,0.0);
        Euler_fcu_target        = Eigen::Vector3d(0.0,0.0,0.0);
        Thrust_target           = 0.0;

        Takeoff_position        = Eigen::Vector3d(0.0,0.0,0.0);
        Takeoff_yaw             = 0.0;
        Hold_position           = Eigen::Vector3d(0.0,0.0,0.0);
        Hold_yaw                = 0.0;
        type_mask_target        = 0;
        frame_target            = 0;


        Land_position           = Eigen::Vector3d(0.0,0.0,0.0);
        Land_yaw                = 0.0;
        flag_set_land_position  = 0;

        state_sub = command_nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &command_to_mavros::state_cb,this);

        position_sub = command_nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, &command_to_mavros::pos_cb,this);

        velocity_sub = command_nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 100, &command_to_mavros::vel_cb,this);

        attitude_sub = command_nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &command_to_mavros::att_cb,this);

        attitude_target_sub = command_nh.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/target_attitude", 10, &command_to_mavros::att_target_cb,this);

        position_target_sub = command_nh.subscribe<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/target_local", 10, &command_to_mavros::pos_target_cb,this);

        actuator_target_sub = command_nh.subscribe<mavros_msgs::ActuatorControl>("/mavros/target_actuator_control", 10, &command_to_mavros::actuator_target_cb,this);

        setpoint_raw_local_pub = command_nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

        actuator_setpoint_pub = command_nh.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 10);

        arming_client = command_nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

        set_mode_client = command_nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    }

    //Geigraphical fence
    Eigen::Vector2d geo_fence_x;
    Eigen::Vector2d geo_fence_y;
    Eigen::Vector2d geo_fence_z;

    //Takeoff Height
    float Takeoff_height;

    //Takeoff Position of the Drone
    Eigen::Vector3d Takeoff_position;
    double Takeoff_yaw;

    //Hold Position of the Drone (For hold mode in command.msg)
    Eigen::Vector3d Hold_position;
    double Hold_yaw;

    //Land Position of the Drone
    Eigen::Vector3d Land_position;
    double Land_yaw;
    int flag_set_land_position;

    //Current state of the drone
    mavros_msgs::State current_state;

    //Current pos of the drone
    Eigen::Vector3d pos_drone_fcu;
    //Current vel of the drone
    Eigen::Vector3d vel_drone_fcu;

    //Current att of the drone
    Eigen::Quaterniond q_fcu;
    Eigen::Vector3d Euler_fcu;
    Eigen::Vector3d rates_fcu;

    //Target typemask [from fcu]
    int type_mask_target;

    int frame_target;

    //Target pos of the drone [from fcu]
    Eigen::Vector3d pos_drone_fcu_target;

    //Target vel of the drone [from fcu]
    Eigen::Vector3d vel_drone_fcu_target;

    //Target accel of the drone [from fcu]
    Eigen::Vector3d accel_drone_fcu_target;

    //Target att of the drone [from fcu]
    Eigen::Quaterniond q_fcu_target;
    Eigen::Vector3d Euler_fcu_target;

    //Target thrust of the drone [from fcu]
    float Thrust_target;

    mavros_msgs::ActuatorControl actuator_target;

    mavros_msgs::ActuatorControl actuator_setpoint;

    mavros_msgs::SetMode mode_cmd;

    mavros_msgs::CommandBool arm_cmd;

    ros::ServiceClient arming_client;

    ros::ServiceClient set_mode_client;

    void set_takeoff_position()
    {
        Takeoff_position = pos_drone_fcu;
        Takeoff_yaw = Euler_fcu[2];
    }

    //Takeoff to the default altitude, pls change the param in PX4: MIS_TAKEOFF_ALT
    void takeoff();

    //Land to current position
    void land();

    //Idle. Do nothing.
    void idle();

    //Loiter in the current position.
    void loiter();


    //Send pos_setpoint and yaw_setpoint in ENU frame to PX4
    void send_pos_setpoint(Eigen::Vector3d pos_sp, float yaw_sp);

    //Send vel_setpoint and yaw_setpoint in ENU frame to PX4
    void send_vel_setpoint(Eigen::Vector3d vel_sp, float yaw_sp);

    //Send vel_setpoint in x,y pos_setpoint in z and yaw_setpoint in ENU frame to PX4
    void send_vel_xy_pos_z_setpoint(Eigen::Vector3d pos_sp,Eigen::Vector3d vel_sp,float yaw_sp);

    //Send pos_setpoint and yaw_setpoint in body frame to PX4
    void send_vel_setpoint_body(Eigen::Vector3d vel_sp, float yaw_sp);

    //用于movebase控制  全速度控制
    void send_vel_yawrate_setpoint(Eigen::Vector3d vel_sp,float yaw_rate_sp);

    //Send accel_setpoint and yaw_setpoint in ENU frame to PX4
    void send_accel_setpoint(Eigen::Vector3d accel_sp, float yaw_sp);

    //dyx send pos,vel,yaw and yaw rate setpoint in ENU frame to PX4
    void send_pos_vel_setpoint(Eigen::Vector3d pos_sp,Eigen::Vector3d vel_sp, float yaw_sp,float yaw_rate_sp);

    //dyx send pos,vel,acc,yaw and yaw rate setpoint in ENU frame to PX4
    void send_pos_vel_acc_setpoint(Eigen::Vector3d pos_sp,Eigen::Vector3d vel_sp,Eigen::Vector3d acc_sp, float yaw_sp,float yaw_rate_sp);

    //Send actuator_setpoint to PX4[Not recommanded. Because the high delay between the onboard computer and Pixhawk]
    void send_actuator_setpoint(Eigen::Vector4d actuator_sp);

    //Printf the parameters
    void printf_param();

    //Pringt the drone state[Full state]
    void prinft_drone_state(float current_time);

    //Pringt the drone state[Simple state]
    void prinft_drone_state2(float current_time);

    //Check for failsafe
    void check_failsafe();

    //printf the geo fence
    void show_geo_fence();
    ros::Publisher setpoint_raw_local_pub;

    private:

        ros::NodeHandle command_nh;
        ros::Subscriber state_sub;
        ros::Subscriber position_sub;
        ros::Subscriber velocity_sub;
        ros::Subscriber attitude_sub;
        ros::Subscriber attitude_target_sub;
        ros::Subscriber position_target_sub;
        ros::Subscriber actuator_target_sub;
        //ros::Publisher setpoint_raw_local_pub;
        ros::Publisher actuator_setpoint_pub;

        void state_cb(const mavros_msgs::State::ConstPtr &msg)
        {
            current_state = *msg;
        }

        void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
        {
            pos_drone_fcu  = Eigen::Vector3d(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
        }

        void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
        {
            vel_drone_fcu = Eigen::Vector3d(msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);
        }

        void att_cb(const sensor_msgs::Imu::ConstPtr& msg)
        {
            q_fcu = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

            //Transform the Quaternion to Euler Angles
            Euler_fcu = quaternion_to_euler(q_fcu);

            rates_fcu = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

        }

        void att_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg)
        {
            q_fcu_target = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

            //Transform the Quaternion to Euler Angles
            Euler_fcu_target = quaternion_to_euler(q_fcu_target);

            Thrust_target = msg->thrust;
        }

        void pos_target_cb(const mavros_msgs::PositionTarget::ConstPtr& msg)
        {
            type_mask_target = msg->type_mask;

            frame_target = msg->coordinate_frame;

            pos_drone_fcu_target = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);

            vel_drone_fcu_target = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);

            accel_drone_fcu_target = Eigen::Vector3d(msg->acceleration_or_force.x, msg->acceleration_or_force.y, msg->acceleration_or_force.z);
        }

        void actuator_target_cb(const mavros_msgs::ActuatorControl::ConstPtr& msg)
        {
            actuator_target = *msg;
        }


};

void command_to_mavros::takeoff()
{
    mavros_msgs::PositionTarget pos_setpoint;

    pos_setpoint.type_mask = 0x1000;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

void command_to_mavros::land()
{
    //If the land_position is not set, set current position of the drone as the land_position
    if(flag_set_land_position == 0)
    {
        Land_position[0] = pos_drone_fcu[0];
        Land_position[1] = pos_drone_fcu[1];
        Land_position[2] = Takeoff_position[2];
        Land_yaw = Euler_fcu[2];
        flag_set_land_position = 1;
    }

    mavros_msgs::PositionTarget pos_setpoint;

    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = Land_position[0];
    pos_setpoint.position.y = Land_position[1];
    pos_setpoint.position.z = Land_position[2];

    pos_setpoint.yaw = Land_yaw;

    //如果距离起飞高度小于20厘米，则直接上锁并切换为手动模式；
    if(abs(pos_drone_fcu[2] - Takeoff_position[2]) < (0.2))
    {
        if(current_state.mode == "OFFBOARD")
        {
            mode_cmd.request.custom_mode = "MANUAL";
            set_mode_client.call(mode_cmd);
        }

        if(current_state.armed)
        {
            arm_cmd.request.value = false;
            arming_client.call(arm_cmd);

        }

        if (arm_cmd.response.success)
        {
            cout<<"Disarm successfully!"<<endl;
        }
    }
    else
    {
        setpoint_raw_local_pub.publish(pos_setpoint);
        cout << "The drone is landing "<< endl;
    }

}

void command_to_mavros::loiter()
{
    mavros_msgs::PositionTarget pos_setpoint;

    //Here pls ref to mavlink_receiver.cpp
    pos_setpoint.type_mask = 0x3000;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

void command_to_mavros::idle()
{
    mavros_msgs::PositionTarget pos_setpoint;

    pos_setpoint.type_mask = 0x4000;

    setpoint_raw_local_pub.publish(pos_setpoint);
}


void command_to_mavros::send_pos_setpoint(Eigen::Vector3d pos_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw

    pos_setpoint.header.stamp = ros::Time::now();

    //uint8 FRAME_LOCAL_NED = 1
    //uint8 FRAME_BODY_NED = 8
    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];

    pos_setpoint.yaw = yaw_sp * M_PI/180;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

void command_to_mavros::send_vel_setpoint(Eigen::Vector3d vel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;

    pos_setpoint.type_mask = 0b100111000111;

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = vel_sp[2];

    pos_setpoint.yaw = yaw_sp * M_PI/180;

    setpoint_raw_local_pub.publish(pos_setpoint);
}
void command_to_mavros::send_vel_xy_pos_z_setpoint(Eigen::Vector3d pos_sp, Eigen::Vector3d vel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget setpoint;

    setpoint.type_mask = 0b100111100011;

    setpoint.coordinate_frame = 1;

    setpoint.velocity.x = vel_sp[0];
    setpoint.velocity.y = vel_sp[1];
    setpoint.position.z = pos_sp[2];

    setpoint.yaw = yaw_sp * M_PI/180;

    setpoint_raw_local_pub.publish(setpoint);
}

void command_to_mavros::send_vel_setpoint_body(Eigen::Vector3d vel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;

    pos_setpoint.type_mask = 0b100111000111;

    //uint8 FRAME_LOCAL_NED = 1
    //uint8 FRAME_BODY_NED = 8
    pos_setpoint.coordinate_frame = 8;

    pos_setpoint.position.x = vel_sp[0];
    pos_setpoint.position.y = vel_sp[1];
    pos_setpoint.position.z = vel_sp[2];

    pos_setpoint.yaw = yaw_sp * M_PI/180;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

void command_to_mavros::send_vel_yawrate_setpoint(Eigen::Vector3d vel_sp,float yaw_rate_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    pos_setpoint.type_mask = 0b010111000111;
    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = vel_sp[0];
    pos_setpoint.position.y = vel_sp[1];
    pos_setpoint.position.z = vel_sp[2];

    pos_setpoint.yaw_rate = yaw_rate_sp * M_PI/180;
    setpoint_raw_local_pub.publish(pos_setpoint);

}

void command_to_mavros::send_accel_setpoint(Eigen::Vector3d accel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;

    pos_setpoint.type_mask = 0b100000111111;
    pos_setpoint.header.stamp = ros::Time::now();
    pos_setpoint.coordinate_frame = 1;


    //pos_setpoint.coordinate_frame = 1;

    pos_setpoint.acceleration_or_force.x = accel_sp[0];
    pos_setpoint.acceleration_or_force.y = accel_sp[1];
    pos_setpoint.acceleration_or_force.z = accel_sp[2];

    pos_setpoint.yaw = yaw_sp * M_PI/180;

    setpoint_raw_local_pub.publish(pos_setpoint);
    cout<<"coor: "<<pos_setpoint.coordinate_frame<<endl;
}


//dyx
void command_to_mavros::send_pos_vel_setpoint(Eigen::Vector3d pos_sp,Eigen::Vector3d vel_sp, float yaw_sp,float yaw_rate_sp)
{
    mavros_msgs::PositionTarget all_setpoint;

    all_setpoint.type_mask = 0b000111000000;

    all_setpoint.coordinate_frame = 1;

    all_setpoint.position.x = pos_sp[0];
    all_setpoint.position.y = pos_sp[1];
    all_setpoint.position.z = pos_sp[2];

    all_setpoint.velocity.x = vel_sp[0];
    all_setpoint.velocity.y = vel_sp[1];
    all_setpoint.velocity.z = vel_sp[2];

    all_setpoint.yaw = yaw_sp * M_PI/180;
    all_setpoint.yaw_rate = yaw_rate_sp * M_PI/180;


    setpoint_raw_local_pub.publish(all_setpoint);
}

void command_to_mavros::send_pos_vel_acc_setpoint(Eigen::Vector3d pos_sp,Eigen::Vector3d vel_sp, Eigen::Vector3d acc_sp,float yaw_sp,float yaw_rate_sp)
{
    mavros_msgs::PositionTarget all_setpoint;

    all_setpoint.type_mask = 0b000000000000;

    all_setpoint.coordinate_frame = 1;

    all_setpoint.position.x = pos_sp[0];
    all_setpoint.position.y = pos_sp[1];
    all_setpoint.position.z = pos_sp[2];

    all_setpoint.velocity.x = vel_sp[0];
    all_setpoint.velocity.y = vel_sp[1];
    all_setpoint.velocity.z = vel_sp[2];

    all_setpoint.acceleration_or_force.x = acc_sp[0];
    all_setpoint.acceleration_or_force.y = acc_sp[1];
    all_setpoint.acceleration_or_force.z = acc_sp[2];

    all_setpoint.yaw = yaw_sp * M_PI/180;
    all_setpoint.yaw_rate = yaw_rate_sp * M_PI/180;


    setpoint_raw_local_pub.publish(all_setpoint);
}
void command_to_mavros::send_actuator_setpoint(Eigen::Vector4d actuator_sp)
{
    actuator_setpoint.group_mix = 0;
    actuator_setpoint.controls[0] = actuator_sp(0);
    actuator_setpoint.controls[1] = actuator_sp(1);
    actuator_setpoint.controls[2] = actuator_sp(2);
    actuator_setpoint.controls[3] = actuator_sp(3);
    actuator_setpoint.controls[4] = 0.0;
    actuator_setpoint.controls[5] = 0.0;
    actuator_setpoint.controls[6] = 0.0;
    actuator_setpoint.controls[7] = 0.0;

    actuator_setpoint_pub.publish(actuator_setpoint);
}

void command_to_mavros::show_geo_fence()
{
    cout << "geo_fence_x : "<< geo_fence_x[0] << " [m]  to  "<<geo_fence_x[1] << " [m]"<< endl;
    cout << "geo_fence_y : "<< geo_fence_y[0] << " [m]  to  "<<geo_fence_y[1] << " [m]"<< endl;
    cout << "geo_fence_z : "<< geo_fence_z[0] << " [m]  to  "<<geo_fence_z[1] << " [m]"<< endl;
}

void command_to_mavros::check_failsafe()
{
    if (pos_drone_fcu[0] < geo_fence_x[0] || pos_drone_fcu[0] > geo_fence_x[1] ||
        pos_drone_fcu[1] < geo_fence_y[0] || pos_drone_fcu[1] > geo_fence_y[1] ||
        pos_drone_fcu[2] < geo_fence_z[0] || pos_drone_fcu[2] > geo_fence_z[1])
    {
        while(ros::ok())
        {
            land();
            cout << "Out of the geo fence, the drone is landing... "<< endl;
        }
    }
}


// 【打印参数函数】
void command_to_mavros::printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout << "takeoff_height : "<< Takeoff_height << endl;

}

void command_to_mavros::prinft_drone_state(float current_time)
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Drone State<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout<<setprecision(1);

    cout << "Time: " << current_time <<" [s] ";

    //是否和飞控建立起连接
    if (current_state.connected == true)
    {
        cout << " [ Connected ]  ";
    }
    else
    {
        cout << " [ Unconnected ]  ";
    }

    //是否上锁
    if (current_state.armed == true)
    {
        cout << "  [ Armed ]   ";
    }
    else
    {
        cout << "  [ DisArmed ]   ";
    }

    cout << " [ " << current_state.mode<<" ]   " <<endl;

    cout<<setprecision(2);

    cout << "Position [X Y Z] : " << pos_drone_fcu[0] << " [ m ] "<< pos_drone_fcu[1]<<" [ m ] "<<pos_drone_fcu[2]<<" [ m ] "<<endl;
    cout << "Velocity [X Y Z] : " << vel_drone_fcu[0] << " [m/s] "<< vel_drone_fcu[1]<<" [m/s] "<<vel_drone_fcu[2]<<" [m/s] "<<endl;

    cout << "Attitude [R P Y] : " << Euler_fcu[0] * 180/M_PI <<" [deg] "<<Euler_fcu[1] * 180/M_PI << " [deg] "<< Euler_fcu[2] * 180/M_PI<<" [deg] "<<endl;

    cout << "Acc_target [X Y Z] : "  << accel_drone_fcu_target[0] << " [m/s^2] "<< accel_drone_fcu_target[1]<<" [m/s^2] "<<accel_drone_fcu_target[2]<<" [m/s^2] "<<endl;

    cout << "Att_target [R P Y] : " << Euler_fcu_target[0] * 180/M_PI <<" [deg] "<<Euler_fcu_target[1] * 180/M_PI << " [deg] "<< Euler_fcu_target[2] * 180/M_PI<<" [deg] "<<endl;

    cout << "Thr_target [0 - 1] : " << Thrust_target <<endl;

    //ned to enu
    cout << "actuator_target [0 1 2 3] : " << actuator_target.controls[0] << " [ ] "<< -actuator_target.controls[1] <<" [ ] "<<-actuator_target.controls[2]<<" [ ] "<<actuator_target.controls[3] <<" [ ] "<<endl;

    cout << "actuator_target [4 5 6 7] : " << actuator_target.controls[4] << " [ ] "<< actuator_target.controls[5] <<" [ ] "<<actuator_target.controls[6]<<" [ ] "<<actuator_target.controls[7] <<" [ ] "<<endl;

    //cout << "Thank you for your support!                    ----Amov Lab" <<endl;
}

void command_to_mavros::prinft_drone_state2(float current_time)
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Drone State<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout << "Time: " << fixed <<setprecision(1)<< current_time <<" [s] ";

    //是否和飞控建立起连接
    if (current_state.connected == true)
    {
        cout << " [ Connected ]  ";
    }
    else
    {
        cout << " [ Unconnected ]  ";
    }

    //是否上锁
    if (current_state.armed == true)
    {
        cout << "  [ Armed ]   ";
    }
    else
    {
        cout << "  [ DisArmed ]   ";
    }

    cout << " [ " << current_state.mode<<" ]   " <<endl;

    cout << "Position [X Y Z] : " << fixed <<setprecision(2)<< pos_drone_fcu[0] << " [ m ] "<< pos_drone_fcu[1]<<" [ m ] "<<pos_drone_fcu[2]<<" [ m ] "<<endl;
    cout << "Velocity [X Y Z] : " << vel_drone_fcu[0] << " [m/s] "<< vel_drone_fcu[1]<<" [m/s] "<<vel_drone_fcu[2]<<" [m/s] "<<endl;
    cout << "Attitude [R P Y] : " << Euler_fcu[0] * 180/M_PI <<" [deg] "<<Euler_fcu[1] * 180/M_PI << " [deg] "<< Euler_fcu[2] * 180/M_PI<<" [deg] "<<endl;
    cout << "Att_target [R P Y] : " << Euler_fcu_target[0] * 180/M_PI <<" [deg] "<<Euler_fcu_target[1] * 180/M_PI << " [deg] "<< Euler_fcu_target[2] * 180/M_PI<<" [deg] "<<endl;

}

}
#endif
