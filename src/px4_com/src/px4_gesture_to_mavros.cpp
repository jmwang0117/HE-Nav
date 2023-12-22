#include <ros/ros.h>
#include <iostream>
#include <px4_command/command.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <math_utils.h>

using namespace std;

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
float fly_height;
ros::Publisher move_pub;
ros::Subscriber gesture_sub;
geometry_msgs::PoseStamped pos_drone;
Eigen::Quaterniond q_fcu;
Eigen::Vector3d Euler_fcu;
px4_command::command Command_now;
std_msgs::String gesture_command;

void generate_com(int sub_mode, float state_desired[4]);
void gesture_cb(const std_msgs::String::ConstPtr& command);

//gesture commands//
void move_forward(); //1
void move_backward(); //2
void move_left(); //3
void move_right(); //4
void move_up(); //5
void move_down(); //6
void turn_left(); //7
void turn_right(); //8
void hold(); //9

void arm(); //10
void disarm(); //11

void takeoff(); //12
void land(); //13

void photo(); //14
void start_record(); //15
void stop_record(); //16

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_gesture");
    ros::NodeHandle nh("~");
    move_pub = nh.advertise<px4_command::command>("/px4/command", 10);
    gesture_sub = nh.subscribe<std_msgs::String>("/gesture/command", 10, gesture_cb);

    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);
    nh.getParam("/px4_pos_controller/Takeoff_height",fly_height);
    ros::Rate rate(10.0);
    cout << "Init success!!!!!!! "<<endl;
    while (ros::ok())
    {   
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

void gesture_cb(const std_msgs::String::ConstPtr &command)
{
    gesture_command = *command;
    if (gesture_command.data == "Forward")
    {
        move_forward();
    }
    else if (gesture_command.data == "Backward")
    {
        move_backward();
    }
    else if (gesture_command.data == "Left")
    {
        move_left();
    }
    else if (gesture_command.data == "Right")
    {
        move_right();
    }
    else if (gesture_command.data == "Up")
    {
        move_up();
    }
    else if (gesture_command.data == "Down")
    {
        move_down();
    }
    else if (gesture_command.data == "TurnLeft")
    {
        turn_left();
    }
    else if (gesture_command.data == "TurnRight")
    {
        turn_right();
    }
    else if (gesture_command.data == "Hover")
    {
        hold();
    }
    else if (gesture_command.data == "Land")
    {
        land();
    }
    else if (gesture_command.data == "Takeoff")
    {
        takeoff();
    }
    else if (gesture_command.data == "Disarm")
    {
        disarm();
    }
    else if (gesture_command.data == "Arm")
    {
        arm();
    }
    else
    {
        cout << "Gesture do not support yet!!!!!!! "<<endl;
    }
}

void generate_com(int sub_mode, float state_desired[4])
{
    static int comid = 1;
    Command_now.sub_mode = sub_mode;
    //# sub_mode 2-bit value:
    //# 0 for position, 1 for vel, 1st for xy, 2nd for z.
    //#                   xy position     xy velocity
    //# z position       	0b00(0)       0b10(2)
    //# z velocity		0b01(1)       0b11(3)
    if ((sub_mode & 0b10) == 0) //xy mode->pose
    {
        Command_now.pos_sp[0] = pos_drone.pose.position.x + state_desired[0];
        Command_now.pos_sp[1] = pos_drone.pose.position.y + state_desired[1];
    }//xy mode->vel
    else
    {
        Command_now.vel_sp[0] = state_desired[0];
        Command_now.vel_sp[1] = state_desired[1];
    }

    if ((sub_mode & 0b01) == 0)//z mode->pose
    {
        //Command_now.pos_sp[2] = state_desired[2];
        Command_now.pos_sp[2] = fly_height ;
    }
    else //z mode->vel
    {
        Command_now.vel_sp[2] = state_desired[2];
    }

    Command_now.yaw_sp = state_desired[3];
    Command_now.comid = comid;
    comid++;
}

/*******
 * float_desired
 * 0 ----x
 * 1 ----y
 * 2 ----z
 * 3 ----yaw
 * *******/
void move_forward()
{
    float state_desired[4];
    state_desired[0] = 1.0;
    Command_now.command = Move_ENU;//dyxtest
    generate_com(0, state_desired);
    move_pub.publish(Command_now);
    cout << "Moving forward!!!!!!! "<<endl;
}

void move_backward()
{
    float state_desired[4];
    state_desired[0] = -1.0;
    Command_now.command = Move_ENU;
    generate_com(0, state_desired);
    move_pub.publish(Command_now);
    cout << "Moving backward!!!!!!! "<<endl;
}


void move_left()
{
    float state_desired[4];
    state_desired[1] = 1.0;
    Command_now.command = Move_ENU;
    generate_com(0, state_desired);
    move_pub.publish(Command_now);
    cout << "Moving left!!!!!!! "<<endl;
}

void move_right()
{
    float state_desired[4];
    state_desired[1] = -1.0;
    Command_now.command = Move_ENU;
    generate_com(0, state_desired);
    move_pub.publish(Command_now);
    cout << "Moving right!!!!!!! "<<endl;
}

void move_up()
{
    float state_desired[4];
    state_desired[2] = 0.3;
    Command_now.command = Move_Body;
    generate_com(0, state_desired);
    move_pub.publish(Command_now);
    cout << "Moving up!!!!!!! "<<endl;
}

void move_down()
{
    float state_desired[4];
    state_desired[2] = -0.3;
    Command_now.command = Move_Body;
    generate_com(0, state_desired);
    move_pub.publish(Command_now);
    cout << "Moving down!!!!!!! "<<endl;
}

void turn_left()
{
    float state_desired[4];
    state_desired[3] = 0.3;
    Command_now.command = Move_Body;
    generate_com(0, state_desired);
    move_pub.publish(Command_now);
    cout << "Turn left!!!!!!! "<<endl;
}

void turn_right()
{
    float state_desired[4];
    state_desired[2] = -0.3;
    Command_now.command = Move_Body;
    generate_com(0, state_desired);
    move_pub.publish(Command_now);
    cout << "Turn right!!!!!!! "<<endl;
}

void hold()
{
    Command_now.command = Hold;
    move_pub.publish(Command_now);
    cout << "Hold!!!!!!! "<<endl;
}

void land()
{
    Command_now.command = Land;
    move_pub.publish(Command_now);
    cout << "Landing!!!!!!! "<<endl;
}

void takeoff()
{
    Command_now.command = Takeoff;
    Command_now.sub_mode = 0;
    move_pub.publish(Command_now);
    cout << "Takingoff!!!!!!! "<<endl;
}

void disarm()
{
    Command_now.command = Disarm;
    move_pub.publish(Command_now);
    cout << "Disarm!!!!!!! "<<endl;
}

void arm()
{
    Command_now.command = Arm;
    move_pub.publish(Command_now);
    cout << "Arm!!!!!!! "<<endl;
}











