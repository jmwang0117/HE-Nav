#include<ros/ros.h>
#include<iostream>
#include<stdio.h>
#include<cstdlib>
#include<time.h>
#include<math.h>

#include<px4_command/command.h>
#include<geometry_msgs/Point.h>
#include<geometry_msgs/Pose.h>


using namespace std;

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

float offset_horizontal;
float offset_vertical;
float sleep_time;

geometry_msgs::Pose pos_drone;
px4_command::command Command_now;

void generate_com(int sub_mode, float state_desired[4]);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "path");
	ros::NodeHandle nh("~");
	
	ros::Rate rate(1.0);
	
	ros::Publisher move_pub = nh.advertise<px4_command::command>("/px4/command", 10);
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    nh.param<float>("offset_horizontal", offset_horizontal, 1.0);
    nh.param<float>("offset_vertical", offset_vertical, 1.0);
    nh.param<float>("sleep_time", sleep_time, 10.0);
	
	int check_flag;
	cout << "Please check the parameter and setting，1 for go on， else for quit: "<<endl;
	cin >> check_flag;
	if(check_flag != 1)
    {
        return -1;
    }

    int step = 0;
    int count = 0;
    //take off
    if (step == 0)
    {   
		cout << "Takeoff !!!!!!!!!!"<<endl;
		count = 0;
		Command_now.command = Takeoff;
		move_pub.publish(Command_now);		
		while (count < sleep_time)
		{
			rate.sleep();
			count ++;
		}
		step = 1;
	}
	//forward 1m
	if (step == 1)
	{   
		cout << "Forward !!!!!!!!!!!!!"<<endl;
		count = 0;
		float state_desired[4];
		state_desired[0] = offset_vertical;
                Command_now.command = Move_Body;
		generate_com(0, state_desired);
		move_pub.publish(Command_now);		
		while (count < sleep_time)
		{
			rate.sleep();
			count ++;
		}
		step = 2;
	}
	//left 2m
    if (step == 2)
	{   
		cout << "Left !!!!!!!!!!!!!"<<endl;
		count = 0;
		float state_desired[4];
		state_desired[1] = offset_horizontal;
                Command_now.command = Move_Body;
		generate_com(0, state_desired);
		move_pub.publish(Command_now);		
		while (count < sleep_time)
		{
			rate.sleep();
			count ++;
		}
		step = 3;
	}
	//forward 5m
	if (step == 3)
	{   
		cout << "Forward !!!!!!!!!!!!!"<<endl;
		count = 0;
		float state_desired[4];
		state_desired[0] = offset_vertical;
                Command_now.command = Move_Body;
		generate_com(0, state_desired);
		move_pub.publish(Command_now);		
		while (count < sleep_time)
		{
			rate.sleep();
			count ++;
		}
		step = 4;
	}
	//right 2m
	if (step == 4)
	{   
		cout << "Right !!!!!!!!!!!!!"<<endl;
		count = 0;
		float state_desired[4];
		state_desired[1] = -offset_horizontal;
                Command_now.command = Move_Body;
		generate_com(0, state_desired);
		move_pub.publish(Command_now);		
		while (count < sleep_time)
		{
			rate.sleep();
			count ++;
		}
		step = 5;
	}
	//land
	if (step == 5)
	{   
		cout << "Land !!!!!!!!!!!!!"<<endl;
		step = 6;
		Command_now.command = Land;
		move_pub.publish(Command_now);	
	}
    
}

void generate_com(int sub_mode, float state_desired[4])
{
	static int comid = 1;
	Command_now.sub_mode = sub_mode;
	if((sub_mode & 0b10) == 0)
	{
		Command_now.pos_sp[0] = state_desired[0];
		Command_now.pos_sp[1] = state_desired[1];
	}
	else
	{
		Command_now.vel_sp[0] = state_desired[0];
		Command_now.vel_sp[1] = state_desired[1];
	}
	
	if((sub_mode & 0b01) == 0)
	{
		Command_now.pos_sp[2] = state_desired[2];
	}
	else
	{
		Command_now.vel_sp[2] = state_desired[2];
	}	
    Command_now.yaw_sp = state_desired[3];
    Command_now.comid = comid;
    comid++;
}






