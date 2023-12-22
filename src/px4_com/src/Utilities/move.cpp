/***************************************************************************************************************************
* move.cpp
*
* Author: Qyp
*
* Update Time: 2019.3.16
*
* Introduction:  test function for sending command.msg
***************************************************************************************************************************/
#include <ros/ros.h>

#include <iostream>
#include <px4_command/command.h>

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

ros::Publisher move_pub;
int Num_StateMachine;
px4_command::command Command_now;
void generate_com(int sub_mode, float state_desired[4]);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move");
    ros::NodeHandle nh;

    move_pub = nh.advertise<px4_command::command>("/px4/command", 10);


    int flag_1;
    float state_desired[4];
    int sub_mode;

    Num_StateMachine = 0;
    //----------------------------------
    //input
    while(ros::ok())
    {
        switch (Num_StateMachine)
        {
            // input
            case 0:
                cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>--------<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
                cout << "Input the flag:  0 for Move_ENU，1 for Move_Body，2 for Land,3 for Disarm ,4 for Hold, 5 for Failsafe Land, 6 for Idle,7 for Takeoff to default height"<<endl;
                cin >> flag_1;

                if (flag_1 == 2)
                {
                    Num_StateMachine = 1;
                    break;
                }

                if (flag_1 == 3)
                {
                    Num_StateMachine = 2;
                    break;
                }

                if (flag_1 == 4)
                {
                    Num_StateMachine = 5;
                    break;
                }

                if (flag_1 == 5)
                {
                    Num_StateMachine = 6;
                    break;
                }

                if (flag_1 == 6)
                {
                    Num_StateMachine = 7;
                    break;
                }

                if (flag_1 == 7)
                {
                    Num_StateMachine = 8;
                    break;
                }


                //如果是机体系移动
                if(flag_1 == 1)
                {
                    Num_StateMachine = 4;
                }//惯性系移动
                else if(flag_1 == 0)
                {
                    Num_StateMachine = 3;
                }

                cout << "Input the sub_mode:  # 0 for xy/z position control; 3 for xy/z velocity control"<<endl;
                cin >> sub_mode;

                cout << "Please input next setpoint [x y z yaw]: "<< endl;

                cout << "setpoint_t[0] --- x [m] : "<< endl;
                cin >> state_desired[0];
                cout << "setpoint_t[1] --- y [m] : "<< endl;
                cin >> state_desired[1];
                cout << "setpoint_t[2] --- z [m] : "<< endl;
                cin >> state_desired[2];
                cout << "setpoint_t[3] --- yaw [du] : "<< endl;
                cout << "500 for input again: "<< endl;
                cin >> state_desired[3];

                //500  重新输入各数值
                if (state_desired[3] == 500)
                {
                    Num_StateMachine = 0;
                }

                break;

        //Land
        case 1:
            Command_now.command = Land;
            move_pub.publish(Command_now);
            Num_StateMachine = 0;
            break;

        //Disarm
        case 2:
            Command_now.command = Disarm;
            move_pub.publish(Command_now);
            Num_StateMachine = 0;
            break;

        //Move_ENU
        case 3:
            Command_now.command = Move_ENU;
            generate_com(sub_mode, state_desired);
            move_pub.publish(Command_now);
            Num_StateMachine = 0;
            break;

        //Move_Body
        case 4:
            Command_now.command = Move_Body;
            generate_com(sub_mode, state_desired);
            move_pub.publish(Command_now);
            Num_StateMachine = 0;
            break;

        //Hold
        case 5:
            Command_now.command = Hold;
            move_pub.publish(Command_now);
            Num_StateMachine = 0;
            break;

        //Failsafe_Land
        case 6:
            Command_now.command = Failsafe_land;
            move_pub.publish(Command_now);
            Num_StateMachine = 0;
            break;

        //Custom
        case 7:
            Command_now.command = Idle;
            move_pub.publish(Command_now);
            Num_StateMachine = 0;
            break;

        //Custom
        case 8:
            Command_now.command = Takeoff;
            move_pub.publish(Command_now);
            Num_StateMachine = 0;
            break;

        }

        sleep(0.2);
    }

    return 0;
}

// float32[3] pos_sp
// float32[3] vel_sp
// float32 yaw_sp
void generate_com(int sub_mode, float state_desired[4]){

    static int comid = 1;
    Command_now.sub_mode = sub_mode;

//# sub_mode 2-bit value:
//# 0 for position, 1 for vel, 1st for xy, 2nd for z.
//#                   xy position     xy velocity
//# z position       	0b00(0)       0b10(2)
//# z velocity		0b01(1)       0b11(3)

    if((sub_mode & 0b10) == 0) //xy channel
    {
        Command_now.pos_sp[0] = state_desired[0];
        Command_now.pos_sp[1] = state_desired[1];
    }
    else
    {
        Command_now.vel_sp[0] = state_desired[0];
        Command_now.vel_sp[1] = state_desired[1];
    }

    if((sub_mode & 0b01) == 0) //z channel
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
