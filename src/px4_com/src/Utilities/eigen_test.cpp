
//头文件
#include <ros/ros.h>


#include <iostream>

//话题头文件

#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


#include <Eigen/Eigen>


using namespace std;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh("~");

    ros::Rate rate(20.0);
    Eigen::Vector3d vector;

    vector       = Eigen::Vector3d(0.0,0.0,0.0);


    float a;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {

                cout << "Setting to POSCTL Mode..." <<endl;


                a = vector.norm();

        //周期休眠
        rate.sleep();
    }

    return 0;

}
