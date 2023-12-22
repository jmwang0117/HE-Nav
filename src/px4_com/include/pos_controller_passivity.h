/***************************************************************************************************************************
* pos_controller_passivity.h
*
* Author: Qyp
*
* Update Time: 2019.5.1
*
* Introduction:  Position Controller using passivity+UDE method
***************************************************************************************************************************/
#ifndef POS_CONTROLLER_PASSIVITY_H
#define POS_CONTROLLER_PASSIVITY_H

#include <Eigen/Eigen>
#include <math.h>
#include <math_utils.h>

using namespace std;

namespace namespace_passivity{

class pos_controller_passivity
{
     //public表明该数据成员、成员函数是对全部用户开放的。全部用户都能够直接进行调用，在程序的不论什么其他地方訪问。
    public:

        //构造函数
        pos_controller_passivity(void):
            pos_passivity_nh("~")
        {
            pos_passivity_nh.param<float>("passivity_MASS", passivity_MASS, 1.0);
            pos_passivity_nh.param<float>("passivity_Kp_XY", passivity_Kp(0), 1.0);
            pos_passivity_nh.param<float>("passivity_Kp_XY", passivity_Kp(1), 1.0);
            pos_passivity_nh.param<float>("passivity_Kp_Z", passivity_Kp(2), 1.0);
            pos_passivity_nh.param<float>("passivity_Kd_XY", passivity_Kd(0), 2.0);
            pos_passivity_nh.param<float>("passivity_Kd_XY", passivity_Kd(1), 2.0);
            pos_passivity_nh.param<float>("passivity_Kd_Z", passivity_Kd(2), 2.0);
            pos_passivity_nh.param<float>("passivity_T_ude_XY", passivity_T_ude(0), 1.0);
            pos_passivity_nh.param<float>("passivity_T_ude_XY", passivity_T_ude(1), 1.0);
            pos_passivity_nh.param<float>("passivity_T_ude_Z", passivity_T_ude(2), 1.0);
            pos_passivity_nh.param<float>("passivity_T1", passivity_T1, 1.0);
            pos_passivity_nh.param<float>("passivity_T2", passivity_T2, 1.0);
            pos_passivity_nh.param<float>("passivity_INT_LIM_X", passivity_INT_LIM(0), 1.0);
            pos_passivity_nh.param<float>("passivity_INT_LIM_Y", passivity_INT_LIM(1), 1.0);
            pos_passivity_nh.param<float>("passivity_INT_LIM_Z", passivity_INT_LIM(2), 5.0);

            pos_passivity_nh.param<float>("passivity_XY_VEL_MAX", passivity_XY_VEL_MAX, 1.0);
            pos_passivity_nh.param<float>("passivity_Z_VEL_MAX", passivity_Z_VEL_MAX, 1.0);

            pos_passivity_nh.param<float>("passivity_THR_MIN", passivity_THR_MIN, 0.1);
            pos_passivity_nh.param<float>("passivity_THR_MAX", passivity_THR_MAX, 0.9);

            pos_passivity_nh.param<float>("passivity_tilt_max", passivity_tilt_max, 20.0);

            pos_passivity_nh.param<float>("passivity_a", passivity_a, 20.0);
            pos_passivity_nh.param<float>("passivity_b", passivity_b, 0.0);

            thrust_sp       = Eigen::Vector3d(0.0,0.0,0.0);
            u_l             = Eigen::Vector3d(0.0,0.0,0.0);
            u_d             = Eigen::Vector3d(0.0,0.0,0.0);
            u_total         = Eigen::Vector3d(0.0,0.0,0.0);
            integral_passivity    = Eigen::Vector3d(0.0,0.0,0.0);
            error_pos       = Eigen::Vector3d(0.0,0.0,0.0);
            error_vel       = Eigen::Vector3d(0.0,0.0,0.0);

            z_last       = Eigen::Vector3d(0.0,0.0,0.0);
            pos_last       = Eigen::Vector3d(0.0,0.0,0.0);
            error_last     = Eigen::Vector3d(0.0,0.0,0.0);
            y1_last       = Eigen::Vector3d(0.0,0.0,0.0);
            y2_last       = Eigen::Vector3d(0.0,0.0,0.0);
            y3_last       = Eigen::Vector3d(0.0,0.0,0.0);

            delta_time      = 0.0;
            flag_offboard   = 0;

            state_sub = pos_passivity_nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &pos_controller_passivity::state_cb,this);

        }

        //Mass of the quadrotor
        float passivity_MASS;

        //passivity control parameter
        Eigen::Vector3f passivity_Kp;

        Eigen::Vector3f passivity_Kd;

        Eigen::Vector3f passivity_T_ude;

        float passivity_T1;

        float passivity_T2;

        //Limitation of passivity integral
        Eigen::Vector3f passivity_INT_LIM;

        //Limitation of the velocity
        float passivity_XY_VEL_MAX;
        float passivity_Z_VEL_MAX;

        //Limitation of the thrust
        float passivity_THR_MIN;
        float passivity_THR_MAX;

        //Limitation of the tilt angle (roll and pitch)  [degree]
        float passivity_tilt_max;

        float passivity_a;
        float passivity_b;

        Eigen::Vector3d error_pos,error_vel;

        //u_l for nominal contorol(PD), u_d for passivity control(disturbance estimator)
        Eigen::Vector3d u_l,u_d,u_total;

        Eigen::Vector3d z_last,pos_last,error_last,y1_last,y2_last,y3_last;

        Eigen::Vector3d integral_passivity;

        //Desired thurst of the drone[the output of this class]
        Eigen::Vector3d thrust_sp;

        //The delta time between now and the last step
        float delta_time;

        //Current state of the drone
        mavros_msgs::State current_state;

        //Flag of the offboard mode [1 for OFFBOARD mode , 0 for non-OFFBOARD mode]
        int flag_offboard;

        //Printf the passivity parameter
        void printf_param();

        //Printf the control result
        void printf_result();

        //Position control main function [Input: current pos, current vel, desired state(pos or vel), sub_mode, time_now; Output: desired thrust;]
        Eigen::Vector3d pos_controller(Eigen::Vector3d pos, Eigen::Vector3d pos_sp, float curtime);

    private:

        ros::NodeHandle pos_passivity_nh;

        ros::Subscriber state_sub;

        void state_cb(const mavros_msgs::State::ConstPtr &msg)
        {
            current_state = *msg;

            if(current_state.mode == "OFFBOARD")
            {
                flag_offboard = 1;
            }else
            {
                flag_offboard = 0;
            }

        }

};




Eigen::Vector3d pos_controller_passivity::pos_controller(Eigen::Vector3d pos, Eigen::Vector3d pos_sp, float dt)
{
    delta_time = dt;

    error_pos = pos_sp - pos;

    //z_k
    Eigen::Vector3d z_k;
    z_k = 1.0f/(passivity_T1 + delta_time)*(passivity_T1 * z_last + error_pos - error_last);

    /* limit rates */
    // for (int i = 0; i < 3; i++)
    // {
    // 	z_k(i) = math::constrain(z_k(i), -0.2f, 0.2f);
    // }

    z_last = z_k;
    error_last = error_pos;

    //u_l
    for (int i = 0; i < 3; i++)
    {
       u_l(i) = passivity_MASS * (passivity_Kp(i) * error_pos(i) + passivity_Kd(i) * z_k(i));
    }

    //UDE term y1 y2 y3
    Eigen::Vector3d y1_k;
    for (int i = 0; i < 3; i++)
    {
        y1_k(i) = 1.0f/(passivity_T_ude(i) + delta_time) * (passivity_T_ude(i) * y1_last(i) + pos(i) - pos_last(i));
    }

    pos_last = pos;
    y1_last = y1_k;

    Eigen::Vector3d y2_k;
    for (int i = 0; i < 3; i++)
    {
        y2_k(i) = 1.0f/(passivity_T1 + delta_time) * (passivity_T1 * y2_last(i) + delta_time * error_pos(i));
        //y2_k(i) =   ( 0.98 * y2_last(i) + 0.0099 * (error_pos(i)+error_last(i)));

    }

    y2_last = y2_k;

   // y2_k = Eigen::Vector3d(0.0,0.0,0.0);
    Eigen::Vector3d y3_k;
    for (int i = 0; i < 3; i++)
    {
        y3_k(i) = 1.0f/(passivity_T_ude(i) + delta_time) * (passivity_T_ude(i) * y3_last(i) + delta_time * (passivity_Kp(i) * integral_passivity(i) + passivity_Kd(i)*y2_k(i)));
    }

    y3_last = y3_k;


    for (int i = 0; i < 3; i++)
    {
        u_d(i) = passivity_MASS * (y1_k(i) - y3_k(i));
    }

    /* explicitly limit the integrator state */
    for (int i = 0; i < 3; i++)
    {
        float integral = 0;
        if(error_pos(i) < 2)
        {
            integral = integral_passivity(i) +  error_pos(i) * delta_time;
        }

        if (u_d(i) > -passivity_INT_LIM(i) && u_d[i] < passivity_INT_LIM(i))
        {
                integral_passivity(i) = integral;
        }

        u_d(i) = constrain_function2(u_d(i), -passivity_INT_LIM(i), passivity_INT_LIM(i));
    }

    //ENU frame
    u_total(0) = u_l(0) - u_d(0);
    u_total(1) = u_l(1) - u_d(1);
    u_total(2) = u_l(2) - u_d(2) + passivity_MASS * 9.8;

    //Thrust to scale thrust[0,1]
    Eigen::Vector3d thrust_sp_scale;
    thrust_sp_scale(0) = (u_total(0) - passivity_b) / passivity_a;
    thrust_sp_scale(1) = (u_total(1) - passivity_b) / passivity_a;
    thrust_sp_scale(2) = (u_total(2) - passivity_b) / passivity_a;

    //Limit the Thrust
    thrust_sp(2) = constrain_function2( thrust_sp_scale(2) , passivity_THR_MIN, passivity_THR_MAX);

    // Get maximum allowed thrust in XY based on tilt angle and excess thrust.
    float thrust_max_XY_tilt = fabs(thrust_sp(2)) * tanf(passivity_tilt_max/180.0*M_PI);
    float thrust_max_XY = sqrtf(passivity_THR_MAX * passivity_THR_MAX - thrust_sp(2) * thrust_sp(2));
    thrust_max_XY = min(thrust_max_XY_tilt, thrust_max_XY);

    // Saturate thrust in XY-direction.
    thrust_sp(0) = thrust_sp_scale(0);
    thrust_sp(1) = thrust_sp_scale(1);

    if ((thrust_sp_scale(0) * thrust_sp_scale(0) + thrust_sp_scale(1) * thrust_sp_scale(1)) > thrust_max_XY * thrust_max_XY) {
            float mag = sqrtf((thrust_sp_scale(0) * thrust_sp_scale(0) + thrust_sp_scale(1) * thrust_sp_scale(1)));
            thrust_sp(0) = thrust_sp_scale(0) / mag * thrust_max_XY;
            thrust_sp(1) = thrust_sp_scale(1) / mag * thrust_max_XY;
    }

    //If not in OFFBOARD mode, set all intergral to zero.
    if(flag_offboard == 0)
    {
        integral_passivity = Eigen::Vector3d(0.0,0.0,0.0);
    }

    return thrust_sp;
}

void pos_controller_passivity::printf_result()
{
    cout <<">>>>>>>>>>>>>>>>>>>>Passivity Position Controller<<<<<<<<<<<<<<<<<<<<<" <<endl;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout<<setprecision(2);

    cout << "delta_time : " << delta_time<< " [s] " <<endl;

    cout << "u_l [X Y Z] : " << u_l[0] << " [N] "<< u_l[1]<<" [N] "<<u_l[2]<<" [N] "<<endl;
    cout << "z_k [X Y Z] : " << z_last[0] << " [N] "<< z_last[1]<<" [N] "<<z_last[2]<<" [N] "<<endl;

    cout << "y1 [X Y Z] : " << y1_last[0] << " [N] "<< y1_last[1]<<" [N] "<<y1_last[2]<<" [N] "<<endl;

    cout << "y2 [X Y Z] : " << y2_last[0] << " [N] "<< y2_last[1]<<" [N] "<<y2_last[2]<<" [N] "<<endl;

    cout << "y3 [X Y Z] : " << y3_last[0] << " [N] "<< y3_last[1]<<" [N] "<<y3_last[2]<<" [N] "<<endl;

    cout << "u_l [X Y Z] : " << u_l[0] << " [N] "<< u_l[1]<<" [N] "<<u_l[2]<<" [N] "<<endl;

    cout << "u_d [X Y Z] : " << u_d[0] << " [N] "<< u_d[1]<<" [N] "<<u_d[2]<<" [N] "<<endl;

    cout << "u_total [X Y Z] : " << u_total[0] << " [N] "<< u_total[1]<<" [N] "<<u_total[2]<<" [N] "<<endl;

    cout << "thrust_sp [X Y Z] : " << thrust_sp[0] << " [m/s^2] "<< thrust_sp[1]<<" [m/s^2] "<<thrust_sp[2]<<" [m/s^2] "<<endl;
}

// 【打印参数函数】
void pos_controller_passivity::printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>passivity Parameter <<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout <<"passivity_MASS : "<< passivity_MASS << endl;

    cout <<"passivity_Kp_X : "<< passivity_Kp(0) << endl;
    cout <<"passivity_Kp_Y : "<< passivity_Kp(1) << endl;
    cout <<"passivity_Kp_Z : "<< passivity_Kp(2) << endl;

    cout <<"passivity_Kd_X : "<< passivity_Kd(0) << endl;
    cout <<"passivity_Kd_Y : "<< passivity_Kd(1) << endl;
    cout <<"passivity_Kd_Z : "<< passivity_Kd(2) << endl;

    cout <<"passivity_T_X : "<< passivity_T_ude(0) << endl;
    cout <<"passivity_T_Y : "<< passivity_T_ude(1) << endl;
    cout <<"passivity_T_Z : "<< passivity_T_ude(2) << endl;
    cout <<"passivity_T1 : "<< passivity_T1 << endl;
    cout <<"passivity_T2 : "<< passivity_T2 << endl;

    cout <<"passivity_XY_VEL_MAX : "<< passivity_XY_VEL_MAX << endl;
    cout <<"passivity_Z_VEL_MAX : "<< passivity_Z_VEL_MAX << endl;

    cout <<"passivity_INT_LIM_X : "<< passivity_INT_LIM(0) << endl;
    cout <<"passivity_INT_LIM_Y : "<< passivity_INT_LIM(1) << endl;
    cout <<"passivity_INT_LIM_Z : "<< passivity_INT_LIM(2) << endl;

    cout <<"passivity_THR_MIN : "<< passivity_THR_MIN << endl;
    cout <<"passivity_THR_MAX : "<< passivity_THR_MAX << endl;

    cout <<"passivity_tilt_max : "<< passivity_tilt_max << endl;
    cout <<"passivity_a : "<< passivity_a << endl;

    cout <<"passivity_b : "<< passivity_b << endl;

}

}
#endif
