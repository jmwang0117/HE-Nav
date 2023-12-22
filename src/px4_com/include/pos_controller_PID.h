/***************************************************************************************************************************
* pos_controller_PID.h
*
* Author: Qyp
*
* Update Time: 2019.5.1
*
* Introduction:  Position Controller using PID (P for pos loop, pid for vel loop)
*         1. Similiar to the position controller in PX4 (1.8.2)
*         2. Ref to : https://github.com/PX4/Firmware/blob/master/src/modules/mc_pos_control/PositionControl.cpp
*         3. Here we didn't consider the mass of the drone, we treat accel_sp is the thrust_sp.
*         4. thrustToAttitude ref to https://github.com/PX4/Firmware/blob/master/src/modules/mc_pos_control/Utility/ControlMath.cpp
*         5. For the derrive of the velocity error, we use a low-pass filter as same in PX4.
*                   Ref to: https://github.com/PX4/Firmware/blob/master/src/lib/controllib/BlockDerivative.cpp
*                           https://github.com/PX4/Firmware/blob/master/src/lib/controllib/BlockLowPass.cpp
*         6. 没有考虑积分器清零的情况，在降落时 或者突然换方向机动时，积分器需要清0
*         7. 推力到欧拉角基本与PX4吻合，但是在极端情况下不吻合。如：z轴期望值为-100时。
***************************************************************************************************************************/
#ifndef POS_CONTROLLER_PID_H
#define POS_CONTROLLER_PID_H

#include <Eigen/Eigen>
#include <math.h>
#include <math_utils.h>

using namespace std;

namespace namespace_PID {

class pos_controller_PID
{
     //public表明该数据成员、成员函数是对全部用户开放的。全部用户都能够直接进行调用，在程序的不论什么其他地方訪问。
    public:

        //构造函数
        pos_controller_PID(void):
            pos_pid_nh("~")
        {
            pos_pid_nh.param<float>("MPC_XY_P", MPC_XY_P, 1.0);
            pos_pid_nh.param<float>("MPC_Z_P", MPC_Z_P, 1.0);
            pos_pid_nh.param<float>("MPC_XY_VEL_P", MPC_XY_VEL_P, 0.1);
            pos_pid_nh.param<float>("MPC_Z_VEL_P", MPC_Z_VEL_P, 0.1);
            pos_pid_nh.param<float>("MPC_XY_VEL_I", MPC_XY_VEL_I, 0.02);
            pos_pid_nh.param<float>("MPC_Z_VEL_I", MPC_Z_VEL_I, 0.02);
            pos_pid_nh.param<float>("MPC_XY_VEL_D", MPC_XY_VEL_D, 0.01);
            pos_pid_nh.param<float>("MPC_Z_VEL_D", MPC_Z_VEL_D, 0.01);
            pos_pid_nh.param<float>("MPC_XY_VEL_MAX", MPC_XY_VEL_MAX, 1.0);
            pos_pid_nh.param<float>("MPC_Z_VEL_MAX", MPC_Z_VEL_MAX, 0.5);
            pos_pid_nh.param<float>("MPC_THRUST_HOVER", MPC_THRUST_HOVER, 0.4);
            pos_pid_nh.param<float>("MPC_THR_MIN", MPC_THR_MIN, 0.1);
            pos_pid_nh.param<float>("MPC_THR_MAX", MPC_THR_MAX, 0.9);
            pos_pid_nh.param<float>("tilt_max", tilt_max, 5.0);
            pos_pid_nh.param<float>("MPC_VELD_LP", MPC_VELD_LP, 5.0);

            pos_drone       = Eigen::Vector3d(0.0,0.0,0.0);
            vel_drone       = Eigen::Vector3d(0.0,0.0,0.0);
            vel_setpoint    = Eigen::Vector3d(0.0,0.0,0.0);
            thrust_sp       = Eigen::Vector3d(0.0,0.0,0.0);
            vel_P_output    = Eigen::Vector3d(0.0,0.0,0.0);
            thurst_int      = Eigen::Vector3d(0.0,0.0,0.0);
            vel_D_output    = Eigen::Vector3d(0.0,0.0,0.0);
            error_vel_dot_last  = Eigen::Vector3d(0.0,0.0,0.0);
            error_vel_last      = Eigen::Vector3d(0.0,0.0,0.0);
            delta_time      = 0.02;
            flag_offboard   = 0;

            state_sub = pos_pid_nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &pos_controller_PID::state_cb,this);

        }

        //PID parameter for the control law
        float MPC_XY_P;
        float MPC_Z_P;
        float MPC_XY_VEL_P;
        float MPC_Z_VEL_P;
        float MPC_XY_VEL_I;
        float MPC_Z_VEL_I;
        float MPC_XY_VEL_D;
        float MPC_Z_VEL_D;

        //Limitation of the velocity
        float MPC_XY_VEL_MAX;
        float MPC_Z_VEL_MAX;

        //Hover thrust of drone (decided by the mass of the drone)
        float MPC_THRUST_HOVER;

        //Limitation of the thrust
        float MPC_THR_MIN;
        float MPC_THR_MAX;

        //Limitation of the tilt angle (roll and pitch)  [degree]
        float tilt_max;

        //Current position and velocity of the drone
        Eigen::Vector3d pos_drone;
        Eigen::Vector3d vel_drone;

        //Desired position and velocity of the drone
        Eigen::Vector3d vel_setpoint;

        //Desired thurst of the drone[the output of this class]
        Eigen::Vector3d thrust_sp;

        //Output of the vel loop in PID [thurst_int is the I]
        Eigen::Vector3d vel_P_output;
        Eigen::Vector3d thurst_int;
        Eigen::Vector3d vel_D_output;

        float MPC_VELD_LP;

        //The delta time between now and the last step
        float delta_time;

        //Derriv of the velocity error in last step [used for the D-output in vel loop]
        Eigen::Vector3d error_vel_dot_last;
        Eigen::Vector3d error_vel_last;

        //Current state of the drone
        mavros_msgs::State current_state;

        //Flag of the offboard mode [1 for OFFBOARD mode , 0 for non-OFFBOARD mode]
        int flag_offboard;

        // Output of thrustToAttitude
        Eigen::Vector3d euler_sp;

        //Printf the PID parameter
        void printf_param();

        //Printf the control result
        void printf_result();

        //Position control main function [Input: current pos, current vel, desired state(pos or vel), sub_mode, time_now; Output: desired thrust;]
        Eigen::Vector3d pos_controller(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d pos_sp, Eigen::Vector3d vel_sp, int sub_mode, float curtime);

        //Position control loop [Input: current pos, desired pos; Output: desired vel]
        void _positionController(Eigen::Vector3d pos_sp, Eigen::Vector3d vel_sp, int sub_mode);

        //Velocity control loop [Input: current vel, desired vel; Output: desired thrust]
        void _velocityController();

        Eigen::Vector3d cal_vel_error_deriv(Eigen::Vector3d error_now);

    private:

        ros::NodeHandle pos_pid_nh;

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


Eigen::Vector3d pos_controller_PID::pos_controller(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d pos_sp, Eigen::Vector3d vel_sp, int sub_mode, float dt)
{
    pos_drone = pos;
    vel_drone = vel;

    delta_time = dt;

    _positionController(pos_sp, vel_sp, sub_mode);

    _velocityController();

    return thrust_sp;
}


void pos_controller_PID::_positionController(Eigen::Vector3d pos_sp, Eigen::Vector3d vel_sp, int sub_mode)
{
    //# sub_mode 2-bit value:
    //# 0 for position, 1 for vel, 1st for xy, 2nd for z.
    //#                   xy position     xy velocity
    //# z position       	0b00(0)       0b10(2)
    //# z velocity		0b01(1)       0b11(3)

    if((sub_mode & 0b10) == 0) //xy channel
    {
        vel_setpoint(0) = MPC_XY_P * (pos_sp(0) - pos_drone(0));
        vel_setpoint(1) = MPC_XY_P * (pos_sp(1) - pos_drone(1));
    }
    else
    {
        vel_setpoint(0) = vel_sp(0);
        vel_setpoint(1) = vel_sp(1);
    }

    if((sub_mode & 0b01) == 0) //z channel
    {
        vel_setpoint(2) = MPC_Z_P  * (pos_sp(2) - pos_drone(2));
    }
    else
    {
        vel_setpoint(2) = vel_sp(2);
    }

    // Limit the velocity setpoint
    vel_setpoint(0) = constrain_function2(vel_setpoint(0), -MPC_XY_VEL_MAX, MPC_XY_VEL_MAX);
    vel_setpoint(1) = constrain_function2(vel_setpoint(1), -MPC_XY_VEL_MAX, MPC_XY_VEL_MAX);
    vel_setpoint(2) = constrain_function2(vel_setpoint(2), -MPC_Z_VEL_MAX, MPC_Z_VEL_MAX);
}

void pos_controller_PID::_velocityController()
{
    // Generate desired thrust setpoint.
    // PID
    // u_des = P(error_vel) + D(error_vel_dot) + I(vel_integral)
    // Umin <= u_des <= Umax
    //
    // Anti-Windup:
    // u_des = _thrust_sp; y = _vel_sp; r = _vel
    // u_des >= Umax and r - y >= 0 => Saturation = true
    // u_des >= Umax and r - y <= 0 => Saturation = false
    // u_des <= Umin and r - y <= 0 => Saturation = true
    // u_des <= Umin and r - y >= 0 => Saturation = false
    //
    // 	Notes:
    // - control output in Z-direction has priority over XY-direction
    // - the equilibrium point for the PID is at hover-thrust

    // - the desired thrust in Z-direction is limited by the thrust limits
    // - the desired thrust in XY-direction is limited by the thrust excess after
    // 	 consideration of the desired thrust in Z-direction. In addition, the thrust in
    // 	 XY-direction is also limited by the maximum tilt.

    Eigen::Vector3d error_vel = vel_setpoint - vel_drone;

    vel_P_output(0) = MPC_XY_VEL_P * error_vel(0);
    vel_P_output(1) = MPC_XY_VEL_P * error_vel(1);
    vel_P_output(2) = MPC_Z_VEL_P  * error_vel(2);

    Eigen::Vector3d vel_error_deriv = cal_vel_error_deriv(error_vel);

    vel_D_output(0) = MPC_XY_VEL_D * vel_error_deriv(0);
    vel_D_output(1) = MPC_XY_VEL_D * vel_error_deriv(1);
    vel_D_output(2) = MPC_Z_VEL_D  * vel_error_deriv(2);


    // Consider thrust in Z-direction. [Here MPC_THRUST_HOVER is added]
    float thrust_desired_Z  = vel_P_output(2) + thurst_int(2) + vel_D_output(2) + MPC_THRUST_HOVER;

    // Apply Anti-Windup in Z-direction.
    // 两种情况：期望推力大于最大推力，且速度误差朝上；期望推力小于最小推力，且速度误差朝下
    bool stop_integral_Z = ( thrust_desired_Z  >= MPC_THR_MAX && error_vel(2) >= 0.0f) ||
                           ( thrust_desired_Z  <= MPC_THR_MIN && error_vel(2) <= 0.0f);
    if (!stop_integral_Z) {
            thurst_int(2) += MPC_Z_VEL_I  * error_vel(2) * delta_time;

            // limit thrust integral
            thurst_int(2) = min(fabs(thurst_int(2)), MPC_THR_MAX ) * sign_function(thurst_int(2));
    }

    // Saturate thrust setpoint in Z-direction.
    thrust_sp(2) = constrain_function2( thrust_desired_Z , MPC_THR_MIN, MPC_THR_MAX);

    // PID-velocity controller for XY-direction.
    float thrust_desired_X;
    float thrust_desired_Y;
    thrust_desired_X  = vel_P_output(0) + thurst_int(0) + vel_D_output(0);
    thrust_desired_Y  = vel_P_output(1) + thurst_int(1) + vel_D_output(1);

    // Get maximum allowed thrust in XY based on tilt angle and excess thrust.
    float thrust_max_XY_tilt = fabs(thrust_sp(2)) * tanf(tilt_max/180.0*M_PI);
    float thrust_max_XY = sqrtf(MPC_THR_MAX * MPC_THR_MAX - thrust_sp(2) * thrust_sp(2));
    thrust_max_XY = min(thrust_max_XY_tilt, thrust_max_XY);

    // Saturate thrust in XY-direction.
    thrust_sp(0) = thrust_desired_X;
    thrust_sp(1) = thrust_desired_Y;


    if ((thrust_desired_X * thrust_desired_X + thrust_desired_Y * thrust_desired_Y) > thrust_max_XY * thrust_max_XY) {
            float mag = sqrtf((thrust_desired_X * thrust_desired_X + thrust_desired_Y * thrust_desired_Y));
            thrust_sp(0) = thrust_desired_X / mag * thrust_max_XY;
            thrust_sp(1) = thrust_desired_Y / mag * thrust_max_XY;
    }

    // Use tracking Anti-Windup for XY-direction: during saturation, the integrator is used to unsaturate the output
    // see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
    // Actually, I dont understand here.
    float arw_gain = 2.f / MPC_XY_VEL_P;

    float vel_err_lim_x,vel_err_lim_y;
    vel_err_lim_x = error_vel(0) - (thrust_desired_X - thrust_sp(0)) * arw_gain;
    vel_err_lim_y = error_vel(1) - (thrust_desired_Y - thrust_sp(1)) * arw_gain;

    // Update integral
    thurst_int(0) += MPC_XY_VEL_I * vel_err_lim_x * delta_time;
    thurst_int(1) += MPC_XY_VEL_I * vel_err_lim_y * delta_time;

    //If not in OFFBOARD mode, set all intergral to zero.
    if(flag_offboard == 0)
    {
        thurst_int = Eigen::Vector3d(0.0,0.0,0.0);
    //thurst_int(2) =0.0;
    }
    cout<<"thust in z: "<<thrust_sp(2)<<endl;
    cout<<"thrustint z: "<<thurst_int(2)<<endl;
    cout<<"delta height "<<pos_drone(2)-1.3<<endl;
    cout<<"vel_setpoint_z: "<<vel_setpoint(2)<<endl;
    cout<<"vel_current__z: "<<vel_drone(2)<<endl;

}

Eigen::Vector3d pos_controller_PID::cal_vel_error_deriv(Eigen::Vector3d error_now)
{
    Eigen::Vector3d error_vel_dot_now;
    error_vel_dot_now = (error_now - error_vel_last)/delta_time;

    error_vel_last = error_now;
    float a,b;
    b = 2 * M_PI * MPC_VELD_LP * delta_time;
    a = b / (1 + b);

    Eigen::Vector3d output;

    output = a * error_vel_dot_now + (1 - a) * error_vel_dot_last ;

    error_vel_dot_last = output;

    return output;
}


void pos_controller_PID::printf_result()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Position Controller<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout<<setprecision(2);

//    cout << "delta_time : " << delta_time<< " [s] " <<endl;

    cout << "Velocity_sp  [X Y Z] : " << vel_setpoint[0] << " [m/s] "<< vel_setpoint[1]<<" [m/s] "<<vel_setpoint[2]<<" [m/s] "<<endl;

//    cout << "Vel_P_output [X Y Z] : " << vel_P_output[0] << " [m/s] "<< vel_P_output[1]<<" [m/s] "<<vel_P_output[2]<<" [m/s] "<<endl;

//    cout << "Vel_I_output [X Y Z] : " << thurst_int[0] << " [m/s] "<< thurst_int[1]<<" [m/s] "<<thurst_int[2]<<" [m/s] "<<endl;

//    cout << "Vel_D_output [X Y Z] : " << vel_D_output[0] << " [m/s] "<< vel_D_output[1]<<" [m/s] "<<vel_D_output[2]<<" [m/s] "<<endl;

    cout << "thrust_sp    [X Y Z] : " << thrust_sp[0] << " [m/s^2] "<< thrust_sp[1]<<" [m/s^2] "<<thrust_sp[2]<<" [m/s^2] "<<endl;
}

// 【打印参数函数】
void pos_controller_PID::printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PID Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout <<"Position Loop:  " <<endl;
    cout <<"MPC_XY_P : "<< MPC_XY_P << endl;
    cout <<"MPC_Z_P : "<< MPC_Z_P << endl;
    cout <<"Velocity Loop:  " <<endl;
    cout <<"MPC_XY_VEL_P : "<< MPC_XY_VEL_P << endl;
    cout <<"MPC_Z_VEL_P : "<< MPC_Z_VEL_P << endl;
    cout <<"MPC_XY_VEL_I : "<< MPC_XY_VEL_I << endl;
    cout <<"MPC_Z_VEL_I : "<< MPC_Z_VEL_I << endl;
    cout <<"MPC_XY_VEL_D : "<< MPC_XY_VEL_D << endl;
    cout <<"MPC_Z_VEL_D : "<< MPC_Z_VEL_D << endl;

    cout <<"Limit:  " <<endl;
    cout <<"MPC_XY_VEL_MAX : "<< MPC_XY_VEL_MAX << endl;
    cout <<"MPC_Z_VEL_MAX : "<< MPC_Z_VEL_MAX << endl;


    cout <<"tilt_max : "<< tilt_max << endl;
    cout <<"MPC_THR_MIN : "<< MPC_THR_MIN << endl;
    cout <<"MPC_THR_MAX : "<< MPC_THR_MAX << endl;
    cout <<"MPC_THRUST_HOVER : "<< MPC_THRUST_HOVER << endl;

}



}
#endif
