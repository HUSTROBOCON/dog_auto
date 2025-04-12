/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef TROTTING_H
#define TROTTING_H

#include "FSM/FSMState.h"
#include "Gait/GaitGenerator.h"
#include "control/BalanceCtrl.h"
// #include "CLFCBF/pos.h"
#include "Path/path.h"

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

class GazeboListener_state
{
public:
GazeboListener_state()
  {
    // 订阅Gazebo的ModelStates话题
    sub_1_p = nh_gazebo.subscribe("/gazebo/model_states", 1, &GazeboListener_state::modelStateCallback, this);
  }
  void modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);


private:
  ros::Subscriber sub_1_p;
  ros::NodeHandle nh_gazebo;
};



class State_Trotting : public FSMState{
public:
    State_Trotting(CtrlComponents *ctrlComp);
    ~State_Trotting();
    void enter();
    void run();
    void exit();
    virtual FSMStateName checkChange();
    void setHighCmd(double vx, double vy, double wz);

    static Vec3 _posGazebo;
    static Vec3 _velGazebo;

    static Vec3 _posnav;
    static Vec3 _velnav;

    // 细杆初始位置与第一根杆距离
    double start_gan = 0.5;
    // 赛道宽度
    double track_width = 1;
    // 细杆间距
    double gan_interval = 1.2;
    // 预设匀速速度
    double v_set = 0.3;
    // 总杆数
    int total_poles = 5;
    // flag
    int flag_gan = 0;
    int count = 0;

    
private:
    void calcTau();
    void calcQQd();
    void calcCmd();
    virtual void getUserCmd();
    void calcBalanceKp();
    bool checkStepOrNot();

    GaitGenerator *_gait;
    Estimator *_est;
    QuadrupedRobot *_robModel;
    BalanceCtrl *_balCtrl;
    Path *_pathplan;
    GazeboListener_state *gazebo_listener;

    // Rob State
    Vec3  _posBody, _velBody;
    double _yaw, _dYaw;
    Vec34 _posFeetGlobal, _velFeetGlobal;
    Vec34 _posFeet2BGlobal;
    RotMat _B2G_RotMat, _G2B_RotMat;
    Vec12 _q;

    // Robot command
    Vec3 _pcd;
    Vec3 _vCmdGlobal, _vCmdBody;
    double _yawCmd, _dYawCmd;
    double _dYawCmdPast;
    Vec3 _wCmdGlobal;
    Vec34 _posFeetGlobalGoal, _velFeetGlobalGoal;
    Vec34 _posFeet2BGoal, _velFeet2BGoal;
    RotMat _Rd;
    Vec3 _ddPcd, _dWbd;
    Vec34 _forceFeetGlobal, _forceFeetBody;
    Vec34 _qGoal, _qdGoal;
    Vec12 _tau;

    // Control Parameters
    double _gaitHeight;
    Vec3 _posError, _velError;
    Mat3 _Kpp, _Kdp, _Kdw;
    Mat2 _Kppp_gan, _Kddp_gan;
    Mat2 _Kppp_po, _Kddp_po;

    double Yaw_kp;
    double _kpw;
    Mat3 _KpSwing, _KdSwing;
    Vec2 _vxLim, _vyLim, _wyawLim;
    Vec4 *_phase;
    VecInt4 *_contact;

    // Calculate average value
    AvgCov *_avg_posError = new AvgCov(3, "_posError", true, 1000, 1000, 1);
    AvgCov *_avg_angError = new AvgCov(3, "_angError", true, 1000, 1000, 1000);

    Vec2 pos;

    ros::NodeHandle nh;

};




#endif  // TROTTING_H