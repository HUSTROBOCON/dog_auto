/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_Trotting.h"
#include <iomanip>

Vec3 State_Trotting::_posGazebo(0.0, 0.0, 0.0); // 初始化静态成员
Vec3 State_Trotting::_velGazebo(0.0, 0.0, 0.0);

State_Trotting::State_Trotting(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::TROTTING, "trotting"),
      _est(ctrlComp->estimator), _phase(ctrlComp->phase),
      _contact(ctrlComp->contact), _robModel(ctrlComp->robotModel),
      _balCtrl(ctrlComp->balCtrl), _pathplan(ctrlComp->pathplan)
{
    _gait = new GaitGenerator(ctrlComp);
    gazebo_listener = new GazeboListener_state();
    _pathplan = new Path();

    _gaitHeight = 0.08;

    _pathplan->cal_pathpoints(); // 计算路径点

    int argc = 0;
    char **argv = nullptr;
    // ros::init(argc, argv, "gazebo_model_state_subscriber");

#ifdef ROBOT_TYPE_Go1
    _Kpp = Vec3(70, 70, 70).asDiagonal();
    _Kdp = Vec3(10, 10, 10).asDiagonal();
    _kpw = 780;
    _Kdw = Vec3(70, 70, 70).asDiagonal();
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
#endif

#ifdef ROBOT_TYPE_A1
    _Kpp = Vec3(20, 20, 100).asDiagonal(); // 20, 20, 100
    _Kdp = Vec3(20, 20, 20).asDiagonal();
    _kpw = 400;
    _Kdw = Vec3(50, 50, 50).asDiagonal();
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
#endif
    _Kddp_gan = Vec2(1, 1).asDiagonal();  // 50,100
    _Kppp_gan = Vec2(1, 1).asDiagonal();  // 50,200
    _Kddp_po = Vec2(0.9, 0.9).asDiagonal();  // 50,100
    _Kppp_po = Vec2(3, 3).asDiagonal();  // 50,200
    Yaw_kp = 100;
    _vxLim = _robModel->getRobVelLimitX();
    _vyLim = _robModel->getRobVelLimitY();
    _wyawLim = _robModel->getRobVelLimitYaw();
    pos << 0,0;
}

State_Trotting::~State_Trotting()
{
    delete _gait;
}

void State_Trotting::enter()
{
    _pcd = _est->getPosition();
    _pcd(2) = -_robModel->getFeetPosIdeal()(2, 0);
    _vCmdBody.setZero();
    _yawCmd = _lowState->getYaw();
    _Rd = rotz(_yawCmd);
    _wCmdGlobal.setZero();

    _ctrlComp->ioInter->zeroCmdPanel();
    _gait->restart();
}

void State_Trotting::exit()
{
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();
}

FSMStateName State_Trotting::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B)
    {
        return FSMStateName::PASSIVE;
    }
    else if (_lowState->userCmd == UserCommand::L2_A)
    {
        return FSMStateName::FIXEDSTAND;
    }
    else
    {
        return FSMStateName::TROTTING;
    }
}

void State_Trotting::run()
{
    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();
    // std::cout<<"pos:"<<_posBody<<std::endl;
    // std::cout<<"vel:"<<_velBody<<std::endl;
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();
    _yaw = _lowState->getYaw();
    _dYaw = _lowState->getDYaw();

    _userValue = _lowState->userValue;

    getUserCmd();
    calcCmd();

    _gait->setGait(_vCmdGlobal.segment(0, 2), _wCmdGlobal(2), _gaitHeight);
    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);

    calcTau();
    calcQQd();

    if (checkStepOrNot())
    {
        _ctrlComp->setStartWave();
    }
    else
    {
        _ctrlComp->setAllStance();
    }

    _lowCmd->setTau(_tau);
    _lowCmd->setQ(vec34ToVec12(_qGoal));
    _lowCmd->setQd(vec34ToVec12(_qdGoal));

    for (int i(0); i < 4; ++i)
    {
        if ((*_contact)(i) == 0)
        {
            _lowCmd->setSwingGain(i);
        }
        else
        {
            _lowCmd->setStableGain(i);
        }
    }
}

bool State_Trotting::checkStepOrNot()
{
    // if( (fabs(_vCmdBody(0)) > 0.03) ||
    //     (fabs(_vCmdBody(1)) > 0.03) ||
    //     (fabs(_posError(0)) > 0.08) ||
    //     (fabs(_posError(1)) > 0.08) ||
    //     (fabs(_velError(0)) > 0.05) ||
    //     (fabs(_velError(1)) > 0.05) ||
    //     (fabs(_dYawCmd) > 0.20) ){
    //     return true;
    // }else{
    //     return false;
    // }
    return true;
}

void State_Trotting::setHighCmd(double vx, double vy, double wz)
{
    _vCmdBody(0) = vx;
    _vCmdBody(1) = vy;
    _vCmdBody(2) = 0;
    _dYawCmd = wz;
}

void State_Trotting::getUserCmd()
{
    // /* up_main版 */
    // ros::spinOnce();
    // _vCmdBody.block(0,0,2,1) = _velnav.block(0,0,2,1);
    // _vCmdBody(2) = 0;

    /* pid版 */
    // ros::spinOnce();
    // // std::cout<<"_vCmdBody.block(0,0,2,1):"<<_vCmdBody.block(0,0,2,1)<<std::endl;
    // _vCmdBody.block(0,0,2,1) = _Kddp * (_vCmdBody.block(0,0,2,1) - _velGazebo.block(0,0,2,1)) + _Kppp * (pos - _posGazebo.block(0,0,2,1));

    /* 走直线版 */
    // ros::spinOnce();

    // if (!flag_gan)
    // {
    //     // 参数定义
    //     double current_segment = floor((_posGazebo(0) - start_gan) / gan_interval); // 向下取整，得到当前段数

    //     // 核心运动逻辑
    //     if (current_segment < 0)
    //     { // 初始段
    //         _vCmdBody.block(0, 0, 2, 1) << 0, v_set;
    //         if (_posGazebo(1) > 0.5 * track_width)
    //         {
    //             _vCmdBody.block(0, 0, 2, 1) << v_set, 0;
    //         }
    //     }
    //     else
    //     {
    //         // 通用绕杆逻辑
    //         int is_even_pole = (int(current_segment) % 2 == 0);
    //         double y_boundary = is_even_pole ? 0.45 * track_width : -0.45 * track_width;

    //         // 相位判断
    //         double phase = fmod(_posGazebo(0) - start_gan, gan_interval) / gan_interval;

    //         if (phase < 0.5)
    //         { // 直行阶段
    //             _vCmdBody.block(0, 0, 2, 1) << v_set, 0;
    //         }
    //         else
    //         { // 绕杆阶段
    //             _vCmdBody.block(0, 0, 2, 1) << 0, (is_even_pole ? -v_set : v_set);

    //             // 终点段
    //             if ((current_segment == total_poles - 1) && (fabs(_posGazebo(1)) < 0.1))
    //             {
    //                 _vCmdBody.block(0, 0, 2, 1) << v_set, 0;
    //                 flag_gan = 1;
    //             }

    //             // 直行阶段
    //             if ((y_boundary > 0 && _posGazebo(1) < -y_boundary) || (y_boundary < 0 && _posGazebo(1) > -y_boundary))
    //             {
    //                 _vCmdBody.block(0, 0, 2, 1) << v_set, 0;
    //             }
    //         }
    //     }
    // }   

    /* 走圆弧版 */
    ros::spinOnce();
    // _vCmdBody.block(0,0,2,1) = _Kddp_gan * (_pathplan->pathpoints_gan.block(count,3,1,2).transpose() - _velGazebo.block(0,0,2,1)) + _Kppp_gan * (_pathplan->pathpoints_gan.block(count,0,1,2).transpose() - _posGazebo.block(0,0,2,1));
    _vCmdBody.block(0,0,2,1) = _Kddp_po * (_pathplan->pathpoints_po.block(count,3,1,2).transpose() - _velGazebo.block(0,0,2,1)) + _Kppp_po * (_pathplan->pathpoints_po.block(count,0,1,2).transpose() - _posGazebo.block(0,0,2,1));

    _vCmdBody(2) = 0;

    count++;

    /* Turning */
    // _dYawCmd = -invNormalize(_userValue.rx, _wyawLim(0), _wyawLim(1));
    // _dYawCmd = Yaw_kp * (atan2(_vCmdBody(1), _vCmdBody(0)) - atan2(_velGazebo(1), _velGazebo(0)));
    // _dYawCmd = 0.9 * _dYawCmdPast + (1 - 0.9) * _dYawCmd;
    // _dYawCmdPast = _dYawCmd;
}

void State_Trotting::calcCmd()
{
    /* Movement */
    _vCmdGlobal = _B2G_RotMat * _vCmdBody;

    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0) - 0.2, _velBody(0) + 0.2));
    _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_velBody(1) - 0.2, _velBody(1) + 0.2));

    _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt, Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05));
    _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * _ctrlComp->dt, Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));

    _vCmdGlobal(2) = 0;

    /* Turning */
    _yawCmd = _yawCmd + _dYawCmd * _ctrlComp->dt;

    _Rd = rotz(_yawCmd);
    _wCmdGlobal(2) = _dYawCmd;
}

void State_Trotting::calcTau()
{

    _posError = _pcd - _posBody;
    _velError = _vCmdGlobal - _velBody;

    _ddPcd = _Kpp * _posError + _Kdp * _velError;
    _dWbd = _kpw * rotMatToExp(_Rd * _G2B_RotMat) + _Kdw * (_wCmdGlobal - _lowState->getGyroGlobal());

    _ddPcd(0) = saturation(_ddPcd(0), Vec2(-3, 3));
    _ddPcd(1) = saturation(_ddPcd(1), Vec2(-3, 3));
    _ddPcd(2) = saturation(_ddPcd(2), Vec2(-5, 5));

    _dWbd(0) = saturation(_dWbd(0), Vec2(-40, 40));
    _dWbd(1) = saturation(_dWbd(1), Vec2(-40, 40));
    _dWbd(2) = saturation(_dWbd(2), Vec2(-10, 10));

    _forceFeetGlobal = -_balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);

    for (int i(0); i < 4; ++i)
    {
        if ((*_contact)(i) == 0)
        {
            _forceFeetGlobal.col(i) = _KpSwing * (_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing * (_velFeetGlobalGoal.col(i) - _velFeetGlobal.col(i));
        }
    }

    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;
    _q = vec34ToVec12(_lowState->getQ());
    _tau = _robModel->getTau(_q, _forceFeetBody);
}

void State_Trotting::calcQQd()
{

    Vec34 _posFeet2B;
    _posFeet2B = _robModel->getFeet2BPositions(*_lowState, FrameType::BODY);

    for (int i(0); i < 4; ++i)
    {
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody);
        // _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody - _B2G_RotMat * (skew(_lowState->getGyro()) * _posFeet2B.col(i)) );  //  c.f formula (6.12)
    }

    _qGoal = vec12ToVec34(_robModel->getQ(_posFeet2BGoal, FrameType::BODY));
    _qdGoal = vec12ToVec34(_robModel->getQd(_posFeet2B, _velFeet2BGoal, FrameType::BODY));
}


void GazeboListener_state::modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    // 输出模型的状态（位置和姿态）
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        if (msg->name[i] == "go1_gazebo")
        {
            // 将模型的位置赋值给CLFCBF::_posGazebo1
            State_Trotting::_posGazebo(0) = msg->pose[i].position.x;
            State_Trotting::_posGazebo(1) = msg->pose[i].position.y;
            State_Trotting::_posGazebo(2) = msg->pose[i].position.z;

            // std::cout << "Position: " << State_Trotting::_posGazebo(0) << std::endl;

            // 将模型的线速度赋值给CLFCBF::_velGazebo1
            State_Trotting::_velGazebo(0) = msg->twist[i].linear.x;
            State_Trotting::_velGazebo(1) = msg->twist[i].linear.y;
            State_Trotting::_velGazebo(2) = msg->twist[i].linear.z;
        }
    }
}