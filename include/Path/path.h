#ifndef PATH_H
#define PATH_H

#include "thirdParty/quadProgpp/QuadProg++.hh"
#include "common/mathTools.h"

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <cmath>

#define PI 3.14159265358979323846

class Path
{
public:
    Path();
    Eigen::MatrixXd gan_path(int pathnum);
    Eigen::MatrixXd po_path(int pathnum);

    int cal_pathnum(int path_index);
    void cal_pathpoints(); //计算路径点

/*
说明：
pathpoints设为（路径点数，12）形状的矩阵
其中12代表：x,y,z,vx,vy,vz,θx,θy,θz,wx,wy,wz
*/
    Eigen::MatrixXd pathpoints_gan;
    Eigen::MatrixXd pathpoints_pufu;
    Eigen::MatrixXd pathpoints_sand;
    Eigen::MatrixXd pathpoints_wall;
    Eigen::MatrixXd pathpoints_bridge;
    Eigen::MatrixXd pathpoints_po;

    Eigen::Vector3d init_pos_gan;
    Eigen::Vector3d init_pos_pufu;
    Eigen::Vector3d init_pos_sand;
    Eigen::Vector3d init_pos_wall;
    Eigen::Vector3d init_pos_bridge;
    Eigen::Vector3d init_pos_po;

    int count=0;

    int path_index;
    double T = 0.002;
    double v_gan = 0.2;
    double v_po = 0.2;


private:
    // 竖杆的参数
    double gan_R = 0.59;  //圆半径


};


#endif