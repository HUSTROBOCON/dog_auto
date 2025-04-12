#include "Path/path.h"

using namespace std;
using namespace Eigen;

Path::Path(){


}

void Path::cal_pathpoints(){
    for(int path_index = 0; path_index < 3; path_index++)
    {
        int pathnum = 0;
        pathnum = cal_pathnum(path_index);
        std::cout<<"pathnum: "<<pathnum<<std::endl;
        
        switch (path_index)
        {
        case 0:
            pathpoints_gan.resize(pathnum*5, 12);
            pathpoints_gan = gan_path(pathnum);

            // for(int i = 0; i < pathnum*5; i+=50)
            // {
            //     std::cout<<"pathpoints_gan: "<<pathpoints_gan(i,0)<<", "<<pathpoints_gan(i,1)<<std::endl;
            // }            
            break;
        case 1:
            break;
        case 2:
            pathpoints_po.resize(pathnum, 12);
            pathpoints_po = po_path(pathnum);
            break;

        default:
            break;
        }

    }
}

int Path::cal_pathnum(int path_index)
{
    int pathnum = 0;
    double half_circle = PI * gan_R;

    switch(path_index)
    {
        case 0: // 竖杆
            pathnum = floor(half_circle / (v_gan * T));  // 仅一个半圆的点数
            break;
        case 1: // 直走
            // pathnum = floor(2 * gan_R / (v_gan * T));
            break;
        case 2: // 斜坡
            pathnum = floor((0.8*6+2.4)/ (v_po * T));  // 7段
            break;
        default:
            break;
    }

    return pathnum;
}

// 实现规划路径
Eigen::MatrixXd Path::gan_path(int pathnum)
{
    Eigen::MatrixXd pathPoints(pathnum*5, 12);
    double switch_phase = 0;  // 切换相位
    init_pos_gan.setZero();

    for (int i = 0; i < pathnum*5; i++)
    {
        // 路径规划
        pathPoints(i, 0) = init_pos_gan(0) + gan_R * cos(-PI + PI * i / pathnum + switch_phase) + gan_R;
        pathPoints(i, 1) = init_pos_gan(1) + gan_R * sin(-PI + PI * i / pathnum);
        pathPoints(i, 2) = 0.35;

        // 刷新初始点
        if((i%pathnum==0) && (i!=0))
        {
            init_pos_gan(0) += 2 * gan_R;
            init_pos_gan(1) = 0;
            switch_phase += PI;
        }
    }

    // 速度用差分求得
    for (int i = 0; i < pathnum*5 - 1; i++)
    {
        pathPoints(i,3) = (pathPoints(i+1,0) - pathPoints(i,0)) / T;
        pathPoints(i,4) = (pathPoints(i+1,1) - pathPoints(i,1)) / T;
    }

    pathPoints(pathnum*5-1,3) = 0;
    pathPoints(pathnum*5-1,4) = 0;

    return pathPoints;
}

// 实现规划路径
Eigen::MatrixXd Path::po_path(int pathnum)
{
    Eigen::MatrixXd pathPoints(pathnum, 12);
    double switch_phase = 0;  // 切换相位
    init_pos_po.setZero();
    init_pos_po(0) = 0;

    for (int i = 0; i < pathnum; i++)
    {
        // 路径规划
        if(i < pathnum/9){
            pathPoints(i, 0) = init_pos_po(0) + 0;
            pathPoints(i, 1) = init_pos_po(1) + 0.8 * i / (pathnum/9);  // 一小段长为0.8，共有1+1+1+3+1+1+1的7段
        }
        else if(i < pathnum/9*2){
            pathPoints(i, 0) = init_pos_po(0) + 0.8 * (i-pathnum/9) / (pathnum/9);
            pathPoints(i, 1) = init_pos_po(1) + 0.8;
        }
        else if(i < pathnum/9*3){
            pathPoints(i, 0) = init_pos_po(0) + 0.8;
            pathPoints(i, 1) = init_pos_po(1) + 0.8 - 0.8 * (i-pathnum/9*2) / (pathnum/9);
            pathPoints(i, 7) = 0.174533;  // 10度
        }
        else if(i < pathnum/9*6){
            pathPoints(i, 0) = init_pos_po(0) + 0.8 + 0.8 * (i-pathnum/9*3) / (pathnum/9);
            pathPoints(i, 1) = init_pos_po(1) + 0;
            pathPoints(i, 7) = 0.174533;  // 10度
        }
        else if(i < pathnum/9*7){
            pathPoints(i, 0) = init_pos_po(0) + 3.2;
            pathPoints(i, 1) = init_pos_po(1) + 0.8 * (i-pathnum/9*6) / (pathnum/9);
            pathPoints(i, 7) = 0.174533;  // 10度
        }
        else if(i < pathnum/9*8){
            pathPoints(i, 0) = init_pos_po(0) + 3.2 + 0.8 * (i-pathnum/9*7) / (pathnum/9);
            pathPoints(i, 1) = init_pos_po(1) + 0.8;
        }
        else{
            pathPoints(i, 0) = init_pos_po(0) + 4;
            pathPoints(i, 1) = init_pos_po(1) + 0.8 - 0.8 * (i-pathnum/9*8) / (pathnum/9);
        }

    }

    // 速度用差分求得
    for (int i = 0; i < pathnum - 1; i++)
    {
        pathPoints(i,3) = (pathPoints(i+1,0) - pathPoints(i,0)) / T;
        pathPoints(i,4) = (pathPoints(i+1,1) - pathPoints(i,1)) / T;
        pathPoints(i,9) = (pathPoints(i+1,7) - pathPoints(i,7)) / T;  // 角速度
    }

    pathPoints(pathnum-1,3) = 0;
    pathPoints(pathnum-1,4) = 0;

    return pathPoints;
}
