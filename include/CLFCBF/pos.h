/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef POS_H
#define POS_H

#include "thirdParty/quadProgpp/QuadProg++.hh"
#include "common/mathTools.h"

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

#define _mpc_step_interval 1

class CLFCBF
{
public:
  CLFCBF();
  void clf_fcn_leader();
  void calConstraints_leader(Vec2 &A_cbf_tr_n, Eigen::Matrix<double, 1, 1> &b_cbf_tr_n);
  void calConstraints_follower(Vec2 &A_cbf_tr_n, Eigen::Matrix<double, 1, 1> &b_cbf_tr_n);
  void solveQP_leader(Vec5 &_u_n);
  void solveQP_follower(Vec4 &_u_n);
  Vec2 calF_leader(int i, Vec2 pos_real1, Vec2 vel_real1, Vec2 pos_real2, Vec2 vel_real2, Vec2 pos_real3, Vec2 vel_real3, Vec2 &pos_n, Vec2 &v_n, Vec5 &_u_n, int robot_name, Vec2 &A_cbf_tr_n, Eigen::Matrix<double, 1, 1> &b_cbf_tr_n);
  Vec2 calF_follower(int i, Vec2 pos_real1, Vec2 vel_real1, Vec2 pos_real2, Vec2 vel_real2, Vec2 pos_real3, Vec2 vel_real3, Vec2 &pos_n, Vec2 &v_n, Vec4 &_u_n, int robot_name, Vec2 &A_cbf_tr_n, Eigen::Matrix<double, 1, 1> &b_cbf_tr_n);
  // void cbf_fcn_out(Vec2 pos_n, Vec2 v_n, Eigen::Matrix<double, 4,2> &A_cbf_tr_n, Eigen::Matrix<double, 4,1> &b_cbf_tr_n);
  void BS1(Vec2 v_n);
  void clf_fs(int i, int robot_name);
  void readDataFromFile(const std::string &filename, Eigen::MatrixXd &mat);
  void OA(int i, int robot_name, Vec2 pos_n, Vec2 v_n, Vec2 &A_cbf_tr_n, Eigen::Matrix<double, 1, 1> &b_cbf_tr_n);
  void SA(int i, int robot_name);

  static Vec2 pos_1, pos_2, pos_3, pos_4;
  static Vec2 v_1, v_2, v_3, v_4;

  Vec2 d12, d21, d13, d31, d23, d32;
  Vec5 _u_1;
  Vec4 _u_2, _u_3;
  double W1, W2, W3;
  Vec2 e12, e21, e23, e32, e13, e31;
  Vec2 v12, v21, v23, v32, v13, v31;
  Vec2 p12, p21, p13, p31, p23, p32;

  // 拼接后大矩阵
  Vec2 A_cbf_tr_1;
  Vec2 A_cbf_tr_2;
  Vec2 A_cbf_tr_3;
  Vec2 A_cbf_tr_4;

  Eigen::Matrix<double, 1, 1> b_cbf_tr_1;
  Eigen::Matrix<double, 1, 1> b_cbf_tr_2;
  Eigen::Matrix<double, 1, 1> b_cbf_tr_3;
  Eigen::Matrix<double, 1, 1> b_cbf_tr_4;

  Eigen::MatrixXd sdf;
  Eigen::MatrixXd GDX;
  Eigen::MatrixXd GDY;

  // const int _mpc_step_interval = 5;

  std::vector<std::vector<double>> positions_1;
  std::vector<double> pos1;
  std::vector<std::vector<double>> positions_2;
  std::vector<double> pos2;
  std::vector<std::vector<double>> positions_3;
  std::vector<double> pos3;

  static Vec3 _posGazebo1, _posGazebo2, _posGazebo3;
  static Vec3 _velGazebo1, _velGazebo2, _velGazebo3;

  Vec2 ar;
  Vec2 pr;
  Vec2 vr;
  Vec2 v_m; // 最大速度

  double T = 0.002;
  double deltav1, deltav2, deltav3;
  int count1 = 0;
  int count2 = 0;
  int count3 = 0;
  int size = 10;
  int lf, rf;

private:
  // GazeboPublisher *gazeboPublisher;

  double k_l = 1000;
  // double k_b1 = 10;
  // double k_b2 = 1000;
  double k_oa = 10;
  double k_v = 40;
  double k_f = 50;
  double k_sa = 10;
  double u_b = 300;
  double q_l = 10;
  double q_i = 0.5;
  double deltap = 30;
  double delta12 = 50;
  double delta21 = 50;
  double delta13 = 50;
  double delta31 = 50;
  double delta23 = 50;
  double delta32 = 50;

  Vec2 A_clf_tr;
  Eigen::Matrix<double, 1, 1> b_clf_tr;

  Vec2 A_clf_fs;
  Eigen::Matrix<double, 1, 1> b_clf_fs;

  Vec2 A_cbf_sa1;
  Eigen::Matrix<double, 1, 1> b_cbf_sa1;
  Vec2 A_cbf_sa2;
  Eigen::Matrix<double, 1, 1> b_cbf_sa2;

  Eigen::Matrix<Vec2, 4, 4> Dij;

  Vec2 A_v;
  Eigen::Matrix<double, 1, 1> b_v;

  // 内部小矩阵
  Vec2 A_cbf_tr1;
  Eigen::Matrix<double, 1, 1> b_cbf_tr1;
  Vec2 A_cbf_tr2;
  Eigen::Matrix<double, 1, 1> b_cbf_tr2;
  Vec2 A_cbf_tr3;
  Eigen::Matrix<double, 1, 1> b_cbf_tr3;
  Vec2 A_cbf_tr4;
  Eigen::Matrix<double, 1, 1> b_cbf_tr4;

  Eigen::MatrixXd _CE, _CI;
  Eigen::VectorXd _ce0, _ci0;
  // leader
  Eigen::Matrix<double, 5, 5> _G_leader;
  Vec5 _G_vec_leader;
  Vec5 _g0T_leader;
  // follower
  Eigen::Matrix<double, 4, 4> _G_follower;
  Vec4 _G_vec_follower;
  Vec4 _g0T_follower;
  Vec2 e_1, e_2;

  // CE、ce0为等式约束，Ci、ci0为不等式约束
  quadprogpp::Matrix<double> G, CE, CI;
  quadprogpp::Vector<double> g0, ce0, ci0, x;

  Vec2 center1; // 障碍物中心（此处为圆）
  Vec2 center2;
  Vec2 center3; // 障碍物中心（此处为圆）
  Vec2 center4; // 障碍物中心（此处为圆）

  std::ofstream file1, file2, file3, file4, file5, file6;
  std::ofstream file7, file8, file9, file10, file11, file12;
  std::ofstream file13, file14, file15;
};

class GazeboPublisher
{
public:
  GazeboPublisher(ros::NodeHandle &nh_)
  {
    // 发布消息到指定话题
    pub = nh_.advertise<geometry_msgs::PoseStamped>("shit", 10);

  }
  void publishOdomCallback(CLFCBF &clfcbf);

private:
  ros::Publisher pub;
};

// class GazeboListener
// {
// public:
//   GazeboListener(ros::NodeHandle &nh_)
//   {
//     // 订阅Gazebo的ModelStates话题
//     sub_1_p = nh_.subscribe("/vrpn_client_node/dog1/pose", 1, &GazeboListener::modelStateCallback1_p, this);
//     sub_1_v = nh_.subscribe("/vrpn_client_node/dog1/twist", 1, &GazeboListener::modelStateCallback1_v, this);
//     sub_2_p = nh_.subscribe("/vrpn_client_node/dog2/pose", 1, &GazeboListener::modelStateCallback2_p, this);
//     sub_2_v = nh_.subscribe("/vrpn_client_node/dog2/twist", 1, &GazeboListener::modelStateCallback2_v, this);
//     sub_3_p = nh_.subscribe("/vrpn_client_node/dog3/pose", 1, &GazeboListener::modelStateCallback3_p, this);
//     sub_3_v = nh_.subscribe("/vrpn_client_node/dog3/twist", 1, &GazeboListener::modelStateCallback3_v, this);
//   }
//   void modelStateCallback1_p(const geometry_msgs::PoseStamped::ConstPtr &pose_msg_);
//   void modelStateCallback1_v(const geometry_msgs::TwistStamped::ConstPtr &twist_msg_);
//   void modelStateCallback2_p(const geometry_msgs::PoseStamped::ConstPtr &pose_msg_);
//   void modelStateCallback2_v(const geometry_msgs::TwistStamped::ConstPtr &twist_msg_);
//   void modelStateCallback3_p(const geometry_msgs::PoseStamped::ConstPtr &pose_msg_);
//   void modelStateCallback3_v(const geometry_msgs::TwistStamped::ConstPtr &twist_msg_);

// private:
//   ros::Subscriber sub_1_p;
//   ros::Subscriber sub_1_v;
//   ros::Subscriber sub_2_p;
//   ros::Subscriber sub_2_v;
//   ros::Subscriber sub_3_p;
//   ros::Subscriber sub_3_v;
// };

class GazeboListener
{
public:
  GazeboListener(ros::NodeHandle &nh_)
  {
    // 订阅Gazebo的ModelStates话题
    sub_1_p = nh_.subscribe("/gazebo/model_states", 1, &GazeboListener::modelStateCallback, this);
  }
  void modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);


private:
  ros::Subscriber sub_1_p;

};

#endif // POS_H
