#include "CLFCBF/pos.h"
#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <fstream>

#include <unistd.h>

// int flag = 0;
double pos[3][3] = {0};
int robot1_pos_flag = 0;
int robot2_pos_flag = 0;
int robot3_pos_flag = 0;

int count = 0;
 
Vec3 CLFCBF::_posGazebo1(1.2, 1.6, 0.0); // 初始化静态成员
Vec3 CLFCBF::_velGazebo1(0.0, 0.0, 0.0);
Vec3 CLFCBF::_posGazebo2(0.4, 0.5, 0.0);
Vec3 CLFCBF::_velGazebo2(0.0, 0.0, 0.0);
Vec3 CLFCBF::_posGazebo3(3.0, 1.2, 0.0);
Vec3 CLFCBF::_velGazebo3(0.0, 0.0, 0.0);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_node"); // 初始化ROS节点
  ros::NodeHandle nh_;

  GazeboPublisher gazebopublisher(nh_);
  GazeboListener listener(nh_);

  Vec2 pos_vec_1, pos_vec_2, pos_vec_3;
  CLFCBF clfcbf;

  double Timer = 0;

  // 设置一个循环，定期发布路径数据
  ros::Rate rate(10); // 10 Hz
  while (ros::ok())
  {
    // 处理回调函数
    ros::spinOnce();
    //  std::cout<<"shit2"<<std::endl;

count++;

    // 发布路径
    gazebopublisher.publishOdomCallback(clfcbf);

    // 按照指定频率循环
    rate.sleep();
  }

  return 0;
}

void GazeboPublisher::publishOdomCallback(CLFCBF &clfcbf)
{
  geometry_msgs::PoseStamped nav_traj_vis_msg1, nav_traj_vis_msg2, nav_traj_vis_msg3;
  nav_msgs::Path _nav_seq_msg1, _nav_seq_msg2, _nav_seq_msg3; // 控制轨迹序列msgs NMPC的TargetTrajectory
  Vec2 u1, u2, u3;

  // 设置 path 消息的 header 信息
  nav_traj_vis_msg1.header.stamp = ros::Time::now();
  nav_traj_vis_msg1.header.frame_id = "map"; // 假设路径是在 "map" 坐标系下
  nav_traj_vis_msg2.header.stamp = ros::Time::now();
  nav_traj_vis_msg2.header.frame_id = "map"; // 假设路径是在 "map" 坐标系下
  nav_traj_vis_msg3.header.stamp = ros::Time::now();
  nav_traj_vis_msg3.header.frame_id = "map"; // 假设路径是在 "map" 坐标系下

  // 每次清除_nav_seq_msg中的所有点
  if (!_nav_seq_msg1.poses.empty())
  {
    _nav_seq_msg1.poses.clear();
  }

  if (!_nav_seq_msg2.poses.empty())
  {
    _nav_seq_msg2.poses.clear();
  }

  if (!_nav_seq_msg3.poses.empty())
  {
    _nav_seq_msg3.poses.clear();
  }

  // 如果是第一次，则连跑n次
  for (int i = 0; i < _mpc_step_interval; i++)
  {
    if (i > 0)
    {
      CLFCBF::_posGazebo1(0) = clfcbf.pos_1(0) * 0.01;
      CLFCBF::_posGazebo1(1) = clfcbf.pos_1(1) * 0.01;
      CLFCBF::_velGazebo1(0) = clfcbf.v_1(0) * 0.01;
      CLFCBF::_velGazebo1(1) = clfcbf.v_1(1) * 0.01;

      CLFCBF::_posGazebo2(0) = clfcbf.pos_2(0) * 0.01;
      CLFCBF::_posGazebo2(1) = clfcbf.pos_2(1) * 0.01;
      CLFCBF::_velGazebo2(0) = clfcbf.v_2(0) * 0.01;
      CLFCBF::_velGazebo2(1) = clfcbf.v_2(1) * 0.01;

      CLFCBF::_posGazebo3(0) = clfcbf.pos_3(0) * 0.01;
      CLFCBF::_posGazebo3(1) = clfcbf.pos_3(1) * 0.01;
      CLFCBF::_velGazebo3(0) = clfcbf.v_3(0) * 0.01;
      CLFCBF::_velGazebo3(1) = clfcbf.v_3(1) * 0.01;
    }

    // std::cout<<"_posGazebo_y:"<< CLFCBF::_posGazebo(1)<<std::endl;

    Vec2 pg1 = CLFCBF::_posGazebo1.block(0, 0, 2, 1) * 100;
    Vec2 vg1 = CLFCBF::_velGazebo1.block(0, 0, 2, 1) * 100;
    Vec2 pg2 = CLFCBF::_posGazebo2.block(0, 0, 2, 1) * 100;
    Vec2 vg2 = CLFCBF::_velGazebo2.block(0, 0, 2, 1) * 100;
    Vec2 pg3 = CLFCBF::_posGazebo3.block(0, 0, 2, 1) * 100;
    Vec2 vg3 = CLFCBF::_velGazebo3.block(0, 0, 2, 1) * 100;

    // 调用上层
    u1 = clfcbf.calF_leader(i, pg1, vg1, pg2, vg2, pg3, vg3, clfcbf.pos_1, clfcbf.v_1, clfcbf._u_1, 1, clfcbf.A_cbf_tr_1, clfcbf.b_cbf_tr_1);
    
    clfcbf.pr += clfcbf.vr * clfcbf.T;
    clfcbf.v_1 += u1 * clfcbf.T;
    clfcbf.pos_1 += clfcbf.v_1 * clfcbf.T;

    nav_traj_vis_msg1.pose.position.x = clfcbf.pos_1(0)  * 0.01;
    nav_traj_vis_msg1.pose.position.y = clfcbf.pos_1(1) * 0.01;
    nav_traj_vis_msg1.pose.position.z = 0;
    nav_traj_vis_msg1.pose.orientation.x = clfcbf.v_1(0) * 0.01;
    nav_traj_vis_msg1.pose.orientation.y = clfcbf.v_1(1) * 0.01;
    nav_traj_vis_msg1.pose.orientation.z = 0;

    // nav_traj_vis_msg1.pose.position.x = 1.2 + count*0.1;
    // nav_traj_vis_msg1.pose.position.y = 1.6;
    // nav_traj_vis_msg1.pose.position.z = 0;
    // nav_traj_vis_msg1.pose.orientation.x = 0.5;
    // nav_traj_vis_msg1.pose.orientation.y = 0;
    // nav_traj_vis_msg1.pose.orientation.z = 0;

    // _nav_seq_msg1.poses.push_back(nav_traj_vis_msg1);

    // if(i==0){
      // std::cout << "ux: " << u1(0) << std::endl;
      // std::cout << "uy: " << u1(1) << std::endl;
    // }

    // if(i==0){
      std::cout << "pr_y: " << clfcbf.pr(1) << std::endl;
      std::cout << "v_y: " << clfcbf.v_1(1) << std::endl;
      std::cout << "pos_y: " << clfcbf.pos_1(1) << std::endl;
    // }


    
    // u2 = clfcbf.calF_follower(i, pg1, vg1, pg2, vg2, pg3, vg3, clfcbf.pos_2, clfcbf.v_2, clfcbf._u_2, 2, clfcbf.A_cbf_tr_2, clfcbf.b_cbf_tr_2);

    // clfcbf.v_2 += u2 * clfcbf.T;
    // // clfcbf.v_2(0) += clfcbf.deltav2;
    // clfcbf.pos_2 += clfcbf.v_2 * clfcbf.T;

    // nav_traj_vis_msg2.pose.position.x = clfcbf.pos_2(0) * 0.01;
    // nav_traj_vis_msg2.pose.position.y = clfcbf.pos_2(1) * 0.01;
    // nav_traj_vis_msg2.pose.position.z = 0;
    // nav_traj_vis_msg2.pose.orientation.x = clfcbf.v_2(0) * 0.01;
    // nav_traj_vis_msg2.pose.orientation.y = clfcbf.v_2(1) * 0.01;
    // nav_traj_vis_msg2.pose.orientation.z = 0;

    // _nav_seq_msg2.poses.push_back(nav_traj_vis_msg2);



    // u3 = clfcbf.calF_follower(i, pg1, vg1, pg2, vg2, pg3, vg3, clfcbf.pos_3, clfcbf.v_3, clfcbf._u_3, 3, clfcbf.A_cbf_tr_3, clfcbf.b_cbf_tr_3);

    // clfcbf.v_3 += u3 * clfcbf.T;
    // // clfcbf.v_3(0) += clfcbf.deltav3;
    // clfcbf.pos_3 += clfcbf.v_3 * clfcbf.T;

    // nav_traj_vis_msg3.pose.position.x = clfcbf.pos_3(0) * 0.01;
    // nav_traj_vis_msg3.pose.position.y = clfcbf.pos_3(1) * 0.01;
    // nav_traj_vis_msg3.pose.position.z = 0;
    // nav_traj_vis_msg3.pose.orientation.x = clfcbf.v_3(0) * 0.01;
    // nav_traj_vis_msg3.pose.orientation.y = clfcbf.v_3(1) * 0.01;
    // nav_traj_vis_msg3.pose.orientation.z = 0;

    // _nav_seq_msg3.poses.push_back(nav_traj_vis_msg3);



    
    std::cout<<std::endl;
    
    // 若到mpc预测最后一步，则需要倒退pr
    if (i == _mpc_step_interval - 1)
    {
      clfcbf.pr = clfcbf.pr - (_mpc_step_interval - 1) * (clfcbf.vr * clfcbf.T);
    }

  }

  // 规划轨迹
  // std::cout << "x:" << nav_traj_vis_msg1.pose.position.x << std::endl;
  // std::cout << "y:" << nav_traj_vis_msg1.pose.position.y << std::endl;

  // 发布消息
  pub.publish(nav_traj_vis_msg1);
  // pub_2.publish(_nav_seq_msg2);
  // pub_3.publish(_nav_seq_msg3);
}

void GazeboListener::modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
   // 输出模型的状态（位置和姿态）
   for (size_t i = 0; i < msg->name.size(); ++i)
   {
       if(msg->name[i] == "go1_gazebo")
       {
           // 将模型的位置赋值给CLFCBF::_posGazebo1
           CLFCBF::_posGazebo1(0) = msg->pose[i].position.x;
           CLFCBF::_posGazebo1(1) = msg->pose[i].position.y;    
           CLFCBF::_posGazebo1(2) = msg->pose[i].position.z;

           // 将模型的线速度赋值给CLFCBF::_velGazebo1
           CLFCBF::_velGazebo1(0) = msg->twist[i].linear.x;
           CLFCBF::_velGazebo1(1) = msg->twist[i].linear.y;
           CLFCBF::_velGazebo1(2) = msg->twist[i].linear.z;

           ROS_INFO("Model Name: %s", msg->name[i].c_str());
           ROS_INFO("Position: [%.2f, %.2f, %.2f]", 
                    msg->pose[i].position.x, 
                    msg->pose[i].position.y, 
                    msg->pose[i].position.z);           
       }

   }   
}