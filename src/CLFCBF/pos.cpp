#include <iostream>
#include "CLFCBF/pos.h"

using namespace std;
using namespace Eigen;

Vec2 CLFCBF::pos_1(120, 160);
Vec2 CLFCBF::v_1(0, 0);
Vec2 CLFCBF::pos_2(40, 50);
Vec2 CLFCBF::v_2(0, 0);
Vec2 CLFCBF::pos_3(300, 120);
Vec2 CLFCBF::v_3(0, 0);

CLFCBF::CLFCBF()
{
  // ref
  ar << 0, 0;
  pr << 170, 170;
  vr << 0, 50;

  // leader
  pos_1 << 120, 160;
  v_1 << 0, 0;
  // follower-1
  pos_2 << 40, 50;
  v_2 << 0, 0;
  // follower-2
  pos_3 << 300, 120;
  v_3 << 0, 0;

  // leader
  _g0T_leader << 0, 0, 0, 0, 0;
  _G_vec_leader << 1, 1, 1, q_l, q_i;
  _G_leader = _G_vec_leader.asDiagonal();
  // follower
  _g0T_follower << 0, 0, 0, 0;
  _G_vec_follower << 1, 1, 1, q_i;
  _G_follower = _G_vec_follower.asDiagonal();
  v_m << 70, 70;
  // configuration
  // 三狗
  d12 << 50, 80;
  d21 = -d12;
  d13 << -50, 80;
  d31 = -d13;
  d23 << -100, 0;
  d32 = -d23;
  // init
  e12 = pos_1 - pos_2 - d12;
  v12 = v_1 - v_2;
  e13 = pos_1 - pos_3 - d13;
  v13 = v_1 - v_3;
  e23 = pos_2 - pos_3 - d23;
  v23 = v_2 - v_3;

  e21 = -e12;
  v21 = -v12;
  e31 = -e13;
  v31 = -v13;
  e32 = -e23;
  v32 = -v23;

  W1 = e12.dot(e12) + v12.dot(v12) + e12.dot(v12) + e13.dot(e13) + v13.dot(v13) + e13.dot(v13);
  W2 = e21.dot(e21) + v21.dot(v21) + e21.dot(v21) + e23.dot(e23) + v23.dot(v23) + e23.dot(v23);
  W3 = e31.dot(e31) + v31.dot(v31) + e31.dot(v31) + e32.dot(e32) + v32.dot(v32) + e32.dot(v32);

  readDataFromFile("/home/yth/project/unitree_guide/src/unitree_guide/src/CLFCBF/sdf.txt", sdf);
  readDataFromFile("/home/yth/project/unitree_guide/src/unitree_guide/src/CLFCBF/GDX.txt", GDX);
  readDataFromFile("/home/yth/project/unitree_guide/src/unitree_guide/src/CLFCBF/GDY.txt", GDY);
}

// 最终计算规划得到的速度
Vec2 CLFCBF::calF_leader(int i, Vec2 pos_real1, Vec2 vel_real1, Vec2 pos_real2, Vec2 vel_real2, Vec2 pos_real3, Vec2 vel_real3, Vec2 &pos_n, Vec2 &v_n, Vec5 &_u_n, int robot_name, Vec2 &A_cbf_tr_n, Eigen::Matrix<double, 1, 1> &b_cbf_tr_n)
{
  // pos_1 = pos_real1;
  // v_1 = vel_real1;
  // pos_2 = pos_real2;
  // v_2 = vel_real2;
  // pos_3 = pos_real3;
  // v_3 = vel_real3;

  // std::cout<<"pr_y:"<<pr(1)<<std::endl;

  if (i == 0)
  {
    file7.open("/home/yth/project/unitree_guide/src/unitree_guide/src/CLFCBF/vx1.txt", ofstream::out | std::ofstream::app);
    file7 << v_1(0) << std::endl; // 写入数据
    file7.close();

    file8.open("/home/yth/project/unitree_guide/src/unitree_guide/src/CLFCBF/vy1.txt", ofstream::out | std::ofstream::app);
    file8 << v_1(1) << std::endl; // 写入数据
    file8.close();
  }

  if (pr(1) > 570)
  {
    vr << 0, 0;
  }

  // Vm
  BS1(v_n);

  // TR
  clf_fcn_leader();
  // FS
  // clf_fs(i, robot_name);
  // OA
  OA(i, robot_name, pos_n, v_n, A_cbf_tr_n, b_cbf_tr_n);
  // SA(i, robot_name);

  calConstraints_leader(A_cbf_tr_n, b_cbf_tr_n);

  solveQP_leader(_u_n);

  return _u_n.block(0,0,2,1);
}

Vec2 CLFCBF::calF_follower(int i, Vec2 pos_real1, Vec2 vel_real1, Vec2 pos_real2, Vec2 vel_real2, Vec2 pos_real3, Vec2 vel_real3, Vec2 &pos_n, Vec2 &v_n, Vec4 &_u_n, int robot_name, Vec2 &A_cbf_tr_n, Eigen::Matrix<double, 1, 1> &b_cbf_tr_n)
{
  pos_1 = pos_real1;
  v_1 = vel_real1;
  pos_2 = pos_real2;
  v_2 = vel_real2;
  pos_3 = pos_real3;
  v_3 = vel_real3;

  if (i == 0)
  {

    if (robot_name == 2)
    {
      file9.open("/home/yth/project/unitree_guide/src/unitree_guide/src/CLFCBF/vx2.txt", ofstream::out | std::ofstream::app);
      file9 << v_2(0) << std::endl; // 写入数据
      file9.close();

      file10.open("/home/yth/project/unitree_guide/src/unitree_guide/src/CLFCBF/vy2.txt", ofstream::out | std::ofstream::app);
      file10 << v_2(1) << std::endl; // 写入数据
      file10.close();
    }

    if (robot_name == 3)
    {
      file11.open("/home/yth/project/unitree_guide/src/unitree_guide/src/CLFCBF/vx3.txt", ofstream::out | std::ofstream::app);
      file11 << v_3(0) << std::endl; // 写入数据
      file11.close();

      file12.open("/home/yth/project/unitree_guide/src/unitree_guide/src/CLFCBF/vy3.txt", ofstream::out | std::ofstream::app);
      file12 << v_3(1) << std::endl; // 写入数据
      file12.close();
    }
  }

  // 从外部读取采样时间
  // T  = _T;
  BS1(v_n);

  clf_fs(i, robot_name);

  OA(i, robot_name, pos_n, v_n, A_cbf_tr_n, b_cbf_tr_n);
  SA(i, robot_name);
  calConstraints_follower(A_cbf_tr_n, b_cbf_tr_n);

  solveQP_follower(_u_n);

  return _u_n.block(0, 0, 2, 1);
}

// 计算 A_clf_tr 和 b_clf_tr（跟踪轨迹）
void CLFCBF::clf_fcn_leader()
{

  // 计算 e_1 和 e_2
  e_1 = pos_1 - pr;
  e_2 = v_1 - vr;
  // 计算 w（e_1 与自身的点积 + e_2 与自身的点积 + e_1 和 e_2 的点积）
  double w = e_1.dot(e_1) + e_2.dot(e_2) + e_1.dot(e_2);

  // 计算 A_clf_tr（e_1 + 2 * e_2）
  A_clf_tr = (2 * e_2 + e_1);

  // 计算 b_clf_tr
  double first_term = -k_l * w;
  double second_term = A_clf_tr.dot(ar);
  double third_term = e_2.dot(e_2);
  double fourth_term = 2 * e_1.dot(e_2);

  b_clf_tr.block(0, 0, 1, 1) = (first_term + second_term - third_term - fourth_term) * Eigen::MatrixXd::Identity(1, 1);
}

void CLFCBF::clf_fs(int i, int robot_name)
{
  // double W;
  // Vec2 pij_dij_1;
  // Vec2 pij_dij_2;

  switch (robot_name)
  {
  case 1:
    e12 = (pos_1 - pos_2) - d12;
    v12 = v_1 - v_2;
    e13 = (pos_1 - pos_3) - d13;
    v13 = v_1 - v_3;

    W1 = e12.dot(e12) + v12.dot(v12) + e12.dot(v12) + e13.dot(e13) + v13.dot(v13) + e13.dot(v13);

    if(i==0){
      file4.open("/home/yth/project/unitree_guide/src/unitree_guide/src/CLFCBF/W1.txt", ofstream::out | std::ofstream::app);
      // 写入txt文件
      file4 << W1 << std::endl; // 写入数据
      file4.close();
    }

    A_clf_fs = 2 * v12 + e12 + 2 * v13 + e13;
    b_clf_fs = (-k_f * W1 - v_1.dot(v12) - 2 * e12.dot(v_1) - v_1.dot(v13) - 2 * e13.dot(v_1)) * Eigen::MatrixXd::Identity(1, 1);

    break;

  case 2:
    e21 = (pos_2 - pos_1) - d21;
    v21 = v_2 - v_1;
    e23 = (pos_2 - pos_3) - d23;
    v23 = v_2 - v_3;

    W2 = e21.dot(e21) + v21.dot(v21) + e21.dot(v21) + e23.dot(e23) + v23.dot(v23) + e23.dot(v23);
    
    if(i==0){
      file5.open("/home/yth/project/unitree_guide/src/unitree_guide/src/CLFCBF/W2.txt", ofstream::out | std::ofstream::app);
      // 写入txt文件
      file5 << W2 << std::endl; // 写入数据
      file5.close();
    }

    A_clf_fs = 2 * v21 + e21 + 2 * v23 + e23;
    b_clf_fs = (-k_f * W2 - v_2.dot(v21) - 2 * e21.dot(v_2) - v_2.dot(v23) - 2 * e23.dot(v_2)) * Eigen::MatrixXd::Identity(1, 1);

    break;

  case 3:
    e32 = (pos_3 - pos_2) - d32;
    v32 = v_3 - v_2;
    e31 = pos_3 - pos_1 - d31;
    v31 = v_3 - v_1;

    W3 = e31.dot(e31) + v31.dot(v31) + e31.dot(v31) + e32.dot(e32) + v32.dot(v32) + e32.dot(v32);

    if(i==0){
      file6.open("/home/yth/project/unitree_guide/src/unitree_guide/src/CLFCBF/W3.txt", ofstream::out | std::ofstream::app);
      // 写入txt文件
      file6 << W3 << std::endl; // 写入数据
      file6.close();
    }

    A_clf_fs = 2 * v31 + e31 + 2 * v32 + e32;
    b_clf_fs = (-k_f * W3 - v_3.dot(v31) - 2 * e31.dot(v_3) - v_3.dot(v32) - 2 * e32.dot(v_3)) * Eigen::MatrixXd::Identity(1, 1);
    break;

  default:
    break;
  }
}

// 约束
void CLFCBF::calConstraints_leader(Vec2 &A_cbf_tr_n, Eigen::Matrix<double, 1, 1> &b_cbf_tr_n)
{
  _CI.resize(7, 5);
  _ci0.resize(7);
  _CE.resize(1, 5);
  _ce0.resize(1);
  _CI.setZero();
  _ci0.setZero();
  _CE.setZero();
  _ce0.setZero();
  // TR
  _CI.block(0, 0, 1, 2) = -A_clf_tr.transpose();
  _CI.block(0, 3, 1, 1) = 1 * Eigen::MatrixXd::Identity(1, 1);
  _ci0.block(0, 0, 1, 1) = b_clf_tr;
  // FS
  // _CI.block(1, 0, 1, 2) = -A_clf_tr.transpose();
  // _CI.block(1, 4, 1, 1) = 1 * Eigen::MatrixXd::Identity(1, 1);
  // _ci0.block(1, 0, 1, 1) = b_clf_tr;
  // OA
  _CI.block(1, 0, 1, 2) = -A_cbf_tr_n.transpose();
  _ci0.block(1, 0, 1, 1) = b_cbf_tr_n;
  // Vm
  _CI.block(2, 0, 1, 2) = -A_v.transpose();
  _ci0.block(2, 0, 1, 1) = b_v;
  // Um
  _CI.block(3, 0, 2, 2) = Eigen::MatrixXd::Identity(2, 2);
  _CI.block(5, 0, 2, 2) = -Eigen::MatrixXd::Identity(2, 2);
  Vec4 one_four;
  one_four << 1, 1, 1, 1;
  _ci0.block(3, 0, 4, 1) = one_four * u_b;
  // 等式约束
  _CE << 0, 0, 1, 0, 0;
  _ce0 << 0;
}

// follower约束
void CLFCBF::calConstraints_follower(Vec2 &A_cbf_tr_n, Eigen::Matrix<double, 1, 1> &b_cbf_tr_n)
{
  _CI.resize(7, 4);
  _ci0.resize(7);
  _CE.resize(1, 4);
  _ce0.resize(1);

  _CI.setZero();
  _ci0.setZero();
  _CE.setZero();
  _ce0.setZero();
  // FS
  _CI.block(0, 0, 1, 2) = -A_clf_fs.transpose();
  _CI.block(0, 3, 1, 1) = 1 * Eigen::MatrixXd::Identity(1, 1);
  _ci0.block(0, 0, 1, 1) = b_clf_fs;
  // OA
  _CI.block(1, 0, 1, 2) = -A_cbf_tr_n.transpose();
  _ci0.block(1, 0, 1, 1) = b_cbf_tr_n;
  // Vm
  _CI.block(2, 0, 1, 2) = -A_v.transpose();
  _ci0.block(2, 0, 1, 1) = b_v;
  // Um
  _CI.block(3, 0, 2, 2) = Eigen::MatrixXd::Identity(2, 2);
  _CI.block(5, 0, 2, 2) = -Eigen::MatrixXd::Identity(2, 2);
  Vec4 one_four;
  one_four << 1, 1, 1, 1;
  _ci0.block(3, 0, 4, 1) = one_four * u_b;
  // 等式约束
  _CE << 0, 0, 1, 0;
  _ce0 << 0;
}

void CLFCBF::solveQP_leader(Vec5 &_u_n)
{
  int n = _u_n.size();
  int m = _ce0.size();
  int p = _ci0.size();

  G.resize(n, n);
  CE.resize(n, m);
  CI.resize(n, p);
  g0.resize(n);
  ce0.resize(m);
  ci0.resize(p);
  x.resize(n);

  for (int i = 0; i < n; ++i)
  {
    for (int j = 0; j < n; ++j)
    {
      G[i][j] = _G_leader(i, j);
    }
  }

  for (int i = 0; i < n; ++i)
  {
    for (int j = 0; j < m; ++j)
    {
      CE[i][j] = (_CE.transpose())(i, j);
    }
  }

  for (int i = 0; i < n; ++i)
  {
    for (int j = 0; j < p; ++j)
    {
      CI[i][j] = (_CI.transpose())(i, j);
    }
  }

  for (int i = 0; i < n; ++i)
  {
    g0[i] = _g0T_leader[i];
  }

  for (int i = 0; i < m; ++i)
  {
    ce0[i] = _ce0[i];
  }

  for (int i = 0; i < p; ++i)
  {
    ci0[i] = _ci0[i];
  }

  double value = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);

  for (int i = 0; i < n; ++i)
  {
    // 输出u限幅，防止变化过快
    _u_n[i] = x[i];
  }

  //   std::cout <<"_u: "<< std::fixed << std::setprecision(6) << _u_n << std::endl;
}
void CLFCBF::solveQP_follower(Vec4 &_u_n)
{
  int n = _u_n.size();
  int m = _ce0.size();
  int p = _ci0.size();

  G.resize(n, n);
  CE.resize(n, m);
  CI.resize(n, p);
  g0.resize(n);
  ce0.resize(m);
  ci0.resize(p);
  x.resize(n);

  for (int i = 0; i < n; ++i)
  {
    for (int j = 0; j < n; ++j)
    {
      G[i][j] = _G_follower(i, j);
    }
  }

  for (int i = 0; i < n; ++i)
  {
    for (int j = 0; j < m; ++j)
    {
      CE[i][j] = (_CE.transpose())(i, j);
    }
  }

  for (int i = 0; i < n; ++i)
  {
    for (int j = 0; j < p; ++j)
    {
      CI[i][j] = (_CI.transpose())(i, j);
    }
  }

  for (int i = 0; i < n; ++i)
  {
    g0[i] = _g0T_follower[i];
  }

  for (int i = 0; i < m; ++i)
  {
    ce0[i] = _ce0[i];
  }

  for (int i = 0; i < p; ++i)
  {
    ci0[i] = _ci0[i];
  }

  double value = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);

  for (int i = 0; i < n; ++i)
  {
    // 输出u限幅，防止变化过快
    _u_n[i] = x[i];
  }

  //   std::cout <<"_u: "<< std::fixed << std::setprecision(6) << _u_n << std::endl;
}

void CLFCBF::BS1(Vec2 v_n)
{
  double I_v;

  I_v = pow(v_m.norm(), 2) - pow(v_n.norm(), 2);

  A_v = 2 * v_n;
  b_v = k_v * I_v * Eigen::MatrixXd::Identity(1, 1);
}

void CLFCBF::readDataFromFile(const std::string &filename, Eigen::MatrixXd &mat)
{
  mat.resize(350, 600);
  // 打开文件
  std::ifstream file(filename);
  if (!file)
  {
    std::cerr << "无法打开文件：" << filename << std::endl;
    return;
  }

  std::string line;
  int row = 0;

  // 逐行读取数据
  while (std::getline(file, line) && row < mat.rows())
  {
    std::istringstream stream(line);
    for (int col = 0; col < mat.cols(); ++col)
    {
      stream >> mat(row, col); // 将数据存入Eigen矩阵
    }
    ++row;
  }
}

void CLFCBF::OA(int i, int robot_name, Vec2 pos_n, Vec2 v_n, Vec2 &A_cbf_tr_n, Eigen::Matrix<double, 1, 1> &b_cbf_tr_n)
{
  Vec2 pos_int = pos_n;
  pos_int(0) = max(1, min(int(pos_int(0)), 349));
  pos_int(1) = max(1, min(int(pos_int(1)), 599));
  double spi = sdf.block(pos_int(0), pos_int(1), 1, 1)(0, 0);
  double gdx = GDX.block(pos_int(0), pos_int(1), 1, 1)(0, 0);
  double gdy = GDY.block(pos_int(0), pos_int(1), 1, 1)(0, 0);

  Vec2 grad;
  grad << gdx, gdy;
  A_cbf_tr_n = -spi * grad;
  b_cbf_tr_n = (spi * (pow(k_oa, 2) * (spi - deltap) + 2 * k_oa * grad.dot(v_n)) - pow(grad.dot(v_n), 2) + v_n.dot(v_n)) * Eigen::MatrixXd::Identity(1, 1);

  // std::cout << "spi-deltap: " << spi - deltap << std::endl;

 if (i == 0)
  {
    switch (robot_name)
    {
    case 1:
      file1.open("/home/yth/project/unitree_guide/src/unitree_guide/src/CLFCBF/OA1.txt", ofstream::out | std::ofstream::app);
      // 写入CSV文件，使用逗号分隔
      file1 << spi - deltap << std::endl; // 写入数据

      file1.close();

      break;

    case 2:
      file2.open("/home/yth/project/unitree_guide/src/unitree_guide/src/CLFCBF/OA2.txt", ofstream::out | std::ofstream::app);
      // 写入CSV文件，使用逗号分隔
      file2 << spi - deltap << std::endl; // 写入数据

      file2.close();

      break;

    case 3:
      file3.open("/home/yth/project/unitree_guide/src/unitree_guide/src/CLFCBF/OA3.txt", ofstream::out | std::ofstream::app);

      // 写入CSV文件，使用逗号分隔
      file3 << spi - deltap << std::endl; // 写入数据

      file3.close();

      break;

    default:
      break;
    }
  }
}

void CLFCBF::SA(int i, int robot_name){
  switch (robot_name)
  {
  case 1:
    p12 = pos_1 - pos_2;
    p13 = pos_1 - pos_3;
    v12 = v_1 - v_2;
    v13 = v_1 - v_3;

    A_cbf_sa1 = -2 * p12;
    b_cbf_sa1 = (v12.dot(v12) + 4 * k_sa * p12.dot(v_1) + 0.5 * pow(k_sa,2) * (pow(p12.norm(),2) - pow(delta12,2))) * Eigen::MatrixXd::Identity(1, 1);

    A_cbf_sa2 = -2 * p13;
    b_cbf_sa2 = (v13.dot(v13) + 4 * k_sa * p13.dot(v_1) + 0.5 * pow(k_sa,2) * (pow(p13.norm(),2) - pow(delta13,2))) * Eigen::MatrixXd::Identity(1, 1);
    
    if(i==0){
      file13.open("/home/yth/project/unitree_guide/src/unitree_guide/src/CLFCBF/p12.txt", ofstream::out | std::ofstream::app);
      // 写入CSV文件，使用逗号分隔
      file13 << pow(p12.norm(),2) - pow(delta12,2) << std::endl; // 写入数据

      file13.close();

      file14.open("/home/yth/project/unitree_guide/src/unitree_guide/src/CLFCBF/p13.txt", ofstream::out | std::ofstream::app);
      // 写入CSV文件，使用逗号分隔
      file14 << pow(p13.norm(),2) - pow(delta13,2) << std::endl; // 写入数据

      file14.close();
    }

    break;
  
  case 2:
    p21 = pos_2 - pos_1;
    p23 = pos_2 - pos_3;
    v21 = v_2 - v_1;
    v23 = v_2 - v_3;

    A_cbf_sa1 = -2 * p21;
    b_cbf_sa1 = (v21.dot(v21) + 4 * k_sa * p21.dot(v_2) + 0.5 * pow(k_sa,2) * (pow(p21.norm(),2) - pow(delta21,2))) * Eigen::MatrixXd::Identity(1, 1);

    A_cbf_sa2 = -2 * p23;
    b_cbf_sa2 = (v23.dot(v23) + 4 * k_sa * p23.dot(v_2) + 0.5 * pow(k_sa,2) * (pow(p23.norm(),2) - pow(delta23,2))) * Eigen::MatrixXd::Identity(1, 1);
    
    if(i==0){
      file15.open("/home/yth/project/unitree_guide/src/unitree_guide/src/CLFCBF/p23.txt", ofstream::out | std::ofstream::app);
      // 写入CSV文件，使用逗号分隔
      file15 << pow(p23.norm(),2) - pow(delta23,2) << std::endl; // 写入数据

      file15.close();
    }

    break;

  case 3:
    p31 = pos_3 - pos_1;  
    p32 = pos_3 - pos_2;
    v31 = v_3 - v_1;
    v32 = v_3 - v_2;

    A_cbf_sa1 = -2 * p31;
    b_cbf_sa1 = (v31.dot(v31) + 4 * k_sa * p31.dot(v_3) + 0.5 * pow(k_sa,2) * (pow(p31.norm(),2) - pow(delta31,2))) * Eigen::MatrixXd::Identity(1, 1);

    A_cbf_sa2 = -2 * p32;
    b_cbf_sa2 = (v32.dot(v32) + 4 * k_sa * p32.dot(v_3) + 0.5 * pow(k_sa,2) * (pow(p32.norm(),2) - pow(delta32,2))) * Eigen::MatrixXd::Identity(1, 1);

    break;

  default:
    break;
  }
}