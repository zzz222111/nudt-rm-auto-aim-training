// Copyright 2022 Chen Jun

/**
 * @file extended_kalman_filter.cpp
 * @brief 可复用的小型扩展卡尔曼滤波器实现。
 *
 * 实现遵循标准 EKF 方程：
 *  - 预测： x_pri = f(x_post); P_pri = F * P_post * F^T + Q
 *  - 更新： K = P_pri * H^T * (H*P_pri*H^T + R)^{-1};
 *          x_post = x_pri + K * (z - h(x_pri));
 *          P_post = (I - K*H) * P_pri
 */

#include "armor_processor/extended_kalman_filter.hpp"

namespace rm_auto_aim
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const VecVecFunc & f, const VecVecFunc & h, const VecMatFunc & j_f, const VecMatFunc & j_h,
  const VoidMatFunc & u_q, const VecMatFunc & u_r, const Eigen::MatrixXd & P0)
: f(f),
  h(h),
  jacobian_f(j_f),
  jacobian_h(j_h),
  update_Q(u_q),
  update_R(u_r),
  P_post(P0),
  n(P0.rows()),
  I(Eigen::MatrixXd::Identity(n, n)),
  x_pri(n),
  x_post(n)
{
  
  
}

void ExtendedKalmanFilter::setState(const Eigen::VectorXd & x0) { x_post = x0; }

Eigen::MatrixXd ExtendedKalmanFilter::predict()
{
  F = jacobian_f(x_post);
  Q = update_Q();
  x_pri = f(x_post);                //没有控制量？？？？？？？？？？？？？？？
  P_pri = F * P_post * F.transpose() + Q;

  // handle the case when there will be no measurement before the next predict  ？？？？？？
  x_post = x_pri;
  P_post = P_pri;

  return x_pri;
}

Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd & z)
{
  
  H = jacobian_h(x_pri);
  R = update_R(z);
  e=z - h(x_pri);
  

  K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
  x_post = x_pri + K * e;
  P_post = (I - K * H) * P_pri;
  //Q=0.9*Q+0.1*(K*e*e.transpose()*K.transpose()+P_post-F * P_post * F.transpose());
  // R=0.9*R+0.1*(z*z.transpose()-H*P_post*H.transpose());
  return x_post;      
}

}  // namespace rm_auto_aim
