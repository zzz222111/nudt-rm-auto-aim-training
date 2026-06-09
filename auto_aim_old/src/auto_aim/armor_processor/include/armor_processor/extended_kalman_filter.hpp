// Copyright 2022 Chen Jun

/**
 * @file extended_kalman_filter.hpp
 * @brief 用于装甲跟踪的小型扩展卡尔曼滤波器辅助类。
 * 
 * 这个
 *
 * 此 EKF (扩展卡尔曼滤波器) 实现了一个通用接口。
 * 在该接口中，非线性的过程函数 (f)、测量函数 (h) 及其雅可比矩阵都是通过 std::function 对象来提供的。
 * 该滤波器会存储先验/后验协方差，并提供 predict() (预测) 和 update() (更新) 方法供 Tracker (跟踪器) 使用。
 *
 * 这种设计使得此滤波器可以复用于不同的状态维度和测量维度，
 * 同时也允许调用者提供自定义的过程噪声和测量噪声生成器。
 */

#ifndef ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
#define ARMOR_PROCESSOR__KALMAN_FILTER_HPP_

#include <Eigen/Dense>
#include <functional>

namespace rm_auto_aim
{

class ExtendedKalmanFilter
{
public:
  ExtendedKalmanFilter() = default;

  using VecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
  using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;
  using VoidMatFunc = std::function<Eigen::MatrixXd()>;

  explicit ExtendedKalmanFilter(
    const VecVecFunc & f, const VecVecFunc & h, const VecMatFunc & j_f, const VecMatFunc & j_h,
    const VoidMatFunc & u_q, const VecMatFunc & u_r, const Eigen::MatrixXd & P0);

  // Set the initial state
  /**
   * @brief 设置初始后验状态 x_post（用于初始化滤波器）。
   * @param x0 初始状态向量
   */
  void setState(const Eigen::VectorXd & x0);

  /**
   * @brief 执行 EKF 的预测步骤。
   * @return Eigen::MatrixXd 预测得到的状态向量（x_pri）
   */
  Eigen::MatrixXd predict();

  /**
   * @brief 使用观测向量 z 更新 EKF。
   * @param z 观测向量
   * @return Eigen::MatrixXd 后验状态向量（x_post）
   */
  Eigen::MatrixXd update(const Eigen::VectorXd & z);

private:
  // Process nonlinear vector function
  VecVecFunc f;
  // Observation nonlinear vector function
  VecVecFunc h;
  // Jacobian of f()
  VecMatFunc jacobian_f;
  Eigen::MatrixXd F;
  // Jacobian of h()
  VecMatFunc jacobian_h;
  Eigen::MatrixXd H;
  // Process noise covariance matrix
  VoidMatFunc update_Q;
  Eigen::MatrixXd Q;
  // Measurement noise covariance matrix
  VecMatFunc update_R;
  Eigen::MatrixXd R;

  // Priori error estimate covariance matrix
  Eigen::MatrixXd P_pri;
  // Posteriori error estimate covariance matrix
  Eigen::MatrixXd P_post;

  // Kalman gain
  Eigen::MatrixXd K;
  Eigen::VectorXd e;

  // System dimensions
  int n;
  int k;

  // N-size identity
  Eigen::MatrixXd I;

  // Priori state
  Eigen::VectorXd x_pri;
  // Posteriori state
  Eigen::VectorXd x_post;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__KALMAN_FILTER_HPP_