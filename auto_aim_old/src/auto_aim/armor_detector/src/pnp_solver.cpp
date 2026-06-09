// Copyright 2022 Chen Jun

#include "armor_detector/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>
#include <vector>
#include <cmath>

namespace rm_auto_aim
{
/**
 * @file pnp_solver.cpp
 * @brief PnP 求解器的实现：将装甲的图像顶点映射为相机坐标系下的位姿。
 *
 * 构造函数将相机内参和畸变系数转换为 OpenCV 的 cv::Mat 格式，并初始化
 * 小/大装甲的 3D 模型点（以米为单位）。
 */
PnPSolver::PnPSolver(
  const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone())
{
  // Unit: m
  /// 定义小/大装甲的3D模型点，单位为米
  constexpr double small_half_y = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
  constexpr double small_half_z = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;

  // constexpr double small_half_z = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0 * cos(15/180*M_PI);
  // constexpr double small_half_x = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0 * sin(15/180*M_PI);

  constexpr double large_half_y = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;
  constexpr double large_half_z = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0;
  // constexpr double large_half_z = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0 * cos(15/180*M_PI);
  // constexpr double large_half_x = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0 * sin(15/180*M_PI);

  /// 以装甲板中心为原点，从装甲板左下角开始，按顺时针顺序定义4个3D模型点
  /// 模型坐标系：x轴指向前方，y轴指向左侧，z轴指向上方
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, -small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, -small_half_z));

  // outpost_armor_points_.emplace_back(cv::Point3f(small_half_x, small_half_y, -small_half_z));
  // outpost_armor_points_.emplace_back(cv::Point3f(-small_half_x, small_half_y, small_half_z));
  // outpost_armor_points_.emplace_back(cv::Point3f(-small_half_x, -small_half_y, small_half_z));
  // outpost_armor_points_.emplace_back(cv::Point3f(small_half_x, -small_half_y, -small_half_z));

  /** 
  outpost_armor_points_.emplace_back(cv::Point3f(0, small_half_y, -small_half_z));
  outpost_armor_points_.emplace_back(cv::Point3f(0, small_half_y, small_half_z));
  outpost_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, small_half_z));
  outpost_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, -small_half_z));
  * rensy注释**/
  
  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, -large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, -large_half_z));

  // large_armor_points_.emplace_back(cv::Point3f(-large_half_x, large_half_y, -large_half_z));
  // large_armor_points_.emplace_back(cv::Point3f(large_half_x, large_half_y, large_half_z));
  // large_armor_points_.emplace_back(cv::Point3f(large_half_x, -large_half_y, large_half_z));
  // large_armor_points_.emplace_back(cv::Point3f(-large_half_x, -large_half_y, -large_half_z));
}

bool PnPSolver::solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec)
{
  std::vector<cv::Point2f> image_armor_points;

  // y轴图像偏移量,手动在图像坐标系中进行补偿
  // float offset_y = 68;

  // cv::Point2f left_bottom, left_top, right_bottom, right_top;
  // left_bottom = armor.left_light.bottom;
  // left_top = armor.left_light.top;
  // right_top = armor.right_light.top;
  // right_bottom = armor.right_light.bottom;

  // /******************进行补偿*****************/
  // left_bottom.y = left_bottom.y + offset_y;
  // left_top.y = left_top.y + offset_y;
  // right_top.y = right_top.y + offset_y;
  // right_bottom.y = right_bottom.y + offset_y;
  // /******************进行补偿*****************/

  //   // Fill in image point
  // image_armor_points.emplace_back(left_bottom);
  // image_armor_points.emplace_back(left_top);
  // image_armor_points.emplace_back(right_top);
  // image_armor_points.emplace_back(right_bottom);

  /// 填充图像点，顺序与3D模型点对应  
  image_armor_points.emplace_back(armor.left_light.bottom);
  image_armor_points.emplace_back(armor.left_light.top);
  image_armor_points.emplace_back(armor.right_light.top);
  image_armor_points.emplace_back(armor.right_light.bottom);

  /// 基于装甲板类型选择对应的3D模型点，调用OpenCV的solvePnP函数进行求解
  auto object_points = armor.type == ArmorType::SMALL ? small_armor_points_: large_armor_points_;
  return cv::solvePnP(
    object_points, image_armor_points, camera_matrix_, dist_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point)
{
  float cx = camera_matrix_.at<double>(0, 2);
  float cy = camera_matrix_.at<double>(1, 2);
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

}  // namespace rm_auto_aim
