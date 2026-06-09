// Copyright 2022 Chen Jun
// Licensed under the MIT License.

/**
 * @file pnp_solver.hpp
 * @brief 为检测到的装甲求解 PnP 并计算与图像中心的距离辅助方法。
 *
 * `PnPSolver` 利用 OpenCV 的 `solvePnP` 将装甲顶点的 2D 图像坐标转换为
 * 相机坐标系下的 3D 位姿。类内部保有小/大装甲的标准 3D 模型点，并提供
 * 计算像素点到相机主点距离的辅助函数。
 */

#ifndef ARMOR_DETECTOR__PNP_SOLVER_HPP_
#define ARMOR_DETECTOR__PNP_SOLVER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core.hpp>

// STD
#include <array>
#include <vector>

#include "armor_detector/armor.hpp"

namespace rm_auto_aim
{
class PnPSolver
{
public:
  /**
   * @brief Construct a PnPSolver from camera intrinsics and distortion.
   * @param camera_matrix 3x3 intrinsic matrix in row-major order (array of 9)
   * @param distortion_coefficients distortion vector (k1,k2,p1,p2,...)
   */
  PnPSolver(
    const std::array<double, 9> & camera_matrix,
    const std::vector<double> & distortion_coefficients);

  /**
   * @brief Solve Perspective-n-Point for a detected armor
   * @param armor Input armor containing 4 image points (from lights)
   * @param rvec Output rotation vector (Rodrigues)
   * @param tvec Output translation vector
   * @return true if PnP succeeded
   */
  bool solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec);

  /**
   * @brief Compute pixel distance between an image point and camera center.
   * @param image_point Image coordinate (2D)
   * @return float Euclidean pixel distance to principal point
   */
  float calculateDistanceToCenter(const cv::Point2f & image_point);

private:
  cv::Mat camera_matrix_;   ///< Camera intrinsic matrix (3x3, CV_64F)
  cv::Mat dist_coeffs_;     ///< Distortion coefficients

  // Unit: mm for model points
  static constexpr float SMALL_ARMOR_WIDTH = 135;
  static constexpr float SMALL_ARMOR_HEIGHT = 56;
  static constexpr float LARGE_ARMOR_WIDTH = 230;
  static constexpr float LARGE_ARMOR_HEIGHT = 56;

  // 3D model points for different armor types (clockwise order)
  std::vector<cv::Point3f> small_armor_points_;
  /**std::vector<cv::Point3f> outpost_armor_points_; rensy注释**/
  std::vector<cv::Point3f> large_armor_points_;

  // Intermediate matrices used by algorithms (kept for compatibility)
  // cv::Mat ArmorPos;
  // cv::Mat MuzzlePos;
  // cv::Mat MuzzlePosUse;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__PNP_SOLVER_HPP_
