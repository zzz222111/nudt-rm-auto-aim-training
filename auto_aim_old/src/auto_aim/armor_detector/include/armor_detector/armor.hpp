// Copyright 2022 Chen Jun
// Licensed under the MIT License.

/**
 * @file armor.hpp
 * @brief 装甲检测中使用的光条与装甲数据结构。
 *
 * 本头文件定义了用于表示检测到的光条（`Light`）和由光条配对出的装甲板（`Armor`）
 * 的轻量级 POD 风格容器。它们保存图像几何信息（中心、角点）、
 * 几何评估量（长度、宽度、倾斜角）以及识别输出（数字裁剪图像、标签、置信度）。
 *
 * 用途说明：
 *  - `Light` 封装从 OpenCV 返回的旋转矩形，并计算常用的点（上端/下端）、长度和宽度。
 *  - `Armor` 将两个光条配对，计算中心并保存数字识别阶段的结果；`type` 表示小装甲或大装甲。
 */

#ifndef ARMOR_DETECTOR__ARMOR_HPP_
#define ARMOR_DETECTOR__ARMOR_HPP_

#include <opencv2/core.hpp>

// STL
#include <algorithm>
#include <string>

namespace rm_auto_aim
{

enum ArmorName
{
  one,
  two,
  three,
  four,
  five,
  sentry,
  outpost,
  base,
  not_armor
};
const std::vector<std::string> ARMOR_NAMES = {"one",    "two",     "three", "four",     "five",
                                              "sentry", "outpost", "base",  "not_armor"};

enum Color
{
  red,
  blue,
  extinguish,
  purple
};
const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple"};


const int RED = 0;
const int BLUE = 1;

enum class ArmorType { SMALL, LARGE, INVALID };
const std::string ARMOR_TYPE_STR[3] = {"small", "large", "invalid"};

/**
 * @brief Lightweight representation of a single detected light bar.
 *
 * Inherits from cv::RotatedRect and computes convenient points (top/bottom),
 * geometric properties (length, width) and approximate tilt angle.
 */
struct Light : public cv::RotatedRect
{
  Light() = default;
  explicit Light(cv::RotatedRect box) : cv::RotatedRect(box)
  {
    cv::Point2f p[4];
    box.points(p);
    std::sort(p, p + 4, [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; });
    top = (p[0] + p[1]) / 2;
    bottom = (p[2] + p[3]) / 2;

    length = cv::norm(top - bottom);
    width = cv::norm(p[0] - p[1]);

    tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
    tilt_angle = tilt_angle / CV_PI * 180;
  }

  int color;
  cv::Point2f top, bottom;
  double length;
  double width;
  float tilt_angle;
};

/**
 * @brief 代表一个配对的装甲板，它由两个 Light 对象构成。
 *
 * 存储了以下信息：
 * 左侧和右侧的灯条 (left/right light)
 * 计算出的中心点 (computed center point)
 * 检测到的装甲板类型 (detected armor type)
 * 数字识别的输出结果 (number recognition outputs)，其中包括：
 * 裁剪后的图像 (cropped image)
 * 识别标签 (label)
 * 置信度 (confidence）
 */
struct Armor
{
  Armor() = default;
  /**
   * @brief Construct an Armor from two lights.
   * @param l1 First light candidate
   * @param l2 Second light candidate
   *
   * The constructor orders the lights left/right by x coordinate and computes
   * the armor center.
   */
  Armor(const Light & l1, const Light & l2)
  {
    /// 判断左右灯条
    if (l1.center.x < l2.center.x) {
      left_light = l1, right_light = l2;
    } else {
      left_light = l2, right_light = l1;
    }
    center = (left_light.center + right_light.center) / 2;
  }

// YOLOV5构造函数
  Armor(
  int color_id, int num_id, float & conf, const cv::Rect & box,
  std::vector<cv::Point2f> armor_keypoints)
: confidence(conf){
  center = (armor_keypoints[0] + armor_keypoints[1] + armor_keypoints[2] + armor_keypoints[3]) / 4;
  Light l1, l2;
 
  left_light.top = armor_keypoints[0];
  right_light.bottom = armor_keypoints[1];
  right_light.bottom = armor_keypoints[2];
  left_light.bottom = armor_keypoints[3];

  left_light.tilt_angle = std::atan2(std::abs(left_light.top.x - left_light.bottom.x), 
                                      std::abs(left_light.top.y - left_light.bottom.y));
  right_light.tilt_angle = std::atan2(std::abs(right_light.top.x - right_light.bottom.x), 
                                      std::abs(right_light.top.y - right_light.bottom.y));           
  number = ARMOR_NAMES[num_id];

  left_light.color = color_id % 2;
  right_light.color = color_id % 2;
  confidence = conf;
  classfication_result = ARMOR_NAMES[num_id] + "_" + COLORS[color_id];
}
  // Light pairs part
  Light left_light, right_light;
  cv::Point2f center;
  ArmorType type; //有待商榷yolo怎么实现

  // Number part
  cv::Mat number_img; //yolo不再需要
  std::string number; 
  float confidence;
  std::string classfication_result;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__ARMOR_HPP_
