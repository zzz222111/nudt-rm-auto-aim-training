// Copyright 2022 Chen Jun
// Licensed under the MIT License.

/**
 * @file detector.hpp
 * @brief 用于提取光条并配对为装甲的检测流水线。
 *
 * `Detector` 类封装图像预处理、光条候选提取、光条配对为装甲以及调试信息的采集。
 * 它还持有一个 `NumberClassifier` 实例用于从装甲板中提取并识别数字。
 *
 * 接口契约（输入/输出）：
 *  - 输入：RGB 图像（cv::Mat），通过 `detect()` 提供
 *  - 输出：包含检测到的装甲候选（vector<Armor>），每个候选含数字裁剪图像与分类结果
 */

#ifndef ARMOR_DETECTOR__DETECTOR_HPP_
#define ARMOR_DETECTOR__DETECTOR_HPP_

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/number_classifier.hpp"
#include "jlcv_interfaces/msg/debug_armors.hpp"
#include "jlcv_interfaces/msg/debug_lights.hpp"

#include <openvino/openvino.hpp>
namespace rm_auto_aim
{
/**
 * @brief Main detection pipeline class.
 *
 * Responsibilities:
 *  - Preprocess incoming RGB frames into a binary mask for the target color.
 *  - Find individual light-bar candidates and pair them into armor
 *    hypotheses.
 *  - Maintain debug outputs and forward number classification to the
 *    `NumberClassifier`.
 */
class Detector
{
public:
  /**
   * @brief Parameters controlling light candidate filtering.
   */
  struct LightParams
  {
    /// Minimum (short side / long side) ratio allowed for a light bar
    double min_ratio;
    /// Maximum (short side / long side) ratio allowed for a light bar
    double max_ratio;
    /// Maximum tilt angle (degrees)
    double max_angle;
  };

  /**
   * @brief Parameters controlling pairing of lights into armors.
   */
  struct ArmorParams
  {
    /// Minimum ratio between light lengths when pairing
    double min_light_ratio;
    /// Distance ranges (in units of light length) for small armors
    double min_small_center_distance;
    double max_small_center_distance;
    /// Distance ranges (in units of light length) for large armors
    double min_large_center_distance;
    double max_large_center_distance;
    /// Maximum horizontal angle between lights
    double max_angle;
  };

  /**
   * @brief 装甲板检测器的构造函数
   * @param bin_thres 预处理的二值化与之 (unused in current
   *        algorithm, kept for compatibility)
   * @param color 敌方的装甲板灯条颜色
   * @param l 灯条尺寸参数
   * @param a 装甲板尺寸参数
   */
  Detector(const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a);

  /**
   * @brief 装甲板yolo检测器的构造函数
   * @param config_path 配置文件路径
   */
  Detector(const std::string &config_path);
  /**
   * @brief Run detection on an RGB image
   * @param input RGB input image
   * @return std::vector<Armor> Detected armors (may be empty)
   */
  std::vector<Armor> detect(const cv::Mat & input);

  /**
   * @brief Convert RGB image into a binary mask highlighting target color
   * @param input RGB input image
   * @return cv::Mat Binary mask (mono8)
   */
  cv::Mat preprocessImage(const cv::Mat & input);

  /**
   * @brief Extract light candidates from binary mask and verify color
   * @param rbg_img Original RGB image (for color checks)
   * @param binary_img Binary mask from preprocessImage
   * @return std::vector<Light> list of validated lights
   */
  std::vector<Light> findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img);

  /**
   * @brief Pair lights to build armor hypotheses
   * @param lights List of candidate lights
   * @return std::vector<Armor> Matched armors
   */
  std::vector<Armor> matchLights(const std::vector<Light> & lights);

  /**
   * @name Debug helpers
   *@{
   */
  cv::Mat getAllNumbersImage();
  void drawResults(cv::Mat & img);
  /**@}*/

  /**
   * 额外新定义的yolo处理函数
   */
  std::vector<Armor> yolodetect(cv::Mat &raw_img);
  std::vector<Armor> postprocess(double scale, cv::Mat & output, const cv::Mat & bgr_img);


  int binary_thres;
  int enemy_color;
  LightParams l;
  ArmorParams a;

  std::unique_ptr<NumberClassifier> classifier;

  // Debug msgs
  cv::Mat binary_img;
  jlcv_interfaces::msg::DebugLights debug_lights;
  jlcv_interfaces::msg::DebugArmors debug_armors;

private:
  /**
   * @brief Check if a rotated rect plausibly represents a light bar.
   */
  bool isLight(const Light & possible_light);

  /**
   * @brief Check whether the rectangle formed by two lights contains another
   *        candidate light (used to reject nested detections).
   */
  bool containLight(
    const Light & light_1, const Light & light_2, const std::vector<Light> & lights);

  /**
   * @brief Geometric checks to assert whether two lights form an armor and
   *        which type (SMALL/LARGE)
   */
  ArmorType isArmor(const Light & light_1, const Light & light_2);

  std::vector<Light> lights_;
  std::vector<Armor> armors_;


  /**
   * 下面是新定义的yolo相关私有成员变量
   */
  std::string model_path_, device_;
  std::string save_path_, debug_path_;
  const int class_num_ = 13;
  const float nms_threshold_ = 0.3;
  const float score_threshold_ = 0.7;
  double min_confidence_, binary_threshold_;

  ov::Core core_;
  ov::CompiledModel compiled_model_;

  cv::Rect roi_;
  cv::Point2f offset_;
  cv::Mat tmp_img_;
  
  friend class MultiThreadDetector;

  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;

  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

  std::vector<Armor> parse(double scale, cv::Mat & output, const cv::Mat & bgr_img);

  void save(const Armor & armor) const;
  void draw_detections(const cv::Mat & img, const std::list<Armor> & armors) const;
  double sigmoid(double x);
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_HPP_
