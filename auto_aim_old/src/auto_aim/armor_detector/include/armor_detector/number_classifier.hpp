// Copyright 2022 Chen Jun

#ifndef ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
#define ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_

// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"

namespace rm_auto_aim
{
/**
 * @class NumberClassifier
 * @brief 封装 ONNX DNN 的数字识别器，用于识别装甲板上的数字。
 *
 * 该分类器保存已加载的网络和标签列表。使用流程：
 *  1. 调用 `extractNumbers()` 将每个装甲的数字 ROI 仿射/透视变换为规范尺寸。
 *  2. 调用 `classify()` 运行网络，并填充 `Armor::number` 及相关字段。
 */
class NumberClassifier
{
public:
  /**
   * @brief Construct a NumberClassifier
   * @param model_path Path to ONNX model
   * @param label_path Path to label text file (one label per line)
   * @param threshold Minimum confidence to accept a classification
   * @param ignore_classes List of class names to ignore (will drop detections)
   */
  NumberClassifier(
    const std::string & model_path, const std::string & label_path, const double threshold,
    const std::vector<std::string> & ignore_classes = {});

  /**
   * @brief Extract and warp number ROIs from each armor into `Armor::number_img`.
   * @param src Source RGB image
   * @param armors Vector of armor candidates to modify in-place
   */
  void extractNumbers(const cv::Mat & src, std::vector<Armor> & armors);

  /**
   * @brief Run the DNN on each armor's `number_img` and fill classification
   *        fields. Low-confidence or ignored classes will be removed from the
   *        armor list.
   * @param armors Vector of armor candidates to classify (may be filtered)
   */
  void classify(std::vector<Armor> & armors);

  /** Confidence threshold for accepting a classification (0..1) */
  double threshold;

private:
  cv::dnn::Net net_;
  std::vector<std::string> class_names_;
  std::vector<std::string> ignore_classes_;
};
}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
