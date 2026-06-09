// Copyright 2022 Chen Jun
// Licensed under the MIT License.

/**
 * @file detector_node.hpp
 * @brief 装甲检测器的 ROS2 组件节点声明。
 *
 * 本节点订阅相机图像，运行 `Detector` 管道以定位装甲，使用 `PnPSolver`
 * 估计 3D 姿态，并发布 `jlcv_interfaces::msg::Armors` 与 RViz 可视化标记。
 * 当开启 `debug` 参数时，还会发布二值图、数字图和调试消息等主题。
 */

#ifndef ARMOR_DETECTOR__DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__DETECTOR_NODE_HPP_

// ROS
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/detector.hpp"
#include "armor_detector/number_classifier.hpp"
#include "armor_detector/pnp_solver.hpp"
#include "jlcv_interfaces/msg/armors.hpp"
#include "jlcv_interfaces/msg/game_status.hpp"

namespace rm_auto_aim
{
/**
 * @brief ROS2 node component that runs the armor detection pipeline.
 *
 * Subscribes to camera images, runs detection/classification, computes PnP
 * poses and publishes `jlcv_interfaces::msg::Armors` and visualization
 * markers. When `debug` is enabled, publishes intermediate images and
 * debug messages.
 */
class ArmorDetectorNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new ArmorDetectorNode
   * @param options NodeOptions forwarded to rclcpp::Node
   */
  ArmorDetectorNode(const rclcpp::NodeOptions & options);

  /**
   * @brief Destroy the ArmorDetectorNode and release resources.
   */
  ~ArmorDetectorNode();// = default;

  /** Video recording helper (optional). */
  cv::VideoWriter record;
private:
  /**
   * @brief Run the detector on an incoming image and return candidate armors.
   * @param img_msg ROS image message (assumed rgb8)
   * @return std::vector<Armor> list of detected armors
   */
  std::vector<Armor> detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg);

  /**
   * @brief Publish RViz markers stored in `marker_array_`.
   */
  void publishMarkers();

  // Camera info subscription
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;

  rclcpp::Subscription<jlcv_interfaces::msg::GameStatus>::SharedPtr game_status_sub_;

  // Camera info
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;

  // Camera center
  cv::Point2f cam_center_;

  // Image subscriptions transport type
  std::string transport_;

  // Detected armors publisher
  jlcv_interfaces::msg::Armors armors_msg_;
  rclcpp::Publisher<jlcv_interfaces::msg::Armors>::SharedPtr armors_pub_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker armor_marker_;
  visualization_msgs::msg::Marker text_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::unique_ptr<Detector> initDetector();
  std::unique_ptr<Detector> initYoloDetector();

  std::shared_ptr<image_transport::Subscriber> img_sub_;
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg);

  std::unique_ptr<PnPSolver> pnp_solver_;

  void createDebugPublishers();
  void destroyDebugPublishers();

  // Armor Detector
  std::unique_ptr<Detector> detector_;

  // Debug information publishers
  bool debug_;
  bool flip_;
  int camera_status = -1;
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
  rclcpp::Publisher<jlcv_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
  rclcpp::Publisher<jlcv_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;
  image_transport::Publisher binary_img_pub_;
  image_transport::Publisher number_img_pub_;
  image_transport::Publisher final_img_pub_;
  bool camera_id_ = 0;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_NODE_HPP_
