// Copyright 2022 Chen Jun
// Licensed under the MIT License.

/**
 * @file detector_node.cpp
 * @brief 实现装甲检测节点的 ROS2 组件。
 * @details 主要负责实现detector_node.hpp中声明的方法。
 * 
 * 本源文件实现了 `detector_node.hpp` 中声明的逻辑：
 * - 参数初始化、订阅相机图像和相机信息；
 * - 初始化 `Detector` 和 `PnPSolver`；
 * - 发布检测到的装甲（`jlcv_interfaces::msg::Armors`）以及可视化标记和调试图像主题。
 */

#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include "rcpputils/filesystem_helper.hpp"

// Eigen
#include <Eigen/Eigen>

// STD
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/detector_node.hpp"

namespace rm_auto_aim
{

ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions& options) : Node("armor_detector", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");
  // record.open("/home/jlcv10/Documents/four.avi", cv::VideoWriter::fourcc('M','J','P','G'), 100.0,cv::Size(1440.0,1080.0));

  // Detector
  detector_ = initYoloDetector();

  // Subscriptions transport type 
  // 决定图像传输方式，压缩还是原图
  transport_ = this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";

  // Armors Publisher
  armors_pub_ = this->create_publisher<jlcv_interfaces::msg::Armors>("/detector/armors", rclcpp::SensorDataQoS());

  // Visualization Marker Publisher
  // Rviz相关话题信息发布
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  // 装甲识别信息可视化
  armor_marker_.ns = "armors";
  armor_marker_.action = visualization_msgs::msg::Marker::ADD;
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.03;
  armor_marker_.scale.y = 0.15;
  armor_marker_.scale.z = 0.12;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.r = 1.0;
  armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  // 数字识别情况的可视化
  text_marker_.ns = "classification"; 
  text_marker_.action = visualization_msgs::msg::Marker::ADD;
  text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker_.scale.z = 0.1;
  text_marker_.color.a = 1.0;
  text_marker_.color.r = 1.0;
  text_marker_.color.g = 1.0;
  text_marker_.color.b = 1.0;
  text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);

  // Debug Publishers
  // 默认启动调试信息发布
  debug_ = this->declare_parameter("debug", true);
  flip_ = this->declare_parameter("flip", false);
  if (debug_)
  {
    createDebugPublishers();
  }

  // Debug param change moniter
  debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  debug_cb_handle_ = debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter& p) {
    debug_ = p.as_bool();
    debug_ ? createDebugPublishers() : destroyDebugPublishers(); //debug 为 true 创建发布者，否则销毁
  });
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
      cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
      pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
      cam_info_sub_.reset();
    });  
  game_status_sub_ = this->create_subscription<jlcv_interfaces::msg::GameStatus>(//收节点信息，获取GameStatus
  "/game_status", rclcpp::SensorDataQoS(), 
  [this](jlcv_interfaces::msg::GameStatus::ConstSharedPtr game_status) {


    if(camera_status != game_status->camera_status || pnp_solver_ == nullptr)
    {
      std::cout<<"camera_status:"<<camera_status<<"\n";
      camera_status = game_status->camera_status;
      camera_id_ = camera_status;
      if(pnp_solver_ != nullptr)
      cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
          cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
          cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
          pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
          cam_info_sub_.reset();
        });  
        
      std::cout << "final_status:" << camera_status << "\n";
    }

  });  


  img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(
      this, "/image_raw", std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1), transport_,
      rmw_qos_profile_sensor_data));

}
ArmorDetectorNode::~ArmorDetectorNode()
{
  record.release();
}

/**
 * @brief 图像回调函数，处理每一帧图像以检测装甲板。
 * @param img_msg 输入的图像消息指针。
 * @details 该方法接收来自相机的图像消息，调用装甲检测器进行检测，并使用PnP求解器计算每个检测到的装甲板的位姿。
 *          最后，将检测结果发布到相应的ROS2话题，并发布用于Rviz可视化的标记。
 * **/

void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg)
{
  auto armors = detectArmors(img_msg);

  /// 使用PnP求解器计算每个检测到的装甲板的位姿并发布结果
  if (pnp_solver_ != nullptr)
  {
    armors_msg_.header = armor_marker_.header = text_marker_.header = img_msg->header;
    armors_msg_.armors.clear();
    armors_msg_.camera_id = camera_id_; /// 填充相机ID信息
    marker_array_.markers.clear(); /// 清空之前的标记数组
    armor_marker_.id = 0; /// 重置标记ID
    text_marker_.id = 0; /// 重置文本标记ID

    jlcv_interfaces::msg::Armor armor_msg;

    /// 遍历所有检测到的装甲板
    for (const auto& armor : armors)
    {
      /// 旋转向量rvec  相机<->世界  平移向量tvec  相机<->世界
      cv::Mat rvec, tvec;
      bool success = pnp_solver_->solvePnP(armor, rvec, tvec);
      if (success)
      {
        armor_msg.number = armor.number;
        // Fill armor_msg with pose
        armor_msg.pose.position.x = tvec.at<double>(0);
        armor_msg.pose.position.y = tvec.at<double>(1) - 0.05;
        armor_msg.pose.position.z = tvec.at<double>(2);

        // rvec to 3x3 rotation matrix
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        // rotation matrix to quaternion
        tf2::Matrix3x3 tf2_rotation_matrix(
            rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
            rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
            rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));
        tf2::Quaternion tf2_quaternion;
        tf2_rotation_matrix.getRotation(tf2_quaternion);
        armor_msg.pose.orientation.x = tf2_quaternion.x();
        armor_msg.pose.orientation.y = tf2_quaternion.y();
        armor_msg.pose.orientation.z = tf2_quaternion.z();
        armor_msg.pose.orientation.w = tf2_quaternion.w();
        // Fill the distance to image center
        armor_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(armor.center);
        // Fill the markers
        armor_marker_.id++;
        armor_marker_.pose = armor_msg.pose;
        text_marker_.id++;
        text_marker_.pose.position = armor_msg.pose.position;
        text_marker_.pose.position.y -= 0.1;
        text_marker_.text = armor.number;
        armors_msg_.armors.emplace_back(armor_msg);
        marker_array_.markers.emplace_back(armor_marker_);
        marker_array_.markers.emplace_back(text_marker_);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "PnP failed!");
      }
    }

    // Publishing detected armors
    armors_pub_->publish(armors_msg_);
    // Publishing marker
    publishMarkers();
  }
}

/**
 * @brief 初始化装甲检测器
 * @return std::unique_ptr<Detector> 装甲检测器实例
 * @details 该方法定义并声明了装甲板检测器的各项参数，包括图像预处理参数、目标选择参数、灯条检测参数和装甲板检测参数，
 *          并基于这些参数创建了 `Detector` 实例。此外，还初始化了数字分类器 `NumberClassifier`，用于对检测到的装甲板进行数字识别。
 * **/

std::unique_ptr<Detector> ArmorDetectorNode::initDetector()
{
  /// 图像预处理参数
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 255;
  int min_lightness = declare_parameter("binary_thres", 80, param_desc);

  /// 目标选择参数
  param_desc.description = "0-RED, 1-BLUE";
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 1;
  auto enemy_color = declare_parameter("enemy_color", BLUE, param_desc);

  /// 灯条检测参数
  Detector::LightParams l_params = { .min_ratio = declare_parameter("light.min_ratio", 0.08),
                                     .max_ratio = declare_parameter("light.max_ratio", 0.40),
                                     .max_angle = declare_parameter("light.max_angle", 40.0) };

  /// 装甲板检测参数
  Detector::ArmorParams a_params = {
    .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.6),
    .min_small_center_distance = declare_parameter("armor.min_small_center_distance", 0.6),
    .max_small_center_distance = declare_parameter("armor.max_small_center_distance", 3.5),
    .min_large_center_distance = declare_parameter("armor.min_large_center_distance", 3.5),
    .max_large_center_distance = declare_parameter("armor.max_large_center_distance", 5.0),
    .max_angle = declare_parameter("armor.max_angle", 35.0)
  };

  /// 初始化装甲板识别器
  auto detector = std::make_unique<Detector>(min_lightness, enemy_color, l_params, a_params);

  /// 初始化数字分类器
  auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
  auto model_path = pkg_path + "/model/mlp_f.onnx";
  auto label_path = pkg_path + "/model/label.txt";
  double threshold = this->declare_parameter("classifier_threshold", 0.7);
  std::vector<std::string> ignore_classes = this->declare_parameter("ignore_classes", std::vector<std::string>{ "negative" });
  detector->classifier = std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);

  return detector;
}

/**
 * 初始化yolo检测器
 */

std::unique_ptr<Detector> ArmorDetectorNode::initYoloDetector()
{

  // rcpputils::fs::path source_file_path(__FILE__) ;
  // auto armor_detector_path = source_file_path.parent_path().parent_path();
  // auto config = armor_detector_path / "configs" / "yolov5.yaml";
  // auto config_path = config.string();
  std::string config_path = "/home/rsy20/RoboMaster/src_25_5_23/src/auto_aim/armor_detector/model/yolov5.xml";
  auto detector = std::make_unique<Detector>(config_path);

  return detector;
}

/**
 * @brief 在输入的图像上运行装甲检测器并返回检测到的装甲列表
 * @param img_msg 输入的ROS图像消息，假设为rgb8格式
 * @return std::vector<Armor> 检测到的装甲列表
 * @details 该方法首先将ROS图像消息转换为OpenCV的cv::Mat格式，然后根据需要对图像进行翻转处理。
 *          接着，更新检测器的参数，并调用检测器的detect方法进行装甲检测。
 *          最后，如果启用了调试模式，还会发布相关的调试信息，包括二值化图像、灯条和装甲数据以及最终的检测结果图像。
 * **/

std::vector<Armor> ArmorDetectorNode::detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg)
{
  /// 把ROS图像转化为opencv可以读取的cv：：Mat格式
  auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

  if (flip_)
  {
    cv::flip(img, img, -1);
  }

  // 更新装甲板检测器里的参数
  detector_->binary_thres = get_parameter("binary_thres").as_int();
  detector_->enemy_color = get_parameter("enemy_color").as_int();
  detector_->classifier->threshold = get_parameter("classifier_threshold").as_double();

  auto armors = detector_->yolodetect(img);
  
  // if (!armors.empty()) printf("armor not found!\n");
  auto final_time = this->now();
  auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;
  // RCLCPP_DEBUG_STREAM(this->get_logger(), "Latency: " << latency << "ms");

  /// 发布调试信息
  if (debug_)
  {
    // record.write(img);

    binary_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img).toImageMsg());

    /// 通过x坐标对灯条和装甲板排序，确保发送的消息顺序一致
    std::sort(detector_->debug_lights.data.begin(), detector_->debug_lights.data.end(),
              [](const auto& l1, const auto& l2) { return l1.center_x < l2.center_x; });
    std::sort(detector_->debug_armors.data.begin(), detector_->debug_armors.data.end(),
              [](const auto& a1, const auto& a2) { return a1.center_x < a2.center_x; });

    lights_data_pub_->publish(detector_->debug_lights);
    armors_data_pub_->publish(detector_->debug_armors);

    auto all_num_img = detector_->getAllNumbersImage();
    number_img_pub_.publish(*cv_bridge::CvImage(img_msg->header, "mono8", all_num_img).toImageMsg());

    detector_->drawResults(img);

    /// 绘制相机中心点
    cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);

    /// 绘制延迟时间
    std::stringstream latency_ss;
    latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
    auto latency_s = latency_ss.str();
    cv::putText(img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    final_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
  }

  return armors;
}

/** 
 * @brief 创建调试消息发布器
 * @details 该方法创建了用于发布调试信息的ROS2发布器，包括灯条数据、装甲板数据以及二值化图像、数字图像和最终结果图像的发布器。
 * **/
void ArmorDetectorNode::createDebugPublishers()
{
  lights_data_pub_ = this->create_publisher<jlcv_interfaces::msg::DebugLights>("/detector/debug_lights", 10);
  armors_data_pub_ = this->create_publisher<jlcv_interfaces::msg::DebugArmors>("/detector/debug_armors", 10);

  binary_img_pub_ = image_transport::create_publisher(this, "/detector/binary_img");
  number_img_pub_ = image_transport::create_publisher(this, "/detector/number_img");
  final_img_pub_ = image_transport::create_publisher(this, "/detector/final_img");
}

/** 
 * @brief 销毁所有调试消息发布器
 * @details 该方法用于销毁用于发布调试消息的发布器
 * **/

void ArmorDetectorNode::destroyDebugPublishers()
{
  lights_data_pub_.reset();
  armors_data_pub_.reset();

  binary_img_pub_.shutdown();
  number_img_pub_.shutdown();
  final_img_pub_.shutdown();
}

/**
 * @brief 发布用于Rviz可视化的标记数组
 * @details 该方法将存储在 `marker_array_` 中的标记发布到相应的ROS2话题，以便在Rviz中进行可视化显示。
 * **/

void ArmorDetectorNode::publishMarkers()
{
  // using Marker = visualization_msgs::msg::Marker;

  // marker_array_.markers.emplace_back(armor_marker_);
  marker_pub_->publish(marker_array_);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)
