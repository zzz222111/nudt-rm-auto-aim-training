// Copyright 2022 Chen Jun

/**
 * @file processor_node.hpp
 * @brief Declares the ROS2 node that receives detected armors, tracks a target
 *        across frames and publishes a fused `Target` message.
 *
 * The node integrates a `Tracker` (which internally uses an EKF) and handles
 * TF transforms, measurement publication for debugging, and RViz markers.
 */

#ifndef ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_

// ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_processor/tracker.hpp"
#include "jlcv_interfaces/msg/armors.hpp"
#include "jlcv_interfaces/msg/target.hpp"
#include "jlcv_interfaces/msg/measurement.hpp"

namespace rm_auto_aim
{
using tf2_filter = tf2_ros::MessageFilter<jlcv_interfaces::msg::Armors>;
class ArmorProcessorNode : public rclcpp::Node
{
public:
  explicit ArmorProcessorNode(const rclcpp::NodeOptions & options);

private:
  /**
   * @brief 当（经过 TF 过滤的）`Armors` 消息到达时调用的回调。
   * @param armors_ptr 指向接收到的 Armors 消息的共享指针
   */
  void armorsCallback(const jlcv_interfaces::msg::Armors::SharedPtr armors_ptr);

  /**
   * @brief 为给定的融合目标消息发布 RViz 标记以进行可视化。
   * @param target_msg 要可视化的目标消息
   */
  void publishMarkers(const jlcv_interfaces::msg::Target & target_msg);

  // Maximum allowable armor distance in the XOY plane
  double max_armor_distance_;

  // The time when the last message was received
  rclcpp::Time last_time_;
  double dt_;

  // Armor tracker
  double s2qxyz_, s2qyaw_, s2qr_;
  double r_xyz_factor, r_yaw;
  double lost_time_thres_;
  std::unique_ptr<Tracker> tracker_;
  
  // Reset tracker service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv_;

  // Subscriber with tf2 message_filter
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<jlcv_interfaces::msg::Armors> armors_sub_;
  std::shared_ptr<tf2_filter> tf2_filter_;

  // Measurement publisher
  rclcpp::Publisher<jlcv_interfaces::msg::Measurement>::SharedPtr measure_pub_;

  // Publisher
  rclcpp::Publisher<jlcv_interfaces::msg::Target>::SharedPtr target_pub_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker linear_v_marker_;
  visualization_msgs::msg::Marker angular_v_marker_;
  visualization_msgs::msg::Marker armors_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_