// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef GIMBAL_COMMAND_PUBLISHER__GIMBAL_COMMAND_PUBLISHER_HPP_
#define GIMBAL_COMMAND_PUBLISHER__GIMBAL_COMMAND_PUBLISHER_HPP_

#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "jlcv_interfaces/msg/target.hpp"
#include "rmoss_projectile_motion/gravity_projectile_solver.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/create_timer_ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace solve_angle {

class SolveAngle : public rclcpp::Node {
public:
  double BULLET_SPEED = 15.4;
  explicit SolveAngle(const rclcpp::NodeOptions &options);

private:
  rclcpp::TimerBase::SharedPtr timer_;
  jlcv_interfaces::msg::Target::SharedPtr target_;
  rmoss_projectile_motion::GravityProjectileSolver solver_;

  // rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr gimbal_state_sub_;
  rclcpp::Subscription<jlcv_interfaces::msg::Target>::SharedPtr target_sub_;
  
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gimbal_command_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr aiming_point_pub_;
  
  const double TIME_OFFSET = 0.075; //0.35;
  const double ARMOR_YAW_LIMIT = 0.6;
  const double ARMOR_YAW_LIMIT_OFFSET = 0.22;
  double ALLOW_ERROR_DISTANCE = 0.04;
  double PITCH_OFFSET = 0.01;  //0.03;  //down +
  double PITCH_OFFSET1 = -0.0;//-0.024; 
  double YAW_OFFSET = -0.012;//0.03; 往左调 +
  double YAW_OFFSET1 = -0.01;//0.03; 往左调 +

  double gimbal_pitch_, gimbal_yaw_;
  double latency, fly_time, horizontal_speed, distance;
  double xc, yc, z, a_n, v_yaw, r;
  double armor_yaw, center_yaw, allow_yaw, tmp_yaw, yaw_diff;
  double pre_xc, pre_yc, pre_yaw, final_x, final_y,final_z;
  double pitch, gimbal_command_pitch_, gimbal_command_yaw_, control_status;
  bool camera_id = 0;
  double min_diff_all = 3.14;
  double last_distance = 0;

  visualization_msgs::msg::Marker aiming_point_;

  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

};

} // namespace solve_angle

#endif // GIMBAL_COMMAND_PUBLISHER__GIMBAL_COMMAND_PUBLISHER_HPP_