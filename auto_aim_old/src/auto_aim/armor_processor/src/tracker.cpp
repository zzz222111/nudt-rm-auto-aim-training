// Copyright 2022 Chen Jun

/**
 * @file tracker.cpp
 * @brief 跟踪器的具体实现：目标选择、EKF 初始化与更新，以及丢失/检测/跟踪的状态机。
 *
 * 跟踪器维护目标身份（字符串 id），并使用 EKF 对位姿/速度/偏航/半径进行平滑。
 * 它尝试将到达的检测与预测位置匹配，对突变进行保护，并依据检测到的装甲类别
 * 对半径/速度等值施加简单约束。
 */

#include "armor_processor/tracker.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <rclcpp/logger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <cfloat>
#include <memory>
#include <string>
#include <iostream>

namespace rm_auto_aim
{
Tracker::Tracker(double max_match_distance, double max_match_yaw_diff)
: tracker_state(LOST),
  tracked_id(std::string("")),
  measurement(Eigen::VectorXd::Zero(4)),
  target_state(Eigen::VectorXd::Zero(9)),
  max_match_distance_(max_match_distance),
  max_match_yaw_diff_(max_match_yaw_diff)
{
}

void Tracker::init(const Armors::SharedPtr & armors_msg)
{
  if (armors_msg->armors.empty()) {
    // std::cout<<"-------------------------------------------------"<<std::endl;
    return;
  }

  // Simply choose the armor that is closest to image center
  double min_distance = DBL_MAX;
  tracked_armor = armors_msg->armors[0];
  for (const auto & armor : armors_msg->armors) {
    if (armor.distance_to_image_center < min_distance) {
      min_distance = armor.distance_to_image_center;
      tracked_armor = armor;
    }
  }

  initEKF(tracked_armor);
  RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "Init EKF!");

  
  
  tracked_id = tracked_armor.number;
  tracker_state = DETECTING;

  updateArmorsNum(tracked_armor);
}

void Tracker::update(const Armors::SharedPtr & armors_msg)
{
  // KF predict
  Eigen::VectorXd ekf_prediction = ekf.predict();
  RCLCPP_DEBUG(rclcpp::get_logger("armor_processor"), "EKF predict");

  bool matched = false;
  // Use KF prediction as default target state if no matched armor is found
  target_state = ekf_prediction;
  //std::cout<< "x:"<<ekf_prediction(0)<<"\n";
  // std::cout<< "y:"<<ekf_prediction(2)<<"\n";
  // std::cout<< "z:"<<ekf_prediction(4)<<"\n";
  // std::cout<<"!!!!!!!!!!!"<<std::endl;
  if (!armors_msg->armors.empty()) {
    Armor same_id_armor;
    int same_id_armors_count = 0;
    double min_position_diff = DBL_MAX;
    double yaw_diff = DBL_MAX;
    auto predicted_position = getArmorPositionFromState(ekf_prediction);
    for (const auto & armor : armors_msg->armors) {
      // Only consider armors with the same id
      if (armor.number == tracked_id) {
        same_id_armor = armor;
        same_id_armors_count++;
        // Calculate the difference between the predicted position and the current armor position
        auto p = armor.pose.position;
        Eigen::Vector3d position_vec(p.x, p.y, p.z);
        double position_diff = (predicted_position - position_vec).norm();
        if (position_diff < min_position_diff) {
          // Find the closest armor
          min_position_diff = position_diff;
          // std::cout<< "ekf_yaw:"<<ekf_prediction(6)<<"\n";
          // std::cout<< "yaw:"<<orientationToYaw(armor.pose.orientation)<<"\n";
          yaw_diff = abs(orientationToYaw(armor.pose.orientation) - ekf_prediction(6));
          // std::cout<< "diff:"<<yaw_diff<<"\n";
          tracked_armor = armor;
        }
      }
    }
    // std::cout<<"min_position_diff:"<<min_position_diff<<std::endl;
    // Store tracker info
    info_position_diff = min_position_diff;
    info_yaw_diff = yaw_diff;

    // Check if the closest armor is close enough
    if (min_position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_) {
      // Matching armor found
      matched = true;
      // std::cout<<"!!!!!!!!!!!"<<std::endl;
      auto p = tracked_armor.pose.position;
      // Update EKF
      double measured_yaw = orientationToYaw(tracked_armor.pose.orientation);
      measurement = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);
      target_state = ekf.update(measurement);
      // std::cout<< "REAL.x:"<<target_state(0)<<"\n";
      // std::cout<< "REAL.y:"<<target_state(2)<<"\n";
      // std::cout<< "REAL.z:"<<target_state(4)<<"\n";
      //std::cout<<"|||||||||||||||||||||||||"<<std::endl;
      RCLCPP_DEBUG(rclcpp::get_logger("armor_processor"), "EKF update");
    } else if (same_id_armors_count == 1 && yaw_diff > max_match_yaw_diff_) {
      // Matched armor not found, but there is only one armor with the same id
      // and ytarget_state(7) = 2.513274; --------------------------------------\n";
        handleArmorJump(same_id_armor);
    } else {
      // No matched armor found
      RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "No matched armor found!");
    }
  }

  //limit
  if(tracked_armor.number == "1")
  {
    if (target_state(8) > 0.40) {
      target_state(8) = 0.40;
      ekf.setState(target_state);
    }
  }
  else if(tracked_armor.number == "2" || tracked_armor.number == "3" || tracked_armor.number == "4" || tracked_armor.number == "5" )
  {
    if (target_state(8) > 0.30) {
      target_state(8) = 0.30;
      ekf.setState(target_state);
    }
  }
  else if(tracked_armor.number == "guard")
  {
    if (target_state(8) > 0.35) {
      target_state(8) = 0.35;
      ekf.setState(target_state);
    }
  }
  else if(tracked_armor.number == "outpost")
  {
    if(abs(target_state(7)) >= 1)
    {
      if(target_state(7)<0) target_state(7) = -2.513274; 
      else target_state(7) = 2.513274; 
      target_state(8) = 0.2765;
      ekf.setState(target_state);
    }
    if (target_state(8) > 0.29) {  //0.28
      target_state(8) = 0.29;
      ekf.setState(target_state);
    }
    else if (target_state(8) < 0.26) {  //0.27
      target_state(8) = 0.26;
      ekf.setState(target_state);
    }
    if(abs(target_state(1))>0.01){
      target_state(1) = 0.01;
      ekf.setState(target_state);
    }
    if(abs(target_state(3))>0.01){
      target_state(3) = 0.01;
      ekf.setState(target_state);
    }
    if(abs(target_state(5))>0.01){
      target_state(5) = 0.01;
      ekf.setState(target_state);
    }
  }

  // Tracking state machine
  if (tracker_state == DETECTING) {
    if (matched) {
      detect_count_++;
      // std::cout<<"detect_count:"<<detect_count_<<std::endl;
      if (detect_count_ > tracking_thres) {
        detect_count_ = 0;
        tracker_state = TRACKING;
      }
    } else {
      detect_count_ = 0;
      tracker_state = LOST;
    }
  } else if (tracker_state == TRACKING) {
    if (!matched) {
      tracker_state = TEMP_LOST;
      lost_count_++;
    }
  } else if (tracker_state == TEMP_LOST) {
    if (!matched) {
      lost_count_++;
      // std::cout<<"lost_count:"<<lost_count_<<std::endl;
      if (lost_count_ > lost_thres) {
        lost_count_ = 0;
        tracker_state = LOST;
      }
    } else {
      tracker_state = TRACKING;
      lost_count_ = 0;
    }
  }
  
}

void Tracker::initEKF(const Armor & a)
{
  double xa = a.pose.position.x;
  double ya = a.pose.position.y;
  double za = a.pose.position.z;
  last_yaw_ = 0;
  double yaw = orientationToYaw(a.pose.orientation);

  // Set initial position at 0.2m behind the target
  target_state = Eigen::VectorXd::Zero(9);
  double r = 0.276;  //0.276
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);
  // last_z = zc, last_r = r;
  dz = za, another_r = r;
  target_state << xc, 0, yc, 0, za, 0, yaw, 0, r;

  ekf.setState(target_state);
}

void Tracker::updateArmorsNum(const Armor & armor)
{
  if (tracked_armor.type == "large" &&
    (tracked_id == "3" || tracked_id == "4" || tracked_id == "5")) {
    tracked_armors_num = ArmorsNum::BALANCE_2;
    task_mode = 2;
  } else if (tracked_id == "outpost") {
    tracked_armors_num = ArmorsNum::OUTPOST_3;
    task_mode = 1;
  } else {
    tracked_armors_num = ArmorsNum::NORMAL_4;
    task_mode = 0;
  }
}

void Tracker::handleArmorJump(const Armor & current_armor)
{
  double yaw = orientationToYaw(current_armor.pose.orientation);
  target_state(6) = yaw;
  updateArmorsNum(current_armor);

  if (tracked_armors_num == ArmorsNum::NORMAL_4) {
      dz = target_state(4);
      target_state(4) = current_armor.pose.position.z;
      std::swap(target_state(8), another_r);  
  }
  RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "Armor jump!");

  auto p = current_armor.pose.position;
  Eigen::Vector3d current_p(p.x, p.y, p.z);
  Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);
  if ((current_p - infer_p).norm() > max_match_distance_) {
    // double r = target_state(8);
    // target_state(0) = p.x + r * cos(yaw);  // xc
    target_state(1) = 0;                   // vxc
    // target_state(2) = p.y + r * sin(yaw);  // yc
    target_state(3) = 0;                   // vyc
    target_state(5) = 0;                   // vza
    RCLCPP_ERROR(rclcpp::get_logger("armor_processor"), "State wrong!");
  }

  ekf.setState(target_state);
}

double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion & q)
{
  // Get armor yaw
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  // Make yaw change continuous
  yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
  last_yaw_ = yaw;
  return yaw;
}

Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd & x)
{
  // Calculate predicted position of the current armor
  double xc = x(0), yc = x(2), za = x(4);
  double yaw = x(6), r = x(8);
  double xa = xc - r * cos(yaw);
  double ya = yc - r * sin(yaw);
  return Eigen::Vector3d(xa, ya, za);
}

}  // namespace rm_auto_aim
