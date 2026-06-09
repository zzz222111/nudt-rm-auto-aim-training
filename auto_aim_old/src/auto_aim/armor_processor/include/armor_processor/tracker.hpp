// Copyright 2022 Chen Jun

/**
 * @file tracker.hpp
 * @brief 高层跟踪器：选择目标装甲并使用扩展卡尔曼滤波维护状态。
 *
 * `Tracker` 从接收的 `jlcv_interfaces::msg::Armors` 中选择要跟踪的装甲，
 * 初始化 EKF，将后续检测与预测状态匹配，处理偶发的身份跳变，
 * 并公开融合后的 `target_state` 与 `measurement` 以供发布。
 */

#ifndef ARMOR_PROCESSOR__TRACKER_HPP_
#define ARMOR_PROCESSOR__TRACKER_HPP_

// Eigen
#include <Eigen/Eigen>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// STD
#include <memory>
#include <string>

#include "armor_processor/extended_kalman_filter.hpp"
#include "jlcv_interfaces/msg/armors.hpp"
#include "jlcv_interfaces/msg/target.hpp"

namespace rm_auto_aim
{
class Tracker
{
enum class ArmorsNum { NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3 };

public:
  Tracker(double max_match_distance, double max_match_yaw_diff);
  
  using Armors = jlcv_interfaces::msg::Armors;
  using Armor = jlcv_interfaces::msg::Armor;

  // void init(const Armors::SharedPtr & armors_msg);

  /**
   * @brief Initialize internal tracker from the first Armors message.
   * @param armors_msg Pointer to incoming Armors message
   */
  void init(const Armors::SharedPtr & armors_msg);

  /**
   * @brief Update tracker state using a new Armors message.
   * @param armors_msg Shared pointer to Armors message
   */
  void update(const Armors::SharedPtr & armors_msg);

  ExtendedKalmanFilter ekf;

  int tracking_thres;  // frame
  double lost_thres;   // second

  enum State {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
  } tracker_state;


  Armor tracked_armor;
  std::string tracked_id;
  ArmorsNum tracked_armors_num;

  double info_position_diff;
  double info_yaw_diff;

  Eigen::VectorXd measurement;
  Eigen::VectorXd target_state;
  int task_mode;

  // To store another pair of armors message
  // double last_z, last_r;
  double dz, another_r;

private:
  void initEKF(const Armor & a);

  void updateArmorsNum(const Armor & a);

  void handleArmorJump(const Armor & a);

  double orientationToYaw(const geometry_msgs::msg::Quaternion & q);

  Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x);

  double max_match_distance_;
  double max_match_yaw_diff_;

  // int tracking_threshold_;
  // int lost_threshold_;

  int detect_count_;
  int lost_count_;

  double last_yaw_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_