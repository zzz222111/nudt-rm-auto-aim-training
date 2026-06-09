// Copyright (c) 2023 ChenJun
// Licensed under the Apache-2.0 License.

#include <angles/angles.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>

#include "solve_angle/solve_angle.hpp"

namespace solve_angle {
  SolveAngle::SolveAngle(
    const rclcpp::NodeOptions &options)
    : Node("solve_angle", options), solver_(BULLET_SPEED) {
      
  RCLCPP_INFO(get_logger(), "Start SolveAngle!");
  //std::cout<<"BULLET_SPEED"<<BULLET_SPEED<<std::endl;
  target_sub_ = create_subscription<jlcv_interfaces::msg::Target>(
    "/processor/target", rclcpp::SensorDataQoS(),
    [this](jlcv_interfaces::msg::Target::SharedPtr msg) {
      target_ = msg;
      ////std::cout << "detect target" << std::endl;
    });

  // gimbal_state_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
  //   "/tf", 1,
  //   [this](tf2_msgs::msg::TFMessage::SharedPtr msg) {
  //     auto q = msg->transforms[0].transform.rotation;
  //     tf2::Quaternion q_tf(q.x, q.y, q.z, q.w);
  //     tf2::Matrix3x3 m(q_tf);
  //     double roll, pitch, yaw;
  //     m.getRPY(roll, pitch, yaw);
  //     gimbal_pitch_ = pitch;
  //     gimbal_yaw_ = yaw;
  //   });


  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  target_frame_ = this->declare_parameter("target_frame", "world");


  gimbal_command_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
    "/gimbal_command", rclcpp::SensorDataQoS());

  aiming_point_.header.frame_id = "world";
  aiming_point_.ns = "aiming_point";
  aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
  aiming_point_.action = visualization_msgs::msg::Marker::ADD;
  aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
  aiming_point_.color.r = 1.0;
  aiming_point_.color.g = 1.0;
  aiming_point_.color.a = 1.0;
  aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);
  aiming_point_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/aiming_point", rclcpp::SensorDataQoS());

  auto publish_command = [this]() -> void {
    if (target_) {
      camera_id = target_->camera_id;
      if (target_->tracking) {
        try {
          geometry_msgs::msg::TransformStamped transform = tf2_buffer_->lookupTransform(
            target_frame_,"pitch_link",  tf2::TimePointZero);

          // 构建4×4矩阵
          tf2::Quaternion tf_q  = tf2::Quaternion(transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w);
          double roll, pitch, yaw;
          tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
          gimbal_yaw_ = yaw;      
          gimbal_pitch_ = pitch;
        } catch (const tf2::TransformException & ex) {
          RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        }


        xc = target_->position.x;
        yc = target_->position.y;
        z = target_->position.z;
        armor_yaw = target_->yaw;

        latency = (this->now() - target_->header.stamp).seconds();
        distance = sqrt(xc*xc+yc*yc)-target_->radius_1;
        horizontal_speed = BULLET_SPEED * cos(gimbal_pitch_);
        fly_time = distance / horizontal_speed + latency;
        // fly_time = 0; //?
        // pre_xc = xc + target_->velocity.x * fly_time;
        // pre_yc = yc + target_->velocity.y * fly_time;
        // double r = target_->radius_1;
        r = target_->radius_1;


        if(target_->id == "outpost" )
        {
          fly_time = distance / horizontal_speed + latency + TIME_OFFSET;
          final_x=xc ;
          final_y=yc ;
          final_z=z;

          // std::cout<<"v_yaw:"<<target_->yaw<<std::endl;
          if(target_->v_yaw>1)
            target_->v_yaw = 2.512;
          else if(target_->v_yaw<-1)
            target_->v_yaw = -2.512;
          else target_->v_yaw = 0;
          pre_yaw=armor_yaw+target_->v_yaw*fly_time; //1.047*(target_->v_yaw>0 ? 1:-1);
        }
        else{
          final_x=xc + target_->velocity.x * fly_time;
          final_y=yc + target_->velocity.y * fly_time;
          final_z=z;
          pre_yaw=armor_yaw+target_->v_yaw*fly_time;
        }
        control_status = 1;
        
        a_n = target_->armors_num;
        center_yaw = atan2(yc, xc);
        allow_yaw = 0.01;  //0.0065;  
        // double min_yaw_diff = M_PI;

        for (size_t i = 1; i < a_n; i++) {
          tmp_yaw = pre_yaw + i * (2 * M_PI / a_n) ;
          yaw_diff =angles::shortest_angular_distance(tmp_yaw, center_yaw);
          // std::cout<<"yaw_diff"<<yaw_diff<<std::endl;
          //std::cout<<"tmp_yaw"<<tmp_yaw<<"center_yaw"<<center_yaw<<std::endl;

          // if(min_yaw_diff>abs(yaw_diff))
          // {
          //   min_yaw_diff = abs(yaw_diff);
          //   aiming_point_.pose.position.x = x_;
          //   aiming_point_.pose.position.y = y_;
          //   aiming_point_.pose.position.z = final_z;
          // }
          
          if(abs(yaw_diff)<allow_yaw && abs(angles::shortest_angular_distance(gimbal_command_yaw_+YAW_OFFSET, gimbal_yaw_))<0.01){ //0.002
          //   if(target_->v_yaw>0){
          //     if(tmp_yaw>center_yaw){
          //       control_status=2;
          //       break;
          //     }
          //   }
          //   else{
          //     if(tmp_yaw<center_yaw){
          //       control_status=2;
          //       break;
          //     }
          //   }
            control_status = 2;
            break;
          }
          // if(target_->id == "outpost" && i>=1)break;
        }

        distance = sqrt(xc*xc+yc*yc);
        //std::cout<<"distance"<<abs(last_distance-distance)/distance<<std::endl;
        // if(abs(last_distance-distance)/distance>0.15)min_diff_all = 3.1415;
        // if(min_yaw_diff<min_diff_all)min_diff_all = min_yaw_diff;
        // last_distance = distance;
        

        //std::cout<<"yaw_diff"<<min_yaw_diff<<"min_diff_all"<<min_diff_all<<std::endl;
        v_yaw = target_->v_yaw;
        // if(target_->id == "outpost"){
        //   if(v_yaw > 0){v_yaw = 2.513;}
        //   if(v_yaw < 0){v_yaw = -2.513;}
        // }
        if(abs(v_yaw)<0.1&&abs(target_->velocity.x)<0.2&&abs(target_->velocity.y)<0.2){
          final_x = xc - r * cos(pre_yaw);
          final_y = yc - r * sin(pre_yaw);
          final_z = z;
          control_status = 2;
        }

        if (control_status != 0) {
          distance = sqrt(final_x*final_x+final_y*final_y);
          solver_.solve(distance, final_z, pitch, BULLET_SPEED);
          gimbal_command_pitch_ = -pitch;
          gimbal_command_yaw_ = atan2(final_y, final_x);

          // Publish aiming point
          aiming_point_.header.stamp = target_->header.stamp;
          aiming_point_.color.g = control_status == 2 ? 1.0 : 0.0;
          aiming_point_pub_->publish(aiming_point_);
        }
      } else {
      control_status = 0;
      }

      // Publish gimbal command
      geometry_msgs::msg::Vector3 gimbal_command;
      if(camera_id==0){
        gimbal_command.x = gimbal_command_pitch_ + PITCH_OFFSET;
        gimbal_command.y = gimbal_command_yaw_ + YAW_OFFSET;
      }
      else{
        gimbal_command.x = gimbal_command_pitch_ + PITCH_OFFSET1;
        gimbal_command.y = gimbal_command_yaw_ + YAW_OFFSET1;
      } 

      // gimbal_command.x = gimbal_command_pitch_ + PITCH_OFFSET;
      // gimbal_command.y = gimbal_command_yaw_ + YAW_OFFSET;

      gimbal_command.z = control_status;
      gimbal_command_pub_->publish(gimbal_command);
      //std::cout<<"shortest_distance:"<<angles::shortest_angular_distance(gimbal_command_yaw_+YAW_OFFSET, gimbal_yaw_)<<std::endl;
    }
  };
  RCLCPP_INFO(get_logger(), "Start SolveAngle timer!");
  timer_ = create_wall_timer(std::chrono::milliseconds(1), publish_command);
}

} // namespace solve_angle

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(
    solve_angle::SolveAngle)