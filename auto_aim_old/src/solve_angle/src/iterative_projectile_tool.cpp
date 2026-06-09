// Copyright 2021 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rmoss_projectile_motion/iterative_projectile_tool.hpp"

#include <cmath>
#include<iostream>

const double GRAVITY = 9.7915;


namespace rmoss_projectile_motion
{

bool IterativeProjectileTool::solve(double target_x, double target_h, double & angle,double vel)
{
  // std::cout<<"target_x"<<target_x<<"target_h"<<target_h<<std::endl;
  double aimed_h, h;
  double dh = 0;
  double tmp_angle = 0;
  double t = 0;
  aimed_h = target_h;


  tmp_angle = atan2(aimed_h, target_x);
  angle = tmp_angle;
  std::cout<<"angle  "<<angle<<std::endl;

  // for (int i = 0; i < max_iter_; i++) {
  //   //tmp_angle = atan2(aimed_h, target_x);
    

  //   if (tmp_angle > 80 * M_PI / 180 || tmp_angle < -80 * M_PI / 180) {
  //     std::cout<<"iterative angle is out of range(-80d,80d)";
  //     return false;
  //   }
  //   forward_motion_func_(tmp_angle, target_x, h, t);
  //   if (t > 10) {
  //     std::cout<<"motion time(" + std::to_string(t) + ") is too long";
  //     return false;
  //   }
  //   dh = target_h - h;
  //   aimed_h = aimed_h + dh;
  //   if (fabs(dh) < 0.001) {
  //     break;
  //   }

  //   double initial_vel_ = target_x / (t * cos(tmp_angle));//飞行时间
  //   tmp_angle -= dh / (-(initial_vel_ * t) / pow(cos(tmp_angle), 2) +
  //         GRAVITY * t * t / (initial_vel_ * initial_vel_) * sin(tmp_angle) / pow(cos(tmp_angle), 3));

  // }
  // if (fabs(dh) > 0.01) {
  //   error_message_ = "height error(" + std::to_string(dh) + ") is too large";
  //   return false;
  // }
  // // std::cout<<"tmp_angle"<<tmp_angle<<std::endl;
  // angle = tmp_angle;

  double k1 = 0.47 * 1.169 * (M_PI * 0.02125 * 0.02125) / 2 /0.045;/// 0.055;  //0.041;

  for(auto i =1;i<100;i++)
  {
          
      // std::cout<<"given_angle"<<given_angle<<"initial_vel_"<<initial_vel_<<std::endl;
      t = (pow(2.718281828, k1 * target_x) - 1) / (k1 * vel * cos(tmp_angle));
      dh = aimed_h - vel * sin(tmp_angle) * t / cos(tmp_angle) 
                + 0.5 * GRAVITY * t * t / cos(tmp_angle) / cos(tmp_angle);
      if(dh<0.000001)break;
      tmp_angle -=dh / (-(vel * t) / pow(cos(tmp_angle), 2) +
          GRAVITY * t * t / (vel * vel) * sin(tmp_angle) / pow(cos(tmp_angle), 3));
  }
  std::cout<<"tmp_angle "<<tmp_angle<<std::endl;
  angle = tmp_angle;
  return true;
}

}  // namespace rmoss_projectile_motion