// -*- mode: c++ -*-
/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *   3. Neither the name of the DRAGON Laboratory nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

/* ROS 2 */
#include <tf2_ros/static_transform_broadcaster.h>
#include <nav_msgs/msg/odometry.h>

/* Kalman Filter library */
#include "kalman_filter/kf_pos_vel_acc_plugin.h"

/* Aerial robot packages */
#include "aerial_robot_estimation/sensor/base_plugin.h"

namespace sensor_plugin
{
enum
{
  ONLY_POS_MODE = 0,
  ONLY_VEL_MODE = 1,
  POS_VEL_MODE = 2,
};

class Odometry : public sensor_plugin::SensorBase
{
public:
  Odometry();
  ~Odometry() {}

  void initialize(rclcpp::Node::SharedPtr node, std::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator, std::string sensor_name,
                  int index) override;


  const bool odomPosMode();

  const KDL::Frame &getBasePose() const { return base_pose_; }

private:
  /* ROS */
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> br_;

  /* Reconfigurable varaible */
  double throttle_rate_;
  double level_pos_noise_sigma_;
  double z_pos_noise_sigma_;
  double vel_noise_sigma_;
  double vel_outlier_thresh_;
  double attitude_noise_sigma_;
  int fusion_mode_;
  bool local_vel_mode_;

  KDL::Frame origin_offset_;  // ^{w}H_{o}: transform between two origins
  KDL::Frame base_pose_;      // ^{w}H_{b} estimated by odometry sensor
  KDL::Twist base_twist_;
  KDL::Frame sensor_pose_, prev_sensor_pose_;
  KDL::Twist sensor_twist_, prev_sensor_twist_;

  std::string odom_origin_frame_;

  double ref_time_stamp_;
  bool external_measurement_ready_{ false };
  aerial_robot_msgs::msg::States states_; /* for debug */

  bool checkStatus();
  bool calcuateOriginOffset();
  void calculateBasePose();
  void calculateBaseVelocity();
  bool isMsgNan(nav_msgs::msg::Odometry msg);
  void activateFuser() override;
  void estimateProcess() override;

  void preProcessState() override;
  void fuse() override;
  void print();

  void publish() override;
  void publishExternalMeasurement();
  void tfBroadcast();
  void rosParamInit() override;
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr vo_msg);
};
}
