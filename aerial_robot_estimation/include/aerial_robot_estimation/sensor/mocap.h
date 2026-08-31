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
#include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>
#include <kalman_filter/kf_pos_vel_acc_plugin.h>
#include <geometry_msgs/msg/pose_stamped.h>

/* Aerial robot packages */
#include "aerial_robot_estimation/sensor/base_plugin.h"
#include "aerial_robot_estimation/sensor/imu.h"
#include "aerial_robot_msgs/msg/states.hpp"


namespace sensor_plugin
{
class Mocap : public sensor_plugin::SensorBase
{
public:
  Mocap();
  ~Mocap() {}

  virtual void initialize(rclcpp::Node::SharedPtr node, std::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                          std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator, std::string sensor_name,
                          int index) override;

protected:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr msg_sub_;
  rclcpp::Publisher<aerial_robot_msgs::msg::States>::SharedPtr state_pub_;

  KDL::Frame raw_pose_, pose_, prev_raw_pose_;
  KDL::Twist raw_twist_, twist_, prev_raw_twist_;

  IirFilter lpf_pos_, lpf_vel_, lpf_omega_;
  /* ROS param */
  double sample_freq_, cutoff_pos_freq_, cutoff_vel_freq_;
  double pos_noise_sigma_, vel_noise_sigma_, attitude_noise_sigma_, acc_bias_noise_sigma_;

  aerial_robot_msgs::msg::States states_; /* for debug */

  void activateFuser() override;
  void estimateProcess() override;

  void preProcessState() override;
  void fuse() override;
  void setState() override;

  void publish() override;
  void publishExternalMeasurement();
  void rosParamInit() override;
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};
}
