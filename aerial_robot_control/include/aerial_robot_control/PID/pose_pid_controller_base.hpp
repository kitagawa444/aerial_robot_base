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
#include <angles/angles.h>
#include <kdl/frames.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

/* Aerial robot packages */
#include "aerial_robot_control/PID/pid.hpp"
#include "aerial_robot_control/base/control_base.hpp"
#include "aerial_robot_msgs/msg/pose_control_pid.hpp"
#include "spinal_msgs/msg/flight_config_cmd.hpp"

namespace aerial_robot_control
{

enum
{
  X,
  Y,
  Z,
  ROLL,
  PITCH,
  YAW,
};

class PosePIDControllerBase : public ControlBase
{
public:
  PosePIDControllerBase();
  virtual ~PosePIDControllerBase() = default;
  virtual void initialize(rclcpp::Node::SharedPtr node, std::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                          std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                          std::shared_ptr<aerial_robot_navigation::NavigationBase> navigator,
                          double ctrl_loop_dt) override;

  bool update();
  virtual void reset() override;

protected:
  rclcpp::Publisher<aerial_robot_msgs::msg::PoseControlPid>::SharedPtr pid_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  aerial_robot_msgs::msg::PoseControlPid pid_msg_;

  std::vector<PID> pid_controllers_;

  double landing_err_z_;
  double safe_landing_height_;
  double force_landing_descending_rate_;

  bool need_yaw_d_control_;
  bool start_roll_pitch_integration_;
  double start_roll_pitch_integration_height_;  // Start integration inside PID
                                                // controller of roll and pitch at
                                                // [this height +
                                                // estimator_->getLandingHeight()]

  KDL::Vector pos_, target_pos_;
  KDL::Vector rpy_, target_rpy_;
  KDL::Rotation cog_rot_, target_rot_;
  KDL::Vector vel_, target_vel_;
  KDL::Vector omega_, target_omega_;
  KDL::Vector target_acc_, target_ang_acc_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
  virtual void controlCore();
  virtual void sendCmd();
  void updateSpinalPositionPidConfig();
  Eigen::MatrixXd getQInv();
};
}  // namespace aerial_robot_control
