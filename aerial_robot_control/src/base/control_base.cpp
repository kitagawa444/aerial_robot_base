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
#include "aerial_robot_control/base/control_base.hpp"

namespace aerial_robot_control
{

void ControlBase::initialize(rclcpp::Node::SharedPtr node, std::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                             std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                             std::shared_ptr<aerial_robot_navigation::NavigationBase> navigator, double ctrl_loop_dt)
{
  node_ = node;
  motor_info_pub_ = node_->create_publisher<spinal_msgs::msg::PwmInfo>("motor_info", 10);
  uav_info_pub_ = node_->create_publisher<spinal_msgs::msg::UavInfo>("uav_info", 10);

  robot_model_ = robot_model;
  estimator_ = estimator;
  navigator_ = navigator;
  ctrl_loop_dt_ = ctrl_loop_dt;

  motor_num_ = robot_model->getRotorNum();
  estimate_mode_ = estimator_->getEstimateMode();
  getParam<int>("controller.uav_model", uav_model_, 0);

  // Parameters for motor control
  getParam<double>("motor_info.max_pwm", max_pwm_, 0.0);
  getParam<double>("motor_info.min_pwm", min_pwm_, 0.0);
  getParam<double>("motor_info.min_thrust", min_thrust_, 0.0);
  getParam<double>("motor_info.force_landing_thrust", force_landing_thrust_, 0.0);
  getParam<double>("motor_info.m_f_rate", m_f_rate_, 0.0);
  getParam<int>("motor_info.vel_ref_num", vel_ref_num, 0);  // TOOD what is vel_ref_num? Better naming!
  getParam<int>("motor_info.pwm_conversion_mode", pwm_conversion_mode_, -1);
  getParam<bool>("controller.param_verbose", param_verbose_, false);

  motor_info_.resize(vel_ref_num);

  for (int i = 0; i < vel_ref_num; i++)
  {
    const std::string ref_prefix = "motor_info.ref" + std::to_string(i + 1);
    double val = 0.0;

    getParam<double>(ref_prefix + ".voltage", val, 0.0);
    motor_info_[i].voltage = val;

    getParam<double>(ref_prefix + ".max_thrust", val, 0.0);
    motor_info_[i].max_thrust = val;

    // Hardcode: up to 5 polynomial coefficients (indices 04)
    for (int j = 0; j < 5; j++)
    {
      getParam<double>(ref_prefix + ".polynominal" + std::to_string(j), val, 0.0);
      motor_info_[i].polynominal[j] = val;
    }
  }
}

bool ControlBase::update()
{
  // Keep spinal configured while disarmed so a direct RC ARM does not depend
  // on a preceding ROS arm request.  activate() rate-limits these static
  // motor/UAV information messages to approximately 10 Hz.
  if (navigator_->getNaviState() == aerial_robot_navigation::ARM_OFF_STATE ||
      navigator_->getNaviState() == aerial_robot_navigation::START_STATE)
  {
    activate();
  }

  if (navigator_->getNaviState() == aerial_robot_navigation::ARM_OFF_STATE && control_timestamp_ > 0)
  {
    reset();
  }

  if (control_timestamp_ < 0)
  {
    if (navigator_->getNaviState() == aerial_robot_navigation::TAKEOFF_STATE)
    {
      reset();
      control_timestamp_ = node_->now().seconds();
    }
    else
    {
      return false;
    }
  }

  return true;
}

void ControlBase::activate()
{
  if (node_->now().seconds() - activate_timestamp_ > 0.1)
  {
    // Send motor and UAV info to UAV at ~10 Hz
    spinal_msgs::msg::PwmInfo motor_info_msg;
    motor_info_msg.max_pwm = max_pwm_;
    motor_info_msg.min_pwm = min_pwm_;
    motor_info_msg.min_thrust = min_thrust_;
    motor_info_msg.force_landing_thrust = force_landing_thrust_;
    motor_info_msg.pwm_conversion_mode = pwm_conversion_mode_;
    motor_info_msg.motor_info.resize(0);
    for (size_t i = 0; i < motor_info_.size(); i++) motor_info_msg.motor_info.push_back(motor_info_[i]);
    motor_info_pub_->publish(motor_info_msg);

    spinal_msgs::msg::UavInfo uav_info_msg;
    uav_info_msg.motor_num = motor_num_;
    uav_info_msg.uav_model = uav_model_;
    uav_info_pub_->publish(uav_info_msg);

    activate_timestamp_ = node_->now().seconds();
  }

  reset();
}

}
