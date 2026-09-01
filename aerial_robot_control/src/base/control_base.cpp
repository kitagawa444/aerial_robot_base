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

#include <algorithm>

namespace aerial_robot_control
{

void ControlBase::initialize(rclcpp::Node::SharedPtr node, std::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                             std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                             std::shared_ptr<aerial_robot_navigation::NavigationBase> navigator, double ctrl_loop_dt)
{
  node_ = node;
  motor_info_pub_ = node_->create_publisher<spinal_msgs::msg::PwmInfo>("motor_info", 10);
  uav_info_pub_ = node_->create_publisher<spinal_msgs::msg::UavInfo>("uav_info", 10);
  position_control_config_pub_ = node_->create_publisher<spinal_msgs::msg::PositionControlConfig>(
      "position_control/config", 1);
  health_config_pub_ = node_->create_publisher<spinal_msgs::msg::HealthConfig>("health/config", 1);
  position_control_setpoint_pub_ = node_->create_publisher<spinal_msgs::msg::PositionControlSetpoint>(
      "position_control/setpoint", 1);
  std::string position_control_service_name;
  getParam<std::string>("controller.position_control.service_name", position_control_service_name,
                        "set_position_control");
  position_control_client_ = node_->create_client<std_srvs::srv::SetBool>(position_control_service_name);

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
  getParam<int>("motor_info.vel_ref_num", vel_ref_num,
                0);  // TOOD what is vel_ref_num? Better naming!
  getParam<int>("motor_info.pwm_conversion_mode", pwm_conversion_mode_, -1);
  getParam<bool>("controller.param_verbose", param_verbose_, false);
  getParam<bool>("controller.spinal_position_control", spinal_position_control_, false);

  std::vector<double> position_p{ 1.5, 1.5, 2.0 };
  std::vector<double> position_i{ 0.0, 0.0, 0.2 };
  std::vector<double> velocity_d{ 1.0, 1.0, 1.2 };
  std::vector<double> integral_limit{ 2.0, 2.0, 2.0 };
  getParam<std::vector<double>>("controller.position_control.position_p", position_p, position_p);
  getParam<std::vector<double>>("controller.position_control.position_i", position_i, position_i);
  getParam<std::vector<double>>("controller.position_control.velocity_d", velocity_d, velocity_d);
  getParam<std::vector<double>>("controller.position_control.integral_limit", integral_limit, integral_limit);
  for (size_t axis = 0; axis < 3; ++axis)
  {
    if (position_p.size() > axis) position_control_config_.position_p[axis] = position_p[axis];
    if (position_i.size() > axis) position_control_config_.position_i[axis] = position_i[axis];
    if (velocity_d.size() > axis) position_control_config_.velocity_d[axis] = velocity_d[axis];
    if (integral_limit.size() > axis) position_control_config_.integral_limit[axis] = integral_limit[axis];
  }
  const auto get_float_param = [this](const std::string &name, float &destination, double default_value)
  {
    double value = default_value;
    getParam<double>(name, value, default_value);
    destination = static_cast<float>(value);
  };
  get_float_param("controller.position_control.yaw_p", position_control_config_.yaw_p, 0.5);
  get_float_param("controller.position_control.yaw_i", position_control_config_.yaw_i, 0.05);
  get_float_param("controller.position_control.max_horizontal_acceleration",
                  position_control_config_.max_horizontal_acceleration, 4.0);
  get_float_param("controller.position_control.max_vertical_acceleration",
                  position_control_config_.max_vertical_acceleration, 5.0);
  get_float_param("controller.position_control.max_tilt_angle", position_control_config_.max_tilt_angle, 0.7);
  int setpoint_timeout_ms = 500;
  getParam<int>("controller.position_control.setpoint_timeout_ms", setpoint_timeout_ms, 500);
  position_control_config_.setpoint_timeout_ms = static_cast<uint32_t>(std::max(setpoint_timeout_ms, 1));
  get_float_param("navigation.max_teleop_xy_vel", position_control_config_.rc_max_horizontal_velocity, 0.5);
  get_float_param("navigation.max_teleop_z_vel", position_control_config_.rc_max_vertical_velocity, 0.5);
  get_float_param("navigation.max_teleop_yaw_vel", position_control_config_.rc_max_yaw_rate, 0.1);
  get_float_param("navigation.joy_stick_deadzone", position_control_config_.rc_deadzone, 0.2);
  int rc_timeout_ms = 100;
  getParam<int>("controller.position_control.rc_timeout_ms", rc_timeout_ms, 100);
  position_control_config_.rc_timeout_ms = static_cast<uint32_t>(std::max(rc_timeout_ms, 1));
  get_float_param("navigation.takeoff_height", position_control_config_.takeoff_height, 1.0);
  get_float_param("navigation.z_convergent_thresh", position_control_config_.takeoff_position_tolerance, 0.05);
  get_float_param("navigation.land_descend_vel", position_control_config_.landing_speed, -0.3);
  get_float_param("navigation.land_pos_convergent_thresh", position_control_config_.landed_height, 0.02);
  get_float_param("navigation.land_vel_convergent_thresh", position_control_config_.landed_velocity, 0.05);
  get_float_param("controller.position_control.takeoff_velocity_tolerance",
                  position_control_config_.takeoff_velocity_tolerance, 0.2);
  double takeoff_stable_time = 1.0;
  getParam<double>("navigation.hover_convergent_duration", takeoff_stable_time, 1.0);
  position_control_config_.takeoff_stable_time_ms = static_cast<uint32_t>(std::max(takeoff_stable_time * 1000.0, 1.0));
  double landed_stable_time = 0.5;
  getParam<double>("navigation.land_check_duration", landed_stable_time, 0.5);
  position_control_config_.landed_stable_time_ms = static_cast<uint32_t>(std::max(landed_stable_time * 1000.0, 1.0));
  int rc_authority_timeout_ms = 500;
  getParam<int>("controller.position_control.rc_authority_timeout_ms", rc_authority_timeout_ms, 500);
  position_control_config_.rc_authority_timeout_ms = static_cast<uint32_t>(std::max(rc_authority_timeout_ms, 1));

  int battery_cell_count = 0;
  getParam<int>("bat_info.bat_cell", battery_cell_count, 0);
  health_config_.battery_cell_count = static_cast<uint8_t>(std::max(0, std::min(battery_cell_count, 255)));
  get_float_param("bat_info.low_voltage_thre", health_config_.battery_low_percentage, 6.0);
  get_float_param("bat_info.low_voltage_hysteresis", health_config_.battery_hysteresis_percentage, 2.0);
  get_float_param("bat_info.high_voltage_cell_thre", health_config_.battery_high_cell_threshold, 1.0);
  get_float_param("bat_info.bat_resistance", health_config_.battery_resistance, 0.0);
  get_float_param("bat_info.bat_resistance_voltage_rate", health_config_.battery_resistance_voltage_rate, 0.0);
  get_float_param("bat_info.hovering_current", health_config_.battery_hovering_current, 0.0);
  int battery_debounce_ms = 1000;
  getParam<int>("bat_info.low_voltage_debounce_ms", battery_debounce_ms, 1000);
  health_config_.battery_debounce_ms = static_cast<uint32_t>(std::max(battery_debounce_ms, 1));
  int primary_imu_timeout_ms = 100;
  getParam<int>("health.primary_imu_timeout_ms", primary_imu_timeout_ms, 100);
  health_config_.primary_imu_timeout_ms = static_cast<uint32_t>(std::max(primary_imu_timeout_ms, 1));
  int control_loop_deadline_ms = 20;
  getParam<int>("health.control_loop_deadline_ms", control_loop_deadline_ms, 20);
  health_config_.control_loop_deadline_ms = static_cast<uint32_t>(std::max(control_loop_deadline_ms, 1));
  int control_loop_miss_limit = 3;
  getParam<int>("health.control_loop_miss_limit", control_loop_miss_limit, 3);
  health_config_.control_loop_miss_limit = static_cast<uint8_t>(std::max(1, std::min(control_loop_miss_limit, 255)));
  if (motor_num_ > 0)
  {
    const float acceleration_to_thrust = static_cast<float>(robot_model_->getMass() / static_cast<double>(motor_num_));
    position_control_config_.vertical_acceleration_to_thrust.assign(motor_num_, acceleration_to_thrust);
  }

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

  if (spinal_position_control_)
  {
    requestPositionControlMode();
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
    health_config_pub_->publish(health_config_);

    if (spinal_position_control_)
    {
      position_control_config_pub_->publish(position_control_config_);
      requestPositionControlMode();
    }

    activate_timestamp_ = node_->now().seconds();
  }

  reset();
}

void ControlBase::publishPositionControlSetpoint()
{
  if (!position_control_setpoint_pub_ || !navigator_) return;
  spinal_msgs::msg::PositionControlSetpoint msg;
  msg.stamp = node_->now();
  const KDL::Vector position = navigator_->getTargetCogPos();
  const KDL::Vector velocity = navigator_->getTargetCogVel();
  const KDL::Vector acceleration = navigator_->getTargetCogAcc();
  const KDL::Vector rpy = navigator_->getTargetCogRPY();
  const KDL::Vector angular_velocity = navigator_->getTargetCogOmega();
  const KDL::Vector angular_acceleration = navigator_->getTargetCogAngularAcc();
  for (size_t axis = 0; axis < 3; ++axis)
  {
    msg.position[axis] = position[axis];
    msg.velocity[axis] = velocity[axis];
    msg.acceleration[axis] = acceleration[axis];
  }
  msg.yaw = rpy.z();
  msg.yaw_rate = angular_velocity.z();
  msg.yaw_acceleration = angular_acceleration.z();
  msg.initial_height = navigator_->getInitHeight();
  msg.horizontal_control_mode = navigator_->getXyControlMode();
  // Basic takeoff and landing setpoints are generated by the FC supervisor.
  // ROS setpoints become authoritative only after the FC reports AIRBORNE.
  msg.active = navigator_->getNaviState() == aerial_robot_navigation::HOVER_STATE;
  msg.manual_control_allowed = navigator_->getNaviState() == aerial_robot_navigation::HOVER_STATE;
  msg.landing = navigator_->getNaviState() == aerial_robot_navigation::LAND_STATE;
  position_control_setpoint_pub_->publish(msg);
}

void ControlBase::requestPositionControlMode()
{
  if (!spinal_position_control_ || position_control_enabled_on_spinal_ || !position_control_client_)
  {
    return;
  }

  const double now = node_->now().seconds();
  if (position_control_request_pending_)
  {
    // The simulation controller can receive this request while it is still
    // transitioning to ACTIVE. In that case rmw may drop the response, so do
    // not leave the client permanently stuck in the pending state.
    if (position_control_request_stamp_ >= 0.0 && now - position_control_request_stamp_ < 1.0) return;
    position_control_request_pending_ = false;
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "Position-control mode request timed out; retrying");
  }
  if (!position_control_client_->service_is_ready()) return;

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  position_control_request_pending_ = true;
  position_control_request_stamp_ = now;
  position_control_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response)
      {
        position_control_request_pending_ = false;
        position_control_request_stamp_ = -1.0;
        position_control_enabled_on_spinal_ = response.get()->success;
        if (!position_control_enabled_on_spinal_)
        {
          RCLCPP_WARN(node_->get_logger(), "Spinal rejected position control mode; it must be disarmed");
        }
      });
}

}  // namespace aerial_robot_control
