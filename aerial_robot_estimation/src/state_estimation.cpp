// -*- Mode: c++ -*-
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
#include "aerial_robot_estimation/sensor/base_plugin.h"
#include "aerial_robot_estimation/state_estimation.h"

#include <cmath>

using namespace aerial_robot_estimation;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("state_estimation");

StateEstimator::StateEstimator()
  : sensor_fusion_flag_(false),
    qu_size_(0),
    unhealth_level_(0),
    prev_pub_stamp_(0),
    flying_flag_(false),
    landing_mode_flag_(false),
    landed_flag_(false),
    landing_height_(0),
    un_descend_flag_(false),
    force_att_control_flag_(false),
    has_groundtruth_odom_(false),
    imu_handlers_(0),
    alt_handlers_(0),
    vo_handlers_(0),
    gps_handlers_(0)
{
  has_refined_yaw_estimate_[EGOMOTION_ESTIMATE] = false;
  has_refined_yaw_estimate_[EXPERIMENT_ESTIMATE] = false;

  for (size_t i = 0; i < ESTIMATE_MODE_COUNT; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      base_pos_status_matrix_.at(i).at(j) = 0;
      cog_pos_status_matrix_.at(i).at(j) = 0;
    }
    base_rot_status_.at(i) = 0;
    cog_rot_status_.at(i) = 0;

    base_pose_.at(i) = KDL::Frame::Identity();
    cog_pose_.at(i) = KDL::Frame::Identity();
    base_twist_.at(i) = KDL::Twist::Zero();
    cog_twist_.at(i) = KDL::Twist::Zero();
    base_acc_.at(i) = KDL::Vector::Zero();
    cog_acc_.at(i) = KDL::Vector::Zero();
  }
}

void StateEstimator::initialize(rclcpp::Node::SharedPtr node,
                                std::shared_ptr<aerial_robot_model::RobotModel> robot_model)
{
  node_ = node;
  robot_model_ = robot_model;

  load();

  baselink_odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("uav/baselink/odom",
                                                                        rclcpp::SystemDefaultsQoS());
  cog_odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("uav/cog/odom", rclcpp::SystemDefaultsQoS());
  ee_contact_odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("uav/ee_contact/odom",
                                                                          rclcpp::SystemDefaultsQoS());
  external_state_pub_ = node_->create_publisher<spinal_msgs::msg::ExternalStateMeasurement>(
      "external_state_measurement", rclcpp::SystemDefaultsQoS());
  spinal_state_sub_ = node_->create_subscription<spinal_msgs::msg::StateEstimate>(
      "state_estimate", rclcpp::SensorDataQoS(),
      std::bind(&StateEstimator::spinalStateCallback, this, std::placeholders::_1));

  node_->get_parameter_or("tf_prefix", tf_prefix_, std::string(""));
  br_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  double rate;
  node_->get_parameter_or("state_pub_rate", rate, 100.0);
  auto period = std::chrono::duration<double>(1.0 / rate);
  state_process_timer_ = rclcpp::create_timer(node_, node_->get_clock(), period,
                                              std::bind(&StateEstimator::process, this));
}

void StateEstimator::load()
{
  auto pattern_match = [](std::string &pl, std::string &pl_candidate)
  {
    int cmp = fnmatch(pl.c_str(), pl_candidate.c_str(), FNM_CASEFOLD);
    if (cmp == 0) return true;

    if (cmp != FNM_NOMATCH)
    {
      RCLCPP_ERROR(LOGGER, "[estimation] Plugin list check error! fnmatch('%s', '%s', FNM_CASEFOLD) -> %d", pl.c_str(),
                   pl_candidate.c_str(), cmp);
    }

    return false;
  };

  node_->get_parameter_or("estimation.mode", requested_estimate_mode_, EGOMOTION_ESTIMATE);
  node_->get_parameter_or("estimation.spinal_state_timeout", spinal_state_timeout_, 0.5);
  if (requested_estimate_mode_ < EGOMOTION_ESTIMATE || requested_estimate_mode_ > SPINAL_MODE_GROUND_TRUTH)
  {
    RCLCPP_ERROR(LOGGER, "[estimation] Invalid mode %d. Expected an estimate mode in [0, 5].",
                 requested_estimate_mode_);
    return;
  }
  estimate_mode_ = isSpinalMode(requested_estimate_mode_) ? SPINAL_ESTIMATE : requested_estimate_mode_;

  std::string estimate_mode_str;
  if (requested_estimate_mode_ == EGOMOTION_ESTIMATE)
  {
    estimate_mode_str = "EGOMOTION_ESTIMATE";
  }
  else if (requested_estimate_mode_ == EXPERIMENT_ESTIMATE)
  {
    estimate_mode_str = "EXPERIMENT_ESTIMATE";
  }
  else if (requested_estimate_mode_ == GROUND_TRUTH)
  {
    estimate_mode_str = "GROUND_TRUTH";
  }
  else if (requested_estimate_mode_ == SPINAL_MODE_EGOMOTION)
  {
    estimate_mode_str = "SPINAL_MODE_EGOMOTION";
  }
  else if (requested_estimate_mode_ == SPINAL_MODE_EXPERIMENT)
  {
    estimate_mode_str = "SPINAL_MODE_EXPERIMENT";
  }
  else if (requested_estimate_mode_ == SPINAL_MODE_GROUND_TRUTH)
  {
    estimate_mode_str = "SPINAL_MODE_GROUND_TRUTH";
  }
  RCLCPP_INFO_STREAM(LOGGER, std::string("\033[32m estimate mode: ") << estimate_mode_str << std::string("\033[0m"));

  /* Kalman filter egomotion plugin initialization for 0: egomotion, 1: experiment */
  fusion_loader_ptr_ = std::make_shared<pluginlib::ClassLoader<kf_plugin::KalmanFilter>>("kalman_filter",
                                                                                         "kf_plugin::KalmanFilter");

  std::string fuse_prefix = "estimation.fusion.";
  for (int i = 0; i < 2; i++)
  {
    /* Kalman filter egomotion plugin list */
    std::string mode_prefix;
    if (i == EGOMOTION_ESTIMATE)
      mode_prefix = "egomotion";
    else if (i == EXPERIMENT_ESTIMATE)
      mode_prefix = "experiment";

    std::vector<std::string> fuser_list;
    if (!node_->get_parameter<std::vector<std::string>>(fuse_prefix + mode_prefix + "_list", fuser_list))
    {
      RCLCPP_ERROR_STREAM(LOGGER, "[estimation] " << fuse_prefix << mode_prefix << "_list is not set");
      return;
    }

    int cnt = 0;
    for (auto &fuser_name : fuser_list)
    {
      for (auto &name : fusion_loader_ptr_->getDeclaredClasses())
      {
        if (!pattern_match(fuser_name, name)) continue;

        std::stringstream fuser_no;
        fuser_no << ++cnt;

        int fuser_id;
        std::string fuser_id_param = fuse_prefix + mode_prefix + "_id" + fuser_no.str();
        if (!node_->get_parameter<int>(fuser_id_param, fuser_id))
        {
          RCLCPP_ERROR(LOGGER, "[estimation] %s, no param in fuser %s id", mode_prefix.c_str(), fuser_no.str().c_str());
          continue;
        }

        std::string fuser_label;
        std::string fuser_name_param = fuse_prefix + mode_prefix + "_label" + fuser_no.str();
        if (!node_->get_parameter<std::string>(fuser_name_param, fuser_label))
        {
          RCLCPP_ERROR(LOGGER, "[estimation] %s, no param in fuser %s name", mode_prefix.c_str(),
                       fuser_no.str().c_str());
          continue;
        }

        try
        {
          std::shared_ptr<kf_plugin::KalmanFilter> plugin_ptr = fusion_loader_ptr_->createSharedInstance(name);
          plugin_ptr->initialize(fuser_label, fuser_id);
          fuser_maps_.at(i).push_back(std::make_pair(name, plugin_ptr));
        }
        catch (const pluginlib::PluginlibException &ex)
        {
          RCLCPP_ERROR(LOGGER, "[estimation] Failed to load kf plugin '%s': %s", name.c_str(), ex.what());
        }
      }
    }
  }

  sensor_loader_ptr_ = std::make_shared<pluginlib::ClassLoader<sensor_plugin::SensorBase>>("aerial_robot_estimation",
                                                                                           "sensor_plugin::SensorBase");

  std::vector<std::string> sensor_list{};
  if (!node_->get_parameter<std::vector<std::string>>(fuse_prefix + "sensor_list", sensor_list))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "[estimation] " << fuse_prefix << "sensor_list is not set");
    return;
  }

  std::vector<int> sensor_index(0);

  for (auto &plugin_name : sensor_list)
  {
    for (auto &name : sensor_loader_ptr_->getDeclaredClasses())
    {
      if (!pattern_match(plugin_name, name)) continue;

      sensors_.push_back(sensor_loader_ptr_->createSharedInstance(name));
      sensor_index.push_back(1);

      if (name.find("imu") != std::string::npos)
      {
        imu_handlers_.push_back(sensors_.back());
        sensor_index.back() = imu_handlers_.size();
      }

      if (name.find("gps") != std::string::npos)
      {
        gps_handlers_.push_back(sensors_.back());
        sensor_index.back() = gps_handlers_.size();
      }

      if (name.find("alt") != std::string::npos)
      {
        alt_handlers_.push_back(sensors_.back());
        sensor_index.back() = alt_handlers_.size();
      }

      if (name.find("vo") != std::string::npos)
      {
        vo_handlers_.push_back(sensors_.back());
        sensor_index.back() = vo_handlers_.size();
      }

      sensors_.back()->initialize(node_, robot_model_, shared_from_this(), name, sensor_index.back());

      break;
    }
  }
}

int StateEstimator::getBasePosStateStatus(uint8_t axis, uint8_t estimate_mode)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  return base_pos_status_matrix_.at(estimate_mode).at(axis);
}

void StateEstimator::setBasePosStateStatus(uint8_t axis, uint8_t estimate_mode, bool status)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  int &curr_status = base_pos_status_matrix_.at(estimate_mode).at(axis);

  if (status)
  {
    curr_status++;
  }
  else
  {
    if (curr_status > 0)
    {
      curr_status--;
    }
    else
    {
      RCLCPP_WARN(LOGGER, "[estimation] wrong pos status update for axis: %d, estimate mode: %d", axis, estimate_mode);
    }
  }
}

int StateEstimator::getBaseRotStateStatus(uint8_t estimate_mode)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  return base_rot_status_.at(estimate_mode);
}

void StateEstimator::setBaseRotStateStatus(uint8_t estimate_mode, bool status)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  int &curr_status = base_rot_status_.at(estimate_mode);

  if (status)
  {
    curr_status++;
  }
  else
  {
    if (curr_status > 0)
    {
      curr_status--;
    }
    else
    {
      RCLCPP_WARN(LOGGER, "[estimation] wrong rot status update for estimate mode: %d", estimate_mode);
    }
  }
}

int StateEstimator::getCogPosStateStatus(uint8_t axis, uint8_t estimate_mode)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  return cog_pos_status_matrix_.at(estimate_mode).at(axis);
}

void StateEstimator::setCogPosStateStatus(uint8_t axis, uint8_t estimate_mode, bool status)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  int &curr_status = cog_pos_status_matrix_.at(estimate_mode).at(axis);

  if (status)
  {
    curr_status++;
  }
  else
  {
    if (curr_status > 0)
    {
      curr_status--;
    }
    else
    {
      RCLCPP_WARN(LOGGER, "[estimation] wrong pos status update for axis: %d, estimate mode: %d", axis, estimate_mode);
    }
  }
}

int StateEstimator::getCogRotStateStatus(uint8_t estimate_mode)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  return cog_rot_status_.at(estimate_mode);
}

void StateEstimator::setCogRotStateStatus(uint8_t estimate_mode, bool status)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  int &curr_status = cog_rot_status_.at(estimate_mode);

  if (status)
  {
    curr_status++;
  }
  else
  {
    if (curr_status > 0)
    {
      curr_status--;
    }
    else
    {
      RCLCPP_WARN(LOGGER, "[estimation] wrong rot status update for estimate mode: %d", estimate_mode);
    }
  }
}

const KDL::Frame StateEstimator::getBasePose(int estimate_mode)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return base_pose_.at(estimate_mode);
}

void StateEstimator::setBasePose(int estimate_mode, KDL::Frame pose)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  base_pose_.at(estimate_mode) = pose;
}

const KDL::Twist StateEstimator::getBaseTwist(int estimate_mode)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return base_twist_.at(estimate_mode);
}

void StateEstimator::setBaseTwist(int estimate_mode, KDL::Twist twist)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  base_twist_.at(estimate_mode) = twist;
}

const KDL::Vector StateEstimator::getBasePos(int estimate_mode)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return base_pose_.at(estimate_mode).p;
}

void StateEstimator::setBasePos(int estimate_mode, KDL::Vector pos)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  base_pose_.at(estimate_mode).p = pos;
}

void StateEstimator::setBasePosX(int estimate_mode, double pos)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  base_pose_.at(estimate_mode).p.x(pos);
}

void StateEstimator::setBasePosY(int estimate_mode, double pos)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  base_pose_.at(estimate_mode).p.y(pos);
}

void StateEstimator::setBasePosZ(int estimate_mode, double pos)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  base_pose_.at(estimate_mode).p.z(pos);
}

const KDL::Vector StateEstimator::getBaseVel(int estimate_mode)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return base_twist_.at(estimate_mode).vel;
}

void StateEstimator::setBaseVel(int estimate_mode, KDL::Vector vel)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  base_twist_.at(estimate_mode).vel = vel;
}

void StateEstimator::setBaseVelX(int estimate_mode, double vel)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  base_twist_.at(estimate_mode).vel.x(vel);
}

void StateEstimator::setBaseVelY(int estimate_mode, double vel)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  base_twist_.at(estimate_mode).vel.y(vel);
}

void StateEstimator::setBaseVelZ(int estimate_mode, double vel)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  base_twist_.at(estimate_mode).vel.z(vel);
}

const KDL::Vector StateEstimator::getBaseAcc(int estimate_mode)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return base_acc_.at(estimate_mode);
}

void StateEstimator::setBaseAcc(int estimate_mode, KDL::Vector acc)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  base_acc_.at(estimate_mode) = acc;
}

const KDL::Rotation StateEstimator::getBaseOrientation(int estimate_mode)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return base_pose_.at(estimate_mode).M;
}

void StateEstimator::setBaseOrientation(int estimate_mode, KDL::Rotation rot)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  base_pose_.at(estimate_mode).M = rot;
}

const KDL::Vector StateEstimator::getBaseEuler(int estimate_mode)
{
  KDL::Rotation rot = getBaseOrientation(estimate_mode);
  double r, p, y;
  rot.GetRPY(r, p, y);
  return KDL::Vector(r, p, y);
}

const KDL::Vector StateEstimator::getBaseAngularVel(int estimate_mode)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  return base_twist_.at(estimate_mode).rot;
}

void StateEstimator::setBaseAngularVel(int estimate_mode, KDL::Vector omega)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  base_twist_.at(estimate_mode).rot = omega;
}

const KDL::Frame StateEstimator::getCogPose(int estimate_mode)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return cog_pose_.at(estimate_mode);
}

void StateEstimator::setCogPose(int estimate_mode, KDL::Frame pose)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  cog_pose_.at(estimate_mode) = pose;
}

const KDL::Twist StateEstimator::getCogTwist(int estimate_mode)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return cog_twist_.at(estimate_mode);
}

void StateEstimator::setCogTwist(int estimate_mode, KDL::Twist twist)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  cog_twist_.at(estimate_mode) = twist;
}

const KDL::Vector StateEstimator::getCogPos(int estimate_mode)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return cog_pose_.at(estimate_mode).p;
}

void StateEstimator::setCogPos(int estimate_mode, KDL::Vector pos)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  cog_pose_.at(estimate_mode).p = pos;
}

const KDL::Vector StateEstimator::getCogVel(int estimate_mode)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return cog_twist_.at(estimate_mode).vel;
}

void StateEstimator::setCogVel(int estimate_mode, KDL::Vector vel)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  cog_twist_.at(estimate_mode).vel = vel;
}

const KDL::Vector StateEstimator::getCogAcc(int estimate_mode)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return cog_acc_.at(estimate_mode);
}

void StateEstimator::setCogAcc(int estimate_mode, KDL::Vector acc)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  cog_acc_.at(estimate_mode) = acc;
}

const KDL::Rotation StateEstimator::getCogOrientation(int estimate_mode)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return cog_pose_.at(estimate_mode).M;
}

void StateEstimator::setCogOrientation(int estimate_mode, KDL::Rotation rot)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  cog_pose_.at(estimate_mode).M = rot;
}

const KDL::Vector StateEstimator::getCogEuler(int estimate_mode)
{
  KDL::Rotation rot = getCogOrientation(estimate_mode);
  double r, p, y;
  rot.GetRPY(r, p, y);
  return KDL::Vector(r, p, y);
}

const tf2::Quaternion StateEstimator::getCogQuaternion(int estimate_mode)
{
  KDL::Rotation rot = getCogOrientation(estimate_mode);
  double qx, qy, qz, qw;
  rot.GetQuaternion(qx, qy, qz, qw);  // NOTE: (x, y, z, w) order & normalized

  return tf2::Quaternion(qx, qy, qz, qw);
}

const KDL::Vector StateEstimator::getCogAngularVel(int estimate_mode)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  return cog_twist_.at(estimate_mode).rot;
}

void StateEstimator::setCogAngularVel(int estimate_mode, KDL::Vector omega)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  cog_twist_.at(estimate_mode).rot = omega;
}

void StateEstimator::setBaseOrientationWxB(int estimate_mode, KDL::Vector v)
{
  KDL::Vector wx_b = v;
  wx_b.Normalize();

  KDL::Rotation rot_inv = getBaseOrientation(estimate_mode).Inverse();
  KDL::Vector wz_b = rot_inv.UnitZ();
  KDL::Vector wy_b = wz_b * wx_b;
  wy_b.Normalize();

  wx_b = wy_b * wz_b;
  wx_b.Normalize();

  rot_inv.UnitX(wx_b);
  rot_inv.UnitY(wy_b);
  rot_inv.UnitZ(wz_b);

  setBaseOrientation(estimate_mode, rot_inv.Inverse());
}

void StateEstimator::setBaseOrientationWzB(int estimate_mode, KDL::Vector v)
{
  KDL::Vector wz_b = v;
  wz_b.Normalize();

  KDL::Rotation rot_inv = getBaseOrientation(estimate_mode).Inverse();
  KDL::Vector wx_b = rot_inv.UnitX();
  KDL::Vector wy_b = wz_b * wx_b;
  wy_b.Normalize();

  wx_b = wy_b * wz_b;
  wx_b.Normalize();

  rot_inv.UnitX(wx_b);
  rot_inv.UnitY(wy_b);
  rot_inv.UnitZ(wz_b);

  setBaseOrientation(estimate_mode, rot_inv.Inverse());
}

void StateEstimator::setCogOrientationWxB(int estimate_mode, KDL::Vector v)
{
  KDL::Vector wx_c = v;
  wx_c.Normalize();

  KDL::Rotation rot_inv = getCogOrientation(estimate_mode).Inverse();
  KDL::Vector wz_c = rot_inv.UnitZ();
  KDL::Vector wy_c = wz_c * wx_c;
  wy_c.Normalize();

  wx_c = wy_c * wz_c;
  wx_c.Normalize();

  rot_inv.UnitX(wx_c);
  rot_inv.UnitY(wy_c);
  rot_inv.UnitZ(wz_c);

  setCogOrientation(estimate_mode, rot_inv.Inverse());
}

void StateEstimator::setCogOrientationWzB(int estimate_mode, KDL::Vector v)
{
  KDL::Vector wz_c = v;
  wz_c.Normalize();

  KDL::Rotation rot_inv = getCogOrientation(estimate_mode).Inverse();
  KDL::Vector wx_c = rot_inv.UnitX();
  KDL::Vector wy_c = wz_c * wx_c;
  wy_c.Normalize();

  wx_c = wy_c * wz_c;
  wx_c.Normalize();

  rot_inv.UnitX(wx_c);
  rot_inv.UnitY(wy_c);
  rot_inv.UnitZ(wz_c);

  setCogOrientation(estimate_mode, rot_inv.Inverse());
}

void StateEstimator::updateBaseQueue(const double timestamp, const KDL::Rotation r_ee, const KDL::Rotation r_ex,
                                     const KDL::Vector omega)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  timestamp_qu_.push_back(timestamp);
  base_rot_ee_qu_.push_back(r_ee);
  base_rot_ex_qu_.push_back(r_ex);
  base_omega_qu_.push_back(omega);

  if (timestamp_qu_.size() > qu_size_)
  {
    timestamp_qu_.pop_front();
    base_rot_ee_qu_.pop_front();
    base_rot_ex_qu_.pop_front();
    base_omega_qu_.pop_front();
  }
}

bool StateEstimator::findBaseRotOmega(const double timestamp, const int mode, KDL::Rotation &r, KDL::Vector &omega,
                                      bool verbose)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);

  if (timestamp_qu_.size() == 0)
  {
    if (verbose)
    {
      RCLCPP_WARN(LOGGER, "[estimation] No valid queue for timestamp to find proper r and omega");
    }

    return false;
  }

  if (timestamp < timestamp_qu_.front())
  {
    if (verbose)
    {
      RCLCPP_WARN_STREAM(LOGGER, "[estimation] Sensor timestamp "
                                     << timestamp << " is earlier than the oldest timestamp " << timestamp_qu_.front()
                                     << " in queue");
    }
    return false;
  }

  if (timestamp > timestamp_qu_.back())
  {
    if (verbose)
    {
      RCLCPP_WARN_STREAM(LOGGER, "[estimation] Sensor timestamp " << timestamp << " is later than the latest timestamp "
                                                                  << timestamp_qu_.back() << " in queue");
    }
    return false;
  }

  size_t candidate_index = (timestamp_qu_.size() - 1) * (timestamp - timestamp_qu_.front()) /
                           (timestamp_qu_.back() - timestamp_qu_.front());

  if (timestamp > timestamp_qu_.at(candidate_index))
  {
    for (auto it = timestamp_qu_.begin() + candidate_index; it != timestamp_qu_.end(); ++it)
    {
      /* Future timestamp, escape */
      if (*it > timestamp)
      {
        if (fabs(*it - timestamp) < fabs(*(it - 1) - timestamp))
          candidate_index = std::distance(timestamp_qu_.begin(), it);
        else
          candidate_index = std::distance(timestamp_qu_.begin(), it - 1);

        // RCLCPP_INFO(LOGGER, "[estimation] Find timestamp sensor vs imu: [%f, %f], candidate: %d", timestamp,
        // timestamp_qu_.at(candidate_index), candidate_index);
        break;
      }
    }
  }
  else
  {
    auto reverse_index = timestamp_qu_.size() - 1 - candidate_index;
    for (auto it = timestamp_qu_.rbegin() + reverse_index; it != timestamp_qu_.rend(); ++it)
    {
      /* Future timestamp, escape */
      if (*it < timestamp)
      {
        if (fabs(*it - timestamp) < fabs(*(it - 1) - timestamp))
        {
          candidate_index = timestamp_qu_.size() - 1 - std::distance(timestamp_qu_.rbegin(), it);
        }
        else
        {
          candidate_index = timestamp_qu_.size() - 1 - std::distance(timestamp_qu_.rbegin(), it - 1);
        }

        // RCLCPP_INFO(LOGGER, "[estimation] Reverse find timestamp sensor vs imu: [%f, %f], %d", timestamp,
        // timestamp_qu_.at(candidate_index) , candidate_index);
        break;
      }
    }
  }

  omega = base_omega_qu_.at(candidate_index);
  switch (mode)
  {
    case EGOMOTION_ESTIMATE:
      r = base_rot_ee_qu_.at(candidate_index);
      break;
    case EXPERIMENT_ESTIMATE:
      r = base_rot_ex_qu_.at(candidate_index);
      break;
    default:
      RCLCPP_ERROR(LOGGER, "[estimation] Search state with timestamp: wrong mode %d", mode);
      return false;
  }

  return true;
}

const double StateEstimator::getImuLatestTimeStamp()
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  return timestamp_qu_.back();
}

const FuserList &StateEstimator::getFuserList(int mode) { return fuser_maps_.at(mode); }

/* Set unhealth level */
void StateEstimator::setUnhealthLevel(uint8_t unhealth_level)
{
  if (unhealth_level > unhealth_level_) unhealth_level_ = unhealth_level;

  /* TODO: should write the solution for the unhealth sensor  */
}

void StateEstimator::process()
{
  /* Check the heartbeat of each sensor */
  sensorHealthCheck();

  /* Publish */
  publish();
}

void StateEstimator::sensorHealthCheck()
{
  for (auto &sensor : sensors_)
  {
    sensor->healthCheck();
  }
}

void StateEstimator::publish()
{
  rclcpp::Time state_stamp(0, 0, node_->get_clock()->get_clock_type());
  if (isSpinalMode(requested_estimate_mode_))
  {
    {
      // Only protect the state metadata copy here. odomPublish() and
      // tfBroadcast() use state getters which acquire state_mutex_ again.
      // Keeping this lock across those calls deadlocks the executor after the
      // first spinal state message.
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!spinal_state_received_) return;
      state_stamp = spinal_state_stamp_;
    }
    if ((node_->get_clock()->now() - state_stamp).seconds() > spinal_state_timeout_)
    {
      RCLCPP_WARN_THROTTLE(LOGGER, *node_->get_clock(), 1000, "[estimation] Spinal state estimate is stale");
      return;
    }
    if (state_stamp.seconds() == 0) return;

    odomPublish(state_stamp);
    tfBroadcast(state_stamp);
    prev_pub_stamp_ = state_stamp;
    return;
  }

  if (imu_handlers_.empty()) return;
  state_stamp = imu_handlers_.at(0)->getTimeStamp();

  if (state_stamp.seconds() == 0) return;

  odomPublish(state_stamp);
  tfBroadcast(state_stamp);
  prev_pub_stamp_ = state_stamp;
}

void StateEstimator::publishExternalStateMeasurement(const rclcpp::Time &stamp, const KDL::Frame &cog_pose,
                                                     const KDL::Twist &cog_twist, uint8_t field_mask,
                                                     const KDL::Vector &position_variance,
                                                     const KDL::Vector &velocity_variance,
                                                     const KDL::Vector &attitude_variance)
{
  if (!external_state_pub_) return;

  spinal_msgs::msg::ExternalStateMeasurement msg;
  msg.stamp = stamp;
  msg.estimate_mode = static_cast<uint8_t>(requested_estimate_mode_);
  msg.field_mask = field_mask;
  for (size_t axis = 0; axis < 3; ++axis)
  {
    msg.position[axis] = cog_pose.p[axis];
    msg.position_variance[axis] = position_variance[axis];
    msg.velocity[axis] = cog_twist.vel[axis];
    msg.velocity_variance[axis] = velocity_variance[axis];
    msg.attitude_variance[axis] = attitude_variance[axis];
    msg.angular_velocity[axis] = cog_twist.rot[axis];
  }
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
  cog_pose.M.GetQuaternion(qx, qy, qz, qw);
  msg.attitude[0] = qx;
  msg.attitude[1] = qy;
  msg.attitude[2] = qz;
  msg.attitude[3] = qw;
  external_state_pub_->publish(msg);
}

void StateEstimator::spinalStateCallback(const spinal_msgs::msg::StateEstimate::SharedPtr msg)
{
  if (!msg) return;

  for (const float value : msg->position)
    if (!std::isfinite(value)) return;
  for (const float value : msg->velocity)
    if (!std::isfinite(value)) return;
  for (const float value : msg->acceleration)
    if (!std::isfinite(value)) return;
  for (const float value : msg->attitude)
    if (!std::isfinite(value)) return;
  for (const float value : msg->angular_velocity)
    if (!std::isfinite(value)) return;

  KDL::Frame cog_pose;
  cog_pose.p = KDL::Vector(msg->position[0], msg->position[1], msg->position[2]);
  cog_pose.M = KDL::Rotation::Quaternion(msg->attitude[0], msg->attitude[1], msg->attitude[2], msg->attitude[3]);
  KDL::Twist cog_twist(KDL::Vector(msg->velocity[0], msg->velocity[1], msg->velocity[2]),
                       KDL::Vector(msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2]));
  const KDL::Vector cog_acceleration(msg->acceleration[0], msg->acceleration[1], msg->acceleration[2]);

  const KDL::Frame cog_to_baselink = robot_model_->getCog2Baselink<KDL::Frame>();
  const KDL::Frame base_pose = cog_pose * cog_to_baselink;
  const KDL::Vector base_angular_velocity = cog_to_baselink.M.Inverse() * cog_twist.rot;
  const KDL::Vector base_velocity = cog_twist.vel + cog_pose.M * (cog_twist.rot * cog_to_baselink.p);

  std::lock_guard<std::mutex> lock(state_mutex_);
  cog_pose_.at(SPINAL_ESTIMATE) = cog_pose;
  cog_twist_.at(SPINAL_ESTIMATE) = cog_twist;
  cog_acc_.at(SPINAL_ESTIMATE) = cog_acceleration;
  base_pose_.at(SPINAL_ESTIMATE) = base_pose;
  base_twist_.at(SPINAL_ESTIMATE) = KDL::Twist(base_velocity, base_angular_velocity);
  base_acc_.at(SPINAL_ESTIMATE) = cog_acceleration;

  const bool horizontal_valid = (msg->validity & spinal_msgs::msg::StateEstimate::HORIZONTAL_POSITION_VALID) != 0U;
  const bool vertical_valid = (msg->validity & spinal_msgs::msg::StateEstimate::VERTICAL_POSITION_VALID) != 0U;
  const bool attitude_valid = (msg->validity & spinal_msgs::msg::StateEstimate::ATTITUDE_VALID) != 0U;
  base_pos_status_matrix_.at(SPINAL_ESTIMATE).at(State::X) = horizontal_valid ? 1 : 0;
  base_pos_status_matrix_.at(SPINAL_ESTIMATE).at(State::Y) = horizontal_valid ? 1 : 0;
  base_pos_status_matrix_.at(SPINAL_ESTIMATE).at(State::Z) = vertical_valid ? 1 : 0;
  cog_pos_status_matrix_.at(SPINAL_ESTIMATE) = base_pos_status_matrix_.at(SPINAL_ESTIMATE);
  base_rot_status_.at(SPINAL_ESTIMATE) = attitude_valid ? 1 : 0;
  cog_rot_status_.at(SPINAL_ESTIMATE) = attitude_valid ? 1 : 0;

  // The MCU clock is not guaranteed to be synchronized to the ROS clock.
  // Use receive time for PC-side freshness checks, odometry, and TF.
  spinal_state_stamp_ = node_->get_clock()->now();
  spinal_state_received_ = true;
}

void StateEstimator::odomPublish(rclcpp::Time stamp)
{
  nav_msgs::msg::Odometry odom_state;
  odom_state.header.stamp = stamp;
  odom_state.header.frame_id = "world";

  /* Publish Baselink odometry */
  std::string base_name = robot_model_->getBaselinkName();
  odom_state.child_frame_id = tf_prefix_.empty() ? base_name : tf_prefix_ + "/" + base_name;
  odom_state.pose.pose = tf2::toMsg(getBasePose(estimate_mode_));
  odom_state.twist.twist = aerial_robot_model::kdlToMsg(getBaseTwist(estimate_mode_));
  baselink_odom_pub_->publish(odom_state);

  /* Publish CoG odometry */
  odom_state.child_frame_id = tf_prefix_.empty() ? "cog" : tf_prefix_ + "/cog";
  odom_state.pose.pose = tf2::toMsg(getCogPose(estimate_mode_));
  odom_state.twist.twist = aerial_robot_model::kdlToMsg(getCogTwist(estimate_mode_));
  cog_odom_pub_->publish(odom_state);

  /* Publish End-Effector Contact Point odometry */
  if (robot_model_->hasFrame("ee_contact"))
  {
    // Conversion
    KDL::Frame ee_pose_;   // Expressed in world frame
    KDL::Twist ee_twist_;  // Expressed in world frame
    robot_model_->convertFromCoGToEEContact(getCogPose(estimate_mode_), getCogTwist(estimate_mode_), ee_pose_,
                                            ee_twist_);
    odom_state.child_frame_id = tf_prefix_.empty() ? "ee_contact" : tf_prefix_ + "/ee_contact";
    odom_state.pose.pose = tf2::toMsg(ee_pose_);
    odom_state.twist.twist = aerial_robot_model::kdlToMsg(ee_twist_);
    ee_contact_odom_pub_->publish(odom_state);
  }
}

void StateEstimator::tfBroadcast(rclcpp::Time stamp)
{
  /* Avoid the redundant timestamp which induces annoying logs from TF server */
  if (stamp.seconds() == prev_pub_stamp_.seconds()) return;

  const auto segments_tf = robot_model_->getSegmentsTf();
  /* Skip if kinemtiacs is not initialized */
  if (segments_tf.size() == 0) return;

  auto tf_base2root = segments_tf.at(robot_model_->getBaselinkName()).Inverse();
  auto tf_world2base = getBasePose(estimate_mode_);
  geometry_msgs::msg::TransformStamped tf_msg = aerial_robot_model::kdlToMsg(tf_world2base * tf_base2root);

  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = "world";
  tf_msg.child_frame_id = tf_prefix_.empty() ? "root" : tf_prefix_ + "/root";

  br_->sendTransform(tf_msg);
}
