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
#include "aerial_robot_estimation/sensor/mocap.h"

using namespace aerial_robot_estimation;

namespace sensor_plugin
{
Mocap::Mocap() : sensor_plugin::SensorBase()
{
  states_.states.resize(6);
  states_.states[0].id = "x";
  states_.states[0].state.resize(2);
  states_.states[1].id = "y";
  states_.states[1].state.resize(2);
  states_.states[2].id = "z";
  states_.states[2].state.resize(2);
  states_.states[3].id = "roll";  // rot_x
  states_.states[3].state.resize(2);
  states_.states[4].id = "pitch";  // rot_y
  states_.states[4].state.resize(2);
  states_.states[5].id = "yaw";  // rot_z
  states_.states[5].state.resize(2);

  raw_pose_ = KDL::Frame::Identity();
  pose_ = KDL::Frame::Identity();
  prev_raw_pose_ = KDL::Frame::Identity();
  raw_twist_ = KDL::Twist::Zero();
  twist_ = KDL::Twist::Zero();
  prev_raw_twist_ = KDL::Twist::Zero();
}

void Mocap::initialize(rclcpp::Node::SharedPtr node, std::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                       std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator, std::string sensor_name,
                       int index)
{
  SensorBase::initialize(node, robot_model, estimator, sensor_name, index);

  std::string topic_name;
  getParam<std::string>("mocap_sub_name", topic_name, std::string("mocap/pose"));
  msg_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      topic_name, rclcpp::SystemDefaultsQoS(), std::bind(&Mocap::poseCallback, this, std::placeholders::_1));

  topic_name = sensor_name + std::to_string(index) + "/states";
  state_pub_ = node_->create_publisher<aerial_robot_msgs::msg::States>(topic_name, rclcpp::SystemDefaultsQoS());


  rosParamInit();

  // Initialize low-pass filter (lpf)
  lpf_pos_ = IirFilter(sample_freq_, cutoff_pos_freq_, 3);
  lpf_vel_ = IirFilter(sample_freq_, cutoff_vel_freq_, 3);
  lpf_omega_ = IirFilter(sample_freq_, cutoff_vel_freq_, 3);
}

void Mocap::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  raw_pose_ = aerial_robot_model::msgToKdl(msg->pose);

  time_stamp_ = msg->header.stamp;

  estimateProcess();

  prev_raw_pose_ = raw_pose_;
  prev_raw_twist_ = raw_twist_;

  prev_time_stamp_ = time_stamp_;
}

void Mocap::estimateProcess()
{
  if (sensor_status_ == Status::INVALID) return;

  /* Initialize */
  if (prev_raw_pose_ == KDL::Frame::Identity())
  {
    Eigen::Vector3d pos_vec;
    tf2::vectorKDLToEigen(raw_pose_.p, pos_vec);
    lpf_pos_.setInitValues(pos_vec);  // Init pos filter with the first non-zero value

    activateFuser();
    estimator_->SetRefinedYawEstimate(EXPERIMENT_ESTIMATE, true);

    return;
  }

  preProcessState();

  publishExternalMeasurement();

  fuse();

  setState();

  publish();

  updateHealthStamp();
}

void Mocap::activateFuser()
{
  /* Set ground truth */
  estimator_->setBasePosStateStatus(State::X, GROUND_TRUTH, true);
  estimator_->setBasePosStateStatus(State::Y, GROUND_TRUTH, true);
  estimator_->setBasePosStateStatus(State::Z, GROUND_TRUTH, true);
  estimator_->setBaseRotStateStatus(GROUND_TRUTH, true);
  estimator_->setCogRotStateStatus(GROUND_TRUTH, true);

  bool fuse_flag = estimate_mode_ & (1 << EXPERIMENT_ESTIMATE);
  if (!fuse_flag) return;

  estimator_->setBasePosStateStatus(State::X, EXPERIMENT_ESTIMATE, true);
  estimator_->setBasePosStateStatus(State::Y, EXPERIMENT_ESTIMATE, true);
  estimator_->setBasePosStateStatus(State::Z, EXPERIMENT_ESTIMATE, true);

  for (auto &fuser : estimator_->getFuserList(EXPERIMENT_ESTIMATE))
  {
    std::string plugin_name = fuser.first;
    FuserPtr kf = fuser.second;
    int id = kf->getId();

    /* X, y, z */
    if (plugin_name == "kalman_filter/kf_pos_vel_acc")
    {
      if (id & (1 << State::X))
      {
        kf->setInitState(raw_pose_.p.x(), 0);
      }

      if (id & (1 << State::Y))
      {
        kf->setInitState(raw_pose_.p.y(), 0);
      }

      if (id & (1 << State::Z))
      {
        kf->setInitState(raw_pose_.p.z(), 0);
      }

      kf->setMeasureFlag();
    }
  }
}

void Mocap::preProcessState()
{
  float delta_t = time_stamp_.seconds() - prev_time_stamp_.seconds();
  raw_twist_.vel = (raw_pose_.p - prev_raw_pose_.p) / delta_t;

  KDL::Rotation rot_delta = prev_raw_pose_.M.Inverse() * raw_pose_.M;
  double r, p, y;
  rot_delta.GetRPY(r, p, y);
  raw_twist_.rot = KDL::Vector(r / delta_t, p / delta_t, y / delta_t);

  pose_.M = raw_pose_.M;  // No low-pass filter for orientation

  /* Low-pass filter */
  Eigen::Vector3d in_vec, out_vec;
  tf2::vectorKDLToEigen(raw_pose_.p, in_vec);
  out_vec = lpf_pos_.filterFunction(in_vec);
  tf2::vectorEigenToKDL(out_vec, pose_.p);

  tf2::vectorKDLToEigen(raw_twist_.vel, in_vec);
  out_vec = lpf_vel_.filterFunction(in_vec);
  tf2::vectorEigenToKDL(out_vec, twist_.vel);

  tf2::vectorKDLToEigen(raw_twist_.rot, in_vec);
  out_vec = lpf_omega_.filterFunction(in_vec);
  tf2::vectorEigenToKDL(out_vec, twist_.rot);
}

void Mocap::fuse()
{
  /* Start experiment estimation */

  bool flag = estimate_mode_ & (1 << EXPERIMENT_ESTIMATE);
  if (!flag) return;

  for (auto &fuser : estimator_->getFuserList(EXPERIMENT_ESTIMATE))
  {
    std::string plugin_name = fuser.first;
    FuserPtr kf = fuser.second;
    int id = kf->getId();

    if (plugin_name == "kalman_filter/kf_pos_vel_acc")
    {
      int index;

      if (id & (1 << State::X))
        index = 0;
      else if (id & (1 << State::Y))
        index = 1;
      else if (id & (1 << State::Z))
        index = 2;
      else
      {
        RCLCPP_ERROR_THROTTLE(logger_, *node_->get_clock(), 1.0, "[MoCap] Wrong index for KF fusion, id is %d", id);
        return;
      }

      // If kf state dimenstion is 2 (pos + vel),
      // force change 3 (pos + vel + acc_bias)
      if (kf->getStateDim() == 2 && acc_bias_noise_sigma_ > 0)
      {
        if (kf->getPredictionNoiseCovariance().rows() == 0) return;

        // Force acc bias estimation by kalman filter
        Eigen::VectorXd input_noise_sigma(2);
        input_noise_sigma << kf->getPredictionNoiseCovariance()(0, 0), acc_bias_noise_sigma_;

        kf->setPredictionNoiseCovariance(input_noise_sigma);
        kf->setInitState(raw_pose_.p[index], 0);
      }

      /* Correction */
      Eigen::VectorXd measure_sigma(1);
      measure_sigma << pos_noise_sigma_;
      Eigen::VectorXd meas(1);
      meas << raw_pose_.p[index];
      std::vector<double> params = { kf_plugin::POS };
      kf->correction(meas, measure_sigma, -1, params);  // No time sync
    }
  }
}

void Mocap::setState()
{
  // EXPERIMENT_ESTIMATE mode
  // only update the wx_b vector (the vector only related to yaw)
  // pos is updated in IMU plugin
  KDL::Frame cog2baselink_tf = robot_model_->getCog2Baselink<KDL::Frame>();
  KDL::Vector wx_baselink = pose_.M.Inverse().UnitX();
  KDL::Vector wx_cog = cog2baselink_tf.M * wx_baselink;
  estimator_->setBaseOrientationWxB(EXPERIMENT_ESTIMATE, wx_baselink);
  estimator_->setCogOrientationWxB(EXPERIMENT_ESTIMATE, wx_cog);

  // GROUND TRUTH mode
  // skip if there is a true plugin handle the ground truth odom
  if (estimator_->hasGroundTruthOdom()) return;

  // Rotation
  // only trust wx_baselink and wx_cog;
  KDL::Rotation rot_baselink = estimator_->getBaseOrientation(EXPERIMENT_ESTIMATE);
  estimator_->setBaseOrientation(GROUND_TRUTH, rot_baselink);
  KDL::Rotation rot_cog = estimator_->getCogOrientation(EXPERIMENT_ESTIMATE);
  estimator_->setCogOrientation(GROUND_TRUTH, rot_cog);

  // Position & vel
  // baselink
  estimator_->setBasePos(GROUND_TRUTH, pose_.p);
  estimator_->setBaseVel(GROUND_TRUTH, twist_.vel);
  // CoG
  /* pos_cog = pos_baselink + rot_baselink * pos_baselink2cog */
  KDL::Vector baselink2cog_vec = cog2baselink_tf.Inverse().p;
  KDL::Vector pos_cog = pose_.p + rot_baselink * baselink2cog_vec;
  estimator_->setCogPos(GROUND_TRUTH, pos_cog);
  /* vel_cog = vel_baselink + rot_baselink * (w x pos_baselink2cog) */
  KDL::Vector vel_cog = twist_.vel + rot_baselink * (twist_.rot * baselink2cog_vec);
  estimator_->setCogVel(GROUND_TRUTH, vel_cog);
}

void Mocap::publish()
{
  states_.header.stamp = time_stamp_;

  for (int i = 0; i < 6; i++)
  {
    if (i < 3)
    {
      states_.states[i].state[0].pos = raw_pose_.p[i];
      states_.states[i].state[0].vel = raw_twist_.vel[i];
      states_.states[i].state[1].pos = pose_.p[i];
      states_.states[i].state[1].vel = twist_.vel[i];
    }
    else
    {
      states_.states[i].state[0].vel = raw_twist_.rot[i - 3];
      states_.states[i].state[1].vel = twist_.rot[i - 3];
    }
  }

  state_pub_->publish(states_);
}

void Mocap::publishExternalMeasurement()
{
  const int mode = estimator_->getRequestedEstimateMode();
  if (mode != SPINAL_MODE_EXPERIMENT && mode != SPINAL_MODE_GROUND_TRUTH) return;

  const KDL::Frame cog_to_baselink = robot_model_->getCog2Baselink<KDL::Frame>();
  const KDL::Vector baselink_to_cog = cog_to_baselink.Inverse().p;
  KDL::Frame cog_pose;
  cog_pose.M = pose_.M * cog_to_baselink.M.Inverse();
  cog_pose.p = pose_.p + pose_.M * baselink_to_cog;

  KDL::Twist cog_twist;
  cog_twist.vel = twist_.vel + pose_.M * (twist_.rot * baselink_to_cog);
  cog_twist.rot = cog_to_baselink.M * twist_.rot;

  const double position_variance = pos_noise_sigma_ * pos_noise_sigma_;
  const double velocity_variance = vel_noise_sigma_ * vel_noise_sigma_;
  const double attitude_variance = attitude_noise_sigma_ * attitude_noise_sigma_;
  estimator_->publishExternalStateMeasurement(time_stamp_, cog_pose, cog_twist,
                                              spinal_msgs::msg::ExternalStateMeasurement::POSITION |
                                                  spinal_msgs::msg::ExternalStateMeasurement::VELOCITY |
                                                  spinal_msgs::msg::ExternalStateMeasurement::ATTITUDE,
                                              KDL::Vector(position_variance, position_variance, position_variance),
                                              KDL::Vector(velocity_variance, velocity_variance, velocity_variance),
                                              KDL::Vector(attitude_variance, attitude_variance, attitude_variance));
}

void Mocap::rosParamInit()
{
  getParam<double>("pos_noise_sigma", pos_noise_sigma_, 0.001);
  getParam<double>("vel_noise_sigma", vel_noise_sigma_, 0.02);
  getParam<double>("attitude_noise_sigma", attitude_noise_sigma_, 0.02);
  getParam<double>("acc_bias_noise_sigma", acc_bias_noise_sigma_, 0.0);
  getParam<double>("sample_freq", sample_freq_, 100.0);
  getParam<double>("cutoff_pos_freq", cutoff_pos_freq_, 20.0);
  getParam<double>("cutoff_vel_freq", cutoff_vel_freq_, 20.0);
}

}

/* Plugin registration */
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::Mocap, sensor_plugin::SensorBase);
