/*
 * Copyright 2017 James Jackson Brigham Young University MAGICC Lab
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROSFLIGHT_PLUGINS_IMU_PLUGIN_H
#define ROSFLIGHT_PLUGINS_IMU_PLUGIN_H

#include <random>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

#include "rosflight_plugins/common.h"


namespace gazebo {

class ImuPlugin : public ModelPlugin {
 public:

  ImuPlugin();
  ~ImuPlugin();

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  void Reset();

  void OnUpdate(const common::UpdateInfo&);

 private:
  std::string namespace_;
  std::string imu_topic_;
  std::string acc_bias_topic_;
  std::string gyro_bias_topic_;
  std::string link_name_;

  ros::NodeHandle nh_;
  ros::Publisher imu_pub_;
  ros::Publisher acc_bias_pub_;
  ros::Publisher gyro_bias_pub_;

  double update_rate_;
  double gyro_stdev_;
  double acc_stdev_;
  double gyro_bias_range_;
  double acc_bias_range_;
  double gyro_bias_walk_stdev_;
  double acc_bias_walk_stdev_;
  double mass_;

  std::default_random_engine random_generator_;
  std::normal_distribution<double> normal_distribution_;
  std::uniform_real_distribution<double> uniform_distribution_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  event::ConnectionPtr updateConnection_;

  common::Time last_time_;

  sensor_msgs::Imu imu_message_;

  math::Vector3 gravity_;
  math::Vector3 prev_velocity_;

  math::Vector3 gyro_bias_;
  math::Vector3 acc_bias_;
};
}

#endif // ROSFLIGHT_PLUGINS_IMU_PLUGIN_H
