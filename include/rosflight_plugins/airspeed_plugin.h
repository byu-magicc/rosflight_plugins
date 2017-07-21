/*
 * Copyright 2015 James Jackson BYU Provo, UT
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

#ifndef ROSFLIGHT_PLUGINS_AIRSPEED_PLUGIN_H
#define ROSFLIGHT_PLUGINS_AIRSPEED_PLUGIN_H

#include <random>
#include <chrono>
#include <cmath>
#include <iostream>

#include <ros/ros.h>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <geometry_msgs/Vector3.h>

#include <rosflight_msgs/Airspeed.h>

namespace rosflight_plugins
{

class AirspeedPlugin : public gazebo::ModelPlugin {
 public:

  AirspeedPlugin();
  ~AirspeedPlugin();

  void InitializeParams();
  void Publish();

 protected:

  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const gazebo::common::UpdateInfo&);

 private:
  std::string namespace_;
  std::string airspeed_topic_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher airspeed_pub_;
  std::string frame_id_;
  std::string link_name_;

  // Gazebo connections
  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr link_;
  gazebo::event::ConnectionPtr updateConnection_;
  gazebo::common::Time last_time_;

  // Random Engine
  std::default_random_engine random_generator_;
  std::normal_distribution<double> standard_normal_distribution_;

  // Wind Connection
  // struct Wind{ double N;  double E;  double D; } wind_;
  // ros::Subscriber wind_speed_sub_;
  // void WindSpeedCallback(const geometry_msgs::Vector3& wind);

  // Message with static info prefilled
  rosflight_msgs::Airspeed airspeed_message_;

  // params
  double pressure_bias_;
  double pressure_noise_sigma_;
  double max_pressure_;
  double min_pressure_;
  double rho_;
};
}

#endif // ROSFLIGHT_PLUGINS_AIRSPEED_PLUGIN_H
