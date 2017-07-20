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

#ifndef ROSFLIGHT_PLUGINS_GPS_PLUGIN_H
#define ROSFLIGHT_PLUGINS_GPS_PLUGIN_H

#include <random>

#include <eigen3/Eigen/Core>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <rosflight_msgs/GPS.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

#include "rosflight_plugins/common.h"

namespace gazebo 
{

class GPSPlugin : public ModelPlugin 
{
 public:

  GPSPlugin();
  ~GPSPlugin();

  void InitializeParams();
  void Publish();

 protected:

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo&);

 private:
  std::string namespace_;
  std::string GPS_topic_;
  ros::NodeHandle* nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher GPS_pub_;
  double update_rate_;
  std::string frame_id_;
  std::string link_name_;
  bool noise_on_;

  std::default_random_engine random_generator_;
  std::normal_distribution<double> standard_normal_distribution_;

  // Gazebo connections
  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  event::ConnectionPtr updateConnection_;

  common::Time last_time_;
  double next_pub_time_;

  // Wind Connection
  struct Wind{ double N;  double E;  double D; } wind_;
  ros::Subscriber wind_speed_sub_;

  rosflight_msgs::GPS GPS_message_;

  double north_stdev_;
  double east_stdev_;
  double alt_stdev_;

  double north_k_GPS_;
  double east_k_GPS_;
  double alt_k_GPS_;
  double sample_time_;

  double north_GPS_error_;
  double east_GPS_error_;
  double alt_GPS_error_;

  double initial_latitude_;
  double initial_longitude_;
  double initial_altitude_;

  double length_latitude_;
  double length_longitude_;

  void measure(double dpn, double dpe, double & dlat, double & dlon);

};
}

#endif // ROSFLIGHT_PLUGINS_GPS_PLUGIN_H
