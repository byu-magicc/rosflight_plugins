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


#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <sensor_msgs/MagneticField.h>

#include <random>
#include <tf/tf.h>

#include "rosflight_plugins/common.h"



namespace gazebo {

class MagnetometerPlugin : public ModelPlugin {
 public:

  MagnetometerPlugin();
  ~MagnetometerPlugin();

 protected:

  void InitializeParams();
  void Publish();
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo&);

 private:

  // ROS connections
  ros::NodeHandle* nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher mag_pub_;

  // Publication Parameters
  std::string namespace_;
  std::string mag_topic_;
  std::string frame_id_;
  std::string link_name_;
  double pub_rate_;
  double next_pub_time_;

  // True Values
  bool noise_on_;
  double inclination_;
  double declination_;

  math::Vector3 inertial_magnetic_field_;
  math::Vector3 bias_vector_;

  sensor_msgs::MagneticField mag_msg_;

  // Noise Parameters
  double noise_sigma_;
  double bias_range_;

  // Random Number stuff for noise
  std::default_random_engine random_gen_;
  std::normal_distribution<double> normal_dist_;
  std::uniform_real_distribution<double> uniform_dist_;

  // Gazebo connections
  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  event::ConnectionPtr updateConnection_;
};
}

#endif // ROSFLIGHT_PLUGINS_GPS_PLUGIN_H
