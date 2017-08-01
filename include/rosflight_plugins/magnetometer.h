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
#include <chrono>
#include <cmath>
#include <iostream>

#include <ros/ros.h>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <sensor_msgs/MagneticField.h>


namespace rosflight_plugins
{

  class MagnetometerPlugin : public gazebo::ModelPlugin {
   public:
    MagnetometerPlugin();
    ~MagnetometerPlugin();

   protected:
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);
    void OnUpdate(const gazebo::common::UpdateInfo&);

   private:
    // ROS Stuff
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher mag_pub_;

    // Publication Parameters
    std::string namespace_;
    std::string mag_topic_;
    std::string link_name_;
    double pub_rate_;
    double sample_time_;

    // params
    bool noise_on_;
    double inclination_;
    double declination_;

    gazebo::math::Vector3 inertial_magnetic_field_;
    gazebo::math::Vector3 bias_vector_;

    sensor_msgs::MagneticField mag_msg_;

    // Noise Parameters
    double noise_sigma_;
    double bias_range_;

    // Random Number stuff for noise
    std::default_random_engine random_gen_;
    std::normal_distribution<double> normal_dist_;
    std::uniform_real_distribution<double> uniform_dist_;

    // Gazebo connections
    gazebo::physics::WorldPtr world_;
    gazebo::physics::ModelPtr model_;
    gazebo::physics::LinkPtr link_;
    gazebo::event::ConnectionPtr updateConnection_;
    gazebo::common::Time last_time_;
  };
}

#endif // ROSFLIGHT_PLUGINS_GPS_PLUGIN_H
