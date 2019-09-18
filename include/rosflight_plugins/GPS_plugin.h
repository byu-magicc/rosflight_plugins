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

#include <rosflight_msgs/GNSS.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>

namespace rosflight_plugins 
{

  class GPSPlugin : public gazebo::ModelPlugin 
  {
   public:
    GPSPlugin();
    ~GPSPlugin();

   protected:
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);
    void OnUpdate(const gazebo::common::UpdateInfo&);

   private:
    // ROS Stuff
    std::string namespace_;
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher GNSS_pub_;
    ros::Publisher GNSS_fix_pub_;
    ros::Publisher GNSS_vel_pub_;

    // Gazebo connections
    std::string link_name_;
    gazebo::physics::WorldPtr world_;
    gazebo::physics::ModelPtr model_;
    gazebo::physics::LinkPtr link_;
    gazebo::event::ConnectionPtr updateConnection_;
    gazebo::common::Time last_time_;

    // Random Engine
    std::default_random_engine random_generator_;
    std::normal_distribution<double> standard_normal_distribution_;

    // Topic
    std::string gnss_topic_;
    std::string gnss_vel_topic_;
    std::string gnss_fix_topic_;

    // Message with static info prefilled
    rosflight_msgs::GNSS gnss_message_;
    sensor_msgs::NavSatFix gnss_fix_message_;
    geometry_msgs::TwistStamped gnss_vel_message_;

    // params
    double pub_rate_;
    bool noise_on_;
    double north_stdev_;
    double east_stdev_;
    double alt_stdev_;

    double north_k_GPS_;
    double east_k_GPS_;
    double alt_k_GPS_;

    double north_GPS_error_;
    double east_GPS_error_;
    double alt_GPS_error_;

    double initial_latitude_;
    double initial_longitude_;
    double initial_altitude_;

    double length_latitude_;
    double length_longitude_;

    double sample_time_;

    void measure(double dpn, double dpe, double & dlat, double & dlon);
    inline double deg_to_rad(double deg) {return deg * M_PI / 180.0;}

  };
}

#endif // ROSFLIGHT_PLUGINS_GPS_PLUGIN_H
