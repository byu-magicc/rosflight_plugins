/*
 * Copyright 2016 James Jackson, Brigham Young University, Provo, UT
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

#ifndef ROSFLIGHT_PLUGINS_RANGE_PLUGIN_H
#define ROSFLIGHT_PLUGINS_RANGE_PLUGIN_H

#include <random>
#include <chrono>
#include <cmath>
#include <iostream>

#include <ros/ros.h>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <rosflight_msgs/Barometer.h>

namespace rosflight_plugins
{

  class BarometerPlugin : public gazebo::ModelPlugin
  {
   public:

    BarometerPlugin();
    ~BarometerPlugin();

   protected:
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
    void OnUpdate(const gazebo::common::UpdateInfo&);

   private:
    // ROS Stuff
    std::string namespace_;
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher alt_pub_;

    // Topic
    std::string message_topic_;

    // params
    double error_stdev_;
    double pub_rate_;
    bool noise_on_;
    double sample_time_;

    // Random Engine
    std::default_random_engine random_generator_;
    std::normal_distribution<double> standard_normal_distribution_;

    // Gazebo Information
    std::string link_name_;
    gazebo::physics::WorldPtr world_;
    gazebo::physics::ModelPtr model_;
    gazebo::physics::LinkPtr link_;
    gazebo::event::ConnectionPtr updateConnection_;
    gazebo::common::Time last_time_;

  };
}

#endif // ROSFLIGHT_PLUGINS_RANGE_PLUGIN_H
