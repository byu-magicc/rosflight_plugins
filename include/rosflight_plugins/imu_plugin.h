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
#include <chrono>
#include <cmath>
#include <iostream>

#include <ros/ros.h>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>

namespace rosflight_plugins
{

  class ImuPlugin : public gazebo::ModelPlugin {
   public:
    ImuPlugin();
    ~ImuPlugin();

   protected:
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);
    void OnUpdate(const gazebo::common::UpdateInfo&);

    void Reset();

   private:
    // ROS Stuff
    std::string namespace_;
    std::string imu_topic_;
    std::string acc_bias_topic_;
    std::string gyro_bias_topic_;
    std::string link_name_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher imu_pub_;
    ros::Publisher acc_bias_pub_;
    ros::Publisher gyro_bias_pub_;

    // params
    bool noise_on_;
    double pub_rate_;
    double sample_time_;
    double gyro_stdev_;
    double acc_stdev_;
    double gyro_bias_range_;
    double acc_bias_range_;
    double gyro_bias_walk_stdev_;
    double acc_bias_walk_stdev_;
    double mass_;

    // Random Engine
    std::default_random_engine random_generator_;
    std::normal_distribution<double> normal_distribution_;
    std::uniform_real_distribution<double> uniform_distribution_;

    gazebo::physics::WorldPtr world_;
    gazebo::physics::ModelPtr model_;
    gazebo::physics::LinkPtr link_;
    gazebo::event::ConnectionPtr updateConnection_;

    gazebo::common::Time last_time_;

    sensor_msgs::Imu imu_message_;

    ignition::math::Vector3d gravity_;

    ignition::math::Vector3d gyro_bias_;
    ignition::math::Vector3d acc_bias_;
  };
}

#endif // ROSFLIGHT_PLUGINS_IMU_PLUGIN_H
