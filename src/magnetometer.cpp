/*
 * Copyright 2015 James Jackson MAGICC Lab, BYU, Provo, UT
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

#include "rosflight_plugins/magnetometer.h"

namespace rosflight_plugins
{

MagnetometerPlugin::MagnetometerPlugin() : ModelPlugin() {}


MagnetometerPlugin::~MagnetometerPlugin() {
  gazebo::event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  nh_.shutdown();
}


void MagnetometerPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    ROS_FATAL("A ROS node for Gazebo has not been initialized, unable to load magnetometer plugin");
    return;
  }
  ROS_INFO("Loaded the magnetometer plugin");

  //
  // Configure Gazebo Integration
  //

  model_ = _model;
  world_ = model_->GetWorld();

  last_time_ = world_->GetSimTime();

  namespace_.clear();

  //
  // Get elements from the robot urdf/sdf file
  //

  if (_sdf->HasElement("namespace"))
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
  else
    ROS_ERROR("[magnetometer_plugin] Please specify a namespace.");

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    ROS_ERROR("[magnetometer_plugin] Please specify a linkName.");

  link_ = model_->GetLink(link_name_);
  if (link_ == nullptr)
    gzthrow("[magnetometer_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  //
  // ROS Node Setup
  //

  nh_ = ros::NodeHandle(namespace_);
  nh_private_ = ros::NodeHandle(namespace_ + "/magnetometer");

  // load params from rosparam server
  noise_on_ = nh_private_.param<bool>("noise_on", true);
  mag_topic_ = nh_private_.param<std::string>("topic", "magnetometor");
  noise_sigma_ = nh_private_.param<double>("stdev", 0.01);
  bias_range_ = nh_private_.param<double>("bias_range", 0.01);
  pub_rate_ = nh_private_.param<double>("rate", 50.0);
  declination_ = nh_private_.param<double>("declination", 0.198584539676); // default to Provo, UT
  inclination_ = nh_private_.param<double>("inclination", 1.14316156541); // default to Provo, UT

  // Set up ROS publisher
  mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>(mag_topic_, 10);

  // Calculate sample time from sensor update rate
  sample_time_ = 1.0/pub_rate_;

  // set up noise parameters
  normal_dist_ = std::normal_distribution<double>(0.0, 1.0);
  uniform_dist_ = std::uniform_real_distribution<double>(-bias_range_, bias_range_);

  // Create a bias offset
  bias_vector_.x = uniform_dist_(random_gen_);
  bias_vector_.y = uniform_dist_(random_gen_);
  bias_vector_.z = uniform_dist_(random_gen_);

  // Figure out inertial magnetic field
  // Gazebo coordinates is NWU and Earth's magnetic field is defined in NED, hence the negative signs
  inertial_magnetic_field_.z = sin(-inclination_);
  inertial_magnetic_field_.x = cos(-inclination_)*cos(-declination_);
  inertial_magnetic_field_.y = cos(-inclination_)*sin(-declination_);

  // turn off noise and bias of noise_on is disabled
  if (!noise_on_)
  {
    noise_sigma_ = 0;
    bias_vector_.x = 0;
    bias_vector_.y = 0;
    bias_vector_.z = 0;
  }

  // Fill in static members of message
  mag_msg_.header.frame_id = link_name_;
  for (int i = 0; i < 3; i++)
      mag_msg_.magnetic_field_covariance[i + 3*i] = noise_sigma_*noise_sigma_;

  // Listen to the update event. This event is broadcast every simulation iteration.
  this->updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&MagnetometerPlugin::OnUpdate, this, std::placeholders::_1));
}


void MagnetometerPlugin::OnUpdate(const gazebo::common::UpdateInfo& _info)
{
  // check if time to publish
  gazebo::common::Time current_time = world_->GetSimTime();
  if ((current_time - last_time_).Double() >= sample_time_) {

    gazebo::math::Pose I_to_B = link_->GetWorldPose();

    gazebo::math::Vector3 noise;
    noise.x = noise_sigma_*normal_dist_(random_gen_);
    noise.y = noise_sigma_*normal_dist_(random_gen_);
    noise.z = noise_sigma_*normal_dist_(random_gen_);

    // combine parts to create a measurement
    gazebo::math::Vector3 measurement = I_to_B.rot.RotateVectorReverse(inertial_magnetic_field_) + noise + bias_vector_;

    // normalize measurement
    gazebo::math::Vector3 normalized = measurement.Normalize();

    mag_msg_.header.stamp.fromSec(world_->GetSimTime().Double());
    mag_msg_.magnetic_field.x =  normalized.x;
    mag_msg_.magnetic_field.y = -normalized.y; // convert to NED for publishing
    mag_msg_.magnetic_field.z = -normalized.z;

    mag_pub_.publish(mag_msg_);

    // save current time
    last_time_ = current_time;
  }
}

GZ_REGISTER_MODEL_PLUGIN(MagnetometerPlugin);
}
