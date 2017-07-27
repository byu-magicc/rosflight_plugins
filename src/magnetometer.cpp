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


namespace gazebo {


MagnetometerPlugin::MagnetometerPlugin() : ModelPlugin() {}


MagnetometerPlugin::~MagnetometerPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (nh_) {
    nh_->shutdown();
    delete nh_;
  }
}


void MagnetometerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  // default params
  namespace_.clear();

  if (_sdf->HasElement("namespace"))
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
  nh_ = new ros::NodeHandle(namespace_);
  nh_private_ = ros::NodeHandle(namespace_ + "/magnetometer");

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_imu_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  frame_id_ = link_name_;
  next_pub_time_ = world_->GetSimTime().Double();

  noise_on_ = nh_private_.param<bool>("noise_on", true);
  mag_topic_ = nh_private_.param<std::string>("topic", "magnetometor");
  noise_sigma_ = nh_private_.param<double>("stdev", 0.01);
  bias_range_ = nh_private_.param<double>("bias_range", 0.01);
  pub_rate_ = nh_private_.param<double>("rate", 50.0);
  declination_ = nh_private_.param<double>("declination", 0.198584539676); // default to Provo, UT
  inclination_ = nh_private_.param<double>("inclination", 1.14316156541); // default to Provo, UT

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

  // Set up ROS publisher
  mag_pub_ = nh_->advertise<sensor_msgs::MagneticField>(mag_topic_, 10);

  // Listen to the update event. This event is broadcast every simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&MagnetometerPlugin::OnUpdate, this, _1));

  // turn off noise and bias of noise_on is disabled
  if (!noise_on_)
  {
    noise_sigma_ = 0;
    bias_vector_.x = 0;
    bias_vector_.y = 0;
    bias_vector_.z = 0;
  }

  // Fill in static members of message
  mag_msg_.header.frame_id = frame_id_;
  mag_msg_.header.seq = 0;
  for(int i = 0; i < 3; i++)
      mag_msg_.magnetic_field_covariance[i + 3*i] = noise_sigma_*noise_sigma_;
}


// This gets called by the world update start event.
void MagnetometerPlugin::OnUpdate(const common::UpdateInfo& _info)
{
    math::Pose I_to_B = link_->GetWorldPose();
    double now = world_->GetSimTime().Double();

    math::Vector3 noise;
    noise.x = noise_sigma_*normal_dist_(random_gen_);
    noise.y = noise_sigma_*normal_dist_(random_gen_);
    noise.z = noise_sigma_*normal_dist_(random_gen_);

    // combine parts to create a measurement
    math::Vector3 measurement = I_to_B.rot.RotateVectorReverse(inertial_magnetic_field_) + noise + bias_vector_;

    // normalize measurement
    math::Vector3 normalized = measurement.Normalize();

    // if time, publish
    if(now > next_pub_time_)
    {
        next_pub_time_ += 1/pub_rate_;
        mag_msg_.header.seq += 1;
        mag_msg_.header.stamp.sec = world_->GetSimTime().sec;
        mag_msg_.header.stamp.nsec = world_->GetSimTime().nsec;
        mag_msg_.magnetic_field.x = normalized.x;
        mag_msg_.magnetic_field.y = -normalized.y; // convert to NED for publishing
        mag_msg_.magnetic_field.z = -normalized.z;
        mag_pub_.publish(mag_msg_);
    }
}

GZ_REGISTER_MODEL_PLUGIN(MagnetometerPlugin);
}
