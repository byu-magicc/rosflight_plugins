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

#include "rosflight_plugins/airspeed_plugin.h"


namespace rosflight_plugins
{

AirspeedPlugin::AirspeedPlugin() : ModelPlugin() {}

AirspeedPlugin::~AirspeedPlugin() {
  gazebo::event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  nh_.shutdown();
}


// void AirspeedPlugin::WindSpeedCallback(const geometry_msgs::Vector3 &wind)
// {
//   wind_.N = wind.x;
//   wind_.E = wind.y;
//   wind_.D = wind.z;
// }


void AirspeedPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    ROS_FATAL("A ROS node for Gazebo has not been initialized, unable to load airspeed plugin");
    return;
  }
  ROS_INFO("Loaded the airspeed plugin");

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
    gzerr << "[airspeed_plugin] Please specify a namespace." << std::endl;

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[airspeed_plugin] Please specify a linkName." << std::endl;

  frame_id_ = link_name_;
  link_ = model_->GetLink(link_name_);
  if (link_ == nullptr)
    gzthrow("[airspeed_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  //
  // ROS Node Setup
  //

  nh_ = ros::NodeHandle(namespace_);
  nh_private_ = ros::NodeHandle(namespace_ + "/airspeed");

  // load params from rosparam server
  airspeed_topic_ = nh_private_.param<std::string>("airspeed_topic", "airspeed/data");
  pressure_bias_ = nh_private_.param<double>("pressure_bias", 0);
  pressure_noise_sigma_ = nh_private_.param<double>("pressure_noise_sigma", 0);
  rho_ = nh_private_.param<double>("air_density", 1.225);
  max_pressure_ = nh_private_.param<double>("max_pressure", 4000.0);
  min_pressure_ = nh_private_.param<double>("min_pressure", 0.0);

  // ROS publishers
  airspeed_pub_ = nh_.advertise<rosflight_msgs::Airspeed>(airspeed_topic_, 10);

  // Configure Noise
  random_generator_ = std::default_random_engine(std::chrono::system_clock::now().time_since_epoch().count());
  standard_normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);

  // Fill static members of airspeed message.
  airspeed_message_.header.frame_id = frame_id_;

  // Listen to the update event. This event is broadcast every simulation iteration.
  this->updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&AirspeedPlugin::OnUpdate, this, std::placeholders::_1));
}

void AirspeedPlugin::OnUpdate(const gazebo::common::UpdateInfo& _info) {

  // Calculate Airspeed
  gazebo::math::Vector3 C_linear_velocity_W_C = link_->GetRelativeLinearVel();
  double u = C_linear_velocity_W_C.x;
  double v = -C_linear_velocity_W_C.y;
  double w = -C_linear_velocity_W_C.z;

  // TODO: Wind is being applied in the inertial frame, not the body-fixed frame
  // double ur = u - wind_.N;
  // double vr = v - wind_.E;
  // double wr = w - wind_.D;
  // double Va = sqrt(pow(ur,2.0) + pow(vr,2.0) + pow(wr,2.0));
  double Va = sqrt(pow(u,2.0) + pow(v,2.0) + pow(w,2.0));

  // Invert Airpseed to get sensor measurement
  double y = rho_*Va*Va/2.0; // Page 130 in the UAV Book
  y += pressure_bias_ + pressure_noise_sigma_*standard_normal_distribution_(random_generator_);

  // constrain: min_pressure < y < max_pressure
  y = (y > max_pressure_) ? max_pressure_ : y;
  y = (y < min_pressure_) ? min_pressure_ : y;

  airspeed_message_.differential_pressure = y;
  airspeed_message_.temperature = 27.0;
  airspeed_message_.velocity = Va;

  airspeed_pub_.publish(airspeed_message_);
}


GZ_REGISTER_MODEL_PLUGIN(AirspeedPlugin);
}
