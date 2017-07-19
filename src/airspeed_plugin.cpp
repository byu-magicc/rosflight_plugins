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


namespace gazebo {


AirspeedPlugin::AirspeedPlugin() : ModelPlugin() {}


AirspeedPlugin::~AirspeedPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (nh_) {
    nh_->shutdown();
    delete nh_;
  }
}

void AirspeedPlugin::WindSpeedCallback(const geometry_msgs::Vector3 &wind){
  wind_.N = wind.x;
  wind_.E = wind.y;
  wind_.D = wind.z;
}



void AirspeedPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  // default params
  namespace_.clear();

  if (_sdf->HasElement("namespace"))
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
  else
    gzerr << "[gazebo_imu_plugin] Please specify a namespace.\n";
  nh_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_imu_plugin] Please specify a linkName.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_imu_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  frame_id_ = link_name_;

  airspeed_topic_ = nh_->param<std::string>("airspeedTopic", "airspeed/data");
  pressure_bias_ = nh_->param<double>("pressureBias", 0);
  pressure_noise_sigma_ = nh_->param<double>("pressureNoiseSigma", 0);
  rho_ = nh_->param<double>("airDensity", 1.225);
  max_pressure_ = nh_->param<double>("maxPressure", 4000.0);
  min_pressure_ = nh_->param<double>("minPressure", 0.0);


  last_time_ = world_->GetSimTime();

  // Listen to the update event. This event is broadcast every simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&AirspeedPlugin::OnUpdate, this, _1));

  airspeed_pub_ = nh_->advertise<rosflight_msgs::Airspeed>(airspeed_topic_, 10);

  // Fill static members of airspeed message.
  airspeed_message_.header.frame_id = frame_id_;

  standard_normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);
}

// This gets called by the world update start event.
void AirspeedPlugin::OnUpdate(const common::UpdateInfo& _info) {

  // Calculate Airspeed
  math::Vector3 C_linear_velocity_W_C = link_->GetRelativeLinearVel();
  double u = C_linear_velocity_W_C.x;
  double v = -C_linear_velocity_W_C.y;
  double w = -C_linear_velocity_W_C.z;

  /// TODO: Wind is being applied in the inertial frame, not the body-fixed frame
  double ur = u - wind_.N;
  double vr = v - wind_.E;
  double wr = w - wind_.D;
//  double Va = sqrt(pow(ur,2.0) + pow(vr,2.0) + pow(wr,2.0));
  double Va = sqrt(pow(u,2.0) + pow(v,2.0) + pow(w,2.0));

  // Invert Airpseed to get sensor measurement
  double y = rho_*Va*Va/2.0; // Page 130 in the UAV Book
  y += pressure_bias_ + pressure_noise_sigma_*standard_normal_distribution_(random_generator_);

  y = (y>max_pressure_)?max_pressure_:y;
  y = (y<min_pressure_)?min_pressure_:y;
  airspeed_message_.differential_pressure = y;
  airspeed_message_.temperature = 27.0;
  airspeed_message_.velocity = Va;

  airspeed_pub_.publish(airspeed_message_);
}


GZ_REGISTER_MODEL_PLUGIN(AirspeedPlugin);
}
