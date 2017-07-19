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

#include "rosflight_plugins/GPS_plugin.h"


namespace gazebo 
{


GPSPlugin::GPSPlugin() : ModelPlugin() {}


GPSPlugin::~GPSPlugin() 
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (nh_) {
    nh_->shutdown();
    delete nh_;
  }
}


void GPSPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) 
{
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

  int numSat;
  getSdfParam<std::string>(_sdf, "GPSTopic", GPS_topic_, "gps/data");
  getSdfParam<double>(_sdf, "north_stdev", north_stdev_, 0.21);
  getSdfParam<double>(_sdf, "east_stdev", east_stdev_, 0.21);
  getSdfParam<double>(_sdf, "alt_stdev", alt_stdev_, 0.40);
  getSdfParam<double>(_sdf, "north_k_GPS", north_k_GPS_, 1.0/1100.0);
  getSdfParam<double>(_sdf, "east_k_GPS", east_k_GPS_, 1.0/1100.0);
  getSdfParam<double>(_sdf, "alt_k_GPS", alt_k_GPS_, 1.0/1100.0);
  getSdfParam<double>(_sdf, "sampleTime", sample_time_, 1.0);
  getSdfParam<double>(_sdf, "initialLatitude", initial_latitude_, 1.0);
  getSdfParam<double>(_sdf, "initialLongitude", initial_longitude_, 1.0);
  getSdfParam<double>(_sdf, "initialAltitude", initial_altitude_, 1.0);
  getSdfParam<int>(_sdf, "numSat", numSat, 7);

  last_time_ = world_->GetSimTime();

  // Listen to the update event. This event is broadcast every simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GPSPlugin::OnUpdate, this, _1));

  GPS_pub_ = nh_->advertise<rosflight_msgs::GPS>(GPS_topic_, 1);
  pub_rate_ = 1.0/sample_time_;

  // Fill static members of airspeed message.
  GPS_message_.header.frame_id = frame_id_;
  GPS_message_.fix = true;
  GPS_message_.NumSat = numSat;

  standard_normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);

  north_GPS_error_ = 0.0;
  east_GPS_error_ = 0.0;
  alt_GPS_error_ = 0.0;
//  gzerr << " finished GPS initializaiton \n" ;
}

// This gets called by the world update start event.
void GPSPlugin::OnUpdate(const common::UpdateInfo& _info) 
{

  // check if time to publish
  common::Time current_time  = world_->GetSimTime();
  if((current_time - last_time_).Double() > 1.0/pub_rate_){

      // Add noise per Gauss-Markov Process (p. 139 UAV Book)
      double noise = north_stdev_*standard_normal_distribution_(random_generator_);
      north_GPS_error_ = exp(-1.0*north_k_GPS_*sample_time_)*north_GPS_error_ + noise;

      noise = east_stdev_*standard_normal_distribution_(random_generator_);
      east_GPS_error_ = exp(-1.0*east_k_GPS_*sample_time_)*east_GPS_error_ + noise;

      noise = alt_stdev_*standard_normal_distribution_(random_generator_);
      alt_GPS_error_ = exp(-1.0*alt_k_GPS_*sample_time_)*alt_GPS_error_ + noise;

      // Find NED position in meters
      math::Pose W_pose_W_C = link_->GetWorldCoGPose();
      double pn =  W_pose_W_C.pos.x + north_GPS_error_;
      double pe = -W_pose_W_C.pos.y + east_GPS_error_;
      double h  =  W_pose_W_C.pos.z + alt_GPS_error_;

      // Convert meters to GPS angle
      double dlat, dlon;
      measure(pn, pe, dlat, dlon);
      GPS_message_.latitude = initial_latitude_ + dlat * 180.0/M_PI;
      GPS_message_.longitude = initial_longitude_ + dlon * 180.0/M_PI;

      // Altitude
      GPS_message_.altitude = initial_altitude_ + h;

      // Get Ground Speed
      math::Vector3 C_linear_velocity_W_C = link_->GetRelativeLinearVel();
      double u = C_linear_velocity_W_C.x;
      double v = -C_linear_velocity_W_C.y;
      double Vg = pow(u*u+v*v,0.5);
      double sigma_vg = pow((u*u*north_stdev_*north_stdev_ + v*v*east_stdev_*east_stdev_)/(u*u+v*v),0.5);
      double ground_speed_error = sigma_vg*standard_normal_distribution_(random_generator_);
      GPS_message_.speed = Vg + ground_speed_error;

      // Get Course Angle
      double chi = atan2(v,u);
      double sigma_chi = pow((u*u*north_stdev_*north_stdev_ + v*v*east_stdev_*east_stdev_)/((u*u+v*v)*(u*u+v*v)),0.5);
      double chi_error = sigma_chi*standard_normal_distribution_(random_generator_);
      GPS_message_.ground_course = chi + chi_error;

      // Publish
      GPS_message_.header.stamp.fromSec(world_->GetSimTime().Double());
      GPS_pub_.publish(GPS_message_);

      last_time_ = current_time;
  }

}


// this assumes a plane tangent to spherical earth at initial lat/lon
void GPSPlugin::measure(double dpn, double dpe, double& dlat, double& dlon)
{
  static const double Re = 6371000.0; // radius of earth in meters
  double R = Re + initial_altitude_; // current radius from earth center

  dlat = asin(dpn/R);
  dlon = asin(dpe/(R*cos(dlat)));
}


GZ_REGISTER_MODEL_PLUGIN(GPSPlugin);
}
