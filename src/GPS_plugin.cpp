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
#include "rosflight_plugins/gazebo_compat.h"

namespace rosflight_plugins
{


GPSPlugin::GPSPlugin() : ModelPlugin() {}


GPSPlugin::~GPSPlugin()
{
  GZ_COMPAT_DISCONNECT_WORLD_UPDATE_BEGIN(updateConnection_);
  nh_.shutdown();
}


void GPSPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    ROS_FATAL("A ROS node for Gazebo has not been initialized, unable to load GPS plugin");
    return;
  }
  ROS_INFO("Loaded the GPS plugin");

  //
  // Configure Gazebo Integration
  //

  model_ = _model;
  world_ = model_->GetWorld();

  last_time_ = GZ_COMPAT_GET_SIM_TIME(world_);

  namespace_.clear();

  //
  // Get elements from the robot urdf/sdf file
  //

  if (_sdf->HasElement("namespace"))
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
  else
    ROS_ERROR("[GPS_plugin] Please specify a namespace.");

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    ROS_ERROR("[GPS_plugin] Please specify a linkName.");

  link_ = model_->GetLink(link_name_);
  if (link_ == nullptr)
    gzthrow("[GPS_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  //
  // ROS Node Setup
  //

  nh_ = ros::NodeHandle(namespace_);
  nh_private_ = ros::NodeHandle(namespace_ + "/gps");

  // load params from rosparam server
  int numSat;
  noise_on_ = nh_private_.param<bool>("noise_on", true);
  GPS_topic_ = nh_private_.param<std::string>("topic", "gps");
  north_stdev_ = nh_private_.param<double>("north_stdev", 0.21);
  east_stdev_ = nh_private_.param<double>("east_stdev", 0.21);
  alt_stdev_ = nh_private_.param<double>("alt_stdev", 0.40);
  north_k_GPS_ = nh_private_.param<double>("k_north", 1.0/1100.0);
  east_k_GPS_ = nh_private_.param<double>("k_east", 1.0/1100.0);
  alt_k_GPS_ = nh_private_.param<double>("k_alt", 1.0/1100.0);
  pub_rate_ = nh_private_.param<double>("rate", 10.0);
  initial_latitude_ = nh_private_.param<double>("initial_latitude", 40.267320); // default to Provo, UT
  initial_longitude_ = nh_private_.param<double>("initial_longitude", -111.635629); // default to Provo, UT
  initial_altitude_ = nh_private_.param<double>("initial_altitude", 1387.0); // default to Provo, UT
  numSat = nh_private_.param<int>("num_sats", 7);

  // ROS Publishers
  GPS_pub_ = nh_.advertise<rosflight_msgs::GPS>(GPS_topic_, 1);

  // Calculate sample time from sensor update rate
  sample_time_ = 1.0/pub_rate_;

  // Configure Noise
  random_generator_ = std::default_random_engine(std::chrono::system_clock::now().time_since_epoch().count());
  standard_normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);

  // disable noise by zeroing the standard deviation of the noise
  if (!noise_on_)
  {
    north_stdev_ = 0;
    east_stdev_ = 0;
    alt_stdev_ = 0;
    north_k_GPS_ = 0;
    east_k_GPS_ = 0;
    alt_k_GPS_ = 0;
  }

  // Fill static members of GPS message.
  GPS_message_.header.frame_id = link_name_;
  GPS_message_.fix = true;
  GPS_message_.NumSat = numSat;

  // initialize GPS error to zero
  north_GPS_error_ = 0.0;
  east_GPS_error_ = 0.0;
  alt_GPS_error_ = 0.0;

  // Listen to the update event. This event is broadcast every simulation iteration.
  this->updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&GPSPlugin::OnUpdate, this, std::placeholders::_1));
}

// This gets called by the world update start event.
void GPSPlugin::OnUpdate(const gazebo::common::UpdateInfo& _info)
{
  // check if time to publish
  gazebo::common::Time current_time = GZ_COMPAT_GET_SIM_TIME(world_);
  if ((current_time - last_time_).Double() >= sample_time_) {

      // Add noise per Gauss-Markov Process (p. 139 UAV Book)
      double noise = north_stdev_*standard_normal_distribution_(random_generator_);
      north_GPS_error_ = exp(-1.0*north_k_GPS_*sample_time_)*north_GPS_error_ + noise;

      noise = east_stdev_*standard_normal_distribution_(random_generator_);
      east_GPS_error_ = exp(-1.0*east_k_GPS_*sample_time_)*east_GPS_error_ + noise;

      noise = alt_stdev_*standard_normal_distribution_(random_generator_);
      alt_GPS_error_ = exp(-1.0*alt_k_GPS_*sample_time_)*alt_GPS_error_ + noise;

      // Find NED position in meters
      GazeboPose W_pose_W_C = GZ_COMPAT_GZ_COMPAT_GET_WORLD_COG_POSE(link_);
      double pn =  GZ_COMPAT_GET_X(GZ_COMPAT_GET_POS(W_pose_W_C)) + north_GPS_error_;
      double pe = -GZ_COMPAT_GET_Y(GZ_COMPAT_GET_POS(W_pose_W_C)) + east_GPS_error_;
      double h  =  GZ_COMPAT_GET_Z(GZ_COMPAT_GET_POS(W_pose_W_C)) + alt_GPS_error_;

      // Convert meters to GPS angle
      double dlat, dlon;
      measure(pn, pe, dlat, dlon);
      GPS_message_.latitude = initial_latitude_ + dlat * 180.0/M_PI;
      GPS_message_.longitude = initial_longitude_ + dlon * 180.0/M_PI;

      // Altitude
      GPS_message_.altitude = initial_altitude_ + h;

      // Get Ground Speed
      GazeboVector C_linear_velocity_W_C = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);
      double u = GZ_COMPAT_GET_X(C_linear_velocity_W_C);
      double v = -GZ_COMPAT_GET_Y(C_linear_velocity_W_C);
      double Vg = sqrt(u*u + v*v);
      double sigma_vg = sqrt((u*u*north_stdev_*north_stdev_ + v*v*east_stdev_*east_stdev_)/(u*u + v*v));
      double ground_speed_error = sigma_vg*standard_normal_distribution_(random_generator_);
      GPS_message_.speed = Vg + ground_speed_error;

      // Get Course Angle
      double psi = -GZ_COMPAT_GET_Z( GZ_COMPAT_GET_EULER( GZ_COMPAT_GET_ROT( W_pose_W_C)));
      double dx = Vg*cos(psi);
      double dy = Vg*sin(psi);
      double chi = atan2(dy,dx);
      double sigma_chi = sqrt((dx*dx*north_stdev_*north_stdev_ + dy*dy*east_stdev_*east_stdev_)/((dx*dx+dy*dy)*(dx*dx+dy*dy)));
      double chi_error = sigma_chi*standard_normal_distribution_(random_generator_);
      GPS_message_.ground_course = chi + chi_error;

      // Publish
      GPS_message_.header.stamp.fromSec(GZ_COMPAT_GET_SIM_TIME(world_).Double());
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
  dlon = asin(dpe/(R*cos(initial_latitude_*M_PI/180.0)));
}


GZ_REGISTER_MODEL_PLUGIN(GPSPlugin);
}
