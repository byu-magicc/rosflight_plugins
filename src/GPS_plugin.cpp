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
#include "rosflight_plugins/gz_compat.h"
#include <sensor_msgs/NavSatStatus.h>

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
  gnss_topic_ = nh_private_.param<std::string>("topic", "gnss");
  gnss_fix_topic_ = nh_private_.param<std::string>("fix_topic", "navsat_compat/fix");
  gnss_vel_topic_ = nh_private_.param<std::string>("vel_topic", "navsat_compat/vel");
  north_stdev_ = nh_private_.param<double>("north_stdev", 0.21);
  east_stdev_ = nh_private_.param<double>("east_stdev", 0.21);
  alt_stdev_ = nh_private_.param<double>("alt_stdev", 0.40);
  velocity_stdev_ = nh_private_.param<double>("velocity_stdev", 0.30);
  north_k_GPS_ = nh_private_.param<double>("k_north", 1.0/1100.0);
  east_k_GPS_ = nh_private_.param<double>("k_east", 1.0/1100.0);
  alt_k_GPS_ = nh_private_.param<double>("k_alt", 1.0/1100.0);
  pub_rate_ = nh_private_.param<double>("rate", 10.0);
  initial_latitude_ = nh_private_.param<double>("initial_latitude", 40.267320); // default to Provo, UT
  initial_longitude_ = nh_private_.param<double>("initial_longitude", -111.635629); // default to Provo, UT
  initial_altitude_ = nh_private_.param<double>("initial_altitude", 1387.0); // default to Provo, UT
  numSat = nh_private_.param<int>("num_sats", 7);

  // ROS Publishers
  GNSS_pub_ = nh_.advertise<rosflight_msgs::GNSS>(gnss_topic_, 1);
  GNSS_fix_pub_ = nh_.advertise<sensor_msgs::NavSatFix>(gnss_fix_topic_, 1);
  GNSS_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(gnss_vel_topic_, 1);


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
    velocity_stdev_ = 0;
    north_k_GPS_ = 0;
    east_k_GPS_ = 0;
    alt_k_GPS_ = 0;
  }

  // Fill static members of GPS message.
  gnss_message_.header.frame_id = link_name_;
  //TODO add constants for UBX fix types
  gnss_message_.fix = 3; // corresponds to a 3D fix

  gnss_fix_message_.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  gnss_fix_message_.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;

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
      GazeboPose W_pose_W_C = GZ_COMPAT_GET_WORLD_COG_POSE(link_);
      double pn =  GZ_COMPAT_GET_X(GZ_COMPAT_GET_POS(W_pose_W_C)) + north_GPS_error_;
      double pe = -GZ_COMPAT_GET_Y(GZ_COMPAT_GET_POS(W_pose_W_C)) + east_GPS_error_;
      double h  =  GZ_COMPAT_GET_Z(GZ_COMPAT_GET_POS(W_pose_W_C)) + alt_GPS_error_;

      // Convert meters to GPS angle
      double dlat, dlon;
      measure(pn, pe, dlat, dlon);
      double latitude_deg = initial_latitude_ + dlat * 180.0/M_PI;
      double longitude_deg = initial_longitude_ + dlon * 180.0/M_PI;

      // Altitude
      double altitude = initial_altitude_ + h;

      // Get Ground Speed
      ignition::math::Vector3d C_linear_velocity_W_C = GZ_COMPAT_IGN_VECTOR(GZ_COMPAT_GET_WORLD_LINEAR_VEL(link_));
      double u = C_linear_velocity_W_C.X();
      double v = -C_linear_velocity_W_C.Y();
      double Vg = sqrt(u*u + v*v);
      double sigma_vg = sqrt((u*u*north_stdev_*north_stdev_ + v*v*east_stdev_*east_stdev_)/(u*u + v*v));
      double ground_speed_error = sigma_vg*standard_normal_distribution_(random_generator_);
      double ground_speed = Vg + ground_speed_error;

      // Get Course Angle
      double psi = -GZ_COMPAT_GET_Z( GZ_COMPAT_GET_EULER( GZ_COMPAT_GET_ROT( W_pose_W_C)));
      double dx = Vg*cos(psi);
      double dy = Vg*sin(psi);
      double chi = atan2(dy,dx);
      double sigma_chi = sqrt((dx*dx*north_stdev_*north_stdev_ + dy*dy*east_stdev_*east_stdev_)/((dx*dx+dy*dy)*(dx*dx+dy*dy)));
      double chi_error = sigma_chi*standard_normal_distribution_(random_generator_);
      double ground_course_rad = chi + chi_error;

      //Calculate other values for messages
      ignition::math::Angle lat_angle(deg_to_rad(initial_latitude_));
      ignition::math::Angle lon_angle(deg_to_rad(initial_longitude_));
      gazebo::common::SphericalCoordinates spherical_coordinates(
          gazebo::common::SphericalCoordinates::SurfaceType::EARTH_WGS84, lat_angle, lon_angle, initial_altitude_,
          ignition::math::Angle::Zero);
      ignition::math::Vector3d lla_position_with_error(deg_to_rad(latitude_deg), deg_to_rad(longitude_deg), altitude);
      ignition::math::Vector3d ecef_position = spherical_coordinates.PositionTransform(lla_position_with_error,
                                                                           gazebo::common::SphericalCoordinates::CoordinateType::SPHERICAL,
                                                                           gazebo::common::SphericalCoordinates::CoordinateType::ECEF);

      ignition::math::Vector3d nwu_vel = GZ_COMPAT_IGN_VECTOR(GZ_COMPAT_GET_WORLD_LINEAR_VEL(link_));
      ignition::math::Vector3d enu_vel(-nwu_vel.Y(), nwu_vel.X(), nwu_vel.Z());
      ignition::math::Vector3d enu_vel_with_noise;
      enu_vel_with_noise.X() += velocity_stdev_ * standard_normal_distribution_(random_generator_);
      enu_vel_with_noise.Y() += velocity_stdev_ * standard_normal_distribution_(random_generator_);
      enu_vel_with_noise.Z() += velocity_stdev_ * standard_normal_distribution_(random_generator_);
      ignition::math::Vector3d ecef_velocity = spherical_coordinates.VelocityTransform(enu_vel_with_noise,
                                                                           gazebo::common::SphericalCoordinates::CoordinateType::GLOBAL,
                                                                           gazebo::common::SphericalCoordinates::CoordinateType::ECEF);

      //Fill the GNSS message
      gnss_message_.position[0] = ecef_position.X();
      gnss_message_.position[1] = ecef_position.Y();
      gnss_message_.position[2] = ecef_position.Z();
      gnss_message_.speed_accuracy = sigma_vg;
      gnss_message_.vertical_accuracy = alt_stdev_;
      gnss_message_.horizontal_accuracy = north_stdev_ > east_stdev_ ? north_stdev_ : east_stdev_;
      gnss_message_.velocity[0] = ecef_velocity.X();
      gnss_message_.velocity[1] = ecef_velocity.Y();
      gnss_message_.velocity[2] = ecef_velocity.Z();

      //Fill the NavSatFix message
      gnss_fix_message_.latitude = latitude_deg;
      gnss_fix_message_.longitude = longitude_deg;
      gnss_fix_message_.altitude = altitude;
      gnss_fix_message_.position_covariance[0] = north_stdev_;
      gnss_fix_message_.position_covariance[4] = east_stdev_;
      gnss_fix_message_.position_covariance[8] = alt_stdev_;
      gnss_fix_message_.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

      //Fill the TwistStamped
      gnss_vel_message_.twist.linear.x = ground_speed * cos(ground_course_rad);
      gnss_vel_message_.twist.linear.y = ground_speed * sin(ground_course_rad);
      //Apparently GPS units don't report vertical speed?
      gnss_vel_message_.twist.linear.z = 0;

      //Time stamps
      gnss_message_.header.stamp.fromSec(GZ_COMPAT_GET_SIM_TIME(world_).Double());
      gnss_message_.time = gnss_message_.header.stamp;
      gnss_vel_message_.header.stamp = gnss_message_.header.stamp;
      gnss_fix_message_.header.stamp = gnss_message_.header.stamp;

      // Publish
      GNSS_pub_.publish(gnss_message_);
      GNSS_fix_pub_.publish(gnss_fix_message_);
      GNSS_vel_pub_.publish(gnss_vel_message_);

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
