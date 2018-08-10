/*
 * Copyright 2016 James Jackson, Brigham Young University - Provo, UT
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

#include "rosflight_plugins/barometer_plugin.h"

namespace rosflight_plugins
{

BarometerPlugin::BarometerPlugin() : gazebo::ModelPlugin() { }

BarometerPlugin::~BarometerPlugin()
{
#if GAZEBO_MAJOR_VERSION >=8
  updateConnection_.reset();
#else
  gazebo::event::Events::DisconnectWorldUpdateBegin(updateConnection_);
#endif
  nh_.shutdown();
}


void BarometerPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    ROS_FATAL("A ROS node for Gazebo has not been initialized, unable to load barometer plugin");
    return;
  }
  ROS_INFO("Loaded the barometer plugin");

  //
  // Configure Gazebo Integration
  //

  model_ = _model;
  world_ = model_->GetWorld();

  last_time_ = GET_SIM_TIME(world_);

  namespace_.clear();
  

  //
  // Get elements from the robot urdf/sdf file
  //

  if (_sdf->HasElement("namespace"))
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
  else
    ROS_ERROR("[barometer_plugin] Please specify a namespace.");

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    ROS_ERROR("[barometer_plugin] Please specify a linkName.");

  link_ = model_->GetLink(link_name_);
  if (link_ == nullptr)
    gzthrow("[barometer_plugin] Couldn't find specified link \"" << link_name_ << "\".");
  

  //
  // ROS Node Setup
  //

  nh_ = ros::NodeHandle(namespace_);
  nh_private_ = ros::NodeHandle(namespace_ + "/barometer");

  // load params from rosparam server
  message_topic_ = nh_private_.param<std::string>("topic", "baro");
  error_stdev_ = nh_private_.param<double>("stdev", 0.10);
  pub_rate_ = nh_private_.param<double>("rate", 50.0);
  noise_on_ = nh_private_.param<bool>("noise_on", true);

  // ROS Publishers
  alt_pub_ = nh_.advertise<rosflight_msgs::Barometer>(message_topic_, 10);

  // Calculate sample time from sensor update rate
  sample_time_ = 1.0/pub_rate_;

  // Configure Noise
  random_generator_ = std::default_random_engine(std::chrono::system_clock::now().time_since_epoch().count());
  standard_normal_distribution_ = std::normal_distribution<double>(0.0, error_stdev_);

  // Listen to the update event. This event is broadcast every simulation iteration.
  this->updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&BarometerPlugin::OnUpdate, this, std::placeholders::_1));
}


void BarometerPlugin::OnUpdate(const gazebo::common::UpdateInfo& _info)
{
  // check if time to publish
  gazebo::common::Time current_time = GET_SIM_TIME(world_);
  if ((current_time - last_time_).Double() >= sample_time_) {

    // pull z measurement out of Gazebo (ENU)
    GazeboPose pose = GET_WORLD_POSE(link_);

    // Create a new barometer message
    rosflight_msgs::Barometer msg;
    msg.altitude = GET_Z(GET_POS(pose));

    // if requested add noise to altitude measurement
    if (noise_on_)
      msg.altitude += standard_normal_distribution_(random_generator_);

    // Invert measurement model for pressure and temperature
    msg.temperature = 25.0; // This is constant for simulation
    msg.pressure = 101325.0*pow(1- (2.25577e-5 * msg.altitude), 5.25588);

    // publish message
    msg.header.stamp.fromSec(GET_SIM_TIME(world_).Double());
    msg.header.frame_id = link_name_;
    alt_pub_.publish(msg);

    // save current time
    last_time_ = current_time;
  }
}

GZ_REGISTER_MODEL_PLUGIN(BarometerPlugin);
}
