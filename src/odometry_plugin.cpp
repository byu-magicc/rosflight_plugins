/*
 * Copyright 2017 James Jackson Brigham Young University
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


#include "rosflight_plugins/odometry_plugin.h"

namespace rosflight_plugins
{

OdometryPlugin::~OdometryPlugin() {
#if GAZEBO_MAJOR_VERSION >=8
  updateConnection_.reset();
#else
  gazebo::event::Events::DisconnectWorldUpdateBegin(updateConnection_);
#endif
  nh_.shutdown();
}


void OdometryPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    ROS_FATAL("A ROS node for Gazebo has not been initialized, unable to load odometry plugin");
    return;
  }
  ROS_INFO("Loaded the odometry plugin");

  //
  // Configure Gazebo Integration
  //

  model_ = _model;
  world_ = model_->GetWorld();

  namespace_.clear();

  //
  // Get elements from the robot urdf/sdf file
  //

  if (_sdf->HasElement("namespace"))
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
  else
    ROS_ERROR("[odometry_plugin] Please specify a namespace.");

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    ROS_ERROR("[odometry_plugin] Please specify a linkName.");

  link_ = model_->GetLink(link_name_);
  if (link_ == nullptr)
    gzthrow("[odometry_plugin] Couldn't find specified link \"" << link_name_ << "\".");
  

  //
  // ROS Node Setup
  //

  nh_ = ros::NodeHandle(namespace_);
  nh_private_ = ros::NodeHandle(namespace_ + "/odometry");

  // load params from rosparam server
  transform_pub_topic_ = nh_private_.param<std::string>("transform_topic", "transform");
  odometry_pub_topic_ = nh_private_.param<std::string>("odometry_topic", "odometry");
  parent_frame_id_ = nh_private_.param<std::string>("frame_id", "world");

#if GAZEBO_MAJOR_VERSION >=8
  parent_link_ = world_->EntityByName(parent_frame_id_);
#else
  parent_link_ = world_->GetEntity(parent_frame_id_);
#endif
  if (parent_link_ == nullptr && parent_frame_id_ != "world")
    gzthrow("[gazebo_odometry_plugin] Couldn't find specified parent link \"" << parent_frame_id_ << "\".");
 
  // ROS Publishers
  transform_NED_pub_ = nh_.advertise<geometry_msgs::TransformStamped>(transform_pub_topic_ + "/NED", 10);
  transform_NWU_pub_ = nh_.advertise<geometry_msgs::TransformStamped>(transform_pub_topic_ + "/NWU", 10);
  odometry_NED_pub_ = nh_.advertise<nav_msgs::Odometry>(odometry_pub_topic_ + "/NED", 10);
  odometry_NWU_pub_ = nh_.advertise<nav_msgs::Odometry>(odometry_pub_topic_+ "/NWU", 10);
  euler_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("euler", 1);

  // Listen to the update event. This event is broadcast every simulation iteration.
  this->updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&OdometryPlugin::OnUpdate, this, std::placeholders::_1));
}


void OdometryPlugin::OnUpdate(const gazebo::common::UpdateInfo& _info) {
  // C denotes child frame, P parent frame, and W world frame.
  // Further C_pose_W_P denotes pose of P wrt. W expressed in C.
  GazeboPose inertial_pose = GET_COG_POSE(link_);
  GazeboVector body_fixed_linear_velocity = GET_RELATIVE_LINEAR_VEL(link_);
  GazeboVector body_fixed_angular_velocity = GET_RELATIVE_ANGULAR_VEL(link_);

  nav_msgs::Odometry odometry_NED, odometry_NWU;
  geometry_msgs::TransformStamped transform_NED, transform_NWU;
  odometry_NWU.header.stamp.sec = (GET_SIM_TIME(world_).sec;
  odometry_NWU.header.stamp.nsec = (GET_SIM_TIME(world_).nsec;
  odometry_NWU.header.frame_id = "world_NWU";
  odometry_NWU.child_frame_id = namespace_;

  // Set the NWU odometry and transform messages
  odometry_NWU.pose.pose.position.x = GET_X(GET_POS(inertial_pose));
  odometry_NWU.pose.pose.position.y = GET_Y(GET_POS(inertial_pose));
  odometry_NWU.pose.pose.position.z = GET_Z(GET_POS(inertial_pose));
  odometry_NWU.pose.pose.orientation.w = GET_W(GET_ROT(inertial_pose));
  odometry_NWU.pose.pose.orientation.x = GET_X(GET_ROT(inertial_pose));
  odometry_NWU.pose.pose.orientation.y = GET_Y(GET_ROT(inertial_pose));
  odometry_NWU.pose.pose.orientation.z = GET_Z(GET_ROT(inertial_pose));
  odometry_NWU.twist.twist.linear.x = GET_X(body_fixed_linear_velocity);
  odometry_NWU.twist.twist.linear.y = GET_Y(body_fixed_linear_velocity);
  odometry_NWU.twist.twist.linear.z = GET_Z(body_fixed_linear_velocity);
  odometry_NWU.twist.twist.angular.x = GET_X(body_fixed_angular_velocity);
  odometry_NWU.twist.twist.angular.y = GET_Y(body_fixed_angular_velocity);
  odometry_NWU.twist.twist.angular.z = GET_Z(body_fixed_angular_velocity);
  odometry_NWU_pub_.publish(odometry_NWU);

  transform_NWU.header = odometry_NWU.header;
  transform_NWU.transform.translation.x = GET_X(GET_POS(inertial_pose));
  transform_NWU.transform.translation.y = GET_Y(GET_POS(inertial_pose));
  transform_NWU.transform.translation.z = GET_Z(GET_POS(inertial_pose));
  transform_NWU.transform.rotation.w = GET_W(GET_ROT(inertial_pose));
  transform_NWU.transform.rotation.x = GET_X(GET_ROT(inertial_pose));
  transform_NWU.transform.rotation.y = GET_Y(GET_ROT(inertial_pose));
  transform_NWU.transform.rotation.z = GET_Z(GET_ROT(inertial_pose));
  transform_NWU_pub_.publish(transform_NWU);

  // Convert from NWU to NED
  odometry_NED.header.stamp.sec = (GET_SIM_TIME(world_)).sec;
  odometry_NED.header.stamp.nsec = (GET_SIM_TIME(world_)).nsec;

  odometry_NED.header.frame_id = "world_NED";
  odometry_NED.child_frame_id = namespace_;

  odometry_NED.pose.pose.position.x = GET_X(GET_POS(inertial_pose));
  odometry_NED.pose.pose.position.y = -GET_Y(GET_POS(inertial_pose));
  odometry_NED.pose.pose.position.z = -GET_Z(GET_POS(inertial_pose));
  odometry_NED.pose.pose.orientation.w = GET_W(GET_ROT(inertial_pose));
  odometry_NED.pose.pose.orientation.x = GET_X(GET_ROT(inertial_pose));
  odometry_NED.pose.pose.orientation.y = -GET_Y(GET_ROT(inertial_pose));
  odometry_NED.pose.pose.orientation.z = -GET_Z(GET_ROT(inertial_pose));
  odometry_NED.twist.twist.linear.x = GET_X(body_fixed_linear_velocity);
  odometry_NED.twist.twist.linear.y = -GET_Y(body_fixed_linear_velocity);
  odometry_NED.twist.twist.linear.z = -GET_Z(body_fixed_linear_velocity);
  odometry_NED.twist.twist.angular.x = GET_X(body_fixed_angular_velocity);
  odometry_NED.twist.twist.angular.y = -GET_Y(body_fixed_angular_velocity);
  odometry_NED.twist.twist.angular.z = -GET_Z(body_fixed_angular_velocity);
  odometry_NED_pub_.publish(odometry_NED);

  transform_NED.header = odometry_NED.header;
  transform_NED.transform.translation.x = GET_X(GET_POS(inertial_pose));
  transform_NED.transform.translation.y = -GET_Y(GET_POS(inertial_pose));
  transform_NED.transform.translation.z = -GET_Z(GET_POS(inertial_pose));
  transform_NED.transform.rotation.w = GET_W(GET_ROT(inertial_pose));
  transform_NED.transform.rotation.x = GET_X(GET_ROT(inertial_pose));
  transform_NED.transform.rotation.y = -GET_Y(GET_ROT(inertial_pose));
  transform_NED.transform.rotation.z = -GET_Z(GET_ROT(inertial_pose));
  transform_NED_pub_.publish(transform_NED);

  // Publish all the topics, for which the topic name is specified.
  if (euler_pub_.getNumSubscribers() > 0) {
    geometry_msgs::Vector3Stamped euler;
    tf::Quaternion q;
    tf::quaternionMsgToTF(odometry_NED.pose.pose.orientation, q);
    tf::Matrix3x3 R(q);
    double roll, pitch, yaw;
    R.getEulerYPR(yaw, pitch, roll);
    euler.header = odometry_NED.header;
    euler.vector.x = roll;
    euler.vector.y = pitch;
    euler.vector.z = yaw;
    euler_pub_.publish(euler);
  }
}

GZ_REGISTER_MODEL_PLUGIN(OdometryPlugin);
}
