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

namespace gazebo {

OdometryPlugin::~OdometryPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (nh_) {
    nh_->shutdown();
    delete nh_;
  }
}


void OdometryPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  odometry_queue_.clear();

  if (_sdf->HasElement("namespace"))
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
  else
    gzerr << "[gazebo_odometry_plugin] Please specify a namespace.\n";
  nh_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("parent_link"))
    link_name_ = _sdf->GetElement("parent_link")->Get<std::string>();
  else
    gzerr << "[gazebo_odometry_plugin] Please specify a parent_link.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_odometry_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  transform_pub_topic_ = nh_->param<std::string>("transform_topic", "transform");
  odometry_pub_topic_ = nh_->param<std::string>("odometry_topic", "odometry");
  parent_frame_id_ = nh_->param<std::string>("frame_id", "world");

  parent_link_ = world_->GetEntity(parent_frame_id_);
  if (parent_link_ == NULL && parent_frame_id_ != "world") {
    gzthrow("[gazebo_odometry_plugin] Couldn't find specified parent link \"" << parent_frame_id_ << "\".");
  }

  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&OdometryPlugin::OnUpdate, this, _1));
  transform_NED_pub_ = nh_->advertise<geometry_msgs::TransformStamped>(transform_pub_topic_ + "/NED", 10);
  transform_NWU_pub_ = nh_->advertise<geometry_msgs::TransformStamped>(transform_pub_topic_ + "/NWU", 10);
  odometry_NED_pub_ = nh_->advertise<nav_msgs::Odometry>(odometry_pub_topic_ + "/NED", 10);
  odometry_NWU_pub_ = nh_->advertise<nav_msgs::Odometry>(odometry_pub_topic_+ "/NWU", 10);
  euler_pub_ = nh_->advertise<geometry_msgs::Vector3Stamped>("euler", 1);
}

// This gets called by the world update start event.
void OdometryPlugin::OnUpdate(const common::UpdateInfo& _info) {
  // C denotes child frame, P parent frame, and W world frame.
  // Further C_pose_W_P denotes pose of P wrt. W expressed in C.
  math::Pose inertial_pose = link_->GetWorldCoGPose();
  math::Vector3 body_fixed_linear_velocity = link_->GetRelativeLinearVel();
  math::Vector3 body_fixed_angular_velocity = link_->GetRelativeAngularVel();

  nav_msgs::Odometry odometry_NED, odometry_NWU;
  geometry_msgs::TransformStamped transform_NED, transform_NWU;
  odometry_NWU.header.stamp.sec = (world_->GetSimTime()).sec;
  odometry_NWU.header.stamp.nsec = (world_->GetSimTime()).nsec;
  odometry_NWU.header.frame_id = "world_NWU";
  odometry_NWU.child_frame_id = namespace_;

  // Set the NWU odometry and transform messages
  odometry_NWU.pose.pose.position.x = inertial_pose.pos.x;
  odometry_NWU.pose.pose.position.y = inertial_pose.pos.y;
  odometry_NWU.pose.pose.position.z = inertial_pose.pos.z;
  odometry_NWU.pose.pose.orientation.w = inertial_pose.rot.w;
  odometry_NWU.pose.pose.orientation.x = inertial_pose.rot.x;
  odometry_NWU.pose.pose.orientation.y = inertial_pose.rot.y;
  odometry_NWU.pose.pose.orientation.z = inertial_pose.rot.z;
  odometry_NWU.twist.twist.linear.x = body_fixed_linear_velocity.x;
  odometry_NWU.twist.twist.linear.y = body_fixed_linear_velocity.y;
  odometry_NWU.twist.twist.linear.z = body_fixed_linear_velocity.z;
  odometry_NWU.twist.twist.angular.x = body_fixed_angular_velocity.x;
  odometry_NWU.twist.twist.angular.y = body_fixed_angular_velocity.y;
  odometry_NWU.twist.twist.angular.z = body_fixed_angular_velocity.z;
  odometry_NWU_pub_.publish(odometry_NWU);

  transform_NWU.header = odometry_NWU.header;
  transform_NWU.transform.translation.x = inertial_pose.pos.x;
  transform_NWU.transform.translation.y = inertial_pose.pos.y;
  transform_NWU.transform.translation.z = inertial_pose.pos.z;
  transform_NWU.transform.rotation.w = inertial_pose.rot.w;
  transform_NWU.transform.rotation.x = inertial_pose.rot.x;
  transform_NWU.transform.rotation.y = inertial_pose.rot.y;
  transform_NWU.transform.rotation.z = inertial_pose.rot.z;
  transform_NWU_pub_.publish(transform_NWU);

  // Convert from NWU to NED
  odometry_NED.header.stamp.sec = (world_->GetSimTime()).sec;
  odometry_NED.header.stamp.nsec = (world_->GetSimTime()).nsec;
  odometry_NED.header.frame_id = "world_NED";
  odometry_NED.child_frame_id = namespace_;
  odometry_NED.pose.pose.position.x = inertial_pose.pos.x;
  odometry_NED.pose.pose.position.y = -inertial_pose.pos.y;
  odometry_NED.pose.pose.position.z = -inertial_pose.pos.z;
  odometry_NED.pose.pose.orientation.w = inertial_pose.rot.w;
  odometry_NED.pose.pose.orientation.x = inertial_pose.rot.x;
  odometry_NED.pose.pose.orientation.y = -inertial_pose.rot.y;
  odometry_NED.pose.pose.orientation.z = -inertial_pose.rot.z;
  odometry_NED.twist.twist.linear.x = body_fixed_linear_velocity.x;
  odometry_NED.twist.twist.linear.y = -body_fixed_linear_velocity.y;
  odometry_NED.twist.twist.linear.z = -body_fixed_linear_velocity.z;
  odometry_NED.twist.twist.angular.x = body_fixed_angular_velocity.x;
  odometry_NED.twist.twist.angular.y = -body_fixed_angular_velocity.y;
  odometry_NED.twist.twist.angular.z = -body_fixed_angular_velocity.z;
  odometry_NED_pub_.publish(odometry_NED);

  transform_NED.header = odometry_NED.header;
  transform_NED.transform.translation.x = inertial_pose.pos.x;
  transform_NED.transform.translation.y = -inertial_pose.pos.y;
  transform_NED.transform.translation.z = -inertial_pose.pos.z;
  transform_NED.transform.rotation.w = inertial_pose.rot.w;
  transform_NED.transform.rotation.x = inertial_pose.rot.x;
  transform_NED.transform.rotation.y = -inertial_pose.rot.y;
  transform_NED.transform.rotation.z = -inertial_pose.rot.z;
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
