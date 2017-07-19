 /*
 * Copyright 2017 James Jackson Brigham Young University MAGICC LAB
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

#include "rosflight_plugins/imu_plugin.h"


using namespace std;

namespace gazebo {

ImuPlugin::ImuPlugin() :
  ModelPlugin(),
  prev_velocity_(0, 0, 0)
{}

ImuPlugin::~ImuPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  nh_.shutdown();
}


void ImuPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  // default params
  namespace_.clear();

  if (_sdf->HasElement("namespace"))
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
  else
    gzerr << "[gazebo_imu_plugin] Please specify a namespace.\n";
  nh_ = ros::NodeHandle(namespace_);

  if (_sdf->HasElement("link_name"))
    link_name_ = _sdf->GetElement("link_name")->Get<std::string>();
  else
    gzerr << "[gazebo_imu_plugin] Please specify a link_name.\n";
  // Get the pointer to the link
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_imu_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  update_rate_ = nh_->param<double>("update_rate", update_rate_, 1000.0);
  imu_topic_ = nh_->param<std::string>("imu_topic", imu_topic_, "imu/data");
  acc_bias_topic_ = nh_->param<std::string>("acc_bias_topic", acc_bias_topic_, "imu/acc_bias");
  gyro_bias_topic_ = nh_->param<std::string>("gyro_bias_topic", gyro_bias_topic_, "imu/gyro_bias");

  gyro_stdev_ = nh_->param<double>("gyro_stdev", gyro_stdev_, 0.13);
  gyro_bias_range_ = nh_->param<double>("gyro_bias_range", gyro_bias_range_, 0.15);
  gyro_bias_walk_stdev_ = nh_->param<double>("gyro_bias_walk_stdev", gyro_bias_walk_stdev_, 0.001);

  acc_stdev_ = nh_->param<double>("acc_stdev", acc_stdev_, 1.15);
  acc_bias_range_ = nh_->param<double>("acc_bias_range", acc_bias_range_, 0.15);
  acc_bias_walk_stdev_ = nh_->param<double>("acc_bias_walk_stdev", acc_bias_walk_stdev_, 0.001);

  // Initialize some parts of the plugin
  last_time_ = world_->GetSimTime();
  gravity_ = world_->GetPhysicsEngine()->GetGravity();
  normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);
  uniform_distribution_ = std::uniform_real_distribution<double>(-1.0, 1.0);

  // Connect update callback
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&ImuPlugin::OnUpdate, this, _1));

  // Set up the publishers
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic_, 10);
  gyro_bias_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(gyro_bias_topic_, 10);
  acc_bias_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(acc_bias_topic_, 10);

  // Set up static members of IMU message
  imu_message_.header.frame_id = link_name_;
  imu_message_.angular_velocity_covariance[0] = gyro_stdev_*gyro_stdev_;
  imu_message_.angular_velocity_covariance[4] = gyro_stdev_*gyro_stdev_;
  imu_message_.angular_velocity_covariance[8] = gyro_stdev_*gyro_stdev_;
  imu_message_.linear_acceleration_covariance[0] = acc_stdev_*acc_stdev_;
  imu_message_.linear_acceleration_covariance[4] = acc_stdev_*acc_stdev_;
  imu_message_.linear_acceleration_covariance[8] = acc_stdev_*acc_stdev_;

  // Initialize the Bias
  gyro_bias_.x = gyro_bias_range_*uniform_distribution_(random_generator_);
  gyro_bias_.y = gyro_bias_range_*uniform_distribution_(random_generator_);
  gyro_bias_.z = gyro_bias_range_*uniform_distribution_(random_generator_);
  acc_bias_.x = acc_bias_range_*uniform_distribution_(random_generator_);
  acc_bias_.y = acc_bias_range_*uniform_distribution_(random_generator_);
  acc_bias_.z = acc_bias_range_*uniform_distribution_(random_generator_);

  // Get mass
  mass_ = link_->GetInertial()->GetMass();
}

void ImuPlugin::Reset()
{
  last_time_ = world_->GetSimTime();
}

// This gets called by the world update start event.
void ImuPlugin::OnUpdate(const common::UpdateInfo& _info)
{
  common::Time current_time  = world_->GetSimTime();
  double dt = (current_time - last_time_).Double();

  if(dt >= 1.0/update_rate_)
  {
    math::Quaternion q_I_NWU = link_->GetWorldPose().rot;
    math::Vector3 omega_B_NWU = link_->GetRelativeAngularVel();
    math::Vector3 uvw_B_NWU = link_->GetRelativeLinearVel();

    // y_acc = F/m - R*g
    math::Vector3 y_acc = link_->GetRelativeForce()/mass_ - q_I_NWU.RotateVectorReverse(gravity_);
    math::Vector3 y_gyro = link_->GetRelativeAngularVel();

    // Apply normal noise
    y_acc.x += acc_stdev_*normal_distribution_(random_generator_);
    y_acc.y += acc_stdev_*normal_distribution_(random_generator_);
    y_acc.z += acc_stdev_*normal_distribution_(random_generator_);
    y_gyro.x += gyro_stdev_*normal_distribution_(random_generator_);
    y_gyro.y += gyro_stdev_*normal_distribution_(random_generator_);
    y_gyro.z += gyro_stdev_*normal_distribution_(random_generator_);

    // Perform Random Walk for biases
    acc_bias_.x += acc_bias_walk_stdev_*normal_distribution_(random_generator_);
    acc_bias_.y += acc_bias_walk_stdev_*normal_distribution_(random_generator_);
    acc_bias_.z += acc_bias_walk_stdev_*normal_distribution_(random_generator_);
    gyro_bias_.x += gyro_bias_walk_stdev_*normal_distribution_(random_generator_);
    gyro_bias_.y += gyro_bias_walk_stdev_*normal_distribution_(random_generator_);
    gyro_bias_.z += gyro_bias_walk_stdev_*normal_distribution_(random_generator_);

    // Add constant Bias to measurement
    y_acc.x += acc_bias_.x;
    y_acc.y += acc_bias_.y;
    y_acc.z += acc_bias_.z;
    y_gyro.x += gyro_bias_.x;
    y_gyro.y += gyro_bias_.y;
    y_gyro.z += gyro_bias_.z;

    // Set Time stamp
    imu_message_.header.stamp.sec = current_time.sec;
    imu_message_.header.stamp.nsec = current_time.nsec;

    // Convert to NED for publishing
    imu_message_.orientation.w = 1;
    imu_message_.orientation.x = 0;
    imu_message_.orientation.y = 0;
    imu_message_.orientation.z = 0;
    imu_message_.orientation.w = q_I_NWU.w;
    imu_message_.orientation.x = q_I_NWU.x;
    imu_message_.orientation.y = -q_I_NWU.y;
    imu_message_.orientation.z = -q_I_NWU.z;

    imu_message_.linear_acceleration.x = y_acc.x;
    imu_message_.linear_acceleration.y = -y_acc.y;
    imu_message_.linear_acceleration.z = -y_acc.z;
    imu_message_.angular_velocity.x = y_gyro.x;
    imu_message_.angular_velocity.y = -y_gyro.y;
    imu_message_.angular_velocity.z = -y_gyro.z;

    imu_pub_.publish(imu_message_);

    last_time_ = current_time;

    // Publish the current value of the biases
    geometry_msgs::Vector3Stamped acc_bias_msg, gyro_bias_msg;
    acc_bias_msg.header = imu_message_.header;
    gyro_bias_msg.header = imu_message_.header;

    // Convert Bias to NED
    acc_bias_msg.vector.x = acc_bias_.x;
    acc_bias_msg.vector.y = -acc_bias_.y;
    acc_bias_msg.vector.z = -acc_bias_.z;
    acc_bias_pub_.publish(acc_bias_msg);

    // Convert Bias to NED
    gyro_bias_msg.vector.x = gyro_bias_.x;
    gyro_bias_msg.vector.y = -gyro_bias_.y;
    gyro_bias_msg.vector.z = -gyro_bias_.z;
    gyro_bias_pub_.publish(gyro_bias_msg);
  }
}


GZ_REGISTER_MODEL_PLUGIN(ImuPlugin);
}
