/*
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
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

#ifndef ROTORS_GAZEBO_PLUGINS_MSG_INTERFACE_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_MSG_INTERFACE_PLUGIN_H

// SYSTEM INCLUDES
#include <random>
#include <thread>

#include <Eigen/Eigen>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>

//=================== ROS =====================//
#include <ros/ros.h>
#include <ros/advertise_options.h>

//============= GAZEBO MSG TYPES ==============//
#include <ConnectGazeboToRosTopic.pb.h>
#include <ConnectRosToGazeboTopic.pb.h>

#include <CommandMotorThrottle.pb.h>
#include <Odometry.pb.h>

//=============== ROS MSG TYPES ===============//
#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>

#include "common.h"

namespace gazebo {

// typedef's to make life easier
typedef const boost::shared_ptr<const std_msgs::msgs::ConnectGazeboToRosTopic> GzConnectGazeboToRosTopicMsgPtr;
typedef const boost::shared_ptr<const std_msgs::msgs::ConnectRosToGazeboTopic> GzConnectRosToGazeboTopicMsgPtr;
typedef const boost::shared_ptr<const nav_msgs::msgs::Odometry> GzOdometryMsgPtr;
typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorThrottle> GzCommandMotorThrottleMsgPtr;

// \brief    ROS interface plugin for Gazebo.
// \details  This routes messages to/from Gazebo and ROS. This is used
//           so that individual plugins are not ROS dependent.
//           This is a WorldPlugin, only one of these is designed to be enabled
//           per Gazebo world.
class GazeboRosInterfacePlugin : public WorldPlugin {
 public:
  GazeboRosInterfacePlugin();
  ~GazeboRosInterfacePlugin();

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

  // \brief  	This gets called by the world update start event.
  // \details	Calculates IMU parameters and then publishes one IMU message.
  void OnUpdate(const common::UpdateInfo&);

 private:
  // \brief  Provides a way for GzConnectGazeboToRosTopicMsgCallback() to
  //         connect a Gazebo subscriber to a ROS publisher.
  // \details
  //   GazeboMsgT  The type of the message that will be subscribed to the
  //   Gazebo framework.
  //   RosMsgT     The type of the message published to the ROS framework.
  template <typename GazeboMsgT, typename RosMsgT>
  void ConnectHelper(void (GazeboRosInterfacePlugin::*fp)(
                         const boost::shared_ptr<GazeboMsgT const>&,
                         ros::Publisher),
                     GazeboRosInterfacePlugin* ptr, std::string gazeboNamespace,
                     std::string gazeboTopicName, std::string rosTopicName,
                     transport::NodePtr gz_node_handle);

  std::vector<gazebo::transport::NodePtr> nodePtrs_;
  std::vector<gazebo::transport::SubscriberPtr> subscriberPtrs_;

  // \brief  Handle for the Gazebo node.
  transport::NodePtr gz_node_handle_;

  // \brief  Handle for the ROS node.
  ros::NodeHandle* ros_node_handle_;

  // \brief  Pointer to the world.
  physics::WorldPtr world_;

  // \brief  Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  // ============================================ //
  // ====== CONNECT GAZEBO TO ROS MESSAGES ====== //
  // ============================================ //

  transport::SubscriberPtr gz_connect_gazebo_to_ros_topic_sub_;
  void GzConnectGazeboToRosTopicMsgCallback(
      GzConnectGazeboToRosTopicMsgPtr& gz_connect_gazebo_to_ros_topic_msg);

  // ============================================ //
  // ====== CONNECT ROS TO GAZEBO MESSAGES ====== //
  // ============================================ //

  transport::SubscriberPtr gz_connect_ros_to_gazebo_topic_sub_;

  // @brief    Subscribes to the provided ROS topic and publishes on the
  // provided Gazebo topic (all info contained within the message).
  // @details  Will create a Gazebo publisher if one doesn't already exist.
  void GzConnectRosToGazeboTopicMsgCallback(
      GzConnectRosToGazeboTopicMsgPtr& gz_connect_ros_to_gazebo_topic_msg);
      
  // ============================================ //
  // ===== GAZEBO->ROS CALLBACKS/CONVERTERS ===== //
  // ============================================ //

  // ODOMETRY
  void GzOdometryMsgCallback(GzOdometryMsgPtr& gz_odometry_msg,
      ros::Publisher ros_publisher);
  nav_msgs::Odometry ros_odometry_msg_;

  // ============================================ //
  // ===== ROS->GAZEBO CALLBACKS/CONVERTERS ===== //
  // ============================================ //

  // COMMAND MOTOR SPEED (this is the same as ACTUATORS!, merge???)
  void RosCommandMotorThrottleMsgCallback(
      const std_msgs::Float64MultiArrayConstPtr& ros_command_motor_throttle_msg_ptr,
      transport::PublisherPtr gz_publisher_ptr);
};

}  // namespace gazebo

#endif  // #ifndef ROTORS_GAZEBO_PLUGINS_MSG_INTERFACE_PLUGIN_H