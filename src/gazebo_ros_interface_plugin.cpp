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

// MODULE
#include <gazebo_ros_interface_plugin.h>

namespace gazebo {

GazeboRosInterfacePlugin::GazeboRosInterfacePlugin()
    : WorldPlugin(), gz_node_handle_(0), ros_node_handle_(0) {}

GazeboRosInterfacePlugin::~GazeboRosInterfacePlugin() {
  updateConnection_->~Connection();

  // Shutdown and delete ROS node handle
  if (ros_node_handle_) {
    ros_node_handle_->shutdown();
    delete ros_node_handle_;
  }
}

void GazeboRosInterfacePlugin::Load(physics::WorldPtr _world,
                                    sdf::ElementPtr _sdf) {
  // \brief    Store the pointer to the model.
  world_ = _world;

  // Get Gazebo node handle
  gz_node_handle_ = transport::NodePtr(new transport::Node());
  gz_node_handle_->Init();

  // Get ROS node handle
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
  }
  ros_node_handle_ = new ros::NodeHandle("gazebo_client");

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosInterfacePlugin::OnUpdate, this, _1));

  // ============================================ //
  // === CONNECT GAZEBO TO ROS MESSAGES SETUP === //
  // ============================================ //

  gz_connect_gazebo_to_ros_topic_sub_ = gz_node_handle_->Subscribe(
      "~/" + kConnectGazeboToRosSubtopic,
      &GazeboRosInterfacePlugin::GzConnectGazeboToRosTopicMsgCallback, this);

  // ============================================ //
  // === CONNECT ROS TO GAZEBO MESSAGES SETUP === //
  // ============================================ //

  gz_connect_ros_to_gazebo_topic_sub_ = gz_node_handle_->Subscribe(
      "~/" + kConnectRosToGazeboSubtopic,
      &GazeboRosInterfacePlugin::GzConnectRosToGazeboTopicMsgCallback, this);
}

void GazeboRosInterfacePlugin::OnUpdate(const common::UpdateInfo& _info) {
  // Do nothing
  // This plugins actions are all executed through message callbacks.
}

/// \brief      A helper class that provides storage for additional parameters
///             that are inserted into the callback.
/// \details
///   GazeboMsgT  The type of the message that will be subscribed to the Gazebo
///   framework.
template <typename GazeboMsgT>
struct ConnectHelperStorage {
  /// \brief    Pointer to the ROS interface plugin class.
  GazeboRosInterfacePlugin* ptr;

  /// \brief    Function pointer to the subscriber callback with additional
  /// parameters.
  void (GazeboRosInterfacePlugin::*fp)(
      const boost::shared_ptr<GazeboMsgT const>&, ros::Publisher ros_publisher);

  /// \brief    The ROS publisher that is passed into the modified callback.
  ros::Publisher ros_publisher;

  /// \brief    This is what gets passed into the Gazebo Subscribe method as a
  ///           callback, and hence can only
  ///           have one parameter (note boost::bind() does not work with the
  ///           current Gazebo Subscribe() definitions).
  void callback(const boost::shared_ptr<GazeboMsgT const>& msg_ptr) {
    (ptr->*fp)(msg_ptr, ros_publisher);
  }
};

template <typename GazeboMsgT, typename RosMsgT>
void GazeboRosInterfacePlugin::ConnectHelper(
    void (GazeboRosInterfacePlugin::*fp)(
        const boost::shared_ptr<GazeboMsgT const>&, ros::Publisher),
    GazeboRosInterfacePlugin* ptr, std::string gazeboNamespace,
    std::string gazeboTopicName, std::string rosTopicName,
    transport::NodePtr gz_node_handle) {
  // One map will be created for each Gazebo message type
  static std::map<std::string, ConnectHelperStorage<GazeboMsgT> > callback_map;

  // Create ROS publisher
  ros::Publisher ros_publisher =
      ros_node_handle_->advertise<RosMsgT>(rosTopicName, 1);

  auto callback_entry = callback_map.emplace(
      gazeboTopicName,
      ConnectHelperStorage<GazeboMsgT>{ptr, fp, ros_publisher});

  // Check if element was already present
  if (!callback_entry.second)
    gzerr << "Tried to add element to map but the gazebo topic name was "
             "already present in map."
          << std::endl;

  // Create subscriber
  gazebo::transport::SubscriberPtr subscriberPtr;
  subscriberPtr = gz_node_handle->Subscribe(
      gazeboTopicName, &ConnectHelperStorage<GazeboMsgT>::callback,
      &callback_entry.first->second);

  // Save a reference to the subscriber pointer so subscriber
  // won't be deleted.
  subscriberPtrs_.push_back(subscriberPtr);
}

void GazeboRosInterfacePlugin::GzConnectGazeboToRosTopicMsgCallback(
    GzConnectGazeboToRosTopicMsgPtr& gz_connect_gazebo_to_ros_topic_msg) {
  const std::string gazeboNamespace = "";
  const std::string gazeboTopicName = gz_connect_gazebo_to_ros_topic_msg->gazebo_topic();
  const std::string rosTopicName = gz_connect_gazebo_to_ros_topic_msg->ros_topic();

  switch (gz_connect_gazebo_to_ros_topic_msg->msgtype()) {
    case std_msgs::msgs::ConnectGazeboToRosTopic::ODOMETRY: {
      ConnectHelper<nav_msgs::msgs::Odometry, nav_msgs::Odometry>(
          &GazeboRosInterfacePlugin::GzOdometryMsgCallback, this,
          gazeboNamespace, gazeboTopicName, rosTopicName, gz_node_handle_);
      break;
    }
    default:
      gzthrow("ConnectGazeboToRosTopic message type with enum val = "
              << gz_connect_gazebo_to_ros_topic_msg->msgtype()
              << " is not supported by GazeboRosInterfacePlugin.");
  }
}

void GazeboRosInterfacePlugin::GzConnectRosToGazeboTopicMsgCallback(
    GzConnectRosToGazeboTopicMsgPtr& gz_connect_ros_to_gazebo_topic_msg) {

  static std::vector<ros::Subscriber> ros_subscribers;

  switch (gz_connect_ros_to_gazebo_topic_msg->msgtype()) {
    case std_msgs::msgs::ConnectRosToGazeboTopic::COMMAND_MOTOR_THROTTLE: {
      transport::PublisherPtr gz_publisher_ptr = gz_node_handle_->Advertise<mav_msgs::msgs::CommandMotorThrottle>(
              gz_connect_ros_to_gazebo_topic_msg->gazebo_topic(), 1);

      // Create ROS subscriber.
      ros::Subscriber ros_subscriber = ros_node_handle_->subscribe<std_msgs::Float64MultiArray>(
        gz_connect_ros_to_gazebo_topic_msg->ros_topic(), 1,
        boost::bind(&GazeboRosInterfacePlugin::RosCommandMotorThrottleMsgCallback, this, _1, gz_publisher_ptr)
      );

      ros_subscribers.push_back(ros_subscriber);

      break;
    }
    default: {
      gzthrow("ConnectRosToGazeboTopic message type with enum val = "
              << gz_connect_ros_to_gazebo_topic_msg->msgtype()
              << " is not supported by GazeboRosInterfacePlugin.");
    }
  }
}

//===========================================================================//
//================ GAZEBO -> ROS MSG CALLBACKS/CONVERTERS ===================//
//===========================================================================//

void GazeboRosInterfacePlugin::GzOdometryMsgCallback(GzOdometryMsgPtr& gz_odometry_msg, ros::Publisher ros_publisher) {
  // We need to convert from a Gazebo message to a ROS message, and then forward
  // the Odometry message to ROS.

  // ============================================ //
  // =================== HEADER ================= //
  // ============================================ //
  ros_odometry_msg_.header.stamp.sec = gz_odometry_msg->time_usec() / 1000000.0;
  ros_odometry_msg_.header.stamp.nsec = gz_odometry_msg->time_usec() * 1000.0;

  // ============================================ //
  // ===================== POSE ================= //
  // ============================================ //
  ros_odometry_msg_.pose.pose.position.x = gz_odometry_msg->position().x();
  ros_odometry_msg_.pose.pose.position.y = gz_odometry_msg->position().y();
  ros_odometry_msg_.pose.pose.position.z = gz_odometry_msg->position().z();

  ros_odometry_msg_.pose.pose.orientation.w = gz_odometry_msg->orientation().w();
  ros_odometry_msg_.pose.pose.orientation.x = gz_odometry_msg->orientation().x();
  ros_odometry_msg_.pose.pose.orientation.y = gz_odometry_msg->orientation().y();
  ros_odometry_msg_.pose.pose.orientation.z = gz_odometry_msg->orientation().z();

  for (int i = 0; i < 6; i++) {
    ros_odometry_msg_.pose.covariance[i] = gz_odometry_msg->pose_covariance(i);
  }

  // ============================================ //
  // ===================== TWIST ================ //
  // ============================================ //
  ros_odometry_msg_.twist.twist.linear.x = gz_odometry_msg->linear_velocity().x();
  ros_odometry_msg_.twist.twist.linear.y = gz_odometry_msg->linear_velocity().y();
  ros_odometry_msg_.twist.twist.linear.z = gz_odometry_msg->linear_velocity().z();

  ros_odometry_msg_.twist.twist.angular.x = gz_odometry_msg->angular_velocity().x();
  ros_odometry_msg_.twist.twist.angular.y = gz_odometry_msg->angular_velocity().y();
  ros_odometry_msg_.twist.twist.angular.z = gz_odometry_msg->angular_velocity().z();

  for (int i = 0; i < 6; i++) {
    ros_odometry_msg_.twist.covariance[i] = gz_odometry_msg->velocity_covariance(i);
  }

  // Publish to ROS framework.
  ros_publisher.publish(ros_odometry_msg_);
}

//===========================================================================//
//================ ROS -> GAZEBO MSG CALLBACKS/CONVERTERS ===================//
//===========================================================================//

void GazeboRosInterfacePlugin::RosCommandMotorThrottleMsgCallback(
    const std_msgs::Float64MultiArray::ConstPtr& ros_actuators_msg_ptr,
    transport::PublisherPtr gz_publisher_ptr) {
  static int64_t seq = 0;

  // Convert ROS message to Gazebo message
  mav_msgs::msgs::CommandMotorThrottle gz_command_motor_throttle_msg;

  for (int i = 0; i < 4; i++) {
    gz_command_motor_throttle_msg.add_motor_throttle(ros_actuators_msg_ptr->data[i]);
  }
  gz_command_motor_throttle_msg.set_dt(0.004);
  gz_command_motor_throttle_msg.set_seq(seq++);

  // Publish to Gazebo
  gz_publisher_ptr->Publish(gz_command_motor_throttle_msg);
}

GZ_REGISTER_WORLD_PLUGIN(GazeboRosInterfacePlugin);

}  // namespace gazebo