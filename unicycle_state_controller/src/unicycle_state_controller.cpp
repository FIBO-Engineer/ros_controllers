///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF Inc nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/*
 * Author: Wim Meeussen
 */

#include <algorithm>
#include <cstddef>
#include <string>
#include <pluginlib/class_list_macros.hpp>
#include <tf/transform_datatypes.h>

#include "unicycle_state_controller/unicycle_state_controller.h"

namespace unicycle_state_controller
{

  bool UnicycleStateController::init(hardware_interface::JointStateInterface* hw,
                                  ros::NodeHandle&                         root_nh,
                                  ros::NodeHandle&                         controller_nh)
  {
    // Get list of joints: This allows specifying a desired order, or
    // alternatively, only publish states for a subset of joints. If the
    // parameter is not set, all joint states will be published in the order
    // specified by the hardware interface.
    if (!controller_nh.getParam("drive_joint", drive_joint) || !controller_nh.getParam("steering_joint", steering_joint)){
      ROS_ERROR("Parameter 'steering_joint' or 'drive_joint' not set");
      return false;
    }

    // get publishing period
    if (!controller_nh.getParam("publish_rate", publish_rate_)){
      ROS_ERROR("Parameter 'publish_rate' not set");
      return false;
    }

    if (!controller_nh.getParam("wheel_radius", wheel_radius_)){
      ROS_ERROR("Parameter 'wheel_radius' not set");
      return false;
    }
    odometry_.setWheelParams(wheel_radius_);

    int velocity_rolling_window_size = 5;
    controller_nh.param("velocity_rolling_window_size", velocity_rolling_window_size, velocity_rolling_window_size);
    ROS_INFO_STREAM("Velocity rolling window size of "<< velocity_rolling_window_size << ".");
    odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);

    controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
    ROS_INFO_STREAM("Base frame_id set to " << base_frame_id_);

    controller_nh.param("odom_frame_id", odom_frame_id_, odom_frame_id_);
    ROS_INFO_STREAM("Odometry frame_id set to " << odom_frame_id_);

    controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
    ROS_INFO_STREAM("Publishing to tf is " << (enable_odom_tf_?"enabled":"disabled"));

    joint_state_.clear();
    joint_state_.push_back(hw->getHandle(drive_joint));
    joint_state_.push_back(hw->getHandle(steering_joint));

    setOdomPubFields(root_nh, controller_nh);

    pub_time_initialized_=false;

    return true;
  }

  void UnicycleStateController::starting(const ros::Time& time)
  {
    // initialize time exactly once, to maintain publish rate through controller resets
    if (!pub_time_initialized_)
    {
      try {
        last_publish_time_ = time - ros::Duration(1.001/publish_rate_); //ensure publish on first cycle
      } catch(std::runtime_error& ex) { // negative ros::Time is not allowed
        last_publish_time_ = ros::Time::MIN;
      }
      pub_time_initialized_ = true;
    }
    odometry_.init(time);
  }

  void UnicycleStateController::setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
  {
    // Get and check params for covariances
    XmlRpc::XmlRpcValue pose_cov_list;
    controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
    ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(pose_cov_list.size() == 6);
    for (int i = 0; i < pose_cov_list.size(); ++i)
      ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    XmlRpc::XmlRpcValue twist_cov_list;
    controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
    ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(twist_cov_list.size() == 6);
    for (int i = 0; i < twist_cov_list.size(); ++i)
      ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    // Setup odometry realtime publisher + odom message constant fields
    odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
    odom_pub_->msg_.header.frame_id = odom_frame_id_;
    odom_pub_->msg_.child_frame_id = base_frame_id_;
    odom_pub_->msg_.pose.pose.position.z = 0;
    odom_pub_->msg_.pose.covariance = {
        static_cast<double>(pose_cov_list[0]), 0., 0., 0., 0., 0.,
        0., static_cast<double>(pose_cov_list[1]), 0., 0., 0., 0.,
        0., 0., static_cast<double>(pose_cov_list[2]), 0., 0., 0.,
        0., 0., 0., static_cast<double>(pose_cov_list[3]), 0., 0.,
        0., 0., 0., 0., static_cast<double>(pose_cov_list[4]), 0.,
        0., 0., 0., 0., 0., static_cast<double>(pose_cov_list[5]) };
    odom_pub_->msg_.twist.twist.linear.y  = 0;
    odom_pub_->msg_.twist.twist.linear.z  = 0;
    odom_pub_->msg_.twist.twist.angular.x = 0;
    odom_pub_->msg_.twist.twist.angular.y = 0;
    odom_pub_->msg_.twist.covariance = {
        static_cast<double>(twist_cov_list[0]), 0., 0., 0., 0., 0.,
        0., static_cast<double>(twist_cov_list[1]), 0., 0., 0., 0.,
        0., 0., static_cast<double>(twist_cov_list[2]), 0., 0., 0.,
        0., 0., 0., static_cast<double>(twist_cov_list[3]), 0., 0.,
        0., 0., 0., 0., static_cast<double>(twist_cov_list[4]), 0.,
        0., 0., 0., 0., 0., static_cast<double>(twist_cov_list[5]) };
    tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
    tf_odom_pub_->msg_.transforms.resize(1);
    tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
    tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
    tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;
  }

  void UnicycleStateController::update(const ros::Time& time, const ros::Duration& /*period*/)
  {
    // limit rate of publishing
    if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time){

      // try to publish
      double wheel_pos  = 0.0;
      const double drive_pos = joint_state_[0].getPosition();
      const double steering_pos = joint_state_[1].getPosition();
      if (std::isnan(drive_pos) || std::isnan(steering_pos))
        return;

      // Estimate linear and angular velocity using joint information
      odometry_.update(drive_pos, steering_pos, time);

      if (last_publish_time_ + ros::Duration(1.0/publish_rate_) < time)
      {
        last_publish_time_ += ros::Duration(1.0/publish_rate_);
        // Compute and store orientation info
        const geometry_msgs::Quaternion orientation(tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

        // Populate odom message and publish
        if (odom_pub_->trylock())
        {
          odom_pub_->msg_.header.stamp = time;
          odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
          odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
          odom_pub_->msg_.pose.pose.orientation = orientation;
          odom_pub_->msg_.twist.twist.linear.x  = odometry_.getLinear();
          odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
          odom_pub_->unlockAndPublish();
        }

        // Publish tf /odom frame
        if (enable_odom_tf_ && tf_odom_pub_->trylock())
        {
          geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
          odom_frame.header.stamp = time;
          odom_frame.transform.translation.x = odometry_.getX();
          odom_frame.transform.translation.y = odometry_.getY();
          odom_frame.transform.rotation = orientation;
          tf_odom_pub_->unlockAndPublish();
        }
      }
    }
    
  }

  void UnicycleStateController::stopping(const ros::Time& /*time*/)
  {}

}

PLUGINLIB_EXPORT_CLASS( unicycle_state_controller::UnicycleStateController, controller_interface::ControllerBase)
