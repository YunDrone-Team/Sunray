/**
*
*  \author     Paul Bovbel <pbovbel@clearpathrobotics.com>
*  \copyright  Copyright (c) 2015, Clearpath Robotics, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Clearpath Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to code@clearpathrobotics.com
*
*/

#include "vrpn_client_ros/vrpn_client_ros.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"

#include <vector>
#include <unordered_set>
#include <algorithm>

namespace
{
  std::unordered_set<std::string> name_blacklist_({"VRPN Control"});
}

namespace vrpn_client_ros
{

  /**
   * check Ros Names as defined here: http://wiki.ros.org/Names
   */
  bool isInvalidFirstCharInName(const char c)
  {
    return ! ( isalpha(c) || c == '/' || c == '~' );
  }

  bool isInvalidSubsequentCharInName(const char c)
  {
    return ! ( isalnum(c) || c == '/' || c == '_' );
  }

  VrpnTrackerRos::VrpnTrackerRos(std::string tracker_name, ConnectionPtr connection, ros::NodeHandle nh)
  {
    tracker_remote_ = std::make_shared<vrpn_Tracker_Remote>(tracker_name.c_str(), connection.get());

    std::string clean_name = tracker_name;

    if (clean_name.size() > 0)
    {
      int start_subsequent = 1;
      if (isInvalidFirstCharInName(clean_name[0])) 
      {
        clean_name = clean_name.substr(1);
        start_subsequent = 0;
      }

      clean_name.erase( std::remove_if( clean_name.begin() + start_subsequent, clean_name.end(), isInvalidSubsequentCharInName ), clean_name.end() );
    }

    init(clean_name, nh, false);
  }

  VrpnTrackerRos::VrpnTrackerRos(std::string tracker_name, std::string host, ros::NodeHandle nh)
  {
    std::string tracker_address;
    tracker_address = tracker_name + "@" + host;
    tracker_remote_ = std::make_shared<vrpn_Tracker_Remote>(tracker_address.c_str());
    init(tracker_name, nh, true);
  }

  void VrpnTrackerRos::init(std::string tracker_name, ros::NodeHandle nh, bool create_mainloop_timer)
  {
    ROS_INFO_STREAM("Creating new tracker " << tracker_name);

    tracker_remote_->register_change_handler(this, &VrpnTrackerRos::handle_pose);
    tracker_remote_->register_change_handler(this, &VrpnTrackerRos::handle_twist);
    tracker_remote_->register_change_handler(this, &VrpnTrackerRos::handle_accel);
    tracker_remote_->shutup = true;

    std::string error;
    if (!ros::names::validate(tracker_name, error))
    {
      ROS_ERROR_STREAM("Invalid tracker name " << tracker_name << ", not creating topics : " << error);
      return;
    }

    this->tracker_name = tracker_name;

    output_nh_ = ros::NodeHandle(nh, tracker_name);

    std::string frame_id;
    nh.param<std::string>("frame_id", frame_id, "world");
    nh.param<bool>("use_server_time", use_server_time_, false);
    nh.param<bool>("broadcast_tf", broadcast_tf_, false);
    nh.param<bool>("process_sensor_id", process_sensor_id_, false);

    pose_msg_.header.frame_id = twist_msg_.header.frame_id = accel_msg_.header.frame_id = transform_stamped_.header.frame_id = frame_id;

    if (create_mainloop_timer)
    {
      double update_frequency;
      nh.param<double>("update_frequency", update_frequency, 100.0);
      mainloop_timer = nh.createTimer(ros::Duration(1 / update_frequency),
                                      boost::bind(&VrpnTrackerRos::mainloop, this));
    }
  }

  VrpnTrackerRos::~VrpnTrackerRos()
  {
    ROS_INFO_STREAM("Destroying tracker " << transform_stamped_.child_frame_id);
    tracker_remote_->unregister_change_handler(this, &VrpnTrackerRos::handle_pose);
    tracker_remote_->unregister_change_handler(this, &VrpnTrackerRos::handle_twist);
    tracker_remote_->unregister_change_handler(this, &VrpnTrackerRos::handle_accel);
  }

  void VrpnTrackerRos::mainloop()
  {
    tracker_remote_->mainloop();
  }

  void VRPN_CALLBACK VrpnTrackerRos::handle_pose(void *userData, const vrpn_TRACKERCB tracker_pose)
  {
    VrpnTrackerRos *tracker = static_cast<VrpnTrackerRos *>(userData);

    ros::Publisher *pose_pub;
    std::size_t sensor_index(0);
    ros::NodeHandle nh = tracker->output_nh_;
    
    if (tracker->process_sensor_id_)
    {
      sensor_index = static_cast<std::size_t>(tracker_pose.sensor);
      nh = ros::NodeHandle(tracker->output_nh_, std::to_string(tracker_pose.sensor));
    }
    
    if (tracker->pose_pubs_.size() <= sensor_index)
    {
      tracker->pose_pubs_.resize(sensor_index + 1);
    }
    pose_pub = &(tracker->pose_pubs_[sensor_index]);

    if (pose_pub->getTopic().empty())
    {
      *pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
    }

    if (pose_pub->getNumSubscribers() > 0)
    {
      if (tracker->use_server_time_)
      {
        tracker->pose_msg_.header.stamp.sec = tracker_pose.msg_time.tv_sec;
        tracker->pose_msg_.header.stamp.nsec = tracker_pose.msg_time.tv_usec * 1000;
      }
      else
      {
        tracker->pose_msg_.header.stamp = ros::Time::now();
      }

      tracker->pose_msg_.pose.position.x = tracker_pose.pos[0];
      tracker->pose_msg_.pose.position.y = tracker_pose.pos[1];
      tracker->pose_msg_.pose.position.z = tracker_pose.pos[2];

      tracker->pose_msg_.pose.orientation.x = tracker_pose.quat[0];
      tracker->pose_msg_.pose.orientation.y = tracker_pose.quat[1];
      tracker->pose_msg_.pose.orientation.z = tracker_pose.quat[2];
      tracker->pose_msg_.pose.orientation.w = tracker_pose.quat[3];

      pose_pub->publish(tracker->pose_msg_);
    }

    if (tracker->broadcast_tf_)
    {
      static tf2_ros::TransformBroadcaster tf_broadcaster;

      if (tracker->use_server_time_)
      {
        tracker->transform_stamped_.header.stamp.sec = tracker_pose.msg_time.tv_sec;
        tracker->transform_stamped_.header.stamp.nsec = tracker_pose.msg_time.tv_usec * 1000;
      }
      else
      {
        tracker->transform_stamped_.header.stamp = ros::Time::now();
      }

      if (tracker->process_sensor_id_)
      {
        tracker->transform_stamped_.child_frame_id = tracker->tracker_name + "/" + std::to_string(tracker_pose.sensor);
      }
      else
      {
        tracker->transform_stamped_.child_frame_id = tracker->tracker_name;
      }

      tracker->transform_stamped_.transform.translation.x = tracker_pose.pos[0];
      tracker->transform_stamped_.transform.translation.y = tracker_pose.pos[1];
      tracker->transform_stamped_.transform.translation.z = tracker_pose.pos[2];

      tracker->transform_stamped_.transform.rotation.x = tracker_pose.quat[0];
      tracker->transform_stamped_.transform.rotation.y = tracker_pose.quat[1];
      tracker->transform_stamped_.transform.rotation.z = tracker_pose.quat[2];
      tracker->transform_stamped_.transform.rotation.w = tracker_pose.quat[3];

      tf_broadcaster.sendTransform(tracker->transform_stamped_);
    }
  }

  void VRPN_CALLBACK VrpnTrackerRos::handle_twist(void *userData, const vrpn_TRACKERVELCB tracker_twist)
  {
    VrpnTrackerRos *tracker = static_cast<VrpnTrackerRos *>(userData);

    ros::Publisher *twist_pub;
    std::size_t sensor_index(0);
    ros::NodeHandle nh = tracker->output_nh_;
    
    if (tracker->process_sensor_id_)
    {
      sensor_index = static_cast<std::size_t>(tracker_twist.sensor);
      nh = ros::NodeHandle(tracker->output_nh_, std::to_string(tracker_twist.sensor));
    }
    
    if (tracker->twist_pubs_.size() <= sensor_index)
    {
      tracker->twist_pubs_.resize(sensor_index + 1);
    }
    twist_pub = &(tracker->twist_pubs_[sensor_index]);

    if (twist_pub->getTopic().empty())
    {
      *twist_pub = nh.advertise<geometry_msgs::TwistStamped>("twist", 1);
    }

    if (twist_pub->getNumSubscribers() > 0)
    {
      if (tracker->use_server_time_)
      {
        tracker->twist_msg_.header.stamp.sec = tracker_twist.msg_time.tv_sec;
        tracker->twist_msg_.header.stamp.nsec = tracker_twist.msg_time.tv_usec * 1000;
      }
      else
      {
        tracker->twist_msg_.header.stamp = ros::Time::now();
      }

      tracker->twist_msg_.twist.linear.x = tracker_twist.vel[0];
      tracker->twist_msg_.twist.linear.y = tracker_twist.vel[1];
      tracker->twist_msg_.twist.linear.z = tracker_twist.vel[2];

      double roll, pitch, yaw;
      tf2::Matrix3x3 rot_mat(
          tf2::Quaternion(tracker_twist.vel_quat[0], tracker_twist.vel_quat[1], tracker_twist.vel_quat[2],
                          tracker_twist.vel_quat[3]));
      rot_mat.getRPY(roll, pitch, yaw);

      tracker->twist_msg_.twist.angular.x = roll;
      tracker->twist_msg_.twist.angular.y = pitch;
      tracker->twist_msg_.twist.angular.z = yaw;

      twist_pub->publish(tracker->twist_msg_);
    }
  }

  void VRPN_CALLBACK VrpnTrackerRos::handle_accel(void *userData, const vrpn_TRACKERACCCB tracker_accel)
  {
    VrpnTrackerRos *tracker = static_cast<VrpnTrackerRos *>(userData);

    ros::Publisher *accel_pub;
    std::size_t sensor_index(0);
    ros::NodeHandle nh = tracker->output_nh_;

    if (tracker->process_sensor_id_)
    {
      sensor_index = static_cast<std::size_t>(tracker_accel.sensor);
      nh = ros::NodeHandle(tracker->output_nh_, std::to_string(tracker_accel.sensor));
    }
    
    if (tracker->accel_pubs_.size() <= sensor_index)
    {
      tracker->accel_pubs_.resize(sensor_index + 1);
    }
    accel_pub = &(tracker->accel_pubs_[sensor_index]);

    if (accel_pub->getTopic().empty())
    {
      *accel_pub = nh.advertise<geometry_msgs::TwistStamped>("accel", 1);
    }

    if (accel_pub->getNumSubscribers() > 0)
    {
      if (tracker->use_server_time_)
      {
        tracker->accel_msg_.header.stamp.sec = tracker_accel.msg_time.tv_sec;
        tracker->accel_msg_.header.stamp.nsec = tracker_accel.msg_time.tv_usec * 1000;
      }
      else
      {
        tracker->accel_msg_.header.stamp = ros::Time::now();
      }

      tracker->accel_msg_.accel.linear.x = tracker_accel.acc[0];
      tracker->accel_msg_.accel.linear.y = tracker_accel.acc[1];
      tracker->accel_msg_.accel.linear.z = tracker_accel.acc[2];

      double roll, pitch, yaw;
      tf2::Matrix3x3 rot_mat(
          tf2::Quaternion(tracker_accel.acc_quat[0], tracker_accel.acc_quat[1], tracker_accel.acc_quat[2],
                          tracker_accel.acc_quat[3]));
      rot_mat.getRPY(roll, pitch, yaw);

      tracker->accel_msg_.accel.angular.x = roll;
      tracker->accel_msg_.accel.angular.y = pitch;
      tracker->accel_msg_.accel.angular.z = yaw;

      accel_pub->publish(tracker->accel_msg_);
    }
  }

  VrpnClientRos::VrpnClientRos(ros::NodeHandle nh, ros::NodeHandle private_nh)
  {
    output_nh_ = private_nh;

    host_ = getHostStringFromParams(private_nh);

    ROS_INFO_STREAM("Connecting to VRPN server at " << host_);
    connection_ = std::shared_ptr<vrpn_Connection>(vrpn_get_connection_by_name(host_.c_str()));
    ROS_INFO("Connection established");

    double update_frequency;
    private_nh.param<double>("update_frequency", update_frequency, 100.0);
    mainloop_timer = nh.createTimer(ros::Duration(1 / update_frequency), boost::bind(&VrpnClientRos::mainloop, this));

    double refresh_tracker_frequency;
    private_nh.param<double>("refresh_tracker_frequency", refresh_tracker_frequency, 0.0);

    if (refresh_tracker_frequency > 0.0)
    {
      refresh_tracker_timer_ = nh.createTimer(ros::Duration(1 / refresh_tracker_frequency),
                                              boost::bind(&VrpnClientRos::updateTrackers, this));
    }

    std::vector<std::string> param_tracker_names_;
    if (private_nh.getParam("trackers", param_tracker_names_))
    {
      for (std::vector<std::string>::iterator it = param_tracker_names_.begin();
           it != param_tracker_names_.end(); ++it)
      {
        trackers_.insert(std::make_pair(*it, std::make_shared<VrpnTrackerRos>(*it, connection_, output_nh_)));
      }
    }
  }

  std::string VrpnClientRos::getHostStringFromParams(ros::NodeHandle host_nh)
  {
    std::stringstream host_stream;
    std::string server;
    int port;

    host_nh.param<std::string>("server", server, "localhost");
    host_stream << server;

    if (host_nh.getParam("port", port))
    {
      host_stream << ":" << port;
    }
    return host_stream.str();
  }

  void VrpnClientRos::mainloop()
  {
    connection_->mainloop();
    if (!connection_->doing_okay())
    {
      ROS_WARN("VRPN connection is not 'doing okay'");
    }
    for (TrackerMap::iterator it = trackers_.begin(); it != trackers_.end(); ++it)
    {
      it->second->mainloop();
    }
  }

  void VrpnClientRos::updateTrackers()
  {
    int i = 0;
    while (connection_->sender_name(i) != NULL)
    {
      if (trackers_.count(connection_->sender_name(i)) == 0 && name_blacklist_.count(connection_->sender_name(i)) == 0)
      {
        ROS_INFO_STREAM("Found new sender: " << connection_->sender_name(i));
        trackers_.insert(std::make_pair(connection_->sender_name(i),
                                        std::make_shared<VrpnTrackerRos>(connection_->sender_name(i), connection_,
                                                                           output_nh_)));
      }
      i++;
    }
  }
}  // namespace vrpn_client_ros
