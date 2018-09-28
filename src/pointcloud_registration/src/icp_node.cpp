/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Author: Brian Gerkey */

#include <algorithm>
#include <vector>
#include <map>
#include <cmath>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

// Signal handling
#include <signal.h>

#include "ros/assert.h"

// roscpp
#include "ros/ros.h"

// Messages that I need
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"
#include "std_srvs/Empty.h"

// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "message_filters/subscriber.h"

// Dynamic_reconfigure
#include "dynamic_reconfigure/server.h"
#include <boost/foreach.hpp>



static const std::string scan_topic_ = "scan";

class ICPNode
{
  public:
    ICPNode();



  private:
    tf::TransformBroadcaster* tfb_;

    // Use a child class to get access to tf2::Buffer class inside of tf_
    struct TransformListenerWrapper : public tf::TransformListener
    {
      inline tf2_ros::Buffer &getBuffer() {return tf2_buffer_;}
    };

    TransformListenerWrapper* tf_;

    bool sent_first_transform_;

    tf::Transform latest_tf_;
    bool latest_tf_valid_;

    // Callbacks
    void icpReceived_(const nav_msgs::OdometryConstPtr& odom);

    //parameter for what odom to use
    std::string odom_frame_id_;

    //paramater to store latest odom pose
    tf::Stamped<tf::Pose> latest_odom_pose_;

    //parameter for what base to use
    std::string base_frame_id_;
    std::string global_frame_id_;

    bool use_map_topic_;
    bool first_map_only_;

    ros::Duration gui_publish_period;
    ros::Time save_pose_last_time;
    ros::Duration save_pose_period;

    geometry_msgs::PoseWithCovarianceStamped last_published_pose;

    message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* laser_scan_filter_;
    ros::Subscriber initial_pose_sub_;

    //Nomotion update control
    bool m_force_update;  // used to temporarily let amcl update samples even when no motion occurs...


    ros::Duration cloud_pub_interval;
    ros::Time last_cloud_pub_time;

    // For slowing play-back when reading directly from a bag file
    ros::WallDuration bag_scan_period_;



    //time for tolerance on the published transform,
    //basically defines how long a map->odom transform is good for
    ros::Duration transform_tolerance_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber icp_received_;

    bool first_map_received_;
    bool first_reconfigure_call_;

    boost::recursive_mutex configuration_mutex_;
    ros::Timer check_laser_timer_;

    int max_beams_, min_particles_, max_particles_;
    double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
    double alpha_slow_, alpha_fast_;
    double z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_;
  //beam skip related params
    bool do_beamskip_;
    double beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_;
    double laser_likelihood_max_dist_;
    double init_pose_[3];
    double init_cov_[3];
    bool tf_broadcast_;

    ros::Time last_laser_received_ts_;
    ros::Duration laser_check_interval_;
};


int
main(int argc, char** argv)
{
  ros::init(argc, argv, "icp");
  //ros::NodeHandle nh;

  // Override default sigint handler
  //signal(SIGINT, sigintHandler);

  // Make our node available to sigintHandler
  //icp_node_ptr.reset(new ICPNode());

    ICPNode icpnode;


  // Without this, our boost locks are not shut down nicely


    ros::spin();
  // To quote Morgan, Hooray!
  return(0);
}

ICPNode::ICPNode() :
        sent_first_transform_(false),
        latest_tf_valid_(false),        
              private_nh_("~"),
        first_map_received_(false),
        first_reconfigure_call_(true)
{
  boost::recursive_mutex::scoped_lock l(configuration_mutex_);

  // Grab params off the param server


  cloud_pub_interval.fromSec(1.0);
  tfb_ = new tf::TransformBroadcaster();
  tf_ = new TransformListenerWrapper();

  icp_received_ = nh_.subscribe ("icp_odom", 1, &ICPNode::icpReceived_, this);

    base_frame_id_ = "iMiev/base_link";
    global_frame_id_ = "iMiev/map";
    odom_frame_id_ = "iMiev/odom";

    tf_broadcast_ = true;
    transform_tolerance_ = ros::Duration(0.3);
  laser_check_interval_ = ros::Duration(15.0);

}

void ICPNode::icpReceived_ (const nav_msgs::OdometryConstPtr& odom)
{
    std::cout<<"received icp at: "<<odom->header.stamp.toSec()<<std::endl;
    tf::Stamped<tf::Pose> odom_to_map;
    tf::Quaternion odom_orient;
    tf::quaternionMsgToTF(odom->pose.pose.orientation, odom_orient);
    try
    {
      tf::Transform tmp_tf(odom_orient,
                           tf::Vector3(odom->pose.pose.position.x,
                                       odom->pose.pose.position.y,
                                       0.0));
      tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(),
                                            odom->header.stamp,
                                            base_frame_id_);
      this->tf_->transformPose(odom_frame_id_,
                               tmp_tf_stamped,
                               odom_to_map);
    }
    catch(tf::TransformException)
    {
      ROS_DEBUG("Failed to subtract base to odom transform");
      return;
    }

    latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                               tf::Point(odom_to_map.getOrigin()));
    latest_tf_valid_ = true;

    if (tf_broadcast_ == true)
    {
      // We want to send a transform that is good up until a
      // tolerance time so that odom can be used
      ros::Time transform_expiration = (odom->header.stamp +
                                        transform_tolerance_);
      tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                          transform_expiration,
                                          global_frame_id_, odom_frame_id_);
      this->tfb_->sendTransform(tmp_tf_stamped);
      sent_first_transform_ = true;
    }
}


