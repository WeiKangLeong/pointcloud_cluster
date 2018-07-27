#include <cstdio>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

tf::TransformListener *tf_listener_;

tf::StampedTransform offset_transform;

tf::Transform latest_icp, now_odom_transform, prev_odom_transform;

bool icp_localize, odom_start;

using namespace std;

void icp_cb(const geometry_msgs::PoseWithCovarianceStamped icp_input)
{
    latest_icp.setIdentity();
    tf::Quaternion icp_rotate;
    latest_icp.setOrigin(tf::Vector3(icp_input.pose.pose.position.x, icp_input.pose.pose.position.y, icp_input.pose.pose.position.z));
    tf::quaternionMsgToTF(icp_input.pose.pose.orientation, icp_rotate);
    latest_icp.setRotation(icp_rotate);
    icp_localize=true;
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "base_link_to_map_publisher");
  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh;
  string parent_frame_id, child_frame_id, odom_frame_id;
  double roll, pitch, yaw, x, y, z, ms;
  nh_private.param("parent_frame_id", parent_frame_id, string("parent_frame"));
  nh_private.param("child_frame_id", child_frame_id, string("child_frame"));
  nh_private.param("odom_frame_id", odom_frame_id, string("odom_frame"));
  nh_private.param("roll", roll, 0.0);
  nh_private.param("pitch", pitch, 0.0);
  nh_private.param("yaw", yaw, 0.0);
  nh_private.param("x", x, 0.0);
  nh_private.param("y", y, 0.0);
  nh_private.param("z", z, 0.0);
  nh_private.param("ms", ms, 10.0);
  tf::StampedTransform transform;
  tf::TransformBroadcaster broadcaster;
  tf_listener_ = new tf::TransformListener();
  tf::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  ros::Duration sleeper(ms / 1000.0);
  //transform = tf::StampedTransform(tf::Transform(q, tf::Vector3(x, y, z)), ros::Time::now() + sleeper, parent_frame_id, child_frame_id);

  ros::Subscriber icp_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped> ("/icp_pose", 1, icp_cb);

  while (ros::ok())
  {
      ros::spinOnce();

      if (icp_localize)
      {
          transform = tf::StampedTransform(latest_icp, ros::Time::now() + sleeper, parent_frame_id, child_frame_id);
          broadcaster.sendTransform(transform);
          icp_localize = false;
      }
      else
      {
          tf::StampedTransform latest_odom_transform;
          tf::Transform difference;
          try{
              tf_listener_->lookupTransform(odom_frame_id, child_frame_id,
                                            ros::Time(0), latest_odom_transform);
          }catch (tf::TransformException &ex) {
              ROS_ERROR_STREAM("Odom to baselink Transform Failed");
              //return;
          }

          if (odom_start==false)
          {
              offset_transform = latest_odom_transform;
              odom_start=true;
          }

          now_odom_transform = offset_transform.inverseTimes(latest_odom_transform);
          difference = prev_odom_transform.inverseTimes(now_odom_transform);
          latest_icp = latest_icp*difference;
          transform = tf::StampedTransform(latest_icp, ros::Time::now() + sleeper, parent_frame_id, child_frame_id);
          broadcaster.sendTransform(transform);
          prev_odom_transform = now_odom_transform;
      }
//    transform.stamp_ = ros::Time::now() + sleeper;
//    broadcaster.sendTransform(transform);
    ROS_DEBUG("Sending transform from %s with parent %s\n", parent_frame_id.c_str(), child_frame_id.c_str());
    sleeper.sleep();
  }


}
