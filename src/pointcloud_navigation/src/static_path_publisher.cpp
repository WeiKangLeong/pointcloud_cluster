/** author: LeeYiyuan **/

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <fstream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "static_path_publisher");

  ros::NodeHandle handle;
  ros::NodeHandle handle_private("~");

  std::string path_file_name;
  ROS_ASSERT(handle_private.getParam("path_file_name", path_file_name));
  std::string path_topic;
  ROS_ASSERT(handle_private.getParam("path_topic", path_topic));
  std::string frame_id;
  ROS_ASSERT(handle_private.getParam("frame_id", frame_id));

  std::ifstream path_file;
  path_file.open(path_file_name, std::ios::in | std::ios::binary);

  nav_msgs::Path path;
  path.header.stamp = ros::Time(0);
  path.header.frame_id = frame_id;
  double x, y;
  while ((path_file >> x) && (path_file >> y))
  {
    path.poses.emplace_back();
    geometry_msgs::PoseStamped& pose = path.poses.back();
    pose.header.stamp = ros::Time(0);
    pose.header.frame_id = frame_id;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 1);
  }

  ros::Publisher path_pub = handle.advertise<nav_msgs::Path>(path_topic, 1, true);
  path_pub.publish(path);

  ros::spin();
}
