#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

tf::TransformBroadcaster *tfb;

ros::Publisher odom_pub;

void odom_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr odom_msg)
{
    tf::Transform odom_transform;

    tf::Quaternion orient;
    tf::quaternionMsgToTF(odom_msg->pose.pose.orientation, orient);
    odom_transform.setOrigin(tf::Vector3(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, 0));
    odom_transform.setRotation(orient.normalized());
    tf::StampedTransform odom_transform_stamped(odom_transform, odom_msg->header.stamp, "/wtf_odom", "/wtf_base_link");
    tfb->sendTransform(odom_transform_stamped);

    //std::cout<<odom_msg->pose.pose.position.x<<" "<<odom_msg->pose.pose.position.y<<std::endl;
}


int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "transform_publisher");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud

    ros::Subscriber can_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_input", 10, odom_callback);



    // Create a ROS publisher for the output point cloud
    //pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
    tfb = new tf::TransformBroadcaster();

    // Spin
    ros::spin ();
}
