#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/filters/passthrough.h>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  //sensor_msgs::PointCloud2 output;

  // Do data processing here...
  //output = *input;

  //pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL (*(input), *(cloud));
  pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2::Ptr cloud_filtered_1(new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2 cloud_filtered_2;

 

  // Create the filtering object
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (0.0, 100.0);
  pass.setFilterLimitsNegative (false);
  pass.filter (*cloud_filtered);

//  //pass.setInputCloud (cloud_filtered);
//  pass.setFilterFieldName ("y");
//  pass.setFilterLimits (-1.0, 1.0);
//  pass.setFilterLimitsNegative (true);
//  pass.filter (*cloud_filtered);

  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-3.0, 0.5);
  pass.setFilterLimitsNegative (false);
  pass.filter (cloud_filtered_2);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered_2, output);
  //output.header.frame_id = input->header.frame_id;

  // Publish the data.
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/new_points", 1);

  // Spin
  ros::spin ();
}

