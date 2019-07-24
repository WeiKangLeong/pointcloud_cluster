#ifndef SCAN_GPS_H
#define SCAN_GPS_H

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <pcl_ros/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>

namespace pointcloud_registration
{
	class Scan_GPS
	{
	public:
		Scan_GPS();

	private:
		ros::NodeHandle nh_;

		ros::Subscriber sub_top_;
		ros::Subscriber sub_front_;
		ros::Subscriber sub_rear_;

		ros::Subscriber sub_gps_;

		ros::Publisher pub_after_matched_;
		ros::Publisher pub_map_;
		
		void top_cb_ (const sensor_msgs::PointCloud2ConstPtr& input);
		void rear_cb_ (const sensor_msgs::PointCloud2ConstPtr& input);
		void front_cb_ (const sensor_msgs::PointCloud2ConstPtr& input);

		void gps_cb_ (const geometry_msgs::PoseWithCovarianceStampedConstPtr& gps);

		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
		pcl::PointCloud<pcl::PointXYZI>::Ptr map_;

		sensor_msgs::PointCloud2 stored_cloud_;

		std::vector<geometry_msgs::Pose>* stored_gps_;

		pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    		pcl::PassThrough<pcl::PointXYZI> pass;

	};
}

#endif
