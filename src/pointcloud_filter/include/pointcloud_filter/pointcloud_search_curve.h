#ifndef PCL_SEARCHCURVE_H
#define PCL_SEARCHCURVE_H

#include <ros/ros.h>

#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <fstream>
#include <iostream>

namespace pointcloud_filter {

	class PCL_searchcurve
	{
	public:
		PCL_searchcurve();
		~PCL_searchcurve();

	private:
		ros::NodeHandle nh_;


		ros::Publisher pub_pcl_;
		ros::Publisher pub_search_;
		ros::Publisher pub_plot_;
		ros::Publisher pub_map_;
		ros::Publisher pub_line_;
		ros::Publisher pub_ransac_line_;
		ros::Publisher pub_line_marker_;

		ros::Subscriber sub_pcl_;

		pcl::VoxelGrid<pcl::PointXYZI> voxel_filter_;

		tf::Transform transform_;

		tf::TransformBroadcaster* tfb_;

		nav_msgs::Path a_line_;
		nav_msgs::Path ransac_line_;

		
		//geometry_msgs::PoseArray 

		void pointcloud_cb_ (const sensor_msgs::PointCloud2ConstPtr& input);

		geometry_msgs::Point difference_point (geometry_msgs::Point pt1, geometry_msgs::Point pt2);

		double interpolate (long double t1, long double t2, long double t, double x1, double x2);

		void estimate_line(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, double b, double m);

		pcl::PointCloud<pcl::PointXYZI>::Ptr extract_groundpoint (pcl::PointCloud<pcl::PointXYZI>::Ptr, double b, double m);

		std::vector<double> ransac_estimate_line(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in);

		double distance_threshold_;

		double vector_threshold_;

		double resolution_;

		std::string pose_location_;

		std::ifstream pose_txt_;

		bool map_;

		std::vector<long double>* pose_time_;
		std::vector<double>* pose_x_;
		std::vector<double>* pose_y_;
		std::vector<double>* pose_yaw_;

		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_map;
		
	};
}




#endif
