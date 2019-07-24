#ifndef PCL_COMBINE_H
#define PCL_COMBINE_H

#include <ros/ros.h>

#include "sensor_msgs/PointCloud2.h"

#include "tf/transform_listener.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_types.h>

namespace pointcloud_transform {

	class PCL_combine
	{
	public:
		PCL_combine();
		~PCL_combine();

	private:
		ros::NodeHandle nh_;

		ros::Publisher pub_pcl_;

		ros::Subscriber sub_top_;
		ros::Subscriber sub_rear_;
		ros::Subscriber sub_front_;

		void top_front_cb_ (const sensor_msgs::PointCloud2ConstPtr& input);

		void top_rear_cb_ (const sensor_msgs::PointCloud2ConstPtr& input);

		void bottom_front_cb_ (const sensor_msgs::PointCloud2ConstPtr& input);

		bool compare_time_ (std_msgs::Header time, bool front_or_rear);

		void combine_pcl_ (sensor_msgs::PointCloud2 front_pcl, sensor_msgs::PointCloud2 rear_pcl);

		tf::TransformListener tf_;

		tf::Transform transform_;

		tf::Quaternion quat_;

		double x_, y_, z_, roll_, pitch_, yaw_;

		sensor_msgs::PointCloud2 top_front_buffer_, top_rear_buffer_;

		std_msgs::Header top_front_header_, top_rear_header_;
		
	};
}




#endif
