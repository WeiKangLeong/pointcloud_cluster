#ifndef PCL_READJUST_H
#define PCL_READJUST_H

#include <ros/ros.h>

#include "sensor_msgs/PointCloud2.h"

#include "tf/transform_listener.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_types.h>

namespace pointcloud_transform {

	class PCL_readjust
	{
	public:
		PCL_readjust();
		~PCL_readjust();

	private:
		ros::NodeHandle nh_;

		ros::Publisher pub_pcl_;

		ros::Subscriber sub_pcl_;

		void pointcloud_cb_ (const sensor_msgs::PointCloud2ConstPtr& input);

		tf::TransformListener tf_;

		tf::Transform transform_;

		tf::Quaternion quat_;

		double x_, y_, z_, roll_, pitch_, yaw_;

		std::string frame_id_;

		bool cut_half_;
		
	};
}




#endif
