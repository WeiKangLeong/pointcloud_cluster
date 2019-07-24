#ifndef PCL_VISUALNORMAL_H
#define PCL_VISUALNORMAL_H

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>

namespace pointcloud_filter 
{
	class PCL_visualizenormal
	{
		public:
			PCL_visualizenormal();
		private:
			ros::NodeHandle nh_;

			ros::Subscriber sub_cloud_;

			ros::Publisher pub_normal_;
			ros::Publisher pub_leftover_;

			void pointcloud_cb_ (const sensor_msgs::PointCloud2ConstPtr& input);

			visualization_msgs::MarkerArray marker_arrow_;

			double view_point_x_, view_point_y_, view_point_z_;

	};
}

#endif
