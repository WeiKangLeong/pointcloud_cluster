#ifndef PCL_ALIGN_H
#define PCL_ALIGN_H

#include <ros/ros.h>

#include "sensor_msgs/PointCloud2.h"

#include "tf/transform_listener.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_types.h>

namespace pointcloud_transform{
    class PCL_Align
    {
        public:
            PCL_Align();
            ~PCL_Align();

        private:
            ros::NodeHandle nh_;

            ros::Publisher pub_align_;

            ros::Subscriber sub_pcl_base_;
            ros::Subscriber sub_pcl_follow_;

            void base_cb_ (const sensor_msgs::PointCloud2ConstPtr& input);
            void follow_cb_ (const sensor_msgs::PointCloud2ConstPtr& input);

            tf::TransformListener tf_;

            tf::Transform transform_;

            tf::Quaternion quat_;

            double x_, y_, z_, roll_, pitch_, yaw_;

            std::string frame_id_;

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_base_;
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_follow_;
    };
}

#endif