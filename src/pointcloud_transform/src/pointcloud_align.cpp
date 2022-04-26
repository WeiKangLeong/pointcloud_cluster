#include "pointcloud_transform/pointcloud_align.h"

namespace pointcloud_transform {

PCL_Align::PCL_Align()
{
    sub_pcl_base_ = nh_.subscribe <sensor_msgs::PointCloud2> ("base_points", 1, &PCL_Align::base_cb_, this);
    sub_pcl_follow_ = nh_.subscribe <sensor_msgs::PointCloud2> ("follow_points", 1, &PCL_Align::follow_cb_, this);

    cloud_follow_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_base_.reset(new pcl::PointCloud<pcl::PointXYZI>);
}