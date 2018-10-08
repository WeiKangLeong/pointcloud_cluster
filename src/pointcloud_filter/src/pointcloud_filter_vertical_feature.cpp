#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <tf/transform_listener.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/time.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <visualization_msgs/Marker.h>

#include "../../../include/grid.h"
//#include <algorithm.h>

#define RES 0.1

Grid* grid_map_;

ros::Publisher pub, marker_pub, pub_object, pub_ransac, pub_grid_map;

pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
pcl::ExtractIndices<pcl::PointXYZ> extract;
pcl::PassThrough<pcl::PointXYZI> pass;
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

int x_max, y_max;

bool start, view_imu;
double a, b, c, d, norm;
double p_a, p_b, p_c, p_d, p_norm;
int change, min_neighbour, iteration;

std::string base_frame_;

double filter_radius_;

tf::Transform shift_cloud, shift_back_cloud;

tf::TransformListener *tf_listener_;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::console::TicToc time;
    time.tic();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_transform(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_gridmap(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_grid(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg (*input, *cloud_in);

    pass.setInputCloud (cloud_in);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-50.0, 50.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_in);

    pass.setInputCloud (cloud_in);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-50.0, 50.0);
    pass.filter (*cloud_in);

    pcl_ros::transformPointCloud (*cloud_in, *cloud_transform, shift_cloud);

    for (int i=0; i<cloud_transform->size(); i++)
    {
        grid_map_->InsertXYZI_with_origin(cloud_transform->points[i], i);
    }

    cloud_gridmap = grid_map_->find_vertical_feature(cloud_in);

    std::cout<<"cloud in size: "<<cloud_in->size()<<std::endl;

//    for (int lb=0; lb<cloud_gridmap->size(); lb++)
//    {
//        pcl::PointXYZI pt_on_map;
//        if (cloud_gridmap->points[lb].z>0.0)
//        {
//            pt_on_map=cloud_gridmap->points[lb];
//            cloud_grid->push_back(pt_on_map);
//        }
//    }

    pcl_ros::transformPointCloud(*cloud_gridmap, *cloud_gridmap, shift_back_cloud);

    tf::StampedTransform align_baselink;

    try{
        tf_listener_->lookupTransform(base_frame_, input->header.frame_id, ros::Time(0), align_baselink);
    }catch (tf::TransformException &ex) {
        ROS_ERROR("Lookup transform for %s and %s failed", base_frame_.c_str(), input->header.frame_id.c_str());
        return;
    }

    pcl_ros::transformPointCloud(*cloud_gridmap, *cloud_gridmap, align_baselink);

    sensor_msgs::PointCloud2 outputn, outputm;
    pcl::toROSMsg (*cloud_gridmap, outputn);
    outputn.header.stamp = input->header.stamp;
    outputn.header.frame_id = base_frame_;
    pub.publish (outputn);

//    pcl::toROSMsg (*cloud_in, outputm);
//    outputm.header.stamp = ros::Time::now();
//    outputm.header.frame_id = "iMiev/base_link";
//    pub_object.publish (outputm);
    std::cout<<"points left: "<<cloud_gridmap->size()<<" in "<<time.toc()<<" ms."<<std::endl;
    grid_map_->ClearGrid();
}

int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pointcloud_filter_vertical");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    filter_radius_ = 50.0;

    priv_nh.getParam("filter_radius", filter_radius_);
    priv_nh.getParam("base_frame", base_frame_);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("cloud_input", 1, cloud_cb);
    //ros::Subscriber sub_odom = nh.subscribe ("/icp_odom", 1, odom_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("vertical_points", 1);
//    pub_object = nh.advertise<sensor_msgs::PointCloud2> ("/object_on_road", 1);
//    marker_pub = nh.advertise<visualization_msgs::Marker>("/floor_normal", 10);
//    pub_grid_map = nh.advertise<nav_msgs::OccupancyGrid> ("map", 1);

    tf_listener_ = new tf::TransformListener();

    shift_cloud.setOrigin(tf::Vector3(filter_radius_, filter_radius_, 0.0));
    shift_cloud.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    shift_back_cloud.setOrigin(tf::Vector3(-filter_radius_, -filter_radius_, 0.0));
    shift_back_cloud.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

    grid_map_ = new Grid(filter_radius_*2, filter_radius_*2, 0, RES, 0.0, 0.0);


    // Spin
    ros::spin ();
}

