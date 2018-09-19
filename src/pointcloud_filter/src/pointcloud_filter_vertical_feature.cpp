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
std::vector<double>* sorting_neighbour_;
std::vector<int>* filter_road_;
std::vector<std::vector<double>* >* floor_level_;

tf::TransformListener *tf_listener_;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_largemap(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_gridmap(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_grid(new pcl::PointCloud<pcl::PointXYZI>);

    pass.setInputCloud (cloud_in);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-50.0, 50.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_in);

    pass.setInputCloud (cloud_in);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-50.0, 50.0);
    pass.filter (*cloud_in);

    pcl::PointXYZI min_point, max_point;
    pcl::getMinMax3D (*cloud_largemap, min_point, max_point);

    int range_x = int(max_point.x-min_point.x+1.0);
    int range_y = int(max_point.y-min_point.y+1.0);

    grid_map_ = new Grid(range_x, range_y, 0, RES, min_point.x, min_point.y);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pointcloud_filter_road");
  ros::NodeHandle nh;  

  nh.param("view_imu", view_imu, false);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("cloud_input", 1, cloud_cb);
  //ros::Subscriber sub_odom = nh.subscribe ("/icp_odom", 1, odom_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/road_map", 1);
  pub_object = nh.advertise<sensor_msgs::PointCloud2> ("/object_on_road", 1);
  marker_pub = nh.advertise<visualization_msgs::Marker>("/floor_normal", 10);
  pub_grid_map = nh.advertise<nav_msgs::OccupancyGrid> ("map", 1);

  tf_listener_ = new tf::TransformListener();

  sorting_neighbour_ = new std::vector<double>;
  filter_road_ = new std::vector<int>;
  floor_level_ = new std::vector<std::vector<double>* >;
  floor_level_->resize(x_max);
  for (int i=0; i<x_max; i++)
  {
      floor_level_->at(i) = new std::vector<double>;
      floor_level_->at(i)->resize(y_max);
  }

  icp.setMaximumIterations (30);
  //icp.setMaxCorrespondenceDistance(0.5);
  //icp.setTransformationEpsilon (1e-7);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_largemap(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_gridmap(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_grid(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile("/home/smaug/loam_cleantech_filtered.pcd", *cloud_largemap);
    std::cout<<"map loaded..."<<std::endl;

    pcl::PointXYZI min_point, max_point;
    pcl::getMinMax3D (*cloud_largemap, min_point, max_point);

    int range_x = int(max_point.x-min_point.x+1.0);
    int range_y = int(max_point.y-min_point.y+1.0);

    grid_map_ = new Grid(range_x, range_y, 0, RES, min_point.x, min_point.y);

    std::cout<<"map loaded with size: "<<cloud_largemap->size()<<" x: "<<range_x<<" y: "<<range_y<<std::endl;

    for (int i=0; i<cloud_largemap->size(); i++)
    {
        grid_map_->InsertXYZI_with_origin(cloud_largemap->points[i], i);
    }

    cloud_gridmap = grid_map_->find_vertical(cloud_largemap);

//    for (int lc=0; lc<cloud_gridmap->size(); lc++)
//    {
//        if (cloud_gridmap->points[lc].z==0.0)
//        {
//            cloud_gridmap->points[lc].intensity = 200;
//        }
//        else
//        {
//            cloud_gridmap->points[lc].intensity = 100;
//        }
//    }

    for (int lb=0; lb<cloud_gridmap->size(); lb++)
    {
        pcl::PointXYZI pt_on_map;
        if (cloud_gridmap->points[lb].z>0.0)
        {
            pt_on_map=cloud_gridmap->points[lb];
            cloud_grid->push_back(pt_on_map);
        }
    }

    sensor_msgs::PointCloud2 outputn, outputm;
    pcl::toROSMsg (*cloud_grid, outputn);
    outputn.header.stamp = ros::Time::now();
    outputn.header.frame_id = "wheelchair/map";
    pub.publish (outputn);

    pcl::toROSMsg (*cloud_largemap, outputm);
    outputm.header.stamp = ros::Time::now();
    outputm.header.frame_id = "wheelchair/map";
    pub_object.publish (outputm);

        nav_msgs::OccupancyGrid msg;
        msg.info.width = range_y/RES;
        msg.info.height = range_x/RES;
        msg.info.resolution = RES;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "wheelchair/map";

        filter_road_ = grid_map_->getMap();

        //std::cout<<"map size: "<<filter_road_->size()<<std::endl;

        for (int i=0; i<filter_road_->size(); i++)
        {
            msg.data.push_back(filter_road_->at(i));
        }

        pub_grid_map.publish(msg);


  // Spin
  ros::spin ();
}

