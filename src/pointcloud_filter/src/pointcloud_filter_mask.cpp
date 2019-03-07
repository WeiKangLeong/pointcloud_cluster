#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int8.h>

#include <pcl/console/time.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include "../../../include/grid.h"

#include <math.h>

#define RES 0.5
#define RANGE_X 800.0
#define RANGE_Y 800.0

Grid* grid_map_;

bool map_created_;

ros::Publisher pub_floor, pub_static, pub_move;

tf::TransformListener *tf_listener_;

std::vector<std::vector<int>* >* floor_in_grid_;

std::string map_frame_, base_frame_;
int filter_integer_;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_comeout(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left_over(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_floor(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_floor_dense(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_map (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_minimap (new pcl::PointCloud<pcl::PointXYZ>);

pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
pcl::PassThrough<pcl::PointXYZ> pass;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr dbscan_kdtree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points (new pcl::PointCloud<pcl::PointXYZ>);
    if (cloud1->size()==0 || cloud2->size()==0)
    {
        std::cout<<"no point cloud"<<std::endl;
    }
    else
    {
        pcl::console::TicToc time;
        time.tic ();


        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        kdtree.setInputCloud(cloud2);

        for (int i=0; i<cloud1->size(); i++)
        {
            pcl::PointXYZ searchpoint;

            searchpoint.x = cloud1->points[i].x;
            searchpoint.y = cloud1->points[i].y;

            double radius = 1.0;

            if ( kdtree.radiusSearch (searchpoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
            {
        //    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
        //      std::cout << "    "  <<   cloudin->points[ pointIdxRadiusSearch[i] ].x
        //                << " " << cloudin->points[ pointIdxRadiusSearch[i] ].y
        //                << " " << cloudin->points[ pointIdxRadiusSearch[i] ].z
        //                << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
    //            std::cout<<"cloud_o have : "<<pointIdxRadiusSearch.size()<<" points. Kd tree takes: "<<time.toc()<<" ms."<<std::endl;

            }
            else
            {
                filtered_points->push_back(searchpoint);
            }
        }


    }
    return filtered_points;

}

void pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::console::TicToc time;
    time.tic ();

    pcl::fromROSMsg (*input, *cloud_comeout);

    //int cloud_size = input->height * input->width;

//    const float bad_point = std::numeric_limits<float>::quiet_NaN();
//    pcl::PointXYZI no_point;
//    no_point.x = no_point.y = no_point.z = bad_point;

//    for (int i=0; i<cloud_comeout->size(); i++)
//    {
//        if (!std::isnan(cloud_comeout->points[i].x))
//        {
//            cloud_comeout->points[i].intensity = i;
//        }
//    }

        //std::cout<<"1: "<<cloud_size<<" "<<input->data.size()<<" "<<cloud_comeout->size()<<std::endl;

    tf::StampedTransform latest_odom_transform;
    try{
        tf_listener_->lookupTransform(map_frame_, base_frame_,
                                      ros::Time(0), latest_odom_transform);
    }catch (tf::TransformException &ex) {
        ROS_ERROR_STREAM("localizer_scan: Looking Transform Failed");
        return;
    }

    tf::Vector3 wheel_position = latest_odom_transform.getOrigin();
    double mini_map_x = wheel_position.x();
    double mini_map_y = wheel_position.y();

    pass.setInputCloud (filtered_map);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-10.0+wheel_position.x(), 10.0+wheel_position.x());
    //pass.setFilterLimits (-30.0, 30.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_minimap);

    pass.setInputCloud (cloud_minimap);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-10.0+wheel_position.y(), 10.0+wheel_position.y());
    //pass.setFilterLimits (-30.0, 30.0);
    pass.filter (*cloud_minimap);

    std::cout<<"x: "<<wheel_position.x()<<" y: "<<wheel_position.y()<<" "<<cloud_comeout->size()<<std::endl;

    pcl_ros::transformPointCloud(*cloud_comeout, *cloud_in, latest_odom_transform);

    std::cout<<filtered_map->size()<<" "<<cloud_minimap->size()<<" "<<cloud_in->size()<<std::endl;

    cloud_filtered = dbscan_kdtree(cloud_in, cloud_minimap);

    if (cloud_filtered->size()>0)
    {
        tf::Transform inverse_transform;
        inverse_transform = latest_odom_transform.inverse();
        pcl_ros::transformPointCloud(*cloud_filtered, *cloud_left_over, inverse_transform);

        sensor_msgs::PointCloud2 filter_pcl2;
        pcl::toROSMsg (*cloud_left_over, filter_pcl2);
        filter_pcl2.header = input->header;
        pub_move.publish(filter_pcl2);


        std::cout<<"cloud size: "<<cloud_in->size()<<" to "<<cloud_filtered->size()<<std::endl;
    }


 /*   if (cloud_filtered->size()>0)
    {
	sensor_msgs::PointCloud2 output3;
	pcl::toROSMsg (*cloud_filtered, output3);
        output3.header.frame_id = map_frame_;
	pub_static.publish (output3);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rs (new pcl::PointCloud<pcl::PointXYZI>);
        //cloud_rs.height = input->height;
        //cloud_rs.width = input->width;
        //cloud_rs.is_dense = input->is_dense;

        //int valid = 0;
        for (int j=0; j<cloud_size; j++)
        {            
            cloud_rs->push_back(no_point);
        }
        for (int k=0; k<cloud_left_over->size(); k++)
        {
            cloud_rs->points[cloud_left_over->points[k].intensity] = cloud_left_over->points[k];
        }



        sensor_msgs::PointCloud2 output2;
        pcl::toROSMsg (*cloud_rs, output2);
        output2.header.frame_id = input->header.frame_id;
            output2.width = input->width;
            output2.height = input->height;
    //	output2.width = output2.data.size()/32;
            output2.point_step = 32;
            output2.row_step = input->row_step;
            output2.fields = input->fields;
        pub_move.publish (output2);

            std::cout<<"compare size: "<<input->data.size()<<" "<<output2.data.size()<<std::endl;


    }
*/
    //grid_map_->ClearGrid();
    cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_left_over.reset(new pcl::PointCloud<pcl::PointXYZ>);

    //std::cout<<"Time used: "<<time.toc()<<std::endl;

}

void map_2d_cb(nav_msgs::OccupancyGrid msg)
{
    if (!map_created_)
    {
        int floor_width = msg.info.width;
        int floor_height = msg.info.height;
        double floor_resolution = msg.info.resolution;

        floor_in_grid_ = new std::vector<std::vector<int>* >;
        floor_in_grid_->resize(floor_height);
        for (int i=0; i<floor_height; i++)
        {
            floor_in_grid_->at(i) = new std::vector<int>;
            floor_in_grid_->at(i)->resize(floor_width);
        }
        int k=0;

        for (int i=0; i<floor_height; i++)
        {
            for (int j=0; j<floor_width; j++)
            {
                floor_in_grid_->at(i)->at(j) = msg.data[k];
                k++;
                if (floor_in_grid_->at(i)->at(j)==100)
                {
                    pcl::PointXYZ centroid;
                    centroid.x = j*floor_resolution;
                    centroid.y = i*floor_resolution;
                    centroid.z = 0.0;
                    cloud_floor->push_back(centroid);
                }
            }
        }

        //grid_map_->ArrangeFloorandStoreXY(cloud_floor);


        voxel_filter.setLeafSize (0.5, 0.5, 0.5);
        voxel_filter.setInputCloud (cloud_floor);
        voxel_filter.filter (*filtered_map);

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg (*filtered_map, output);
        output.header.frame_id = map_frame_;
        pub_floor.publish(output);
        std::cout<<filtered_map->size()<<" points map loaded to pointcloud"<<std::endl;
        //filtered_map.reset(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_floor.reset(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_floor_dense.reset(new pcl::PointCloud<pcl::PointXYZ>);

        map_created_=true;
    }


}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pointcloud_filter_mask");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    map_frame_ = "map";
    filter_integer_ = 0;
    priv_nh.getParam("map_frame", map_frame_);
    priv_nh.getParam("base_frame", base_frame_);
    priv_nh.getParam("filter_integer", filter_integer_);

    //grid_map_ = new Grid(RANGE_X, RANGE_Y, 0, RES, 0, 0);

    map_created_=false;

    tf_listener_ = new tf::TransformListener();

    ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2> ("origin_input", 1, pointcloud_cb);
    ros::Subscriber sub_map = nh.subscribe<nav_msgs::OccupancyGrid> ("map_mask", 1, map_2d_cb);

    pub_move = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_left", 1);
    pub_floor = nh.advertise<sensor_msgs::PointCloud2> ("/floor_points", 1);
    pub_static = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_out", 1);
    //pub_cluster = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_left", 1);


    ros::spin ();
}
