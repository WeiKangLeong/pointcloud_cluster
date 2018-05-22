#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/console/time.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>

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

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_comeout(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_left_over(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_floor(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_floor_dense(new pcl::PointCloud<pcl::PointXYZ>);

void pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::console::TicToc time;
    time.tic ();

    pcl::fromROSMsg (*input, *cloud_comeout);

    int cloud_size = input->height * input->width;

    const float bad_point = std::numeric_limits<float>::quiet_NaN();
    pcl::PointXYZI no_point;
    no_point.x = no_point.y = no_point.z = bad_point;

    for (int i=0; i<cloud_comeout->size(); i++)
    {
        if (!std::isnan(cloud_comeout->points[i].x))
        {
            cloud_comeout->points[i].intensity = i;
        }
    }

        //std::cout<<"1: "<<cloud_size<<" "<<input->data.size()<<" "<<cloud_comeout->size()<<std::endl;

    tf::StampedTransform latest_imu_transform;
    try{
        tf_listener_->lookupTransform("wtf_odom", "wtf_base_link",
                                      ros::Time(0), latest_imu_transform);
    }catch (tf::TransformException &ex) {
        ROS_ERROR_STREAM("localizer_scan: Looking Transform Failed");
        return;
    }

//    tf::Vector3 amcl_position;
//    amcl_position = latest_imu_transform.getOrigin();
//    double current_amcl_x = (double)amcl_position.x();
//    double current_amcl_y = (double)amcl_position.y();


    pcl_ros::transformPointCloud(*cloud_comeout, *cloud_in, latest_imu_transform);

        //std::cout<<"2: "<<cloud_comeout->size()<<" "<<cloud_in->size()<<std::endl;

    grid_map_->filter_roadXY(cloud_in);

    cloud_filtered = grid_map_->GetCloud2(cloud_in);

    tf::Transform inverse_transform;
    inverse_transform = latest_imu_transform.inverse();
    pcl_ros::transformPointCloud(*cloud_filtered, *cloud_left_over, inverse_transform);


    //std::cout<<"cloud size: "<<cloud_in->size()<<" to "<<cloud_filtered->size()<<std::endl;

    if (cloud_filtered->size()>0)
    {
	sensor_msgs::PointCloud2 output3;
	pcl::toROSMsg (*cloud_filtered, output3);
	output3.header.frame_id = "map";
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

        /*std::cout<<"cloud_comeout: "<<cloud_comeout->size()<<" cloud_in: "<<cloud_in->size()<<" cloud_filtered: "<<
                   cloud_filtered->size()<<" cloud_leftover: "<<cloud_left_over->size()<<" cloud_rs: "<<cloud_rs->size()
                <<" "<<valid<<std::endl;*/

        sensor_msgs::PointCloud2 output2;
        pcl::toROSMsg (*cloud_rs, output2);
        output2.header.frame_id = "rslidar";
            output2.width = input->width;
            output2.height = input->height;
    //	output2.width = output2.data.size()/32;
            output2.point_step = 32;
            output2.row_step = input->row_step;
            output2.fields = input->fields;
        pub_move.publish (output2);

            std::cout<<"compare size: "<<input->data.size()<<" "<<output2.data.size()<<std::endl;

    //    sensor_msgs::PointCloud2 output4;
    //    pcl::toROSMsg (*cloud_left_over, output4);
    //    output4.header.frame_id = "map";
    //    pub_cluster.publish (output4);
    }

    grid_map_->ClearGrid();
    cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_left_over.reset(new pcl::PointCloud<pcl::PointXYZI>);

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
                if (floor_in_grid_->at(i)->at(j)==0)
                {
                    pcl::PointXYZ centroid;
                    centroid.x = j*floor_resolution;
                    centroid.y = i*floor_resolution;
                    centroid.z = 0.0;
                    cloud_floor->push_back(centroid);
                }
            }
        }

        grid_map_->ArrangeFloorandStoreXY(cloud_floor);

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg (*cloud_floor, output);
        output.header.frame_id = "map";
        pub_floor.publish(output);
        std::cout<<cloud_floor->size()<<" points map loaded to pointcloud"<<std::endl;
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

    grid_map_ = new Grid(RANGE_X, RANGE_Y, 0, RES, 0, 0);

    map_created_=false;

    tf_listener_ = new tf::TransformListener();

    ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2> ("origin_input", 1, pointcloud_cb);
    ros::Subscriber sub_map = nh.subscribe<nav_msgs::OccupancyGrid> ("/map_mask", 1, map_2d_cb);

    pub_move = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_at_origin", 1);
    pub_floor = nh.advertise<sensor_msgs::PointCloud2> ("/floor_points", 1);
    pub_static = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_out", 1);
    //pub_cluster = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_left", 1);


    ros::spin ();
}
