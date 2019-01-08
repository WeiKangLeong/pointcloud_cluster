/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

// roscpp
#include "ros/ros.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <nav_msgs/OccupancyGrid.h>

// For pcl format
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/common/common.h>

#include "dbscan.h"

class grid_clustering
{
public:
    grid_clustering();


private:
    //callback
    void pointcloud_cb_ (const sensor_msgs::PointCloud2ConstPtr& input);

    ros::NodeHandle nh_;

    ros::Subscriber pointcloud_input_;

    ros::Publisher map_pub_;
    ros::Publisher adjusted_pub_;
    ros::Publisher clustered_pub_;

    pcl::PassThrough<pcl::PointXYZI> pass;
    pcl::VoxelGrid<pcl::PointXYZI> voxel;

    double detect_x_, detect_y_, resolution_;

    std::vector<std::vector<int>* >* grid_table_;
    std::vector<int>* chess_plate_;
    std::vector<std::vector<int>* >* cluster_table_;

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>* cloud_table_;

    nav_msgs::OccupancyGrid map_;

    DBSCAN* dbscan;

    int total_grid_;

    // function to support
    void clear_grid();

};

int main (int argc, char** argv)
{
    ros::init (argc, argv, "grid_cluster");

    grid_clustering use_grid;

    ros::spin();

    return(0);
}

grid_clustering::grid_clustering()
{
    pointcloud_input_ = nh_.subscribe<sensor_msgs::PointCloud2> ("cloud_input", 1, &grid_clustering::pointcloud_cb_, this);

    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid> ("grid_map", 1);
    adjusted_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("adjusted_input", 1);
    clustered_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("clustered_output", 1);

    detect_x_ = 30.0;
    detect_y_ = 30.0;
    resolution_ = 0.1;

    ros::NodeHandle priv_nh_("~");
    priv_nh_.getParam("detect_x", detect_x_);
    priv_nh_.getParam("detect_y", detect_y_);
    priv_nh_.getParam("resolution", resolution_);

    cluster_table_ = new std::vector<std::vector<int>* >;
    grid_table_ = new std::vector<std::vector<int>* >;
    grid_table_->resize(4*(detect_x_/resolution_)*(detect_y_/resolution_));
    for (int j=0; j<grid_table_->size(); j++)
    {
        grid_table_->at(j) = new std::vector<int>;
    }
    chess_plate_ = new std::vector<int>;
    chess_plate_->resize(4*(detect_x_/resolution_)*(detect_y_/resolution_));

    cloud_table_ = new std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>;

    map_.info.resolution = resolution_;
    map_.info.width = detect_y_*2/resolution_;
    map_.info.height = detect_x_*2/resolution_;
    map_.header.frame_id = "iMiev/base_link";
    //map_.header.stamp = ros::Time::now();

    total_grid_ = int(map_.info.width)*int(map_.info.height);

    std::cout<<"total grid cell: "<<total_grid_<<std::endl;

    for (int m=0; m<total_grid_; m++)
    {
        map_.data.push_back(-1);
    }
//    for (int mm=180000; mm<180600; mm++)
//    {
//        map_.data[mm] = 0;
//    }
    //std::cout<<int(map_.data[2])<<std::endl;

}

void grid_clustering::pointcloud_cb_(const sensor_msgs::PointCloud2ConstPtr &input)
{
    pcl::console::TicToc time;
    time.tic ();

    pcl::PointCloud<pcl::PointXYZI>::Ptr first_input (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr adjusted_input (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr compressed_input (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg (*input, *first_input);

    std::cout<<"total points: "<<first_input->size()<<std::endl;

    pass.setInputCloud (first_input);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-detect_x_, detect_x_);
    pass.filter (*first_input);

    pass.setInputCloud (first_input);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-detect_y_, detect_y_);
    pass.filter (*first_input);

    voxel.setLeafSize (0.1, 0.1, 0.1);
    voxel.setInputCloud (first_input);
    voxel.filter (*compressed_input);

    std::cout<<"filtered input: "<<compressed_input->size()<<std::endl;

    for (int i=0; i<compressed_input->size(); i++)
    {
        double grid_x = compressed_input->points[i].x + detect_x_;
        double grid_y = compressed_input->points[i].y + detect_y_;

//        pcl::PointXYZI grid_xy;
//        grid_xy.x = grid_x;
//        grid_xy.y = grid_y;
//        adjusted_input->push_back(grid_xy);

        int place = int(grid_x/resolution_)*map_.info.height+int(grid_y/resolution_);
        int map_place = int(grid_y/resolution_)*map_.info.width+int(grid_x/resolution_);

//        if (place>=total_grid_ || place<1)
//        {
//            std::cout<<"place: "<<place<<" map_place: "<<map_place<<std::endl;
//        }
//        if (map_place>=total_grid_ || map_place<1)
//        {
//            std::cout<<"place: "<<place<<" map_place: "<<map_place<<std::endl;
//        }

        grid_table_->at(place)->push_back(i);

        //std::cout<<"x:"<<grid_x<<" y:"<<grid_y<<"="<<place<<std::endl;

//        if (grid_table_->at(place)->size()>5)
//        {
            chess_plate_->at(place) = 1;
            map_.data[map_place] = 0;
//        }
    }



    for (int ii=0; ii<chess_plate_->size(); ii++)
    {
        if (chess_plate_->at(ii)==1)
        {
            pcl::PointXYZI g_xy;
            g_xy.x = (ii)/(double(map_.info.height))*resolution_;
            g_xy.y = (ii)%int(map_.info.width)*resolution_;

            adjusted_input->push_back(g_xy);
        }
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg (*adjusted_input, output);
    output.header.frame_id = "iMiev/base_link";
    adjusted_pub_.publish(output);

    std::cout<<"compressed input: "<<adjusted_input->size()<<std::endl;

    cloud_table_ = dbscan->dbscan_cluster_flat_intensity(adjusted_input);

    //compressed_input->clear();


//    int new_cluster = 1;

//    std::vector<int> cluster_grid;
//    if (chess_plate_->at(0)==1)
//    {
//        new_cluster++;
//        chess_plate_->at(0)==new_cluster;
//    }
//    for (int j=1; j<chess_plate_->size(); j++)
//    {
//        if (chess_plate_->at(j)==1)
//        {
//            if (j<600)
//            {
//                if (chess_plate_->at(j-1)>0)
//                {
//                    chess_plate_->at(j)=chess_plate_->at(j-1);
//                }
//                else
//                {
//                    new_cluster++;
//                    chess_plate_->at(j)==new_cluster;
//                }
//            }
//            else if (j%600==1)
//            {
//                if (chess_plate_->at(j-600)>0)
//                {
//                    chess_plate_->at(j)=chess_plate_->at(j-600);
//                }
//                else
//                {
//                    new_cluster++;
//                    chess_plate_->at(j)==new_cluster;
//                }
//            }
//            else
//            {
//                if (chess_plate_->at(j-600)>0)
//                {
//                    chess_plate_->at(j)=chess_plate_->at(j-600);
//                }
//                else if (chess_plate_->at(j-1)>0)
//                {
//                    chess_plate_->at(j)=chess_plate_->at(j-1);
//                }
//                else
//                {
//                    new_cluster++;
//                    chess_plate_->at(j)==new_cluster;
//                }
//            }
//        }


//    }


    std::cout<<"total cluster found: "<<cloud_table_->size()<<std::endl;

    int color = 255/cloud_table_->size();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZI>);

    for (int kk=0; kk<cloud_table_->size(); kk++)
    {
        for (int kkk=0; kkk<cloud_table_->at(kk)->size(); kkk++)
        {
            pcl::PointXYZI cluster_point;
            cluster_point = cloud_table_->at(kk)->points[kkk];
            cluster_point.intensity = (kk+1)*color;
            cluster_cloud->push_back(cluster_point);
        }
    }

    sensor_msgs::PointCloud2 output2;
    pcl::toROSMsg (*cluster_cloud, output2);
    output2.header.frame_id = "iMiev/base_link";
    clustered_pub_.publish(output2);

    std::cout<<"cluster end... in " << time.toc () << " ms"<<std::endl;


    map_.header.stamp = input->header.stamp;
    map_pub_.publish(map_);

    clear_grid();
}

void grid_clustering::clear_grid()
{
    grid_table_->clear();
    chess_plate_->clear();

    cloud_table_->clear();

    grid_table_->resize(4*(detect_x_/resolution_)*(detect_y_/resolution_));
    for (int j=0; j<grid_table_->size(); j++)
    {
        grid_table_->at(j) = new std::vector<int>;
    }
    //chess_plate_ = new std::vector<int>;
    chess_plate_->resize(4*(detect_x_/resolution_)*(detect_y_/resolution_));

    cloud_table_ = new std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>;


    for (int m=0; m<int(map_.info.width)*int(map_.info.height); m++)
    {
        map_.data[m] = -1;
    }
}
