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

    pcl::PassThrough<pcl::PointXYZI> pass;

    double detect_x_, detect_y_, resolution_;

    std::vector<std::vector<int>* >* grid_table_;
    std::vector<int>* chess_plate_;
    std::vector<std::vector<int>* >* cluster_table_;

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
    pointcloud_input_ = nh_.subscribe<sensor_msgs::PointCloud2> ("/rslidar_points", 1, &grid_clustering::pointcloud_cb_, this);

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
}

void grid_clustering::pointcloud_cb_(const sensor_msgs::PointCloud2ConstPtr &input)
{
    pcl::console::TicToc time;
    time.tic ();

    pcl::PointCloud<pcl::PointXYZI>::Ptr first_input (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg (*input, *first_input);

    pass.setInputCloud (first_input);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-detect_x_, detect_x_);
    pass.filter (*first_input);

    pass.setInputCloud (first_input);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-detect_y_, detect_y_);
    pass.filter (*first_input);

    for (int i=0; i<first_input->size(); i++)
    {
        double grid_x = first_input->points[i].x + detect_x_;
        double grid_y = first_input->points[i].y + detect_y_;

        int place = int(grid_x*10)+int(grid_y*10)+1;

        grid_table_->at(place)->push_back(i);

        if (grid_table_->at(place)->size()>5)
        {
            chess_plate_->at(place) = 1;
        }
    }

    int new_cluster = 1;

    std::vector<int> cluster_grid;
    if (chess_plate_->at(0)==1)
    {
        new_cluster++;
        chess_plate_->at(0)==new_cluster;
    }
    for (int j=1; j<chess_plate_->size(); j++)
    {
        if (chess_plate_->at(j)==1)
        {
            if (j<600)
            {
                if (chess_plate_->at(j-1)>0)
                {
                    chess_plate_->at(j)=chess_plate_->at(j-1);
                }
                else
                {
                    new_cluster++;
                    chess_plate_->at(j)==new_cluster;
                }
            }
            else if (j%600==1)
            {
                if (chess_plate_->at(j-600)>0)
                {
                    chess_plate_->at(j)=chess_plate_->at(j-600);
                }
                else
                {
                    new_cluster++;
                    chess_plate_->at(j)==new_cluster;
                }
            }
            else
            {
                if (chess_plate_->at(j-600)>0)
                {
                    chess_plate_->at(j)=chess_plate_->at(j-600);
                }
                else if (chess_plate_->at(j-1)>0)
                {
                    chess_plate_->at(j)=chess_plate_->at(j-1);
                }
                else
                {
                    new_cluster++;
                    chess_plate_->at(j)==new_cluster;
                }
            }
        }


    }
    std::cout<<"total cluster found: "<<new_cluster<<std::endl;

    int color = 255/new_cluster;

    int number_of_grid=0;

    for (int l=1; l<new_cluster; l++)
    {
        for (int k=1; k<chess_plate_->size(); k++)
        {
            if (chess_plate_->at(k)==l)
            {
                cluster_grid.push_back(k);
                number_of_grid++;
            }

        }
        std::cout<<"cluster "<<l<<" got "<<cluster_grid.size()<<std::endl;

        //cluster_table_->push_back(cluster_grid);
        cluster_grid.clear();
        //cluster_grid = new std::vector<int>;

    }
    std::cout<<"number of grid with points: "<<number_of_grid<<std::endl;





    std::cout<<"cluster end... in " << time.toc () << " ms"<<std::endl;
    std::cout<<"total points: "<<first_input->size()<<std::endl;
}

void grid_clustering::clear_grid()
{
    grid_table_->clear();
    chess_plate_->clear();
    grid_table_->resize(4*(detect_x_/resolution_)*(detect_y_/resolution_));
    for (int j=0; j<grid_table_->size(); j++)
    {
        grid_table_->at(j) = new std::vector<int>;
    }
    //chess_plate_ = new std::vector<int>;
    chess_plate_->resize(4*(detect_x_/resolution_)*(detect_y_/resolution_));
}
