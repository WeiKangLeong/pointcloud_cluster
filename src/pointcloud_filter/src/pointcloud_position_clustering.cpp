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
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/common/common.h>

#include "dbscan.h"

class position_clustering
{
public:
    position_clustering();


private:
    //callback
    void pointcloud_cb_ (const sensor_msgs::PointCloud2ConstPtr& input);

    ros::NodeHandle nh_;

    ros::Subscriber pointcloud_input_;

    ros::Publisher map_pub_;
    ros::Publisher first_layer_;
    ros::Publisher second_layer_;
    ros::Publisher third_layer_;
    ros::Publisher fourth_layer_;
    ros::Publisher fifth_layer_;
    ros::Publisher sixth_layer_;
    ros::Publisher seventh_layer_;
    ros::Publisher eighth_layer_;
    ros::Publisher ninth_layer_;
    ros::Publisher tenth_layer_;
    ros::Publisher eleventh_layer_;
    ros::Publisher twelve_layer_;
    ros::Publisher thirteen_layer_;
    ros::Publisher fourteen_layer_;
    ros::Publisher fifteen_layer_;
    ros::Publisher sixteen_layer_;
    ros::Publisher seventeen_layer_;
    ros::Publisher eighteen_layer_;
    ros::Publisher nineteen_layer_;
    ros::Publisher twenty_layer_;
    ros::Publisher twentyone_layer_;
    ros::Publisher twentytwo_layer_;
    ros::Publisher twentythree_layer_;
    ros::Publisher twentyfour_layer_;
    ros::Publisher twentyfive_layer_;
    ros::Publisher twentysix_layer_;
    ros::Publisher twentyseven_layer_;
    ros::Publisher twentyeight_layer_;
    ros::Publisher twentynine_layer_;
    ros::Publisher thirty_layer_;
    ros::Publisher thirtyone_layer_;
    ros::Publisher thirtytwo_layer_;


    pcl::PassThrough<pcl::PointXYZI> pass;
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

    double detect_x_, detect_y_, resolution_;
    int total_grid_;
    double density_;

    std::vector<std::vector<int>* >* grid_table_;
    std::vector<int>* chess_plate_;
    std::vector<std::vector<int>* >* cluster_table_;

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>* cloud_table_;

    nav_msgs::OccupancyGrid map_;

    DBSCAN* dbscan;    

    // function to support
    void clear_grid();

};

int main (int argc, char** argv)
{
    ros::init (argc, argv, "position_cluster");

    position_clustering use_pose;

    ros::spin();

    return(0);
}

position_clustering::position_clustering()
{
    pointcloud_input_ = nh_.subscribe<sensor_msgs::PointCloud2> ("cloud_input", 1, &position_clustering::pointcloud_cb_, this);

    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid> ("grid_map", 1);
    first_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer1", 1);
    second_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer2", 1);
    third_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer3", 1);
    fourth_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer4", 1);
     fifth_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer5", 1);
     sixth_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer6", 1);
     seventh_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer7", 1);
     eighth_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer8", 1);
     ninth_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer9", 1);
     tenth_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer10", 1);
     eleventh_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer11", 1);
     twelve_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer12", 1);
     thirteen_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer13", 1);
     fourteen_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer14", 1);
     fifteen_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer15", 1);
     sixteen_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer16", 1);
     seventeen_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer17", 1);
     eighteen_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer18", 1);
     nineteen_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer19", 1);
     twenty_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer20", 1);
     twentyone_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer21", 1);
     twentytwo_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer22", 1);
     twentythree_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer23", 1);
     twentyfour_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer24", 1);
     twentyfive_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer25", 1);
     twentysix_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer26", 1);
     twentyseven_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer27", 1);
     twentyeight_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer28", 1);
     twentynine_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer29", 1);
     thirty_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer30", 1);
     thirtyone_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer31", 1);
     thirtytwo_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer32", 1);

    detect_x_ = 30.0;
    detect_y_ = 30.0;
    resolution_ = 0.1;
    density_ = 0.1;

    ros::NodeHandle priv_nh_("~");
    priv_nh_.getParam("detect_x", detect_x_);
    priv_nh_.getParam("detect_y", detect_y_);
    priv_nh_.getParam("resolution", resolution_);
    priv_nh_.getParam("density", density_);



}

void position_clustering::pointcloud_cb_(const sensor_msgs::PointCloud2ConstPtr &input)
{
    pcl::console::TicToc time;
    time.tic ();

    pcl::PointCloud<pcl::PointXYZI>::Ptr first_input (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr adjusted_input (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr compressed_input (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer1 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer2 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer3 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer4 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer5 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer6 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer7 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer8 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer9 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer10 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer11 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer12 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer13 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer14 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer15 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer16 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer17 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer18 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer19 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer20 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer21 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer22 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer23 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer24 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer25 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer26 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer27 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer28 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer29 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer30 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer31 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr layer32 (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg (*input, *first_input);

    std::cout<<"size of cloud: "<<first_input->size()<<std::endl;

    int nan_pt = 0;

    for (int k=0; k<first_input->size(); k++)
    {
        if (!pcl::isFinite(first_input->points[k]))
        {
            nan_pt++;
        }
        if (k<2028)
        {
            layer1->push_back(first_input->points[k]);
        }
        else if (k<2028*2)
        {
            layer2->push_back(first_input->points[k]);
        }
        else if (k<2028*3)
        {
            layer3->push_back(first_input->points[k]);
        }
        else if (k<2028*4)
        {
            layer4->push_back(first_input->points[k]);
        }
        else if (k<2028*5)
        {
            layer5->push_back(first_input->points[k]);
        }
        else if (k<2028*6)
        {
            layer6->push_back(first_input->points[k]);
        }
        else if (k<2028*7)
        {
            layer7->push_back(first_input->points[k]);
        }
        else if (k<2028*8)
        {
            layer8->push_back(first_input->points[k]);
        }
        else if (k<2028*9)
        {
            layer9->push_back(first_input->points[k]);
        }
        else if (k<2028*10)
        {
            layer10->push_back(first_input->points[k]);
        }
        else if (k<2028*11)
        {
            layer11->push_back(first_input->points[k]);
        }
        else if (k<2028*12)
        {
            layer12->push_back(first_input->points[k]);
        }
        else if (k<2028*13)
        {
            layer13->push_back(first_input->points[k]);
        }
        else if (k<2028*14)
        {
            layer14->push_back(first_input->points[k]);
        }
        else if (k<2028*15)
        {
            layer15->push_back(first_input->points[k]);
        }
        else if (k<2028*16)
        {
            layer16->push_back(first_input->points[k]);
        }
        else if (k<2028*17)
        {
            layer17->push_back(first_input->points[k]);
        }
        else if (k<2028*18)
        {
            layer18->push_back(first_input->points[k]);
        }
        else if (k<2028*19)
        {
            layer19->push_back(first_input->points[k]);
        }
        else if (k<2028*20)
        {
            layer20->push_back(first_input->points[k]);
        }
        else if (k<2028*21)
        {
            layer21->push_back(first_input->points[k]);
        }
        else if (k<2028*22)
        {
            layer22->push_back(first_input->points[k]);
        }
        else if (k<2028*23)
        {
            layer23->push_back(first_input->points[k]);
        }
        else if (k<2028*24)
        {
            layer24->push_back(first_input->points[k]);
        }
        else if (k<2028*25)
        {
            layer25->push_back(first_input->points[k]);
        }
        else if (k<2028*26)
        {
            layer26->push_back(first_input->points[k]);
        }
        else if (k<2028*27)
        {
            layer27->push_back(first_input->points[k]);
        }
        else if (k<2028*28)
        {
            layer28->push_back(first_input->points[k]);
        }
        else if (k<2028*29)
        {
            layer29->push_back(first_input->points[k]);
        }
        else if (k<2028*30)
        {
            layer30->push_back(first_input->points[k]);
        }
        else if (k<2028*31)
        {
            layer31->push_back(first_input->points[k]);
        }
        else if (k<2028*32)
        {
            layer32->push_back(first_input->points[k]);
        }

    }
    sensor_msgs::PointCloud2 cloud1,cloud2,cloud3,cloud4,cloud5,cloud6,cloud7,cloud8,cloud9,cloud10,cloud11,cloud12,cloud13,cloud14,
            cloud15,cloud16,cloud17,cloud18,cloud19,cloud20,cloud21,cloud22,cloud23,cloud24,cloud25,cloud26,cloud27,cloud28,cloud29,
            cloud30,cloud31,cloud32;
    pcl::toROSMsg (*layer1, cloud1);
    cloud1.header = input->header;
    first_layer_.publish(cloud1);
    pcl::toROSMsg (*layer2, cloud2);
    cloud2.header = input->header;
    second_layer_.publish(cloud2);
    pcl::toROSMsg (*layer3, cloud3);
    cloud3.header = input->header;
    third_layer_.publish(cloud3);
    pcl::toROSMsg (*layer4, cloud4);
    cloud4.header = input->header;
    fourth_layer_.publish(cloud4);
    pcl::toROSMsg (*layer5, cloud5);
    cloud5.header = input->header;
    fifth_layer_.publish(cloud5);
    pcl::toROSMsg (*layer6, cloud6);
    cloud6.header = input->header;
    sixth_layer_.publish(cloud6);
    pcl::toROSMsg (*layer7, cloud7);
    cloud7.header = input->header;
    seventh_layer_.publish(cloud7);
    pcl::toROSMsg (*layer8, cloud8);
    cloud8.header = input->header;
    eighth_layer_.publish(cloud8);
    pcl::toROSMsg (*layer9, cloud9);
    cloud9.header = input->header;
    ninth_layer_.publish(cloud9);
    pcl::toROSMsg (*layer10, cloud10);
    cloud10.header = input->header;
    tenth_layer_.publish(cloud10);
    pcl::toROSMsg (*layer11, cloud11);
    cloud11.header = input->header;
    eleventh_layer_.publish(cloud11);
    pcl::toROSMsg (*layer12, cloud12);
    cloud12.header = input->header;
    twelve_layer_.publish(cloud12);
    pcl::toROSMsg (*layer13, cloud13);
    cloud13.header = input->header;
    thirteen_layer_.publish(cloud13);
    pcl::toROSMsg (*layer14, cloud14);
    cloud14.header = input->header;
    fourteen_layer_.publish(cloud14);
    pcl::toROSMsg (*layer15, cloud15);
    cloud15.header = input->header;
    fifteen_layer_.publish(cloud15);
    pcl::toROSMsg (*layer16, cloud16);
    cloud16.header = input->header;
    sixteen_layer_.publish(cloud16);
    pcl::toROSMsg (*layer17, cloud17);
    cloud17.header = input->header;
    seventeen_layer_.publish(cloud17);
    pcl::toROSMsg (*layer18, cloud18);
    cloud18.header = input->header;
    eighteen_layer_.publish(cloud18);
    pcl::toROSMsg (*layer19, cloud19);
    cloud19.header = input->header;
    nineteen_layer_.publish(cloud19);
    pcl::toROSMsg (*layer20, cloud20);
    cloud20.header = input->header;
    twenty_layer_.publish(cloud20);
    pcl::toROSMsg (*layer21, cloud21);
    cloud21.header = input->header;
    twentyone_layer_.publish(cloud21);
    pcl::toROSMsg (*layer22, cloud22);
    cloud22.header = input->header;
    twentytwo_layer_.publish(cloud22);
    pcl::toROSMsg (*layer23, cloud23);
    cloud23.header = input->header;
    twentythree_layer_.publish(cloud23);
    pcl::toROSMsg (*layer24, cloud24);
    cloud24.header = input->header;
    twentyfour_layer_.publish(cloud24);
    pcl::toROSMsg (*layer25, cloud25);
    cloud25.header = input->header;
    twentyfive_layer_.publish(cloud25);
    pcl::toROSMsg (*layer26, cloud26);
    cloud26.header = input->header;
    twentysix_layer_.publish(cloud26);
    pcl::toROSMsg (*layer27, cloud27);
    cloud27.header = input->header;
    twentyseven_layer_.publish(cloud27);
    pcl::toROSMsg (*layer28, cloud28);
    cloud28.header = input->header;
    twentyeight_layer_.publish(cloud28);
    pcl::toROSMsg (*layer29, cloud29);
    cloud29.header = input->header;
    twentynine_layer_.publish(cloud29);
    pcl::toROSMsg (*layer30, cloud30);
    cloud30.header = input->header;
    thirty_layer_.publish(cloud30);
    pcl::toROSMsg (*layer31, cloud31);
    cloud31.header = input->header;
    thirtyone_layer_.publish(cloud31);
    pcl::toROSMsg (*layer32, cloud32);
    cloud32.header = input->header;
    thirtytwo_layer_.publish(cloud32);

//    std::cout<<"first layer: "<<layer1->size()<<std::endl;
//    std::cout<<"second layer: "<<layer2->size()<<std::endl;
    std::cout<<"NaN points: "<<nan_pt<<std::endl;


    clear_grid();
}

void position_clustering::clear_grid()
{

}
