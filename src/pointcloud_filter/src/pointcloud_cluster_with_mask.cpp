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

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <math.h>
#include <pcl/io/pcd_io.h>
// PCL specific includes
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/tfMessage.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Vector3.h>

#include <algorithm>
#include <cmath>
#include <visualization_msgs/Marker.h>
//#include <array>
#include <sstream>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/PointIndices.h>

#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include "dbscan.h"

class cluster_with_mask
{
public:
    cluster_with_mask();

private:
    //callback
    void pointcloud_cb_ (const sensor_msgs::PointCloud2ConstPtr& input);
//    void point_cb_ (const geometry_msgs::PointStampedConstPtr& pt_in);

    ros::NodeHandle nh_;

    ros::Subscriber pointcloud_input_;
    ros::Subscriber point_input_;

    ros::Publisher pub_minimask_;
    ros::Publisher marker_pub_;
    ros::Publisher arrow_pub_;
    ros::Publisher pub_filter_;
    ros::Publisher box_pub_;

    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    tf::TransformListener *tf_listener_;

    std::vector<double>* pose_and_vel_;
    std::vector<std::vector<double>* >* state_vector_;
    std::vector<std::vector<double>* >* prev_state_vector_;

    std::string base_frame_, map_frame_, map_location_;

    DBSCAN* dbscan;

    visualization_msgs::Marker line_strip, arrow;

    jsk_recognition_msgs::BoundingBoxArray pedestrian_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_largemap;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_minimap;

    // function to support
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_with_mask(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void put_marker(pcl::PointXYZ min, pcl::PointXYZ max, int index);
    void compare_pedestrian(pcl::PointXYZ centroid_new, int number_cluster);
    void put_arrow(pcl::PointXYZ start, pcl::PointXYZ end, int index);

};

int main (int argc, char** argv)
{
    ros::init (argc, argv, "cluster_with_mask");

    cluster_with_mask mask;

    ros::spin();

    return(0);
}

cluster_with_mask::cluster_with_mask()
{    
    pointcloud_input_ = nh_.subscribe<sensor_msgs::PointCloud2> ("cloud_input", 1, &cluster_with_mask::pointcloud_cb_, this);
//    point_input_ = nh_.subscribe<geometry_msgs::PointStamped> ("clicked_point", 1, &cluster_with_mask::point_cb_, this);

    pub_minimask_ = nh_.advertise<sensor_msgs::PointCloud2> ("/minimask",1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>( "/box", 10);
    arrow_pub_ = nh_.advertise<visualization_msgs::Marker>("/arrow", 1);
    pub_filter_ = nh_.advertise<sensor_msgs::PointCloud2> ("/cloud_filter", 1);
    box_pub_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray> ("/pedestrian_box", 10);

    ros::NodeHandle priv_nh_("~");
    priv_nh_.getParam("base_frame", base_frame_);
    priv_nh_.getParam("map_frame", map_frame_);
    priv_nh_.getParam("map_location", map_location_);
//    priv_nh_.getParam("resolution", resolution_);
//    priv_nh_.getParam("density", density_);
//    priv_nh_.getParam("sequence_text_file", text_file_);

    tf_listener_ = new tf::TransformListener();

    state_vector_ = new std::vector<std::vector<double>* >;
    prev_state_vector_ = new std::vector<std::vector<double>* >;
    pose_and_vel_ = new std::vector<double>;
    pose_and_vel_->resize(4);

    cloud_largemap.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_minimap.reset(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile(map_location_, *cloud_largemap);

    std::cout<<"Filtered map contains "<<cloud_largemap->size ()<< " data points from map" << std::endl;

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D (*cloud_largemap, min_pt, max_pt);
    min_pt.x = min_pt.x - 1.0;
    min_pt.y = min_pt.y - 1.0;
    max_pt.x = max_pt.x + 1.0;
    max_pt.y = max_pt.y + 1.0;

    tf::Transform offset_transform;
    offset_transform.setOrigin(tf::Vector3(-min_pt.x-1.0, -min_pt.y-1.0, 0.0));
    offset_transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

    pcl_ros::transformPointCloud(*cloud_largemap, *cloud_largemap, offset_transform);

}

void cluster_with_mask::pointcloud_cb_ (const sensor_msgs::PointCloud2ConstPtr& input)
{

        pcl::console::TicToc time;
        time.tic ();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_minimap (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_stored (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_from_rs (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr rs_to_base (new pcl::PointCloud<pcl::PointXYZ>);
        // initialize marker and clear previous marker

        line_strip.header.frame_id = map_frame_;
        line_strip.header.stamp = input->header.stamp;
        line_strip.action = 3;
        marker_pub_.publish(line_strip);
        arrow.header.frame_id = map_frame_;
        arrow.header.stamp = input->header.stamp;
        arrow.action = 3;
        arrow_pub_.publish(arrow);
        pedestrian_.header = input->header;
        pedestrian_.header.frame_id = "rslidar";


        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>* cluster_;
        cluster_ = new std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>;

        tf::StampedTransform latest_odom_transform;
        try{
            tf_listener_->lookupTransform(map_frame_, base_frame_, ros::Time(0), latest_odom_transform);
        }catch (tf::TransformException &ex) {
            ROS_ERROR_STREAM("localizer_scan: Looking Transform Failed");
            return;
        }

        tf::StampedTransform rs_base_transform;
        try{
            tf_listener_->lookupTransform("rslidar", "iMiev/base_link", ros::Time(0), rs_base_transform);
        }catch (tf::TransformException &ex) {
            ROS_ERROR_STREAM("localizer_scan: Looking Transform Failed");
            return;
        }

        nav_msgs::Odometry odometry;
        tf::Vector3 center_point = latest_odom_transform.getOrigin();
        tf::quaternionTFToMsg(latest_odom_transform.getRotation(), odometry.pose.pose.orientation);
        odometry.pose.pose.position.x = center_point.x();
        odometry.pose.pose.position.y = center_point.y();
        odometry.pose.pose.position.z = center_point.z();

        std::cout<<"x: "<<center_point.x()<<" y: "<<center_point.y()<<" z: "<<center_point.z()<<std::endl;

        pcl::fromROSMsg (*input, *cloud_in);

        //pcl_ros::transformPointCloud (*cloud_in, *cloud_in, latest_odom_transform);

        // Create the filtering mask
        pass.setInputCloud (cloud_largemap);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (-35.0+center_point.x(), 35.0+center_point.x());
        pass.filter (*cloud_minimap);

        pass.setInputCloud (cloud_minimap);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (-35.0+center_point.y(), 35.0+center_point.y());
        pass.filter (*cloud_minimap);

        sensor_msgs::PointCloud2 output2;
        pcl::toROSMsg (*cloud_minimap, output2);
        output2.header.frame_id = map_frame_;
        pub_minimask_.publish(output2);

        std::cout<<"cloud in size: "<<cloud_in->size()<<", mask size: "<<cloud_minimap->size()<<std::endl;

        kdtree.setInputCloud (cloud_minimap);

        filtered_cloud = filter_with_mask(cloud_in);

        cluster_ = dbscan->dbscan_cluster(filtered_cloud);

        state_vector_->resize(cluster_->size());

        for (int i=0; i<cluster_->size(); i++)
        {
            // resize the pose and vel size to 4
            state_vector_->at(i) = new std::vector<double>;
            state_vector_->at(i)->resize(4);

            pcl::PointXYZ max_point, min_point, cluster_center;
            //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_of_cluster = cluster_->at(i);
            //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_of_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::getMinMax3D (*cluster_->at(i), min_point, max_point);

            put_marker(min_point, max_point, i);

            cluster_center.x=0.0; cluster_center.y=0.0; cluster_center.z=0.0;
            double mean_x=0.0; double mean_y=0.0; double mean_z=0.0;
            for (int nn=0; nn<cluster_->at(i)->size(); nn++)
            {
                mean_x=mean_x+cluster_->at(i)->points[nn].x;
                mean_y=mean_y+cluster_->at(i)->points[nn].y;
                mean_z=mean_z+cluster_->at(i)->points[nn].z;
            }

            //std::cout<<"total x: "<<mean_x<<" y: "<<mean_y<<" z: "<<mean_z<<std::endl;
            cluster_center.x = mean_x/cluster_->at(i)->size();
            cluster_center.y = mean_y/cluster_->at(i)->size();
            cluster_center.z = mean_z/cluster_->at(i)->size();
            std::cout<<i<<" center x: "<<cluster_center.x<<" center y: "<<cluster_center.y<<" center z: "<<cluster_center.z<<std::endl;
            //std::cout<<"current cluster "<<i<<" size: "<<cluster_->at(i)->size()<<std::endl;
            //std::cout<<"previous cluster size: "<<prev_state_vector_->size()<<std::endl;
            //std::cout<<"current cluster size: "<<state_vector_->size()<<std::endl;

            centroid_stored->push_back(cluster_center);



            if (prev_state_vector_->size()>0)
            {
                compare_pedestrian(cluster_center, i);
            }
            else
            {
                state_vector_->at(i)->at(0)=cluster_center.x;
                state_vector_->at(i)->at(1)=cluster_center.y;
                state_vector_->at(i)->at(2)=0.0;
                state_vector_->at(i)->at(3)=0.0;

            }

        }

        tf::Transform origin_tf = latest_odom_transform.inverse();

        pcl_ros::transformPointCloud (*centroid_stored, *centroid_from_rs, origin_tf);

        pcl_ros::transformPointCloud (*centroid_from_rs, *rs_to_base, rs_base_transform);

        for (int kk=0; kk<rs_to_base->size(); kk++)
        {
            jsk_recognition_msgs::BoundingBox bb;
            bb.header = pedestrian_.header;
            bb.pose.position.x = rs_to_base->points[kk].x;
            bb.pose.position.y = rs_to_base->points[kk].y;
            bb.pose.position.z = rs_to_base->points[kk].z;

            std::cout<<"cluster from origin "<<bb.pose.position.x<<" "<<bb.pose.position.y<<" "<<bb.pose.position.z<<std::endl;

            bb.dimensions.x = 1.0;
            bb.dimensions.y = 1.0;
            bb.dimensions.z = 2.0;

            bb.label = kk;

            pedestrian_.boxes.push_back(bb);
        }

        prev_state_vector_->clear();
        for (int jj=0; jj<state_vector_->size(); jj++)
        {
            prev_state_vector_->assign(state_vector_->begin(), state_vector_->end());
            state_vector_->clear();
        }

        box_pub_.publish(pedestrian_);
        pedestrian_.boxes.clear();

        std::cout<<"Found " <<cluster_->size()<<" clusters in "<<time.toc()<<" s."<<std::endl;
}

//void cluster_with_mask::point_cb_(const geometry_msgs::PointStamped::ConstPtr& initial)
//{
////    std::vector<int> NearSearch(1);
////    std::vector<float> NearSquaredDistance(1);
//    pcl::PointXYZ searching;
//    searching.x = initial->point.x;
//    searching.y = initial->point.y;
//    searching.z = initial->point.z;
////    std::cout<<" with x: "<<searching.x<<" y: "<<searching.y<<" z: "<<searching.z<<std::endl;
////    kdtree.nearestKSearch (searching, 1, NearSearch, NearSquaredDistance);
////    std::cout<<"Squared distance: "<<NearSquaredDistance[0]<<std::endl;

//    dbscan->db_point_select(filtered_cloud, searching);
//}

pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_with_mask::filter_with_mask(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ searchPoint;
    float radius = 0.5;
    std::vector<int> NearSearch(1);
    std::vector<float> NearSquaredDistance(1);
    for (int i=0; i<cloud->size(); i++)
    {
        searchPoint = cloud->points[i];
        kdtree.nearestKSearch (searchPoint, 1, NearSearch, NearSquaredDistance);
        if (NearSquaredDistance[0]>radius)
        {
            cloud_filter->push_back(cloud->points[i]);
        }

    }
    //std::cout<<"cloud filter size: "<<cloud_filter->size()<<std::endl;
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg (*cloud_filter, output);
    output.header.frame_id = map_frame_;
    pub_filter_.publish(output);

    return (cloud_filter);

}

void cluster_with_mask::put_marker(pcl::PointXYZ min, pcl::PointXYZ max, int index)
{
    visualization_msgs::Marker line_strip;
    line_strip.header.stamp = ros::Time::now();
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.id = index;
    line_strip.header.frame_id = map_frame_;
    line_strip.pose.orientation.w = 1.0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.1;

    line_strip.color.a = 1.0;
    line_strip.color.r = 0.0;
    line_strip.color.g = 1.0;
    line_strip.color.b = 0.0;
    geometry_msgs::Point border_1, border_2, border_3, border_4;
    border_1.x = max.x;
    border_1.y = max.y;
    border_1.z = min.z;
    line_strip.points.push_back(border_1);
    border_2.x = min.x;
    border_2.y = max.y;
    border_2.z = min.z;
    line_strip.points.push_back(border_2);
    border_3.x = min.x;
    border_3.y = min.y;
    border_3.z = min.z;
    line_strip.points.push_back(border_3);
    border_4.x = max.x;
    border_4.y = min.y;
    border_4.z = min.z;
    line_strip.points.push_back(border_4);
    line_strip.points.push_back(border_1);

    marker_pub_.publish(line_strip);


}

void cluster_with_mask::compare_pedestrian(pcl::PointXYZ centroid_new, int number_cluster)
{
    double min_dist = 1000.0;
    int t = 0;
    for (int zz=0; zz<prev_state_vector_->size(); zz++)
    {
        //std::cout<<"previous centroid "<<zz<<" is: "<<prev_state_vector_->at(zz)->at(0)<<" "<<prev_state_vector_->at(zz)->at(1)<<std::endl;
        double minus_x = centroid_new.x-prev_state_vector_->at(zz)->at(0);
        double minus_y = centroid_new.y-prev_state_vector_->at(zz)->at(1);
        double new_dist = sqrt(minus_x*minus_x + minus_y*minus_y);
        //std::cout<<"dist: "<<new_dist<<std::endl;
        if (new_dist<min_dist)
        {
            min_dist = new_dist;
            t=zz;
            //std::cout<<"the nearest previous centroid is: "<<prev_state_vector_->at(zz)->at(0)<<" "<<prev_state_vector_->at(zz)->at(1)<<std::endl;
        }
    }

    if (!(min_dist==1000.0 || min_dist>2.0))
    {
        state_vector_->at(number_cluster)->at(0) = centroid_new.x;
        state_vector_->at(number_cluster)->at(1) = centroid_new.y;
        state_vector_->at(number_cluster)->at(2) = centroid_new.x-prev_state_vector_->at(t)->at(0);
        state_vector_->at(number_cluster)->at(3) = centroid_new.y-prev_state_vector_->at(t)->at(1);

        pcl::PointXYZ arrow_start, arrow_end;
        arrow_end.x = centroid_new.x+10*state_vector_->at(number_cluster)->at(2);
        arrow_end.y = centroid_new.y+10*state_vector_->at(number_cluster)->at(3);
        arrow_end.z = centroid_new.z;
        arrow_start = centroid_new;

        put_arrow(arrow_start, arrow_end, number_cluster);
    }
    else
    {
        state_vector_->at(number_cluster)->at(0) = centroid_new.x;
        state_vector_->at(number_cluster)->at(1) = centroid_new.y;
        state_vector_->at(number_cluster)->at(2) = 0.0;
        state_vector_->at(number_cluster)->at(3) = 0.0;
    }
}

void cluster_with_mask::put_arrow(pcl::PointXYZ start, pcl::PointXYZ end, int index)
{
    visualization_msgs::Marker arrow;
    arrow.header.stamp = ros::Time::now();
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.id = index;
    arrow.header.frame_id = map_frame_;
    arrow.pose.orientation.w = 1.0;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.scale.x = 0.1;

    arrow.color.a = 1.0;
    arrow.color.r = 0.0;
    arrow.color.g = 1.0;
    arrow.color.b = 0.0;
    geometry_msgs::Point border_1, border_2;
    border_1.x = start.x;
    border_1.y = start.y;
    border_1.z = start.z;
    arrow.points.push_back(border_1);
    border_2.x = end.x;
    border_2.y = end.y;
    border_2.z = end.z;
    arrow.points.push_back(border_2);

    std::cout<<"Velocity for pedestrian "<<index<<" is at "<<end.x<<" and "<<end.y<<std::endl;


    arrow_pub_.publish(arrow);
}
