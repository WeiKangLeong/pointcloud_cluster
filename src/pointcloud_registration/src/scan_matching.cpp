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

// Messages that I need
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"
#include "std_srvs/Empty.h"

// For pcl format
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/common/common.h>

// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include <tf_conversions/tf_eigen.h>

class ICP_scan_matchNode
{
  public:
    ICP_scan_matchNode();

  private:

    void print4x4Matrix (const Eigen::Matrix4f & matrix);
    Eigen::Matrix4f setTransformMatrix(double a, bool b);
    Eigen::Matrix4f localize();

    tf::TransformBroadcaster* tfb_;

    // Use a child class to get access to tf2::Buffer class inside of tf_
    struct TransformListenerWrapper : public tf::TransformListener
    {
      inline tf2_ros::Buffer &getBuffer() {return tf2_buffer_;}
    };

    TransformListenerWrapper* tf_;

    tf::Transform latest_tf_;
    bool tf_broadcast_;
    bool latest_tf_valid_;

    // Callbacks
    void pointcloud_cb_ (const sensor_msgs::PointCloud2ConstPtr& input);
    void initial_pose_cb_(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& posing);
    void map_cb_(const sensor_msgs::PointCloud2ConstPtr& map);

    //parameter for what odom to use
    std::string odom_frame_id_;

    //paramater to store latest odom pose
    tf::Stamped<tf::Pose> latest_odom_pose_;

    tf::Transform difference, shift_cloud_, shift_back_cloud_;

    tf::Vector3 difference_pose;

    double guess_movement;

    //parameter for what base to use
    std::string base_frame_id_;
    std::string global_frame_id_;

    std::string map_location;

    //time for tolerance on the published transform,
    //basically defines how long a map->odom transform is good for
    ros::Duration transform_tolerance_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber icp_received_;
    ros::Subscriber input_pointcloud_;
    ros::Subscriber input_localize_;
    ros::Subscriber input_map_;
    ros::Publisher pub_icp_odom_;
    ros::Publisher pub_input_odom_;
    ros::Publisher pub_minimap_;
    ros::Publisher pub_aligned_;
    ros::Publisher pub_initialized_;

    int chance_;
    int icp_max_iter_;
    double icp_max_distance_;
    double icp_filter_size_;

    double roll, pitch, yaw;

    bool location_confirm_, odom_start_, start;

    // Map point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_largemap;

    // PointCloud we will be using
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_minimap;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transform;
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud;

    // various functions
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;

    tf::Transform transform, vehicle_pose, offset_transform, prev_odom_transform;

    geometry_msgs::PoseWithCovarianceStamped global_initialize_;

};


int
main(int argc, char** argv)
{
  ros::init(argc, argv, "icp");

    ICP_scan_matchNode scan_matching_icp;

    ros::spin();
  return(0);
}

ICP_scan_matchNode::ICP_scan_matchNode()
{
  // Grab params off the param server


  tfb_ = new tf::TransformBroadcaster();
  tf_ = new TransformListenerWrapper();

  //icp_received_ = nh_.subscribe ("icp_odom", 1, &ICPNode::icpReceived_, this);
  input_pointcloud_ = nh_.subscribe <sensor_msgs::PointCloud2> ("input", 1, &ICP_scan_matchNode::pointcloud_cb_, this);
  input_localize_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped> ("/initialpose", 1, &ICP_scan_matchNode::initial_pose_cb_, this);
  input_map_ = nh_.subscribe<sensor_msgs::PointCloud2> ("/full_map", 1, &ICP_scan_matchNode::map_cb_, this);

  pub_icp_odom_ = nh_.advertise <nav_msgs::Odometry> ("icp_odom", 1);
  pub_input_odom_ = nh_.advertise <nav_msgs::Odometry> ("input_odom", 1);
  pub_aligned_ = nh_.advertise <sensor_msgs::PointCloud2> ("/velo_cloud", 1);
  pub_minimap_ = nh_.advertise <sensor_msgs::PointCloud2> ("/minimap", 1);
  pub_initialized_ = nh_.advertise <geometry_msgs::PoseWithCovarianceStamped> ("/iMiev/initialpose", 1);

  ros::NodeHandle priv_nh("~");

  ROS_INFO("start");

  priv_nh.getParam("map_location", map_location);
  priv_nh.getParam("base_frame_id", base_frame_id_);
  priv_nh.getParam("global_frame_id", global_frame_id_);
  priv_nh.getParam("odom_frame_id", odom_frame_id_);
  priv_nh.getParam("icp_iteration", icp_max_iter_);
  priv_nh.getParam("icp_distance", icp_max_distance_);
  priv_nh.getParam("icp_filter_size", icp_filter_size_);

    transform_tolerance_ = ros::Duration(0.3);

    cloud_largemap.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_in.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_minimap.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_transform.reset(new pcl::PointCloud<pcl::PointXYZ>);
    final_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    chance_ = 0;
    odom_start_ = false;
    location_confirm_ = false;
    start = false;

    shift_cloud_.setOrigin(tf::Vector3(100.0, 100.0, 0.0));
    shift_cloud_.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    shift_back_cloud_.setOrigin(tf::Vector3(-100.0, -100.0, 0.0));
    shift_back_cloud_.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

    transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
    transform.setRotation(tf::Quaternion(0.0,0.0,0.0,1.0));


}

void ICP_scan_matchNode::initial_pose_cb_(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& posing)
{
    geometry_msgs::PoseWithCovarianceStamped initial;
    initial = *posing;
    tf::Quaternion initial_rotation;

    tf::quaternionMsgToTF(initial.pose.pose.orientation, initial_rotation);

    double ra,pa,ya;
    vehicle_pose.setOrigin(tf::Vector3(initial.pose.pose.position.x,initial.pose.pose.position.y,initial.pose.pose.position.z));
    vehicle_pose.setRotation(initial_rotation.normalize());
    transform = vehicle_pose;
    tf::Matrix3x3(transform.getRotation()).getRPY(ra, pa, ya);

    std::cout<<"I have added a new pose at X: "<<initial.pose.pose.position.x<<" Y "<<initial.pose.pose.position.y<<
               " z "<<initial.pose.pose.position.z<<std::endl;
    std::cout<<"with a heading of R: "<<ra<<" P "<<pa<<" Y "<<ya<<std::endl;

    if (start){

        pcl::console::TicToc time;
        time.tic ();
        Eigen::Matrix4f transformation_matrix;
        transformation_matrix = localize();

        double translation_x, translation_y, translation_z;
        translation_x = transformation_matrix(0,3);
        translation_y = transformation_matrix(1,3);
        translation_z = transformation_matrix(2,3);

        tf::Matrix3x3 tf3d;
        tf3d.setValue(static_cast<double>(transformation_matrix(0,0)), static_cast<double>(transformation_matrix(0,1)), static_cast<double>(transformation_matrix(0,2)),
            static_cast<double>(transformation_matrix(1,0)), static_cast<double>(transformation_matrix(1,1)), static_cast<double>(transformation_matrix(1,2)),
            static_cast<double>(transformation_matrix(2,0)), static_cast<double>(transformation_matrix(2,1)), static_cast<double>(transformation_matrix(2,2)));

        tf::Quaternion tfqt,rotate;
        //tf3d.getRotation(tfqt);
        tf3d.getRPY (roll,pitch,yaw);
        //tf3d.setRPY (0.0, pitch, yaw);
        tf3d.getRotation(tfqt);

        tf::Transform transform2;
        transform2.setOrigin(tf::Vector3(translation_x, translation_y, translation_z));
        transform2.setRotation(tfqt);

        transform = transform * transform2;

        sensor_msgs::PointCloud2 output2;
        pcl_ros::transformPointCloud (*cloud_in, *final_cloud, transform);

        pcl::toROSMsg (*final_cloud, output2);
        output2.header.frame_id = global_frame_id_;
        pub_aligned_.publish(output2);

        tf::Vector3 final_pose;
        final_pose = transform.getOrigin();
        global_initialize_.pose.pose.position.x = final_pose.x();
        global_initialize_.pose.pose.position.y = final_pose.y();
        global_initialize_.pose.pose.position.z = final_pose.z();
        tf::Quaternion final_orientation;
        final_orientation = transform.getRotation();
        tf::quaternionTFToMsg(final_orientation, global_initialize_.pose.pose.orientation);

        global_initialize_.header.frame_id = global_frame_id_;

        pub_initialized_.publish(global_initialize_);

        std::cout<<"pointcloud callback end... in " << time.toc () << " ms"<<std::endl;

        ros::shutdown();
    }

}

void ICP_scan_matchNode::map_cb_(const sensor_msgs::PointCloud2ConstPtr& map)
{
    pcl::fromROSMsg (*map, *cloud_largemap);
    std::cout<<"map received with points: "<<cloud_largemap->size()<<std::endl;
    start=true;
}

void ICP_scan_matchNode::print4x4Matrix (const Eigen::Matrix4f & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

Eigen::Matrix4f ICP_scan_matchNode::setTransformMatrix(double a, bool b)
{
    double ro, pi, ya, add_angle;
    add_angle=a;
    Eigen::Matrix4f init_guess;

    if (b==true)
    {
        Eigen::AngleAxisf init_rotation_roll (0.0, Eigen::Vector3f::UnitX ());
        Eigen::AngleAxisf init_rotation_pitch (0.0, Eigen::Vector3f::UnitY ());
        Eigen::AngleAxisf init_rotation_yaw (add_angle, Eigen::Vector3f::UnitZ ());
        Eigen::Translation3f init_translation(0, 0, 0);
        init_guess = (init_translation * init_rotation_roll * init_rotation_pitch * init_rotation_yaw).matrix ();
    }
    else
    {

        tf::Matrix3x3(difference.getRotation()).getRPY(ro, pi, ya);

        Eigen::AngleAxisf init_rotation_roll (0.0, Eigen::Vector3f::UnitX ());
        Eigen::AngleAxisf init_rotation_pitch (pi, Eigen::Vector3f::UnitY ());
        Eigen::AngleAxisf init_rotation_yaw (ya, Eigen::Vector3f::UnitZ ());
        Eigen::Translation3f init_translation(difference_pose.x(), difference_pose.y(), difference_pose.z());
        init_guess = (init_translation * init_rotation_roll * init_rotation_pitch * init_rotation_yaw).matrix ();
        guess_movement = (pow(((difference_pose.x()*difference_pose.x()) + (difference_pose.y()*difference_pose.y())),0.5))/10;

    }
    return(init_guess);
}

Eigen::Matrix4f ICP_scan_matchNode::localize()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_comeout(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_origin(new pcl::PointCloud<pcl::PointXYZ>);

    tf::Transform origin_tf = transform.inverse();

    pcl_ros::transformPointCloud (*cloud_largemap, *map_origin, origin_tf);

    //std::cout<<cloud_transform->size()<<" "<<map_origin->size()<<std::endl;
    double score, second_score;

    Eigen::Matrix4f  use_odom, compare_matrix;
    use_odom = setTransformMatrix(0.0, true);

    icp.setMaximumIterations (icp_max_iter_);
    icp.setMaxCorrespondenceDistance(icp_max_distance_);
    //icp.setTransformationEpsilon (1e-7);

    icp.setInputSource (cloud_transform);
    icp.setInputTarget (map_origin);
    icp.align (*cloud_comeout);
    score=icp.getFitnessScore();
    compare_matrix = icp.getFinalTransformation ();

    std::cout << "ICP X: " << compare_matrix(0,3)<< " Y: " << compare_matrix(1,3)<< " Z: " << compare_matrix(2,3)<< std::endl;

    std::cout << "Applied iterations with " <<score<< std::endl;

    return (compare_matrix);
}

void ICP_scan_matchNode::pointcloud_cb_ (const sensor_msgs::PointCloud2ConstPtr& input)
{

        pcl::fromROSMsg (*input, *cloud_in);

        voxel_filter.setLeafSize (icp_filter_size_, icp_filter_size_, icp_filter_size_);
        voxel_filter.setInputCloud (cloud_in);
        voxel_filter.filter (*cloud_transform);
        std::cout << "Filtered cloud contains " << cloud_transform->size ()
              << " data points from original point cloud: "<<cloud_in->size() << std::endl;



}


