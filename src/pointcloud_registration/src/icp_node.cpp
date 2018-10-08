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

/* Author: Brian Gerkey */

#include <algorithm>
#include <vector>
#include <map>
#include <cmath>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

// Signal handling
#include <signal.h>

#include "ros/assert.h"

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
#include "message_filters/subscriber.h"

// Dynamic_reconfigure
#include "dynamic_reconfigure/server.h"
#include <boost/foreach.hpp>


class ICPNode
{
  public:
    ICPNode();

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

    tf::Transform difference;

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

    int chance_;
    int icp_max_iter_;
    double icp_max_distance_;
    double icp_filter_size_;

    bool location_confirm_, odom_start_;

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

};


int
main(int argc, char** argv)
{
  ros::init(argc, argv, "icp");

    ICPNode icpnode;

    ros::spin();
  return(0);
}

ICPNode::ICPNode()
{
  // Grab params off the param server


  tfb_ = new tf::TransformBroadcaster();
  tf_ = new TransformListenerWrapper();

  //icp_received_ = nh_.subscribe ("icp_odom", 1, &ICPNode::icpReceived_, this);
  input_pointcloud_ = nh_.subscribe <sensor_msgs::PointCloud2> ("input", 1, &ICPNode::pointcloud_cb_, this);
  input_localize_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped> ("/initialpose", 1, &ICPNode::initial_pose_cb_, this);
  input_map_ = nh_.subscribe<sensor_msgs::PointCloud2> ("/full_map", 1, &ICPNode::map_cb_, this);

  pub_icp_odom_ = nh_.advertise <nav_msgs::Odometry> ("icp_odom", 1);
  pub_input_odom_ = nh_.advertise <nav_msgs::Odometry> ("input_odom", 1);
  pub_aligned_ = nh_.advertise <sensor_msgs::PointCloud2> ("/velo_cloud", 1);
  pub_minimap_ = nh_.advertise <sensor_msgs::PointCloud2> ("/minimap", 1);

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

    transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
    transform.setRotation(tf::Quaternion(0.0,0.0,0.0,1.0));

}
/*
void ICPNode::icpReceived_ (const nav_msgs::OdometryConstPtr& odom)
{
    std::cout<<"received icp at: "<<odom->header.stamp.toSec()<<std::endl;
    tf::Stamped<tf::Pose> odom_to_map;
    tf::Quaternion odom_orient;
    tf::quaternionMsgToTF(odom->pose.pose.orientation, odom_orient);
    try
    {
      tf::Transform tmp_tf(odom_orient,
                           tf::Vector3(odom->pose.pose.position.x,
                                       odom->pose.pose.position.y,
                                       0.0));
      tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(),
                                            odom->header.stamp,
                                            base_frame_id_);
      this->tf_->transformPose(odom_frame_id_,
                               tmp_tf_stamped,
                               odom_to_map);
    }
    catch(tf::TransformException)
    {
      ROS_DEBUG("Failed to subtract base to odom transform");
      return;
    }

    latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                               tf::Point(odom_to_map.getOrigin()));
    latest_tf_valid_ = true;

    if (tf_broadcast_ == true)
    {
      // We want to send a transform that is good up until a
      // tolerance time so that odom can be used
      ros::Time transform_expiration = (odom->header.stamp +
                                        transform_tolerance_);
      tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                          transform_expiration,
                                          global_frame_id_, odom_frame_id_);
      this->tfb_->sendTransform(tmp_tf_stamped);
    }
}*/

void ICPNode::initial_pose_cb_(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& posing)
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

    location_confirm_=true;
    //start=true;

}

void ICPNode::map_cb_(const sensor_msgs::PointCloud2ConstPtr& map)
{
    pcl::fromROSMsg (*map, *cloud_largemap);
    std::cout<<"map received with points: "<<cloud_largemap->size()<<std::endl;
}

void ICPNode::print4x4Matrix (const Eigen::Matrix4f & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

Eigen::Matrix4f ICPNode::setTransformMatrix(double a, bool b)
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

Eigen::Matrix4f ICPNode::localize()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_comeout(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_origin(new pcl::PointCloud<pcl::PointXYZ>);

    tf::Transform origin_tf = transform.inverse();

    pcl_ros::transformPointCloud (*cloud_minimap, *map_origin, origin_tf);

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

    second_score = sqrt(compare_matrix(0,3)*compare_matrix(0,3)+compare_matrix(1,3)*compare_matrix(1,3)+compare_matrix(2,3)*compare_matrix(2,3));

    std::cout << "ICP X: " << compare_matrix(0,3)<< " Y: " << compare_matrix(1,3)<< " Z: " << compare_matrix(2,3)<< std::endl;
    std::cout << "odomX: " << difference_pose.x()<< " Y: " << difference_pose.y()<< " Z: " << difference_pose.z()<< std::endl;

    if (difference_pose.x()+difference_pose.y()==0.0)
    {
        second_score = 0.0;
    }
    std::cout << "second_score = "<< second_score<<std::endl;

    if (chance_==5)
    {
        compare_matrix=use_odom;
        std::cout<<"observe..."<<std::endl;
        chance_=0;
    }
    else if (chance_==4)
    {
        std::cout<<"I think we can give him a chance"<<std::endl;
        chance_++;
    }
    else if (score<0.5 && second_score<0.5)
    {
        std::cout<<"I think it is a good icp"<<std::endl;
    }
    else if (score<0.5)
    {
        chance_++;
        std::cout<<"Icp ok for "<<chance_<<" times"<<std::endl;
        //compare_matrix(2,3)=0.0;
        compare_matrix=use_odom;
        std::cout<<"using odom info..."<<std::endl;
    }

//    else if (score>0.7 || second_score>2.0)
//    {
//        compare_matrix=use_odom;
//        std::cout<<"using odom info..."<<std::endl;
//    }

    else
    {
        chance_=0;
        compare_matrix=use_odom;
        std::cout<<"using odom info..."<<std::endl;
    }

    std::cout << "Applied iterations with " <<score<< std::endl;

    return (compare_matrix);
}

void ICPNode::pointcloud_cb_ (const sensor_msgs::PointCloud2ConstPtr& input)
{
    tf::StampedTransform latest_odom_transform;
    try{
        tf_->lookupTransform(odom_frame_id_, base_frame_id_,
                                      ros::Time(0), latest_odom_transform);
    }catch (tf::TransformException &ex) {
        ROS_ERROR_STREAM("localizer_scan: Looking Transform Failed");
        return;
    }

    if (odom_start_==false)
    {
        offset_transform = latest_odom_transform;
        odom_start_=true;
    }

    tf::Transform now_odom_transform;
    now_odom_transform = offset_transform.inverseTimes(latest_odom_transform);


//    if (odom_received)
//    {
        // The point clouds we will be using
        pcl::console::TicToc time;
        time.tic ();
        pcl::fromROSMsg (*input, *cloud_in);
        sensor_msgs::PointCloud2 output_minimap, output_running_pointcloud;


        //now_odom_transform = odom_transform;

        difference = prev_odom_transform.inverseTimes(now_odom_transform);

        difference_pose = difference.getOrigin();

        transform = transform * difference;

        /*** imu data ***/
//        double now_r, now_p, now_y;
//        tf::Quaternion imu_rpy;
//        tf::Matrix3x3(transform.getRotation()).getRPY(now_r, now_p, now_y);

//        imu_rpy.setRPY(i_r, i_p,now_y);
//        transform.setRotation(imu_rpy.normalize());

//        diff_p = i_p - prev_p;
//        prev_p = i_p;


        /******* vehicle odom ********/
        vehicle_pose = vehicle_pose * difference;
//        tf::quaternionTFToMsg(imu_rpy, input_odom.pose.pose.orientation);
        nav_msgs::Odometry input_odom;
        tf::quaternionTFToMsg(vehicle_pose.getRotation(), input_odom.pose.pose.orientation);
        tf::Vector3 wheel_position = vehicle_pose.getOrigin();
        input_odom.pose.pose.position.x = wheel_position.x();
        input_odom.pose.pose.position.y = wheel_position.y();
//        wheel_in_z = wheel_in_z + sqrt((difference_pose.x()*difference_pose.x())+(difference_pose.y()*difference_pose.y()))*tan(diff_p);
//        input_odom.pose.pose.position.z = wheel_in_z;
        input_odom.pose.pose.position.z = wheel_position.z();
        input_odom.header.frame_id = global_frame_id_;
        pub_input_odom_.publish(input_odom);
        /***--------------------------------******/

        // Defining a rotation matrix and translation vector
        Eigen::Matrix4f transformation_matrix;
        float translation_x, translation_y, translation_z;
        double roll, pitch, yaw;
        tf::Vector3 now_transform=transform.getOrigin();
        //std::cout<<"x: "<<now_transform.x()<<" y: "<<now_transform.y()<<" z: "<<now_transform.z()<<std::endl;

        pass.setInputCloud (cloud_in);
        pass.setFilterFieldName ("x");
        //pass.setFilterLimits (-30.0+now_transform.x(), 30.0+now_transform.x());
        pass.setFilterLimits (-30.0, 30.0);
        //pass.setFilterLimitsNegative (true);
        pass.filter (*cloud_in);

        pass.setInputCloud (cloud_in);
        pass.setFilterFieldName ("y");
        //pass.setFilterLimits (-30.0+now_transform.y(), 30.0+now_transform.y());
        pass.setFilterLimits (-30.0, 30.0);
        pass.filter (*cloud_in);

        // Create the filtering map

        pass.setInputCloud (cloud_largemap);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (-35.0+now_transform.x(), 35.0+now_transform.x());
        //pass.setFilterLimits (-30.0, 30.0);
        //pass.setFilterLimitsNegative (true);
        pass.filter (*cloud_minimap);

        pass.setInputCloud (cloud_minimap);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (-35.0+now_transform.y(), 35.0+now_transform.y());
        //pass.setFilterLimits (-30.0, 30.0);
        pass.filter (*cloud_minimap);

    //    voxel_filter.setLeafSize (0.2, 0.2, 0.2);
    //	voxel_filter.setInputCloud (cloud_minimap);
    //	voxel_filter.filter (*cloud_minimap);

//        double low = cloud_minimap->points[0].z;
//        for (int h=1; h<cloud_minimap->size(); h++)
//        {
//            if (cloud_minimap->points[h].z<low)
//            {
//                low = cloud_minimap->points[h].z;
//            }
//        }

        /*** showing map portion of matching with cloud
         * pcl::toROSMsg (*cloud_minimap, output_minimap);
        output_minimap.header.frame_id = global_frame_id_;
        pub_minimap.publish(output_minimap); ***/

        /***------------reposition z using map info-------------***/

        //transform.setOrigin(tf::Vector3(now_transform.x(), now_transform.y(), low+3));

//        if (location_confirm || start)
//        pcl_ros::transformPointCloud (*cloud_in, *cloud_transform, transform);
//        else
//            *cloud_transform=*cloud_in;

//        pcl::toROSMsg (*cloud_transform, output_running_pointcloud);
//        output_running_pointcloud.header.frame_id = "map";
        //pub_pointcloud.publish(output_running_pointcloud);



        if (location_confirm_)
        {
            tf::StampedTransform lidar_baselink;

            try{
                tf_->lookupTransform(base_frame_id_, input->header.frame_id, ros::Time(0), lidar_baselink);
            }catch (tf::TransformException &ex) {
                ROS_ERROR("Lookup transform for %s and %s failed", base_frame_id_.c_str(), input->header.frame_id.c_str());
                return;
            }
            pcl_ros::transformPointCloud (*cloud_in, *cloud_in, lidar_baselink);
        voxel_filter.setLeafSize (icp_filter_size_, icp_filter_size_, icp_filter_size_);
        voxel_filter.setInputCloud (cloud_in);
        voxel_filter.filter (*cloud_transform);
        std::cout << "Filtered cloud contains " << cloud_transform->size ()
              << " data points from original point cloud: "<<cloud_in->size() << std::endl;

        transformation_matrix = localize();

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





        double r, p, y;
        tf::Matrix3x3(transform.getRotation()).getRPY(r, p, y);
        //std::cout<<"first transform end..."<<std::endl;

        nav_msgs::Odometry icp_odom;

        tf::quaternionTFToMsg(transform.getRotation(), icp_odom.pose.pose.orientation);
        tf::Vector3 new_nav_position = transform.getOrigin();

        icp_odom.pose.pose.position.x = new_nav_position.x();
        icp_odom.pose.pose.position.y = new_nav_position.y();
        icp_odom.pose.pose.position.z = new_nav_position.z();
//        input_odom.pose.pose.position.z = wheel_in_z;

        std::cout<<"x: "<<new_nav_position.x()<<" y: "<<new_nav_position.y()<<" z: "<<new_nav_position.z()<<std::endl;
        std::cout<<"R: "<<r<<" P: "<<p<<" Y: "<<y<<std::endl;
 /*       input_odom.header.frame_id = global_frame_id_;
        input_odom.header.stamp = input->header.stamp;

        pub_icp_odom.publish(input_odom);

        geometry_msgs::PoseWithCovarianceStamped icp_pose;
        icp_pose.header = input_odom.header;
        icp_pose.header.stamp = input->header.stamp;
        icp_pose.pose = input_odom.pose;

        pub_localize.publish(icp_pose);*/

        tf::Stamped<tf::Pose> odom_to_map;
        try
        {
          tf::Stamped<tf::Pose> tmp_tf_stamped (transform.inverse(),
                                                input->header.stamp,
                                                base_frame_id_);
          this->tf_->transformPose(odom_frame_id_,
                                   tmp_tf_stamped,
                                   odom_to_map);
        }
        catch(tf::TransformException)
        {
          ROS_DEBUG("Failed to subtract base to odom transform");
          return;
        }

        latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                                   tf::Point(odom_to_map.getOrigin()));
        latest_tf_valid_ = true;


          // We want to send a transform that is good up until a
          // tolerance time so that odom can be used
          ros::Time transform_expiration = (input->header.stamp +
                                            transform_tolerance_);
          tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                              transform_expiration,
                                              global_frame_id_, odom_frame_id_);
          this->tfb_->sendTransform(tmp_tf_stamped);


        }

        sensor_msgs::PointCloud2 output2;
        pcl_ros::transformPointCloud (*cloud_in, *final_cloud, transform);

        pcl::toROSMsg (*final_cloud, output2);
        output2.header.frame_id = global_frame_id_;
        pub_aligned_.publish(output2);

        //*cloud_previous=*cloud_in;
        prev_odom_transform = now_odom_transform;
        std::cout<<"pointcloud callback end... in " << time.toc () << " ms"<<std::endl;

}


