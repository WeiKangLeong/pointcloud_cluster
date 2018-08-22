#include <iostream>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <math.h>
#include <pcl/io/pcd_io.h>
// PCL specific includes
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Transform.h>
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
#include <fstream>
#include <pcl/io/ply_io.h>
//#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/common/common.h>

ros::Publisher pub_icp_odom, pub_input_odom, pub_particle_odom, pub_map, pub_aligned, pub_minimap, pub_pointcloud, vis_pub, pub_localize;

//typedef pcl::PointXYZI PointT;
//typedef pcl::PointCloud<PointXYZI> PointCloudT;

nav_msgs::Odometry guess_odom, initial_odom, offset_odom, input_odom;

double global_roll, global_pitch, global_yaw, r, p, y, first_score, second_score, guess_movement,
        i_r, i_p, i_y, wheel_in_z, diff_p, prev_p;

double result;

tf::TransformBroadcaster *tfb;
tf::TransformListener *tf_listener_;
tf::Transform transform, transform2, transform3, offset_transform, odom_transform, now_odom_transform, prev_odom_transform, diff_odom_transform,
                turn90, difference, initial_pose, vehicle_pose, imu_transform, previous_transform, latest_tf_;
tf::Quaternion quater;

tf::Transform transform_result;

std::ofstream myfile;

tf::Vector3 difference_pose, wheel_position, now_transform, prev_diff_pose;
Eigen::Matrix4f second_matrix, Point[10],theOne;

// The point clouds we will be using
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_previous(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp(new pcl::PointCloud<pcl::PointXYZ>);

//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_minimap(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr velodyne_cloud (new pcl::PointCloud<pcl::PointXYZ>);

// Map point cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_largemap(new pcl::PointCloud<pcl::PointXYZ>);

// various functions
pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
pcl::PassThrough<pcl::PointXYZ> pass;
pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;

bool start, location_confirm, find_orient, odom_start, odom_received;

bool indoor=false;

int danger,chance;

std::string base_frame_id_, global_frame_id_, odom_frame_id_, map_location, file_location;

void write_to_file(double p_x, double p_y, int degree, double score)
{
    myfile<<p_x<<" "<<p_y<<" "<<" "<<degree<<" "<<score<<"\n";
}

void print4x4Matrix (const Eigen::Matrix4f & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

Eigen::Matrix4f setTransformMatrix(double a, bool b)
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

tf::Transform EigenToTF(Eigen::Matrix4f eigen_matrix)
{
    tf::Transform eigen_TF;
    double t_x = eigen_matrix(0,3);
    double t_y = eigen_matrix(1,3);
    double t_z = eigen_matrix(2,3);

    tf::Matrix3x3 tf3d;
    tf3d.setValue(static_cast<double>(eigen_matrix(0,0)), static_cast<double>(eigen_matrix(0,1)), static_cast<double>(eigen_matrix(0,2)),
        static_cast<double>(eigen_matrix(1,0)), static_cast<double>(eigen_matrix(1,1)), static_cast<double>(eigen_matrix(1,2)),
        static_cast<double>(eigen_matrix(2,0)), static_cast<double>(eigen_matrix(2,1)), static_cast<double>(eigen_matrix(2,2)));

    tf::Quaternion tfqt,rotate;
    //tf3d.getRotation(tfqt);
    //tf3d.getRPY (roll,pitch,yaw);
    //tf3d.setRPY (0.0, pitch, yaw);
    tf3d.getRotation(tfqt);

    eigen_TF.setOrigin(tf::Vector3(t_x, t_y, t_z));
    eigen_TF.setRotation(tfqt);

    return eigen_TF;
}

tf::Transform setTransform(double p_x, double p_y, double yaw)
{
    tf::Transform lazy_tf;
    tf::Quaternion lazy_qt;
    lazy_tf.setOrigin(tf::Vector3(p_x, p_y, 0.0));
    lazy_qt.setRPY(0, 0, yaw);
    lazy_tf.setRotation(lazy_qt);
    return lazy_tf;
}

void searching(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target, double pose_x, double pose_y)
{
    //std::cout<<"searching loop"<<std::endl;
    tf::Transform map_pose, map_pose_inverse;
    map_pose = setTransform(pose_x, pose_y, 0);
    map_pose_inverse = map_pose.inverse();
    pcl::PointCloud<pcl::PointXYZ>::Ptr mini_map (new pcl::PointCloud<pcl::PointXYZ>);
    pcl_ros::transformPointCloud (*target, *mini_map, map_pose_inverse);
    for (int k=0; k<60; k++)
    {
        //std::cout<<"now is in degree: "<<k*5<<std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rotate(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Matrix4f icp_matrix;
        double icp_score;
        tf::Transform rotate;
        tf::Quaternion degree;
        degree.setRPY(0, 0, 6*k*3.1415926/180);
        rotate.setRotation(degree);
        pcl_ros::transformPointCloud (*source, *cloud_rotate, rotate);

        //std::cout<<"cloud source: "<<cloud_rotate->size()<<" cloud target "<<mini_map->size()<< std::endl;

        icp.setInputSource (cloud_rotate);
        icp.setInputTarget (mini_map);
        icp.align (*cloud_icp);
        icp_score=icp.getFitnessScore();
        icp_matrix = icp.getFinalTransformation ();
        if (icp_score<0.8)
        {
            for (int n=0; n<5; n++)
            {
                icp.setInputSource (cloud_icp);
                icp.setInputTarget (mini_map);
                icp.align (*cloud_icp);
                icp_score=icp.getFitnessScore();
                icp_matrix = icp.getFinalTransformation ();
                write_to_file(pose_x, pose_y, k, icp_score);
                std::cout<<"icp score: "<<icp_score<<std::endl;
            }
        }
        else
        {
            write_to_file(pose_x, pose_y, k, icp_score);
            std::cout<<"icp score: "<<icp_score<<std::endl;
        }

        pcl_ros::transformPointCloud (*cloud_icp, *cloud_icp, map_pose);
        sensor_msgs::PointCloud2 output3;
        pcl::toROSMsg (*cloud_icp, output3);
        output3.header.frame_id = global_frame_id_;
        pub_aligned.publish(output3);
        /*if (icp_score<0.1)
        {
            transform_result = EigenToTF(icp_matrix);
            transform_result = transform_result * map_pose;
            return;
        }
        else */if (icp_score<result)
        {
            result = icp_score;
            transform_result = EigenToTF(icp_matrix);
            transform_result = transform_result * map_pose;
        }
    }

    
}

void pose_info(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointXYZ min_point, max_point;
    pcl::getMinMax3D (*cloud_largemap, min_point, max_point);
    min_point.x = min_point.x - 1.0;
    min_point.y = min_point.y - 1.0;
    max_point.x = max_point.x + 1.0;
    max_point.y = max_point.y + 1.0;

    double start_x = min_point.x +35.0;
    double start_y = min_point.y +35.0;
    int max_i = ((max_point.x-min_point.x)-70)/5;
    int max_j = ((max_point.y-min_point.y)-70)/5;
    int count = 0;

    std::cout<<start_x<<" "<<start_y<<" "<<max_i<<" "<<max_j<<std::endl;

    for (int i=0; i<max_i; i++)
    {
        for (int j=0; j<max_j; j++)
        {
            write_to_file(5*i+start_x, 5*j+start_y, 0, 0.0);
        }
    }

    myfile.close();
    std::cout<<"txt saved."<<std::endl;
    //std::cout<<"pointcloud callback end... in " << time.toc () << " ms"<<std::endl;
    ros::shutdown();
}


void cut_map(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

    pcl::PointXYZ min_point, max_point;
    pcl::getMinMax3D (*cloud_largemap, min_point, max_point);
    min_point.x = min_point.x - 1.0;
    min_point.y = min_point.y - 1.0;
    max_point.x = max_point.x + 1.0;
    max_point.y = max_point.y + 1.0;

    double start_x = min_point.x +35.0;
    double start_y = min_point.y +35.0;
    int max_i = ((max_point.x-min_point.x)-70)/5;
    int max_j = ((max_point.y-min_point.y)-70)/5;
    int count = 0;

    std::cout<<start_x<<" "<<start_y<<" "<<max_i<<" "<<max_j<<std::endl;

    for (int i=0; i<max_i; i++)
    {
        for (int j=0; j<max_j; j++)
        {
            count++;
            std::cout<<"loop "<<count<<std::endl;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_minimap(new pcl::PointCloud<pcl::PointXYZ>);
            pass.setInputCloud (cloud_largemap);
            pass.setFilterFieldName ("x");
            pass.setFilterLimits (5*i+start_x-35.0, 5*i+start_x+35.0);
            pass.filter (*cloud_minimap);
            pass.setInputCloud (cloud_minimap);
            pass.setFilterFieldName ("y");
            pass.setFilterLimits (5*j+start_y-35.0, 5*j+start_y+35.0);
            pass.filter (*cloud_minimap);
            pass.setInputCloud (cloud_minimap);
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg (*cloud_minimap, output);
            output.header.frame_id = global_frame_id_;
            pub_minimap.publish(output);
            if (cloud_minimap->size()>cloud->size()*0.5)
            {
                searching(cloud, cloud_minimap, 5*i+start_x, 5*j+start_y);
            }
            else
                std::cout<<"skip loop due to size: "<<cloud_minimap->size()<<std::endl;

        }
    }



}

void pointcloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::console::TicToc time;
    time.tic ();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transform(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*input, *cloud_in);
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
    std::cout<<"scan is in"<<std::endl;
    voxel_filter.setLeafSize (0.2, 0.2, 0.2);
    voxel_filter.setInputCloud (cloud_in);
    voxel_filter.filter (*cloud_transform);
    cut_map(cloud_transform);


        tf::Matrix3x3(transform_result.getRotation()).getRPY(r, p, y);
        //std::cout<<"first transform end..."<<std::endl;                

        tf::quaternionTFToMsg(transform_result.getRotation(), input_odom.pose.pose.orientation);
        tf::Vector3 new_nav_position = transform_result.getOrigin();

        input_odom.pose.pose.position.x = new_nav_position.x();
        input_odom.pose.pose.position.y = new_nav_position.y();
        input_odom.pose.pose.position.z = new_nav_position.z();
//        input_odom.pose.pose.position.z = wheel_in_z;

        std::cout<<"x: "<<new_nav_position.x()<<" y: "<<new_nav_position.y()<<" z: "<<new_nav_position.z()<<std::endl;
        std::cout<<"R: "<<r<<" P: "<<p<<" Y: "<<y<<std::endl;
        input_odom.header.frame_id = global_frame_id_;

        pub_icp_odom.publish(input_odom);

        geometry_msgs::PoseWithCovarianceStamped icp_pose;
        icp_pose.header = input_odom.header;
        icp_pose.pose = input_odom.pose;

        pub_localize.publish(icp_pose);



        sensor_msgs::PointCloud2 output2;
        pcl_ros::transformPointCloud (*cloud_in, *final_cloud, transform_result);

        pcl::toROSMsg (*final_cloud, output2);
        output2.header.frame_id = global_frame_id_;
        pub_pointcloud.publish(output2);

        myfile.close();
        std::cout<<"txt saved."<<std::endl;
        std::cout<<"pointcloud callback end... in " << time.toc () << " ms"<<std::endl;
        ros::shutdown();

}

void initial_pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& posing)
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

    location_confirm=true;
    //start=true;

}



int main(int argc, char** argv)
{
	// Initialize ROS
        ros::init(argc, argv, "global_icp");
	ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

	ROS_INFO("start");
	
	std::string map_location;
	priv_nh.getParam("map_location", map_location);
        priv_nh.getParam("file_location", file_location);
        priv_nh.getParam("base_frame_id", base_frame_id_);
        priv_nh.getParam("global_frame_id", global_frame_id_);
        priv_nh.getParam("odom_frame_id", odom_frame_id_);

        myfile.open ("/home/weikang/weikang_ws/src/pointcloud_cluster/src/pointcloud_registration/map/kidnap_pose.txt");
        myfile<<"pose_x pose_y degree score\n";

        icp.setMaximumIterations (5);
        icp.setMaxCorrespondenceDistance(5.0);

    ros::Subscriber input_pointcloud = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, pointcloud_cb);
    ros::Subscriber input_localize = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped> ("/initialpose",1, initial_pose_cb);

    // Create a ROS publisher for the output point cloud
    pub_icp_odom = nh.advertise<nav_msgs::Odometry> ("icp_odom", 1);
    pub_input_odom = nh.advertise<nav_msgs::Odometry> ("input_odom", 1);
    pub_map = nh.advertise<sensor_msgs::PointCloud2> ("/large_map", 1);
    pub_minimap = nh.advertise<sensor_msgs::PointCloud2> ("/minimap",1);
    pub_aligned = nh.advertise<sensor_msgs::PointCloud2> ("/velo_cloud",1);
    pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2> ("/running_pointcloud",1);
    vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 1);
    pub_localize = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("/initial_pose", 1);

    result = 100.0;

    pcl::io::loadPCDFile(map_location, *cloud_largemap);

    std::cout<<cloud_largemap->size()<<std::endl;

    //pose_info(cloud_largemap);

    ros::spin();
}
