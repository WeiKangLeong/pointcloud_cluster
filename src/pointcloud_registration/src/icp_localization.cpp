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
//#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/common/common.h>

ros::Publisher pub_icp_odom, pub_input_odom, pub_particle_odom, pub_map, pub_aligned, pub_minimap, pub_pointcloud, vis_pub;

//typedef pcl::PointXYZI PointT;
//typedef pcl::PointCloud<PointXYZI> PointCloudT;

nav_msgs::Odometry guess_odom, initial_odom, offset_odom, input_odom, particle_odom[5];

double global_roll, global_pitch, global_yaw, r, p, y, particle_probability[5], guess_movement, first_score, second_score,
        i_r, i_p, i_y, wheel_in_z, diff_p, prev_p;

tf::TransformBroadcaster *tfb;
tf::TransformListener *tf_listener_;
tf::Transform transform, transform2, transform3, offset_transform, odom_transform, now_odom_transform, prev_odom_transform, diff_odom_transform,
                turn90, difference, initial_pose, vehicle_pose, imu_transform, previous_transform;
tf::Quaternion quater;
tf::Vector3 difference_pose, wheel_position, now_transform, prev_diff_pose;
Eigen::Matrix4f second_matrix, Point[10],theOne;

// The point clouds we will be using
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_previous(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transform(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_minimap(new pcl::PointCloud<pcl::PointXYZ>);
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

Eigen::Matrix4f self_localize()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_icp = *cloud_in;
    if (start==false)
        *cloud_previous =*cloud_in;
    Eigen::Matrix4f guess_orient;

    icp.setInputSource (cloud_previous);
    icp.setInputTarget (cloud_icp);
    icp.align (*cloud_previous);
    guess_orient=icp.getFinalTransformation();
    tf::Matrix3x3 mat;
    mat.setValue(guess_orient(0,0),guess_orient(0,1),guess_orient(0,2),guess_orient(1,0),guess_orient(1,1),guess_orient(1,2),
                 guess_orient(2,0),guess_orient(2,1),guess_orient(2,2));
    double r1,p1,y1;
    mat.getRPY (r1,p1,y1);
//    std::cout<<"x: "<<guess_orient(0,3)<<" y: "<<guess_orient(1,3)<<" z: "<<guess_orient(2,3)<<std::endl;
//    std::cout<<"r: "<<r1<<" p: "<<p1<<" y: "<<y1<<std::endl;
//    std::cout<<"score fitness: "<<icp.getFitnessScore()<<std::endl;
    find_orient=true;
    return(guess_orient);
}

double icp_with_score(Eigen::Matrix4f nnn)
{
    //std::cout<<"icp with score start..."<<std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_particle(new pcl::PointCloud<pcl::PointXYZ>);
    icp.setMaximumIterations (1);
    icp.setInputSource (filtered_cloud);
    icp.setInputTarget (cloud_minimap);
    icp.align (*cloud_particle, nnn);

    //std::cout<<"icp with score end..."<<std::endl;

    return (icp.getFitnessScore());
}


Eigen::Matrix4f localize()
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_comeout(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_origin(new pcl::PointCloud<pcl::PointXYZ>);

    tf::Transform origin_tf = transform.inverse();

    pcl_ros::transformPointCloud (*cloud_minimap, *map_origin, origin_tf);
//    sensor_msgs::PointCloud2 output3;

//    pcl::toROSMsg (*map_origin, output3);
//    output3.header.frame_id = "map";
//    pub_map.publish(output3);

    double score;

    Eigen::Matrix4f known_orient,predict_forward, predict_left, predict_right, previous_transform, use_odom, odom_matrix, compare_matrix;
    //known_orient = self_localize();
    use_odom = setTransformMatrix(0.0, true);


    //*cloud_comein = *input;
    icp.setMaximumIterations (5);
    icp.setMaxCorrespondenceDistance(2.0);
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
    std::cout << "s_score = "<< second_score<<std::endl;

    if (chance==5)
    {
        compare_matrix=use_odom;
        std::cout<<"observe..."<<std::endl;
        chance=0;
    }
    else if (chance==4)
    {
        std::cout<<"I think we can give him a chance"<<std::endl;
        chance++;
    }
    else if (score<0.5 && second_score<0.5)
    {
        std::cout<<"I think it is a good icp"<<std::endl;
    }
    else if (score<0.5)
    {
        chance++;
        std::cout<<"Icp ok for "<<chance<<" times"<<std::endl;
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
        chance=0;
        compare_matrix=use_odom;
        std::cout<<"using odom info..."<<std::endl;
    }


    std::cout << "Applied iterations with " <<score<< std::endl;

    prev_diff_pose=difference_pose;

//    sensor_msgs::PointCloud2 output2;
//    pcl::transformPointCloud (*filtered_cloud, *cloud_return, compare_matrix[1]);
//    pcl::toROSMsg (*cloud_return, output2);
//    output2.header.frame_id = "map";
//    pub_aligned.publish(output2);
    return (compare_matrix);
}

void pointcloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    tf::StampedTransform latest_odom_transform;
    try{
        tf_listener_->lookupTransform("wtf_odom", "wtf_base_link",
                                      ros::Time(0), latest_odom_transform);
    }catch (tf::TransformException &ex) {
        ROS_ERROR_STREAM("localizer_scan: Looking Transform Failed");
        return;
    }

    if (odom_start==false)
    {
        offset_transform = latest_odom_transform;
        odom_start=true;

    }



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
        tf::quaternionTFToMsg(vehicle_pose.getRotation(), input_odom.pose.pose.orientation);
        wheel_position = vehicle_pose.getOrigin();
        input_odom.pose.pose.position.x = wheel_position.x();
        input_odom.pose.pose.position.y = wheel_position.y();
//        wheel_in_z = wheel_in_z + sqrt((difference_pose.x()*difference_pose.x())+(difference_pose.y()*difference_pose.y()))*tan(diff_p);
//        input_odom.pose.pose.position.z = wheel_in_z;
        input_odom.pose.pose.position.z = wheel_position.z();
        input_odom.header.frame_id = "map";
        pub_input_odom.publish(input_odom);
        /***--------------------------------******/



        // Defining a rotation matrix and translation vector
        Eigen::Matrix4f transformation_matrix;
        float translation_x, translation_y, translation_z;
        double roll, pitch, yaw;
        now_transform=transform.getOrigin();
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

        pcl::toROSMsg (*cloud_minimap, output_minimap);
        output_minimap.header.frame_id = "map";
        pub_minimap.publish(output_minimap);



        /***------------reposition z using map info-------------***/

        //transform.setOrigin(tf::Vector3(now_transform.x(), now_transform.y(), low+3));

//        if (location_confirm || start)
//        pcl_ros::transformPointCloud (*cloud_in, *cloud_transform, transform);
//        else
//            *cloud_transform=*cloud_in;

//        pcl::toROSMsg (*cloud_transform, output_running_pointcloud);
//        output_running_pointcloud.header.frame_id = "map";
        //pub_pointcloud.publish(output_running_pointcloud);

        voxel_filter.setLeafSize (0.2, 0.2, 0.2);
        voxel_filter.setInputCloud (cloud_in);
        voxel_filter.filter (*cloud_transform);
        std::cout << "Filtered cloud contains " << cloud_transform->size ()
              << " data points from velodyne point cloud" << std::endl;


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

        transform2.setOrigin(tf::Vector3(translation_x, translation_y, translation_z));
        transform2.setRotation(tfqt);


        transform = transform * transform2;

        tf::Matrix3x3(transform.getRotation()).getRPY(r, p, y);
        //std::cout<<"first transform end..."<<std::endl;                

        tf::quaternionTFToMsg(transform.getRotation(), input_odom.pose.pose.orientation);
        tf::Vector3 new_nav_position = transform.getOrigin();

        input_odom.pose.pose.position.x = new_nav_position.x();
        input_odom.pose.pose.position.y = new_nav_position.y();
        input_odom.pose.pose.position.z = new_nav_position.z();
//        input_odom.pose.pose.position.z = wheel_in_z;

        std::cout<<"x: "<<new_nav_position.x()<<" y: "<<new_nav_position.y()<<" z: "<<new_nav_position.z()<<std::endl;
        std::cout<<"R: "<<r<<" P: "<<p<<" Y: "<<y<<std::endl;
        input_odom.header.frame_id = "map";

        pub_icp_odom.publish(input_odom);

        tf::StampedTransform odom_transform_stamped(transform, input->header.stamp, "/wtf_base_link", "/wtf_icp");
        tfb->sendTransform(odom_transform_stamped);

        sensor_msgs::PointCloud2 output2;
        pcl_ros::transformPointCloud (*cloud_in, *final_cloud, transform);

        pcl::toROSMsg (*final_cloud, output2);
        output2.header.frame_id = "map";
        pub_aligned.publish(output2);

        start=true;
        //*cloud_previous=*cloud_in;
        prev_odom_transform = now_odom_transform;
        previous_transform = transform2;
        std::cout<<"pointcloud callback end... in " << time.toc () << " ms"<<std::endl;

        odom_received = false;
    //}




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

//void odom_cb (const nav_msgs::Odometry::ConstPtr& odom)
//{
//    if (odom_start==false)
//    {
//        tf::Quaternion offset_rotation;
//        tf::quaternionMsgToTF(odom->pose.pose.orientation, offset_rotation);
//        offset_transform.setOrigin(tf::Vector3(odom->pose.pose.position.x,odom->pose.pose.position.y,odom->pose.pose.position.z));
//        offset_transform.setRotation(offset_rotation.normalize());
//        odom_start=true;
//        prev_odom_transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
//        prev_odom_transform.setRotation(tf::Quaternion(0.0,0.0,0.0,1.0));
//    }

//    tf::Quaternion new_rotation;
//    tf::quaternionMsgToTF(odom->pose.pose.orientation, new_rotation);
//    odom_transform.setOrigin(tf::Vector3(odom->pose.pose.position.x,odom->pose.pose.position.y,odom->pose.pose.position.z));
//    odom_transform.setRotation(new_rotation.normalize());

//    odom_transform = offset_transform.inverseTimes(odom_transform);



//    odom_received = true;



//}

void imu_cb(sensor_msgs::Imu::ConstPtr imu_data)
{
    tf::Quaternion imu_rotation;
    tf::quaternionMsgToTF(imu_data->orientation, imu_rotation);
    imu_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    imu_transform.setRotation(imu_rotation);
    tf::Matrix3x3(imu_transform.getRotation()).getRPY(i_r, i_p, i_y);

}




int main(int argc, char** argv)
{
	// Initialize ROS
	ros::init(argc, argv, "my_pcl_tutorial");
	ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

	//i=0;j=0;k=0;l=0;
	ROS_INFO("start");



	// Create a ROS subscriber for the input point cloud
    //ros::Subscriber input_odom = nh.subscribe<nav_msgs::Odometry> ("odom", 1, odom_cb);
    ros::Subscriber input_pointcloud = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, pointcloud_cb);
	ros::Subscriber input_localize = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped> ("/initialpose",1, initial_pose_cb);
    ros::Subscriber input_imu = nh.subscribe<sensor_msgs::Imu> ("imu",1, imu_cb);

	// Create a ROS publisher for the output point cloud
    pub_icp_odom = nh.advertise<nav_msgs::Odometry> ("icp_odom", 1);
    pub_input_odom = nh.advertise<nav_msgs::Odometry> ("input_odom", 1);
    pub_map = nh.advertise<sensor_msgs::PointCloud2> ("/large_map", 1);
	pub_minimap = nh.advertise<sensor_msgs::PointCloud2> ("/minimap",1);
    pub_aligned = nh.advertise<sensor_msgs::PointCloud2> ("/velo_cloud",1);
    pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2> ("/running_pointcloud",1);
    vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 1);

    tfb = new tf::TransformBroadcaster();
    tf_listener_ = new tf::TransformListener();
    odom_start = false;

    transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
    transform.setRotation(tf::Quaternion(0.0,0.0,0.0,1.0));

    prev_odom_transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
    prev_odom_transform.setRotation(tf::Quaternion(0.0,0.0,0.0,1.0));

    tf::Quaternion degree90;
    degree90.setRPY(0.0,0.0,90.0);
    turn90.setOrigin(tf::Vector3(0,0,0));
    turn90.setRotation(degree90);

//    pcl::io::loadPCDFile("/home/smaug/Desktop/Raw Data/new/utown_big.pcd", *cloud_largemap);
//    pcl::io::loadPCDFile("/home/smaug/Desktop/Raw Data/loam_map.pcd", *cloud_largemap);
//    pcl::io::loadPCDFile("/home/smaug/Desktop/Raw Data/map_binary_compressed.pcd", *cloud_largemap);
//    pcl::io::loadPCDFile("/home/smaug/Desktop/Raw Data/small_loop/small_loop_onenorth.pcd", *cloud_largemap);
//   pcl::io::loadPCDFile("/home/smaug/Desktop/Raw Data/170830_onenorth/map_voxel_grid.pcd", *cloud_largemap);
    pcl::io::loadPCDFile("/home/smaug/Desktop/Raw Data/utown_3d/utown_clean_zerotwo.pcd", *cloud_largemap);
   //pcl_ros::transformPointCloud (*cloud_largemap, *cloud_largemap, turn90);

//    voxel_filter.setLeafSize (0.1, 0.1, 0.1);
//    voxel_filter.setInputCloud (cloud_largemap);
//    voxel_filter.filter (*cloud_largemap);
//    outrem.setInputCloud(cloud_largemap);
//    outrem.setRadiusSearch(1.0);
//    outrem.setMinNeighborsInRadius (10);
//    // apply filter
//    outrem.filter (*cloud_largemap);

    std::cout << "Filtered map contains " << cloud_largemap->size ()
          << " data points from map" << std::endl;

//    sensor_msgs::PointCloud2 output;
//    pcl::toROSMsg (*cloud_largemap, output);
//    output.header.frame_id = "map";
//    pub_map.publish(output);
    ROS_INFO("map loaded.");
	// Spin
    ros::spin();
}
