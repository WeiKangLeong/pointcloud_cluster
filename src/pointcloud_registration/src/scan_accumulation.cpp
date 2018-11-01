#include <ros/ros.h>
// PCL specific includes

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <pcl/registration/icp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PointIndices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <visualization_msgs/Marker.h>

ros::Publisher pub, pub_temp_map, pub_full_map, pub_source, pub_target, pub_icp_odom;

tf::TransformListener* tf_listener_;

std::string odom_frame_id_, base_frame_id_, map_frame_id_;

int count;

bool odom_start;

tf::Transform transform, offset_transform, now_odom_transform, difference, prev_odom_transform;

Eigen::Matrix4f use_odom;

pcl::PointCloud<pcl::PointXYZI>::Ptr map (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr temp_map (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_minimap (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_accumulated (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr previous_accumulated (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr final_map (new pcl::PointCloud<pcl::PointXYZI>);

pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
pcl::PassThrough<pcl::PointXYZI> pass;

Eigen::Matrix4f localize()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_comeout(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr map_origin(new pcl::PointCloud<pcl::PointXYZI>);

//    pcl_ros::transformPointCloud(*cloud_minimap, *map_origin, transform);

    Eigen::Matrix4f compare_matrix;

    icp.setMaximumIterations (5);
    icp.setMaxCorrespondenceDistance(0.5);
    //icp.setTransformationEpsilon (1e-7);

    sensor_msgs::PointCloud2 source, target;
    pcl::toROSMsg(*cloud_minimap, target);
    target.header.frame_id = odom_frame_id_;
    pub_target.publish(target);

    icp.setInputSource (cloud_accumulated);
    icp.setInputTarget (cloud_minimap);
    icp.align (*cloud_comeout);
    double score=icp.getFitnessScore();
    compare_matrix = icp.getFinalTransformation ();


    pcl::toROSMsg(*cloud_comeout, source);
    source.header.frame_id = odom_frame_id_;
    pub_source.publish(source);


    if (score<0.2)
    {
        std::cout << "Applied iterations with " <<score<< std::endl;
    }
    else
    {
        compare_matrix = use_odom;
    }


    return (compare_matrix);
}

void pc_cb(const sensor_msgs::PointCloud2ConstPtr input)
{
    tf::StampedTransform latest_odom_transform;
    try{
        tf_listener_->lookupTransform(odom_frame_id_, base_frame_id_,
                                      ros::Time(0), latest_odom_transform);
    }catch (tf::TransformException &ex) {
        ROS_ERROR_STREAM("Looking Transform from odom and base_link Failed");
        return;
    }

    if (odom_start==false)
    {
        offset_transform = latest_odom_transform;
        odom_start=true;
    }

    now_odom_transform = offset_transform.inverseTimes(latest_odom_transform);

    difference = prev_odom_transform.inverseTimes(now_odom_transform);

    transform = transform * difference;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_transform (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg (*input, *cloud_in);

    pcl_ros::transformPointCloud(*cloud_in, *cloud_transform, transform);

    *temp_map += *cloud_transform;



    count++;

    if (count==5)
    {
        count=0;

        voxel_filter.setLeafSize (0.2, 0.2, 0.2);
        voxel_filter.setInputCloud (temp_map);
        voxel_filter.filter (*cloud_accumulated);
        if (map->size()==0)
        {
            *map += *cloud_accumulated;
            *previous_accumulated = *cloud_accumulated;
        }

        tf::Vector3 odom_now, odom_previous;
        odom_now = now_odom_transform.getOrigin();
        odom_previous = prev_odom_transform.getOrigin();
        pcl_ros::transformPointCloud(*cloud_accumulated, *cloud_accumulated, now_odom_transform.inverse());

        pass.setInputCloud (cloud_accumulated);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (-30.0, 30.0);
        pass.filter (*cloud_accumulated);

        pass.setInputCloud (cloud_accumulated);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (-30.0, 30.0);
        pass.filter (*cloud_accumulated);


        pcl_ros::transformPointCloud(*previous_accumulated, *previous_accumulated, prev_odom_transform.inverse());
        pass.setInputCloud (previous_accumulated);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (-30.0, 30.0);
        pass.filter (*cloud_minimap);

        pass.setInputCloud (cloud_minimap);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (-35.0, 35.0);
        pass.filter (*cloud_minimap);


        Eigen::Matrix4f transformation_matrix = localize();

        double translation_x = transformation_matrix(0,3);
        double translation_y = transformation_matrix(1,3);
        double translation_z = transformation_matrix(2,3);

        tf::Matrix3x3 tf3d;
        tf3d.setValue(static_cast<double>(transformation_matrix(0,0)), static_cast<double>(transformation_matrix(0,1)), static_cast<double>(transformation_matrix(0,2)),
            static_cast<double>(transformation_matrix(1,0)), static_cast<double>(transformation_matrix(1,1)), static_cast<double>(transformation_matrix(1,2)),
            static_cast<double>(transformation_matrix(2,0)), static_cast<double>(transformation_matrix(2,1)), static_cast<double>(transformation_matrix(2,2)));

        tf::Quaternion tfqt;
        //tf3d.getRotation(tfqt);
        double roll, pitch, yaw;
        tf3d.getRPY (roll,pitch,yaw);
        //tf3d.setRPY (0.0, pitch, yaw);
        tf3d.getRotation(tfqt);

        tf::Transform transform2;
        transform2.setOrigin(tf::Vector3(translation_x, translation_y, translation_z));
        transform2.setRotation(tfqt);

        transform = transform * transform2;

        pcl::PointCloud<pcl::PointXYZI>::Ptr transform_temp (new pcl::PointCloud<pcl::PointXYZI>);
        pcl_ros::transformPointCloud(*temp_map, *transform_temp, transform2);

        sensor_msgs::PointCloud2 tempmap_output;
        pcl::toROSMsg (*transform_temp, tempmap_output);
        tempmap_output.header.frame_id = odom_frame_id_;
        pub_temp_map.publish(tempmap_output);

        nav_msgs::Odometry icp_odom;
        tf::quaternionTFToMsg(transform.getRotation(), icp_odom.pose.pose.orientation);
        tf::Vector3 icp_position = transform.getOrigin();
        icp_odom.pose.pose.position.x = icp_position.x();
        icp_odom.pose.pose.position.y = icp_position.y();
        icp_odom.pose.pose.position.z = icp_position.z();

        icp_odom.header.frame_id = odom_frame_id_;
        icp_odom.header.stamp = input->header.stamp;

        pub_icp_odom.publish(icp_odom);

//        *map += *temp_map;

//        voxel_filter.setInputCloud (map);
//        voxel_filter.filter (*final_map);

        previous_accumulated.reset(new pcl::PointCloud<pcl::PointXYZI>);
        *previous_accumulated += *temp_map;
        temp_map.reset(new pcl::PointCloud<pcl::PointXYZI>);
        map.reset(new pcl::PointCloud<pcl::PointXYZI>);
        cloud_accumulated.reset(new pcl::PointCloud<pcl::PointXYZI>);
        cloud_minimap.reset(new pcl::PointCloud<pcl::PointXYZI>);

//        *map = *final_map;

//        sensor_msgs::PointCloud2 map_output;
//        pcl::toROSMsg (*map, map_output);
//        map_output.header.frame_id = map_frame_id_;
//        pub_full_map.publish(map_output);

    }

    prev_odom_transform = now_odom_transform;
}

int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "show_map");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    tf_listener_ = new tf::TransformListener();

    priv_nh.getParam("odom_frame", odom_frame_id_);
    priv_nh.getParam("base_frame", base_frame_id_);
    priv_nh.getParam("map_frame", map_frame_id_);

    ros::Subscriber sub_pcl = nh.subscribe <sensor_msgs::PointCloud2> ("rslidar_points", 1, pc_cb);

    pub = nh.advertise<sensor_msgs::PointCloud2> ("/map_in_total", 1);
    pub_full_map = nh.advertise<sensor_msgs::PointCloud2> ("/full_map", 1);
    pub_temp_map = nh.advertise<sensor_msgs::PointCloud2> ("/temp_map", 1);
    pub_target = nh.advertise<sensor_msgs::PointCloud2> ("/target_map", 1);
    pub_source = nh.advertise<sensor_msgs::PointCloud2> ("/source_map", 1);
    pub_icp_odom = nh.advertise<nav_msgs::Odometry> ("/icp_odom", 1);

    count = 0;

    odom_start = false;

    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

    prev_odom_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    prev_odom_transform.setRotation(tf::Quaternion (0.0, 0.0, 0.0, 1.0));

    use_odom.setIdentity();

    ros::spin();
}

