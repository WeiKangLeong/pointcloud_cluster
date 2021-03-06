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

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/PointIndices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <visualization_msgs/Marker.h>

//#include <opencv2/opencv.hpp>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

ros::Publisher pub, pub_cylinder, pub_full_map;

//using namespace cv;

int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "show_map");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    std::string map_location, global_frame_id;
    priv_nh.getParam("map_location", map_location);
    priv_nh.getParam("global_frame_id", global_frame_id);

    pub = nh.advertise<sensor_msgs::PointCloud2> ("/map_in_total", 1);
    pub_full_map = nh.advertise<sensor_msgs::PointCloud2> ("/full_map", 1);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PassThrough<pcl::PointXYZI> pass;
    
    ros::Duration(5.0).sleep();

    pcl::io::loadPCDFile(map_location, *cloud_in);

    pcl::PointXYZI min_point, max_point;
    pcl::getMinMax3D (*cloud_in, min_point, max_point);
    min_point.x = min_point.x - 1.0;
    min_point.y = min_point.y - 1.0;
    max_point.x = max_point.x + 1.0;
    max_point.y = max_point.y + 1.0;

    tf::Transform offset_transform;
//    tf::Quaternion degree90;
//    degree90.setRPY(0.0,0.0,90.0);
    offset_transform.setOrigin(tf::Vector3(-min_point.x-1.0, -min_point.y-1.0, 0.0));
    offset_transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

    pcl_ros::transformPointCloud(*cloud_in, *cloud_in, offset_transform);

    sensor_msgs::PointCloud2 output2;
    pcl::toROSMsg (*cloud_in, output2);
    output2.header.frame_id = global_frame_id;
    pub_full_map.publish(output2);

    max_point.x = max_point.x -min_point.x;
    max_point.y = max_point.y -min_point.y;
    min_point.x = 0.0;
    min_point.y = 0.0;

    int sum_size=0;
    //std::cout<<cloud_in->size()<<std::endl;

//    sensor_msgs::Image pt_image;

    if (cloud_in->size()<4000000)
    {
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg (*cloud_in, output);
        output.header.frame_id = global_frame_id;
        pub.publish (output);
    }
    else
    {
        int divide = (cloud_in->size()/1000000);
        double x_range = max_point.x - min_point.x;
        double y_range = max_point.y - min_point.y;

        for (int i=1; i<divide; i++)
        {
            for (int j=1; j<divide; j++)
            {

                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_large_map(new pcl::PointCloud<pcl::PointXYZI>);
                pass.setInputCloud (cloud_in);
                pass.setFilterFieldName ("x");
                pass.setFilterLimits (min_point.x+(x_range/divide)*(i-1), min_point.x+(x_range/divide)*i);
                pass.filter (*cloud_large_map);
                pass.setInputCloud (cloud_large_map);
                pass.setFilterFieldName ("y");
                pass.setFilterLimits (min_point.y+(y_range/divide)*(j-1), min_point.y+(y_range/divide)*j);
                pass.filter (*cloud_large_map);
                pass.setInputCloud (cloud_large_map);

                sensor_msgs::PointCloud2 output;
                pcl::toROSMsg (*cloud_large_map, output);
                output.header.frame_id = global_frame_id;
                pub.publish (output);
                sum_size = sum_size+cloud_large_map->size();
            }
        }

    }


    std::cout<<"cloud in size: "<<cloud_in->size()<<" and "<<sum_size<<" points."<<std::endl;

    ros::spin();
}

