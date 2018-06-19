#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/console/time.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <pcl_ros/filters/passthrough.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/MarkerArray.h>

#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


ros::Publisher pub_floor, pub_static, pub_move, image_pub;

tf::TransformListener *tf_listener_;

Eigen::Vector3d intrinsic_1_, intrinsic_2_, intrinsic_3_;

bool image_received, pcl_received;

double img_time, pcl_time;

std::vector<std::vector<double>* >* box_points;

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_comeout(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_left_over(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_floor(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_floor_dense(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_color(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_on_image(new pcl::PointCloud<pcl::PointXYZI>);

pcl::PassThrough<pcl::PointXYZI> pass;

cv_bridge::CvImagePtr cv_ptr, cv_ptr2;

tf::Transform lidar_to_cam;

void fusioncallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::PointCloud2ConstPtr& input)
{
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    pcl::fromROSMsg (*input, *cloud_in);

    int cloud_size = cloud_in->size();

    pass.setInputCloud (cloud_in);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0.85, 50.0);
    pass.filter (*cloud_in);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-25.0, 25.0);
    pass.filter (*cloud_in);

    pcl_ros::transformPointCloud(*cloud_in, *cloud_color, lidar_to_cam);

    sensor_msgs::PointCloud2 output3;
    pcl::toROSMsg(*cloud_color, output3);
    output3.header = input->header;
    pub_static.publish(output3);

    int count=0;

    std::cout<<"Pointcloud size: "<<cloud_size<<" to: "<<cloud_color->size()<<std::endl;
    pcl::PointXYZI point_left;
    pcl::PointXYZI point_project;
    cloud_on_image->clear();
    std::cout<<"image receive: "<<image_received<<std::endl;

    for (int i=0; i<cloud_color->size(); i++)
    {
        point_left.x=-cloud_color->points[i].y;
        point_left.y=-cloud_color->points[i].z;
        point_left.z=cloud_color->points[i].x;

        Eigen::Vector3d point_vector(point_left.x, point_left.y, point_left.z);


        point_project.z = point_vector.dot(intrinsic_3_);
        point_project.x = point_vector.dot(intrinsic_1_)/point_project.z;
        point_project.y = point_vector.dot(intrinsic_2_)/point_project.z;
        point_project.z = point_project.z/point_project.z;


        if (point_project.x>0.0 && point_project.x<2048.0 && point_project.y>0.0 && point_project.y<1536.0)
        {
            point_project.intensity = count;
            cloud_on_image->push_back(point_project);
            //std::cout<<"image receive: "<<image_received<<std::endl;
//            if (image_received)
//            {
                cv::circle(cv_ptr->image, cv::Point(point_project.x/2, point_project.y/2), 0.5, CV_RGB(255,0,0));
//            }

            count++;
        }
    }

    sensor_msgs::PointCloud2 output2;
    pcl::toROSMsg(*cloud_on_image, output2);
    output2.header = input->header;
    pub_move.publish(output2);

    //cloud_on_image->clear();
    cloud_left_over->clear();
    box_points->clear();

    image_pub.publish(cv_ptr->toImageMsg());

    img_time = msg->header.stamp.toSec();
    pcl_time = input->header.stamp.toSec();

    double time_diff = img_time-pcl_time;
    std::cout<<"time diff: "<<time_diff<<std::endl;
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    //std::cout<<"taking image"<<std::endl;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    //std::cout<<"after taking image"<<std::endl;
    img_time = msg->header.stamp.toSec();

    double time_diff = img_time-pcl_time;
    std::cout<<"image time diff: "<<time_diff<<std::endl;

    if (std::abs(time_diff)<0.04)
    {
        for (int i=0; i<cloud_on_image->size(); i++)
        {
            cv::circle(cv_ptr->image, cv::Point(cloud_on_image->points[i].x/2, cloud_on_image->points[i].y/2), 0.5, CV_RGB(255,0,0));

        }
        pcl_received=false;
        image_pub.publish(cv_ptr->toImageMsg());
        std::cout<<"published"<<std::endl;
    }
    image_received=true;

}

//void imageCb2(const sensor_msgs::ImageConstPtr& msg)
//{
//    std::cout<<"taking image"<<std::endl;
//    try
//    {
//      cv_ptr2 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//    }
//    catch (cv_bridge::Exception& e)
//    {
//      ROS_ERROR("cv_bridge exception: %s", e.what());
//      return;
//    }
//    std::cout<<"after taking image"<<std::endl;
//    image_received=true;

//}

void box_cb(const visualization_msgs::MarkerConstPtr& markers_array)
{
    std::cout<<markers_array->id<<" and "<<markers_array->points.size()<<std::endl;
    std::vector<double>* corner_points = new std::vector<double>;

    if ((markers_array->id)*4 == markers_array->points.size())
    {
        corner_points->resize(markers_array->points.size());
        for (int i=0; i<markers_array->id; i++)
        {
            double left_corner_x = markers_array->points[4*i].x;
            double left_corner_y = markers_array->points[4*i].y;
            corner_points->push_back(left_corner_x);
            corner_points->push_back(left_corner_y);
            double right_corner_x = markers_array->points[4*i+2].x;
            double right_corner_y = markers_array->points[4*i+2].y;
            corner_points->push_back(right_corner_x);
            corner_points->push_back(right_corner_y);
            box_points->push_back(corner_points);
            corner_points->clear();
        }
    }

}

void pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::console::TicToc time;
    time.tic ();

    pcl::fromROSMsg (*input, *cloud_in);

    int cloud_size = cloud_in->size();

    pass.setInputCloud (cloud_in);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0.85, 50.0);
    pass.filter (*cloud_in);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-25.0, 25.0);
    pass.filter (*cloud_in);

    pcl_ros::transformPointCloud(*cloud_in, *cloud_color, lidar_to_cam);

    sensor_msgs::PointCloud2 output3;
    pcl::toROSMsg(*cloud_color, output3);
    output3.header = input->header;
    pub_static.publish(output3);

    int count=0;

    std::cout<<"Pointcloud size: "<<cloud_size<<" to: "<<cloud_color->size()<<std::endl;
    pcl::PointXYZI point_left;
    pcl::PointXYZI point_project;
    cloud_on_image->clear();
    std::cout<<"image receive: "<<image_received<<std::endl;

    for (int i=0; i<cloud_color->size(); i++)
    {        
        point_left.x=-cloud_color->points[i].y;
        point_left.y=-cloud_color->points[i].z;
        point_left.z=cloud_color->points[i].x;

        Eigen::Vector3d point_vector(point_left.x, point_left.y, point_left.z);


        point_project.z = point_vector.dot(intrinsic_3_);
        point_project.x = point_vector.dot(intrinsic_1_)/point_project.z;
        point_project.y = point_vector.dot(intrinsic_2_)/point_project.z;
        point_project.z = point_project.z/point_project.z;


        if (point_project.x>0.0 && point_project.x<2048.0 && point_project.y>0.0 && point_project.y<1536.0)
        {
            point_project.intensity = count;
            cloud_on_image->push_back(point_project);
            //std::cout<<"image receive: "<<image_received<<std::endl;
//            if (image_received)
//            {
//                cv::circle(cv_ptr->image, cv::Point(point_project.x/2, point_project.y/2), 0.5, CV_RGB(255,0,0));
//            }

            count++;
        }
    }

    sensor_msgs::PointCloud2 output2;
    pcl::toROSMsg(*cloud_on_image, output2);
    output2.header = input->header;
    pub_move.publish(output2);

    //cloud_on_image->clear();
    cloud_left_over->clear();
    box_points->clear();

//    if (image_received)
//    {
//        image_pub.publish(cv_ptr->toImageMsg());
//    }

    pcl_time = input->header.stamp.toSec();

    //std::cout<<"pcl difference: "<<pcl_time-img_time<<std::endl;
    pcl_received=true;

    std::cout<<"time taken: "<<time.toc()<<std::endl;


}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pointcloud_filter_box");
    ros::NodeHandle nh;

    tf_listener_ = new tf::TransformListener();

    ros::Subscriber sub_box = nh.subscribe<visualization_msgs::Marker> ("box", 1, box_cb);
    //ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2> ("cluster", 1, pointcloud_cb);
    //ros::Subscriber sub_image = nh.subscribe<sensor_msgs::Image> ("/iMiev/tf_object_detection", 1, imageCb);
    //ros::Subscriber sub_image2 = nh.subscribe<sensor_msgs::Image> ("/camera/image_raw", 1, imageCb2);

    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/iMiev/tf_object_detection", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, "cluster", 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2> sync(image_sub, pcl_sub, 10);
    sync.registerCallback(boost::bind(&fusioncallback, _1, _2));

    pub_move = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_move", 1);
    pub_floor = nh.advertise<sensor_msgs::PointCloud2> ("/floor_points", 1);
    pub_static = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_out", 1);
    //pub_cluster = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_left", 1);
    image_pub = nh.advertise<sensor_msgs::Image> ("image_output", 1);

    Eigen::Vector3d temp_intr1(1217, 0.0, 1087.4);
    Eigen::Vector3d temp_intr2(0.0, 1209.1, 834.55);
//    Eigen::Vector3d temp_intr1(484.538, 0.0, 371.507);
//    Eigen::Vector3d temp_intr2(0.0, 484.798, 184.8);
    Eigen::Vector3d temp_intr3(0.0, 0.0, 1.0);
    //intrinsic_ << 484.538, 0.0, 371.507, 0.0, 484.798, 184.8, 0.0, 0.0, 1.0;
    intrinsic_1_ = temp_intr1;
    intrinsic_2_ = temp_intr2;
    intrinsic_3_ = temp_intr3;

    lidar_to_cam.setOrigin(tf::Vector3(-0.8, 0.0, 0.4));
    lidar_to_cam.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

    box_points = new std::vector<std::vector<double>* >;

    image_received = false;

    ros::spin ();
}
