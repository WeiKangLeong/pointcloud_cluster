#include "pointcloud_transform/pointcloud_combine.h"

#define PI_ 3.1415926

namespace pointcloud_transform {

PCL_combine::PCL_combine()
{
    sub_top_ = nh_.subscribe <sensor_msgs::PointCloud2> ("top_front_points", 1, &PCL_combine::top_front_cb_, this);
    sub_rear_ = nh_.subscribe <sensor_msgs::PointCloud2> ("top_rear_points", 1, &PCL_combine::top_rear_cb_, this);
    //sub_front_ = nh_.subscribe <sensor_msgs::PointCloud2> ("bottom_front_points", 1, &PCL_combine::bottom_front_cb_, this);
    pub_pcl_ = nh_.advertise<sensor_msgs::PointCloud2> ("combine_points", 1);

    ros::NodeHandle priv_nh("~");
    priv_nh.getParam("x", x_);
    priv_nh.getParam("y", y_);
    priv_nh.getParam("z", z_);
    priv_nh.getParam("roll", roll_);
    priv_nh.getParam("pitch", pitch_);
    priv_nh.getParam("yaw", yaw_);

    roll_ = roll_*PI_/180.0;
    pitch_ = pitch_*PI_/180.0;
    yaw_ = yaw_*PI_/180.0;

    quat_.setRPY(roll_, pitch_, yaw_);

    transform_.setOrigin(tf::Vector3(x_, y_, z_));
    transform_.setRotation(quat_);
}

PCL_combine::~PCL_combine()
{

}

void PCL_combine::combine_pcl_ (sensor_msgs::PointCloud2 front_pcl, sensor_msgs::PointCloud2 rear_pcl)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_front (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rear (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_combine (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg (front_pcl, *cloud_front);
    pcl::fromROSMsg (rear_pcl, *cloud_rear);

    *cloud_combine = *cloud_front + *cloud_rear;

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg (*cloud_combine, output);
    output.header = front_pcl.header;
    output.header.frame_id = "base_link";

    pub_pcl_.publish(output);
}

bool PCL_combine::compare_time_ (std_msgs::Header time, bool front_or_rear)
{
    double difference=0.0;

    if (front_or_rear==false)
    {
        difference = time.stamp.toSec() - top_rear_buffer_.header.stamp.toSec();
        //std::cout<<front_or_rear<<" time difference: "<<difference<<std::endl;
        //std::cout<<time.stamp.toSec() - top_front_buffer_.header.stamp.toSec()<<std::endl;
    }
    else if (front_or_rear==true)
    {
        difference = time.stamp.toSec() - top_front_buffer_.header.stamp.toSec();
        //std::cout<<front_or_rear<<" time difference: "<<difference<<std::endl;
        //std::cout<<time.stamp.toSec() - top_rear_buffer_.header.stamp.toSec()<<std::endl;
    }



    if (std::abs(difference)<0.05 && std::abs(difference)>0.0)
        return true;
    else return false;

}

void PCL_combine::top_front_cb_ (const sensor_msgs::PointCloud2ConstPtr& input)
{
    top_front_buffer_ = *input;

    //std::cout<<"front time: "<<top_front_buffer_.header.stamp.toNSec();
    if (compare_time_(top_front_buffer_.header, 0))
    {
        combine_pcl_(top_front_buffer_, top_rear_buffer_);
    }
}

void PCL_combine::top_rear_cb_ (const sensor_msgs::PointCloud2ConstPtr& input)
{
    top_rear_buffer_ = *input;
    //std::cout<<"rear time: "<<top_rear_buffer_.header.stamp.toNSec();
    if (compare_time_(top_rear_buffer_.header, 1))
    {
        combine_pcl_(top_front_buffer_, top_rear_buffer_);
    }
}
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pointcloud_readjust");
    
    pointcloud_transform::PCL_combine pcl_combine;
    // Spin
    ros::spin ();

    return 0;
}

