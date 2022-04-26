#include "pointcloud_transform/pointcloud_readjust.h"

#define PI_ 3.1415926

namespace pointcloud_transform {

PCL_readjust::PCL_readjust()
{
    sub_pcl_ = nh_.subscribe <sensor_msgs::PointCloud2> ("velodyne_points", 1, &PCL_readjust::pointcloud_cb_, this);
    pub_pcl_ = nh_.advertise<sensor_msgs::PointCloud2> ("transform_points", 1);

    ros::NodeHandle priv_nh("~");
    priv_nh.getParam("x", x_);
    priv_nh.getParam("y", y_);
    priv_nh.getParam("z", z_);
    priv_nh.getParam("roll", roll_);
    priv_nh.getParam("pitch", pitch_);
    priv_nh.getParam("yaw", yaw_);
    priv_nh.getParam("frame_id", frame_id_);
    priv_nh.getParam("cut_half", cut_half_);

//    roll_ = roll_*PI_/180.0;
//    pitch_ = pitch_*PI_/180.0;
//    yaw_ = yaw_*PI_/180.0;

    quat_.setRPY(roll_, pitch_, yaw_);

    transform_.setOrigin(tf::Vector3(x_, y_, z_));
    transform_.setRotation(quat_);
}

PCL_readjust::~PCL_readjust()
{

}


void PCL_readjust::pointcloud_cb_ (const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_transform (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_search (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg (*input, *cloud_in);

    //std::cout<<input->data.size()<<" "<<cloud_in->size()<<std::endl;

    pcl_ros::transformPointCloud(*cloud_in, *cloud_transform, transform_);

    //std::cout<<"pcl field: "<<input->fields[0]<<" "<<input->fields[1]<<" "<<input->fields[2]<<" "<<input->fields[3]<<" "<<input->fields[4]<<std::endl;
    //std::cout<<"pointcloud width: "<<input->width<<" height: "<<input->height<<std::endl;
    //std::cout<<"cloud_in: "<<input->data.size()<<" cloud_transform: "<<cloud_transform->size()<<std::endl;

    sensor_msgs::PointCloud2 output;

    if (cut_half_)
    {
        for (int i=(cloud_transform->size())/2; i<cloud_transform->size(); i++)
        {
            cloud_search->push_back(cloud_transform->points[i]);
        }
        pcl::toROSMsg(*cloud_search, output);
    }

    else
    {
        pcl::toROSMsg(*cloud_transform, output);
    }


    //pcl::toROSMsg(*cloud_transform, output);

    output.header = input->header;
    output.header.frame_id = frame_id_;
    output.header.stamp = ros::Time::now();

    // Publish the data.
    pub_pcl_.publish (output);
}
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pointcloud_readjust");
    
    pointcloud_transform::PCL_readjust pcl_readjust;
    // Spin
    ros::spin ();

    return 0;
}

