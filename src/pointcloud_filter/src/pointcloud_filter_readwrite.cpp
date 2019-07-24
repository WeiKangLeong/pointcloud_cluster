#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/ply_io.h>

#include <iostream>

ros::Publisher pub;

pcl::PointCloud<pcl::PointXYZI>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZI>);

pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;

//int k;

//void cloudtopoint(pcl::PointXYZ *pi, pcl::PointXYZ *po)
//{
//    po->x = pi->x;
//    po->y = pi->y;
//    po->z = pi->z;
//}

//void store_plane_in_file()
//{
//    int plane_number;
//    plane_number=0;
//    bool more_plane=true;
//    std::string y="y";
//    pcl::PointCloud<pcl::PointXYZI>::Ptr plane_cloud (new pcl::PointCloud<pcl::PointXYZI>);
//    while (more_plane)
//    {
//        int point_number;
//        double pt;
//        int answer;
//        pcl::PointXYZI one_of_the_point;
//        one_of_the_point.intensity = plane_number;
//        for (int i=0; i<4; i++)
//        {
//            std::cout<<"Plane number "<<plane_number<<", point "<<i<<": "<<std::endl;
//            std::cout<<"x: ";
//            std::cin>>pt;
//            one_of_the_point.x=pt;
//            std::cout<<"y: ";
//            std::cin>>pt;
//            one_of_the_point.y=pt;
//            std::cout<<"z: ";
//            std::cin>>pt;
//            one_of_the_point.z=pt;
//            plane_cloud->push_back(one_of_the_point);
//        }
//        std::cout<<"more plane? (1/0): ";
//        std::cin>>answer;
//        std::cout<<answer<<std::endl;
//        if(answer==1)
//        {
//            more_plane = true;
//            plane_number++;
//            std::cout<<"add one more plane"<<std::endl;
//        }
//        else
//        {
//            more_plane = false;
//            plane_number=0;
//        }
//    }
//    pcl::io::savePCDFileASCII("/home/smaug/Desktop/Raw Data/enterprise/corridor.pcd",*plane_cloud);
//}

void save_cb (std_msgs::Bool save_button)
{
    if (save_button.data)
    {
        pcl::io::savePCDFileBinaryCompressed("/home/locmodv2/rear_map.pcd", *global_cloud);
        std::cout<<"saved map"<<std::endl;
    }
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg (*input, *cloud);

    *global_cloud += *cloud;

    voxel_filter.setLeafSize (0.1, 0.1, 0.1);
    voxel_filter.setInputCloud (global_cloud);
    voxel_filter.filter (*global_cloud);

    std::cout<<"cloud size: "<<global_cloud->size()<<std::endl;

//    sensor_msgs::PointCloud2 output;
//    pcl::toROSMsg (*global_cloud, output);
//    output.header = input->header;

//    pub.publish(output);
    //pcl::io::savePCDFileBinaryCompressed("/home/locmodv2/xy.pcd", *cloud);

    //ROS_INFO("cloud received.");

}

int
main (int argc, char** argv)
{
  // Initialize ROS

    std::cout<<"before run"<<std::endl;
  ros::init (argc, argv, "read_write");
  ros::NodeHandle nh;

  std::cout<<"1"<<std::endl;
  //ros::Duration(1.0).sleep();
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, cloud_cb);
  ros::Subscriber sub_save = nh.subscribe<std_msgs::Bool> ("save", 1, save_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/read_cloud", 1);

  std::cout<<"start"<<std::endl;

  ros::spin ();
}

