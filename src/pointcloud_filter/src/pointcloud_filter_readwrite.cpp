#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/ply_io.h>

#include <iostream>

ros::Publisher pub;

pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZ>);

int k;

void cloudtopoint(pcl::PointXYZ *pi, pcl::PointXYZ *po)
{
    po->x = pi->x;
    po->y = pi->y;
    po->z = pi->z;
}

void store_plane_in_file()
{
    int plane_number;
    plane_number=0;
    bool more_plane=true;
    std::string y="y";
    pcl::PointCloud<pcl::PointXYZI>::Ptr plane_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    while (more_plane)
    {
        int point_number;
        double pt;
        int answer;
        pcl::PointXYZI one_of_the_point;
        one_of_the_point.intensity = plane_number;
        for (int i=0; i<4; i++)
        {
            std::cout<<"Plane number "<<plane_number<<", point "<<i<<": "<<std::endl;
            std::cout<<"x: ";
            std::cin>>pt;
            one_of_the_point.x=pt;
            std::cout<<"y: ";
            std::cin>>pt;
            one_of_the_point.y=pt;
            std::cout<<"z: ";
            std::cin>>pt;
            one_of_the_point.z=pt;
            plane_cloud->push_back(one_of_the_point);
        }
        std::cout<<"more plane? (1/0): ";
        std::cin>>answer;
        std::cout<<answer<<std::endl;
        if(answer==1)
        {
            more_plane = true;
            plane_number++;
            std::cout<<"add one more plane"<<std::endl;
        }
        else
        {
            more_plane = false;
            plane_number=0;
        }
    }
    pcl::io::savePCDFileASCII("/home/smaug/Desktop/Raw Data/enterprise/corridor.pcd",*plane_cloud);
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*input, *cloud);

  pcl::io::savePCDFileBinaryCompressed("/home/smaug/loam_cleantech_noground.pcd", *cloud);

  ROS_INFO("cloud saved.");

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ;
  ros::init (argc, argv, "read_write");
  ros::NodeHandle nh;

  ros::Duration(1.0).sleep();
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/read_cloud", 1);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_largemap(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_minimap(new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_blah(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile("/home/smaug/loam_cleantech_filtered.pcd", *cloud_largemap);
  std::cout<<"map loaded..."<<std::endl;
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg (*cloud_largemap, output);
  output.header.frame_id = "wheelchair/map";
  pub.publish(output);
//  pcl::PassThrough<pcl::PointXYZ> pass;
//  pass.setInputCloud (cloud_largemap);
//  pass.setFilterFieldName ("x");
//  pass.setFilterLimits (-30.0, 30.0);
//  //pass.setFilterLimitsNegative (true);
//  pass.filter (*cloud_minimap);

//  pass.setInputCloud (cloud_minimap);
//  pass.setFilterFieldName ("y");
//  pass.setFilterLimits (-30.0, 30.0);
//  pass.filter (*cloud_minimap);

//store_plane_in_file();

//   pcl::io::savePCDFileBinaryCompressed("/home/smaug/Desktop/Raw Data/30x30map.pcd", *cloud_minimap);
/*    pcl::PLYWriter pply;
    pcl::PLYReader ppry;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_coloured(new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Vector4f origin = Eigen::Vector4f::Zero();
    Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
//    int precision=8;
//    bool use_camera=false;

    pcl::io::loadPCDFile("/home/smaug/Desktop/Raw Data/utown_3d/loam_utown_clean.pcd",*cloud_in);
    std::cout<<"loaded map ( "<<cloud_in->size()<<" ) in pcd"<<std::endl;

*/
/*  std::cout<<"map loaded"<<std::endl;
    pcl::VoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setInputCloud (cloud_largemap);
    approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
    approximate_voxel_filter.filter (*cloud_filtered);
    pcl::io::savePCDFileBinaryCompressed("/home/smaug/loam_cleantech_filtered.pcd",*cloud_filtered);
    std::cout<<"map filtered"<<std::endl;

//    pply.write("/home/smaug/catkin_ws/src/simmobility/subsegment.ply",cloud_in,origin,orientation,false,true);
//    std::cout<<"saved map in ply"<<std::endl;
//    //cloud_in.clear();
    pcl::io::savePCDFileBinaryCompressed("/home/smaug/Desktop/Raw Data/utown_3d/utown_clean_zerotwo.pcd",*cloud_filtered);
    std::cout<<"loaded "<<cloud_filtered->size()<<" points in pcd"<<std::endl;
*/
//    std::vector<int>* intensity_value;
//    intensity_value = new std::vector<int>;

//    for (int i=0; i<cloud_xyzi->size(); i++)
//    {
//        intensity_value->push_back(cloud_xyzi->points[i].intensity);
//    }

//    float min_intensity = *std::min_element(intensity_value->begin(),intensity_value->end());
//    float max_intensity = *std::max_element(intensity_value->begin(),intensity_value->end());
//    float diff_intensity = max_intensity- min_intensity;

//    for (int i=0; i<cloud_xyzi->size(); i++)
//    {
//        float value = 1.0 - cloud_xyzi->points[i].intensity/255;
//        double color[3];
//        float hsv = value * 5.0f + 1.0f;
//        int ihsv = floor(hsv);
//        float f = hsv - ihsv;
//        if ( !(ihsv&1) ) f = 1 - f; // if i is even
//        float n = 1 - f;
//        if      (ihsv <= 1) color[0] = n, color[1] = 0, color[2] = 1;
//        else if (ihsv == 2) color[0] = 0, color[1] = n, color[2] = 1;
//        else if (ihsv == 3) color[0] = 0, color[1] = 1, color[2] = n;
//        else if (ihsv == 4) color[0] = n, color[1] = 1, color[2] = 0;
//        else if (ihsv >= 5) color[0] = 1, color[1] = n, color[2] = 0;
//        pcl::PointXYZRGB point;
//        point.x=cloud_xyzi->points[i].x;
//        point.y=cloud_xyzi->points[i].y;
//        point.z=cloud_xyzi->points[i].z;
//        point.r=int(color[0] * 255.0);
//        point.g=int(color[1] * 255.0);
//        point.b=int(color[2] * 255.0);
//        cloud_coloured->push_back(point);
//    }

//    pply.write("/home/smaug/Desktop/Raw Data/utown_3d/loam_utown.ply", *cloud_coloured,false,true);
//    std::cout<<"saved map in ply"<<std::endl;
//    cloud_in.reset(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::io::loadPCDFile("/home/smaug/Desktop/Raw Data/velodyne_3p/20170407/best_right_velodyne.pcd",*cloud_in);
//    std::cout<<"loaded map in pcd"<<std::endl;
//    pply.write("/home/smaug/Desktop/Raw Data/velodyne_3p/20170407/best_right_velodyne.ply",cloud_in,origin,orientation,false,true);
//    std::cout<<"saved map in ply"<<std::endl;
    //cloud_in.clear();

//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
//    ppry.read ("/home/smaug/Desktop/Raw Data/utown_3d/loam_utown.ply", *cloud_out, 0);
//    std::cout<<"loaded "<<cloud_out->size()<<" points map in ply"<<std::endl;
//    pcl::io::savePCDFileBinaryCompressed("/home/smaug/Desktop/Raw Data/utown_3d/loam_utown_clean.pcd", *cloud_out);
//    std::cout<<"saved map in pcd"<<std::endl;
//    ppry.read ("/home/smaug/Desktop/Raw Data/velodyne_3p/20170407/best_left_velodyne_3.ply", *cloud_out, 0);
//    std::cout<<"loaded map in ply"<<std::endl;
//    std::cout<<"Voxel grid point cloud size: "<<cloud_filtered->size()<<std::endl;
//    pcl::io::savePCDFileASCII("/home/smaug/Desktop/Raw Data/no_ground/map_normal_pca_weikang_filtered.pcd", *cloud_filtered);
//    std::cout<<"saved map in pcd"<<std::endl;
//    ppry.read ("/home/smaug/Desktop/Raw Data/velodyne_3p/20170407/best_right_velodyne_3.ply", *cloud_out, 0);
//    std::cout<<"loaded map in ply"<<std::endl;
//    pcl::io::savePCDFileBinaryCompressed("/home/smaug/Desktop/Raw Data/velodyne_3p/20170407/best_right_velodyne_3.pcd", *cloud_out);
//    std::cout<<"saved map in pcd"<<std::endl;

    //cloud_coloured.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    //cloud_xyzi.reset(new pcl::PointCloud<pcl::PointXYZI>);
  // Spin
  ros::spin ();
}

