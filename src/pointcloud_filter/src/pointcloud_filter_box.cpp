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


ros::Publisher pub_floor, pub_static, pub_move;

tf::TransformListener *tf_listener_;

Eigen::Vector3d intrinsic_1_, intrinsic_2_, intrinsic_3_;

std::vector<std::vector<int>* >* floor_in_grid_;

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_comeout(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_left_over(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_floor(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_floor_dense(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_on_image(new pcl::PointCloud<pcl::PointXYZRGB>);

pcl::PassThrough<pcl::PointXYZRGB> pass;

/*void box_cb(const visualization_msgs::MarkerArrayConstPtr& markers_array)
{

    int markers_size = markers_array->markers.size() - 1;
    std::cout<<"Marker array size: "<<markers_size<<std::endl;
//    if (markers_array->at(markers_size).points.size()>0)
//    {
//        markers_array->at(markers_size).
//    }
    for (int i=0; i<markers_size; i++)
    {
        std::cout<<"point size: "<<markers_array->markers.at(i).points.size()<<std::endl;
    }

}*/

void pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::console::TicToc time;
    time.tic ();

    pcl::fromROSMsg (*input, *cloud_color);

    int cloud_size = cloud_color->size();

//    x:0.85
//    y:-+0.7
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    //cloud_comeout = cloud_color;
    pass.setInputCloud (cloud_color);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0.85, 200.0);
    pass.filter (*cloud_color);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-25.0, 25.0);
    pass.filter (*cloud_color);

    sensor_msgs::PointCloud2 output3;
    pcl::toROSMsg(*cloud_color, output3);
    output3.header = input->header;
    pub_static.publish(output3);

    std::cout<<"Pointcloud size: "<<cloud_size<<" to: "<<cloud_color->size()<<std::endl;
    int prev_group = cloud_color->points[0].r*1000000+cloud_color->points[0].g*1000+cloud_color->points[0].b;
    pcl::PointXYZI point_left;
    pcl::PointXYZRGB point_project;
    int count=0;

    Eigen::Vector3d point_vector, after_project;

    for (int i=0; i<cloud_size; i++)
    {
        int group = cloud_color->points[i].r*1000000+cloud_color->points[i].g*1000+cloud_color->points[i].b;
        //std::cout<<"group: "<<int(cloud_color->points[i].r*1000000+cloud_color->points[i].g*1000+cloud_color->points[i].b)<<std::endl;
        point_left.x=-cloud_color->points[i].y;
        point_left.y=cloud_color->points[i].z;
        point_left.z=cloud_color->points[i].x;

        Eigen::Vector3d point_vector(point_left.x, point_left.y, point_left.z);
        //after_project = point_vector.cross(intrinsic_);

        if (group==prev_group)
        {


        }
        else
        {
            count++;
            prev_group=group;

        }
        point_left.intensity=count;
        cloud_left_over->push_back(point_left);
        point_project.z = point_vector.dot(intrinsic_3_);
        point_project.x = point_vector.dot(intrinsic_1_)/point_project.z;
        point_project.y = point_vector.dot(intrinsic_2_)/point_project.z;
        point_project.z = point_project.z/point_project.z;




        point_project.r = cloud_color->points[i].r;
        point_project.g = cloud_color->points[i].g;
        point_project.b = cloud_color->points[i].b;
        cloud_on_image->push_back(point_project);
    }

    std::cout<<"Total cluster: "<<count<<std::endl;
    sensor_msgs::PointCloud2 output2;
    pcl::toROSMsg(*cloud_on_image, output2);
    output2.header = input->header;
    pub_move.publish(output2);

    cloud_on_image->clear();
    cloud_left_over->clear();

    std::cout<<"time taken: "<<time.toc()<<std::endl;


}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pointcloud_filter_box");
    ros::NodeHandle nh;

    tf_listener_ = new tf::TransformListener();

    //ros::Subscriber sub_box = nh.subscribe<visualization_msgs::MarkerArray> ("/percept_info", 1, box_cb);
    ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2> ("/cluster", 1, pointcloud_cb);

    pub_move = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_move", 1);
    pub_floor = nh.advertise<sensor_msgs::PointCloud2> ("/floor_points", 1);
    pub_static = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_out", 1);
    //pub_cluster = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_left", 1);

    Eigen::Vector3d temp_intr1(484.538, 0.0, 371.507);
    Eigen::Vector3d temp_intr2(0.0, 484.798, 184.8);
    Eigen::Vector3d temp_intr3(0.0, 0.0, 1.0);
    //intrinsic_ << 484.538, 0.0, 371.507, 0.0, 484.798, 184.8, 0.0, 0.0, 1.0;
    intrinsic_1_ = temp_intr1;
    intrinsic_2_ = temp_intr2;
    intrinsic_3_ = temp_intr3;


    ros::spin ();
}
