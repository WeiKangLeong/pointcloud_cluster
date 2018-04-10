#include <ros/ros.h>

#include <std_msgs/Float32MultiArray.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/filters/passthrough.h>

#include <pcl/console/time.h>

#include "../include/dbscan.h"

DBSCAN *db;

ros::Publisher pub;

tf::Transform to_two_dimension, to_camera_frame;
double fx, fy, tx, ty;
std_msgs::Header pc_header;
std::vector<double>* box_value;
double box_time;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PassThrough<pcl::PointXYZ> pass;

void array_cb(std_msgs::Float32MultiArray box_)
{
    box_time = ros::Time::now().toSec();
    box_value = new std::vector<double>;
    box_value->resize(box_.data.size());
    for (int i=0; i<box_.data.size(); i++)
    {
        box_value->push_back(box_.data[i]);
    }
}

std::vector<std::vector<int>*>* check_box(pcl::PointCloud<pcl::PointXYZI>::Ptr image_pt)
{
    double time_diff = ros::Time::now().toSec() - box_time;
    std::cout<<"Box time and point cloud time difference: "<<time_diff<<std::endl;
    std::vector<std::vector<int>*>* pt_in_box = new std::vector<std::vector<int>*>;
    pt_in_box->resize(box_value->size()/4);
    for (int b=0; b<box_value->size()/4; b++)
    {
        double min_x, min_y, max_x, max_y;
        min_x = box_value->at(b*4);
        min_y = box_value->at(b*4+1);
        max_x = box_value->at(b*4+2);
        max_y = box_value->at(b*4+3);

        for (int i=0; i<image_pt->size(); i++)
        {
            if (min_x<image_pt->points[i].x<max_x)
            {
                if (min_y<image_pt->points[i].y<max_y)
                {
                    pt_in_box->at(b)->push_back(image_pt->points[i].intensity);
                }
            }
        }
    }
    return pt_in_box;

}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::console::TicToc time;
    time.tic();
    pc_header = input->header;
    // Create a container for the data.
    pcl::fromROSMsg (*input, *cloud_in);

//    pass.setInputCloud (cloud_in);
//    pass.setFilterFieldName ("y");
//    pass.setFilterLimits (-5.0, 5.0);
//    pass.filter (*cloud_in);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_on_image(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_on_image_trimmed(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_from_camera(new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<int>* pt_in_image = new std::vector<int>;
    //std::vector<int>* pt_in_box = new std::vector<int>;

    pcl_ros::transformPointCloud (*cloud_in, *cloud_from_camera, to_camera_frame);

    pass.setInputCloud (cloud_from_camera);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0.0, 100.0);
    pass.filter (*cloud_from_camera);

    for (int i=0; i<cloud_from_camera->size(); i++)
    {
        pcl::PointXYZI pt_img;
        pt_img.x = tx+fx*cloud_from_camera->points[i].x/cloud_from_camera->points[i].x;
        pt_img.y = ty+fy*cloud_from_camera->points[i].y/cloud_from_camera->points[i].x;
        pt_img.z = 0.0;
        pt_img.intensity = i;
        cloud_on_image->push_back(pt_img);
    }

    for (int i=0; i<cloud_on_image->size(); i++)
    {
        double c_x, c_y;
        c_x = cloud_on_image->points[i].x;
        c_y = cloud_on_image->points[i].y;
        if (0.0<c_x<2048.0)
        {
            if (0.0<c_y<1536.0)
            {
                pcl::PointXYZI c_pt = cloud_on_image->points[i];
                cloud_on_image_trimmed->push_back(c_pt);
                pt_in_image->push_back(i);
            }
        }
    }

    std::cout<<"cloud: "<<cloud_in<<" cloud_image: "<<cloud_on_image->size()<<" cloud_trimmed: "<<cloud_on_image_trimmed<<std::endl;
    sensor_msgs::PointCloud2 output;

    pcl::toROSMsg (*cloud_on_image_trimmed, output);
    output.header = pc_header;

    std::vector<std::vector<int>* >* box_divider = new std::vector<std::vector<int>* >;
    box_divider = check_box(cloud_on_image_trimmed);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>* dbscan_output = new std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>* cluster_objects = new std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>;
    std::vector<int>* object_index = new std::vector<int>;
    std::vector<int>* map_index = new std::vector<int>;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_box_i (new pcl::PointCloud<pcl::PointXYZI>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_box = new pcl::PointCloud<pcl::PointXYZ>;

    for (int i=0; i<box_divider->size(); i++)
    {
        for (int j=0; j<box_divider->at(i)->size(); j++)
        {
            pcl::PointXYZI point_new;
            point_new.x = cloud_in->points[box_divider->at(i)->at(j)].x;
            point_new.y = cloud_in->points[box_divider->at(i)->at(j)].y;
            point_new.z = cloud_in->points[box_divider->at(i)->at(j)].z;
            point_new.intensity = box_divider->at(i)->at(j);
            cloud_in_box_i->push_back(point_new);
        }
        //pcl::copyPointCloud(cloud_in_box_i, cloud_in_box);
        dbscan_output = db->dbscan_cluster_intensity(cloud_in_box_i);
        if (dbscan_output->size()>0)
        {
            for (int k=0; k<dbscan_output->size(); k++)
            {
                cluster_objects->push_back(dbscan_output->at(k));
                for (int l=0; l<dbscan_output->at(k)->size(); l++)
                {
                    object_index->push_back(dbscan_output->at(k)->points[l].intensity);
                }
            }
        }
        cloud_in_box_i->clear();
        dbscan_output->clear();
    }

    // Publish the data.
    pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "fit_image_with_pointcloud");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
  ros::Subscriber sub_box = nh.subscribe ("array", 1, array_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);


  //tf::Matrix3x3 rotate_matrix(0.03222100, -0.020143000, 0.99927800, -0.9992260, -0.02320500, 0.03175100, 0.0225490, -0.9995280, -0.0208750);
  tf::Matrix3x3 rotate_matrix(0.03222100, -0.9992260, 0.0225490, -0.020143000, -0.02320500, -0.9995280, 0.99927800,   0.03175100, -0.0208750);
  tf::Vector3 translate(0.018866, -0.4866910, -0.73494900);
  tf::Transform pc_to_image_tf(rotate_matrix, translate);
  to_two_dimension = pc_to_image_tf;
  to_camera_frame.setOrigin(tf::Vector3(-0.734949, 0.018866, -0.486691));
  to_camera_frame.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

  fx = 1.14294066717895;
  fy = 1.14242809061681;
  tx = 966.236430746141;
  ty = 793.910441558118;

  // Spin
  ros::spin ();
}

