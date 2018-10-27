#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl_ros/filters/extract_indices.h>

ros::Publisher pub;

pcl::PassThrough<pcl::PointXYZI> pass;

bool filter_inside_, filter_outside_;
double filter_size_;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg (*input, *cloud_in);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);

    if (filter_outside_)
    {
        pass.setInputCloud (cloud_in);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (-filter_size_, filter_size_);
        pass.filter (*cloud_in);

        pass.setInputCloud (cloud_in);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (-filter_size_, filter_size_);
        //pass.setFilterLimitsNegative (true);
        pass.filter (*cloud_in);

    }
    else if (filter_inside_)
    {
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        for (int i = 0; i < cloud_in->size(); i++)
        {
            if (cloud_in->points[i].x<1.0 && cloud_in->points[i].x>-1.0)
            {
                if (cloud_in->points[i].y<1.0 && cloud_in->points[i].y>-1.0)
                {
                    inliers->indices.push_back(i);
                }

            }
        }
        extract.setInputCloud(cloud_in);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud_in);
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg (*cloud_in, output);
    output.header = input->header;

  // Publish the data.
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "passthrough");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  priv_nh.getParam("filter_inside", filter_inside_);
  priv_nh.getParam("filter_outside", filter_outside_);
  priv_nh.getParam("filter_size", filter_size_);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}

