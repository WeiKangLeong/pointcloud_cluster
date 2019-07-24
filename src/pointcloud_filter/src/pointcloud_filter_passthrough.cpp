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

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_combine(new pcl::PointCloud<pcl::PointXYZI>);

bool filter_inside_, filter_outside_, filter_height_;
bool filter_combine_;
int combine_flag_;
double filter_size_;
double filter_upper_z_;
double filter_lower_z_;
double filter_upper_x_;
double filter_lower_x_;
double filter_upper_y_;
double filter_lower_y_;

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
        pass.setFilterLimits (filter_lower_x_, filter_upper_x_);
        pass.filter (*cloud_in);

        pass.setInputCloud (cloud_in);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (filter_lower_y_, filter_upper_y_);
        //pass.setFilterLimitsNegative (true);
        pass.filter (*cloud_in);

    }
    else if (filter_inside_)
    {
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        for (int i = 0; i < cloud_in->size(); i++)
        {
            if (cloud_in->points[i].x<filter_upper_x_ && cloud_in->points[i].x>filter_lower_x_)
            {
                if (cloud_in->points[i].y<filter_upper_y_ && cloud_in->points[i].y>filter_lower_y_)
                {
                    if (cloud_in->points[i].z<filter_upper_z_ && cloud_in->points[i].z>filter_lower_z_)
                    {
                        inliers->indices.push_back(i);
                    }
                }

            }
        }
        extract.setInputCloud(cloud_in);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud_in);
    }

    if (filter_height_)
    {
        pass.setInputCloud (cloud_in);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (filter_lower_z_, filter_upper_z_);
        pass.filter (*cloud_in);
    }

    sensor_msgs::PointCloud2 output;

    if (filter_combine_)
    {
        *cloud_combine += *cloud_in;
        combine_flag_++;
        if (combine_flag_==2)
        {
            pcl::toROSMsg (*cloud_combine, output);
            output.header = input->header;

            // Publish the data.
            pub.publish (output);
            combine_flag_ = 0;
            cloud_combine.reset(new pcl::PointCloud<pcl::PointXYZI>);
        }
    }
    else
    {
        pcl::toROSMsg (*cloud_in, output);
        output.header = input->header;

        // Publish the data.
        pub.publish (output);
    }
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "passthrough");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  filter_inside_ = false;
  filter_outside_ = false;
  filter_height_ = false;
  filter_combine_ = false;
  filter_size_ = 0.0;
  filter_upper_z_ = 0.0;
  filter_lower_z_ = 0.0;
  combine_flag_ = 0;

  priv_nh.getParam("filter_inside", filter_inside_);
  priv_nh.getParam("filter_outside", filter_outside_);
  priv_nh.getParam("filter_height", filter_height_);
  priv_nh.getParam("filter_combine", filter_combine_);
  priv_nh.getParam("filter_size", filter_size_);
  priv_nh.getParam("filter_upper_z", filter_upper_z_);
  priv_nh.getParam("filter_lower_z", filter_lower_z_);
  priv_nh.getParam("filter_upper_x", filter_upper_x_);
  priv_nh.getParam("filter_lower_x", filter_lower_x_);
  priv_nh.getParam("filter_upper_y", filter_upper_y_);
  priv_nh.getParam("filter_lower_y", filter_lower_y_);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}

