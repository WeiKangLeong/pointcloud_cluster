#include "pointcloud_filter/pointcloud_visualize_normal.h"

namespace pointcloud_filter
{
    PCL_visualizenormal::PCL_visualizenormal()
    {
        view_point_x_ = 0.0;
        view_point_y_ = 0.0;
        view_point_z_ = 0.0;

        ros::NodeHandle priv_nh("~");
        priv_nh.getParam("view_point_x", view_point_x_);
        priv_nh.getParam("view_point_y", view_point_y_);
        priv_nh.getParam("view_point_z", view_point_z_);

        sub_cloud_ = nh_.subscribe <sensor_msgs::PointCloud2> ("cloud_in", 1, &PCL_visualizenormal::pointcloud_cb_, this);

        pub_normal_ = nh_.advertise <visualization_msgs::MarkerArray> ("normal_arrow", 1);
        pub_leftover_ = nh_.advertise<sensor_msgs::PointCloud2> ("suspect_cloud", 1);
    }

    void PCL_visualizenormal::pointcloud_cb_ (const sensor_msgs::PointCloud2ConstPtr& input)
    {
        ros::Time time_justnow = ros::Time::now();
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg (*input, *cloud_in);

        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
        ne.setInputCloud (cloud_in);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
        ne.setSearchMethod (tree);

        // Output datasets
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

        // Use all neighbors in a sphere of radius 3cm
        ne.setRadiusSearch (0.5);

        ne.setViewPoint (view_point_x_, view_point_y_, view_point_z_);

        // Compute the features
        ne.compute (*cloud_normals);

        for (int i=0; i<cloud_normals->size(); i++)
        {
            double roll = atan(cloud_normals->points[i].normal_z/cloud_normals->points[i].normal_y);
            double pitch = atan(cloud_normals->points[i].normal_z/cloud_normals->points[i].normal_x);
            if (roll>1.39 || pitch>1.39 || roll<-1.39 || pitch<-1.39)
            {
                visualization_msgs::Marker normal_arrow;
                normal_arrow.header = input->header;
                normal_arrow.ns = "normal";
                normal_arrow.id = i;
                normal_arrow.type = 0;
                normal_arrow.action = 0;

                normal_arrow.scale.x = 0.01;
                normal_arrow.scale.y = 0.0;
                normal_arrow.scale.z = 0.0;

                normal_arrow.lifetime = ros::Duration (0.0);

                normal_arrow.color.r = 0.0;
                normal_arrow.color.g = 1.0;
                normal_arrow.color.b = 0.0;
                normal_arrow.color.a = 1.0;

                if (!(std::isnan(cloud_normals->points[i].normal_x))&&!(std::isnan(cloud_normals->points[i].normal_y))&&!(std::isnan(cloud_normals->points[i].normal_z)))
                {
                    geometry_msgs::Point arrow_point;
                    arrow_point.x = cloud_in->points[i].x;
                    arrow_point.y = cloud_in->points[i].y;
                    arrow_point.z = cloud_in->points[i].z;
                    normal_arrow.points.push_back(arrow_point);

                    arrow_point.x = cloud_in->points[i].x+cloud_normals->points[i].normal_x;
                    arrow_point.y = cloud_in->points[i].y+cloud_normals->points[i].normal_y;
                    arrow_point.z = cloud_in->points[i].z+cloud_normals->points[i].normal_z;
                    normal_arrow.points.push_back(arrow_point);

                    marker_arrow_.markers.push_back(normal_arrow);
                }

            }
//            else
//            {
//                std::cout<<i<<" : "<<cloud_normals->points[i].normal_x<<" "<<cloud_normals->points[i].normal_y<<" "<<cloud_normals->points[i].normal_z<<" || "<<roll<<" "<<pitch<<std::endl;
//            }

        }

        pub_normal_.publish(marker_arrow_);

        marker_arrow_.markers.clear();

        ros::Time time_now = ros::Time::now();
        double time_used = time_now.toSec() - time_justnow.toSec();
        //std::cout<<"The time used: "<<time_used<<std::endl;
    }
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pointcloud_visualizenormal");

    pointcloud_filter::PCL_visualizenormal pcl_visualizenormal;
    // Spin
    ros::spin ();

    return 0;
}
