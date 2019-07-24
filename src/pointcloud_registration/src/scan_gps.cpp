#include <pointcloud_registration/scan_gps.h>

namespace pointcloud_registration
{
    Scan_GPS::ScanGPS()
    {
        sub_top_ = nh_.subscribe <sensor_msgs::PointCloud2> ("cloud_top", 1, &ScanGPS::top_cb_);

        sub_gps_ = nh_.subscribe <geometry_msgs::PoseWithCovarianceStamped> ("gps_position", 1, &ScanGPS::gps_cb_);

        pub_after_matched_ = nh_.advertise<sensor_msgs::PointCloud2> ("after_matched", 1);
        pub_map_ = nh_.advertise<sensor_msgs::PointCloud2> ("gps_map", 1);

        cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
        map_.reset(new pcl::PointCloud<pcl::PointXYZI>);

        stored_gps_ = new std::vector<geometry_msgs::Pose>;
    }

    void Scan_GPS::top_cb_ (const sensor_msgs::PointCloud2ConstPtr& input)
    {
        //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg (*input, *cloud_);
        stored_cloud_ = *input;

    }

    void Scan_GPS::gps_cb_ (const geometry_msgs::PoseWithCovarianceStampedConstPtr& gps)
    {
        double cov_x = 0.0;
        double cov_y = 0.0;

        cov_x = gps->pose.covariance[0];
        cov_y = gps->pose.covariance[14];

        if (cov_x<0.1 && cov_y<0.1)
        {
            stored_gps_->push_back(gps->pose.pose);
        }
    }

    void Scan_GPS::processing()
    {

    }
}
