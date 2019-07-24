#include "ros/ros.h"

#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>

#include <pcl_ros/filters/passthrough.h>

class ICP_calibrate_lidar
{
public:
    ICP_calibrate_lidar();

private:
    void top_front_cb_ (const sensor_msgs::PointCloud2ConstPtr& input);
    void top_rear_cb_ (const sensor_msgs::PointCloud2ConstPtr& input);
    void bottom_front_cb_ (const sensor_msgs::PointCloud2ConstPtr& input);
    void initial_pose_cb_(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& posing);
    void map_cb_ (const sensor_msgs::PointCloud2ConstPtr& input);

    ros::NodeHandle nh_;
    ros::Subscriber sub_top_front_;
    ros::Subscriber sub_top_rear_;
    ros::Subscriber sub_bottom_front_;
    ros::Subscriber sub_map_;

    ros::Publisher pub_top_front_;
    ros::Publisher pub_top_rear_;
    ros::Publisher pub_bottom_front_;
    ros::Publisher pub_full_map_;
    ros::Publisher pub_portion_map_;

    bool start_;


    std::string map_location_;

    double topfront_roll_, topfront_pitch_, topfront_yaw_, topfront_x_, topfront_y_, topfront_z_;
    double toprear_roll_, toprear_pitch_, toprear_yaw_, toprear_x_, toprear_y_, toprear_z_;
    double bottom_roll_, bottom_pitch_, bottom_yaw_, bottom_x_, bottom_y_, bottom_z_;

    tf::Transform topfront_tf_, toprear_tf_, bottom_tf_, baselink_tf_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr topfront_pointcloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr toprear_pointcloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr bottomfront_pointcloud_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr full_map_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr portion_map_;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;

    geometry_msgs::PoseWithCovarianceStamped global_initialize_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "icp");

    ICP_calibrate_lidar icp_calibrate_lidar;

    ros::spin();
    return(0);
}

ICP_calibrate_lidar::ICP_calibrate_lidar()
{
    sub_top_front_ = nh_.subscribe <sensor_msgs::PointCloud2> ("topfront_pointcloud", 1, &ICP_calibrate_lidar::top_front_cb_, this);
    sub_top_rear_ = nh_.subscribe <sensor_msgs::PointCloud2> ("toprear_pointcloud", 1, &ICP_calibrate_lidar::top_rear_cb_, this);
    sub_bottom_front_ = nh_.subscribe <sensor_msgs::PointCloud2> ("bottomrear_pointcloud", 1, &ICP_calibrate_lidar::bottom_front_cb_, this);

    sub_map_ = nh_.subscribe <sensor_msgs::PointCloud2> ("full_map", 1, &ICP_calibrate_lidar::map_cb_, this);

    ros::NodeHandle priv_nh("~");
    priv_nh.getParam("topfront_roll", topfront_roll_);
    priv_nh.getParam("topfront_pitch", topfront_pitch_);
    priv_nh.getParam("topfront_yaw", topfront_yaw_);
    priv_nh.getParam("topfront_x", topfront_x_);
    priv_nh.getParam("topfront_y", topfront_y_);
    priv_nh.getParam("topfront_z", topfront_z_);
    priv_nh.getParam("toprear_roll", toprear_roll_);
    priv_nh.getParam("toprear_pitch", toprear_pitch_);
    priv_nh.getParam("toprear_yaw", toprear_yaw_);
    priv_nh.getParam("toprear_x", toprear_x_);
    priv_nh.getParam("toprear_y", toprear_y_);
    priv_nh.getParam("toprear_z", toprear_z_);
    priv_nh.getParam("bottom_roll", bottom_roll_);
    priv_nh.getParam("bottom_pitch", bottom_pitch_);
    priv_nh.getParam("bottom_yaw", bottom_yaw_);
    priv_nh.getParam("bottom_x", bottom_x_);
    priv_nh.getParam("bottom_y", bottom_y_);
    priv_nh.getParam("bottom_z", bottom_z_);

//    priv_nh.getParam("map_location", map_location_);

//    pcl::io::loadPCDFile(map_location_, *cloud_in);

    topfront_tf_.setOrigin(tf::Vector3(topfront_x_, topfront_y_, topfront_z_));
    tf::Quaternion rpy_to_qt;
    rpy_to_qt.setRPY(topfront_roll_, topfront_pitch_, topfront_yaw_);
    topfront_tf_.setRotation(rpy_to_qt);

    toprear_tf_.setOrigin(tf::Vector3(toprear_x_, toprear_y_, toprear_z_));
    rpy_to_qt.setRPY(toprear_roll_, toprear_pitch_, toprear_yaw_);
    toprear_tf_.setRotation(rpy_to_qt);

    bottom_tf_.setOrigin(tf::Vector3(bottom_x_, bottom_y_, bottom_z_));
    rpy_to_qt.setRPY(bottom_roll_, bottom_pitch_, bottom_yaw_);
    bottom_tf_.setRotation(rpy_to_qt);

    topfront_pointcloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    toprear_pointcloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    bottomfront_pointcloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    full_map_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    portion_map_.reset(new pcl::PointCloud<pcl::PointXYZI>);
}

void ICP_calibrate_lidar::initial_pose_cb_(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& posing)
{
    geometry_msgs::PoseWithCovarianceStamped initial;
    initial = *posing;
    tf::Quaternion initial_rotation;

    tf::quaternionMsgToTF(initial.pose.pose.orientation, initial_rotation);

    double ra,pa,ya;
    baselink_tf_.setOrigin(tf::Vector3(initial.pose.pose.position.x,initial.pose.pose.position.y,initial.pose.pose.position.z));
    baselink_tf_.setRotation(initial_rotation.normalize());
    tf::Matrix3x3(baselink_tf_.getRotation()).getRPY(ra, pa, ya);

    std::cout<<"I have added a new pose at X: "<<initial.pose.pose.position.x<<" Y "<<initial.pose.pose.position.y<<
               " z "<<initial.pose.pose.position.z<<std::endl;
    std::cout<<"with a heading of R: "<<ra<<" P "<<pa<<" Y "<<ya<<std::endl;

    start_ = true;

    if (full_map_->size()>0)
    {

    }


}

void ICP_calibrate_lidar::map_cb_(const sensor_msgs::PointCloud2ConstPtr &input)
{
    pcl::fromROSMsg (*input, *full_map_);


}

void ICP_calibrate_lidar::top_front_cb_(const sensor_msgs::PointCloud2ConstPtr &input)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg (*input, *cloud_in);

    pcl_ros::transformPointCloud (*cloud_in, *topfront_pointcloud_, baselink_tf_*topfront_tf_);

    sensor_msgs::PointCloud2 output_top;
    pcl::toROSMsg (*topfront_pointcloud_, output_top);
    output_top.header = input->header;
    output_top.header.frame_id = "map";

    pub_top_front_.publish(output_top);
}

void ICP_calibrate_lidar::top_rear_cb_(const sensor_msgs::PointCloud2ConstPtr &input)
{

}

void ICP_calibrate_lidar::bottom_front_cb_(const sensor_msgs::PointCloud2ConstPtr &input)
{

}
