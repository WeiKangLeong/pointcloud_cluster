#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include <laser_geometry/laser_geometry.h>

#include <tf/transform_listener.h>

// For pcl format
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/common.h>

class GridMapRegister
{
  public:
    GridMapRegister();

  private:


    ros::Subscriber input_pointcloud_;
    ros::Subscriber input_laser_;
    ros::Subscriber input_map_;

    ros::Publisher map_pub_;

    tf::TransformListener* tfl_;

    tf::Transform latest_tf_;
    bool tf_broadcast_;
    bool latest_tf_valid_;

    // Callbacks
    void pointcloud_cb_ (const sensor_msgs::PointCloud2ConstPtr& input);
    void laser_cb_ (const sensor_msgs::LaserScanConstPtr& laser);
    void map_cb_ (const nav_msgs::OccupancyGridConstPtr& input);

    void calculate_grid_(double x1, double y1, double x2, double y2);
    void point_obstacle_(double a, double b);
    void point_free_(double c, double d);

    //parameter for what odom to use
    std::string odom_frame_id_;

    //paramater to store latest odom pose
    tf::Stamped<tf::Pose> latest_odom_pose_;

    tf::Transform difference;

    tf::Vector3 difference_pose;

    double resolution_;

    //parameter for what base to use
    std::string base_frame_id_;
    std::string global_frame_id_;

    std::string map_location;

    //time for tolerance on the published transform,
    //basically defines how long a map->odom transform is good for
    ros::Duration transform_tolerance_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    nav_msgs::OccupancyGrid map_msg_;
    nav_msgs::OccupancyGrid store_map_;

    laser_geometry::LaserProjection projector_;

};


int
main(int argc, char** argv)
{
    ros::init(argc, argv, "gridmap_register");

    GridMapRegister gridmap;

    ros::spin();

    return(0);
}

GridMapRegister::GridMapRegister()
{
  // Grab params off the param server


  tfl_ = new tf::TransformListener();

  input_pointcloud_ = nh_.subscribe <sensor_msgs::PointCloud2> ("input", 1, &GridMapRegister::pointcloud_cb_, this);
  input_laser_ = nh_.subscribe <sensor_msgs::LaserScan> ("laser_input", 1, &GridMapRegister::laser_cb_, this);
  input_map_ = nh_.subscribe <nav_msgs::OccupancyGrid> ("map", 1, &GridMapRegister::map_cb_, this);

  map_pub_ = nh_.advertise <nav_msgs::OccupancyGrid> ("grid_pcl", 1);

  ros::NodeHandle priv_nh("~");

  ROS_INFO("start");

  priv_nh.getParam("map_location", map_location);
  priv_nh.getParam("base_frame_id", base_frame_id_);
  priv_nh.getParam("global_frame_id", global_frame_id_);
  priv_nh.getParam("odom_frame_id", odom_frame_id_);
  priv_nh.getParam("resolution", resolution_);


  //map_msg_.info.resolution = resolution_;
  //map_msg_.header.frame_id = base_frame_id_;




}

void GridMapRegister::map_cb_(const nav_msgs::OccupancyGridConstPtr &input)
{
    map_msg_ = *input;
    resolution_ = map_msg_.info.resolution;
    std::cout<<"received "<<map_msg_.info.width<<" x "<<map_msg_.info.height<<std::endl;
}

void GridMapRegister::laser_cb_ (const sensor_msgs::LaserScanConstPtr& input)
{
    tf::StampedTransform latest_odom_transform, baselink_transform;
    try{
        tfl_->lookupTransform(global_frame_id_, base_frame_id_,
                                      ros::Time(0), latest_odom_transform);
    }catch (tf::TransformException &ex) {
        ROS_ERROR_STREAM("grid_registration: Looking Transform Failed");
        return;
    }
    try{
        tfl_->lookupTransform(base_frame_id_, "iMiev/front_bottom_lidar",
                                      ros::Time(0), baselink_transform);
    }catch (tf::TransformException &ex) {
        ROS_ERROR_STREAM("grid_registration: Looking Transform Failed");
        return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tf (new pcl::PointCloud<pcl::PointXYZI>);
    sensor_msgs::PointCloud2 laser_cloud;
    projector_.projectLaser(*input, laser_cloud);
    pcl::fromROSMsg (laser_cloud, *cloud_in);

//    pcl::PointXYZI min_point, max_point;
//    pcl::getMinMax3D (*cloud_in, min_point, max_point);

//    tf::Transform pc_tf;
//    pc_tf.setOrigin(tf::Vector3(-min_point.x, -min_point.y, 0.0));
//    pc_tf.setRotation(tf::Quaternion(0.0,0.0,0.0,1.0));
//    pcl_ros::transformPointCloud (*cloud_in, *cloud_tf, pc_tf);
    pcl_ros::transformPointCloud (*cloud_in, *cloud_in, baselink_transform);
    pcl_ros::transformPointCloud (*cloud_in, *cloud_tf, latest_odom_transform);

    tf::Vector3 latest_origin;
    latest_origin = latest_odom_transform.getOrigin();

//    double range_x = max_point.x - min_point.x;
//    double range_y = max_point.y - min_point.y;

//    map_msg_.data.clear();
//    map_msg_.info.width = range_x/resolution_;
//    map_msg_.info.height = range_y/resolution_;
//    map_msg_.header.stamp = ros::Time::now();

//    for (int l=0; l<map_msg_.info.width*map_msg_.info.height; l++)
//    {
//        map_msg_.data.push_back(-1);
//    }

    for (int k=0; k<cloud_tf->size(); k++)
    {
        double x_pt, y_pt;
        x_pt = cloud_tf->points[k].x;
        y_pt = cloud_tf->points[k].y;

        calculate_grid_(double(latest_origin.x()), double(latest_origin.y()), x_pt, y_pt);

        int map_place = int(y_pt/resolution_)*map_msg_.info.width+int(x_pt/resolution_);

//        if (map_place>map_msg_.data.size())
//        {
//            std::cout<<"pixel number: "<<map_place<<" map size: "<<map_msg_.data.size()<<std::endl;
//        }

//        else
//        {
//            map_msg_.data[map_place] = 100;
//        }


    }

    map_pub_.publish(map_msg_);
}

void GridMapRegister::pointcloud_cb_ (const sensor_msgs::PointCloud2ConstPtr& input)
{
    tf::StampedTransform latest_odom_transform;
    try{
        tfl_->lookupTransform(global_frame_id_, base_frame_id_,
                                      ros::Time(0), latest_odom_transform);
    }catch (tf::TransformException &ex) {
        ROS_ERROR_STREAM("grid_registration: Looking Transform Failed");
        return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tf (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg (*input, *cloud_in);

//    pcl::PointXYZI min_point, max_point;
//    pcl::getMinMax3D (*cloud_in, min_point, max_point);

//    tf::Transform pc_tf;
//    pc_tf.setOrigin(tf::Vector3(-min_point.x, -min_point.y, 0.0));
//    pc_tf.setRotation(tf::Quaternion(0.0,0.0,0.0,1.0));
//    pcl_ros::transformPointCloud (*cloud_in, *cloud_tf, pc_tf);
    pcl_ros::transformPointCloud (*cloud_in, *cloud_tf, latest_odom_transform);

    tf::Vector3 latest_origin;
    latest_origin = latest_odom_transform.getOrigin();

//    double range_x = max_point.x - min_point.x;
//    double range_y = max_point.y - min_point.y;

//    map_msg_.data.clear();
//    map_msg_.info.width = range_x/resolution_;
//    map_msg_.info.height = range_y/resolution_;
//    map_msg_.header.stamp = ros::Time::now();

//    for (int l=0; l<map_msg_.info.width*map_msg_.info.height; l++)
//    {
//        map_msg_.data.push_back(-1);
//    }

    point_free_(double(latest_origin.y()), double (latest_origin.x()));

    for (int k=0; k<cloud_tf->size(); k++)
    {
        double x_pt, y_pt;
        x_pt = cloud_tf->points[k].x;
        y_pt = cloud_tf->points[k].y;

        calculate_grid_(double(latest_origin.x()), double(latest_origin.y()), x_pt, y_pt);

        int map_place = int(y_pt/resolution_)*map_msg_.info.width+int(x_pt/resolution_);



//        if (map_place>map_msg_.data.size())
//        {
//            std::cout<<"pixel number: "<<map_place<<" map size: "<<map_msg_.data.size()<<std::endl;
//        }

//        else
//        {
//            map_msg_.data[map_place] = 100;
//        }


    }

    map_pub_.publish(map_msg_);
}

void GridMapRegister::calculate_grid_(double x1, double y1, double x2, double y2)
{
    //double x1=0.0;
    //double y1=0.0;
    double ystep, xstep;
    double error;
    double errorprev;
    double y=y1;
    double x=x1;
    double ddy, ddx;
    double dx = (x2-x1)/resolution_;
    double dy = (y2-y1)/resolution_;
    point_obstacle_(y2, x2);

    if (dy<0)
    {
        ystep=-0.1;
        dy=-dy;
    }
    else
    {
        ystep=0.1;
    }
    if (dx<0)
    {
        xstep=-0.1;
        dx=-dx;
    }
    else
    {
        xstep=0.1;
    }
    ddy = 2*dy;
    ddx = 2*dx;

    if (ddx>=ddy)
    {
        errorprev = error = dx;
        for (int i=0; i<dx; i++)
        {
            x += xstep;
            error += ddy;
            if (error>ddx)
            {
                y += ystep;
                error -= ddx;
                if (error+errorprev < ddx)
                    point_free_(y-ystep, x);
                else if (error+errorprev > ddx)
                    point_free_(y, x-xstep);
                else
                {
                    point_free_(y-ystep, x);
                    point_free_(y, x-xstep);
                }
            }
            point_free_(y,x);
            errorprev = error;
        }
    }
    else
    {
        errorprev = error = dy;
        for (int j=0; j<dy; j++)
        {
            y += ystep;
            error += ddx;
            if (error>ddy)
            {
                x += xstep;
                error -= ddy;
                if (error+errorprev <ddy)
                    point_free_(y, x-xstep);
                else if (error+errorprev > ddy)
                    point_free_(y-ystep, x);
                else
                {
                    point_free_(y, x-xstep);
                    point_free_(y-ystep,x);
                }
            }
            point_free_(y,x);
            errorprev = error;
        }
    }
}

void GridMapRegister::point_obstacle_(double a, double b)
{
    //std::cout<<a<<" "<<b<<std::endl;

}

void GridMapRegister::point_free_(double a, double b)
{
    //std::cout<<a<<" "<<b<<std::endl;
    int map_place = int(a/resolution_)*map_msg_.info.width+int(b/resolution_);
    if (map_place>map_msg_.data.size())
    {
        std::cout<<"pixel number: "<<map_place<<" map size: "<<map_msg_.data.size()<<std::endl;
    }

    else
    {
        if (map_msg_.data[map_place]!=100)
        {
            map_msg_.data[map_place] = 0;
        }

    }

}
