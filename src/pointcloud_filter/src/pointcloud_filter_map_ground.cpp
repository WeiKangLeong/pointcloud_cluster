#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <tf/transform_listener.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/time.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <visualization_msgs/Marker.h>

#include "../../../include/grid.h"
//#include <algorithm.h>

#define RES 0.1
#define RANGE_X 30.0
#define RANGE_Y 30.0

Grid* grid_map_;

ros::Publisher pub, marker_pub, pub_object, pub_ransac, pub_grid_map;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_indices(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_previous(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_comeout(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_lowest_mapped(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZI>);

pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
pcl::ExtractIndices<pcl::PointXYZ> extract;
pcl::PassThrough<pcl::PointXYZI> pass;
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

int x_max, y_max;

bool start, view_imu;
double a, b, c, d, norm;
double p_a, p_b, p_c, p_d, p_norm;
int change, min_neighbour, iteration;
std::vector<double>* sorting_neighbour_;
std::vector<int>* filter_road_;
std::vector<std::vector<double>* >* floor_level_;

std::string map_frame_, map_location_;
double resolution_;

tf::TransformListener *tf_listener_;

void find_a_plane()
{
    int n = cloud_lowest_mapped->size();
    geometry_msgs::Point centroid, r;
   for (int w=0; w<n; w++)
   {
       centroid.x = centroid.x + cloud_lowest_mapped->points[w].x;
       centroid.y = centroid.y + cloud_lowest_mapped->points[w].y;
       centroid.z = centroid.z + cloud_lowest_mapped->points[w].z;
   }
   centroid.x = centroid.x/n;
   centroid.y = centroid.y/n;
   centroid.z = centroid.z/n;

   //std::cout<<centroid.x<<" "<<centroid.y<<" "<<centroid.z<<std::endl;

   double xx=0, xy=0, xz=0, yy=0, yz=0, zz=0, det_x=0, aa=0, bb=0;
   a=0; b=0; c=0; d=0;

   for (int w=0; w<n; w++)
   {
       r.x = cloud_lowest_mapped->points[w].x - centroid.x;
       r.y = cloud_lowest_mapped->points[w].y - centroid.y;
       r.z = cloud_lowest_mapped->points[w].z - centroid.z;
       xx = xx + (r.x)*(r.x);
       xy = xy + (r.x)*(r.y);
       xz = xz + (r.x)*(r.z);
       yy = yy + (r.y)*(r.y);
       yz = yz + (r.y)*(r.z);
       zz = zz + (r.z)*(r.z);


   }

   det_x = yy*zz - yz*yz;
   aa = (xz*yz - xy*zz)/det_x;
   bb = (xy*yz - xz*yy)/det_x;
   a = 1.0;
   b = aa;
   c = bb;
    d = -(a*centroid.x + b*centroid.y + c*centroid.z);
    norm = sqrt(a*a + b*b + c*c);
    //std::cout<<a<<" "<<b<<" "<<c<<" "<<d<<" "<<norm<<std::endl;
    int outlier=0;
    double distance = 0.0;
    for (int ddd=0; ddd<cloud_lowest_mapped->size(); ddd++)
    {
        distance = distance + fabs((a*cloud_lowest_mapped->points[ddd].x + b*cloud_lowest_mapped->points[ddd].y + c*cloud_lowest_mapped->points[ddd].z + d)/norm);
    }

    distance = distance/cloud_lowest_mapped->size();
    //std::cout<<"Overall distance: "<<distance<<std::endl;

    // distance < 0.1 for ransac=0.1
    if (distance<0.1)
    {
       double p_x, p_y;

       for (int i=0; i<x_max; i++)
       {
           for (int j=0; j<y_max; j++)
           {
               p_x = (double)RANGE_X/2 - (double)i*RES + 0.0;
               p_y = (double)RANGE_Y/2 - (double)j*RES + 0.0;
               floor_level_->at(i)->at(j) = -(d + a*p_x + b*p_y)/c;
           }
       }

       visualization_msgs::Marker line_strip;
       //visualization_msgs::MarkerArray ma;
       line_strip.header.frame_id = "wheelchair/map";
       //line_strip.header.stamp = ros::Time::now();
       //line_strip.ns = "pedestrian";

       line_strip.action = visualization_msgs::Marker::ADD;
       line_strip.pose.orientation.w = 1.0;

       line_strip.id = 0;
       line_strip.type = visualization_msgs::Marker::LINE_STRIP;
       line_strip.scale.x = 0.1;

       line_strip.color.a = 1.0;
       line_strip.color.r = 0.0;
       line_strip.color.g = 1.0;
       line_strip.color.b = 0.0;

       geometry_msgs::Point border_1, border_2, border_3, border_4;
       border_1.x = (double)RANGE_X/2;
       border_1.y = (double)RANGE_X/2;
       border_1.z = -(d + a*border_1.x + b*border_1.y)/c;

       line_strip.points.push_back(border_1);
       border_2.x = (double)RANGE_X/2;
       border_2.y = (double)RANGE_X/2 - (double)(y_max-1)*RES + 0.0;
       border_2.z = -(d + a*border_2.x + b*border_2.y)/c;

       line_strip.points.push_back(border_2);
       border_3.x = (double)RANGE_X/2 - (double)(x_max-1)*RES + 0.0;
       border_3.y = (double)RANGE_X/2 - (double)(y_max-1)*RES + 0.0;
       border_3.z = -(d + a*border_3.x + b*border_3.y)/c;

       line_strip.points.push_back(border_3);
       border_4.x = (double)RANGE_X/2 - (double)(x_max-1)*RES + 0.0;
       border_4.y = (double)RANGE_X/2;
       border_4.z = -(d + a*border_4.x + b*border_4.y)/c;

       line_strip.points.push_back(border_4);

       line_strip.points.push_back(border_1);

       //ma.markers.push_back(line_strip);
       marker_pub.publish(line_strip);

       p_a=a;
       p_b=b;
       p_c=c;
       p_d=d;
       p_norm=norm;
    }
    else
    {
        a=p_a;
        b=p_b;
        c=p_c;
        d=p_d;
        norm=p_norm;
        for (int ddd=0; ddd<cloud_lowest_mapped->size(); ddd++)
        {
            distance = distance + fabs((a*cloud_lowest_mapped->points[ddd].x + b*cloud_lowest_mapped->points[ddd].y + c*cloud_lowest_mapped->points[ddd].z + d)/norm);
        }

        distance = distance/cloud_lowest_mapped->size();
        //std::cout<<"Previous plane overall distance: "<<distance<<std::endl;
    }

}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::console::TicToc time;
    time.tic ();

    pcl::PointCloud<pcl::PointXYZ>::Ptr shift_pointcloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*input, *cloud_all);
    pcl::fromROSMsg (*input, *shift_pointcloud);
    if (view_imu==true)
    {
        tf::StampedTransform latest_imu_transform;
        try{
            tf_listener_->lookupTransform("/wtf_base_link", "wtf_velodyne",
                                          ros::Time(0), latest_imu_transform);
        }catch (tf::TransformException &ex) {
            ROS_ERROR_STREAM("localizer_scan: Looking Transform Failed");
            return;
        }
        pcl_ros::transformPointCloud(*cloud_all, *cloud_all, latest_imu_transform);
    }

    // Create a container for the data.

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*shift_pointcloud, centroid);

    tf::Transform tozero;
    tozero.setOrigin(tf::Vector3(-centroid[0], -centroid[1], 0.0));
    tozero.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    pcl_ros::transformPointCloud(*cloud_all, *cloud_all, tozero);

    std::cout<<centroid[0]<<" "<<centroid[1]<<std::endl;

    //<<" "<<centroid.y<<" "<<centroid.z
    *cloud_indices = *cloud_all;
    *cloud_in = *cloud_all;

//    pass.setInputCloud (cloud_all);
//    pass.setFilterFieldName ("x");
//    pass.setFilterLimits (-RANGE_X, RANGE_X);
//    pass.filter (*cloud_in);
//    pass.setInputCloud (cloud_in);
//    pass.setFilterFieldName ("y");
//    pass.setFilterLimits (-RANGE_Y, RANGE_Y);
//    pass.filter (*cloud_in);
//    pass.setInputCloud (cloud_in);
//    pass.setFilterFieldName ("z");
//    pass.setFilterLimits (-3.0, -1.0);
//    pass.filter (*cloud_in);

    // put index for every point
    for (int i=0; i<cloud_indices->size(); i++)
    {
        cloud_indices->points[i].intensity = i;
    }



        for(int cnt_i=0; cnt_i<cloud_in->size(); cnt_i++)
        {
                grid_map_->InsertXYZI(cloud_in->points[cnt_i], cnt_i);
        }

        filter_road_ = grid_map_->find_lowest_indices(cloud_in);

        for (int cnt_r=0; cnt_r<filter_road_->size(); cnt_r++)
        {
            cloud_filtered->push_back(cloud_indices->points[filter_road_->at(cnt_r)]);
        }

        pcl::ModelCoefficients coefficients;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.1);

        //seg.setInputCloud (cloud_indices->makeShared ());
        seg.setInputCloud (cloud_filtered->makeShared ());
        seg.segment (*inliers, coefficients);

        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZI> extract;

        // Extract the inliers
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_lowest_mapped);

        sensor_msgs::PointCloud2 output3;
        pcl::toROSMsg (*cloud_filtered, output3);
        output3.header = input->header;
        output3.header.frame_id = "wheelchair/map";
        pub.publish (output3);

        std::cout<<"PointCloud size: "<<cloud_in->size()<<" to "<<cloud_filtered->size()<<" to "<<cloud_lowest_mapped->size()<<std::endl;

        find_a_plane();

        cloud_lowest_mapped.reset(new pcl::PointCloud<pcl::PointXYZI>);
        int outlier_p=0;

        for (int cnt=0; cnt<cloud_all->size(); cnt++)
        {
            double ptoplane = fabs(a*cloud_all->points[cnt].x + b*cloud_all->points[cnt].y + c*cloud_all->points[cnt].z + d)/norm;

            //double min_z = fabs(sqrt(cloud_all->points[cnt].x*cloud_all->points[cnt].x + cloud_all->points[cnt].y*cloud_all->points[cnt].y)) * tan(PI*2.0/180);

            double min_z = 0.5;
            if (ptoplane<min_z)
            {
                outlier_p++;
                cloud_lowest_mapped->push_back(cloud_all->points[cnt]);
            }
            else
            {
                cloud_f->push_back(cloud_indices->points[cnt]);
            }
        }
        //std::cout<<"Points near to plane: "<<outlier_p<<std::endl;

//        sensor_msgs::PointCloud2 output;
//        pcl::toROSMsg (*cloud_lowest_mapped, output);
//        output.header = input->header;
//        output.header.frame_id = "wheelchair/map";
//        pub.publish (output);

        std::cout<<"Road plane: "<<cloud_lowest_mapped->size()<< " object cloud: "<<cloud_f->size()<<std::endl;
        sensor_msgs::PointCloud2 output2;
        pcl::toROSMsg (*cloud_f, output2);
        output2.header = input->header;
        output2.header.frame_id = "wheelchair/map";
        pub_object.publish (output2);

        grid_map_->ClearGrid();
        cloud_lowest_mapped.reset(new pcl::PointCloud<pcl::PointXYZI>);
        cloud_f.reset(new pcl::PointCloud<pcl::PointXYZI>);
        cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZI>);
        cloud_in.reset(new pcl::PointCloud<pcl::PointXYZI>);
        filter_road_->clear();

    std::cout<<"Time used: "<<time.toc()<<std::endl;

}

//void odom_cb(const nav_msgs::Odometry::ConstPtr& odom)
//{
//    tf::Transform odom_transform;
//    tf::Quaternion new_rotation;
//    tf::quaternionMsgToTF(odom->pose.pose.orientation, new_rotation);
//    odom_transform.setOrigin(tf::Vector3(odom->pose.pose.position.x,odom->pose.pose.position.y,odom->pose.pose.position.z));
//    odom_transform.setRotation(new_rotation.normalize());


//    pcl_ros::transformPointCloud(*cloud_lowest_mapped, *cloud_lowest_mapped, odom_transform);
//    sensor_msgs::PointCloud2 output;
//    pcl::toROSMsg (*cloud_lowest_mapped, output);
//    output.header.frame_id = "map";

//    // Publish the data.
//    pub.publish (output);
//    cloud_lowest_mapped.reset(new pcl::PointCloud<pcl::PointXYZI>);


//    //cloud_lowest_mapped->insert(cloud_lowest_mapped->end(), cloud_filtered->begin(), cloud_filtered->end());
//}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pointcloud_filter_map");
  ros::NodeHandle nh;  
  ros::NodeHandle priv_nh("~");

  priv_nh.getParam("map_location", map_location_);
  priv_nh.getParam("map_frame", map_frame_);
  priv_nh.getParam("resolution", resolution_);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("cloud_input", 1, cloud_cb);
  //ros::Subscriber sub_odom = nh.subscribe ("/icp_odom", 1, odom_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/road_map", 1);
  pub_object = nh.advertise<sensor_msgs::PointCloud2> ("/object_on_road", 1);
  marker_pub = nh.advertise<visualization_msgs::Marker>("/floor_normal", 10);
  pub_grid_map = nh.advertise<nav_msgs::OccupancyGrid> ("map", 1);

  tf_listener_ = new tf::TransformListener();

  sorting_neighbour_ = new std::vector<double>;
  filter_road_ = new std::vector<int>;
  floor_level_ = new std::vector<std::vector<double>* >;
  floor_level_->resize(x_max);
  for (int i=0; i<x_max; i++)
  {
      floor_level_->at(i) = new std::vector<double>;
      floor_level_->at(i)->resize(y_max);
  }

  icp.setMaximumIterations (30);
  //icp.setMaxCorrespondenceDistance(0.5);
  //icp.setTransformationEpsilon (1e-7);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_largemap(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_gridmap(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_grid(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(map_location_, *cloud_largemap);
    std::cout<<"map loaded...in "<<map_location_<<" with a frame: "<<map_frame_<<std::endl;

    pcl::PointXYZI min_point, max_point;
    pcl::getMinMax3D (*cloud_largemap, min_point, max_point);

    int range_x = int(max_point.x-min_point.x+1.0);
    int range_y = int(max_point.y-min_point.y+1.0);

    grid_map_ = new Grid(range_x, range_y, 0, resolution_, min_point.x, min_point.y);

    std::cout<<"map loaded with size: "<<cloud_largemap->size()<<" x: "<<range_x<<" y: "<<range_y<<std::endl;

    nav_msgs::OccupancyGrid msg;
    msg.info.width = range_x/resolution_;
    msg.info.height = range_y/resolution_;
    msg.info.resolution = resolution_;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = map_frame_;

    int total_grid_ = int(msg.info.width)*int(msg.info.height);

    for (int m=0; m<total_grid_; m++)
    {
        msg.data.push_back(-1);
    }

    for (int i=0; i<cloud_largemap->size(); i++)
    {
        grid_map_->InsertXYZI_with_origin(cloud_largemap->points[i], i);
    }

    cloud_gridmap = grid_map_->find_vertical(cloud_largemap);

//    for (int lc=0; lc<cloud_gridmap->size(); lc++)
//    {
//        if (cloud_gridmap->points[lc].z==0.0)
//        {
//            cloud_gridmap->points[lc].intensity = 200;
//        }
//        else
//        {
//            cloud_gridmap->points[lc].intensity = 100;
//        }
//    }

    for (int lb=0; lb<cloud_gridmap->size(); lb++)
    {
        pcl::PointXYZI pt_on_map;
        if (cloud_gridmap->points[lb].z>0.0)
        {
            pt_on_map=cloud_gridmap->points[lb];
            cloud_grid->push_back(pt_on_map);
        }
//        else if (cloud_gridmap->points[lb].z==-1)
//        {
//            double grid_x = cloud_gridmap->points[lb].x;// + range_x/2;
//            double grid_y = cloud_gridmap->points[lb].y;// + range_y/2;

//            int place = int(grid_x/resolution_)*msg.info.height+int(grid_y/resolution_);
//            int map_place = int(grid_y/resolution_)*msg.info.width+int(grid_x/resolution_);

//            if (map_place>msg.data.size())
//            {
//                std::cout<<"pixel number: "<<map_place<<" map size: "<<msg.data.size()<<std::endl;
//            }

//            else
//            {
//                msg.data[map_place] = 0;
//            }
//        }
    }

    sensor_msgs::PointCloud2 outputn, outputm;
    pcl::toROSMsg (*cloud_grid, outputn);
    outputn.header.stamp = ros::Time::now();
    outputn.header.frame_id = map_frame_;
    pub.publish (outputn);

    pcl::toROSMsg (*cloud_largemap, outputm);
    outputm.header.stamp = ros::Time::now();
    outputm.header.frame_id = map_frame_;
    pub_object.publish (outputm);



        //filter_road_ = grid_map_->getMap();

        std::cout<<"map size: "<<cloud_grid->size()<<std::endl;

        for (int i=0; i<cloud_grid->size(); i++)
        {
            double grid_x = cloud_grid->points[i].x;// + range_x/2;
            double grid_y = cloud_grid->points[i].y;// + range_y/2;



            int place = int(grid_x/resolution_)*msg.info.height+int(grid_y/resolution_);
            int map_place = int(grid_y/resolution_)*msg.info.width+int(grid_x/resolution_);

            if (map_place>msg.data.size())
            {
                std::cout<<"pixel number: "<<map_place<<" map size: "<<msg.data.size()<<std::endl;
            }

            else
            {
                msg.data[map_place] = 100;
            }
        }

//        for (int i=0; i<filter_road_->size(); i++)
//        {
//            msg.data.push_back(filter_road_->at(i));
//        }

        pub_grid_map.publish(msg);


  // Spin
  ros::spin ();
}

