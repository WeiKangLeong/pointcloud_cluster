#include <ros/ros.h>
// PCL specific includes

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl/registration/icp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PointIndices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

ros::Publisher pub, pub_temp_map, pub_full_map, pub_source, pub_target, pub_icp_odom;

tf::TransformListener* tf_listener_;

std::string odom_frame_id_, base_frame_id_, map_frame_id_;

int count, src_trgt, count_pcl, count_pose;

bool odom_start;

tf::Transform transform, offset_transform, now_odom_transform, difference, prev_odom_transform;

Eigen::Matrix4f use_odom;

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>* pcl_node;
std::vector<nav_msgs::Odometry>* pose_node;

pcl::PointCloud<pcl::PointXYZI>::Ptr map (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr temp_map (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_minimap (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_accumulated (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr previous_accumulated (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr final_map (new pcl::PointCloud<pcl::PointXYZI>);

pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
pcl::PassThrough<pcl::PointXYZI> pass;

using namespace visualization_msgs;


// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
// %EndTag(vars)%


// %Tag(Box)%
Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::SPHERE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}


void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  int cloud_no = std::atoi(feedback->marker_name.c_str());

    switch ( feedback->event_type )
    {
        case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
          ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
          break;
        sensor_msgs::PointCloud2 pcl_output;
        pcl::toROSMsg (*pcl_node->at(cloud_no), pcl_output);
        pcl_output.header.frame_id = odom_frame_id_;
        if (src_trgt==0)
        {
            src_trgt=1;
            pub_target.publish(pcl_output);
            count_pcl=cloud_no;
        }
        else if (src_trgt==1)
        {
            src_trgt=0;
            pub_source.publish(pcl_output);

            std::cout<<"match between node "<<count_pcl<<" and "<<cloud_no<<" ?"<<std::endl;
            std::string enter_command;
            std::cin>>enter_command;
            if (enter_command.c_str()=="y")
            {

            }
        }


    }

  server->applyChanges();
}

void makeButtonMarker( const tf::Vector3& position, int i)
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = odom_frame_id_;
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  std::stringstream ss;
  ss<< i;
  int_marker.name = ss.str();
  int_marker.description = "Button "+ss.str();

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.name = "button_control";

  Marker marker = makeBox( int_marker );
  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

//void store_pose_cb (nav_msgs::Odometry after_map_odom)
//{
//    tf::Vector3 loam_pose;
//    loam_pose.x() = after_map_odom.pose.pose.position.x;
//    loam_pose.y() = after_map_odom.pose.pose.position.y;
//    loam_pose.z() = after_map_odom.pose.pose.position.z;
//    pose_node->push_back(after_map_odom);
//    count_pose++;
//}

Eigen::Matrix4f localize(pcl::PointCloud<pcl::PointXYZI>::Ptr source_pcl, pcl::PointCloud<pcl::PointXYZI>::Ptr target_pcl)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_comeout(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr map_origin(new pcl::PointCloud<pcl::PointXYZI>);

//    pcl_ros::transformPointCloud(*cloud_minimap, *map_origin, transform);

    Eigen::Matrix4f compare_matrix;

    icp.setMaximumIterations (5);
    icp.setMaxCorrespondenceDistance(0.5);
    //icp.setTransformationEpsilon (1e-7);

    sensor_msgs::PointCloud2 source, target;
    pcl::toROSMsg(*cloud_minimap, target);
    target.header.frame_id = odom_frame_id_;
    pub_target.publish(target);

    icp.setInputSource (cloud_minimap);
    icp.setInputTarget (cloud_accumulated);
    icp.align (*cloud_comeout);
    double score=icp.getFitnessScore();
    compare_matrix = icp.getFinalTransformation ();


    pcl::toROSMsg(*cloud_comeout, source);
    source.header.frame_id = odom_frame_id_;
    pub_source.publish(source);

    if (score<0.2)
    {
        std::cout << "Applied iterations with " <<score<< std::endl;
    }
    else
    {
        compare_matrix = use_odom;
    }


    return (compare_matrix);
}

void pcl_and_pose_cb(const sensor_msgs::PointCloud2ConstPtr input, const nav_msgs::OdometryConstPtr after_map_odom)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg (*input, *cloud_in);
    pcl_node->push_back(cloud_in);

    tf::Vector3 loam_pose(after_map_odom->pose.pose.position.x,after_map_odom->pose.pose.position.y,after_map_odom->pose.pose.position.z);
//    loam_pose.x() = after_map_odom.pose.pose.position.x;
//    loam_pose.y() = after_map_odom.pose.pose.position.y;
//    loam_pose.z() = after_map_odom.pose.pose.position.z;
    pose_node->push_back(*after_map_odom);
    count++;

    makeButtonMarker(loam_pose, count);
    server->applyChanges();

}

void pc_cb(const sensor_msgs::PointCloud2ConstPtr input)
{
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg (*input, *cloud_in);
        pcl_node->push_back(cloud_in);

        count_pcl++;

//    tf::StampedTransform latest_odom_transform;
//    try{
//        tf_listener_->lookupTransform(odom_frame_id_, base_frame_id_,
//                                      ros::Time(0), latest_odom_transform);
//    }catch (tf::TransformException &ex) {
//        ROS_ERROR_STREAM("Looking Transform from odom and base_link Failed");
//        return;
//    }

//    if (odom_start==false)
//    {
//        offset_transform = latest_odom_transform;
//        odom_start=true;
//    }

//    now_odom_transform = offset_transform.inverseTimes(latest_odom_transform);

//    difference = prev_odom_transform.inverseTimes(now_odom_transform);

//    transform = transform * difference;

//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_transform (new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::fromROSMsg (*input, *cloud_in);

//    pcl_ros::transformPointCloud(*cloud_in, *cloud_transform, difference);

//    *temp_map += *cloud_transform;



//    count++;

//    if (count==5)
//    {
//        count=0;

//        voxel_filter.setLeafSize (0.2, 0.2, 0.2);
//        voxel_filter.setInputCloud (temp_map);
//        voxel_filter.filter (*cloud_accumulated);
//        if (map->size()==0)
//        {
//            *map += *cloud_accumulated;
//            *previous_accumulated = *cloud_accumulated;
//        }

//        tf::Vector3 odom_now, odom_previous;
//        odom_now = now_odom_transform.getOrigin();
//        odom_previous = prev_odom_transform.getOrigin();
//        //pcl_ros::transformPointCloud(*cloud_accumulated, *cloud_accumulated, now_odom_transform.inverse());

//        pass.setInputCloud (cloud_accumulated);
//        pass.setFilterFieldName ("x");
//        pass.setFilterLimits (-30.0, 30.0);
//        pass.filter (*cloud_accumulated);

//        pass.setInputCloud (cloud_accumulated);
//        pass.setFilterFieldName ("y");
//        pass.setFilterLimits (-30.0, 30.0);
//        pass.filter (*cloud_accumulated);

//        pcl_ros::transformPointCloud(*previous_accumulated, *previous_accumulated, prev_odom_transform.inverse());
//        pass.setInputCloud (previous_accumulated);
//        pass.setFilterFieldName ("x");
//        pass.setFilterLimits (-30.0, 30.0);
//        pass.filter (*cloud_minimap);

//        pass.setInputCloud (cloud_minimap);
//        pass.setFilterFieldName ("y");
//        pass.setFilterLimits (-35.0, 35.0);
//        pass.filter (*cloud_minimap);

//        Eigen::Matrix4f transformation_matrix = localize();

//        double translation_x = transformation_matrix(0,3);
//        double translation_y = transformation_matrix(1,3);
//        double translation_z = transformation_matrix(2,3);

//        tf::Matrix3x3 tf3d;
//        tf3d.setValue(static_cast<double>(transformation_matrix(0,0)), static_cast<double>(transformation_matrix(0,1)), static_cast<double>(transformation_matrix(0,2)),
//            static_cast<double>(transformation_matrix(1,0)), static_cast<double>(transformation_matrix(1,1)), static_cast<double>(transformation_matrix(1,2)),
//            static_cast<double>(transformation_matrix(2,0)), static_cast<double>(transformation_matrix(2,1)), static_cast<double>(transformation_matrix(2,2)));

//        tf::Quaternion tfqt;
//        //tf3d.getRotation(tfqt);
//        double roll, pitch, yaw;
//        tf3d.getRPY (roll,pitch,yaw);
//        //tf3d.setRPY (0.0, pitch, yaw);
//        tf3d.getRotation(tfqt);

//        tf::Transform transform2;
//        transform2.setOrigin(tf::Vector3(translation_x, translation_y, translation_z));
//        transform2.setRotation(tfqt);

//        transform = transform * transform2;

//        pcl::PointCloud<pcl::PointXYZI>::Ptr transform_temp (new pcl::PointCloud<pcl::PointXYZI>);
//        pcl_ros::transformPointCloud(*temp_map, *transform_temp, transform2);

//        sensor_msgs::PointCloud2 tempmap_output;
//        pcl::toROSMsg (*transform_temp, tempmap_output);
//        tempmap_output.header.frame_id = odom_frame_id_;
//        pub_temp_map.publish(tempmap_output);

//        nav_msgs::Odometry icp_odom;
//        tf::quaternionTFToMsg(transform.getRotation(), icp_odom.pose.pose.orientation);
//        tf::Vector3 icp_position = transform.getOrigin();
//        icp_odom.pose.pose.position.x = icp_position.x();
//        icp_odom.pose.pose.position.y = icp_position.y();
//        icp_odom.pose.pose.position.z = icp_position.z();

//        icp_odom.header.frame_id = odom_frame_id_;
//        icp_odom.header.stamp = input->header.stamp;

//        pub_icp_odom.publish(icp_odom);

////        *map += *temp_map;

////        voxel_filter.setInputCloud (map);
////        voxel_filter.filter (*final_map);

//        previous_accumulated.reset(new pcl::PointCloud<pcl::PointXYZI>);
//        *previous_accumulated += *temp_map;
//        temp_map.reset(new pcl::PointCloud<pcl::PointXYZI>);
//        map.reset(new pcl::PointCloud<pcl::PointXYZI>);
//        cloud_accumulated.reset(new pcl::PointCloud<pcl::PointXYZI>);
//        cloud_minimap.reset(new pcl::PointCloud<pcl::PointXYZI>);

////        *map = *final_map;

////        sensor_msgs::PointCloud2 map_output;
////        pcl::toROSMsg (*map, map_output);
////        map_output.header.frame_id = map_frame_id_;
////        pub_full_map.publish(map_output);

//    }

//    prev_odom_transform = now_odom_transform;
}

int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "show_map");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    tf_listener_ = new tf::TransformListener();

    priv_nh.getParam("odom_frame", odom_frame_id_);
    priv_nh.getParam("base_frame", base_frame_id_);
    priv_nh.getParam("map_frame", map_frame_id_);

//    ros::Subscriber sub_pcl = nh.subscribe <sensor_msgs::PointCloud2> ("laser_cloud_surround", 1, pc_cb);
//    ros::Subscriber sub_pose = nh.subscribe <nav_msgs::Odometry> ("aft_mapped_to_init", 1, store_pose_cb);

    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, "laser_cloud_surround", 1);
    message_filters::Subscriber<nav_msgs::Odometry> pose_sub(nh, "aft_mapped_to_init", 1);
    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, nav_msgs::Odometry> sync(pcl_sub, pose_sub, 10);
    sync.registerCallback(boost::bind(&pcl_and_pose_cb, _1, _2));

    pub = nh.advertise<sensor_msgs::PointCloud2> ("/map_in_total", 1);
    pub_full_map = nh.advertise<sensor_msgs::PointCloud2> ("/full_map", 1);
    pub_temp_map = nh.advertise<sensor_msgs::PointCloud2> ("/temp_map", 1);
    pub_target = nh.advertise<sensor_msgs::PointCloud2> ("/target_map", 1);
    pub_source = nh.advertise<sensor_msgs::PointCloud2> ("/source_map", 1);
    pub_icp_odom = nh.advertise<nav_msgs::Odometry> ("/icp_odom", 1);

    count = 0;
    count_pcl = 0;
    count_pose = 0;

    src_trgt = 0;

    odom_start = false;

    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

    prev_odom_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    prev_odom_transform.setRotation(tf::Quaternion (0.0, 0.0, 0.0, 1.0));

    use_odom.setIdentity();

    pcl_node = new std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr >;

    pose_node = new std::vector<nav_msgs::Odometry>;

    //ros::Timer frame_timer = nh.createTimer(ros::Duration(0.01), frameCallback);

    server.reset( new interactive_markers::InteractiveMarkerServer("show_map","",false) );

    server->applyChanges();

    ros::spin();

    server.reset();
}

