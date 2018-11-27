#include <ros/ros.h>
// PCL specific includes

#include <string>

// for pose relation in gtsam
#include <std_msgs/Int16MultiArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>
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

#include <fstream>

std::ofstream myfile;

ros::Publisher pub, pub_temp_map, pub_full_map, pub_source, pub_target, pub_icp_odom, pub_store_odom, pub_pose_link;

tf::TransformListener* tf_listener_;

std::string odom_frame_id_, base_frame_id_, map_frame_id_, file_name_, directory_name_;

int count, src_trgt, count_pcl, count_pose;

bool odom_start;

tf::Transform transform, offset_transform, now_odom_transform, difference, prev_odom_transform;

nav_msgs::Path stored_odometry;

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


tf::Transform EigenToTF(Eigen::Matrix4f eigen_matrix)
{
    tf::Transform eigen_TF;
    double t_x = eigen_matrix(0,3);
    double t_y = eigen_matrix(1,3);
    double t_z = eigen_matrix(2,3);

    tf::Matrix3x3 tf3d;
    tf3d.setValue(static_cast<double>(eigen_matrix(0,0)), static_cast<double>(eigen_matrix(0,1)), static_cast<double>(eigen_matrix(0,2)),
        static_cast<double>(eigen_matrix(1,0)), static_cast<double>(eigen_matrix(1,1)), static_cast<double>(eigen_matrix(1,2)),
        static_cast<double>(eigen_matrix(2,0)), static_cast<double>(eigen_matrix(2,1)), static_cast<double>(eigen_matrix(2,2)));

    tf::Quaternion tfqt,rotate;
    //tf3d.getRotation(tfqt);
    //tf3d.getRPY (roll,pitch,yaw);
    //tf3d.setRPY (0.0, pitch, yaw);
    tf3d.getRotation(tfqt);

    eigen_TF.setOrigin(tf::Vector3(t_x, t_y, t_z));
    eigen_TF.setRotation(tfqt);

    return eigen_TF;
}

Eigen::Matrix4f localize(int source_pcl, int target_pcl)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_comeout(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_origin(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_origin(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_icp(new pcl::PointCloud<pcl::PointXYZI>);

//    pcl_ros::transformPointCloud(*cloud_minimap, *map_origin, transform);
    tf::Transform target_tf, source_tf, inv_target_tf, inv_source_tf, icp_tf;
    target_tf.setOrigin(tf::Vector3(pose_node->at(target_pcl).pose.pose.position.x, pose_node->at(target_pcl).pose.pose.position.y,
                                    pose_node->at(target_pcl).pose.pose.position.z));
    target_tf.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    source_tf.setOrigin(tf::Vector3(pose_node->at(source_pcl).pose.pose.position.x, pose_node->at(source_pcl).pose.pose.position.y,
                                    pose_node->at(source_pcl).pose.pose.position.z));
    source_tf.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

    inv_source_tf = source_tf.inverse();
    inv_target_tf = target_tf.inverse();

//    pcl_ros::transformPointCloud (*pcl_node->at(target_pcl), *target_origin, inv_target_tf);
//    pcl_ros::transformPointCloud (*pcl_node->at(source_pcl), *source_origin, inv_source_tf);

    *target_origin = *pcl_node->at(target_pcl);
    *source_origin = *pcl_node->at(source_pcl);

    Eigen::Matrix4f compare_matrix;

    icp.setMaximumIterations (5);
    icp.setMaxCorrespondenceDistance(0.5);
    //icp.setTransformationEpsilon (1e-7);

    sensor_msgs::PointCloud2 source, target;
    pcl::toROSMsg(*target_origin, target);
    target.header.frame_id = odom_frame_id_;
    pub_target.publish(target);

    icp.setInputSource (source_origin);
    icp.setInputTarget (target_origin);
    icp.align (*cloud_comeout);
    double score=icp.getFitnessScore();
    compare_matrix = icp.getFinalTransformation ();


    pcl::toROSMsg(*cloud_comeout, source);
    source.header.frame_id = odom_frame_id_;
    pub_source.publish(source);

    std::cout<<"does this look ok? (y/n)"<<std::endl;
    std::string feedback;
    std::cin>>feedback;
    if (feedback.c_str()=="y")
    {
          icp_tf = EigenToTF(compare_matrix);
          icp_tf = icp_tf * source_tf;
          //pcl_ros::transformPointCloud (*cloud_comeout, *pcl_node->at(source_pcl), source_tf);
          tf::Vector3 icp_position;
          icp_position = icp_tf.getOrigin();
//          pose_node->at(source_pcl).pose.pose.position.x = icp_position.x();
//          pose_node->at(source_pcl).pose.pose.position.y = icp_position.y();
//          pose_node->at(source_pcl).pose.pose.position.z = icp_position.z();
//          tf::quaternionTFToMsg(icp_tf.getRotation(), pose_node->at(source_pcl).pose.pose.orientation);
          tf::Matrix3x3 tf3d;
          tf3d.setRotation(icp_tf.getRotation());
          double roll, pitch, yaw;
          tf3d.getRPY(roll, pitch, yaw);

          std_msgs::Int16MultiArray betweenPose;
          betweenPose.data.resize(5);
          betweenPose.data.push_back(source_pcl);
          betweenPose.data.push_back(target_pcl);
          betweenPose.data.push_back(icp_position.x());
          betweenPose.data.push_back(icp_position.y());
          betweenPose.data.push_back(yaw);

          pub_pose_link.publish(betweenPose);
    }
    else
    {
        std::cout<<"nothing changed..."<<std::endl;
    }

    return (compare_matrix);
}

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
                    localize(count_pcl, cloud_no);
              }
          }
          break;



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

  std::cout<<"x: "<<position.x()<<" y: "<<position.y()<<" z: "<<position.z()<<std::endl;
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

void pcl_and_pose_cb(const sensor_msgs::PointCloud2ConstPtr input, const nav_msgs::OdometryConstPtr after_map_odom)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_origin (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg (*input, *cloud_in);
    //pcl_node->push_back(cloud_in);
    std::cout<<"cloud size: "<<cloud_in->size()<<std::endl;

    tf::Vector3 loam_pose(after_map_odom->pose.pose.position.x,after_map_odom->pose.pose.position.y,after_map_odom->pose.pose.position.z);
//    loam_pose.x() = after_map_odom.pose.pose.position.x;
//    loam_pose.y() = after_map_odom.pose.pose.position.y;
//    loam_pose.z() = after_map_odom.pose.pose.position.z;
    pose_node->push_back(*after_map_odom);

    tf::Transform loam_tf, inverse_loam_tf;
    loam_tf.setOrigin(loam_pose);
    tf::Quaternion loam_tf_orientation;
    tf::quaternionMsgToTF(after_map_odom->pose.pose.orientation, loam_tf_orientation);
    loam_tf.setRotation(loam_tf_orientation);

    inverse_loam_tf = loam_tf.inverse();
    pcl_ros::transformPointCloud (*cloud_in, *cloud_origin, inverse_loam_tf);
    pcl_node->push_back(cloud_origin);

    // save in pcd
    std::string save_file_name;
    std::stringstream nn;
    nn<<count;
    save_file_name = directory_name_+ "/" + file_name_ + "_" + nn.str();
    pcl::io::savePCDFileBinaryCompressed(save_file_name, *cloud_origin);

    std::string save_odom;
    save_odom = directory_name_+"/odom.txt";

    myfile<<after_map_odom->pose.pose.position.x<<" "<<after_map_odom->pose.pose.position.y<<" "<<after_map_odom->pose.pose.position.z<<
            " "<<after_map_odom->pose.pose.orientation.x<<" "<<after_map_odom->pose.pose.orientation.y<<" "<<
            after_map_odom->pose.pose.orientation.z<<" "<<after_map_odom->pose.pose.orientation.w<<"\n";

    geometry_msgs::PoseStamped pcl_odom;

    pcl_odom.pose = after_map_odom->pose.pose;

    stored_odometry.poses.push_back(pcl_odom);

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

void pose_refine_cb (nav_msgs::Path refined_odom)
{
    server.reset( new interactive_markers::InteractiveMarkerServer("show_map","",false) );

    server->applyChanges();

    count = 0;

    pose_node->clear();

    for (int j=0; j<refined_odom.poses.size(); j++)
    {
        nav_msgs::Odometry temp_odom;
        tf::Vector3 new_loam_pose (refined_odom.poses[j].pose.position.x, refined_odom.poses[j].pose.position.y, 0.0);
        temp_odom.pose.pose = refined_odom.poses[j].pose;
        pose_node->push_back(temp_odom);

        makeButtonMarker(new_loam_pose, j+1);
        //server->applyChanges();
    }


}

int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "show_map");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    tf_listener_ = new tf::TransformListener();

    bool load_pcd;

    priv_nh.getParam("odom_frame", odom_frame_id_);
    priv_nh.getParam("base_frame", base_frame_id_);
    priv_nh.getParam("map_frame", map_frame_id_);
    priv_nh.getParam("file_name", file_name_);
    priv_nh.getParam("directory_name", directory_name_);
    priv_nh.getParam("offline_mapping", load_pcd);

//    ros::Subscriber sub_pcl = nh.subscribe <sensor_msgs::PointCloud2> ("laser_cloud_surround", 1, pc_cb);
//    ros::Subscriber sub_pose = nh.subscribe <nav_msgs::Odometry> ("aft_mapped_to_init", 1, store_pose_cb);

    ros::Subscriber sub_refine_pose = nh.subscribe <nav_msgs::Path> ("final_pose", 1, pose_refine_cb);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, "velodyne_cloud_registered", 1);
    message_filters::Subscriber<nav_msgs::Odometry> pose_sub(nh, "aft_mapped_to_init", 1);
    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, nav_msgs::Odometry> sync(pcl_sub, pose_sub, 10);
    sync.registerCallback(boost::bind(&pcl_and_pose_cb, _1, _2));

    pub = nh.advertise<sensor_msgs::PointCloud2> ("/map_in_total", 1);
    pub_full_map = nh.advertise<sensor_msgs::PointCloud2> ("/full_map", 1);
    pub_temp_map = nh.advertise<sensor_msgs::PointCloud2> ("/temp_map", 1);
    pub_target = nh.advertise<sensor_msgs::PointCloud2> ("/target_map", 1);
    pub_source = nh.advertise<sensor_msgs::PointCloud2> ("/source_map", 1);
    pub_store_odom = nh.advertise<nav_msgs::Path> ("store_odom", 1);
    pub_pose_link = nh.advertise <std_msgs::Int16MultiArray> ("new_link", 1);

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

    server.reset( new interactive_markers::InteractiveMarkerServer("show_map","",false) );

    server->applyChanges();

    std::string odom_txt_name;
    odom_txt_name = directory_name_+"/odom.txt";

    if (load_pcd)
    {
        while (1)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr one_small_cloud (new pcl::PointCloud<pcl::PointXYZI>);
            std::string portion_map;
            std::stringstream zz;
            zz<< count;
            portion_map = directory_name_ +"/" +file_name_ + "_" + zz.str();

            //pcl::io::loadPCDFile(portion_map, *one_small_cloud);
            if (pcl::io::loadPCDFile (portion_map, *one_small_cloud) == -1) //* load the file
            {
                std::cout<<"pcd files end at "<<count<<std::endl;
                return (-1);
            }
            pcl_node->push_back(one_small_cloud);
            count++;

        }
        std::ifstream load_odom;
        load_odom.open(odom_txt_name.c_str(), std::ios::app);
        double value;
        int text_position;
        nav_msgs::Odometry record_odom;
        while (load_odom >> value)
        {
            text_position++;
            if (text_position==1)
            {
                record_odom.pose.pose.position.x = value;
            }
            else if (text_position==2)
            {
                record_odom.pose.pose.position.y = value;
            }
            else if (text_position==3)
            {
                record_odom.pose.pose.position.z = value;
            }
            else if (text_position==4)
            {
                record_odom.pose.pose.orientation.x = value;
            }
            else if (text_position==5)
            {
                record_odom.pose.pose.orientation.y = value;
            }
            else if (text_position==6)
            {
                record_odom.pose.pose.orientation.z = value;
            }
            else if (text_position==7)
            {
                record_odom.pose.pose.orientation.w = value;
                pose_node->push_back(record_odom);
                tf::Vector3 record_pose(record_odom.pose.pose.position.x, record_odom.pose.pose.position.y, 0.0);
                makeButtonMarker(record_pose, pose_node->size());
                text_position=0;
            }

        }
    }
    else
    {
        myfile.open (odom_txt_name.c_str());
    }



    //ros::Timer frame_timer = nh.createTimer(ros::Duration(0.01), frameCallback);



    ros::spin();

    server.reset();
}

