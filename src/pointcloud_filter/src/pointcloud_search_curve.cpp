#include "pointcloud_filter/pointcloud_search_curve.h"

namespace pointcloud_filter
{
	PCL_searchcurve::PCL_searchcurve()
	{
        ros::NodeHandle priv_nh("~");
        priv_nh.getParam("pose_location", pose_location_);
        //priv_nh.getParam("output_path", output_path_);
        priv_nh.getParam("resolution", resolution_);
        priv_nh.getParam("map", map_);

		sub_pcl_ = nh_.subscribe <sensor_msgs::PointCloud2> ("transform_points", 1, &PCL_searchcurve::pointcloud_cb_, this);
    	pub_pcl_ = nh_.advertise<sensor_msgs::PointCloud2> ("curve_points", 1);
        pub_search_ = nh_.advertise<sensor_msgs::PointCloud2> ("search_points", 1);
        pub_map_ = nh_.advertise<sensor_msgs::PointCloud2> ("map_points", 1);
        pub_plot_ = nh_.advertise<sensor_msgs::PointCloud2> ("plot_points", 1);
        pub_line_ = nh_.advertise<nav_msgs::Path> ("a_line", 1);
        pub_ransac_line_ = nh_.advertise<nav_msgs::Path> ("ransac_line", 1);
        pub_line_marker_ = nh_.advertise<visualization_msgs::Marker> ("line_marker", 1);

        voxel_filter_.setLeafSize (resolution_, resolution_, resolution_);

        cloud_map.reset(new pcl::PointCloud<pcl::PointXYZI>);

//        tf::Quaternion quat;
//        quat.setRPY(0.0, 0.174533, 0.0);
//        transform_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
//        transform_.setRotation(quat);

        pose_time_ = new std::vector<long double>;
        pose_x_ = new std::vector<double>;
        pose_y_ = new std::vector<double>;
        pose_yaw_ = new std::vector<double>;

        distance_threshold_ = 0.1;
        vector_threshold_ = 0.05;

        tfb_ = new tf::TransformBroadcaster();

        if (map_)
        {
            pose_txt_.open(pose_location_.c_str(), std::ios::app);

            double value;
            int count = 0;

            if (pose_txt_.is_open())
            {
                std::cout<<"poses is open"<<std::endl;
                while (pose_txt_ >> value)
                {

                    count++;
                    //std::cout<<count<<" "<<value<<std::endl;
                    if (count ==1)
                    {
                        pose_time_->push_back(value);
                    }
                    else if (count == 2)
                    {
                        pose_x_->push_back(value);
                    }
                    else if (count == 3)
                    {
                        pose_y_->push_back(value);
                    }
                    else if (count==4)
                    {
                        pose_yaw_->push_back(value);
                    }
                    else if (count == 9)
                    {
                        count = 0;
                    }
                }
                pose_txt_.close();
                std::cout<<"poses saved: "<<pose_x_->size()<<" "<<pose_y_->size()<<" "<<pose_yaw_->size()<<std::endl;
            }
            else
            {
                std::cout<<"cannot open "<<pose_location_<<std::endl;
            }
        }






	}

	PCL_searchcurve::~PCL_searchcurve()
	{

	}

    double PCL_searchcurve::interpolate(long double t1, long double t2, long double t, double x1, double x2)
    {
        double current_x;
        current_x = x2 - (x2-x1)*(t2-t)/(t2-t1);
        return (current_x);
    }

    geometry_msgs::Point PCL_searchcurve::difference_point(geometry_msgs::Point pt1, geometry_msgs::Point pt2)
    {
        geometry_msgs::Point diff;
        diff.x = pt2.x - pt1.x;
        diff.y = pt2.y - pt1.y;
        diff.z = pt2.z - pt1.z;

        return diff;
    }

    void PCL_searchcurve::estimate_line(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, double b, double m)
    {

    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr PCL_searchcurve::extract_groundpoint (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_layer, double b, double m)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_return (new pcl::PointCloud<pcl::PointXYZI>);
        for (int t=0; t<cloud_layer->size(); t++)
        {
            double dist_pt_to_line = (cloud_layer->points[t].y + m*cloud_layer->points[t].z + b)/sqrt(1+m*m);
            //std::cout<<t<<": "<<cloud_layer->points[t].y<<" "<<cloud_layer->points[t].z<<" , distance: "<<dist_pt_to_line<<std::endl;
            if (std::abs(dist_pt_to_line)<0.15)
            {
                cloud_return->push_back(cloud_layer->points[t]);
            }
        }
        return cloud_return;
    }

    std::vector<double> PCL_searchcurve::ransac_estimate_line(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in)
    {
        /************* ransac here *******************/

        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_LINE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.05);

        seg.setInputCloud (cloud_in);
        seg.segment (*inliers, *coefficients);

//        std::cout<<"coefficient size: "<<coefficients->values.size()<<" "<<coefficients->values[0]<<" "<<coefficients->values[1]
//                <<" "<<coefficients->values[2]<<" "<<coefficients->values[3]<<" "<<coefficients->values[4]<<" "<<coefficients->values[5]<<" "<<std::endl;

        pcl::PointXYZ ransac_one, ransac_two;
        ransac_one.x = coefficients->values[0];
        ransac_one.y = coefficients->values[1];
        ransac_one.z = coefficients->values[2];

        ransac_two.x = coefficients->values[0] + coefficients->values[3];
        ransac_two.y = coefficients->values[1] + coefficients->values[4];
        ransac_two.z = coefficients->values[2] + coefficients->values[5];

        double m_ransac, b_ransac;
        m_ransac = (ransac_one.y-ransac_two.y)/(ransac_one.z - ransac_two.z);
        b_ransac = ransac_one.y - m_ransac*ransac_one.z;        

        std::vector<double> line_coeff;
        line_coeff.push_back(b_ransac);
        line_coeff.push_back(m_ransac);

        return line_coeff;

    }

	void PCL_searchcurve::pointcloud_cb_ (const sensor_msgs::PointCloud2ConstPtr& input)
	{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_transform (new pcl::PointCloud<pcl::PointXYZI>);
	    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_search (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plot (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ransac (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_store (new pcl::PointCloud<pcl::PointXYZI>);

	    pcl::fromROSMsg (*input, *cloud_in);


        /***************************** mapping purpose ********************************/
        if (map_)
        {

            double prev_x, prev_y, prev_yaw, future_x, future_y, future_yaw, interpolate_x, interpolate_y, interpolate_yaw;

            long double header_time = input->header.stamp.toSec();

            for (int j=0; j<pose_time_->size()-1; j++)
            {
                if (header_time>pose_time_->at(j))
                {
                    if (j==pose_time_->size())
                    {
                        std::cout<<"point cloud timestamp is further than the pose timestamp"<<std::endl;
                        return;
                    }
                    else if (header_time<pose_time_->at(j+1))
                    {
                        //std::cout<<pose_time_->at(j)<<" "<<header_time<<" "<<pose_time_->at(j+1)<<std::endl;


                        prev_x = pose_x_->at(j);
                        prev_y = pose_y_->at(j);
                        prev_yaw = pose_yaw_->at(j);
                        future_x = pose_x_->at(j+1);
                        future_y = pose_y_->at(j+1);
                        future_yaw = pose_yaw_->at(j+1);

                        if (future_yaw-prev_yaw>180.0)
                        {
                            prev_yaw = prev_yaw + 360.0;
                        }
                        else if (future_yaw-prev_yaw<-180.0)
                        {
                            future_yaw = future_yaw + 360.0;
                        }

                        interpolate_x = interpolate(pose_time_->at(j), pose_time_->at(j+1), header_time, prev_x, future_x);
                        interpolate_y = interpolate(pose_time_->at(j), pose_time_->at(j+1), header_time, prev_y, future_y);
                        interpolate_yaw = interpolate(pose_time_->at(j), pose_time_->at(j+1), header_time, prev_yaw, future_yaw);


                        interpolate_yaw = interpolate_yaw * 3.1415926/180.0;

                        break;
                    }
                }
            }
            /*************************************************************************************/


            //std::cout<<interpolate_x<<" "<<interpolate_y<<std::endl;

            transform_.setOrigin(tf::Vector3(interpolate_x, interpolate_y, 0.0));
            tf::Quaternion heading;
            heading.setRPY(0.0, 0.0, interpolate_yaw);
            transform_.setRotation(heading);

        }

        std::vector<double> valid_x;
        std::vector<double> valid_y;
        std::vector<double> valid_z;

        double z_bar = 0.0;
        double y_bar = 0.0;
        int count_pt = 0;        

        int layer_count =0;

//        visualization_msgs::Marker line_marker_;
//        line_marker_.header = input->header;
//        line_marker_.id = 0;
//        line_marker_.type = visualization_msgs::Marker::LINE_STRIP;
//        line_marker_.action = visualization_msgs::Marker::ADD;

//        line_marker_.scale.x = 0.1;
//        line_marker_.scale.y = 0.1;
//        line_marker_.scale.z = 0.1;
//        line_marker_.color.a = 1.0; // Don't forget to set the alpha!
//        line_marker_.color.r = 0.0;
//        line_marker_.color.g = 1.0;
//        line_marker_.color.b = 0.0;

//        pcl::PointXYZ point_zero; // first point for every layer

        for (int i=0; i<cloud_in->size(); i++)
        {
            int layer = i/900;
            int color = 16*int(layer);
            double pt_x = cloud_in->points[i].x;
            double pt_y = cloud_in->points[i].y;
            double pt_z = cloud_in->points[i].z;

            double xy = sqrt(pt_x*pt_x+pt_y*pt_y);           


            if (xy<90.0)
            {
                if (cloud_in->points[i].x==0.0||cloud_in->points[i-1].x==0.0)
                {

                }
                else
                {

//                    if (i%900==0)
//                    {
//                        point_zero.x = pt_x;
//                        point_zero.y = pt_y;
//                        point_zero.z = pt_z;
//                    }


                    pcl::PointXYZI color_pt;
                    color_pt.x = cloud_in->points[i].x;
                    color_pt.y = cloud_in->points[i].y;
                    color_pt.z = cloud_in->points[i].z;
                    color_pt.intensity = color;

                    cloud_transform->push_back(color_pt);


                    if (layer==layer_count)
                    {

                        pcl::PointXYZI z_pt;
                        z_pt.x = 0.0;
                        z_pt.y = cloud_in->points[i].y;
                        z_pt.z = cloud_in->points[i].z;
                        z_pt.intensity = color;
                        //cloud_search->push_back(z_pt);

                        valid_x.push_back(pt_x);
                        valid_y.push_back(pt_y);
                        valid_z.push_back(pt_z);
                        z_bar = z_bar + pt_z;
                        y_bar = y_bar + pt_y;

                        //pcl::PointXYZ

                        cloud_store->push_back(color_pt);
                        cloud_ransac->push_back(z_pt);

                        count_pt++;

//                        double dz = cloud_in->points[i].z - cloud_in->points[i-1].z;
//                        double dz_dy = dz/(pt_y - cloud_in->points[i-1].y);

                    }
                    else
                    {
                        if (layer<11)
                        {
                            layer_count++;

                            std::vector<double> b_and_m;
                            b_and_m = ransac_estimate_line(cloud_ransac);
                            //std::cout<<"cloud size: "<<cloud_ransac->size()<<" "<<layer<<": "<<b_and_m[0]<<" "<<b_and_m[1]<<std::endl;

                            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane;
                            cloud_plane = extract_groundpoint (cloud_store, b_and_m[0], b_and_m[1]);
                            *cloud_search += *cloud_plane;

                            geometry_msgs::Point line_pt;
                            geometry_msgs::PoseStamped line_path;

                            line_pt.x = cloud_store->points[0].x;
                            line_pt.y = -5.0;
                            line_pt.z = (-5.0-b_and_m[0])/b_and_m[1];

                            //line_marker_.points.push_back(line_pt);
                            line_path.pose.position = line_pt;
                            ransac_line_.poses.push_back(line_path);

                            line_pt.x = cloud_store->points[0].x;
                            line_pt.y = 5.0;
                            line_pt.z = (5.0-b_and_m[0])/b_and_m[1];

                            //line_marker_.points.push_back(line_pt);
                            line_path.pose.position = line_pt;
                            ransac_line_.poses.push_back(line_path);

                            b_and_m.clear();

                        }
                        cloud_ransac.reset(new pcl::PointCloud<pcl::PointXYZI>);
                        cloud_store.reset(new pcl::PointCloud<pcl::PointXYZI>);

                    }


                }
            }


        }

        ransac_line_.header = input->header;
        pub_ransac_line_.publish(ransac_line_);
        ransac_line_.poses.clear();



        /***** publish point cloud *********/
        sensor_msgs::PointCloud2 output, output2, output_map, output_plot;
        pcl::toROSMsg(*cloud_transform, output);
        output.header = input->header;
        pub_pcl_.publish(output);

        pcl::toROSMsg(*cloud_search, output2);
        output2.header = input->header;
        pub_search_.publish(output2);

        pcl::toROSMsg(*cloud_plot, output_plot);
        output_plot.header = input->header;
        pub_plot_.publish(output_plot);


        /******** publish map *******/
        if (map_)
        {
            pcl_ros::transformPointCloud(*cloud_transform, *cloud_transform, transform_);

            *cloud_map += *cloud_transform;

            voxel_filter_.setInputCloud(cloud_map);
            voxel_filter_.filter(*cloud_map);

            pcl::toROSMsg(*cloud_map, output_map);
            output_map.header = input->header;
            pub_map_.publish(output_map);
        }



        /******** publish transform *********/
        geometry_msgs::TransformStamped velodyne_baselink;
        velodyne_baselink.header.stamp = input->header.stamp;
        velodyne_baselink.header.frame_id = "base_link";
        velodyne_baselink.child_frame_id = "velodyne_bottom";

        velodyne_baselink.transform.translation.x = 0.0;
        velodyne_baselink.transform.translation.y = 0.0;
        velodyne_baselink.transform.translation.z = 0.0;
        velodyne_baselink.transform.rotation.x = 0.0;
        velodyne_baselink.transform.rotation.y = 0.0;
        velodyne_baselink.transform.rotation.z = 0.0;
        velodyne_baselink.transform.rotation.w = 1.0;

        tfb_->sendTransform(velodyne_baselink);


    }



}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pointcloud_searchcurve");
    
    pointcloud_filter::PCL_searchcurve pcl_searchcurve;
    // Spin
    ros::spin ();

    return 0;
}
