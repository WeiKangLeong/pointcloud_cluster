/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

// roscpp
#include "ros/ros.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/PointStamped.h>

#include <nav_msgs/OccupancyGrid.h>

// For pcl format
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PointIndices.h>
#include <pcl/io/pcd_io.h>

// ransac
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "dbscan.h"

#include <fstream>
#include <vector>
#include <iostream>

class position_clustering
{
public:
    position_clustering();

private:
    //callback
    void pointcloud_cb_ (const sensor_msgs::PointCloud2ConstPtr& input);
    void point_cb_ (const geometry_msgs::PointStampedConstPtr& pt_in);

    ros::NodeHandle nh_;

    ros::Subscriber pointcloud_input_;
    ros::Subscriber point_input_;

    ros::Publisher map_pub_;
    ros::Publisher first_layer_;
    ros::Publisher second_layer_;
    ros::Publisher third_layer_;
    ros::Publisher fourth_layer_;
    ros::Publisher fifth_layer_;
    ros::Publisher sixth_layer_;
    ros::Publisher seventh_layer_;
    ros::Publisher eighth_layer_;
    ros::Publisher ninth_layer_;
    ros::Publisher tenth_layer_;
    ros::Publisher eleventh_layer_;
    ros::Publisher twelve_layer_;
    ros::Publisher thirteen_layer_;
    ros::Publisher fourteen_layer_;
    ros::Publisher fifteen_layer_;
    ros::Publisher sixteen_layer_;
    ros::Publisher seventeen_layer_;
    ros::Publisher eighteen_layer_;
    ros::Publisher nineteen_layer_;
    ros::Publisher twenty_layer_;
    ros::Publisher twentyone_layer_;
    ros::Publisher twentytwo_layer_;
    ros::Publisher twentythree_layer_;
    ros::Publisher twentyfour_layer_;
    ros::Publisher twentyfive_layer_;
    ros::Publisher twentysix_layer_;
    ros::Publisher twentyseven_layer_;
    ros::Publisher twentyeight_layer_;
    ros::Publisher twentynine_layer_;
    ros::Publisher thirty_layer_;
    ros::Publisher thirtyone_layer_;
    ros::Publisher thirtytwo_layer_;

    ros::Publisher ransac_output_;
    ros::Publisher ransac_reject_;

    ros::Publisher find_relation_;

    pcl::PassThrough<pcl::PointXYZI> pass;
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree;

    pcl::PointXYZI pt_1_, pt_2_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr save_cloud_;

    bool got_one_point_;

    double detect_x_, detect_y_, resolution_;
    int total_grid_;
    double density_;
    std::string text_file_;

    std::vector<std::vector<int>* >* grid_table_;
    std::vector<int>* chess_plate_;
    std::vector<std::vector<int>* >* cluster_table_;

    std::vector<int> layer_table_;

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>* cloud_table_;

    nav_msgs::OccupancyGrid map_;

    DBSCAN* dbscan;    

    // function to support
    void clear_grid();
    pcl::PointCloud<pcl::PointXYZI>::Ptr check_circle(pcl::PointCloud<pcl::PointXYZI>::Ptr layer_input);
    double check_distance(pcl::PointXYZI pt_one, pcl::PointXYZI pt_two);
    void check_pt_number(pcl::PointXYZI pt);
    double check_next_pt(pcl::PointXYZI pt_one, pcl::PointXYZI pt_two);

};

int main (int argc, char** argv)
{
    ros::init (argc, argv, "position_cluster");

    position_clustering use_pose;

    ros::spin();

    return(0);
}

position_clustering::position_clustering()
{    
    pointcloud_input_ = nh_.subscribe<sensor_msgs::PointCloud2> ("cloud_input", 1, &position_clustering::pointcloud_cb_, this);
    point_input_ = nh_.subscribe<geometry_msgs::PointStamped> ("clicked_point", 1, &position_clustering::point_cb_, this);

    ransac_output_ = nh_.advertise<sensor_msgs::PointCloud2> ("ransac_output", 1);
    ransac_reject_ = nh_.advertise<sensor_msgs::PointCloud2> ("ransac_reject", 1);

    find_relation_ = nh_.advertise<sensor_msgs::PointCloud2> ("find_relation", 1);

    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid> ("grid_map", 1);
    first_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer1", 1);
    second_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer2", 1);
    third_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer3", 1);
    fourth_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer4", 1);
    fifth_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer5", 1);
    sixth_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer6", 1);
    seventh_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer7", 1);
    eighth_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer8", 1);
    ninth_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer9", 1);
    tenth_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer10", 1);
    eleventh_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer11", 1);
    twelve_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer12", 1);
    thirteen_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer13", 1);
    fourteen_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer14", 1);
    fifteen_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer15", 1);
    sixteen_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer16", 1);
    seventeen_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer17", 1);
    eighteen_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer18", 1);
    nineteen_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer19", 1);
    twenty_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer20", 1);
    twentyone_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer21", 1);
    twentytwo_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer22", 1);
    twentythree_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer23", 1);
    twentyfour_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer24", 1);
    twentyfive_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer25", 1);
    twentysix_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer26", 1);
    twentyseven_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer27", 1);
    twentyeight_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer28", 1);
    twentynine_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer29", 1);
    thirty_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer30", 1);
    thirtyone_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer31", 1);
    thirtytwo_layer_ = nh_.advertise<sensor_msgs::PointCloud2> ("layer32", 1);

    detect_x_ = 30.0;
    detect_y_ = 30.0;
    resolution_ = 0.1;
    density_ = 0.1;

    ros::NodeHandle priv_nh_("~");
    priv_nh_.getParam("detect_x", detect_x_);
    priv_nh_.getParam("detect_y", detect_y_);
    priv_nh_.getParam("resolution", resolution_);
    priv_nh_.getParam("density", density_);
    priv_nh_.getParam("sequence_text_file", text_file_);

    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_CIRCLE2D);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (1.5);

    // initialize kd tree
    tree.reset(new pcl::search::KdTree<pcl::PointXYZI>);

    // read sequence of layer from text file
    std::string line;
    std::ifstream myfile;
    myfile.open(text_file_.c_str(), std::ios::app);
    if (myfile.is_open())
    {
        while ( getline (myfile,line) )
        {
          layer_table_.push_back(std::atoi(line.c_str()));
        }
        myfile.close();
        std::cout<<layer_table_.size()<<" layers has been recorded."<<std::endl;
    }

    else std::cout << "Unable to open file"<<std::endl;

    save_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    got_one_point_ = false;

    cloud_table_ = new std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr >;
    cloud_table_->resize(32);
    for (int g=0; g<32; g++)
    {
        cloud_table_->at(g).reset(new pcl::PointCloud<pcl::PointXYZI>);
    }


}

void position_clustering::pointcloud_cb_(const sensor_msgs::PointCloud2ConstPtr &input)
{
    pcl::console::TicToc time;
    time.tic ();

    pcl::PointCloud<pcl::PointXYZI>::Ptr first_input (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr adjusted_input (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr compressed_input (new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<pcl::PointCloud<pcl::PointXYZI> >* thirty_two_pcl;
    thirty_two_pcl = new std::vector<pcl::PointCloud<pcl::PointXYZI> >;
    thirty_two_pcl->resize(32);

    std::vector<pcl::PointCloud<pcl::PointXYZI> >* cluster_centroid;
    cluster_centroid = new std::vector<pcl::PointCloud<pcl::PointXYZI> >;

    std::vector<pcl::PointCloud<pcl::PointXYZI> > organised_cluster;

    pcl::fromROSMsg (*input, *first_input);

    pcl::fromROSMsg (*input, *save_cloud_);

    std::cout<<"size of cloud: "<<first_input->size()<<std::endl;

    int nan_pt = 0;
    int num_pt = 0;
    double distance_travelled = 0.0;

    for (int i=0; i<layer_table_.size(); i++)
    {
        for (int j=0; j<2028; j++)
        {
            pcl::PointXYZI pushy_pt;
            pushy_pt = first_input->points[j+2028*(layer_table_.at(i)-1)];
            if (pcl::isFinite(pushy_pt))
            {
                cloud_table_->at(i)->push_back(pushy_pt);
                num_pt++;
            }
        }
    }

    std::cout<<"number of valid points: "<<num_pt<<std::endl;
    int cluster_number = 0;

    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI> > > total_cluster;
    total_cluster.resize(32);
    std::vector<pcl::PointCloud<pcl::PointXYZI> > nn_cluster_;

    for (int fff=0; fff<layer_table_.size(); fff++)
    {
        for (int f=0; f<cloud_table_->at(fff)->size(); f++)
        {
            pcl::PointXYZI pt1, pt2;
            pt1 = cloud_table_->at(fff)->points[f];
            pt2 = cloud_table_->at(fff)->points[f+1];

            double howfar = check_next_pt(pt1, pt2);


            if (howfar<0.2)
            {
                //std::cout<<howfar<<std::endl;
                compressed_input->push_back(pt1);
                cluster_number++;
            }
            else
            {
                if (cluster_number>5)
                {
                    nn_cluster_.push_back(*compressed_input);
                    compressed_input->clear();
                    cluster_number=0;
                }
                else
                {
                    compressed_input->clear();
                    cluster_number=0;
                }
            }

        }
        if (cluster_number>5)
        {
            nn_cluster_.push_back(*compressed_input);
            compressed_input->clear();
            cluster_number=0;
        }

        pcl::PointCloud<pcl::PointXYZI> temp_cluster_centroid;
        pcl::PointXYZI temp_centroid;



        for (int p=0; p<nn_cluster_.size(); p++)
        {
            int cluster_size = 0;
            temp_centroid.x = 0.0;
            temp_centroid.y = 0.0;
            for (int pp=0; pp<nn_cluster_.at(p).size(); pp++)
            {
                pcl::PointXYZI db_pt;
                db_pt = nn_cluster_.at(p).points[pp];
                db_pt.intensity = p*10;
                thirty_two_pcl->at(fff).push_back(db_pt);

                temp_centroid.x += db_pt.x;
                temp_centroid.y += db_pt.y;
                cluster_size++;


            }

            temp_centroid.x = temp_centroid.x/cluster_size;
            temp_centroid.y = temp_centroid.y/cluster_size;
            temp_cluster_centroid.push_back(temp_centroid);

        }
        cluster_centroid->push_back(temp_cluster_centroid);

        //std::cout<<"centroid found: "<<temp_cluster_centroid.size()<<std::endl;
        //std::cout<<"Layer: "<<fff<<" we found "<<nn_cluster_.size()<<" clusters with "<<thirty_two_pcl->at(fff).size()<<" points."<<std::endl;
        total_cluster.at(fff)=(nn_cluster_);
        nn_cluster_.clear();
    }

    int just_count=0;
    int points_in_total_cluster=0;
    for (int t=0; t<total_cluster.size(); t++)
    {
        for (int t_c=0; t_c<total_cluster.at(t).size(); t_c++)
        {
            just_count++;
            points_in_total_cluster += total_cluster.at(t).at(t_c).size();
        }
    }
    std::cout<<"found "<<just_count<<" clusters and "<<points_in_total_cluster<<" points in total"<<std::endl;

    std::vector<std::vector<int> > empty_table;
    empty_table.resize(32);
    for (int e=0; e<32; e++)
    {
        empty_table.at(e).resize(cluster_centroid->at(e).size());
        for(int et=0; et<empty_table.at(e).size(); et++)
        {
            empty_table.at(e).at(et) = -1;
        }
    }

    int nearest_k=1;

    for (int mb=1; mb<cluster_centroid->size(); mb++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp_input (new pcl::PointCloud<pcl::PointXYZI>);
        *temp_input = cluster_centroid->at(mb);
        std::cout<<mb<<" centroid found: "<<temp_input->size()<<std::endl;
        if (temp_input->size()>0)
        {
            kdtree.setInputCloud(temp_input);
            for (int nmb = 0; nmb<cluster_centroid->at(mb-1).size(); nmb++)
            {
                std::vector<int> nearest_index(nearest_k);
                std::vector<float> nearest_distance(nearest_k);
                pcl::PointXYZI search_pt = cluster_centroid->at(mb-1).points[nmb];
                if (kdtree.nearestKSearch(search_pt, nearest_k, nearest_index, nearest_distance)>0)
                {
                    if (nearest_distance.at(0)<0.25)
                    {
                        empty_table.at(mb-1).at(nmb) = nearest_index.at(0);
                    }
                }
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZI> temp_cluster;
    int cluster_size = 0;
    for (int h=0; h<empty_table.size()-1; h++)
    {
        for (int g=0; g<empty_table.at(h).size(); g++)
        {
            int pointing_to=empty_table.at(h).at(g);
            if (pointing_to>-1)
            {
                cluster_size++;
                int cluster_growing=0;
                temp_cluster += total_cluster.at(h).at(g);
                while (pointing_to>-1)
                {
                    cluster_growing++;
                    temp_cluster += total_cluster.at(h+cluster_growing).at(pointing_to);
                    pointing_to = empty_table.at(h+cluster_growing).at(pointing_to);
                    empty_table.at(h+cluster_growing).at(pointing_to) = -1;
                }

            }
        }
    }


/*


    std::vector<std::vector<std::vector<int, int> > > cluster_control;
    std::vector<std::vector<int, int> > cluster_group;
    std::vector<int, int> cluster_index;

    int group_number = 0;
    int layer_away = 0;
    for (int a=0; a<empty_table.size(); a--)
    {
        for (int b=0; b<empty_table.at(a).size(); b++)
        {
            if (empty_table.at(a).at(b)>-1)
            {
                layer_away++;
                cluster_index.at(0) = a;
                cluster_index.at(1) = b;
                int finding_index=empty_table.at(a).at(b);
                cluster_group.push_back(cluster_index);
                int clustering = empty_table.at(a+layer_away).at(finding_index);
                while (clustering>-1)
                {
                    cluster_index.at(0) = a+layer_away;
                    cluster_index.at(1) = clustering;
                    finding_index = empty_table.at(a+layer_away).at(finding_index);

                    for (int c=0; c<empty_table.at(a+layer_away).at(finding_index); c++)
                    {
                        cluster_index.at(0) =
                    }
                }
            }
        }
    }


    std::cout<<"Total cluster found: "<<organised_cluster.size()<< " with "<<accumulated_pt<<std::endl;

    for (int cluster=0; cluster<organised_cluster.size(); cluster++)
    {
        for (int point=0; point<organised_cluster.at(cluster).size(); point++)
        {
            adjusted_input->push_back(organised_cluster.at(cluster).points[point]);
        }
    }
*/

/*    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>* dbscan_cluster_;
    dbscan_cluster_ = new std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>;

    dbscan_cluster_ = dbscan->dbscan_cluster_threshold(cloud_table_->at(5), 0.1);

    std::cout<<"in layer: "<<"5"<<" we found "<<dbscan_cluster_->size()<<std::endl;

    for (int p=0; p<dbscan_cluster_->size(); p++)
    {
        for (int pp=0; pp<dbscan_cluster_->at(p)->size(); pp++)
        {
            pcl::PointXYZI db_pt;
            db_pt = dbscan_cluster_->at(p)->at(pp);
            db_pt.intensity = p*10;
            adjusted_input->push_back(db_pt);
        }
    }
*/

//    for (int i=0; i<layer_table_.size(); i++)
//    {
//        for (int j=0; j<2027; j++)
//        {
//            if (pcl::isFinite(first_input->points[j+2028*(layer_table_.at(i)-1)])&&pcl::isFinite(first_input->points[j+1+2028*(layer_table_.at(i)-1)]))
//            {
//                double pt_pt = check_next_pt(first_input->points[j+2028*(layer_table_.at(i)-1)], first_input->points[j+1+2028*(layer_table_.at(i)-1)]);
//            }
//        }
//    }

    //for (int i=0; i<layer_table_.size(); i++)
 /*   for (int i=0; i<0; i++)
    {
        for (int j=0; j<2028; j++)
        {
            pcl::PointXYZI target_pt;
            target_pt = first_input->points[j+2028*int(layer_table_.at(i)-1)];
            if (pcl::isFinite(target_pt))
            {
                if (i==0)
                {
                    pcl::PointXYZI nearby_pt;
                    nearby_pt = first_input->points[j+2028*int(layer_table_.at(i+1)-1)];
                    if (pcl::isFinite(nearby_pt))
                    {
                        double pt_diff = check_distance(target_pt,nearby_pt);
                        //std::cout<< pt_diff<<std::endl;0
                        if (pt_diff<0.5)
                        {
                            compressed_input->push_back(target_pt);
                            std::cout<< pt_diff<<std::endl;
                        }
                    }

                }
                else if (i==31)
                {
                    pcl::PointXYZI nearby_pt;
                    nearby_pt = first_input->points[j+2028*int(layer_table_.at(i-1)-1)];
                    if (pcl::isFinite(nearby_pt))
                    {
                        double pt_diff = check_distance(target_pt,nearby_pt);
                        //std::cout<< pt_diff<<std::endl;0
                        if (pt_diff<0.5)
                        {
                            compressed_input->push_back(target_pt);
                            std::cout<< pt_diff<<std::endl;
                        }
                    }
                }
                else
                {
                    pcl::PointXYZI nearby_pt;
                    nearby_pt = first_input->points[j+2028*int(layer_table_.at(i+1)-1)];
                    if (pcl::isFinite(nearby_pt))
                    {
                        double pt_diff = check_distance(target_pt,nearby_pt);
                        //std::cout<< pt_diff<<std::endl;0
                        if (pt_diff<0.5)
                        {
                            compressed_input->push_back(target_pt);
                            std::cout<< pt_diff<<std::endl;
                        }
                    }
                }
            }
        }
    }*/

/*    for (int k=40560; k<41560; k++)
    {
        layer20->push_back(first_input->points[k]);
    }

    for (int k=44616; k<45616; k++)
    {
        layer22->push_back(first_input->points[k]);
    }

    for (int k=36504; k<37504; k++)
    {
        layer18->push_back(first_input->points[k]);
    }
*/

/*
    for (int k=0; k<first_input->size(); k++)
    {
        if (pcl::isFinite(first_input->points[k]))
        {
            nan_pt++;
        }
        if (k<2028)
        {
            layer1->push_back(first_input->points[k]);
        }
        else if (k<2028*2)
        {
            layer2->push_back(first_input->points[k]);
        }
        else if (k<2028*3)
        {
            layer3->push_back(first_input->points[k]);
        }
        else if (k<2028*4)
        {
            layer4->push_back(first_input->points[k]);
        }
        else if (k<2028*5)
        {
            layer5->push_back(first_input->points[k]);
        }
        else if (k<2028*6)
        {
            layer6->push_back(first_input->points[k]);
        }
        else if (k<2028*7)
        {
            layer7->push_back(first_input->points[k]);
        }
        else if (k<2028*8)
        {
            layer8->push_back(first_input->points[k]);
        }
        else if (k<2028*9)
        {
            layer9->push_back(first_input->points[k]);
        }
        else if (k<2028*10)
        {
            layer10->push_back(first_input->points[k]);
        }
        else if (k<2028*11)
        {
            layer11->push_back(first_input->points[k]);
        }
        else if (k<2028*12)
        {
            layer12->push_back(first_input->points[k]);
        }
        else if (k<2028*13)
        {
            layer13->push_back(first_input->points[k]);
        }
        else if (k<2028*14)
        {
            layer14->push_back(first_input->points[k]);
        }
        else if (k<2028*15)
        {
            layer15->push_back(first_input->points[k]);
        }
        else if (k<2028*16)
        {
            layer16->push_back(first_input->points[k]);
        }
        else if (k<2028*17)
        {
            layer17->push_back(first_input->points[k]);
        }
        else if (k<2028*18)
        {
            layer18->push_back(first_input->points[k]);
        }
        else if (k<2028*19)
        {
            layer19->push_back(first_input->points[k]);
        }
        else if (k<2028*20)
        {
//            if (pcl::isFinite(first_input->points[k]) && pcl::isFinite(first_input->points[k+4056]))
//            {
//                double pt_dist = check_distance(first_input->points[k], first_input->points[k+4056]);
//                if (pt_dist<1.0)
//                {
                    layer20->push_back(first_input->points[k]);
//                }
//                distance_travelled = distance_travelled+pt_dist;
//                num_pt++;
//            }

        }
        else if (k<2028*21)
        {
            layer21->push_back(first_input->points[k]);
        }
        else if (k<2028*22)
        {
            layer22->push_back(first_input->points[k]);
        }
        else if (k<2028*23)
        {
            layer23->push_back(first_input->points[k]);
        }
        else if (k<2028*24)
        {
            layer24->push_back(first_input->points[k]);
        }
        else if (k<2028*25)
        {
            layer25->push_back(first_input->points[k]);
        }
        else if (k<2028*26)
        {
            layer26->push_back(first_input->points[k]);
        }
        else if (k<2028*27)
        {
            layer27->push_back(first_input->points[k]);
        }
        else if (k<2028*28)
        {
            layer28->push_back(first_input->points[k]);
        }
        else if (k<2028*29)
        {
            layer29->push_back(first_input->points[k]);
        }
        else if (k<2028*30)
        {
            layer30->push_back(first_input->points[k]);
        }
        else if (k<2028*31)
        {
            layer31->push_back(first_input->points[k]);
        }
        else if (k<2028*32)
        {
            layer32->push_back(first_input->points[k]);
        }

    }
*/


/*    sensor_msgs::PointCloud2 cloud1,cloud2,cloud3,cloud4,cloud5,cloud6,cloud7,cloud8,cloud9,cloud10,cloud11,cloud12,cloud13,cloud14,
            cloud15,cloud16,cloud17,cloud18,cloud19,cloud20,cloud21,cloud22,cloud23,cloud24,cloud25,cloud26,cloud27,cloud28,cloud29,
            cloud30,cloud31,cloud32;
    pcl::toROSMsg (*cloud_table_->at(0), cloud1);
    cloud1.header = input->header;
    first_layer_.publish(cloud1);
    pcl::toROSMsg (*cloud_table_->at(1), cloud2);
    cloud2.header = input->header;
    second_layer_.publish(cloud2);
    pcl::toROSMsg (*cloud_table_->at(2), cloud3);
    cloud3.header = input->header;
    third_layer_.publish(cloud3);
    pcl::toROSMsg (*cloud_table_->at(3), cloud4);
    cloud4.header = input->header;
    fourth_layer_.publish(cloud4);
    pcl::toROSMsg (*cloud_table_->at(4), cloud5);
    cloud5.header = input->header;
    fifth_layer_.publish(cloud5);
    pcl::toROSMsg (*cloud_table_->at(5), cloud6);
    cloud6.header = input->header;
    sixth_layer_.publish(cloud6);
    pcl::toROSMsg (*cloud_table_->at(6), cloud7);
    cloud7.header = input->header;
    seventh_layer_.publish(cloud7);
    pcl::toROSMsg (*cloud_table_->at(7), cloud8);
    cloud8.header = input->header;
    eighth_layer_.publish(cloud8);
    pcl::toROSMsg (*cloud_table_->at(8), cloud9);
    cloud9.header = input->header;
    ninth_layer_.publish(cloud9);
    pcl::toROSMsg (*cloud_table_->at(9), cloud10);
    cloud10.header = input->header;
    tenth_layer_.publish(cloud10);
    pcl::toROSMsg (*cloud_table_->at(10), cloud11);
    cloud11.header = input->header;
    eleventh_layer_.publish(cloud11);
    pcl::toROSMsg (*cloud_table_->at(11), cloud12);
    cloud12.header = input->header;
    twelve_layer_.publish(cloud12);
    pcl::toROSMsg (*cloud_table_->at(12), cloud13);
    cloud13.header = input->header;
    thirteen_layer_.publish(cloud13);
    pcl::toROSMsg (*cloud_table_->at(13), cloud14);
    cloud14.header = input->header;
    fourteen_layer_.publish(cloud14);
    pcl::toROSMsg (*cloud_table_->at(14), cloud15);
    cloud15.header = input->header;
    fifteen_layer_.publish(cloud15);
    pcl::toROSMsg (*cloud_table_->at(15), cloud16);
    cloud16.header = input->header;
    sixteen_layer_.publish(cloud16);
    pcl::toROSMsg (*cloud_table_->at(16), cloud17);
    cloud17.header = input->header;
    seventeen_layer_.publish(cloud17);
    pcl::toROSMsg (*cloud_table_->at(17), cloud18);
    cloud18.header = input->header;
    eighteen_layer_.publish(cloud18);
    pcl::toROSMsg (*cloud_table_->at(18), cloud19);
    cloud19.header = input->header;
    nineteen_layer_.publish(cloud19);
    pcl::toROSMsg (*cloud_table_->at(19), cloud20);
    cloud20.header = input->header;
    twenty_layer_.publish(cloud20);
    pcl::toROSMsg (*cloud_table_->at(20), cloud21);
    cloud21.header = input->header;
    twentyone_layer_.publish(cloud21);
    pcl::toROSMsg (*cloud_table_->at(21), cloud22);
    cloud22.header = input->header;
    twentytwo_layer_.publish(cloud22);
    pcl::toROSMsg (*cloud_table_->at(22), cloud23);
    cloud23.header = input->header;
    twentythree_layer_.publish(cloud23);
    pcl::toROSMsg (*cloud_table_->at(23), cloud24);
    cloud24.header = input->header;
    twentyfour_layer_.publish(cloud24);
    pcl::toROSMsg (*cloud_table_->at(24), cloud25);
    cloud25.header = input->header;
    twentyfive_layer_.publish(cloud25);
    pcl::toROSMsg (*cloud_table_->at(25), cloud26);
    cloud26.header = input->header;
    twentysix_layer_.publish(cloud26);
    pcl::toROSMsg (*cloud_table_->at(26), cloud27);
    cloud27.header = input->header;
    twentyseven_layer_.publish(cloud27);
    pcl::toROSMsg (*cloud_table_->at(27), cloud28);
    cloud28.header = input->header;
    twentyeight_layer_.publish(cloud28);
    pcl::toROSMsg (*cloud_table_->at(28), cloud29);
    cloud29.header = input->header;
    twentynine_layer_.publish(cloud29);
    pcl::toROSMsg (*cloud_table_->at(29), cloud30);
    cloud30.header = input->header;
    thirty_layer_.publish(cloud30);
    pcl::toROSMsg (*cloud_table_->at(30), cloud31);
    cloud31.header = input->header;
    thirtyone_layer_.publish(cloud31);
    pcl::toROSMsg (*cloud_table_->at(31), cloud32);
    cloud32.header = input->header;
    thirtytwo_layer_.publish(cloud32);
*/
    sensor_msgs::PointCloud2 cloud1,cloud2,cloud3,cloud4,cloud5,cloud6,cloud7,cloud8,cloud9,cloud10,cloud11,cloud12,cloud13,cloud14,
            cloud15,cloud16,cloud17,cloud18,cloud19,cloud20,cloud21,cloud22,cloud23,cloud24,cloud25,cloud26,cloud27,cloud28,cloud29,
            cloud30,cloud31,cloud32;
    pcl::toROSMsg (thirty_two_pcl->at(0), cloud1);
    cloud1.header = input->header;
    first_layer_.publish(cloud1);
    pcl::toROSMsg (thirty_two_pcl->at(1), cloud2);
    cloud2.header = input->header;
    second_layer_.publish(cloud2);
    pcl::toROSMsg (thirty_two_pcl->at(2), cloud3);
    cloud3.header = input->header;
    third_layer_.publish(cloud3);
    pcl::toROSMsg (thirty_two_pcl->at(3), cloud4);
    cloud4.header = input->header;
    fourth_layer_.publish(cloud4);
    pcl::toROSMsg (thirty_two_pcl->at(4), cloud5);
    cloud5.header = input->header;
    fifth_layer_.publish(cloud5);
    pcl::toROSMsg (thirty_two_pcl->at(5), cloud6);
    cloud6.header = input->header;
    sixth_layer_.publish(cloud6);
    pcl::toROSMsg (thirty_two_pcl->at(6), cloud7);
    cloud7.header = input->header;
    seventh_layer_.publish(cloud7);
    pcl::toROSMsg (thirty_two_pcl->at(7), cloud8);
    cloud8.header = input->header;
    eighth_layer_.publish(cloud8);
    pcl::toROSMsg (thirty_two_pcl->at(8), cloud9);
    cloud9.header = input->header;
    ninth_layer_.publish(cloud9);
    pcl::toROSMsg (thirty_two_pcl->at(9), cloud10);
    cloud10.header = input->header;
    tenth_layer_.publish(cloud10);
    pcl::toROSMsg (thirty_two_pcl->at(10), cloud11);
    cloud11.header = input->header;
    eleventh_layer_.publish(cloud11);
    pcl::toROSMsg (thirty_two_pcl->at(11), cloud12);
    cloud12.header = input->header;
    twelve_layer_.publish(cloud12);
    pcl::toROSMsg (thirty_two_pcl->at(12), cloud13);
    cloud13.header = input->header;
    thirteen_layer_.publish(cloud13);
    pcl::toROSMsg (thirty_two_pcl->at(13), cloud14);
    cloud14.header = input->header;
    fourteen_layer_.publish(cloud14);
    pcl::toROSMsg (thirty_two_pcl->at(14), cloud15);
    cloud15.header = input->header;
    fifteen_layer_.publish(cloud15);
    pcl::toROSMsg (thirty_two_pcl->at(15), cloud16);
    cloud16.header = input->header;
    sixteen_layer_.publish(cloud16);
    pcl::toROSMsg (thirty_two_pcl->at(16), cloud17);
    cloud17.header = input->header;
    seventeen_layer_.publish(cloud17);
    pcl::toROSMsg (thirty_two_pcl->at(17), cloud18);
    cloud18.header = input->header;
    eighteen_layer_.publish(cloud18);
    pcl::toROSMsg (thirty_two_pcl->at(18), cloud19);
    cloud19.header = input->header;
    nineteen_layer_.publish(cloud19);
    pcl::toROSMsg (thirty_two_pcl->at(19), cloud20);
    cloud20.header = input->header;
    twenty_layer_.publish(cloud20);
    pcl::toROSMsg (thirty_two_pcl->at(20), cloud21);
    cloud21.header = input->header;
    twentyone_layer_.publish(cloud21);
    pcl::toROSMsg (thirty_two_pcl->at(21), cloud22);
    cloud22.header = input->header;
    twentytwo_layer_.publish(cloud22);
    pcl::toROSMsg (thirty_two_pcl->at(22), cloud23);
    cloud23.header = input->header;
    twentythree_layer_.publish(cloud23);
    pcl::toROSMsg (thirty_two_pcl->at(23), cloud24);
    cloud24.header = input->header;
    twentyfour_layer_.publish(cloud24);
    pcl::toROSMsg (thirty_two_pcl->at(24), cloud25);
    cloud25.header = input->header;
    twentyfive_layer_.publish(cloud25);
    pcl::toROSMsg (thirty_two_pcl->at(25), cloud26);
    cloud26.header = input->header;
    twentysix_layer_.publish(cloud26);
    pcl::toROSMsg (thirty_two_pcl->at(26), cloud27);
    cloud27.header = input->header;
    twentyseven_layer_.publish(cloud27);
    pcl::toROSMsg (thirty_two_pcl->at(27), cloud28);
    cloud28.header = input->header;
    twentyeight_layer_.publish(cloud28);
    pcl::toROSMsg (thirty_two_pcl->at(28), cloud29);
    cloud29.header = input->header;
    twentynine_layer_.publish(cloud29);
    pcl::toROSMsg (thirty_two_pcl->at(29), cloud30);
    cloud30.header = input->header;
    thirty_layer_.publish(cloud30);
    pcl::toROSMsg (thirty_two_pcl->at(30), cloud31);
    cloud31.header = input->header;
    thirtyone_layer_.publish(cloud31);
    pcl::toROSMsg (thirty_two_pcl->at(31), cloud32);
    cloud32.header = input->header;
    thirtytwo_layer_.publish(cloud32);

    sensor_msgs::PointCloud2 find_cloud;
    pcl::toROSMsg (*adjusted_input, find_cloud);
    find_cloud.header = input->header;
    find_relation_.publish(find_cloud);



//    std::cout<<"first layer: "<<layer1->size()<<std::endl;
//    std::cout<<"second layer: "<<layer2->size()<<std::endl;
//    std::cout<<"twenty layer: "<<layer20->size()<<std::endl;
//    std::cout<<"NaN points: "<<nan_pt<<std::endl;
    std::cout<<"New points: "<<adjusted_input->size()<<std::endl;
    std::cout<<"time taken: "<<time.toc()<<std::endl;
    //std::cout<<"average pt distance: "<<distance_travelled/num_pt<<std::endl;

    clear_grid();
    adjusted_input->clear();
}

pcl::PointCloud<pcl::PointXYZI>::Ptr position_clustering::check_circle(pcl::PointCloud<pcl::PointXYZI>::Ptr layer_input)
{
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    seg.setInputCloud (layer_input->makeShared ());
    seg.segment (*inliers, coefficients);

    pcl::PointCloud<pcl::PointXYZI>::Ptr global_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr reject_cloud (new pcl::PointCloud<pcl::PointXYZI>);

    for (size_t i = 0; i < inliers->indices.size (); ++i)
    {
        global_cloud->push_back(layer_input->points[inliers->indices[i]]);
    }

    std::cout<<"ransac size: "<<global_cloud->size()<<std::endl;

//    sensor_msgs::PointCloud2 ransac_cloud;
//    pcl::toROSMsg (*global_cloud, ransac_cloud);
//    ransac_cloud.header = input->header;
//    ransac_output_.publish(ransac_cloud);

    pcl::ExtractIndices<pcl::PointXYZI> extract;

    // Extract the inliers
    extract.setInputCloud (layer_input);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*reject_cloud);

//    sensor_msgs::PointCloud2 reject_ransac;
//    pcl::toROSMsg (*reject_cloud, reject_ransac);
//    reject_ransac.header = input->header;
//    ransac_reject_.publish(reject_ransac);
    return reject_cloud;
}

double position_clustering::check_distance(pcl::PointXYZI pt_one, pcl::PointXYZI pt_two)
{
    double d_x = pt_one.x-pt_two.x;
    double d_y = pt_one.y-pt_two.y;
    double d_z = pt_one.z-pt_two.z;
    double distance = sqrt(d_x*d_x+d_y*d_y+d_z*d_z);
    return distance;
}

double position_clustering::check_next_pt(pcl::PointXYZI pt_one, pcl::PointXYZI pt_two)
{
    double d_x = pt_one.x-pt_two.x;
    double d_y = pt_one.y-pt_two.y;
    double distance = sqrt(d_x*d_x+d_y*d_y);
    return distance;
}

void position_clustering::clear_grid()
{
    cloud_table_->clear();
    cloud_table_ = new std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr >;
    cloud_table_->resize(32);
    for (int g=0; g<32; g++)
    {
        cloud_table_->at(g).reset(new pcl::PointCloud<pcl::PointXYZI>);
    }
}

void position_clustering::check_pt_number(pcl::PointXYZI pt)
{
    //std::cout<<"checking with cloud size: "<<save_cloud_->size()<<std::endl;
    //std::cout<<"save cloud pt "<<save_cloud_->points[100].x<<" "<<save_cloud_->points[100].y<<std::endl;
    int count_cloud = 0;
    for (int t=0; t<save_cloud_->size(); t++)
    {
        if (pcl::isFinite(save_cloud_->points[t]))
        {
            double compute = check_distance(pt,save_cloud_->points[t]);
            //std::cout<<compute<<" ";
            if (compute<0.01)
            {
                std::cout<<"point number: "<<t<<std::endl;
            }
        }


    }std::cout<<std::endl;
    //std::cout<<"process "<<count_cloud<<" times."<<std::endl;
}

void position_clustering::point_cb_(const geometry_msgs::PointStampedConstPtr &pt_in)
{
    if (got_one_point_)
    {
        pt_2_.x = pt_in->point.x;
        pt_2_.y = pt_in->point.y;
        pt_2_.z = pt_in->point.z;
        got_one_point_ = false;
        check_pt_number(pt_2_);
        double hey = check_distance(pt_1_, pt_2_);
        std::cout<<hey<<std::endl;
    }
    else
    {
        got_one_point_ = true;
        pt_1_.x = pt_in->point.x;
        pt_1_.y = pt_in->point.y;
        pt_1_.z = pt_in->point.z;
        check_pt_number(pt_1_);
    }
}
