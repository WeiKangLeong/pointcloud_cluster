#ifndef DB_SCAN_H
#define DB_SCAN_H

class DBSCAN
{

public:

void db_point_select(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointXYZ the_chosen)
{
//    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//    kdtree.setInputCloud (input_cloud);
//    std::vector<int> NearSearch(1);
//    std::vector<float> NearSquaredDistance(1);
//    pcl::PointXYZ searching;
//    searching.x = the_chosen.x;
//    searching.y = the_chosen.y;
//    searching.z = the_chosen.z;
//    //std::cout<<" with x: "<<searching.x<<" y: "<<searching.y<<" z: "<<searching.z<<std::endl;
//    kdtree.nearestKSearch (searching, 1, NearSearch, NearSquaredDistance);
//    std::cout<<NearSearch[0]<<" with x: "<<searching.x<<" y: "<<searching.y<<" z: "<<searching.z<<std::endl;

    for (int i=0; i<input_cloud->size(); i++)
    {
        std::cout<<i<<": "<<input_cloud->points[i].x<<" "<<input_cloud->points[i].y<<" "<<input_cloud->points[i].z<<std::endl;
    }

    //int chosen_index = NearSearch[0];
    //return (chosen_index);
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>* dbscan_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_3(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_total(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>* cluster_cloud_;
    cluster_cloud_ = new std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>;

    std::vector<int>* cluster_1;
    std::vector<int>* cluster_2;
    std::vector<int>* cluster_3;
    std::vector<int>* cloudin_cluster;
    std::vector<int>* cluster_noise;
    std::vector<double>* cluster_center;
    cluster_1 = new std::vector<int>;
    cluster_2 = new std::vector<int>;
    cluster_3 = new std::vector<int>;
    cloudin_cluster = new std::vector<int>;
    cluster_noise = new std::vector<int>;
    cluster_center = new std::vector<double>;

    //cloudin_cluster->resize(cloudin->size());
    for (int i=0; i<cloudin->size(); i++)
    {
	cloudin_cluster->push_back(i);
        //std::cout<<cloudin_cluster->at(i)<<": "<<cloudin->points[i].x<<" "<<cloudin->points[i].y<<" "<<cloudin->points[i].z<<std::endl;
    }

    //std::cout<<"cloudin_cluster size: "<<cloudin_cluster->size()<<std::endl;

    cloud_3 = cloudin;

    int cloud_3_size=0;

    while (cloud_3->size()>0)
    {

        cloud_3_size=cloud_3->size();
        int new_cloud_size=cloud_3_size;
        //std::cout<<"new cloud3 size: "<<cloud_3->size()<<" and label size: "<<cloudin_cluster->size()<<std::endl;
        //cluster_1->push_back(0);
        for (int i=0; i<cloudin->size(); i++)
        {
            if (cloudin_cluster->at(i)>-1)
            {
                cluster_1->push_back(i);
                break;
            }
        }
        if (cluster_1->size()==0)
        {
            break;
        }

        cloud_1->push_back(cloudin->points[cluster_1->at(0)]);
        //std::cout<<"checking first point: "<<cluster_1->at(0)<<std::endl;
        //std::cout<<"x1: "<<cloud_1->points[0].x<<" y1: "<<cloud_1->points[0].y<<" z1: "<<cloud_1->points[0].z<<std::endl;
        cloud_2 = cloud_3;

        for (int i=0; i<cluster_1->size(); i++)
        {
            //std::cout<<"cloud 1 size: "<<cloud_1->size()<<std::endl;
            int m=0;
            // double d_dist = 2.0*tan(3.14159*2/180)*sqrt(cloud_1->points[i].x * cloud_1->points[i].x + cloud_1->points[i].y * cloud_1->points[i].y);
            double d_dist = 0.5;

            for (int j=0; j<cloudin_cluster->size(); j++)
            {
                if (cloudin_cluster->at(j)>=0)
                {
                    //std::cout<<"cloud 2 size: "<<cloud_2->size()<<std::endl;
                    double x_dist = cloudin->points[cluster_1->at(i)].x - cloudin->points[j].x;
                    double y_dist = cloudin->points[cluster_1->at(i)].y - cloudin->points[j].y;
                    double z_dist = cloudin->points[cluster_1->at(i)].z - cloudin->points[j].z;
                    double square_dist = sqrt(x_dist*x_dist+y_dist*y_dist+z_dist*z_dist);

//                    std::cout<<"x1: "<<cloudin->points[j].x<<" y1: "<<cloudin->points[j].y<<" z1: "<<cloudin->points[j].z<<std::endl;
//                    std::cout<<"x_dist: "<<x_dist<<" y_dist: "<<y_dist<<" z_dist "<<z_dist<<" and square dist"<<square_dist<<std::endl;

                    //float dist2 = pcl::geometry::squaredDistance(cloud_1->points[i], cloudin->points[j]);

                    //std::cout<<square_dist<<" and "<<dist2<<std::endl;

                    if (square_dist<d_dist)
                    {

                        cluster_2->push_back(j);
                        //std::cout<<"difference: "<<square_dist<<std::endl;
                        if (cluster_2->size()==3)
                        {
                            for (int k=0; k<cluster_2->size(); k++)
                            {
                                bool equal=false;
                                for(int l=0; l<cluster_1->size(); l++)
                                {
                                    if (cluster_1->at(l)==cluster_2->at(k))
                                    {
                                        equal = true;
                                        break;
                                    }
                                }
                                if (equal==false)
                                {
                                    cluster_3->push_back(cluster_2->at(k));
                                }
                            }

                            //std::cout<<"cluster_3 :"<<cluster_3->size()<<" "<<cluster_2->at(0)<<" "<<cluster_2->at(1)<<" "<<cluster_2->at(2)<<std::endl;

                            cluster_1->insert(cluster_1->end(),cluster_3->begin(),cluster_3->end());
                            for (int oo=0; oo<cluster_3->size(); oo++)
                            {
                                cloudin_cluster->at(cluster_3->at(oo))=-1;
                                //cloud_1->push_back(cloud_3->points[cluster_3->at(oo)]);
                                //std::cout<<cluster_3->at(oo)<<" ";
                                //chosen_point = cloud
                            }
                            //std::cout<<std::endl;
                            //chosen_point = cluster_3;

                            cluster_2->clear();
                            cluster_3->clear();
                            break;
                        }

                    }
                }
            }
            cluster_2->clear();

        }
        cloudin_cluster->at(cluster_1->at(0)) = -1;
//        if (cloud_1->size()==1)
//        {
//            break;
//        }
        //std::cout<<"new cloud 1 size: "<<cloud_1->size()<<std::endl;
        pcl::PointXYZ centroid;
        centroid.x=0.0;
        centroid.y=0.0;
        centroid.z=0.0;

        if (cluster_1->size()>3)
        {
            for (int cloud_pt=0; cloud_pt<cloudin_cluster->size(); cloud_pt++)
            {
                if (cloudin_cluster->at(cloud_pt)>=0)
                {
                    temp_cloud->push_back(cloudin->points[cloud_pt]);

                }
//                else
//                {
//                    std::cout<<cloud_pt<<" ";
//                }

            }
            //std::cout<<std::endl;

            for (int i=0; i<cluster_1->size(); i++)
            {
                //std::cout<<centroid.x<<" "<<centroid.y<<" "<<centroid.z<<std::endl;
                //std::cout<<cloud_1->points[i].x<<" "<<cloud_1->points[i].y<<" "<<cloud_1->points[i].z<<std::endl;
                //std::cout<<cluster_1->at(i)<<": "<<cloudin->points[cluster_1->at(i)].x<<" "<<cloudin->points[cluster_1->at(i)].y<<" "<<cloudin->points[cluster_1->at(i)].z<<std::endl;
                centroid.x = centroid.x + cloudin->points[cluster_1->at(i)].x;
                centroid.y = centroid.y + cloudin->points[cluster_1->at(i)].y;
                centroid.z = centroid.z + cloudin->points[cluster_1->at(i)].z;
                cloud_1->push_back(cloudin->points[cluster_1->at(i)]);
            }
            centroid.x = centroid.x / cluster_1->size();
            centroid.y = centroid.y / cluster_1->size();
            centroid.z = centroid.z / cluster_1->size();

            //track_point = centroid;
            int cluster_size_now = cluster_cloud_->size();
            //std::cout<<"cluster "<<cluster_size_now<<" centroid: "<<centroid.x<<" "<<centroid.y<<" "<<centroid.z<<" with "<<cloud_1->size()<<" points."<<std::endl;

            if (centroid.z>=2.5)
            {
                std::cout<<"Cluster centroid too high."<<std::endl;
            }
            else if (centroid.z<-1.7)
            {
                std::cout<<"cluster centroid too low."<<std::endl;
            }
           
            else
            {
                

                //find_minimum_oriented_bounding_box(cloud_1);
                //mark_on_map(centroid, cluster_size_now);

                cloud_3.reset(new pcl::PointCloud<pcl::PointXYZ>);
                cloud_3 = temp_cloud;
                temp_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
                cluster_cloud_->push_back(cloud_1);
                //cloud_total->insert(cloud_total->end(),cloud_1->begin(), cloud_1->end());


            }

        }
        else
        {
            for (int cloud_pt=1; cloud_pt<cloud_3->size(); cloud_pt++)
            {
                temp_cloud->push_back(cloud_3->points[cloud_pt]);
            }
            cloud_3.reset(new pcl::PointCloud<pcl::PointXYZ>);
            cloud_3 = temp_cloud;
            temp_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
            cloudin_cluster->at(cluster_1->at(0)) = -1;

        }

        cluster_1->clear();

        cloud_1.reset(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_2.reset(new pcl::PointCloud<pcl::PointXYZ>);



    }

//    if (cloud_total->size()>0)
//    {
//        pcl::PointXYZ center_new;
//        int size_new = cloud_total->size();
//        for (int i=0; i<size_new; i++)
//        {
//            center_new.x = center_new.x+cloud_total->points[i].x;
//            center_new.y = center_new.y+cloud_total->points[i].y;
//            center_new.z = center_new.z+cloud_total->points[i].z;
//        }
//        center_new.x = center_new.x/size_new;
//        center_new.y = center_new.y/size_new;
//        center_new.z = center_new.z/size_new;

        

//        cluster_cloud_->push_back(cloud_total);
//    }


    cloud_3.reset(new pcl::PointCloud<pcl::PointXYZ>);
    //std::cout<<"cluster size: "<<cluster_cloud_->size()<<std::endl;
    cloudin_cluster->clear();
    cluster_2->clear();
    cluster_3->clear();
    cloud_total.reset(new pcl::PointCloud<pcl::PointXYZ>);
    //cluster_cloud_->clear();
    return (cluster_cloud_);

}


/* ------------ dbscan cluster with intensity --------------- */
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>* dbscan_cluster_intensity(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudin)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_3(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_total(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>* cluster_cloud_;
    cluster_cloud_ = new std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>;

    std::vector<int>* cluster_1;
    std::vector<int>* cluster_2;
    std::vector<int>* cluster_3;
    std::vector<int>* cloudin_cluster;
    std::vector<int>* cluster_noise;
    std::vector<double>* cluster_center;
    cluster_1 = new std::vector<int>;
    cluster_2 = new std::vector<int>;
    cluster_3 = new std::vector<int>;
    cloudin_cluster = new std::vector<int>;
    cluster_noise = new std::vector<int>;
    cluster_center = new std::vector<double>;

    //cloudin_cluster->resize(cloudin->size());
    for (int i=0; i<cloudin->size(); i++)
    {
        cloudin_cluster->push_back(i);
        //std::cout<<cloudin_cluster->at(i)<<": "<<cloudin->points[i].x<<" "<<cloudin->points[i].y<<" "<<cloudin->points[i].z<<std::endl;
    }

    //std::cout<<"cloudin_cluster size: "<<cloudin_cluster->size()<<std::endl;

    cloud_3 = cloudin;

    int cloud_3_size=0;

    while (cloud_3->size()>0)
    {

        cloud_3_size=cloud_3->size();
        int new_cloud_size=cloud_3_size;
        //std::cout<<"new cloud3 size: "<<cloud_3->size()<<" and label size: "<<cloudin_cluster->size()<<std::endl;
        //cluster_1->push_back(0);
        for (int i=0; i<cloudin->size(); i++)
        {
            if (cloudin_cluster->at(i)>-1)
            {
                cluster_1->push_back(i);
                break;
            }
        }
        if (cluster_1->size()==0)
        {
            break;
        }

        cloud_1->push_back(cloudin->points[cluster_1->at(0)]);
        //std::cout<<"checking first point: "<<cluster_1->at(0)<<std::endl;
        //std::cout<<"x1: "<<cloud_1->points[0].x<<" y1: "<<cloud_1->points[0].y<<" z1: "<<cloud_1->points[0].z<<std::endl;
        cloud_2 = cloud_3;

        for (int i=0; i<cluster_1->size(); i++)
        {
            //std::cout<<"cloud 1 size: "<<cloud_1->size()<<std::endl;
            int m=0;
            // double d_dist = 2.0*tan(3.14159*2/180)*sqrt(cloud_1->points[i].x * cloud_1->points[i].x + cloud_1->points[i].y * cloud_1->points[i].y);
            double d_dist = 0.5;

            for (int j=0; j<cloudin_cluster->size(); j++)
            {
                if (cloudin_cluster->at(j)>=0)
                {
                    //std::cout<<"cloud 2 size: "<<cloud_2->size()<<std::endl;
                    double x_dist = cloudin->points[cluster_1->at(i)].x - cloudin->points[j].x;
                    double y_dist = cloudin->points[cluster_1->at(i)].y - cloudin->points[j].y;
                    double z_dist = cloudin->points[cluster_1->at(i)].z - cloudin->points[j].z;
                    double square_dist = sqrt(x_dist*x_dist+y_dist*y_dist+z_dist*z_dist);

//                    std::cout<<"x1: "<<cloudin->points[j].x<<" y1: "<<cloudin->points[j].y<<" z1: "<<cloudin->points[j].z<<std::endl;
//                    std::cout<<"x_dist: "<<x_dist<<" y_dist: "<<y_dist<<" z_dist "<<z_dist<<" and square dist"<<square_dist<<std::endl;

                    //float dist2 = pcl::geometry::squaredDistance(cloud_1->points[i], cloudin->points[j]);

                    //std::cout<<square_dist<<" and "<<dist2<<std::endl;

                    if (square_dist<d_dist)
                    {

                        cluster_2->push_back(j);
                        //std::cout<<"difference: "<<square_dist<<std::endl;
                        if (cluster_2->size()==3)
                        {
                            for (int k=0; k<cluster_2->size(); k++)
                            {
                                bool equal=false;
                                for(int l=0; l<cluster_1->size(); l++)
                                {
                                    if (cluster_1->at(l)==cluster_2->at(k))
                                    {
                                        equal = true;
                                        break;
                                    }
                                }
                                if (equal==false)
                                {
                                    cluster_3->push_back(cluster_2->at(k));
                                }
                            }

                            //std::cout<<"cluster_3 :"<<cluster_3->size()<<" "<<cluster_2->at(0)<<" "<<cluster_2->at(1)<<" "<<cluster_2->at(2)<<std::endl;

                            cluster_1->insert(cluster_1->end(),cluster_3->begin(),cluster_3->end());
                            for (int oo=0; oo<cluster_3->size(); oo++)
                            {
                                cloudin_cluster->at(cluster_3->at(oo))=-1;
                                //cloud_1->push_back(cloud_3->points[cluster_3->at(oo)]);
                                //std::cout<<cluster_3->at(oo)<<" ";
                                //chosen_point = cloud
                            }
                            //std::cout<<std::endl;
                            //chosen_point = cluster_3;

                            cluster_2->clear();
                            cluster_3->clear();
                            break;
                        }

                    }
                }
            }
            cluster_2->clear();

        }
        cloudin_cluster->at(cluster_1->at(0)) = -1;
//        if (cloud_1->size()==1)
//        {
//            break;
//        }
        //std::cout<<"new cloud 1 size: "<<cloud_1->size()<<std::endl;
        pcl::PointXYZ centroid;
        centroid.x=0.0;
        centroid.y=0.0;
        centroid.z=0.0;

        if (cluster_1->size()>3)
        {
            for (int cloud_pt=0; cloud_pt<cloudin_cluster->size(); cloud_pt++)
            {
                if (cloudin_cluster->at(cloud_pt)>=0)
                {
                    temp_cloud->push_back(cloudin->points[cloud_pt]);

                }
//                else
//                {
//                    std::cout<<cloud_pt<<" ";
//                }

            }
            //std::cout<<std::endl;

            for (int i=0; i<cluster_1->size(); i++)
            {
                //std::cout<<centroid.x<<" "<<centroid.y<<" "<<centroid.z<<std::endl;
                //std::cout<<cloud_1->points[i].x<<" "<<cloud_1->points[i].y<<" "<<cloud_1->points[i].z<<std::endl;
                //std::cout<<cluster_1->at(i)<<": "<<cloudin->points[cluster_1->at(i)].x<<" "<<cloudin->points[cluster_1->at(i)].y<<" "<<cloudin->points[cluster_1->at(i)].z<<std::endl;
                centroid.x = centroid.x + cloudin->points[cluster_1->at(i)].x;
                centroid.y = centroid.y + cloudin->points[cluster_1->at(i)].y;
                centroid.z = centroid.z + cloudin->points[cluster_1->at(i)].z;
                cloud_1->push_back(cloudin->points[cluster_1->at(i)]);
            }
            centroid.x = centroid.x / cluster_1->size();
            centroid.y = centroid.y / cluster_1->size();
            centroid.z = centroid.z / cluster_1->size();

            //track_point = centroid;
            int cluster_size_now = cluster_cloud_->size();
            //std::cout<<"cluster "<<cluster_size_now<<" centroid: "<<centroid.x<<" "<<centroid.y<<" "<<centroid.z<<" with "<<cloud_1->size()<<" points."<<std::endl;

            if (centroid.z>=2.5)
            {
                std::cout<<"Cluster centroid too high."<<std::endl;
            }
            else if (centroid.z<-1.7)
            {
                std::cout<<"cluster centroid too low."<<std::endl;
            }

            else
            {


                //find_minimum_oriented_bounding_box(cloud_1);
                //mark_on_map(centroid, cluster_size_now);

                cloud_3.reset(new pcl::PointCloud<pcl::PointXYZI>);
                cloud_3 = temp_cloud;
                temp_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
                cluster_cloud_->push_back(cloud_1);
                //cloud_total->insert(cloud_total->end(),cloud_1->begin(), cloud_1->end());


            }

        }
        else
        {
            for (int cloud_pt=1; cloud_pt<cloud_3->size(); cloud_pt++)
            {
                temp_cloud->push_back(cloud_3->points[cloud_pt]);
            }
            cloud_3.reset(new pcl::PointCloud<pcl::PointXYZI>);
            cloud_3 = temp_cloud;
            temp_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
            cloudin_cluster->at(cluster_1->at(0)) = -1;

        }

        cluster_1->clear();

        cloud_1.reset(new pcl::PointCloud<pcl::PointXYZI>);
        cloud_2.reset(new pcl::PointCloud<pcl::PointXYZI>);



    }

//    if (cloud_total->size()>0)
//    {
//        pcl::PointXYZ center_new;
//        int size_new = cloud_total->size();
//        for (int i=0; i<size_new; i++)
//        {
//            center_new.x = center_new.x+cloud_total->points[i].x;
//            center_new.y = center_new.y+cloud_total->points[i].y;
//            center_new.z = center_new.z+cloud_total->points[i].z;
//        }
//        center_new.x = center_new.x/size_new;
//        center_new.y = center_new.y/size_new;
//        center_new.z = center_new.z/size_new;



//        cluster_cloud_->push_back(cloud_total);
//    }


    cloud_3.reset(new pcl::PointCloud<pcl::PointXYZI>);
    //std::cout<<"cluster size: "<<cluster_cloud_->size()<<std::endl;
    cloudin_cluster->clear();
    cluster_2->clear();
    cluster_3->clear();
    cloud_total.reset(new pcl::PointCloud<pcl::PointXYZI>);
    //cluster_cloud_->clear();
    return (cluster_cloud_);

}

/* ------------ dbscan cluster with 2d intensity --------------- */
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>* dbscan_cluster_flat_intensity(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudin)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_3(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_total(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>* cluster_cloud_;
    cluster_cloud_ = new std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>;

    std::vector<int>* cluster_1;
    std::vector<int>* cluster_2;
    std::vector<int>* cluster_3;
    std::vector<int>* cloudin_cluster;
    std::vector<int>* cluster_noise;
    std::vector<double>* cluster_center;
    cluster_1 = new std::vector<int>;
    cluster_2 = new std::vector<int>;
    cluster_3 = new std::vector<int>;
    cloudin_cluster = new std::vector<int>;
    cluster_noise = new std::vector<int>;
    cluster_center = new std::vector<double>;

    //cloudin_cluster->resize(cloudin->size());
    for (int i=0; i<cloudin->size(); i++)
    {
        cloudin_cluster->push_back(i);
        //std::cout<<cloudin_cluster->at(i)<<": "<<cloudin->points[i].x<<" "<<cloudin->points[i].y<<" "<<cloudin->points[i].z<<std::endl;
    }

    //std::cout<<"cloudin_cluster size: "<<cloudin_cluster->size()<<std::endl;

    cloud_3 = cloudin;

    int cloud_3_size=0;

    while (cloud_3->size()>0)
    {

        cloud_3_size=cloud_3->size();
        int new_cloud_size=cloud_3_size;
        //std::cout<<"new cloud3 size: "<<cloud_3->size()<<" and label size: "<<cloudin_cluster->size()<<std::endl;
        //cluster_1->push_back(0);
        for (int i=0; i<cloudin->size(); i++)
        {
            if (cloudin_cluster->at(i)>-1)
            {
                cluster_1->push_back(i);
                break;
            }
        }
        if (cluster_1->size()==0)
        {
            break;
        }

        cloud_1->push_back(cloudin->points[cluster_1->at(0)]);
        //std::cout<<"checking first point: "<<cluster_1->at(0)<<std::endl;
        //std::cout<<"x1: "<<cloud_1->points[0].x<<" y1: "<<cloud_1->points[0].y<<" z1: "<<cloud_1->points[0].z<<std::endl;
        cloud_2 = cloud_3;

        for (int i=0; i<cluster_1->size(); i++)
        {
            //std::cout<<"cloud 1 size: "<<cloud_1->size()<<std::endl;
            int m=0;
            // double d_dist = 2.0*tan(3.14159*2/180)*sqrt(cloud_1->points[i].x * cloud_1->points[i].x + cloud_1->points[i].y * cloud_1->points[i].y);
            double d_dist = 1.0;

            for (int j=0; j<cloudin_cluster->size(); j++)
            {
                if (cloudin_cluster->at(j)>=0)
                {
                    //std::cout<<"cloud 2 size: "<<cloud_2->size()<<std::endl;
                    double x_dist = cloudin->points[cluster_1->at(i)].x - cloudin->points[j].x;
                    double y_dist = cloudin->points[cluster_1->at(i)].y - cloudin->points[j].y;
                    //double z_dist = cloudin->points[cluster_1->at(i)].z - cloudin->points[j].z;
                    double square_dist = sqrt(x_dist*x_dist+y_dist*y_dist);

//                    std::cout<<"x1: "<<cloudin->points[j].x<<" y1: "<<cloudin->points[j].y<<" z1: "<<cloudin->points[j].z<<std::endl;
//                    std::cout<<"x_dist: "<<x_dist<<" y_dist: "<<y_dist<<" z_dist "<<z_dist<<" and square dist"<<square_dist<<std::endl;

                    //float dist2 = pcl::geometry::squaredDistance(cloud_1->points[i], cloudin->points[j]);

                    //std::cout<<square_dist<<" and "<<dist2<<std::endl;

                    if (square_dist<d_dist)
                    {

                        cluster_2->push_back(j);
                        //std::cout<<"difference: "<<square_dist<<std::endl;
                        if (cluster_2->size()==3)
                        {
                            for (int k=0; k<cluster_2->size(); k++)
                            {
                                bool equal=false;
                                for(int l=0; l<cluster_1->size(); l++)
                                {
                                    if (cluster_1->at(l)==cluster_2->at(k))
                                    {
                                        equal = true;
                                        break;
                                    }
                                }
                                if (equal==false)
                                {
                                    cluster_3->push_back(cluster_2->at(k));
                                }
                            }

                            //std::cout<<"cluster_3 :"<<cluster_3->size()<<" "<<cluster_2->at(0)<<" "<<cluster_2->at(1)<<" "<<cluster_2->at(2)<<std::endl;

                            cluster_1->insert(cluster_1->end(),cluster_3->begin(),cluster_3->end());
                            for (int oo=0; oo<cluster_3->size(); oo++)
                            {
                                cloudin_cluster->at(cluster_3->at(oo))=-1;
                                //cloud_1->push_back(cloud_3->points[cluster_3->at(oo)]);
                                //std::cout<<cluster_3->at(oo)<<" ";
                                //chosen_point = cloud
                            }
                            //std::cout<<std::endl;
                            //chosen_point = cluster_3;

                            cluster_2->clear();
                            cluster_3->clear();
                            break;
                        }

                    }
                }
            }
            cluster_2->clear();

        }
        cloudin_cluster->at(cluster_1->at(0)) = -1;
//        if (cloud_1->size()==1)
//        {
//            break;
//        }
        //std::cout<<"new cloud 1 size: "<<cloud_1->size()<<std::endl;
        pcl::PointXYZ centroid;
        centroid.x=0.0;
        centroid.y=0.0;
        centroid.z=0.0;

        if (cluster_1->size()>3)
        {
            for (int cloud_pt=0; cloud_pt<cloudin_cluster->size(); cloud_pt++)
            {
                if (cloudin_cluster->at(cloud_pt)>=0)
                {
                    temp_cloud->push_back(cloudin->points[cloud_pt]);

                }
//                else
//                {
//                    std::cout<<cloud_pt<<" ";
//                }

            }
            //std::cout<<std::endl;

            for (int i=0; i<cluster_1->size(); i++)
            {
                //std::cout<<centroid.x<<" "<<centroid.y<<" "<<centroid.z<<std::endl;
                //std::cout<<cloud_1->points[i].x<<" "<<cloud_1->points[i].y<<" "<<cloud_1->points[i].z<<std::endl;
                //std::cout<<cluster_1->at(i)<<": "<<cloudin->points[cluster_1->at(i)].x<<" "<<cloudin->points[cluster_1->at(i)].y<<" "<<cloudin->points[cluster_1->at(i)].z<<std::endl;
                centroid.x = centroid.x + cloudin->points[cluster_1->at(i)].x;
                centroid.y = centroid.y + cloudin->points[cluster_1->at(i)].y;
                centroid.z = centroid.z + cloudin->points[cluster_1->at(i)].z;
                cloud_1->push_back(cloudin->points[cluster_1->at(i)]);
            }
            centroid.x = centroid.x / cluster_1->size();
            centroid.y = centroid.y / cluster_1->size();
            centroid.z = centroid.z / cluster_1->size();

            //track_point = centroid;
            int cluster_size_now = cluster_cloud_->size();
            //std::cout<<"cluster "<<cluster_size_now<<" centroid: "<<centroid.x<<" "<<centroid.y<<" "<<centroid.z<<" with "<<cloud_1->size()<<" points."<<std::endl;

            if (centroid.z>=2.5)
            {
                std::cout<<"Cluster centroid too high."<<std::endl;
            }
            else if (centroid.z<-1.7)
            {
                std::cout<<"cluster centroid too low."<<std::endl;
            }

            else
            {


                //find_minimum_oriented_bounding_box(cloud_1);
                //mark_on_map(centroid, cluster_size_now);

                cloud_3.reset(new pcl::PointCloud<pcl::PointXYZI>);
                cloud_3 = temp_cloud;
                temp_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
                cluster_cloud_->push_back(cloud_1);
                //cloud_total->insert(cloud_total->end(),cloud_1->begin(), cloud_1->end());


            }

        }
        else
        {
            for (int cloud_pt=1; cloud_pt<cloud_3->size(); cloud_pt++)
            {
                temp_cloud->push_back(cloud_3->points[cloud_pt]);
            }
            cloud_3.reset(new pcl::PointCloud<pcl::PointXYZI>);
            cloud_3 = temp_cloud;
            temp_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
            cloudin_cluster->at(cluster_1->at(0)) = -1;

        }

        cluster_1->clear();

        cloud_1.reset(new pcl::PointCloud<pcl::PointXYZI>);
        cloud_2.reset(new pcl::PointCloud<pcl::PointXYZI>);



    }

//    if (cloud_total->size()>0)
//    {
//        pcl::PointXYZ center_new;
//        int size_new = cloud_total->size();
//        for (int i=0; i<size_new; i++)
//        {
//            center_new.x = center_new.x+cloud_total->points[i].x;
//            center_new.y = center_new.y+cloud_total->points[i].y;
//            center_new.z = center_new.z+cloud_total->points[i].z;
//        }
//        center_new.x = center_new.x/size_new;
//        center_new.y = center_new.y/size_new;
//        center_new.z = center_new.z/size_new;



//        cluster_cloud_->push_back(cloud_total);
//    }


    cloud_3.reset(new pcl::PointCloud<pcl::PointXYZI>);
    //std::cout<<"cluster size: "<<cluster_cloud_->size()<<std::endl;
    cloudin_cluster->clear();
    cluster_2->clear();
    cluster_3->clear();
    cloud_total.reset(new pcl::PointCloud<pcl::PointXYZI>);
    //cluster_cloud_->clear();
    return (cluster_cloud_);

}

private:

};

#endif
