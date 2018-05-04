#ifndef GRID_H
#define GRID_H

#include <algorithm>

#define NUMBER 2
#define PI 3.1416
#define Ground -1.8

typedef pcl::PointXYZ PointType;
typedef pcl::PointXYZI PointType2;

struct Location
{
    Location(unsigned int m, unsigned int n)
    {
        m_ = m;
        n_ = n;
    }

    Location()
    {
        m_ = 0;
        n_ = 0;
    }

    unsigned int m_;
    unsigned int n_;
};


class Grid
{

public:

    std::vector<std::vector<std::vector<int>* >* >* grid_num_;
    std::vector<int>* cluster_;
    std::vector<int>* indices_;
    std::vector<std::vector<double>* >* floor_;
    std::vector<double>* sorting_height_;

    double range_x_;
    double range_y_;
    double range_z_;
    double res_;
    unsigned int grid_number_m_;
    unsigned int grid_number_n_;
    double center_x_;
    double center_y_;

    pcl::PointCloud<PointType>::Ptr cloud_;
    pcl::PointCloud<PointType>::Ptr cloud_2;
    pcl::PointCloud<PointType2>::Ptr cloud_3;


    Grid(double range_x, double range_y, double range_z, double res, double center_x, double center_y)
    {

        range_x_ = range_x;
        range_y_ = range_y;
        range_z_ = range_z;
        res_ = res;
        center_x_ = center_x;
        center_y_ = center_y;

        indices_ = new std::vector<int>;
        cluster_ = new std::vector<int>;

        grid_number_m_ = range_x/res;
        grid_number_n_ = range_y/res;

        grid_num_ = new std::vector<std::vector<std::vector<int>* >* >;
        grid_num_->resize(grid_number_m_);

        floor_ = new std::vector<std::vector<double>* >;
        floor_->resize(grid_number_m_);

        for(int cnt_m=0;cnt_m<grid_number_m_;cnt_m++)
        {
            grid_num_->at(cnt_m) = new std::vector<std::vector<int>* >;
            grid_num_->at(cnt_m)->resize(grid_number_n_);

            floor_->at(cnt_m) = new std::vector<double>;
            floor_->at(cnt_m)->resize(grid_number_n_);

                for(int cnt_n=0;cnt_n<grid_number_n_;cnt_n++)
                {
                    grid_num_->at(cnt_m)->at(cnt_n) = new std::vector<int>;
                    floor_->at(cnt_m)->at(cnt_n)=10.0;
            /*grid_num_->at(cnt_m)->at(cnt_n)->resize(0);
            for (int cnt_o=0; cnt_o<32; cnt_o++)
            {
                grid_num_->at(cnt_m)->at(cnt_n)->push_back(0);
            }*/
                }
        }

        cloud_.reset(new pcl::PointCloud<PointType>());
        cloud_2.reset(new pcl::PointCloud<PointType>());
        cloud_3.reset(new pcl::PointCloud<PointType2>());
    }

    void InsertXYZI(PointType2 point_in, int cnt)
    {
        int n_pos, m_pos, o_pos;

        if(((point_in.y<(center_y_+range_y_/2))&&(point_in.y>(center_y_-range_y_/2)))
                &&((point_in.x<(center_x_+range_x_/2))&&(point_in.x>(center_x_-range_x_/2))))
        {

            n_pos = int((range_y_/2 - point_in.y + center_y_)/res_);
            m_pos = int((range_x_/2 - point_in.x + center_x_)/res_);

            if(n_pos==grid_number_n_)
                return;

            if(m_pos==grid_number_m_)
                return;

        grid_num_->at(m_pos)->at(n_pos)->insert(grid_num_->at(m_pos)->at(n_pos)->end(),cnt);

        }
    }

    void InsertXYZ(PointType point_in, int cnt)
    {
        int n_pos, m_pos, o_pos;

        if(((point_in.y<(center_y_+range_y_/2))&&(point_in.y>(center_y_-range_y_/2)))
                &&((point_in.x<(center_x_+range_x_/2))&&(point_in.x>(center_x_-range_x_/2))))
        {

            n_pos = int((range_y_/2 - point_in.y + center_y_)/res_);
            m_pos = int((range_x_/2 - point_in.x + center_x_)/res_);
	    
            if(n_pos==grid_number_n_)
                return;

            if(m_pos==grid_number_m_)
                return;

        grid_num_->at(m_pos)->at(n_pos)->insert(grid_num_->at(m_pos)->at(n_pos)->end(),cnt);
	    /*if (o_pos==0)
            grid_num_->at(m_pos)->at(n_pos)->at(o_pos)=cnt;
	    else
	    grid_num_->at(m_pos)->at(n_pos)->at(o_pos+1)=cnt;*/

        //o_pos = (grid_num_->at(m_pos)->at(n_pos)->size());
        //std::cout<<cnt<<" "<<n_pos<<" "<<m_pos<<" "<<o_pos<<std::endl;

            /*if(grid_num_->at(m_pos)->at(n_pos)->at(o_pos)>NUMBER)
            {
                for(int cnt_i=0; cnt_i< grid_num_->at(m_pos)->at(n_pos)->at(o_pos); cnt_i++)
                {
                    PointType point;
                    point.x = (double)range_x_/2 - (double)m_pos*res_ + center_x_;
                    point.y = (double)range_y_/2 - (double)n_pos*res_ + center_y_;
                    point.z = 0;
                    cloud_->push_back(point);
                }
                grid_num_->at(m_pos)->at(n_pos) = 0;
            }*/
        }
    }

    void Filter_Z_drivable_region(PointType point_in, int cnt)
    {
        int n_pos, m_pos, o_pos;

        if(((point_in.y<(center_y_+range_y_/2))&&(point_in.y>(center_y_-range_y_/2)))
                &&((point_in.x<(center_x_+range_x_/2))&&(point_in.x>(center_x_-range_x_/2))))
        {

            n_pos = int((range_y_/2 - point_in.y + center_y_)/res_);
            m_pos = int((range_x_/2 - point_in.x + center_x_)/res_);



            double sea_level = floor_->at(m_pos)->at(n_pos);
            double dist_x = (double)range_x_/2 - (double)m_pos*res_ + center_x_;
            double dist_y = (double)range_y_/2 - (double)n_pos*res_ + center_y_;
            double min_z = 0.8 * abs(sqrt(dist_x*dist_x + dist_y*dist_y)) * tan(PI*2.0/180);

            if (point_in.z>(sea_level+min_z))
            {
                grid_num_->at(m_pos)->at(n_pos)->insert(grid_num_->at(m_pos)->at(n_pos)->end(),cnt);

            }

        //o_pos = (grid_num_->at(m_pos)->at(n_pos)->size());

        }
    }

    void filter_road(pcl::PointCloud<PointType2>::Ptr _cloud_in)
    {
        int c=0;
        for (int cnt=0; cnt<_cloud_in->size(); cnt++)
        {
            int n_pos, m_pos, o_pos;

            if(((_cloud_in->points[cnt].y<(center_y_+range_y_/2))&&(_cloud_in->points[cnt].y>(center_y_-range_y_/2)))
                    &&((_cloud_in->points[cnt].x<(center_x_+range_x_/2))&&(_cloud_in->points[cnt].x>(center_x_-range_x_/2))))
            {

                n_pos = int((range_y_/2 - _cloud_in->points[cnt].y + center_y_)/res_);
                m_pos = int((range_x_/2 - _cloud_in->points[cnt].x + center_x_)/res_);

                if(n_pos==grid_number_n_)
                    return;

                if(m_pos==grid_number_m_)
                    return;
//                std::cout<<n_pos<<" "<<m_pos<<" "<<std::endl;
//                           std::cout<<floor_->at(m_pos)->at(n_pos)<<std::endl;

                if (floor_->at(m_pos)->at(n_pos)<9.0)
                {
                    c++;
                    double sea_level = floor_->at(m_pos)->at(n_pos);
//                    double dist_x = (double)range_x_/2 - (double)m_pos*res_ + center_x_;
//                    double dist_y = (double)range_y_/2 - (double)n_pos*res_ + center_y_;
//                    double min_z = 0.8*abs(sqrt(dist_x*dist_x + dist_y*dist_y)) * tan(PI*2.0/180);

                    if (_cloud_in->points[cnt].z>sea_level && _cloud_in->points[cnt].z<2.0)
                    {
                        grid_num_->at(m_pos)->at(n_pos)->insert(grid_num_->at(m_pos)->at(n_pos)->end(),cnt);
                        //PointType2 point = cloud_2->points[grid_num_->at(cnt_m)->at(cnt_n)->at(cnt_k)];
                        //cloud_3->push_back(_cloud_in->points[cnt]);

                    }
                }

            }
        }
        std::cout<<c<<std::endl;
    }

    void filter_roadXY(pcl::PointCloud<PointType2>::Ptr _cloud_in)
    {

        int c=0;
        for (int cnt=0; cnt<_cloud_in->size(); cnt++)
        {
            int n_pos, m_pos, o_pos;

            if(((_cloud_in->points[cnt].y<range_y_)&&(_cloud_in->points[cnt].y>0.0)
                    &&((_cloud_in->points[cnt].x<range_x_)&&(_cloud_in->points[cnt].x>0.0))))
            {

                n_pos = int(_cloud_in->points[cnt].y);
                m_pos = int(_cloud_in->points[cnt].x);

                if(n_pos==grid_number_n_)
                    return;

                if(m_pos==grid_number_m_)
                    return;

                if (floor_->at(m_pos)->at(n_pos)<9.0)
                {

                    double sea_level = floor_->at(m_pos)->at(n_pos);

                    if (_cloud_in->points[cnt].z>(sea_level+0.1) && _cloud_in->points[cnt].z<2.5)
                    {
                        c++;
                        grid_num_->at(m_pos)->at(n_pos)->insert(grid_num_->at(m_pos)->at(n_pos)->end(),cnt);
                        cluster_->push_back(cnt);
                    }
                }
            }

        }
        std::cout<<c<<std::endl;
    }

    std::vector<int>* find_lowest_indices(pcl::PointCloud<PointType2>::Ptr point_in)
    {
        bool keep_flag;

        //cloud_2=point_in;
        //std::cout<<"finding lowest point."<<std::endl;
        for(int cnt_m=0;cnt_m<grid_number_m_;cnt_m++)
        {
            double dist_x = (double)range_x_/2 - (double)cnt_m*res_ + center_x_;

            for(int cnt_n=0;cnt_n<grid_number_n_;cnt_n++)
            {
                keep_flag=false;

                double dist_y = (double)range_y_/2 - (double)cnt_n*res_ + center_y_;
                double min_z = 0.8 * abs(sqrt(dist_x*dist_x + dist_y*dist_y)) * tan(PI*2.0/180);

                int grid_size = grid_num_->at(cnt_m)->at(cnt_n)->size();

                if (grid_size>1)
                {

                    sorting_height_ = new std::vector<double>;
                    // can improve here to finding the lowest z point before subtracting
                    for (int cnt_t=0; cnt_t<grid_size; cnt_t++)
                    {

                        sorting_height_->push_back(point_in->points[grid_num_->at(cnt_m)->at(cnt_n)->at(cnt_t)].z);

                    }

                    double lowest_z = *std::min_element(sorting_height_->begin(),sorting_height_->end());

                    sorting_height_->clear();

                    for (int cnt_j=0; cnt_j<grid_size; cnt_j++)
                    {
                        PointType2 point_current=point_in->points[grid_num_->at(cnt_m)->at(cnt_n)->at(cnt_j)];

                        if (point_current.z == lowest_z)
                        {
                            indices_->push_back(grid_num_->at(cnt_m)->at(cnt_n)->at(cnt_j));
                            break;
                        }
                    }


                }


            }


        }
        //std::cout<<"found "<<indices_->size()<<" lowest points"<<std::endl;
        return indices_;
    }

    pcl::PointCloud<PointType2>::Ptr find_lowest_point(pcl::PointCloud<PointType2>::Ptr point_in)
    {
        bool keep_flag;
        //cloud_2=point_in;
        //std::cout<<"finding lowest point."<<std::endl;
        for(int cnt_m=0;cnt_m<grid_number_m_;cnt_m++)
        {
            double dist_x = (double)range_x_/2 - (double)cnt_m*res_ + center_x_;

            for(int cnt_n=0;cnt_n<grid_number_n_;cnt_n++)
            {
                keep_flag=false;

                double dist_y = (double)range_y_/2 - (double)cnt_n*res_ + center_y_;
                double min_z = 0.8 * abs(sqrt(dist_x*dist_x + dist_y*dist_y)) * tan(PI*2.0/180);

                int grid_size = grid_num_->at(cnt_m)->at(cnt_n)->size();

                if (grid_size>1)
                {

                    sorting_height_ = new std::vector<double>;
                    // can improve here to finding the lowest z point before subtracting
                    for (int cnt_t=0; cnt_t<grid_size; cnt_t++)
                    {

                        sorting_height_->push_back(point_in->points[grid_num_->at(cnt_m)->at(cnt_n)->at(cnt_t)].z);

                    }

                    double lowest_z = *std::min_element(sorting_height_->begin(),sorting_height_->end());

                    sorting_height_->clear();

                    for (int cnt_j=0; cnt_j<grid_size; cnt_j++)
                    {
                        PointType2 point_current=point_in->points[grid_num_->at(cnt_m)->at(cnt_n)->at(cnt_j)];
                        //double height_diff = point_current.z - lowest_z;
                        //std::cout<<height_diff<<std::endl;
                        if (point_current.z == lowest_z)
                        {
                            cloud_3->push_back(point_current);
                            break;
                        }
                    }


                }


            }


        }
        return cloud_3;
    }

    pcl::PointCloud<PointType>::Ptr find_lowest_point(pcl::PointCloud<PointType>::Ptr point_in)
    {
        bool keep_flag;
        //cloud_2=point_in;
        //std::cout<<"finding lowest point."<<std::endl;
        for(int cnt_m=0;cnt_m<grid_number_m_;cnt_m++)
        {
            double dist_x = (double)range_x_/2 - (double)cnt_m*res_ + center_x_;

            for(int cnt_n=0;cnt_n<grid_number_n_;cnt_n++)
            {
                keep_flag=false;

                double dist_y = (double)range_y_/2 - (double)cnt_n*res_ + center_y_;
                double min_z = 0.8 * abs(sqrt(dist_x*dist_x + dist_y*dist_y)) * tan(PI*2.0/180);

                int grid_size = grid_num_->at(cnt_m)->at(cnt_n)->size();

                if (grid_size>1)
                {

                    sorting_height_ = new std::vector<double>;
                    // can improve here to finding the lowest z point before subtracting
                    for (int cnt_t=0; cnt_t<grid_size; cnt_t++)
                    {

                        sorting_height_->push_back(point_in->points[grid_num_->at(cnt_m)->at(cnt_n)->at(cnt_t)].z);

                    }

                    double lowest_z = *std::min_element(sorting_height_->begin(),sorting_height_->end());

                    sorting_height_->clear();

                    for (int cnt_j=0; cnt_j<grid_size; cnt_j++)
                    {
                        PointType point_current=point_in->points[grid_num_->at(cnt_m)->at(cnt_n)->at(cnt_j)];
                        //double height_diff = point_current.z - lowest_z;
                        //std::cout<<height_diff<<std::endl;
                        if (point_current.z == lowest_z)
                        {
                            cloud_2->push_back(point_current);
                            break;
                        }
                    }


                }


            }


        }
        return cloud_2;
    }

    void store_floor(std::vector<std::vector<double>* >* floor)
    {
        floor_ = floor;
    }

    int GetSize(int row, int column)
    {
        int size_ = grid_num_->at(row)->at(column)->size();
        return size_;
    }

    PointType GetCentroid(int row, int column)
    {

        int grid_size = grid_num_->at(row)->at(column)->size();
        cluster_ = new std::vector<int>;
        PointType point_2, centroid;
        //std::cout<<"grid size: "<<grid_size<<std::endl;
            for (int cnt_k=0; cnt_k<grid_size; cnt_k++)
            {

                point_2 = cloud_2->points[grid_num_->at(row)->at(column)->at(cnt_k)];
                centroid.x = centroid.x + point_2.x;
                centroid.y = centroid.y + point_2.y;
                centroid.z = centroid.z + point_2.z;
                //std::cout<<"x: "<<point_2.x<<" y: "<<point_2.y<<" z: "<<point_2.z<<std::endl;
                //cloud_3->push_back(point_2);
                cluster_->push_back(grid_num_->at(row)->at(column)->at(cnt_k));

            }
        return centroid;

    }

    PointType GetCentroid2(int row, int column)
    {

        int grid_size = grid_num_->at(row)->at(column)->size();
        cluster_ = new std::vector<int>;
        PointType2 point_2;
        PointType centroid;
        //std::cout<<"grid size: "<<grid_size<<std::endl;
            for (int cnt_k=0; cnt_k<grid_size; cnt_k++)
            {

                point_2 = cloud_3->points[grid_num_->at(row)->at(column)->at(cnt_k)];
                centroid.x = centroid.x + point_2.x;
                centroid.y = centroid.y + point_2.y;
                centroid.z = centroid.z + point_2.z;
                //std::cout<<"x: "<<point_2.x<<" y: "<<point_2.y<<" z: "<<point_2.z<<std::endl;
                //cloud_3->push_back(point_2);
                cluster_->push_back(grid_num_->at(row)->at(column)->at(cnt_k));

            }
        return centroid;

    }

    std::vector<int>* GetCluster()
    {
        return cluster_;
    }

    pcl::PointCloud<PointType>::Ptr GetCloud()
    {
        for (int cnt_m=0; cnt_m<grid_number_m_; cnt_m++)
        {
            for (int cnt_n=0; cnt_n<grid_number_n_; cnt_n++)
            {
                int grid_size = grid_num_->at(cnt_m)->at(cnt_n)->size();
                if (grid_size>2)
                {
                    for (int cnt_k=0; cnt_k<grid_size; cnt_k++)
                    {
                        PointType point = cloud_2->points[grid_num_->at(cnt_m)->at(cnt_n)->at(cnt_k)];                        
                        cloud_->push_back(point);
                    }
                }
            }
        }
        return cloud_;
    }

    pcl::PointCloud<PointType2>::Ptr GetCloud2(pcl::PointCloud<PointType2>::Ptr _cloud_in)
    {
        for (int cnt_m=0; cnt_m<grid_number_m_; cnt_m++)
        {
            for (int cnt_n=0; cnt_n<grid_number_n_; cnt_n++)
            {
                int grid_size = grid_num_->at(cnt_m)->at(cnt_n)->size();
                if (grid_size>2)
                {
                    for (int cnt_k=0; cnt_k<grid_size; cnt_k++)
                    {
                        PointType2 point = _cloud_in->points[grid_num_->at(cnt_m)->at(cnt_n)->at(cnt_k)];
                        cloud_3->push_back(point);
                    }
                }
            }
        }
        return cloud_3;
    }

    void Detect_vehicle(pcl::PointCloud<PointType2>::Ptr point_in)
    {
        cloud_3=point_in;
    }


    void Detect_vehicle(pcl::PointCloud<PointType>::Ptr point_in)
    {
        //bool keep_flag;
        cloud_2=point_in;
        //std::cout<<"arranging table."<<std::endl;
        /*for(int cnt_m=0;cnt_m<grid_number_m_;cnt_m++)
        {
            double dist_x = (double)range_x_/2 - (double)cnt_m*res_ + center_x_;

            for(int cnt_n=0;cnt_n<grid_number_n_;cnt_n++)
            {
                keep_flag=false;

                double dist_y = (double)range_y_/2 - (double)cnt_n*res_ + center_y_;
                double min_z = 0.8 * abs(sqrt(dist_x*dist_x + dist_y*dist_y)) * tan(PI*2.0/180);

                int grid_size = grid_num_->at(cnt_m)->at(cnt_n)->size();

                if (grid_size>2)
                {

                    sorting_height_ = new std::vector<double>;
                    // can improve here to finding the lowest z point before subtracting
                    for (int cnt_t=0; cnt_t<grid_size; cnt_t++)
                    {

                        sorting_height_->push_back(point_in->points[grid_num_->at(cnt_m)->at(cnt_n)->at(cnt_t)].z);

                    }

                    double lowest_z = *std::min_element(sorting_height_->begin(),sorting_height_->end());

                    sorting_height_->clear();

                    for (int cnt_j=0; cnt_j<grid_size; cnt_j++)
                    {
                        PointType point_current=point_in->points[grid_num_->at(cnt_m)->at(cnt_n)->at(cnt_j)];
                        double height_diff = point_current.z - lowest_z;

                        if (abs(height_diff)>min_z && abs(height_diff)<2.0)
                        {
                            keep_flag = true;
                            break;
                        }

                    }


                }

                if (keep_flag==false)
                {
                    grid_num_->at(cnt_m)->at(cnt_n)->clear();
                }
//                else
//                {
//                    for (int cnt_k=0; cnt_k<grid_size; cnt_k++)
//                    {
//                        PointType point = point_in->points[grid_num_->at(cnt_m)->at(cnt_n)->at(cnt_k)];
//                        //cloud_->push_back(point);
//                    }
//                }
            }


        }*/
     }

    pcl::PointCloud<PointType>::Ptr ArrangeFloor(pcl::PointCloud<PointType2>::Ptr cloud)
    {
        pcl::PointCloud<PointType>::Ptr floor_height_cloud (new pcl::PointCloud<PointType>);
        for (int row_size=0; row_size<grid_number_m_; row_size++)
        {
            for (int col_size=0; col_size<grid_number_n_; col_size++)
            {
                int grid_size = grid_num_->at(row_size)->at(col_size)->size();
                if (grid_size>0)
                {
                    PointType2 point_2;
                    PointType centroid;
                    double height=0.0;
                    for (int w_size=0; w_size<grid_size; w_size++)
                    {
                        point_2 = cloud->points[grid_num_->at(row_size)->at(col_size)->at(w_size)];
                        height = height + point_2.z;
                    }
                    centroid.x = (double)range_x_/2 - (double)row_size*res_ + center_x_;
                    centroid.y = (double)range_y_/2 - (double)col_size*res_ + center_y_;
                    centroid.z = height/grid_size;
                    floor_height_cloud->push_back(centroid);
                }
            }
        }
        return (floor_height_cloud);
    }

    void ArrangeFloorandStore(pcl::PointCloud<PointType>::Ptr cloud)
    {
        int table=0;
        for (int closize=0; closize<cloud->size();closize++)
        {
            PointType point_in = cloud->points[closize];
            int n_pos, m_pos;

            if(((point_in.y<(center_y_+range_y_/2))&&(point_in.y>(center_y_-range_y_/2)))
                    &&((point_in.x<(center_x_+range_x_/2))&&(point_in.x>(center_x_-range_x_/2))))
            {

                n_pos = int((range_y_/2 - point_in.y + center_y_)/res_);
                m_pos = int((range_x_/2 - point_in.x + center_x_)/res_);

                if(n_pos==grid_number_n_)
                    return;

                if(m_pos==grid_number_m_)
                    return;

            floor_->at(m_pos)->at(n_pos)=point_in.z;
            table++;

            }
        }
        std::cout<<"arranging table with "<<table<<" from "<<cloud->size()<<std::endl;
    }

    void ArrangeFloorandStoreXY(pcl::PointCloud<PointType>::Ptr cloud)
    {
        int table=0;
        for (int closize=0; closize<cloud->size();closize++)
        {
            PointType point_in = cloud->points[closize];
            int n_pos, m_pos;

            if(((point_in.y<range_y_)&&(point_in.y>0.0)
                    &&((point_in.x<range_x_)&&(point_in.x>0.0))))
            {

                n_pos = int(point_in.y);
                m_pos = int(point_in.x);

                if(n_pos==grid_number_n_)
                    return;

                if(m_pos==grid_number_m_)
                    return;

            floor_->at(m_pos)->at(n_pos)=point_in.z;
            table++;

            }
        }
        std::cout<<"arranging table with "<<table<<" from "<<cloud->size()<<std::endl;
    }

    void ArrangeTable(pcl::PointCloud<PointType>::Ptr point_in)
    {
        bool keep_flag;
        cloud_2=point_in;
        //std::cout<<"arranging table."<<std::endl;
        for(int cnt_m=0;cnt_m<grid_number_m_;cnt_m++)
        {
            double dist_x = (double)range_x_/2 - (double)cnt_m*res_ + center_x_;

            for(int cnt_n=0;cnt_n<grid_number_n_;cnt_n++)
            {
                keep_flag=false;

                double dist_y = (double)range_y_/2 - (double)cnt_n*res_ + center_y_;
                double min_z = 0.8 * abs(sqrt(dist_x*dist_x + dist_y*dist_y)) * tan(PI*2.0/180);

                int grid_size = grid_num_->at(cnt_m)->at(cnt_n)->size();

                if (grid_size>2)
                {

                    sorting_height_ = new std::vector<double>;
                    // can improve here to finding the lowest z point before subtracting
                    for (int cnt_t=0; cnt_t<grid_size; cnt_t++)
                    {

                        sorting_height_->push_back(point_in->points[grid_num_->at(cnt_m)->at(cnt_n)->at(cnt_t)].z);

                    }
                    //std::cout<<"finish sorting"<<std::endl;

                    double lowest_z = *std::min_element(sorting_height_->begin(),sorting_height_->end());
                    //std::cout<<lowest_z<<std::endl;
                    sorting_height_->clear();

                    //PointType point_last=point_in->points[grid_num_->at(cnt_m)->at(cnt_n)->at(grid_size-1)];

                    for (int cnt_j=0; cnt_j<grid_size; cnt_j++)
                    {
                        PointType point_current=point_in->points[grid_num_->at(cnt_m)->at(cnt_n)->at(cnt_j)];
                        double height_diff = point_current.z - lowest_z;
                        //std::cout<<height_diff<<std::endl;
                        if (abs(height_diff)>min_z && abs(height_diff)<2.0)
                        {
                            keep_flag = true;
                            break;

//                            if (abs(height_diff)>2.0)
//                            {
//                                keep_flag = false;

//                            }
                        }
//                        if (abs(height_diff)>0.8*min_z)
//                        {
//                            keep_flag = true;

//                        }
                    }


                }

                if (keep_flag==false)
                {
                    grid_num_->at(cnt_m)->at(cnt_n)->clear();
                }
//                else
//                {
//                    for (int cnt_k=0; cnt_k<grid_size; cnt_k++)
//                    {
//                        PointType point = point_in->points[grid_num_->at(cnt_m)->at(cnt_n)->at(cnt_k)];
//                        //cloud_->push_back(point);
//                    }
//                }
            }

            
        }
     }

    void ClearMovingGrid(int row1, int column1)
    {
        grid_num_->at(row1)->at(column1)->clear();
    }

    void ClearFloor()
    {
        floor_->clear();
    }

    void ClearGrid()
    {
        for(int cnt_m=0;cnt_m<grid_number_m_;cnt_m++)
        {
            //floor_->at(cnt_m)->clear();
                for(int cnt_n=0;cnt_n<grid_number_n_;cnt_n++)
                {

                    grid_num_->at(cnt_m)->at(cnt_n)->clear();
                }
        }
        cloud_.reset(new pcl::PointCloud<PointType>());
        cloud_2.reset(new pcl::PointCloud<PointType>());
        cloud_3.reset(new pcl::PointCloud<PointType2>());

        indices_->clear();
        cluster_->clear();
        /*
        delete grid_num_;
        grid_num_ = new std::vector<std::vector<std::vector<int>*>*>;
        grid_num_->resize(grid_number_m_);
        for(int cnt_m=0;cnt_m<grid_number_m_;cnt_m++)
        {
            grid_num_->at(cnt_m) = new std::vector<std::vector<int>* >;
	    grid_num_->at(cnt_m)->resize(grid_number_n_);
            for(int cnt_n=0;cnt_n<grid_number_n_;cnt_n++)
            {
                grid_num_->at(cnt_m)->at(cnt_n) = new std::vector<int>;
		grid_num_->at(cnt_m)->at(cnt_n)->clear();
		/*for (int cnt_o=0; cnt_o<32; cnt_o++)
		{
			grid_num_->at(cnt_m)->at(cnt_n)->push_back(0);
        }
            }
        }
        std::cout<<"Grid capacity: "<<grid_num_->capacity()<<std::endl;

        cloud_.reset(new pcl::PointCloud<PointType>());*/
    }

private:

};

#endif
