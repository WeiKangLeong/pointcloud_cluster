#include <ros/ros.h>

#include <sensor_msgs/Image.h>

#include <fstream>
#include <vector>
#include <iostream>

#include <Eigen/Dense>

ros::Publisher pub_image;

int index_size;
//Eigen::MatrixXd a1234(3145687, 4);
//Eigen::MatrixXd ros_ind(3145687, 4);

std::vector<double>* a1_ = new std::vector<double>;
std::vector<double>* a2_ = new std::vector<double>;
std::vector<double>* a3_ = new std::vector<double>;
std::vector<double>* a4_ = new std::vector<double>;
std::vector<int>* ind_ = new std::vector<int>;
std::vector<int>* ind1_ = new std::vector<int>;
std::vector<int>* ind2_ = new std::vector<int>;
std::vector<int>* ind3_ = new std::vector<int>;
std::vector<int>* ind4_ = new std::vector<int>;

int sub2ind(const int row,const int col,const int rows,const int cols)
{
   return (cols-1)*(row)+rows-1;
}

void ind2sub(const int sub,const int cols,const int rows,int &row,int &col)
{
   row=sub/cols;
   col=sub%cols;
}

int matlab2ros(int mat_ind)
{
    int row = mat_ind/1536;
    int col = mat_ind%1536;
    return col*2048+row;
}

void receive_image_cb(const sensor_msgs::ImagePtr& img)
{
    double time_in, time_out, take_time;
    time_in = ros::Time::now().toSec();
    sensor_msgs::Image undistort;
    undistort.header = img->header;
    undistort.height = img->height;
    undistort.width = img->width;
    undistort.encoding = img->encoding;
    undistort.is_bigendian = img->is_bigendian;
    undistort.step = img->step;
    //undistort.data = img->data;

    int data_size = img->height * img->width;

    std::cout<<"image preparing: "<<data_size<<std::endl;

    /*for (int l=0; l<data_size-3; l++)
    {
        undistort.data[l*3] = 0;
        undistort.data[l*3+2] = 0;
    }

    //undistort.data.resize(data_size*3);

    Eigen::MatrixXd ind_to_cab_1(3145687, 4);
    Eigen::MatrixXd ind_to_cab_2(3145687, 4);
    Eigen::MatrixXd ind_to_cab_3(3145687, 4);
    for (int dd=0; dd<index_size; dd++)
    {
        ind_to_cab_1(dd, 0) = img->data[3*ros_ind(dd,0)];
        ind_to_cab_1(dd, 1) = img->data[3*ros_ind(dd,1)];
        ind_to_cab_1(dd, 2) = img->data[3*ros_ind(dd,2)];
        ind_to_cab_1(dd, 3) = img->data[3*ros_ind(dd,3)];
        ind_to_cab_2(dd, 0) = img->data[3*ros_ind(dd,0)+1];
        ind_to_cab_2(dd, 1) = img->data[3*ros_ind(dd,1)+1];
        ind_to_cab_2(dd, 2) = img->data[3*ros_ind(dd,2)+1];
        ind_to_cab_2(dd, 3) = img->data[3*ros_ind(dd,3)+1];
        ind_to_cab_3(dd, 0) = img->data[3*ros_ind(dd,0)+2];
        ind_to_cab_3(dd, 1) = img->data[3*ros_ind(dd,1)+2];
        ind_to_cab_3(dd, 2) = img->data[3*ros_ind(dd,2)+2];
        ind_to_cab_3(dd, 3) = img->data[3*ros_ind(dd,3)+2];
    }

    Eigen::MatrixXd A(3145687,1);
    Eigen::MatrixXd B(3145687,1);
    Eigen::MatrixXd C(3145687,1);

    std::cout<<"here"<<std::endl;
//    std::vector<double>* B = new std::vector<double>;
//    B->resize(3145687);
    A= ind_to_cab_1.dot(a1234.transpose());
    B= ind_to_cab_2.adjoint()*(a1234);
    C= ind_to_cab_3.adjoint()*(a1234);

    std::cout<<"there"<<std::endl;



    for (int m=0; m<index_size; m++)
    {
        undistort.data[3*ind_->at(m)]=A(m,0);
        undistort.data[3*ind_->at(m)+1]=B(m,0);
        undistort.data[3*ind_->at(m)+2]=C(m,0);
        std::cout<<A(m,0)<<" "<<B(m,0)<<" "<<C(m,0)<<std::endl;
    }

    std::cout<<"complete"<<std::endl;*/



    //std::cout<<"undistorted image size from: "<<img->data.size()<<" to "<<undistort.data.size()<<std::endl;
    /*std::cout<<int(img->data[ind1_->at(100)])<<" "<<double(img->data[ind1_->at(100)])<<" "<<int(img->data[ind2_->at(100)])<<" "<<double(img->data[ind2_->at(100)])<<std::endl;

    std::cout<<int(a1_->at(100)*double(img->data[ind1_->at(100)]) + a2_->at(100)*double(img->data[ind2_->at(100)])
                    + a3_->at(100)*double(img->data[ind3_->at(100)]) + a4_->at(100)*double(img->data[ind4_->at(100)]))
            <<" "<<a1_->at(100)*double(img->data[ind1_->at(100)]) + a2_->at(100)*double(img->data[ind2_->at(100)])
            + a3_->at(100)*double(img->data[ind3_->at(100)]) + a4_->at(100)*double(img->data[ind4_->at(100)])<<
            std::endl;

    std::cout<<undistort.data.size()<<std::endl;*/

    for (int l=0; l<data_size*3; l++)
    {
        undistort.data.push_back(255);
    }

    for (int i=0; i<index_size; i++)
    {
        //std::cout<<i<<" ";
        undistort.data[3*ind_->at(i)] = uint8_t(a1_->at(i)*img->data[3*ind1_->at(i)] + a2_->at(i)*img->data[3*ind2_->at(i)]
                + a3_->at(i)*img->data[3*ind3_->at(i)] + a4_->at(i)*img->data[3*ind4_->at(i)]);
        undistort.data[3*ind_->at(i)+1] = uint8_t(a1_->at(i)*double(img->data[3*ind1_->at(i)+1]) + a2_->at(i)*double(img->data[3*ind2_->at(i)+1])
                + a3_->at(i)*double(img->data[3*ind3_->at(i)+1]) + a4_->at(i)*double(img->data[3*ind4_->at(i)+1]));
        undistort.data[3*ind_->at(i)+2] = uint8_t(a1_->at(i)*double(img->data[3*ind1_->at(i)+2]) + a2_->at(i)*double(img->data[3*ind2_->at(i)+2])
                + a3_->at(i)*double(img->data[3*ind3_->at(i)+2]) + a4_->at(i)*double(img->data[3*ind4_->at(i)+2]));
        //std::cout<<undistort.data[i]<<" "<<std::endl;
    }

    //std::cout<<"prepared 3 channel."<<std::endl;
    //std::cout<<"undistorted image size from: "<<img->data.size()<<" to "<<undistort.data.size()<<std::endl;
    pub_image.publish(undistort);
    //std::cout<<undistort.data.size()<<std::endl;
    time_out = ros::Time::now().toSec();
    take_time = time_out-time_in;
    std::cout<<"time taken: "<<take_time<<std::endl;

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "undistort_image");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
  //ros::Subscriber sub_box = nh.subscribe ("array", 1, array_cb);
  ros::Subscriber sub_image = nh.subscribe("image", 1, receive_image_cb);

  // Create a ROS publisher for the output point cloud
  //pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
    pub_image = nh.advertise<sensor_msgs::Image> ("/undistorted_image", 1);

  std::vector<int>* x_ = new std::vector<int>;
  std::vector<int>* y_ = new std::vector<int>;
  std::vector<int>* x1_ = new std::vector<int>;
  std::vector<int>* y1_ = new std::vector<int>;
  std::vector<int>* x2_ = new std::vector<int>;
  std::vector<int>* y2_ = new std::vector<int>;
  std::vector<int>* x3_ = new std::vector<int>;
  std::vector<int>* y3_ = new std::vector<int>;
  std::vector<int>* x4_ = new std::vector<int>;
  std::vector<int>* y4_ = new std::vector<int>;


  // open a1234 and new indice
  std::ifstream calibrate_a1234, calibrate_index;
  int count = 0;
  int count_2 = 0;
  int row=0;
  int col=0;

  int old_index;
  double value;

  calibrate_a1234.open("/home/smaug/Downloads/calib_example/calibrate_image/a1234.txt", std::ios::app);
  calibrate_index.open("/home/smaug/Downloads/calib_example/calibrate_image/new_ind.txt", std::ios::app);
  if (calibrate_a1234.is_open()){
      std::cout << "a1234 is open."<<std::endl;
      while(calibrate_a1234 >> value)
      {
        //a1234 = Eigen::MatrixXd::Zero();
          //vec.push_back(value);
          //std::cout << "value is " <<value<< std::endl;
          count++;
          if (count==1)
          {
              a1_->push_back(value);
              //a1234(row, count-1)=value;
          }
          else if (count==2)
          {
              a2_->push_back(value);
              //a1234(row, count-1)=value;
          }
          else if (count==3)
          {
              a3_->push_back(value);
              //a1234(row, count-1)=value;
          }
          else if (count==4)
          {
              a4_->push_back(value);
              //a1234(row, count-1)=value;
              count = 0;
              //row++;
          }

      }
      calibrate_a1234.close();
      std::cout<<"a1234 is saved: "<<a4_->size()<<" rows." <<std::endl;
  }
  else std::cout << "Unable to open the a1234";

  index_size=a1_->size();

  if (calibrate_index.is_open()){
      std::cout << "index is open."<<std::endl;
      while(calibrate_index >> old_index)
      {

          //vec.push_back(value);
          //std::cout << "value is " <<value<< std::endl;
          count_2++;
          if (count_2==1)
          {
              y_->push_back(old_index);
          }
          else if (count_2==2)
          {
              x_->push_back(old_index);
          }
          else if (count_2==3)
          {
              y1_->push_back(old_index);
          }
          else if (count_2==4)
          {
              x1_->push_back(old_index);
          }
          else if (count_2==5)
          {
              y2_->push_back(old_index);
          }
          else if (count_2==6)
          {
              x2_->push_back(old_index);
          }
          else if (count_2==7)
          {
              y3_->push_back(old_index);
          }
          else if (count_2==8)
          {
              x3_->push_back(old_index);
          }
          else if (count_2==9)
          {
              y4_->push_back(old_index);
          }
          else if (count_2==10)
          {
              x4_->push_back(old_index);
              count_2=0;
          }

      }
      calibrate_index.close();
      std::cout<<"file is saved: "<<y4_->size()<<" rows." <<std::endl;
  }
  else std::cout << "Unable to open the index";

  //int image_size = x_->size();
  ind_->resize(index_size);
  ind1_->resize(index_size);
  ind2_->resize(index_size);
  ind3_->resize(index_size);
  ind4_->resize(index_size);

//    for (int i=0; i<index_size; i++)
//    {
//        ind_->at(i) = sub2ind(1536, 2048, x_->at(i), y_->at(i));
//        ind1_->at(i) = sub2ind(1536, 2048, x1_->at(i), y1_->at(i));
//        ind2_->at(i) = sub2ind(1536, 2048, x2_->at(i), y2_->at(i));
//        ind3_->at(i) = sub2ind(1536, 2048, x3_->at(i), y3_->at(i));
//        ind4_->at(i) = sub2ind(1536, 2048, x4_->at(i), y4_->at(i));
//    }
    for (int i=0; i<index_size; i++)
    {
        ind_->at(i) = matlab2ros(sub2ind(2048, 1536, y_->at(i), x_->at(i)));
        ind1_->at(i) = matlab2ros(sub2ind(2048, 1536, y1_->at(i), x1_->at(i)));
        ind2_->at(i) = matlab2ros(sub2ind(2048, 1536, y2_->at(i), x2_->at(i)));
        ind3_->at(i) = matlab2ros(sub2ind(2048, 1536, y3_->at(i), x3_->at(i)));
        ind4_->at(i) = matlab2ros(sub2ind(2048, 1536, y4_->at(i), x4_->at(i)));
        //std::cout<<ind_->at(i)<<" ";
    }

    /*for (int kk=0; kk<index_size; kk++)
    {
        //index<< matlab2ros(sub2ind(2048, 1536, y_->at(i), x_->at(i)));
        ros_ind(kk,0)= matlab2ros(sub2ind(2048, 1536, y1_->at(kk), x1_->at(kk)));
        ros_ind(kk,1)= matlab2ros(sub2ind(2048, 1536, y2_->at(kk), x2_->at(kk)));
        ros_ind(kk,2)= matlab2ros(sub2ind(2048, 1536, y3_->at(kk), x3_->at(kk)));
        ros_ind(kk,3)= matlab2ros(sub2ind(2048, 1536, y4_->at(kk), x4_->at(kk)));
    }*/

    std::cout<<" index rearranged."<<std::endl;


  // Spin
  ros::spin ();
}
