#include <ros/ros.h>

#include <sensor_msgs/Image.h>

#include <fstream>
#include <vector>
#include <iostream>

double value;

std::vector<std::vector<double>* >* a1234_;
std::vector<std::vector<double>* >* indices_;

//void receive_image_cb(const sensor_msgs::ImagePtr &img)
//{

//}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "undistort_image");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
  //ros::Subscriber sub_box = nh.subscribe ("array", 1, array_cb);
  //ros::Subscriber sub_image = nh.subscribe <sensor_msgs::Image> ("image", 1, receive_image_cb);

  // Create a ROS publisher for the output point cloud
  //pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  a1234_ = new std::vector<std::vector<double>* >;
  std::vector<double>* a_row_;
  a_row_ = new std::vector<double>;
  a_row_->resize(4);
  indices_ = new std::vector<std::vector<double>* >;
  std::vector<double>* index_row_;
  index_row_ = new std::vector<double>;
  index_row_->resize(4);

  // open a1234 and new indice
  std::ifstream calibrate_a1234, calibrate_index;
  int count = 0;
  int count_2 = 0;
  calibrate_a1234.open("/home/smaug/Downloads/calib_example/calibrate_image/a1234.txt", std::ios::app);
  calibrate_index.open("/home/smaug/Downloads/calib_example/calibrate_image/new_ind.txt", std::ios::app);
  if (calibrate_a1234.is_open()){
      std::cout << "a1234 is open."<<std::endl;
      while(calibrate_a1234 >> value)
      {
          //vec.push_back(value);
          //std::cout << "value is " <<value<< std::endl;
          count++;
          if (count==4)
          {
              a_row_->push_back(value);
              a1234_->push_back(a_row_);
              a_row_->clear();
              count=0;
          }
          else
          {
              a_row_->push_back(value);
          }
      }
      calibrate_a1234.close();
      std::cout<<"a1234 is saved: "<<a1234_->size()<<" rows." <<std::endl;
  }
  else std::cout << "Unable to open the a1234";

  if (calibrate_index.is_open()){
      std::cout << "index is open."<<std::endl;
      while(calibrate_index >> value)
      {
          //vec.push_back(value);
          //std::cout << "value is " <<value<< std::endl;
          count_2++;
          if (count_2==10)
          {
              index_row_->push_back(value);
              a1234_->push_back(index_row_);
              index_row_->clear();
              count_2=0;
          }
          else
          {
              index_row_->push_back(value);
          }
      }
      calibrate_index.close();
      std::cout<<"file is saved: "<<a1234_->size()<<" rows." <<std::endl;
  }
  else std::cout << "Unable to open the index";

  // Spin
  ros::spin ();
}
