#include <ros/ros.h>

#include <sensor_msgs/Image.h>

#include <fstream>
#include <vector>
#include <iostream>

double value;

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

  // open a1234 and new indice
  std::ifstream myFile;
  int count = 0;
  myFile.open("/home/smaug/catkin_ws/src/pointcloud_cluster/src/pointcloud_fit_image/test.txt", std::ios::app);
  if (myFile.is_open()){
      std::cout << "File is open."<<std::endl;
      while(myFile >> value){
          //vec.push_back(value);
          //std::cout << "value is " <<value<< std::endl;
          count++;
          if (count%4==0)
          }
      myFile.close();
  }
  else std::cout << "Unable to open the file";


  // Spin
  ros::spin ();
}
