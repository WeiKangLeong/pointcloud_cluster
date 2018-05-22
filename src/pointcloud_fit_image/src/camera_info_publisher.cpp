#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float32.h>

ros::Publisher camera_header;

//void publish_color_info(const sensor_msgs::ImagePtr& color)
//{
//    std::cout<<color->data.size()<<std::endl;
//}

void publish_camera_info(const sensor_msgs::ImagePtr& head)
{
    sensor_msgs::CameraInfo cam_info;
    cam_info.header = head->header;
    //std::cout<<head->header.stamp<<std::endl;
    cam_info.height = 1536;
    cam_info.width = 2048;

    cam_info.distortion_model = "plumb_bob";

    cam_info.D.push_back(-0.38746);
    cam_info.D.push_back(0.30421);
    cam_info.D.push_back(-0.00026875);
    cam_info.D.push_back(-0.001613);
    cam_info.D.push_back(0.0);
    cam_info.K[0] = 1217.1;
    cam_info.K[1] = 0;
    cam_info.K[2] = 1087.4;
    cam_info.K[3] = 0;
    cam_info.K[4] = 1209.1;
    cam_info.K[5] = 834.55;
    cam_info.K[6] = 0.0;
    cam_info.K[7] = 0.0;
    cam_info.K[8] = 1.0;

    camera_header.publish(cam_info);

    //std::cout<<head->data.size()<<std::endl;
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "camera_info");
    ros::NodeHandle nh;

    ros::Subscriber sub_image = nh.subscribe("/camera/image_raw", 1, publish_camera_info);
    //ros::Subscriber sub_color = nh.subscribe("/image_rect_color", 1, publish_color_info);
    camera_header = nh.advertise<sensor_msgs::CameraInfo> ("camera_info", 1);

    ros::spin();
}
