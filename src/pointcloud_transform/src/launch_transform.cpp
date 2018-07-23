//#include "localizer_odom.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>


tf::TransformBroadcaster *tfb;
#define G3X_IMU_DRIFT (-143.57733/180*M_PI)
#define WHEEL_RADIUS  (0.265 * 1.1203529)
#define GEAR_RATIO  6.066
const double COEFF_WHEEL_SPEED = (double)8191.0 / 16382.0 * 2.0 * M_PI  * WHEEL_RADIUS / 60.0; // m/s
const double COEFF_MOTORSPEED_ODOM = ((1.0)/GEAR_RATIO * 3.1415926 / 30.0 * WHEEL_RADIUS);

ros::Publisher odom_pub;
ros::Time last_odom_stamp;
double motor_rpm;
double can_vehicle_speed;
double front_left_vehicle_speed, front_right_vehicle_speed, rear_left_vehicle_speed, rear_right_vehicle_speed;
bool is_reverse;
double wheel_vehicle_speed, motor_vehicle_speed;
bool is_first_imu;
bool is_first_gps;
double last_odom_x = 0;
double last_odom_y = 0;
bool view_odom;

    void imu_callback(const sensor_msgs::Imu::ConstPtr &imu_msg)
    {

            double imu_roll, imu_pitch, imu_yaw, imu_angular_z;
            tf::Quaternion orient;
            tf::quaternionMsgToTF(imu_msg->orientation, orient);
            tf::Matrix3x3(orient).getRPY(imu_roll, imu_pitch, imu_yaw);
            imu_yaw += G3X_IMU_DRIFT;
            imu_angular_z = imu_msg->angular_velocity.z;

            /* To ROS */
            double ros_roll, ros_pitch, ros_yaw; //ENU system
            ros_pitch = -imu_pitch;
            ros_roll = imu_roll;
            if(imu_yaw < -M_PI/2)
                ros_yaw = -imu_yaw - 1.5*M_PI;
            else
                ros_yaw = 0.5*M_PI - imu_yaw;
            double ros_angular_z = -imu_angular_z;

            /* build baselink->odom in 2D plane */
            double velocity = wheel_vehicle_speed * cos(ros_pitch);
            if(velocity>5.0)
                velocity -=0.2;

            double yaw_rate = ros_angular_z;
            double delta_t = (imu_msg->header.stamp - last_odom_stamp).toSec();
            last_odom_stamp = imu_msg->header.stamp;
            double delta_x, delta_y;
            if(yaw_rate!=0)
            {
                delta_x = velocity / yaw_rate*(sin(ros_yaw + yaw_rate*delta_t)-sin(ros_yaw));
                delta_y = velocity / yaw_rate*(-cos(ros_yaw + yaw_rate*delta_t)+cos(ros_yaw));
            }else
            {
                delta_x = velocity * cos(ros_yaw) * delta_t;
                delta_y = velocity * sin(ros_yaw) * delta_t;
            }
            tf::Transform odom_transform;
            odom_transform.setRotation(tf::createQuaternionFromYaw(ros_yaw));
            if(fabs(velocity)>0.1) // to do probably no need
            {
                last_odom_x += delta_x;
                last_odom_y += delta_y;
            }


            /* build imu->baselink in 3D space */
            tf::Transform imu_transform;
            imu_transform.setRotation((tf::createQuaternionFromRPY(ros_roll, ros_pitch, 0.0)).normalized());
            imu_transform.setOrigin(tf::Vector3(0, 0, 0));
            tf::StampedTransform imu_transform_stamped(imu_transform, imu_msg->header.stamp, "/wtf_base_link", "/wtf_velodyne");
            tfb->sendTransform(imu_transform_stamped);



            /* odom test */
                geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, ros_yaw);
                nav_msgs::Odometry DR_odom;
                DR_odom.header.stamp = imu_msg->header.stamp;
                DR_odom.header.frame_id = "/wtf_odom";
                DR_odom.child_frame_id = "/wtf_base_link";
                DR_odom.pose.pose.orientation.x = geoQuat.x;
                DR_odom.pose.pose.orientation.y = geoQuat.y;
                DR_odom.pose.pose.orientation.z = geoQuat.z;
                DR_odom.pose.pose.orientation.w = geoQuat.w;
                DR_odom.pose.pose.position.x = last_odom_x;
                DR_odom.pose.pose.position.y = last_odom_y;
                DR_odom.pose.pose.position.z = 0;
                odom_pub.publish(DR_odom);


    }


    void odom_callback(const nav_msgs::Odometry::ConstPtr odom_input)
    {
        tf::Transform odom_transform;
        odom_transform.setRotation(tf::Quaternion(odom_input->pose.pose.orientation.x, odom_input->pose.pose.orientation.y, odom_input->pose.pose.orientation.z, odom_input->pose.pose.orientation.w));
        /* build baselink->odom in 2D space*/
        odom_transform.setOrigin(tf::Vector3(odom_input->pose.pose.position.x, odom_input->pose.pose.position.y, odom_input->pose.pose.position.z));
        tf::StampedTransform odom_transform_stamped(odom_transform, odom_input->header.stamp, "/wtf_odom", "/wtf_base_link");
        tfb->sendTransform(odom_transform_stamped);
    }

int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "launch_transform");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    //ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

    ros::Subscriber imu_sub = nh.subscribe("/charles_imu", 10, imu_callback);
    ros::Subscriber odom_sub = nh.subscribe("/charles_odom", 10, odom_callback);

    odom_pub = nh.advertise<nav_msgs::Odometry>("/canbus_odom", 10);


    // Create a ROS publisher for the output point cloud
    //pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
    tfb = new tf::TransformBroadcaster();

    // Spin
    ros::spin ();
}




