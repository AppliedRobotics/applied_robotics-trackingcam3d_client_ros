#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>

#include <sensor_msgs/CameraInfo.h>

#include "MyDispToLaserRectDyn.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <iostream>
using namespace std;

ros::Publisher scan_pub_;
bool is_K_empty = 1;
double K[9];
sensor_msgs::LaserScan scan_;
unsigned int num_readings = 320;
double laser_frequency = 40;

int count = 0;

float k_pixel = 640 / num_readings;

//     [fx  0 cx]
// K = [ 0 fy cy]
//     [ 0  0  1]

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Step1:  Read depth map 
    //ROS_INFO("image format: %s %dx%d", img_msg->encoding.c_str(), img_msg->height, img_msg->width);
    int height = img_msg->height;
    int width = img_msg->width;
    //  Cast through pointer , Read as 16UC1 data , The unit is mm
    unsigned short *depth_data = (unsigned short*)&img_msg->data[0];
    
    // Step2:  Depth map to point cloud 
    double ranges[num_readings];
    double ranges_max[num_readings];
    double intensities[num_readings];
    
    
    ros::Time scan_time = ros::Time::now();
    scan_.header.stamp = scan_time;
    scan_.header.frame_id = "map";
    scan_.ranges.resize(num_readings);
    scan_.intensities.resize(num_readings);
    //std::fill(scan_.intensities.begin(), scan_.intensities.end(), 0);
    
    int w_ = 640;
    int h_ = 480;
    float cx_ = K[2];
    float f_ = K[0];
    float laser_right_angle_ = - (float)atan((double)(w_ - cx_)/f_); //rad
    float laser_left_angle_ = (float)atan((double)(cx_/f_)); //rad
    float laser_angle_of_view_ = laser_left_angle_ - laser_right_angle_;
 
  //Represent in ROS laser_scan coordinates where 0 is along x axis
    float laser_scan_increment_ = laser_angle_of_view_ / num_readings;
    float laser_c_angle_ = atan((float)cx_ / f_);
    scan_.angle_increment = laser_scan_increment_;
    scan_.angle_min = laser_right_angle_; // rad
    scan_.angle_max =  laser_left_angle_;
    scan_.range_min = 0.3;
    scan_.range_max = 5;
    
    scan_.time_increment = (1 / laser_frequency) / (num_readings);
    for(int uy=239; uy<height; uy++)
    {
    for( int i = 0; i < num_readings; i++)
    {
            float x, y, z, scan_range;
            z = *(depth_data + uy * width + (int)round(i*k_pixel)) / 1000.0;
            if(z!=0)
            {
                x = z * ((int)round(i*k_pixel) - K[2]) / K[0];
                y = z * (uy - K[5]) / K[4];
                 
                if (z < ranges_max[i])
                {
                    ranges_max[i] = z;
                    scan_range = sqrt(x*x + z*z);
                    scan_.ranges[i] = scan_range;
                }
                
                
            }
            else
            {
                scan_.ranges[i] = 0;
            }
  
            ROS_INFO("x = %d  , z == %f", (int)round(x), z); 
            //ROS_INFO("pxl = %d  , i == %d", (int)round(i*k_pixel), i);  
            //ROS_INFO("pxl = %d  ", scan_range);  
    }
    }

    // Step3:  Publish
    // ROS_INFO("scan_range1 = %f  ", scan_.ranges[160]);
    // ROS_INFO("scan_range2 = %f  ", scan_.ranges[161]);
    // ROS_INFO("scan_range3 = %f  ", scan_.ranges[162]);
    // ROS_INFO("scan_range4 = %f  ", scan_.ranges[163]);
    scan_pub_.publish(scan_); 
}


void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &camera_info_msg)
{
    //  Read camera parameters 
    if(is_K_empty)
    {
        for(int i=0; i<9; i++)
        {
            K[i] = camera_info_msg->K[i];
            cout << K[i];
        }
        is_K_empty = 0;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_tutorial_node");
    ros::NodeHandle n;
    ros::Subscriber sub_img = n.subscribe("/trackingcam3d_client_ros/trackingcam3d0/depth/image_rect", 100, img_callback);

ros::Subscriber sub_cmara_info = n.subscribe("/trackingcam3d_client_ros/trackingcam3d0/left/camera_info", 1, camera_info_callback);
    scan_pub_ = n.advertise<sensor_msgs::LaserScan>("/scan", 50);
    ROS_INFO("Runing ...");
    ros::spin();
    return 0;
}