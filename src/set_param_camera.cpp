#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <iostream>
using namespace std;

bool is_K_empty = 1;
double K[9];
//     [fx  0 cx]
// K = [ 0 fy cy]
//     [ 0  0  1]

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
    ros::param::set("/global_param", 5);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_param");
    ros::NodeHandle n;
    ros::Subscriber sub_cmara_info = n.subscribe("/trackingcam3d_client_ros/trackingcam3d0/left/camera_info", 1, camera_info_callback);    
    ROS_INFO("Runing ...");
    ros::spin();
    return 0;
}