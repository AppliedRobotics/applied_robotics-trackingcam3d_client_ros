#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(10.0);
  std::string robot_name;
  node.getParam("/frame_tf_broadcaster/robot_name", robot_name);
  float x = 0;
  float y = 0;
  float z = 0;
  float q1 = 0;
  float q2 = -1.5;
  float q3 = 1.5;
  float q4 = 0;

  float x_camera = 0;
  float y_camera = 0;
  float z_camera = 0;
  float q1_camera = 0;
  float q2_camera = 0;
  float q3_camera = 0;
  float q4_camera = 1;
  node.getParam("/frame_tf_broadcaster/x_camera", x_camera);
  node.getParam("/frame_tf_broadcaster/y_camera", y_camera);
  node.getParam("/frame_tf_broadcaster/z_camera", z_camera);
  node.getParam("/frame_tf_broadcaster/q1_camera", q1_camera);
  node.getParam("/frame_tf_broadcaster/q2_camera", q2_camera);
  node.getParam("/frame_tf_broadcaster/q3_camera", q3_camera);
  node.getParam("/frame_tf_broadcaster/q4_camera", q4_camera);
  
  
  while (node.ok()){
    transform.setOrigin( tf::Vector3(x, y, z) );
    //transform.setRotation( tf::Quaternion(q1, -1.5, 1.5, 0) );
    transform.setRotation( tf::Quaternion(q1, q2, q3, q4));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "global_stereo_camera", "stereo_camera_first"));
    
    transform.setOrigin( tf::Vector3(x_camera, y_camera, z_camera) );
    //transform.setRotation( tf::Quaternion(q1, -1.5, 1.5, 0) );
    transform.setRotation( tf::Quaternion(q1_camera, q2_camera, q3_camera, q4_camera));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), robot_name.c_str(), "global_stereo_camera"));
    rate.sleep();
  }
  return 0;
};