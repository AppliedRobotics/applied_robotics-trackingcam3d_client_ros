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
  float q2 = 0;
  float q3 = 0;
  float q4 = 0;
  node.getParam("/frame_tf_broadcaster/x", x);
  node.getParam("/frame_tf_broadcaster/y", y);
  node.getParam("/frame_tf_broadcaster/z", z);
  node.getParam("/frame_tf_broadcaster/q1", q1);
  node.getParam("/frame_tf_broadcaster/q2", q2);
  node.getParam("/frame_tf_broadcaster/q3", q3);
  node.getParam("/frame_tf_broadcaster/q4", q4);
  
  while (node.ok()){
    transform.setOrigin( tf::Vector3(x, y, z) );
    //transform.setRotation( tf::Quaternion(q1, -1.5, 1.5, 0) );
    transform.setRotation( tf::Quaternion(q1, q2, q3, q4));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), robot_name.c_str(), "stereo_camera"));
    rate.sleep();
  }
  return 0;
};