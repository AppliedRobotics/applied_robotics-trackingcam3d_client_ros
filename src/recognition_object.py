#!/usr/bin/env python3
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import MarkerArray,Marker
from geometry_msgs.msg import Pose
import math

class Get_distance_from_camera:

  def __init__(self):     
    self.marker_array_msg = MarkerArray()
    marker = Marker()
    self.marker_array_msg.markers.append(marker)
    self.bridge = CvBridge()
    
    self.camera_info_sub = message_filters.Subscriber('/trackingcam3d_client_ros/trackingcam3d0/left/camera_info', CameraInfo)
           	
    self.image_sub = message_filters.Subscriber("/trackingcam3d_client_ros/trackingcam3d0/left/image_raw",Image)
    self.depth_sub = message_filters.Subscriber("/trackingcam3d_client_ros/trackingcam3d0/depth/image_rect",Image)
        
    self.pub = rospy.Publisher('/trackingcam3d_client_ros/recognition_object', Image, queue_size=1)	
    topic = '/trackingcam3d_client_ros/visualization_marker_array'
    self.publisher_marker = rospy.Publisher(topic, MarkerArray)

  def callback(self, rgb_data, depth_data, camera_info):
    
    try:
      camera_info_K = np.array(camera_info.K)
      
      # Intrinsic camera matrix for the raw (distorted) images.
      #     [fx  0 cx]
      # K = [ 0 fy cy]
      #     [ 0  0  1]
    
      m_fx = camera_info.K[0]
      m_fy = camera_info.K[4]
      m_cx = camera_info.K[2]
      m_cy = camera_info.K[5]
      inv_fx = 1. / m_fx
      inv_fy = 1. / m_fy
    
    
      cv_rgb = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
      depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
      depth_image_re = cv2.resize(depth_image, (640, 480))
      depth_array = np.array(depth_image_re, dtype=np.float32)
      cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
      depth_8 = (depth_array * 255).round().astype(np.uint8)
      cv_depth = np.zeros_like(cv_rgb)
      cv_depth[:,:,0] = depth_8
      cv_depth[:,:,1] = depth_8
      cv_depth[:,:,2] = depth_8
      
      gray = cv2.cvtColor(cv_rgb, cv2.COLOR_BGR2GRAY)
      gray = cv2.medianBlur(gray, 5)

      rows = gray.shape[0]
      circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 2,
                               param1=100, param2=30,
                               minRadius=5, maxRadius=60)
      rgb_height, rgb_width, rgb_channels = cv_rgb.shape
      if circles is not None:
        for i in circles[0,:]:
          if i[2] < 0 and i[2] > 50 :
            print(i[2])
            continue
          cv2.circle(cv_rgb,(i[0],i[1]),i[2],(0,255,0),2)
          
          roi_depth = depth_image_re[int(i[1]-i[2]):int(i[1]+i[2]),int(i[0]-i[2]):int(i[0]+i[2])]
          
          n = 0
          sum = 0
          for p in range(0, roi_depth.shape[0]):
              for  l in range(0, roi_depth.shape[1]):
                  value = roi_depth.item(p, l)
                  if value > 0.:
                      n = n + 1
                      sum = sum + value
          if n !=0:
            mean_z = sum / n
            point_z = mean_z * 0.001; # distance in meters
            point_x = ((int(i[0]) + 320) - m_cx) * point_z * inv_fx
            point_y = ((int(i[1]) + 240) - m_cy) * point_z * inv_fy
            k_scale = i[2]
            marker = Marker()
            marker.header.frame_id = "stereo_camera"
            marker.id = 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = Pose()
            marker.color.r = 240.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.scale.x = 0.002 * k_scale
            marker.scale.y = 0.002 * k_scale
            marker.scale.z = 0.002 * k_scale
            marker.frame_locked = True
            marker.ns = "Goal"
            marker.pose.position.x = point_x
            marker.pose.position.y = point_y
            marker.pose.position.z = point_z
            self.marker_array_msg.markers[0] = marker
                
            dist = math.sqrt(point_x * point_x + point_y * point_y + point_z * point_z)
            
            dist_str = "dist:" + str(format(dist, '.2f')) + "m"
            print(dist_str)
            
    except CvBridgeError as e:
      print(e)
      
    rgbd = np.concatenate((cv_rgb, cv_depth), axis=1)

    #convert opencv format back to ros format and publish result
    try:
      faces_message = self.bridge.cv2_to_imgmsg(rgbd, "bgr8")
      self.pub.publish(faces_message)
      self.publisher_marker.publish(self.marker_array_msg)
    except CvBridgeError as e:
      print(e)
    

def main(args):
  rospy.init_node('unibas_face_distance_calculator', anonymous=True)
  fd = Get_distance_from_camera()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)