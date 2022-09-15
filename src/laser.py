#!/usr/bin/env python3
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import math

class get_face_distance_from_camera:

  def __init__(self):     
     
    self.bridge = CvBridge()
    
    self.camera_info_sub = message_filters.Subscriber('/trackingcam3d_client_ros/trackingcam3d0/left/camera_info', CameraInfo)
           	
    self.image_sub = message_filters.Subscriber("/trackingcam3d_client_ros/trackingcam3d0/left/image_raw", Image)
    self.depth_sub = message_filters.Subscriber("/trackingcam3d_client_ros/trackingcam3d0/depth/image_rect", Image)
        
    self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub, self.camera_info_sub], queue_size=10, slop=0.5)
    self.ts.registerCallback(self.callback)
        
    self.pub = rospy.Publisher('/trackingcam3d_client_ros/test_images', Image, queue_size=1)	
    
    
    self.scan_pub = rospy.Publisher('scan', LaserScan, queue_size=50)
    current_time = rospy.Time.now()
    self.scan = LaserScan()
    self.num_readings = 320
    laser_frequency = 40
    self.scan.header.stamp = current_time
    self.scan.header.frame_id = 'map'
    self.scan.angle_min = -1.97
    self.scan.angle_max = 0.57 * 2 - 1.97
    self.scan.angle_increment = 0.57 * 2 / self.num_readings
    self.scan.time_increment = (1.0 / laser_frequency) / (self.num_readings)
    self.scan.range_min = 0.25
    self.scan.range_max = 5.0
    self.scan.ranges = np.array([5.0] * 320)
    self.k_pixel = 320 / self.num_readings

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
      inv_fx = float(1. / m_fx)
      inv_fy = float(1. / m_fy)
    
    
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
      rgb_height, rgb_width, rgb_channels = cv_rgb.shape
      np.set_printoptions(threshold=np.inf)
      roi_depth = depth_image
      #print((roi_depth))
      #print('правый верзний ',roi_depth[40, 300]) # y x
      #print('правый bottom ',roi_depth[220, 300])
      #print('left верзний ',roi_depth[40, 30])
      #print('left bottom ',roi_depth[200, 30])
      dist_min = np.array([5.0] * 320)
      current_time = rospy.Time.now()
      self.scan.header.stamp = current_time
      #print(roi_depth.item(10, 10))
      for y in range(0, roi_depth.shape[0]- 25):
          for x in range(0, self.num_readings):
              point_z = float(roi_depth.item(y, int(roi_depth.shape[1] - x * self.k_pixel - 1)) * 0.001)
              z = roi_depth.item(y, x)
              #print('x: ', x, 'y: ', y, "z: ", z)
              if point_z > 0.3 :
                point_x = float(((x + roi_depth.shape[1] / 2) - m_cx) * point_z * inv_fx)
                point_y = float(((y + roi_depth.shape[0] / 2) - m_cy) * point_z * inv_fy)
                dist = float(math.sqrt(point_x * point_x + point_z * point_z + point_y * point_y ))
                dist_str = "dist: " + str(format(dist, '.2f')) + "m"
                print(point_y)
                if point_y < 0.05:
                    if dist < dist_min[x] or dist_min[x] == 0.0:
                    #print(dist_str)
                    #print('dist ', dist_min[x])
                        dist_min[x] = dist = float(math.sqrt(point_x * point_x + point_z * point_z))
                        self.scan.ranges[x] = dist_min[x]
                    if self.scan.ranges[x] < 0.5:
                    #print('z ', point_z)
                    #print('x ', point_x)
                    #print('y ', point_y)
                    #print('dist ', dist)
                    #print('_____________')
                        pass
      # for x in range(5, self.num_readings - 5):
      #   mean = (dist_min[x - 3] + dist_min[x -2] + dist_min[x - 1]  + dist_min[x + 1] + dist_min[x + 2] + dist_min[x + 3]) / 6
      #   k = dist_min[x] / mean
      #   if k > 1.2 or k < 0.8:
      #     dist_min[x] = mean
        
      #   self.scan.ranges[x] = dist_min[x]
      self.scan_pub.publish(self.scan)
      
            
    except CvBridgeError as e:
      print(e)
      
    rgbd = np.concatenate((cv_rgb, cv_depth), axis=1)

    #convert opencv format back to ros format and publish result
    

def main(args):
  rospy.init_node('unibas_face_distance_calculator', anonymous=True)
  fd = get_face_distance_from_camera()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)