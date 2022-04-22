#!/usr/bin/env python
from __future__ import print_function
from ctypes.wintypes import RGB

import roslib
roslib.load_manifest('unibas_face_distance_calculator')
import sys
import rospy
import cv2
import numpy as np
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import math
import pyrealsense2 as rs2
from cmath import pi

class get_face_distance_from_camera:

  def __init__(self):     
     
    self.bridge = CvBridge()
    
    self.camera_info_sub = message_filters.Subscriber('/camera/color/camera_info', CameraInfo)
    
    
    self.image_sub = message_filters.Subscriber("/camera/color/image_raw",Image)
    self.depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw",Image)

    self.depth_info_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/camera_info",CameraInfo)
    
    # synchronize the data
    self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub, self.camera_info_sub, self.depth_info_sub], queue_size=10, slop=0.5)
    self.ts.registerCallback(self.callback)
        
    self.pub = rospy.Publisher('/unibas_face_distance_calculator/faces', Image, queue_size=1)	

    self.intrinsics = None
  
  def imageDepthInfoCallback(self,cameraInfo):
    try:
        # import pdb; pdb.set_trace()
        if self.intrinsics:
          return
        self.intrinsics = rs2.intrinsics()
        self.intrinsics.width = cameraInfo.width
        self.intrinsics.height = cameraInfo.height
        self.intrinsics.ppx = cameraInfo.K[2]
        self.intrinsics.ppy = cameraInfo.K[5]
        self.intrinsics.fx = cameraInfo.K[0]
        self.intrinsics.fy = cameraInfo.K[4]
        if cameraInfo.distortion_model == 'plumb_bob':
          self.intrinsics.model = rs2.distortion.brown_conrady
        elif cameraInfo.distortion_model == 'equidistant':
          self.intrinsics.model = rs2.distortion.kannala_brandt4
        self.intrinsics.coeffs = [i for i in cameraInfo.D]
    except CvBridgeError as e:
        print(e)
        return

  def callback(self, rgb_data, depth_data, camera_info,depth_info):
    
    try:
      self.imageDepthInfoCallback(depth_info)
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
      
      depth_image = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding="passthrough")

      depth_array = np.array(depth_image, dtype=np.float32)
      # print(depth_array)
      cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
      depth_8 = (depth_array * 255).round().astype(np.uint8)
      cv_depth = np.zeros_like(cv_rgb)
      print(cv_depth)
      # cv_depth = np.zeros(480)
      # cv_depth[:,:,0] = depth_8
      # cv_depth[:,:,1] = depth_8
      # cv_depth[:,:,2] = depth_8
      
      # detected the face
      face_cascade = cv2.CascadeClassifier('/home/chris/ros_ws/hybrid/src/Vision/unibas_face_distance_calculator/haarcascade/haarcascade_frontalface_default.xml')
      gray = cv2.cvtColor(cv_rgb, cv2.COLOR_BGR2GRAY)
      faces = face_cascade.detectMultiScale(gray, 1.3, 5)
      rgb_height, rgb_width, rgb_channels = cv_rgb.shape
      for (x,y,w,h) in faces:
        cv2.rectangle(cv_rgb,(x,y),(x+w,y+h),(255,0,0),2)
        # cv2.rectangle(cv_depth,(x,y),(x+w,y+h),(255,0,0),2)
        cv2.rectangle(cv_rgb,(x+30,y+30),(x+w-30,y+h-30),(0,0,255),2)
        # cv2.rectangle(cv_depth,(x+30,y+30),(x+w-30,y+h-30),(0,0,255),2)
        roi_depth = depth_image[y+30:y+h-30, x+30:x+w-30]

        face_center = (int(x + w/2), int(y+h/2))
        a = x + w/2
        b = y+h/2
        cv2.line(cv_rgb,(int(a)+10,int(b)+10),(int(a)-10,int(b)-10),(255,0,0),2)
        cv2.line(cv_rgb,(int(a)-10,int(b)+10),(int(a)+10,int(b)-10),(255,0,0),2)
        
        if self.intrinsics:
          distance = depth_image[face_center[1],face_center[0]]

          result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [face_center[0], face_center[1]], distance)
          print ('result:', result)

        # n = 0
        # sum = 0
        # for i in range(0,roi_depth.shape[0]):
        #     for j in range(0,roi_depth.shape[1]):
        #         value = roi_depth.item(i, j)
        #         if value > 0.:
        #             n = n + 1
        #             sum = sum + value
        
        # mean_z = sum / n
        
        # point_z = mean_z * 0.001; # distance in meters
        # point_x = ((x + w/2) - m_cx) * point_z * inv_fx
        # point_y = ((y + h/2) - m_cy) * point_z * inv_fy
        
        x_str = "X: " + str(format(result[0], '.2f'))
        y_str = "Y: " + str(format(result[1], '.2f'))
        z_str = "Z: " + str(format(result[2], '.2f'))
                
        cv2.putText(cv_rgb, x_str, (x+w, y), cv2.FONT_HERSHEY_SIMPLEX,  
                   0.7, (0,0,255), 1, cv2.LINE_AA) 
        cv2.putText(cv_rgb, y_str, (x+w, y+20), cv2.FONT_HERSHEY_SIMPLEX,  
                   0.7, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(cv_rgb, z_str, (x+w, y+40), cv2.FONT_HERSHEY_SIMPLEX,  
                   0.7, (0,0,255), 1, cv2.LINE_AA)
                   
        dist = math.sqrt(result[0] * result[0] + result[1] * result[1] + result[2] * result[2])
        
        dist_str = "dist:" + str(format(dist, '.2f')) + "m"
        
        cv2.putText(cv_rgb, dist_str, (x+w, y+60), cv2.FONT_HERSHEY_SIMPLEX,  
                   0.7, (0,255,0), 1, cv2.LINE_AA)
            
    except CvBridgeError as e:
      print(e)
      
    # rgbd = np.concatenate((cv_rgb,depth_image), axis=1)

    #convert opencv format back to ros format and publish result
    try:
      faces_message = self.bridge.cv2_to_imgmsg(cv_rgb, "bgr8")
      self.pub.publish(faces_message)
    except CvBridgeError as e:
      print(e)
    

def main(args):
  rospy.init_node('unibas_face_distance_calculator', anonymous=True)
  fd = get_face_distance_from_camera()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

