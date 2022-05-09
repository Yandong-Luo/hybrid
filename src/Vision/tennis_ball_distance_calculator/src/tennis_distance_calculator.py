#!/usr/bin/env python
from __future__ import print_function
from multiprocessing.connection import Listener

import roslib
import sys
import rospy
import cv2
import numpy as np
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from tennis_ball_distance_calculator.msg import ball_info
from geometry_msgs.msg import PointStamped
import tf
import tf2_ros
import math

class get_ball_distance_from_camera:

  def __init__(self):     
    
    # get parameter
    self.rgb_image_topic = rospy.get_param("RGB_image_topic","/locobot/camera/color/image_raw")

    self.depth_image_topic = rospy.get_param("depth_image_topic","/locobot/camera/aligned_depth_to_color/image_raw")

    self.camera_info_topic = rospy.get_param("camera_info_topic","/locobot/camera/color/camera_info")

    self.bridge = CvBridge()

    self.camera_info_sub = message_filters.Subscriber(self.camera_info_topic, CameraInfo)
           	
    self.image_sub = message_filters.Subscriber(self.rgb_image_topic,Image)
    self.depth_sub = message_filters.Subscriber(self.depth_image_topic,Image)
        
    self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub, self.camera_info_sub], queue_size=10, slop=0.5)
    self.ts.registerCallback(self.callback)
        
    self.detected_pub = rospy.Publisher('/tennis_ball_distance_calculator/detected_image', Image, queue_size=1)

    self.ball_info_pub = rospy.Publisher('/tennis_ball_distance_calculator/ball_info',ball_info,queue_size=10)

    self.ball_found = False

    # self.got_tf_transform = False

    # self.transfrom_to_arm_link()
  
  def transfrom_to_arm_link(self,ball_point):
      self.listener = tf.TransformListener()
      self.listener.waitForTransform("locobot/tilt_link","locobot/arm_base_link",rospy.Time(),rospy.Duration(4.0))

      got_tf_transform = False
      while got_tf_transform == False:
        try:
          rospy.loginfo('Waiting for the robot transform')
          now = rospy.Time.now()
          self.listener.waitForTransform("locobot/tilt_link","locobot/arm_base_link",now,rospy.Duration(4.0))

          # (trans,rot) = self.listener.lookupTransform("/locobot/arm_base_link", "/locobot/tilt_link", now)
          
          # print(np.matrix(trans))

          # print(np.matrix(rot))
          
          got_tf_transform = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          got_tf_transform = False
      
      # have got the transform matrix, and then just calculate the position of tennis ball at the frame of arm_base_link
      if got_tf_transform == True:
        # rospy.loginfo("Have got transform")
        ball_in_arm_frame = self.listener.transformPoint("locobot/arm_base_link",ball_point)
        # rospy.loginfo("ball x position:%f" % ball_in_arm_frame.point.x)
        # rospy.loginfo("ball y position:%f" % ball_in_arm_frame.point.y)
        # rospy.loginfo("ball z position:%f" % ball_in_arm_frame.point.z)
        return ball_in_arm_frame
      else:
        return

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
      depth_array = np.array(depth_image, dtype=np.float32)
      cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
      depth_8 = (depth_array * 255).round().astype(np.uint8)
      cv_depth = np.zeros_like(cv_rgb)
      cv_depth[:,:,0] = depth_8
      cv_depth[:,:,1] = depth_8
      cv_depth[:,:,2] = depth_8
      
      ball_cascade = cv2.CascadeClassifier('/home/locobot/interbotix_ws/src/tennis_ball_distance_calculator/haarcascade/cascade_12stages_24dim_0_25far.xml')
      gray = cv2.cvtColor(cv_rgb, cv2.COLOR_BGR2GRAY)
      balls = ball_cascade.detectMultiScale(gray, 1.3, 5)
      rgb_height, rgb_width, rgb_channels = cv_rgb.shape

      # draw the axis 
      cv2.line(cv_rgb,(int(rgb_width/2),int(rgb_height/2)),(int(rgb_width/2)+150,int(rgb_height/2)),(0,255,0),2)
      cv2.line(cv_rgb,(int(rgb_width/2),int(rgb_height/2)),(int(rgb_width/2),int(rgb_height/2)+150),(0,255,0),2)
      cv2.putText(cv_rgb, "x axis", (int(rgb_width/2)+180,int(rgb_height/2)), cv2.FONT_HERSHEY_SIMPLEX,  
                   0.7, (200,255,0), 1, cv2.LINE_AA)
      cv2.putText(cv_rgb, "y axis", (int(rgb_width/2),int(rgb_height/2)+180), cv2.FONT_HERSHEY_SIMPLEX,  
                   0.7, (125,255,0), 1, cv2.LINE_AA)
      self.ball_found = False

      for (x,y,w,h) in balls:
        self.ball_found = True
        cv2.rectangle(cv_rgb,(x,y),(x+w,y+h),(255,0,0),2)
        cv2.rectangle(cv_depth,(x,y),(x+w,y+h),(255,0,0),2)
        # cv2.rectangle(cv_rgb,(x+30,y+30),(x+w-30,y+h-30),(0,0,255),2)
        # cv2.rectangle(cv_depth,(x+30,y+30),(x+w-30,y+h-30),(0,0,255),2)
        roi_depth = depth_image[y+30:y+h-30, x+30:x+w-30]
        a = x + w/2
        b = y+h/2
        cv2.line(cv_rgb,(int(a)+10,int(b)+10),(int(a)-10,int(b)-10),(0,255,0),2)
        cv2.line(cv_rgb,(int(a)-10,int(b)+10),(int(a)+10,int(b)-10),(0,255,0),2)
        
        
        n = 0
        sum = 0
        for i in range(0,roi_depth.shape[0]):
            for j in range(0,roi_depth.shape[1]):
                value = roi_depth.item(i, j)
                if value > 0.:
                    n = n + 1
                    sum = sum + value
        if n == 0:
          return
        mean_z = sum / n
        
        point_z = mean_z * 0.001; # distance in meters
        point_x = ((x + w/2) - m_cx) * point_z * inv_fx
        point_y = ((y + h/2) - m_cy) * point_z * inv_fy
        
        x_str = "X: " + str(format(point_x, '.3f'))
        y_str = "Y: " + str(format(point_y, '.3f'))
        z_str = "Z: " + str(format(point_z, '.3f'))
                
        cv2.putText(cv_rgb, x_str, (x+w, y), cv2.FONT_HERSHEY_SIMPLEX,  
                   0.7, (0,0,255), 1, cv2.LINE_AA) 
        cv2.putText(cv_rgb, y_str, (x+w, y+20), cv2.FONT_HERSHEY_SIMPLEX,  
                   0.7, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(cv_rgb, z_str, (x+w, y+40), cv2.FONT_HERSHEY_SIMPLEX,  
                   0.7, (0,0,255), 1, cv2.LINE_AA)
                   
        dist = math.sqrt(point_x * point_x + point_y * point_y + point_z * point_z)
        
        dist_str = "dist:" + str(format(dist, '.2f')) + "m"
        
        cv2.putText(cv_rgb, dist_str, (x+w, y+60), cv2.FONT_HERSHEY_SIMPLEX,  
                   0.7, (0,255,0), 1, cv2.LINE_AA)
            
    except CvBridgeError as e:
      print(e)
      
    rgbd = np.concatenate((cv_rgb, cv_depth), axis=1)

    
    try:
      # convert opencv format back to ros format and publish result
      balls_message = self.bridge.cv2_to_imgmsg(rgbd, "bgr8")
      self.detected_pub.publish(balls_message)

      ball_msg = ball_info()
      # publish the pose information of tennis ball
      if self.ball_found:
        ball_msg.found_tennis = True
        ball_pose_info = PointStamped()
        ball_pose_info.header.frame_id = "locobot/tilt_link"
        ball_pose_info.header.stamp =rospy.Time(0)

        # the camera frame is different from the tilt_link. We need to compensated it
        # x axis at the tilt frame  ----> z axis at the camera image frame
        # y axis at the tilt frame  ----> -x axis at the camera image frame
        # z axis at the tilt frame  ----> -y axis at the camera image frame
        ball_pose_info.point.x = point_z
        ball_pose_info.point.y = -point_x
        ball_pose_info.point.z = -point_y
        
        # ball_msg.ball_pose = ball_pose_info
        ball_msg.distance.data = dist

        # transform the position of tennis ball to arm_base_link
        ball_info_in_arm_frame = self.transfrom_to_arm_link(ball_pose_info)

        ball_msg.ball_pose = ball_info_in_arm_frame

      else:
        ball_msg.found_tennis = False

        ball_pose_info = PointStamped()
        ball_pose_info.header.frame_id = "locobot/tilt_link"
        ball_pose_info.header.stamp =rospy.Time(0)
        
        ball_pose_info.point.x = 255.0
        ball_pose_info.point.y = 255.0
        ball_pose_info.point.z = 255.0

        ball_msg.ball_pose = ball_pose_info
        ball_msg.distance.data = 255.0
        # ball_info.ball_pose.position = None

      self.ball_info_pub.publish(ball_msg)
      # ball_info
    except CvBridgeError as e:
      print(e)

  



def main(args):
  rospy.init_node('unibas_ball_distance_calculator', anonymous=True)
  fd = get_ball_distance_from_camera()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

