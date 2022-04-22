#!/usr/bin/env python

import roslib
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import numpy as np

bridge = CvBridge()
face_cascade = cv2.CascadeClassifier('/home/chris/ros_ws/hybrid/src/Vision/ros-object-detection-haar/object_detection_haar/nodes/haar/haarcascade_frontalface_default.xml')
# eye_cascade = cv2.CascadeClassifier('/home/chris/ros_ws/hybrid/src/Vision/ros-object-detection-haar/object_detection_haar/nodes/haar/haarcascade_eye.xml')
# ball_cascade = cv2.CascadeClassifier('/home/chris/ros_ws/hybrid/src/Vision/ros-object-detection-haar/object_detection_haar/nodes/haar/cascade_final.xml')

def image_callback(ros_image):
    print ('got an image')
    global bridge
  #convert ros_image into an opencv-compatible image
    try:
        img = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
  #from now on, you can work exactly like with opencv

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    for (x,y,w,h) in faces:
        img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
    #     roi_gray = gray[y:y+h, x:x+w]
    #     roi_color = img[y:y+h, x:x+w]
        # eyes = eye_cascade.detectMultiScale(roi_gray)
        # for (ex,ey,ew,eh) in eyes:
        #     cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
    # balls = ball_cascade.detectMultiScale(gray,20,20)
    # for(bx,by,bw,bh) in balls:
    #     cv2.rectangle(img,(bx,by),(bx+bw,by+bh),(0,255,0),2)
        # font = cv2.FONT_HERSHEY_SIMPLEX
        # cv2.putText(img,'Watch',(bx-bw,by-bh), font, 0.5, (11,255,255), 2, cv2.LINE_AA)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img,'Face Detection with ROS & OpenCV!',(15,450), font, 1,(255,255,255),2,cv2.LINE_AA)
    cv2.imshow('img',img)
    cv2.waitKey(3)


def main(args):
    rospy.init_node('image_converter', anonymous=True)
  #for turtlebot3 waffle
  #image_topic="/camera/rgb/image_raw/compressed"
  #for usb cam
  #image_topic="/usb_cam/image_raw"
    image_sub = rospy.Subscriber("/camera/color/image_raw",Image, image_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)