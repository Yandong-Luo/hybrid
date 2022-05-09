#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
import message_filters
from numpy import imag, round, floor
import cv2
from cv_bridge import CvBridge

class ball_detector:
    def __init__(self) -> None:
        # Subscribe to /camera/color/image_raw to read the image data
        self.image_rgb_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.process_image_callback)
        self.image_depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw",Image)

        self.camera_info_sub = message_filters.Subscriber('/camera/rgb/camera_info', CameraInfo)
        
        self.pub = rospy.Publisher('/ball_detector/ball', Image, queue_size=1)	

        # Object to convert Ros message to OpenCV image
        self.bridge = CvBridge()

    
    # Processes the image, and detect the position of ball
    def detect_ball(self,img):
        #img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        # adjust alpha and beta
        alpha=2
        beta=0
        adj = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)

        # sobel
        x_sobel = cv2.Sobel(adj, ddepth=cv2.CV_32F, dx=1, dy=0, ksize=3)
        y_sobel = cv2.Sobel(adj, ddepth=cv2.CV_32F, dx=0, dy=1, ksize=3)

        x_sobel = cv2.convertScaleAbs(x_sobel)
        y_sobel = cv2.convertScaleAbs(y_sobel)

        combined = cv2.addWeighted(x_sobel, 0.5, y_sobel, 0.5, 0)

        # detect circle in the image
        dp = 1
        minDist = 700
        param1 = 1100
        param2 = 20
        minRadius = 10
        maxRadius = 90

        circle = cv2.HoughCircles(
            combined,
            method=cv2.HOUGH_GRADIENT,
            dp=dp,
            minDist=minDist,
            param1=param1,
            param2=param2,
            minRadius=minRadius,
            maxRadius=maxRadius)

        if circle is not None:
            circle_data = dict(
                zip({"x", "y", "r"},
                    round(circle[0, 0]).astype("int")))

            return circle_data

        
    def process_image_callback(self,rgb_data):
        # convert ROS image message to OpenCV image
        img = self.bridge.imgmsg_to_cv2(rgb_data, desired_encoding="passthrough")

        # detect the ball position
        ball = self.detect_ball(img)
        if ball is not None:
            ball_pos = int(floor(ball["x"] / (img.shape[1] / 3)))
            msg = ("Ball (x, y, r): ("
                + str(ball["x"]) + ", "
                + str(ball["y"]) + ", "
                + str(ball["r"]) + ")")
            
            x = ball["x"]
            y = ball["y"]
            x_str = "X: " + str(format(x, '.2f'))
            y_str = "Y: " + str(format(y, '.2f'))
            cv2.putText(img, x_str, (x, y), cv2.FONT_HERSHEY_SIMPLEX,  
                   0.7, (0,0,255), 1, cv2.LINE_AA) 
            cv2.putText(img, y_str, (x, y+20), cv2.FONT_HERSHEY_SIMPLEX,  
                   0.7, (0,0,255), 1, cv2.LINE_AA)

        else:  # zero velocities
            msg = "Ball position: NONE"
        
        faces_message = self.bridge.cv2_to_imgmsg(img, "bgr8")
        self.pub.publish(faces_message)
        rospy.loginfo(msg)

def main():
    # Initialize the process_image node and create a handle to it
    rospy.init_node("ball_detector", anonymous=True)

    # Create a an object of class CommandRobotClient
    m_ball_detector = ball_detector()

    # Handle ROS communication events
    rospy.spin()

if __name__ == "__main__":
    main()
