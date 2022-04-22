#!/usr/bin/python3
from tkinter import Image
import rospy
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError

class viewer:
    def __init__(self):
        self.bridge = CvBridge()
        # 
        self.image_rgb_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)
        self.image_depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw",Image,self.callback)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as error:
            print(error)

        cv2.imshow("ball + depth",cv_image)
        cv2.waitKey(30)

def main(args):
    v = viewer()
    rospy.init_node("image_pub",anonymous=True)
    rospy.loginfo('image_pub node started')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass