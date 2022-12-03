#!/usr/bin/env python3
"""
This node finds lanes using a Canny edge detector and Hough transform.
It is intended to detect right and left lane borders always as straight lines. 
Curves are also detected as straight lines under the assumption that curvature
is small enough.
Detected lines are published in normal form (rho, theta) as 2-float arrays.
rho is measured in pixels and theta in radians
"""
import cv2
import numpy
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#
# Convert to grayscale              
# Apply blur filter to reduce noise 
# Canny edge detector               
#
def detect_edges(img):
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY) 
    blur = cv2.GaussianBlur(gray, (5, 5), 0)       
    canny = cv2.Canny(blur, 50, 150)               
    return canny

#
# Lane borders are always below the horizon and due to perspective,
# they are always inside a triangle-shaped region in the bottom
# half of the image. This function returns such region of interest.
# NOTE: It is assumed a 640x480 image. 
#
def region_of_interest(img):
    triangle = numpy.array([[
        (0, 450), 
        (640, 450),
        (320, 250) 
    ]])
    mask = numpy.zeros_like(img)
    cv2.fillPoly(mask, triangle, 255)
    masked_img = cv2.bitwise_and(img, mask)
    return masked_img

def callback_rgb_image(msg):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    cv2.imshow("Original RGB", img)
    cv2.waitKey(10)

def main():
    print("INITIALIZING LANE DETECTION DEMO...")
    rospy.init_node("lane_detector")
    rospy.Subscriber('/camera/rgb/raw', Image, callback_rgb_image)
    rate = rospy.Rate(10)
    rospy.spin()
    

if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass

    

