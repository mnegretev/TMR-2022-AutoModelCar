#!/usr/bin/env python3
"""
This node finds lanes using a Canny edge detector and Hough transform.
It is intended to detect right and left lane borders always as straight lines. 
Curves are also detected as straight lines under the assumption that curvature
is small enough.
Detected lines are published in normal form (rho, theta) as 2-float arrays.
rho is measured in pixels and theta in radians
"""
import math
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
# they are always inside a trapezoid-shaped region in the bottom
# half of the image. This function returns such region of interest.
#
def region_of_interest(img):
    triangle = numpy.array([[
        (                  0, int(img.shape[0]*0.97)),  #Left-Bottom vertex              *
        (int(img.shape[1])  , int(img.shape[0]*0.97)),  #Right-Bottom vertex           *   *
        (int(img.shape[1])  , int(img.shape[0]*0.80)),  #Right-top vertex            *       *
        (int(img.shape[1]/2), int(img.shape[0]*0.40)),  #Center-top vertex           * * * * *
        (                  0, int(img.shape[0]*0.80)),  #Left-top vertex
    ]])
    mask = numpy.zeros_like(img)
    cv2.fillPoly(mask, triangle, (255,255,255))
    masked_img = cv2.bitwise_and(img, mask)
    return img[int(0.4*img.shape[0]):int(0.97*img.shape[0]) ,:,:]

#
# This function removes all lines with slopes outside the range -0.3, 0.3 rad
#
def filter_lines(lines):
    filtered_lines = []
    for i in range(len(lines)):
        x1, y1, x2, y2 = lines[i][0]
        theta = math.atan((y2 - y1)/(x2 - x1))
        if (theta < -0.3 or theta > 0.3):
            filtered_lines.append([x1, y1, x2, y2])
    return filtered_lines if len(filtered_lines) > 0 else None
        

def callback_rgb_image(msg):
    bridge = CvBridge()
    img   = bridge.imgmsg_to_cv2(msg, 'bgr8')
    img   = img[int(0.4*img.shape[0]):int(0.97*img.shape[0]) ,:,:]
    canny = detect_edges(img)
    lines = cv2.HoughLinesP(canny, 2, numpy.pi/180, 80, minLineLength=80, maxLineGap=100)
    lines = filter_lines(lines)
    if lines is None:
        return
    for i in range(len(lines)):
        l = lines[i]
        cv2.line(img, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
        
    cv2.imshow("Region of interest", img)
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

    

