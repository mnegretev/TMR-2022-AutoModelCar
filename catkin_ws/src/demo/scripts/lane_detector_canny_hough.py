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

def to_normal_form(x1, y1, x2, y2):
    A = y2 - y1
    B = x1 - x2
    C = A*x1 + B*y1
    theta = math.atan2(B,A)
    rho   = C/math.sqrt(A*A + B*B)
    if rho < 0:
        theta += math.pi
        rho = -rho
    return numpy.asarray([rho, theta])


#
# This function removes all lines with slopes outside the range -0.3, 0.3 rad
# and also removes vertical lines.
# It returns lines in two sets: left-border lines and right-border lines, based
# only in the slopes. 
#
def filter_lines(lines):
    left_lines  = []
    right_lines = []
    for x1, y1, x2, y2 in lines:
        rho, theta = to_normal_form(x1, y1, x2, y2)
        if (theta > -(math.pi/2-0.3) and theta < -0.1) or (theta > 0.1 and theta < (math.pi/2 - 0.3)):
            right_lines.append([x1, y1, x2, y2])
        if (theta > (math.pi/2 + 0.3) and theta < math.pi*0.9) or (theta > -0.9*math.pi and theta < -(math.pi/2 + 0.3)):
            left_lines.append([x1, y1, x2, y2])
    left_lines  = left_lines  if len(left_lines)  > 0 else None
    right_lines = right_lines if len(right_lines) > 0 else None
    return left_lines, right_lines

def weighted_average(lines):
    if lines is None or len(lines) == 0:
        return None
    weights = numpy.asarray([math.sqrt((x2 - x1)**2 + (y2 - y1)**2) for x1, y1, x2, y2 in lines])
    weights = weights/sum(weights)
    weighted_average = numpy.asarray([0.0,0.0])
    for i in range(len(lines)):
        x1, y1, x2, y2 = lines[i]
        rho , theta = to_normal_form(x1, y1, x2, y2)
        print(rho, theta)
        weighted_average[0] += rho*weights[i]
        weighted_average[1] += theta*weights[i]
    return weighted_average
         
def translate_lines_to_bottom_center(lines, x_center, y_center):
    if lines is None:
        return None
    new_lines = []
    for x1, y1, x2, y2 in lines:
        nx1 = x1 - x_center
        nx2 = x2 - x_center
        ny1 = y_center - y1
        ny2 = y_center - y2
        new_lines.append([nx1, ny1, nx2, ny2])
    return new_lines

def normal_line_to_points(rho, theta, length):
    a  = math.cos(theta)
    b  = math.sin(theta)
    x1 = a*rho - b*length
    y1 = b*rho + a*length
    x2 = a*rho + b*length
    y2 = b*rho - a*length
    return x1, y1, x2, y2
    
def callback_rgb_image(msg):
    bridge = CvBridge()
    img   = bridge.imgmsg_to_cv2(msg, 'bgr8')
    img   = img[int(0.4*img.shape[0]):int(0.97*img.shape[0]) ,:,:]
    canny = detect_edges(img)
    lines = cv2.HoughLinesP(canny, 2, numpy.pi/180, 80, minLineLength=80, maxLineGap=100)[:,0]
    if lines is not None:
        for x1, y1, x2, y2 in lines:
            cv2.line(img, (x1, y1), (x2, y2), (255,0,255), 3, cv2.LINE_AA)
    lines = translate_lines_to_bottom_center(lines, img.shape[1]/2, img.shape[0])
    left_lines, right_lines = filter_lines(lines)
    mean_left_line  = weighted_average(left_lines)
    mean_right_line = weighted_average(right_lines)
    if mean_left_line is not None:
        rho, theta = mean_left_line
        x1, y1, x2, y2 = normal_line_to_points(rho, theta, img.shape[0])
        x1 = int(x1 + img.shape[1]/2)
        x2 = int(x2 + img.shape[1]/2)
        y1 = int(img.shape[0] - y1)
        y2 = int(img.shape[0] - y2)
        cv2.line(img, (x1, y1), (x2, y2), (255,0,0), 3, cv2.LINE_AA)
    if mean_right_line is not None:
        rho, theta = mean_right_line
        x1, y1, x2, y2 = normal_line_to_points(rho, theta, img.shape[0])
        x1 = int(x1 + img.shape[1]/2)
        x2 = int(x2 + img.shape[1]/2)
        y1 = int(img.shape[0] - y1)
        y2 = int(img.shape[0] - y2)
        cv2.line(img, (x1, y1), (x2, y2), (0,0,255), 3, cv2.LINE_AA)
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

    

