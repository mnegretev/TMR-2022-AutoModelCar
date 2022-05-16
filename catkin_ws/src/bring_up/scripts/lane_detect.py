#!/usr/bin/env python2

""" 
    NODE TO GET THE RGB IMAGE FROM CAMERA 
    AND PROCESS IT TO GET LEFT & RIGHT LINES OF LANE  
"""

# LIBRARIES
from __future__ import division
from collections import Iterable
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
import cv2
from cv_bridge import CvBridge
import numpy as np
import math


# GLOBAL VARIABLES
polar_left_border = []
polar_right_border = []


# GET EDGES OF AN IMAGE
def canny(image):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)                              # CONVERT TO GRAY SCALE
    blur = cv2.GaussianBlur(gray, (5, 5), 0)                                    # APPLY BLUR FILTER TO REDUCE NOISE
    canny = cv2.Canny(blur, 50, 150)                                            # DETECT EDGES WITH CANNY

    return canny

# CALCULATE INTEREST REGION
def region_of_interest(image):
    height = image.shape[0]
    triangle = np.array([[
        (0, 450),                                                               # FIRST POINT, LEFT-BOTTOM
        (640, 450),                                                             # SECOND POINT, RIGHT-BOTTOM
        (320, 250)                                                              # THIRD POINT, CENTER
    ]])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, triangle, 255)
    masked_img = cv2.bitwise_and(image, mask)

    return masked_img

# DISPLAY LINES AS AN IMAGE
def display_lines(image, lines):
    line_img = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)                                    # GET 2 POINTS OF EACH LINE
            cv2.line(line_img, (x1, y1), (x2, y2), [255, 0, 0], 10)             # DRAW A LINE 
    return line_img                                                             # RETURN AN IMAGE WITH LINES


# GET THE EQUATION OF THE LINE
def avg_slope_intercept(image, lines):
    left_fit = []                                                               # LEFT LINES
    right_fit = []                                                              # RIGHT LINES
    
    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)                                        # GET 2 POINTS OF EACH LINE
        parameters = np.polyfit((x1, x2), (y1, y2), 1)                          # CALCULATE SLOPE(m) AND INTERCEPT(b) WITH y = mx + b
        slope = parameters[0]                                                   # SLOPE(m)
        intercept = parameters[1]                                               # INTERCEPT(b)
        
        angle = math.atan(slope)

        if angle < -0.3 or angle > 0.3:
            if slope < 0:
                left_fit.append((slope, intercept))                             # WITH (-m, b) 
            else:
                right_fit.append((slope, intercept))                            # WITH (+m, b)

    
    left_fit_avg = np.average(left_fit, axis=0)                                 # LEFT LINES AVG [m, b]
    right_fit_avg = np.average(right_fit, axis=0)                               # RIGHT LINES AVG [m ,b]
    left_line = make_coordinates(image, left_fit_avg)                           # LEFT LINE COORDINATES
    right_line = make_coordinates(image, right_fit_avg)                         # RIGHT LINE COORDINATES
    

    return np.array([left_line, right_line])                                    # RETURN LEFT AND RIGHT COORDINATES

# MAKE COORDINATES OF THE LINES
def make_coordinates(image, line_parameters):
    if isinstance(line_parameters, Iterable):
        slope, intercept = line_parameters                                      # SLOPE = m, INTERCEPT = b
        y1 = image.shape[0]                                                     # IMG HEIGHT (480)
        y2 = int((y1*(3/5)))
        x1 = int((y1 - intercept)/slope)                                        # X1 = (Y1 - b)/m
        x2 = int((y2 - intercept)/slope)                                        # X1 = (Y1 - b)/m
        return np.array([x1, y1, x2, y2])                                       # RETURN TWO POINTS AS A LINE (X1, Y1) - (X2, Y2)
    else:
        return np.array([0,0,0,0])

# CALCULATE DISTANCE AND ANGLE FROM CAMERA CENTER
def calculate_distance_angle(line, width, height, side):
    x1, y1, x2, y2 = line.reshape(4)                                            # GET POINTS OF LANE LINE 
    xp = (x1 + x2)/2                                                            # 'XP' IS THE AVG OF 'X1' AND 'X2'
    yp = (y1 + y2)/2                                                            # 'YP' IS THE AVG OF 'Y1' AND 'Y2'        

    if(side):                                                                   # LEFT LANE
        a = (width/2) - xp                                                      # ADJACENT SIDE FOR DISTANCE 'D'
        a_angle = xp - x1                                                       # ADJANCENT SIDE FOR THETA ANGLE
    else:                                                                       # RIGHT LANE
        a = xp -(width/2)                                                       # ADJACENT SIDE FOR DISTANCE 'D'
        a_angle = x1 -xp                                                        # ADJANCENT SIDE FOR THETA ANGLE

    # CALCULATE THE DISTANCE WITH PYTHAGORES THEOREM
    b = height - yp                                                             # OPPSITE SIDE FOR DISTANCE AND ANGLE
    distance = math.sqrt( (math.pow(a, 2) + math.pow(b, 2)) )                   # DISTANCE 'D'

    # CALCULATE ANGLE
    if a_angle > 0.0:
        angle = math.atan((b/a_angle))                                          # ANGLE IN RADIANS
        angle_degrees = (angle * (180/math.pi))                                 # ANGLE IN DEGREES
    else:
        angle = 0.0

    return [distance, angle]                                                    # RETURN A LIST WITH CURRENT DISTANCE & ANGLE


# CALLBACK LANE DETECT
def callback_lane_detect(msg):

    global polar_left_border, polar_right_border

    bridge = CvBridge()
    cv2_img = bridge.imgmsg_to_cv2(msg, 'bgr8')                                 # ROS IMAGE TO CV2 IMAGE ( RGB )
    lane_img = np.copy(cv2_img)                                                 # COPY OF ''cv_img'
    height = lane_img.shape[0]                                                  # X AXIS
    width = lane_img.shape[1]                                                   # Y AXIS
    canny_img = canny(lane_img)                                                 # CANNY IMAGE ( EDGES )
    cropped_img = region_of_interest(canny_img)                                 # COPPRED IMAGE ( ONLY INTEREST REGION ) 
    hough_lines = cv2.HoughLinesP(                                                    # HOUGHP ( LANE LINES )
            cropped_img,
            2,
            np.pi/180,
            80,
            minLineLength=40,
            maxLineGap=50
        )
    if hough_lines is not None:                                                                   # IF THERE ARE LINES
        avg_borders = avg_slope_intercept(lane_img, hough_lines)                                    # LEFT AND RIGHT LINES AS COORDINATES
        left_border, right_border = avg_borders.reshape(2,4)
        polar_left_border = calculate_distance_angle(left_border, width, height, True)          # GET DISTANCE AND ANGLE FOR LEFT LINE
        polar_right_border = calculate_distance_angle(right_border, width, height, False)       # GET DISTANCE AND ANGLE FOR RIGHT LINE
        # border_img = display_lines(lane_img, avg_borders)                                       # DISPLAY LINES IN A IMAGE
        # combo_img = cv2.addWeighted(lane_img, 0.8, border_img, 1, 1)                          # LANE_IMG + LINES
        # cv2.imshow('Result', combo_img)                                                     # DISPLAY IMAGE 
        # cv2.waitKey(33)
    else:                                                                                   # IF THERE AREN'T LINES
        polar_left_border = [0.0,0.0]                                                         # DISTANCE AND ANGLE NOT CALCULATED FOR LEFT LINE
        polar_right_border = [0.0,0.0]                                                        # DISTANCE AND ANGLE NOT CALCULATED FOR RIGHT LINE
        # cv2.imshow('Result', lane_img)                                                      # DISPLAY ORIGINAL IMAGE 
        # cv2.waitKey(33)

# MAIN FUNCTION
def main():

    global polar_left_border, polar_right_border

    print('Lane Detect Node...')
    rospy.init_node('lane_detect')
    rate = rospy.Rate(10)

    # SUBSCRIBERS
    rospy.Subscriber('/camera/rgb/raw', Image, callback_lane_detect)

     # MESSAGES
    msg_left_border = Float64MultiArray()
    msg_right_border = Float64MultiArray()

    # PUBLISHERS
    pub_left_border = rospy.Publisher('/left_border', Float64MultiArray, queue_size=10)
    pub_right_border = rospy.Publisher('/right_border', Float64MultiArray, queue_size=10)


    while not rospy.is_shutdown():
        # LEFT AND RIGHT LINES
        msg_left_border.data = polar_left_border
        msg_right_border.data = polar_right_border

        # PUBLISH LEFT AND RIGHT LINES
        pub_left_border.publish(msg_left_border)
        pub_right_border.publish(msg_right_border)
        
        rate.sleep()



if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass