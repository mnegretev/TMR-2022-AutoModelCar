#!/usr/bin/env python

import rospy
import random
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import cv2
import numpy as np
import math
import os
import time

start = time.time()
lanes_to_publish_left = ""
lanes_to_publish_right = ""
lane_publisherL, lane_publisherR = "", ""
degrees_publisher = ""
min_val = 33
max_val = 112
k_size_y = 5
k_size_x = 5
votes = 35
degl = 44
degr = 127
tolerance1 = 10
tolerance2 = 20
left_rho_goal = 392
right_rho_goal = -12


def canny_frame(frame_gray):
    global max_val, min_val, k_size_y, k_size_x
    blured_frame = cv2.GaussianBlur(frame_gray, (k_size_y, k_size_x), 0)
    cannied_frame = cv2.Canny(blured_frame, min_val, max_val)
    return cannied_frame, blured_frame

def crop_frame(frame_cannied, shp):
    polygon = np.int32(
            np.array(
                [
                    [
                        (0, shp[0] / 2),
                        (0, shp[0]),
                        (shp[1], shp[0]),
                        (shp[1], shp[0] / 2)
                        ]
                    ]
                )
            )
    zeros = np.zeros_like(frame_cannied)
    cv2.fillPoly(zeros, polygon, 255)
    regioned_image = cv2.bitwise_and(frame_cannied, frame_cannied, mask = zeros)
    return regioned_image

def color_seg(frame_color, frame_gray, frame_interest):
    color_max = np.array(
            [
                200, 200, 200
                ]
            )
    color_min = np.array(
            [
                21, 22, 16
                ]
            )
    color_mask = cv2.bitwise_and(frame_color, frame_color, mask=frame_interest)
    ranged_frame = cv2.inRange(color_mask, color_min, color_max)
    return ranged_frame

def callback_raw_image(data):
    global lanes_to_publish_left, lanes_to_publish_right, lane_publisherL, lane_publisherR, degrees_publisher,max_val, min_val, k_size_y, k_size_x, votes, degl, degr, tolerance1, tolerance2, gui, left_rho_goal, right_rho_goal, start
    brdg = CvBridge()
    raw_frame = brdg.imgmsg_to_cv2(data)
    coppied_frame = np.copy(raw_frame)
    gray_frame = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2GRAY)
    cannied_frame, blured_frame = canny_frame(gray_frame)
    kernel = np.ones((2, 1), np.uint8)
    cannied_frame = cv2.erode(cannied_frame, kernel)
    interest_frame = crop_frame(cannied_frame, raw_frame.shape) #15
    possible_lines = cv2.HoughLines(interest_frame, 1, np.pi/180, votes)
    linesL = []
    linesR = []
    linesC = []
    degrees = []
    if possible_lines is not None:
        if tolerance1 is not 10:
            tolerance1 = 10
        const = 180 / math.pi
        l = 0
        r = 0
        c = 0
        left_rho = 0
        left_theta = 0
        right_rho = 0
        right_theta = 0
        center_rho = 0
        center_theta = 0
        for line in possible_lines:
            theta = line[0][1]
            grad = round( theta * const, 4)
            if grad < 180:
                rho = line[0][0]
                if degl - tolerance1 < grad and grad < degl + tolerance1:
                    left_rho += rho
                    left_theta += theta
                    l += 1
                elif degr - tolerance1 < grad and grad < degr + tolerance1:
                    right_rho += rho
                    right_theta += theta
                    r += 1
        if l != 0:
            prom_left_rho = left_rho / l
            prom_left_theta = left_theta / l
            linesL = [
                    prom_left_rho,
                    prom_left_theta,
                    ]
        if r != 0:
            prom_right_rho = right_rho / r
            prom_right_theta = right_theta / r
            linesR = [
                    prom_right_rho,
                    prom_right_theta
                    ]
    else:
        tolerance1 += 5
    lanes_to_publish_left = np.array(linesL, dtype=np.float32)
    lanes_to_publish_right = np.array(linesR, dtype=np.float32)
    lane_publisherL.publish(lanes_to_publish_left)
    lane_publisherR.publish(lanes_to_publish_right)

def main():
    print("INITIALIZING VISION NODE")
    global lanes_to_publish_left, lanes_to_publish_right, lane_publisherL, lane_publisherR, degrees_publisher
    rospy.init_node('raw_img_subscriber', anonymous = True)
    rospy.Subscriber('/camera/rgb/raw', Image, callback_raw_image)
    lane_publisherL = rospy.Publisher("/raw_lanes_left", numpy_msg(Floats), queue_size=10)
    lane_publisherR = rospy.Publisher("/raw_lanes_right", numpy_msg(Floats), queue_size=10)
    degrees_publisher = rospy.Publisher("/combined_degrees", numpy_msg(Floats), queue_size=10)
    loop = rospy.Rate(60)
    print("VISION NODE INITIALIZED SUCCESFULLY")
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == "__main__":
    main()

