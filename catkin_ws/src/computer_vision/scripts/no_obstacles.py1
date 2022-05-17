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

lanes_to_publish_left = ""
lanes_to_publish_right = ""
lane_publisherL, lane_publisherR = "", ""

def canny_frame(frame_gray):
    blured_frame = cv2.GaussianBlur(frame_gray, (5, 5), 0)
    cannied_frame = cv2.Canny(blured_frame, 70, 120)
    return cannied_frame
max_val = 250
min_val = 100
k_size_x = 5
k_size_y = 5


def canny_frame(frame_gray):
    global max_val, min_val, k_size_y, k_size_x
    blured_frame = cv2.GaussianBlur(frame_gray, (k_size_y, k_size_x), 0)
    cannied_frame = cv2.Canny(blured_frame, min_val, max_val)
    return cannied_frame, blured_frame

def crop_frame(frame_cannied, shp):
    polygon = np.array(
            [
                [
                    (0, shp[0] / 2),
                    (0, shp[0]),
                    (shp[1], shp[0]),
                    (shp[1], shp[0] / 2)
                    ]
                ]
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
    global lanes_to_publish_left, lanes_to_publish_right, lane_publisherL, lane_publisherR, max_val, min_val, k_size_y, k_size_x
    brdg = CvBridge()
    raw_frame = brdg.imgmsg_to_cv2(data)
    coppied_frame = np.copy(raw_frame)
    gray_frame = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2GRAY)
    cannied_frame, blured_frame = canny_frame(gray_frame) #10
    interest_frame = crop_frame(cannied_frame, raw_frame.shape) #15
    #color_frame = color_seg(coppied_frame, gray_frame, interest_frame) #31
    possible_lines = cv2.HoughLines(interest_frame, 1, np.pi/180, 25)
    linesL = []
    linesR = []
    if possible_lines is not None:
        const = 180 / math.pi
        l = 0
        r = 0
        left_rho = 0
        left_theta = 0
        right_rho = 0
        right_theta = 0
        for line in possible_lines:
            theta = line[0][1]
            grad = round( theta * const, 4)
            if grad < 180:
                rho = line[0][0]
                if 55 < 65 and 65 < 75:
                    left_rho += rho
                    left_theta += theta
                    l += 1
                elif 117 < 127 and 127 < 137:
                    right_rho += rho
                    right_theta += theta
                    r += 1
        if l != 0:
            prom_left_rho = left_rho / l
            prom_left_theta = left_theta / l
            """
            a = math.cos(prom_left_theta)
            b = math.sin(prom_left_theta)
            x1 = a * prom_left_rho
            y1 = b * prom_left_rho
            pt1 = (int(x1 + 1000 * (-b)), int(y1 + 1000 * (a)))
            pt2 = (int(x1 - 1000 * (-b)), int(y1 - 1000 * (a)))
            cv2.line(raw_frame, pt1, pt2, (255, 0, 0), 3)
            #"""
            linesL = [
                    prom_left_rho,
                    prom_left_theta,
                    ]
        if r != 0:
            prom_right_rho = right_rho / r
            prom_right_theta = right_theta / r
            """
            a = math.cos(prom_right_theta)
            b = math.sin(prom_right_theta)
            x1 = a * prom_right_rho
            y1 = b * prom_right_rho
            pt1 = (int(x1 + 1000 * (-b)), int(y1 + 1000 * (a)))
            pt2 = (int(x1 - 1000 * (-b)), int(y1 - 1000 * (a)))
            cv2.line(raw_frame, pt1, pt2, (0, 255, 0), 3)
            #"""
            linesR = [
                    prom_right_rho,
                    prom_right_theta
                    ]
    lanes_to_publish_left = np.array(linesL, dtype=np.float32)
    lanes_to_publish_right = np.array(linesR, dtype=np.float32)
    """
    cv2.imshow("frame", raw_frame)
    cv2.imshow("Canny", interest_frame)
    cv2.imshow("Blur", blured_frame)
    k = cv2.waitKey(1)
    if k == ord('a'):
        max_val += 10
    elif k == ord('d'):
        max_val -= 10
    elif k == ord('w'):
        min_val += 10
    elif k == ord('s'):
        min_val -= 10
    elif k == ord('i'):
        k_size_y += 2
    elif k == ord('k'):
        if k_size_y > 0:
            k_size_y -= 2
        else:
            print("No es posible")
    elif k == ord('j'):
        if k_size_x > 0:
            k_size_x -= 2
        else:
            print("No es posible")
    elif k == ord('l'):
        k_size_x += 2
    elif k == ord('p'):
        print([min_val, max_val, k_size_y, k_size_x])
    #"""
    lane_publisherL.publish(lanes_to_publish_left)
    lane_publisherR.publish(lanes_to_publish_right)

def main():
    print("INITIALIZING NODE")
    global lanes_to_publish_left, lanes_to_publish_right, lane_publisherL, lane_publisherR
    rospy.init_node('raw_img_subscriber', anonymous = True)
    rospy.Subscriber('/raw_image', Image, callback_raw_image)
    lane_publisherL = rospy.Publisher("/raw_lanes_left", numpy_msg(Floats), queue_size=10)
    lane_publisherR = rospy.Publisher("/raw_lanes_right", numpy_msg(Floats), queue_size=10)
    loop = rospy.Rate(60)
    print("NODE INITIALIZED SUCCESFULLY")
    while not rospy.is_shutdown():
        loop.sleep()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

