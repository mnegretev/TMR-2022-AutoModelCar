#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float64
from rospy_tutorials.msg import Floats
import ros_numpy
import os
import math

flag = 0
last = 0
delta = 0
count = 0
pub = 0
bbox = [0, 0, 0, 0, 0, 0, -15]
center = 0

def bounding_box(
        x_max, x_min,
        y_max, y_min,
        z_max, z_min,
        data, steps = 20
        ):
    grad_x = grad_y = grad_z = 0
    n = 0
    for i in range(0, len(data), steps):
        x, y, z = data[i]
        if x_min < x < x_max and y_min < y < y_max and z_min < z < z_max:
            grad_x += x
            grad_y += y
            grad_z += z
            n += 1
    if n > 0:
        grad_x /= n
        grad_y /= n
        grad_z /= n
    else:
        grad_x = 0
        grad_y = 0
        grad_z = 0
    return grad_x, grad_y, grad_z

def callback_cloud(msg):
    global flag, delta, last, count, pub, bbox, center
    arr = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
    x_pos = 0
    y_pos = 0
    z_pos = 0
    n_pos = 0
    x_pos, y_pos, z_pos = bounding_box(
            bbox[0], bbox[1],
            bbox[2], bbox[3],
            bbox[4], bbox[5],
            arr
            )
    distance = bbox[6]
    if distance < z_pos < 0:
        pub.publish(True)
        center.publish(x_pos)
    else:
        pub.publish(False)

def callback_bbox(msg):
    global bbox
    bbox = msg.data

def main():
    global pub, center
    print("Initializing node.....")
    rospy.init_node ('software_obstacle_detector',anonymous=True)
    rospy.Subscriber("/point_cloud", PointCloud2, callback_cloud)
    pub = rospy.Publisher("/bbox_flag", Bool, queue_size=10)
    center = rospy.Publisher("/center_x", Float64, queue_size=10)
    rospy.Subscriber("/bbox_values", Floats, callback_bbox)
    loop = rospy.Rate(60)
    print("Nodo exitosoooo!!.....")
    while not rospy.is_shutdown():
        loop.sleep
    rospy.spin()

main()

