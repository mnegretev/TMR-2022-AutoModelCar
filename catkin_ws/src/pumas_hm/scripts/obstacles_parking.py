#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
import ros_numpy
import os
import math

flag = 0
last = 0
delta = 0
count = 0
pub = 0

def callback_cloud(msg):
    global flag, delta, last, count, pub
    arr = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
    x_pos = 0
    y_pos = 0
    z_pos = 0
    n_pos = 0
    for i  in range(0, len(arr), 20):
        point = arr[i]
        x = point[0]
        y = point[1]
        z = point[2]
        if 0.0 < x < 4.0 and -2.0 < z < 2.0 and -1.5 < y < 0.5:
            n_pos += 1
            x_pos += x
            y_pos += y
            z_pos += z
    delta = last - x_pos
    if delta < -4 and not flag:
        count +=1
        flag = True
        last = x_pos
        if n_pos > 0:
            x_pos /= n_pos
            y_pos /= n_pos
            z_pos /= n_pos
    elif delta > 5 and flag:
        count +=1
        flag = False
        last = x_pos
        if n_pos > 0:
            x_pos /= n_pos
            y_pos /= n_pos
            z_pos /= n_pos
    if count >= 3:
        pub.publish(True)
    else:
        pub.publish(False)

def main():
    global pub
    print("Initializing node.....")
    rospy.init_node ('software_obstacle_detector',anonymous=True)
    rospy.Subscriber("/point_cloud", PointCloud2, callback_cloud)
    pub = rospy.Publisher("/parking_flag", Bool, queue_size=10)
    loop = rospy.Rate(60)
    print("Nodo exitosoooo!!.....")
    while not rospy.is_shutdown():
        loop.sleep
    rospy.spin()

main()

