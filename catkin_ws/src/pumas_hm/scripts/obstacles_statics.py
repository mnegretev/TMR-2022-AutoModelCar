#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
import ros_numpy
import os
import math

flag = True
last = 0
delta = 0
count = 0
pub = 0

def callback_cloud(msg):
    global flag, delta, last, count, pub
    if flag:
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
            if -1.5 < x < 1.5 and -30.0 < z < -1.5 and -1.7 < y < -1.5:
                n_pos += 1
                x_pos += x
                y_pos += y
                z_pos += z
        if n_pos > 0:
            z_pos /= n_pos
        distance = -20.0
        print(z_pos)
        if distance < z_pos < 0:
            pub.publish(True)
        else:
            pub.publish(False)
    else:
        pub.publish(False)

def callback_flag(msg):
    global flag
    flag = msg.data

def main():
    global pub
    print("Initializing node.....")
    rospy.init_node ('software_obstacle_detector',anonymous=True)
    rospy.Subscriber("/point_cloud", PointCloud2, callback_cloud)
    rospy.Subscriber("/rebased_flag", Bool, callback_flag)
    pub = rospy.Publisher("/obstacle_flag", Bool, queue_size=10)
    loop = rospy.Rate(60)
    print("Nodo exitosoooo!!.....")
    while not rospy.is_shutdown():
        loop.sleep
    rospy.spin()

main()

