#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64, Bool
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import math
from datetime import datetime

left_lines = ""
right_lines = ""
steering_value = Float64(0)
speed_value = Float64(0)
goal_left_rho = 0
goal_left_theta = 0
goal_right_rho = 0
goal_right_theta = 0
iterator = 0
suma_left_rho = 0
suma_left_theta = 0
suma_right_rho = 0
suma_right_theta = 0
flag = False
past = 0

def error_rho(rho_left = goal_left_rho, rho_right = goal_right_rho):
    global goal_left_rho, goal_right_rho
    e = ( 0.002 * (goal_left_rho - rho_left) ) + ( 0.002 * (goal_right_rho - rho_right) )
    return e

def error_theta(theta_left = goal_left_theta, theta_right = goal_right_theta):
    global goal_left_theta, goal_right_theta
    e = ( 0.18 * (goal_left_theta - theta_left) ) + ( 0.18 * (goal_right_theta - theta_right) )
    return e

def get_ideal_lanes():
    global left_lines, right_lines, iterator, suma_left_rho, suma_left_theta, suma_right_rho, suma_right_theta
    if len(left_lines) == 2 and len(right_lines) == 2:
        suma_left_rho += left_lines[0]
        suma_left_theta += left_lines[1]
        suma_right_rho += right_lines[0]
        suma_right_theta += right_lines[1]
        iterator += 1

def decide():
    global left_lines, right_lines, speed_value, steering_value, filename
    speed_value = 0.2
    steering_value = 0.0
    rho_left = 0
    theta_left = 0
    rho_right = 0
    theta_right = 0
    data = []
    sentido = ""
    e_theta = 0
    if len(left_lines) == 2 and len(right_lines) < 2:
        rho_left = left_lines[0]
        theta_left = left_lines[1]
        theta_left = theta_left
        e_rho = error_rho(rho_left)
        spd = 15
        e_theta = error_theta(theta_left)
        sentido = "L"
    elif len(left_lines) < 2 and len(right_lines) == 2:
        rho_right = right_lines[0]
        theta_right = right_lines[1]
        theta_right = theta_right
        spd = 15
        e_rho = error_rho(rho_right = rho_right)
        e_theta = - error_theta(theta_right = theta_right)
        sentido = "R"
    elif len(left_lines) == 2 and len(right_lines) == 2:
        rho_left = left_lines[0]
        theta_left = left_lines[1]
        rho_right = right_lines[0]
        theta_right = right_lines[1]
        e_rho = error_rho(rho_left, rho_right)
        e_theta = error_theta(theta_left, theta_right)
        spd = 20
        sentido = "C"
    else:
        spd = 15
        e_theta = .08
        e_rho = 0
        spd_tmp = 0
        sentido = "NA"
    strng = e_theta
    if strng >= 0.4:
        strng = 0.4
    elif strng <= -0.4:
        strng = -0.4
    return spd, strng

def callback_left(msg):
    global left_lines
    left_lines = msg.data

def callback_right(msg):
    global right_lines
    right_lines = msg.data

def callback_obstacle_flag(msg):
    global flag
    flag = msg.data

def main():
    global speed_value, steering_value, filename, iterator, goal_left_rho, goal_left_theta, goal_right_rho, goal_right_theta, suma_left_rho, suma_left_theta, suma_right_rho, suma_right_theta, flag1, flag, flag1, past
    print("INITIALIZING LANES CONTROL NODE...")
    rospy.init_node('hardware_control', anonymous=True)
    speed = rospy.Publisher('/speed', Float64, queue_size=10)
    steering = rospy.Publisher('/steering', Float64, queue_size=10)
    obstacle_flag = rospy.Publisher('/rebased_flag', Bool, queue_size=10)
    rospy.Subscriber("/raw_lanes_left", Floats, callback_left)
    rospy.Subscriber("/raw_lanes_right", Floats, callback_right)
    rospy.Subscriber("/obstacle_flag", Bool, callback_obstacle_flag)
    loop = rospy.Rate(20)
    print("NODE INITIALIZED SUCCESFULLY")
    print("training")
    while not rospy.is_shutdown() and iterator < 20:
        get_ideal_lanes()
        loop.sleep()
    goal_left_theta = suma_left_theta / iterator
    goal_left_rho = suma_left_rho / iterator
    goal_right_theta = suma_right_theta / iterator
    goal_right_rho = suma_right_rho / iterator
    print("trained")
    iterator = 0
    while not rospy.is_shutdown():
        if not flag and iterator == 0:
            iterator = 0
            speed_value, steering_value = decide()
            past = steering_value
        elif flag or iterator > 0:
            if iterator < 20:
                speed_value = 20.0
                steering_value = -.44 #- past
                print("1")
            elif 20 <= iterator < 40:
                speed_value = 20.0
                steering_value = 0.44
                print("2")
            elif 40 <= iterator < 70:
                speed_value = 25.0
                steering_value = 0.0
                print("3")
            elif 70 <= iterator < 90:
                speed_value = 20.0
                steering_value = .44
                print("4")
            elif 90 <= iterator < 110:
                speed_value = 20.0
                steering_value = -.44# - past
                print("5")
            else:
                steering_value = 0.0
                iterator = -1
                print("6")
            iterator += 1
        speed.publish(speed_value)
        steering.publish(steering_value)
        loop.sleep()
    speed.publish(0.0)
    steering.publish(0.0)

main()

