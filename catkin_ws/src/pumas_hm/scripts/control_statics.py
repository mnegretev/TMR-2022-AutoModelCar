#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64, Bool
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import math
from datetime import datetime
now = datetime.now()
filename = now.strftime("%d-%m-%Y-%H-%M-%S") + ".csv"
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
flag1 = True

def error_rho(rho_left = goal_left_rho, rho_right = goal_right_rho):
    global goal_left_rho, goal_right_rho
    e = ( 0.5 * (goal_left_rho - abs(rho_left)) ) + ( 0.5 * (goal_right_rho - abs(rho_right)) )
    return round(e, 3)

def error_theta(theta_left = goal_left_theta, theta_right = goal_right_theta):
    global goal_left_theta, goal_right_theta
    e = ( 0.5 * (goal_left_theta - abs(theta_left)) ) + ( 0.5 * (goal_right_theta - abs(theta_right)) )
    return round(e, 3)

def get_ideal_lanes():
    global left_lines, right_lines, iterator, suma_left_rho, suma_left_theta, suma_right_rho, suma_right_theta
    if len(left_lines) == 2 and len(right_lines) == 2:
        suma_rho_left = abs(left_lines[0])
        suma_left_theta += abs(left_lines[1])
        suma_rho_right = abs(right_lines[0])
        suma_right_theta += abs(right_lines[1])
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
        spd = 10
        e_theta = error_theta(theta_left)
        sentido = "L"
    elif len(left_lines) < 2 and len(right_lines) == 2:
        rho_right = right_lines[0]
        theta_right = right_lines[1]
        theta_right = theta_right
        spd = 10
        e_rho = error_rho(rho_right = rho_right)
        e_theta = - error_theta(theta_left = theta_right)
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
        spd = 10
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

def callback_parking_flag(msg):
    global flag
    flag = msg.data

def main():
    global speed_value, steering_value, filename, iterator, goal_left_rho, goal_left_theta, goal_right_rho, goal_right_theta, suma_left_rho, suma_left_theta, suma_right_rho, suma_right_theta, flag1, flag, flag1
    print("INITIALIZING LANES CONTROL NODE...")
    rospy.init_node('hardware_control', anonymous=True)
    speed = rospy.Publisher('/speed', Float64, queue_size=10)
    steering = rospy.Publisher('/steering', Float64, queue_size=10)
    obstacle_flag = rospy.Publisher('/rebased_flag', Bool, queue_size=10)
    rospy.Subscriber("/raw_lanes_left", Floats, callback_left)
    rospy.Subscriber("/raw_lanes_right", Floats, callback_right)
    rospy.Subscriber("/obstacle_flag", Bool, callback_parking_flag)
    loop = rospy.Rate(60)
    print("NODE INITIALIZED SUCCESFULLY")
    print("training")
    while not rospy.is_shutdown() and iterator < 100:
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
        elif flag or iterator < 1000:
            if iterator < 50:
                speed_value = 20.0
                steering_value = -.44
            elif 50 <= iterator < 89:
                speed_value = 25.0
                steering_value = 0.44
            elif 80 <= iterator < 180:
                speed_value = 25.0
                steering_value = 0.0
            elif 180 <= iterator < 219:
                speed_value = 25.0
                steering_value = .44
            elif 219 <= iterator < 249:
                speed_value = 20.0
                steering_value = -.44
            else:
                steering_value = 0.0
                iterator = -1
            iterator += 1
        speed.publish(speed_value)
        steering.publish(steering_value)
        loop.sleep()
    speed.publish(0.0)
    steering.publish(0.0)

main()

