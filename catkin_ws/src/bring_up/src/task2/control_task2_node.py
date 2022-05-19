#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Float64
import corke_acker_control


def chatter_callback(message):

    k_v = 2.0
    k_h = 1.0

    goal = np.asarray(message.data)
    current_config = np.asarray([0.0, 0.0, 0.0]) 
    speed, steer = corke_acker_control.get_input(current_config, goal, k_v, k_h)
    
    steer_pb = steer * -1.0
    speed_pb = Float64(speed)
    steer_pb = Float64(steer_pb)

    pub.publish(speed_pb)
    pubb.publish(steer_pb)
    

def listener():

    global pub #speed
    global pubb #steer
    
    pub = rospy.Publisher('/speed', Float64, queue_size=10)
    pubb = rospy.Publisher('/steering',Float64, queue_size = 10)

    rospy.init_node('control',anonymous=True)

    rospy.Subscriber("/goal_config", Float32MultiArray, chatter_callback)

    rospy.spin()

if __name__ == '__main__':
    
    listener()