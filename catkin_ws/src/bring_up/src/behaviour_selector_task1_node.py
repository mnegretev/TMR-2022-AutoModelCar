#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
import planner_acker


def chatter_callback(message):

    alph_int = np.asarray(message.data)
    goal = planner_acker.goal_config(float(alph_int[0]), float(alph_int[1]))
    goal_pb = Float32MultiArray(data=goal)
    pub.publish(goal_pb)


def listener():
    
    global pub
    
    pub = rospy.Publisher('/goal_config', Float32MultiArray, queue_size=10)

    rospy.init_node('behaviour_selector_task1',anonymous=True)

    rospy.Subscriber("/lane_angle", Float32MultiArray, chatter_callback)

    rospy.spin()

if __name__ == '__main__':
    
    listener()