#!/usr/bin/env python

import random
from random import uniform
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Int16
import planner_acker
import time

situation = {"Free": 0,
            "Overtaking": 1,
            "Overtaked": 2,
            "Blocked": 3}

current_sit= 0


def disc_sit_detec(O):

    discrets, dista= discret(O)

    if discrets[2] == True:
        return situation["Blocked"]
    elif discrets[1] == True or discrets[8] == True:
        return situation["Overtaking"]
    elif discrets[7] == True:
        return situation["Overtaking"]
       




def discret(O):

    O = np.asarray(O,dtype=float)
    n = 5
    num_obs = int(sum(O[:n]))
    a_x = 2.0
    a_y = 1.5
    celdas = np.ones((num_obs,9))
    count = n
    dist = 0
    discr = np.zeros(9)
    if num_obs > 0 :
        
        for i in range (0, num_obs): #recorre por objeto 
            
            #extraemos valores minimos y maximos de x, y del objeto
            bmin_x = O[count]
            bmax_x = O[count + 2]
            bmin_y = O[count + 1]
            bmax_y = O[count + 3]
            count += 4
            
            if bmin_y > a_y :
                temp = np.zeros(9)
                temp[[3,4,5]] = True
                celdas[i,:] = np.logical_and(celdas[i,:], temp)
            elif bmin_y > -a_y :
                temp = np.zeros(9)
                temp[[3,4,5,6,2]] = True
                celdas[i,:] = np.logical_and(celdas[i,:], temp)


            if bmax_y  <- a_y :
                temp = np.zeros(9)
                temp[[1,8,7]] = True
                celdas[i,:] = np.logical_and(celdas[i,:], temp)
            elif bmax_y < a_y :
                temp = np.zeros(9)
                temp[[1,2,8,6,7]] = True
                celdas[i,:] = np.logical_and(celdas[i,:], temp)


            # verificar para x

            if bmin_x > a_x :
                temp = np.zeros(9)
                temp[[1,2,3]] = True
                celdas[i,:] = np.logical_and(celdas[i,:], temp)
            elif bmin_x > -a_x :
                temp = np.zeros(9)
                temp[[1,2,3,4,8]] = True
                celdas[i,:] = np.logical_and(celdas[i,:], temp)

            if bmax_x  <- a_x :
                temp = np.zeros(9)
                temp[[5,6,7]] = True
                celdas[i,:] = np.logical_and(celdas[i,:], temp)
            elif bmax_x < a_x :
                temp = np.zeros(9)
                temp[[4,8,5,6,7]] = True
                celdas[i,:] = np.logical_and(celdas[i,:], temp)
            
            discr = np.logical_or(celdas[i,:], discr)
            
            #consultar
            if celdas[i,2] == True:
                dist = bmin_x
        del(temp)
        del(celdas)
    return discr, dist


def obstacle_callback(message):

    global current_sit

    obs = np.asarray(message.data)
    
    sit = disc_sit_detec(obs)

    if current_sit == situation["Free"]:

        if sit == situation["Blocked"]:

            current_sit = situation["Blocked"]
        
        elif sit == situation["Overtaked"]:

            current_sit = situation["Overtaked"]

    elif current_sit == situation["Overtaking"]:
        
        if sit == situation["Overtaked"]:

            current_sit = situation["Overtaked"]


        

    


def lane_angle_callback(message):

    global current_sit, pub, pub2, init_time

    qg = []
    alpha, b = np.asarray(message.data)
    b_metros = b*4.0/60.0
    goal = planner_acker.goal_config(float(alpha), float(b))
    
    if current_sit == situation["Free"]:

        qg = Float32MultiArray(data = goal)
        pub.publish(qg)

    elif current_sit == situation["Blocked"]:
        #inicio rebase
        current_sit = situation["Overtaking"]

    elif current_sit == situation["Overtaking"]:
        
        dx = 4.0
        offset = 4.0
        dy = offset + b_metros
        p_l = [[dx],[dy]]

        theta = alpha
        c, s = np.cos(theta), np.sin(theta)
        R_l_a_v = np.array(((c, -s), (s, c)))

        goal_p = np.matmul(R_l_a_v, p_l)
        g_x = goal_p[0,0]
        g_y = goal_p[1,0]

        qg = [g_x, g_y, 0.0]
        
        qg = Float32MultiArray(data = qg)
        pub.publish(qg)

        #if abs(dy) <= 0.5 :
        #    current_sit = situation["Overtaked"]

        prob = random.uniform(0,1.0)
        if prob < 0.009:
            current_sit = situation["Overtaked"]

    elif current_sit == situation["Overtaked"]:

        qg = Float32MultiArray(data = goal)
        current_sit = situation["Free"]
        pub.publish(qg)

    situa = current_sit
    pub2.publish(situa)

    
  
    

def listener():
    
    global pub, pub2, current_sit
    
    pub = rospy.Publisher('/goal_config', Float32MultiArray, queue_size=1)
    pub2 = rospy.Publisher('/sit', Int16, queue_size=1)

    rospy.init_node('behaviour_selector_task',anonymous=True)

    rospy.Subscriber("/obstacles", Float32MultiArray, obstacle_callback)
    rospy.Subscriber('/lane_angle', Float32MultiArray, lane_angle_callback)

    rospy.spin()

if __name__ == '__main__':
    
    time.sleep(10)
    listener()
    