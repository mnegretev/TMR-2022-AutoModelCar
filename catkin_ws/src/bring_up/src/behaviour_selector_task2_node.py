#!/usr/bin/env python

import math
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Int16
import planner_acker

situation = {"Free": 0,
            "Overtaking": 1,
            "Overtaked": 2,
            "Blocked": 3}

current_sit= 0


def sit_detec(O):

    front_obs, dista = front_obstacles(O)

    if front_obs == False:
        return situation["Free"]

    elif front_obs == True:
        
        if dista < 14.0 :

            return situation["Blocked"]
        
        else:
            return situation["Free"]



def front_obstacles(O):

    O = np.asarray(O,dtype=float)
    n = 5
    num_obs = int(sum(O[:n]))
    a = 4.0
    b = -1.0
    count = n
    dist = 0
    front = False
    if num_obs > 0 :
        
        for i in range (0, num_obs): #recorre por objeto 
            
            #extraemos valores minimos y maximos de x, y del objeto
            bmin_x = O[count]
            bmax_x = O[count + 2]
            bmin_y = O[count + 1]
            bmax_y = O[count + 3]
            count += 4
            
            if (bmax_x > b or bmin_x > b) :

                if(bmin_y > -a or bmax_y < a):
                    front = True
                    dist = bmin_x
                    break
            

    return front, dist


def obstacle_callback(message):

    global current_sit

    obs = np.asarray(message.data)
    
    sit = sit_detec(obs)

    if current_sit == situation["Free"]:

        if sit == situation["Blocked"]:

            current_sit = situation["Blocked"]
        
        elif sit == situation["Free"]:

            current_sit = situation["Free"]

    
    elif current_sit == situation["Blocked"]:
        pass

        '''if sit == situation["Blocked"]:

            current_sit = situation["Blocked"]
        
        elif sit == situation["Free"]:

            current_sit = situation["Free"]'''
    
    '''elif current_sit == situation["Overtaking"]:

        #continua carrill izquierdo
        qg = Float32MultiArray(data = qg_np)

    elif current_sit == situation["Overtaked"]:

        #continua carrill izquierdo'''
    


def lane_angle_callback(message):

    global current_sit,pub2, pub

    alph_int = np.asarray(message.data)
    goal = planner_acker.goal_config(float(alph_int[0]), float(alph_int[1]))
    
    if current_sit == situation["Free"]:

        qg = Float32MultiArray(data = goal)

    elif current_sit == situation["Blocked"]:

        if alph_int[0] < -40.0*math.pi/180.0:
            current_sit = situation["Overtaking"]
            qg = [20.0,8.0,(50.0*math.pi/180.0)] #muevo izquierda
            qg = Float32MultiArray(data = qg)
            
             
    
    elif current_sit == situation["Overtaking"]:

        if alph_int[1] < 30.0 : #si ya me movi a la izq me muevo a la derecha
            qg = [5.0,-3.0,(15.0*math.pi/180.0)] # muevo derecha
            qg = Float32MultiArray(data = qg)
            current_sit = situation["Overtaked"]
        else:
            qg = [3.0,0.0,0.0]
            qg = Float32MultiArray(data = qg)

    elif current_sit == situation["Overtaked"]:

        current_sit = situation["Free"]
    
    situa = current_sit
    pub2.publish(situa)
    pub.publish(qg)

    
  
    

def listener():
    
    global pub, pub2, current_sit
    
    pub = rospy.Publisher('/goal_config', Float32MultiArray, queue_size=10)
    pub2 = rospy.Publisher('/sit', Int16, queue_size=10)

    rospy.init_node('behaviour_selector_task',anonymous=True)

    rospy.Subscriber("/obstacles", Float32MultiArray, obstacle_callback)
    rospy.Subscriber('/lane_angle', Float32MultiArray, lane_angle_callback)

    rospy.spin()

if __name__ == '__main__':
    
    listener()