#!/usr/bin/env python3
import numpy
from controller import Supervisor

# TIME_STEP
TIME_STEP = 33

# SUPERVISOR
robot = Supervisor()

# NODES
car_1 = robot.getFromDef('OBS-1')
#car_2 = robot.getFromDef('OBS-2')
car_3 = robot.getFromDef('OBS-3')
car_4 = robot.getFromDef('OBS-4')
car_5 = robot.getFromDef('OBS-5')

# GET START POSITION FOR EACH CAR
sp_1 = car_1.getPosition()
#sp_2 = car_2.getPosition()
#sp_3 = car_3.getPosition()
sp_4 = car_4.getPosition()
sp_5 = car_5.getPosition()
sp_1 = [sp_1[0], sp_1[1], sp_1[2] + numpy.random.uniform(-2.0, 2.0)]
#sp_2 = [sp_2[0] + numpy.random.uniform(-15.0, 15.0), sp_2[1], sp_2[2]]
#sp_3 = [sp_3[0] + numpy.random.uniform(-15.0, 15.0), sp_3[1], sp_3[2]]
sp_5 = [sp_5[0], sp_5[1], sp_5[2] + numpy.random.uniform(-5.0, 5.0)]

# GET THE TRANSLATION FIELD FOR EACH CAR
tf_1 = car_1.getField('translation')
#tf_2 = car_2.getField('translation')
#tf_3 = car_3.getField('translation')
tf_4 = car_4.getField('translation')
tf_5 = car_5.getField('translation')

tf_1.setSFVec3f(sp_1)
#tf_2.setSFVec3f(sp_2)
#tf_3.setSFVec3f(sp_3)
tf_5.setSFVec3f(sp_5)

vel_1 = [0.0, 0.0, numpy.random.uniform(1.0, 3.0), 0.0, 0.0, 0.0] 
#vel_2 = [0.0, numpy.random.uniform(3.0, 6.0), 0.0, 0.0, 0.0, 0.0] 
#vel_3 = [0.0, 0.0, numpy.random.uniform(3.0, 6.0), 0.0, 0.0, 0.0]  
vel_4 = [0.0, 0.0, numpy.random.uniform(3.0, 6.0), 0.0, 0.5, 0.0]
vel_5 = [0.0, 0.0, numpy.random.uniform(3.0, 6.0), 0.0, 0.0, 0.0] 


def main():
    print('Starting Controller Supervisor...')
    i = 0
    while robot.step(TIME_STEP) != -1:
        if i == 0:                              # SET INITIAL VELOCITY
            car_1.setVelocity(vel_1)
            #car_2.setVelocity(vel_2)
            #car_3.setVelocity(vel_3)
            #car_4.setVelocity(vel_4)
            #car_5.setVelocity(vel_5)
        elif i == 700:                          # RETURN TO INITIAL
            tf_1.setSFVec3f(sp_1)
            #tf_2.setSFVec3f(sp_2)
            #tf_3.setSFVec3f(sp_3)
            #tf_4.setSFVec3f(sp_4)
            #tf_5.setSFVec3f(sp_5)
            i = 0
        i+=1
        

if __name__ == "__main__":
    try:
        main()
    except:
        pass

