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
#car_3 = robot.getFromDef('OBS-3')
car_4 = robot.getFromDef('OBS-4')
#car_5 = robot.getFromDef('OBS-5')

# GET START POSITION FOR EACH CAR
sp_1 = car_1.getPosition()
#sp_2 = car_2.getPosition()
#sp_3 = car_3.getPosition()
sp_4 = car_4.getPosition()
#sp_5 = car_5.getPosition()
sp_1 = [sp_1[0], sp_1[1], sp_1[2] + numpy.random.uniform(-15.0, 15.0)]
#sp_2 = [sp_2[0] + numpy.random.uniform(-15.0, 15.0), sp_2[1], sp_2[2]]
#sp_3 = [sp_3[0] + numpy.random.uniform(-15.0, 15.0), sp_3[1], sp_3[2]]
#sp_5 = [sp_5[0], sp_5[1], sp_5[2] + numpy.random.uniform(-15.0, 15.0)]

# GET THE TRANSLATION FIELD FOR EACH CAR
tf_1 = car_1.getField('translation')
#tf_2 = car_2.getField('translation')
#tf_3 = car_3.getField('translation')
tf_4 = car_4.getField('translation')
#tf_5 = car_5.getField('translation')

tf_1.setSFVec3f(sp_1)
#tf_2.setSFVec3f(sp_2)
#tf_3.setSFVec3f(sp_3)
#tf_5.setSFVec3f(sp_5)

def main():
    print('Starting Controller Supervisor for Navigation with Static Obstacles...')
        

if __name__ == "__main__":
    try:
        main()
    except:
        pass

