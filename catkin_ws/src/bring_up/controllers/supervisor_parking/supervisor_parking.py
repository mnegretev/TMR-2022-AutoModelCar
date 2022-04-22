#!/usr/bin/env python3
import numpy
from controller import Supervisor

# TIME_STEP
TIME_STEP = 33

# SUPERVISOR
robot = Supervisor()

# NODES
car_1 = robot.getFromDef('OBS-1')
car_2 = robot.getFromDef('OBS-2')

# GET START POSITION FOR EACH CAR
sp_1 = car_1.getPosition()
sp_2 = car_2.getPosition()
delta= numpy.random.uniform(-4.0, 4.0)
sp_1 = [sp_1[0], sp_1[1], sp_1[2] + delta]
sp_2 = [sp_2[0], sp_2[1], sp_2[2] + delta]

# GET THE TRANSLATION FIELD FOR EACH CAR
tf_1 = car_1.getField('translation')
tf_2 = car_2.getField('translation')

tf_1.setSFVec3f(sp_1)
tf_2.setSFVec3f(sp_2)

def main():
    print('Starting Controller Supervisor for Navigation with Static Obstacles...')
        

if __name__ == "__main__":
    try:
        main()
    except:
        pass

