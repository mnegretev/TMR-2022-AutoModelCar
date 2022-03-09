#!/usr/bin/env python3

from controller import Supervisor

# TIME_STEP
TIME_STEP = 33

# SUPERVISOR
robot = Supervisor()

# NODES
car_1 = robot.getFromDef('OBS-1')
car_2 = robot.getFromDef('OBS-2')
car_3 = robot.getFromDef('OBS-3')
car_4 = robot.getFromDef('OBS-4')
car_5 = robot.getFromDef('OBS-5')

# GET INITIAL POSITION FOR EACH CAR
sp_1 = car_1.getPosition()
sp_2 = car_2.getPosition()
sp_3 = car_3.getPosition()
sp_4 = car_4.getPosition()
sp_5 = car_5.getPosition()

# GET THE TRANSLATION FIELD FOR EACH CAR
tf_1 = car_1.getField('translation')
tf_2 = car_2.getField('translation')
tf_3 = car_3.getField('translation')
tf_4 = car_4.getField('translation')
tf_5 = car_5.getField('translation')

# Z VELOCITY 
vel = [0.0, 0.0, 5.0, 0.0, 0.0, 0.0]
vel_1 = [0.0, 0.0, -5.0, 0.0, 0.0, 0.0]

def main():
    print('Starting Controller Supervisor...')
    #while robot.step(TIME_STEP) != -1:
        #tf_1.setSFVec3f([0, 0])
        

if __name__ == "__main__":
    try:
        main()
    except:
        pass

