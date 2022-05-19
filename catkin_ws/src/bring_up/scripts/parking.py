#!/usr/bin/env python3

"""
    NODE FOR AUTONOMOUS PARKING  
"""

# LIBRARIES
from turtle import speed
import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64

# STATES
SM_WAIT_TURN_RIGHT  = 'SM_WAIT_TURN_RIGHT'
SM_WAIT_GO_FORWARD  = 'SM_WAIT_GO_FORWARD'
SM_WAIT_TURN_LEFT   = 'SM_WAIT_TURN_LEFT'
SM_FINISH_PARKING   = 'SM_FINISH_PARKING'
SM_WAIT_GO_BACK     = 'SM_WAIT_GO_BACK'
SM_WAIT_BREAK       = 'SM_WAIT_BREAK'
SM_TURN_RIGHT       = 'SM_TURN_RIGHT'
SM_GO_FORWARD       = 'SM_GO_FORWARD'
SM_TURN_LEFT        = 'SM_TURN_LEFT'
SM_GO_BACK          = 'SM_GO_BACK'
SM_BREAK            = 'SM_BREAK'
SM_INIT             = 'SM_INIT'

# GLOBAL VARIABLES
position        = [0.0, 0.0]
positions       = []
start_parking   = None
speed           = None


# CAR POSE CALLBACK
def callback_car_pose(msg):

    global positions, position, start_parking

    if msg.poses[0].position.z != 0.0:
        position = [msg.poses[0].position.x, msg.poses[0].position.z]
    else:
        if position[0] != 0.0 and position[1] != 0.0:
            positions.append(position)
        position = [0.0, 0.0]

    # TWO CARS DETECTED
    if len(positions) == 2:
        start_parking = True
        positions.clear()
    else:
        start_parking = False

    

# MAIN FUNCTION
def main():
    global start_parking, speed

    # INIT NODE
    print('Parking Node...')
    rospy.init_node('parking_node')
    rate = rospy.Rate(10)

    # PARAMS
    if rospy.has_param('/speed'):
        speed = rospy.get_param('/speed')


    # SUBSCRIBERS
    rospy.Subscriber('/object_pose', PoseArray, callback_car_pose)

    # PUBLISHERS
    pub_speed       = rospy.Publisher('/speed', Float64, queue_size=10)
    pub_steering    = rospy.Publisher('/steering', Float64, queue_size=10)

    # FOR FIRST STATE
    state = SM_INIT
    count = 0


    while not rospy.is_shutdown():

        # STATE MACHINE TO PARKING
        if state == SM_INIT:                        # STATE INIT
            pub_speed.publish(speed)
            if start_parking:
                state = SM_BREAK
            else:
                state = SM_INIT

        elif state == SM_BREAK:                     # STATE BREAK
            pub_speed.publish(0.0)
            count = 0
            state = SM_WAIT_BREAK
        
        elif state == SM_WAIT_BREAK:                # STATE WAIT BREAK
            count += 1
            if count > 10:
                state = SM_GO_BACK
            else:
                state = SM_WAIT_BREAK
        
        elif state == SM_GO_BACK:                   # STATE GO BACK
            count = 0
            pub_speed.publish(-10.0)
            state = SM_WAIT_GO_BACK

        elif state == SM_WAIT_GO_BACK:              # STATE WAIT GO BACK
            count += 1
            if count > 20:
                state = SM_TURN_RIGHT
            else:
                state = SM_WAIT_GO_BACK

        elif state == SM_TURN_RIGHT:                # STATE TURN RIGHT BACK
            count = 0
            pub_steering.publish(0.4)
            state = SM_WAIT_TURN_RIGHT

        elif state == SM_WAIT_TURN_RIGHT:           # STATE WAIT TURN RIGHT BACK
            count += 1
            if count > 20:
                state = SM_TURN_LEFT
            else: 
                state = SM_WAIT_TURN_RIGHT
        
        elif state == SM_TURN_LEFT:                 # STATE TURN LEFT BACK
            count = 0
            pub_steering.publish(-0.4)
            state = SM_WAIT_TURN_LEFT

        elif state == SM_WAIT_TURN_LEFT:            # STATE WAIT TURN LEFT BACK
            count += 1
            if count > 20:
                state = SM_GO_FORWARD
            else:
                state = SM_WAIT_TURN_LEFT

        elif state == SM_GO_FORWARD:                # STATE GO FORWARD
            count = 0
            pub_steering.publish(0.0)
            pub_speed.publish(10.0)
            state = SM_WAIT_GO_FORWARD

        elif state == SM_WAIT_GO_FORWARD:           # STATE WAIT GO FORWARD
            count += 1
            if count > 10:
                state = SM_FINISH_PARKING
            else:
                state = SM_WAIT_GO_FORWARD

        elif state == SM_FINISH_PARKING:            # STATE FINISH PARKING
            count = 0
            pub_speed.publish(0.0)
            pub_steering.publish(0.0)
            speed = 0.0
            start_parking = False
            state = SM_INIT

        rate.sleep()

    

if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException
