#!/usr/bin/env python3

""" 
    ENABLE AND COMPUTE CONTROL LAWS (CRUISE SPEED & STEERING ANGLE) FOR PASS STATE 
"""

import rospy
from std_msgs.msg import Bool, Float64

# STATES
SM_WAIT_NEW_OVERTAKE    = 'SM_WAIT_NEW_OVERTAKE'
SM_WAIT_TURN_LEFT       = 'SM_WAIT_TURN_LEFT'
SM_WAIT_TURN_RIGHT      = 'SM_WAIT_TURN_RIGHT'
SM_WAIT_GO_STRAIGHT     = 'SM_WAIT_STRAIGHT'
SM_WAIT_ALIGN_RIGHT     = 'SM_WAIT_ALIGN_RIGHT'
SM_WAIT_ALIGN_LEFT      = 'SM_WAIT_ALIGN_LEFT'
SM_FINISH_OVERTAKE      = 'SM_FINISH_OVERTAKE'
SM_TURN_LEFT            = 'SM_TURN_LEFT'
SM_TURN_RIGHT           = 'SM_TURN_RIGHT'
SM_GO_STRAIGHT          = 'SM_GO_STRAIGHT'
SM_ALIGN_RIGHT          = 'SM_ALIGN_RIGHT'
SM_ALIGN_LEFT           = 'SM_ALIGN_LEFT'
SM_START                = 'SM_START'


# GLOBAL VARIABLES
start_overtake      = None
current_steering    = None
dynamic             = None


# CALLBACK ENABLE PASS
def callback_start_overtake(msg):
    global start_overtake
    start_overtake = msg.data

# CALLBACK CURRENT STEERING
def callback_current_steering(msg):
    global current_steering
    current_steering = msg.data


# MAIN FUNCTION
def main():
    
    global start_overtake, current_steering, dynamic
    
    print('Overtake Node...')
    rospy.init_node('overtake_node')
    rate = rospy.Rate(10)

    # PARAMS 
    if rospy.has_param('/dynamic'):
        dynamic = rospy.get_param('/dynamic')

    # SUBSCRIBERS
    rospy.Subscriber('/start_overtake', Bool, callback_start_overtake)
    rospy.Subscriber('/steering', Float64, callback_current_steering)

    # PUBLISHERS
    pub_overtake_finished = rospy.Publisher('/overtake_finished', Bool, queue_size=10)
    pub_steering = rospy.Publisher('/steering', Float64, queue_size=10)
    pub_speed = rospy.Publisher('/speed', Float64, queue_size=10)
    
    state   = SM_START
    count   = 0
    i       = 0

    while not rospy.is_shutdown():

        if state == SM_START:                                   # STATE START
            print('STATE MACHINE TO OVERTAKE')
            state = SM_WAIT_NEW_OVERTAKE
        
        elif state == SM_WAIT_NEW_OVERTAKE:                     # STATE WAIT NEW OVERTAKE
            if start_overtake:
                i += 1
                print('ACCION DE REBASE:', i)
                pub_steering.publish(0.0)
                pub_speed.publish(20.0)
                state = SM_TURN_LEFT
            else:
                state = SM_WAIT_NEW_OVERTAKE
        
        elif state == SM_TURN_LEFT:                             # STATE TURN LEFT
            print('GIRANDO A LA IZQUIERDA')
            pub_steering.publish(current_steering - 0.35)
            count = 0
            state = SM_WAIT_TURN_LEFT

        elif state == SM_WAIT_TURN_LEFT:                        # STATE WAIT TURN LEFT
            count += 1
            if count > 10:
                state = SM_ALIGN_RIGHT
            else:
                state = SM_WAIT_TURN_LEFT

        elif state == SM_ALIGN_RIGHT:                            # STATE TURN RIGHT
            print('ALINEANDO DERECHA')
            pub_steering.publish(current_steering + 0.35)
            count = 0
            state = SM_WAIT_ALIGN_RIGHT

        elif state == SM_WAIT_ALIGN_RIGHT:                       # STATE WAIT TURN RIGHT
            count += 1

            if i == 1 and dynamic:
                print('REBASANDO')
                if count > 0:
                    pub_steering.publish(0.0)
                    pub_speed.publish(30.0)
                    x = 0
                    while x < 65:
                        x += 1
                    state = SM_TURN_RIGHT
                else: 
                    state = SM_WAIT_ALIGN_RIGHT
            else:
                if count > 0:
                    state = SM_TURN_RIGHT
                else:
                    state = SM_WAIT_ALIGN_RIGHT

        elif state == SM_TURN_RIGHT:                          # STATE TURN RIGHT 2
            print('GIRANDO A LA DERECHA')
            pub_steering.publish(current_steering + 0.6)
            count = 0
            state = SM_WAIT_TURN_RIGHT
        
        elif state == SM_WAIT_TURN_RIGHT:                     # STATE WAIT TURN RIGHT 2
            count += 1
            if count > 8:
                state = SM_ALIGN_LEFT
            else:
                state = SM_WAIT_TURN_RIGHT
        
        elif state == SM_ALIGN_LEFT:                           # STATE TURN LEFT 2
            print('ALINEADO IZQUIERDA')
            pub_steering.publish(current_steering - 0.6)
            count = 0
            state = SM_WAIT_ALIGN_LEFT

        elif state == SM_WAIT_ALIGN_LEFT:                      # STATE WAIT TURN LEFT 2
            count += 1
            if count > 5:
                state = SM_GO_STRAIGHT
            else:
                state = SM_WAIT_ALIGN_LEFT


        elif state == SM_GO_STRAIGHT:
            print('AVANZANDO')
            pub_steering.publish(0.0)
            pub_speed.publish(0.0)
            count = 0
            state = SM_WAIT_GO_STRAIGHT

        
        elif state == SM_WAIT_GO_STRAIGHT:
            count += 1
            if count > 0:
                state = SM_FINISH_OVERTAKE
            else:
                state = SM_WAIT_GO_STRAIGHT

        elif state == SM_FINISH_OVERTAKE:                       # STATE FINISH OVERTAKE
            pub_overtake_finished.publish(True)
            state = SM_WAIT_NEW_OVERTAKE

        
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException

