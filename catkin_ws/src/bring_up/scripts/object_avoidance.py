#!/usr/bin/env python3

"""
    NODE TO CREATE A STATES MACHINE AND CHOOSE THE CORRECT BEHAVIOR
    (PASS, CRUISE, KEEP DISTANCE), ENABLE THE CORRESPOND STATE
"""

# LIBRARIES
import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool, Float64

# STATES
SM_LANE_TRACKING    = 'SM_LANE_TRACKING'
SM_START_OVERTAKE   = 'SM_START_OVERTAKE'
SM_WAIT_OVERTAKE    = 'SM_WAIT_OVERTAKE'
SM_INIT             = 'SM_INIT'
SM_FINISH           = 'SM_FINISH'

# GLOBAL VARIABLES
overtake_finished = None
car_detected = None

# CAR POSE CALLBACK
def callback_car_pose(msg):
    global car_detected

    if msg.poses[0].position.z != 0.0 and msg.poses[0].position.z > -15.0:
        car_detected = True
    else:
        car_detected = False
    

# OVERTAKE FINISHED CALLBACK
def callback_overtake_finished(msg):
    global overtake_finished
    overtake_finished = msg.data


# MAIN FUNCTION
def main():
    global overtake_finished, car_detected

    # INIT NODE
    print('Object Avoidance Node...')
    rospy.init_node('object_avoidance')
    rate = rospy.Rate(10)

    # SUBSCRIBERS
    rospy.Subscriber('/object_pose', PoseArray, callback_car_pose)
    rospy.Subscriber('/overtake_finished', Bool, callback_overtake_finished)

    # PUBLISHERS
    pub_enable_LT = rospy.Publisher('/enable_LT', Bool, queue_size=10)
    pub_start_overtake = rospy.Publisher('/start_overtake', Bool, queue_size=10)

    # STATE MACHINE
    state = SM_INIT

    while not rospy.is_shutdown():

        if state == SM_INIT:                                # STATE INIT 
            print('INIT STATE MACHINE AVOIDANCE')
            state = SM_LANE_TRACKING

        elif state == SM_LANE_TRACKING:                     # STATE LANE TRACKING
            if car_detected:
                state = SM_START_OVERTAKE
            else:
                pub_enable_LT.publish(True)                 # ENABLE LANE TRACKING
                state = SM_LANE_TRACKING
        
        elif state == SM_START_OVERTAKE:                    # STATE START OVERTAKE
            pub_start_overtake.publish(True)                # START OVERTAKE
            pub_enable_LT.publish(False)                    # DISABLE LANE TRACKING
            state = SM_WAIT_OVERTAKE

        elif state == SM_WAIT_OVERTAKE:                     # STATE WAIT OVERTAKE
            pub_start_overtake.publish(False)               # START OVERTAKE
            if overtake_finished:
                state = SM_FINISH
            else:
                state = SM_WAIT_OVERTAKE
        
        elif state == SM_FINISH:                            # STATE OVERTAKE FINISHED
            overtake_finished = False
            state = SM_LANE_TRACKING

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException
