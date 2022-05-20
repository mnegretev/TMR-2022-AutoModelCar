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

# FOR CURVED LINES
SM_WAIT_ALIGN_RIGHT_1   = 'SM_WAIT_ALIGN_RIGHT_1'
SM_WAIT_GO_STRAIGHT_1   = 'SM_WAIT_GO_STRAIGHT_1'
SM_WAIT_ALIGN_LEFT_1    = 'SM_WAIT_ALIGN_LEFT_1'
SM_WAIT_TURN_LEFT_1     = 'SM_WAIT_TURN_LEFT_1'
SM_ALIGN_RIGHT_1        = 'SM_ALIGN_RIGHT_1'
SM_GO_STRAIGHT_1        = 'SM_GO_STRAIGHT_1'
SM_ALIGN_LEFT_1         = 'SM_ALIGN_LEFT_1'
SM_GO = 'SM_GO'
SM_WAIT_GO = 'SM_WAIT_GO'




# GLOBAL VARIABLES
start_overtake      = None
current_steering    = None
dynamic             = None
overtake_time       = 65

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
    
    global start_overtake, current_steering, dynamic, overtake_time
    
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
                if i > 3:
                    state = SM_FINISH_OVERTAKE
                else:
                    pub_steering.publish(0.0)
                    pub_speed.publish(20.0)
                    state = SM_TURN_LEFT
            else:
                state = SM_WAIT_NEW_OVERTAKE

        # INIT PASS ACTION
        elif state == SM_TURN_LEFT:                             # STATE TURN LEFT
            if i > 1:                                           # PARA GIRO EN CURVAS
                print('GIRANDO A LA IZQUIERDA EN CURVA')
                pub_steering.publish(current_steering - 0.60)    
                count = 0
                state = SM_WAIT_TURN_LEFT_1
            else:                                               # PARA GIRO EN RECTAS
                print('GIRANDO A LA IZQUIERDA EN RECTA')
                pub_steering.publish(current_steering - 0.35)
                count = 0
                state = SM_WAIT_TURN_LEFT
        
        ########### FOR STRAIGHT LINES ##########
        elif state == SM_WAIT_TURN_LEFT:                        # PRIMER GIRO A LA IZQUIERDA
            count += 1
            if count > 6:
                state = SM_ALIGN_RIGHT
            else:
                state = SM_WAIT_TURN_LEFT

        elif state == SM_ALIGN_RIGHT:                            # ALINEA A LA DERECHA
            print('ALINEANDO DERECHA')
            pub_steering.publish(current_steering + 0.35)
            count = 0
            state = SM_WAIT_ALIGN_RIGHT

        elif state == SM_WAIT_ALIGN_RIGHT:                       # ESPERA ALIENAR A LA DERECHA
            count += 1
            if count > 1:
                state = SM_GO
            else:
                state = SM_WAIT_ALIGN_RIGHT
            # if i == 1 and dynamic:                               # PARA REBASE DINÁMICO                  
            #     print('REBASANDO')
            #     if count > 0:
            #         pub_steering.publish(0.0)
            #         pub_speed.publish(40.0)
            #         x = 0
            #         while x < overtake_time:                    # TIEMPO DE REBASE DIÁMICO
            #             x += 1
            #         state = SM_TURN_RIGHT
            #     else: 
            #         state = SM_WAIT_ALIGN_RIGHT
            # else:                                               # PARA REBASE ESTÁTICO
            #     if count > 0:
            #         state = SM_TURN_RIGHT
            #     else:
            #         state = SM_WAIT_ALIGN_RIGHT

        elif state == SM_GO:
            print('ADELANTE')
            pub_steering.publish(0.0)
            pub_speed.publish(25.0)
            count = 0
            state = SM_WAIT_GO

        elif state == SM_WAIT_GO:                       
            count += 1
            if count > 8:
                state = SM_TURN_RIGHT
            else:
                state = SM_WAIT_GO

        elif state == SM_TURN_RIGHT:                            # SEGUNDO GIRO A LA IZQUIERDA
            print('GIRANDO A LA DERECHA')
            pub_steering.publish(current_steering + 0.6)
            count = 0
            state = SM_WAIT_TURN_RIGHT
        
        elif state == SM_WAIT_TURN_RIGHT:                       # ESPERA EL SEGUNDO GIRO A LA DERECHA
            count += 1
            if count > 8:
                state = SM_ALIGN_LEFT
            else:
                state = SM_WAIT_TURN_RIGHT
        
        elif state == SM_ALIGN_LEFT:                            # ALINEA A LA IZQUIERDA
            print('ALINEADO IZQUIERDA')
            pub_steering.publish(current_steering - 0.6)
            count = 0
            state = SM_WAIT_ALIGN_LEFT

        elif state == SM_WAIT_ALIGN_LEFT:                      # ESPERA ALINEADO A LA IZQUIERDA
            count += 1
            if count > 5:
                state = SM_GO_STRAIGHT
            else:
                state = SM_WAIT_ALIGN_LEFT


        elif state == SM_GO_STRAIGHT:                           # AVANZA EN LÍNEA RECTA
            print('AVANZANDO')
            pub_steering.publish(0.0)
            pub_speed.publish(0.0)
            count = 0
            state = SM_WAIT_GO_STRAIGHT

        
        elif state == SM_WAIT_GO_STRAIGHT:                      # ESPERA AVANZAR EN LÍNEA RECTA
            count += 1
            if count > 0:
                state = SM_FINISH_OVERTAKE
            else:
                state = SM_WAIT_GO_STRAIGHT


        ########### FOR CURVED LINES ##########
        elif state == SM_WAIT_TURN_LEFT_1:                      # GIRAR A LA IZQUIERDA              
            count += 1
            if count > 7:
                state = SM_ALIGN_RIGHT_1
            else:
                state = SM_WAIT_TURN_LEFT_1

        elif state == SM_ALIGN_RIGHT_1:                            
            print('ALINEANDO DERECHA')                          # ALINEAR A LA DERECHA
            pub_steering.publish(current_steering + 1.40)
            count = 0
            state = SM_WAIT_ALIGN_RIGHT_1

        elif state == SM_WAIT_ALIGN_RIGHT_1:                       
            count += 1
            if count > 7:
                state = SM_ALIGN_LEFT_1
            else:
                state = SM_WAIT_ALIGN_RIGHT_1

        elif state == SM_ALIGN_LEFT_1:                           
            print('ALINEADO IZQUIERDA')                         # ALINEAR A LA IZQUIERDA
            #pub_steering.publish(current_steering - 0.6)
            pub_steering.publish(-0.4)
            count = 0
            state = SM_WAIT_ALIGN_LEFT_1

        elif state == SM_WAIT_ALIGN_LEFT_1:                      
            count += 1
            if count > 6:
                state = SM_GO_STRAIGHT_1
            else:
                state = SM_WAIT_ALIGN_LEFT_1


        elif state == SM_GO_STRAIGHT_1:
            print('AVANZANDO')                                  # AVANZAR
            pub_steering.publish(0.0)
            pub_speed.publish(0.0)
            count = 0
            state = SM_WAIT_GO_STRAIGHT_1

        
        elif state == SM_WAIT_GO_STRAIGHT_1:
            count += 1
            if count > 0:
                state = SM_FINISH_OVERTAKE
            else:
                state = SM_WAIT_GO_STRAIGHT_1


        # FINISH OVRETAKE
        elif state == SM_FINISH_OVERTAKE:                       # STATE FINISH OVERTAKE
            pub_overtake_finished.publish(True)
            state = SM_WAIT_NEW_OVERTAKE

        
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException

