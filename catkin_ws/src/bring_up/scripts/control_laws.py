#!/usr/bin/env python3

""" 
    CLASS TO COMPUTE CONTROL LAWS FOR KEEP DISTANCE & CRUISE BEHAVIORS

    CRUISE : 
        CONSTANT SPEED, STEERING ANGLE IS CALCULATED BY THE DETECTED LINES OF CURRENT LANE
    KEEP DISTANCE : 
        CONSTANT SPEED BUT MINIMUM, STEERING ANGLE IS CALCULATED BY THE DETECTED LINES OF CURRENT LANE

"""

class Control:
    
    # INIT OBJECTS
    def __init__( self ):
        self.cruise_speed = 0.0
        self.steering_angle = 0.0

    # CONTROL LAW FOR LANE TRACKING & KEEP DISTANCE
    def control_law(self, left_border, right_border, speed):

        # NO LINES DETECTED
        if (left_border[0] == 0.0 and left_border[1] == 0.0) and (right_border[0] == 0.0 and right_border[1] == 0.0):
            self.cruise_speed = 0.0
            self.steering_angle = 0.0
        # BOTH LINES DETECTED
        elif (left_border[0] != 0.0 and left_border[1] != 0.0) and (right_border[0] != 0.0 and right_border[1] != 0.0):
            self.steering_angle = self.compute_steering_angle_avg(left_border, right_border)
            self.cruise_speed = speed
        # LEFT LINES DETECTED
        elif (left_border[0] != 0.0 and left_border[1] != 0.0) and (right_border[0] == 0 or right_border[1] == 0.0):
            self.steering_angle = self.compute_steering_angle(left_border, right_border, True)
            self.cruise_speed = speed
        # RIGHT LINES DETECTED
        elif (left_border[0] == 0.0 or left_border[1] == 0.0) and (right_border[0] != 0 and right_border[1] != 0.0):
            self.steering_angle = self.compute_steering_angle(left_border, right_border, False)
            self.cruise_speed = speed

    # CONPUTE STEERING ANGLE BY SIDE
    def compute_steering_angle(self, left_border, right_border, side):
        kd = 0.0045                                                                  # CONSTANT FOR DISTANCE ERROR
        ka = 0.01                                                                   # CONSTANT FOR ANGLE ERROR
    
        detected_distance, detected_angle = [0.0, 0.0]                              # INITIAL STATE FOR DETECTED MEASURES
        goal_distance, goal_angle = [0.0, 0.0]                                      # INTIAL STATE FOR GOAL MEASURES
        steering = 0.0                                                              # INITIAL STATE FOR STEERING

        if side:                                                                    # IF ONLY THERE ARE LEFT LINES
            detected_distance, detected_angle = left_border                           # DETECTED MEASURES FOR LEFT LINES
            goal_distance, goal_angle = [190.8952592391964, 0.7086262721276703]     # GOAL MEASURES FOR LEFT LINES

        else:                                                                       # IF ONLY THERE ARE RIGHT LINES
            detected_distance, detected_angle = right_border                        # DETECTED MEASURES FOR RIGHT LINES
            goal_distance, goal_angle = [194.36306233438492, 0.6955929768432806]    # GOAL MEASURES FOR RIGHT LINES

        ed = goal_distance - detected_distance                                      # CALCULATE DISTANCE ERROR
        ea = goal_angle - detected_angle                                            # CALCULATE ANGLE ERROR

        if ed == 0.0 or ea == 0.0:                                                  # THE CAR IS ALIGNED 
            steering = 0.0
        elif side:                                                                  # THE CAR IS NOT ALIGNED
            steering = (kd * ed) + (ka * ea)                                        # CALCULATE STEERING ACCORDING TO LEFT LINES
        else:                                                                       # THE CAR IS NOT ALIGNED
            steering = (-1) * ((kd * ed) + (ka * ea))                               # CALCULATE STEERING ACCORDING TO RIGHT LINES

        return steering                                                             # RETURN THE CORRESPOND STEERING

    # COMPUTE AVG STEERING ANGLE (BOTH LINES)
    def compute_steering_angle_avg(self, left_border, right_border):
        kd = 0.00001
        ka = 0.01

        detec_dist_left, detec_angle_left = left_border                               # DETECTED MEASURES FOR LEFT LINES
        detec_dist_right, detec_angle_right = right_border                            # DETECTED MEASURES FOR RIGHT LINES

        avg_detec_dist = (detec_dist_left + detec_dist_right)/2                       # AVG OF DETECTED DISTANCE 
        avg_detec_angle = (detec_angle_left + detec_angle_right)/ 2                   # AVG OF DETECTED ANGLE 

        # CHECK IN FIRST FRAME
        goal_dist_left, goal_angle_left = [190.8952592391964, 0.7086262721276703]     # GOAL MEASURES FOR LEFT LINES
        goal_dist_right, goal_angle_right = [194.36306233438492, 0.6955929768432806]  # GOAL MEASURES FOR RIGHT LINES

        avg_goal_dist = (goal_dist_left + goal_dist_right)/2.0                        # AVG OF GOAL DISTANCE
        avg_goal_angle = (goal_angle_left + goal_angle_right)/2.0                     # AVG OF GOAL ANGLE

        ed = avg_goal_dist - avg_detec_dist                                           # CALCULATE DISTANCE ERROR
        ea = avg_goal_angle - avg_detec_angle                                         # CALCULATE ANGLE ERROR

        if ed == 0.0 or ea == 0.0:                                                    # THE CAR IS ALIGNED
            steering = 0.0                                                                  
        else:
            steering = (kd * ed) + (ka * ea)                                          # CALCULATE STEERING ACCORDING THE AVGS

        return steering                                                               # RETURN THE CORRESPOND ANGLE
        
        
        

