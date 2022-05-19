import math

# revisar
def minus_angle(a1, a2):
    #angle = math.pi - abs(abs(a1 - a2) - math.pi); 
    angle = a1 - a2
    return angle

# corke's book kinematic controller
def get_input(current, goal, k_v, k_h):
    e_x = goal[0] - current[0]
    e_y = goal[1] - current[1]
    v = k_v * math.sqrt(e_x*e_x + e_y*e_y)
    theta_goal = math.atan2(e_y, e_x)
    gamma = k_h * minus_angle(theta_goal, current[2])
    return v, gamma
    