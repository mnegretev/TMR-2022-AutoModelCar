import math

def goal_config(alpha, pixel_offset):
    r = 4.0
    metros_carril = 4.0 # meters
    pixeles_carril = 60.0 # pixels
    pix2meters = metros_carril/pixeles_carril

    #x = r * math.cos(alpha)
    x = r
    offset = pixel_offset * pix2meters
    y = offset
    theta = alpha
    return [x,y,theta]