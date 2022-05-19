
from code import interact
import cv2 
import numpy as np
import math


def pivot(warped, pivot = 5, max_flip = 5, init_row = 0, end_row = 5):
    for col in range(0, max_flip):
        for row in range(init_row, end_row):
            warped[row, pivot-col] = warped[row, pivot+col]
    return warped


'''
Input: target image
'''
def get_lines(image, g_kernel = (3,3), scaled_size = (80,80)):
    #Parameters
    #Gaussian blur kernel
    
    #Canny
    threshold1=100
    threshold2=200
    
    #recuadro
    top_left  = [180, 330]
    top_right = [460, 330]
    bot_right = [690, 479] # cambie el recuadro
    bot_left  = [-50, 479]
    
    # Hough
    rho = 3 #  distance resolution in pixels of the Hough grid
    theta = (5)*np.pi/180 # angular resolution in radians of the Hough grid
    threshold = 10    # minimum number of votes (intersections in Hough grid cell)
    min_line_length = 17 #minimum number of pixels making up a line
    max_line_gap = 35  # maximum gap in pixels between connectable line segments

    try:
        line_img = np.zeros(scaled_size, dtype=np.uint8)
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        img_blur = cv2.GaussianBlur(img_gray, g_kernel, 0) # Kernel mas grande

        src_corners = np.float32([top_left, top_right, bot_right, bot_left])

        x_offset = 0
        y_offset = 0
        new_top_left  = [x_offset, y_offset]
        new_top_right = [line_img.shape[1] - x_offset, y_offset]
        new_bot_right = [line_img.shape[1] - x_offset, line_img.shape[0] - y_offset]
        new_bot_left  = [x_offset, line_img.shape[0] - y_offset]
        dst_corners = np.float32([new_top_left, new_top_right, new_bot_right, new_bot_left])

        transform_matrix = cv2.getPerspectiveTransform(src_corners, dst_corners)

        warped = cv2.warpPerspective(img_blur, transform_matrix, line_img.shape[::-1], flags=cv2.INTER_LINEAR)
        height, width = warped.shape[:2]
        center = (width/2, height/2)
        rotate_matrix = cv2.getRotationMatrix2D(center=center, angle=-90, scale=1)
        #warped = cv2.rotate(warped, cv2.cv2.ROTATE_90_CLOCKWISE) # esto lo cambie
        warped = cv2.warpAffine(src=warped, M=rotate_matrix, dsize=(width, height))

        #borrar cofre y extras
        pivot(warped, pivot = 7, max_flip = 7, init_row = 0, end_row = 5)
        pivot(warped, pivot = 3, max_flip = 4, init_row = 25, end_row = 55)
        pivot(warped, pivot = 7, max_flip = 7, init_row = 75, end_row = 80)
        #borrar cofre y extras
        pivot(warped, pivot = 1, max_flip = 2, init_row = 0, end_row = 80)
        #pivot(warped, pivot = 5, max_flip = 6, init_row = 25, end_row = 55)
        #pivot(warped, pivot = 5, max_flip = 6, init_row = 75, end_row = 80)

        edges = cv2.Canny(image=warped, threshold1=threshold1, threshold2=threshold2) # Canny Edge Detection

        # Run Hough on edge detected image
        lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]), min_line_length, max_line_gap)

        return lines, warped
    except:
        print('error in get_lines')
        return [[[]]], image


def slope2angle(m):
    x = 100.0
    y = m*x  
    theta = math.atan2(y,x)
    return theta



def filter_lines(idx_l, idx_r, lines, line_equations, left_line_eq, right_line_eq, image2show):
    d = 60.0 #pixeles between lines for prediction
    n_left_lines = len(idx_l)
    n_right_lines = len(idx_r)

    #hay lineas izquierdas
    if n_left_lines>0:
        new_left_line_eq = average_line_eq(line_equations[idx_l])

        # fusionar lecturas
        left_line_eq = average_line_eq(np.array([left_line_eq, new_left_line_eq]))

        #print("left:", left_line_eq)
        left_line = line_to_draw(left_line_eq)
        image2show = draw_lines(image2show, lines[idx_l], color=(0,150,0))
        image2show = draw_lines(image2show, [[left_line]], color=(0,255,0))

        #y no hay lineas derechas
        if n_right_lines==0:
            #calcular la linea derecha a partir de un estimado de la izquierda
            ml = left_line_eq[0]
            bl = left_line_eq[1]
            beta = math.pi - slope2angle(ml)
            r = d / math.cos(beta)
            #print("r:", r)
            new_right_line_eq = [ml, bl-r]
            print("infered right:", new_right_line_eq)

            #Fusionar lecturas
            right_line_eq = average_line_eq(np.array([right_line_eq, new_right_line_eq]))

            right_line = line_to_draw(right_line_eq)
            #image2show = draw_lines(image2show, lines[idx_r], color=(0,0,255))
            image2show = draw_lines(image2show, [[right_line]], color=(0,255,255))


    if n_right_lines>0:
        new_right_line_eq = average_line_eq(np.array(line_equations[idx_r]))
        #print("right:", right_line_eq)

        #Fusionar lecturas
        right_line_eq = average_line_eq(np.array([right_line_eq, new_right_line_eq]))

        right_line = line_to_draw(right_line_eq)
        image2show = draw_lines(image2show, lines[idx_r], color=(0,0,255))
        image2show = draw_lines(image2show, [[right_line]], color=(0,255,255))

        #y no hay linea izquierda
        if n_left_lines == 0:
            mr = right_line_eq[0]
            br = right_line_eq[1]
            beta = math.pi - slope2angle(mr)
            r = d / math.cos(beta)
            #print("r:", r)
            new_left_line_eq = [mr, br+r]
            print("infered left:", new_left_line_eq)
            
            # Fusionar lecturas
            left_line_eq = average_line_eq(np.array([left_line_eq, new_left_line_eq]))

            left_line = line_to_draw(left_line_eq)
            #image2show = draw_lines(image2show, lines[idx_r], color=(0,0,255))
            image2show = draw_lines(image2show, [[left_line]], color=(0,255,0))


    if n_left_lines == 0 and n_right_lines == 0:
        #pintar las anteriores
        left_line = line_to_draw(left_line_eq)
        image2show = draw_lines(image2show, [[left_line]], color=(0,255,0))

        right_line = line_to_draw(right_line_eq)
        image2show = draw_lines(image2show, [[right_line]], color=(0,255,255))

    return left_line_eq, right_line_eq, image2show




'''
Return indices

thr_angle in degrees
'''
def classify_line_eq(lines, L_left_prev, L_right_prev, thr_angle = 30.0, thr_b = 25, max_m = 1.72, dimx = 80.0):
    thr_angle = thr_angle * math.pi / 180.0

    L_l = []
    L_r = []

    #print("L prev:", L_left_prev)
    #print("R prev:", L_right_prev)
    m_l, b_l = L_left_prev
    m_r, b_r = L_right_prev
    
    try: 
        if lines is None or len(lines)==0:
            return L_l, L_r

        for i, line in enumerate(lines):
            m = line[0]
            b = line[1]
            #print("line: ", line)

            # check if it is not vertical
            if m<= max_m and m >= -max_m:
                cb_i = m * dimx/2 + b
            
                # check if belongs to left
                cb_l = m_l * dimx/2 + b_l
                if abs( slope2angle(m) - slope2angle(m_l) ) < thr_angle and abs(cb_i-cb_l) < thr_b:
                    L_l.append(i)
                    #print("left") 
                
                cb_r = m_r * dimx/2 + b_r
                if abs( slope2angle(m) - slope2angle(m_r) ) < thr_angle and abs(cb_i-cb_r) < thr_b:
                    L_r.append(i)
                    #print("right")
        return L_l, L_r
    except:
        print("error in line classification")
        return [], []    




def draw_lines(img, lines, color=(0,255,0)): 
    # Dibujar las lineas detectadas
    if len(img.shape)==2: # one channel 
        image2show = cv2.merge([img,img,img])
    else:
        image2show = np.copy(img)
    #print(type(lines))
    #lines, image2show = get_lines(image)
    if lines is not None:
        for line in lines:
            for x1,y1,x2,y2 in line:
                cv2.line(image2show,(x1,y1),(x2,y2),color,1) 
                
    return image2show



def get_leq(line):
    #print(line)
    try:
        x1, y1, x2, y2 = line[0]
        x1 = float(x1)
        y1 = float(y1)
        x2 = float(x2)
        y2 = float(y2)
        if(x2 == x1):
            m = float('inf')
        else:
            m = float((y2-y1)/(x2-x1))
        b = float(y1 - m * x1)
        return np.array([m, b])
    except:
        print('Error in get_leq')
        return np.array([0, 0])



def get_line_equations(lines):
    equations = []
    try:
        if len(lines>0):
            for l in lines:
                equations.append(get_leq(l))
            return np.array(equations)
        else:
            return []
    except:
        print("error in computing line equations")
        return []


def normalize_eq(eq, norm_m, norm_b):
    #print(eq[:,0] )
    M = np.array(eq[:,0]) * (1.0/norm_m)
    
    B = np.array(eq[:,1] * (1.0/norm_b))
    
    Eqq = np.array([[mm, bb] for mm, bb in zip(M, B)])
    return Eqq


def average_line_eq(equations):
    # to be finished
    #print("average of:", equations)
    try:
        if equations is not None:    
            if len(equations) > 0:
                mean_slope = np.average(equations[:,0])
                mean_intercept = np.average(equations[:,1])
                #print("mean slope:", mean_slope)
                #x1 = int(0)
                #y1 = int(dy/2)
                #x2 = int(dx)
                #y2 = int(mean_slope*dx + dy/2)
            #alpha = np.arctan((y2-dy)/dx)
            #return [x1, y1, x2, y2], alpha
            return [mean_slope, mean_intercept]
        else:
            return -1
    except:
        print('Error in getting average line')
        return -1



def line_to_draw(equation, dimx=80, dimy=80):
    try:
        m = equation[0]
        b = equation[1]
        #print("mean slope:", mean_slope)
        x1 = float(0)
        y1 = b
        x2 = float(dimx/2)
        y2 = float(m*x2 + b)
        #alpha = np.arctan((y2-dy)/dx)
        return [int(x1), int(y1), int(x2), int(y2)] #pixels
    except:
        return [0,0,0,0]
        print("Error in line draw")



'''
Alignment:
0 both
1 left
2 right
'''
def get_center_line(L_l_eq = [], L_r_eq = [], alignment = 0, dimensions = (80,80)):
    # to be finished
    alpha = 0
    intercept = 0
    alignment_offset = 28 #pixeles: parameter to investigate
    try:
        if alignment == 0:
            #print(L_l_eq, L_r_eq)
            #print([L_l_eq, L_r_eq])
            equations = np.array([L_l_eq, L_r_eq])
            #print(equations)
            [mean_slope, mean_b] = average_line_eq(equations)
            #b = dimensions[1]/2
            b = mean_b
            #print("mean slope:", mean_slope)
            x1 = 0.0
            y1 = float(b)
            x2 = float(dimensions[0]/2)
            y2 = float(mean_slope*x2 + b)
            # target angle
            alpha = np.arctan((y2-y1)/(x2-x1))
            intercept = dimensions[1]/2 - b
            return [int(x1), int(y1), int(x2), int(y2)], float(alpha), float(intercept)
        elif alignment == 2:
            #print(L_l_eq, L_r_eq)
            #print([L_l_eq, L_r_eq])
            equations = np.array([L_r_eq])
            #print(equations)
            [mean_slope, mean_b] = average_line_eq(equations)
            #b = dimensions[1]/2
            b = mean_b - alignment_offset
            #print("mean slope:", mean_slope)
            x1 = 0.0
            y1 = float(b)
            x2 = float(dimensions[0]/2)
            y2 = float(mean_slope*x2 + b)
            # target angle
            alpha = np.arctan((y2-y1)/(x2-x1))
            intercept = dimensions[1]/2 - b
            return [int(x1), int(y1), int(x2), int(y2)], float(alpha), float(intercept)
        elif alignment == 1:
            #print(L_l_eq, L_r_eq)
            #print([L_l_eq, L_r_eq])
            equations = np.array([L_l_eq])
            #print(equations)
            [mean_slope, mean_b] = average_line_eq(equations)
            #b = dimensions[1]/2
            b = mean_b + alignment_offset
            #print("mean slope:", mean_slope)
            x1 = 0.0
            y1 = float(b)
            x2 = float(dimensions[0]/2)
            y2 = float(mean_slope*x2 + b)
            # target angle
            alpha = np.arctan((y2-y1)/(x2-x1))
            intercept = dimensions[1]/2 - b
            return [int(x1), int(y1), int(x2), int(y2)], float(alpha), float(intercept)
        else:
            return [0, 0, 0, 0], alpha, intercept
    except:
        print('Error in getting center line')
        return [0, 0, 0, 0], alpha, intercept
        
        
        
def compute_center_lane(n_right_lines, n_left_lines, left_line_eq, right_line_eq, image2show):
    alpha = 0
    intercept = 0
    cline=[]

     # compute center line
    if n_right_lines>0 and n_left_lines>0:
        cline, alpha, intercept = get_center_line(L_l_eq = left_line_eq, L_r_eq= right_line_eq, alignment=0, dimensions=(80,80))
        cline = [[cline]]
        image2show = draw_lines(image2show, cline, color=(255,0,0))
        #print(cline)

    elif n_right_lines>0:
        cline, alpha, intercept = get_center_line(L_l_eq = [], L_r_eq= right_line_eq, alignment=2, dimensions=(80,80))
        cline = [[cline]]
        image2show = draw_lines(image2show, cline, color=(255,0,0))
        #print(cline)

    # left alignment
    elif n_left_lines>0:
        cline, alpha, intercept = get_center_line(L_l_eq = left_line_eq, L_r_eq= [], alignment=1, dimensions=(80,80))
        cline = [[cline]]
        image2show = draw_lines(image2show, cline, color=(255,0,0))
        #print(cline)

    return cline, alpha, intercept, image2show
