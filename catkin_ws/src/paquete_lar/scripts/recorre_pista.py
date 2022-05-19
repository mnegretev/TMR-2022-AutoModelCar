#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import matplotlib
from matplotlib.pyplot import imshow
from matplotlib import pyplot as plt
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float64

#Agrega aqui todas tus variables globales
bridge = CvBridge()
vel = 0.0
direc = 0.0
situation='onNormalRoad'
lastangle = 0.0

def draw_squares_in_view (view):
    cv2.rectangle(view,(344,0),(450,128),(0,255,0),3)
    cv2.rectangle(view,(144,0),(250,128),(255,0,0),3)
    return view

def open_views (data0):
    cv_image = bridge.imgmsg_to_cv2(data0, "bgr8") #La informacion recibida por la camara es un arreglo, por lo que hay que convertirla a formato imagen
    crop_img = cv_image[330:480, 20:620] #recortamos imagen
   
    """ #Guardamos el ancho y el largo en estas variables.
    ancho = cv_image.shape[1] # colummnas 
    alto = cv_image.shape[0] # filas

    # Iniciamos la trasformación:
    pts1 = np.float32([[15,300],[0, alto], [ancho, 300], [ancho, alto]])

    ##Seleccionamos puntos de interes
    x = 200
    pts2 = np.float32([[0,0], [x, alto], [ancho, 0], [ancho-x, alto]])

    ## Se calcula la matriz para correción de perspectiva:
    M = cv2.getPerspectiveTransform(pts1, pts2)

    ## Obtenemos la imagen con corrección de perspectiva:
    crop_img1 = cv2.warpPerspective(cv_image, M, (ancho, alto))
    ancho1 = crop_img1.shape[1] # colummnas 
    alto1 = crop_img1.shape[0]
    crop_img = crop_img1[250:alto1, 20:ancho1] """

    gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY) #la filtramos a grises
    frame = gray.copy() #generamos una imagen nueva con todos los cambios y recortes
    result = crop_img.copy()
    edges = cv2.Canny(frame, 50, 150)

    """ cv2.imshow("Original", frame) #mostramos la imagen obtenida
    cv2.imshow("edges",edges)
    cv2.imshow("res",result) """
    return [frame, result, edges]
    """ sendback = {frame: frame, edges:edges, result: result}
    print(sendback)
    return sendback """

def get_lines_from_image (view):
  #view parameter should be edges
    lines = cv2.HoughLinesP(view,1,np.pi/180,20,minLineLength=30,maxLineGap=30)
    lines2 = cv2.HoughLines(view, 1, np.pi / 180, 130)

    return lines2

def get_meta_line (line) :
            rho, theta = line
            theta_grados = theta*180/ 3.1416
            angulo = 180 - (theta_grados + 90)
          
            a = np.cos(theta)
            
            b = np.sin(theta)
                
            x0 = a*rho
                
            y0 = b*rho
                
            x1 = int(x0 + 1000*(-b))
                
            y1 = int(y0 + 1000*(a))
            
            x2 = int(x0 - 1000*(-b))                
            y2 = int(y0 - 1000*(a))
            meta_line = (theta, theta_grados, abs(angulo), x1,y1,x2,y2)
            return meta_line

def processing_lines (lines, view = []):
    try:

        global direc, vel,situation, lastangle
        print(len(lines[0]))
        for line in lines[0]:
           
            """    if lastlines <=2:
                lastlines.push(meta_line)
            else :
                lastlines.remove[0]
                lastlines.push(meta_line) """
            
            meta_line =  get_meta_line(line)
            #cuando esta en curva
            if meta_line[2] < 30 and meta_line[2] > 15 :
                
                print(meta_line)
                situation='onCurve' 
                normal_angle = 34
                curve_angle_rad =  normal_angle - meta_line[2] 
                curve_angle_grad = curve_angle_rad * (3.1416/180)
                print("giroooooo")
                lastangle = curve_angle_grad * -1  
                direc =   curve_angle_grad * -1  
                vel = 5.0
                #direc =  -60.0
             
                #Vpub.publish(vel)
                #Spub.publish(direc) 
         
            #cuando esta en camino normal         
            elif meta_line[2] > 32 and  meta_line[2] < 40 :
                print(meta_line)
                situation='onNormalRoad'
                direc =  0.0  
                lastangle = 0.0

            if len(view) != 0:
                cv2.line(view,(meta_line[3],meta_line[4]), (meta_line[5],meta_line[6]), (0,0,255),2)
                return view
       
    except:
        print('no hay linea')
        #situation='onNormalRoad'
        direc =  lastangle  


   
def view_main(data0):
    global direc, vel, situation

    #agregar funcion que afecte velocidad y direccion para efectos de avance

    #agregamos un ejemplo de como obtener la imagen de la camara del auto
    ## detectar solo una linea:
    #print(lines[0, 0], "Esta es x1")


    views = open_views(data0) # (frame, result, edges)
    lines = get_lines_from_image(views[2])
    views[1] = draw_squares_in_view (views[1])
    processing_lines(lines, views[1] )

    #direc=0.0
    print(situation)
    if(situation== 'onNormalRoad'):
        vel = 30.0
    #print('steering: ', direc, 'speed: ', vel)
    Vpub.publish(vel)
    Spub.publish(direc)
   
    #cv2.imshow("Original", views[0]) #mostramos la imagen obtenida
    #cv2.imshow("edges",views[2])
    #cv2.imshow("res",views[1])
    cv2.waitKey(3) #para mantener activa la ventana de opencv

def prueba():
        print("asd")

if __name__ == '__main__':
    try:
        print("Equipo: Autos_LAR")
        print("Nodo inicializado: recorre_pista.py")
        rospy.init_node('recorre_pista',anonymous=True)	
        #incluimos funciones que nos permiten publicar en los topicos de ros											
        Vpub = rospy.Publisher('/speed',Float64,queue_size=15)				 
        Spub = rospy.Publisher('/steering',Float64,queue_size=15)
        #La siguiente funcion es para suscribirse a el topico de la camara y poder leeer sus datos
        #Incluimos la funcion callback_v que nos va a permitir ver todos los cambios de la camara en tiempo real
        rospy.Subscriber("/camera/rgb/raw",Image,view_main)	 						
        prueba()
        rospy.spin()
        
        
    except rospy.ROSInterruptException:
        pass

    cv2.destroyAllWindows()
