#! /usr/bin/env python3

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

#agrega aqui todas tus funciones


#incluimos nuestra funcion del Callback()

def callback_v(data0):
    global direc, vel

    #agregar funcion que afecte velocidad y direccion para efectos de avance

    #agregamos un ejemplo de como obtener la imagen de la camara del auto
    cv_image = bridge.imgmsg_to_cv2(data0, "bgr8") #La informacion recibida por la camara es un arreglo, por lo que hay que convertirla a formato imagen
    #crop_img = cv_image[330:480, 20:620] #recortamos imagen
    #Guardamos el ancho y el largo en estas variables.
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

    ## Mostramos la imagen resultante:

    crop_img = crop_img1[250:alto1, 20:ancho1]

    gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY) #la filtramos a grises
    frame = gray.copy() #generamos una imagen nueva con todos los cambios y recortes
    result = crop_img.copy()
    cv2.imshow("Original", frame) #mostramos la imagen obtenida
    edges = cv2.Canny(frame, 50, 150)
    cv2.imshow("edges",edges)
    lines = cv2.HoughLinesP(edges,1,np.pi/180,20,minLineLength=30,maxLineGap=30)
    ## detectar solo una linea:
    #print(lines[0, 0], "Esta es x1")
    i = 0
    
    lines2 = cv2.HoughLines(edges, 1, np.pi / 180, 130)
    """
    for line in lines2[0]:
        rho, theta = line
        theta_grados = theta*180/ 3.1416
        angulo = 180 - (theta_grados + 90)
        x = (theta, theta_grados, abs(angulo))
        print(x)
        
     """
    
    try:
     print(len(lines2))
     for line in lines2:
            rho, theta = line[0]
            theta_grados = theta*180/ 3.1416
            angulo = 180 - (theta_grados + 90)
          
            
            
            
            
            # Stores the value of cos(theta) in a
            a = np.cos(theta)
            
            # Stores the value of sin(theta) in b
            b = np.sin(theta)
                
            # x0 stores the value rcos(theta)
            x0 = a*rho
                
            # y0 stores the value rsin(theta)
            y0 = b*rho
                
            # x1 stores the rounded off value of (rcos(theta)-1000sin(theta))
            x1 = int(x0 + 1000*(-b))
                
            # y1 stores the rounded off value of (rsin(theta)+1000cos(theta))
            y1 = int(y0 + 1000*(a))
            
            # x2 stores the rounded off value of (rcos(theta)+1000sin(theta))
            x2 = int(x0 - 1000*(-b))
                
            # y2 stores the rounded off value of (rsin(theta)-1000cos(theta))
            y2 = int(y0 - 1000*(a))
                
            # cv2.line draws a line in img from the point(x1,y1) to (x2,y2).
            # (0,0,255) denotes the colour of the line to be
            #drawn. In this case, it is red.
            x = (theta, theta_grados, abs(angulo), x1,y1,x2,y2)
            
            cv2.line(result,(x1,y1), (x2,y2), (0,0,255),2)
            if(x1>0 and x1 < 380):
                print(x)
               

                if x[2] < 80 : 
                    print("giroooooo")
                    direc =  -60.0
                    vel = 1.0
    
                else :
                    direc =  0.0  

                
    except:
     pass
     #print("An exception occurred")
    """
    try:
        for line in lines2:
            rho, theta = line[0]
            theta_grados = theta*180/ 3.1416
            angulo = 180 - (theta_grados + 90)
            x = (theta, theta_grados, abs(angulo))
            print(x)
            
            
            # Stores the value of cos(theta) in a
            a = np.cos(theta)
            
            # Stores the value of sin(theta) in b
            b = np.sin(theta)
                
            # x0 stores the value rcos(theta)
            x0 = a*rho
                
            # y0 stores the value rsin(theta)
            y0 = b*rho
                
            # x1 stores the rounded off value of (rcos(theta)-1000sin(theta))
            x1 = int(x0 + 1000*(-b))
                
            # y1 stores the rounded off value of (rsin(theta)+1000cos(theta))
            y1 = int(y0 + 1000*(a))
            
            # x2 stores the rounded off value of (rcos(theta)+1000sin(theta))
            x2 = int(x0 - 1000*(-b))
                
            # y2 stores the rounded off value of (rsin(theta)-1000cos(theta))
            y2 = int(y0 - 1000*(a))
                
            # cv2.line draws a line in img from the point(x1,y1) to (x2,y2).
            # (0,0,255) denotes the colour of the line to be
            #drawn. In this case, it is red.
            cv2.line(result,(x1,y1), (x2,y2), (0,0,255),2)
    except:
	    print("Not a list")
    
    """
    


    """
    for x1,y1,x2,y2 in lines[i]:
        if lines[i].size:
            print("Esta mal")
        i+=1
        if(x1>200 and x1 < 400):
            print(x1,"   ",y1,"   ",x2,"   ",y2)
            cv2.line(result,(x1,y1),(x2,y2),(255,0,0),5)
    """
    cv2.imshow("res",result)

    #publicamos el avance del auto de los topicos con los valores que asignemos
    direc=0.0
    vel = 10.0
    #print('steering: ', direc, 'speed: ', vel)
    Vpub.publish(vel)
    Spub.publish(direc)

    cv2.waitKey(3) #para mantener activa la ventana de opencv



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
        rospy.Subscriber("/camera/rgb/raw",Image,callback_v)	 						
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    cv2.destroyAllWindows()
