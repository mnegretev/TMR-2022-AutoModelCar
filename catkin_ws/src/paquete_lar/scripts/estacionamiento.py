#! /usr/bin/env python
# -*- coding: utf-8 -*-

from ast import Global
from re import X
import cv2
import rospy
import numpy as np
import time
from sensor_msgs import point_cloud2 as pc2
import ros_numpy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from std_msgs.msg import Float64

bridge = CvBridge()
vel = 0.0
direc = 0.0
avance = 0
mov=0
x = 0
z = 0

Car = False
rebasa = False
k=0

def movimientos(x,z):
    global mov, vel, direc
    if(mov==0):
        #Primera posicion
        print("estoy listo para girar")
        vel= 15.0
        direc=0.0
        Vpub.publish(vel)
        Spub.publish(direc)
        time.sleep(9)
        mov=1
        #pass
        
    if(mov==1):
        #Avanza segundo auto
        #vel = 25.0
        direc = -0.8
        vel=1.0
        Vpub.publish(vel)
        Spub.publish(direc)
        time.sleep(0.3)
        vel= 0.0
        direc=0.0
        Vpub.publish(vel)
        Spub.publish(direc)
        mov = 2
    if(mov==2):
        vel = -10.0
        direc = 0.15
        Vpub.publish(vel)
        Spub.publish(direc)
        time.sleep(4)
        vel= 0.0
        direc=0.0
        Vpub.publish(vel)
        Spub.publish(direc)
        mov = 3
    if(mov==3):
        vel = 9.0
        direc = 0.9
        Vpub.publish(vel)
        Spub.publish(direc)
        time.sleep(2)
        vel= 0.0
        direc=0.0
        Vpub.publish(vel)
        Spub.publish(direc)
        mov = 4

def detecta_auto(data):
    global Car,rebasa, vel, direc, k, x, z, mov

    pc = ros_numpy.numpify(data)
    x_list = np.array(np.where((pc['x']>=-0.8)&(pc['x']<=0.8)))
    z_list = np.array(np.where((pc['z']>=-10.0)&(pc['z']<=-0.0)))
    #y_list = np.array(pc['y'])
    '''print(pc['y'])
    print(pc['x'])
    print(pc['z'])
    print("x: ", int(x_list.shape[1]/10))
    print("z: ", int(z_list.shape[1]/100))'''
    x = int(x_list.shape[1]/10)
    z = int(z_list.shape[1]/100)
    time.sleep(0.1)
    k += 1
    print(k)
    
    Vpub.publish(vel)

    #if(k==60):
     #   vel=0.0
      #  Vpub.publish(vel)
    movimientos(x,z)
    #Spub.publish(direc)
    #print("y: ", y_list.shape[1])


def obten_img(data):
    global vel, direc, rebasa
    #agregamos un ejemplo de como obtener la imagen de la camara del auto
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8") #La informacion recibida por la camara es un arreglo, por lo que hay que convertirla a formato imagen
    crop_img = cv_image[330:480, 20:620] #recortamos imagen
    gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY) #la filtramos a grises
    frame = gray.copy() #generamos una imagen nueva con todos los cambios y recortes
    result = crop_img.copy()
    #cv2.imshow("Original", frame) #mostramos la imagen obtenida
    edges = cv2.Canny(frame, 50, 150)
    #cv2.imshow("edges",edges)
    lines = cv2.HoughLinesP(edges,1,np.pi/180,40,minLineLength=30,maxLineGap=30)
    i = 0
    for x1,y1,x2,y2 in lines[0]:
        i+=1
        cv2.line(result,(x1,y1),(x2,y2),(255,0,0),5)
    
    #cv2.imshow("res",result)
    
    #if rebasa != True:
    Vpub.publish(vel)
    Spub.publish(direc)

    cv2.waitKey(3)


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
        rospy.Subscriber("/camera/rgb/raw",Image,obten_img)	 
        rospy.Subscriber('/point_cloud', PointCloud2, detecta_auto)						
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    cv2.destroyAllWindows()
