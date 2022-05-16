#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float64
from sklearn.linear_model import RANSACRegressor
ransac = RANSACRegressor()
bridge = CvBridge()

l = 800 
y_ref = 200 #280
f_vis = 30.0
h = 1.0/f_vis
e_y_h = 0.0

L_h = []

u = 0.0
v = 55.0 #50.0

#******************************************************************************************
#******************************************************************************************
#******************************************************************************************
def fit_ransac(L,y_ref):
	N = len(L)
	X = []
	Y = []
	for i in range (0,N):
		x,y = L[i]
		X.append(x)
		Y.append(y)
	X = np.reshape(np.array(X),(N,1))
	Y = np.reshape(np.array(Y),(N,1))
	reg = ransac.fit(Y,X) # Se intercambian los ejes
	xt1 = 0.0
	xt2 = 1.0
	X_m = np.array([[xt1], [xt2]]) #np.reshape(np.array([x1, x2]),(2,1))
	yt1,yt2 = reg.predict(X_m)
	m = (yt2-yt1)/(xt2-xt1)
	b = yt2-m*xt2
	e_th = np.tan(m)
	x0 = 2999.0
	y0 = 999.0
	e_y = y_ref+(y0-m*x0-b)/np.sqrt(m**2.0+1.0)
	return e_y,e_th
#******************************************************************************************
#******************************************************************************************
#******************************************************************************************
def tip(imagenN):
	#H=np.array([[-6.35525898e-01,-4.13559812,1.20532334e+03],[1.35594473e-15,-1.39846029e+01,3.76394532e+03],[4.36508411e-19,-4.13091833e-03,1.0]]) #Banqueta
	H=np.array([[-6.91532078e-01,-4.10279290,1.22043455e+03],[-7.90733282e-15,-1.40301597e+01,3.82447908e+03],[-3.11463939e-18,-4.10457750e-03,1.0]])  #Suelo
	imagenH = cv2.warpPerspective(imagenN, H, (2000,3000),borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0)) 
	return imagenH
#******************************************************************************************
#******************************************************************************************
#******************************************************************************************
def full_line_detector(imagen0,l): 
	L = []
	N = int(l/50)
	F = True
	stride = 10
	for j in range (2999, 2999-l, -N):
		for i in range(1999, 0, -stride):	
			if (imagen0[j][i]>=100):
				L.append([i,j])
				F = False
				break		
	return L
#******************************************************************************************
#******************************************************************************************
#******************************************************************************************
def callback_V(data0):
	global u, v, e_y_h
	global L_h
	imagen0 = bridge.imgmsg_to_cv2(data0, "bgr8") 					
	lower = np.array([0,0,0]) #np.array([10,0,100])
	upper = np.array([179,100,75]) #np.array([30,100,150])
	imagenF = cv2.inRange(cv2.cvtColor(imagen0,cv2.COLOR_BGR2HSV),lower,upper)
	imagenF = tip(imagenF)

	L = full_line_detector(imagenF,l)	
	
	if (len(L)>=25): L_h = L
	else: L = L_h
	
	e_y,e_th = fit_ransac(L,y_ref)
	#print(e_th)
	
	# CONTROL
	ky = -0.00279704
	kth = -0.11339012
	kdy = -0.0
	
	#	v(km/h)		ky					kth							R				Q
	# 50		-0.00221603 	-0.10240642 ++
	#	50		-0.00221403		-0.10236577 
	#	55		-0.00279704		-0.11339012				50			0.004*I++
	de_y = (e_y-e_y_h)/h
	e_y_h = e_y
	u = np.arctan(ky*e_y+kth*e_th+kdy*de_y)
	if (u>0.6981317): u = 0.6981317
	if (u<-0.6981317): u = -0.6981317
	#print('steering ',u*(180.0/np.pi))

 	#Visualizacion
	imagenF = cv2.cvtColor(imagenF,cv2.COLOR_GRAY2BGR)
	for k in range (0, len(L)): 
		x1,y1 = L[k]
		imagenF = cv2.circle(imagenF,(x1,y1),10,(0, 0, 255),-1)
	scale_percent = 20 # percent of original size
	width = int(imagenF.shape[1] * scale_percent / 100)
	height = int(imagenF.shape[0] * scale_percent / 100)
	dim = (width, height)
	resized = cv2.resize(imagenF, dim, interpolation = cv2.INTER_AREA)
	cv2.imshow('homografia',resized)	
	cv2.moveWindow("homografia", 400,20)
	cv2.waitKey(1)
	
	Vpub.publish(v) 
	Spub.publish(u)
#******************************************************************************************}
#******************************************************************************************
#******************************************************************************************
if __name__ == '__main__':
	print("Equipo: DotMEX-CAR")
	print("Nodo inicializado: TMR_01.py")
	rospy.init_node('TMR_01',anonymous=True)												
	Vpub = rospy.Publisher('speed',Float64,queue_size=15)				 
	Spub = rospy.Publisher('steering',Float64,queue_size=15)
	rospy.Subscriber("/camera/rgb/raw",Image,callback_V)	 						
	rospy.spin()
