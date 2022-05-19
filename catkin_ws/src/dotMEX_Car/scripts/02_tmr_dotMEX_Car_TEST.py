#! /usr/bin/env python
# Nodo para la prueba: Navegacion CON obstaculos
# Equipo: dotMEX-CAR 2022
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from std_msgs.msg import Float64

from sensor_msgs import point_cloud2 as pc2
import ros_numpy

bridge = CvBridge()
FT = 0
l = 50
x_ref = 120
x1 = 120
x2 = 120
x1_h = 120
u = 0.0
v = 5.0

Ev = False
Obj = False
k = 0

#******************************************************************************************
#******************************************************************************************
#******************************************************************************************
def tip(imagenN):
	H=np.array([[-5.88392618e-02,-4.02514041e-01,1.19565927e+02],[1.24049432e-18,-1.34260497,3.67342070e+02],[4.47714719e-21,-4.01176785e-03,1.0]])  #Banqueta
	imagenH = cv2.warpPerspective(imagenN, H, (200,300),borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0)) 
	return imagenH
#******************************************************************************************
#******************************************************************************************
#******************************************************************************************
def roi_zone(x):
	assert (x>=0) and (x<=199), 'x out of limits'
	if (x>120) and (x<=199):
		y = int(round(-1.7722*x+511.6582))
	if (x>=80) and (x<=120):
		y = 299
	if (x>=0) and (x<80):
		y = int(round(1.6875*x+164.0))
	return y
#******************************************************************************************
#******************************************************************************************
#******************************************************************************************
def vec_create(x,stride,side):
	"""
	j = 0
	xv = []
	for i in range(0,2*stride+1):
		if ((-1)**i==-1): j = j+1
		xv.append(x+j*(-1)**i)
	"""
	if(side==1):
		xi = x+stride
		xd = x-stride
	else:
		xi = x-stride
		xd = x+stride
	if(xi<0): xi = 0
	if(xi>199): xi = 299
	if(xd<0): xi = 0
	if(xd>199): xi = 299
	xv = np.arange(xi,xd,(-1)*side)
	return xv
#******************************************************************************************
#******************************************************************************************
#******************************************************************************************
def line_detector(imagen0,x1,l,side):
	K = True
	stridex = 5
	stridey = 5
	y1 = roi_zone(x1)
	x1v = vec_create(x1,stridex,side)
	while (K==True):
		m = y1+stridey
		if (m>=299): m = 299
		for j in range(m,y1-stridey,-1):
			for i in x1v:
				if imagen0[j][i]==255:
					x1 = i
					y1 = j
					K = False
					break
			x1v = vec_create(x1,stridex,side)
			if (K==False): break
		if (K==True): 
			x1 = x1-1*side
			y1 = roi_zone(x1)
	x2 = x1
	x2v = vec_create(x2,stridex,side)
	for j in range(y1-1,y1-l,-1):
		for i in x2v:
			if imagen0[j][i]==255:
				x2 = i
				y2 = j
				K = False
				break
		x2v = vec_create(x2,stridex,side)			
	return x1,y1,x2,y2
#******************************************************************************************
#******************************************************************************************
#******************************************************************************************
def callback_L(data1):
	global Ev,Obj, FT, k #, step, R_h
	#global FT, FTY, D, curl, s
	#global u,v

	pc = ros_numpy.numpify(data1)
	x_list = np.array(np.where((pc['x']>=-0.8)&(pc['x']<=0.8)))
	z_list = np.array(np.where((pc['z']>=-10.0)&(pc['z']<=-0.0)))
	#print(z_list.shape[1])
	
	if (x_list.shape[1]>=1960) and (z_list.shape[1]>=13900): Obj = True
	else: Obj = False
	if (Obj==True): 
		Ev = True
		FT = 0
	if (Ev==True): 
		k = k+1
		print('****************')
	if (k>=500):
		Ev = False
		FT = 0
		k = 0
#******************************************************************************************
#******************************************************************************************
#******************************************************************************************
def callback_V(data0):
	global u, v
	global FT 
	global x1, x2, x1_h, x_ref

	imagen0 = bridge.imgmsg_to_cv2(data0, "bgr8") 	
	imagenF = tip(imagen0)	
	lower1 = np.array([10,0,100]) 
	upper1 = np.array([30,100,150]) 
	imagenF1 = cv2.inRange(cv2.cvtColor(imagenF,cv2.COLOR_BGR2HSV),lower1,upper1)
	lower2 = np.array([0,0,100]) 
	upper2 = np.array([179,50,255]) 
	imagenF2 = cv2.inRange(cv2.cvtColor(imagenF,cv2.COLOR_BGR2HSV),lower2,upper2)
	imagenF = cv2.GaussianBlur(imagenF1+imagenF2,(5,5),0)				
	_,imagenF = cv2.threshold(imagenF,25,255,cv2.THRESH_BINARY)
	imagenF = cv2.Sobel(imagenF, cv2.CV_8U, 1, 0, ksize=5, borderType=cv2.BORDER_DEFAULT)

	y1 = 0
	y2 = 0

	if (Ev==False):	
		side = 1
		x_ref = 120
	else: 
		side = -1
		x_ref = 70
	
	if (FT<=5):
		if (Ev==False):	x1 = 180
		else: x1 = 20
		FT = FT+1
	else: 
		x1 = x1_h

	x1,y1,x2,y2 = line_detector(imagenF,x1,l,side)
	x1_h = x1

	# CONTROL
	ky = 0.02
	kth = 0.8

	e_y = x1-x_ref
	e_th = np.arctan2(x2-x1,l)
	u = np.arctan(ky*e_y+kth*e_th)
	if (u>0.83): u = 0.83
	if (u<-0.83): u = -0.83
	Vpub.publish(v) 
	Spub.publish(u)
	
#******************************************************************************************
#******************************************************************************************
#******************************************************************************************
if __name__ == '__main__':
	print("Equipo: DotMEX-CAR")
	print("Nodo inicializado: TMR_02.py")
	rospy.init_node('TMR_02',anonymous=True)												
	Vpub = rospy.Publisher('speed',Float64,queue_size=50)				 
	Spub = rospy.Publisher('steering',Float64,queue_size=50)
	rospy.Subscriber("/camera/rgb/raw",Image,callback_V)	 
	rospy.Subscriber('/point_cloud', PointCloud2, callback_L)		
	rospy.spin()
