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
FT = 0
l = 200 
y_ref = 200
f_vis = 30.0
h = 1.0/f_vis
u = 0.0
v = 25.0
e_y_h = 0.0 
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
	x1 = 0.0
	x2 = 1.0
	X_m = np.array([[x1], [x2]]) #np.reshape(np.array([x1, x2]),(2,1))
	y1,y2 = reg.predict(X_m)
	m = (y2-y1)/(x2-x1)
	b = y2-m*x2
	e_th = np.tan(m)
	x0 = 2999.0
	y0 = 999.0
	e_y = y_ref+(y0-m*x0-b)/np.sqrt(m**2.0+1.0)
	return e_y,e_th
def tip(imagenN):
	H=np.array([[-6.56487687e-01,-4.22155788,1.22445980e+03],[1.69184144e-01,-1.46707619e+01,3.90600552e+03],[6.99108032e-05,-4.25093586e-03,1.0]]) 
	imagenH = cv2.warpPerspective(imagenN, H, (2000,3000),borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0)) 
	return imagenH
def roi_zone(x):
	assert (x>=0) and (x<=1999), 'x out of limits'
	if (x>1249) and (x<=1999):
		y = int(round(-1.752*x+5187.248))
	if (x>=764) and (x<=1249):
		y = 2999
	if (x>=0) and (x<764):
		y = int(round(1.9607*x+1501))
	return y
def vec_create(x,stride):
	j = 0
	xv = []
	for i in range(0,2*stride+1):
		if ((-1)**i==-1): j = j+1
		xv.append(x+j*(-1)**i)
	return xv
def line_verify(imagen0,j,i):
	G = True
	c = 0
	for k in range (i,i-50,-1):
		if (imagen0[j][k]==0): c = c+1
	if (c>25): G = False
	return G
def line_detector(imagen0,l): #,x1,l,side
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
	"""
	G = line_verify(imagen0,j,i)
	if(G==True):
		L.append([i,j])
		F = False
		break		
	else: break
	"""

	"""
	K = True
	stride = 7
	y1 = roi_zone(x1)
	x1v = vec_create(x1,stride)
	while (K==True):
		m = y1+stride
		if (m>2999):m = 2999
		#if (y1+stride>2992): m = 2992-y1 #280
		#else: m = stride
		for j in range(m,y1-stride,-1):
			for i in x1v:
				if imagen0[j][i]==255:
					x1 = i
					y1 = j
					K = False
					break
			#x1v = vec_create(x1,stride)
			if (K==False): break
		
		if (K==True): 
			# side=1 -> Der2Izq; side=-1 -> Izq2Der
			x1 = x1-1*side 
			y1 = roi_zone(x1)
	x2 = x1
	x2v = vec_create(x2,stride)
	for j in range(y1-1,y1-l,-1):
		for i in x2v:
			if imagen0[j][i]==255:
				x2 = i
				y2 = j
				K = False
				break
		#x2v = vec_create(x2,stride)			
	"""
	return L #x1,y1,x2,y2
#******************************************************************************************
#******************************************************************************************
#******************************************************************************************
def callback_V(data0):
	global u, v, e_y_h#, ie_y
	global FT
	global x1, x2, x1_h, x2_h
	imagen0 = bridge.imgmsg_to_cv2(data0, "bgr8") 					
	lower = np.array([0,0,0])
	upper = np.array([179,100,100])
	imagenF = cv2.inRange(cv2.cvtColor(imagen0,cv2.COLOR_BGR2HSV),lower,upper)
	imagenF = tip(imagenF)
	#kernelG = np.ones((5,5))/81	
	#imagenF = cv2.filter2D(imagenF,-1,kernelG)
	
	L = line_detector(imagenF,l)	
	e_y,e_th = fit_ransac(L,y_ref)

	#print(e_y,e_th)
	"""
	y1 = 0
	y2 = 0
	if (FT<=30):
		x1 = 1400#1250
		FT = FT+1
	else: 
		x1 = x1_h
	x1,y1,x2,y2 = line_detector(imagenF,x1,l,1)
	x1_h = x1
	x2_h = x2
	"""
	ky = -0.000025   # 0.000025 #
	kth = -0.00005 #-1.35777292e-02 #0.00005 # 
	kdy = -0.0000015  #0.0000015 
	de_y = (e_y-e_y_h)/h
	e_y_h = e_y
	u = np.arctan(ky*e_y+kth*e_th+kdy*de_y)*(180.0/np.pi)
	if (u>38.5): u = 38.5
	if (u<-38.5): u = -38.5
	print('steering ',u)

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
