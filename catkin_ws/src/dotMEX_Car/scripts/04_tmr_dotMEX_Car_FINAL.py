#! /usr/bin/env python
# Nodo para la prueba: Estacionamiento en paralelo
# Equipo: dotMEX-CAR 2022
import rospy
import cv2
import numpy as np
from sklearn.linear_model import RANSACRegressor
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2 as pc2
import ros_numpy 

f_imu = 30.0 
h = 1.0/f_imu
yaw0 = 180.0
yaw_h = 0.0
Dyaw = 0.0
FTY = True
D = 45.0
k = 0
step = 0
d_ref = 5.0
u = 0.0
v = 12.5 #15.0

ransac = RANSACRegressor()
#**************************************************************************************
#**************************************************************************************
#**************************************************************************************
def steer_control0(m,b):
	d = abs(b/np.sqrt(m**2+1))
	e_y = d-d_ref
	e_th = -np.arctan(1/m)
	Ky = 0.02143299*1.5
	Kth = 0.25015021*1.5
	u = np.arctan(Ky*e_y+Kth*e_th)
	return u, d
#**************************************************************************************
#**************************************************************************************
#**************************************************************************************
def fit_ransac(X,Y):
	L = len(X)
	X = np.reshape(X,(L,1))
	Y = np.reshape(Y,(L,1))
	reg = ransac.fit(X,Y)
	x1 = 0.0
	x2 = 1.0
	X_m = np.reshape(np.array([x1, x2]),(2,1))
	y1,y2 = reg.predict(X_m)
	m = (y2-y1)/(x2-x1)
	b = y2-m*x2
	return(m,b)
#******************************************************************************************
#******************************************************************************************
#******************************************************************************************
def callback_L(data1):
	global u, v, step
	global FTY, k
	pc = ros_numpy.numpify(data1)
	if (step==0):
		y_list = np.where((pc['y']>=0.0)&(pc['y']<=0.10))
		X = []
		Z = []
		for i in y_list[0]: 
			if (pc['x'][i]>0.0) and (pc['x'][i]<8.0):
				X.append(pc['x'][i])
				Z.append(pc['z'][i])
		m,b = fit_ransac(X,Z)
		u,d = steer_control0(m,b)
		c = np.count_nonzero((pc['z']>-1.0) & (pc['z']<1.0) & (pc['x'][i]>0.0) & (pc['x'][i]<6.0))
		if (c>6000) and (abs(d-d_ref)<=0.1): #0.1
			FTY = False
			step = step+1
	if (step==1):
		u = 30.0*(np.pi/180.0) #30.0
		v = -5.0
		if (Dyaw<=-D): step = step+1
	if (step==2):
		u = -30.0*(np.pi/180.0)
		v = -5.0
		if (Dyaw>=0.0): step = step+1
	if (step==3):
		u = 0.0
		v = 2.5
		k = k+1
		if (k>=60): step = step+1
	if (step==4):
		u = 0.0
		v = 0.0
		print('Fin de la maniobra')
	Vpub.publish(v) 
	Spub.publish(u)	
#**************************************************************************************
#**************************************************************************************
#**************************************************************************************
def callback_imu(data2):
	global yaw0, yaw_h, Dyaw
	gz = data2.angular_velocity.y
	yaw = (yaw_h+h*gz)		
	if (yaw>60*np.pi) or (yaw<-60*np.pi): 
		yaw = 0.0
		yaw0 = 0.0
	yaw_h = yaw
	if (FTY==True): yaw0 = yaw
	Dyaw = (yaw0-yaw)*(180.0/np.pi)
#**************************************************************************************
#**************************************************************************************
#**************************************************************************************		
if __name__ == '__main__':
	print("Equipo: DotMEX-CAR")
	print("Nodo inicializado: TMR_04.py")
	rospy.init_node('TMR_04',anonymous=True)
	Vpub = rospy.Publisher('/speed',Float64,queue_size=15)				 
	Spub = rospy.Publisher('steering',Float64,queue_size=15)				 															 							
	rospy.Subscriber('/point_cloud', PointCloud2, callback_L)	
	rospy.Subscriber('/gyro', Imu, callback_imu)	
	rospy.spin()

