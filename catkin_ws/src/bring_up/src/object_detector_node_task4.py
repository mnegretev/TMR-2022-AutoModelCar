#!/usr/bin/env python

from hashlib import new
from this import d
import open3d as o3d
import numpy as np
import struct as st
import copy 
import sklearn
from sklearn.preprocessing import StandardScaler
from sklearn.cluster import DBSCAN
import pandas as pd
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2, PointField
#import ros_numpy


## Open3D version: 0.9 (?)

### PointCloud2 message:
### Header header
# 2D structure of the point cloud.
### uint32 height
### uint32 width
# Describes the channels and their layout in the binary data blob.
#PointField[] fields

#rosmsg show sensor_msgs/PointCloud2
#std_msgs/Header header
#  uint32 seq
#  time stamp
#  string frame_id
#uint32 height
#uint32 width
#sensor_msgs/PointField[] fields
#  uint8 INT8=1
#  uint8 UINT8=2
#  uint8 INT16=3
#  uint8 UINT16=4
#  uint8 INT32=5
#  uint8 UINT32=6
#  uint8 FLOAT32=7
#  uint8 FLOAT64=8
#  string name
#  uint32 offset
#  uint8 datatype
#  uint32 count
#bool is_bigendian
#uint32 point_step
#uint32 row_step
#uint8[] data
#bool is_dense

media = 0.0


def chatter_callback(message):

    #el mensaje.data contiene todos los valores en una columna 
    #de las 3 componentes x,y,z y ha almacenado 4 bits por cada una en el arreglo data
    # para extraerlos necesito un arreglo de
    # (tamano de datos/(4 * 3), 3)
    global media 
    siz = int(len(message.data)/(4*3))
    p_cloud =np.zeros((siz,3))
    #p_cloud = ros_numpy.numpify(message.data)
    c = 0

    for i in range (0, siz):
        for j in range (0,3):
            lol = message.data[c+0]+message.data[c+1]+ message.data[c+2]+message.data[c+3] # extrae los cuatro bits correspondientes a x,y o z
            lo = st.unpack('f',lol) # desempaco los valores bits en valores float entrega tupla (valor, )
             #elimino rayas extra
            if (lo[0] > 30 or lo[0] < -30):
                p_cloud[i,0] = 0.0
                p_cloud[i,1] = 0.0
                p_cloud[i,2] = 0.0
                if j == 0:
                    c += 12
                elif j == 1:
                    c += 8
                elif j == 2:
                    c += 4

                break
            else:               
                c += 4 # sumo 4 a contador y asi visitar los siguientes 4 bits
                p_cloud[i,j] = lo[0] # almaceno el valor
        

    new = p_cloud[np.intersect1d(np.where((p_cloud[:,1] < -.25)),np.where((p_cloud[:,1] > -1)))] #y
#    new = new[np.intersect1d(np.where(new[:,2] < .3),np.where(new[:,2] > -.3))]#z
    new = new[np.intersect1d(np.where(new[:,2] < .3),np.where(new[:,2] > -0.5))]#z
    new  = new[np.intersect1d(np.where(new[:,0] > 1.5),np.where(new[:,0] < 4.5))] #x 3.5

    alpha = 0.5

    if len(new) > 0:
        media = alpha * media + (1.0 -alpha)* 1.0
    else:
        media = alpha * media + (1.0 -alpha)* 0.0
    
    if media > .5: # hay auto
        p = [1.0]
        pub_pn = Float32MultiArray(data =p)
        pub.publish(pub_pn)
    else:       # no hay auto
        p = [0.0]
        pub_pn = Float32MultiArray(data = p)
        pub.publish(pub_pn)

    
    


def listener():

    global pub
    
    pub = rospy.Publisher('/obstacles', Float32MultiArray, queue_size=10)

    rospy.Subscriber("/point_cloud",PointCloud2,chatter_callback)
    rospy.init_node('object_detector_node',anonymous=True)

    rospy.spin()

if __name__ == '__main__':
    
    listener()
