#!/usr/bin/env python

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



def chatter_callback(message):

    #el mensaje.data contiene todos los valores en una columna 
    #de las 3 componentes x,y,z y ha almacenado 4 bits por cada una en el arreglo data
    # para extraerlos necesito un arreglo de
    # (tamano de datos/(4 * 3), 3)

    siz = int(len(message.data)/(4*3))
    p_cloud = np.zeros((siz, 3))
    c = 0

    for i in range (0, siz):
        for j in range (0,3):
            lol = message.data[c+0]+message.data[c+1]+ message.data[c+2]+message.data[c+3] # extrae los cuatro bits correspondientes a x,y o z
            lo = st.unpack('f',lol) # desempaco los valores bits en valores float entrega tupla (valor, )
             #elimino rayas extra
            if (lo[0] > 50 or lo[0] < -50):
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
        
        ##########OPEN 3D############################## nube de puntos
    pcd = o3d.geometry.PointCloud() #creo geometria PointCloud()
    pcd.points = o3d.utility.Vector3dVector(p_cloud) # de numpy array a Pointcloud()
    pcd.remove_none_finite_points()# filtro de valores no finitos
    #rotacion y traslacion de la nube de puntos al marco de referencia del auto
    R = np.array(pcd.get_rotation_matrix_from_xyz((-np.pi/2, -np.pi/2 , np.pi)))
    pcd.rotate(R, center=True)
    pcd.translate((0,0.0,.348))
        
    p_cloud = np.asarray(pcd.points)
    del(pcd)

    new = p_cloud[p_cloud[:,2]< .25] #filtro z_up
    new = new[new[:,2] > -1.5] #filtro z_down
    new = new[new[:,1] < 10] #filtro y_up
    new = new[new[:,1] > -10] #filtro y_down
    new_1 = new[new[:,0] > 4] #filtro x_down
    new = new[new[:,0] < -2] #filtro y_down
    new_p_cloud = np.concatenate((new,new_1),axis=0)

    new_p_cloud = np.delete(new_p_cloud,2,axis=1) #modificar
    del(p_cloud)
    del(new)
    del(new_1)    

    
     #Estandarizado de datos
    X = StandardScaler().fit_transform(new_p_cloud)
        # Prepara datos en un dataframe de Pandas
    col_name = ['x' + str(idx) for idx in range(0, new_p_cloud.shape[1])]
    df = pd.DataFrame(new_p_cloud, columns=col_name)
        #DBSCAN clusterizado
    db = DBSCAN(eps=0.5, min_samples=10).fit(X)
    core_samples_mask = np.zeros_like(db.labels_, dtype=bool) #Revisar si son importantes
    core_samples_mask[db.core_sample_indices_] = True
    labels = db.labels_
    
    df['cluster_label'] = labels
    df = df.loc[(df['cluster_label'] > -1)] #purgo el ruido
    labels = labels[labels>=0]
    del(X)
    n_obj = len(np.unique(labels)) # num de objetos
    N = 5 # numero de obstaculos a detectar
    O = np.zeros((N+4*N))
    count = N
    
    for i in range (0, np.minimum(n_obj,N)):
        if i < N :
            O[i] = 1.0
        #grupo = np.array([element for element, etiqueta in zip (new_p_cloud,labels) if etiqueta != -1])
        grupo = df[df['cluster_label'] == i]
        O[count],O[count+1],_= grupo.min(axis=0)
        count += 2
        O[count],O[count+1],_= grupo.max(axis=0)
        count += 2

    del(new_p_cloud)
    del(count)
    O = Float32MultiArray(data = O)
    pub.publish(O)


    

    '''
    downpcd = pcd_copy.voxel_down_sample(voxel_size=.5) # creo voxeles
    #almacenamiento de nube voxelizada
    o3d.io.write_point_cloud("PC_voxel.pcd", downpcd, write_ascii=True)'''
    
    


def listener():

    global pub
    
    pub = rospy.Publisher('/obstacles', Float32MultiArray)

    rospy.Subscriber("/point_cloud",PointCloud2,chatter_callback)
    rospy.init_node('object_detector_node',anonymous=True)

    rospy.spin()

if __name__ == '__main__':
    
    listener()
