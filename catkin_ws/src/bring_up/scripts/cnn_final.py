#!/usr/bin/env python
# encoding: utf-8
import tensorflow.compat.v1 as tf
tf.disable_v2_behavior() 
import scipy.misc
import cv2
import rospy
import model
from std_msgs.msg    import Float64
from sensor_msgs.msg import Image
from cv_bridge       import CvBridge, CvBridgeError

sess = tf.InteractiveSession()
saver = tf.train.Saver()
init_op = tf.global_variables_initializer()
sess.run(init_op)
saver.restore(sess, "save_final/model_01.ckpt")

bridge    = CvBridge()       
def callback_img(data):
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    crop_img = cv_image[330:480, 20:620]
    image = scipy.misc.imresize(cv_image[-150:], [66, 200]) / 255.0
    degrees = model.y.eval(feed_dict={model.x: [image], model.keep_prob: 1.0}, session = sess)[0][0] * 180.0 / scipy.pi
    radians = ((degrees*scipy.pi)/(180))+(-0.028)
    print(radians)
    Ster.publish(radians)
    Vel.publish(Float64(19))
    cv2.imshow("camara",crop_img)    
    cv2.waitKey(3)
def start():
    global Ster
    global Vel
    Ster = rospy.Publisher('/steering',Float64,queue_size=1)
    Vel  = rospy.Publisher('/speed', Float64,queue_size=1)
    rospy.Subscriber('/camera/rgb/raw',Image,callback_img)
    rospy.init_node('cnn_lunettes')
    print('Iniciando')
    rospy.spin()

if __name__ == '__main__':
        start()