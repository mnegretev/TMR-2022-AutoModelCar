#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import lined 
import cv2, cv_bridge
#----------------------------------------------------------------------------------------------

left_line_eq = [0.0, 11.0] #initial lines
right_line_eq = [0.0, 70.0]

def callback(msg):
    
    global left_line_eq, right_line_eq
    bridge = cv_bridge.CvBridge()
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    lines, image2show = lined.get_lines(img)
        #print(lines)
    line_equations = lined.get_line_equations(lines)

        #print(line_equations)
    idx_l, idx_r = lined.classify_line_eq(line_equations, left_line_eq, right_line_eq)

    left_line_eq, right_line_eq, image2show = lined.filter_lines(idx_l, idx_r, lines, line_equations, left_line_eq, right_line_eq, image2show)

    cline, alpha, intercept, image2show = lined.compute_center_lane(1, 1, left_line_eq, right_line_eq, image2show)
    #invertimos angulo por OpenCV xd
    alpha_pb = [-alpha, intercept]

    alpha_pb = Float32MultiArray(data = alpha_pb)
    lines_pb = bridge.cv2_to_imgmsg(image2show)
    pub.publish(alpha_pb)
    pub2.publish(lines_pb)

	


	

def detect_window():

	global pub, pub2

	pub = rospy.Publisher('/lane_angle', Float32MultiArray , queue_size=10)
	pub2 = rospy.Publisher('/lined_image',Image, queue_size=10)

	rospy.Subscriber('/camera/rgb/raw', Image, callback, queue_size = 10)

	rospy.init_node('lane_detector', anonymous = False)
	
	rospy.spin()



if __name__ == '__main__':

	detect_window()
	
