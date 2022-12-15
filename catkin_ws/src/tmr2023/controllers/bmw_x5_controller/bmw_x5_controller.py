#!/usr/bin/env python3
import math
from vehicle import Driver
from controller import Camera, Lidar #Gyro,GPS
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, PointCloud2, PointField, NavSatFix, NavSatStatus, Imu
from rosgraph_msgs.msg import Clock

# INIT DRIVER
driver = Driver()
driver.setCruisingSpeed(0.0)                                  # SPEED CONTROL km/h - INITIAL SPEED
driver.setSteeringAngle(0.0)                                  # STEERING ANGLE - INITIAL ANGLE

# CONSTANTS
TIME_STEP = int(driver.getBasicTimeStep())

# INIT SENSORS
camera = Camera('camera')                                     # GET CAMERA FROM DEVICES
camera.enable(TIME_STEP)    

lidar = Lidar('Velodyne HDL-64E')
lidar.enable(TIME_STEP)
lidar.enablePointCloud()

# INIT GPS
# gps = GPS('gps')
# gps.enable(TIME_STEP)

# # INIT GYRO
# gyro = Gyro('gyro')
# gyro.enable(TIME_STEP)


# Cruise speed callback
def callback_cruise_speed( msg ):
  driver.setCruisingSpeed(msg.data)

# STEERING ANGLE CALLBACK
def callback_steering_angle(msg):
  driver.setSteeringAngle(-msg.data)

# MAIN FUNCTION
def main():

  # INIT ROS
  print('STARTING BMW X5 CONTROLLER NODE ...')
  rospy.init_node('bmw_x5_controller')
  rate = rospy.Rate(1000/TIME_STEP)

  print('SIMULATION FOR THE  AUTOMODELCAR LEAGUE OF THE MEXICAN ROBOTICS TOURNAMENT')
  print('TO MOVE THE VEHICLE, USE THE CORRESPONDING TOPICS')
  print('CHECK ONLINE DOCUMENTATION AT https://github.com/mnegretev/TMR-2022-AutoModelCar')
  
  # IMAGE MESSAGE
  msg_image = Image()
  msg_image.height = camera.getHeight()
  msg_image.width = camera.getWidth()
  msg_image.is_bigendian = False
  msg_image.step = camera.getWidth() * 4
  msg_image.encoding = 'bgra8'

  # POINT CLOUD2 MESSAGE 
  msg_point_cloud = PointCloud2()
  msg_point_cloud.header.stamp = rospy.Time.now()
  msg_point_cloud.header.frame_id = 'lidar_link'  
  msg_point_cloud.height = 1
  msg_point_cloud.width = lidar.getNumberOfPoints()
  msg_point_cloud.point_step = 20
  msg_point_cloud.row_step = 20 * lidar.getNumberOfPoints()
  msg_point_cloud.is_dense = False
  msg_point_cloud.fields = [
    PointField(name = 'x', offset = 0, datatype = PointField.FLOAT32, count = 1),
    PointField(name = 'y', offset = 4, datatype = PointField.FLOAT32, count = 1),
    PointField(name = 'z', offset = 8, datatype = PointField.FLOAT32, count = 1),
  ]
  msg_point_cloud.is_bigendian = False
  '''
  # GPS MESSAGE
  msg_gps = NavSatFix()
  msg_gps.header.stamp = rospy.Time.now()
  msg_gps.header.frame_id = 'gps_link'
  

  # GYRO MESSAGE
  msg_gyro = Imu()
  msg_gyro.header.stamp = rospy.Time.now()
  msg_gyro.header.frame_id = 'gyro_link
  '''
  

  # PUBLISHERS
  pub_clock        = rospy.Publisher('/clock', Clock, queue_size=1)
  pub_camera_data  = rospy.Publisher('/camera/rgb/raw', Image, queue_size=10)
  pub_point_cloud  = rospy.Publisher('/point_cloud'   , PointCloud2, queue_size=10)
  # pub_nav_gps   = rospy.Publisher('/gps', NavSatFix, queue_size=10)
  # pub_imu_gyro     = rospy.Publisher('/gyro', Imu, queue_size=10)

  # SUBSCRIBERS
  rospy.Subscriber('/speed'  , Float64, callback_cruise_speed  )
  rospy.Subscriber('/steering', Float64, callback_steering_angle)
  
  print("Using timestep=" + str(TIME_STEP) + " ms")
  time_lidar_last_reading  = driver.getTime()
  time_camera_last_reading = driver.getTime()
  while driver.step() != -1 and not rospy.is_shutdown():
    current_t = driver.getTime()
    msg_clock = Clock()
    msg_clock.clock.secs  = int(current_t)
    msg_clock.clock.nsecs = int(round(1000 * (current_t - msg_clock.clock.secs)) * 1.0e+6)
    pub_clock.publish(msg_clock)

    if (current_t - time_lidar_last_reading) >= 0.085:
      time_lidar_last_reading = current_t                            # Lidar readings are published every 100 ms
      msg_point_cloud.data = lidar.getPointCloud(data_type='buffer') # Get point cloud from lidar
      msg_point_cloud.header.stamp = rospy.Time.now()                # Stamp the current lidar reading
      pub_point_cloud.publish(msg_point_cloud)                       # Publish point cloud ros message
    if (current_t - time_camera_last_reading) >= 0.028:
      time_camera_last_reading = current_t
      msg_image.data = camera.getImage()                             # Get image data from camera
      pub_camera_data.publish(msg_image)                             # Publish image ros message
    
    '''  
    msg_gyro.angular_velocity.x = gyro.getValues()[0]                      # GET X COMPONENT FROM GYRO
    msg_gyro.angular_velocity.y = gyro.getValues()[1]                      # GET Y COMPONENT FROM GYRO
    msg_gyro.angular_velocity.z = gyro.getValues()[2]                      # GET Z COMPONENT FROM GYRO
    msg_gps.latitude = gps.getValues()[0]                                  # GET X COMPONENT FROM GPS  
    msg_gps.longitude = gps.getValues()[1]                                 # GET Y COMPONENT FROM GPS  
    msg_gps.altitude = gps.getValues()[2]  
    '''                               # GET Z COMPONENT FROM GPS 
    
    
    
    
    # pub_imu_gyro.publish(msg_gyro)                                         # PUBLISHING IMU MESSAGE
    # pub_nav_gps.publish(msg_gps)                                           # PUBLISHING NAVSATFIX MESSAGE
    rate.sleep()
    
 

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
