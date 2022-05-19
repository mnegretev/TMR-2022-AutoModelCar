#include <math.h>

#include <sstream>

#include <termios.h>
#include <assert.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h> /* for strcpy() */
#include <stdlib.h>  /* for atoi(3),rand(3) */
#include <time.h>

#include <iostream>
#include <atomic>

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt16.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"

struct ROSDATA{
  double steering;
  int speed;
  double rpx, rpy, rptheta; // real pose
  double gyroseq, 
         gyroox, 
         gyrooy, 
         gyrooz, 
         gyroow, // orientation 
         gyroavx, // angular velocity 
         gyroavy, 
         gyroavz,
         gyrolax, // linear acceleration 
         gyrolay, 
         gyrolaz;
 double gpslatitude;
 double gpslongitude;
 double gpsaltitude;  
 unsigned int sec;
 unsigned int nsec;
 
};
typedef struct ROSDATA ROSDATA; 


void *ros_processing_threadfunc(void *arg);
void *ros_capture_threadfunc(void *arg);
void gyroCallback(const sensor_msgs::Imu::ConstPtr& msg);
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
void clockCallback(const ros::TimerEvent& e);

