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

#include <unistd.h>
#include <pthread.h>

#include <iostream>
#include <chrono>
#include <atomic>

#include <iostream>
//#include <chrono>
//#include <atomic>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt16.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"


#include "ros_processing.hpp"
#include "utils.hpp"


#define KEYCODE_I 0x69
#define KEYCODE_J 0x6a
#define KEYCODE_K 0x6b
#define KEYCODE_L 0x6c
#define KEYCODE_Q 0x71
#define KEYCODE_Z 0x7a
#define KEYCODE_W 0x77
#define KEYCODE_X 0x78
#define KEYCODE_E 0x65
#define KEYCODE_C 0x63
#define KEYCODE_U 0x75
#define KEYCODE_O 0x6F
#define KEYCODE_M 0x6d
#define KEYCODE_R 0x72
#define KEYCODE_V 0x76
#define KEYCODE_T 0x74
#define KEYCODE_B 0x62

#define KEYCODE_COMMA 0x2c
#define KEYCODE_PERIOD 0x2e

ros::Time t_start;

ROSDATA rosdata;
extern volatile bool die;
pthread_mutex_t ros_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t ros_cond = PTHREAD_COND_INITIALIZER;

extern unsigned int action;

// steering_pwm [950, 2150]; it seems that 1500 = 90 degrees (apparently)
// speed_pwm [-1000, 1000]; 50 seems to be OK
// steering_radians [.512, -0.498] (left, right steering in rads)
void *ros_processing_threadfunc(void *arg){


  ros::NodeHandle *n = (ros::NodeHandle *)arg;

  ros::Rate loop_rate(20);  // 1/10 Secs

  ros::Publisher pubSpeed = n->advertise<std_msgs::Float64>("/speed", 10);

  // This MUST be here
  mssleep(200); 

  ros::Publisher pubSteering = n->advertise<std_msgs::Float64>("/steering", 10);

  // This MUST be here
  mssleep(200); 

  double speed = 0;
  double steering = 0;

  std_msgs::Float64 steeringMsg;
  std_msgs::Float64 speedMsg;  

  std::cout << "Ros node running..." << std::endl;

  while (!die){

    if (action == 0){

      speed = 5;
      speedMsg.data = speed;
      pubSpeed.publish(speedMsg);

    } else
    if (action == 1){

      speed = 0;
      speedMsg.data = speed;
      pubSpeed.publish(speedMsg);
      // This MUST be here
      mssleep(1000); 

      steering = -50;
      steeringMsg.data = steering;//static_cast<int16_t>(steering);
      pubSteering.publish(steeringMsg);
      // This MUST be here
      mssleep(200);

      speed = -3;
      speedMsg.data = speed;
      pubSpeed.publish(speedMsg);
      // This MUST be here
      mssleep(9000);  
      
      speed = 0;
      speedMsg.data = speed;
      pubSpeed.publish(speedMsg);
      // This MUST be here
      mssleep(200); 

      steering = 50;
      steeringMsg.data = steering;//static_cast<int16_t>(steering);
      pubSteering.publish(steeringMsg);
      // This MUST be here
      mssleep(700); 

      speed = -3;
      speedMsg.data = speed;
      pubSpeed.publish(speedMsg);
      // This MUST be here
      mssleep(6500); 

      speed = 0;
      speedMsg.data = speed;
      pubSpeed.publish(speedMsg);
      // This MUST be here
      mssleep(1000); 

      steering = -75;
      steeringMsg.data = steering;//static_cast<int16_t>(steering);
      pubSteering.publish(steeringMsg);
      // This MUST be here
      mssleep(500); 


      speed = 5;
      speedMsg.data = speed;
      pubSpeed.publish(speedMsg);
      // This MUST be here
      mssleep(2500); 

speed = 0;
      speedMsg.data = speed;
      pubSpeed.publish(speedMsg);
      // This MUST be here
      mssleep(500); 

steering = 25;
      steeringMsg.data = steering;//static_cast<int16_t>(steering);
      pubSteering.publish(steeringMsg);
      // This MUST be here
      mssleep(500); 

      speed = -5;
      speedMsg.data = speed;
      pubSpeed.publish(speedMsg);
      // This MUST be here
      mssleep(3000); 
speed = 0;
      speedMsg.data = speed;
      pubSpeed.publish(speedMsg);
      // This MUST be here
      mssleep(500); 
      std::cout << "Aquí entró..." << std::endl;
      action = 2;
break;
    }

    loop_rate.sleep(); 
 }

  steeringMsg.data = static_cast<int16_t>(0);
  pubSteering.publish(steeringMsg);

  // This MUST be here
  mssleep(10); 

  speedMsg.data = (0);
  pubSpeed.publish(speedMsg);

  // This MUST be here
  mssleep(10); 

  std::cout << "ros_processing is about to die " << std::endl;
  std::cout.flush();     

 return NULL;

}

//
void *ros_capture_threadfunc(void *arg){

//ros::NodeHandle n;

  ros::NodeHandle *n = (ros::NodeHandle *)arg;

  ros::Rate loop_rate(10);
  ros::Subscriber gyro = n->subscribe("/gyro", 10, gyroCallback);
  ros::Subscriber gps = n->subscribe("/gps", 10, gpsCallback);
  t_start = ros::Time::now(); // This is always 0.0
  ros::Timer r = n->createTimer(ros::Duration(0.1), clockCallback);

  // This MUST be here
  mssleep(10); 

  while(ros::ok() && !die) {

    int mutex_locked = pthread_mutex_lock(&ros_mutex); 
    assert(mutex_locked == 0);

      ros::spinOnce();
 
    int mutex_cond = pthread_cond_signal(&ros_cond); 
    assert(mutex_cond == 0);
    int mutex_unlocked = pthread_mutex_unlock(&ros_mutex); 
    assert(mutex_unlocked == 0);  

    loop_rate.sleep();
  }

  int mutex_cond = pthread_cond_signal(&ros_cond); 
  assert(mutex_cond == 0);
  int mutex_unlocked = pthread_mutex_unlock(&ros_mutex); 
  assert(mutex_unlocked == 0);  

  std::cout << "ros_capture is about to die " << std::endl;
  std::cout.flush();  

 return NULL;

}


void gyroCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%i]", msg->data);
  //std::cout << "gyro Seq: " << msg->header.seq << std::endl;

    rosdata.gyroseq = msg->header.seq;
 // std::cout << "gyro Orientation x: " << msg->orientation.x << " y: " << msg->orientation.y << " z: " << msg->orientation.z << " w: " << msg->orientation.w  << std::endl;
    rosdata.gyroox = msg->orientation.x;
    rosdata.gyrooy = msg->orientation.y; 
    rosdata.gyrooz = msg->orientation.z; 
    rosdata.gyroow = msg->orientation.w; 

 // std::cout << "gyro Angular velocity x: " << msg->angular_velocity.x << " y: " << msg->angular_velocity.y  << " z: " << msg->angular_velocity.z    << std::endl;
    rosdata.gyroavx = msg->angular_velocity.x; // angular velocity 
    rosdata.gyroavy = msg->angular_velocity.y; 
    rosdata.gyroavz = msg->angular_velocity.z,

 // std::cout << "gyro Linear acceleration x: " << msg->linear_acceleration.x << " y: " <<  msg->linear_acceleration.y << " z: " << msg->linear_acceleration.z  << std::endl;
    rosdata.gyrolax = msg->linear_acceleration.x;// linear acceleration 
    rosdata.gyrolay = msg->linear_acceleration.y;  
    rosdata.gyrolaz = msg->linear_acceleration.z;


}


void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%i]", msg->data);
 // std::cout << "GPS:  " << " latitude: " << msg->latitude << " longitude: " << msg->longitude << " altitude: " << msg->altitude << std::endl;

    rosdata.gpslatitude = msg->latitude;
    rosdata.gpslongitude = msg->longitude;   
    rosdata.gpsaltitude = msg->altitude;


}

// http://wiki.ros.org/roscpp_tutorials/Tutorials/Timers
void clockCallback(const ros::TimerEvent& e)
{

   ros::Duration d_elapsed = ros::Time::now() - t_start;
   //std::cout << "Sec:  " << d_elapsed.sec << " nSec: " << d_elapsed.nsec << std::endl;
   rosdata.sec = d_elapsed.sec;
   rosdata.nsec = d_elapsed.nsec;

}


