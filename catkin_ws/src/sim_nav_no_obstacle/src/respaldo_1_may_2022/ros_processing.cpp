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

ROSDATA rosdata;
extern volatile bool die;
pthread_mutex_t ros_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t ros_cond = PTHREAD_COND_INITIALIZER;

// steering_pwm [950, 2150]; it seems that 1500 = 90 degrees (apparently)
// speed_pwm [-1000, 1000]; 50 seems to be OK
// steering_radians [.512, -0.498] (left, right steering in rads)
void *ros_processing_threadfunc(void *arg){

 int kfd = 0;
 char c;
 struct termios cooked, raw;

  ros::NodeHandle *n = (ros::NodeHandle *)arg;

  ros::Rate loop_rate(10);  // 1/10 Secs

  ros::Publisher pubSpeed = n->advertise<std_msgs::Float64>("/speed", 10);

  // This MUST be here
  mssleep(50); 

  ros::Publisher pubSteering = n->advertise<std_msgs::Float64>("/steering", 10);

  // This MUST be here
  mssleep(50); 

  double speed = 0;
  double steering = 0;

  std_msgs::Float64 steeringMsg;
  steeringMsg.data = static_cast<int16_t>(steering);
  pubSteering.publish(steeringMsg);

  // This MUST be here
  mssleep(10); 

  std_msgs::Float64 speedMsg;  
  speedMsg.data = speed;
  pubSpeed.publish(speedMsg);

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Moving around:");
  puts("   u    i    o");
  puts("   j    k    l");
  puts("   m    ,    .");
  puts("");
  puts("q/z : increase/decrease max speeds by 10%");
  puts("w/x : increase/decrease only linear speed by 10%");
  puts("e/c : increase/decrease only angular speed by 10%");
  puts("");
  puts("anything else : stop");
  puts("---------------------------");

  bool alive = true;
  std::cout << "Ros node running..." << std::endl;
  int count = 0;

  bool stopped = false;
  while (!die){

      // get the next event from the keyboard
      if(read(kfd, &c, 1) < 0)
      {
        perror("read():");
        exit(-1);
      }
      //intervals:
      // Steering_pwm [950, 2150], left-right
      // Speed_pwm [-1000, 1000] backwards forward 

      switch(c)  
      {
         case KEYCODE_I:
            if (speed > -100 && speed < 100){
               if (speed < 0){  
                  speed = 0;
                  speedMsg.data = speed;
                  pubSpeed.publish(speedMsg);
                  mssleep(100); 
               }
               pubSpeed.publish(speedMsg);
               speed += 5;
               speedMsg.data = speed;
               pubSpeed.publish(speedMsg);
            } 
         break;
         case KEYCODE_K:
            speed = 0;
            speedMsg.data = speed;
            pubSpeed.publish(speedMsg);
          break;
          case KEYCODE_J:
            if (steering > -0.45){
               steering -= .05;  
               steeringMsg.data = steering;
               pubSteering.publish(steeringMsg);
               std::cout << "Steering " << steering << std::endl;
               std::cout.flush();     
            }        
          break;
          case KEYCODE_L:
            if (steering < .5){
               steering += .05;  
               steeringMsg.data = steering;
               pubSteering.publish(steeringMsg);
               std::cout << "Steering " << steering << std::endl;
               std::cout.flush();     
            } 
          break;
          case KEYCODE_COMMA:
            if (speed > -100 && speed < 100){
               if (speed > 0){  
                  speed = 0;
                  speedMsg.data = speed;
                  pubSpeed.publish(speedMsg);
                  mssleep(100); 
               } 
               speed -= 5;
               speedMsg.data = speed;
               pubSpeed.publish(speedMsg);
            } 
          break;
          case KEYCODE_T:
            die = true;
          break;
      }

    loop_rate.sleep(); 
 }

  steeringMsg.data = static_cast<int16_t>(1500);
  pubSteering.publish(steeringMsg);

  // This MUST be here
  mssleep(10); 

  speedMsg.data = (0);
  pubSpeed.publish(speedMsg);

  // This MUST be here
  mssleep(10); 

  tcsetattr(STDIN_FILENO, TCSAFLUSH, &cooked);
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

  //ros::Subscriber steering = n->subscribe("/sensors/steering", 500, steeringCallback);


  //ros::Subscriber speed = n->subscribe("/sensors/speed", 500, speedCallback);

  // This MUST be here
  mssleep(10); 

  while(ros::ok() && !die) {

    //std::cout << "Inside rose capture " << std::endl;
    //std::cout.flush();   

    ros::spinOnce();
    loop_rate.sleep();
  }

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



/*

void real_poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%i]", msg->data);
  //std::cout << "Real pose:  " << "x: " << msg->x << " y: " << msg->y << " theta: " << msg->theta << std::endl;
  int mutex_locked = pthread_mutex_lock(&ros_mutex); 
  assert(mutex_locked == 0);

    rosdata.rpx = msg->x;
    rosdata.rpy = msg->y;   
    rosdata.rptheta = msg->theta;

  int mutex_cond = pthread_cond_signal(&ros_cond); 
  assert(mutex_cond == 0);
  int mutex_unlocked = pthread_mutex_unlock(&ros_mutex); 
  assert(mutex_unlocked == 0);  

}



void speedCallback(const std_msgs::Int16::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%i]", msg->data);
  //std::cout << "Speed:  " << msg->data << std::endl;
  int mutex_locked = pthread_mutex_lock(&ros_mutex); 
  assert(mutex_locked == 0);

    rosdata.speed = msg->data;

  int mutex_cond = pthread_cond_signal(&ros_cond); 
  assert(mutex_cond == 0);
  int mutex_unlocked = pthread_mutex_unlock(&ros_mutex); 
  assert(mutex_unlocked == 0);  


}



void steeringCallback(const std_msgs::Int16::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%i]", msg->data);
  //std::cout << "Steering:  " << msg->data << std::endl;

  int mutex_locked = pthread_mutex_lock(&ros_mutex); 
  assert(mutex_locked == 0);

    rosdata.steering = msg->data;

  int mutex_cond = pthread_cond_signal(&ros_cond); 
  assert(mutex_cond == 0);
  int mutex_unlocked = pthread_mutex_unlock(&ros_mutex); 
  assert(mutex_unlocked == 0);  


}



       //std::cout << "Real pose:  " << "x: " << rosdata.rpx << " y: " << rosdata.rpy << " theta: " << rosdata.rptheta << std::endl;
       std::cout << "Speed:  " << rosdata.speed << std::endl;
       std::cout << "Steering:  " << rosdata.steering << std::endl;
       std::cout << "gyro Seq: " << rosdata.gyroseq << std::endl;
       std::cout << "gyro Orientation x: " << rosdata.gyroox << " y: " << rosdata.gyrooy << " z: " << rosdata.gyrooz << " w: " << rosdata.gyroow  << std::endl;
       std::cout << "gyro Angular velocity x: " << rosdata.gyroavx << " y: " << rosdata.gyroavy  << " z: " << rosdata.gyroavz    << std::endl;
       std::cout << "gyro Linear acceleration x: " << rosdata.gyrolax << " y: " <<  rosdata.gyrolay << " z: " << rosdata.gyrolaz  << std::endl;
      std::cout.flush();     

*/



