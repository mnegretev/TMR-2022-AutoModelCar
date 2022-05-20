/*
# Released under MIT License

Copyright (c) 2022 Héctor Hugo Avilés Arriaga.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <rosgraph_msgs/Clock.h>


#include "image_processing.hpp"
#include "utils.hpp"
#include "segmentation.hpp"
#include "glwindow.hpp"

// Global
// HHAA: This is the function pointer type for window position callbacks. A window position callback function has the following signature: void callback_name(GLFWwindow* window, int xpos, int ypos)
volatile unsigned int width = 640;
volatile unsigned int height = 480;
extern volatile bool die;

// Shared OpenCV images
cv::Mat orig_image(cv::Size(width, height), CV_8UC3);

pthread_mutex_t img_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t img_cond = PTHREAD_COND_INITIALIZER;

// HHAA
void *image_processing_threadfunc(void *){

    // Define the images that will be used first
    cv::Mat rgb_img;
    cv::Mat colorized_depth;
           
    cv::Rect myROI(0, 0, 640, 400); 

    glfwNamedWindow("RGB video"); 
                   
    int i = 0;
    while (!die) {
      int mutex_locked = pthread_mutex_lock(&img_mutex); 
      assert(mutex_locked == 0);

        int cond_wait = pthread_cond_wait(&img_cond, &img_mutex); 
        assert(cond_wait == 0);
        if (die) break; 

        rgb_img = orig_image.clone();

      int mutex_unlocked = pthread_mutex_unlock(&img_mutex); 
      assert(mutex_unlocked == 0);

      /* Process images here */

      glfwImageShow("RGB video", rgb_img);
           
    }

    int mutex_unlocked = pthread_mutex_unlock(&img_mutex); 
    assert(mutex_unlocked == 0);
    
    std::cout << "Image processing is about to die " << std::endl;
    std::cout.flush();    
        
 return NULL;

}


// HHAA
void *image_capture_threadfunc(void *arg){

    ros::NodeHandle *n = (ros::NodeHandle *)arg;
    ros::Rate loop_rate(10);

    image_transport::ImageTransport it(*n);
    image_transport::Subscriber sub = it.subscribe("/camera/rgb/raw", 1, imageCallback);

    // This MUST be here
    mssleep(10); 

    while(ros::ok() && !die) {

      int mutex_locked = pthread_mutex_lock(&img_mutex); 
      assert(mutex_locked == 0);

         ros::spinOnce();

      int mutex_cond = pthread_cond_signal(&img_cond); 
      assert(mutex_cond == 0);
      int mutex_unlocked = pthread_mutex_unlock(&img_mutex); 
      assert(mutex_unlocked == 0);  

      loop_rate.sleep();
 
   }

   int mutex_cond = pthread_cond_signal(&img_cond); 
   assert(mutex_cond == 0);
   int mutex_unlocked = pthread_mutex_unlock(&img_mutex); 
   assert(mutex_unlocked == 0);  

   std::cout << "Image capture is about to die " << std::endl;
   std::cout.flush();        

 return NULL;

}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {

      //cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

     //cv_bridge::toCvShare(msg, "bgr8")->image.copyTo(orig_image);
     orig_image = cv_bridge::toCvShare(msg, "bgr8")->image.clone(); 
     //glfwImageShow("RGB video", orig_image);


  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}
