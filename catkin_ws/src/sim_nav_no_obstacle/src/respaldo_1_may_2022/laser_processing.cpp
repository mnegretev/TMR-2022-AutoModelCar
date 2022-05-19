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
#include <math.h>

#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <opencv2/opencv.hpp>

#include "utils.hpp"
#include "glwindow.hpp"
#include "laser_processing.hpp"


// Global
// HHAA: This is the function pointer type for window position callbacks. A window position callback function has the following signature: void callback_name(GLFWwindow* window, int xpos, int ypos)
extern volatile bool die;

extern volatile unsigned int width;
extern volatile unsigned int height;


pthread_mutex_t laser_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t laser_cond = PTHREAD_COND_INITIALIZER;

// HHAA
void *laser_processing_threadfunc(void *){

  cv::Mat laser_img(cv::Size(width, height), CV_8UC3);
  
//    glfwNamedWindow("Laser readings"); 
    

    while (!die) {
      int mutex_locked = pthread_mutex_lock(&laser_mutex); 
      assert(mutex_locked == 0);      
        int cond_wait = pthread_cond_wait(&laser_cond, &laser_mutex);
        assert(cond_wait == 0);

        /* process data here */
        
        //glfwImageShow("Laser readings", laser_img);
                
      int mutex_unlocked = pthread_mutex_unlock(&laser_mutex); 
      assert(mutex_unlocked == 0);
            
    }
       
    std::cout << "Laser processing is about to die " << std::endl;
    std::cout.flush();    
        
 return NULL;

}


// HHAA
void *laser_capture_threadfunc(void *){

    mssleep(20);

    while (!die) { 
      int mutex_locked = pthread_mutex_lock(&laser_mutex); 
      assert(mutex_locked == 0);
      

      int mutex_cond = pthread_cond_signal(&laser_cond); 
      assert(mutex_cond == 0);
            
      int mutex_unlocked = pthread_mutex_unlock(&laser_mutex); 
      assert(mutex_unlocked == 0);
                        
      mssleep(1);
                                      
    }


    int mutex_cond = pthread_cond_signal(&laser_cond); 
    assert(mutex_cond == 0);
    int mutex_unlocked = pthread_mutex_unlock(&laser_mutex); 
    assert(mutex_unlocked == 0);

    
    std::cout << "Laser capture is about to die " << std::endl;
    std::cout.flush();        

 return NULL;

}


