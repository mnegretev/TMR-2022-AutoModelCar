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

double obs_theta = 0;
double obs_rho = 0;
double best_theta = 0.896055;
double best_rho =  330.932;

// Shared OpenCV images
cv::Mat orig_image(cv::Size(width, height), CV_8UC3);

pthread_mutex_t img_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t img_cond = PTHREAD_COND_INITIALIZER;

// Revisar: https://www.youtube.com/watch?v=7M99dovhx8M
void *image_processing_threadfunc(void *){

    // Define the images that will be used first
    cv::Mat orig, image_roi;
    cv::Mat processed;
    cv::Mat transformed(480, 640, CV_8UC3); //Destination for warped image                  
    cv::Mat windowed(480, 640, CV_8UC3);    
    double best_line[5] = {351.347, 0.873172, 1, 0, 0}; // rho, theta, A,B,C
    bool first_time = true;

    std::vector<cv::Rect> windows;
    
    int y_rect = 384;
    int h_rect = 96; 
    for (int i = 0; i < 5; i++){
       windows.push_back(cv::Rect(440, y_rect, 200, h_rect));                   
       y_rect = y_rect - h_rect;
        
    }
                   
    glfwNamedWindow("image_roi");     
    glfwNamedWindow("image_grey");     
    glfwNamedWindow("Windowed");    

    mssleep(2000);
    int contador = 0;
/*    
    double gamma_ = 2;
    cv::Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.ptr();
    for( int i = 0; i < 256; ++i)
        p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma_) * 255.0);
*/           

    while (!die) {
      int mutex_locked = pthread_mutex_lock(&img_mutex); 
      assert(mutex_locked == 0);
                          
        int cond_wait = pthread_cond_wait(&img_cond, &img_mutex); 
        assert(cond_wait == 0);
        if (die) break;
            
           image_roi = orig_image.clone(); 


                    
  //      cv::Mat orig_prev_gamma = orig_image.clone();
  //        orig = orig_prev_gamma.clone();
  //      cv::LUT(orig_prev_gamma, lookUpTable, orig);        

      //  cv::Mat image_roi = orig_image.clone();   

      int mutex_unlocked = pthread_mutex_unlock(&img_mutex); 
      assert(mutex_unlocked == 0);

      // Extract ROI myROI(x, y, width, height)
      // HHAA: ROI dimensions MUST be tuned.
      //cv::Rect myROI(100, 239, 540, 240); // pt1(x,y), pt2(x + width, y + height)      
         
 //    if (contador == 3){ 

     // Extract ROI myROI(x, y, width, height)
     // HHAA: ROI dimensions MUST be tuned.
     //    cv::Rect myROI(100, 239, 540, 240); // pt1(x,y), pt2(x + width, y + height)      
      //Convert to gray 
      cv::Mat image_grey;
      cv::cvtColor(image_roi, image_grey, cv::COLOR_RGB2GRAY);
   
      // Extract white info 
      cv::Mat maskWhite;
      // HHAA: White threshold MUST be tuned.
      cv::inRange(image_grey, cv::Scalar(150, 150, 150), cv::Scalar(255, 255, 255), maskWhite);            
      cv::bitwise_and(image_grey, maskWhite, processed); // Extract what matches   
                     
      // Blur the image a bit so that gaps are smoother 
      // HHAA: Kernel size could be fine-tuned.
      const cv::Size kernelSize = cv::Size(9, 9);
      cv::GaussianBlur(processed, processed, kernelSize, 0);      
   
      //Try to fill the gaps 
      // HHAA: Is this block necessary?
      cv::Mat kernel = cv::Mat::ones(15, 15, CV_8U);
      cv::dilate(processed, processed, kernel);
      cv::erode(processed, processed, kernel);
      cv::morphologyEx(processed, processed, cv::MORPH_CLOSE, kernel);
   
      // Keep only what's above 100 value, other is then black 
      // HHAA: White threshold MUST be tuned.     
      const int thresholdVal = 100;
      cv::threshold(processed, processed, thresholdVal, 255, cv::THRESH_BINARY);
  //    cv::adaptiveThreshold(processed, processed, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 51, 0);

   
      // Perspective transform
      // Define points that are used for generating bird's eye view. This was done by trial and error. Best to prepare sliders and configure for each use case. 
      // Vertices MUST be identified by experimentation (use adjust).
      //x1 -40 x2 820 x3 860 x4 -70 y1 180 y2 280
      // x1 -60 x2 650 x3 840 x4 -120 y1 100 y2 280
 
/*     
      int x1 = -60;
      int x2 = 650;
      int x3 = 840;
      int x4 = -120;
      int y1 = 100;
      int y2 = 280;
*/
/*

      int x1 = 60;
      int x2 = 640;
      int x3 = 570;
      int x4 = 100;
      int y1 = 110;
      int y2 = 60;
*/

// x1 150 x2 650 x3 560 x4 190 y1 160 y2 80
// x1 -20 x2 730 x3 610 x4 150 y1 110 y2 0
// -10 x2 600 x3 520 x4 140 y1 80 y2 -10
// x1 100 x2 650 x3 750 x4 110 y1 150 y2 130
// x1 -520 x2 490 x3 340 x4 -340 y1 300 y2 320 


      int x1 = -520;
      int x2 = 490;
      int x3 = 340;
      int x4 = -340;
      int y1 = 300;
      int y2 = 320;        


      cv::Point2f srcVertices[4]; 
      srcVertices[0] = cv::Point(x1, y1);
      srcVertices[1] = cv::Point(x2, y1);
      srcVertices[2] = cv::Point(x3, y2);
      srcVertices[3] = cv::Point(x4, y2);       
   
      // Destination vertices. Output is 640 by 480px       
      cv::Point2f dstVertices[4];
      dstVertices[0] = cv::Point(0, 0);
      dstVertices[1] = cv::Point(640, 0);
      dstVertices[2] = cv::Point(640, 480);
      dstVertices[3] = cv::Point(0, 480);
      
      // Prepare matrix for transform and get the warped image 
      cv::Mat perspectiveMatrix = getPerspectiveTransform(srcVertices, dstVertices);
      //For transforming back into original image space 
      cv::Mat invertedPerspectiveMatrix;
      cv::invert(perspectiveMatrix, invertedPerspectiveMatrix);
      
      cv::warpPerspective(processed, transformed, perspectiveMatrix, transformed.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);      
              
      
      transformed.copyTo(windowed);              
   
      // HHAA: the position and dimensions of the slicing windows MUST be tuned.     
    
   //Sliding window for the right side 

      std::vector<cv::Point2f> pts = slidingWindow(transformed, windowed, windows);

      if (pts.size() > 0){
      
         std::vector<cv::Point2f> outPts;
         perspectiveTransform(pts, outPts, invertedPerspectiveMatrix);
   
         //Draw the right side
         for (int i = 0; i < outPts.size() - 1; ++i)
         {
           //cv::line(image_roi, outPts[i], outPts[i + 1], cv::Scalar(255, 0, 0), 8);
         }
      
         cv::Point p1 = outPts[0];
         cv::Point p2 = outPts[outPts.size() - 1];  
          
         cv::line(image_roi, p1, p2, cv::Scalar(255, 0, 100), 3);

         double m = (p2.y - p1.y)/ (double) (p2.x - p1.x);  
         obs_theta =  atan(m);

         if (obs_theta < 0){

            obs_theta += M_PI*2; 
         }
                 
         std::cout.flush(); 
         cv::Point mid;

         if (first_time){

            obs_theta = best_theta; 
            obs_rho = best_rho; 
            
            first_time = false;
         }  else {


         mid.x = round((p2.x - p1.x) / 2.0) + p1.x;   
         mid.y = round((p2.y - p1.y) / 2.0) + p1.y;   

<<<<<<< HEAD
         obs_rho = sqrt((pow(320 - mid.x, 2) + pow(141 - mid.y, 2)));
=======
         obs_rho = sqrt((pow(320 - mid.x, 2) + pow(480 - mid.y, 2)));
>>>>>>> 1c17498e2aac724188221c2cab7997d9f73b19be


         } 

         cv::line(image_roi, cv::Point(mid.x, mid.y), cv::Point(320, 479), cv::Scalar(255, 0, 255), 3, cv::LINE_AA);  
         

      } //  (pts.size() > 0)


    //  contador ++;
      glfwImageShow("Windowed", windowed);    
      glfwImageShow("image_grey", image_grey);    
      glfwImageShow("image_roi", image_roi);    
           
    }

    int mutex_unlocked = pthread_mutex_unlock(&img_mutex); 
    assert(mutex_unlocked == 0);
    
    std::cout << "Image processing is about to die " << std::endl;
    std::cout.flush();    

        
 return NULL;

}


/*
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

      // Process images here 

      glfwImageShow("RGB video", rgb_img);
           
    }

    int mutex_unlocked = pthread_mutex_unlock(&img_mutex); 
    assert(mutex_unlocked == 0);
    
    std::cout << "Image processing is about to die " << std::endl;
    std::cout.flush();    
        
 return NULL;

}
x1 -20 x2 730 x3 610 x4 150 y1 110 y2 0
*/

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

std::vector<cv::Point2f> slidingWindow(cv::Mat image, cv::Mat  windowed,     std::vector<cv::Rect> &windows)
{
    std::vector<cv::Point2f> points;
    const cv::Size imgSize = image.size();

<<<<<<< HEAD
    for (int i = 0; i < 4; i++)
=======
    for (int i = 0; i < 5; i++)
>>>>>>> 1c17498e2aac724188221c2cab7997d9f73b19be
    {
    
        float currentX = windows.at(i).x + windows.at(i).width * 0.5f;

        cv::rectangle(windowed, cv::Rect(windows.at(i).x, windows.at(i).y, windows.at(i).width, windows.at(i).height), cv::Scalar(255, 255, 0), 1, 8, 0); 
       
//        std::cout << windows[i].x << " " << windows[i].y << " " << windows[i].width << " " <<  windows[i].height << std::endl; 
//        std::cout.flush();                        
        cv::Mat roi = image(windows.at(i)); //Extract region of interest         
        cv::Mat locations;   // output, locations of non-zero pixels 

        cv::findNonZero(roi, locations); //Get all non-black pixels. All are white in our case         

        float avgX = 0.0f;
        
       // std::cout << "i: " << i << " locations.elemSize() " <<  locations.elemSize() << std::endl;
        if (locations.elemSize() > 0){
        
            for (int j = 0; j < locations.elemSize(); ++j) //Calculate average X position         
            {
                float x = locations.at<cv::Point>(j).x;
                avgX += windows.at(i).x + x; // x-coordinate relative to the original image
            }

            avgX = locations.empty() ? currentX : avgX / locations.elemSize();

            //std::cout << " avgX " << avgX << " elemSize " <<  locations.elemSize() <<  std::endl;
            //std::cout.flush();               
        
            cv::Point point(avgX, windows.at(i).y + windows.at(i).height * 0.5f);
            points.push_back(point);
        
            //Move x position
            windows.at(i).x = (int)round(avgX - windows.at(i).width *.5);
            
            if (i < windows.size() - 1){
               if (windows.at(i).x + windows.at(i).width < windows.at(i + 1).x ||  windows.at(i).x > windows.at(i + 1).x + round(windows.at(i + 1).width)){
                   //pts.erase(pts.begin() + i);
                   windows.at(i).x = windows.at(i + 1).x;
                   points.pop_back();
                   cv::Point new_point(
                         windows.at(i + 1).x + round(windows.at(i + 1).width * 0.5f),
                         windows.at(i).y + windows.at(i).height * 0.5f);
                   points.push_back(new_point);     
                   
               }                 
            
            } // if (i < windows.size() - 1)


            
            if (i == windows.size() - 1){
               if (windows.at(i).x + windows.at(i).width < windows.at(i - 1).x ||  windows.at(i).x > windows.at(i - 1).x + round(windows.at(i - 1).width)){

            
                  float avgX = 0.0f;
                  for (int j = 0; j < windows.size() - 1; ++j){
                    avgX += windows.at(i).x; 
                  }
                  avgX = avgX / (windows.size() - 1);

                  windows.at(i).x = avgX; //windows.at(i - 1).x;
                  points.pop_back();
                  cv::Point new_point(windows.at(i).x, windows.at(i).y + windows.at(i).height * 0.5f);
                  points.push_back(new_point);
               }  
                
            }
            
            
            //std::cout << " windows.at(i).x " << windows.at(i).x << " point.x " <<  point.x << " currentX " << currentX <<  std::endl;
        //std::cout << " windows.at(i).width " << windows.at(i).width <<  std::endl;        
        //std::cout.flush();               
                
        //Make sure the window doesn't overflow, we get an error if we try to get data outside the matrix         
        
            if (windows.at(i).x < 0)
               windows.at(i).x = 0;
            if (windows.at(i).x + windows.at(i).width >= imgSize.width)
               windows.at(i).x = imgSize.width - windows.at(i).width - 1;
        
        } // if locations.elemSize() > 0
        
        //std::cout << " Overflow windows.at(i).x " << windows.at(i).x <<  std::endl;
        //std::cout.flush();           

                
    }
    
    return points;
}

