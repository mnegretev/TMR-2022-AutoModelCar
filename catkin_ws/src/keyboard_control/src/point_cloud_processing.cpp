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
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "image_processing.hpp"
#include "utils.hpp"
#include "segmentation.hpp"
#include "glwindow.hpp"
#include "point_cloud_processing.hpp"

// Global
// HHAA: This is the function pointer type for window position callbacks. A window position callback function has the following signature: void callback_name(GLFWwindow* window, int xpos, int ypos)
extern volatile unsigned int width;
extern volatile unsigned int height;
extern volatile bool die;

const int MAX_DIST = 30000;

// Shared OpenCV images
cv::Mat rear_grey(cv::Size(width, height), CV_8UC1);
cv::Mat frontal_grey(cv::Size(width, height), CV_8UC1);

pthread_mutex_t pc_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t pc_cond = PTHREAD_COND_INITIALIZER;

unsigned int action = 0;

// HHAA
void *point_cloud_capture_threadfunc(void *arg){

    cv::Mat roi_rear_grey;
    cv::Mat roi_frontal_grey;

    ros::NodeHandle *n = (ros::NodeHandle *)arg;
    ros::Rate loop_rate(10);

    ros::Subscriber pc = n->subscribe ("/point_cloud", 1, point_cloudCallback);

    cv::Rect ROI_rear(0, 239, 640, 120); 
    cv::Rect ROI_frontal(0, 239, 640, 200); 

    //glfwNamedWindow("Colorized Point Cloud"); 
    glfwNamedWindow("rear_grey");                   
    glfwNamedWindow("frontal_grey");  

    // This MUST be here
    mssleep(10); 

    ImageSegmentation frontal;
    ImageSegmentation rear;

    frontal.set_min_blob_size(10, 10);
    rear.set_min_blob_size(10, 10);

    action = 0;

    while(ros::ok() && !die) {

      int mutex_locked = pthread_mutex_lock(&pc_mutex); 
      assert(mutex_locked == 0);

         ros::spinOnce();

         cv::Mat roi_rear_grey = rear_grey(ROI_rear); 
         cv::Mat roi_frontal_grey = frontal_grey(ROI_frontal); 

         // max dilation_size = 21
         int dilation_size = 6;
         cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                       cv::Size(2* dilation_size-1 , 2*dilation_size-1 ),
                       cv::Point( dilation_size, dilation_size ) );

         cv::Mat temp_rear(cv::Size(239, 120), CV_8UC1);
         cv::dilate( roi_rear_grey, temp_rear, element );
         temp_rear.copyTo(roi_rear_grey);

         cv::Mat temp_frontal(cv::Size(239, 200), CV_8UC1);
         cv::dilate( roi_frontal_grey, temp_frontal, element); 
         temp_frontal.copyTo(roi_frontal_grey);

         frontal.search_blobs(roi_frontal_grey, scan_pix_grey); 
         frontal.delete_small_blobs();
         std::vector <cv::Rect> frontal_blobs = frontal.get_blobs();
         frontal.draw_blobs(roi_frontal_grey, 1);
         int frontal_num_blobs = frontal.get_num_blobs();  
         std::cout << "Frontal blobs: " << frontal_num_blobs << std::endl;
         std::cout.flush();  
         frontal.free_blob_memory(); 

         rear.search_blobs(roi_rear_grey, scan_pix_grey); 
         rear.delete_small_blobs();   
         std::vector <cv::Rect> rear_blobs = rear.get_blobs(); 
         rear.draw_blobs(roi_rear_grey, 1);
         int rear_num_blobs = rear.get_num_blobs(); 
         std::cout << "Rear blobs: " << rear_num_blobs << std::endl;
         std::cout.flush();  
         rear.free_blob_memory(); 

         
       //  if ((rear_num_blobs == 1 && frontal_num_blobs == 0) || rear_blobs.at(0).x + rear_blobs.at(0).width *0.5 > 130){
         if (rear_blobs.size() > 0 && rear_blobs.at(0).x + rear_blobs.at(0).width *0.5 > 170){  
            action = 1; // stop the car
            std::cout << "Action: " << action << std::endl;
            std::cout.flush();  
         }   

      int mutex_cond = pthread_cond_signal(&pc_cond); 
      assert(mutex_cond == 0);
      int mutex_unlocked = pthread_mutex_unlock(&pc_mutex); 
      assert(mutex_unlocked == 0);  

      glfwImageShow("rear_grey", roi_rear_grey);
      glfwImageShow("frontal_grey", roi_frontal_grey);

     // roi_rear_grey.setTo(0);
     // roi_frontal_grey.setTo(0);

      loop_rate.sleep();
 
   }

   int mutex_cond = pthread_cond_signal(&pc_cond); 
   assert(mutex_cond == 0);
   int mutex_unlocked = pthread_mutex_unlock(&pc_mutex); 
   assert(mutex_unlocked == 0);  

   std::cout << "Point Cloud capture is about to die " << std::endl;
   std::cout.flush();        

 return NULL;

}

//http://wiki.ros.org/pcl/Tutorials#pcl.2FTutorials.2Fhydro.CA-3571590cfeccbc210e304977f9d1f64a4f348d6e_16
// https://stackoverflow.com/questions/46717428/pclpointcloud-to-cvmat-depth-image
void point_cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{

  int offset;
  float x, y, z;
  int i;

  double u, v;
  double f_x = 5.1546587738782500e+02; // px 
  double f_y = 5.1612859980415396e+02; // px
  double c_x = 3.3519858748295297e+02;
  double c_y = 2.2531724670860700e+02;    

/*

  std::cout << "Height " << input->height << std::endl;
  std::cout << "width " << input->width << std::endl;
  std::cout << "is_bigendian " << input->is_bigendian << std::endl;
  std::cout << "point_step " << input->point_step << std::endl;
  fprintf(stdout, "point_step %d\n", input->point_step); 
  std::cout << "is_dense " << input->is_dense << std::endl;
  std::cout << "row_step " << input->row_step << std::endl;
  fprintf(stdout, "row_step %d\n", input->row_step); 

  std::cout << "0 name " << input->fields[0].name << std::endl;
  std::cout << "offset " << input->fields[0].offset << std::endl;
  std::cout << "datatypes " << input->fields[0].datatype << std::endl;
  std::cout << "counts " << input->fields[0 ].count << std::endl;

  std::cout << "1 name " << input->fields[1].name << std::endl;
  std::cout << "offset " << input->fields[1 ].offset << std::endl;
  std::cout << "datatypes " << input->fields[1 ].datatype << std::endl;
  std::cout << "counts " << input->fields[1 ].count << std::endl;

  std::cout << "2 name " << input->fields[2].name << std::endl;
  std::cout << "offset " << input->fields[2 ].offset << std::endl;
  std::cout << "datatypes " << input->fields[2 ].datatype << std::endl;
  std::cout << "counts " << input->fields[2 ].count << std::endl;

*/

  rear_grey.setTo( cv::Scalar( 0x00 ) );
  frontal_grey.setTo( cv::Scalar( 0x00 ) );

  offset = 0; 
  for (i = 0; i < input->width; i++){

     memcpy(&x, &input->data[offset + input->fields[0].offset], sizeof x);
     memcpy(&y, &input->data[offset + input->fields[1].offset], sizeof y);
     memcpy(&z, &input->data[offset + input->fields[2].offset], sizeof z);

     x *= 1000; //mm
     y *= 1000; //mm
     z *= 1000; //mm

     /* Rear view */
     if ( !(y < -1650) && !(x > 4800) ){ // Altura del coche al centro 1.45 y traslación del lidar .32 es igual a una distancia de 1.77m desde el plano ZX del lídar al suelo 
           if (!isinf(x) && !isinf(y) && !isinf(z) && (z > 0 && z < MAX_DIST)){
      
               u = ((f_x*x)/ z) + c_x;
               v = ((f_y*y)/ z) + c_y;
               // Rotation and translation
               u = -u + width;
               v = -v + height;   
      
               if (( u >= 0 && u < width) //z in mm
                                   && (v >= 0 && v < height)){    
      
                     // Scale (z / (8000/256))                 
                     unsigned char grey = 256 -
                        (unsigned char)round((fabs(z) / (double)MAX_DIST) * 256);
                                   
                     set_pixel_grey(rear_grey, u, v, grey);
               } 
      
           } // z
      
      
           if (!isinf(x) && !isinf(y) && !isinf(z) && (z < 0 && z > -MAX_DIST)){
      
               u = ((f_x*x)/ z) + c_x;
               v = ((f_y*y)/ z) + c_y;
               // Rotation and translation
               u = -u + width;
      
               if (( u >= 0 && u < width) //z in mm
                                   && (v >= 0 && v < height)){    
      
                     // Scale (z / (8000/256))                 
                     unsigned char grey = 256 -
                        (unsigned char)round((fabs(z) / (double)MAX_DIST) * 256);
                                   
                     set_pixel_grey(frontal_grey, u, v, grey);
               } 
      
           } 

     } //if ( !(y > && y <) ){

     offset += input->point_step;      
  
  } // for

/*
     // max dilation_size = 21
     int dilation_size = 3;
     cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                       cv::Size(2* dilation_size-1 , 2*dilation_size-1 ),
                       cv::Point( dilation_size, dilation_size ) );
     cv::Mat img_temp(cv::Size(width, height), CV_8UC1);
     cv::dilate( rear_grey, img_temp, element );
     img_temp.copyTo(rear_grey);
     cv::dilate( frontal_grey, img_temp, element); //cv::Mat(), cv::Point(-1, -1), 2, 1, 1 );
     img_temp.copyTo(frontal_grey);
 
*/
}
