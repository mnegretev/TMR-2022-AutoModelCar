/*
 *   -
 *   Video processing using OpenCV4 and OpenGL
 *   Copyright (C) <2022>  
 *     Hector Hugo Aviles-Arriaga
 *    
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 *   
 */

#include <ctype.h>
#include <stdio.h>
#include <pthread.h>
#include <getopt.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>
#include <sys/times.h>

#include <opencv2/opencv.hpp>

#include "segmentation.hpp"

//--------------------------------------------
void ImageSegmentation::set_ROI(int xsup, int ysup, int xinf, int yinf){

   if (ROI == NULL){
     ROI = new BLOB;
   }  
   
   ROI->xsup = xsup;
   ROI->ysup = ysup;
   ROI->xinf = xinf;
   ROI->yinf = yinf;     
  
}
      
//--------------------------------------------
void ImageSegmentation::release_ROI(void){

     delete ROI;
     ROI = NULL;

}

//------------------------------------------------------------------------------------
void ImageSegmentation::set_skip(unsigned int xskip, unsigned int yskip){

     if (xskip <= 0 || xskip > 5) { 
        std::cout << "xSkip must be set between 1 and 5 pixels. Current value is " << xSkip  << std::endl;
        xSkip = 1;
     } else {
       xSkip = xskip; 
     }
     
     if (yskip <= 0 || yskip > 5) { 
        std::cout << "ySkip must be set between 1 and 5 pixels. Current value is " << ySkip << std::endl;
        ySkip = 1;
     } else {
       ySkip = yskip; 
     } 

}      


//------------------------------------------------------------------------------------
long int ImageSegmentation::insert_blob(int x, int y){
 BLOB blob;

   blob.xsup = x;
   blob.ysup = y;
   blob.xinf = x;
   blob.yinf = y;

   L.push_back(blob);
    
 return L.size() - 1;   

}

//------------------------------------------------------------------------------------
void ImageSegmentation::free_blob_memory(void){

   L.clear();   
   if (L.size() > 0){ 
      L.resize(0);
      L.shrink_to_fit();
   }
   scan_line.clear();
   if (scan_line.size() > 0){
      scan_line.resize(0);
      scan_line.shrink_to_fit();
   }    
   rect_list.clear();
   if (rect_list.size() > 0){
      rect_list.resize(0);
      rect_list.shrink_to_fit();       
   }
}

//------------------------------------------------------------------------------------
void ImageSegmentation::set_min_blob_size(unsigned int xmin, unsigned int ymin){

    if (xmin < 0) { 
        std::cout << "xmin must be greater than or equal to 0. Current value is " << xMin << std::endl;
        xMin = 0;
     } else {
       xMin = xmin; 
     }

    if (ymin < 0) { 
        std::cout << "ymin must be greater than or equal to 0. Current value is " << yMin << std::endl;
        yMin = 0;
     } else {
       yMin = ymin; 
     }
}

//------------------------------------------------------------------------------------
void ImageSegmentation::delete_small_blobs(void){  
  unsigned long int width = 0;
  unsigned long int height = 0;

    for (int i = 0; i < L.size(); i++){
       width = L[i].xinf - L[i].xsup;
       height = L[i].yinf - L[i].ysup;       
       if (width <= xMin || height <= yMin){ 
          L[i].xsup = -1;
          L[i].ysup = -1;
          L[i].xinf = -1; 
          L[i].yinf = -1; 

       }
    }  
/*    
    for (int i = 0; i < L.size(); i++){
       if (L[i].xinf == -1){ 
          L.erase(L.begin() + i); 
       }
    }     
*/
}


//------------------------------------------------------------------------------------
long int ImageSegmentation::get_num_blobs(void){
    
   return (long int)L.size(); 
   
}



//------------------------------------------------------------------------------------
void ImageSegmentation::draw_blobs(cv::Mat &frame, unsigned int thickness){
 cv::Rect rect;
  
   if (thickness > 10 || thickness <= 0)
      thickness = 1;
  
   //if (L.size() == 0) 
       // std::cout << "No blobs" << std::endl;
  
   for (int i = 0; i < L.size(); i++){
   
      if (L[i].xsup != -1){
         rect.x = L[i].xsup;
         rect.y = L[i].ysup;
         rect.width = L[i].xinf - L[i].xsup + 1;
         rect.height = L[i].yinf - L[i].ysup + 1;
         cv::rectangle(frame, 
                    rect, 
                    cv::Scalar(0, 255, 0), 
                    thickness, 8, 0);
      }                   
   }  
   
}


//------------------------------------------------------------------------------------
cv::Rect ImageSegmentation::get_biggest_blob(void){
  unsigned long int area = 0; 
  unsigned long int idx; 

    for (int i = 0; i < L.size(); i++){
       unsigned long int a1 = (unsigned long int)(L[i].xinf - L[i].xsup) 
                                               * (L[i].yinf - L[i].ysup); 
       if (a1 > area){
         idx = i;
         area = a1;
       }

    }
    
    cv::Rect biggest;
    biggest.x = L[idx].xsup;
    biggest.y = L[idx].ysup;
    biggest.width = L[idx].xinf - L[idx].xsup + 1;
    biggest.height = L[idx].yinf - L[idx].ysup + 1;
    
  return biggest; 
   
}

//------------------------------------------------------------------------------------
std::vector <cv::Rect> ImageSegmentation::get_blobs(void){

 rect_list.clear();
 rect_list.resize(0);
 rect_list.shrink_to_fit();  
 cv::Rect rect;
   
   for (int i = 0; i < L.size(); i++){
      if (L[i].xsup != -1){   
        rect.x = L[i].xsup;
        rect.y = L[i].ysup;
        rect.width = L[i].xinf - L[i].xsup + 1;
        rect.height = L[i].yinf - L[i].ysup + 1;     
      
        rect_list.push_back(rect);
      }  
   }  
 
  return rect_list; 
   
}


/*--------------------------------------------------------------------------------------*/
void ImageSegmentation::search_blobs(const cv::Mat &frame, 
               bool (&ps)(int x, int y, const cv::Mat &frame)){

   unsigned int xsup = 0; 
   unsigned int ysup = 0; 
   unsigned int xinf = frame.cols; 
   unsigned int yinf = frame.rows;

   if (L.size() > 0){ 
      L.clear();
      L.resize(0);
      L.shrink_to_fit();  
   }   
   
   scan_line.resize(frame.cols);
   LINE l = {0, -1}; 
   scan_line.assign(scan_line.size(), l);
   
   if (ROI != NULL){
      xsup = ROI->xsup;
      ysup = ROI->ysup;
      xinf = ROI->xinf;
      yinf = ROI->yinf;
   } 

   for (int i = ysup; i < yinf; i+=ySkip)
       line_segmentation(frame, xsup, i, xinf, ps);
     
}



/*--------------------------------------------------------------------------------------*/
void ImageSegmentation::line_segmentation(const cv::Mat &frame, int x, 
                     int y, int x2, 
                          bool (&ps)(int x, int y, const cv::Mat &frame)){
  int prev_pixel = 0;
  int idx = 0; 
    
  do{
          
      if (ps(x, y, frame)){
          if ((prev_pixel == 0) && (scan_line[idx].marked == 0)){
            // Create new BLOB 	
            scan_line[idx].marked = 1;
            scan_line[idx].index = insert_blob(x, y);
            prev_pixel = 1;
            
          } else
          if ((prev_pixel == 1) && (scan_line[idx].marked == 0)){
            // Add current pixel to BLOB pointed by the previous pixel 
            scan_line[idx].marked = 1;
            scan_line[idx].index = scan_line[idx-1].index;
            if (x > L[scan_line[idx].index].xinf)
                { L[scan_line[idx].index].xinf = x; } 
            if (y > L[scan_line[idx].index].yinf)
                { L[scan_line[idx].index].yinf = y; }
                
          } else
          if ((prev_pixel == 0) && (scan_line[idx].marked == 1)){	
            // Add current pixel to the BLOB pointed by previous scan scan_line pixel 
            if (L[scan_line[idx].index].yinf < y)	
                { L[scan_line[idx].index].yinf = y; }		
                prev_pixel = 1;
                
          } else		  
            if ((prev_pixel == 1) && (scan_line[idx].marked == 1)
               && (scan_line[idx-1].index != scan_line[idx].index)){
               // Merge Blobs if previous scan scan_line pixel and previous pixel are pointed to different Blobs 	
            if (L[scan_line[idx].index].xsup > L[scan_line[idx-1].index].xsup 
                             && L[scan_line[idx-1].index].xsup >= 0)
                { L[scan_line[idx].index].xsup =
                                      L[scan_line[idx-1].index].xsup; }
            if (L[scan_line[idx].index].ysup > L[scan_line[idx-1].index].ysup 
                             && L[scan_line[idx-1].index].ysup >= 0)
                { L[scan_line[idx].index].ysup =
                                      L[scan_line[idx-1].index].ysup; }
            if (L[scan_line[idx].index].xinf < L[scan_line[idx-1].index].xinf 
                             && L[scan_line[idx-1].index].xinf >= 0)
                { L[scan_line[idx].index].xinf =
                                      L[scan_line[idx-1].index].xinf; }
            if (L[scan_line[idx].index].yinf < L[scan_line[idx-1].index].yinf 
                             && L[scan_line[idx-1].index].yinf >= 0)
                { L[scan_line[idx].index].yinf =
                                      L[scan_line[idx-1].index].yinf; }

            L[scan_line[idx-1].index].ysup = -1; 
            L[scan_line[idx-1].index].xsup = -1;			
            L[scan_line[idx-1].index].xinf = -1;
            L[scan_line[idx-1].index].yinf = -1; 
     
            for (int i = 0; i < frame.cols; i++){
                if (scan_line[i].index == scan_line[idx-1].index){
                   scan_line[i].index = scan_line[idx].index;
                }	
            }	
            
          } else {
            /* Merge Blobs if previous scan scan_line pixel and previous pixel are pointed to the same Blobs 	*/
            if (L[scan_line[idx].index].xinf < x 
                            && L[scan_line[idx].index].xinf >= 0)
               { L[scan_line[idx].index].xinf = x; }		
            if (L[scan_line[idx].index].yinf < y 
                            && L[scan_line[idx].index].yinf >= 0)
               { L[scan_line[idx].index].yinf = y; }  
               		
          }	
      } else {
          scan_line[idx].marked = 0;
          prev_pixel = 0;
      }

      idx += 1;
      x += xSkip; 
 
  } while (x < x2);

}


