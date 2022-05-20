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


class ImageSegmentation {

  private:
  
      struct BLOB{
	int xsup;
	int ysup;
	int xinf;
	int yinf;
	struct BLOB *next;
      };
      typedef struct BLOB BLOB; 

      struct LINE{
	char marked;
	long int index;
      };
      typedef struct LINE LINE;   
   
      unsigned int xSkip = 1, ySkip = 1;      
      unsigned int xMin = 0, yMin = 0;            
      std::vector <BLOB> L; // Blob 
      std::vector <LINE> scan_line; // Pixel scan line
      std::vector <cv::Rect> rect_list;
      BLOB *ROI = NULL;
      unsigned int num_blobs = 0; 
      
      
      //--------------------------------------------      
      long int insert_blob(int x, int y);
      //--------------------------------------------
      void line_segmentation(const cv::Mat &frame, int x, 
                 int y, int x2, 
                 bool (&ps)(int x, int y, const cv::Mat &frame));
      
  public: 
  
      // Constructor
      //ImageSegmentation(unsigned int xskip, unsigned int yskip); 

      //--------------------------------------------
      void set_skip(unsigned int xskip, unsigned int yskip);     
      
      //--------------------------------------------
      void set_ROI(int xsup, int ysup, int xinf, int yinf);     
      
      //--------------------------------------------
      void release_ROI(void);          

      //--------------------------------------------
      void set_min_blob_size(unsigned int xmin, unsigned int ymin);

      //--------------------------------------------
      void draw_blobs(cv::Mat &frame, unsigned int thickness);

      //--------------------------------------------
      void delete_small_blobs(void);
      
      //--------------------------------------------
      void free_blob_memory(void);      
            
      //--------------------------------------------      
      cv::Rect get_biggest_blob(void);
      
      //--------------------------------------------            
      std::vector <cv::Rect> get_blobs(void);

      //--------------------------------------------            
      long int get_num_blobs(void);
            
      //--------------------------------------------
      void search_blobs(const cv::Mat &frame, bool (&ps)(int x, int y, const cv::Mat &frame));
      
      
      // Destructor.
      ~ImageSegmentation(){
          free_blob_memory(); 
          release_ROI();	       
      }                

     
}; // End class segmentation



    
