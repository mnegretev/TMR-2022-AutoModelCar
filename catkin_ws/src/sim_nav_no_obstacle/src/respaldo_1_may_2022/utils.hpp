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

// 
int mssleep(long milliseconds);

//
void leftRotatebyOne(int arr[], int n);

// 
void leftRotate(int arr[], int d, int n);

//static inline void delay(sl_word_size_t ms);
  
// Get the color of a pixel (x,y) of the original image
void get_pixel_color(const cv::Mat &frame, int x, 
                                 int y, unsigned char *color); 

// set the color of a pixel (x,y) of the original image
void set_pixel_color(const cv::Mat &frame, int x, 
                   int y, unsigned char *color);
        
// Get the color of a pixel (x,y) of an image
unsigned char get_pixel_grey(const cv::Mat &frame, int x, int y);      

// Get the color of a pixel (x,y) of an image
unsigned char set_pixel_grey(const cv::Mat &frame, int x, 
                              int y, unsigned char intensity);

