/*   This program is distributed in the hope that it will be useful,
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

#include "utils.hpp"

//
int mssleep(long milliseconds){

  struct timespec rem;
  struct timespec req = {
        (int)(milliseconds / 1000), /*secs (Must be Non-Negative)*/
        (milliseconds % 1000) * 1000000 /*nano (Must be in range of 0 to 999999999)*/
  };

  return nanosleep(&req, &rem); 

}

/*
static inline void delay(sl_word_size_t ms){
  while (ms >= 1000){
      usleep(1000*1000);
      ms-=1000;
  };
  if (ms!=0)
        usleep(ms*1000);
}
*/

// Get the color of a pixel (x,y) of the original image
void get_pixel_color(const cv::Mat &frame, int x, int y, unsigned char *color){

   int offset = 0;

   if (x < 0) { x = 0; }
   if (x >= frame.cols) { x = frame.cols - 1; }
   if (y < 0) { y = 0; }
   if (y >= frame.rows) { y = frame.rows - 1; }

   // Get data buffer 
   uchar* ptr = (uchar *)frame.data;

   // Clean memory
   memset(color, 0, 3);
   offset += y * frame.step;
   // Offset
   offset +=  frame.channels() * x;

   // Assume BGR pixel order
   color[0] = (unsigned char)ptr[offset++];
   color[1] = (unsigned char)ptr[offset++];
   color[2] = (unsigned char)ptr[offset];

}


// Set the color of a pixel (x,y) of the original image
void set_pixel_color(const cv::Mat &frame, int x, int y, unsigned char *color){

   int offset = 0;

   if (x < 0) { x = 0; }
   if (x >= frame.cols) { x = frame.cols - 1; }
   if (y < 0) { y = 0; }
   if (y >= frame.rows) { y = frame.rows - 1; }
   
      // Get data buffer 
   uchar* ptr = (uchar *)frame.data;

   offset += y * frame.step;
  
   // Offset
   offset +=  frame.channels() * x;

   // Assume BGR pixel order
   ptr[offset++] = (unsigned char)color[0];
   ptr[offset++] = (unsigned char)color[1];
   ptr[offset] = (unsigned char)color[2];

}


// Get the color of a pixel (x,y) of an image
unsigned char get_pixel_grey(const cv::Mat &frame, int x, int y){

   int offset = 0;

   if (x < 0) { x = 0; }
   if (x >= frame.cols) { x = frame.cols - 1; }
   if (y < 0) { y = 0; }
   if (y >= frame.rows) { y = frame.rows - 1; }

   offset += y * frame.step;
  
   // Offset
   offset +=  frame.channels() * x;

   return (frame.data[offset]);

}

// Get the color of a pixel (x,y) of an image
unsigned char set_pixel_grey(const cv::Mat &frame, int x, int y, unsigned char intensity){

   int offset = 0;

   if (x < 0) { return 0; }
   if (x >= frame.cols) { return 0;  }
   if (y < 0) { return 0;  }
   if (y >= frame.rows) { return 0;  }

   offset += y * frame.step;
  
   // Offset
   offset +=  frame.channels() * x;

   // Assume BGR pixel order
   frame.data[offset] = intensity;

 return 1;

}


// HHAA
bool scan_pix_color(int x, int y, const cv::Mat &frame){

  unsigned char color[3];
  
    get_pixel_color(frame, x, y, color);
    if (color[0] == 0 && color[1] == 0 && color[2] == 255){
       return true; 
    }

  return false;

}


// HHAA
bool scan_pix_grey(int x, int y, const cv::Mat &frame){

  unsigned char intensity;
  
    intensity = get_pixel_grey(frame, x, y);
    if (intensity != 0){
       return true; 
    }

  return false;

}

//------------------------------------------------------------------------------------
void draw_blobs(std::vector <cv::Rect> &blobs, cv::Mat &frame){
  
   for (int i = 0; i < blobs.size(); i++){
      cv::rectangle(frame, 
                    blobs[i], 
                    cv::Scalar(255, 255, 255), 
                    1, 8, 0);     
   }  
   
}


