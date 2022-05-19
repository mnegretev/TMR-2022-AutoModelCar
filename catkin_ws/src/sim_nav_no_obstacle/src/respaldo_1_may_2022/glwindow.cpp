/*
# Released under MIT License

Copyright (c) 2017 insaneyilin.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

// https://discourse.glfw.org/t/how-to-create-multiple-window/1398/2

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

#include <iostream>
#include <string>
#include <vector>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <opencv2/opencv.hpp>

#include "glwindow.hpp"

extern volatile bool die;

using std::cout;
using std::endl;

volatile int window_width = 640;
volatile int window_height = 480;
volatile bool display_windows;

// List and names of the windows created by the user
std::vector<GLFWwindow*> windowList; // Pointers to the windows
std::vector<std::string> windowName; // Names of the windows

// HHAA
void glfwSetWidthHeight(unsigned int w, unsigned int h){

  window_width = w;
  window_height = h;

}

// HHAA
void glfwDestroyWindows(void){ 

    for (int i = 0; i < windowList.size(); i++){
        glfwDestroyWindow(windowList[i]);
    }

    windowList.clear();
    windowList.resize(0);
    windowList.shrink_to_fit();       
    windowName.clear();
    windowName.resize(0);
    windowName.shrink_to_fit();           

    glfwTerminate();   

}

// HHAA
GLFWwindow* getWindow(const std::string winName){

    for (int i = 0; i < windowName.size(); i++){
        if (windowName[i].compare(winName) == 0){
           return windowList[i];
        }
    }   
    
  return NULL; 

}

// HHAA
bool glfw_Init(bool ow){

   display_windows = ow;
   if (!display_windows){
      cout << "Display windows was set to false " << endl;
      return true;
   }   
  
   if (!glfwInit()) {
       exit(EXIT_FAILURE);
   }
   
   int major, minor, revision;
   glfwGetVersion(&major, &minor, &revision);
 
   printf("Running against GLFW %i.%i.%i\n", major, minor, revision);   

   glfwSetErrorCallback(error_callback);

   glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
   glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0); 
   
 return true;  

}


// HHAA
bool glfwNamedWindow(const std::string winName){

 static GLenum err;
 
   if (!display_windows){
      return true;
   }
   
   GLFWwindow* window = glfwCreateWindow(window_width, 
                          window_height, winName.c_str(), NULL, NULL); 
   if (window == NULL) {
        glfwTerminate();
        cout << "Error creating the window " << winName << glewGetErrorString(err) << endl;
        std::cout.flush();
        exit(EXIT_FAILURE);
   }
   
   windowList.push_back(window);
   windowName.push_back(winName); 

   glfwSetKeyCallback(windowList[windowList.size() - 1], key_callback);
   //glfwSetWindowSizeCallback(windowList[windowList.size() - 1], resize_callback);

   glfwMakeContextCurrent(windowList[windowList.size() - 1]);
   glfwSwapInterval(1);

   // First time
   if (windowList.size() == 1){

      //  Initialise glew (must occur AFTER window creation or glew will error)
      err = glewInit();
      if (GLEW_OK != err){
         cout << "GLEW initialisation error: " << glewGetErrorString(err) << endl;
         std::cout.flush();    
         exit(-1);
      }
      cout << "GLEW okay - using version: " << glewGetString(GLEW_VERSION) << endl;
            
   } 

       
  return true;   

}


// HHAA
bool glfwImageShow(const std::string winName, cv::Mat &frame) {

    if (!display_windows)
       return true;

    cv::Size frame_size = frame.size(); 
    
    GLFWwindow* window = getWindow(winName);
    if (window == NULL){    
        cout << "Error: No window with the name " << winName << endl;
        std::cout.flush(); 
        return false;
    }

    //if (frame.isSubmatrix() == true)
    //  glfwSetWindowSize(window, 640, 480);  
    //else

     
    glfwSetWindowSize(window, frame_size.width, frame_size.height);
    //glfwSetWindowSize(window, 640, 480);    
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    glViewport(0, 0, frame_size.width, frame_size.height); // use a screen size of WIDTH x HEIGHT

    glMatrixMode(GL_PROJECTION);     // Make a simple 2D projection on the entire window
    glLoadIdentity();
    glOrtho(0.0, frame_size.width, frame_size.height, 0.0, 0.0, 100.0);

    glMatrixMode(GL_MODELVIEW);    // Set the matrix mode to objectmodeling

    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClearDepth(0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear the window    

    // Clear color and depth buffers
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);     // Operate on model-view matrix

    glEnable(GL_TEXTURE_2D);
    GLuint image_tex = matToTexture(frame, GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR, GL_CLAMP);

    // Draw a quad GL_UNPACK_ROW_LENGTH
    glBegin(GL_QUADS);
    glTexCoord2i(0, 0); glVertex2i(0, 0);
    glTexCoord2i(0, 1); glVertex2i(0, frame_size.height);
    glTexCoord2i(1, 1); glVertex2i(frame_size.width, frame_size.height);
    glTexCoord2i(1, 0); glVertex2i(frame_size.width, 0);
    glEnd();

    glDeleteTextures(1, &image_tex);
    glDisable(GL_TEXTURE_2D);
    
    glfwSwapBuffers(window);
    glfwPollEvents();
    
  return true;  
    
}


// Function turn a cv::Mat into a texture, and return the texture ID as a GLuint for use
GLuint matToTexture(cv::Mat &mat, GLenum minFilter, GLenum magFilter, GLenum wrapFilter) {
    // Generate a number for our textureID's unique handle
    GLuint textureID;
    glGenTextures(1, &textureID);

    // Bind to our texture handle
    glBindTexture(GL_TEXTURE_2D, textureID);

    // Catch silly-mistake texture interpolation method for magnification
    if (magFilter == GL_LINEAR_MIPMAP_LINEAR  ||
            magFilter == GL_LINEAR_MIPMAP_NEAREST ||
            magFilter == GL_NEAREST_MIPMAP_LINEAR ||
            magFilter == GL_NEAREST_MIPMAP_NEAREST)
    {
        cout << "You can't use MIPMAPs for magnification - setting filter to GL_LINEAR" << endl;
        std::cout.flush();
        magFilter = GL_LINEAR;
    }

    // Set texture interpolation methods for minification and magnification
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, magFilter);

    // Set texture clamping method
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapFilter);

    // Set incoming texture format to:
    // GL_BGR       for CV_CAP_OPENNI_BGR_IMAGE,
    // GL_LUMINANCE for CV_CAP_OPENNI_DISPARITY_MAP,
    // Work out other mappings as required ( there's a list in comments in main() )
    GLenum inputColourFormat = GL_BGR;
    if (mat.channels() == 1)
    {
        inputColourFormat = GL_LUMINANCE;
    }


    // https://stackoverflow.com/questions/16809833/opencv-image-loading-for-opengl-texture
    
    // use fast 4-byte alignment (default anyway) if possible
    glPixelStorei(GL_UNPACK_ALIGNMENT, (mat.step & 3) ? 1 : 4);

    // set length of one complete row in data (doesn't need to equal image.cols)
    glPixelStorei(GL_UNPACK_ROW_LENGTH, mat.step/mat.elemSize());

    // https://www.khronos.org/registry/OpenGL-Refpages/es3.0/html/glPixelStorei.xhtml

     glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
     
     glPixelStorei(GL_UNPACK_IMAGE_HEIGHT, 0);
     glPixelStorei(GL_UNPACK_ALIGNMENT, 0);

/*
    GLint range[2];
    glGetIntegerv(GL_PACK_ROW_LENGTH, range);
    std::cout << "GL_PACK_ROW_LENGTH " << *range << std::endl;

    glGetIntegerv(GL_PACK_SKIP_ROWS, range);
    std::cout << "GL_PACK_SKIP_ROWS " << *range << std::endl;

    glGetIntegerv(GL_PACK_SKIP_PIXELS, range);
    std::cout << "GL_PACK_SKIP_PIXELS " << *range << std::endl;

    glGetIntegerv(GL_PACK_ALIGNMENT, range);
    std::cout << "GL_PACK_ALIGNMENT " << *range << std::endl;

    glGetIntegerv(GL_UNPACK_ROW_LENGTH, range);
    std::cout << "GL_UNPACK_ROW_LENGTH " << *range << std::endl;

    glGetIntegerv(GL_UNPACK_IMAGE_HEIGHT, range);
    std::cout << "GL_UNPACK_IMAGE_HEIGHT " << *range << std::endl;
  
    glGetIntegerv(GL_UNPACK_SKIP_ROWS, range);
    std::cout << "GL_UNPACK_SKIP_ROWS " << *range << std::endl;

    glGetIntegerv(GL_UNPACK_SKIP_PIXELS, range); 
    std::cout << "GL_UNPACK_SKIP_PIXELS "<< *range << std::endl;

    glGetIntegerv(GL_UNPACK_SKIP_IMAGES, range);
    std::cout << "GL_UNPACK_SKIP_IMAGES " << *range << std::endl;

    glGetIntegerv(GL_UNPACK_ALIGNMENT, range);  
    std::cout << "GL_UNPACK_ALIGNMENT " << *range << std::endl;
    std::cout.flush();
*/

    // Create the texture
    glTexImage2D(GL_TEXTURE_2D,     // Type of texture
                 0,                 // Pyramid level (for mip-mapping) - 0 is the top level
                 GL_RGB8,            // Internal colour format to convert to
                 mat.cols,          // Image width  i.e. 640 for Kinect in standard mode
                 mat.rows,          // Image height i.e. 480 for Kinect in standard mode
                 0,                 // Border width in pixels (can either be 1 or 0)
                 inputColourFormat, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                 GL_UNSIGNED_BYTE,  // Image data type
                 mat.ptr());        // The actual image data itself

    // If we're using mipmaps then generate them. Note: This requires OpenGL 3.0 or higher
    if (minFilter == GL_LINEAR_MIPMAP_LINEAR  ||
            minFilter == GL_LINEAR_MIPMAP_NEAREST ||
            minFilter == GL_NEAREST_MIPMAP_LINEAR ||
            minFilter == GL_NEAREST_MIPMAP_NEAREST)
    {
        glGenerateMipmap(GL_TEXTURE_2D);
    }

    return textureID;
}

void error_callback(int error, const char* description) {
    fprintf(stderr, "Error: %s\n", description);
}


void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
}


void resize_callback(GLFWwindow* window, int new_width, int new_height) {

/*
    glViewport(0, 0, window_width = new_width, window_height = new_height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, window_width, window_height, 0.0, 0.0, 100.0);
    glMatrixMode(GL_MODELVIEW);
*/

}






