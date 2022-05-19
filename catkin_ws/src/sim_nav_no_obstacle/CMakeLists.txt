cmake_minimum_required(VERSION 3.0.2)
project(sim_nav_no_obstacle)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  rosgraph_msgs
  pcl_conversions
  pcl_ros
 # include_directories(include ${catkin_INCLUDE_DIRS})
)

# https://stackoverflow.com/questions/1620918/cmake-and-libpthread
# https://github.com/IntelRealSense/librealsense/issues/825#issuecomment-429065088
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(GLEW 2.0 REQUIRED)
find_package(glfw3 3.0 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED )

#set( GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE )
#set( GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE )
#set( GLFW_BUILD_EXAMPLES OFF CACHE glfwBOOL "" FORCE )




## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )


## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
  INCLUDE_DIRS #include
  LIBRARIES sim_nav_no_obstacle
  CATKIN_DEPENDS roscpp rospy std_msgs image_transport cv_bridge rosgraph_msgs pcl_conversions pcl_ros
)

###########
## Build ##
###########


include_directories(
 include ${realsense_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} 
)



#############
## Install ##
#############



#############
## Testing ##
#############

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
add_executable(sim_nav_no_obstacle src/glwindow.cpp src/main.cpp src/utils.cpp src/image_processing.cpp src/ros_processing.cpp src/segmentation.cpp src/point_cloud_processing.cpp)
target_link_libraries(sim_nav_no_obstacle PUBLIC Threads::Threads ${catkin_LIBRARIES} GLEW::GLEW glfw OpenGL::GL ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${OpenCV_LIBS})

