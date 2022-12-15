cmake_minimum_required(VERSION 3.0.2)
project(demo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

find_package(Qt5Widgets REQUIRED)

if (Qt5Widgets_FOUND)
    if (Qt5Widgets_VERSION VERSION_LESS 5.0)
        message(FATAL_ERROR "Minimum supported Qt5 version is 5.7.1!")
    endif()
else()
    message(SEND_ERROR "The Qt5Widgets library could not be found!")
endif(Qt5Widgets_FOUND)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR} 
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}
  "/usr/include/qwt/"
)


file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/resource.qrc )
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/local/*.hpp src/*.h)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

catkin_package()

add_definitions(${Qt5Widgets_DEFINITIONS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS} -std=c++11")

qt5_wrap_ui(QT_FORMS_HPP ${QT_FORMS})
qt5_add_resources(QT_RESOURCES_CPP ${QT_RESOURCES})
qt5_wrap_cpp(QT_MOC_HPP ${QT_MOC})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

add_executable(simple_gui
  ${QT_FORMS_HPP}
  ${QT_SOURCES}
  ${QT_RESOURCES_CPP}
  ${QT_MOC_HPP}
  )

qt5_use_modules(simple_gui Widgets)

add_dependencies(simple_gui ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

file(GLOB QWT_LIBRARIES "/usr/lib/libqwt*")
target_link_libraries(simple_gui
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  ${QWT_LIBRARIES}
)

