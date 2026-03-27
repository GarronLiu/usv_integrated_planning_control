cmake_minimum_required(VERSION 3.0.2)
project(eskf_estimator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES eskf_estimator
  DEPENDS GeographicLib
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

## Declare a C++ library
set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")
add_library(${PROJECT_NAME}
   src/eskf_estimator.cpp
)
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

add_executable(usv_estimator_node 
src/usv_estimator_node.cpp
)
target_link_libraries(usv_estimator_node
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  ${PROJECT_NAME}
)


