cmake_minimum_required(VERSION 3.0.2)
project(usv_ipc_manager)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++14")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wfatal-errors -std=c++14 -Wall -pthread -Wno-unknown-pragmas")
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp  
  geometry_msgs
  visualization_msgs
  mavros_msgs
  sensor_msgs
  std_msgs
  tf  
  path_searching
  bspline
  bspline_opt
  vehicle_centric_map
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES usv_ipc_manager
  CATKIN_DEPENDS bspline bspline_opt path_searching vehicle_centric_map
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node 
  src/usv_ipc_manager_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${GeographicLib_LIBRARIES}
 )

 add_executable(usv_path_generator_node 
 src/usv_path_generator_node.cpp
)

target_link_libraries(usv_path_generator_node
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

