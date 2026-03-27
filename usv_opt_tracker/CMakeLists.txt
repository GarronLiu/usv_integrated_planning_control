cmake_minimum_required(VERSION 3.0.2)
project(usv_opt_tracker)

#set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall -march=native -mtune=native -mavx2 -mfma")# vectorization for intel cpu
set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall -pthread -march=native -mtune=native")

SET(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros_msgs
  nav_msgs
  std_msgs
  tf
  dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(ct_core REQUIRED)
find_package(ct_rbd REQUIRED)
find_package(ct_optcon REQUIRED)
find_package(ct_models REQUIRED)
list(APPEND CMAKE_PREFIX_PATH "/opt")
find_package(blasfeo QUIET)
find_package(hpipm QUIET)
if(blasfeo_FOUND AND hpipm_FOUND)
  message(STATUS "Found HPIPM and BLASFEO")
  set(HPIPM ON)
  list(APPEND HPIPM_LIBS hpipm blasfeo)
  list(APPEND ct_optcon_COMPILE_DEFINITIONS HPIPM)
else()
  message(WARNING "Could not find HPIPM or BLASFEO")
endif()

set(CT_/home/garronliu/1_SLAM/fast_livo2_ws/src/FAST-LIVO2-main/launch/mapping_avia_marslvig.launch/home/garronliu/1_SLAM/fast_livo2_ws/src/FAST-LIVO2-main/launch/mapping_avia_marslvig.launch/home/garronliu/1_SLAM/fast_livo2_ws/src/FAST-LIVO2-main/launch/mapping_avia_marslvig.launchOPTCON_EXAMPLE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/usv_opt_tracker")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/usv_opt_tracker/exampleDir.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/usv_opt_tracker/exampleDir.h)


#add dynamic reconfigure api
generate_dynamic_reconfigure_options(  
		cfg/dynamic_tunes.cfg  
		#...
		)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES usv_opt_tracker
  #  CATKIN_DEPENDS ct_core ct_models ct_optcon ct_rbd roscpp
  #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${ct_core_INCLUDE_DIRS}
  ${ct_rbd_INCLUDE_DIRS}
  ${ct_optcon_INCLUDE_DIRS}
  ${ct_models_INCLUDE_DIRS}
  ${blasfeo_INCLUDE_DIRS}
  ${hpipm_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  include/usv_opt_tracker/usv_opt_tracker.h
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${ct_core_LIBRARIES}
  ${ct_rbd_LIBRARIES}
  ${ct_optcon_LIBRARIES}
  ${ct_models_LIBRARIES}
  ${HPIPM_LIBS}
)

add_executable(${PROJECT_NAME}_node
  src/usv_opt_tracker_node.cpp
)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(wamv_state_converter_node
  src/wamv_state_converter.cpp
)
target_link_libraries(wamv_state_converter_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)


