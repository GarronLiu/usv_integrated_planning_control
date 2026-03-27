cmake_minimum_required(VERSION 3.10)
project(dynamic_occupancy_grid)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  message_filters
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(DOGM REQUIRED)

find_package(OpenCV 3.2.0 QUIET)
if (NOT OpenCV_FOUND)
	set(OpenCV_INCLUDE_DIRS "" CACHE PATH "" )
	set(OpenCV_LIBS "" CACHE FILEPATH "" )
endif()

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES dynamic_occupancy_grid
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${DOGM_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( ${PROJECT_NAME} 
    src/dynamic_occupancy_grid.cpp 
    src/raycast.cpp
    )
target_link_libraries( ${PROJECT_NAME}
    ${DOGM_LIBRARIES}
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    )  

add_executable(${PROJECT_NAME}_node src/dynamic_occupancy_grid_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${DOGM_LIBRARIES} ${catkin_LIBRARIES}	${OpenCV_LIBS})
