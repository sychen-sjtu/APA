cmake_minimum_required(VERSION 3.0.2)
project(planning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  parking_slot_detection
)

find_package(OpenCV 3 REQUIRED COMPONENTS core imgcodecs imgproc calib3d)
find_package(Eigen3 3.3 REQUIRED)
find_package(ompl REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES planning
#  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

add_executable(planning
  src/main.cpp
  src/planner.cpp
)

target_link_libraries(planning
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  ${OpenCV_LIBS}
  ompl
)

