cmake_minimum_required(VERSION 3.0.2)
project(manager)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  nav_msgs
  tf
  parking_slot_detection
)

find_package(OpenCV 3 REQUIRED COMPONENTS core imgcodecs imgproc calib3d)
find_package(Eigen3 3.3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES manager
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs parking_slot_detection
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
  ../common/include
)

add_executable(manager_node
  src/main.cpp
  src/manager.cpp
  src/parking_slot_manager.cpp
  src/visualizer.cpp
  ../common/src/math_utils.cpp
)

# add_dependencies(manager parking_slot_detection_generate_messages_cpp)

target_link_libraries(manager_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  ${OpenCV_LIBS}
)
# add_dependencies(manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(manager_node  parking_slot_detection_generate_messages_cpp)
