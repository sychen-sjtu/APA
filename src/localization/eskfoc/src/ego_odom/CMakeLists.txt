cmake_minimum_required(VERSION 2.8.3)
project(ego_odom)

if( NOT CMAKE_BUILD_TYPE )
    set( CMAKE_BUILD_TYPE Release FORCE )
endif()


# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14 -g) # for std::make_unique
set(CMAKE_CXX_STANDARD 14)
add_definitions(-w)

#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")


find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs nav_msgs cv_bridge eigen_conversions cyber_msgs)
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
# find_package(GeographicLib REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
        include/        
${catkin_INCLUDE_DIRS})

catkin_package()

# add_executable(data_generator
#         simulator/src/data_generator_node.cpp
#         simulator/src/data_generator.cpp
# )

# target_link_libraries(data_generator ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(ekf_estimator
        ekf_estimator/src/ekf_estimator_node.cpp
        ekf_estimator/src/ekf_estimator.cpp
)

target_link_libraries(ekf_estimator ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})

add_dependencies(ekf_estimator  cyber_msgs_generate_messages_cpp)
