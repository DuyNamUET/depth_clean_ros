cmake_minimum_required(VERSION 2.8.3)
project(depth_clean_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  realsense2_camera
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(depth_clean src/depth_clean.cpp)
target_link_libraries(depth_clean ${catkin_LIBRARIES})